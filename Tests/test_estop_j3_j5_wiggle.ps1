param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [ValidateRange(2.0, 30.0)]
    [double]$J3Degrees = 20.0,
    [ValidateRange(2.0, 30.0)]
    [double]$J5Degrees = 20.0,
    [ValidateRange(2.0, 20.0)]
    [double]$JointSpeedDegS = 12.0,
    [ValidateRange(10, 1000)]
    [int]$AccelerationRpmS = 150,
    [ValidateRange(0, 1000)]
    [int]$EndpointPauseMs = 120,
    [ValidateRange(0, 1000000)]
    [int]$MaxCycles = 0,
    [switch]$VerifyCurrentEstopLockout,
    [int]$ReadTimeoutMs = 2500
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Safety envelope for this test:
# - J3 moves only from calibrated zero toward +J3Degrees and back.
# - J5 moves only from calibrated zero toward +J5Degrees and back.
# - Both amplitudes remain inside the J3<=15 deg J5 +45 deg interlock.
# - MaxCycles=0 means continue until PD15 E-stop is pressed.

$CmdGetState = [byte]0x01
$CmdBenchEnable = [byte]0x21
$CmdBenchStop = [byte]0x23
$CmdBenchMoveRel = [byte]0x24
$RobotStateReady = 1
$RobotErrNotReady = 4
$FaultEstop = [uint32]0x00000800
$RobotStatePayloadSize = 60

function Get-Crc8([byte[]]$Data) {
    [byte]$crc = 0
    foreach ($item in $Data) {
        [byte]$value = $item
        for ($bit = 0; $bit -lt 8; $bit++) {
            $mix = ($crc -bxor $value) -band 1
            $crc = [byte]($crc -shr 1)
            if ($mix -ne 0) {
                $crc = [byte]($crc -bxor 0x8C)
            }
            $value = [byte]($value -shr 1)
        }
    }
    return $crc
}

function New-Frame([byte]$Command, [byte[]]$Payload) {
    if ($null -eq $Payload) {
        $Payload = [byte[]]@()
    }

    [byte[]]$body = New-Object byte[] ($Payload.Length + 1)
    $body[0] = $Command
    for ($index = 0; $index -lt $Payload.Length; $index++) {
        $body[$index + 1] = $Payload[$index]
    }

    [byte[]]$frame = New-Object byte[] ($body.Length + 4)
    $frame[0] = 0xAA
    $frame[1] = [byte]$body.Length
    for ($index = 0; $index -lt $body.Length; $index++) {
        $frame[$index + 2] = $body[$index]
    }
    $frame[$body.Length + 2] = Get-Crc8 $body
    $frame[$body.Length + 3] = 0x55
    return ,$frame
}

function Read-Frame([System.IO.Ports.SerialPort]$Serial) {
    do {
        $stx = $Serial.ReadByte()
    } while ($stx -ne 0xAA)

    $length = $Serial.ReadByte()
    if ($length -le 0 -or $length -ge 128) {
        throw "Invalid response length: $length"
    }

    [byte[]]$body = New-Object byte[] $length
    for ($index = 0; $index -lt $length; $index++) {
        $body[$index] = [byte]$Serial.ReadByte()
    }
    [byte]$receivedCrc = [byte]$Serial.ReadByte()
    $etx = $Serial.ReadByte()

    if ($etx -ne 0x55) {
        throw ('Invalid response ETX: 0x{0:X2}' -f $etx)
    }
    if ((Get-Crc8 $body) -ne $receivedCrc) {
        throw 'Response CRC mismatch'
    }

    [byte[]]$payload = if ($length -gt 1) {
        $body[1..($length - 1)]
    } else {
        [byte[]]@()
    }
    return [pscustomobject]@{
        Command = $body[0]
        Payload = $payload
    }
}

function Invoke-Command(
    [System.IO.Ports.SerialPort]$Serial,
    [byte]$Command,
    [byte[]]$Payload
) {
    [byte[]]$frame = New-Frame $Command $Payload
    $Serial.Write($frame, 0, $frame.Length)
    $response = Read-Frame $Serial
    if ($response.Command -ne $Command) {
        throw (
            'Command mismatch: sent=0x{0:X2}, received=0x{1:X2}' -f
            $Command,
            $response.Command)
    }
    return $response
}

function Get-RobotState([System.IO.Ports.SerialPort]$Serial) {
    $response = Invoke-Command $Serial $CmdGetState ([byte[]]@())
    if ($response.Payload.Length -ne $RobotStatePayloadSize) {
        throw (
            'GET_STATE payload length={0}, expected={1}' -f
            $response.Payload.Length,
            $RobotStatePayloadSize)
    }
    return [pscustomobject]@{
        RunState = [BitConverter]::ToInt32(
            $response.Payload, 0)
        EnabledMask = $response.Payload[52]
        MovingMask = $response.Payload[54]
        FaultFlags = [BitConverter]::ToUInt32(
            $response.Payload, 56)
    }
}

function Write-BeU16([uint16]$Value) {
    return [byte[]]@(
        [byte](($Value -shr 8) -band 0xFF),
        [byte]($Value -band 0xFF))
}

function Write-BeU32([uint32]$Value) {
    return [byte[]]@(
        [byte](($Value -shr 24) -band 0xFF),
        [byte](($Value -shr 16) -band 0xFF),
        [byte](($Value -shr 8) -band 0xFF),
        [byte]($Value -band 0xFF))
}

function Invoke-ResultCommand(
    [System.IO.Ports.SerialPort]$Serial,
    [byte]$Command,
    [byte[]]$Payload
) {
    $response = Invoke-Command $Serial $Command $Payload
    if ($response.Payload.Length -ne 1) {
        throw (
            'Command 0x{0:X2}: invalid result payload length {1}' -f
            $Command,
            $response.Payload.Length)
    }
    return [int]$response.Payload[0]
}

function Enable-Motor(
    [System.IO.Ports.SerialPort]$Serial,
    [byte]$MotorId
) {
    $result = Invoke-ResultCommand `
        $Serial `
        $CmdBenchEnable `
        ([byte[]]@($MotorId))
    if ($result -ne 0) {
        throw "Motor $MotorId enable rejected: code=$result"
    }
}

function Stop-Motor(
    [System.IO.Ports.SerialPort]$Serial,
    [byte]$MotorId
) {
    $result = Invoke-ResultCommand `
        $Serial `
        $CmdBenchStop `
        ([byte[]]@($MotorId))
    if ($result -ne 0) {
        Write-Warning "Motor $MotorId stop rejected: code=$result"
    }
}

function Move-JointRelative(
    [System.IO.Ports.SerialPort]$Serial,
    [byte]$MotorId,
    [byte]$RawDirection,
    [double]$JointDegrees,
    [double]$GearRatio
) {
    [uint32]$motorDegreesTenths = [uint32][math]::Round(
        $JointDegrees * $GearRatio * 10.0)
    [uint16]$motorVelocityTenths = [uint16][math]::Round(
        ($JointSpeedDegS * $GearRatio / 6.0) * 10.0)

    [byte[]]$payload =
        ([byte[]]@($MotorId, $RawDirection)) +
        (Write-BeU32 $motorDegreesTenths) +
        (Write-BeU16 $motorVelocityTenths) +
        (Write-BeU16 ([uint16]$AccelerationRpmS))

    return Invoke-ResultCommand `
        $Serial `
        $CmdBenchMoveRel `
        $payload
}

function Wait-ForEndpointOrEstop(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MovementMs
) {
    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt $MovementMs) {
        $state = Get-RobotState $Serial
        if (($state.FaultFlags -band $FaultEstop) -ne 0) {
            return $true
        }
        if ($state.FaultFlags -ne 0) {
            throw (
                'Unexpected fault during motion: 0x{0:X8}' -f
                $state.FaultFlags)
        }
        Start-Sleep -Milliseconds 60
    }
    return $false
}

$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = $ReadTimeoutMs
$serial.WriteTimeout = $ReadTimeoutMs

$estopObserved = $false
$cycle = 0
$j3Ratio = 50.890
$j5Ratio = 26.850
$movementMs = [int][math]::Ceiling(
    ([math]::Max($J3Degrees, $J5Degrees) /
        $JointSpeedDegS) * 1000.0 + 500.0)

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    $initial = Get-RobotState $serial
    if ($VerifyCurrentEstopLockout) {
        $estopObserved = $true
        if (($initial.FaultFlags -band $FaultEstop) -eq 0) {
            throw (
                'ESTOP is not currently latched: fault=0x{0:X8}' -f
                $initial.FaultFlags)
        }

        $enableResult = Invoke-ResultCommand `
            $serial `
            $CmdBenchEnable `
            ([byte[]]@(3))
        if ($enableResult -ne $RobotErrNotReady) {
            throw (
                'Post-ESTOP J3 enable returned {0}, expected NOT_READY ({1})' -f
                $enableResult,
                $RobotErrNotReady)
        }

        $locked = Get-RobotState $serial
        if ($locked.EnabledMask -ne 0 -or
            $locked.MovingMask -ne 0 -or
            ($locked.FaultFlags -band $FaultEstop) -eq 0) {
            throw (
                'Post-ESTOP lockout invalid: enabled=0x{0:X2}, moving=0x{1:X2}, fault=0x{2:X8}' -f
                $locked.EnabledMask,
                $locked.MovingMask,
                $locked.FaultFlags)
        }
        Write-Host (
            'ESTOP_LOCKOUT=PASS J3_ENABLE=NOT_READY enabled=0x{0:X2} moving=0x{1:X2} fault=0x{2:X8}' -f
            $locked.EnabledMask,
            $locked.MovingMask,
            $locked.FaultFlags) -ForegroundColor Green
        return
    }

    if ($initial.RunState -ne $RobotStateReady -or
        $initial.FaultFlags -ne 0) {
        throw (
            'Startup guard not satisfied: state={0}, fault=0x{1:X8}' -f
            $initial.RunState,
            $initial.FaultFlags)
    }

    Enable-Motor $serial 3
    Enable-Motor $serial 5

    Write-Host (
        'J3/J5 E-stop test running: J3=0..+{0} deg, J5=0..+{1} deg' -f
        $J3Degrees,
        $J5Degrees)
    Write-Host 'Press the PD15 user key now. The test stops on ESTOP.'

    while ($MaxCycles -eq 0 -or $cycle -lt $MaxCycles) {
        foreach ($rawDirection in [byte[]]@(0, 1)) {
            $j3Result = Move-JointRelative `
                $serial 3 $rawDirection $J3Degrees $j3Ratio
            if ($j3Result -ne 0) {
                $state = Get-RobotState $serial
                if (($state.FaultFlags -band $FaultEstop) -ne 0) {
                    $estopObserved = $true
                    break
                }
                throw "J3 move rejected: code=$j3Result"
            }

            $j5Result = Move-JointRelative `
                $serial 5 $rawDirection $J5Degrees $j5Ratio
            if ($j5Result -ne 0) {
                $state = Get-RobotState $serial
                if (($state.FaultFlags -band $FaultEstop) -ne 0) {
                    $estopObserved = $true
                    break
                }
                throw "J5 move rejected: code=$j5Result"
            }

            if (Wait-ForEndpointOrEstop $serial $movementMs) {
                $estopObserved = $true
                break
            }
            if ($EndpointPauseMs -gt 0) {
                Start-Sleep -Milliseconds $EndpointPauseMs
            }
        }

        if ($estopObserved) {
            break
        }
        $cycle++
        Write-Host ("cycle={0} complete; both joints returned to zero" -f
            $cycle)
    }

    $final = Get-RobotState $serial
    if ($estopObserved) {
        if ($final.EnabledMask -ne 0 -or
            $final.MovingMask -ne 0 -or
            ($final.FaultFlags -band $FaultEstop) -eq 0) {
            throw (
                'ESTOP state invalid: enabled=0x{0:X2}, moving=0x{1:X2}, fault=0x{2:X8}' -f
                $final.EnabledMask,
                $final.MovingMask,
                $final.FaultFlags)
        }
        Write-Host (
            'ESTOP=PASS enabled=0x{0:X2} moving=0x{1:X2} fault=0x{2:X8}' -f
            $final.EnabledMask,
            $final.MovingMask,
            $final.FaultFlags) -ForegroundColor Green
    } else {
        Write-Host (
            'Completed {0} cycles at calibrated zero without ESTOP.' -f
            $cycle)
    }
}
finally {
    if ($serial.IsOpen) {
        if (-not $estopObserved) {
            try { Stop-Motor $serial 3 } catch {}
            try { Stop-Motor $serial 5 } catch {}
        }
        $serial.Close()
    }
    $serial.Dispose()
}

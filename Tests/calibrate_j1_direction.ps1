param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [ValidateSet(0, 1)]
    [int]$RawDirection = 0,
    [ValidateRange(5, 60)]
    [int]$DurationSeconds = 20,
    [double]$MotorVelocityRpm = 30.0,
    [int]$AccelerationRpmS = 80,
    [int]$ReadTimeoutMs = 4000,
    [switch]$Execute
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$CmdHello = 0x00
$CmdBenchQuery = 0x20
$CmdBenchEnable = 0x21
$CmdBenchStop = 0x23
$CmdBenchMoveRel = 0x24

$J1MotorId = 1
$J1GearRatio = 50.0
$AllMotorIds = @(1, 2, 3, 4, 5, 6)

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
        $Payload = @()
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
    [byte]$receivedCrc = $Serial.ReadByte()
    $etx = $Serial.ReadByte()
    if ($etx -ne 0x55 -or (Get-Crc8 $body) -ne $receivedCrc) {
        throw 'Invalid protocol response'
    }

    [byte[]]$payload = @()
    if ($length -gt 1) {
        $payload = New-Object byte[] ($length - 1)
        for ($index = 1; $index -lt $length; $index++) {
            $payload[$index - 1] = $body[$index]
        }
    }
    return [pscustomobject]@{
        Command = $body[0]
        Payload = $payload
    }
}

function Invoke-Cmd(
    [System.IO.Ports.SerialPort]$Serial,
    [byte]$Command,
    [byte[]]$Payload
) {
    $frame = New-Frame $Command $Payload
    $Serial.Write($frame, 0, $frame.Length)
    return Read-Frame $Serial
}

function Assert-Ok(
    [pscustomobject]$Response,
    [byte]$Command,
    [string]$Step
) {
    if ($Response.Command -ne $Command -or
        $Response.Payload.Length -ne 1 -or
        $Response.Payload[0] -ne 0) {
        $code = if ($Response.Payload.Length -gt 0) {
            $Response.Payload[0]
        } else {
            -1
        }
        throw "$Step failed: code=$code"
    }
}

function Write-BeU16([uint16]$Value) {
    return [byte[]]@(
        [byte](($Value -shr 8) -band 0xFF)
        [byte]($Value -band 0xFF)
    )
}

function Write-BeU32([uint32]$Value) {
    return [byte[]]@(
        [byte](($Value -shr 24) -band 0xFF)
        [byte](($Value -shr 16) -band 0xFF)
        [byte](($Value -shr 8) -band 0xFF)
        [byte]($Value -band 0xFF)
    )
}

function Get-MotorState(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId
) {
    $response = Invoke-Cmd $Serial $CmdBenchQuery ([byte[]]@([byte]$MotorId))
    if ($response.Command -ne $CmdBenchQuery -or
        $response.Payload.Length -lt 28) {
        throw "M$MotorId query failed"
    }

    $positionUrad = [BitConverter]::ToInt32($response.Payload, 4)
    return [pscustomobject]@{
        Online = $response.Payload[1] -ne 0
        PositionMotorDegrees =
            [math]::Round($positionUrad * 180.0 / 3141593.0, 3)
        Velocity = [BitConverter]::ToInt32($response.Payload, 8)
    }
}

function Enable-Motor(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId
) {
    Assert-Ok `
        (Invoke-Cmd $Serial $CmdBenchEnable ([byte[]]@([byte]$MotorId))) `
        $CmdBenchEnable `
        "ENABLE M$MotorId"
}

function Stop-J1([System.IO.Ports.SerialPort]$Serial) {
    Assert-Ok `
        (Invoke-Cmd $Serial $CmdBenchStop ([byte[]]@([byte]$J1MotorId))) `
        $CmdBenchStop `
        'STOP J1'
}

function Move-J1([System.IO.Ports.SerialPort]$Serial) {
    # Command beyond the observation window, then STOP at DurationSeconds.
    # This produces one uninterrupted direction instead of a wiggle/return.
    $motorDegrees =
        $MotorVelocityRpm * 6.0 * ($DurationSeconds + 5.0)
    $payload = [byte[]]@([byte]$J1MotorId, [byte]$RawDirection) +
        (Write-BeU32 ([uint32][math]::Round($motorDegrees * 10.0))) +
        (Write-BeU16 ([uint16][math]::Round($MotorVelocityRpm * 10.0))) +
        (Write-BeU16 ([uint16]$AccelerationRpmS))

    Assert-Ok `
        (Invoke-Cmd $Serial $CmdBenchMoveRel $payload) `
        $CmdBenchMoveRel `
        'MOVE J1'
}

if ($MotorVelocityRpm -le 0 -or $MotorVelocityRpm -gt 60) {
    throw 'MotorVelocityRpm must be in (0, 60]'
}
if ($AccelerationRpmS -le 0 -or $AccelerationRpmS -gt 150) {
    throw 'AccelerationRpmS must be in (0, 150]'
}

$commandMotorDegrees =
    $MotorVelocityRpm * 6.0 * ($DurationSeconds + 5.0)
$expectedObservedJointDegrees =
    $MotorVelocityRpm * 6.0 * $DurationSeconds /
    $J1GearRatio
$waitMs = $DurationSeconds * 1000

Write-Host '==== ZEROARM J1 DIRECTION CALIBRATION ===='
Write-Host ('raw_dir={0}, duration={1}s, expected_joint_move={2:N1}deg, command_motor_move={3:N1}deg, motor_rpm={4}' -f
    $RawDirection,
    $DurationSeconds,
    $expectedObservedJointDegrees,
    $commandMotorDegrees,
    $MotorVelocityRpm)
Write-Host 'No SET_ZERO and no DISABLE command will be sent.'

if (-not $Execute) {
    Write-Host 'DIAGNOSTIC ONLY: pass -Execute to allow the one-way motion.'
    exit 0
}

$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = $ReadTimeoutMs
$serial.WriteTimeout = $ReadTimeoutMs
$serial.DtrEnable = $false
$serial.RtsEnable = $false

$motionActive = $false
$enabledMotorIds = New-Object System.Collections.Generic.List[int]

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    $hello = Invoke-Cmd $serial $CmdHello @()
    $helloText = [Text.Encoding]::ASCII.GetString($hello.Payload)
    if ($hello.Command -ne $CmdHello -or $helloText -ne 'ZEROARM/1.0') {
        throw "HELLO failed: $helloText"
    }

    foreach ($motorId in $AllMotorIds) {
        $state = Get-MotorState $serial $motorId
        if (-not $state.Online) {
            throw "M$motorId is offline; refusing J1 motion"
        }
        Enable-Motor $serial $motorId
        $enabledMotorIds.Add($motorId) | Out-Null
    }

    Stop-J1 $serial
    Enable-Motor $serial $J1MotorId
    $before = Get-MotorState $serial $J1MotorId

    Write-Host ('J1 moving continuously for {0}s -- observe from the top...' -f
        $DurationSeconds)
    Move-J1 $serial
    $motionActive = $true
    Start-Sleep -Milliseconds $waitMs
    Stop-J1 $serial
    $motionActive = $false

    foreach ($motorId in $AllMotorIds) {
        Enable-Motor $serial $motorId
    }
    Start-Sleep -Milliseconds 300

    $after = Get-MotorState $serial $J1MotorId
    $deltaMotor = [math]::Round(
        $after.PositionMotorDegrees - $before.PositionMotorDegrees,
        3)
    Write-Host ('DIRECTION TEST PASS: before={0}deg, after={1}deg, delta={2} motor deg, velocity={3}' -f
        $before.PositionMotorDegrees,
        $after.PositionMotorDegrees,
        $deltaMotor,
        $after.Velocity) -ForegroundColor Green
    Write-Host 'All six motors are ENABLED. Reply with top-view CW or CCW.'
} catch {
    if ($serial.IsOpen) {
        if ($motionActive) {
            try {
                Stop-J1 $serial
            } catch {
                Write-Host "STOP J1 failed: $($_.Exception.Message)" -ForegroundColor Red
            }
        }
        foreach ($motorId in $enabledMotorIds) {
            try {
                Enable-Motor $serial $motorId
            } catch {
                Write-Host "WARN: could not re-enable M$motorId" -ForegroundColor Yellow
            }
        }
    }
    throw
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}

param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [double]$AwayJointDegrees = 3.0,
    [double]$BackoffJointDegrees = 1.0,
    [double]$SeekJointDegrees = 6.0,
    [double]$MotorVelocityRpm = 15.0,
    [int]$AccelerationRpmS = 50,
    [int]$ReadTimeoutMs = 4000,
    [string]$ProgrammerPath,
    [switch]$Execute
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$CmdHello = 0x00
$CmdBenchQuery = 0x20
$CmdBenchEnable = 0x21
$CmdBenchStop = 0x23
$CmdBenchMoveRel = 0x24
$CmdBenchSetZero = 0x25

$J1MotorId = 1
$J1GearRatio = 50.0
$J1LimitAddress = '0x48001010'
$J1LimitBit = 7
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
        throw "Invalid protocol length: $length"
    }

    [byte[]]$body = New-Object byte[] $length
    for ($index = 0; $index -lt $length; $index++) {
        $body[$index] = [byte]$Serial.ReadByte()
    }
    [byte]$receivedCrc = $Serial.ReadByte()
    $etx = $Serial.ReadByte()
    if ($etx -ne 0x55) {
        throw ('Invalid ETX: 0x{0:X2}' -f $etx)
    }
    if ((Get-Crc8 $body) -ne $receivedCrc) {
        throw 'Protocol CRC mismatch'
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
        throw ('{0} failed: response=0x{1:X2}, code={2}' -f
            $Step, $Response.Command, $code)
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
        $code = if ($response.Payload.Length -eq 1) {
            $response.Payload[0]
        } else {
            -1
        }
        throw "M$MotorId query failed: code=$code"
    }

    $positionUrad = [BitConverter]::ToInt32($response.Payload, 4)
    return [pscustomobject]@{
        Online = $response.Payload[1] -ne 0
        PositionMotorDegrees =
            [math]::Round($positionUrad * 180.0 / 3141593.0, 3)
    }
}

function Enable-Motor(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId
) {
    Assert-Ok `
        (Invoke-Cmd $Serial $CmdBenchEnable ([byte[]]@([byte]$MotorId))) `
        $CmdBenchEnable "ENABLE M$MotorId"
}

function Stop-J1([System.IO.Ports.SerialPort]$Serial) {
    Assert-Ok `
        (Invoke-Cmd $Serial $CmdBenchStop ([byte[]]@([byte]$J1MotorId))) `
        $CmdBenchStop 'STOP J1'
}

function Move-J1(
    [System.IO.Ports.SerialPort]$Serial,
    [byte]$Direction,
    [double]$JointDegrees
) {
    $motorDegrees = $JointDegrees * $J1GearRatio
    $degreesTenths = [uint32][math]::Round($motorDegrees * 10.0)
    $velocityTenths = [uint16][math]::Round($MotorVelocityRpm * 10.0)
    $payload = [byte[]]@([byte]$J1MotorId, $Direction) +
        (Write-BeU32 $degreesTenths) +
        (Write-BeU16 $velocityTenths) +
        (Write-BeU16 ([uint16]$AccelerationRpmS))

    Assert-Ok `
        (Invoke-Cmd $Serial $CmdBenchMoveRel $payload) `
        $CmdBenchMoveRel `
        ('MOVE J1 dir={0} joint_deg={1}' -f $Direction, $JointDegrees)
}

function Set-J1Zero([System.IO.Ports.SerialPort]$Serial) {
    Assert-Ok `
        (Invoke-Cmd $Serial $CmdBenchSetZero ([byte[]]@([byte]$J1MotorId))) `
        $CmdBenchSetZero 'SET_ZERO J1'
}

function Resolve-Programmer([string]$RequestedPath) {
    if ($RequestedPath) {
        if (-not (Test-Path -LiteralPath $RequestedPath -PathType Leaf)) {
            throw "STM32_Programmer_CLI not found: $RequestedPath"
        }
        return (Resolve-Path -LiteralPath $RequestedPath).Path
    }

    $command = Get-Command 'STM32_Programmer_CLI.exe' -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        return $command.Source
    }

    $candidates = @(
        'C:\ST\STM32CubeCLT_1.22.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
        'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    )
    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return $candidate
        }
    }
    throw 'STM32_Programmer_CLI.exe not found'
}

function Read-J1LimitHigh([string]$CliPath) {
    $output = & $CliPath `
        '-c' 'port=SWD' 'mode=HOTPLUG' `
        '-r32' $J1LimitAddress '4' 2>&1 | Out-String
    if ($LASTEXITCODE -ne 0) {
        throw "ST-Link GPIO read failed:`n$output"
    }

    $match = [regex]::Match(
        $output,
        '(?im)^\s*0x48001010\s*:\s*([0-9a-f]{8})\s*$')
    if (-not $match.Success) {
        throw "GPIOE_IDR missing from ST-Link output:`n$output"
    }

    $idr = [Convert]::ToUInt32($match.Groups[1].Value, 16)
    return (($idr -shr $J1LimitBit) -band 1) -ne 0
}

function Wait-J1Limit(
    [string]$CliPath,
    [bool]$ExpectedHigh,
    [double]$TimeoutSeconds
) {
    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.Elapsed.TotalSeconds -lt $TimeoutSeconds) {
        $high = Read-J1LimitHigh $CliPath
        Write-Host ('  limit PE7={0} t={1:N2}s' -f
            ($(if ($high) { 'HIGH' } else { 'LOW' })),
            $timer.Elapsed.TotalSeconds)
        if ($high -eq $ExpectedHigh) {
            return $true
        }
    }
    return $false
}

function Get-MoveTimeoutSeconds([double]$JointDegrees) {
    $motorDegrees = $JointDegrees * $J1GearRatio
    $seconds = $motorDegrees / ($MotorVelocityRpm * 6.0)
    return $seconds + 2.0
}

if ($AwayJointDegrees -le 0 -or $AwayJointDegrees -gt 5 -or
    $BackoffJointDegrees -le 0 -or $BackoffJointDegrees -gt 2 -or
    $SeekJointDegrees -le 0 -or $SeekJointDegrees -gt 10) {
    throw 'Motion bounds exceeded (away<=5, backoff<=2, seek<=10 joint degrees)'
}
if ($MotorVelocityRpm -le 0 -or $MotorVelocityRpm -gt 30) {
    throw 'MotorVelocityRpm must be in (0, 30]'
}
if ($AccelerationRpmS -le 0 -or $AccelerationRpmS -gt 100) {
    throw 'AccelerationRpmS must be in (0, 100]'
}

$cliPath = Resolve-Programmer $ProgrammerPath
$initialLimitHigh = Read-J1LimitHigh $cliPath
Write-Host '==== ZEROARM J1 LIMIT/HOME TEST ===='
Write-Host ('J1 limit initial: PE7={0}' -f
    ($(if ($initialLimitHigh) { 'HIGH (active)' } else { 'LOW (released)' })))
Write-Host ('motion: away={0}deg, backoff={1}deg, seek_max={2}deg, motor_rpm={3}' -f
    $AwayJointDegrees, $BackoffJointDegrees, $SeekJointDegrees, $MotorVelocityRpm)

if (-not $Execute) {
    Write-Host 'DIAGNOSTIC ONLY: pass -Execute to allow motion.'
    exit 0
}
if (-not $initialLimitHigh) {
    throw 'J1 is not currently on the limit; this bounded release/return test will not move.'
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

    Write-Host 'Checking and enabling all motors for holding torque...'
    foreach ($motorId in $AllMotorIds) {
        $state = Get-MotorState $serial $motorId
        if (-not $state.Online) {
            throw "M$motorId is offline; refusing J1 motion"
        }
        Enable-Motor $serial $motorId
        $enabledMotorIds.Add($motorId) | Out-Null
        Write-Host ("  M$motorId ONLINE + ENABLED")
    }

    # Prove that the stop path is accepted before starting motion.
    Stop-J1 $serial
    Enable-Motor $serial $J1MotorId
    Write-Host 'J1 STOP path: PASS; all motors remain enabled.'

    Write-Host 'Stage 1: move J1 away from the active limit (dir=0)...'
    Move-J1 $serial 0 $AwayJointDegrees
    $motionActive = $true
    $released = Wait-J1Limit `
        $cliPath `
        $false `
        (Get-MoveTimeoutSeconds $AwayJointDegrees)
    Stop-J1 $serial
    $motionActive = $false
    Enable-Motor $serial $J1MotorId
    if (-not $released) {
        throw 'J1 PE7 did not change HIGH->LOW while moving away; no zero was written.'
    }

    Write-Host 'Stage 2: add a small backoff and verify PE7 stays LOW...'
    Move-J1 $serial 0 $BackoffJointDegrees
    $motionActive = $true
    Start-Sleep -Milliseconds ([int](
        (Get-MoveTimeoutSeconds $BackoffJointDegrees) * 1000))
    Stop-J1 $serial
    $motionActive = $false
    Enable-Motor $serial $J1MotorId
    if (Read-J1LimitHigh $cliPath) {
        throw 'J1 limit did not remain released after backoff; no zero was written.'
    }

    Write-Host 'Stage 3: seek J1 limit slowly (dir=1)...'
    Move-J1 $serial 1 $SeekJointDegrees
    $motionActive = $true
    $triggered = Wait-J1Limit `
        $cliPath `
        $true `
        (Get-MoveTimeoutSeconds $SeekJointDegrees)
    Stop-J1 $serial
    $motionActive = $false
    Enable-Motor $serial $J1MotorId
    if (-not $triggered) {
        throw 'J1 PE7 did not change LOW->HIGH during bounded seek; no zero was written.'
    }

    Start-Sleep -Milliseconds 150
    if (-not (Read-J1LimitHigh $cliPath)) {
        throw 'J1 limit was not stable after STOP; no zero was written.'
    }

    Write-Host 'Stage 4: stable limit confirmed; writing J1 motor zero...'
    Set-J1Zero $serial
    Start-Sleep -Milliseconds 300
    $finalState = Get-MotorState $serial $J1MotorId

    foreach ($motorId in $AllMotorIds) {
        Enable-Motor $serial $motorId
    }

    Write-Host ('J1 HOME PASS: final motor position={0}deg, PE7=HIGH, all motors ENABLED.' -f
        $finalState.PositionMotorDegrees) -ForegroundColor Green
} catch {
    if ($serial.IsOpen) {
        if ($motionActive) {
            try {
                Stop-J1 $serial
            } catch {
                Write-Host "EMERGENCY STOP command failed: $($_.Exception.Message)" -ForegroundColor Red
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

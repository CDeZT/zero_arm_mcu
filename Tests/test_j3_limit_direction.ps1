param(
    [ValidateSet(3, 4, 5)]
    [int]$TestJoint = 3,
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [ValidateSet(0, 1)]
    [int]$RawDirection = 0,
    [ValidateRange(0.5, 225.0)]
    [double]$JointDegrees = 2.0,
    [ValidateRange(1.0, 100.0)]
    [double]$MotorVelocityRpm = 10.0,
    [ValidateRange(1, 50)]
    [int]$AccelerationRpmS = 30,
    [int]$ReadTimeoutMs = 4000,
    [string]$ProgrammerPath,
    [switch]$ReturnHome,
    [switch]$ContinueAfterRelease,
    [switch]$AllowInactiveStart,
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

$J3MotorId = $TestJoint
$J3GearRatio = if ($TestJoint -eq 3) {
    50.89
} elseif ($TestJoint -eq 4) {
    51.0
} else {
    26.85
}
$J3LimitAddress = '0x48001010'
$J3LimitBit = if ($TestJoint -eq 3) {
    9
} elseif ($TestJoint -eq 4) {
    10
} else {
    11
}
$J3Name = "J$TestJoint"
$LimitLabel = "PE$J3LimitBit"
$AllMotorIds = @(1, 2, 3, 4, 5, 6)

if (-not $ReturnHome -and
    $JointDegrees -gt 8.0 -and
    -not $ContinueAfterRelease) {
    throw 'RELEASE_LIMIT direction tests are capped at 8 joint degrees.'
}

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
            $positionUrad * 180.0 / 3141593.0
        VelocityTenthsRpm =
            [BitConverter]::ToInt32($response.Payload, 8)
        CurrentMa =
            [BitConverter]::ToUInt16($response.Payload, 12)
        Status =
            [BitConverter]::ToUInt16($response.Payload, 14)
        FaultFlags =
            [BitConverter]::ToUInt32($response.Payload, 16)
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

function Stop-J3([System.IO.Ports.SerialPort]$Serial) {
    Assert-Ok `
        (Invoke-Cmd $Serial $CmdBenchStop ([byte[]]@([byte]$J3MotorId))) `
        $CmdBenchStop `
        'STOP J3'
}

function Set-J3Zero([System.IO.Ports.SerialPort]$Serial) {
    Assert-Ok `
        (Invoke-Cmd $Serial $CmdBenchSetZero ([byte[]]@([byte]$J3MotorId))) `
        $CmdBenchSetZero `
        'SET_ZERO J3'
}

function Move-J3([System.IO.Ports.SerialPort]$Serial) {
    $motorDegrees = $JointDegrees * $J3GearRatio
    $payload = [byte[]]@([byte]$J3MotorId, [byte]$RawDirection) +
        (Write-BeU32 ([uint32][math]::Round($motorDegrees * 10.0))) +
        (Write-BeU16 ([uint16][math]::Round($MotorVelocityRpm * 10.0))) +
        (Write-BeU16 ([uint16]$AccelerationRpmS))

    Assert-Ok `
        (Invoke-Cmd $Serial $CmdBenchMoveRel $payload) `
        $CmdBenchMoveRel `
        'MOVE J3'
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

function Read-J3LimitHigh([string]$CliPath) {
    $output = & $CliPath `
        '-c' 'port=SWD' 'mode=HOTPLUG' `
        '-r32' $J3LimitAddress '4' 2>&1 | Out-String
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
    return (($idr -shr $J3LimitBit) -band 1) -ne 0
}

$cliPath = Resolve-Programmer $ProgrammerPath
$initialLimitHigh = Read-J3LimitHigh $cliPath
$expectedMotorDegrees = $JointDegrees * $J3GearRatio
$moveMs = [int][math]::Ceiling(
    ($expectedMotorDegrees / ($MotorVelocityRpm * 6.0) * 1000.0) +
    700.0)

Write-Host ("==== ZEROARM {0} LIMIT DIRECTION TEST ====" -f $J3Name)
Write-Host ('initial_limit={0}, raw_dir={1}, joint_move={2}deg, motor_move={3:N2}deg, motor_rpm={4}' -f
    ($(if ($initialLimitHigh) { 'HIGH (active)' } else { 'LOW' })),
    $RawDirection,
    $JointDegrees,
    $expectedMotorDegrees,
    $MotorVelocityRpm)
if ($ReturnHome) {
    Write-Host 'Mode=RETURN_HOME: LOW->HIGH stops immediately, then SET_ZERO.'
} else {
    Write-Host 'Mode=RELEASE_LIMIT: HIGH->LOW stops immediately; no SET_ZERO.'
}
Write-Host 'No DISABLE command will be sent.'

if (-not $Execute) {
    Write-Host 'DIAGNOSTIC ONLY: pass -Execute to allow motion.'
    exit 0
}
if ($ReturnHome) {
    $expectedHomeRawDirection = 1
    if ($TestJoint -eq 3 -and
        $RawDirection -ne $expectedHomeRawDirection) {
        throw ("RETURN_HOME for {0} requires calibrated raw direction {1}." -f
            $J3Name,
            $expectedHomeRawDirection)
    }
    if ($initialLimitHigh) {
        throw "$J3Name limit is already HIGH; refusing return motion."
    }
} elseif (-not $initialLimitHigh -and
          -not $AllowInactiveStart) {
    throw "$J3Name is not currently active HIGH; refusing this at-limit direction test."
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

    if ($TestJoint -eq 4) {
        $j3State = Get-MotorState $serial 3
        $j3JointDegrees =
            $j3State.PositionMotorDegrees / 50.89
        if ($j3JointDegrees -le 15.0) {
            throw ('J3 is only {0:N3}deg from zero; J4 motion requires J3 > 15deg.' -f
                $j3JointDegrees)
        }
        Write-Host ('J3 interlock PASS: {0:N3}deg > 15deg' -f
            $j3JointDegrees)
    } elseif ($TestJoint -eq 5 -and -not $ReturnHome) {
        $j3State = Get-MotorState $serial 3
        $j5State = Get-MotorState $serial 5
        $j3JointDegrees =
            $j3State.PositionMotorDegrees / 50.89
        $j5JointDegrees =
            $j5State.PositionMotorDegrees / 26.85
        $j5TargetDegrees = $j5JointDegrees +
            $(if ($RawDirection -eq 0) {
                $JointDegrees
            } else {
                -$JointDegrees
            })

        $encoderToleranceDegrees = 0.25
        if ($j5TargetDegrees -lt (-35.0 - $encoderToleranceDegrees) -or
            $j5TargetDegrees -gt (135.0 + $encoderToleranceDegrees)) {
            throw ('J5 target {0:N3}deg is outside calibrated range -35..135deg.' -f
                $j5TargetDegrees)
        }
        if ($j5TargetDegrees -gt (45.0 + $encoderToleranceDegrees) -and
            $j3JointDegrees -le 15.0) {
            throw ('J5 target {0:N3}deg requires J3 > 15deg; J3 is {1:N3}deg.' -f
                $j5TargetDegrees,
                $j3JointDegrees)
        }
        if ($j5TargetDegrees -gt (60.0 + $encoderToleranceDegrees) -and
            $j3JointDegrees -le 45.0) {
            throw ('J5 target {0:N3}deg requires J3 > 45deg; J3 is {1:N3}deg.' -f
                $j5TargetDegrees,
                $j3JointDegrees)
        }
        Write-Host ('J5 interlock PASS: target={0:N3}deg, J3={1:N3}deg.' -f
            $j5TargetDegrees,
            $j3JointDegrees)
    }

    foreach ($motorId in $AllMotorIds) {
        try {
            $state = Get-MotorState $serial $motorId
            if ($motorId -eq $J3MotorId -and
                (-not $state.Online -or
                 $state.FaultFlags -ne 0)) {
                throw 'M3 offline or faulted; refusing J3 motion'
            }
        } catch {
            if ($motorId -eq $J3MotorId) {
                throw
            }
            Write-Host ("WARN: M{0} pre-query failed; ENABLE will still be sent: {1}" -f
                $motorId,
                $_.Exception.Message) -ForegroundColor Yellow
        }
        Enable-Motor $serial $motorId
        $enabledMotorIds.Add($motorId) | Out-Null
    }

    Stop-J3 $serial
    Enable-Motor $serial $J3MotorId
    $before = Get-MotorState $serial $J3MotorId

    Write-Host "$J3Name small motion starting now..."
    Move-J3 $serial
    $motionActive = $true
    $limitGoalReached = $false
    $monitorWatch = [System.Diagnostics.Stopwatch]::StartNew()
    $lastLimitHigh = $initialLimitHigh
    while ($monitorWatch.ElapsedMilliseconds -lt $moveMs) {
        $sampleLimitHigh = Read-J3LimitHigh $cliPath
        $releasedThisSample = $lastLimitHigh -and -not $sampleLimitHigh
        if ($sampleLimitHigh -ne $lastLimitHigh) {
            Write-Host ("{0} transition: {{0}}->{{1}} at {{2}}ms" -f $LimitLabel -f
                ($(if ($lastLimitHigh) { 'HIGH' } else { 'LOW' })),
                ($(if ($sampleLimitHigh) { 'HIGH' } else { 'LOW' })),
                $monitorWatch.ElapsedMilliseconds)
            $lastLimitHigh = $sampleLimitHigh
        }
        # A release test may begin with the switch already inactive.  In that
        # case LOW alone is not an event; only an observed HIGH->LOW edge may
        # stop the commanded clearance move.
        $sampleGoalReached = if ($ReturnHome) {
            $sampleLimitHigh
        } else {
            $releasedThisSample
        }
        if ($sampleGoalReached -and
            (-not $ContinueAfterRelease -or $ReturnHome)) {
            $limitGoalReached = $true
            if ($ReturnHome) {
                Write-Host "$LimitLabel triggered HIGH; stopping $J3Name immediately."
            } else {
                Write-Host "$LimitLabel released LOW; stopping $J3Name immediately."
            }
            break
        }
        Start-Sleep -Milliseconds 75
    }
    Stop-J3 $serial
    $motionActive = $false
    Enable-Motor $serial $J3MotorId
    Start-Sleep -Milliseconds 250

    $triggerState = Get-MotorState $serial $J3MotorId
    $finalLimitHigh = Read-J3LimitHigh $cliPath
    $after = $triggerState
    if ($ReturnHome -and
        $limitGoalReached -and
        $finalLimitHigh) {
        Set-J3Zero $serial
        Start-Sleep -Milliseconds 250
        $after = Get-MotorState $serial $J3MotorId
    }
    foreach ($motorId in $AllMotorIds) {
        Enable-Motor $serial $motorId
    }

    $deltaMotor =
        $triggerState.PositionMotorDegrees -
        $before.PositionMotorDegrees
    $deltaJoint = $deltaMotor / $J3GearRatio
    $travelFraction =
        [math]::Abs($deltaMotor) /
        $expectedMotorDegrees

    Write-Host ('before_motor={0:N3}deg after_motor={1:N3}deg delta_motor={2:N3}deg delta_joint={3:N4}deg' -f
        $before.PositionMotorDegrees,
        $triggerState.PositionMotorDegrees,
        $deltaMotor,
        $deltaJoint)
    Write-Host ('before_current={0}mA after_current={1}mA before_status=0x{2:X4} after_status=0x{3:X4} velocity={4}' -f
        $before.CurrentMa,
        $after.CurrentMa,
        $before.Status,
        $after.Status,
        $after.VelocityTenthsRpm)

    if ($ReturnHome -and
        $limitGoalReached -and
        $finalLimitHigh) {
        Write-Host ('zeroed_motor={0:N3}deg zero_error_joint={1:N5}deg limit=HIGH' -f
            $after.PositionMotorDegrees,
            ($after.PositionMotorDegrees / $J3GearRatio))
        if ([math]::Abs($after.PositionMotorDegrees) -gt 0.2) {
            Write-Host 'RESULT=FAIL_HOME_ZERO encoder did not reset to zero.' -ForegroundColor Red
            exit 4
        }
        Write-Host "RESULT=PASS_HOME $LimitLabel LOW->HIGH, STOP, SET_ZERO and encoder read-back passed." -ForegroundColor Green
        exit 0
    }
    if (-not $ReturnHome -and $limitGoalReached) {
        Write-Host "RESULT=PASS_AWAY_DIRECTION $LimitLabel changed HIGH->LOW." -ForegroundColor Green
        exit 0
    }
    if (-not $ReturnHome -and
        $ContinueAfterRelease -and
        $travelFraction -ge 0.95 -and
        $after.FaultFlags -eq 0 -and
        $after.Status -eq $before.Status) {
        Write-Host ("RESULT=PASS_BOUNDED_MOVE {0} reached {1:N4}deg after leaving {2}." -f
            $J3Name,
            $deltaJoint,
            $LimitLabel) -ForegroundColor Green
        exit 0
    }
    if ($ReturnHome) {
        Write-Host "RESULT=FAIL_HOME_NOT_TRIGGERED; maximum bounded return completed without $LimitLabel HIGH." -ForegroundColor Red
        exit 5
    }
    if ($travelFraction -lt 0.30 -or
        $after.FaultFlags -ne 0 -or
        $after.Status -ne $before.Status) {
        Write-Host "RESULT=LIKELY_TOWARD_HARD_STOP_OR_CURRENT_PROTECTION; $LimitLabel stayed HIGH." -ForegroundColor Red
        exit 2
    }
    Write-Host "RESULT=INCONCLUSIVE full small move completed but $LimitLabel stayed HIGH; do not continue automatically." -ForegroundColor Yellow
    exit 3
} catch {
    if ($serial.IsOpen) {
        if ($motionActive) {
            try {
                Stop-J3 $serial
            } catch {
                Write-Host "STOP J3 failed: $($_.Exception.Message)" -ForegroundColor Red
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

param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [double]$ConfiguredGearRatio = 50.0,
    [double]$HomeMaxJointDegrees = 30.0,
    [double]$RotationMaxJointDegrees = 400.0,
    [double]$HomeMotorRpm = 30.0,
    [double]$RotationMotorRpm = 120.0,
    [int]$AccelerationRpmS = 100,
    [int]$ReadTimeoutMs = 4000,
    [double]$RatioTolerancePercent = 3.0,
    [string]$ProgrammerPath,
    [string]$ReportPath,
    [switch]$HomeOnly,
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
$AllMotorIds = @(1, 2, 3, 4, 5, 6)
$J1LimitAddress = '0x48001010'
$J1LimitBit = 7
$RawDirectionCcw = 0
$RawDirectionCw = 1

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
            $positionUrad * 180.0 / 3141593.0
        VelocityTenthsRpm =
            [BitConverter]::ToInt32($response.Payload, 8)
        CurrentMa =
            [BitConverter]::ToUInt16($response.Payload, 12)
        Status =
            [BitConverter]::ToUInt16($response.Payload, 14)
        FaultFlags =
            [BitConverter]::ToUInt32($response.Payload, 16)
        SampleCount =
            if ($response.Payload.Length -ge 32) {
                [BitConverter]::ToUInt32($response.Payload, 28)
            } else {
                [uint32]0
            }
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

function Set-J1Zero([System.IO.Ports.SerialPort]$Serial) {
    Assert-Ok `
        (Invoke-Cmd $Serial $CmdBenchSetZero ([byte[]]@([byte]$J1MotorId))) `
        $CmdBenchSetZero `
        'SET_ZERO J1'
}

function Move-J1(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$RawDirection,
    [double]$MaxJointDegrees,
    [double]$MotorRpm
) {
    $motorDegrees = $MaxJointDegrees * $ConfiguredGearRatio
    $payload = [byte[]]@([byte]$J1MotorId, [byte]$RawDirection) +
        (Write-BeU32 ([uint32][math]::Round($motorDegrees * 10.0))) +
        (Write-BeU16 ([uint16][math]::Round($MotorRpm * 10.0))) +
        (Write-BeU16 ([uint16]$AccelerationRpmS))

    Assert-Ok `
        (Invoke-Cmd $Serial $CmdBenchMoveRel $payload) `
        $CmdBenchMoveRel `
        "MOVE J1 raw_dir=$RawDirection"
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

function Get-MoveTimeoutSeconds(
    [double]$MaxJointDegrees,
    [double]$MotorRpm
) {
    $motorDegrees = $MaxJointDegrees * $ConfiguredGearRatio
    return ($motorDegrees / ($MotorRpm * 6.0)) + 3.0
}

function Get-MonotonicViolationCount(
    [object[]]$PhaseSamples,
    [int]$RawDirection
) {
    $violations = 0
    $previous = $null
    foreach ($sample in $PhaseSamples) {
        if ($null -ne $previous) {
            if ($RawDirection -eq $RawDirectionCcw -and
                $sample.MotorDegrees -lt ($previous - 2.0)) {
                $violations++
            }
            if ($RawDirection -eq $RawDirectionCw -and
                $sample.MotorDegrees -gt ($previous + 2.0)) {
                $violations++
            }
        }
        $previous = $sample.MotorDegrees
    }
    return $violations
}

function Invoke-RotateUntilLimit(
    [System.IO.Ports.SerialPort]$Serial,
    [string]$CliPath,
    [string]$Phase,
    [int]$RawDirection,
    [double]$MaxJointDegrees,
    [double]$MotorRpm,
    [bool]$RequireRelease
) {
    $before = Get-MotorState $Serial $J1MotorId
    $released = -not (Read-J1LimitHigh $CliPath)
    if ($RequireRelease) {
        $released = $false
    }

    Move-J1 $Serial $RawDirection $MaxJointDegrees $MotorRpm
    $script:MotionActive = $true
    $timer = [Diagnostics.Stopwatch]::StartNew()
    $timeoutSeconds = Get-MoveTimeoutSeconds $MaxJointDegrees $MotorRpm
    $index = 0
    $triggered = $false

    while ($timer.Elapsed.TotalSeconds -lt $timeoutSeconds) {
        $state = Get-MotorState $Serial $J1MotorId
        $limitHigh = Read-J1LimitHigh $CliPath
        $jointDegrees = (
            $state.PositionMotorDegrees -
            $before.PositionMotorDegrees) /
            $ConfiguredGearRatio

        $script:Samples.Add([pscustomobject]@{
            Phase = $Phase
            Index = $index
            ElapsedSeconds =
                [math]::Round($timer.Elapsed.TotalSeconds, 4)
            RawDirection = $RawDirection
            MotorDegrees =
                [math]::Round($state.PositionMotorDegrees, 4)
            JointDegreesConfiguredRatio =
                [math]::Round($jointDegrees, 4)
            VelocityTenthsRpm = $state.VelocityTenthsRpm
            CurrentMa = $state.CurrentMa
            LimitHigh = $limitHigh
            EncoderSampleCount = $state.SampleCount
        }) | Out-Null
        $index++

        if (-not $limitHigh) {
            $released = $true
        }
        if ($released -and $limitHigh) {
            $triggered = $true
            break
        }

        if (($index % 20) -eq 0) {
            Write-Host ('  {0}: joint={1:N1}deg motor={2:N1}deg limit={3}' -f
                $Phase,
                $jointDegrees,
                $state.PositionMotorDegrees,
                ($(if ($limitHigh) { 'HIGH' } else { 'LOW' })))
        }
    }

    Stop-J1 $Serial
    $script:MotionActive = $false
    Enable-Motor $Serial $J1MotorId
    Start-Sleep -Milliseconds 200
    $after = Get-MotorState $Serial $J1MotorId

    if (-not $triggered) {
        throw "$Phase did not find the J1 limit before the bounded move ended"
    }
    if (-not (Read-J1LimitHigh $CliPath)) {
        throw "$Phase limit signal was not stable HIGH after STOP"
    }

    $phaseSamples = @(
        $script:Samples | Where-Object { $_.Phase -eq $Phase }
    )
    $deltaMotorDegrees =
        $after.PositionMotorDegrees -
        $before.PositionMotorDegrees
    $jointDegreesConfigured =
        [math]::Abs($deltaMotorDegrees) /
        $ConfiguredGearRatio
    $measuredRatio =
        [math]::Abs($deltaMotorDegrees) / 360.0
    $ratioErrorPercent =
        [math]::Abs(
            $measuredRatio - $ConfiguredGearRatio) /
        $ConfiguredGearRatio * 100.0
    $violations = Get-MonotonicViolationCount `
        $phaseSamples `
        $RawDirection
    $pass =
        $ratioErrorPercent -le $RatioTolerancePercent -and
        $violations -eq 0

    return [pscustomobject]@{
        Phase = $Phase
        RawDirection = $RawDirection
        Samples = $phaseSamples.Count
        StartMotorDegrees =
            [math]::Round($before.PositionMotorDegrees, 3)
        EndMotorDegrees =
            [math]::Round($after.PositionMotorDegrees, 3)
        DeltaMotorDegrees =
            [math]::Round($deltaMotorDegrees, 3)
        JointDegreesConfiguredRatio =
            [math]::Round($jointDegreesConfigured, 3)
        MeasuredGearRatio =
            [math]::Round($measuredRatio, 5)
        RatioErrorPercent =
            [math]::Round($ratioErrorPercent, 4)
        MonotonicViolations = $violations
        Verdict = if ($pass) { 'PASS' } else { 'FAIL' }
    }
}

if ($ConfiguredGearRatio -le 0) {
    throw 'ConfiguredGearRatio must be positive'
}
if ($HomeMaxJointDegrees -le 0 -or $HomeMaxJointDegrees -gt 420) {
    throw 'HomeMaxJointDegrees must be in (0, 420]'
}
if ($RotationMaxJointDegrees -lt 370 -or
    $RotationMaxJointDegrees -gt 420) {
    throw 'RotationMaxJointDegrees must be between 370 and 420'
}
if ($HomeMotorRpm -le 0 -or $HomeMotorRpm -gt 60 -or
    $RotationMotorRpm -le 0 -or $RotationMotorRpm -gt 180) {
    throw 'Motor RPM exceeds the script safety bound'
}
if ($AccelerationRpmS -le 0 -or $AccelerationRpmS -gt 200) {
    throw 'AccelerationRpmS must be in (0, 200]'
}

$cliPath = Resolve-Programmer $ProgrammerPath
$initialLimitHigh = Read-J1LimitHigh $cliPath
$script:Samples =
    New-Object System.Collections.Generic.List[object]
$results =
    New-Object System.Collections.Generic.List[object]
$script:MotionActive = $false
$fatal = $null
$homeResult = $null
$finalZeroState = $null

Write-Host '==== ZEROARM J1 FULL-ROTATION HOME/ENCODER TEST ===='
Write-Host ('initial_limit={0}, configured_ratio={1}:1' -f
    ($(if ($initialLimitHigh) { 'HIGH' } else { 'LOW' })),
    $ConfiguredGearRatio)
Write-Host ('home=CW(raw 1), validation=CCW(raw 0)+CW(raw 1), max={0}deg each' -f
    $RotationMaxJointDegrees)
Write-Host 'No DISABLE command will be sent.'

if (-not $Execute) {
    Write-Host 'DIAGNOSTIC ONLY: pass -Execute to allow motion.'
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
$enabledMotorIds =
    New-Object System.Collections.Generic.List[int]

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    $hello = Invoke-Cmd $serial $CmdHello @()
    $helloText = [Text.Encoding]::ASCII.GetString($hello.Payload)
    if ($hello.Command -ne $CmdHello -or
        $helloText -ne 'ZEROARM/1.0') {
        throw "HELLO failed: $helloText"
    }

    Write-Host 'Checking and enabling M1..M6 for holding torque...'
    foreach ($motorId in $AllMotorIds) {
        $state = Get-MotorState $serial $motorId
        if (-not $state.Online -or $state.FaultFlags -ne 0) {
            throw "M$motorId offline or faulted; refusing motion"
        }
        Enable-Motor $serial $motorId
        $enabledMotorIds.Add($motorId) | Out-Null
    }

    Stop-J1 $serial
    Enable-Motor $serial $J1MotorId
    Write-Host 'STOP path PASS; all motors ENABLED.'

    Write-Host 'Stage 1/3: CW search for J1 zero limit...'
    $homeResult = Invoke-RotateUntilLimit `
        $serial `
        $cliPath `
        'HOME_CW' `
        $RawDirectionCw `
        $HomeMaxJointDegrees `
        $HomeMotorRpm `
        $false
    Set-J1Zero $serial
    Enable-Motor $serial $J1MotorId
    Start-Sleep -Milliseconds 300
    $finalZeroState = Get-MotorState $serial $J1MotorId
    if ([math]::Abs($finalZeroState.PositionMotorDegrees) -gt 5.0) {
        throw ('J1 SET_ZERO verification failed: encoder={0:N3} motor deg' -f
            $finalZeroState.PositionMotorDegrees)
    }
    if (-not (Read-J1LimitHigh $cliPath)) {
        throw 'J1 SET_ZERO verification failed: PE7 is not stable HIGH'
    }
    Write-Host ('  HOME PASS at motor={0:N2}deg; zero written.' -f
        $homeResult.EndMotorDegrees) -ForegroundColor Green

    if (-not $HomeOnly) {
        Write-Host 'Stage 2/3: CCW one-turn encoder/ratio validation...'
        $ccw = Invoke-RotateUntilLimit `
            $serial `
            $cliPath `
            'ONE_TURN_CCW' `
            $RawDirectionCcw `
            $RotationMaxJointDegrees `
            $RotationMotorRpm `
            $true
        $results.Add($ccw) | Out-Null
        Set-J1Zero $serial
        Enable-Motor $serial $J1MotorId
        Write-Host ('  CCW {0}: motor_delta={1:N2}deg joint={2:N3}deg measured_ratio={3:N5}:1 violations={4}' -f
            $ccw.Verdict,
            $ccw.DeltaMotorDegrees,
            $ccw.JointDegreesConfiguredRatio,
            $ccw.MeasuredGearRatio,
            $ccw.MonotonicViolations) -ForegroundColor Green

        Write-Host 'Stage 3/3: CW one-turn encoder/ratio validation...'
        $cw = Invoke-RotateUntilLimit `
            $serial `
            $cliPath `
            'ONE_TURN_CW' `
            $RawDirectionCw `
            $RotationMaxJointDegrees `
            $RotationMotorRpm `
            $true
        $results.Add($cw) | Out-Null
        Set-J1Zero $serial
        Write-Host ('  CW {0}: motor_delta={1:N2}deg joint={2:N3}deg measured_ratio={3:N5}:1 violations={4}' -f
            $cw.Verdict,
            $cw.DeltaMotorDegrees,
            $cw.JointDegreesConfiguredRatio,
            $cw.MeasuredGearRatio,
            $cw.MonotonicViolations) -ForegroundColor Green
    }

    foreach ($motorId in $AllMotorIds) {
        Enable-Motor $serial $motorId
    }
} catch {
    $fatal = $_.Exception.Message
    if ($serial.IsOpen) {
        if ($script:MotionActive) {
            try {
                Stop-J1 $serial
                $script:MotionActive = $false
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
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}

$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
if ([string]::IsNullOrWhiteSpace($ReportPath)) {
    $ReportPath = Join-Path `
        $PSScriptRoot `
        "j1_full_rotation_report_$stamp.txt"
}
$samplePath = [IO.Path]::ChangeExtension(
    $ReportPath,
    '.samples.csv')

$reportLines =
    New-Object System.Collections.Generic.List[string]
$reportLines.Add('ZEROARM J1 FULL-ROTATION HOME/ENCODER REPORT')
$reportLines.Add("time=$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')")
$reportLines.Add('direction.raw0=top-view CCW')
$reportLines.Add('direction.raw1=top-view CW')
$reportLines.Add('continuous_rotation=true')
$reportLines.Add("configured_gear_ratio=$ConfiguredGearRatio")
$reportLines.Add('limit.active_level=HIGH')
if ($null -ne $homeResult -and $null -ne $finalZeroState) {
    $reportLines.Add(
        ('HOME_CW: verdict=PASS search_motor_delta_deg={0} search_joint_deg={1} final_encoder_motor_deg={2} final_limit=HIGH zero_written=true' -f
            $homeResult.DeltaMotorDegrees,
            $homeResult.JointDegreesConfiguredRatio,
            ([math]::Round($finalZeroState.PositionMotorDegrees, 3))))
}
foreach ($result in $results) {
    $reportLines.Add(
        ('{0}: verdict={1} samples={2} motor_delta_deg={3} joint_deg_ratio_calc={4} measured_ratio={5} ratio_error_percent={6} monotonic_violations={7}' -f
            $result.Phase,
            $result.Verdict,
            $result.Samples,
            $result.DeltaMotorDegrees,
            $result.JointDegreesConfiguredRatio,
            $result.MeasuredGearRatio,
            $result.RatioErrorPercent,
            $result.MonotonicViolations))
}
if ($null -ne $fatal) {
    $reportLines.Add("overall=FAIL error=$fatal")
} elseif (@($results | Where-Object { $_.Verdict -ne 'PASS' }).Count -gt 0) {
    $reportLines.Add('overall=FAIL validation_out_of_tolerance')
} else {
    $reportLines.Add('overall=PASS')
}
$reportLines.Add("samples_file=$samplePath")

$reportLines | Set-Content -LiteralPath $ReportPath -Encoding UTF8
$script:Samples |
    Export-Csv -LiteralPath $samplePath -NoTypeInformation -Encoding UTF8

Write-Host ''
Write-Host ($reportLines -join [Environment]::NewLine)
Write-Host "report_file=$ReportPath"
Write-Host "samples_file=$samplePath"

if ($null -ne $fatal) {
    throw $fatal
}
if (@($results | Where-Object { $_.Verdict -ne 'PASS' }).Count -gt 0) {
    exit 2
}

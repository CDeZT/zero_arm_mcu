param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [int[]]$MotorIds = @(1, 2, 3, 4, 5, 6),
    [double]$Turns = 300.0,
    [double]$VelocityRpm = 1500.0,
    [int]$AccelerationRpmS = 800,
    [int]$ExtraSettleMs = 1500,
    [int]$ReadTimeoutMs = 4000,
    [double]$ToleranceFraction = 0.10,
    [double]$MinAbsoluteToleranceDeg = 30.0,
    [switch]$ConfirmBareMotor,
    [switch]$ScanOnly,
    [switch]$VerboseHex
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Batch motor validation for ZeroArm bench firmware.
# Default: scan IDs then motion-test only online motors when -ConfirmBareMotor is set.

$CmdHello = 0x00
$CmdGetState = 0x01
$CmdBenchQuery = 0x20
$CmdBenchEnable = 0x21
$CmdBenchDisable = 0x22
$CmdBenchStop = 0x23
$CmdBenchMoveRel = 0x24

$RobotStatePayloadSize = 60
$MaxTurns = 1500.0
$MaxVelocityRpm = 1500.0
$MaxAcceleration = 2000

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

function To-Hex([byte[]]$Bytes) {
    return (($Bytes | ForEach-Object { '{0:X2}' -f $_ }) -join ' ')
}

function New-Frame([byte]$Command, [byte[]]$Payload) {
    if ($null -eq $Payload) {
        $Payload = @()
    }

    [byte[]]$body = New-Object byte[] ($Payload.Length + 1)
    $body[0] = $Command
    for ($i = 0; $i -lt $Payload.Length; $i++) {
        $body[$i + 1] = [byte]$Payload[$i]
    }

    [byte]$length = [byte]$body.Length
    [byte]$crc = Get-Crc8 $body
    [byte[]]$frame = New-Object byte[] ($body.Length + 4)
    $frame[0] = 0xAA
    $frame[1] = $length
    for ($i = 0; $i -lt $body.Length; $i++) {
        $frame[$i + 2] = $body[$i]
    }
    $frame[$body.Length + 2] = $crc
    $frame[$body.Length + 3] = 0x55
    return , $frame
}

function Write-Frame(
    [System.IO.Ports.SerialPort]$Serial,
    [byte[]]$Frame
) {
    if ($VerboseHex) {
        Write-Host ("TX: {0}" -f (To-Hex $Frame))
    }
    $Serial.Write($Frame, 0, $Frame.Length)
}

function Read-Frame(
    [System.IO.Ports.SerialPort]$Serial
) {
    do {
        $stx = $Serial.ReadByte()
    } while ($stx -ne 0xAA)

    $length = $Serial.ReadByte()
    if ($length -le 0 -or $length -ge 128) {
        throw "Invalid response length: $length"
    }

    [byte[]]$body = New-Object byte[] $length
    for ($i = 0; $i -lt $length; $i++) {
        $body[$i] = [byte]$Serial.ReadByte()
    }

    [byte]$rxCrc = [byte]$Serial.ReadByte()
    $etx = $Serial.ReadByte()
    if ($etx -ne 0x55) {
        throw ('Invalid ETX: 0x{0:X2}' -f $etx)
    }

    [byte]$calc = Get-Crc8 $body
    if ($rxCrc -ne $calc) {
        throw ('CRC mismatch rx=0x{0:X2} exp=0x{1:X2}' -f $rxCrc, $calc)
    }

    [byte[]]$payload = @()
    if ($length -gt 1) {
        $payload = New-Object byte[] ($length - 1)
        for ($i = 0; $i -lt ($length - 1); $i++) {
            $payload[$i] = $body[$i + 1]
        }
    }

    [byte[]]$all = New-Object byte[] ($length + 4)
    $all[0] = 0xAA
    $all[1] = [byte]$length
    for ($i = 0; $i -lt $length; $i++) {
        $all[$i + 2] = $body[$i]
    }
    $all[$length + 2] = $rxCrc
    $all[$length + 3] = 0x55

    if ($VerboseHex) {
        Write-Host ("RX: {0}" -f (To-Hex $all))
    }

    return [pscustomobject]@{
        Command = $body[0]
        Payload = $payload
        Hex = To-Hex $all
    }
}

function Invoke-Cmd(
    [System.IO.Ports.SerialPort]$Serial,
    [byte]$Command,
    [byte[]]$Payload
) {
    if ($null -eq $Payload) {
        $Payload = @()
    }
    $frame = New-Frame $Command $Payload
    Write-Frame $Serial $frame
    return Read-Frame $Serial
}

function Assert-Ok(
    [pscustomobject]$Response,
    [byte]$Command,
    [string]$Step
) {
    if ($Response.Command -ne $Command) {
        throw ('{0}: unexpected cmd 0x{1:X2}' -f $Step, $Response.Command)
    }
    if ($Response.Payload.Length -ne 1) {
        throw ('{0}: expected 1-byte result, got {1}' -f $Step, $Response.Payload.Length)
    }
    if ($Response.Payload[0] -ne 0) {
        throw ('{0}: error code {1}' -f $Step, $Response.Payload[0])
    }
}

function Parse-BenchState([byte[]]$Payload) {
    if ($Payload.Length -ne 28 -and $Payload.Length -ne 32) {
        throw (
            'Unexpected bench state length {0}. Need bench firmware (28/32 bytes).' -f
            $Payload.Length)
    }

    $pos = [BitConverter]::ToInt32($Payload, 4)
    $sample = if ($Payload.Length -ge 32) {
        [BitConverter]::ToUInt32($Payload, 28)
    } else {
        [uint32]0
    }

    return [pscustomobject]@{
        MotorId = $Payload[0]
        Online = $Payload[1]
        PositionUrad = $pos
        PositionDegrees = [math]::Round($pos * 180.0 / 3141593.0, 3)
        Velocity = [BitConverter]::ToInt32($Payload, 8)
        CurrentMa = [BitConverter]::ToUInt16($Payload, 12)
        Status = [BitConverter]::ToUInt16($Payload, 14)
        FaultFlags = [BitConverter]::ToUInt32($Payload, 16)
        CanTxErrors = [BitConverter]::ToUInt32($Payload, 20)
        FeedbackFaults = [BitConverter]::ToUInt32($Payload, 24)
        Sample = $sample
        Bytes = $Payload.Length
    }
}

function Get-BenchState(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId
) {
    $response = Invoke-Cmd $Serial $CmdBenchQuery ([byte[]]@([byte]$MotorId))
    if ($response.Payload.Length -eq 1) {
        return [pscustomobject]@{
            MotorId = $MotorId
            Online = 0
            PositionDegrees = 0
            Sample = 0
            Bytes = 1
            ErrorCode = $response.Payload[0]
            Rejected = $true
        }
    }

    $state = Parse-BenchState $response.Payload
    $state | Add-Member -NotePropertyName Rejected -NotePropertyValue $false
    $state | Add-Member -NotePropertyName ErrorCode -NotePropertyValue 0
    return $state
}

function Get-BenchStatePolled(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId,
    [int]$Polls = 4,
    [int]$DelayMs = 120
) {
    $last = $null
    for ($i = 0; $i -lt $Polls; $i++) {
        $last = Get-BenchState $Serial $MotorId
        if ($last.Rejected) {
            return $last
        }
        if ($last.Online -ne 0) {
            if ($last.Bytes -eq 28 -and $i -lt 1) {
                Start-Sleep -Milliseconds $DelayMs
                continue
            }
            return $last
        }
        Start-Sleep -Milliseconds $DelayMs
    }
    return $last
}

function Get-WaitMs(
    [double]$TurnsValue,
    [double]$VelocityValue,
    [int]$ExtraMs
) {
    $seconds = ($TurnsValue / $VelocityValue) * 60.0
    $wait = [int][math]::Ceiling(($seconds * 1.40 * 1000.0) + $ExtraMs)
    if ($wait -lt 1500) {
        $wait = 1500
    }
    return $wait
}

function Write-BeU16([uint16]$Value) {
    return [byte[]]@(
        [byte](($Value -shr 8) -band 0xFF),
        [byte]($Value -band 0xFF)
    )
}

function Write-BeU32([uint32]$Value) {
    return [byte[]]@(
        [byte](($Value -shr 24) -band 0xFF),
        [byte](($Value -shr 16) -band 0xFF),
        [byte](($Value -shr 8) -band 0xFF),
        [byte]($Value -band 0xFF)
    )
}

function Invoke-SafeStopDisable(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId
) {
    try {
        $stop = Invoke-Cmd $Serial $CmdBenchStop ([byte[]]@([byte]$MotorId))
        Assert-Ok $stop $CmdBenchStop 'STOP'
    } catch {
        Write-Host ("  safe_stop=FAIL motor={0} detail={1}" -f $MotorId, $_.Exception.Message)
    }

    try {
        $disable = Invoke-Cmd $Serial $CmdBenchDisable ([byte[]]@([byte]$MotorId))
        Assert-Ok $disable $CmdBenchDisable 'DISABLE'
    } catch {
        Write-Host ("  safe_disable=FAIL motor={0} detail={1}" -f $MotorId, $_.Exception.Message)
    }
}

function Test-OneMotorMotion(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId,
    [double]$TurnsValue,
    [double]$VelocityValue,
    [int]$Accel,
    [int]$WaitMs,
    [double]$TolFrac,
    [double]$MinTolDeg
) {
    $result = [ordered]@{
        MotorId = $MotorId
        Online = $false
        Motion = 'SKIP'
        BeforeDeg = $null
        AfterFwDeg = $null
        AfterRvDeg = $null
        DeltaFwDeg = $null
        ResidualDeg = $null
        Pass = $false
        Detail = ''
    }

    $before = Get-BenchStatePolled $Serial $MotorId
    if ($before.Rejected -or $before.Online -eq 0) {
        $result.Detail = if ($before.Rejected) {
            "query rejected code=$($before.ErrorCode)"
        } else {
            'offline'
        }
        return [pscustomobject]$result
    }

    $result.Online = $true
    $result.BeforeDeg = $before.PositionDegrees

    $enable = Invoke-Cmd $Serial $CmdBenchEnable ([byte[]]@([byte]$MotorId))
    Assert-Ok $enable $CmdBenchEnable 'ENABLE'
    Start-Sleep -Milliseconds 250

    $degreesTenths = [uint32][math]::Round($TurnsValue * 3600.0)
    $velocityTenths = [uint16][math]::Round($VelocityValue * 10.0)
    $acceleration = [uint16]$Accel

    $fwPayload = [byte[]]@(
        [byte]$MotorId,
        [byte]0
    ) + (Write-BeU32 $degreesTenths) + (Write-BeU16 $velocityTenths) + (Write-BeU16 $acceleration)

    $fw = Invoke-Cmd $Serial $CmdBenchMoveRel $fwPayload
    Assert-Ok $fw $CmdBenchMoveRel 'MOVE_FW'
    Start-Sleep -Milliseconds $WaitMs

    $mid = Get-BenchStatePolled $Serial $MotorId
    $result.AfterFwDeg = $mid.PositionDegrees
    $result.DeltaFwDeg = [math]::Round($mid.PositionDegrees - $before.PositionDegrees, 3)

    $rvPayload = [byte[]]@(
        [byte]$MotorId,
        [byte]1
    ) + (Write-BeU32 $degreesTenths) + (Write-BeU16 $velocityTenths) + (Write-BeU16 $acceleration)

    $rv = Invoke-Cmd $Serial $CmdBenchMoveRel $rvPayload
    Assert-Ok $rv $CmdBenchMoveRel 'MOVE_RV'
    Start-Sleep -Milliseconds $WaitMs

    $after = Get-BenchStatePolled $Serial $MotorId
    $result.AfterRvDeg = $after.PositionDegrees
    $result.ResidualDeg = [math]::Round($after.PositionDegrees - $before.PositionDegrees, 3)

    Invoke-SafeStopDisable $Serial $MotorId

    $expected = $TurnsValue * 360.0
    $minForward = [math]::Max($MinTolDeg, $expected * (1.0 - $TolFrac))
    # Accept either sign depending on motor direction mapping.
    $absDelta = [math]::Abs([double]$result.DeltaFwDeg)
    $absResidual = [math]::Abs([double]$result.ResidualDeg)
    $maxResidual = [math]::Max($MinTolDeg, $expected * $TolFrac)

    if ($absDelta -lt $minForward) {
        $result.Motion = 'FAIL'
        $result.Detail = (
            'forward too small: delta={0} expected~{1}' -f
            $result.DeltaFwDeg,
            $expected)
        $result.Pass = $false
        return [pscustomobject]$result
    }

    if ($absResidual -gt $maxResidual) {
        $result.Motion = 'FAIL'
        $result.Detail = (
            'did not return: residual={0} allowed={1}' -f
            $result.ResidualDeg,
            $maxResidual)
        $result.Pass = $false
        return [pscustomobject]$result
    }

    $result.Motion = 'PASS'
    $result.Detail = (
        'fw_delta={0} residual={1} sign={2}' -f
        $result.DeltaFwDeg,
        $result.ResidualDeg,
        ($(if ($result.DeltaFwDeg -ge 0) { '+' } else { '-' })))
    $result.Pass = $true
    return [pscustomobject]$result
}

# ---- validate params ----
foreach ($id in $MotorIds) {
    if ($id -lt 1 -or $id -gt 6) {
        throw "MotorIds must be 1..6, got $id"
    }
}
if ($Turns -le 0 -or $Turns -gt $MaxTurns) {
    throw "Turns must be in (0, $MaxTurns]"
}
if ($VelocityRpm -le 0 -or $VelocityRpm -gt $MaxVelocityRpm) {
    throw "VelocityRpm must be in (0, $MaxVelocityRpm]"
}
if ($AccelerationRpmS -le 0 -or $AccelerationRpmS -gt $MaxAcceleration) {
    throw "AccelerationRpmS must be in (0, $MaxAcceleration]"
}

$waitMs = Get-WaitMs $Turns $VelocityRpm $ExtraSettleMs
$doMotion = $ConfirmBareMotor.IsPresent -and (-not $ScanOnly.IsPresent)

Write-Host '==== ZEROARM multi-motor batch test ===='
Write-Host ("port={0} motors={1} turns={2} rpm={3} wait_ms={4} motion={5}" -f
    $Port,
    ($MotorIds -join ','),
    $Turns,
    $VelocityRpm,
    $waitMs,
    $doMotion)

$serial = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, None, 8, One
$serial.ReadTimeout = $ReadTimeoutMs
$serial.WriteTimeout = $ReadTimeoutMs
$serial.DtrEnable = $false
$serial.RtsEnable = $false

$report = New-Object System.Collections.Generic.List[object]
$onlineIds = New-Object System.Collections.Generic.List[int]

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    Write-Host '--- Stage A: UART / MCU ---'
    $hello = Invoke-Cmd $serial $CmdHello @()
    $helloText = [Text.Encoding]::ASCII.GetString($hello.Payload)
    if ($hello.Command -ne $CmdHello -or $helloText -ne 'ZEROARM/1.0') {
        throw ("HELLO failed: {0}" -f $hello.Hex)
    }
    Write-Host 'UART_HELLO=PASS ZEROARM/1.0'

    $state = Invoke-Cmd $serial $CmdGetState @()
    if ($state.Command -ne $CmdGetState -or $state.Payload.Length -ne $RobotStatePayloadSize) {
        throw ("GET_STATE failed length={0}" -f $state.Payload.Length)
    }
    $runState = [BitConverter]::ToInt32($state.Payload, 0)
    $fault = [BitConverter]::ToUInt32($state.Payload, 56)
    Write-Host ("GET_STATE=PASS run_state={0} fault=0x{1:X8}" -f $runState, $fault)

    Write-Host '--- Stage B: scan motor IDs ---'
    foreach ($id in $MotorIds) {
        try {
            $q = Get-BenchStatePolled $serial $id
            if ($q.Rejected) {
                Write-Host ("motor {0}: REJECT code={1}" -f $id, $q.ErrorCode)
                $report.Add([pscustomobject]@{
                        MotorId = $id
                        Online = $false
                        Motion = 'SKIP'
                        BeforeDeg = $null
                        AfterFwDeg = $null
                        AfterRvDeg = $null
                        DeltaFwDeg = $null
                        ResidualDeg = $null
                        Pass = $false
                        Detail = "reject code=$($q.ErrorCode)"
                    }) | Out-Null
                continue
            }

            if ($q.Online -ne 0) {
                Write-Host (
                    "motor {0}: ONLINE pos={1} deg status=0x{2:X4} fault=0x{3:X8}" -f
                    $id,
                    $q.PositionDegrees,
                    $q.Status,
                    $q.FaultFlags)
                $onlineIds.Add($id) | Out-Null
                if (-not $doMotion) {
                    $report.Add([pscustomobject]@{
                            MotorId = $id
                            Online = $true
                            Motion = 'SKIP'
                            BeforeDeg = $q.PositionDegrees
                            AfterFwDeg = $null
                            AfterRvDeg = $null
                            DeltaFwDeg = $null
                            ResidualDeg = $null
                            Pass = $true
                            Detail = 'online only'
                        }) | Out-Null
                }
            } else {
                Write-Host ("motor {0}: OFFLINE" -f $id)
                $report.Add([pscustomobject]@{
                        MotorId = $id
                        Online = $false
                        Motion = 'SKIP'
                        BeforeDeg = $null
                        AfterFwDeg = $null
                        AfterRvDeg = $null
                        DeltaFwDeg = $null
                        ResidualDeg = $null
                        Pass = $false
                        Detail = 'offline (check address/CAN/power/120ohm)'
                    }) | Out-Null
            }
        } catch {
            Write-Host ("motor {0}: ERROR {1}" -f $id, $_.Exception.Message)
            $report.Add([pscustomobject]@{
                    MotorId = $id
                    Online = $false
                    Motion = 'SKIP'
                    BeforeDeg = $null
                    AfterFwDeg = $null
                    AfterRvDeg = $null
                    DeltaFwDeg = $null
                    ResidualDeg = $null
                    Pass = $false
                    Detail = $_.Exception.Message
                }) | Out-Null
        }
    }

    Write-Host ("scan_summary online=[{0}] count={1}" -f (($onlineIds -join ','), $onlineIds.Count))

    if (-not $doMotion) {
        Write-Host 'motion=SKIPPED (use -ConfirmBareMotor to run move tests)'
    } else {
        Write-Host '--- Stage C: motion test online motors ---'
        foreach ($id in $onlineIds) {
            Write-Host ("testing motor {0} ..." -f $id)
            try {
                $one = Test-OneMotorMotion `
                    $serial $id $Turns $VelocityRpm $AccelerationRpmS `
                    $waitMs $ToleranceFraction $MinAbsoluteToleranceDeg
                $report.Add($one) | Out-Null
                Write-Host (
                    "motor {0}: motion={1} delta={2} residual={3} detail={4}" -f
                    $one.MotorId,
                    $one.Motion,
                    $one.DeltaFwDeg,
                    $one.ResidualDeg,
                    $one.Detail)
            } catch {
                Invoke-SafeStopDisable $serial $id
                $report.Add([pscustomobject]@{
                        MotorId = $id
                        Online = $true
                        Motion = 'FAIL'
                        BeforeDeg = $null
                        AfterFwDeg = $null
                        AfterRvDeg = $null
                        DeltaFwDeg = $null
                        ResidualDeg = $null
                        Pass = $false
                        Detail = $_.Exception.Message
                    }) | Out-Null
                Write-Host ("motor {0}: motion=FAIL detail={1}" -f $id, $_.Exception.Message)
            }
        }
    }

    Write-Host ''
    Write-Host '==== REPORT ===='
    $report |
        Sort-Object MotorId |
        Format-Table MotorId, Online, Motion, BeforeDeg, DeltaFwDeg, ResidualDeg, Pass, Detail -AutoSize |
        Out-String |
        Write-Host

    $onlineCount = @($report | Where-Object { $_.Online }).Count
    $motionPass = @($report | Where-Object { $_.Motion -eq 'PASS' }).Count
    $motionFail = @($report | Where-Object { $_.Motion -eq 'FAIL' }).Count

    Write-Host ("SUMMARY online={0}/{1} motion_pass={2} motion_fail={3}" -f
        $onlineCount,
        $MotorIds.Count,
        $motionPass,
        $motionFail)

    if ($doMotion) {
        if ($motionFail -gt 0 -or $motionPass -eq 0) {
            Write-Host 'overall=FAIL'
            exit 1
        }
        Write-Host 'overall=PASS'
        exit 0
    } else {
        if ($onlineCount -eq 0) {
            Write-Host 'overall=FAIL_NO_MOTOR_ONLINE'
            exit 2
        }
        Write-Host 'overall=PASS_SCAN_ONLY'
        exit 0
    }
} catch {
    Write-Host ("overall=FAIL error={0}" -f $_.Exception.Message)
    exit 3
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}

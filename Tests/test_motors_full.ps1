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
    [string]$ReportPath = '',
    [switch]$ScanOnly,
    [switch]$VerboseHex
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# One-shot full motor bench:
# 1) UART/MCU
# 2) scan MotorIds
# 3) motion test each online motor (unless -ScanOnly)
# 4) print final report and write optional report file

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
    if ($null -eq $Payload) { $Payload = @() }
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

function Write-Frame([System.IO.Ports.SerialPort]$Serial, [byte[]]$Frame) {
    if ($VerboseHex) { Write-Host ("TX: {0}" -f (To-Hex $Frame)) }
    $Serial.Write($Frame, 0, $Frame.Length)
}

function Read-Frame([System.IO.Ports.SerialPort]$Serial) {
    do { $stx = $Serial.ReadByte() } while ($stx -ne 0xAA)
    $length = $Serial.ReadByte()
    if ($length -le 0 -or $length -ge 128) { throw "Invalid response length: $length" }
    [byte[]]$body = New-Object byte[] $length
    for ($i = 0; $i -lt $length; $i++) { $body[$i] = [byte]$Serial.ReadByte() }
    [byte]$rxCrc = [byte]$Serial.ReadByte()
    $etx = $Serial.ReadByte()
    if ($etx -ne 0x55) { throw ('Invalid ETX: 0x{0:X2}' -f $etx) }
    [byte]$calc = Get-Crc8 $body
    if ($rxCrc -ne $calc) { throw ('CRC mismatch rx=0x{0:X2} exp=0x{1:X2}' -f $rxCrc, $calc) }

    [byte[]]$payload = @()
    if ($length -gt 1) {
        $payload = New-Object byte[] ($length - 1)
        for ($i = 0; $i -lt ($length - 1); $i++) { $payload[$i] = $body[$i + 1] }
    }

    [byte[]]$all = New-Object byte[] ($length + 4)
    $all[0] = 0xAA
    $all[1] = [byte]$length
    for ($i = 0; $i -lt $length; $i++) { $all[$i + 2] = $body[$i] }
    $all[$length + 2] = $rxCrc
    $all[$length + 3] = 0x55
    if ($VerboseHex) { Write-Host ("RX: {0}" -f (To-Hex $all)) }

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
    if ($null -eq $Payload) { $Payload = @() }
    Write-Frame $Serial (New-Frame $Command $Payload)
    return Read-Frame $Serial
}

function Assert-Ok([pscustomobject]$Response, [byte]$Command, [string]$Step) {
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
        throw ('Unexpected bench state length {0}' -f $Payload.Length)
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

function Get-ErrorName([int]$Code) {
    switch ($Code) {
        0 { return 'OK' }
        1 { return 'ARGUMENT' }
        2 { return 'STATE' }
        3 { return 'RANGE' }
        4 { return 'NOT_READY' }
        5 { return 'NOT_CONFIGURED' }
        6 { return 'QUEUE_FULL' }
        7 { return 'IO_OR_TIMEOUT' }
        8 { return 'NOT_IMPLEMENTED' }
        default { return ("CODE_{0}" -f $Code) }
    }
}

function Get-BenchState([System.IO.Ports.SerialPort]$Serial, [int]$MotorId) {
    $response = Invoke-Cmd $Serial $CmdBenchQuery ([byte[]]@([byte]$MotorId))
    if ($response.Payload.Length -eq 1) {
        $code = [int]$response.Payload[0]
        # ROBOT_ERR_IO(7) after query timeout => motor not answering on CAN.
        $link = if ($code -eq 7) { 'OFFLINE' } else { 'REJECT' }
        return [pscustomobject]@{
            MotorId = $MotorId
            Online = 0
            PositionDegrees = 0
            Status = 0
            FaultFlags = 0
            Sample = 0
            Bytes = 1
            Rejected = $true
            ErrorCode = $code
            LinkHint = $link
            ErrorName = Get-ErrorName $code
        }
    }
    $state = Parse-BenchState $response.Payload
    $state | Add-Member -NotePropertyName Rejected -NotePropertyValue $false
    $state | Add-Member -NotePropertyName ErrorCode -NotePropertyValue 0
    $state | Add-Member -NotePropertyName LinkHint -NotePropertyValue 'PASS'
    $state | Add-Member -NotePropertyName ErrorName -NotePropertyValue 'OK'
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
        if ($last.Rejected) { return $last }
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

function Get-WaitMs([double]$TurnsValue, [double]$VelocityValue, [int]$ExtraMs) {
    $seconds = ($TurnsValue / $VelocityValue) * 60.0
    $wait = [int][math]::Ceiling(($seconds * 1.40 * 1000.0) + $ExtraMs)
    if ($wait -lt 1500) { $wait = 1500 }
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

function Invoke-SafeStopDisable([System.IO.Ports.SerialPort]$Serial, [int]$MotorId) {
    try {
        Assert-Ok (Invoke-Cmd $Serial $CmdBenchStop ([byte[]]@([byte]$MotorId))) $CmdBenchStop 'STOP'
    } catch {
        Write-Host ("  safe_stop=FAIL id={0} {1}" -f $MotorId, $_.Exception.Message)
    }
    try {
        Assert-Ok (Invoke-Cmd $Serial $CmdBenchDisable ([byte[]]@([byte]$MotorId))) $CmdBenchDisable 'DISABLE'
    } catch {
        Write-Host ("  safe_disable=FAIL id={0} {1}" -f $MotorId, $_.Exception.Message)
    }
}

function Test-OneMotor(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId,
    [double]$TurnsValue,
    [double]$VelocityValue,
    [int]$Accel,
    [int]$WaitMs,
    [double]$TolFrac,
    [double]$MinTolDeg,
    [bool]$DoMotion
) {
    $row = [ordered]@{
        MotorId = $MotorId
        Online = $false
        Link = 'FAIL'
        Motion = 'SKIP'
        BeforeDeg = $null
        AfterFwDeg = $null
        AfterRvDeg = $null
        DeltaFwDeg = $null
        ResidualDeg = $null
        DirSign = ''
        Verdict = 'FAIL'
        Detail = ''
    }

    $q = Get-BenchStatePolled $Serial $MotorId
    if ($q.Rejected) {
        $row.Link = $q.LinkHint
        if ($q.ErrorCode -eq 7) {
            $row.Detail = 'no CAN response (addr/power/wiring/120ohm/baud)'
        } else {
            $row.Detail = ("query reject {0}({1})" -f $q.ErrorName, $q.ErrorCode)
        }
        return [pscustomobject]$row
    }
    if ($q.Online -eq 0) {
        $row.Detail = 'offline: address/CAN/power/120ohm/baud'
        $row.Link = 'OFFLINE'
        return [pscustomobject]$row
    }

    $row.Online = $true
    $row.Link = 'PASS'
    $row.BeforeDeg = $q.PositionDegrees

    if (-not $DoMotion) {
        $row.Motion = 'SKIP'
        $row.Verdict = 'PASS'
        $row.Detail = ("online pos={0} status=0x{1:X4}" -f $q.PositionDegrees, $q.Status)
        return [pscustomobject]$row
    }

    try {
        Assert-Ok (Invoke-Cmd $Serial $CmdBenchEnable ([byte[]]@([byte]$MotorId))) $CmdBenchEnable 'ENABLE'
        Start-Sleep -Milliseconds 250

        $before = Get-BenchStatePolled $Serial $MotorId
        $row.BeforeDeg = $before.PositionDegrees

        $degreesTenths = [uint32][math]::Round($TurnsValue * 3600.0)
        $velocityTenths = [uint16][math]::Round($VelocityValue * 10.0)
        $acceleration = [uint16]$Accel

        $fwPayload = [byte[]]@([byte]$MotorId, [byte]0) +
            (Write-BeU32 $degreesTenths) +
            (Write-BeU16 $velocityTenths) +
            (Write-BeU16 $acceleration)
        Assert-Ok (Invoke-Cmd $Serial $CmdBenchMoveRel $fwPayload) $CmdBenchMoveRel 'MOVE_FW'
        Start-Sleep -Milliseconds $WaitMs

        $mid = Get-BenchStatePolled $Serial $MotorId
        $row.AfterFwDeg = $mid.PositionDegrees
        $row.DeltaFwDeg = [math]::Round($mid.PositionDegrees - $before.PositionDegrees, 3)

        $rvPayload = [byte[]]@([byte]$MotorId, [byte]1) +
            (Write-BeU32 $degreesTenths) +
            (Write-BeU16 $velocityTenths) +
            (Write-BeU16 $acceleration)
        Assert-Ok (Invoke-Cmd $Serial $CmdBenchMoveRel $rvPayload) $CmdBenchMoveRel 'MOVE_RV'
        Start-Sleep -Milliseconds $WaitMs

        $after = Get-BenchStatePolled $Serial $MotorId
        $row.AfterRvDeg = $after.PositionDegrees
        $row.ResidualDeg = [math]::Round($after.PositionDegrees - $before.PositionDegrees, 3)

        Invoke-SafeStopDisable $Serial $MotorId

        $expected = $TurnsValue * 360.0
        $minForward = [math]::Max($MinTolDeg, $expected * (1.0 - $TolFrac))
        $maxResidual = [math]::Max($MinTolDeg, $expected * $TolFrac)
        $absDelta = [math]::Abs([double]$row.DeltaFwDeg)
        $absResidual = [math]::Abs([double]$row.ResidualDeg)
        $row.DirSign = if ($row.DeltaFwDeg -ge 0) { '+' } else { '-' }

        if ($absDelta -lt $minForward) {
            $row.Motion = 'FAIL'
            $row.Verdict = 'FAIL'
            $row.Detail = ("forward too small delta={0} need>={1}" -f $row.DeltaFwDeg, [math]::Round($minForward, 1))
            return [pscustomobject]$row
        }
        if ($absResidual -gt $maxResidual) {
            $row.Motion = 'FAIL'
            $row.Verdict = 'FAIL'
            $row.Detail = ("not returned residual={0} allow<={1}" -f $row.ResidualDeg, [math]::Round($maxResidual, 1))
            return [pscustomobject]$row
        }

        $row.Motion = 'PASS'
        $row.Verdict = 'PASS'
        $row.Detail = ("fw={0}deg residual={1}deg" -f $row.DeltaFwDeg, $row.ResidualDeg)
        return [pscustomobject]$row
    } catch {
        try { Invoke-SafeStopDisable $Serial $MotorId } catch {}
        $row.Motion = 'FAIL'
        $row.Verdict = 'FAIL'
        $row.Detail = $_.Exception.Message
        return [pscustomobject]$row
    }
}

foreach ($id in $MotorIds) {
    if ($id -lt 1 -or $id -gt 6) { throw "MotorIds must be 1..6, got $id" }
}
if ($Turns -le 0 -or $Turns -gt $MaxTurns) { throw "Turns must be in (0, $MaxTurns]" }
if ($VelocityRpm -le 0 -or $VelocityRpm -gt $MaxVelocityRpm) { throw "VelocityRpm must be in (0, $MaxVelocityRpm]" }
if ($AccelerationRpmS -le 0 -or $AccelerationRpmS -gt $MaxAcceleration) { throw "AccelerationRpmS must be in (0, $MaxAcceleration]" }

$waitMs = Get-WaitMs $Turns $VelocityRpm $ExtraSettleMs
$doMotion = -not $ScanOnly.IsPresent
$startedAt = Get-Date

Write-Host '============================================================'
Write-Host ' ZEROARM FULL MOTOR TEST'
Write-Host '============================================================'
Write-Host ("time     : {0}" -f $startedAt.ToString('yyyy-MM-dd HH:mm:ss'))
Write-Host ("port     : {0}" -f $Port)
Write-Host ("motors   : {0}" -f ($MotorIds -join ','))
Write-Host ("turns    : {0}" -f $Turns)
Write-Host ("rpm      : {0} (motor shaft)" -f $VelocityRpm)
Write-Host ("accel    : {0}" -f $AccelerationRpmS)
Write-Host ("wait_ms  : {0}" -f $waitMs)
Write-Host ("mode     : {0}" -f ($(if ($doMotion) { 'SCAN+MOTION' } else { 'SCAN_ONLY' })))
Write-Host '------------------------------------------------------------'

$serial = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, None, 8, One
$serial.ReadTimeout = $ReadTimeoutMs
$serial.WriteTimeout = $ReadTimeoutMs
$serial.DtrEnable = $false
$serial.RtsEnable = $false

$lines = New-Object System.Collections.Generic.List[string]
function Log([string]$Text) {
    Write-Host $Text
    $script:lines.Add($Text) | Out-Null
}

$report = New-Object System.Collections.Generic.List[object]
$uartPass = $false
$mcuReady = $false
$exitCode = 0

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    Log '[1/3] UART / MCU'
    $hello = Invoke-Cmd $serial $CmdHello @()
    $helloText = [Text.Encoding]::ASCII.GetString($hello.Payload)
    if ($hello.Command -ne $CmdHello -or $helloText -ne 'ZEROARM/1.0') {
        throw ("HELLO failed: {0}" -f $hello.Hex)
    }
    Log '  HELLO     = PASS  ZEROARM/1.0'

    $state = Invoke-Cmd $serial $CmdGetState @()
    if ($state.Command -ne $CmdGetState -or $state.Payload.Length -ne $RobotStatePayloadSize) {
        throw ("GET_STATE failed length={0}" -f $state.Payload.Length)
    }
    $runState = [BitConverter]::ToInt32($state.Payload, 0)
    $fault = [BitConverter]::ToUInt32($state.Payload, 56)
    $mcuReady = ($runState -eq 1)
    $uartPass = $true
    Log ("  GET_STATE = PASS  run_state={0} fault=0x{1:X8}" -f $runState, $fault)
    if (-not $mcuReady) {
        Log '  WARN: MCU not READY'
    }

    Log '[2/3] Scan motors'
    foreach ($id in $MotorIds) {
        Write-Host ("  probing motor {0}..." -f $id)
        $row = Test-OneMotor `
            $serial $id $Turns $VelocityRpm $AccelerationRpmS `
            $waitMs $ToleranceFraction $MinAbsoluteToleranceDeg $false
        if ($row.Online) {
            Log ("  motor {0}: ONLINE  pos={1} deg" -f $id, $row.BeforeDeg)
        } else {
            Log ("  motor {0}: {1}  {2}" -f $id, $row.Link, $row.Detail)
        }
        if (-not $doMotion) {
            $report.Add($row) | Out-Null
        }
    }

    if ($doMotion) {
        Log '[3/3] Motion test (online motors only)'
        foreach ($id in $MotorIds) {
            $probe = Get-BenchStatePolled $serial $id
            if ($probe.Rejected -or $probe.Online -eq 0) {
                $link = if ($probe.Rejected) { $probe.LinkHint } else { 'OFFLINE' }
                $detail = if ($probe.Rejected -and $probe.ErrorCode -eq 7) {
                    'no CAN response'
                } elseif ($probe.Rejected) {
                    ("reject {0}({1})" -f $probe.ErrorName, $probe.ErrorCode)
                } else {
                    'offline'
                }
                $row = [pscustomobject]@{
                    MotorId = $id
                    Online = $false
                    Link = $link
                    Motion = 'SKIP'
                    BeforeDeg = $null
                    AfterFwDeg = $null
                    AfterRvDeg = $null
                    DeltaFwDeg = $null
                    ResidualDeg = $null
                    DirSign = ''
                    Verdict = 'FAIL'
                    Detail = $detail
                }
                $report.Add($row) | Out-Null
                Log ("  motor {0}: SKIP motion ({1})" -f $id, $row.Detail)
                continue
            }

            Log ("  motor {0}: MOTION start turns={1} rpm={2}" -f $id, $Turns, $VelocityRpm)
            $row = Test-OneMotor `
                $serial $id $Turns $VelocityRpm $AccelerationRpmS `
                $waitMs $ToleranceFraction $MinAbsoluteToleranceDeg $true
            $report.Add($row) | Out-Null
            Log (
                "  motor {0}: {1}  delta={2} residual={3}  {4}" -f
                $id,
                $row.Verdict,
                $row.DeltaFwDeg,
                $row.ResidualDeg,
                $row.Detail)
        }
    } else {
        Log '[3/3] Motion skipped (-ScanOnly)'
    }
} catch {
    $exitCode = 3
    Log ("FATAL: {0}" -f $_.Exception.Message)
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}

$endedAt = Get-Date
$elapsedSec = [math]::Round(($endedAt - $startedAt).TotalSeconds, 1)

$online = @($report | Where-Object { $_.Online }).Count
$offline = @($report | Where-Object { -not $_.Online }).Count
$motionPass = @($report | Where-Object { $_.Motion -eq 'PASS' }).Count
$motionFail = @($report | Where-Object { $_.Motion -eq 'FAIL' }).Count
$verdictPass = @($report | Where-Object { $_.Verdict -eq 'PASS' }).Count

Log ''
Log '============================================================'
Log ' FINAL REPORT'
Log '============================================================'
Log ("finished : {0}" -f $endedAt.ToString('yyyy-MM-dd HH:mm:ss'))
Log ("elapsed  : {0}s" -f $elapsedSec)
Log ("uart     : {0}" -f ($(if ($uartPass) { 'PASS' } else { 'FAIL' })))
Log ("mcu      : {0}" -f ($(if ($mcuReady) { 'READY' } else { 'NOT_READY/UNKNOWN' })))
Log ("scanned  : {0}" -f ($MotorIds -join ','))
Log ("online   : {0}" -f $online)
Log ("offline  : {0}" -f $offline)
if ($doMotion) {
    Log ("motion   : pass={0} fail={1}" -f $motionPass, $motionFail)
}
Log '------------------------------------------------------------'
Log ' Motor | Online | Link    | Motion | DeltaFw | Residual | Verdict | Detail'
Log '-------+--------+---------+--------+---------+----------+---------+------------------------------'

foreach ($r in ($report | Sort-Object MotorId)) {
    $line = (
        ' {0,5} | {1,6} | {2,-7} | {3,-6} | {4,7} | {5,8} | {6,-7} | {7}' -f
        $r.MotorId,
        ($(if ($r.Online) { 'YES' } else { 'NO' })),
        $r.Link,
        $r.Motion,
        $(if ($null -ne $r.DeltaFwDeg) { '{0:N1}' -f $r.DeltaFwDeg } else { '-' }),
        $(if ($null -ne $r.ResidualDeg) { '{0:N1}' -f $r.ResidualDeg } else { '-' }),
        $r.Verdict,
        $r.Detail
    )
    Log $line
}

Log '------------------------------------------------------------'

if ($exitCode -eq 3) {
    Log 'OVERALL = FAIL (fatal communication/protocol error)'
} elseif (-not $uartPass) {
    $exitCode = 3
    Log 'OVERALL = FAIL (UART/MCU path broken)'
} elseif ($doMotion) {
    if ($motionFail -gt 0) {
        $exitCode = 1
        Log 'OVERALL = FAIL (one or more motion tests failed)'
    } elseif ($motionPass -eq 0) {
        $exitCode = 2
        Log 'OVERALL = FAIL (no online motor completed motion)'
    } else {
        $exitCode = 0
        Log ("OVERALL = PASS  ({0} motor(s) motion OK)" -f $motionPass)
    }
} else {
    if ($online -eq 0) {
        $exitCode = 2
        Log 'OVERALL = FAIL (no motor online)'
    } else {
        $exitCode = 0
        Log ("OVERALL = PASS_SCAN_ONLY  ({0} online)" -f $online)
    }
}

Log '============================================================'

if ([string]::IsNullOrWhiteSpace($ReportPath)) {
    $stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
    $ReportPath = Join-Path $PSScriptRoot ("motor_full_report_{0}.txt" -f $stamp)
}
$lines -join [Environment]::NewLine | Set-Content -LiteralPath $ReportPath -Encoding UTF8
Write-Host ("report_file = {0}" -f $ReportPath)

exit $exitCode

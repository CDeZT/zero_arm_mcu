param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [string]$Joints = '1,2,3,4,5,6',
    [double]$JointDegrees = 10.0,
    [int]$Cycles = 10,
    [double]$MotorVelocityRpm = 400.0,
    [int]$AccelerationRpmS = 400,
    [int]$ExtraSettleMs = 800,
    [int]$ReadTimeoutMs = 4000,
    [double]$ToleranceFraction = 0.20,
    [switch]$VerboseHex
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Joint-space relative wiggle using bench motor commands.
# motor_degrees = joint_degrees * gear_ratio
# motor dir for +joint accounts for motor_sign.

$CmdHello = 0x00
$CmdGetState = 0x01
$CmdBenchQuery = 0x20
$CmdBenchEnable = 0x21
$CmdBenchDisable = 0x22
$CmdBenchStop = 0x23
$CmdBenchMoveRel = 0x24

# joint_index 1..6 -> motor_id, gear_ratio, motor_sign
$JointTable = @{
    1 = @{ MotorId = 1; Ratio = 50.000; Sign = 1 }
    2 = @{ MotorId = 2; Ratio = 50.890; Sign = -1 }
    3 = @{ MotorId = 3; Ratio = 50.890; Sign = -1 }
    4 = @{ MotorId = 4; Ratio = 51.000; Sign = -1 }
    5 = @{ MotorId = 5; Ratio = 26.850; Sign = 1 }
    6 = @{ MotorId = 6; Ratio = 51.000; Sign = -1 }
}

function Get-Crc8([byte[]]$Data) {
    [byte]$crc = 0
    foreach ($item in $Data) {
        [byte]$value = $item
        for ($bit = 0; $bit -lt 8; $bit++) {
            $mix = ($crc -bxor $value) -band 1
            $crc = [byte]($crc -shr 1)
            if ($mix -ne 0) { $crc = [byte]($crc -bxor 0x8C) }
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
    for ($i = 0; $i -lt $Payload.Length; $i++) { $body[$i + 1] = [byte]$Payload[$i] }
    [byte]$length = [byte]$body.Length
    [byte]$crc = Get-Crc8 $body
    [byte[]]$frame = New-Object byte[] ($body.Length + 4)
    $frame[0] = 0xAA
    $frame[1] = $length
    for ($i = 0; $i -lt $body.Length; $i++) { $frame[$i + 2] = $body[$i] }
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
    if ($length -le 0 -or $length -ge 128) { throw "bad len $length" }
    [byte[]]$body = New-Object byte[] $length
    for ($i = 0; $i -lt $length; $i++) { $body[$i] = [byte]$Serial.ReadByte() }
    [byte]$rxCrc = [byte]$Serial.ReadByte()
    $etx = $Serial.ReadByte()
    if ($etx -ne 0x55) { throw ("bad etx 0x{0:X2}" -f $etx) }
    if ((Get-Crc8 $body) -ne $rxCrc) { throw 'crc mismatch' }
    [byte[]]$payload = @()
    if ($length -gt 1) {
        $payload = New-Object byte[] ($length - 1)
        for ($i = 0; $i -lt ($length - 1); $i++) { $payload[$i] = $body[$i + 1] }
    }
    if ($VerboseHex) {
        [byte[]]$all = New-Object byte[] ($length + 4)
        $all[0] = 0xAA; $all[1] = [byte]$length
        for ($i = 0; $i -lt $length; $i++) { $all[$i + 2] = $body[$i] }
        $all[$length + 2] = $rxCrc; $all[$length + 3] = 0x55
        Write-Host ("RX: {0}" -f (To-Hex $all))
    }
    return [pscustomobject]@{ Command = $body[0]; Payload = $payload }
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
        throw ('{0}: bad cmd 0x{1:X2}' -f $Step, $Response.Command)
    }
    if ($Response.Payload.Length -ne 1 -or $Response.Payload[0] -ne 0) {
        $code = if ($Response.Payload.Length -ge 1) { $Response.Payload[0] } else { -1 }
        throw ('{0}: error code {1}' -f $Step, $code)
    }
}

function Parse-Bench([byte[]]$Payload) {
    if ($Payload.Length -lt 28) { throw ("bench payload len {0}" -f $Payload.Length) }
    $pos = [BitConverter]::ToInt32($Payload, 4)
    return [pscustomobject]@{
        MotorId = $Payload[0]
        Online = $Payload[1]
        PositionDegrees = [math]::Round($pos * 180.0 / 3141593.0, 3)
        Sample = if ($Payload.Length -ge 32) { [BitConverter]::ToUInt32($Payload, 28) } else { [uint32]0 }
    }
}

function Get-Bench(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId,
    [int]$Polls = 5
) {
    $last = $null
    for ($i = 0; $i -lt $Polls; $i++) {
        $rx = Invoke-Cmd $Serial $CmdBenchQuery ([byte[]]@([byte]$MotorId))
        if ($rx.Payload.Length -eq 1) {
            return [pscustomobject]@{
                MotorId = $MotorId
                Online = 0
                PositionDegrees = 0
                Sample = 0
                Rejected = $true
                Code = $rx.Payload[0]
            }
        }
        $last = Parse-Bench $rx.Payload
        $last | Add-Member -NotePropertyName Rejected -NotePropertyValue $false
        $last | Add-Member -NotePropertyName Code -NotePropertyValue 0
        if ($last.Online -ne 0) { return $last }
        Start-Sleep -Milliseconds 100
    }
    return $last
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

function Get-WaitMs([double]$MotorDegrees, [double]$VelocityRpm, [int]$ExtraMs) {
    $turns = [math]::Abs($MotorDegrees) / 360.0
    $seconds = ($turns / $VelocityRpm) * 60.0
    $wait = [int][math]::Ceiling(($seconds * 1.50 * 1000.0) + $ExtraMs)
    if ($wait -lt 600) { $wait = 600 }
    return $wait
}

function Invoke-SafeStopOnly(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId
) {
    # Assembled arm: NEVER disable. Holding torque must remain.
    try {
        Assert-Ok (Invoke-Cmd $Serial $CmdBenchStop ([byte[]]@([byte]$MotorId))) $CmdBenchStop 'STOP'
    } catch {
        Write-Host ("  WARN stop failed motor={0}: {1}" -f $MotorId, $_.Exception.Message)
    }
}

function Invoke-EnsureEnabled(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId
) {
    Assert-Ok (Invoke-Cmd $Serial $CmdBenchEnable ([byte[]]@([byte]$MotorId))) $CmdBenchEnable 'ENABLE'
}

function Move-MotorRel(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId,
    [byte]$Direction,
    [double]$MotorDegrees,
    [double]$VelocityRpm,
    [int]$Accel
) {
    $degreesTenths = [uint32][math]::Round([math]::Abs($MotorDegrees) * 10.0)
    $velocityTenths = [uint16][math]::Round($VelocityRpm * 10.0)
    $acceleration = [uint16]$Accel
    $payload = [byte[]]@([byte]$MotorId, $Direction) +
        (Write-BeU32 $degreesTenths) +
        (Write-BeU16 $velocityTenths) +
        (Write-BeU16 $acceleration)
    Assert-Ok (Invoke-Cmd $Serial $CmdBenchMoveRel $payload) $CmdBenchMoveRel 'MOVE_REL'
}

function Get-DirForJointPlus([int]$Sign) {
    # X_V2 dir: 0 = positive motor, 1 = negative motor
    if ($Sign -ge 0) { return [byte]0 } else { return [byte]1 }
}

function Get-DirForJointMinus([int]$Sign) {
    if ($Sign -ge 0) { return [byte]1 } else { return [byte]0 }
}

$JointList = @()
foreach ($token in ($Joints -split '[,;\s]+')) {
    if ([string]::IsNullOrWhiteSpace($token)) { continue }
    $id = 0
    if (-not [int]::TryParse($token, [ref]$id)) {
        throw "Invalid joint token: $token"
    }
    if ($id -lt 1 -or $id -gt 6) {
        throw "Joint must be 1..6, got $id"
    }
    $JointList += $id
}
if ($JointList.Count -eq 0) {
    throw 'No joints specified'
}
if ($JointDegrees -le 0 -or $JointDegrees -gt 45) {
    throw "JointDegrees must be in (0, 45], got $JointDegrees"
}
if ($Cycles -lt 1 -or $Cycles -gt 50) {
    throw "Cycles must be 1..50"
}
if ($MotorVelocityRpm -le 0 -or $MotorVelocityRpm -gt 800) {
    throw "MotorVelocityRpm must be in (0, 800]"
}

$report = New-Object System.Collections.Generic.List[object]
$started = Get-Date

Write-Host '============================================================'
Write-Host ' ZEROARM JOINT +/- WIGGLE TEST'
Write-Host '============================================================'
Write-Host ("time     : {0}" -f $started.ToString('yyyy-MM-dd HH:mm:ss'))
Write-Host ("port     : {0}" -f $Port)
Write-Host ("joints   : {0}" -f ($JointList -join ','))
Write-Host ("amplitude: +/-{0} deg joint" -f $JointDegrees)
Write-Host ("cycles   : {0} round-trips per joint" -f $Cycles)
Write-Host ("motor_rpm: {0}" -f $MotorVelocityRpm)
Write-Host '------------------------------------------------------------'
Write-Host 'WARNING: assembled arm motion. Keep e-stop ready.'
Write-Host 'NOTE: motors stay ENABLED after each joint (no DISABLE).'
Write-Host '------------------------------------------------------------'

$serial = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, None, 8, One
$serial.ReadTimeout = $ReadTimeoutMs
$serial.WriteTimeout = $ReadTimeoutMs
$serial.DtrEnable = $false
$serial.RtsEnable = $false

$activeMotor = 0
$enabledMotors = New-Object System.Collections.Generic.List[int]
$fatal = $null

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    $hello = Invoke-Cmd $serial $CmdHello @()
    $helloText = [Text.Encoding]::ASCII.GetString($hello.Payload)
    if ($helloText -ne 'ZEROARM/1.0') { throw "HELLO failed: $helloText" }
    Write-Host 'UART HELLO = PASS'

    $state = Invoke-Cmd $serial $CmdGetState @()
    if ($state.Payload.Length -lt 60) { throw 'GET_STATE failed' }
    $run = [BitConverter]::ToInt32($state.Payload, 0)
    $fault = [BitConverter]::ToUInt32($state.Payload, 56)
    Write-Host ("GET_STATE = PASS run={0} fault=0x{1:X8}" -f $run, $fault)

    # Pre-enable every online joint first so the arm does not collapse
    # while a later joint is being exercised.
    Write-Host 'Pre-enabling online joints (holding torque)...'
    foreach ($joint in $JointList) {
        if (-not $JointTable.ContainsKey($joint)) { continue }
        $mid = [int]$JointTable[$joint].MotorId
        try {
            $probe = Get-Bench $serial $mid
            if ($probe.Rejected -or $probe.Online -eq 0) {
                Write-Host ("  J{0}/M{1}: offline, skip pre-enable" -f $joint, $mid)
                continue
            }
            Invoke-EnsureEnabled $serial $mid
            if (-not $enabledMotors.Contains($mid)) {
                $enabledMotors.Add($mid) | Out-Null
            }
            Write-Host ("  J{0}/M{1}: ENABLED" -f $joint, $mid)
        } catch {
            Write-Host ("  J{0}/M{1}: pre-enable FAIL {2}" -f $joint, $mid, $_.Exception.Message)
        }
    }
    Start-Sleep -Milliseconds 500

    foreach ($joint in $JointList) {
        if (-not $JointTable.ContainsKey($joint)) {
            throw "Unknown joint $joint"
        }
        $cfg = $JointTable[$joint]
        $motorId = [int]$cfg.MotorId
        $ratio = [double]$cfg.Ratio
        $sign = [int]$cfg.Sign
        $motorDeg = $JointDegrees * $ratio
        $waitMs = Get-WaitMs $motorDeg $MotorVelocityRpm $ExtraSettleMs
        $dirPlus = Get-DirForJointPlus $sign
        $dirMinus = Get-DirForJointMinus $sign

        Write-Host ''
        Write-Host ("--- Joint {0} / motor {1}  ratio={2}  motor_move={3:N1} deg  wait={4}ms ---" -f
            $joint, $motorId, $ratio, $motorDeg, $waitMs)

        $row = [ordered]@{
            Joint = $joint
            MotorId = $motorId
            Online = $false
            CyclesOk = 0
            StartDeg = $null
            EndDeg = $null
            ResidualMotorDeg = $null
            ResidualJointDeg = $null
            Verdict = 'FAIL'
            Detail = ''
        }

        try {
            $q = Get-Bench $serial $motorId
            if ($q.Rejected -or $q.Online -eq 0) {
                $row.Detail = if ($q.Rejected) { "offline/reject code=$($q.Code)" } else { 'offline' }
                Write-Host ("  SKIP: {0}" -f $row.Detail)
                $report.Add([pscustomobject]$row) | Out-Null
                continue
            }

            $row.Online = $true
            $row.StartDeg = $q.PositionDegrees
            Write-Host ("  online start_motor_pos={0} deg" -f $q.PositionDegrees)

            # Keep holding torque. Re-assert enable before motion.
            Invoke-EnsureEnabled $serial $motorId
            if (-not $enabledMotors.Contains($motorId)) {
                $enabledMotors.Add($motorId) | Out-Null
            }
            $activeMotor = $motorId
            Start-Sleep -Milliseconds 200

            $before = Get-Bench $serial $motorId
            $row.StartDeg = $before.PositionDegrees

            for ($c = 1; $c -le $Cycles; $c++) {
                Write-Host ("  cycle {0}/{1}: +{2} joint deg" -f $c, $Cycles, $JointDegrees)
                Move-MotorRel $serial $motorId $dirPlus $motorDeg $MotorVelocityRpm $AccelerationRpmS
                Start-Sleep -Milliseconds $waitMs

                Write-Host ("  cycle {0}/{1}: -{2} joint deg (return)" -f $c, $Cycles, $JointDegrees)
                Move-MotorRel $serial $motorId $dirMinus $motorDeg $MotorVelocityRpm $AccelerationRpmS
                Start-Sleep -Milliseconds $waitMs

                $row.CyclesOk = $c
            }

            $after = Get-Bench $serial $motorId
            $row.EndDeg = $after.PositionDegrees
            $row.ResidualMotorDeg = [math]::Round($after.PositionDegrees - $before.PositionDegrees, 3)
            $row.ResidualJointDeg = [math]::Round($row.ResidualMotorDeg / $ratio, 3)

            # Stop motion only if needed; immediately re-enable for holding.
            # Do NOT disable on an assembled arm.
            Invoke-EnsureEnabled $serial $motorId
            $activeMotor = 0

            $maxResidualMotor = [math]::Max(15.0, $motorDeg * $ToleranceFraction)
            if ([math]::Abs([double]$row.ResidualMotorDeg) -gt $maxResidualMotor) {
                $row.Verdict = 'FAIL'
                $row.Detail = ("residual motor {0} deg (joint {1}) too large; motor left ENABLED" -f
                    $row.ResidualMotorDeg, $row.ResidualJointDeg)
            } else {
                $row.Verdict = 'PASS'
                $row.Detail = ("{0} cycles ok, residual joint {1} deg; motor left ENABLED" -f
                    $row.CyclesOk, $row.ResidualJointDeg)
            }
            Write-Host ("  result: {0}  residual_joint={1} deg  (ENABLED hold)" -f $row.Verdict, $row.ResidualJointDeg)
        } catch {
            if ($activeMotor -ne 0) {
                try { Invoke-EnsureEnabled $serial $activeMotor } catch {}
                $activeMotor = 0
            }
            $row.Verdict = 'FAIL'
            $row.Detail = $_.Exception.Message
            Write-Host ("  result: FAIL  {0}" -f $row.Detail)
        }

        $report.Add([pscustomobject]$row) | Out-Null
    }

    # Final hold: re-enable every motor that was online during the run.
    Write-Host ''
    Write-Host 'Final hold: re-assert ENABLE on all exercised motors...'
    foreach ($mid in $enabledMotors) {
        try {
            Invoke-EnsureEnabled $serial $mid
            Write-Host ("  motor {0}: ENABLED hold" -f $mid)
        } catch {
            Write-Host ("  motor {0}: ENABLE hold FAIL {1}" -f $mid, $_.Exception.Message)
        }
    }
} catch {
    $fatal = $_.Exception.Message
    if ($serial.IsOpen) {
        foreach ($mid in $enabledMotors) {
            try { Invoke-EnsureEnabled $serial $mid } catch {}
        }
        if ($activeMotor -ne 0) {
            try { Invoke-EnsureEnabled $serial $activeMotor } catch {}
        }
    }
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}

$ended = Get-Date
$elapsed = [math]::Round(($ended - $started).TotalSeconds, 1)
$pass = @($report | Where-Object { $_.Verdict -eq 'PASS' }).Count
$fail = @($report | Where-Object { $_.Verdict -eq 'FAIL' }).Count
$online = @($report | Where-Object { $_.Online }).Count

Write-Host ''
Write-Host '============================================================'
Write-Host ' FINAL REPORT'
Write-Host '============================================================'
Write-Host ("finished : {0}" -f $ended.ToString('yyyy-MM-dd HH:mm:ss'))
Write-Host ("elapsed  : {0}s" -f $elapsed)
Write-Host ("online   : {0}" -f $online)
Write-Host ("pass     : {0}" -f $pass)
Write-Host ("fail     : {0}" -f $fail)
if ($fatal) { Write-Host ("fatal    : {0}" -f $fatal) }
Write-Host '------------------------------------------------------------'
Write-Host ' Joint | Motor | Online | Cycles | ResidualJ | Verdict | Detail'
Write-Host '-------+-------+--------+--------+-----------+---------+---------------------------'
foreach ($r in ($report | Sort-Object Joint)) {
    Write-Host (
        ' {0,5} | {1,5} | {2,6} | {3,6} | {4,9} | {5,-7} | {6}' -f
        $r.Joint,
        $r.MotorId,
        ($(if ($r.Online) { 'YES' } else { 'NO' })),
        $r.CyclesOk,
        $(if ($null -ne $r.ResidualJointDeg) { '{0:N2}' -f $r.ResidualJointDeg } else { '-' }),
        $r.Verdict,
        $r.Detail
    )
}
Write-Host '------------------------------------------------------------'

$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$reportPath = Join-Path $PSScriptRoot ("joint_wiggle_report_{0}.txt" -f $stamp)
$report | Format-Table -AutoSize | Out-String | Set-Content -LiteralPath $reportPath -Encoding UTF8
Write-Host ("report_file = {0}" -f $reportPath)

if ($fatal) {
    Write-Host 'OVERALL = FAIL (fatal)'
    exit 3
}
if ($fail -gt 0 -or $pass -eq 0) {
    Write-Host 'OVERALL = FAIL'
    exit 1
}
Write-Host 'OVERALL = PASS'
exit 0

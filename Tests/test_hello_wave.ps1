param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [double]$MotorVelocityRpm = 520.0,
    [int]$AccelerationRpmS = 700,
    [int]$Waves = 8,
    [double]$J5Amp = 38.0,
    [double]$J6Amp = 34.0,
    [int]$ReadTimeoutMs = 4000,
    [switch]$SkipSetZero
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Big, obvious HELLO wave.
# - J5 is the main waving joint (large amplitude)
# - J6 assists wrist swing
# - Closed-loop from encoder feedback
# - ENABLE hold only, never DISABLE
# - Hard return to encoder 0 at end

$CmdHello = 0x00
$CmdGetState = 0x01
$CmdBenchQuery = 0x20
$CmdBenchEnable = 0x21
$CmdBenchMoveRel = 0x24
$CmdBenchSetZero = 0x25
$PoseLimitDeg = 40.0

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

function Invoke-Cmd(
    [System.IO.Ports.SerialPort]$Serial,
    [byte]$Command,
    [byte[]]$Payload
) {
    if ($null -eq $Payload) { $Payload = @() }
    $frame = New-Frame $Command $Payload
    $Serial.Write($frame, 0, $frame.Length)
    do { $stx = $Serial.ReadByte() } while ($stx -ne 0xAA)
    $length = $Serial.ReadByte()
    [byte[]]$body = New-Object byte[] $length
    for ($i = 0; $i -lt $length; $i++) { $body[$i] = [byte]$Serial.ReadByte() }
    [byte]$rxCrc = [byte]$Serial.ReadByte()
    $etx = $Serial.ReadByte()
    if ($etx -ne 0x55) { throw 'bad etx' }
    if ((Get-Crc8 $body) -ne $rxCrc) { throw 'crc mismatch' }
    [byte[]]$payloadOut = @()
    if ($length -gt 1) {
        $payloadOut = New-Object byte[] ($length - 1)
        for ($i = 0; $i -lt ($length - 1); $i++) { $payloadOut[$i] = $body[$i + 1] }
    }
    return [pscustomobject]@{ Command = $body[0]; Payload = $payloadOut }
}

function Assert-Ok([pscustomobject]$Response, [byte]$Command, [string]$Step) {
    if ($Response.Command -ne $Command -or $Response.Payload.Length -ne 1 -or $Response.Payload[0] -ne 0) {
        $code = if ($Response.Payload.Length -ge 1) { $Response.Payload[0] } else { -1 }
        throw ('{0}: fail code={1}' -f $Step, $code)
    }
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

function Clamp([double]$Value, [double]$Min, [double]$Max) {
    if ($Value -lt $Min) { return $Min }
    if ($Value -gt $Max) { return $Max }
    return $Value
}

function Get-MotorDeg(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId
) {
    $rx = Invoke-Cmd $Serial $CmdBenchQuery ([byte[]]@([byte]$MotorId))
    if ($rx.Payload.Length -eq 1) {
        throw ("query fail M{0} code={1}" -f $MotorId, $rx.Payload[0])
    }
    return [math]::Round([BitConverter]::ToInt32($rx.Payload, 4) * 180.0 / 3141593.0, 3)
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
    if ([math]::Abs($MotorDegrees) -lt 0.25) { return }
    $degreesTenths = [uint32][math]::Round([math]::Abs($MotorDegrees) * 10.0)
    $velocityTenths = [uint16][math]::Round($VelocityRpm * 10.0)
    $acceleration = [uint16]$Accel
    $payload = [byte[]]@([byte]$MotorId, $Direction) +
        (Write-BeU32 $degreesTenths) +
        (Write-BeU16 $velocityTenths) +
        (Write-BeU16 $acceleration)
    Assert-Ok (Invoke-Cmd $Serial $CmdBenchMoveRel $payload) $CmdBenchMoveRel 'MOVE'
}

function Get-TargetMotorDeg([int]$Joint, [double]$JointDeg) {
    $cfg = $JointTable[$Joint]
    return ([double]$cfg.Sign) * $JointDeg * ([double]$cfg.Ratio)
}

function Get-MoveTimeMs([double]$MaxMotorDeg, [double]$VelocityRpm, [int]$Accel, [double]$Overlap = 0.68) {
    $revs = [math]::Abs($MaxMotorDeg) / 360.0
    if ($VelocityRpm -le 1.0) { $VelocityRpm = 1.0 }
    $constSec = $revs * 60.0 / $VelocityRpm
    $rampSec = 0.0
    if ($Accel -gt 0) {
        $vRps = $VelocityRpm / 60.0
        $aRps2 = $Accel / 60.0
        if ($aRps2 -gt 0.01) { $rampSec = $vRps / $aRps2 }
    }
    $ms = [int][math]::Ceiling((($constSec + 0.2 * $rampSec) * $Overlap + 0.05) * 1000.0)
    if ($ms -lt 180) { $ms = 180 }
    if ($ms -gt 8000) { $ms = 8000 }
    return $ms
}

function Set-PoseValues([hashtable]$Values) {
    $out = @{}
    foreach ($j in 1..6) {
        if ($Values.ContainsKey($j)) {
            $out[$j] = Clamp ([double]$Values[$j]) (-1.0 * $PoseLimitDeg) $PoseLimitDeg
        } else {
            $out[$j] = 0.0
        }
    }
    return $out
}

function Move-ToPose(
    [System.IO.Ports.SerialPort]$Serial,
    [hashtable]$TargetPose,
    [string]$Name,
    [double]$VelocityScale = 1.0,
    [string]$Mode = 'flow'
) {
    Write-Host (">> {0}" -f $Name)
    $vel = [math]::Max(150.0, $MotorVelocityRpm * $VelocityScale)
    $acc = [int][math]::Max(180, [math]::Round($AccelerationRpmS * $VelocityScale))
    $maxMag = 0.0
    $moves = New-Object System.Collections.Generic.List[object]
    $targets = @{}

    $current = @{}
    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        if (-not $current.ContainsKey($mid)) {
            $current[$mid] = Get-MotorDeg $Serial $mid
        }
    }

    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        $tgtJoint = Clamp ([double]$TargetPose[$joint]) (-1.0 * $PoseLimitDeg) $PoseLimitDeg
        $tgtMotor = Get-TargetMotorDeg $joint $tgtJoint
        $curMotor = [double]$current[$mid]
        $delta = $tgtMotor - $curMotor
        $targets[$mid] = $tgtMotor
        if ([math]::Abs($delta) -lt 0.5) { continue }
        $dir = if ($delta -ge 0) { [byte]0 } else { [byte]1 }
        $mag = [math]::Abs($delta)
        if ($mag -gt $maxMag) { $maxMag = $mag }
        $moves.Add([pscustomobject]@{ MotorId = $mid; Dir = $dir; Mag = $mag }) | Out-Null
    }

    if ($moves.Count -eq 0) { return }
    foreach ($m in $moves) {
        Move-MotorRel $Serial $m.MotorId $m.Dir $m.Mag $vel $acc
    }

    if ($Mode -eq 'settle') {
        $wait = Get-MoveTimeMs $maxMag $vel $acc 0.95
        Start-Sleep -Milliseconds $wait
        foreach ($joint in 1..6) {
            $mid = [int]$JointTable[$joint].MotorId
            $cur = Get-MotorDeg $Serial $mid
            $tgt = [double]$targets[$mid]
            $delta = $tgt - $cur
            if ([math]::Abs($delta) -lt 2.0) { continue }
            $dir = if ($delta -ge 0) { [byte]0 } else { [byte]1 }
            Move-MotorRel $Serial $mid $dir ([math]::Abs($delta)) ([math]::Max(180.0, $vel * 0.7)) $acc
        }
        Start-Sleep -Milliseconds 700
    } else {
        Start-Sleep -Milliseconds (Get-MoveTimeMs $maxMag $vel $acc 0.62)
    }
}

function Move-AllMotorsToZero(
    [System.IO.Ports.SerialPort]$Serial,
    [string]$Name
) {
    Write-Host (">> {0}" -f $Name)
    $maxMag = 0.0
    $moves = New-Object System.Collections.Generic.List[object]
    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        $cur = Get-MotorDeg $Serial $mid
        Write-Host ("   M{0} now={1}" -f $mid, $cur)
        if ([math]::Abs($cur) -lt 0.8) { continue }
        $dir = if ($cur -gt 0) { [byte]1 } else { [byte]0 }
        $mag = [math]::Abs($cur)
        if ($mag -gt $maxMag) { $maxMag = $mag }
        $moves.Add([pscustomobject]@{ MotorId = $mid; Dir = $dir; Mag = $mag }) | Out-Null
    }
    if ($moves.Count -eq 0) {
        Write-Host '   all near zero'
        return
    }
    foreach ($m in $moves) {
        Move-MotorRel $Serial $m.MotorId $m.Dir $m.Mag $MotorVelocityRpm $AccelerationRpmS
    }
    $wait = Get-MoveTimeMs $maxMag $MotorVelocityRpm $AccelerationRpmS 1.0
    if ($wait -lt 1200) { $wait = 1200 }
    Start-Sleep -Milliseconds $wait

    for ($pass = 1; $pass -le 2; $pass++) {
        $any = $false
        foreach ($joint in 1..6) {
            $mid = [int]$JointTable[$joint].MotorId
            $cur = Get-MotorDeg $Serial $mid
            if ([math]::Abs($cur) -lt 1.0) { continue }
            $any = $true
            $dir = if ($cur -gt 0) { [byte]1 } else { [byte]0 }
            Write-Host ("   cleanup#{0} M{1}={2}" -f $pass, $mid, $cur)
            Move-MotorRel $Serial $mid $dir ([math]::Abs($cur)) 260 320
        }
        if (-not $any) { break }
        Start-Sleep -Milliseconds 2500
    }

    Write-Host '   final:'
    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        Write-Host ("   M{0}={1}" -f $mid, (Get-MotorDeg $Serial $mid))
    }
}

if ($J5Amp -le 0 -or $J5Amp -gt 40) { throw 'J5Amp must be in (0,40]' }
if ($J6Amp -le 0 -or $J6Amp -gt 40) { throw 'J6Amp must be in (0,40]' }
if ($Waves -lt 1 -or $Waves -gt 20) { throw 'Waves must be 1..20' }

$started = Get-Date
Write-Host '============================================================'
Write-Host ' ZEROARM BIG HELLO WAVE  (J5 large amplitude)'
Write-Host '============================================================'
Write-Host ("port={0} waves={1} J5Amp={2} J6Amp={3} rpm={4}" -f $Port, $Waves, $J5Amp, $J6Amp, $MotorVelocityRpm)
Write-Host 'NOTE: ENABLE hold only; hard encoder zero at end'
Write-Host '------------------------------------------------------------'

$serial = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, None, 8, One
$serial.ReadTimeout = $ReadTimeoutMs
$serial.WriteTimeout = $ReadTimeoutMs
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$enabled = New-Object System.Collections.Generic.List[int]
$fatal = $null

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    $hello = Invoke-Cmd $serial $CmdHello @()
    if ([Text.Encoding]::ASCII.GetString($hello.Payload) -ne 'ZEROARM/1.0') {
        throw 'HELLO failed'
    }
    Write-Host 'UART HELLO = PASS'

    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        Invoke-EnsureEnabled $serial $mid
        if (-not $SkipSetZero.IsPresent) {
            try {
                Assert-Ok (Invoke-Cmd $serial $CmdBenchSetZero ([byte[]]@([byte]$mid))) $CmdBenchSetZero 'SET_ZERO'
                Write-Host ("  J{0}/M{1}: ENABLED + SET_ZERO" -f $joint, $mid)
            } catch {
                Write-Host ("  J{0}/M{1}: ENABLED" -f $joint, $mid)
            }
        } else {
            Write-Host ("  J{0}/M{1}: ENABLED now={2}" -f $joint, $mid, (Get-MotorDeg $serial $mid))
        }
        if (-not $enabled.Contains($mid)) { $enabled.Add($mid) | Out-Null }
    }

    Move-AllMotorsToZero $serial 'pre hard zero'
    foreach ($mid in $enabled) { Invoke-EnsureEnabled $serial $mid }

    # Ready pose: arm up so wave is obvious; J5 high.
    $ready = Set-PoseValues @{
        1 = 8
        2 = 22
        3 = 14
        4 = 8
        5 = 20
        6 = 0
    }
    Move-ToPose $serial $ready 'raise for hello' 1.0 'settle'
    Start-Sleep -Milliseconds 200

    # Big wave: J5 is the main visual swing, J6 assists, J1/J4 slight body turn.
    for ($i = 1; $i -le $Waves; $i++) {
        $left = Set-PoseValues @{
            1 = -12
            2 = 22
            3 = 14
            4 = -16
            5 = $J5Amp
            6 = -$J6Amp
        }
        $right = Set-PoseValues @{
            1 = 16
            2 = 22
            3 = 14
            4 = 16
            5 = -$J5Amp
            6 = $J6Amp
        }
        # Alternate high/low J5 extremes for very obvious hello.
        if (($i % 2) -eq 0) {
            $left[5] = -$J5Amp
            $right[5] = $J5Amp
        }
        Move-ToPose $serial $left ("HELLO L #$i  J5={0}" -f $left[5]) 1.25 'flow'
        Move-ToPose $serial $right ("HELLO R #$i  J5={0}" -f $right[5]) 1.25 'flow'
    }

    Move-ToPose $serial $ready 'end hello pose' 1.0 'flow'
    Start-Sleep -Milliseconds 250
    Move-AllMotorsToZero $serial 'FINAL hard encoder zero'
    foreach ($mid in $enabled) { Invoke-EnsureEnabled $serial $mid }

    Write-Host ''
    Write-Host 'Final angles:'
    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        $m = Get-MotorDeg $serial $mid
        $j = [math]::Round($m / [double]$JointTable[$joint].Ratio, 3)
        Write-Host ("  J{0}/M{1}: motor={2} joint_approx={3}" -f $joint, $mid, $m, $j)
    }
    Write-Host 'HELLO WAVE COMPLETE = PASS'
} catch {
    $fatal = $_.Exception.Message
    Write-Host ("HELLO WAVE FAIL: {0}" -f $fatal)
    if ($serial.IsOpen) {
        try { Move-AllMotorsToZero $serial 'emergency zero' } catch {}
        foreach ($mid in $enabled) {
            try { Invoke-EnsureEnabled $serial $mid } catch {}
        }
    }
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}

$elapsed = [math]::Round(((Get-Date) - $started).TotalSeconds, 1)
Write-Host ("elapsed = {0}s" -f $elapsed)
if ($fatal) { exit 1 }
exit 0

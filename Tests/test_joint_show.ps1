param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [double]$MotorVelocityRpm = 560.0,
    [int]$AccelerationRpmS = 750,
    [int]$ReadTimeoutMs = 4000,
    [double]$SettleTolMotorDeg = 25.0,
    [switch]$SkipSetZero,
    [switch]$VerboseHex
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Fluid closed-loop gesture party for assembled arm.
# - Feedback-based absolute motor targeting (no software-pose drift)
# - Mostly time-based waits for continuous motion (no micro-stops)
# - Settle only on hard rest / hard zero
# - Pose clamp +/-40 deg joint
# - ENABLE hold only, never DISABLE

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

function Write-Frame([System.IO.Ports.SerialPort]$Serial, [byte[]]$Frame) {
    if ($VerboseHex) {
        Write-Host ("TX: {0}" -f (($Frame | ForEach-Object { '{0:X2}' -f $_ }) -join ' '))
    }
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

function Get-MotorDeg(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId
) {
    $rx = Invoke-Cmd $Serial $CmdBenchQuery ([byte[]]@([byte]$MotorId))
    if ($rx.Payload.Length -eq 1) {
        throw ("QUERY fail motor={0} code={1}" -f $MotorId, $rx.Payload[0])
    }
    $pos = [BitConverter]::ToInt32($rx.Payload, 4)
    return [math]::Round($pos * 180.0 / 3141593.0, 3)
}

function Invoke-EnsureEnabled(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId
) {
    Assert-Ok (Invoke-Cmd $Serial $CmdBenchEnable ([byte[]]@([byte]$MotorId))) $CmdBenchEnable 'ENABLE'
}

function Invoke-SetZero(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId
) {
    Assert-Ok (Invoke-Cmd $Serial $CmdBenchSetZero ([byte[]]@([byte]$MotorId))) $CmdBenchSetZero 'SET_ZERO'
}

function Move-MotorRel(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$MotorId,
    [byte]$Direction,
    [double]$MotorDegrees,
    [double]$VelocityRpm,
    [int]$Accel
) {
    if ([math]::Abs($MotorDegrees) -lt 0.2) { return }
    $degreesTenths = [uint32][math]::Round([math]::Abs($MotorDegrees) * 10.0)
    $velocityTenths = [uint16][math]::Round($VelocityRpm * 10.0)
    $acceleration = [uint16]$Accel
    $payload = [byte[]]@([byte]$MotorId, $Direction) +
        (Write-BeU32 $degreesTenths) +
        (Write-BeU16 $velocityTenths) +
        (Write-BeU16 $acceleration)
    Assert-Ok (Invoke-Cmd $Serial $CmdBenchMoveRel $payload) $CmdBenchMoveRel 'MOVE_REL'
}

function Clamp([double]$Value, [double]$Min, [double]$Max) {
    if ($Value -lt $Min) { return $Min }
    if ($Value -gt $Max) { return $Max }
    return $Value
}

function New-EmptyPose {
    $p = @{}
    foreach ($j in 1..6) { $p[$j] = 0.0 }
    return $p
}

function Format-Pose([hashtable]$Pose) {
    return (
        (1..6 | ForEach-Object {
            $v = if ($Pose.ContainsKey($_)) { [double]$Pose[$_] } else { 0.0 }
            'J{0}={1,5:N0}' -f $_, $v
        }) -join ' '
    )
}

function Set-PoseValues([hashtable]$Values) {
    $out = New-EmptyPose
    foreach ($j in 1..6) {
        if ($Values.ContainsKey($j)) {
            $out[$j] = Clamp ([double]$Values[$j]) (-1.0 * $PoseLimitDeg) $PoseLimitDeg
        } elseif ($Values.ContainsKey("$j")) {
            $out[$j] = Clamp ([double]$Values["$j"]) (-1.0 * $PoseLimitDeg) $PoseLimitDeg
        } else {
            $out[$j] = 0.0
        }
    }
    return $out
}

function Get-TargetMotorDeg([int]$Joint, [double]$JointDeg) {
    $cfg = $JointTable[$Joint]
    return ([double]$cfg.Sign) * $JointDeg * ([double]$cfg.Ratio)
}

function Get-MoveTimeMs(
    [double]$MaxMotorDeg,
    [double]$VelocityRpm,
    [int]$Accel,
    [double]$Overlap = 0.72
) {
    # Overlap < 1 starts next pose before full stop => much smoother look.
    $revs = [math]::Abs($MaxMotorDeg) / 360.0
    if ($VelocityRpm -le 1.0) { $VelocityRpm = 1.0 }
    $constSec = $revs * 60.0 / $VelocityRpm
    $rampSec = 0.0
    if ($Accel -gt 0) {
        $vRps = $VelocityRpm / 60.0
        $aRps2 = $Accel / 60.0
        if ($aRps2 -gt 0.01) { $rampSec = ($vRps / $aRps2) }
    }
    $sec = ($constSec + 0.22 * $rampSec) * $Overlap + 0.05
    $ms = [int][math]::Ceiling($sec * 1000.0)
    if ($ms -lt 160) { $ms = 160 }
    if ($ms -gt 9000) { $ms = 9000 }
    return $ms
}

function Move-ToPose(
    [System.IO.Ports.SerialPort]$Serial,
    [hashtable]$TargetPose,
    [string]$Name,
    [double]$VelocityScale = 1.0,
    [string]$WaitMode = 'flow',   # flow | settle
    [double]$Overlap = 0.72
) {
    Write-Host (">> {0}" -f $Name)

    $vel = [math]::Max(140.0, $MotorVelocityRpm * $VelocityScale)
    $acc = [int][math]::Max(160, [math]::Round($AccelerationRpmS * $VelocityScale))
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
        $cfg = $JointTable[$joint]
        $mid = [int]$cfg.MotorId
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

    if ($WaitMode -eq 'settle') {
        # Short hard wait then one cleanup for true rest poses.
        $waitMs = Get-MoveTimeMs $maxMag $vel $acc 0.95
        Start-Sleep -Milliseconds $waitMs
        foreach ($joint in 1..6) {
            $mid = [int]$JointTable[$joint].MotorId
            $cur = Get-MotorDeg $Serial $mid
            $tgt = [double]$targets[$mid]
            $delta = $tgt - $cur
            if ([math]::Abs($delta) -lt 2.0) { continue }
            $dir = if ($delta -ge 0) { [byte]0 } else { [byte]1 }
            Move-MotorRel $Serial $mid $dir ([math]::Abs($delta)) ([math]::Max(180.0, $vel * 0.65)) $acc
        }
        Start-Sleep -Milliseconds 700
    } else {
        $waitMs = Get-MoveTimeMs $maxMag $vel $acc $Overlap
        Start-Sleep -Milliseconds $waitMs
    }
}

function Move-AllMotorsToZero(
    [System.IO.Ports.SerialPort]$Serial,
    [string]$Name = 'hard zero from encoder'
) {
    Write-Host (">> {0}" -f $Name)
    $maxMag = 0.0
    $moves = New-Object System.Collections.Generic.List[object]
    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        $cur = Get-MotorDeg $Serial $mid
        Write-Host ("   M{0}: now={1}" -f $mid, $cur)
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
    $waitMs = Get-MoveTimeMs $maxMag $MotorVelocityRpm $AccelerationRpmS 1.0
    if ($waitMs -lt 1200) { $waitMs = 1200 }
    Start-Sleep -Milliseconds $waitMs

    for ($pass = 1; $pass -le 2; $pass++) {
        $any = $false
        foreach ($joint in 1..6) {
            $mid = [int]$JointTable[$joint].MotorId
            $cur = Get-MotorDeg $Serial $mid
            if ([math]::Abs($cur) -lt 1.2) { continue }
            $any = $true
            $dir = if ($cur -gt 0) { [byte]1 } else { [byte]0 }
            Write-Host ("   cleanup#{0} M{1}={2}" -f $pass, $mid, $cur)
            Move-MotorRel $Serial $mid $dir ([math]::Abs($cur)) 300 380
        }
        if (-not $any) { break }
        Start-Sleep -Milliseconds 2200
    }

    Write-Host '   final feedback:'
    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        Write-Host ("   M{0}={1}" -f $mid, (Get-MotorDeg $Serial $mid))
    }
}

function Pause-Ms([int]$Ms, [string]$Why) {
    if ($Ms -le 0) { return }
    Write-Host ("   hold {0}ms ({1})" -f $Ms, $Why)
    Start-Sleep -Milliseconds $Ms
}

function Hold-EnableAll(
    [System.IO.Ports.SerialPort]$Serial,
    [System.Collections.Generic.List[int]]$Motors
) {
    foreach ($mid in $Motors) {
        try { Invoke-EnsureEnabled $Serial $mid } catch {}
    }
}

if ($MotorVelocityRpm -le 0 -or $MotorVelocityRpm -gt 900) {
    throw "MotorVelocityRpm must be in (0, 900]"
}

$started = Get-Date
Write-Host '============================================================'
Write-Host ' ZEROARM FLUID FUN SHOW  (+/-40, continuous, closed-loop)'
Write-Host '============================================================'
Write-Host ("time      : {0}" -f $started.ToString('yyyy-MM-dd HH:mm:ss'))
Write-Host ("port      : {0}" -f $Port)
Write-Host ("motor_rpm : {0}" -f $MotorVelocityRpm)
Write-Host 'flow      : overlapping time waits, few hard rests'
Write-Host 'gestures  : hello, yes/no, look, bow, invite, bounce,'
Write-Host '            think, surprise, cheer, spin, peekaboo,'
Write-Host '            robot-dance, high-five, heart, swagger, bye'
Write-Host 'NOTE      : ENABLE hold; hard encoder zero at end'
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

    $state = Invoke-Cmd $serial $CmdGetState @()
    if ($state.Payload.Length -lt 60) { throw 'GET_STATE failed' }
    Write-Host ("GET_STATE = PASS run={0} fault=0x{1:X8}" -f
        ([BitConverter]::ToInt32($state.Payload, 0)),
        ([BitConverter]::ToUInt32($state.Payload, 56)))

    Write-Host 'Enable all joints...'
    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        Invoke-EnsureEnabled $serial $mid
        if (-not $SkipSetZero.IsPresent) {
            try {
                Invoke-SetZero $serial $mid
                Write-Host ("  J{0}/M{1}: ENABLED + SET_ZERO now={2}" -f $joint, $mid, (Get-MotorDeg $serial $mid))
            } catch {
                Write-Host ("  J{0}/M{1}: ENABLED now={2}" -f $joint, $mid, (Get-MotorDeg $serial $mid))
            }
        } else {
            Write-Host ("  J{0}/M{1}: ENABLED now={2}" -f $joint, $mid, (Get-MotorDeg $serial $mid))
        }
        if (-not $enabled.Contains($mid)) { $enabled.Add($mid) | Out-Null }
    }

    Move-AllMotorsToZero $serial 'pre-show hard zero'
    Hold-EnableAll $serial $enabled
    $rest = New-EmptyPose

    # 1 Hello
    Write-Host ''
    Write-Host '=== 1) HELLO ==='
    Move-ToPose $serial (Set-PoseValues @{ 1 = 8; 2 = 24; 3 = 16; 4 = 8; 5 = 32; 6 = 0 }) 'raise' 1.05 'flow' 0.8
    for ($i = 1; $i -le 6; $i++) {
        Move-ToPose $serial (Set-PoseValues @{ 1 = -14; 2 = 24; 3 = 16; 4 = -18; 5 = 36; 6 = -36 }) "hi L$i" 1.25 'flow' 0.62
        Move-ToPose $serial (Set-PoseValues @{ 1 = 20; 2 = 24; 3 = 16; 4 = 20; 5 = 36; 6 = 36 }) "hi R$i" 1.25 'flow' 0.62
    }
    Move-ToPose $serial $rest 'rest' 1.05 'settle' 0.95

    # 2 Yes / No
    Write-Host ''
    Write-Host '=== 2) YES / NO ==='
    for ($i = 1; $i -le 3; $i++) {
        Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 36; 3 = 26; 4 = 0; 5 = 14; 6 = 0 }) "yes$i" 1.2 'flow' 0.6
        Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 8; 3 = 6; 4 = 0; 5 = 8; 6 = 0 }) "yesUp$i" 1.2 'flow' 0.6
    }
    for ($i = 1; $i -le 4; $i++) {
        Move-ToPose $serial (Set-PoseValues @{ 1 = -34; 2 = 14; 3 = 10; 4 = -16; 5 = 10; 6 = -14 }) "noL$i" 1.3 'flow' 0.58
        Move-ToPose $serial (Set-PoseValues @{ 1 = 34; 2 = 14; 3 = 10; 4 = 16; 5 = 10; 6 = 14 }) "noR$i" 1.3 'flow' 0.58
    }
    Move-ToPose $serial $rest 'rest' 1.05 'settle' 0.95

    # 3 Look
    Write-Host ''
    Write-Host '=== 3) LOOK ==='
    Move-ToPose $serial (Set-PoseValues @{ 1 = -38; 2 = 10; 3 = 8; 4 = -20; 5 = 12; 6 = -16 }) 'lookL' 1.15 'flow' 0.7
    Pause-Ms 140 'L'
    Move-ToPose $serial (Set-PoseValues @{ 1 = 38; 2 = 10; 3 = 8; 4 = 20; 5 = 12; 6 = 16 }) 'lookR' 1.15 'flow' 0.7
    Pause-Ms 140 'R'
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = -14; 3 = -10; 4 = 0; 5 = 30; 6 = 0 }) 'lookUp' 1.05 'flow' 0.7
    Pause-Ms 120 'up'
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 32; 3 = 22; 4 = 0; 5 = 8; 6 = 0 }) 'lookDn' 1.05 'flow' 0.7
    Move-ToPose $serial $rest 'rest' 1.05 'settle' 0.95

    # 4 Bow
    Write-Host ''
    Write-Host '=== 4) BOW ==='
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 12; 3 = 8; 4 = 0; 5 = 10; 6 = 0 }) 'ready' 1.0 'flow' 0.75
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 38; 3 = 30; 4 = 0; 5 = 18; 6 = 0 }) 'deep' 0.75 'settle' 0.95
    Pause-Ms 320 'respect'
    Move-ToPose $serial $rest 'up' 0.85 'settle' 0.95

    # 5 Invite
    Write-Host ''
    Write-Host '=== 5) INVITE ==='
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 18; 3 = 12; 4 = 0; 5 = 22; 6 = 0 }) 'open' 1.1 'flow' 0.7
    Move-ToPose $serial (Set-PoseValues @{ 1 = -30; 2 = 16; 3 = 12; 4 = -24; 5 = 28; 6 = -20 }) 'invL' 1.1 'flow' 0.7
    Pause-Ms 160 'L'
    Move-ToPose $serial (Set-PoseValues @{ 1 = 30; 2 = 16; 3 = 12; 4 = 24; 5 = 28; 6 = 20 }) 'invR' 1.1 'flow' 0.7
    Pause-Ms 160 'R'
    Move-ToPose $serial $rest 'rest' 1.05 'settle' 0.95

    # 6 Excited bounce
    Write-Host ''
    Write-Host '=== 6) BOUNCE ==='
    for ($i = 1; $i -le 6; $i++) {
        $s = if (($i % 2) -eq 0) { -1 } else { 1 }
        Move-ToPose $serial (Set-PoseValues @{ 1 = 12 * $s; 2 = 30; 3 = 20; 4 = 14 * $s; 5 = 32; 6 = 20 * $s }) "up$i" 1.35 'flow' 0.55
        Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 10; 3 = 8; 4 = 0; 5 = 12; 6 = 0 }) "dn$i" 1.35 'flow' 0.55
    }

    # 7 Think -> Aha (no hard rest between)
    Write-Host ''
    Write-Host '=== 7) THINK / AHA ==='
    Move-ToPose $serial (Set-PoseValues @{ 1 = 18; 2 = 22; 3 = 18; 4 = 22; 5 = 30; 6 = 14 }) 'think' 0.95 'flow' 0.75
    for ($i = 1; $i -le 3; $i++) {
        Move-ToPose $serial (Set-PoseValues @{ 1 = 24; 2 = 24; 3 = 18; 4 = 26; 5 = 32; 6 = 18 }) "thR$i" 1.0 'flow' 0.62
        Move-ToPose $serial (Set-PoseValues @{ 1 = 10; 2 = 18; 3 = 14; 4 = 12; 5 = 24; 6 = 8 }) "thL$i" 1.0 'flow' 0.62
    }
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 6; 3 = 4; 4 = 0; 5 = 22; 6 = 0 }) 'aha' 1.2 'flow' 0.7
    Pause-Ms 140 'idea'
    Move-ToPose $serial $rest 'rest' 1.05 'settle' 0.95

    # 8 Surprise
    Write-Host ''
    Write-Host '=== 8) SURPRISE ==='
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 8; 3 = 6; 4 = 0; 5 = 8; 6 = 0 }) 'calm' 1.0 'flow' 0.7
    Move-ToPose $serial (Set-PoseValues @{ 1 = -20; 2 = -12; 3 = -8; 4 = 18; 5 = 36; 6 = 30 }) 'shock' 1.4 'flow' 0.65
    Pause-Ms 180 '!'
    Move-ToPose $serial (Set-PoseValues @{ 1 = 20; 2 = -8; 3 = -6; 4 = -18; 5 = 34; 6 = -28 }) 'flip' 1.3 'flow' 0.65
    Move-ToPose $serial $rest 'rest' 1.1 'settle' 0.95

    # 9 Cheer / clap-ish
    Write-Host ''
    Write-Host '=== 9) CHEER ==='
    for ($i = 1; $i -le 6; $i++) {
        Move-ToPose $serial (Set-PoseValues @{ 1 = 14; 2 = 16; 3 = 10; 4 = -22; 5 = 26; 6 = -26 }) "open$i" 1.35 'flow' 0.52
        Move-ToPose $serial (Set-PoseValues @{ 1 = -14; 2 = 22; 3 = 16; 4 = 22; 5 = 30; 6 = 26 }) "close$i" 1.35 'flow' 0.52
    }
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 28; 3 = 18; 4 = 0; 5 = 32; 6 = 0 }) 'yay' 1.1 'flow' 0.7
    Pause-Ms 160 'yay'
    Move-ToPose $serial $rest 'rest' 1.1 'settle' 0.95

    # 10 Spin greeting
    Write-Host ''
    Write-Host '=== 10) SPIN GREET ==='
    foreach ($a in @(-36, -20, 0, 20, 36, 20, 0, -20, -36, 0)) {
        $hand = if ($a -ge 0) { 30 } else { -30 }
        Move-ToPose $serial (Set-PoseValues @{
                1 = $a; 2 = 20; 3 = 14; 4 = [math]::Round($a * 0.45, 0); 5 = 30; 6 = $hand
            }) "spin$a" 1.2 'flow' 0.58
    }
    Move-ToPose $serial $rest 'rest' 1.05 'settle' 0.95

    # 11 Peekaboo
    Write-Host ''
    Write-Host '=== 11) PEEKABOO ==='
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 30; 3 = 24; 4 = 0; 5 = 34; 6 = 0 }) 'cover' 1.1 'flow' 0.7
    Pause-Ms 180 'hide'
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 8; 3 = 6; 4 = 0; 5 = 12; 6 = 0 }) 'peek' 1.3 'flow' 0.6
    Pause-Ms 120 'hi'
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 30; 3 = 24; 4 = 0; 5 = 34; 6 = 0 }) 'cover2' 1.15 'flow' 0.65
    Pause-Ms 120 'hide'
    Move-ToPose $serial (Set-PoseValues @{ 1 = 18; 2 = 10; 3 = 8; 4 = 12; 5 = 18; 6 = 16 }) 'side' 1.25 'flow' 0.65
    Pause-Ms 120 'boo'
    Move-ToPose $serial $rest 'rest' 1.05 'settle' 0.95

    # 12 Robot dance groove
    Write-Host ''
    Write-Host '=== 12) ROBOT DANCE ==='
    for ($i = 1; $i -le 8; $i++) {
        $s = if (($i % 2) -eq 0) { 1 } else { -1 }
        Move-ToPose $serial (Set-PoseValues @{
                1 = 24 * $s; 2 = 18; 3 = 12; 4 = 18 * (-1 * $s); 5 = 26; 6 = 26 * $s
            }) "groove$i" 1.35 'flow' 0.5
    }
    for ($i = 1; $i -le 6; $i++) {
        $s = if (($i % 2) -eq 0) { 1 } else { -1 }
        Move-ToPose $serial (Set-PoseValues @{
                1 = 32 * $s; 2 = 24; 3 = 14; 4 = 22 * (-1 * $s); 5 = 30; 6 = 32 * $s
            }) "fast$i" 1.45 'flow' 0.46
    }
    Move-ToPose $serial $rest 'rest' 1.15 'settle' 0.95

    # 13 High five
    Write-Host ''
    Write-Host '=== 13) HIGH FIVE ==='
    Move-ToPose $serial (Set-PoseValues @{ 1 = 8; 2 = 8; 3 = 6; 4 = 10; 5 = 20; 6 = 8 }) 'windup' 1.05 'flow' 0.7
    Move-ToPose $serial (Set-PoseValues @{ 1 = 12; 2 = -8; 3 = -6; 4 = 14; 5 = 36; 6 = 20 }) 'up' 1.25 'flow' 0.65
    Pause-Ms 180 'slap'
    Move-ToPose $serial (Set-PoseValues @{ 1 = 8; 2 = 16; 3 = 10; 4 = 8; 5 = 18; 6 = 6 }) 'rebound' 1.15 'flow' 0.65
    Move-ToPose $serial $rest 'rest' 1.05 'settle' 0.95

    # 14 Heart-ish / cute pose sequence
    Write-Host ''
    Write-Host '=== 14) CUTE HEART ==='
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 20; 3 = 16; 4 = 0; 5 = 24; 6 = 0 }) 'center' 1.0 'flow' 0.7
    Move-ToPose $serial (Set-PoseValues @{ 1 = -16; 2 = 24; 3 = 18; 4 = -20; 5 = 30; 6 = -18 }) 'left lobe' 1.05 'flow' 0.65
    Move-ToPose $serial (Set-PoseValues @{ 1 = 16; 2 = 24; 3 = 18; 4 = 20; 5 = 30; 6 = 18 }) 'right lobe' 1.05 'flow' 0.65
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 30; 3 = 22; 4 = 0; 5 = 28; 6 = 0 }) 'bottom point' 1.0 'flow' 0.7
    Pause-Ms 200 'heart'
    Move-ToPose $serial $rest 'rest' 1.05 'settle' 0.95

    # 15 Swagger walk-ish
    Write-Host ''
    Write-Host '=== 15) SWAGGER ==='
    for ($i = 1; $i -le 6; $i++) {
        $s = if (($i % 2) -eq 0) { 1 } else { -1 }
        Move-ToPose $serial (Set-PoseValues @{
                1 = 18 * $s
                2 = 16 + 6 * $s
                3 = 10
                4 = 12 * (-1 * $s)
                5 = 18
                6 = 14 * $s
            }) "swagger$i" 1.2 'flow' 0.55
    }
    Move-ToPose $serial $rest 'rest' 1.05 'settle' 0.95

    # 16 Conductor / orchestra
    Write-Host ''
    Write-Host '=== 16) CONDUCTOR ==='
    for ($i = 1; $i -le 5; $i++) {
        Move-ToPose $serial (Set-PoseValues @{ 1 = -10; 2 = 12; 3 = 8; 4 = -24; 5 = 28; 6 = -20 }) "batonL$i" 1.2 'flow' 0.55
        Move-ToPose $serial (Set-PoseValues @{ 1 = 10; 2 = 18; 3 = 12; 4 = 24; 5 = 20; 6 = 22 }) "batonR$i" 1.2 'flow' 0.55
        Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 8; 3 = 6; 4 = 0; 5 = 32; 6 = 0 }) "batonUp$i" 1.15 'flow' 0.55
    }
    Move-ToPose $serial $rest 'rest' 1.05 'settle' 0.95

    # 17 Chicken wing joke
    Write-Host ''
    Write-Host '=== 17) CHICKEN WINGS ==='
    for ($i = 1; $i -le 6; $i++) {
        Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 28; 3 = 22; 4 = -28; 5 = 20; 6 = -16 }) "wingIn$i" 1.3 'flow' 0.5
        Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 16; 3 = 10; 4 = 28; 5 = 24; 6 = 18 }) "wingOut$i" 1.3 'flow' 0.5
    }
    Move-ToPose $serial $rest 'rest' 1.1 'settle' 0.95

    # 18 Macarena-lite (simple stepwise arm pattern)
    Write-Host ''
    Write-Host '=== 18) MACARENA-lite ==='
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 10; 3 = 8; 4 = 0; 5 = 12; 6 = 0 }) 'm1' 1.05 'flow' 0.7
    Move-ToPose $serial (Set-PoseValues @{ 1 = 12; 2 = 12; 3 = 8; 4 = 18; 5 = 20; 6 = 12 }) 'm2' 1.1 'flow' 0.65
    Move-ToPose $serial (Set-PoseValues @{ 1 = -12; 2 = 12; 3 = 8; 4 = -18; 5 = 20; 6 = -12 }) 'm3' 1.1 'flow' 0.65
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 20; 3 = 14; 4 = 0; 5 = 28; 6 = 0 }) 'm4' 1.1 'flow' 0.65
    Move-ToPose $serial (Set-PoseValues @{ 1 = 16; 2 = 22; 3 = 14; 4 = 16; 5 = 24; 6 = 16 }) 'm5' 1.15 'flow' 0.6
    Move-ToPose $serial (Set-PoseValues @{ 1 = -16; 2 = 22; 3 = 14; 4 = -16; 5 = 24; 6 = -16 }) 'm6' 1.15 'flow' 0.6
    Move-ToPose $serial (Set-PoseValues @{ 1 = 0; 2 = 28; 3 = 18; 4 = 0; 5 = 30; 6 = 0 }) 'm7' 1.1 'flow' 0.65
    Move-ToPose $serial $rest 'rest' 1.05 'settle' 0.95

    # 19 Final bye
    Write-Host ''
    Write-Host '=== 19) BYE ==='
    Move-ToPose $serial (Set-PoseValues @{ 1 = 12; 2 = 26; 3 = 18; 4 = 10; 5 = 34; 6 = 0 }) 'raise' 1.0 'flow' 0.75
    for ($i = 1; $i -le 7; $i++) {
        Move-ToPose $serial (Set-PoseValues @{ 1 = -12; 2 = 26; 3 = 18; 4 = -20; 5 = 36; 6 = -36 }) "byeL$i" 1.2 'flow' 0.55
        Move-ToPose $serial (Set-PoseValues @{ 1 = 22; 2 = 26; 3 = 18; 4 = 22; 5 = 36; 6 = 36 }) "byeR$i" 1.2 'flow' 0.55
    }
    Move-ToPose $serial (Set-PoseValues @{ 1 = 12; 2 = 26; 3 = 18; 4 = 10; 5 = 34; 6 = 0 }) 'hold' 0.95 'flow' 0.75
    Pause-Ms 220 'bye'

    Move-AllMotorsToZero $serial 'FINAL hard encoder zero'
    Hold-EnableAll $serial $enabled

    Write-Host ''
    Write-Host 'Final measured angles:'
    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        $cur = Get-MotorDeg $serial $mid
        $japprox = [math]::Round($cur / [double]$JointTable[$joint].Ratio, 3)
        Write-Host ("  J{0}/M{1}: motor={2} joint_approx={3}" -f $joint, $mid, $cur, $japprox)
    }
    Write-Host 'SHOW COMPLETE = PASS'
} catch {
    $fatal = $_.Exception.Message
    Write-Host ("SHOW FAIL: {0}" -f $fatal)
    if ($serial.IsOpen) {
        try { Move-AllMotorsToZero $serial 'emergency zero' } catch {}
        Hold-EnableAll $serial $enabled
    }
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}

$elapsed = [math]::Round(((Get-Date) - $started).TotalSeconds, 1)
Write-Host ("elapsed = {0}s" -f $elapsed)
if ($fatal) { exit 1 }
exit 0

param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [double]$MotorVelocityRpm = 480.0,
    [int]$AccelerationRpmS = 650,
    [int]$ReadTimeoutMs = 4000,
    [switch]$SkipSetZero
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Big, obvious multi-gesture combo for assembled arm.
# Each gesture uses large joint travel and a clear hold so humans can see it.
# Closed-loop from encoder feedback. ENABLE hold only. Hard zero at end.

$CmdHello = 0x00
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
    if ([math]::Abs($MotorDegrees) -lt 0.3) { return }
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

function Get-MoveTimeMs(
    [double]$MaxMotorDeg,
    [double]$VelocityRpm,
    [int]$Accel,
    [double]$Overlap = 0.92
) {
    $revs = [math]::Abs($MaxMotorDeg) / 360.0
    if ($VelocityRpm -le 1.0) { $VelocityRpm = 1.0 }
    $constSec = $revs * 60.0 / $VelocityRpm
    $rampSec = 0.0
    if ($Accel -gt 0) {
        $vRps = $VelocityRpm / 60.0
        $aRps2 = $Accel / 60.0
        if ($aRps2 -gt 0.01) { $rampSec = $vRps / $aRps2 }
    }
    $ms = [int][math]::Ceiling((($constSec + 0.25 * $rampSec) * $Overlap + 0.08) * 1000.0)
    if ($ms -lt 280) { $ms = 280 }
    if ($ms -gt 10000) { $ms = 10000 }
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
    [string]$Mode = 'settle'
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
        $mid = [int]$JointTable[$joint].MotorId
        $tgtJoint = Clamp ([double]$TargetPose[$joint]) (-1.0 * $PoseLimitDeg) $PoseLimitDeg
        $tgtMotor = Get-TargetMotorDeg $joint $tgtJoint
        $curMotor = [double]$current[$mid]
        $delta = $tgtMotor - $curMotor
        $targets[$mid] = $tgtMotor
        if ([math]::Abs($delta) -lt 0.6) { continue }
        $dir = if ($delta -ge 0) { [byte]0 } else { [byte]1 }
        $mag = [math]::Abs($delta)
        if ($mag -gt $maxMag) { $maxMag = $mag }
        $moves.Add([pscustomobject]@{ MotorId = $mid; Dir = $dir; Mag = $mag }) | Out-Null
    }

    if ($moves.Count -eq 0) {
        Write-Host '   (already there)'
        return
    }

    foreach ($m in $moves) {
        Move-MotorRel $Serial $m.MotorId $m.Dir $m.Mag $vel $acc
    }

    if ($Mode -eq 'flow') {
        Start-Sleep -Milliseconds (Get-MoveTimeMs $maxMag $vel $acc 0.70)
        return
    }

    # settle: wait almost full move + one cleanup so pose is visually complete
    Start-Sleep -Milliseconds (Get-MoveTimeMs $maxMag $vel $acc 0.98)
    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        $cur = Get-MotorDeg $Serial $mid
        $tgt = [double]$targets[$mid]
        $delta = $tgt - $cur
        if ([math]::Abs($delta) -lt 2.5) { continue }
        $dir = if ($delta -ge 0) { [byte]0 } else { [byte]1 }
        Move-MotorRel $Serial $mid $dir ([math]::Abs($delta)) ([math]::Max(180.0, $vel * 0.65)) $acc
    }
    Start-Sleep -Milliseconds 800
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
    if ($wait -lt 1500) { $wait = 1500 }
    Start-Sleep -Milliseconds $wait

    for ($pass = 1; $pass -le 3; $pass++) {
        $any = $false
        foreach ($joint in 1..6) {
            $mid = [int]$JointTable[$joint].MotorId
            $cur = Get-MotorDeg $Serial $mid
            if ([math]::Abs($cur) -lt 1.0) { continue }
            $any = $true
            $dir = if ($cur -gt 0) { [byte]1 } else { [byte]0 }
            Write-Host ("   cleanup#{0} M{1}={2}" -f $pass, $mid, $cur)
            Move-MotorRel $Serial $mid $dir ([math]::Abs($cur)) 240 320
        }
        if (-not $any) { break }
        Start-Sleep -Milliseconds 2800
    }

    Write-Host '   final feedback:'
    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        Write-Host ("   M{0}={1}" -f $mid, (Get-MotorDeg $Serial $mid))
    }
}

function Hold-EnableAll(
    [System.IO.Ports.SerialPort]$Serial,
    [System.Collections.Generic.List[int]]$Motors
) {
    foreach ($mid in $Motors) {
        try { Invoke-EnsureEnabled $Serial $mid } catch {}
    }
}

function Pause-Ms([int]$Ms, [string]$Why) {
    Write-Host ("   >>> HOLD {0}ms  ({1})" -f $Ms, $Why)
    Start-Sleep -Milliseconds $Ms
}

if ($MotorVelocityRpm -le 0 -or $MotorVelocityRpm -gt 900) {
    throw "MotorVelocityRpm must be in (0, 900]"
}

$started = Get-Date
Write-Host '============================================================'
Write-Host ' ZEROARM OBVIOUS GESTURE COMBO'
Write-Host '============================================================'
Write-Host ("time : {0}" -f $started.ToString('yyyy-MM-dd HH:mm:ss'))
Write-Host ("port : {0}" -f $Port)
Write-Host ("rpm  : {0}" -f $MotorVelocityRpm)
Write-Host 'style: LARGE poses + long holds so each action is obvious'
Write-Host 'seq  : hello / nod / no / look / bow / surprise / bye'
Write-Host 'NOTE : ENABLE hold; hard encoder zero at end; no SET_ZERO by default'
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
            # default: do NOT rewrite zero unless user explicitly omits -SkipSetZero
            # still allow optional set-zero only when requested without switch
        }
        $now = Get-MotorDeg $serial $mid
        Write-Host ("  J{0}/M{1}: ENABLED now={2}" -f $joint, $mid, $now)
        if (-not $enabled.Contains($mid)) { $enabled.Add($mid) | Out-Null }
    }

    # Always start from real encoder zero
    Move-AllMotorsToZero $serial 'PRE hard zero from encoder'
    Hold-EnableAll $serial $enabled
    $rest = Set-PoseValues @{}

    # =========================================================
    # 1) BIG HELLO - J5 large swing, long holds
    # =========================================================
    Write-Host ''
    Write-Host '=== 1) BIG HELLO (你好) ==='
    Move-ToPose $serial (Set-PoseValues @{
            1 = 10; 2 = 26; 3 = 16; 4 = 10; 5 = 28; 6 = 0
        }) 'raise hand high' 0.95 'settle'
    Pause-Ms 500 'see raised hand'

    for ($i = 1; $i -le 6; $i++) {
        Move-ToPose $serial (Set-PoseValues @{
                1 = -14; 2 = 26; 3 = 16; 4 = -18; 5 = 40; 6 = -36
            }) ("HELLO LEFT  #$i  (J5=+40)" ) 1.15 'settle'
        Pause-Ms 280 'left'
        Move-ToPose $serial (Set-PoseValues @{
                1 = 18; 2 = 26; 3 = 16; 4 = 20; 5 = -40; 6 = 36
            }) ("HELLO RIGHT #$i  (J5=-40)") 1.15 'settle'
        Pause-Ms 280 'right'
    }
    Move-ToPose $serial $rest 'back to rest after hello' 0.95 'settle'
    Pause-Ms 600 'gesture complete'

    # =========================================================
    # 2) BIG NOD YES
    # =========================================================
    Write-Host ''
    Write-Host '=== 2) BIG NOD (YES) ==='
    for ($i = 1; $i -le 4; $i++) {
        Move-ToPose $serial (Set-PoseValues @{
                1 = 0; 2 = 40; 3 = 30; 4 = 0; 5 = 16; 6 = 0
            }) ("YES down #$i") 1.05 'settle'
        Pause-Ms 350 'down'
        Move-ToPose $serial (Set-PoseValues @{
                1 = 0; 2 = 6; 3 = 4; 4 = 0; 5 = 8; 6 = 0
            }) ("YES up   #$i") 1.05 'settle'
        Pause-Ms 300 'up'
    }
    Move-ToPose $serial $rest 'rest after yes' 0.95 'settle'
    Pause-Ms 500 'gesture complete'

    # =========================================================
    # 3) BIG SHAKE NO
    # =========================================================
    Write-Host ''
    Write-Host '=== 3) BIG SHAKE (NO) ==='
    for ($i = 1; $i -le 5; $i++) {
        Move-ToPose $serial (Set-PoseValues @{
                1 = -38; 2 = 16; 3 = 10; 4 = -18; 5 = 12; 6 = -16
            }) ("NO left  #$i") 1.15 'settle'
        Pause-Ms 280 'left'
        Move-ToPose $serial (Set-PoseValues @{
                1 = 38; 2 = 16; 3 = 10; 4 = 18; 5 = 12; 6 = 16
            }) ("NO right #$i") 1.15 'settle'
        Pause-Ms 280 'right'
    }
    Move-ToPose $serial $rest 'rest after no' 0.95 'settle'
    Pause-Ms 500 'gesture complete'

    # =========================================================
    # 4) LOOK LEFT / RIGHT / UP / DOWN
    # =========================================================
    Write-Host ''
    Write-Host '=== 4) LOOK AROUND ==='
    Move-ToPose $serial (Set-PoseValues @{
            1 = -40; 2 = 12; 3 = 8; 4 = -22; 5 = 14; 6 = -18
        }) 'LOOK LEFT' 1.0 'settle'
    Pause-Ms 700 'stare left'
    Move-ToPose $serial (Set-PoseValues @{
            1 = 40; 2 = 12; 3 = 8; 4 = 22; 5 = 14; 6 = 18
        }) 'LOOK RIGHT' 1.0 'settle'
    Pause-Ms 700 'stare right'
    Move-ToPose $serial (Set-PoseValues @{
            1 = 0; 2 = -16; 3 = -12; 4 = 0; 5 = 34; 6 = 0
        }) 'LOOK UP' 0.95 'settle'
    Pause-Ms 600 'stare up'
    Move-ToPose $serial (Set-PoseValues @{
            1 = 0; 2 = 36; 3 = 26; 4 = 0; 5 = 8; 6 = 0
        }) 'LOOK DOWN' 0.95 'settle'
    Pause-Ms 600 'stare down'
    Move-ToPose $serial $rest 'center' 0.95 'settle'
    Pause-Ms 500 'gesture complete'

    # =========================================================
    # 5) DEEP BOW
    # =========================================================
    Write-Host ''
    Write-Host '=== 5) DEEP BOW ==='
    Move-ToPose $serial (Set-PoseValues @{
            1 = 0; 2 = 12; 3 = 8; 4 = 0; 5 = 12; 6 = 0
        }) 'bow ready' 0.9 'settle'
    Pause-Ms 350 'breath'
    Move-ToPose $serial (Set-PoseValues @{
            1 = 0; 2 = 40; 3 = 34; 4 = 0; 5 = 20; 6 = 0
        }) 'bow DEEP' 0.7 'settle'
    Pause-Ms 900 'hold respect'
    Move-ToPose $serial (Set-PoseValues @{
            1 = 0; 2 = 12; 3 = 8; 4 = 0; 5 = 12; 6 = 0
        }) 'bow rise' 0.75 'settle'
    Pause-Ms 400 'stand'
    Move-ToPose $serial $rest 'rest after bow' 0.9 'settle'
    Pause-Ms 500 'gesture complete'

    # =========================================================
    # 6) SURPRISE (very obvious jump)
    # =========================================================
    Write-Host ''
    Write-Host '=== 6) SURPRISE ==='
    Move-ToPose $serial (Set-PoseValues @{
            1 = 0; 2 = 10; 3 = 6; 4 = 0; 5 = 8; 6 = 0
        }) 'calm' 1.0 'settle'
    Pause-Ms 400 'quiet...'
    Move-ToPose $serial (Set-PoseValues @{
            1 = -22; 2 = -14; 3 = -10; 4 = 20; 5 = 40; 6 = 32
        }) 'SURPRISE jump' 1.25 'settle'
    Pause-Ms 700 'shock hold'
    Move-ToPose $serial $rest 'recover' 1.0 'settle'
    Pause-Ms 500 'gesture complete'

    # =========================================================
    # 7) BIG BYE wave (J5 huge)
    # =========================================================
    Write-Host ''
    Write-Host '=== 7) BIG BYE (拜拜) ==='
    Move-ToPose $serial (Set-PoseValues @{
            1 = 12; 2 = 28; 3 = 18; 4 = 12; 5 = 30; 6 = 0
        }) 'raise bye hand' 0.95 'settle'
    Pause-Ms 450 'ready bye'

    for ($i = 1; $i -le 7; $i++) {
        Move-ToPose $serial (Set-PoseValues @{
                1 = -12; 2 = 28; 3 = 18; 4 = -20; 5 = 40; 6 = -38
            }) ("BYE LEFT  #$i  J5=+40") 1.15 'settle'
        Pause-Ms 250 'L'
        Move-ToPose $serial (Set-PoseValues @{
                1 = 20; 2 = 28; 3 = 18; 4 = 22; 5 = -40; 6 = 38
            }) ("BYE RIGHT #$i  J5=-40") 1.15 'settle'
        Pause-Ms 250 'R'
    }

    Move-ToPose $serial (Set-PoseValues @{
            1 = 12; 2 = 28; 3 = 18; 4 = 12; 5 = 30; 6 = 0
        }) 'bye hold pose' 0.95 'settle'
    Pause-Ms 700 'bye smile'

    # HARD final zero
    Move-AllMotorsToZero $serial 'FINAL hard encoder zero'
    Hold-EnableAll $serial $enabled

    Write-Host ''
    Write-Host 'Final measured angles:'
    foreach ($joint in 1..6) {
        $mid = [int]$JointTable[$joint].MotorId
        $m = Get-MotorDeg $serial $mid
        $j = [math]::Round($m / [double]$JointTable[$joint].Ratio, 3)
        Write-Host ("  J{0}/M{1}: motor={2} joint_approx={3}" -f $joint, $mid, $m, $j)
    }
    Write-Host 'COMBO COMPLETE = PASS'
} catch {
    $fatal = $_.Exception.Message
    Write-Host ("COMBO FAIL: {0}" -f $fatal)
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

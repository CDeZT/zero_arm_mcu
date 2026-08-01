param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [int]$ReadTimeoutMs = 2500,
    [double]$PositionToleranceDegrees = 2.0,
    [switch]$SkipJ5Motion
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$CmdGetState = [byte]0x01
$CmdEnable = [byte]0x02
$CmdStop = [byte]0x04
$CmdSetJointTarget = [byte]0x05
$RobotOk = 0
$RobotErrRange = 3
$ReadyState = 1

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
        $Payload = [byte[]]@()
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
    [byte]$receivedCrc = [byte]$Serial.ReadByte()
    $etx = $Serial.ReadByte()
    if ($etx -ne 0x55) {
        throw ('Invalid ETX: 0x{0:X2}' -f $etx)
    }
    if ((Get-Crc8 $body) -ne $receivedCrc) {
        throw 'Response CRC mismatch'
    }

    [byte[]]$payload = if ($length -gt 1) {
        $body[1..($length - 1)]
    } else {
        [byte[]]@()
    }
    return [pscustomobject]@{
        Command = $body[0]
        Payload = $payload
    }
}

function Invoke-Command(
    [System.IO.Ports.SerialPort]$Serial,
    [byte]$Command,
    [byte[]]$Payload
) {
    [byte[]]$frame = New-Frame $Command $Payload
    $Serial.Write($frame, 0, $frame.Length)
    $response = Read-Frame $Serial
    if ($response.Command -ne $Command) {
        throw (
            'Command mismatch: sent=0x{0:X2}, received=0x{1:X2}' -f
            $Command,
            $response.Command)
    }
    return $response
}

function Invoke-Result(
    [System.IO.Ports.SerialPort]$Serial,
    [byte]$Command,
    [byte[]]$Payload
) {
    $response = Invoke-Command $Serial $Command $Payload
    if ($response.Payload.Length -ne 1) {
        throw (
            'Command 0x{0:X2}: result length={1}' -f
            $Command,
            $response.Payload.Length)
    }
    return [int]$response.Payload[0]
}

function Get-State([System.IO.Ports.SerialPort]$Serial) {
    $response = Invoke-Command `
        $Serial `
        $CmdGetState `
        ([byte[]]@())
    if ($response.Payload.Length -ne 60) {
        throw (
            'GET_STATE payload length={0}' -f
            $response.Payload.Length)
    }

    $actual = New-Object double[] 6
    for ($joint = 0; $joint -lt 6; $joint++) {
        $actualUrad = [BitConverter]::ToInt32(
            $response.Payload,
            28 + 4 * $joint)
        $actual[$joint] =
            $actualUrad / 17453.0
    }
    return [pscustomobject]@{
        RunState = [BitConverter]::ToInt32(
            $response.Payload, 0)
        ActualDegrees = $actual
        EnabledMask = $response.Payload[52]
        HomedMask = $response.Payload[53]
        MovingMask = $response.Payload[54]
        FaultFlags = [BitConverter]::ToUInt32(
            $response.Payload, 56)
    }
}

function Write-BeI32([int32]$Value) {
    [byte[]]$bytes =
        [BitConverter]::GetBytes($Value)
    [Array]::Reverse($bytes)
    return $bytes
}

function Write-BeU16([uint16]$Value) {
    return [byte[]]@(
        [byte](($Value -shr 8) -band 0xFF),
        [byte]($Value -band 0xFF))
}

function New-TargetPayload([double[]]$Degrees) {
    if ($Degrees.Length -ne 6) {
        throw 'A six-joint target is required'
    }

    [byte[]]$payload = [byte[]]@()
    foreach ($degreesValue in $Degrees) {
        [int32]$urad = [int32][math]::Round(
            $degreesValue * 17453.0)
        $payload += Write-BeI32 $urad
    }
    $payload += Write-BeU16 1000
    $payload += Write-BeU16 0
    return $payload
}

function Send-Target(
    [System.IO.Ports.SerialPort]$Serial,
    [double[]]$Degrees,
    [int]$ExpectedResult,
    [string]$Label
) {
    $result = Invoke-Result `
        $Serial `
        $CmdSetJointTarget `
        (New-TargetPayload $Degrees)
    if ($result -ne $ExpectedResult) {
        throw (
            '{0}: result={1}, expected={2}' -f
            $Label,
            $result,
            $ExpectedResult)
    }
    Write-Host (
        '{0}: {1}' -f
        $Label,
        $(if ($result -eq 0) {
            'ACCEPTED'
        } else {
            'REJECTED_RANGE'
        }))
}

function Wait-Target(
    [System.IO.Ports.SerialPort]$Serial,
    [double[]]$Degrees,
    [string]$Label,
    [int]$TimeoutMs = 12000
) {
    $watch = [Diagnostics.Stopwatch]::StartNew()
    $last = $null

    while ($watch.ElapsedMilliseconds -lt $TimeoutMs) {
        $last = Get-State $Serial
        if ($last.FaultFlags -ne 0) {
            throw (
                '{0}: fault=0x{1:X8}' -f
                $Label,
                $last.FaultFlags)
        }

        $allReached = $true
        foreach ($joint in @(0, 2, 3, 4)) {
            if ([math]::Abs(
                    $last.ActualDegrees[$joint] -
                    $Degrees[$joint]) -gt
                    $PositionToleranceDegrees) {
                $allReached = $false
                break
            }
        }
        if ($allReached) {
            Write-Host (
                '{0}: REACHED actual=[J1 {1:N2}, J3 {2:N2}, J4 {3:N2}, J5 {4:N2}]' -f
                $Label,
                $last.ActualDegrees[0],
                $last.ActualDegrees[2],
                $last.ActualDegrees[3],
                $last.ActualDegrees[4])
            return
        }
        Start-Sleep -Milliseconds 100
    }

    if ($null -eq $last) {
        throw "$Label`: no state sample"
    }
    throw (
        '{0}: timeout actual=[J1 {1:N2}, J3 {2:N2}, J4 {3:N2}, J5 {4:N2}]' -f
        $Label,
        $last.ActualDegrees[0],
        $last.ActualDegrees[2],
        $last.ActualDegrees[3],
        $last.ActualDegrees[4])
}

function Move-And-Wait(
    [System.IO.Ports.SerialPort]$Serial,
    [double[]]$Degrees,
    [string]$Label,
    [int]$TimeoutMs = 12000
) {
    Send-Target `
        $Serial `
        $Degrees `
        $RobotOk `
        $Label
    Wait-Target `
        $Serial `
        $Degrees `
        $Label `
        $TimeoutMs
}

function Target(
    [double]$J1 = 0,
    [double]$J3 = 0,
    [double]$J4 = 0,
    [double]$J5 = 0
) {
    return [double[]]@(
        $J1,
        90,
        $J3,
        $J4,
        $J5,
        0)
}

$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = $ReadTimeoutMs
$serial.WriteTimeout = $ReadTimeoutMs
$completed = $false

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    $initial = Get-State $serial
    if ($initial.RunState -ne $ReadyState -or
        $initial.FaultFlags -ne 0) {
        throw (
            'Initial state={0}, fault=0x{1:X8}' -f
            $initial.RunState,
            $initial.FaultFlags)
    }

    $enableResult = Invoke-Result `
        $serial `
        $CmdEnable `
        ([byte[]]@(0x1F))
    if ($enableResult -ne $RobotOk) {
        throw "ENABLE J1..J5 result=$enableResult"
    }
    Start-Sleep -Milliseconds 250

    Write-Host '--- Rejection tests at calibrated zero ---'
    Send-Target $serial (Target -J3 -1) $RobotErrRange 'J3 below 0'
    Send-Target $serial (Target -J3 136) $RobotErrRange 'J3 above 135'
    Send-Target $serial (Target -J4 1) $RobotErrRange 'J4 blocked at J3=0'
    Send-Target $serial (Target -J5 -36) $RobotErrRange 'J5 below -35'
    Send-Target $serial (Target -J5 46) $RobotErrRange 'J5 +46 blocked at J3=0'

    Write-Host '--- J1 continuous joint ---'
    Move-And-Wait $serial (Target -J1 30) 'J1 +30'
    Move-And-Wait $serial (Target) 'J1 return 0'

    Write-Host '--- J3 clearance and J4 limits ---'
    Move-And-Wait $serial (Target -J3 20) 'J3 +20 clearance'
    Send-Target $serial (Target -J3 20 -J4 91) $RobotErrRange 'J4 above +90'
    Send-Target $serial (Target -J3 20 -J4 -91) $RobotErrRange 'J4 below -90'
    Send-Target $serial (Target -J3 20 -J5 61) $RobotErrRange 'J5 +61 blocked at J3=20'
    Move-And-Wait $serial (Target -J3 20 -J4 90) 'J4 +90' 16000
    Move-And-Wait $serial (Target -J3 20) 'J4 +90 return 0' 16000
    Move-And-Wait $serial (Target -J3 20 -J4 -90) 'J4 -90' 16000
    Move-And-Wait $serial (Target -J3 20) 'J4 -90 return 0' 16000

    if (-not $SkipJ5Motion) {
        Write-Host '--- J5 -35/+60 tier at J3=20 ---'
        Move-And-Wait $serial (Target -J3 20 -J5 -35) 'J5 -35'
        Move-And-Wait $serial (Target -J3 20) 'J5 -35 return 0'
        Move-And-Wait $serial (Target -J3 20 -J5 60) 'J5 +60'
        Move-And-Wait $serial (Target -J3 20) 'J5 +60 return 0'
    } else {
        Write-Host '--- J5 motion skipped by operator; J5 remains at 0 ---'
    }

    Write-Host '--- Reverse-transition closure ---'
    Move-And-Wait $serial (Target -J3 20 -J4 20) 'J4 +20'
    Send-Target $serial (Target) $RobotErrRange 'Concurrent J4 center/J3 lower blocked'
    Move-And-Wait $serial (Target -J3 20) 'Center J4 before lowering J3'

    Write-Host '--- J5 +135 tier at J3>45 ---'
    Move-And-Wait $serial (Target -J3 55) 'J3 +55 clearance'
    if (-not $SkipJ5Motion) {
        Move-And-Wait $serial (Target -J3 55 -J5 61) 'J5 +61 allowed'
        Send-Target $serial (Target -J3 45 -J5 61) $RobotErrRange 'J3 lower at J5 +61 blocked'
        Move-And-Wait $serial (Target -J3 55) 'J5 +61 return 0'
        Send-Target $serial (Target -J3 55 -J5 136) $RobotErrRange 'J5 above +135'
        Move-And-Wait $serial (Target -J3 55 -J5 135) 'J5 +135' 18000
        Move-And-Wait $serial (Target -J3 55) 'J5 +135 return 0' 18000
    } else {
        Write-Host '--- J5 extended motion skipped by operator ---'
    }

    Write-Host '--- J3 +135 endpoint ---'
    Move-And-Wait $serial (Target -J3 135) 'J3 +135' 18000
    Move-And-Wait $serial (Target -J3 30) 'J3 return +30' 18000

    Write-Host '--- Park away from limits for 20s auto-home test ---'
    Move-And-Wait `
        $serial `
        (Target -J1 30 -J3 30 -J4 -20 -J5 $(if ($SkipJ5Motion) { 0 } else { -5 })) `
        'AUTO_HOME_START_POSITION' `
        16000

    $completed = $true
    Write-Host 'LIVE_CONSTRAINTS=PASS'
    Write-Host 'Serial closes now. Do not send commands; auto-home timer starts from the final state sample.'
}
catch {
    Write-Host (
        'LIVE_CONSTRAINTS=FAIL error={0}' -f
        $_.Exception.Message) -ForegroundColor Red
    if ($serial.IsOpen) {
        try {
            [void](Invoke-Result `
                $serial `
                $CmdStop `
                ([byte[]]@()))
        } catch {}
    }
    throw
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}

if (-not $completed) {
    exit 1
}

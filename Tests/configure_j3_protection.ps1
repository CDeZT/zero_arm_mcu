param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [ValidateRange(1, 6)]
    [int]$MotorId = 3,
    [ValidateRange(40, 150)]
    [int]$TemperatureC = 100,
    [ValidateRange(500, 10000)]
    [int]$CurrentMa = 2000,
    [ValidateRange(50, 5000)]
    [int]$DetectionTimeMs = 100,
    [int]$ReadTimeoutMs = 4000,
    [switch]$Execute
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$CmdHello = 0x00
$CmdBenchEnable = 0x21
$CmdBenchGetProtection = 0x26
$CmdBenchSetProtection = 0x27
$J3MotorId = $MotorId
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

    [byte[]]$body = @($Command) + $Payload
    return [byte[]]@(0xAA, [byte]$body.Length) +
        $body +
        @((Get-Crc8 $body), 0x55)
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
    if ($etx -ne 0x55 -or
        (Get-Crc8 $body) -ne $receivedCrc) {
        throw 'Invalid protocol response'
    }

    [byte[]]$payload = @()
    if ($length -gt 1) {
        $payload = [byte[]]$body[1..($length - 1)]
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
    [byte[]]$frame = New-Frame $Command $Payload
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

function Read-BeU16([byte[]]$Data, [int]$Offset) {
    return [uint16](
        ([uint16]$Data[$Offset] -shl 8) -bor
        $Data[$Offset + 1])
}

function Get-J3Protection(
    [System.IO.Ports.SerialPort]$Serial
) {
    $response = Invoke-Cmd `
        $Serial `
        $CmdBenchGetProtection `
        ([byte[]]@([byte]$J3MotorId))
    if ($response.Command -ne $CmdBenchGetProtection -or
        $response.Payload.Length -ne 7) {
        $code = if ($response.Payload.Length -eq 1) {
            $response.Payload[0]
        } else {
            -1
        }
        throw "GET_PROTECTION failed: code=$code"
    }
    return [pscustomobject]@{
        MotorId = $response.Payload[0]
        TemperatureC = Read-BeU16 $response.Payload 1
        CurrentMa = Read-BeU16 $response.Payload 3
        DetectionTimeMs = Read-BeU16 $response.Payload 5
    }
}

Write-Host ("==== ZEROARM M{0} PROTECTION CONFIGURATION ====" -f $MotorId)
Write-Host ("target: temperature={0}C current={1}mA time={2}ms save=true" -f
    $TemperatureC,
    $CurrentMa,
    $DetectionTimeMs)
if (-not $Execute) {
    Write-Host 'DIAGNOSTIC ONLY: pass -Execute to write persistent motor configuration.'
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

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    $hello = Invoke-Cmd $serial $CmdHello @()
    $helloText = [Text.Encoding]::ASCII.GetString(
        $hello.Payload)
    if ($hello.Command -ne $CmdHello -or
        $helloText -ne 'ZEROARM/1.0') {
        throw "HELLO failed: $helloText"
    }

    $before = Get-J3Protection $serial
    Write-Host ("before: temperature={0}C current={1}mA time={2}ms" -f
        $before.TemperatureC,
        $before.CurrentMa,
        $before.DetectionTimeMs)

    $payload =
        [byte[]]@([byte]$J3MotorId, [byte]1) +
        (Write-BeU16 ([uint16]$TemperatureC)) +
        (Write-BeU16 ([uint16]$CurrentMa)) +
        (Write-BeU16 ([uint16]$DetectionTimeMs))
    Assert-Ok `
        (Invoke-Cmd $serial $CmdBenchSetProtection $payload) `
        $CmdBenchSetProtection `
        'SET_PROTECTION'
    Start-Sleep -Milliseconds 150

    $after = Get-J3Protection $serial
    Write-Host ("after: temperature={0}C current={1}mA time={2}ms" -f
        $after.TemperatureC,
        $after.CurrentMa,
        $after.DetectionTimeMs)

    if ($after.TemperatureC -ne $TemperatureC -or
        $after.CurrentMa -ne $CurrentMa -or
        $after.DetectionTimeMs -ne $DetectionTimeMs) {
        throw 'Protection read-back does not match target.'
    }

    foreach ($enabledId in $AllMotorIds) {
        Assert-Ok `
            (Invoke-Cmd `
                $serial `
                $CmdBenchEnable `
                ([byte[]]@([byte]$enabledId))) `
            $CmdBenchEnable `
            "ENABLE M$enabledId"
    }
    Write-Host ("RESULT=PASS persistent M{0} protection updated and verified; M1..M6 enabled." -f $MotorId) -ForegroundColor Green
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}

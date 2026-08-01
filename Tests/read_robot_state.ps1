param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [int]$ReadTimeoutMs = 2500,
    [switch]$RequireReady
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

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

[byte[]]$body = @(0x01)
[byte[]]$request = @(
    0xAA,
    0x01,
    0x01,
    (Get-Crc8 $body),
    0x55)

$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = $ReadTimeoutMs
$serial.WriteTimeout = $ReadTimeoutMs

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()
    $serial.Write($request, 0, $request.Length)

    do {
        $stx = $serial.ReadByte()
    } while ($stx -ne 0xAA)

    $length = $serial.ReadByte()
    if ($length -ne 61) {
        throw "GET_STATE response length=$length, expected=61"
    }

    [byte[]]$responseBody = New-Object byte[] $length
    for ($index = 0; $index -lt $length; $index++) {
        $responseBody[$index] =
            [byte]$serial.ReadByte()
    }
    [byte]$receivedCrc = [byte]$serial.ReadByte()
    $etx = $serial.ReadByte()

    if ($etx -ne 0x55) {
        throw ('Invalid ETX: 0x{0:X2}' -f $etx)
    }
    if ((Get-Crc8 $responseBody) -ne
        $receivedCrc) {
        throw 'GET_STATE response CRC mismatch'
    }
    if ($responseBody[0] -ne 0x01) {
        throw (
            'Unexpected response command: 0x{0:X2}' -f
            $responseBody[0])
    }

    [byte[]]$payload =
        $responseBody[1..($responseBody.Length - 1)]
    $runState = [BitConverter]::ToInt32(
        $payload, 0)
    $enabledMask = $payload[52]
    $homedMask = $payload[53]
    $movingMask = $payload[54]
    $faultFlags = [BitConverter]::ToUInt32(
        $payload, 56)

    $targetDegrees = New-Object double[] 6
    $actualDegrees = New-Object double[] 6
    for ($joint = 0; $joint -lt 6; $joint++) {
        $targetUrad = [BitConverter]::ToInt32(
            $payload,
            4 + 4 * $joint)
        $actualUrad = [BitConverter]::ToInt32(
            $payload,
            28 + 4 * $joint)
        $targetDegrees[$joint] =
            [math]::Round(
                $targetUrad / 17453.0,
                3)
        $actualDegrees[$joint] =
            [math]::Round(
                $actualUrad / 17453.0,
                3)
    }

    Write-Output (
        'state={0} enabled=0x{1:X2} homed=0x{2:X2} moving=0x{3:X2} fault=0x{4:X8}' -f
        $runState,
        $enabledMask,
        $homedMask,
        $movingMask,
        $faultFlags)
    Write-Output (
        'target_deg={0}' -f
        (($targetDegrees |
            ForEach-Object { '{0:N3}' -f $_ }) -join ','))
    Write-Output (
        'actual_deg={0}' -f
        (($actualDegrees |
            ForEach-Object { '{0:N3}' -f $_ }) -join ','))

    if ($RequireReady -and
        ($runState -ne 1 -or
         $movingMask -ne 0 -or
         $faultFlags -ne 0)) {
        throw 'Robot state is not READY and fault-free'
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}

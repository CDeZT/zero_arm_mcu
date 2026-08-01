param(
    [string]$Port = 'COM3',
    [int]$Rounds = 100,
    [int]$BaudRate = 115200,
    [int]$ReadTimeoutMs = 2000,
    [int]$InterRoundDelayMs = 5
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Safety boundary: this script only transmits CMD_HELLO (0x00) and
# CMD_GET_STATE (0x01), including malformed copies used for parser recovery.
$CmdHello = 0x00
$CmdGetState = 0x01
$RobotStateReady = 1
$RobotStatePayloadSize = 60

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

function New-ReadOnlyRequest([byte]$Command) {
    if ($Command -ne $CmdHello -and
        $Command -ne $CmdGetState) {
        throw ('Unsafe command rejected by test: 0x{0:X2}' -f $Command)
    }

    [byte[]]$body = @($Command)
    [byte]$crc = Get-Crc8 $body
    return [byte[]]@(0xAA, 0x01, $Command, $crc, 0x55)
}

function Read-ProtocolFrame(
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
    for ($index = 0; $index -lt $length; $index++) {
        $body[$index] = [byte]$Serial.ReadByte()
    }

    [byte]$receivedCrc = $Serial.ReadByte()
    $etx = $Serial.ReadByte()
    if ($etx -ne 0x55) {
        throw ('Invalid response ETX: 0x{0:X2}' -f $etx)
    }

    [byte]$calculatedCrc = Get-Crc8 $body
    if ($receivedCrc -ne $calculatedCrc) {
        throw (
            'Response CRC mismatch: received=0x{0:X2}, expected=0x{1:X2}' -f
            $receivedCrc,
            $calculatedCrc)
    }

    [byte[]]$payload = if ($length -gt 1) {
        $body[1..($length - 1)]
    } else {
        @()
    }

    return [pscustomobject]@{
        Command = $body[0]
        Payload = $payload
    }
}

function Write-Bytes(
    [System.IO.Ports.SerialPort]$Serial,
    [byte[]]$Bytes
) {
    $Serial.Write($Bytes, 0, $Bytes.Length)
}

function Assert-Hello(
    [System.IO.Ports.SerialPort]$Serial,
    [byte[]]$Request
) {
    Write-Bytes $Serial $Request
    $response = Read-ProtocolFrame $Serial
    $text = [Text.Encoding]::ASCII.GetString($response.Payload)
    if ($response.Command -ne $CmdHello -or
        $text -ne 'ZEROARM/1.0') {
        throw (
            'Unexpected HELLO response: command=0x{0:X2}, text={1}' -f
            $response.Command,
            $text)
    }
}

function Assert-State(
    [System.IO.Ports.SerialPort]$Serial,
    [byte[]]$Request
) {
    Write-Bytes $Serial $Request
    $response = Read-ProtocolFrame $Serial
    if ($response.Command -ne $CmdGetState -or
        $response.Payload.Length -ne $RobotStatePayloadSize) {
        throw (
            'Unexpected GET_STATE response: command=0x{0:X2}, length={1}' -f
            $response.Command,
            $response.Payload.Length)
    }

    $runState = [BitConverter]::ToInt32(
        $response.Payload,
        0)
    $enabledMask = $response.Payload[52]
    $movingMask = $response.Payload[54]
    $faultFlags = [BitConverter]::ToUInt32(
        $response.Payload,
        56)

    if ($runState -ne $RobotStateReady -or
        $enabledMask -ne 0 -or
        $movingMask -ne 0 -or
        $faultFlags -ne 0) {
        throw (
            'Unsafe/unexpected state: state={0}, enabled=0x{1:X2}, moving=0x{2:X2}, faults=0x{3:X8}' -f
            $runState,
            $enabledMask,
            $movingMask,
            $faultFlags)
    }
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

[byte[]]$helloRequest = New-ReadOnlyRequest $CmdHello
[byte[]]$stateRequest = New-ReadOnlyRequest $CmdGetState

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    # A bad CRC must be rejected without producing a response.
    [byte[]]$badHello = $helloRequest.Clone()
    $badHello[3] = [byte]($badHello[3] -bxor 0xFF)
    $savedTimeout = $serial.ReadTimeout
    $serial.ReadTimeout = 300
    Write-Bytes $serial $badHello
    try {
        [void](Read-ProtocolFrame $serial)
        throw 'Bad-CRC HELLO unexpectedly produced a response'
    } catch [TimeoutException] {
        Write-Output 'bad_crc_rejection=PASS'
    } finally {
        $serial.ReadTimeout = $savedTimeout
        $serial.DiscardInBuffer()
    }

    # Noise and a completed truncated frame must not prevent recovery.
    [byte[]]$noise = 0x12, 0x34, 0x56, 0x78, 0x9A
    Write-Bytes $serial $noise
    Assert-Hello $serial $helloRequest
    Write-Output 'noise_recovery=PASS'

    [byte[]]$badCompletedFrame = 0xAA, 0x01, 0x00, 0xFF, 0x55
    Write-Bytes $serial $badCompletedFrame
    Assert-State $serial $stateRequest
    Write-Output 'bad_frame_recovery=PASS'

    $timer = [Diagnostics.Stopwatch]::StartNew()
    for ($round = 1; $round -le $Rounds; $round++) {
        Assert-Hello $serial $helloRequest
        Assert-State $serial $stateRequest
        if ($InterRoundDelayMs -gt 0) {
            Start-Sleep -Milliseconds $InterRoundDelayMs
        }
    }
    $timer.Stop()

    $frames = 2 * $Rounds
    $framesPerSecond = if ($timer.Elapsed.TotalSeconds -gt 0) {
        $frames / $timer.Elapsed.TotalSeconds
    } else {
        0
    }
    Write-Output (
        'stress=PASS rounds={0} frames={1} elapsed_ms={2} frames_per_second={3:N1}' -f
        $Rounds,
        $frames,
        $timer.ElapsedMilliseconds,
        $framesPerSecond)
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}

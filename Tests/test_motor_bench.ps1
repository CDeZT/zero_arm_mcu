param(
    [string]$Port = 'COM3',
    [int]$BaudRate = 115200,
    [int]$MotorId = 1,
    [double]$Turns = 300.0,
    [double]$VelocityRpm = 1500.0,
    [int]$AccelerationRpmS = 800,
    [int]$ExtraSettleMs = 2000,
    [int]$ReadTimeoutMs = 5000,
    [switch]$ConfirmBareMotor,
    [switch]$DiagnoseOnly
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Requires firmware with CONFIG_MOTOR_BENCH_TEST=1.
$CmdHello = 0x00
$CmdGetState = 0x01
$CmdBenchQuery = 0x20
$CmdBenchEnable = 0x21
$CmdBenchDisable = 0x22
$CmdBenchStop = 0x23
$CmdBenchMoveRel = 0x24

$RobotStateReady = 1
$RobotStatePayloadSize = 60
$BenchStatePayloadSize = 40
$MaxTurns = 1500.0
$MaxVelocityRpm = 1500.0
$MaxAcceleration = 2000
$MaxMotorId = 6

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

function New-Frame([byte]$Command, [byte[]]$Payload = @()) {
    [byte[]]$body = @($Command) + $Payload
    [byte]$length = [byte]$body.Length
    [byte]$crc = Get-Crc8 $body
    return [byte[]]@(0xAA, $length) + $body + @($crc, 0x55)
}

function Write-Bytes(
    [System.IO.Ports.SerialPort]$Serial,
    [byte[]]$Bytes
) {
    $hex = ($Bytes | ForEach-Object { '{0:X2}' -f $_ }) -join ' '
    Write-Host ("TX: {0}" -f $hex)
    $Serial.Write($Bytes, 0, $Bytes.Length)
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

    [byte[]]$frameBytes = [byte[]]@(0xAA, [byte]$length) + $body + @($receivedCrc, 0x55)
    $hex = ($frameBytes | ForEach-Object { '{0:X2}' -f $_ }) -join ' '
    Write-Host ("RX: {0}" -f $hex)

    [byte[]]$payload = if ($length -gt 1) {
        $body[1..($length - 1)]
    } else {
        @()
    }

    return [pscustomobject]@{
        Command = $body[0]
        Payload = $payload
        RawHex = $hex
    }
}

function Invoke-CommandFrame(
    [System.IO.Ports.SerialPort]$Serial,
    [byte]$Command,
    [byte[]]$Payload = @()
) {
    $frame = New-Frame $Command $Payload
    Write-Bytes $Serial $frame
    return Read-ProtocolFrame $Serial
}

function Assert-ResultResponse(
    [pscustomobject]$Response,
    [byte]$Command,
    [string]$Step
) {
    if ($Response.Command -ne $Command) {
        throw (
            '{0}: unexpected command 0x{1:X2}' -f
            $Step,
            $Response.Command)
    }
    if ($Response.Payload.Length -ne 1) {
        throw (
            '{0}: expected 1-byte result, got length {1}' -f
            $Step,
            $Response.Payload.Length)
    }
    $code = $Response.Payload[0]
    if ($code -ne 0) {
        throw (
            '{0}: firmware returned ROBOT error code {1}' -f
            $Step,
            $code)
    }
}

function Get-Int32Le([byte[]]$Payload, [int]$Offset) {
    return [BitConverter]::ToInt32($Payload, $Offset)
}

function Get-UInt32Le([byte[]]$Payload, [int]$Offset) {
    return [BitConverter]::ToUInt32($Payload, $Offset)
}

function Get-UInt16Le([byte[]]$Payload, [int]$Offset) {
    return [BitConverter]::ToUInt16($Payload, $Offset)
}

function Parse-BenchState([byte[]]$Payload) {
    if ($Payload.Length -ne 28 -and $Payload.Length -ne $BenchStatePayloadSize) {
        throw (
            'Unexpected bench state length: {0} (need 28 or {1}). Flash firmware with CONFIG_MOTOR_BENCH_TEST.' -f
            $Payload.Length,
            $BenchStatePayloadSize)
    }

    $posUrad = Get-Int32Le $Payload 4
    $sample = if ($Payload.Length -ge 32) {
        Get-UInt32Le $Payload 28
    } else {
        [uint32]0
    }

    return [pscustomobject]@{
        MotorId = $Payload[0]
        Online = $Payload[1]
        PositionUrad = $posUrad
        Velocity = Get-Int32Le $Payload 8
        CurrentMa = Get-UInt16Le $Payload 12
        Status = Get-UInt16Le $Payload 14
        FaultFlags = Get-UInt32Le $Payload 16
        CanTxErrors = Get-UInt32Le $Payload 20
        FeedbackFaults = Get-UInt32Le $Payload 24
        PositionSampleCount = $sample
        TargetSubmitCount = if ($Payload.Length -ge 36) {
            Get-UInt32Le $Payload 32
        } else {
            [uint32]0
        }
        TargetSendCount = if ($Payload.Length -ge 40) {
            Get-UInt32Le $Payload 36
        } else {
            [uint32]0
        }
        PositionDegrees = [math]::Round($posUrad * 180.0 / 3141593.0, 3)
        PayloadBytes = $Payload.Length
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

function Get-MoveWaitMs(
    [double]$TurnsValue,
    [double]$VelocityValue,
    [int]$ExtraMs
) {
    if ($VelocityValue -le 0) {
        throw 'Velocity must be positive'
    }
    # one-way duration + margin + extra settle
    $seconds = ($TurnsValue / $VelocityValue) * 60.0
    $wait = [int][math]::Ceiling(($seconds * 1.35 * 1000.0) + $ExtraMs)
    if ($wait -lt 2000) {
        $wait = 2000
    }
    return $wait
}

function Invoke-SafeStopDisable(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$Id
) {
    try {
        $stop = Invoke-CommandFrame $Serial $CmdBenchStop @([byte]$Id)
        Assert-ResultResponse $stop $CmdBenchStop 'SAFE_STOP'
        Write-Output 'safe_stop=PASS'
    } catch {
        Write-Output ("safe_stop=FAIL detail={0}" -f $_.Exception.Message)
    }

    try {
        $disable = Invoke-CommandFrame $Serial $CmdBenchDisable @([byte]$Id)
        Assert-ResultResponse $disable $CmdBenchDisable 'SAFE_DISABLE'
        Write-Output 'safe_disable=PASS'
    } catch {
        Write-Output ("safe_disable=FAIL detail={0}" -f $_.Exception.Message)
    }
}

function Get-BenchState(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$Id
) {
    $query = Invoke-CommandFrame $Serial $CmdBenchQuery @([byte]$Id)
    if ($query.Payload.Length -eq 1) {
        throw (
            'BENCH_QUERY rejected with code {0}' -f
            $query.Payload[0])
    }
    return Parse-BenchState $query.Payload
}

function Get-BenchStatePolled(
    [System.IO.Ports.SerialPort]$Serial,
    [int]$Id,
    [uint32]$MinSample = 0,
    [int]$Polls = 8,
    [int]$PollDelayMs = 200
) {
    $last = $null
    for ($i = 0; $i -lt $Polls; $i++) {
        $last = Get-BenchState $Serial $Id
        if ($last.Online -ne 0 -and
            ($MinSample -eq 0 -or
             $last.PositionSampleCount -gt $MinSample -or
             $last.PayloadBytes -eq 28)) {
            # With 28-byte legacy payload, re-query a few times and accept latest.
            if ($last.PayloadBytes -eq 28 -and $i -lt 2) {
                Start-Sleep -Milliseconds $PollDelayMs
                continue
            }
            return $last
        }
        Start-Sleep -Milliseconds $PollDelayMs
    }
    if ($null -eq $last) {
        throw 'No bench state received'
    }
    return $last
}

if ($MotorId -lt 1 -or $MotorId -gt $MaxMotorId) {
    throw "MotorId must be 1..$MaxMotorId, got $MotorId"
}
if ($Turns -le 0 -or $Turns -gt $MaxTurns) {
    throw "Turns must be in (0, $MaxTurns], got $Turns"
}
if ($VelocityRpm -le 0 -or $VelocityRpm -gt $MaxVelocityRpm) {
    throw "VelocityRpm must be in (0, $MaxVelocityRpm], got $VelocityRpm"
}
if ($AccelerationRpmS -le 0 -or $AccelerationRpmS -gt $MaxAcceleration) {
    throw "AccelerationRpmS must be in (0, $MaxAcceleration], got $AccelerationRpmS"
}

$degreesTenths = [uint32][math]::Round($Turns * 3600.0)
$velocityTenths = [uint16][math]::Round($VelocityRpm * 10.0)
$acceleration = [uint16]$AccelerationRpmS
$settleMs = Get-MoveWaitMs $Turns $VelocityRpm $ExtraSettleMs
$doMove = $ConfirmBareMotor.IsPresent -and (-not $DiagnoseOnly.IsPresent)

Write-Output '==== ZEROARM single-motor bench test ===='
Write-Output ("port={0} motor_id={1} turns={2} velocity_rpm={3} settle_ms={4} move_enabled={5}" -f
    $Port,
    $MotorId,
    $Turns,
    $VelocityRpm,
    $settleMs,
    $doMove)

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

$moveStarted = $false

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    Write-Output '--- Stage A: PC <-> MCU UART ---'
    $hello = Invoke-CommandFrame $serial $CmdHello
    $helloText = [Text.Encoding]::ASCII.GetString($hello.Payload)
    if ($hello.Command -ne $CmdHello -or $helloText -ne 'ZEROARM/1.0') {
        throw (
            'HELLO failed: command=0x{0:X2} text={1}. Check USB/VCP COM port and firmware running.' -f
            $hello.Command,
            $helloText)
    }
    Write-Output ("stage_a_uart_hello=PASS text={0}" -f $helloText)

    $state = Invoke-CommandFrame $serial $CmdGetState
    if ($state.Command -ne $CmdGetState -or
        $state.Payload.Length -ne $RobotStatePayloadSize) {
        throw (
            'GET_STATE failed: command=0x{0:X2} length={1}' -f
            $state.Command,
            $state.Payload.Length)
    }

    $runState = Get-Int32Le $state.Payload 0
    $faultFlags = Get-UInt32Le $state.Payload 56
    Write-Output (
        'stage_a_get_state=PASS run_state={0} fault_flags=0x{1:X8}' -f
        $runState,
        $faultFlags)
    if ($runState -ne $RobotStateReady) {
        Write-Output 'diag=MCU_NOT_READY'
    }

    Write-Output '--- Stage B: MCU <-> CAN <-> Motor ---'
    try {
        $bench = Get-BenchStatePolled $serial $MotorId
    } catch {
        Write-Output 'diag=BENCH_QUERY_FAILED'
        Write-Output 'likely_causes=old_firmware_without_bench,wrong_com_port,mcu_hung,protocol_mismatch,can_offline'
        throw $_.Exception.Message
    }

    Write-Output (
        'stage_b_bench_query=PASS motor_id={0} online={1} pos_deg={2} velocity={3} status=0x{4:X4} fault=0x{5:X8} can_tx_errors={6} feedback_faults={7} sample={8} target_submit={9} target_send={10} bytes={11}' -f
        $bench.MotorId,
        $bench.Online,
        $bench.PositionDegrees,
        $bench.Velocity,
        $bench.Status,
        $bench.FaultFlags,
        $bench.CanTxErrors,
        $bench.FeedbackFaults,
        $bench.PositionSampleCount,
        $bench.TargetSubmitCount,
        $bench.TargetSendCount,
        $bench.PayloadBytes)

    if ($bench.Online -eq 0) {
        Write-Output 'diag=CAN_OR_MOTOR_OFFLINE'
        Write-Output 'likely_causes=wrong_motor_address,canh_canl_swap,missing_120ohm_termination,baud_not_500k,motor_power_off,no_common_gnd,can_transceiver_fault'
        throw 'Motor did not answer CAN query'
    }

    Write-Output 'connection_path=PASS uart_and_can_motor_online'

    if (-not $doMove) {
        Write-Output 'motion=SKIPPED reason=DiagnoseOnly_or_missing_-ConfirmBareMotor'
        Write-Output 'overall=PASS_DIAGNOSE_ONLY'
        return
    }

    Write-Output '--- Stage C: bare-motor multi-turn motion ---'
    Write-Output 'motion_mode=relative_to_current_position(raf=2)'

    $enable = Invoke-CommandFrame $serial $CmdBenchEnable @([byte]$MotorId)
    Assert-ResultResponse $enable $CmdBenchEnable 'ENABLE'
    Write-Output 'stage_c_enable=PASS'
    Start-Sleep -Milliseconds 500

    $before = Get-BenchStatePolled $serial $MotorId
    Write-Output (
        'stage_c_before pos_deg={0} sample={1}' -f
        $before.PositionDegrees,
        $before.PositionSampleCount)

    $forwardPayload =
        @([byte]$MotorId, [byte]0) +
        (Write-BeU32 $degreesTenths) +
        (Write-BeU16 $velocityTenths) +
        (Write-BeU16 $acceleration)
    $forward = Invoke-CommandFrame $serial $CmdBenchMoveRel $forwardPayload
    Assert-ResultResponse $forward $CmdBenchMoveRel 'MOVE_FORWARD'
    $moveStarted = $true
    Write-Output ("stage_c_move_forward=ACCEPTED turns={0} dir=CW(0) wait_ms={1}" -f $Turns, $settleMs)
    Start-Sleep -Milliseconds $settleMs

    $mid = Get-BenchStatePolled $serial $MotorId $before.PositionSampleCount
    $deltaForward = [math]::Round(
        $mid.PositionDegrees - $before.PositionDegrees,
        3)
    Write-Output (
        'stage_c_after_forward pos_deg={0} delta_deg={1} sample={2}' -f
        $mid.PositionDegrees,
        $deltaForward,
        $mid.PositionSampleCount)

    $reversePayload =
        @([byte]$MotorId, [byte]1) +
        (Write-BeU32 $degreesTenths) +
        (Write-BeU16 $velocityTenths) +
        (Write-BeU16 $acceleration)
    $reverse = Invoke-CommandFrame $serial $CmdBenchMoveRel $reversePayload
    Assert-ResultResponse $reverse $CmdBenchMoveRel 'MOVE_REVERSE'
    Write-Output ("stage_c_move_reverse=ACCEPTED turns={0} dir=CCW(1) wait_ms={1}" -f $Turns, $settleMs)
    Start-Sleep -Milliseconds $settleMs

    $after = Get-BenchStatePolled $serial $MotorId $mid.PositionSampleCount
    $deltaRoundTrip = [math]::Round(
        $after.PositionDegrees - $before.PositionDegrees,
        3)
    Write-Output (
        'stage_c_after_reverse pos_deg={0} roundtrip_delta_deg={1} sample={2}' -f
        $after.PositionDegrees,
        $deltaRoundTrip,
        $after.PositionSampleCount)

    Invoke-SafeStopDisable $serial $MotorId
    $moveStarted = $false

    $expected = $Turns * 360.0
    $minForward = [math]::Max(90.0, $expected * 0.15)
    $maxReturnError = [math]::Max(90.0, $expected * 0.15)

    if ([math]::Abs($deltaForward) -lt $minForward) {
        Write-Output 'diag=MOTION_TOO_SMALL'
        Write-Output 'likely_causes=enable_failed,drive_mode_mismatch,wrong_units,insufficient_wait,motor_not_powered_for_motion'
        throw (
            'Forward position change too small: delta={0} deg, expected around {1} deg' -f
            $deltaForward,
            $expected)
    }

    if ([math]::Abs($deltaRoundTrip) -gt $maxReturnError) {
        Write-Output 'diag=MOTION_DID_NOT_RETURN'
        Write-Output 'likely_causes=direction_mapping,missed_reverse_command,insufficient_wait,motor_fault_midway'
        throw (
            'Round-trip did not return near start: residual={0} deg, allowed={1} deg' -f
            $deltaRoundTrip,
            $maxReturnError)
    }

    Write-Output 'motor_motion=PASS'
    Write-Output 'overall=PASS'
    Write-Output ("summary motor_id={0} forward_delta_deg={1} residual_deg={2}" -f
        $MotorId,
        $deltaForward,
        $deltaRoundTrip)
} catch {
    Write-Output ("overall=FAIL error={0}" -f $_.Exception.Message)
    if ($serial.IsOpen -and $moveStarted) {
        Invoke-SafeStopDisable $serial $MotorId
    }
    throw
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}

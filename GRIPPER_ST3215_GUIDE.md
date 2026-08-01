# ST-3215-C001 gripper bring-up

## Electrical connection

The ST-3215-C001 is an STS-series TTL single-bus servo. Its factory serial
settings are 1,000,000 baud, 8 data bits, no parity, 1 stop bit, and ID 1.

Use two UARTs:

- USART1 stays at 115,200 8N1 and connects the MCU to the PC/serial assistant.
- USART2 is configured as 1,000,000 8N1 half-duplex single-wire mode.
- USART2_TX is PA2. On the supplied STM32G474VET6 V1.1 schematic, PA2 is J2
  pin 40.

With all power switched off:

1. Connect PA2 to the power/control board's logic `BUS`.
2. Connect the STM32 board ground, power/control board ground, and servo ground.
3. Supply the servo `VCC` from a regulated 7.4 V source capable of handling the
   servo's startup and stall current. Never power the servo from the STM32
   board's 3.3 V or 5 V rail.
4. Connect the servo to the second `BUS/VCC/GND` group if those two groups are
   parallel daisy-chain connectors.

Before powering it, verify that the control board does not route 7.4 V onto the
`BUS` pin and that its bus interface accepts 3.3 V MCU logic. A board with two
three-pin groups often just provides two parallel daisy-chain ports; that does
not by itself prove that it contains level shifting or contention protection.

## STM32CubeMX settings

In Pinout & Configuration:

1. Select `Connectivity > USART2`.
2. Set Mode to `Half-Duplex (Single Wire)`.
3. Assign `USART2_TX` to `PA2`.
4. Set baud rate to `1000000`.
5. Set word length `8 Bits`, parity `None`, stop bits `1`.
6. Set hardware flow control `None`, oversampling `16`, FIFO mode disabled.
7. Do not enable USART2 DMA or interrupts for this first blocking bring-up.
8. In the PA2 GPIO settings use alternate function `AF7`, push-pull, no
   internal pull, and very-high speed.

Keep USART1 and its RX/TX DMA configuration unchanged. Generate code and confirm
that `MX_USART2_UART_Init()` calls `HAL_HalfDuplex_Init(&huart2)`.

## MCU host protocol

The existing host frame is:

`AA LEN CMD PAYLOAD CRC8 55`

`LEN` counts `CMD + PAYLOAD`. CRC8 is calculated over `CMD + PAYLOAD`.
The serial assistant must send and display hexadecimal bytes without appending
CR/LF.

Gripper commands:

| CMD | Request payload | Response payload |
|---|---|---|
| `30` PING | `ID` | `RESULT ID SERVO_ERROR` |
| `31` READ | `ID ADDRESS LENGTH` | `RESULT ID SERVO_ERROR DATA...` |
| `32` WRITE | `ID ADDRESS DATA...` | `RESULT ID SERVO_ERROR` |
| `33` MOVE | `ID POS_BE SPEED_BE ACCEL` | `RESULT ID SERVO_ERROR` |
| `34` TORQUE | `ID MODE` | `RESULT ID SERVO_ERROR` |

`MODE` is 0 for torque off, 1 for torque on, and 2 for damping. `POSITION` is
limited to 0..4095. Servo register words are little-endian internally; the
firmware converts the MOVE command's big-endian host values.

Bring-up order:

1. Send HELLO and expect `ZEROARM/1.0`.
2. Send PING to ID 1.
3. Read address `0x38`, length 15, and check voltage, temperature, position, and
   status.
4. With the gripper mechanically unloaded, enable torque.
5. Move only a small distance around the observed position.
6. Determine safe open/closed positions before using the full range.

## Interactive script

Install pyserial and start the console:

```powershell
python -m pip install pyserial
python Tools\gripper_console.py --port COM7
```

Useful commands:

```text
hello
ping 1
status 1
torque 1 on
move 1 2100 300 20
watch 1 10
torque 1 off
```

Every request and response is printed as a hexadecimal frame, so the same bytes
can be copied into a conventional serial assistant.

The existing `gripper_u16` field in `SET_JOINT_TARGET` is reserved and decoded,
but this bring-up firmware deliberately does not make that field move the servo.
It should be connected only after the safe open/closed positions and coordinated
motion semantics have been calibrated.

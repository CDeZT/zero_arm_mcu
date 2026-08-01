#!/usr/bin/env python3
"""Interactive PC console for the ZeroArm MCU -> Feetech STS gripper bridge."""

from __future__ import annotations

import argparse
import shlex
import sys
import time
from dataclasses import dataclass

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: run `python -m pip install pyserial`."
    ) from exc


STX = 0xAA
ETX = 0x55

CMD_HELLO = 0x00
CMD_GRIPPER_PING = 0x30
CMD_GRIPPER_READ = 0x31
CMD_GRIPPER_WRITE = 0x32
CMD_GRIPPER_MOVE = 0x33
CMD_GRIPPER_TORQUE = 0x34

RESULT_NAMES = {
    0: "OK",
    1: "ARGUMENT",
    2: "NOT_INITIALIZED",
    3: "TX_ERROR",
    4: "RX_TIMEOUT",
    5: "BAD_PACKET",
    6: "SERVO_ERROR",
}


def crc8(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if (crc & 1) else (crc >> 1)
    return crc


def make_frame(command: int, payload: bytes = b"") -> bytes:
    if not 0 <= command <= 0xFF:
        raise ValueError("command must fit in one byte")
    body = bytes((command,)) + payload
    if len(body) >= 128:
        raise ValueError("frame body is too long")
    return bytes((STX, len(body))) + body + bytes((crc8(body), ETX))


def hex_line(data: bytes) -> str:
    return " ".join(f"{value:02X}" for value in data)


def parse_number(text: str) -> int:
    return int(text, 0)


def parse_data_byte(text: str) -> int:
    value = int(text, 16)
    if not 0 <= value <= 0xFF:
        raise ValueError(f"byte out of range: {text}")
    return value


@dataclass
class HostFrame:
    command: int
    payload: bytes
    raw: bytes


class GripperConsole:
    def __init__(self, port: str, baud: int, timeout: float) -> None:
        self.serial = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=timeout,
            dsrdtr=False,
            rtscts=False,
        )
        self.response_timeout = timeout

    def close(self) -> None:
        self.serial.close()

    def _read_exact(self, length: int, deadline: float) -> bytes | None:
        result = bytearray()
        while len(result) < length and time.monotonic() < deadline:
            chunk = self.serial.read(length - len(result))
            if chunk:
                result.extend(chunk)
        return bytes(result) if len(result) == length else None

    def read_frame(self, timeout: float | None = None) -> HostFrame | None:
        deadline = time.monotonic() + (
            self.response_timeout if timeout is None else timeout
        )

        while time.monotonic() < deadline:
            start = self.serial.read(1)
            if not start or start[0] != STX:
                continue

            length_raw = self._read_exact(1, deadline)
            if length_raw is None:
                return None
            length = length_raw[0]
            if length == 0 or length >= 128:
                continue

            body = self._read_exact(length, deadline)
            trailer = self._read_exact(2, deadline)
            if body is None or trailer is None:
                return None

            raw = start + length_raw + body + trailer
            if trailer[1] != ETX or trailer[0] != crc8(body):
                print(f"RX invalid: {hex_line(raw)}")
                continue

            return HostFrame(body[0], body[1:], raw)

        return None

    def request(self, command: int, payload: bytes = b"") -> HostFrame | None:
        frame = make_frame(command, payload)
        self.serial.reset_input_buffer()
        self.serial.write(frame)
        self.serial.flush()
        print(f"TX: {hex_line(frame)}")

        deadline = time.monotonic() + self.response_timeout
        while time.monotonic() < deadline:
            response = self.read_frame(deadline - time.monotonic())
            if response is None:
                break
            print(f"RX: {hex_line(response.raw)}")
            if response.command == command:
                return response

        print("RX: timeout")
        return None


def show_gripper_result(frame: HostFrame | None) -> bytes | None:
    if frame is None or len(frame.payload) < 3:
        return None
    result, servo_id, servo_error = frame.payload[:3]
    name = RESULT_NAMES.get(result, f"UNKNOWN_{result}")
    print(
        f"result={name}, id={servo_id}, "
        f"servo_error=0x{servo_error:02X}"
    )
    return frame.payload[3:] if result == 0 else None


def decode_sign_bit(raw: int, sign_bit: int) -> int:
    magnitude_mask = sign_bit - 1
    magnitude = raw & magnitude_mask
    return -magnitude if raw & sign_bit else magnitude


def show_status(data: bytes | None) -> None:
    if data is None:
        return
    if len(data) != 15:
        print(f"unexpected status length: {len(data)}")
        return

    u16le = lambda offset: data[offset] | (data[offset + 1] << 8)
    position = decode_sign_bit(u16le(0), 0x8000)
    speed = decode_sign_bit(u16le(2), 0x8000)
    load = decode_sign_bit(u16le(4), 0x0400)
    target = decode_sign_bit(u16le(11), 0x8000)
    current_raw = u16le(13)

    print(
        f"position={position} ({position * 0.087:.2f} deg), "
        f"target={target}, speed={speed} raw, load={load / 10:.1f}%, "
        f"voltage={data[6] / 10:.1f} V, temp={data[7]} C, "
        f"status=0x{data[9]:02X}, moving={data[10]}, "
        f"current={current_raw * 6.5:.1f} mA"
    )


HELP = """Commands:
  hello
  ping [id]
  status [id]
  watch [id] [hz]
  torque [id] on|off|damping
  move [id] <position 0..4095> [speed] [acceleration]
  read [id] <address> <length>
  write [id] <address> <hex-byte> [hex-byte ...]
  frame <command> [hex-byte ...]   print a frame without sending it
  help
  quit

Numbers use decimal by default; addresses accept forms such as 0x38.
Data bytes after `write` and `frame` are hexadecimal (for example: 01 FF 2A).
"""


def run_command(console: GripperConsole, words: list[str]) -> bool:
    if not words:
        return True
    command = words[0].lower()

    if command in {"quit", "exit", "q"}:
        return False
    if command in {"help", "?"}:
        print(HELP)
        return True
    if command == "hello":
        response = console.request(CMD_HELLO)
        if response is not None:
            print(response.payload.decode("ascii", errors="replace"))
        return True
    if command == "ping":
        servo_id = parse_number(words[1]) if len(words) > 1 else 1
        show_gripper_result(
            console.request(CMD_GRIPPER_PING, bytes((servo_id,)))
        )
        return True
    if command == "status":
        servo_id = parse_number(words[1]) if len(words) > 1 else 1
        response = console.request(
            CMD_GRIPPER_READ, bytes((servo_id, 0x38, 15))
        )
        show_status(show_gripper_result(response))
        return True
    if command == "watch":
        servo_id = parse_number(words[1]) if len(words) > 1 else 1
        hz = float(words[2]) if len(words) > 2 else 10.0
        if not 0.2 <= hz <= 50.0:
            raise ValueError("watch rate must be between 0.2 and 50 Hz")
        print("Polling status; press Ctrl+C to return to the prompt.")
        try:
            while True:
                response = console.request(
                    CMD_GRIPPER_READ, bytes((servo_id, 0x38, 15))
                )
                show_status(show_gripper_result(response))
                time.sleep(1.0 / hz)
        except KeyboardInterrupt:
            print()
        return True
    if command == "torque":
        if len(words) == 2:
            servo_id, mode_text = 1, words[1].lower()
        elif len(words) == 3:
            servo_id, mode_text = parse_number(words[1]), words[2].lower()
        else:
            raise ValueError("usage: torque [id] on|off|damping")
        modes = {"off": 0, "on": 1, "damping": 2}
        if mode_text not in modes:
            raise ValueError("torque mode must be on, off, or damping")
        show_gripper_result(
            console.request(
                CMD_GRIPPER_TORQUE,
                bytes((servo_id, modes[mode_text])),
            )
        )
        return True
    if command == "move":
        if len(words) < 2:
            raise ValueError(
                "usage: move [id] <position> [speed] [acceleration]"
            )
        if len(words) == 2:
            servo_id, offset = 1, 1
        else:
            servo_id, offset = parse_number(words[1]), 2
        if len(words) <= offset:
            raise ValueError("missing position")
        position = parse_number(words[offset])
        speed = parse_number(words[offset + 1]) if len(words) > offset + 1 else 300
        acceleration = (
            parse_number(words[offset + 2]) if len(words) > offset + 2 else 20
        )
        if not 0 <= position <= 4095:
            raise ValueError("position must be in 0..4095")
        if not 0 <= speed <= 0x7FFF:
            raise ValueError("speed must be in 0..32767")
        if not 0 <= acceleration <= 254:
            raise ValueError("acceleration must be in 0..254")
        payload = bytes(
            (
                servo_id,
                position >> 8,
                position & 0xFF,
                speed >> 8,
                speed & 0xFF,
                acceleration,
            )
        )
        show_gripper_result(console.request(CMD_GRIPPER_MOVE, payload))
        return True
    if command == "read":
        if len(words) == 3:
            servo_id, offset = 1, 1
        elif len(words) == 4:
            servo_id, offset = parse_number(words[1]), 2
        else:
            raise ValueError("usage: read [id] <address> <length>")
        address = parse_number(words[offset])
        length = parse_number(words[offset + 1])
        if not 0 <= address <= 0xFF or not 1 <= length <= 32:
            raise ValueError("address must fit a byte and length must be 1..32")
        data = show_gripper_result(
            console.request(
                CMD_GRIPPER_READ, bytes((servo_id, address, length))
            )
        )
        if data is not None:
            print(f"data: {hex_line(data)}")
        return True
    if command == "write":
        if len(words) < 3:
            raise ValueError(
                "usage: write [id] <address> <hex-byte> [hex-byte ...]"
            )
        if words[1].lower().startswith("0x"):
            servo_id, offset = 1, 1
        else:
            servo_id, offset = parse_number(words[1]), 2
        address = parse_number(words[offset])
        values = bytes(parse_data_byte(value) for value in words[offset + 1 :])
        if not values:
            raise ValueError("at least one data byte is required")
        show_gripper_result(
            console.request(
                CMD_GRIPPER_WRITE,
                bytes((servo_id, address)) + values,
            )
        )
        return True
    if command == "frame":
        if len(words) < 2:
            raise ValueError("usage: frame <command> [hex-byte ...]")
        host_command = parse_number(words[1])
        payload = bytes(parse_data_byte(value) for value in words[2:])
        print(hex_line(make_frame(host_command, payload)))
        return True

    raise ValueError(f"unknown command: {command}")


def choose_port(explicit_port: str | None) -> str:
    if explicit_port:
        return explicit_port
    ports = list(list_ports.comports())
    if len(ports) == 1:
        print(f"Using {ports[0].device}: {ports[0].description}")
        return ports[0].device
    if not ports:
        raise SystemExit("No serial ports found. Pass one explicitly with --port.")

    print("Available ports:")
    for item in ports:
        print(f"  {item.device}: {item.description}")
    raise SystemExit("More than one serial port found. Select one with --port.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-p", "--port", help="MCU host serial port, for example COM7")
    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=115200,
        help="MCU host baud rate (default: 115200)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="host response timeout in seconds (default: 0.5)",
    )
    parser.add_argument(
        "-c",
        "--command",
        help='run one command and exit, for example "ping 1"',
    )
    args = parser.parse_args()

    port = choose_port(args.port)
    console = GripperConsole(port, args.baud, args.timeout)
    print(f"Connected to {port} at {args.baud} 8N1.")
    try:
        if args.command:
            run_command(console, shlex.split(args.command))
            return 0

        print(HELP)
        while True:
            try:
                words = shlex.split(input("gripper> "))
                if not run_command(console, words):
                    break
            except (ValueError, IndexError) as exc:
                print(f"error: {exc}")
            except EOFError:
                break
    finally:
        console.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

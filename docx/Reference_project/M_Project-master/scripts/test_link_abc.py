#!/usr/bin/env python3
"""
Complete end-to-end test: Link A (homing) + Link B (command ack) + Link C (action done)
Captures 30+ lines of serial log for each link as evidence.
"""
import serial, time, struct, subprocess, os

PROJ = r'C:\Users\ASUS\Desktop\danpianji\M_Project'
os.chdir(PROJ)
OPENOCD = r'C:\Tools\OpenOCD-20260121-0.12.0\bin\openocd.exe'

CRC8_TABLE = [0x00,0x5E,0xBC,0xE2,0x61,0x3F,0xDD,0x83,0xC2,0x9C,0x7E,0x20,0xA3,0xFD,0x1F,0x41,0x9D,0xC3,0x21,0x7F,0xFC,0xA2,0x40,0x1E,0x5F,0x01,0xE3,0xBD,0x3E,0x60,0x82,0xDC,0x23,0x7D,0x9F,0xC1,0x42,0x1C,0xFE,0xA0,0xE1,0xBF,0x5D,0x03,0x80,0xDE,0x3C,0x62,0xBE,0xE0,0x02,0x5C,0xDF,0x81,0x63,0x3D,0x7C,0x22,0xC0,0x9E,0x1D,0x43,0xA1,0xFF,0x46,0x18,0xFA,0xA4,0x27,0x79,0x9B,0xC5,0x84,0xDA,0x38,0x66,0xE5,0xBB,0x59,0x07,0xDB,0x85,0x67,0x39,0xBA,0xE4,0x06,0x58,0x19,0x47,0xA5,0xFB,0x78,0x26,0xC4,0x9A,0x65,0x3B,0xD9,0x87,0x04,0x5A,0xB8,0xE6,0xA7,0xF9,0x1B,0x45,0xC6,0x98,0x7A,0x24,0xF8,0xA6,0x44,0x1A,0x99,0xC7,0x25,0x7B,0x3A,0x64,0x86,0xD8,0x5B,0x05,0xE7,0xB9,0x8C,0xD2,0x30,0x6E,0xED,0xB3,0x51,0x0F,0x4E,0x10,0xF2,0xAC,0x2F,0x71,0x93,0xCD,0x11,0x4F,0xAD,0xF3,0x70,0x2E,0xCC,0x92,0xD3,0x8D,0x6F,0x31,0xB2,0xEC,0x0E,0x50,0xAF,0xF1,0x13,0x4D,0xCE,0x90,0x72,0x2C,0x6D,0x33,0xD1,0x8F,0x0C,0x52,0xB0,0xEE,0x32,0x6C,0x8E,0xD0,0x53,0x0D,0xEF,0xB1,0xF0,0xAE,0x4C,0x12,0x91,0xCF,0x2D,0x73,0xCA,0x94,0x76,0x28,0xAB,0xF5,0x17,0x49,0x08,0x56,0xB4,0xEA,0x69,0x37,0xD5,0x8B,0x57,0x09,0xEB,0xB5,0x36,0x68,0x8A,0xD4,0x95,0xCB,0x29,0x77,0xF4,0xAA,0x48,0x16,0xE9,0xB7,0x55,0x0B,0x88,0xD6,0x34,0x6A,0x2B,0x75,0x97,0xC9,0x4A,0x14,0xF6,0xA8,0x74,0x2A,0xC8,0x96,0x15,0x4B,0xA9,0xF7,0xB6,0xE8,0x0A,0x54,0xD7,0x89,0x6B,0x35]

EVT = {0x80:'ACK_RECV',0x81:'ACT_DONE',0x82:'MOT_ERR',0x83:'MOT_DATA',
       0x84:'HOME_START',0x85:'HOME_DONE',0x86:'HOME_FAIL',0x87:'ESTOP',0x88:'TIMEOUT'}

def crc8(d):
    if len(d)==0: return 0
    c=d[0]
    for i in range(1,len(d)): c=CRC8_TABLE[c^d[i]]
    return c

def build_frame(cmd, data=b''):
    p = bytes([cmd]) + data
    c = crc8(p)
    return bytes([0xAA, len(p)]) + p + bytes([c, 0x55])

def parse_all(raw):
    """Parse both text lines and protocol frames from raw serial data"""
    lines = []
    frames = []
    
    # Extract text lines
    text = raw.decode('ascii', errors='ignore')
    for line in text.replace('\r\n', '\n').split('\n'):
        line = line.strip()
        if line and '[' in line or '=' in line or 'M_Project' in line:
            lines.append(line)
    
    # Extract protocol frames
    i = 0
    while i < len(raw) - 4:
        if raw[i] == 0xAA:
            plen = raw[i+1]
            end = i + 2 + plen + 2
            if end <= len(raw) and raw[end-1] == 0x55:
                chunk = raw[i:end]
                cmd = raw[i+2]
                if cmd == 0x00 and plen >= 4:
                    evt = raw[i+3]; func = raw[i+4]; st = raw[i+5]
                    extra = raw[i+6:end-2] if plen > 4 else b''
                    frames.append({
                        'type': 'EVT',
                        'evt': EVT.get(evt, f'0x{evt:02X}'),
                        'func': func,
                        'status': st,
                        'extra': extra,
                        'raw': chunk
                    })
                else:
                    frames.append({
                        'type': 'RSP',
                        'cmd': cmd,
                        'raw': chunk
                    })
                i = end
                continue
        i += 1
    return lines, frames

# ============================================================
# MAIN TEST
# ============================================================
out = open(os.path.join(PROJ, 'full_test_log.txt'), 'w', encoding='utf-8')
out.write("=" * 70 + "\n")
out.write("  M_Project - Three Link End-to-End Verification\n")
out.write(f"  Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
out.write("=" * 70 + "\n\n")

# Open serial
ser = serial.Serial('COM8', 115200, timeout=0.5)
ser.reset_input_buffer()

# Reset MCU
out.write(">>> Resetting MCU via OpenOCD...\n")
subprocess.run([OPENOCD, '-f', 'interface/cmsis-dap.cfg', '-f', 'target/stm32g4x.cfg',
                '-c', 'init; reset run; shutdown'], capture_output=True, timeout=10)
out.write(">>> MCU reset done\n\n")

# ============================================================
# LINK A: Wait for homing to complete (up to 10s)
# ============================================================
out.write("=" * 70 + "\n")
out.write("  LINK A: Power-on Auto Homing\n")
out.write("=" * 70 + "\n\n")

all_data = b''
start = time.time()
home_done = False
while time.time() - start < 10.0:
    chunk = ser.read(1024)
    if chunk:
        all_data += chunk
        # Check if HOME_DONE received
        if b'\x00\x85' in chunk:
            time.sleep(0.5)
            all_data += ser.read(4096)
            home_done = True
            break

lines_a, frames_a = parse_all(all_data)
out.write("--- Text Output (debug prints) ---\n")
for i, line in enumerate(lines_a):
    out.write(f"  {i+1:3d}| {line}\n")

out.write(f"\n--- Protocol Frames ({len(frames_a)} total) ---\n")
for i, f in enumerate(frames_a):
    if f['type'] == 'EVT':
        extra_hex = f' data={f["extra"].hex(" ")}' if f['extra'] else ''
        out.write(f"  {i+1:3d}| EVT: {f['evt']:12s} func=0x{f['func']:02X} st=0x{f['status']:02X}{extra_hex}  raw={f['raw'].hex(' ')}\n")
    else:
        out.write(f"  {i+1:3d}| RSP: cmd=0x{f['cmd']:02X}  raw={f['raw'].hex(' ')}\n")

out.write(f"\n--- Link A Result: {'PASS' if home_done else 'FAIL'} ---\n")
out.write(f"    HOME_START received: {any(f['type']=='EVT' and f['evt']=='HOME_START' for f in frames_a)}\n")
out.write(f"    ACK_RECV (config):   {sum(1 for f in frames_a if f['type']=='EVT' and f['evt']=='ACK_RECV')}\n")
out.write(f"    MOT_DATA (periodic): {sum(1 for f in frames_a if f['type']=='EVT' and f['evt']=='MOT_DATA')}\n")
out.write(f"    HOME_DONE received:  {any(f['type']=='EVT' and f['evt']=='HOME_DONE' for f in frames_a)}\n")
out.write(f"    ACT_DONE (9F):       {any(f['type']=='EVT' and f['evt']=='ACT_DONE' for f in frames_a)}\n")

# ============================================================
# LINK B + C: Send move command
# ============================================================
out.write("\n\n" + "=" * 70 + "\n")
out.write("  LINK B+C: Command Ack + Action Done\n")
out.write("=" * 70 + "\n\n")

time.sleep(0.5)
ser.reset_input_buffer()

# Send CMD_MOVE_ABS: 90 degrees, 100 RPM, acc=200
pos_data = struct.pack('>i', 900) + struct.pack('>H', 1000) + struct.pack('>H', 200)
frame = build_frame(0x02, pos_data)
out.write(f">>> Sending CMD_MOVE_ABS: pos=90.0deg vel=100RPM acc=200\n")
out.write(f"    Frame: {frame.hex(' ')}\n\n")
ser.write(frame)

# Collect for 15 seconds or until ACT_DONE
all_data_bc = b''
start = time.time()
act_done = False
while time.time() - start < 15.0:
    chunk = ser.read(1024)
    if chunk:
        all_data_bc += chunk
        if b'\x00\x81' in chunk:
            time.sleep(0.3)
            all_data_bc += ser.read(4096)
            act_done = True
            break

lines_bc, frames_bc = parse_all(all_data_bc)
out.write("--- Text Output (debug prints) ---\n")
for i, line in enumerate(lines_bc):
    out.write(f"  {i+1:3d}| {line}\n")

out.write(f"\n--- Protocol Frames ({len(frames_bc)} total) ---\n")
for i, f in enumerate(frames_bc):
    if f['type'] == 'EVT':
        extra_hex = f' data={f["extra"].hex(" ")}' if f['extra'] else ''
        out.write(f"  {i+1:3d}| EVT: {f['evt']:12s} func=0x{f['func']:02X} st=0x{f['status']:02X}{extra_hex}  raw={f['raw'].hex(' ')}\n")
    else:
        out.write(f"  {i+1:3d}| RSP: cmd=0x{f['cmd']:02X}  raw={f['raw'].hex(' ')}\n")

has_ack = any(f['type']=='EVT' and f['evt']=='ACK_RECV' for f in frames_bc)
has_done = any(f['type']=='EVT' and f['evt']=='ACT_DONE' for f in frames_bc)
has_data = any(f['type']=='EVT' and f['evt']=='MOT_DATA' for f in frames_bc)

out.write(f"\n--- Link B Result: {'PASS' if has_ack else 'FAIL'} ---\n")
out.write(f"    ACK_RECV (02 confirm): {has_ack}\n")
out.write(f"--- Link C Result: {'PASS' if has_done else 'FAIL'} ---\n")
out.write(f"    ACT_DONE (9F complete): {has_done}\n")
out.write(f"    MOT_DATA (periodic):    {has_data} ({sum(1 for f in frames_bc if f['type']=='EVT' and f['evt']=='MOT_DATA')} frames)\n")

# ============================================================
# FINAL SUMMARY
# ============================================================
out.write("\n\n" + "=" * 70 + "\n")
out.write("  FINAL SUMMARY\n")
out.write("=" * 70 + "\n\n")
out.write(f"  Link A (Power-on Homing):     {'PASS' if home_done else 'FAIL'}\n")
out.write(f"  Link B (Command Ack):         {'PASS' if has_ack else 'FAIL'}\n")
out.write(f"  Link C (Action Done):         {'PASS' if has_done else 'FAIL'}\n")
out.write(f"  Data Relay (Periodic Report): {'PASS' if has_data else 'FAIL'}\n")
out.write(f"\n  Total Link A frames: {len(frames_a)}\n")
out.write(f"  Total Link B+C frames: {len(frames_bc)}\n")
all_pass = home_done and has_ack and has_done
out.write(f"\n  ALL THREE LINKS: {'*** PASS ***' if all_pass else '*** FAIL ***'}\n")

out.close()
ser.close()
print(f"Test complete. All pass: {all_pass}")
print(f"Full log: {os.path.join(PROJ, 'full_test_log.txt')}")

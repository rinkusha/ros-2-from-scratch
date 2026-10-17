#!/usr/bin/env python3
"""Bring-up console for the ESP32 joint controller. No ROS needed.

Talks the same COBS/CRC16 protocol as arm_common/protocol.hpp, so you can
check wiring, encoder direction and a first set of gains from any machine
with pyserial, before the ROS 2 bridge is involved.

    pip install pyserial
    python3 esp32_console.py /dev/ttyUSB0            # stream state at 5 Hz
    python3 esp32_console.py /dev/ttyUSB0 ping
    python3 esp32_console.py /dev/ttyUSB0 zero
    python3 esp32_console.py /dev/ttyUSB0 gains 0 --kp 0.8 --kd 0.02 --out-limit 0.5
    python3 esp32_console.py /dev/ttyUSB0 enable
    python3 esp32_console.py /dev/ttyUSB0 step 0 0.3    # move joint 0 to 0.3 rad, hold 3 s
    python3 esp32_console.py /dev/ttyUSB0 disable

Keep the struct layouts below in sync with protocol.hpp (kMaxJoints = 6).
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("pyserial is required: pip install pyserial")

PROTOCOL_VERSION = 1
MAX_JOINTS = 6

MSG_SETPOINT, MSG_GAINS, MSG_COMMAND = 0x01, 0x02, 0x03
MSG_STATE, MSG_ACK, MSG_LOG = 0x81, 0x82, 0x83
CMD = {"ping": 0, "enable": 1, "disable": 2, "zero": 3, "clear_fault": 4}
STATUS = {0: "disabled", 1: "enabled", 2: "hold", 3: "FAULT"}
ACK = {0: "ok", 1: "bad joint", 2: "bad value", 3: "refused", 4: "unknown"}
ALL_JOINTS = 0xFF

HDR = struct.Struct("<BBH")
SETPOINT = struct.Struct("<BBHB" + "f" * MAX_JOINTS * 3)
GAINS = struct.Struct("<BBHB9f")
COMMAND = struct.Struct("<BBHBB")
STATE = struct.Struct("<BBHIHHHB" + "B" * MAX_JOINTS + "B" * MAX_JOINTS + "f" * MAX_JOINTS * 3)
ACKMSG = struct.Struct("<BBHBHB")


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    idx = 0
    while idx <= len(data):
        nxt = data.find(b"\x00", idx)
        if nxt < 0:
            nxt = len(data)
        chunk = data[idx:nxt]
        while len(chunk) >= 254:
            out.append(0xFF)
            out += chunk[:254]
            chunk = chunk[254:]
        out.append(len(chunk) + 1)
        out += chunk
        idx = nxt + 1
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    idx = 0
    while idx < len(data):
        code = data[idx]
        if code == 0:
            raise ValueError("zero inside frame")
        idx += 1
        out += data[idx : idx + code - 1]
        idx += code - 1
        if code != 0xFF and idx < len(data):
            out.append(0)
    return bytes(out)


def frame(payload: bytes) -> bytes:
    c = crc16(payload)
    return cobs_encode(payload + struct.pack("<H", c)) + b"\x00"


def unframe(body: bytes):
    raw = cobs_decode(body)
    if len(raw) < 3:
        return None
    payload, (c,) = raw[:-2], struct.unpack("<H", raw[-2:])
    return payload if crc16(payload) == c else None


class Link:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.seq = 0
        self.buf = bytearray()

    def next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFFFF
        return self.seq

    def send(self, payload: bytes):
        self.ser.write(frame(payload))

    def poll(self):
        """Yields decoded payloads that arrived since the last call."""
        self.buf += self.ser.read(4096)
        while True:
            end = self.buf.find(b"\x00")
            if end < 0:
                return
            body, self.buf = bytes(self.buf[:end]), self.buf[end + 1 :]
            if body:
                payload = unframe(body)
                if payload is not None:
                    yield payload

    def wait_ack(self, seq: int, timeout: float = 0.5) -> str:
        deadline = time.time() + timeout
        while time.time() < deadline:
            for p in self.poll():
                if len(p) == ACKMSG.size and p[1] == MSG_ACK:
                    _, _, _, _, acked_seq, result = ACKMSG.unpack(p)
                    if acked_seq == seq:
                        return ACK.get(result, str(result))
                elif p[1] == MSG_LOG:
                    print_log(p)
        return "no ack"

    def command(self, name: str, joint: int = ALL_JOINTS) -> str:
        seq = self.next_seq()
        self.send(COMMAND.pack(PROTOCOL_VERSION, MSG_COMMAND, seq, CMD[name], joint))
        return self.wait_ack(seq)

    def gains(self, joint: int, kp, ki, kd, kff_v, i_limit, out_limit, vel_filter_hz, pos_min, pos_max) -> str:
        seq = self.next_seq()
        self.send(GAINS.pack(PROTOCOL_VERSION, MSG_GAINS, seq, joint, kp, ki, kd, kff_v, i_limit, out_limit,
                             vel_filter_hz, pos_min, pos_max))
        return self.wait_ack(seq)

    def setpoint(self, positions, velocities=None):
        n = len(positions)
        pos = list(positions) + [0.0] * (MAX_JOINTS - n)
        vel = list(velocities or [0.0] * n) + [0.0] * (MAX_JOINTS - n)
        self.send(SETPOINT.pack(PROTOCOL_VERSION, MSG_SETPOINT, self.next_seq(), n, *pos, *vel, *([0.0] * MAX_JOINTS)))


def print_log(p: bytes):
    level = {0: "E", 1: "W"}.get(p[4], "I")
    print(f"  [fw {level}] {p[5:].decode(errors='replace')}")


def print_state(p: bytes):
    f = STATE.unpack(p)
    _, _, seq, t_us, sp_seq, sp_age, loop_max, n = f[:8]
    status = f[8 : 8 + MAX_JOINTS]
    fault = f[8 + MAX_JOINTS : 8 + 2 * MAX_JOINTS]
    pos = f[8 + 2 * MAX_JOINTS : 8 + 3 * MAX_JOINTS]
    vel = f[8 + 3 * MAX_JOINTS : 8 + 4 * MAX_JOINTS]
    eff = f[8 + 4 * MAX_JOINTS : 8 + 5 * MAX_JOINTS]
    print(f"t={t_us/1e6:9.3f}s seq={seq:5d} sp_age={sp_age:4d}ms loop_max={loop_max:5d}us")
    for i in range(n):
        print(f"  j{i}: {STATUS.get(status[i], '?'):9s} fault={fault[i]:02x} "
              f"pos={pos[i]:+8.4f} vel={vel[i]:+8.3f} eff={eff[i]:+6.3f}")


def stream(link: Link, seconds: float, every: float = 0.2):
    last = 0.0
    end = time.time() + seconds
    while time.time() < end:
        for p in link.poll():
            if len(p) == STATE.size and p[1] == MSG_STATE:
                if time.time() - last >= every:
                    print_state(p)
                    last = time.time()
            elif p[1] == MSG_LOG:
                print_log(p)
        time.sleep(0.005)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=921600)
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("watch").add_argument("--seconds", type=float, default=1e9)
    for name in CMD:
        sub.add_parser(name)
    g = sub.add_parser("gains")
    g.add_argument("joint", type=int)
    g.add_argument("--kp", type=float, default=0.5)
    g.add_argument("--ki", type=float, default=0.0)
    g.add_argument("--kd", type=float, default=0.01)
    g.add_argument("--kff-v", type=float, default=0.0)
    g.add_argument("--i-limit", type=float, default=0.3)
    g.add_argument("--out-limit", type=float, default=0.5)
    g.add_argument("--vel-filter-hz", type=float, default=50.0)
    g.add_argument("--pos-min", type=float, default=-3.14)
    g.add_argument("--pos-max", type=float, default=3.14)
    s = sub.add_parser("step", help="hold one joint at a position for a few seconds (others at 0)")
    s.add_argument("joint", type=int)
    s.add_argument("position", type=float)
    s.add_argument("--seconds", type=float, default=3.0)
    s.add_argument("--joints", type=int, default=3, help="joint_count the firmware was built with")
    args = ap.parse_args()

    link = Link(args.port, args.baud)
    if args.cmd in CMD:
        print(f"{args.cmd}: {link.command(args.cmd)}")
    elif args.cmd == "gains":
        print("gains:", link.gains(args.joint, args.kp, args.ki, args.kd, args.kff_v, args.i_limit, args.out_limit,
                                   args.vel_filter_hz, args.pos_min, args.pos_max))
    elif args.cmd == "step":
        target = [0.0] * args.joints
        target[args.joint] = args.position
        end = time.time() + args.seconds
        last = 0.0
        while time.time() < end:  # 100 Hz, like the bridge
            link.setpoint(target)
            for p in link.poll():
                if len(p) == STATE.size and p[1] == MSG_STATE and time.time() - last >= 0.2:
                    print_state(p)
                    last = time.time()
                elif p[1] == MSG_LOG:
                    print_log(p)
            time.sleep(0.01)
    else:
        stream(link, getattr(args, "seconds", 1e9))


if __name__ == "__main__":
    main()

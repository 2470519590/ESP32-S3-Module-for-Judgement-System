"""Decode the ESP32 compact Xbox UART0 control stream."""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Missing pyserial. Install it in py310: conda run -n py310 pip install pyserial", file=sys.stderr)
    raise SystemExit(2)


HEADER = b"\xc3\x3c"
FRAME_LENGTH = 14


@dataclass
class XboxState:
    sequence: int
    lx: int
    ly: int
    rx: int
    ry: int
    lt: int
    rt: int
    flags: int
    dpad: int
    buttons_1: int
    buttons_2: int

    @property
    def buttons(self) -> str:
        names: list[str] = []
        for bit, name in ((0x01, "A"), (0x02, "B"), (0x08, "X"), (0x10, "Y"),
                          (0x40, "LB"), (0x80, "RB")):
            if self.buttons_1 & bit:
                names.append(name)
        for bit, name in ((0x04, "VIEW"), (0x08, "MENU"), (0x10, "XBOX"),
                          (0x20, "LS"), (0x40, "RS")):
            if self.buttons_2 & bit:
                names.append(name)
        return " ".join(names) if names else "-"

    @property
    def dpad_text(self) -> str:
        return {0: "-", 1: "UP", 2: "UP+RIGHT", 3: "RIGHT", 4: "DOWN+RIGHT",
                5: "DOWN", 6: "DOWN+LEFT", 7: "LEFT", 8: "UP+LEFT"}.get(self.dpad, f"?{self.dpad}")


def decode(frame: bytes) -> XboxState | None:
    if len(frame) != FRAME_LENGTH or frame[:2] != HEADER:
        return None
    if crc8_atm(frame[:-1]) != frame[-1]:
        return None
    return XboxState(
        sequence=frame[2],
        lx=int.from_bytes(frame[4:5], "little", signed=True),
        ly=int.from_bytes(frame[5:6], "little", signed=True),
        rx=int.from_bytes(frame[6:7], "little", signed=True),
        ry=int.from_bytes(frame[7:8], "little", signed=True),
        lt=frame[8], rt=frame[9], flags=frame[3], dpad=frame[10],
        buttons_1=frame[11], buttons_2=frame[12],
    )


def crc8_atm(data: bytes) -> int:
    """CRC-8/ATM: poly 0x07, init 0x00, no reflection, xorout 0x00."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def axis(value: int) -> str:
    normalized = value / 127
    return f"{value:4d} ({normalized:+.2f})"


def dashboard(port: str, state: XboxState, received: int, lost: int, bad: int,
              rate: float) -> None:
    lines = [
        f" Xbox UART monitor | {port} | 115200 8N1 ",
        f" frames: {received:7d}   lost(seq): {lost:5d}   bad/resync: {bad:5d}   rate: {rate:5.1f} Hz",
        f" flags      : BLE={'ON' if state.flags & 1 else 'OFF'} HID={'OK' if state.flags & 2 else 'WAIT'}",
        f" left stick : X {axis(state.lx)}   Y {axis(state.ly)}",
        f" right stick: X {axis(state.rx)}   Y {axis(state.ry)}",
        f" triggers   : LT {state.lt:3d}/255   RT {state.rt:3d}/255",
        f" D-pad      : {state.dpad_text:<11}  buttons: {state.buttons}",
        f" raw        : seq={state.sequence:3d} buttons1=0x{state.buttons_1:02X} buttons2=0x{state.buttons_2:02X}",
        " Ctrl+C exits. Data is decoded from the ESP32's 14-byte binary stream.",
    ]
    width = max(len(line) for line in lines)
    print("\x1b[2J\x1b[H" + "+" + "=" * (width + 2) + "+")
    for line in lines:
        print("| " + line.ljust(width) + " |")
    print("+" + "=" * (width + 2) + "+", flush=True)


def available_ports() -> str:
    ports = [f"{item.device} ({item.description})" for item in list_ports.comports()]
    return "\n".join(ports) if ports else "No serial ports found."


def main() -> int:
    parser = argparse.ArgumentParser(description="Decode ESP32 Xbox UART frames")
    parser.add_argument("port", nargs="?", help="Serial port, e.g. COM12")
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--refresh", type=float, default=10.0, help="Dashboard refresh rate in Hz")
    args = parser.parse_args()
    if args.list:
        print(available_ports())
        return 0
    if not args.port:
        parser.error("port is required; use --list to find it")

    try:
        uart = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as error:
        print(f"Cannot open {args.port}: {error}", file=sys.stderr)
        return 2

    buffer = bytearray()
    received = lost = bad = 0
    previous_sequence: int | None = None
    last_state: XboxState | None = None
    last_dashboard = last_rate_time = time.monotonic()
    rate_frames = 0
    rate = 0.0
    print(f"Reading {args.port} at {args.baud}; waiting for C3 3C frames...", flush=True)
    try:
        while True:
            data = uart.read(uart.in_waiting or 1)
            if data:
                buffer.extend(data)
            while True:
                start = buffer.find(HEADER)
                if start < 0:
                    if len(buffer) > 1:
                        del buffer[:-1]
                    break
                if start:
                    bad += start
                    del buffer[:start]
                if len(buffer) < FRAME_LENGTH:
                    break
                frame = bytes(buffer[:FRAME_LENGTH])
                state = decode(frame)
                if state is None:
                    bad += 1
                    del buffer[0]
                    continue
                del buffer[:FRAME_LENGTH]
                if previous_sequence is not None:
                    gap = (state.sequence - previous_sequence) & 0xFF
                    if 1 < gap < 128:
                        lost += gap - 1
                previous_sequence = state.sequence
                last_state = state
                received += 1
                rate_frames += 1

            now = time.monotonic()
            if now - last_rate_time >= 1.0:
                rate = rate_frames / (now - last_rate_time)
                rate_frames = 0
                last_rate_time = now
            if last_state is not None and now - last_dashboard >= 1.0 / max(args.refresh, 0.5):
                dashboard(args.port, last_state, received, lost, bad, rate)
                last_dashboard = now
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        uart.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

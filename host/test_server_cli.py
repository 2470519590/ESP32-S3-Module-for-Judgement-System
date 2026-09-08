"""Formal V2 robot server CLI for the current Wi-Fi integration.

The ESP32 must be in real-data mode: status and events come from L431PM.
The Wi-Fi is temporary test infrastructure; the protocol and CLI are V2.
"""
from __future__ import annotations

import asyncio
import argparse
import shlex
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

MAGIC = 0x5254
V1 = 1
V2 = 2

FRAME_STATUS = 1
FRAME_DEATH = 2
FRAME_REVIVE = 3
FRAME_HIT = 4
FRAME_ATTACK = 5
FRAME_SHOOT_ENABLED = 6
FRAME_SHOOT_DISABLED = 7
FRAME_REFEREE_LINK_DOWN = 8
FRAME_REFEREE_LINK_UP = 9
FRAME_GAME_START = 0x81
FRAME_GAME_END = 0x82
FRAME_ASSIGNMENT = 0x83
FRAME_STATUS_REQUEST = 0x84
FRAME_SET_HP = 0x85
FRAME_ACK = 0xF0

EVENT_NAMES = {
    FRAME_DEATH: "DEATH", FRAME_REVIVE: "REVIVE", FRAME_HIT: "HIT",
    FRAME_ATTACK: "ATTACK", FRAME_SHOOT_ENABLED: "SHOOT_ON",
    FRAME_SHOOT_DISABLED: "SHOOT_OFF", FRAME_REFEREE_LINK_DOWN: "LINK_DOWN",
    FRAME_REFEREE_LINK_UP: "LINK_UP",
}


def mac_bytes(text: str) -> bytes:
    chunks = text.replace("-", ":").split(":")
    if len(chunks) != 6:
        raise ValueError("MAC must be xx:xx:xx:xx:xx:xx")
    return bytes(int(chunk, 16) for chunk in chunks)


def decode(data: bytes) -> str:
    if len(data) < 4:
        return f"unknown short packet ({len(data)} B): {data.hex(' ')}"
    magic, version, frame_type = struct.unpack_from("<HBB", data)
    if magic != MAGIC:
        return f"unknown magic ({len(data)} B): {data.hex(' ')}"

    if version == V1:
        if frame_type == FRAME_STATUS and len(data) == 10:
            _, _, _, robot, team, hp, alive, shoot = struct.unpack("<HBBBBHBB", data)
            return f"V1 STATUS robot={robot} team={team} hp={hp} alive={alive} shoot={shoot}"
        if frame_type == FRAME_HIT and len(data) == 8:
            _, _, _, robot, team, hp = struct.unpack("<HBBBBH", data)
            return f"V1 HIT robot={robot} team={team} hp={hp}"
        if frame_type in {FRAME_DEATH, FRAME_REVIVE, FRAME_ATTACK, FRAME_SHOOT_ENABLED, FRAME_SHOOT_DISABLED} and len(data) == 6:
            _, _, _, robot, team = struct.unpack("<HBBBB", data)
            return f"V1 EVENT type={frame_type} robot={robot} team={team}"

    if version == V2:
        if frame_type == FRAME_STATUS and len(data) == 13:
            _, _, _, robot, hp, heat, power, alive, shoot = struct.unpack("<HBBBHHHBB", data)
            return (f"V2 STATUS robot={robot} hp={hp} heat={heat} power={power}W "
                    f"alive={alive} shoot={shoot}")
        if frame_type == FRAME_ACK and len(data) == 10:
            _, _, _, acked_type, transaction_id, result = struct.unpack("<HBBBIB", data)
            return f"V2 ACK type=0x{acked_type:02X} tx={transaction_id} result={result}"
        if frame_type in {FRAME_DEATH, FRAME_REVIVE} and len(data) == 9:
            _, _, _, robot, transaction_id = struct.unpack("<HBBBI", data)
            return f"V2 EVENT type={frame_type} robot={robot} tx={transaction_id}"
        if frame_type in {FRAME_HIT, FRAME_ATTACK, FRAME_SHOOT_ENABLED, FRAME_SHOOT_DISABLED,
                          FRAME_REFEREE_LINK_DOWN, FRAME_REFEREE_LINK_UP} and len(data) == 5:
            _, _, _, robot = struct.unpack("<HBBB", data)
            return f"V2 EVENT type={frame_type} robot={robot}"

    return f"unknown V{version} type=0x{frame_type:02X} ({len(data)} B): {data.hex(' ')}"


@dataclass
class CliServer:
    sock: socket.socket
    robot_port: int
    transaction_id: int = 1
    print_lock: threading.Lock = field(default_factory=threading.Lock)
    stats_lock: threading.Lock = field(default_factory=threading.Lock)
    running: bool = True
    window_rx: int = 0
    window_status: int = 0
    window_events: int = 0
    window_auto_acks: int = 0
    window_protocol_acks: int = 0
    window_bad: int = 0
    window_intervals: list[float] = field(default_factory=list)
    last_rx_time: float | None = None
    max_gap_ms: float = 0.0
    event_counts: dict[int, int] = field(default_factory=dict)
    last_status: dict[int, tuple[int, int, int, int, int]] = field(default_factory=dict)
    last_event: str = "-"
    ui_message: Callable[[str], None] | None = None
    robot_peers: dict[int, str] = field(default_factory=dict)

    def output(self, text: str) -> None:
        if self.ui_message is not None:
            self.ui_message(text)
            return
        with self.print_lock:
            print(text, flush=True)

    def next_transaction(self) -> int:
        value = self.transaction_id
        self.transaction_id = 1 if value == 0xFFFFFFFF else value + 1
        return value

    def send(self, ip: str, frame: bytes, description: str) -> None:
        self.sock.sendto(frame, (ip, self.robot_port))
        self.output(f"TX {description} | {frame.hex(' ')}")

    def robot_ip(self, robot_id: int) -> str:
        ip = self.robot_peers.get(robot_id)
        if ip is None:
            raise ValueError(f"robot {robot_id} has not sent a status frame")
        return ip

    def acknowledge_reliable_event(self, data: bytes, peer: tuple[str, int]) -> None:
        """ACK V2 death/revive immediately, so ESP32 retransmission can be tested."""
        if len(data) != 9:
            return
        magic, version, frame_type, _robot_id, transaction_id = struct.unpack("<HBBBI", data)
        if magic != MAGIC or version != V2 or frame_type not in {FRAME_DEATH, FRAME_REVIVE}:
            return
        ack = struct.pack("<HBBBIB", MAGIC, V2, FRAME_ACK, frame_type, transaction_id, 0)
        # ESP32 receives every server downlink, including ACK, on its fixed
        # listen port rather than the temporary source port of UDP uplink.
        self.sock.sendto(ack, (peer[0], self.robot_port))
        with self.stats_lock:
            self.window_auto_acks += 1

    def record_packet(self, data: bytes, peer: tuple[str, int]) -> None:
        now = time.monotonic()
        with self.stats_lock:
            self.window_rx += 1
            if self.last_rx_time is not None:
                gap_ms = (now - self.last_rx_time) * 1000.0
                self.window_intervals.append(gap_ms)
                self.max_gap_ms = max(self.max_gap_ms, gap_ms)
            self.last_rx_time = now
            if len(data) >= 4 and data[:2] == struct.pack("<H", MAGIC) and data[2] == V2:
                frame_type = data[3]
                if frame_type == FRAME_STATUS and len(data) == 13:
                    _, _, _, robot, hp, heat, power, alive, shoot = struct.unpack(
                        "<HBBBHHHBB", data)
                    self.last_status[robot] = (hp, heat, power, alive, shoot)
                    self.robot_peers[robot] = peer[0]
                    self.window_status += 1
                elif frame_type == FRAME_ACK and len(data) == 10:
                    self.window_protocol_acks += 1
                    self.last_event = decode(data)
                elif frame_type != FRAME_STATUS:
                    self.window_events += 1
                    self.event_counts[frame_type] = self.event_counts.get(frame_type, 0) + 1
                    self.last_event = decode(data)
            else:
                self.window_bad += 1

    def summary(self) -> str:
        with self.stats_lock:
            rx, status, events, bad = (self.window_rx, self.window_status,
                                       self.window_events, self.window_bad)
            acks = self.window_auto_acks
            protocol_acks = self.window_protocol_acks
            self.window_rx = self.window_status = self.window_events = self.window_bad = 0
            self.window_auto_acks = 0
            self.window_protocol_acks = 0
            intervals = self.window_intervals
            self.window_intervals = []
            avg_gap = sum(intervals) / len(intervals) if intervals else 0.0
            jitter = ((sum((value - avg_gap) ** 2 for value in intervals) /
                       len(intervals)) ** 0.5) if intervals else 0.0
            max_gap = self.max_gap_ms
            self.max_gap_ms = 0.0
            states = " ".join(
                f"R{robot}:hp={hp} alive={alive} shoot={shoot}"
                for robot, (hp, _heat, _power, alive, shoot) in sorted(self.last_status.items())
            ) or "-"
            event_text = " ".join(f"{EVENT_NAMES.get(kind, f'0x{kind:02X}')}:{count}"
                                  for kind, count in sorted(self.event_counts.items())) or "-"
            self.event_counts.clear()
            return (f"1s rx={rx:3d} status={status:3d} events={events:2d} ack={acks:2d} "
                    f"udp_ack={protocol_acks:2d} bad={bad:2d}\n"
                    f"   net gap(avg/jitter/max)={avg_gap:5.1f}/{jitter:5.1f}/{max_gap:5.1f} ms "
                    f"| event_types={event_text}\n"
                    f"   {states} | last={self.last_event}")

    def summary_loop(self) -> None:
        while self.running:
            time.sleep(1.0)
            if self.running:
                self.output(self.summary())


def run_prompt_toolkit_ui(server: CliServer) -> None:
    """Run a real split-pane terminal UI: live status above, normal input below."""
    from prompt_toolkit.application import Application
    from prompt_toolkit.key_binding import KeyBindings
    from prompt_toolkit.layout import HSplit, Layout
    from prompt_toolkit.widgets import Frame, TextArea

    status = TextArea(read_only=True, scrollbar=True, wrap_lines=False)
    command = TextArea(height=1, prompt="> ", multiline=False)
    messages: list[str] = []
    lock = threading.Lock()

    def add_message(text: str) -> None:
        with lock:
            messages.extend(text.splitlines() or [text])
            del messages[:-6]

    server.ui_message = add_message

    def refresh() -> None:
        with lock:
            recent = "\n".join(messages)
        live = server.summary()
        status.text = ("ESP32/L431PM 正式 V2 联调 | 真实 L431PM 数据 | 测试 Wi-Fi\n"
                       + live + ("\n\n" + recent if recent else ""))

    bindings = KeyBindings()

    @bindings.add("enter")
    def _(event) -> None:
        line = command.text
        command.text = ""
        if not server.command(line):
            server.running = False
            event.app.exit()
        refresh()

    @bindings.add("c-c")
    def _(event) -> None:
        server.running = False
        event.app.exit()

    app = Application(
        layout=Layout(HSplit([
            Frame(status, title="实时通信状态（每秒刷新）"),
            Frame(command, title="命令输入（正常输入，Enter 执行）"),
        ]), focused_element=command),
        key_bindings=bindings,
        full_screen=True,
        refresh_interval=1.0,
    )
    refresh()
    try:
        async def run_application() -> None:
            # The task must be created after prompt_toolkit has started its
            # asyncio loop; creating it before app.run_async() raises
            # RuntimeError: no running event loop.
            app.create_background_task(_refresh_task(app, refresh))
            await app.run_async()

        asyncio.run(run_application())
    finally:
        server.ui_message = None


async def _refresh_task(app, refresh: Callable[[], None]) -> None:
    while True:
        refresh()
        app.invalidate()
        await asyncio.sleep(1.0)


def server_command(self: CliServer, line: str) -> bool:
    """Normal command handler kept at module scope for the split-pane UI."""
    parts = shlex.split(line)
    if not parts:
        return True
    name = parts[0].lower()
    if name in {"quit", "exit"}:
        return False
    if name == "help":
        self.output("Commands: stats | start ROBOT_ID | end ROBOT_ID | hp ROBOT_ID HP | status ROBOT_ID | assign ROBOT_ID CONTROLLER_MAC | quit")
        return True
    if name == "stats":
        self.output(self.summary())
        return True
    try:
        if name == "assign" and len(parts) == 3:
            robot_id = int(parts[1])
            ip = self.robot_ip(robot_id)
            tx = self.next_transaction()
            if not 1 <= robot_id <= 255:
                raise ValueError("robot ID must be 1..255")
            frame = struct.pack("<HBBB6sI", MAGIC, V2, FRAME_ASSIGNMENT,
                                robot_id, mac_bytes(parts[2]), tx)
            self.send(ip, frame, f"ASSIGN robot={robot_id} controller={parts[2]} tx={tx}")
        elif name in {"start", "end"} and len(parts) == 2:
            robot_id = int(parts[1])
            ip = self.robot_ip(robot_id)
            frame_type = FRAME_GAME_START if name == "start" else FRAME_GAME_END
            tx = self.next_transaction()
            frame = struct.pack("<HBBBI", MAGIC, V2, frame_type, robot_id, tx)
            self.send(ip, frame, f"{name.upper()} target={robot_id} tx={tx}")
        elif name == "hp" and len(parts) == 3:
            robot_id = int(parts[1])
            ip = self.robot_ip(robot_id)
            tx = self.next_transaction()
            hp = int(parts[2])
            if not 0 <= hp <= 300:
                raise ValueError("HP must be 0..300")
            frame = struct.pack("<HBBBHI", MAGIC, V2, FRAME_SET_HP, robot_id, hp, tx)
            self.send(ip, frame, f"SET_HP target={robot_id} hp={hp} tx={tx}")
        elif name == "status" and len(parts) == 2:
            robot_id = int(parts[1])
            ip = self.robot_ip(robot_id)
            frame = struct.pack("<HBBB", MAGIC, V2, FRAME_STATUS_REQUEST, robot_id)
            self.send(ip, frame, f"STATUS_REQUEST target={robot_id}")
        else:
            self.output("Invalid command. Type help.")
    except (ValueError, OSError, struct.error) as exc:
        self.output(f"Command error: {exc}")
    return True


CliServer.command = server_command


def receive_loop(server: CliServer) -> None:
    while True:
        try:
            data, peer = server.sock.recvfrom(256)
        except OSError:
            return
        server.record_packet(data, peer)
        server.acknowledge_reliable_event(data, peer)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0", help="local IPv4 address")
    parser.add_argument("--port", type=int, default=5005, help="local server UDP port")
    parser.add_argument("--robot-port", type=int, default=5006, help="ESP32 downlink UDP port")
    parser.add_argument("--watch", action="store_true",
                        help="print one receive summary per second; no interactive commands")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))
    server = CliServer(sock=sock, robot_port=args.robot_port)
    threading.Thread(target=receive_loop, args=(server,), daemon=True).start()
    server.output(f"Formal V2 CLI listening on {args.bind}:{args.port}; robot downlink port={args.robot_port}")
    server.output("Network: evil rats crazily squeak | data source: real L431PM")
    try:
        if args.watch:
            while True:
                time.sleep(1.0)
                server.output(server.summary())
        else:
            run_prompt_toolkit_ui(server)
    except (EOFError, KeyboardInterrupt):
        server.output("Stopping.")
    finally:
        server.running = False
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""PC-side standard ICMP monitor; Python 3.10+, standard library only."""
from __future__ import annotations
import argparse, csv, platform, re, socket, struct, subprocess, sys, time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

def one_ping(target: str, timeout_ms: int) -> tuple[bool, float | None]:
    if platform.system().lower().startswith("win"):
        command = ["ping", "-4", "-n", "1", "-w", str(timeout_ms), target]
    else:
        command = ["ping", "-c", "1", "-W", str(max(1, timeout_ms // 1000)), target]
    # Windows ping uses the system code page. Decoding it as UTF-8 can corrupt
    # localized output and make a successful reply look like a timeout.
    result = subprocess.run(command, capture_output=True, text=False)
    output = result.stdout.decode("mbcs", errors="replace") if platform.system().lower().startswith("win") else result.stdout.decode(errors="replace")
    output += result.stderr.decode("mbcs", errors="replace") if platform.system().lower().startswith("win") else result.stderr.decode(errors="replace")
    match = re.search(r"(?:time|时间)[=<]\s*([0-9]+(?:[.,][0-9]+)?)\s*ms", output, re.I)
    if match:
        return True, float(match.group(1).replace(",", "."))
    # Some localized Windows builds translate the time label. The Reply line
    # and zero exit status still reliably indicate that the ICMP echo returned.
    if result.returncode == 0 and re.search(r"(?:Reply from|来自)", output, re.I):
        return True, None
    return False, None

def decode_robot_packet(data: bytes) -> dict | None:
    """Decode the fixed 20-byte robot status/event test packet."""
    if len(data) != 20:
        return None
    magic, kind, robot_id, team, seq, hp, heat, power, combat, event, rssi, channel, timestamp = struct.unpack("<HBBBBHHHBBbBI", data)
    if magic != 0x5231 or kind not in (1, 2):
        return None
    return locals()

def metric(value: object, width: int = 6) -> str:
    if isinstance(value, (int, float)):
        return f"{float(value):{width}.1f}"
    return f"{'-':>{width}}"

def dashboard(history: deque[tuple[str, list[str]]]) -> None:
    width = 76
    print("\x1b[2J\x1b[H", end="")
    labels = ("CURRENT 1s WINDOW", "PREVIOUS 1s WINDOW", "PREVIOUS 2s WINDOW")
    for index, (timestamp, lines) in enumerate(history):
        title = f"Wi-Fi Robot Network Test | {labels[index]} | {timestamp}"
        print("+" + "=" * width + "+")
        print("|" + f" {title}".ljust(width) + "|")
        print("+" + "-" * width + "+")
        for line in lines:
            print("| " + line[:width - 2].ljust(width - 2) + " |")
        print("+" + "=" * width + "+")
        if index != len(history) - 1:
            print()
    print("(current and previous two 1-second windows)", flush=True)

def main() -> int:
    p = argparse.ArgumentParser(description="Standard ICMP Wi-Fi quality monitor")
    p.add_argument("target", help="ESP32 IP or AP gateway IP")
    p.add_argument("--interval", type=float, default=0.1)
    p.add_argument("--timeout-ms", type=int, default=1000)
    p.add_argument("--duration", type=float, default=0, help="0 means until Ctrl-C")
    p.add_argument("--csv", type=Path, default=Path("wifi_monitor.csv"))
    p.add_argument("--verbose", action="store_true", help="print every ping")
    p.add_argument("--no-dashboard", action="store_true", help="use one-line summaries instead of the dashboard")
    p.add_argument("--telemetry-port", type=int, default=5005,
                   help="UDP port for ESP diagnostic telemetry; 0 disables it")
    a = p.parse_args(); a.csv.parent.mkdir(parents=True, exist_ok=True)
    exists = a.csv.exists() and a.csv.stat().st_size > 0
    if exists:
        first_line = a.csv.open("r", encoding="utf-8", errors="replace").readline()
        if "pc_local" not in first_line:
            a.csv = a.csv.with_name(a.csv.stem + "_local" + a.csv.suffix)
            exists = a.csv.exists() and a.csv.stat().st_size > 0
            print(f"Legacy CSV header detected; writing local-time log to {a.csv}", flush=True)
    start = time.monotonic(); next_summary = start + 1.0; seq = 0
    history: deque[tuple[str, list[str]]] = deque(maxlen=3)
    total_tx = total_rx = window_tx = window_rx = 0; window_rtts = []; window_jitters = []; previous_rtt = None
    telemetry = None
    monitor_lock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        monitor_lock.bind(("127.0.0.1", 54545))
    except OSError:
        print("Another wifi_monitor.py instance is already running; stop it first.", file=sys.stderr)
        return 2
    telemetry_status_rx = telemetry_status_loss = telemetry_events = 0
    telemetry_last_seq = None
    telemetry_info = {}
    if a.telemetry_port:
        telemetry = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        telemetry.bind(("0.0.0.0", a.telemetry_port))
        telemetry.setblocking(False)
    with a.csv.open("a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if not exists:
            w.writerow(["pc_local", "pc_utc", "pc_unix_s", "pc_monotonic_s", "target", "seq", "ok", "rtt_ms", "window_tx", "window_rx", "window_loss_pct", "window_avg_ms"])
        print(f"Monitoring {a.target}; interval={a.interval:.3f}s; CSV: {a.csv.resolve()}", flush=True)
        try:
            while a.duration <= 0 or time.monotonic() - start < a.duration:
                cycle = time.monotonic(); ok, rtt = one_ping(a.target, a.timeout_ms)
                total_tx += 1; window_tx += 1
                if ok:
                    total_rx += 1; window_rx += 1
                    if rtt is not None:
                        window_rtts.append(rtt)
                        if previous_rtt is not None: window_jitters.append(abs(rtt - previous_rtt))
                        previous_rtt = rtt
                now = time.monotonic(); wall = time.time(); loss = (window_tx - window_rx) * 100 / window_tx
                if telemetry:
                    while True:
                        try:
                            data, _ = telemetry.recvfrom(512)
                            packet = decode_robot_packet(data)
                            if packet:
                                telemetry_info = packet
                                if packet["kind"] == 1:
                                    telemetry_status_rx += 1
                                    if telemetry_last_seq is not None:
                                        gap = (packet["seq"] - telemetry_last_seq) & 0xff
                                        if gap > 1: telemetry_status_loss += gap - 1
                                    telemetry_last_seq = packet["seq"]
                                else:
                                    telemetry_events += 1
                        except BlockingIOError:
                            break
                avg = sum(window_rtts) / len(window_rtts) if window_rtts else ""
                local_time = datetime.fromtimestamp(wall).astimezone().isoformat(timespec="milliseconds")
                utc_time = datetime.fromtimestamp(wall, timezone.utc).isoformat(timespec="milliseconds")
                w.writerow([local_time, utc_time, f"{wall:.6f}", f"{now-start:.6f}", a.target, seq, int(ok), rtt if rtt is not None else "", window_tx, window_rx, f"{loss:.2f}", avg]); f.flush()
                if a.verbose: print(f"seq={seq:6d} {'OK' if ok else 'TIMEOUT':7s} rtt={rtt if rtt is not None else '-'} ms", flush=True)
                if now >= next_summary:
                    total_loss = (total_tx - total_rx) * 100 / total_tx
                    ordered = sorted(window_rtts)
                    p95 = ordered[min(len(ordered) - 1, max(0, int(len(ordered) * 0.95) - 1))] if ordered else '-'
                    minimum = min(window_rtts) if window_rtts else '-'
                    jitter = sum(window_jitters) / len(window_jitters) if window_jitters else '-'
                    rssi = telemetry_info.get("rssi", "-")
                    channel = telemetry_info.get("channel", "-")
                    combat = telemetry_info.get("combat", "-")
                    robot_id = telemetry_info.get("robot_id", "-")
                    status_loss = telemetry_status_loss * 100 / max(1, telemetry_status_rx + telemetry_status_loss)
                    rows = [
                        f"ICMP  tx/rx: {window_tx:3d}/{window_rx:3d}   loss: {loss:6.2f}%   total: {total_loss:6.2f}%",
                        f"RTT   min/avg/p95/max: {metric(minimum)}/{metric(avg)}/{metric(p95)}/{metric(max(window_rtts) if window_rtts else '-')} ms",
                        f"      jitter: {metric(jitter)} ms",
                        "",
                        f"UDP   status received: {telemetry_status_rx:5d}   loss: {status_loss:6.2f}%",
                        f"      events: {telemetry_events:3d}   robot: {str(robot_id):>3}   combat: {str(combat):>1}",
                        "",
                        f"WIFI  RSSI: {str(rssi):>4} dBm   channel: {str(channel):>2}",
                        f"      interval: {a.interval:.3f} s   target: {a.target}",
                    ]
                    if a.no_dashboard:
                        print(" | ".join(row.strip() for row in rows if row.strip()), flush=True)
                    else:
                        history.appendleft((local_time, rows))
                        dashboard(history)
                    window_tx = window_rx = 0; window_rtts.clear(); window_jitters.clear(); previous_rtt = None; next_summary = now + 1.0
                seq += 1; time.sleep(max(0, a.interval - (time.monotonic() - cycle)))
        except KeyboardInterrupt: print("Stopping...", flush=True)
    print(f"Summary: tx={total_tx} rx={total_rx} loss={(total_tx-total_rx)*100/total_tx if total_tx else 0:.2f}%")
    monitor_lock.close()
    return 0

if __name__ == "__main__": sys.exit(main())

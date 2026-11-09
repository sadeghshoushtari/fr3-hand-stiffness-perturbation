"""Sensor log on the laptop for a perturbation session when the board cannot reach the robot PC.

  python laptop_sensor_logger.py [--board 192.168.137.50] [--out <dir>] [--max-min 60]

The board stays on the laptop hotspot and sends its features here (UDP 50500). Every packet is written with the
laptop's Unix time to <out>/sensors_laptop.csv; perturbation_test writes the trial times as Unix time on the robot
PC, and analyze_perturbation.py --sensors matches the two by clock time. Both clock offsets to internet time are
measured (SNTP) at start and end and stored in <out>/clock.json.

Control while running, by files in <out> (so it can be driven from another shell):
  VIBRATOR  present -> vibrator on (re-armed every 60 s, the board's own limit is 600 s); absent -> off
  STOP      present -> vibrator off, final clock check, exit
A status line (amplitudes, pair phases and how often they were sent, last 2 s) is printed every 2 s.
"""

import argparse
import cmath
import json
import math
import socket
import struct
import time
from pathlib import Path

SENSOR_PORT = 50500
COMMAND_PORT = 50601
REARM_S = 60.0
STATUS_S = 2.0
NTP_SERVERS = ("ptbtime1.ptb.de", "time.windows.com", "pool.ntp.org")
PAIRS = {"triceps": (8, 9), "biceps": (10, 11)}  # mounting of 11-05: clip A (ch0, ch1) triceps, clip B biceps


def ntp_offset_s():
    """Local clock minus server time (s), from one SNTP exchange; None if no server answers."""
    for server in NTP_SERVERS:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(2.0)
            t1 = time.time()
            s.sendto(b"\x1b" + 47 * b"\0", (server, 123))
            data = s.recv(48)
            t4 = time.time()
            secs, frac = struct.unpack("!II", data[40:48])
            server_time = secs - 2208988800 + frac / 2 ** 32
            return round((t1 + t4) / 2 - server_time, 4), server
        except OSError:
            continue
    return None, None


def board_command(sock, board, text):
    # Windows reports an unreachable board as ConnectionResetError on the reply; the board is then simply offline.
    try:
        sock.sendto(text.encode(), (board, COMMAND_PORT))
        return sock.recvfrom(100)[0].decode()
    except OSError:
        return "no reply"


def status(window):
    if not window:
        return "no packets in the last 2 s"
    amps = [sorted(r[4 + k] for r in window)[len(window) // 2] for k in range(4)]
    parts = ["amps " + " ".join(f"{a:5.1f}" for a in amps)]
    for name, (c, s) in PAIRS.items():
        z = [complex(r[c], r[s]) for r in window if r[c] or r[s]]
        if z:
            m = sum(z) / len(z)
            parts.append(f"{name} {math.degrees(cmath.phase(m)):5.0f} deg sent {len(z)}/{len(window)}")
        else:
            parts.append(f"{name} not sent")
    if any(a == 0.0 for a in amps):
        parts.append("WARNING: a channel reads exactly 0 (connection)")
    return " | ".join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", default="192.168.137.50")
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parent / "perturbation_logs" /
                    time.strftime("%Y%m%d_%H%M%S_laptop"))
    ap.add_argument("--max-min", type=float, default=60.0, help="exit (vibrator off) after this many minutes")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    clock = {"start": dict(zip(("offset_s", "server"), ntp_offset_s())), "start_unix_s": time.time()}
    print(f"clock offset to internet time at start: {clock['start']}", flush=True)

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", SENSOR_PORT))
    rx.settimeout(0.2)
    cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd.settimeout(1.0)

    vibrator_on = False
    last_rearm = 0.0
    last_status = time.time()
    window = []
    t_start = time.time()
    with open(args.out / "sensors_laptop.csv", "w") as f:
        f.write("unix_s,values...\n")
        try:
            while not (args.out / "STOP").exists() and time.time() - t_start < 60.0 * args.max_min:
                want = (args.out / "VIBRATOR").exists()
                now = time.time()
                if want and (not vibrator_on or now - last_rearm > REARM_S):
                    reply = board_command(cmd, args.board, "start")
                    if not vibrator_on:
                        print(f"{time.strftime('%H:%M:%S')} vibrator on ({reply})", flush=True)
                    vibrator_on, last_rearm = True, now
                elif not want and vibrator_on:
                    print(f"{time.strftime('%H:%M:%S')} vibrator off ({board_command(cmd, args.board, 'stop')})",
                          flush=True)
                    vibrator_on = False
                try:
                    data = rx.recvfrom(2048)[0]
                    values = struct.unpack("<%dd" % (len(data) // 8), data)
                    t = time.time()
                    f.write(f"{t:.4f}," + ",".join(f"{v:.6g}" for v in values) + "\n")
                    f.flush()
                    window.append(values)
                except (socket.timeout, ConnectionResetError):
                    pass
                if time.time() - last_status >= STATUS_S:
                    print(f"{time.strftime('%H:%M:%S')} {status(window)}", flush=True)
                    window, last_status = [], time.time()
        finally:
            print(f"{time.strftime('%H:%M:%S')} vibrator off ({board_command(cmd, args.board, 'stop')})", flush=True)

    clock["end"] = dict(zip(("offset_s", "server"), ntp_offset_s()))
    clock["end_unix_s"] = time.time()
    (args.out / "clock.json").write_text(json.dumps(clock, indent=2))
    print(f"clock offset at end: {clock['end']}; log in {args.out}", flush=True)


if __name__ == "__main__":
    main()

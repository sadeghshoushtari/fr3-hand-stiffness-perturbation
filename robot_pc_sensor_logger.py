"""Sensor log on the robot PC, board connected by USB (firmware esp32_robot_stiffness 1.1.0 or later).

  python3 robot_pc_sensor_logger.py [--port /dev/ttyUSB0] [--out <dir>] [--max-min 60] [--udp 127.0.0.1:50500]
                                    [--commands 50601]          (or ./sensors.sh)

The board prints one line per feature update, "F,<12 values>" (same values as its UDP packet). Every line is written
with this PC's Unix time to <out>/sensors.csv. perturbation_test runs on the same PC and writes the trial times as
Unix time, so no clock correction is needed:

  python3 analyze_perturbation.py perturbation_logs/<date_time> --sensors <out>/sensors.csv

--udp HOST:PORT also sends every line as 12 doubles (the board's UDP packet format) to that address; default
127.0.0.1:50500, the ESP32 block of the MOSAIC pipelines; --udp off sends nothing.

--commands PORT listens on that UDP port (default 50601, 0 = off) for vibrator commands, so that MOSAIC switches the vibrator: a text ending
in "on" switches it on, one ending in "off" switches it off (the CommandSender block sends e.g. "IMU0on" / "IMU0off";
mosaic/session_*.json use port 50601). The same re-arm and time limit as the keyboard apply.

Keyboard (type and press Enter): on = vibrator on (re-armed every 60 s, the board's own limit is 600 s),
off = vibrator off, q = vibrator off and exit. A status line (amplitudes, pair phases and how often they were sent,
last 2 s) is printed every 2 s. Needs read access to the port (group dialout); no extra Python packages.
"""

import argparse
import cmath
import math
import os
import select
import socket
import struct
import sys
import termios
import time
from pathlib import Path

REARM_S = 60.0
STATUS_S = 2.0
PAIRS = {"triceps": (8, 9), "biceps": (10, 11)}  # mounting of 11-05: clip A (ch0, ch1) triceps, clip B biceps


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attr = termios.tcgetattr(fd)
    attr[0] = 0                                              # iflag: raw input
    attr[1] = 0                                              # oflag: raw output
    attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL   # cflag: 8N1, no modem control
    attr[3] = 0                                              # lflag: no echo, no line editing
    attr[4] = attr[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attr)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def board_command(fd, text):
    os.write(fd, (text + "\n").encode())


def status(window):
    if not window:
        return "no data in the last 2 s"
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
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parent / "perturbation_logs" /
                    time.strftime("%Y%m%d_%H%M%S_sensors"))
    ap.add_argument("--max-min", type=float, default=60.0, help="exit (vibrator off) after this many minutes")
    ap.add_argument("--udp", default="127.0.0.1:50500", help="send every line as 12 doubles to HOST:PORT (off: none)")
    ap.add_argument("--commands", type=int, default=50601, help="UDP port for vibrator on/off commands (0: none)")
    args = ap.parse_args()
    udp = None
    if args.udp and args.udp != "off":
        host, port = args.udp.rsplit(":", 1)
        udp = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (host, int(port)))
    commands = None
    if args.commands:
        commands = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        commands.bind(("127.0.0.1", args.commands))
    args.out.mkdir(parents=True, exist_ok=True)

    fd = open_port(args.port)
    print(f"reading {args.port}, log in {args.out}" + (f", sending to {args.udp}" if udp else "")
          + (f", vibrator commands on port {args.commands}" if commands else "")
          + "; type on / off / q and Enter", flush=True)

    vibrator_on = False
    last_rearm = 0.0
    last_status = time.time()
    window = []
    buf = b""
    t_start = time.time()
    with open(args.out / "sensors.csv", "w") as f:
        f.write("unix_s,values...\n")
        try:
            while time.time() - t_start < 60.0 * args.max_min:
                ready = select.select([fd, sys.stdin] + ([commands] if commands else []), [], [], 0.2)[0]
                key = None
                if commands in ready:
                    text = commands.recv(256).decode("ascii", "replace").strip().lower()
                    key = "off" if text.endswith("off") else "on" if text.endswith("on") else ""
                    print(f"{time.strftime('%H:%M:%S')} command '{text}' -> vibrator {key or 'unchanged'}", flush=True)
                elif sys.stdin in ready:
                    key = sys.stdin.readline().strip().lower()
                if key is not None:
                    if key == "q":
                        break
                    if key == "on":
                        vibrator_on, last_rearm = True, time.time()
                        board_command(fd, "start")
                    elif key == "off":
                        vibrator_on = False
                        board_command(fd, "stop")
                    elif key:
                        print("type on, off or q", flush=True)
                if vibrator_on and time.time() - last_rearm > REARM_S:
                    board_command(fd, "start")
                    last_rearm = time.time()
                if fd in ready:
                    buf += os.read(fd, 4096)
                    *lines, buf = buf.split(b"\n")
                    t = time.time()
                    for raw in lines:
                        line = raw.decode("ascii", "replace").strip()
                        if not line.startswith("F,"):
                            if line:
                                print(f"{time.strftime('%H:%M:%S')} board: {line}", flush=True)
                            continue
                        try:
                            values = [float(x) for x in line[2:].split(",")]
                        except ValueError:
                            continue
                        if len(values) != 12:
                            continue  # partial line after the port was opened
                        f.write(f"{t:.4f}," + ",".join(f"{v:.6g}" for v in values) + "\n")
                        window.append(values)
                        if udp:
                            udp[0].sendto(struct.pack("<12d", *values), udp[1])
                    f.flush()
                if time.time() - last_status >= STATUS_S:
                    state = "ON " if vibrator_on else "off"
                    print(f"{time.strftime('%H:%M:%S')} vibrator {state} | {status(window)}", flush=True)
                    window, last_status = [], time.time()
        except KeyboardInterrupt:
            pass
        finally:
            board_command(fd, "stop")
            print(f"{time.strftime('%H:%M:%S')} vibrator off; log in {args.out}", flush=True)
            os.close(fd)


if __name__ == "__main__":
    main()

"""Virtual operator, board, robot and IMUs for the headless session test (standard library only).

python simulate_session.py <scenario.json> calibration|run <out_dir>

Plays the time windows of scenario.json (make_scenario.py) from its start:
  board   5 Hz, 12 doubles to 127.0.0.1:50500 (esp32_robot_stiffness features; pair phases of the window + 1.5 deg
          noise; [8, 9] triceps cos/sin, [10, 11] biceps)
  IMU     50 Hz, 8 floats to 127.0.0.1:50700 (upper-arm and forearm quaternions of the window's posture)
  robot   (calibration) perturbation.cpp: offsets from port 50900 (scenario "robot_ports") (clamped to 20 mm, 0.1 m/s, zero after
          0.25 s without a packet); the handle settles where K_h dx = K_r (dx_d - dx), K_h = the window's hand
          stiffness; 15 doubles of state at 500 Hz to 127.0.0.1:50910
  outputs (run) records the pipeline's output: port 50510 (StiffnessProjection, 10 doubles) and 4500 (stiffness
          command, 1 double; scenario "slave_port")
Writes <out_dir>/sim_features.csv (t, 12 values sent) and, in run mode, sim_monitor.csv and sim_slave.csv.
"""
import json
import math
import random
import socket
import struct
import sys
import time
from pathlib import Path

LOCAL = "127.0.0.1"


def solve3(a, b):
    """x with a x = b (3 x 3, Cramer)."""
    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    d = det(a)
    x = []
    for c in range(3):
        m = [[b[r] if k == c else a[r][k] for k in range(3)] for r in range(3)]
        x.append(det(m) / d)
    return x


def main():
    scenario = json.loads(Path(sys.argv[1]).read_text())
    mode, out = sys.argv[2], Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    part = scenario[mode]
    windows, end_s = part["windows"], part["end_s"]
    home, k_r = scenario["home"], scenario["k_robot"]
    rng = random.Random(5)

    def window(t):
        for w in windows:
            if w["t0"] <= t < w["t1"]:
                return w
        return windows[-1] if t >= windows[-1]["t1"] else windows[0]

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    offsets = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    offset_port, state_port = scenario.get("robot_ports", [50900, 50910])
    offsets.bind((LOCAL, offset_port))
    offsets.setblocking(False)
    listeners = {}
    if mode == "run":
        slave_port = scenario.get("slave_port", 4500)
        for port in (50510, slave_port):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind((LOCAL, port))
            s.setblocking(False)
            listeners[port] = s
    feat_file = open(out / "sim_features.csv", "w")
    out_files = {50510: open(out / "sim_monitor.csv", "w"), slave_port: open(out / "sim_slave.csv", "w")} \
        if listeners else {}

    target, target_t, offset = [0.0, 0.0, 0.0], -1.0, [0.0, 0.0, 0.0]
    t0 = time.perf_counter()
    last = {"board": -1.0, "imu": -1.0, "robot": -1.0}
    counter, prev = 0, 0.0
    while True:
        t = time.perf_counter() - t0
        if t > end_s:
            break
        dt, prev = t - prev, t
        w = window(t)
        # offsets from MOSAIC (latest packet wins)
        while True:
            try:
                data = offsets.recv(1024)
            except BlockingIOError:
                break
            if len(data) >= 24:
                v = struct.unpack(f"<{len(data) // 8}d", data)
                target, target_t = [max(-0.02, min(0.02, x)) for x in v[:3]], t
        goal = target if t - target_t < 0.25 else [0.0, 0.0, 0.0]
        offset = [o + max(-0.1 * dt, min(0.1 * dt, g - o)) for o, g in zip(offset, goal)]
        if t - last["robot"] >= 0.002:
            last["robot"] = t
            a = [[k_r * (r == c) + w["K_hand"][r][c] for c in range(3)] for r in range(3)]
            dx = solve3(a, [k_r * o for o in offset])
            x = [home[i] + dx[i] + rng.gauss(0, 5e-6) for i in range(3)]
            xd = [home[i] + offset[i] for i in range(3)]
            f_ext = [sum(w["K_hand"][r][c] * dx[c] for c in range(3)) for r in range(3)]  # FR3: robot on hand
            tx.sendto(struct.pack("<15d", t, *x, *xd, *f_ext, 0.0, 1.0, 0.0, 0.0, k_r), (LOCAL, state_port))
        if t - last["board"] >= 0.2:
            last["board"] = t
            counter += 1
            v = [2.0, float(counter), 0.0, 1.0, 50.0, 48.0, 52.0, 47.0, 0, 0, 0, 0]
            for (ci, si), ph in (((10, 11), w["phase_deg"][0]), ((8, 9), w["phase_deg"][1])):
                p = math.radians(ph + rng.gauss(0, 1.5))
                v[ci], v[si] = math.cos(p), math.sin(p)
            tx.sendto(struct.pack("<12d", *v), (LOCAL, 50500))
            feat_file.write(f"{t:.4f}," + ",".join(repr(x) for x in v) + "\n")
        if t - last["imu"] >= 0.02:
            last["imu"] = t
            tx.sendto(struct.pack("<8f", *w["imu"]), (LOCAL, 50700))
        for port, s in listeners.items():
            while True:
                try:
                    data = s.recv(1024)
                except BlockingIOError:
                    break
                vals = struct.unpack(f"<{len(data) // 8}d", data)
                out_files[port].write(f"{t:.4f}," + ",".join(repr(x) for x in vals) + "\n")
        time.sleep(0.0005)
    for f in [feat_file, *out_files.values()]:
        f.close()
    print(f"simulation ({mode}) done after {end_s:.0f} s")


if __name__ == "__main__":
    main()

"""Virtual operator, board, robot and IMUs for the headless session test (standard library only).

python simulate_session.py <scenario.json> calibration|run <out_dir> [<hand_file>]

Plays the time windows of scenario.json (make_scenario.py) from its start:
  board   5 Hz, 12 doubles to 127.0.0.1:50500 (esp32_robot_stiffness features; pair phases of the window + 1.5 deg
          noise; [8, 9] biceps cos/sin, [10, 11] triceps)
  IMU     50 Hz, 8 floats to 127.0.0.1:50700 (upper-arm and forearm quaternions of the window's posture)
  hand    (calibration) the window's hand stiffness K_h (9 numbers, N/m) and rotational hand stiffness K_hr (9
          numbers, Nm/rad) written to <hand_file>, which the perturbation program built against the libfranka stub
          (test/robot_stub) reads
  outputs (run) records the pipeline's output: port 50510 (StiffnessProjection, 12 doubles) and 4500 (translational
          and rotational stiffness commands, 2 doubles; scenario "slave_port")
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


def main():
    scenario = json.loads(Path(sys.argv[1]).read_text())
    mode, out = sys.argv[2], Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    part = scenario[mode]
    windows, end_s = part["windows"], part["end_s"]
    hand_file = Path(sys.argv[4]) if len(sys.argv) > 4 else None
    rng = random.Random(5)

    def window(t):
        for w in windows:
            if w["t0"] <= t < w["t1"]:
                return w
        return windows[-1] if t >= windows[-1]["t1"] else windows[0]

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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

    hand = None
    t0 = time.perf_counter()
    last = {"board": -1.0, "imu": -1.0}
    counter = 0
    while True:
        t = time.perf_counter() - t0
        if t > end_s:
            break
        w = window(t)
        if hand_file is not None and hand != (w["K_hand"], w.get("K_hand_rot")):
            hand = (w["K_hand"], w.get("K_hand_rot"))
            tmp = hand_file.with_suffix(".tmp")
            tmp.write_text(" ".join(repr(v) for k in hand if k is not None for row in k for v in row) + "\n")
            tmp.replace(hand_file)
        if t - last["board"] >= 0.2:
            last["board"] = t
            counter += 1
            v = [2.0, float(counter), 0.0, 1.0, 50.0, 48.0, 52.0, 47.0, 0, 0, 0, 0]
            for (ci, si), ph in (((8, 9), w["phase_deg"][0]), ((10, 11), w["phase_deg"][1])):
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

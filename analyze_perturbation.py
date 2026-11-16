"""Hand stiffness per trial from a perturbation.cpp log, next to the elastography features of the same trial.

  python analyze_perturbation.py <log_dir> [--hold <from> <to>] [--sensors <laptop_dir>/sensors_laptop.csv]

For every trial (trials.csv) the steps are cut from robot.csv (step onset = first ramp-up sample), dx_d and dx are
taken as hold-window mean minus pre-window mean (hold window: from ramp + 0.45 s to ramp + hold - 0.05 s after the
onset, from the trial's ramp_s and hold_s; 0.6-1.1 s for the 15 mm steps, 1.45-1.95 s for the 5 cm pushes), and two
3x3 estimates are fitted:
  spring : K_h dx = K_r (dx_d - dx), from positions and the commanded K_r only
  wrench : K = -dF/dx, F = -O_F_ext_hat_K (force of the hand on the robot); a trial labelled "empty" (nobody
           holding) is subtracted per direction
Rotation trials (kind "rotation", labels rot_...) give the rotational 3x3 stiffness in Nm/rad the same way, with the
handle's rotation vector (base frame, relative to the start of the trial) for dx and the moments for F; "rot_empty"
is their empty trial. Their columns are named r.._spring / r.._wrench instead of k.._spring / k.._wrench.
The sensor packets of the trial give the mean phase and amplitudes of both pairs: from sensors.csv (same clock as the
robot), or with --sensors from the laptop log, matched by Unix time (each side corrected by its measured offset to
internet time: clock.json next to the laptop log, --robot-clock-offset for the lab PC) and trimmed by --trim s at
both ends of the trial.
Writes <log_dir>/summary.csv and prints one line per trial.
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from stiffness_identification import stiffness_from_spring, stiffness_from_wrench

PRE_S = (-0.3, -0.05)
RAMP_UP = 2
# esp32_robot_stiffness output: [8, 9] cos/sin of pair ch0->ch1, [10, 11] pair ch2->ch3. Mounting of 11-05:
# clip A (ch0, ch1) on the triceps, clip B (ch2, ch3) on the biceps.
PAIRS = {"triceps": (8, 9), "biceps": (10, 11)}
AMPS = slice(4, 8)


def load_robot(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    return data


def load_sensors(path):
    rows = []
    with open(path) as f:
        next(f)
        for line in f:
            rows.append([float(x) for x in line.split(",")])
    width = min(len(r) for r in rows) if rows else 0
    return np.array([r[:width] for r in rows])


def window(t, y, t0, t1):
    sel = (t >= t0) & (t < t1)
    if not sel.any():
        raise ValueError(f"no samples in [{t0:.2f}, {t1:.2f})")
    return y[sel].mean(axis=0)


def rotation_vectors(q, q_ref):
    """Rotation vector (base frame) of q * q_ref^-1 for quaternions (w, x, y, z) in rows."""
    w0, v0 = q_ref[0], -q_ref[1:]  # conjugate of the reference
    w, v = q[:, 0], q[:, 1:]
    rw = w * w0 - v @ v0
    rv = w[:, None] * v0 + w0 * v + np.cross(v, v0)
    rv[rw < 0] *= -1.0
    rw = np.abs(rw)
    n = np.linalg.norm(rv, axis=1)
    angle = 2.0 * np.arctan2(n, rw)
    scale = np.where(n > 1e-12, angle / np.maximum(n, 1e-12), 2.0)
    return rv * scale[:, None]


def trial_steps(robot, trial, hold_s, rotation=False):
    sel = robot["trial"] == trial
    t = robot["t_s"][sel]
    phase = robot["phase"][sel]
    step = robot["step"][sel]
    # On the FR3, O_F_ext_hat_K is the force the robot applies to the hand (first lab run 11-12: all fits came out
    # negative, including the empty trial), so it is negated to get the force of the hand on the robot.
    if rotation:
        xd = np.column_stack([robot[c][sel] for c in ("rdx", "rdy", "rdz")])
        q = np.column_stack([robot[c][sel] for c in ("qw", "qx", "qy", "qz")])
        x = rotation_vectors(q, q[0]) if q.size else np.empty((0, 3))
        f = -np.column_stack([robot[c][sel] for c in ("mx", "my", "mz")])
    else:
        xd = np.column_stack([robot[c][sel] for c in ("xd", "yd", "zd")])
        x = np.column_stack([robot[c][sel] for c in ("x", "y", "z")])
        f = -np.column_stack([robot[c][sel] for c in ("fx", "fy", "fz")])
    axis = robot["axis"][sel]
    sign = robot["sign"][sel]
    # unit vector of the step (logs from 11-16 on; diagonal pushes have axis 6)
    unit = (np.column_stack([robot[c][sel] for c in ("dirx", "diry", "dirz")])
            if "dirx" in robot.dtype.names else np.zeros((t.size, 3)))
    out = []
    for s in np.unique(step[step >= 0]):
        idx = np.flatnonzero((step == s) & (phase == RAMP_UP))
        if idx.size == 0:
            continue
        onset = t[idx[0]]
        d = {}
        for name, y in (("dx_d", xd), ("dx", x), ("df", f)):
            d[name] = window(t, y, onset + hold_s[0], onset + hold_s[1]) - window(t, y, onset + PRE_S[0],
                                                                                  onset + PRE_S[1])
        d["direction"] = (int(axis[idx[0]]), int(sign[idx[0]]), *np.round(unit[idx[0]], 2).tolist())
        out.append(d)
    return out, (t[0], t[-1]) if t.size else (np.nan, np.nan)


def sensor_summary(sensors, t0, t1):
    if sensors.size == 0:
        return {}
    sel = (sensors[:, 0] >= t0) & (sensors[:, 0] < t1)
    rows = sensors[sel, 1:]
    out = {"sensor_packets": int(sel.sum())}
    if not sel.any():
        return out
    for name, (c, s) in PAIRS.items():
        z = rows[:, c] + 1j * rows[:, s]
        z = z[np.abs(z) > 0]
        if z.size:
            m = z.mean()
            out[f"{name}_phase_deg"] = float(np.degrees(np.angle(m)))
            out[f"{name}_phase_sd_deg"] = float(np.degrees(np.sqrt(-2 * np.log(min(abs(m), 1 - 1e-12)))))
            out[f"{name}_sent"] = float(z.size / sel.sum())
    for k, a in enumerate(np.median(rows[:, AMPS], axis=0)):
        out[f"amp_ch{k}"] = float(a)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log_dir", type=Path)
    ap.add_argument("--hold", type=float, nargs=2, default=None,
                    help="hold window after step onset, s (default from the trial's ramp and hold)")
    ap.add_argument("--sensors", type=Path, help="laptop sensor log (laptop_sensor_logger.py) instead of sensors.csv")
    ap.add_argument("--robot-clock-offset", type=float, default=0.0, help="lab PC clock minus internet time, s")
    ap.add_argument("--trim", type=float, default=3.0, help="s cut from both ends of a trial for the sensor mean")
    args = ap.parse_args()

    robot = load_robot(args.log_dir / "robot.csv")
    laptop_offset = 0.0
    if args.sensors:
        sensors = load_sensors(args.sensors)
        clock_file = args.sensors.parent / "clock.json"
        if clock_file.exists():
            clock = json.loads(clock_file.read_text())
            offsets = [clock[k]["offset_s"] for k in ("start", "end") if clock.get(k, {}).get("offset_s") is not None]
            laptop_offset = float(np.mean(offsets)) if offsets else 0.0
        print(f"laptop sensor log, clock offsets: laptop {laptop_offset:+.3f} s, robot PC {args.robot_clock_offset:+.3f} s")
    elif (args.log_dir / "sensors.csv").exists():
        sensors = load_sensors(args.log_dir / "sensors.csv")
    else:
        sensors = np.empty((0, 0))
        print("no sensor log: pass --sensors <laptop_dir>/sensors_laptop.csv")
    with open(args.log_dir / "trials.csv") as f:
        trials = list(csv.DictReader(f))

    for tr in trials:
        tr["kind"] = tr.get("kind") or "translation"  # logs before 11-13 have translation trials only
    def hold_window(tr):
        # rotation steps have a longer ramp and hold: from 0.7 s after the ramp to just before the ramp down
        if tr["kind"] == "rotation":
            return float(tr["ramp_s"]) + 0.7, float(tr["ramp_s"]) + float(tr["hold_s"]) - 0.05
        if args.hold is not None:
            return args.hold
        ramp, hold = float(tr.get("ramp_s") or 0.15), float(tr.get("hold_s") or 1.0)
        return ramp + 0.45, ramp + hold - 0.05

    steps = {int(tr["trial"]): trial_steps(robot, int(tr["trial"]), hold_window(tr), tr["kind"] == "rotation")
             for tr in trials}
    empty = {}  # keyed by step direction; rotation directions have axis 3..5, so the two kinds do not mix
    for tr in trials:
        if tr["label"] in ("empty", "rot_empty"):
            for d in steps[int(tr["trial"])][0]:
                empty.setdefault(d["direction"], []).append(d["df"])
    empty = {k: np.mean(v, axis=0) for k, v in empty.items()}

    summary = []
    for tr in trials:
        n = int(tr["trial"])
        st, (t0, t1) = steps[n]
        rotation = tr["kind"] == "rotation"
        p = "r" if rotation else "k"
        row = {"trial": n, "label": tr["label"], "kind": tr["kind"], "steps": len(st)}
        if len(st) >= 3:
            dx_d = np.array([d["dx_d"] for d in st])
            dx = np.array([d["dx"] for d in st])
            df = np.array([d["df"] for d in st])
            k_robot = float(tr["k_robot_nm_per_rad"] if rotation else tr["k_robot_n_per_m"])
            k_spring, r2_spring = stiffness_from_spring(dx_d, dx, k_robot)
            df_empty = (np.array([empty[d["direction"]] for d in st])
                        if tr["label"] not in ("empty", "rot_empty") and all(d["direction"] in empty for d in st)
                        else None)
            k_wrench, r2_wrench = stiffness_from_wrench(dx, df, df_empty)
            row.update({"r2_spring": r2_spring, "r2_wrench": r2_wrench, "empty_subtracted": df_empty is not None})
            for name, k in (("spring", k_spring), ("wrench", k_wrench)):
                for i, a in enumerate("xyz"):
                    row[f"{p}{a}{a}_{name}"] = k[i, i]
                row[f"{p}xy_{name}"], row[f"{p}xz_{name}"], row[f"{p}yz_{name}"] = k[0, 1], k[0, 2], k[1, 2]
                row[f"{p}_volume_{name}"] = float(np.cbrt(max(np.linalg.det(k), 0.0)))
        if args.sensors:
            # laptop time of a robot-PC Unix time: remove the robot PC's offset, add the laptop's
            shift = laptop_offset - args.robot_clock_offset
            t0, t1 = float(tr["start_unix_s"]) + shift, float(tr["end_unix_s"]) + shift
        elif "start_s" in tr and tr.get("end_s"):
            t0, t1 = float(tr["start_s"]), float(tr["end_s"])
        row.update(sensor_summary(sensors, t0 + args.trim, t1 - args.trim))
        summary.append(row)
        unit, fmt = ("Nm/rad", ".1f") if rotation else ("N/m", ".0f")
        ks = (f"spring diag {row[f'{p}xx_spring']:{fmt}}/{row[f'{p}yy_spring']:{fmt}}/{row[f'{p}zz_spring']:{fmt}} "
              f"{unit} (R2 {row['r2_spring']:.2f}), wrench {row[f'{p}xx_wrench']:{fmt}}/{row[f'{p}yy_wrench']:{fmt}}/"
              f"{row[f'{p}zz_wrench']:{fmt}}" if f"{p}xx_spring" in row else "too few steps")
        ph = ", ".join(f"{p} {row[f'{p}_phase_deg']:.0f} deg" for p in PAIRS if f"{p}_phase_deg" in row)
        print(f"trial {n} {tr['label']}: {len(st)} steps, {ks} | sensors {row.get('sensor_packets', 0)} packets {ph}")

    keys = list(dict.fromkeys(k for r in summary for k in r))
    with open(args.log_dir / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(summary)


if __name__ == "__main__":
    main()

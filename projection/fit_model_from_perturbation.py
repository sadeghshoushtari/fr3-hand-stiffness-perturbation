"""Per-session calibration of the muscle-space model (stiffness_model.StiffnessModel) from a perturbation session.

  python fit_model_from_perturbation.py <log_dir>/summary.csv [--out <log_dir>/stiffness_model.json]
        [--upper-arm-deg 0] [--elbow-deg 90] [--axes x] [--single-index biceps]
        [--index phase|speed2 --drive-hz 45 --spacing-m 0.02 --lag-offset-deg 0]

summary.csv is written by analyze_perturbation.py (translation trials). Every trial except the empty one
becomes a calibration sample: the posture (held fixed during the session), the two elastography indices scaled to
0..1, and the hand stiffness of the spring estimate with the empty trial's stiffness subtracted. The group parameters
(a_g, b_g) of the model are fitted to the chosen Cartesian axes only (--axes; default x: at the handle, y was at the
resolution floor and z runs along the grip in the 11-12 runs). A leave-one-out check against two baselines (mean of
the other trials; K linear in the biceps index) is printed, then the model fitted on all trials is saved:
stiffness_model.json, and stiffness_projection.json for the MOSAIC block StiffnessProjection (mosaic_export.py).

Indices (per sensor pair, relative to the session's relaxed trials):
  The pair phase is angle(Z_far / Z_near) of the robot firmware's lock-in (reference e^{-i w t}), i.e. minus the
  travel lag: a faster wave in a stiffer muscle raises it.
  phase  : phase change, sign flipped (the sign that rose with the measured
           stiffness in the 11-12 runs at 80 Hz, where the phase fell on contraction, opposite to a travel-time change;
           so at 80 Hz the phase change is not travel time alone)
  speed2 : (c / c_relaxed)^2 - 1 with the wave speed c = 360 f d / lag, lag = (offset - phase) mod 360 deg. Shear
           modulus mu = rho c^2 grows about linearly with muscle force (Nordez & Hug 2010; Bouillard et al. 2011),
           so this index is the relative change of the modulus. Needs a lag below 360 deg (no wrapping): at 20 mm
           spacing and c >= 2 m/s, a drive of 45 Hz or lower. --lag-offset-deg corrects a known sensor/mounting offset.
Posture: upper arm angle forward of vertical and elbow angle, operator facing the robot (forward = robot -x, up = +z).
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from arm_model import posture_from_directions
from mosaic_export import export as export_mosaic
from stiffness_model import StiffnessModel, Trial

PAIRS = ("biceps", "triceps")
AXES = {"x": 0, "y": 1, "z": 2}
TORSO = np.column_stack([[-1.0, 0, 0], [0, 0, 1.0], [0, 1.0, 0]])  # anterior, up, right in the robot base frame


def wrap_deg(a):
    return (np.asarray(a, dtype=float) + 180.0) % 360.0 - 180.0


def session_posture(upper_arm_deg, elbow_deg):
    t, e = np.radians(upper_arm_deg), np.radians(elbow_deg)
    return posture_from_directions(np.array([-np.sin(t), 0, -np.cos(t)]),
                                   np.array([-np.sin(t + e), 0, -np.cos(t + e)]), torso=TORSO)


def kmat(r):
    g = lambda k: float(r[f"k{k}_spring"])
    return np.array([[g("xx"), g("xy"), g("xz")], [g("xy"), g("yy"), g("yz")], [g("xz"), g("yz"), g("zz")]])


def circ_mean(deg):
    return float(np.degrees(np.angle(np.mean(np.exp(1j * np.radians(deg))))))


def indices(rows, kind, drive_hz, spacing_m, lag_offset_deg):
    """Raw index per pair and trial (0 at the relaxed trials)."""
    out = {}
    for pair in PAIRS:
        phase = np.array([float(r[f"{pair}_phase_deg"]) for r in rows])
        relaxed = [p for p, r in zip(phase, rows) if r["label"] == "relaxed"]
        if not relaxed:
            raise ValueError("no 'relaxed' trial to reference the indices to")
        if kind == "phase":
            out[pair] = -wrap_deg(phase - circ_mean(relaxed))
        else:
            lag = (lag_offset_deg - phase) % 360.0
            lag_relaxed = (lag_offset_deg - circ_mean(relaxed)) % 360.0
            speed = 360.0 * drive_hz * spacing_m / np.maximum(lag, 1.0)
            c_relaxed = 360.0 * drive_hz * spacing_m / max(lag_relaxed, 1.0)
            out[pair] = (speed / c_relaxed) ** 2 - 1.0
    return out


def sigmas(raw, scale):
    """Index scaled to 0..1 by the training trials' largest value, negative values clipped to 0."""
    return [{p: max(float(raw[p][i]), 0.0) / scale[p] for p in PAIRS} for i in range(len(raw[PAIRS[0]]))]


def fit(trials_k, sig, posture, weight):
    trials = [Trial(posture, s, stiffness=k, weight=weight) for k, s in zip(trials_k, sig)]
    return StiffnessModel().fit(trials)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("summary", type=Path)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--upper-arm-deg", type=float, default=0.0, help="upper arm forward of vertical")
    ap.add_argument("--elbow-deg", type=float, default=90.0, help="elbow flexion, 0 = straight")
    ap.add_argument("--axes", default="x", help="Cartesian axes fitted, e.g. x or xz")
    ap.add_argument("--single-index", choices=PAIRS, help="drive both indices with one pair (other pair unusable)")
    ap.add_argument("--index", choices=("phase", "speed2"), default="phase")
    ap.add_argument("--drive-hz", type=float, default=80.0)
    ap.add_argument("--spacing-m", type=float, default=0.02)
    ap.add_argument("--lag-offset-deg", type=float, default=0.0)
    args = ap.parse_args()

    with open(args.summary) as f:
        rows = [r for r in csv.DictReader(f) if (r.get("kind") or "translation") == "translation" and r.get("kxx_spring")]
    empty = [kmat(r) for r in rows if r["label"] == "empty"]
    k_empty = np.mean(empty, axis=0) if empty else np.zeros((3, 3))
    rows = [r for r in rows if r["label"] != "empty" and r.get("biceps_phase_deg") and r.get("triceps_phase_deg")]
    labels = [r["label"] for r in rows]
    K = [kmat(r) - k_empty for r in rows]
    raw = indices(rows, args.index, args.drive_hz, args.spacing_m, args.lag_offset_deg)
    if args.single_index:
        raw = {p: raw[args.single_index] for p in PAIRS}
    for p in PAIRS:
        if np.max(raw[p]) <= 0:
            print(f"warning: the {p} index never rises above relaxed; with --index speed2 this means the lag wraps "
                  f"or the offset is wrong (lower --drive-hz, check --lag-offset-deg)")
    posture = session_posture(args.upper_arm_deg, args.elbow_deg)
    ax = [AXES[a] for a in args.axes]
    weight = np.zeros((3, 3))
    weight[np.ix_(ax, ax)] = 1.0

    pred = {"mean": [], "linear": [], "model": []}
    for i in range(len(rows)):
        keep = [j for j in range(len(rows)) if j != i]
        scale = {p: max(max(raw[p][j] for j in keep), 1e-9) for p in PAIRS}
        s_all = sigmas(raw, scale)
        pred["mean"].append(np.mean([K[j] for j in keep], axis=0))
        b = np.array([raw["biceps"][j] for j in keep])
        coef = np.linalg.lstsq(np.column_stack([np.ones_like(b), b]), np.array([K[j].ravel() for j in keep]),
                               rcond=None)[0]
        pred["linear"].append((np.array([1.0, raw["biceps"][i]]) @ coef).reshape(3, 3))
        try:
            m = fit([K[j] for j in keep], [s_all[j] for j in keep], posture, weight)
            pred["model"].append(m.cartesian_stiffness(posture, s_all[i]))
        except np.linalg.LinAlgError:
            pred["model"].append(np.full((3, 3), np.nan))

    names = "/".join(f"K{a}{a}" for a in args.axes)
    print(f"{len(rows)} trials, index {args.index}, fitted {names}; leave-one-out prediction (N/m):")
    print(f"  {'trial':20s} {'biceps':>7s} {'triceps':>7s}  " + "  ".join(f"{a + ' ' + n:>12s}" for n in pred
                                                                     for a in args.axes) + "   measured")
    for i, lab in enumerate(labels):
        vals = "  ".join(f"{pred[n][i][j, j]:12.0f}" for n in pred for j in ax)
        print(f"  {lab:20s} {raw['biceps'][i]:7.2f} {raw['triceps'][i]:7.2f}  {vals}   "
              + " ".join(f"{K[i][j, j]:.0f}" for j in ax))
    for n in pred:
        err = [np.sqrt(np.nanmean([(pred[n][i][j, j] - K[i][j, j]) ** 2 for i in range(len(rows))])) for j in ax]
        print(f"  rmse {n:7s} " + "  ".join(f"K{a}{a} {e:.0f}" for a, e in zip(args.axes, err)))

    scale = {p: max(float(np.max(raw[p])), 1e-9) for p in PAIRS}
    model = fit(K, sigmas(raw, scale), posture, weight)
    print("fitted (a, b) per group: " + ", ".join(f"{g} ({a:.3f}, {b:.3f})" for g, (a, b) in model.params.items()))
    out = args.out or args.summary.parent / "stiffness_model.json"
    model.save(out)
    saved = json.loads(out.read_text())  # the runtime needs the same index definition and scale
    saved["index"] = {"kind": args.index, "scale": scale, "single_index": args.single_index,
                      "drive_hz": args.drive_hz, "spacing_m": args.spacing_m, "lag_offset_deg": args.lag_offset_deg,
                      "posture": {"upper_arm_deg": args.upper_arm_deg, "elbow_deg": args.elbow_deg},
                      "fitted_axes": args.axes,
                      "relaxed_phase_deg": {p: circ_mean([float(r[f"{p}_phase_deg"]) for r in rows
                                                          if r["label"] == "relaxed"]) for p in PAIRS}}
    out.write_text(json.dumps(saved, indent=2))
    print(f"saved {out} (index scale {', '.join(f'{p} {scale[p]:.3g}' for p in PAIRS)})")
    mosaic = out.with_name("stiffness_projection.json")  # for the MOSAIC block StiffnessProjection
    export_mosaic(model, saved["index"], posture, mosaic)
    print(f"saved {mosaic}")


if __name__ == "__main__":
    main()

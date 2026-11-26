"""Refits logged calibrations with the two-pair hand model (hand_model.py) and compares it with straight lines.

  python refit_calibration.py <calibration_*.json> [...] [--reduction mean|volume] [--handle-offset X Y Z]
        [--ridge 0.3]

Input: session files of the PerturbationCalibration block (trials with K, K_rot and pair phases; the model it fitted,
for the index definition). Per trial: drive d = mean sigma (index of the session model), measured translational and
rotational stiffness (empty-handle trial subtracted) reduced to one number each. Printed per session:
  - the fit on the session's fitting trials (those before the first validation trial, or all), parameters and the
    measured vs modelled scalars of every trial (modelled = what the robot would measure, hand_model.measured_blocks);
  - leave-one-out over the muscle trials: each trial predicted by a model fitted on the others, error in log units
    (0.1 = 10 %) for the hand model and for a straight line k = c0 + c1 d per channel (the commands before format 3);
  - the slave commands the hand model gives the trials.
"""

import argparse
import json

import numpy as np

import mosaic_export as mx
from hand_model import RIDGE, HandModel, drive, reduce

NAMES = ("translation", "rotation")


def sym(m):
    m = np.asarray(m, float)
    return 0.5 * (m + m.T)


def sigma(model, phases):
    idx = model["index"]
    out = {}
    for p, v in zip(mx.PAIRS, phases):
        if v is None:
            return None
        d = (idx.get("direction") or {}).get(p, -1.0)
        hi = (idx.get("window_high_deg") or {}).get(p, mx.DEFAULT_WINDOW_HIGH_DEG)
        raw = mx.raw_index(v, idx["relaxed_phase_deg"][p], idx, d, hi)
        out[p] = min(max(raw, 0.0) / idx["scale"][p], idx["sigma_max"])
    if idx.get("single_index"):
        out = {p: out[idx["single_index"]] for p in mx.PAIRS}
    return out


def trials_of(session):
    trials = session["trials"]
    empty = [t for t in trials if t["code"] < -0.5 and t.get("K")]
    k_e = np.mean([sym(t["K"]) for t in empty], axis=0) if empty else np.zeros((3, 3))
    r_e = [sym(t["K_rot"]) for t in empty if t.get("K_rot")]
    r_e = np.mean(r_e, axis=0) if r_e else np.zeros((3, 3))
    out = []
    for t in trials:
        if t["code"] < -0.5 or not t.get("K"):
            continue
        s = sigma(session["model"], t["phase_deg"])
        if s is None:
            continue
        out.append({"n": t["number"], "code": t["code"], "validation": t.get("validation") is not None,
                    "d": drive(s), "Kt": sym(t["K"]) - k_e, "Kr": sym(t["K_rot"]) - r_e if t.get("K_rot") else None})
    return out


def line_predict(train, x, ch, reduction):
    pts = [(t["d"], reduce(t["Kt" if ch == 0 else "Kr"], reduction)) for t in train
           if (ch == 0 or t["Kr"] is not None)]
    pts = [(d, v) for d, v in pts if np.isfinite(v)]
    if len(pts) < 2 or np.ptp([d for d, _ in pts]) < 1e-9:
        return float("nan")
    c1, c0 = np.polyfit([d for d, _ in pts], [v for _, v in pts], 1)
    return float(c0 + c1 * x)


def fitted(train, args):
    m = HandModel(reduction=args.reduction, handle_offset=args.handle_offset)
    return m.fit([(t["d"], t["Kt"], t["Kr"]) for t in train], ridge=args.ridge)


def log_err(pred, meas):
    return float(np.log(pred / meas)) if np.isfinite(pred) and pred > 0 and np.isfinite(meas) and meas > 0 \
        else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--reduction", choices=("volume", "mean"), default="mean")
    ap.add_argument("--handle-offset", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("X", "Y", "Z"))
    ap.add_argument("--ridge", type=float, default=RIDGE)
    args = ap.parse_args()
    summary = []
    for path in args.sessions:
        session = json.load(open(path))
        trials = trials_of(session)
        print(f"\n=== {path.split('/')[-1]}: {len(trials)} muscle trials with both phases, reduction {args.reduction}, "
              f"handle offset {list(args.handle_offset)} m")
        if len(trials) < 3:
            print("  fewer than 3 usable trials: skipped")
            continue
        first_val = next((i for i, t in enumerate(trials) if t["validation"]), len(trials))
        train = trials[:first_val] if first_val >= 2 else trials
        m = fitted(train, args)
        print("  fit on trials " + " ".join(str(t["n"]) for t in train) + ": "
              + ", ".join(f"{k} {v:.4g}" for k, v in m.params.items()) + f", cost {m.fit_report['cost']:.3g}")
        relaxed = [t["d"] for t in train if abs(t["code"]) < 1e-9]
        top = [t["d"] for t in train if t["code"] >= max(u["code"] for u in train) - 1e-9]
        m.set_commands(np.mean(relaxed) if relaxed else 0.0, np.mean(top))
        print("   #  code  val    d   | k_t measured  model | k_r measured model | operator k_t  k_r | command N/m Nm/rad")
        for t in trials:
            mt, mr = m.measured_scalars(t["d"])
            kt, kr = m.scalars(t["d"])
            meas_r = reduce(t["Kr"], args.reduction) if t["Kr"] is not None else float("nan")
            print(f"  {t['n']:2d}  {t['code']:4.1f}  {'yes' if t['validation'] else ' - '}  {t['d']:5.2f} | "
                  f"{reduce(t['Kt'], args.reduction):8.0f}  {mt:8.0f} | {meas_r:7.1f}  {mr:6.1f} | "
                  f"{kt:8.0f}  {kr:6.1f} | {m.command('translation', t['d']):6.0f}  {m.command('rotation', t['d']):5.1f}")
        # leave one out over all muscle trials
        err = {("model", c): [] for c in range(2)} | {("line", c): [] for c in range(2)}
        for i, t in enumerate(trials):
            rest = trials[:i] + trials[i + 1:]
            mi = fitted(rest, args)
            pred = mi.measured_scalars(t["d"])
            for ch in range(2):
                k = t["Kt"] if ch == 0 else t["Kr"]
                if k is None:
                    continue
                meas = reduce(k, args.reduction)
                err[("model", ch)].append(log_err(pred[ch], meas))
                err[("line", ch)].append(log_err(line_predict(rest, t["d"], ch, args.reduction), meas))
        rms = {k: float(np.sqrt(np.nanmean(np.square(v)))) if v else float("nan") for k, v in err.items()}
        print("  leave-one-out rms error (log units): " + ", ".join(
            f"{NAMES[ch]} model {rms[('model', ch)]:.2f} line {rms[('line', ch)]:.2f}" for ch in range(2)))
        summary.append((path.split("/")[-1], rms))
    if len(summary) > 1:
        print("\nsummary, leave-one-out rms log error (translation model / line, rotation model / line):")
        for name, rms in summary:
            print(f"  {name}: {rms[('model', 0)]:.2f} / {rms[('line', 0)]:.2f}, "
                  f"{rms[('model', 1)]:.2f} / {rms[('line', 1)]:.2f}")


if __name__ == "__main__":
    main()

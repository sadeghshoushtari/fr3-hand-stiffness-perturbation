"""Reference data for the C# check of StiffnessProjectionModel (Program.cs in this folder).

  python make_reference.py <out_dir> [--model <stiffness_projection.json> --sensors <sensors.csv>]

Writes per case <case>_model.json, <case>_features.csv and <case>_expected.csv (projection/mosaic_export.py run packet
by packet, the reference of the block):
  log     a fitted session model (--model) on the recorded feature vectors of that session (--sensors), if given
  speed2  the prior model with a 45 Hz wave-speed index on synthetic packets: wave speeds 1.5-12 m/s through the
          20 mm pair, random phases, packets without phase (0, 0) and short vectors
  phase   the same packets with the phase index, single index on the biceps
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

# projection/ (mosaic_export.py and the model) sits in a parent folder of this one
sys.path.insert(0, str(next(p / "projection" for p in Path(__file__).resolve().parents
                            if (p / "projection" / "mosaic_export.py").exists())))
import mosaic_export as mx  # noqa: E402
from fit_model_from_perturbation import session_posture  # noqa: E402
from stiffness_model import PRIOR, StiffnessModel  # noqa: E402


def write(out, case, data, packets):
    proj = mx.Projection(data)
    (out / f"{case}_model.json").write_text(json.dumps(data))
    with open(out / f"{case}_features.csv", "w", newline="") as f:
        w = csv.writer(f)
        for p in packets:
            w.writerow([repr(float(x)) for x in p])
    with open(out / f"{case}_expected.csv", "w", newline="") as f:
        w = csv.writer(f)
        for p in packets:
            w.writerow([repr(float(x)) for x in proj.step(p)])
    print(f"{case}: {len(packets)} packets")


def synthetic(rng, n=2000, f=45.0, d=0.02):
    packets = []
    for i in range(n):
        v = np.zeros(12)
        v[0], v[1] = 2, i
        for pair in ("biceps", "triceps"):
            c, s = mx.FEATURES[pair]
            r = rng.uniform()
            if r < 0.6:  # travel phase of a wave at speed c through the pair: phase = -lag
                deg = -360.0 * f * d / rng.uniform(1.5, 12.0)
            elif r < 0.9:
                deg = rng.uniform(-180.0, 180.0)
            else:
                continue  # no phase
            v[c], v[s] = np.cos(np.radians(deg)), np.sin(np.radians(deg))
        packets.append(v[:rng.integers(8, 11)] if rng.uniform() < 0.02 else v)
    return packets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--model", type=Path)
    ap.add_argument("--sensors", type=Path)
    args = ap.parse_args()
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    if args.model and args.sensors:
        rows = np.genfromtxt(args.sensors, delimiter=",", skip_header=1)
        write(out, "log", mx.load(args.model), [r[1:] for r in rows])
    rng = np.random.default_rng(7)
    packets = synthetic(rng)
    model = StiffnessModel(params={g: (a * 1.2, b) for g, (a, b) in PRIOR.items()})
    posture = session_posture(0.0, 90.0)
    relaxed = -360.0 * 45.0 * 0.02 / 2.5  # 2.5 m/s relaxed wave
    for case, kind, single, scale in (("speed2", "speed2", None, {"biceps": 4.0, "triceps": 3.0}),
                                      ("phase", "phase", "biceps", {"biceps": 60.0, "triceps": 60.0})):
        idx = {"kind": kind, "drive_hz": 45.0, "spacing_m": 0.02, "lag_offset_deg": 0.0, "single_index": single,
               "scale": scale, "relaxed_phase_deg": {"biceps": relaxed, "triceps": relaxed + 10.0}}
        data = mx.export(model, idx, posture, out / f"{case}_model.json")
        write(out, case, data, packets)


if __name__ == "__main__":
    main()

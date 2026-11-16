"""Reference fits from projection/stiffness_model.py for FitCheck (mosaic/blocks/test/fit).

python make_fit_reference.py <out.json>

Synthetic stiffness trials (model with known parameters + 5 % noise, sigmas 0..1, two postures) fitted by
StiffnessModel.fit; also the identification K = dF dx^T (dx dx^T)^-1 on random perturbations.
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "projection"))
from arm_model import ArmModel  # noqa: E402
from fit_model_from_perturbation import session_posture  # noqa: E402
from stiffness_model import StiffnessModel, Trial, identified_stiffness  # noqa: E402

rng = np.random.default_rng(11)
arm = ArmModel(0.31, 0.27, 0.07)
truth = {"biceps": (0.03, 0.40), "triceps": (0.02, 0.20), "elbow_flexors": (0.015, 0.30), "shoulder": (0.02, 0.10)}
true_model = StiffnessModel(arm, params=truth)
cases = []
for name, axes in (("xyz", "xyz"), ("x", "x")):
    w = np.zeros((3, 3))
    ax = ["xyz".index(a) for a in axes]
    w[np.ix_(ax, ax)] = 1.0
    trials = []
    for i in range(8):
        angles = (0.0, 90.0) if i % 2 == 0 else (20.0, 70.0)
        p = session_posture(*angles)
        s = {"biceps": float(rng.uniform(0, 1)), "triceps": float(rng.uniform(0, 1))}
        k = true_model.cartesian_stiffness(p, s)
        k = k * (1 + 0.05 * rng.normal(size=(3, 3)))
        k = 0.5 * (k + k.T)
        trials.append((angles, s, k))
    fitted = StiffnessModel(arm).fit([Trial(session_posture(*a), s, stiffness=k, weight=w) for a, s, k in trials])
    x = [v for g in fitted.params for v in fitted.params[g]]
    cases.append({"name": name, "weight": w.tolist(),
                  "trials": [{"angles": a, "sigma": [s["biceps"], s["triceps"]], "K": k.tolist()} for a, s, k in trials],
                  "x": x, "cost": fitted.fit_report["cost"]})
dx = rng.normal(0, 0.01, (3, 6))
df = rng.normal(0, 3, (3, 6))
out = {"arm": [arm.upper_arm_m, arm.forearm_m, arm.hand_m], "cases": cases,
       "identify": {"dx": dx.T.tolist(), "df": df.T.tolist(), "K": identified_stiffness(dx, df).tolist()}}
Path(sys.argv[1]).write_text(json.dumps(out))
for c in cases:
    print(c["name"], "x", np.round(c["x"], 4), "cost", c["cost"])

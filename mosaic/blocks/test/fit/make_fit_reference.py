"""Reference fits from projection/stiffness_model.py for FitCheck (mosaic/blocks/test/fit).

python make_fit_reference.py <out.json> [<calibration_*.json> ...]

Synthetic stiffness trials (model with known parameters + 5 % noise, sigmas 0..1, two postures) fitted by
StiffnessModel.fit; also the identification K = dF dx^T (dx dx^T)^-1 on random perturbations. Hand model
(hand_model.py): synthetic translational and rotational trials (known parameters + 5 % noise, one trial without
rotation) fitted by HandModel.fit; and, for each session file given, the hand-model fit of refit_calibration.py on the
session's fitting trials, which PerturbationCalibrationCore.Replay must reproduce.
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "projection"))
from arm_model import ArmModel  # noqa: E402
from fit_model_from_perturbation import session_posture  # noqa: E402
from stiffness_model import StiffnessModel, Trial, identified_stiffness  # noqa: E402
from hand_model import HandModel  # noqa: E402
import refit_calibration  # noqa: E402

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
hand_cases = []
for name, reduction, offset in (("hand_mean", "mean", (0.0, 0.0, -0.1)), ("hand_volume", "volume", (0.02, 0.0, -0.12))):
    truth = HandModel(arm, params={"a0": 0.02, "b": 0.12, "s_w": 3.0, "c_r": 15.0}, reduction=reduction,
                      handle_offset=offset)
    trials = []
    for i, d in enumerate((0.0, 0.3, 0.55, 0.8, 1.0)):
        kt, kr = truth.measured(d)
        kt, kr = (0.5 * (k + k.T) for k in (k * (1 + 0.05 * rng.normal(size=(3, 3))) for k in (kt, kr)))
        trials.append((d, kt, None if i == 2 else kr))
    fitted = HandModel(arm, reduction=reduction, handle_offset=offset).fit(trials)
    hand_cases.append({"name": name, "reduction": reduction, "offset": list(offset),
                       "trials": [{"d": d, "Kt": kt.tolist(), "Kr": None if kr is None else kr.tolist()}
                                  for d, kt, kr in trials],
                       "params": [fitted.params[k] for k in ("a0", "b", "s_w", "c_r")], "cost": fitted.fit_report["cost"]})
replays = []
for path in sys.argv[2:]:
    session = json.loads(Path(path).read_text())
    trials = refit_calibration.trials_of(session)
    first_val = next((i for i, t in enumerate(trials) if t["validation"]), len(trials))
    train = trials[:first_val] if first_val >= 2 else trials
    fitted = HandModel().fit([(t["d"], t["Kt"], t["Kr"]) for t in train])
    replays.append({"path": str(Path(path).resolve()), "params": [fitted.params[k] for k in ("a0", "b", "s_w", "c_r")],
                    "cost": fitted.fit_report["cost"], "trials": [t["n"] for t in train]})
out = {"arm": [arm.upper_arm_m, arm.forearm_m, arm.hand_m], "cases": cases, "hand": hand_cases, "replays": replays,
       "identify": {"dx": dx.T.tolist(), "df": df.T.tolist(), "K": identified_stiffness(dx, df).tolist()}}
Path(sys.argv[1]).write_text(json.dumps(out))
for c in cases + hand_cases + replays:
    print(c.get("name", c.get("path")), "x", np.round(c.get("x", c.get("params")), 4), "cost", c["cost"])

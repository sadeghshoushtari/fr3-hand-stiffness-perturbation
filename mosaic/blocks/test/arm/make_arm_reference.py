"""Reference geometry from projection/arm_model.py for ArmModelCheck (mosaic/blocks/test/arm).

python make_arm_reference.py <out.json>
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "projection"))
from arm_model import ArmModel, posture_from_directions, rotvec_to_matrix  # noqa: E402
from fit_model_from_perturbation import TORSO, session_posture  # noqa: E402

rng = np.random.default_rng(3)
cases = []
for upper, fore, (t, e) in [(0.30, 0.27, (0, 90)), (0.33, 0.26, (20, 60)), (0.28, 0.29, (-10, 120))]:
    cases.append(("session", upper, fore, (t, e), session_posture(t, e)))
for _ in range(3):  # random orientations, shoulder away from the origin
    d = rotvec_to_matrix(rng.normal(0, 0.6, 3)) @ np.array([0, 0, -1.0])
    f = rotvec_to_matrix(rng.normal(0, 0.8, 3)) @ d
    p = posture_from_directions(d, f, torso=rotvec_to_matrix(rng.normal(0, 0.3, 3)), shoulder=rng.normal(0, 0.2, 3))
    cases.append(("free", 0.31, 0.27, None, p))

out = []
for kind, upper, fore, angles, p in cases:
    arm = ArmModel(upper, fore)
    out.append({"kind": kind, "upper_arm_m": upper, "forearm_m": fore, "angles_deg": angles,
                "upper_arm": p.upper_arm.tolist(), "elbow_angle": p.elbow_angle, "shoulder": p.shoulder.tolist(),
                "torso": p.torso.tolist(), "lengths": arm.muscle_lengths(p).tolist(),
                "J": arm.arm_jacobian(p).tolist(), "Jm": arm.muscle_jacobian(p).tolist(),
                "H": arm.muscle_hessians(p).tolist(), "J6": arm.hand_jacobian(p).tolist()})
Path(sys.argv[1]).write_text(json.dumps({"names": list(ArmModel().names), "cases": out}))
print(f"{len(out)} cases -> {sys.argv[1]}")

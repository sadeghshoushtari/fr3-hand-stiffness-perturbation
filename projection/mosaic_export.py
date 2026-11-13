"""Model file for the MOSAIC block StiffnessProjection (mosaic/blocks/StiffnessProjection.cs), and the same
computation in Python as the reference the block is checked against.

The session posture is fixed (no IMUs yet), so the geometry of the fitted model is evaluated once here: arm Jacobian J
(3 x 4), muscle Jacobian J_m (M x 4) and the muscle Hessians d2l/dq2 (M x 4 x 4). Per sensor packet the block then
only computes
  phase   -> index (as fit_model_from_perturbation: "phase" or "speed2", relative to the session's relaxed phase)
  sigma   =  clip(max(index, 0) / scale, 0, sigma_max)
  u_g     =  a_g + b_g sum_p w_gp sigma_p,   f = F_max u,   k = gamma f / l0
  K_j     =  J_m^T diag(k) J_m + sum_i f_i d2l_i/dq2   (no load at the hand)
  K_c     =  (J K_j^-1 J^T)^-1   in the robot base frame, N/m
Feature vector: esp32_robot_stiffness output (12 doubles); [8, 9] cos/sin of the clip A pair (triceps since 11-05),
[10, 11] clip B (biceps). A pair is unusable when its cos/sin are 0 (amplitude below the firmware threshold).
"""

import json
from pathlib import Path

import numpy as np

from stiffness_model import GAMMA

FORMAT = "stiffness_projection/1"
FEATURES = {"biceps": (10, 11), "triceps": (8, 9)}
PAIRS = ("biceps", "triceps")
OUTPUT = ("kxx", "kyy", "kzz", "kxy", "kxz", "kyz", "k_volume", "sigma_biceps", "sigma_triceps", "valid")


def export(model, index, posture, path, sigma_max=1.5, features=FEATURES):
    """Write the MOSAIC model file. model: fitted StiffnessModel; index: the "index" block that
    fit_model_from_perturbation saves (kind, scale, relaxed_phase_deg, single_index, drive_hz, spacing_m,
    lag_offset_deg); posture: the session posture."""
    if model.joint_groups:
        raise NotImplementedError("joint-level groups are not supported by the MOSAIC block")
    geo = model._geometry(posture)
    groups = [{"name": g, "members": [int(i) for i in model.members[g]], "drivers": dict(drivers),
               "a": float(model.params[g][0]), "b": float(model.params[g][1])}
              for g, (_, drivers) in model.groups.items()]
    data = {
        "format": FORMAT,
        "output": list(OUTPUT),
        "gamma": float(GAMMA),
        "muscles": list(model.arm.names),
        "f_max": model.arm.f_max.tolist(),
        "l0": model.arm.l0.tolist(),
        "J": geo["J"].tolist(),
        "Jm": geo["Jm"].tolist(),
        "H": geo["H"].tolist(),
        "groups": groups,
        "index": {"kind": index["kind"], "drive_hz": float(index["drive_hz"]), "spacing_m": float(index["spacing_m"]),
                  "lag_offset_deg": float(index["lag_offset_deg"]), "single_index": index.get("single_index"),
                  "scale": {p: float(index["scale"][p]) for p in PAIRS},
                  "relaxed_phase_deg": {p: float(index["relaxed_phase_deg"][p]) for p in PAIRS},
                  "sigma_max": float(sigma_max), "features": {p: list(features[p]) for p in PAIRS}},
        "posture": index.get("posture"),
    }
    Path(path).write_text(json.dumps(data, indent=1))
    return data


def load(path):
    return json.loads(Path(path).read_text())


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def raw_index(phase_deg, relaxed_deg, idx):
    if idx["kind"] == "phase":
        return -wrap_deg(phase_deg - relaxed_deg)
    c = 360.0 * idx["drive_hz"] * idx["spacing_m"]
    lag = (idx["lag_offset_deg"] - phase_deg) % 360.0
    lag_relaxed = (idx["lag_offset_deg"] - relaxed_deg) % 360.0
    return (max(lag_relaxed, 1.0) / max(lag, 1.0)) ** 2 - 1.0  # (c / c_relaxed)^2 - 1, c = 360 f d / lag


def sigmas(data, features):
    """sigma per pair from one feature vector, or None if a needed pair has no phase."""
    idx = data["index"]
    raw = {}
    for p in PAIRS:
        ci, si = idx["features"][p]
        if max(ci, si) >= len(features):  # short vector: no phase
            raw[p] = None
            continue
        c, s = float(features[ci]), float(features[si])
        raw[p] = raw_index(np.degrees(np.arctan2(s, c)), idx["relaxed_phase_deg"][p], idx) if c * c + s * s > 0.25 \
            else None
    if idx.get("single_index"):
        raw = {p: raw[idx["single_index"]] for p in PAIRS}
    if any(r is None for r in raw.values()):
        return None
    return {p: min(max(raw[p], 0.0) / idx["scale"][p], idx["sigma_max"]) for p in PAIRS}


def stiffness(data, sigma):
    """K_c (3 x 3, N/m) for given sigmas, or None if K_j is not positive definite."""
    f_max, l0 = np.array(data["f_max"]), np.array(data["l0"])
    u = np.zeros(len(f_max))
    for g in data["groups"]:
        u[g["members"]] = g["a"] + g["b"] * sum(w * sigma[p] for p, w in g["drivers"].items())
    f = f_max * u
    k = data["gamma"] * f / l0
    jm, J, H = np.array(data["Jm"]), np.array(data["J"]), np.array(data["H"])
    kj = jm.T @ (k[:, None] * jm) + np.einsum("i,ijk->jk", f, H)
    kj = 0.5 * (kj + kj.T)
    if np.linalg.eigvalsh(kj).min() <= 0:
        return None
    kc = np.linalg.inv(J @ np.linalg.solve(kj, J.T))
    return 0.5 * (kc + kc.T)


def output(kc, sigma, valid):
    vol = float(np.linalg.det(kc) ** (1.0 / 3.0)) if np.linalg.det(kc) > 0 else 0.0
    return [kc[0, 0], kc[1, 1], kc[2, 2], kc[0, 1], kc[0, 2], kc[1, 2], vol, sigma["biceps"], sigma["triceps"],
            1.0 if valid else 0.0]


class Projection:
    """Stateful reference of the block: an unusable packet repeats the last valid output with valid = 0 (zeros
    before the first valid packet)."""

    def __init__(self, data):
        self.data = data
        self.last = None

    def step(self, features):
        sigma = sigmas(self.data, features)
        kc = stiffness(self.data, sigma) if sigma is not None else None
        if kc is None:
            return (self.last[:-1] + [0.0]) if self.last is not None else [0.0] * len(OUTPUT)
        self.last = output(kc, sigma, True)
        return list(self.last)

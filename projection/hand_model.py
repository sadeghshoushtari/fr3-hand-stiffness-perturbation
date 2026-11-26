"""Two-pair hand stiffness model (model file format stiffness_projection/3): two elastography indices -> one drive ->
6x6 hand stiffness at the handle centre -> the slave's two commands, one translational and one rotational stiffness.

  drive     d = (sigma_biceps + sigma_triceps) / 2
  muscles   u_g = rho_g (a0 + b d) for every group of stiffness_model.GROUPS, fixed ratios rho_g (GROUP_RATIO);
            f = F_max u, k = gamma f / l0
  arm       K_arm = J_m^T diag(k) J_m + sum_i f_i d2l_i/dq2          (4 x 4: shoulder x, y, z, elbow)
  wrist     K_w = s_w diag(WRIST_PASSIVE) + c_r d I                  (3 x 3: forearm twist, wrist flexion, deviation)
  hand      K6 = (J6 diag(K_arm, K_w)^-1 J6^T)^-1                    (6 x 6 at the handle centre, robot base frame)
  commands  k_t = reduce(K6[:3, :3]), k_r = reduce(K6[3:, 3:]); reduce = "volume" (cube root of the determinant) or
            "mean" (mean of the diagonal, default); each command maps the operator's span human_low..human_high (k at the
            relaxed and at the max calibration trials) linearly onto the slave's range and holds it at the ends.

Why this form (~/reports/Two sensor pair stiffness commands.md): two indices recorded along a co-contraction line
identify one property of the hand stiffness, its overall size, and both FR3 commands are size quantities. All groups
therefore follow one common drive with fixed ratios (one drive for all muscles: Ajoudani et al. 2018, Wu et al. 2020),
posture sets the shape through the arm geometry, and the wrist and forearm get joint-level stiffness: the passive
values of Formica et al. 2012, scaled per session, plus a rise with the drive. The wrist is in series with the arm, so
it also changes the translational block. At a fixed posture both commands are monotone in d; the projection adds the
posture dependence (IMUs) and the coupling of the two blocks.

Calibration (fit): the 4 parameters (a0, b, s_w, c_r) are fitted to the robot-measured translational and rotational
stiffness of the trials (empty-handle stiffness subtracted) through their reduced scalars, in log units
(log(model / measured); with the volume this is the isotropic part of the log-Euclidean error), with a ridge pull of
weight RIDGE towards the prior in log units. Reduction "mean" is the default: in the replayed calibrations the measured lateral translation
(robot y, 30-200 N/m) and rotation about the vertical sit near the robot's noise floor, and the volume, pulled down
by the softest axis, followed that noise (leave-one-out error of the 11-19 calibration 0.67 log units with the volume,
0.28 with the mean). The model is compared with what the robot measures, not with K6 itself (measured_blocks): the
calibration robot (perturbation.cpp) moves and turns its flange, not the handle centre, and holds the other degrees of
freedom with its own springs, rotation at 10 Nm/rad during the whole trial and translation at 1000 N/m. With the
handle centre off the flange, the operator's translational stiffness adds to the measured rotational stiffness.
"""

import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from arm_model import ArmModel, skew
from stiffness_model import GAMMA, GROUPS

FORMAT3 = "stiffness_projection/3"
PAIRS = ("biceps", "triceps")
WRIST_JOINTS = ("forearm_twist", "wrist_flexion", "wrist_deviation")
# activation per unit of (a0 + b d) of each group, relative to the elbow muscles: the b ratios of stiffness_model.PRIOR
# (shoulder 0.15 / elbow 0.25); muscle size is already in F_max (MoBL-ARMS)
GROUP_RATIO = {"biceps": 1.0, "triceps": 1.0, "elbow_flexors": 1.0, "shoulder": 0.6}
# passive joint stiffness, Nm/rad, Formica et al. 2012 (10 subjects, mean of the two directions): pronation/supination
# 0.21, flexion/extension 0.79, radial/ulnar deviation 1.48
WRIST_PASSIVE = (0.21, 0.79, 1.48)
PARAMS = ("a0", "b", "s_w", "c_r")
# a0, b: stiffness_model.PRIOR of the elbow groups; s_w = 1: Formica's passive values; c_r: Nm/rad per unit drive, a
# tenth to a third of the wrist stiffness of strong co-contraction (19.5-32.1 Nm/rad, Milner et al. 1995)
PRIOR = {"a0": 0.01, "b": 0.25, "s_w": 1.0, "c_r": 10.0}
BOUNDS = {"a0": (1e-5, 1.0), "b": (1e-5, 2.0), "s_w": (1e-3, 1e3), "c_r": (1e-3, 1e3)}
# weight of the pull towards the prior, per parameter, in log units: 0.3 left the leave-one-out error of the 11-19
# calibration unchanged and lowered it in the two drifting calibrations of 11-25 (refit_calibration.py); 1.0 raised it
RIDGE = 0.3
REDUCTIONS = ("volume", "mean")
VOLUME_FLOOR = 1e-3  # volume: eigenvalues below this share of the largest are raised to it (measured blocks)
# what the calibration robot holds while it measures (perturbation.cpp): rotation at kRotationTrialStiffness / 2 =
# 10 Nm/rad during the whole trial, translation at kRobotTranslationalStiffness = 1000 N/m
ROBOT_TRANSLATION_HOLD = 1000.0
ROBOT_ROTATION_HOLD = 10.0
SLAVE_RANGE = {"translation": (300.0, 700.0), "rotation": (20.0, 40.0)}  # N/m, Nm/rad (mosaic_export)
CHANNEL = {"translation": 0, "rotation": 1}


def drive(sigma):
    return 0.5 * (float(sigma["biceps"]) + float(sigma["triceps"]))


def reduce(k, mode="volume"):
    """One number for a 3x3 stiffness block: "volume" = cube root of the determinant (geometric mean of the
    eigenvalues, the nearest isotropic stiffness in the log-Euclidean sense), eigenvalues below VOLUME_FLOOR of the
    largest raised to it; "mean" = mean of the diagonal (nearest isotropic stiffness in the Frobenius sense). NaN when
    the block has no positive eigenvalue (volume) or a trace <= 0 (mean)."""
    k = 0.5 * (np.asarray(k, float) + np.asarray(k, float).T)
    if mode == "mean":
        v = float(np.trace(k) / 3.0)
        return v if v > 0 else float("nan")
    if mode != "volume":
        raise ValueError(mode)
    w = np.linalg.eigvalsh(k)
    if w.max() <= 0:
        return float("nan")
    return float(np.prod(np.maximum(w, VOLUME_FLOOR * w.max())) ** (1.0 / 3.0))


def measured_blocks(k6, offset=np.zeros(3), hold_t=ROBOT_TRANSLATION_HOLD, hold_r=ROBOT_ROTATION_HOLD):
    """What the calibration robot measures for a hand stiffness k6 (6 x 6 at the handle centre): translational
    stiffness at the flange while the robot holds the rotation with hold_r (Nm/rad), rotational stiffness about the
    flange while it holds the position with hold_t (N/m). offset: handle centre - flange (m, base frame).

    Moving the reference point: [v_handle; w] = T [v_flange; w], T = [[I, -skew(offset)], [0, I]], K_f = T^T K6 T.
    Translation step: K_rt dx + (K_rr + hold_r) dtheta = 0, so K_t = K_tt - K_tr (K_rr + hold_r I)^-1 K_rt; rotation
    step likewise with the translation held."""
    t = np.eye(6)
    t[:3, 3:] = -skew(np.asarray(offset, float))
    kf = t.T @ k6 @ t
    ktt, ktr, krt, krr = kf[:3, :3], kf[:3, 3:], kf[3:, :3], kf[3:, 3:]
    kt = ktt - ktr @ np.linalg.solve(krr + hold_r * np.eye(3), krt)
    kr = krr - krt @ np.linalg.solve(ktt + hold_t * np.eye(3), ktr)
    return 0.5 * (kt + kt.T), 0.5 * (kr + kr.T)


class HandModel:
    def __init__(self, arm=None, upper_arm_deg=90.0, elbow_deg=90.0, params=None, reduction="mean",
                 handle_offset=(0.0, 0.0, 0.0), robot_hold=(ROBOT_TRANSLATION_HOLD, ROBOT_ROTATION_HOLD),
                 wrist_passive=WRIST_PASSIVE, group_ratio=None):
        """handle_offset: handle centre - flange of the calibration robot (m, base frame), for measured_blocks."""
        from fit_model_from_perturbation import session_posture
        if reduction not in REDUCTIONS:
            raise ValueError(f"reduction must be one of {REDUCTIONS}")
        self.arm = arm or ArmModel()
        self.upper_arm_deg, self.elbow_deg = float(upper_arm_deg), float(elbow_deg)
        self.params = dict(PRIOR if params is None else params)
        self.reduction = reduction
        self.handle_offset = np.asarray(handle_offset, float)
        self.robot_hold = tuple(float(v) for v in robot_hold)
        self.wrist_passive = np.asarray(wrist_passive, float)
        self.group_ratio = dict(GROUP_RATIO if group_ratio is None else group_ratio)
        index = {n: i for i, n in enumerate(self.arm.names)}
        self.members = {g: [index[m] for m in muscles] for g, (muscles, _) in GROUPS.items()}
        self.rho = np.zeros(len(self.arm.names))
        for g, idx in self.members.items():
            self.rho[idx] = self.group_ratio[g]
        self.commands = {}
        self.fit_report = {}
        self.set_posture(session_posture(self.upper_arm_deg, self.elbow_deg))

    def set_posture(self, posture):
        """Arm geometry at a posture: J6 and the arm joint stiffness per unit of (a0 + b d)."""
        self.j6 = self.arm.hand_jacobian(posture)
        jm, hess = self.arm.muscle_jacobian(posture), self.arm.muscle_hessians(posture)
        f = self.rho * self.arm.f_max  # muscle force per unit of (a0 + b d)
        k = GAMMA * f / self.arm.l0
        unit = jm.T @ (k[:, None] * jm) + np.einsum("i,ijk->jk", f, hess)
        self.k_arm_unit = 0.5 * (unit + unit.T)

    # --- forward model ---

    def joint_stiffness(self, d, params=None):
        p = params or self.params
        kj = np.zeros((7, 7))
        kj[:4, :4] = (p["a0"] + p["b"] * d) * self.k_arm_unit
        kj[4:, 4:] = p["s_w"] * np.diag(self.wrist_passive) + p["c_r"] * d * np.eye(3)
        return kj

    def hand_stiffness(self, d, params=None):
        """K6 (6 x 6, N/m, N/rad, Nm/rad) at the handle centre; raises LinAlgError if K_j is not positive definite."""
        kj = self.joint_stiffness(d, params)
        if np.linalg.eigvalsh(kj).min() <= 0:
            raise np.linalg.LinAlgError("joint stiffness not positive definite")
        k6 = np.linalg.inv(self.j6 @ np.linalg.solve(kj, self.j6.T))
        return 0.5 * (k6 + k6.T)

    def scalars(self, d, params=None):
        """(k_t N/m, k_r Nm/rad): reduced translational and rotational blocks of K6, the operator's stiffness the
        commands follow."""
        k6 = self.hand_stiffness(d, params)
        return reduce(k6[:3, :3], self.reduction), reduce(k6[3:, 3:], self.reduction)

    def measured(self, d, params=None):
        """(K_t, K_r) as the calibration robot would measure them (measured_blocks)."""
        return measured_blocks(self.hand_stiffness(d, params), self.handle_offset, *self.robot_hold)

    def measured_scalars(self, d, params=None):
        kt, kr = self.measured(d, params)
        return reduce(kt, self.reduction), reduce(kr, self.reduction)

    # --- commands ---

    def set_commands(self, d_relaxed, d_max, ranges=None):
        """Span of each command: the operator's k at the relaxed and at the max drive of the calibration."""
        ranges = ranges or SLAVE_RANGE
        lo, hi = self.scalars(d_relaxed), self.scalars(d_max)
        self.commands = {name: {"human_low": lo[ch], "human_high": hi[ch], "min": ranges[name][0],
                                "max": ranges[name][1], "drive_relaxed": float(d_relaxed), "drive_max": float(d_max)}
                         for name, ch in CHANNEL.items()}

    @staticmethod
    def map_command(c, k):
        if c["human_high"] <= c["human_low"]:
            return float(c["min"])
        u = float(np.clip((k - c["human_low"]) / (c["human_high"] - c["human_low"]), 0.0, 1.0))
        return float(c["min"] + (c["max"] - c["min"]) * u)

    def command(self, name, d):
        c = self.commands.get(name)
        if c is None:
            return None
        return self.map_command(c, self.scalars(d)[CHANNEL[name]])

    # --- calibration ---

    def fit(self, trials, prior=None, ridge=RIDGE):
        """trials: (d, K_t or None, K_r or None), measured, empty-handle stiffness subtracted. Fits PARAMS in log
        units; a trial channel whose reduced measurement is not positive is left out (reported)."""
        prior = dict(PRIOR if prior is None else prior)
        rows, skipped = [], []
        for i, (d, kt, kr) in enumerate(trials):
            for ch, k in ((0, kt), (1, kr)):
                if k is None:
                    continue
                v = reduce(k, self.reduction)
                if np.isfinite(v) and v > 0:
                    rows.append((i, d, ch, v))
                else:
                    skipped.append((i, ch))
        if not rows:
            raise ValueError("no trial with a usable stiffness")
        lo = np.log([BOUNDS[n][0] for n in PARAMS])
        hi = np.log([BOUNDS[n][1] for n in PARAMS])
        x_prior = np.log([prior[n] for n in PARAMS])

        def residuals(x):
            p = dict(zip(PARAMS, np.exp(x)))
            r = []
            for _, d, ch, v in rows:
                try:
                    m = self.measured_scalars(d, p)[ch]
                except np.linalg.LinAlgError:
                    m = float("nan")
                r.append(np.log(m / v) if np.isfinite(m) and m > 0 else 10.0)
            return np.concatenate([r, ridge * (x - x_prior)])

        sol = least_squares(residuals, np.clip(x_prior, lo, hi), bounds=(lo, hi))
        self.params = dict(zip(PARAMS, (float(v) for v in np.exp(sol.x))))
        res = residuals(sol.x)[:len(rows)]
        self.fit_report = {"cost": float(sol.cost), "status": int(sol.status), "ridge": ridge,
                           "prior": prior, "reduction": self.reduction,
                           "log_error": [{"trial": i, "channel": ("translation", "rotation")[ch], "measured": v,
                                          "log_error": float(e)} for (i, _, ch, v), e in zip(rows, res)],
                           "skipped": [{"trial": i, "channel": ("translation", "rotation")[ch]} for i, ch in skipped]}
        return self

    # --- model file ---

    def to_dict(self):
        return {
            "format": FORMAT3,
            "gamma": float(GAMMA),
            "muscles": list(self.arm.names),
            "arm": {"upper_arm_m": self.arm.upper_arm_m, "forearm_m": self.arm.forearm_m, "hand_m": self.arm.hand_m},
            "posture": {"upper_arm_deg": self.upper_arm_deg, "elbow_deg": self.elbow_deg},
            "muscle_groups": [{"name": g, "members": [int(i) for i in self.members[g]],
                               "ratio": float(self.group_ratio[g])} for g in GROUPS],
            "drive": {"a0": self.params["a0"], "b": self.params["b"],
                      "input": "d = mean of sigma_biceps and sigma_triceps; u = ratio * (a0 + b d)"},
            "wrist": {"joints": list(WRIST_JOINTS), "passive": self.wrist_passive.tolist(), "s_w": self.params["s_w"],
                      "c_r": self.params["c_r"], "input": "K_w = s_w diag(passive) + c_r d I, Nm/rad"},
            "reduction": self.reduction,
            "measurement": {"handle_offset_m": self.handle_offset.tolist(), "robot_translation_hold": self.robot_hold[0],
                            "robot_rotation_hold": self.robot_hold[1]},
            "commands": {name: dict(c, unit="N/m" if name == "translation" else "Nm/rad")
                         for name, c in self.commands.items()} or None,
            "fit": self.fit_report,
        }

    @classmethod
    def from_dict(cls, d):
        if d.get("format") != FORMAT3:
            raise ValueError(f"not a {FORMAT3} model")
        arm = ArmModel(**d["arm"])
        if list(arm.names) != list(d["muscles"]):
            raise ValueError("muscle list of the model file does not match arm_model.MUSCLES")
        m = d.get("measurement") or {}
        model = cls(arm, d["posture"]["upper_arm_deg"], d["posture"]["elbow_deg"],
                    params={"a0": d["drive"]["a0"], "b": d["drive"]["b"], "s_w": d["wrist"]["s_w"],
                            "c_r": d["wrist"]["c_r"]},
                    reduction=d.get("reduction", "mean"), handle_offset=m.get("handle_offset_m", (0.0, 0.0, 0.0)),
                    robot_hold=(m.get("robot_translation_hold", ROBOT_TRANSLATION_HOLD),
                                m.get("robot_rotation_hold", ROBOT_ROTATION_HOLD)),
                    wrist_passive=d["wrist"].get("passive", WRIST_PASSIVE),
                    group_ratio={g["name"]: g["ratio"] for g in d["muscle_groups"]})
        model.commands = {name: {k: v for k, v in c.items() if k != "unit"}
                          for name, c in (d.get("commands") or {}).items() if c}
        model.fit_report = d.get("fit") or {}
        return model

    def save(self, path, index=None):
        data = self.to_dict()
        if index is not None:
            data["index"] = index
        Path(path).write_text(json.dumps(data, indent=1))
        return data

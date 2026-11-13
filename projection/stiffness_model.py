"""Elastography indices + posture -> muscle stiffness -> joint stiffness -> Cartesian endpoint stiffness.

  muscle    k_i = gamma F_max_i u_g / l0_i,   f_i = F_max_i u_g,   u_g = a_g + b_g sigma_g
  joint     K_j = J_m^T diag(k) J_m + sum_i f_i d2l_i/dq2 - d(J^T F_env)/dq + sum_j u_j S_j
  endpoint  K_c = (J K_j^-1 J^T)^-1        (3 x 3, translational, world = robot base frame)

- sigma: per-session normalised elastography index of the measured muscles, 0 = relaxed, 1 = strongest
  calibration contraction (shear modulus scales with muscle force and short-range stiffness: Bouillard 2011,
  Vigotsky 2020). Two are measured: "biceps" and "triceps".
- u_g: force level of a muscle group as a fraction of F_max. Measured muscles follow their own index;
  unmeasured muscles follow a synergy of the measured ones (Ajoudani et al. 2017: biceps + triceps set the volume
  of the ellipsoid, posture sets its shape). Groups and drivers are in GROUPS.
- S_j: joint-level stiffness of structures that are not modelled as lines (optional; none by default, the rotator
  cuff, pectoralis and latissimus that keep the shoulder stiff in every direction are lines of the MoBL-ARMS set).
- k = gamma F / l0: short-range stiffness, gamma = 23.4 (Cui et al. 2008), as used by Hu et al. 2011.
- The first term is the whiteboard chain K_el = J_mus K_mus J_mus^T, K_cart = J_arm K_j J_arm^T (there, J_arm maps
  joint stiffness to the hand, i.e. the inverse-transpose of the velocity Jacobian here). The other two terms
  are the geometric stiffness of the muscle forces and of the load at the hand (McIntyre et al. 1996, Hu et al.
  2011); both vanish for small forces.
- The arm has 4 joint coordinates and 3 endpoint directions, so joint stiffness is mapped through compliance
  (exact for a redundant arm) instead of a pseudo-inverse.

Calibration fits the 2 parameters per group (a_g >= 0, b_g >= 0) to Cartesian stiffness identified with robot
perturbations (stiffness trials), to isometric handle forces (force trials, cheaper: needs no perturbation because
k follows from f), or to both. With no calibration the literature prior gives plausible N/m values.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares, nnls

from arm_model import ArmModel, Posture

GAMMA = 23.4
INDICES = ("biceps", "triceps")

# group: (muscles, weights over the measured indices)
SHOULDER = ("deltoid_anterior", "deltoid_middle", "deltoid_posterior", "supraspinatus", "infraspinatus", "subscapularis",
            "teres_minor", "teres_major", "pectoralis_clavicular", "pectoralis_sternal", "pectoralis_ribs",
            "latissimus_thoracic", "latissimus_lumbar", "latissimus_iliac", "coracobrachialis")
GROUPS = {
    "biceps": (("biceps_long", "biceps_short"), {"biceps": 1.0}),
    "triceps": (("triceps_long", "triceps_lateral", "triceps_medial", "anconeus"), {"triceps": 1.0}),
    "elbow_flexors": (("brachialis", "brachioradialis", "extensor_carpi_radialis_longus", "pronator_teres"),
                      {"biceps": 1.0}),
    "shoulder": (SHOULDER, {"biceps": 0.5, "triceps": 0.5}),
}

# joint-level group: (diagonal joint stiffness at u = 1, Nm/rad, over shoulder_x, shoulder_y, shoulder_z, elbow;
# weights over the measured indices), for stiffness of structures that are not modelled as lines. Empty since the
# MoBL-ARMS muscle set: rotator cuff, pectoralis and latissimus are lines now.
JOINT_GROUPS = {}

# prior (a_g, b_g). a = 0.01: relaxed joint stiffness ~12 Nm/rad at shoulder and elbow, as the relaxed intercepts of
# Gomi & Osu 1998 (8.7-10.8 Nm/rad); with the arm hanging and the elbow at 90 deg it gives K_xx ~160 N/m along the
# forearm (the 11-12 robot sessions measured 90-250 N/m relaxed). b: K_xx ~3500 N/m at full co-contraction in that
# posture. Calibration replaces both per session.
PRIOR = {"biceps": (0.01, 0.25), "triceps": (0.01, 0.25), "elbow_flexors": (0.01, 0.25), "shoulder": (0.01, 0.15)}


@dataclass
class Trial:
    """One calibration sample: posture, indices and what was measured at the handle."""
    posture: Posture
    sigma: dict
    stiffness: np.ndarray = None  # 3 x 3 Cartesian stiffness from perturbations, N/m
    weight: np.ndarray = None  # 3 x 3 weights of the stiffness entries in the fit (e.g. 0 for unreliable axes)
    force: np.ndarray = None  # force the hand applies to the handle, N (isometric trial)
    load: np.ndarray = field(default_factory=lambda: np.zeros(3))  # force the environment applies to the hand


class StiffnessModel:
    def __init__(self, arm=None, groups=None, joint_groups=None, params=None):
        self.arm = arm or ArmModel()
        self.groups = groups or GROUPS
        self.joint_groups = JOINT_GROUPS if joint_groups is None else joint_groups
        self.params = dict(PRIOR if params is None else params)
        index = {n: i for i, n in enumerate(self.arm.names)}
        self.members = {g: [index[m] for m in muscles] for g, (muscles, _) in self.groups.items()}
        self.fit_report = {}

    # --- forward model ---

    @property
    def all_groups(self):
        return {**self.groups, **self.joint_groups}

    def group_drive(self, sigma):
        return {g: sum(w * float(sigma[i]) for i, w in drivers.items()) for g, (_, drivers) in self.all_groups.items()}

    def muscle_levels(self, sigma, params=None):
        """u_i per muscle (fraction of F_max)."""
        params = params or self.params
        drive = self.group_drive(sigma)
        u = np.zeros(len(self.arm.names))
        for g, idx in self.members.items():
            a, b = params[g]
            u[idx] = a + b * drive[g]
        return u

    def _geometry(self, posture, load=np.zeros(3)):
        return {"J": self.arm.arm_jacobian(posture), "Jm": self.arm.muscle_jacobian(posture),
                "H": self.arm.muscle_hessians(posture), "G": self.arm.load_stiffness(posture, load)}

    def joint_stiffness(self, posture, sigma, load=np.zeros(3), geo=None, params=None, geometric=True):
        geo = geo or self._geometry(posture, load)
        u = self.muscle_levels(sigma, params)
        k = GAMMA * self.arm.f_max * u / self.arm.l0
        kj = geo["Jm"].T @ (k[:, None] * geo["Jm"])
        drive = self.group_drive(sigma)
        for g, (diag, _) in self.joint_groups.items():
            a, b = (params or self.params)[g]
            kj = kj + (a + b * drive[g]) * np.diag(diag)
        if geometric:
            kj = kj + np.einsum("i,ijk->jk", self.arm.f_max * u, geo["H"]) - geo["G"]
        return 0.5 * (kj + kj.T)

    @staticmethod
    def to_cartesian(kj, J):
        """K_c = (J K_j^-1 J^T)^-1; raises if K_j is not positive definite (unstable posture/load)."""
        if np.linalg.eigvalsh(kj).min() <= 0:
            raise np.linalg.LinAlgError("joint stiffness not positive definite")
        kc = np.linalg.inv(J @ np.linalg.solve(kj, J.T))
        return 0.5 * (kc + kc.T)

    def cartesian_stiffness(self, posture, sigma, load=np.zeros(3), geo=None, params=None, geometric=True):
        geo = geo or self._geometry(posture, load)
        return self.to_cartesian(self.joint_stiffness(posture, sigma, load, geo, params, geometric), geo["J"])

    def elbow_torque(self, posture, sigma, geo=None, params=None):
        """Elbow torque of the modelled muscles, Nm (+ = flexion): -J_m^T f, elbow row."""
        geo = geo or self._geometry(posture)
        return float(-(geo["Jm"].T @ (self.arm.f_max * self.muscle_levels(sigma, params)))[3])

    def handle_force(self, posture, sigma, geo=None, params=None):
        """Smallest handle force (hand on handle, N) that balances the modelled elbow torque.

        Only the elbow is fully modelled (biceps, brachialis, brachioradialis, triceps); shoulder torque is shared
        with muscles that are not in the model, so force trials constrain the elbow row only."""
        geo = geo or self._geometry(posture)
        j = geo["J"][:, 3]
        return j * self.elbow_torque(posture, sigma, geo, params) / (j @ j)

    # --- calibration ---

    def _vector(self, params):
        return np.array([v for g in self.all_groups for v in params[g]])

    def _params(self, x):
        return {g: (float(x[2 * i]), float(x[2 * i + 1])) for i, g in enumerate(self.all_groups)}

    def _force_rows(self, trial, geo):
        """Elbow torque (J^T F)[3] is linear in the parameters: -J_m[:, 3]^T diag(F_max) (a + b drive)."""
        drive = self.group_drive(trial.sigma)
        row = []
        for g, idx in self.members.items():
            t = -geo["Jm"][idx, 3] @ self.arm.f_max[idx]
            row += [t, t * drive[g]]
        row += [0.0, 0.0] * len(self.joint_groups)  # joint-level stiffness makes no torque
        return np.array([row]), np.array([geo["J"][:, 3] @ trial.force])

    def fit(self, trials, force_weight=1.0, prior_weight=0.05, geometric=True):
        """Least squares on relative Frobenius errors (stiffness) and torque errors (force), parameters in [0, 1], weak
        pull towards the prior."""
        geos = [self._geometry(t.posture, t.load) for t in trials]
        prior = self._vector(PRIOR)
        k_trials = [(t, g) for t, g in zip(trials, geos) if t.stiffness is not None]
        f_trials = [(t, g) for t, g in zip(trials, geos) if t.force is not None]
        if not k_trials and not f_trials:
            raise ValueError("no trial has a stiffness or a force")

        def residuals(x):
            p = self._params(x)
            r = []
            for t, g in k_trials:
                try:
                    kc = self.cartesian_stiffness(t.posture, t.sigma, t.load, g, p, geometric)
                except np.linalg.LinAlgError:
                    kc = np.full((3, 3), 1e6)
                w = np.ones((3, 3)) if t.weight is None else np.asarray(t.weight, float)
                r.append((w * (kc - t.stiffness) / np.linalg.norm(w * t.stiffness)).ravel())
            for t, g in f_trials:
                a, b = self._force_rows(t, g)
                r.append(force_weight * (a @ x - b) / max(np.linalg.norm(b), 1.0))
            r.append(prior_weight * (x - prior) / np.maximum(prior, 1e-3))
            return np.concatenate(r)

        x0 = prior
        if f_trials:  # linear start from the force trials
            a = np.vstack([self._force_rows(t, g)[0] for t, g in f_trials])
            b = np.concatenate([self._force_rows(t, g)[1] for t, g in f_trials])
            x_lin, _ = nnls(a, b)
            x0 = np.where(x_lin > 0, x_lin, prior)
        sol = least_squares(residuals, x0, bounds=(0.0, 1.0))
        self.params = self._params(sol.x)
        self.fit_report = {"cost": float(sol.cost), "stiffness_trials": len(k_trials), "force_trials": len(f_trials),
                           "status": int(sol.status)}
        return self

    def save(self, path):
        Path(path).write_text(json.dumps({"params": self.params, "groups": {g: [list(m), d] for g, (m, d) in
                                          self.groups.items()}, "joint_groups": {g: [list(s), d] for g, (s, d) in
                                          self.joint_groups.items()}, "arm": {"upper_arm_m": self.arm.upper_arm_m,
                                          "forearm_m": self.arm.forearm_m, "hand_m": self.arm.hand_m},
                                          "fit": self.fit_report}, indent=2))

    @classmethod
    def load(cls, path):
        d = json.loads(Path(path).read_text())
        groups = {g: (tuple(m), drivers) for g, (m, drivers) in d["groups"].items()}
        joint_groups = {g: (tuple(s), drivers) for g, (s, drivers) in d["joint_groups"].items()}
        model = cls(ArmModel(**d["arm"]), groups, joint_groups, {g: tuple(v) for g, v in d["params"].items()})
        model.fit_report = d.get("fit", {})
        return model


# --- what the robot gets ---

def ellipsoid(kc):
    """Principal stiffnesses (ascending, N/m) and directions (columns)."""
    return np.linalg.eigh(kc)


def scalar_stiffness(kc, mode="volume", direction=None):
    """One N/m value for a slave that takes a scalar: ellipsoid volume (geometric mean), mean, or along a direction."""
    if mode == "volume":
        return float(np.linalg.det(kc) ** (1.0 / 3.0))
    if mode == "mean":
        return float(np.trace(kc) / 3.0)
    if mode == "direction":
        u = np.asarray(direction, float)
        u = u / np.linalg.norm(u)
        return float(u @ kc @ u)
    raise ValueError(mode)


def identified_stiffness(dx, df):
    """Cartesian stiffness from perturbations, K = dF dx^T (dx dx^T)^-1, symmetrised (Zeynalli 2025, eq. 2.14).

    dx: 3 x N handle displacements, df: 3 x N restoring force changes (force of the hand on the robot, sign
    such that K is positive for a stiff arm)."""
    dx, df = np.asarray(dx, float), np.asarray(df, float)
    k = df @ dx.T @ np.linalg.inv(dx @ dx.T)
    return 0.5 * (k + k.T)

"""Arm model and muscle -> joint -> Cartesian stiffness chain."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm_model import ArmModel, Posture, posture_from_directions, rot_z, rotvec_to_matrix  # noqa: E402
from stiffness_model import (PRIOR, StiffnessModel, Trial, ellipsoid, identified_stiffness,  # noqa: E402
                             scalar_stiffness)

ARM = ArmModel()
TELEOP = Posture(rot_z(np.radians(30)), np.radians(90))  # upper arm 30 deg forward, forearm pointing forward


def postures(n, seed):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        r = rotvec_to_matrix([rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), rng.uniform(0.0, 0.9)])
        out.append(Posture(r, rng.uniform(np.radians(50), np.radians(120))))
    return out


def test_arm_jacobian_matches_cross_product():
    """Shoulder columns of J are e_k x (x_hand - j), as in Sierotowicz & Castellini 2023, eq. 8."""
    J = ARM.arm_jacobian(TELEOP)
    hand = ARM.handle(TELEOP)
    for k in range(3):
        assert np.allclose(J[:, k], np.cross(np.eye(3)[k], hand), atol=1e-6)
    elbow = TELEOP.upper_arm @ np.array([0, -ARM.upper_arm_m, 0])
    assert np.allclose(J[:, 3], np.cross(TELEOP.upper_arm[:, 2], hand - elbow), atol=1e-6)


@pytest.mark.parametrize("muscle, low, high", [("biceps_long", 0.035, 0.050), ("brachialis", 0.020, 0.035),
                                               ("brachioradialis", 0.050, 0.085), ("triceps_long", -0.025, -0.015)])
def test_elbow_moment_arms_at_90_deg(muscle, low, high):
    """Murray et al. 1995 / Pigeon et al. 1996 ranges at 90 deg flexion (brachioradialis up to the MoBL-ARMS 8 cm)."""
    r = ARM.moment_arms(Posture(np.eye(3), np.radians(90)))[ARM.names.index(muscle), 3]
    assert low <= r <= high


def test_posture_from_directions():
    p = posture_from_directions([0, -1, 0], [1, 0, 0])
    assert p.elbow_angle == pytest.approx(np.pi / 2)
    assert np.allclose(p.upper_arm, np.eye(3))
    with pytest.raises(ValueError):
        posture_from_directions([0, -1, 0], [0, -1, 0.01])


def test_whiteboard_elbow_chain():
    """Elbow entry of J_m^T K_m J_m equals sum r_i^2 k_i (K_el = J_mus K_mus J_mus^T on the whiteboard)."""
    m = StiffnessModel()
    sigma = {"biceps": 0.6, "triceps": 0.3}
    kj = m.joint_stiffness(TELEOP, sigma, geometric=False) - sum(
        (a + b * m.group_drive(sigma)[g]) * np.diag(d) for g, (d, _) in m.joint_groups.items()
        for a, b in [m.params[g]])
    k = 23.4 * ARM.f_max * m.muscle_levels(sigma) / ARM.l0
    r = ARM.moment_arms(TELEOP)[:, 3]
    assert kj[3, 3] == pytest.approx(np.sum(r ** 2 * k))


def test_ellipse_points_to_shoulder_and_grows_with_cocontraction():
    m = StiffnessModel()
    relaxed = m.cartesian_stiffness(TELEOP, {"biceps": 0.0, "triceps": 0.0})
    stiff = m.cartesian_stiffness(TELEOP, {"biceps": 1.0, "triceps": 1.0})
    for kc in (relaxed, stiff):
        assert np.allclose(kc, kc.T) and np.linalg.eigvalsh(kc).min() > 0
    major = ellipsoid(stiff)[1][:, 2]
    to_shoulder = -ARM.handle(TELEOP) / np.linalg.norm(ARM.handle(TELEOP))
    assert abs(major @ to_shoulder) > 0.95  # Mussa-Ivaldi et al. 1985
    assert scalar_stiffness(stiff) > 3 * scalar_stiffness(relaxed)


def test_load_stiffness_symmetric():
    G = ARM.load_stiffness(TELEOP, [0.0, -20.0, 0.0])
    assert np.allclose(G, G.T, atol=1e-6) and np.abs(G).max() > 0


def test_identified_stiffness():
    rng = np.random.default_rng(1)
    K = np.array([[900.0, 150.0, 0.0], [150.0, 400.0, 30.0], [0.0, 30.0, 200.0]])
    dx = rng.normal(0, 0.01, (3, 60))
    assert np.allclose(identified_stiffness(dx, K @ dx + rng.normal(0, 0.05, (3, 60))), K, rtol=0.05, atol=5)


def subject():
    """A 'true' subject: other arm lengths, other parameters than the prior."""
    true = {"biceps": (0.04, 0.35), "triceps": (0.01, 0.15), "elbow_flexors": (0.03, 0.20),
            "shoulder": (0.03, 0.20), "shoulder_other": (0.20, 0.50)}
    return StiffnessModel(ArmModel(0.33, 0.29), params=true)


def trials_from(truth, kind, seed, noise=0.05):
    rng = np.random.default_rng(seed)
    out = []
    levels = [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0), (0.8, 0.1), (0.1, 0.8)]
    for p in postures(6, seed):
        for b, t in levels:
            sigma = {"biceps": b, "triceps": t}
            if kind == "stiffness":
                kc = truth.cartesian_stiffness(p, sigma)
                e = rng.normal(0, noise, (3, 3))
                out.append(Trial(p, sigma, stiffness=kc + np.linalg.norm(kc) * 0.5 * (e + e.T) / 3))
            else:
                f = truth.handle_force(p, sigma)
                out.append(Trial(p, sigma, force=f + rng.normal(0, noise * max(np.linalg.norm(f), 1.0), 3)))
    return out


def rel_error(model, truth, n=8):
    errs = []
    for p in postures(n, 99):
        for s in (0.0, 0.5, 1.0):
            sigma = {"biceps": s, "triceps": s}
            k_true = truth.cartesian_stiffness(p, sigma)
            errs.append(np.linalg.norm(model.cartesian_stiffness(p, sigma) - k_true) / np.linalg.norm(k_true))
    return float(np.median(errs))


def test_calibration_from_perturbations_beats_prior():
    truth = subject()
    prior = StiffnessModel(ArmModel(0.33, 0.29))
    fitted = StiffnessModel(ArmModel(0.33, 0.29)).fit(trials_from(truth, "stiffness", 2))
    assert rel_error(fitted, truth) < 0.10 < rel_error(prior, truth)


def test_force_trials_fix_elbow_torque_and_help_few_perturbations():
    """Isometric forces alone cannot see co-contraction (flexor and extensor torques cancel), but they calibrate the
    elbow torque and improve a calibration that has perturbations in only one posture."""
    truth = subject()
    forces = trials_from(truth, "force", 3)
    by_force = StiffnessModel(ArmModel(0.33, 0.29)).fit(forces)
    prior = StiffnessModel(ArmModel(0.33, 0.29))
    cases = [(p, {"biceps": b, "triceps": t}) for p in postures(5, 7) for b, t in ((1, 0), (0, 1), (0.5, 0.2))]
    err = lambda m: np.median([abs(m.elbow_torque(p, s) - truth.elbow_torque(p, s)) for p, s in cases])
    assert err(by_force) < 0.1 * err(prior)
    one_posture = trials_from(truth, "stiffness", 2)[:3]
    assert rel_error(StiffnessModel(ArmModel(0.33, 0.29)).fit(one_posture + forces), truth) < rel_error(
        StiffnessModel(ArmModel(0.33, 0.29)).fit(one_posture), truth)


def test_save_load(tmp_path):
    m = StiffnessModel(ArmModel(0.31, 0.26), params={**PRIOR, "biceps": (0.05, 0.4)})
    m.save(tmp_path / "model.json")
    back = StiffnessModel.load(tmp_path / "model.json")
    sigma = {"biceps": 0.4, "triceps": 0.2}
    assert np.allclose(back.cartesian_stiffness(TELEOP, sigma), m.cartesian_stiffness(TELEOP, sigma))


def test_weighted_fit_ignores_unreliable_axes():
    """A stiffness entry with weight 0 (here a corrupted K_zz) does not pull the fit."""
    truth = subject()
    trials = trials_from(truth, "stiffness", 2, noise=0.0)
    w = np.zeros((3, 3))
    w[0, 0] = w[1, 1] = w[0, 1] = w[1, 0] = 1.0
    bad = [Trial(t.posture, t.sigma, stiffness=t.stiffness + np.diag([0, 0, 5000.0]), weight=w) for t in trials]
    fitted = StiffnessModel(ArmModel(0.33, 0.29)).fit(bad)
    unweighted = StiffnessModel(ArmModel(0.33, 0.29)).fit(
        [Trial(t.posture, t.sigma, stiffness=t.stiffness) for t in bad])
    assert rel_error(fitted, truth) < 0.5 * rel_error(unweighted, truth)

"""Two-pair hand model: hand Jacobian, 6x6 stiffness, measurement model, reductions, fit, commands, model file."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mosaic_export as mx  # noqa: E402
from arm_model import ArmModel, skew  # noqa: E402
from fit_model_from_perturbation import session_posture  # noqa: E402
from hand_model import PRIOR, HandModel, measured_blocks, reduce  # noqa: E402
from stiffness_model import GROUPS, StiffnessModel  # noqa: E402

POSTURE = session_posture(90.0, 90.0)
TRUE = {"a0": 0.02, "b": 0.12, "s_w": 3.0, "c_r": 15.0}


def test_hand_jacobian_extends_arm_jacobian():
    arm = ArmModel()
    j6 = arm.hand_jacobian(POSTURE)
    assert np.allclose(j6[:3, :4], arm.arm_jacobian(POSTURE), atol=1e-9)
    # rotation rows: shoulder = world axes, elbow = flexion axis, wrist joints = forearm y, x, z
    r_fa = POSTURE.upper_arm @ np.array([[np.cos(POSTURE.elbow_angle), -np.sin(POSTURE.elbow_angle), 0],
                                         [np.sin(POSTURE.elbow_angle), np.cos(POSTURE.elbow_angle), 0], [0, 0, 1]])
    expected = np.column_stack([np.eye(3), POSTURE.upper_arm[:, 2], r_fa[:, 1], r_fa[:, 0], r_fa[:, 2]])
    assert np.allclose(j6[3:], expected, atol=1e-9)
    # forearm twist turns the hand about the axis through the handle: no handle motion
    assert np.allclose(j6[:3, 4], 0.0, atol=1e-9)


def test_stiff_wrist_gives_the_arm_compliance_of_the_group_model():
    """With a very stiff wrist the translational compliance at the handle is that of the 4-joint model with the same
    muscle levels (groups at ratio * (a0 + b d))."""
    hand = HandModel(params={"a0": 0.02, "b": 0.12, "s_w": 1e6, "c_r": 0.0})
    d = 0.7
    old = StiffnessModel(params={g: (r * 0.02, r * 0.12) for g, r in hand.group_ratio.items()})
    sigma = {"biceps": d, "triceps": d}  # every group of GROUPS is driven by weights summing to 1
    assert all(abs(sum(w.values()) - 1.0) < 1e-12 for _, w in GROUPS.values())
    kc = old.cartesian_stiffness(POSTURE, sigma)
    c6 = np.linalg.inv(hand.hand_stiffness(d))
    assert np.allclose(c6[:3, :3], np.linalg.inv(kc), rtol=1e-6)


def test_measurement_model():
    rng = np.random.default_rng(3)
    a = rng.normal(size=(6, 6))
    k6 = a @ a.T + 6 * np.eye(6)
    kt, kr = measured_blocks(k6, np.zeros(3), 1e12, 1e12)
    assert np.allclose(kt, k6[:3, :3]) and np.allclose(kr, k6[3:, 3:])
    # only a translational spring at the handle, offset r from the flange, flange position held: rotation about the
    # flange meets r^T-skew K r-skew
    k = np.diag([800.0, 300.0, 2000.0])
    r = np.array([0.02, -0.01, -0.12])
    k6 = np.zeros((6, 6))
    k6[:3, :3] = k
    k6[3:, 3:] = 1e-9 * np.eye(3)
    _, kr = measured_blocks(k6, r, 1e12, 0.0)
    assert np.allclose(kr, skew(r).T @ k @ skew(r), atol=1e-6)
    # a soft hold lowers what is measured
    kt_soft, _ = measured_blocks(HandModel().hand_stiffness(1.0), r, 1000.0, 10.0)
    kt_hard, _ = measured_blocks(HandModel().hand_stiffness(1.0), r, 1e9, 1e9)
    assert np.trace(kt_soft) < np.trace(kt_hard)


def test_reduce():
    k = np.diag([100.0, 200.0, 400.0])
    assert reduce(k, "mean") == pytest.approx(700.0 / 3.0)
    assert reduce(k, "volume") == pytest.approx(200.0)
    rot = np.linalg.qr(np.random.default_rng(0).normal(size=(3, 3)))[0]
    assert reduce(rot @ k @ rot.T, "volume") == pytest.approx(200.0)
    assert np.isnan(reduce(-k, "volume")) and np.isnan(reduce(-k, "mean"))


def test_both_blocks_grow_with_the_drive():
    hand = HandModel()
    kt = [hand.scalars(d)[0] for d in (0.0, 0.5, 1.0)]
    kr = [hand.scalars(d)[1] for d in (0.0, 0.5, 1.0)]
    assert kt[0] < kt[1] < kt[2] and kr[0] < kr[1] < kr[2]


@pytest.mark.parametrize("reduction", ["mean", "volume"])
def test_fit_recovers_the_parameters(reduction):
    truth = HandModel(params=TRUE, reduction=reduction, handle_offset=(0.0, 0.0, -0.1))
    trials = [(d, *truth.measured(d)) for d in (0.0, 0.4, 0.7, 1.0)]
    fitted = HandModel(reduction=reduction, handle_offset=(0.0, 0.0, -0.1)).fit(trials, ridge=0.0)
    for k, v in TRUE.items():
        assert fitted.params[k] == pytest.approx(v, rel=1e-3)


def test_ridge_pulls_towards_the_prior_and_handles_missing_rotation():
    truth = HandModel(params=TRUE)
    trials = [(d, truth.measured(d)[0], None) for d in (0.0, 0.5, 1.0)]  # no rotational trials
    fitted = HandModel().fit(trials, ridge=0.3)
    assert fitted.params["s_w"] == pytest.approx(PRIOR["s_w"], rel=0.5)
    assert fitted.params["c_r"] == pytest.approx(PRIOR["c_r"], rel=0.5)
    for d, kt, _ in trials:
        assert fitted.measured_scalars(d)[0] == pytest.approx(reduce(kt, "mean"), rel=0.1)


def test_commands_map_the_span(tmp_path):
    hand = HandModel(params=TRUE)
    hand.set_commands(0.0, 1.0)
    assert hand.command("translation", 0.0) == pytest.approx(300.0)
    assert hand.command("translation", 1.0) == pytest.approx(700.0)
    assert hand.command("rotation", 0.0) == pytest.approx(20.0)
    assert hand.command("rotation", 1.0) == pytest.approx(40.0)
    assert 20.0 < hand.command("rotation", 0.5) < 40.0
    assert hand.command("translation", 1.4) == pytest.approx(700.0)


def test_model_file_round_trip_and_projection(tmp_path):
    hand = HandModel(params=TRUE, handle_offset=(0.0, 0.0, -0.12))
    hand.set_commands(0.05, 0.9)
    index = {"kind": "phase", "drive_hz": 45.0, "spacing_m": 0.02, "lag_offset_deg": 0.0, "single_index": None,
             "scale": {"biceps": 30.0, "triceps": 40.0}, "relaxed_phase_deg": {"biceps": 10.0, "triceps": -20.0},
             "direction": {"biceps": 1.0, "triceps": -1.0}, "window_high_deg": {"biceps": 180.0, "triceps": 180.0},
             "sigma_max": 1.5, "features": {p: list(mx.FEATURES[p]) for p in mx.PAIRS}}
    hand.save(tmp_path / "m.json", index)
    data = mx.load(tmp_path / "m.json")
    again = HandModel.from_dict(data)
    assert again.params == pytest.approx(hand.params)
    assert np.allclose(again.handle_offset, hand.handle_offset)
    proj = mx.Projection(data)
    f = np.zeros(12)
    f[8], f[9] = np.cos(np.radians(25.0)), np.sin(np.radians(25.0))  # biceps +15 deg: sigma 0.5
    f[10], f[11] = np.cos(np.radians(-40.0)), np.sin(np.radians(-40.0))  # triceps -20 deg, direction -1: sigma 0.5
    out = proj.step(f)
    k6 = hand.hand_stiffness(0.5)
    assert out[:3] == pytest.approx(list(np.diag(k6[:3, :3])))
    assert out[7:10] == pytest.approx([0.5, 0.5, 1.0])
    assert out[10] == pytest.approx(hand.command("translation", 0.5))
    assert out[11] == pytest.approx(hand.command("rotation", 0.5))
    held = proj.step(np.zeros(12))
    assert held[9] == 0.0 and held[10] == out[10]

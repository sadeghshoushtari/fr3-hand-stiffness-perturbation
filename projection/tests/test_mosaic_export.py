"""MOSAIC model file: the per-packet reference equals the fitted model and the fit script's index definitions."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mosaic_export as mx  # noqa: E402
from fit_model_from_perturbation import indices, session_posture  # noqa: E402
from stiffness_model import PRIOR, StiffnessModel  # noqa: E402

POSTURE = session_posture(0.0, 90.0)


def index_block(kind="speed2", single=None):
    return {"kind": kind, "drive_hz": 45.0, "spacing_m": 0.02, "lag_offset_deg": 0.0, "single_index": single,
            "scale": {"biceps": 2.0, "triceps": 1.5}, "relaxed_phase_deg": {"biceps": -150.0, "triceps": -170.0}}


def features(biceps_deg, triceps_deg):
    f = np.zeros(12)
    for p, deg in (("biceps", biceps_deg), ("triceps", triceps_deg)):
        if deg is not None:
            c, s = mx.FEATURES[p]
            f[c], f[s] = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return f


@pytest.fixture
def model():
    params = {g: (a * 1.3, b * 0.8) for g, (a, b) in PRIOR.items()}
    return StiffnessModel(params=params)


def test_exported_stiffness_equals_model(model, tmp_path):
    data = mx.export(model, index_block(), POSTURE, tmp_path / "m.json")
    data = mx.load(tmp_path / "m.json")
    rng = np.random.default_rng(1)
    for _ in range(20):
        sigma = {"biceps": rng.uniform(0, 1.5), "triceps": rng.uniform(0, 1.5)}
        assert np.allclose(mx.stiffness(data, sigma), model.cartesian_stiffness(POSTURE, sigma), rtol=1e-9)


@pytest.mark.parametrize("kind", ["phase", "speed2"])
def test_packet_index_equals_fit_script_index(model, tmp_path, kind):
    data = mx.export(model, index_block(kind), POSTURE, tmp_path / "m.json")
    phases = [-150.0, -120.0, -95.0, 170.0]  # last one has wrapped past +-180
    rows = [{"label": "relaxed" if i == 0 else "t", "biceps_phase_deg": str(p), "triceps_phase_deg": str(p - 20.0)}
            for i, p in enumerate(phases)]
    raw = indices(rows, kind, 45.0, 0.02, 0.0)
    for i, p in enumerate(phases):
        s = mx.sigmas(data, features(p, p - 20.0))
        for pair, scale in (("biceps", 2.0), ("triceps", 1.5)):
            assert s[pair] == pytest.approx(min(max(raw[pair][i], 0.0) / scale, 1.5))


def test_unusable_packet_holds_last_output(model, tmp_path):
    data = mx.export(model, index_block(), POSTURE, tmp_path / "m.json")
    proj = mx.Projection(data)
    assert proj.step(features(None, -170.0)) == [0.0] * len(mx.OUTPUT)  # nothing valid yet
    good = proj.step(features(-120.0, -160.0))
    assert good[mx.VALID] == 1.0
    held = proj.step(features(-120.0, None))
    assert held[mx.VALID] == 0.0 and [v for i, v in enumerate(held) if i != mx.VALID] == \
        [v for i, v in enumerate(good) if i != mx.VALID]


def test_single_index_ignores_the_other_pair(model, tmp_path):
    idx = index_block(single="biceps")
    idx["scale"]["triceps"] = idx["scale"]["biceps"]  # the fit script scales both by the same raw index
    data = mx.export(model, idx, POSTURE, tmp_path / "m.json")
    s = mx.sigmas(data, features(-120.0, None))
    assert s is not None and s["triceps"] == s["biceps"]

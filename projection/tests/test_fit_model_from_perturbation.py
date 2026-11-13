"""Index definitions of fit_model_from_perturbation."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fit_model_from_perturbation import indices, session_posture  # noqa: E402


def rows_for(speeds, f=45.0, d=0.02, offset=0.0):
    """Trials whose pair phase is the travel lag of a wave at the given speeds (first = relaxed)."""
    lag = 360.0 * f * d / np.asarray(speeds)
    phase = (offset - lag + 180.0) % 360.0 - 180.0
    return [{"label": "relaxed" if i == 0 else f"t{i}", "biceps_phase_deg": str(p), "triceps_phase_deg": str(p)}
            for i, p in enumerate(phase)]


def test_speed2_index_is_relative_modulus_change():
    speeds = np.array([2.5, 3.5, 5.0, 7.0])
    raw = indices(rows_for(speeds), "speed2", 45.0, 0.02, 0.0)
    assert np.allclose(raw["biceps"], (speeds / 2.5) ** 2 - 1.0)


def test_phase_index_is_the_flipped_phase_change():
    """The index rises when the measured pair phase falls (sign of the 11-12 runs at 80 Hz)."""
    rows = [{"label": "relaxed", "biceps_phase_deg": "-4", "triceps_phase_deg": "170"},
            {"label": "strong", "biceps_phase_deg": "-20", "triceps_phase_deg": "-170"}]
    raw = indices(rows, "phase", 80.0, 0.02, 0.0)
    assert raw["biceps"] == pytest.approx([0.0, 16.0]) and raw["triceps"] == pytest.approx([0.0, -20.0])


def test_session_posture_faces_the_robot():
    p = session_posture(0.0, 90.0)
    assert np.allclose(p.torso @ [1, 0, 0], [-1, 0, 0]) and np.allclose(p.torso @ [0, 1, 0], [0, 0, 1])
    assert np.allclose(p.upper_arm, p.torso)  # arm hanging: upper-arm frame = torso frame

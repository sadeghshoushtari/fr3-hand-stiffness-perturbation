"""Scenario of the headless session test (run_session_test.sh): a virtual operator with known model parameters.

python make_scenario.py <out_dir> <windows_out_dir>

Writes to <out_dir>:
  scenario.json            what simulate_session.py plays: per time window the operator state (sigmas, pair phases,
                           posture quaternions) and the hand stiffness at the handle (model + empty-handle stiffness)
  calibration.json, run.json   mosaic/session_*.json with the model and log paths moved to <windows_out_dir>
  actions_calibration.txt, actions_run.txt   card presses for the pipeline runner (test/replay)
Calibration: empty handle, relaxed, medium and max effort (Effort start capture; the perturbation program runs the steps and
the trial ends with them), then fit; phases of the calibration mounting (biceps phase rises
15 deg per unit sigma from -48.7 deg, triceps falls 10 deg per unit sigma from 149.2 deg). Run: a new mounting
(relaxed -30 / 160 deg, both directions reversed), set with the Session trigger (relaxed, then tense), then 60 %
effort at the session posture and at a second posture from the IMUs.
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "projection"))
from arm_model import ArmModel  # noqa: E402
from fit_model_from_perturbation import session_posture  # noqa: E402
from stiffness_model import StiffnessModel  # noqa: E402

TRUTH = {"biceps": (0.01, 0.12), "triceps": (0.01, 0.08), "elbow_flexors": (0.008, 0.10), "shoulder": (0.01, 0.04)}
K_EMPTY = [[40.0, 5.0, 0.0], [5.0, 35.0, 0.0], [0.0, 0.0, 60.0]]
# rotational hand stiffness (Nm/rad): K_ROT_BASE * (1 + 3 * mean sigma), plus the robot's own share K_EMPTY_ROT
K_ROT_BASE = [2.0, 1.5, 3.0]
K_EMPTY_ROT = [[0.3, 0.0, 0.0], [0.0, 0.3, 0.0], [0.0, 0.0, 0.3]]
LEVELS = {"empty": None, "relaxed": (0.0, 0.0), "medium": (0.5, 0.45), "max": (1.0, 1.0), "effort60": (0.6, 0.7)}
TRIAL_S, GAP_S, FIRST_S = 50.0, 4.0, 4.0  # a trial of perturbation.cpp (12 shifts + 6 turns) takes 40-47 s
# robot offset, robot state, slave command: moved for the test (Windows reserves ranges around 50900 on some PCs and
# IPsec uses 4500)
PORTS = {50900: 51900, 50910: 51910, 4500: 51500}
REPO = Path(__file__).resolve().parents[4]


def quat(r):
    """Rotation matrix -> unit quaternion (w, x, y, z), Shepperd's method."""
    tr = np.trace(r)
    if tr > 0:
        s = 2 * np.sqrt(tr + 1)
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2 * np.sqrt(1 + r[0, 0] - r[1, 1] - r[2, 2])
        q = [(r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s]
    elif r[1, 1] > r[2, 2]:
        s = 2 * np.sqrt(1 + r[1, 1] - r[0, 0] - r[2, 2])
        q = [(r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s]
    else:
        s = 2 * np.sqrt(1 + r[2, 2] - r[0, 0] - r[1, 1])
        q = [(r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s]
    return [float(v) for v in q]


def posture_quats(p):
    c, s = np.cos(p.elbow_angle), np.sin(p.elbow_angle)
    fa = p.upper_arm @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return quat(p.upper_arm) + quat(fa)


def main():
    out, win = Path(sys.argv[1]), sys.argv[2].rstrip("\\/")
    out.mkdir(parents=True, exist_ok=True)
    arm = ArmModel(0.30, 0.27, 0.07)
    truth = StiffnessModel(arm, params=TRUTH)
    home = session_posture(90.0, 90.0)
    second = session_posture(70.0, 110.0)

    def window(t0, t1, level, mount, posture=home):
        sig = LEVELS[level]
        k = np.array(K_EMPTY)
        if sig is not None:
            k = k + truth.cartesian_stiffness(posture, {"biceps": sig[0], "triceps": sig[1]})
        s = sig or (0.0, 0.0)
        k_rot = np.array(K_EMPTY_ROT)
        if sig is not None:
            k_rot = k_rot + np.diag(K_ROT_BASE) * (1.0 + 3.0 * 0.5 * (s[0] + s[1]))
        phase = [mount[0] + mount[1] * s[0], mount[2] + mount[3] * s[1]]
        return {"t0": t0, "t1": t1, "level": level, "sigma": list(s), "phase_deg": phase, "K_hand": k.tolist(),
                "K_hand_rot": k_rot.tolist(), "imu": posture_quats(posture)}

    # triceps: the phase turns by 212 deg at full effort (run of 11-17), past the 180 deg of the shortest wrap
    calib_mount = (-48.7, 15.0, 149.2, 212.0)
    windows, actions, t = [], [], FIRST_S
    # the fit runs by itself after "max"; the last trial validates the model
    for label, code in (("empty", -1), ("relaxed", 0), ("medium", 0.5), ("max", 1), ("medium", 0.5)):
        windows.append(window(t - GAP_S / 2, t + TRIAL_S + GAP_S / 2, label, calib_mount))
        # the Trigger card stops the capture by itself after its capture duration (default 5 s)
        actions += [f"{t:.1f} capture Effort {label}", f"{t + 5.0:.1f} stop Effort"]
        t += TRIAL_S + GAP_S
    calib_end = t + 4.0

    run_mount = (-30.0, -15.0, 160.0, 212.0)
    run = [window(0.0, 10.0, "relaxed", run_mount), window(10.0, 18.0, "max", run_mount),
           window(18.0, 32.0, "effort60", run_mount), window(32.0, 46.0, "effort60", run_mount, second)]
    run_actions = ["4 capture Session relaxed", "9 stop Session", "12 capture Session tense", "17 stop Session"]
    checks = [{"t0": 22.0, "t1": 30.0, "posture": [90.0, 90.0], "sigma": LEVELS["effort60"]},
              {"t0": 36.0, "t1": 44.0, "posture": [70.0, 110.0], "sigma": LEVELS["effort60"]}]

    (out / "scenario.json").write_text(json.dumps({
        "truth": TRUTH, "k_empty": K_EMPTY, "k_rot_base": K_ROT_BASE, "k_empty_rot": K_EMPTY_ROT, "robot_ports": [PORTS[50900], PORTS[50910]], "slave_port": PORTS[4500],
        "calibration": {"windows": windows, "end_s": calib_end},
        "run": {"windows": run, "end_s": 48.0, "checks": checks}}, indent=1))
    (out / "actions_calibration.txt").write_text("\n".join(actions) + "\n")
    (out / "actions_run.txt").write_text("\n".join(run_actions) + "\n")
    for name, src in (("calibration", "session_calibration.json"), ("run", "session_run.json")):
        text = (REPO / "mosaic" / src).read_text()
        text = text.replace("/home/f_airob/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/mosaic_models/", win.replace("\\", "/") + "/model/")
        text = text.replace("/home/f_airob/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/mosaic_logs/", win.replace("\\", "/") + "/logs/")
        for port, test_port in PORTS.items():
            text = text.replace(f"{port}]", f"{test_port}]").replace(f"[{port},", f"[{test_port},")
        (out / f"{name}.json").write_text(text)
    print(f"scenario in {out}: calibration {calib_end:.0f} s, run 48 s")


if __name__ == "__main__":
    main()

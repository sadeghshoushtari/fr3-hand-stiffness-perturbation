"""Checks of the headless session test (run_session_test.sh) against the scenario and the Python model.

python check_session.py <dir>   (<dir>: scenario.json, model/, logs/, sim_*.csv of both runs)

Calibration: trials of the perturbation program (one per Effort start, none for fit); identified K per trial vs the
simulated hand; relaxed reference, index direction and scale vs the
calibration mounting; the block's fit vs StiffnessModel.fit on the same trials; fitted model vs the true model.
Run: Session trigger result (reference and direction of the new mounting); pipeline output vs mosaic_export
(projection/) on the same feature vectors at the session posture and at the IMU posture; stiffness command =
clip(volume, 50, 800); output vs the true model.
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "projection"))
import mosaic_export as mx  # noqa: E402
from arm_model import ArmModel  # noqa: E402
from fit_model_from_perturbation import session_posture  # noqa: E402
from stiffness_model import StiffnessModel, Trial  # noqa: E402

PAIRS = ("biceps", "triceps")
failures = []


def check(name, ok, detail):
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    if not ok:
        failures.append(name)


def rel(a, b):
    return float(np.linalg.norm(np.asarray(a) - np.asarray(b)) / np.linalg.norm(b))


def main():
    d = Path(sys.argv[1])
    sc = json.loads((d / "scenario.json").read_text())
    truth = StiffnessModel(ArmModel(0.30, 0.27, 0.07), params={g: tuple(v) for g, v in sc["truth"].items()})
    k_empty = np.array(sc["k_empty"])

    # --- calibration ---
    rows = [line.split(",") for line in (d / "robot_log" / "trials.csv").read_text().splitlines()[1:]]
    labels = [r[1] for r in rows]
    check("calibration: trials of the perturbation program",
          labels == ["empty", "relaxed", "medium", "max", "medium"] and all(r[6] == "1" for r in rows),
          f"{labels}, completed {[r[6] for r in rows]}")
    session = json.loads(sorted((d / "logs" / "calibration").glob("calibration_*.json"))[-1].read_text())
    trials = session["trials"]
    windows = {w["level"]: w for w in sc["calibration"]["windows"]}
    names = {-1: "empty", 0: "relaxed", 0.5: "medium", 1: "max"}
    errs, errs_rot = [], []
    for t in trials:
        w = windows[names[round(t["code"], 1) if t["code"] > -0.5 else -1]]
        errs.append(rel(t["K"], w["K_hand"]) if t["K"] is not None else np.inf)
        errs_rot.append(rel(t["K_rot"], w["K_hand_rot"]) if t.get("K_rot") is not None else np.inf)
    n_rot = [sum(c["rotation"] for c in t["cycles"]) for t in trials]
    check("calibration: K per trial", len(trials) == 5 and max(errs) < 0.03
          and all(len(t["cycles"]) - r == 12 for t, r in zip(trials, n_rot)),
          f"{len(trials)} trials, translation cycles {[len(t['cycles']) - r for t, r in zip(trials, n_rot)]}, "
          "relative error vs simulated hand " + " ".join(f"{e:.3f}" for e in errs))
    check("calibration: rotational K per trial", max(errs_rot) < 0.05 and all(r == 6 for r in n_rot),
          f"rotation cycles {n_rot}, relative error vs simulated hand " + " ".join(f"{e:.3f}" for e in errs_rot))

    data = json.loads((d / "model" / "stiffness_projection.json.calibration").read_text())
    idx = data["index"]
    rel_ok = abs(idx["relaxed_phase_deg"]["biceps"] + 48.7) < 1.5 and abs(idx["relaxed_phase_deg"]["triceps"] - 149.2) < 1.5
    check("calibration: relaxed reference", rel_ok, f"{idx['relaxed_phase_deg']} (simulated -48.7, 149.2)")
    check("calibration: index direction", idx["direction"] == {"biceps": 1.0, "triceps": 1.0},
          f"{idx['direction']} (simulated +1, +1; triceps turns 212 deg)")
    check("calibration: index scale", abs(idx["scale"]["biceps"] - 15) < 2 and abs(idx["scale"]["triceps"] - 212) < 6,
          f"{ {p: round(v, 2) for p, v in idx['scale'].items()} } (simulated 15, 212 deg)")
    val = [t for t in trials if t.get("validation") is not None]
    verr = [t["validation"].get("relative_error", np.inf) for t in val]
    check("calibration: validation trial after the automatic fit (3x3 model)",
          [t["number"] for t in val] == [5] and max(verr) < 0.10,
          f"trials {[t['number'] for t in val]}, relative error predicted vs measured "
          + " ".join(f"{e:.3f}" for e in verr))
    for name, tol in (("translation", 0.15), ("rotation", 0.05)):
        e = [abs(t["validation"][f"{name}_predicted"] - t["validation"][f"{name}_measured"])
             / abs(t["validation"][f"{name}_measured"]) if t["validation"].get(f"{name}_predicted") is not None else np.inf
             for t in val]
        check(f"calibration: validation of the {name} command map", len(e) == 1 and max(e) < tol,
              f"command {val[0]['validation'].get(f'{name}_predicted')}, measured {val[0]['validation'].get(f'{name}_measured')}"
              f", relative difference {max(e) if e else float('nan'):.3f}")
    cmd = data.get("commands") or {}
    rot = cmd.get("rotation") or {}
    k_rot_mean = float(np.mean(sc["k_rot_base"]))
    check("calibration: rotational command map vs simulated hand", bool(rot)
          and abs(rot["c0"] - k_rot_mean) < 0.05 * k_rot_mean and abs(rot["c1"] - 3 * k_rot_mean) < 0.05 * 3 * k_rot_mean,
          f"c0 {rot.get('c0')}, c1 {rot.get('c1')} (simulated {k_rot_mean:.3f}, {3 * k_rot_mean:.3f}); translation "
          f"{cmd.get('translation')}")

    # the same fit in Python: K - K_empty, sigma = max(wrap_index(direction * (phase - relaxed)), 0) / scale
    k_e = np.mean([np.array(t["K"]) for t in trials if t["code"] < -0.5], axis=0)
    muscle = [t for t in trials if t["code"] > -0.5 and t.get("validation") is None]
    raw = [[mx.wrap_index_deg(idx["direction"][p] * (t["phase_deg"][i] - idx["relaxed_phase_deg"][p])) for i, p in
            enumerate(PAIRS)] for t in muscle]
    sig = [{p: max(r[i], 0.0) / idx["scale"][p] for i, p in enumerate(PAIRS)} for r in raw]
    posture = session_posture(90.0, 90.0)
    py = StiffnessModel(ArmModel(0.30, 0.27, 0.07)).fit(
        [Trial(posture, s, stiffness=np.array(t["K"]) - k_e) for t, s in zip(muscle, sig)])
    x_cs = np.array([[g["a"], g["b"]] for g in data["groups"]]).ravel()
    x_py = np.array([v for g in py.params for v in py.params[g]])
    check("calibration: fit vs Python StiffnessModel.fit", np.max(np.abs(x_cs - x_py)) < 0.01,
          f"largest parameter difference {np.max(np.abs(x_cs - x_py)):.1e}")
    pred = [rel(mx.stiffness(data, {"biceps": w["sigma"][0], "triceps": w["sigma"][1]}),
                np.array(w["K_hand"]) - k_empty) for w in sc["calibration"]["windows"] if w["level"] != "empty"]
    x_true = np.array([v for g in sc["truth"] for v in sc["truth"][g]])
    check("calibration: fitted model vs true model", max(pred) < 0.10,
          "relative error per level " + " ".join(f"{e:.3f}" for e in pred)
          + f"; parameters fitted {np.round(x_cs, 3).tolist()} true {x_true.tolist()}")

    # --- run ---
    data = json.loads((d / "model" / "stiffness_projection.json").read_text())
    idx = data["index"]
    check("run: session relaxed reference", abs(idx["relaxed_phase_deg"]["biceps"] + 30) < 1.5
          and abs(idx["relaxed_phase_deg"]["triceps"] - 160) < 1.5, f"{idx['relaxed_phase_deg']} (simulated -30, 160)")
    check("run: session direction", idx["direction"] == {"biceps": -1.0, "triceps": 1.0},
          f"{idx['direction']} (simulated -1, +1)")
    feats = np.loadtxt(d / "sim_features_run.csv", delimiter=",")
    mon = np.loadtxt(d / "sim_monitor.csv", delimiter=",")
    slave = np.loadtxt(d / "sim_slave.csv", delimiter=",")
    arm = ArmModel(**data["arm"])
    for c in sc["run"]["checks"]:
        p = session_posture(*c["posture"])
        geo = (arm.arm_jacobian(p), arm.muscle_jacobian(p), arm.muscle_hessians(p))
        f = feats[(feats[:, 0] > c["t0"]) & (feats[:, 0] < c["t1"])]
        m = mon[(mon[:, 0] > c["t0"]) & (mon[:, 0] < c["t1"])]
        exp = []
        for v in f[:, 1:]:
            s = mx.sigmas(data, v)
            kc = mx.stiffness(data, s, geo)
            exp.append(mx.output(kc, s, True, data))
        exp, got = np.mean(exp, axis=0), np.mean(m[:, 1:], axis=0)
        cols = [0, 1, 2, 3, 4, 5, 6, 10, 11]
        e = np.max(np.abs(got[cols] - exp[cols]) / np.maximum(np.abs(exp[cols]), 1.0))
        check(f"run: output at posture {c['posture']} vs Python", len(m) > 20 and e < 0.01 and got[9] == 1.0,
              f"{len(m)} packets, mean K diag {np.round(got[:3]).tolist()} expected {np.round(exp[:3]).tolist()}, "
              f"largest relative difference {e:.1e}")
        kt = truth.cartesian_stiffness(p, {"biceps": c["sigma"][0], "triceps": c["sigma"][1]})
        kg = np.array([[got[0], got[3], got[4]], [got[3], got[1], got[5]], [got[4], got[5], got[2]]])
        check(f"run: output at posture {c['posture']} vs true model", rel(kg, kt) < 0.10,
              f"relative error {rel(kg, kt):.3f}, true K diag {np.round(np.diag(kt)).tolist()}")
        s = slave[(slave[:, 0] > c["t0"]) & (slave[:, 0] < c["t1"])]
        sbar = 0.5 * (c["sigma"][0] + c["sigma"][1])
        k_rot_true = float(np.mean(sc["k_rot_base"])) * (1 + 3 * sbar)
        cr = data["commands"]["rotation"]
        exp_rot = cr["min"] + (cr["max"] - cr["min"]) * float(np.clip(
            (k_rot_true - cr["human_low"]) / (cr["human_high"] - cr["human_low"]), 0.0, 1.0))
        in_range = all(data["commands"][n]["min"] <= got[i] <= data["commands"][n]["max"]
                       for n, i in (("translation", 10), ("rotation", 11)))
        check(f"run: two commands to the slave at posture {c['posture']}", len(s) > 20 and s.shape[1] == 3
              and abs(np.mean(s[:, 1]) - got[10]) < 0.01 * got[10] and abs(np.mean(s[:, 2]) - got[11]) < 0.01 * got[11]
              and abs(got[11] - exp_rot) < 0.08 * (cr["max"] - cr["min"]) and in_range,
              f"translation {np.mean(s[:, 1]):.1f} N/m, rotation {np.mean(s[:, 2]):.2f} Nm/rad (simulated hand "
              f"{k_rot_true:.2f} Nm/rad, span {cr['human_low']:.2f}-{cr['human_high']:.2f} mapped onto "
              f"{cr['min']}-{cr['max']}: {exp_rot:.2f}); within the slave ranges: {in_range}")
    print("ALL CHECKS PASSED" if not failures else f"{len(failures)} CHECK(S) FAILED")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()

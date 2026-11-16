"""Runs one pipeline with the pipeline runner (test/replay) and the simulator in step (standard library only).

python run_pipeline_with_sim.py <runner.dll> <pipeline.json> <actions.txt> <scenario.json> calibration|run <out_dir>
    [<hand_file>]

Starts the runner, waits until its pipeline is built ("pipeline running"), then starts simulate_session.py, so that
the card presses of actions.txt and the scenario windows share one time base (to about 0.2 s).
"""
import json
import subprocess
import sys
from pathlib import Path

runner, pipeline, actions, scenario, mode, out = sys.argv[1:7]
hand = sys.argv[7:8]
seconds = json.loads(Path(scenario).read_text())[mode]["end_s"] + 1.0
with open(Path(out) / f"runner_{mode}.log", "w") as log:
    proc = subprocess.Popen(["dotnet", runner, pipeline, str(seconds), actions], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, bufsize=1)
    sim = None
    for line in proc.stdout:
        log.write(line)
        log.flush()
        if sim is None and line.startswith("pipeline running"):
            sim = subprocess.Popen([sys.executable, str(Path(__file__).with_name("simulate_session.py")), scenario,
                                    mode, out, *hand])
        if line.startswith(("[Calibration]", "[Stiffness]")) or " s  " in line:
            print(line.rstrip(), flush=True)
    proc.wait()
    if sim is not None:
        sim.wait()
print(f"runner exit {proc.returncode}")
sys.exit(proc.returncode)

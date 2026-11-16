#!/bin/bash
# Headless end-to-end test of mosaic/session_calibration.json and mosaic/session_run.json (WSL + Windows .NET and
# Python, as on the development laptop): the pipelines run in the pipeline runner (test/replay, built against a MOSAIC
# clone with the Elastography blocks), simulate_session.py plays operator, board, robot and IMUs, check_session.py
# compares the results with the scenario and the Python model.
#
#   MOSAIC_DIR='C:\Users\Sadegh\mosaic-2.0' WORK='C:\Users\Sadegh\fr3_build\sess' ./run_session_test.sh
#
# The script copies the six block files into <MOSAIC_DIR>/MOSAIC/MOSAIC/Models/Elastography (the clone's BlockFactory
# must list stiffnessprojection and perturbationcalibration, see ../../README.md). Takes about 5 minutes.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
blocks=$(cd "$here/../.." && pwd)
MOSAIC_DIR=${MOSAIC_DIR:-'C:\Users\Sadegh\mosaic-2.0'}
WORK=${WORK:-'C:\Users\Sadegh\fr3_build\sess'}
work=$(wslpath "$WORK")
mosaic=$(wslpath "$MOSAIC_DIR")

rm -rf "$work"
mkdir -p "$work/model" "$work/runner" "$work/sim"
python3 "$here/make_scenario.py" "$work" "$WORK"
cp "$blocks"/{ArmModel,StiffnessProjectionModel,StiffnessProjection,StiffnessFit,PerturbationCalibrationCore,PerturbationCalibration}.cs \
   "$mosaic/MOSAIC/MOSAIC/Models/Elastography/"
cp "$blocks/test/replay/Program.cs" "$blocks/test/replay/MosaicPipelineRunner.csproj" "$work/runner/"
cp "$here/simulate_session.py" "$here/run_pipeline_with_sim.py" "$work/sim/"
powershell.exe -NoProfile -Command "cd '$WORK\\runner'; dotnet build -c Release -o bin -p:MosaicDir='$MOSAIC_DIR' 2>&1 | Select-String -Pattern ' error |Build succeeded'" | tr -d '\r'

run() {  # <mode> <pipeline> <actions>
  python.exe "$WORK\\sim\\run_pipeline_with_sim.py" "$WORK\\runner\\bin\\MosaicPipelineRunner.dll" "$WORK\\$2" \
    "$WORK\\$3" "$WORK\\scenario.json" "$1" "$WORK" | tr -d '\r'
  mv "$work/sim_features.csv" "$work/sim_features_$1.csv"
}
echo "--- calibration pipeline"
run calibration calibration.json actions_calibration.txt
cp "$work/model/stiffness_projection.json" "$work/model/stiffness_projection.json.calibration"
echo "--- run pipeline"
run run run.json actions_run.txt
echo "--- checks"
python3 "$here/check_session.py" "$work"

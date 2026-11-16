#!/bin/bash
# Headless end-to-end test of mosaic/session_calibration.json and mosaic/session_run.json (WSL + Windows .NET and
# Python, as on the development laptop): the pipelines run in the pipeline runner (test/replay, built against a MOSAIC
# clone with the Elastography blocks); perturbation.cpp, built against the libfranka stub (../robot_stub: handle mass
# and hand spring), runs the pushes; simulate_session.py plays operator, board and IMUs and sets the stub's hand
# stiffness; check_session.py compares the results with the scenario and the Python model. udp_relay.py connects the
# loopback of Windows (runner) and WSL (perturbation program).
#
#   MOSAIC_DIR='C:\Users\Sadegh\mosaic-2.0' WORK='C:\Users\Sadegh\fr3_build\sess' ./run_session_test.sh
#
# The script copies the six block files into <MOSAIC_DIR>/MOSAIC/MOSAIC/Models/Elastography (the clone's BlockFactory
# must list stiffnessprojection and perturbationcalibration, see ../../README.md). Takes about 7 minutes.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
blocks=$(cd "$here/../.." && pwd)
MOSAIC_DIR=${MOSAIC_DIR:-'C:\Users\Sadegh\mosaic-2.0'}
WORK=${WORK:-'C:\Users\Sadegh\fr3_build\sess'}
work=$(wslpath "$WORK")
mosaic=$(wslpath "$MOSAIC_DIR")

rm -rf "$work"
mkdir -p "$work/model" "$work/runner" "$work/sim"
g++ -std=c++17 -O2 -Wall -Wextra "$blocks/../../perturbation.cpp" -I/usr/include/eigen3 -I"$blocks/test/robot_stub" \
  -o "$work/perturbation_stub" -pthread
python3 "$here/make_scenario.py" "$work" "$WORK"
cp "$blocks"/{ArmModel,StiffnessProjectionModel,StiffnessProjection,StiffnessFit,PerturbationCalibrationCore,PerturbationCalibration}.cs \
   "$mosaic/MOSAIC/MOSAIC/Models/Elastography/"
cp "$blocks/test/replay/Program.cs" "$blocks/test/replay/MosaicPipelineRunner.csproj" "$work/runner/"
cp "$here/simulate_session.py" "$here/run_pipeline_with_sim.py" "$here/udp_relay.py" "$work/sim/"
powershell.exe -NoProfile -Command "cd '$WORK\\runner'; dotnet build -c Release -o bin -p:MosaicDir='$MOSAIC_DIR' 2>&1 | Select-String -Pattern ' error |Build succeeded'" | tr -d '\r'

run() {  # <mode> <pipeline> <actions> [<hand file>]
  python.exe "$WORK\\sim\\run_pipeline_with_sim.py" "$WORK\\runner\\bin\\MosaicPipelineRunner.dll" "$WORK\\$2" \
    "$WORK\\$3" "$WORK\\scenario.json" "$1" "$WORK" ${4:+"$4"} | tr -d '\r'
  mv "$work/sim_features.csv" "$work/sim_features_$1.csv"
}

# perturbation program (WSL) <-> runner (Windows): runner -> 127.0.0.1:51900 (Windows) -> WSL:50900;
# program -> 127.0.0.1:50910 (WSL) -> Windows:51911 -> 127.0.0.1:51910 (runner)
wsl_ip=$(ip -4 addr show eth0 | awk '/inet /{split($2, a, "/"); print a[1]}')
host_ip=$(ip route | awk '/default/{print $3}')
pids=()
cleanup() { [ ${#pids[@]} -gt 0 ] && kill "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT
python.exe "$WORK\\sim\\udp_relay.py" 127.0.0.1:51900 "$wsl_ip:50900" & pids+=($!)
python.exe "$WORK\\sim\\udp_relay.py" 0.0.0.0:51911 127.0.0.1:51910 & pids+=($!)
python3 "$here/udp_relay.py" 127.0.0.1:50910 "$host_ip:51911" & pids+=($!)
fifo_dir=$(mktemp -d)
mkfifo "$fifo_dir/console"
FRANKA_STUB_HAND="$work/hand.txt" "$work/perturbation_stub" --out "$work/robot_log" < "$fifo_dir/console" \
  > "$work/robot_console.txt" 2>&1 & robot=$!
exec 3>"$fifo_dir/console"
echo >&3  # Enter: home move
sleep 2

echo "--- calibration pipeline"
run calibration calibration.json actions_calibration.txt "$WORK\\hand.txt"
echo q >&3
wait $robot
exec 3>&-
rm -rf "$fifo_dir"
echo "--- perturbation program console"
grep -v "^  push" "$work/robot_console.txt" || true
cp "$work/model/stiffness_projection.json" "$work/model/stiffness_projection.json.calibration"
echo "--- run pipeline"
run run run.json actions_run.txt
echo "--- checks"
python3 "$here/check_session.py" "$work"

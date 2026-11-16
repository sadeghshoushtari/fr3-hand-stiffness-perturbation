#!/bin/bash
# Starts MOSAIC from the thesis folder layout: <thesis>/mosaic-2.0 next to <thesis>/fr3-hand-stiffness-perturbation
# (MOSAIC_DIR overrides it). Creates the model and log folders that mosaic/session_*.json use.
set -e
here=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
thesis=$(dirname "$here")
mkdir -p "$thesis/mosaic_models" "$thesis/mosaic_logs/calibration" "$thesis/mosaic_logs/run"
cd "${MOSAIC_DIR:-$thesis/mosaic-2.0}/MOSAIC"
exec dotnet run --project MOSAIC.Desktop "$@"

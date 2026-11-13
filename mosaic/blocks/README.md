# MOSAIC block StiffnessProjection

Runs the muscle-space projection model (`projection/stiffness_model.py`) inside MOSAIC: elastography feature vector
from the ESP32 in, hand stiffness for the robot out, at the rate of the sensor packets (5 Hz).

- `StiffnessProjectionModel.cs`: the computation, without MOSAIC dependencies (index, activation, muscle and joint
  stiffness, K_c = (J K_j^-1 J^T)^-1 for the fixed session posture).
- `StiffnessProjection.cs`: the MOSAIC block around it.

## Model file

`projection/fit_model_from_perturbation.py` writes `stiffness_projection.json` next to `stiffness_model.json` after a
perturbation session (`projection/mosaic_export.py`). It holds the fitted group parameters, the index definition of
that session (`phase` or `speed2`, drive frequency, pair spacing, relaxed reference phases, scale) and the arm
geometry evaluated at the session posture. Copy it to the path in the pipeline, e.g.
`C:\Users\Sadegh\Desktop\mosaic_models\stiffness_projection.json`.

## Install into a MOSAIC clone

1. Copy both `.cs` files to `MOSAIC/MOSAIC/Models/Elastography/`.
2. In `MOSAIC/MOSAIC/Components/Factory/BlockFactory.cs`, add to the `switch` in `Create`:
   ```csharp
   "stiffnessprojection"
       => Models.Elastography.StiffnessProjection.ConfigureInput(sp, m),
   ```
3. Build MOSAIC as usual.

## Pipeline

`mosaic/elastography_projection.json`: UDPClient on port 50500 (ESP32 features, `double`) ->
StiffnessProjection -> UDPClient to 127.0.0.1:50510.

Input: the 12 doubles of `esp32_robot_stiffness` ([8, 9] cos/sin of clip A = triceps, [10, 11] clip B = biceps, as
mounted since 11-05; the indices are in the model file). Output, 10 doubles:

| index | value |
|---|---|
| 0-5 | K_xx, K_yy, K_zz, K_xy, K_xz, K_yz in N/m, robot base frame |
| 6 | K volume (cube root of det K), N/m |
| 7-8 | sigma biceps, sigma triceps (0 = relaxed, 1 = strongest calibration trial, capped at 1.5) |
| 9 | 1 = computed from this packet, 0 = packet without a usable phase: values of the last valid packet |

The robot does not read this output yet; mapping it to the robot command is the last integration step.

## Checks

- `test/`: `python make_reference.py <dir> [--model <stiffness_projection.json> --sensors <sensors.csv>]`, then
  `dotnet run -- <dir> log speed2 phase` compares the C# model with the Python reference packet by packet.
- `test/replay/`: `MosaicPipelineRunner` runs a pipeline JSON without the GUI; `replay_udp.py` sends the feature
  vectors over UDP and compares what the pipeline sends back.

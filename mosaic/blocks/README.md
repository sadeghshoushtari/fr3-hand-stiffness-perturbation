# MOSAIC blocks for the elastography stiffness chain

Two blocks run the muscle-space projection model (`projection/stiffness_model.py`) inside MOSAIC:

- `StiffnessProjection`: elastography features (+ IMU posture) in, hand stiffness for the robot out, at the rate of
  the sensor packets (5 Hz). Sets the session reference (relaxed phase, index direction) from a Trigger card.
- `PerturbationCalibration`: identifies the hand stiffness per trial from the perturbation program's state stream,
  records the elastography phases, and on "fit" fits the model and writes the model file that `StiffnessProjection`
  reads.

Pipelines that use them: `mosaic/session_calibration.json` and `mosaic/session_run.json`, described step by step in
`mosaic/SESSION_PIPELINES.md`.

| File | Content |
|---|---|
| `ArmModel.cs` | port of `projection/arm_model.py`: posture -> J, J_m, muscle Hessians (25 MoBL-ARMS lines) |
| `StiffnessProjectionModel.cs` | model file, index, K_c = (J K_j^-1 J^T)^-1, session reference; no MOSAIC dependency |
| `StiffnessFit.cs` | K = dF dx^T (dx dx^T)^-1 and the bounded least-squares fit of `StiffnessModel.fit` |
| `PerturbationCalibrationCore.cs` | trials, cycles, fit and model file of the calibration; no MOSAIC dependency |
| `StiffnessProjection.cs`, `PerturbationCalibration.cs` | the MOSAIC blocks around them |

## Model file

`stiffness_projection.json`, format `stiffness_projection/2` (written by `PerturbationCalibration`, or by
`projection/mosaic_export.export_v2`): fitted group parameters, arm lengths, session posture, and the index definition
(kind, drive frequency, relaxed phase, direction and scale of each pair). The block evaluates the arm geometry itself,
at the session posture or at the posture from the IMUs. Format `stiffness_projection/1` (geometry stored at the
session posture, direction -1 for both pairs; `projection/fit_model_from_perturbation.py`) is still read, without the
IMU posture.

## Install into MOSAIC (robot PC)

1. Copy the six `.cs` files above to `MOSAIC/MOSAIC/Models/Elastography/` (replace the older two).
2. In `MOSAIC/MOSAIC/Components/Factory/BlockFactory.cs`, in the `switch` of `Create`, next to the other entries:
   ```csharp
   "stiffnessprojection"
       => (typeof(Models.Elastography.StiffnessProjection),
           () => Models.Elastography.StiffnessProjection.ConfigureInput(sp, m)),
   "perturbationcalibration"
       => (typeof(Models.Elastography.PerturbationCalibration),
           () => Models.Elastography.PerturbationCalibration.ConfigureInput(sp, m)),
   ```
   (Older MOSAIC versions without the tuple form: `"stiffnessprojection" => Models.Elastography.StiffnessProjection.ConfigureInput(sp, m),`
   and the same for `perturbationcalibration`.) The blocks do not call `InitDumper`; the factory of the current
   MOSAIC does that for every block with a `Path`.
3. Build: `dotnet build MOSAIC.Desktop/MOSAIC.Desktop.csproj` in the `MOSAIC` folder.

## Inputs and outputs

`StiffnessProjection` (Params: model file, `imu:<block>`, `session:<block>`):

| Input | Content |
|---|---|
| features | 12 doubles of `esp32_robot_stiffness` ([8, 9] cos/sin of clip A = triceps, [10, 11] clip B = biceps) |
| IMU | 8 values: unit quaternions (w, x, y, z) of upper arm and forearm in the robot base frame |
| Session | Trigger with `relaxed:0`, `tense:1`: start/stop capture sets the relaxed reference / the index direction |

Output, 10 doubles: K_xx, K_yy, K_zz, K_xy, K_xz, K_yz (N/m, robot base frame), K volume (cube root of det K),
sigma biceps, sigma triceps (0 = relaxed, 1 = strongest calibration trial, capped at 1.5), valid (0 = packet without
a usable phase: values of the last valid packet).

`PerturbationCalibration` (Params: model file to write, then `key:value` options, see the header of
`PerturbationCalibration.cs`): inputs ESP32 features, Robot state (15 doubles from `perturbation_test --udp`),
LoadCell, IMU, the Stimulus (PerturbationPlan) and the Effort trigger (codes -1 empty, 0 relaxed, 0..1 effort,
9 fit). Output per trial: trial, code, cycles, K entries; after the fit: -1, 9, trials, (a, b) per group. The trials
and the fit are also written to `calibration_<date_time>.json` in the block's `Path`.

## Checks

All run without the robot. On the development laptop the C# projects are built with the Windows .NET SDK.

- `test/`: `python make_reference.py <dir>`, then `dotnet run -- <dir> speed2 phase v2`: the projection against
  `projection/mosaic_export.py` packet by packet (format 1 to 1e-12; format 2, geometry evaluated in C# and in
  Python, to 1e-3).
- `test/arm/`: `python make_arm_reference.py <file>`, `dotnet run -- <file>`: `ArmModel` against `arm_model.py`
  (J, J_m to 1e-10, Hessians to 1e-6) at six postures, also through the quaternion posture.
- `test/fit/`: `python make_fit_reference.py <file>`, `dotnet run -- <file>`: the fit against `StiffnessModel.fit`
  (parameters to 1e-6), the identification, and the trial identification of the calibration with the force from the
  robot spring and from the FR3 wrench.
- `test/replay/`: `MosaicPipelineRunner` runs a pipeline JSON without the GUI and presses card buttons at given times.
- `test/session/run_session_test.sh`: both session pipelines in MOSAIC against a simulated operator, board, robot and
  IMUs with known model parameters (`make_scenario.py`, `simulate_session.py`); `check_session.py` compares the
  identified stiffness, the session reference, the fit (with `StiffnessModel.fit` on the same trials) and the live
  output (with `mosaic_export` on the same packets, at the session posture and at an IMU posture).

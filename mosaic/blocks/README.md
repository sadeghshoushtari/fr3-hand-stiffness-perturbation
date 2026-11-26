# MOSAIC blocks for the elastography stiffness chain

Two blocks run the projection model inside MOSAIC: the two-pair hand model of `projection/hand_model.py` (model
file format 3, the default since 2026-11-26) or the group model of `projection/stiffness_model.py` (format 2):

- `StiffnessProjection`: elastography features (+ IMU posture) in, hand stiffness for the robot out, at the rate of
  the sensor packets (5 Hz). Sets the session reference (relaxed phase, index direction) from a Trigger card.
- `PerturbationCalibration`: identifies the hand stiffness per trial from the perturbation program's state stream,
  records the elastography phases, and on "fit" fits the model and writes the model file that `StiffnessProjection`
  reads.

Pipelines that use them: `mosaic/session_calibration.json` and `mosaic/session_run.json`, described step by step in
`mosaic/SESSION_PIPELINES.md`.

| File | Content |
|---|---|
| `ArmModel.cs` | port of `projection/arm_model.py`: posture -> J, J_m, muscle Hessians (25 MoBL-ARMS lines), hand Jacobian (6x7, with the three wrist joints) |
| `HandModel.cs` | port of `projection/hand_model.py`: drive -> joint stiffness (arm + wrist) -> 6x6 hand stiffness, reductions, what the robot measures |
| `StiffnessProjectionModel.cs` | model file, index, K_c = (J K_j^-1 J^T)^-1 (format 2) or the hand model (format 3), session reference; no MOSAIC dependency |
| `StiffnessFit.cs` | K = dF dx^T (dx dx^T)^-1, the bounded least-squares fit of `StiffnessModel.fit` and the 4-parameter fit of `HandModel.fit` |
| `PerturbationCalibrationCore.cs` | trials, cycles, fit and model file of the calibration; no MOSAIC dependency |
| `StiffnessProjection.cs`, `PerturbationCalibration.cs` | the MOSAIC blocks around them |

## Model file

`stiffness_projection.json`, format `stiffness_projection/3` (written by `PerturbationCalibration` with the default
`model:hand`, or by `HandModel.save` in `projection/hand_model.py`): the hand model. Drive d = (sigma biceps + sigma
triceps) / 2; muscle levels ratio_g (a0 + b d) for the four muscle groups (ratios 1, 1, 1, 0.6) give the 4x4 arm joint
stiffness; the wrist (forearm twist, flexion, deviation) has s_w times the passive stiffness of Formica 2012 plus
c_r d; the 6x6 hand stiffness at the handle centre is (J6 K_q^-1 J6^T)^-1 in the robot base frame. Each 3x3 block is
reduced to one number (`reduction`: mean of the diagonal, or cube root of the determinant) and the relaxed..max span
of these numbers is mapped onto the slave's two ranges (`commands`). The file also holds the handle offset from the
flange and the robot holds used in the fit (the fit compares with what the robot measures at its flange with the other
degrees of freedom held by its springs).

Format `stiffness_projection/2` (written by `PerturbationCalibration` with `model:groups`, or by
`projection/mosaic_export.export_v2`): fitted group parameters, arm lengths, session posture, and the index definition
(kind, drive frequency, relaxed phase, direction and scale of each pair). The block evaluates the arm geometry itself,
at the session posture or at the posture from the IMUs. Format `stiffness_projection/1` (geometry stored at the
session posture, direction -1 for both pairs; `projection/fit_model_from_perturbation.py`) is still read, without the
IMU posture.

## Install into MOSAIC (robot PC)

1. Copy the seven `.cs` files above to `MOSAIC/MOSAIC/Models/Elastography/` (`HandModel.cs` is new since 2026-11-26;
   replace the others).
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
| features | 12 doubles of `esp32_robot_stiffness` ([8, 9] cos/sin of clip A = biceps, [10, 11] clip B = triceps; mounting of 11-17) |
| IMU | 8 values: unit quaternions (w, x, y, z) of upper arm and forearm in the robot base frame |
| Session | Trigger with `relaxed:0`, `tense:1`: start/stop capture sets the relaxed reference / the index direction |

Output, 12 doubles: K_xx, K_yy, K_zz, K_xy, K_xz, K_yz (N/m, robot base frame; format 3: the translational block
of the 6x6 hand stiffness), K volume (cube root of det K), sigma biceps, sigma triceps (0 = relaxed, 1 = strongest
calibration trial, capped at 1.5), valid (0 = packet without a usable phase: values of the last valid packet),
translational command (N/m) and rotational command (Nm/rad) from the `commands` of the model file. Format 3: the
reduced translational and rotational block of the hand model at the drive, their span human_low..human_high (the
model at the relaxed and at the max calibration trials) mapped linearly onto the slave's range (min..max: 300-700
N/m, 20-40 Nm/rad) and held at the ends. Format 2: the operator's overall stiffness c0 + c1 * mean sigma (a line
fitted by the calibration to the mean of the diagonal of its trials), mapped the same way. Model files from before the span mapping
(no human_low/high, written by the calibration blocks before 2026-11-21) still load: their span is taken as mean
sigma 0..1 (relaxed reference..strongest calibration trial) and their command goes to the slave's range.

`PerturbationCalibration` (Params: model file to write, then `key:value` options, see the header of
`PerturbationCalibration.cs`): inputs ESP32 features, Robot state (17 doubles from `perturbation.cpp`, with the push
phase and trial number that define the cycles and the end of a trial), LoadCell, IMU and the Effort trigger (codes -1
empty, 0 relaxed, 0..1 effort; 9 fits again). The fit runs by itself after the first trial at code 1; later trials are validation trials (predicted vs measured stiffness). A Stimulus block can drive the cycles instead (`plan:` input). Output per trial: trial, code, cycles, K entries; after the fit: -1, 9, trials, then a0, b, s_w, c_r (hand model)
or (a, b) per group (group model). The trials, the robot holds seen in each step, and the fit are also written to
`calibration_<date_time>.json` in the block's `Path`. Options of the hand model: `model:hand|groups`,
`reduction:mean|volume`, `handle_offset:x;y;z` (handle centre - flange, m, base frame), `ridge:0.3`.

## Checks

All run without the robot. On the development laptop the C# projects are built with the Windows .NET SDK.

- `test/`: `python make_reference.py <dir>`, then `dotnet run -- <dir> speed2 phase v2 window v3 v3vol`: the
  projection against `projection/mosaic_export.py` packet by packet (format 1 to 1e-12; formats 2 and 3, geometry
  evaluated in C# and in Python, to 1e-3).
- `test/arm/`: `python make_arm_reference.py <file>`, `dotnet run -- <file>`: `ArmModel` against `arm_model.py`
  (J, J_m and the hand Jacobian J6 to 1e-10, Hessians to 1e-6) at six postures, also through the quaternion
  posture.
- `test/fit/`: `python make_fit_reference.py <file> [<calibration_*.json> ...]`, `dotnet run -- <file> [-v]`: the
  fit against `StiffnessModel.fit` (parameters to 1e-6), the identification, the trial identification of the
  calibration with the force from the robot spring and from the FR3 wrench, the hand-model fit against
  `HandModel.fit`, and `PerturbationCalibrationCore.Replay` of each logged session against
  `projection/refit_calibration.py` on the same trials (-v prints the calibration log of the replay).
- `test/replay/`: `MosaicPipelineRunner` runs a pipeline JSON without the GUI and presses card buttons at given times.
- `test/session/run_session_test.sh`: both session pipelines in MOSAIC against a simulated operator, board and IMUs
  with known model parameters, with `perturbation.cpp` built against the libfranka stub `test/robot_stub` (handle mass
  and hand spring) running the pushes (`make_scenario.py`, `simulate_session.py`); `check_session.py` compares the
  identified stiffness, the session reference, the fit (with `StiffnessModel.fit` on the same trials) and the live
  output (with `mosaic_export` on the same packets, at the session posture and at an IMU posture).

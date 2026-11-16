# Session pipelines: calibration and run

Two MOSAIC pipelines share one model file (`stiffness_projection.json`):

- `session_calibration.json`: once per person (and after hardware changes). MOSAIC runs the perturbation sequence on
  the robot, records sensors, robot, load cell and IMUs, identifies the hand stiffness per trial and fits the model.
  The fit writes the model file.
- `session_run.json`: every teleoperation session. MOSAIC reads the sensors and IMUs, sets the session reference
  (relaxed phase and index direction of this mounting), computes the hand stiffness with the fitted model and sends
  it to the slave.

They are separate because they drive the robot with different programs (perturbation program vs. teleoperation
slave), which must never run at the same time.

## Blocks

| Block | Type | Built-in | Role |
|---|---|---|---|
| ESP32 | UDPClient | yes | elastography features (12 doubles) from the USB bridge, port 50500 |
| Vibrator | CommandSender | yes | vibrator on/off: Apply to device sends one text per data stream, ending in "on"/"off", to the USB bridge, port 50601; the last one (Magnetometer) decides, so All = on, None = off |
| IMUControl | CommandSender | yes | starts/stops the lab's ESP IMU nodes (placeholder IP/port until the IMUs are available) |
| IMU | UDPClient | yes | IMU quaternions, port 50700 (placeholder) |
| LoadCell | UDPClient | yes | K6D80 + GSV-8DS force/torque, port 50800 (placeholder until its connection is known) |
| Effort | Trigger | yes | operator state of the trial: empty (-1), relaxed (0), effort30/60/100 (0.3/0.6/1); fit (9) fits the model |
| Clock | Clock | yes | 100 Hz tick that runs the Stimulus |
| PerturbationPlan | Stimulus | yes | per direction: rest 2 s, ramp 0.5 s, hold 1 s, capture 1 s, return 1 s; +-15 mm in x, y, z (33 s per trial) |
| PlanVector | Selector | yes | strips the state name, leaves the 6-element equilibrium offset (dx, dy, dz in m, rx, ry, rz in rad) |
| Robot | UDPClient | yes | sends the offset to the perturbation program (port 50900), receives its state (port 50910) |
| Calibration | PerturbationCalibration | custom | per trial: identified K; on "fit": relaxed reference, direction and scale per pair, model parameters; writes the model file |
| Session | Trigger | yes | run: relaxed (0) and tense (1) captures set the reference and direction of this mounting |
| Stiffness | StiffnessProjection | custom | features + posture (IMU, else session posture) -> 3x3 hand stiffness |
| StiffnessVolume | Projector | yes | selects the scalar stiffness (element 6, ellipsoid volume) for the slave |
| SafeRange | Function | yes | clips the command to 50-800 N/m |
| ToSlave | UDPClient | yes | stiffness command to the slave, port 4500 |
| ToMonitor | UDPClient | yes | full output to port 50510 (watch_stiffness.py) |

## Outside MOSAIC (commanded from MOSAIC over UDP)

| Program | Why outside | Link |
|---|---|---|
| USB bridge `python3 sensors.py` (MOSAIC ports built in) | no built-in block reads the board over USB | sends the board's features to MOSAIC as if they came over Wi-Fi; switches the vibrator on the Vibrator card's commands |
| Perturbation program `./perturbation` (robot 169.254.105.210 and MOSAIC mode built in) | libfranka 1 kHz real-time control | follows the offset from MOSAIC (clamped to +-20 mm, at most 0.1 m/s, back to zero 0.25 s after the last packet), streams its state |
| Teleoperation slave (with energy tank) | libfranka 1 kHz real-time control | reads the stiffness command on port 4500 (to be added) |

## Ports

| Port | From -> to | Content |
|---|---|---|
| 50500 | USB bridge -> MOSAIC | 12 doubles, elastography features |
| 50601 | MOSAIC -> USB bridge | text, vibrator on/off |
| 50700 | IMU nodes -> MOSAIC | 8 values, quaternions (placeholder) |
| 50800 | load cell -> MOSAIC | force/torque (placeholder) |
| 50900 | MOSAIC -> perturbation program | 6 doubles, equilibrium offset |
| 50910 | perturbation program -> MOSAIC | 15 doubles: t, x, y, z, x_d, y_d, z_d, F_ext x, y, z, q_w, q_x, q_y, q_z, K_r |
| 4500 | MOSAIC -> slave | 1 double, stiffness command (N/m) |
| 50510 | MOSAIC -> monitor | 10 doubles, full stiffness output |

## Folders (robot PC)

Everything sits in the thesis folder `~/Mohammadsadegh_Shoushtaridehshal_Master_Thesis`:
`fr3-hand-stiffness-perturbation/` (this repository), `mosaic-2.0/` (MOSAIC with the blocks installed, see
`mosaic/blocks/README.md`), `mosaic_models/` (model file), `mosaic_logs/` (logs). The pipelines in this folder already
use these paths. Build the robot program once (and after it changes), in the repository folder:
`g++ -std=c++17 -O2 -Wall -Wextra perturbation.cpp -I/usr/include/eigen3 -o perturbation -lfranka -pthread`.

## Calibration session, step by step

The board is on USB; the emergency stop is within reach. Three terminals; in terminals 1 and 2 first
`cd ~/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/fr3-hand-stiffness-perturbation`:

1. Terminal 1: `python3 sensors.py`
2. Terminal 2: `./perturbation`. Hands off the handle, press Enter: the robot moves to the home pose and holds the handle.
3. Terminal 3: `cd ~/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/mosaic-2.0/MOSAIC && dotnet run --project MOSAIC.Desktop`.
   In MOSAIC: File -> Open ->
   `~/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/fr3-hand-stiffness-perturbation/mosaic/session_calibration.json`.
   Terminal 3 prints what the Calibration block does.
4. If the Clock card is not running, press its Start.
5. Vibrator card: data streams All, then Apply to device (= vibrator on). Terminal 1 prints `command ... -> vibrator on`.
6. One trial per operator state, in this order: empty, relaxed, effort30, effort60, effort100.
   - empty: nobody touches the handle. Others: the operator holds the handle in the session posture (upper arm
     vertical, elbow 90 deg) and keeps the state (relaxed, or co-contraction at about 30 / 60 / 100 %).
   - Effort card: select the state, press Start capture.
   - PerturbationPlan card: press Start. The robot makes 6 steps (33 s).
   - When the card shows that it has finished: Effort card, Stop capture. Terminal 3 prints
     `trial n (...) done: 6 cycles, K diag ... N/m, phase biceps ... deg, triceps ... deg`.
7. Effort card: select fit, Start capture, then Stop capture. Terminal 3 prints the relaxed reference, the direction
   of each pair, the fitted parameters and `model written to ...`.
8. Vibrator card: None, then Apply to device (= vibrator off). Terminal 2: q and Enter. Terminal 1: q and Enter.

Model: `mosaic_models/stiffness_projection.json` (the previous one is kept as `.bak`); trials and fit:
`mosaic_logs/calibration/calibration_<date_time>.json`.

## Teleoperation session, step by step

1. Terminal 1: `python3 sensors.py` (in the repository folder)
2. Teleoperation slave and master as usual (the slave reads the stiffness command on port 4500 once that input is
   added; until then watch the output with
   `python3 ~/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/mosaic_models/watch_stiffness.py`).
3. Terminal 3: MOSAIC as above, File -> Open -> `.../fr3-hand-stiffness-perturbation/mosaic/session_run.json`. Vibrator card: All, then Apply to device.
4. Session card: select relaxed, Start capture, operator relaxes for 5 s, Stop capture.
5. Session card: select tense, Start capture, operator co-contracts for 5 s, Stop capture. Terminal 3 prints the
   phase change and the direction of each pair; the model file is updated.
6. Teleoperate. At the end: Vibrator card OFF, Terminal 1: q and Enter.

Repeat steps 4-5 after the sensors were taken off and put back on.

## Status

Done and checked without the robot (`mosaic/blocks/test/`): both custom blocks, both pipelines in MOSAIC against a
simulated operator, board, robot and IMUs; the bridge's vibrator commands (fake board on a pseudo-terminal); the
perturbation program's UDP mode (compiled against stub libfranka headers and run against a stub robot loop). Not yet
tested on the robot. To add: the slave's stiffness input on port 4500. Placeholders to fill when the hardware is
there: IMU node address and format (quaternions aligned to the arm segments), load-cell connection, frame and sign.

# Session pipelines: calibration and run

Two MOSAIC pipelines share one model file (`stiffness_projection.json`):

- `session_calibration.json`: once per person (and after hardware changes). MOSAIC starts each trial of the
  perturbation program (12 fast steps of 15 mm), records sensors, robot, load cell and IMUs,
  identifies the hand stiffness per trial and fits the model.
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
| Effort | Trigger | yes | operator state of the trial: empty (-1), relaxed (0), medium (0.5), max (1). Start capture starts the trial on the robot; the fit runs by itself after the first max trial, later trials validate the model |
| Robot | UDPClient | yes | sends the Effort code to the perturbation program (port 50900), receives its state (port 50910) |
| Calibration | PerturbationCalibration | custom | per step: dx, dF; per trial: identified K (the trial ends when the robot's trial ends); fit after the first max trial: quality check (GOOD / WEAK / REDO), relaxed reference, direction, scale and index window per pair, model parameters, writes the model file; validation trials after it: predicted vs measured K, validation verdict (PASS / FAIL / REPEAT VALIDATION). Param `handle_offset:0;0;-0.12`: middle of the grip 12 cm below the robot flange (estimate; replace with the measured length) |
| Session | Trigger | yes | run: relaxed (0) and tense (1) captures set the reference and direction of this mounting |
| Stiffness | StiffnessProjection | custom | features + posture (IMU, else session posture) -> 3x3 hand stiffness |
| Commands | Projector | yes | selects the two commands (elements 10, 11): the operator's translational and rotational stiffness from the hand model (6x6 hand stiffness at the handle, each 3x3 block reduced to one number) with its relaxed..max span mapped linearly onto the slave's range, 300-700 N/m and 20-40 Nm/rad (slave gain 40-80, of which half acts), as the Virtuose trigger's 0..1 is mapped onto 300-700 N/m |
| ToSlave | UDPClient | yes | the two stiffness commands to the slave, port 4500 |
| ToMonitor | UDPClient | yes | full output to port 50510 (watch_stiffness.py) |

## Outside MOSAIC (commanded from MOSAIC over UDP)

| Program | Why outside | Link |
|---|---|---|
| USB bridge `python3 sensors.py` (MOSAIC ports built in) | no built-in block reads the board over USB | sends the board's features to MOSAIC as if they came over Wi-Fi; switches the vibrator on the Vibrator card's commands |
| Perturbation program `./perturbation` (robot 169.254.105.210 and MOSAIC mode built in) | libfranka 1 kHz real-time control | runs one trial of 12 translation steps (15 mm in 0.15 s, K_r = 1000 N/m) and 6 rotation steps (20 deg in 0.6 s, 10 Nm/rad), about 43 s, per Effort start, prints each step, streams its state with the step phase |
| Teleoperation slave (with energy tank) | libfranka 1 kHz real-time control | reads the two stiffness commands on port 4500 (to be added) |

## Ports

| Port | From -> to | Content |
|---|---|---|
| 50500 | USB bridge -> MOSAIC | 12 doubles, elastography features |
| 50601 | MOSAIC -> USB bridge | text, vibrator on/off |
| 50700 | IMU nodes -> MOSAIC | 8 values, quaternions (placeholder) |
| 50800 | load cell -> MOSAIC | force/torque (placeholder) |
| 50900 | MOSAIC -> perturbation program | 1 double, Effort code: starts a trial (-1 empty, 0 relaxed, 0..1 effort; 9 ignored) |
| 50910 | perturbation program -> MOSAIC | 100 Hz, 22 doubles: t, x, y, z, x_d, y_d, z_d, F_ext x, y, z, q_w, q_x, q_y, q_z, K_r, phase (0 idle, 1 rest, 2 ramp up, 3 hold, 4 ramp down), trial number (0 between trials), desired rotation offset x, y, z (rad), rotational K_r (Nm/rad), step kind (0 shift, 1 turn) |
| 4500 | MOSAIC -> slave | 2 doubles: translational stiffness command (N/m), rotational stiffness command (Nm/rad) |
| 50510 | MOSAIC -> monitor | 12 doubles, full stiffness output |

## Folders (robot PC)

Everything sits in the thesis folder `~/Mohammadsadegh_Shoushtaridehshal_Master_Thesis`:
`fr3-hand-stiffness-perturbation/` (this repository), `mosaic-2.0/` (MOSAIC with the blocks installed, see
`mosaic/blocks/README.md`), `mosaic_models/` (model file), `mosaic_logs/` (logs). The pipelines in this folder already
use these paths. Build the robot program once (and after it changes), in the repository folder:
`g++ -std=c++17 -O2 -Wall -Wextra perturbation.cpp -I/usr/include/eigen3 -o perturbation -lfranka -pthread`.

## Calibration session, step by step

Before the session, on the laptop: the bench signal check and the guided test (`signal_check.py`, `guided_test.py`)
both end with a verdict; calibrate only after the guided test prints `VERDICT: BOTH PASS`.

The board is on USB; the emergency stop is within reach. Three terminals; in terminals 1 and 2 first
`cd ~/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/fr3-hand-stiffness-perturbation`:

1. Terminal 1: `python3 sensors.py`
2. Terminal 2: `./perturbation`. Hands off the handle, press Enter: the robot moves to the home pose and holds the
   handle. The program prints `Ready` and then only takes `q`.
3. Terminal 3: `cd ~/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/mosaic-2.0/MOSAIC && dotnet run --project MOSAIC.Desktop`.
   In MOSAIC: File -> Open ->
   `~/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/fr3-hand-stiffness-perturbation/mosaic/session_calibration.json`.
4. Vibrator card: data streams All, then Apply to device (= vibrator on). Terminal 1 prints `vibrator ON`.
5. One trial per operator state, in this order: empty, relaxed, medium, max.
   - empty: nobody touches the handle. Others: the operator holds the handle in the session posture (upper arm
     horizontal pointing forward, forearm vertical up, elbow 90 deg; `posture:fixed;90;90`) and keeps the state (relaxed, or co-contraction medium / maximal) for the whole trial
     without reacting to the steps; trunk against the backrest (best strapped to the chair); the elbow rests on a
     support at the height of the posture, so the shoulder does not carry the arm, and the hand holds the handle
     without pressing on it. Without the support (2026-11-19) the operator pressed 8-26 N on the handle, more in
     some trials than in others, and the sensor phases did not follow the stiffness; with it, 4-11 N, and the mean
     index followed the overall stiffness (r = 0.91 over 6 trials). For max, reach full contraction before Start
     capture and hold it.
   - Effort card: select the state, press Start capture. The robot starts at once: Terminal 2 prints
     `[TRIAL n] <state>: 12 steps of 15 mm and 6 turns of 20 deg, about 43 s` and then `step 1/18: +x` ...
     `step 18/18` (turns are named `+rx` ...).
   - The trial ends by itself after the 18th step: Terminal 2 prints `[TRIAL n] done.`, Terminal 3 prints
     `trial n (...) done: 12 cycles, K diag ... N/m, 6 turns, K_rot diag ... Nm/rad, phase biceps ..., triceps ...`. The Effort card stops
     its capture by itself after a few seconds (its capture duration); that does not end the trial.
   - Rest about a minute between trials.
6. The fit runs by itself when the max trial is done: Terminal 3 prints `full effort trial done: checking the trials
   and fitting the model`, then the check of the trials:
   - `CHECK biceps (clip A): ...`, `CHECK triceps (clip B): ...`: share of packets with a phase in each trial (needs
     80 %), relaxed -> max phase change against the scatter within a trial (needs 2.0x), medium between relaxed and
     max; `CHECK effort: ...`: robot-measured overall stiffness of relaxed, medium, max (max needs 1.5x relaxed).
   - `VERDICT: GOOD` (go on with the validation trials), `WEAK` (a medium trial out of order: repeat it, or go by the
     validation) or `REDO` (with the clip and what to do; the model file is still written, do not use it: reopen
     `session_calibration.json` and repeat the calibration). Backtest on the sessions of 11-17 to 11-25: GOOD only for
     the two sessions whose model worked (11-19 17:26 and 11-23 14:05 UTC).
   Then the relaxed reference, the direction of each pair, the index scale and window, the hand model fit
   (`fit: a0 ..., b ..., s_w ..., c_r ...`, then per trial the measured vs modelled translational and rotational
   stiffness as the robot measures them), `model written to ...`, and the two commands (`translation command:
   operator ... N/m (projection at drive ...) mapped onto 300-700 N/m; rotation command: ...`). The drive is the mean
   of the two pair indices; the model is the hand model of `projection/hand_model.py` (model file format 3, option
   `model:groups` gives the older group model with straight command lines).
   Then validation trials, as above: relaxed, medium, max. Terminal 3 prints `validation trial n (...)`: the drive,
   the translational and rotational stiffness measured vs what the model says the robot would measure, the two slave
   commands from the phases and from the robot's measurement, and the 3x3 model vs measured (session file,
   `validation`).
   Once a relaxed and a max validation trial exist, `VALIDATION CHECK: ...` (slave command predicted from the phases
   vs mapped from the robot's measurement; rotation listed too, not part of the verdict) and `VALIDATION: PASS` (open `session_run.json`), `FAIL` (repeat the
   calibration) or `REPEAT VALIDATION` (the robot measured no stiffening at max: repeat the validation trials).
   PASS needs: robot max at least 1.5x relaxed, predicted max at least half the slave range above relaxed, and both
   predictions within a quarter of the range of the measured ones. Both results are in the session file (`check`,
   `validation_check`).
7. Vibrator card: None, then Apply to device (= vibrator off). Terminal 2: q and Enter. Terminal 1: q and Enter.

Where the results go (`<thesis>` = `~/Mohammadsadegh_Shoushtaridehshal_Master_Thesis`):

| Written by | File | Content |
|---|---|---|
| Calibration block (MOSAIC) | `<thesis>/mosaic_models/stiffness_projection.json` (previous one kept as `.bak`) | fitted model, read by the run pipeline |
| Calibration block (MOSAIC) | `<thesis>/mosaic_logs/calibration/calibration_<date_time>.json` | per trial: each step (dx_d, dx, dF), K, mean phases, phase scatter and packets; after fit: the model and the check verdict; after validation: the validation verdict |
| MOSAIC blocks with a Path | `<thesis>/mosaic_logs/calibration/*.csv` | MOSAIC's own recording of the ESP32, Robot, IMU and LoadCell streams |
| `./perturbation` | `<thesis>/fr3-hand-stiffness-perturbation/perturbation_logs/<date_time>/robot.csv`, `trials.csv` | raw robot data at 1 kHz, trial start and end |
| `sensors.py` | `<thesis>/fr3-hand-stiffness-perturbation/perturbation_logs/<date_time>_sensors/sensors.csv` | raw sensor packets |

## Teleoperation session, step by step

1. Terminal 1: `python3 sensors.py` (in the repository folder)
2. Teleoperation slave and master as usual (the slave reads the two stiffness commands on port 4500 once that input is
   added; until then watch the output with
   `python3 ~/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/mosaic_models/watch_stiffness.py`).
3. Terminal 3: MOSAIC as above, File -> Open -> `.../fr3-hand-stiffness-perturbation/mosaic/session_run.json`. Vibrator card: All, then Apply to device.
4. Session card: select relaxed, Start capture, operator relaxes for 5 s, Stop capture.
5. Session card: select tense, Start capture, operator co-contracts for 5 s, Stop capture. Terminal 3 prints the
   phase change and the direction of each pair; the model file is updated.
6. Teleoperate. At the end: Vibrator card OFF, Terminal 1: q and Enter.

Repeat steps 4-5 after the sensors were taken off and put back on.

## Status

Done and checked without the robot (`mosaic/blocks/test/`): both custom blocks; both pipelines in MOSAIC against a
simulated operator, board and IMUs, with the real perturbation program built against a libfranka stub that simulates
the handle and the hand (`test/robot_stub`); the bridge's vibrator commands (fake board on a pseudo-terminal). Not yet
tested on the robot. To add: the slave's input for the two stiffness commands on port 4500. Placeholders to fill when the hardware is
there: IMU node address and format (quaternions aligned to the arm segments), load-cell connection, frame and sign.

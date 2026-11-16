# FR3 hand stiffness perturbation test

Measures the stiffness of a person's hand and arm at the handle of a Franka Research 3 (FR3) robot, and logs a
wearable muscle sensor at the same time. It is part of an MSc thesis on tele-impedance: the robot's stiffness in
teleoperation is to follow the operator's arm stiffness, estimated online from a wearable elastography device on the
upper arm. This test gives the reference: what the operator's endpoint stiffness actually is in each muscle state,
measured by the robot, next to what the wearable sensor reads.

## Method

The robot holds the handle with a known Cartesian impedance (K_r = 1000 N/m on x, y and z) and shifts its
equilibrium point by small steps. When nobody holds the handle, it follows each step completely. When a hand holds
it, the handle moves only part of the way, because the hand pushes back. At rest the two springs balance:

    K_h dx = K_r (dx_d - dx)

dx_d is the commanded step and dx is how far the handle actually moved. Twelve steps per trial (+x, -x, +y, -y, +z,
-z, twice each, in shuffled order) give a 3x3 stiffness matrix K_h by least squares.

A second, independent estimate uses the FR3's own estimate of the external force (`O_F_ext_hat_K`, from its joint
torque sensors): K = -dF/dx. On the FR3 this estimate is the wrench the robot applies to the hand (first lab run,
2026-11-12), so the analysis uses its negative. A trial recorded with nobody holding the handle (`empty`) is subtracted per direction to
remove errors of the robot's dynamics model. No force/torque sensor is needed.

Step timing (relative to the robot's home pose):

| Part     | Duration                  | Equilibrium point                    |
|----------|---------------------------|--------------------------------------|
| Lead-in  | 3 s                       | home                                 |
| Ramp up  | 0.15 s (raised cosine)    | home to home + 15 mm along one axis  |
| Hold     | 1 s                       | home + 15 mm                         |
| Ramp down| 0.15 s                    | back to home                         |
| Rest     | random 1.5-2.5 s          | home                                 |

The random rests stop the operator from anticipating the next step.

**Rotation trials.** A trial whose label starts with `rot_` turns the handle instead: the equilibrium orientation is
rotated by +-20 deg about the x, y and z axes of the base frame through the end-effector point, twice each, with
0.6 s ramps and a 1.5 s hold (slower than the shifts, to keep the wrist joints well below their speed guard), while
the position stays held at 1000 N/m. For these trials the rotational stiffness of the
robot is lowered from 40 to 10 Nm/rad (blended over 0.5 s at the start and end of the trial), so the hand takes a
measurable share of each turn; at most about 3.5 Nm acts on the hand. The orientation law is the slave's (quaternion
vector part), whose torque for small angles is half its gain: the gains are 80 and 20, the effective stiffnesses 40
and 10 Nm/rad, and trials.csv records the effective value. The same two estimates give the 3x3 rotational stiffness
in Nm/rad, with the handle's rotation vector for dx and the moments of `O_F_ext_hat_K` for F; `rot_empty` is the
empty trial of the rotation steps. A trial lasts about 40-45 s and the operator keeps
one muscle state for the whole trial (for example relaxed, or co-contracting lightly, medium or strongly). With a hand
stiffness of 300-1500 N/m the handle moves 6-11.5 mm of each 15 mm step and the hand feels 3.5-9 N.

## Hardware

- Franka Research 3 with libfranka, controlled from a Linux PC with a real-time kernel (1 kHz torque control).
- A handle on the FR3 flange that the operator holds in the teleoperation posture.
- Wearable sensor: ESP32 board with a vibration exciter and four vibration sensors, one pair on the triceps and one on
  the biceps. It sends, about 5 times per second, 12 values: version, sequence number, quality flags, drive level,
  the four channel amplitudes, and the cosine and sine of the phase difference within each sensor pair. Over USB
  (CP2102, 115200 baud) each update is one text line `F,<12 values>`; the same values also go out over Wi-Fi as a
  UDP packet of 12 little-endian doubles. The commands `start` and `stop` switch the exciter (over USB, or over UDP
  port 50601).

## Files

| File | Runs on | Purpose |
|---|---|---|
| `perturbation_test.cpp` | robot PC | The robot program. One self-contained C++17 file (libfranka, Eigen). |
| `robot_pc_sensor_logger.py` | robot PC | Logs the sensor board over USB with the PC's clock and switches the exciter. Standard library only. |
| `laptop_sensor_logger.py` | another computer | Same, for a board on Wi-Fi when it cannot reach the robot PC. Also measures the clock offset to internet time. |
| `analyze_perturbation.py` | anywhere | Stiffness per trial from the robot log, matched with the sensor log. Needs numpy and scipy. |
| `stiffness_identification.py` | anywhere | The two stiffness fits used by the analysis. |
| `mosaic/*.json` | MOSAIC | Online pipelines from sensor features to the stiffness command (see below). |
| `projection/` | anywhere | Muscle-space arm model, its per-session fit to the measured stiffness, and the export for MOSAIC. Needs numpy and scipy. |
| `mosaic/session_*.json` | MOSAIC | End-to-end session pipelines: perturbation calibration and teleoperation run (see `mosaic/SESSION_PIPELINES.md`). |
| `mosaic/blocks/` | MOSAIC | C# blocks `StiffnessProjection` (fitted model live) and `PerturbationCalibration` (calibration inside MOSAIC), and their checks. |

## Build

```bash
g++ -std=c++17 -O2 -Wall -Wextra perturbation_test.cpp -I/usr/include/eigen3 -o perturbation_test -lfranka -pthread
```

## Run

The sensor board is plugged into the robot PC by USB. Your user needs access to the serial port (once:
`sudo usermod -aG dialout $USER`, then log out and in).

Terminal 1, sensors:

```bash
python3 robot_pc_sensor_logger.py            # or ./sensors.sh; options: --port /dev/ttyUSB0 --out <dir> --udp off --commands 0
```

Type `on` and Enter to start the exciter, `off` to stop it, `q` to quit (the exciter is always switched off on exit).
Every 2 s it prints the four amplitudes and the phase of each pair, and warns if a channel reads exactly zero (usually
a loose sensor cable). By default it also forwards every feature line to MOSAIC as the board's UDP packet (port 50500)
and lets MOSAIC switch the exciter (Vibrator card, port 50601).

Terminal 2, robot:

```bash
./perturbation_test --console                # trials from this console; robot 169.254.105.210 unless an IP is given
./robot.sh                                   # MOSAIC drives the perturbations (default mode; builds the program first)
```

1. The program connects and asks you to press Enter. The robot then moves to the home pose. **Do not hold the handle
   during this move** and keep the emergency stop in reach.
2. The robot holds the handle at home. Type a label for the operator's state and press Enter to run one trial.
3. The first trial should be `empty`, with nobody touching the handle. It is the first, safe check of the motion and is
   needed for the force-based estimate.
4. Then the operator holds the handle, for example: `relaxed`, `cocontract_light`, `cocontract_medium`,
   `cocontract_strong`, `relaxed`. Short breaks between trials.
5. For the rotational stiffness: `rot_empty` (nobody touching), then the same states with `rot_` in front
   (`rot_relaxed`, `rot_cocontract_light`, ...).
6. `q` quits. Ctrl+C stops the robot at once.

Safety stops (the control loop ends): handle more than 50 mm or 35 deg from the equilibrium pose, handle faster than
0.5 m/s or 2 rad/s, or a joint above 85 % of a conservative velocity limit. The libfranka collision reflex (35 N) also
stops the robot: biceps and triceps trials need a small, steady pull or push (about 10 N), not a strong one. The robot is not driven home after a stop; the next run moves
it home first.

## Logs

`perturbation_test` writes to `perturbation_logs/<date_time>/`:

- `robot.csv`, 1 kHz: time, trial, step, schedule phase, step axis (0-2 translation, 3-5 rotation) and sign,
  equilibrium point, handle position and orientation, `O_F_ext_hat_K`, handle velocity, equilibrium rotation offset.
- `trials.csv`: label, start and end of each trial (program clock and Unix time), whether it completed, kind
  (translation or rotation) and the robot stiffness used.

`robot_pc_sensor_logger.py` writes `perturbation_logs/<date_time>_sensors/sensors.csv`: Unix time and the 12 values of
every sensor update.

## Analysis

```bash
python3 analyze_perturbation.py perturbation_logs/<date_time> --sensors perturbation_logs/<date_time>_sensors/sensors.csv
```

For every step, dx_d and dx are the mean over 0.6-1.1 s after step onset minus the mean over 0.3-0.05 s before it. Per
trial it prints and writes to `summary.csv`: the 3x3 stiffness from both estimates, the fit R^2, the volume measure
det(K)^(1/3), and the mean phase, phase scatter and amplitudes of both sensor pairs within the trial (3 s cut from both
ends). Both logs come from the same PC clock, so no time correction is needed. For a sensor log recorded on another
computer with `laptop_sensor_logger.py`, the clock offsets are taken from its `clock.json` and `--robot-clock-offset`.

## MOSAIC pipelines (online stiffness command)

`mosaic/` holds the two pipeline configurations for MOSAIC, the block-based real-time framework used in the lab
(not included here). They turn the sensor features into a 0..1 stiffness command for the teleoperation:

    ESP32 (UDPClient, port 50500) -> PhaseFeatures (values 8-11: cos/sin of both pair phases)
      -> MuscleModel (IncrementalPredictor, ridge regression, trained online from Calibration)
      -> Command (clip to 0..1) -> ToRobot (UDP to 127.0.0.1:50510)

| File | Output |
|---|---|
| `elastography_stiffness.json` | `[biceps, triceps, u]`, u = (biceps + triceps) / 2 is the co-contraction level (tele-impedance) |
| `elastography_stiffness_split.json` | `[biceps, triceps]` (biceps -> translational, triceps -> rotational stiffness) |

Calibration is done with the `Calibration` trigger block: the operator holds each state (relaxed, a biceps load of
0.2 / 0.4 / 1.2 kg, triceps light / medium / max, co-contraction light / medium / max) while its target is active, in
two rounds; the model is fitted to those samples. The robot side maps the 0..1 values to N/m and Nm/rad within its
own limits. The `Path` entries are log folders on the computer where the pipeline was built; change them to a local
folder. The pipelines read the board over Wi-Fi (UDP); on a PC where the board is connected by USB, the `ESP32` block
has to be replaced by a serial input.

## Projection model and live block

`projection/` maps the two sensor indices to the hand stiffness through the arm: muscle activation per group, muscle
stiffness (23.4 F / l0), joint stiffness through the muscle moment arms (25 shoulder and elbow muscles fitted to the
MoBL-ARMS model), and the 3x3 Cartesian stiffness at the handle in the robot base frame. After a session, its 8 group
parameters are fitted to the stiffness measured by the robot:

```bash
python3 analyze_perturbation.py perturbation_logs/<date_time> --sensors perturbation_logs/<date_time>_sensors/sensors.csv
python3 projection/fit_model_from_perturbation.py perturbation_logs/<date_time>/summary.csv --single-index biceps \
    --index speed2 --drive-hz 45
```

`--index phase` uses the pair phase change, `--index speed2` the wave speed through the 20 mm pair (relative change
of the shear modulus); the latter needs a drive frequency at which the phase lag does not wrap (45 Hz). The fit writes
`stiffness_model.json` and `stiffness_projection.json`. The MOSAIC block `StiffnessProjection` (`mosaic/blocks/`,
pipeline `mosaic/elastography_projection.json`) reads the second file and outputs, for every sensor packet, the
stiffness in N/m (K_xx ... K_yz, volume), the two indices and a valid flag; see `mosaic/blocks/README.md` for how to
add it to MOSAIC and for the checks (the C# block reproduces the Python model to 1e-12, also through a MOSAIC
pipeline over UDP).

The whole session can also run in MOSAIC (`mosaic/session_calibration.json`, `mosaic/session_run.json`): the
`PerturbationCalibration` block drives `perturbation_test --udp` through a Stimulus block, identifies the stiffness per
trial, fits the model and writes `stiffness_projection.json` (format 2: the block evaluates the arm geometry itself,
so the posture can come from IMUs); in the run pipeline a Session trigger sets the relaxed reference and the index
direction of each mounting. Step-by-step use: `mosaic/SESSION_PIPELINES.md`.

## Status

The robot program shares its home pose, default behaviour, motion generator, damping law, nullspace term, torque-rate
limit, collision thresholds and joint-velocity limits with the teleoperation slave of the same project. So far it has
been tested only against a simulated stand-in for libfranka (a 3 kg point mass held by a simulated hand of 800 / 400 /
250 N/m): the analysis recovered 802 / 401 / 250 N/m from positions and 800 / 401 / 249 N/m from forces. The first run
on the real FR3 starts with the `empty` trial.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Nodes;

namespace MOSAIC.Models.Elastography;

/// <summary>
/// Perturbation calibration without MOSAIC dependencies (the PerturbationCalibration block feeds it).
/// </summary>
/// <remarks>
/// Trials: an Effort action starts a trial (code: -1 empty handle, 0 relaxed, 0..1 co-contraction level); the trial ends
/// when the perturbation program reports the end of its trial (state value 16 back to 0); stop capture is then ignored
/// (without the program's state, e.g. with a Stimulus block, stop capture ends the trial). The fit runs by itself after
/// the first trial at full effort (code 1) with a stiffness, or on code 9 ("fit"); it uses the trials before it and
/// writes the model file. Trials after the fit are validation trials: the model predicts their stiffness from their
/// phases, and the prediction is compared with the measured stiffness (log and session file).
/// Cycles: within a trial each push of the perturbation program (state value 15: 1 rest, 2 push out, 3 hold, 4 return)
/// gives one perturbation: dx = mean handle position in the hold after <see cref="Options.CaptureSettleS"/> - mean in
/// the rest after <see cref="Options.RestSettleS"/>, likewise the equilibrium dx_d and the force. A Stimulus block
/// (rest -> rise -> hold -> capture -> fall) can drive the cycles instead (robot state without values 15, 16).
/// Restoring force of the hand:
///   robot    dF = K_r (dx_d - dx)     (robot as spring: K_h dx = K_r (dx_d - dx))
///   wrench   dF = sign d(F_ext)       (libfranka O_F_ext_hat_K; on the FR3 it is the force of the robot on the hand
///                                      (analyze_perturbation.py, run of 11-12), so sign +1)
///   loadcell dF = sign R_ee d(F_lc)   (handle load cell, sensor frame rotated by the end-effector orientation; sign
///                                      -1 if the sensor measures the force of the hand on the handle: to verify)
/// Rotation steps (robot state with 22 values, step kind 1): dtheta = rotation vector of the mean handle orientation in
/// the hold relative to the mean in the rest (base frame), likewise the equilibrium offset dtheta_d, and the moment
/// dM = K_r,rot (dtheta_d - dtheta) (robot as rotational spring, whatever the force mode); they give the trial's
/// rotational stiffness K_rot (Nm/rad) the same way.
/// A trial with three or more cycles spanning 3-D gives K = dF dx^T (dx dx^T)^-1 (symmetrised); its phases are the
/// circular means of all feature vectors during the trial; its posture is fixed or the mean IMU posture.
/// Fit: K_empty (mean of empty trials) is subtracted; relaxed reference = circular mean of the relaxed trials;
/// direction per pair = the sign for which the index (<see cref="StiffnessProjectionModel.WrapIndexDeg"/>) rises with
/// the effort code over the trials; index per trial, scale = its largest value, sigma = max(index, 0) / scale;
/// (a_g, b_g) fitted with <see cref="StiffnessFit.Fit"/>. Commands: the overall translational and rotational stiffness
/// of each trial (mean of the diagonal of K - K_empty and of K_rot - K_rot,empty) fitted by least squares as
/// c0 + c1 * mean sigma (StiffnessProjectionModel.CommandMap).
/// Robot state vector (perturbation.cpp): t, x, y, z, x_d, y_d, z_d, F_ext x, y, z, q_w, q_x, q_y, q_z, K_r, phase,
/// trial number, desired rotation offset x, y, z, rotational K_r (Nm/rad), step kind (0 translation, 1 rotation).
/// </remarks>
public sealed class PerturbationCalibrationCore
{
    public const int RobotStateLength = 15;
    public const int RobotStateLengthWithPhase = 17;
    public const int RobotStateLengthWithRotation = 22;
    public const double FitCode = 9.0;

    public sealed class Options
    {
        public string ModelPath { get; set; } = "stiffness_projection.json";
        public string? LogDir { get; set; }
        public string ForceMode { get; set; } = "robot";
        public double? ForceSign { get; set; }  // default +1 for wrench, -1 for loadcell
        public bool ImuPosture { get; set; }
        public double UpperArmDeg { get; set; } = 90.0;
        public double ElbowDeg { get; set; } = 90.0;
        public string Axes { get; set; } = "xyz";
        public double[] ArmLengths { get; set; } = { 0.30, 0.27, 0.07 };
        public string IndexKind { get; set; } = "phase";
        public double DriveHz { get; set; } = 45.0;
        public double SpacingM { get; set; } = 0.02;
        public double LagOffsetDeg { get; set; }
        public string? SingleIndex { get; set; }
        public double RestSettleS { get; set; } = 0.4;     // rests of 0.8-1.2 s
        public double CaptureSettleS { get; set; } = 0.35;  // the handle settles 0.3 s after a 15 mm ramp (11-17)
        public double[] TranslationCommandRange { get; set; } = { 50.0, 800.0 };  // N/m
        public double[] RotationCommandRange { get; set; } = { 1.0, 40.0 };       // Nm/rad
        public int MinCycles { get; set; } = 3;
    }

    public sealed record Cycle(double[] Dx, double[] DxDesired, double[] Df, bool Rotation = false);

    public sealed record Trial(int Number, double Code, double[,]? K, List<Cycle> Cycles, double?[] Phase,
        int[] PhaseCounts, Posture Posture, double[]? ImuQuaternions, double[,]? KRot = null);

    private sealed class Window
    {
        public readonly double[] X = new double[3], Xd = new double[3], F = new double[3];
        public readonly double[] Q = new double[4], Rd = new double[3];
        public double KRot;
        public int N, NRot;
        public bool Rotation;

        public void Clear()
        {
            Array.Clear(X);
            Array.Clear(Xd);
            Array.Clear(F);
            Array.Clear(Q);
            Array.Clear(Rd);
            KRot = 0;
            N = NRot = 0;
            Rotation = false;
        }

        public double[] Mean(double[] v) => v.Select(s => s / N).ToArray();
    }

    private readonly Options _o;
    private readonly ArmModel _arm;
    private readonly int[][] _features;
    private readonly Window _rest = new(), _capture = new();
    private double _captureKr;
    private readonly PhaseCapture _phases = new();
    private readonly double[] _imuSum = new double[8];
    private int _imuCount;
    private double[]? _lastImu;
    private double[]? _lastLoadCell;
    private string _stimState = "idle";
    private double _stimStateSince;
    private double? _trialCode;
    private int _boundRobotTrial, _closedRobotTrial;  // trial numbers of the perturbation program
    private double _lastRobotT = double.NegativeInfinity;
    private List<Cycle>? _cycles;
    private readonly HashSet<int> _validationTrials = new();
    private readonly Dictionary<int, Validation> _validation = new();
    private double[,] _kEmpty = new double[3, 3], _kRotEmpty = new double[3, 3];

    /// <summary>Validation trial: 3x3 model prediction vs measurement (translation), and the two overall stiffnesses
    /// (mean of the diagonal) predicted by the command maps before the slave's range is applied vs measured, with
    /// the command the slave would get.</summary>
    private sealed record Validation(double[,] Predicted, double[,] Measured, double Error, double TranslationMeasured,
        double TranslationPredicted, double TranslationCommand, double? RotationMeasured, double? RotationPredicted,
        double? RotationCommand);
    private readonly string _sessionStamp;

    public List<Trial> Trials { get; } = new();
    public Action<string> Log { get; set; } = _ => { };
    public StiffnessProjectionModel? FittedModel { get; private set; }
    public string StimulusState => _stimState;
    public bool TrialOpen => _trialCode is not null;

    public PerturbationCalibrationCore(Options options)
    {
        _o = options;
        _arm = new ArmModel(options.ArmLengths[0], options.ArmLengths[1], options.ArmLengths[2]);
        _features = new StiffnessProjectionModel.IndexDefinition().Features;
        _sessionStamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
    }

    // --- inputs ---

    public void OnFeatures(IReadOnlyList<double> features)
    {
        if (_trialCode is null)
            return;
        var phase = new double?[2];
        for (int p = 0; p < 2; p++)
        {
            int ci = _features[p][0], si = _features[p][1];
            if (Math.Max(ci, si) < features.Count && features[ci] * features[ci] + features[si] * features[si] > 0.25)
                phase[p] = Math.Atan2(features[si], features[ci]) * 180.0 / Math.PI;
        }
        _phases.Add(phase);
    }

    public void OnImu(IReadOnlyList<double> q)
    {
        if (q.Count < 8)
            return;
        var v = q.Take(8).ToArray();
        if (Math.Abs(Norm4(v, 0) - 1) > 0.1 || Math.Abs(Norm4(v, 4) - 1) > 0.1)
            return;
        // keep each quaternion in the hemisphere of the previous one so that the mean is meaningful
        if (_lastImu is not null)
            for (int s = 0; s < 8; s += 4)
                if (v[s] * _lastImu[s] + v[s + 1] * _lastImu[s + 1] + v[s + 2] * _lastImu[s + 2] + v[s + 3] * _lastImu[s + 3] < 0)
                    for (int k = s; k < s + 4; k++)
                        v[k] = -v[k];
        _lastImu = v;
        if (_trialCode is null)
            return;
        for (int k = 0; k < 8; k++)
            _imuSum[k] += v[k];
        _imuCount++;
    }

    public void OnLoadCell(IReadOnlyList<double> ft)
    {
        if (ft.Count >= 3)
            _lastLoadCell = new[] { ft[0], ft[1], ft[2] };
    }

    public void OnRobot(IReadOnlyList<double> s, double t)
    {
        if (s.Count < RobotStateLength)
            return;
        if (s.Count >= RobotStateLengthWithPhase)
        {
            // MOSAIC delivers every packet in its own task: drop packets older than the last one, time with the
            // robot clock
            t = s[0];
            if (t < _lastRobotT - 1.0)
            {
                // the robot clock went back: the perturbation program was restarted (its trial numbers start again
                // at 1); run of 11-17: all packets of the first trial were dropped as old
                Log("perturbation program restarted");
                _lastRobotT = double.NegativeInfinity;
                _closedRobotTrial = 0;
                _boundRobotTrial = 0;
            }
            if (t <= _lastRobotT)
                return;
            _lastRobotT = t;
            OnRobotPhase((int)Math.Round(s[15]), (int)Math.Round(s[16]), t);
        }
        Window? w = _stimState switch
        {
            "rest" when t - _stimStateSince >= _o.RestSettleS => _rest,
            "capture" => _capture,
            _ => null,
        };
        if (w is null)
            return;
        if (s.Count >= RobotStateLengthWithRotation)
        {
            // orientation mean: quaternions kept in the hemisphere of the window's sum
            double sign = w.NRot > 0 && s[10] * w.Q[0] + s[11] * w.Q[1] + s[12] * w.Q[2] + s[13] * w.Q[3] < 0 ? -1 : 1;
            for (int k = 0; k < 4; k++)
                w.Q[k] += sign * s[10 + k];
            for (int k = 0; k < 3; k++)
                w.Rd[k] += s[17 + k];
            w.KRot += s[20];
            w.NRot++;
            if (w == _capture && s[21] > 0.5)
                w.Rotation = true;
        }
        var f = Force(s);
        if (f is null)
            return;
        for (int k = 0; k < 3; k++)
        {
            w.X[k] += s[1 + k];
            w.Xd[k] += s[4 + k];
            w.F[k] += f[k];
        }
        w.N++;
        if (w == _capture)
            _captureKr += s[14];
    }

    /// <summary>
    /// Phase and trial number of the perturbation program. An open trial is bound to the first robot trial that starts
    /// after it (not one already closed); its cycles come from that robot trial only, and it ends with it.
    /// </summary>
    private void OnRobotPhase(int phase, int robotTrial, double t)
    {
        if (_trialCode is not null && _boundRobotTrial == 0 && robotTrial > _closedRobotTrial)
            _boundRobotTrial = robotTrial;
        if (_trialCode is null || _boundRobotTrial == 0 || robotTrial != _boundRobotTrial)
        {
            OnStimulus("idle", t);
            if (_trialCode is not null && _boundRobotTrial != 0)
                CloseTrial();  // the robot trial ended
            return;
        }
        string state = phase switch { 1 => "rest", 2 => "rise", 3 => "hold", 4 => "fall", _ => "idle" };
        if (state == "hold" && (_stimState == "capture"
                                || (_stimState == "hold" && t - _stimStateSince >= _o.CaptureSettleS)))
            state = "capture";
        OnStimulus(state, t);
    }

    /// <summary>Stimulus state (rest, rise, hold, capture, fall) at time t, s.</summary>
    public void OnStimulus(string state, double t)
    {
        if (state == _stimState)
            return;
        if (_stimState == "capture")
            CloseCycle();
        if (state == "rest")
            _rest.Clear();
        if (state == "capture")
        {
            _capture.Clear();
            _captureKr = 0;
        }
        _stimState = state;
        _stimStateSince = t;
    }

    /// <summary>Effort trigger: code at start capture, null at stop capture.</summary>
    public void OnEffort(double? code)
    {
        if (code is double c)
        {
            if (c >= FitCode - 0.5)
            {
                Fit();
                return;
            }
            if (_trialCode is not null)
                CloseTrial();
            _trialCode = c;
            _cycles = new List<Cycle>();
            _phases.Clear();
            Array.Clear(_imuSum);
            _imuCount = 0;
            Log($"trial {Trials.Count + 1} started: {Label(c)}");
            return;
        }
        if (_trialCode is null)
            return;
        // With the perturbation program the trial ends with the robot's trial: the Trigger card stops the capture by
        // itself after its capture duration (default 5 s), long before the 12 pushes are done.
        if (double.IsFinite(_lastRobotT))
            return;
        CloseTrial();
    }

    // --- cycles and trials ---

    private double[]? Force(IReadOnlyList<double> s)
    {
        switch (_o.ForceMode)
        {
            case "robot":
                return new double[3];  // from K_r, dx_d and dx per cycle (CloseCycle)
            case "wrench":
                double sw = _o.ForceSign ?? 1.0;
                return new[] { sw * s[7], sw * s[8], sw * s[9] };
            case "loadcell":
                if (_lastLoadCell is null)
                    return null;
                var r = ArmModel.QuaternionToMatrix(new[] { s[10], s[11], s[12], s[13] });
                var f = new double[3];
                for (int i = 0; i < 3; i++)
                    for (int k = 0; k < 3; k++)
                        f[i] += (_o.ForceSign ?? -1.0) * r[i, k] * _lastLoadCell[k];
                return f;
            default:
                throw new InvalidOperationException($"unknown force mode '{_o.ForceMode}'");
        }
    }

    private void CloseCycle()
    {
        if (_cycles is null || _rest.N == 0 || _capture.N == 0)
            return;
        if (_capture.Rotation)
        {
            if (_rest.NRot == 0 || _capture.NRot == 0)
                return;
            var th = RotationVector(_capture.Q, _rest.Q);
            var thd = _capture.Rd.Zip(_rest.Rd, (a, b) => a / _capture.NRot - b / _rest.NRot).ToArray();
            double krot = _capture.KRot / _capture.NRot;
            var dm = thd.Zip(th, (d, x) => krot * (d - x)).ToArray();
            _cycles.Add(new Cycle(th, thd, dm, Rotation: true));
            Log($"  cycle {_cycles.Count} (rotation): dtheta_d [{Fmt(thd, 180 / Math.PI)}] deg, "
                + $"dtheta [{Fmt(th, 180 / Math.PI)}] deg, dM [{Fmt(dm, 1)}] Nm");
            return;
        }
        var x0 = _rest.Mean(_rest.X);
        var x1 = _capture.Mean(_capture.X);
        var dx = x1.Zip(x0, (a, b) => a - b).ToArray();
        var dxd = _capture.Mean(_capture.Xd).Zip(_rest.Mean(_rest.Xd), (a, b) => a - b).ToArray();
        double[] df;
        if (_o.ForceMode == "robot")
        {
            double kr = _captureKr / _capture.N;
            df = dxd.Zip(dx, (d, x) => kr * (d - x)).ToArray();
        }
        else
        {
            df = _capture.Mean(_capture.F).Zip(_rest.Mean(_rest.F), (a, b) => a - b).ToArray();
        }
        _cycles.Add(new Cycle(dx, dxd, df));
        Log($"  cycle {_cycles.Count}: dx_d [{Fmt(dxd, 1000)}] mm, dx [{Fmt(dx, 1000)}] mm, dF [{Fmt(df, 1)}] N");
    }

    /// <summary>Rotation vector (base frame) of q * q_ref^-1 from summed quaternions (w, x, y, z).</summary>
    private static double[] RotationVector(double[] qSum, double[] refSum)
    {
        double n1 = Math.Sqrt(qSum.Sum(v => v * v)), n0 = Math.Sqrt(refSum.Sum(v => v * v));
        double w = qSum[0] / n1, x = qSum[1] / n1, y = qSum[2] / n1, z = qSum[3] / n1;
        double w0 = refSum[0] / n0, x0 = -refSum[1] / n0, y0 = -refSum[2] / n0, z0 = -refSum[3] / n0;  // conjugate
        double rw = w * w0 - (x * x0 + y * y0 + z * z0);
        double rx = w * x0 + w0 * x + (y * z0 - z * y0);
        double ry = w * y0 + w0 * y + (z * x0 - x * z0);
        double rz = w * z0 + w0 * z + (x * y0 - y * x0);
        if (rw < 0) (rw, rx, ry, rz) = (-rw, -rx, -ry, -rz);
        double n = Math.Sqrt(rx * rx + ry * ry + rz * rz);
        double scale = n > 1e-12 ? 2.0 * Math.Atan2(n, rw) / n : 2.0;
        return new[] { rx * scale, ry * scale, rz * scale };
    }

    private double[,]? IdentifyCycles(List<Cycle> cycles) => cycles.Count >= _o.MinCycles
        ? StiffnessFit.Identify(cycles.Select(c => c.Dx).ToList(), cycles.Select(c => c.Df).ToList())
        : null;

    private void CloseTrial()
    {
        double code = _trialCode!.Value;
        var allCycles = _cycles ?? new List<Cycle>();
        var cycles = allCycles.Where(c => !c.Rotation).ToList();
        var rotCycles = allCycles.Where(c => c.Rotation).ToList();
        _trialCode = null;
        if (_boundRobotTrial != 0)
            _closedRobotTrial = _boundRobotTrial;
        _boundRobotTrial = 0;
        _cycles = null;
        var k = IdentifyCycles(cycles);
        var kRot = IdentifyCycles(rotCycles);
        double[]? q = null;
        Posture posture;
        if (_o.ImuPosture && _imuCount > 0)
        {
            q = _imuSum.Select(v => v / _imuCount).ToArray();
            posture = ArmModel.FromQuaternions(q[..4], q[4..8], ArmModel.SessionTorso);
        }
        else
        {
            if (_o.ImuPosture)
                Log("  no IMU data in this trial: session posture used");
            posture = ArmModel.SessionPosture(_o.UpperArmDeg, _o.ElbowDeg);
        }
        var trial = new Trial(Trials.Count + 1, code, k, allCycles, _phases.Mean(), _phases.Counts, posture, q, kRot);
        Trials.Add(trial);
        Log($"trial {trial.Number} ({Label(code)}) done: {cycles.Count} cycles, "
            + (k is null ? $"NO stiffness (needs {_o.MinCycles}+ cycles in 3 directions)"
                : $"K diag {k[0, 0]:F0} {k[1, 1]:F0} {k[2, 2]:F0} N/m")
            + (rotCycles.Count == 0 ? "" : kRot is null ? $", {rotCycles.Count} turns, NO rotational stiffness"
                : $", {rotCycles.Count} turns, K_rot diag {kRot[0, 0]:F1} {kRot[1, 1]:F1} {kRot[2, 2]:F1} Nm/rad")
            + $", phase biceps {Deg(trial.Phase[0])} ({trial.PhaseCounts[0]}), triceps {Deg(trial.Phase[1])} "
            + $"({trial.PhaseCounts[1]})");
        if (FittedModel is not null)
        {
            _validationTrials.Add(trial.Number);
            Validate(trial, FittedModel);
        }
        else if (code >= 1.0 - 1e-9 && k is not null)
        {
            Log("full effort trial done: fitting the model");
            Fit();
            return;
        }
        SaveSession(FittedModel);
    }

    /// <summary>Validation trial: stiffness predicted by the fitted model from the trial's phases vs measured
    /// (empty-handle stiffness subtracted); relative error = Frobenius norm on the fitted axes.</summary>
    private void Validate(Trial trial, StiffnessProjectionModel model)
    {
        if (trial.K is null || trial.Code < -0.5)
            return;
        var sigma = new double[2];
        for (int p = 0; p < 2; p++)
        {
            int src = _o.SingleIndex is null ? p : StiffnessProjectionModel.PairIndex(_o.SingleIndex);
            if (trial.Phase[src] is not double ph)
            {
                Log($"validation trial {trial.Number}: no {StiffnessProjectionModel.Pairs[src]} phase, no prediction");
                return;
            }
            double raw = StiffnessProjectionModel.RawIndex(model.Index, src, ph);
            sigma[p] = Math.Min(Math.Max(raw, 0.0) / model.Index.Scale[src], model.Index.SigmaMax);
        }
        var g = _arm.Geometry(trial.Posture);
        var pred = model.Stiffness(sigma, model.Groups, g.J, g.Jm, g.H);
        if (pred is null)
        {
            Log($"validation trial {trial.Number}: model not positive definite at sigma {sigma[0]:F2} {sigma[1]:F2}");
            return;
        }
        var meas = new double[3, 3];
        double num = 0, den = 0;
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
            {
                meas[r, c] = trial.K[r, c] - _kEmpty[r, c];
                if (_o.Axes.Contains("xyz"[r]) && _o.Axes.Contains("xyz"[c]))
                {
                    num += (pred[r, c] - meas[r, c]) * (pred[r, c] - meas[r, c]);
                    den += meas[r, c] * meas[r, c];
                }
            }
        double err = Math.Sqrt(num / Math.Max(den, 1e-12));
        double tMeas = Overall(trial.K, _kEmpty);
        double tPred = model.TranslationCommand?.Line(sigma) ?? double.NaN;
        double tCmd = model.TranslationCommand?.Eval(sigma) ?? double.NaN;
        double? rMeas = trial.KRot is null ? null : Overall(trial.KRot, _kRotEmpty);
        double? rPred = model.RotationCommand?.Line(sigma), rCmd = model.RotationCommand?.Eval(sigma);
        _validation[trial.Number] = new Validation(pred, meas, err, tMeas, tPred, tCmd, rMeas, rPred, rCmd);
        static string Capped(double predicted, double? command, string format) =>
            command is double c && Math.Abs(c - predicted) > 1e-9 ? $", slave command capped to {c.ToString(format)}" : "";
        Log($"validation trial {trial.Number} ({Label(trial.Code)}): sigma {sigma[0]:F2} {sigma[1]:F2}; "
            + $"translation predicted {tPred:F0} N/m vs measured {tMeas:F0} N/m ({RelDiff(tPred, tMeas)}"
            + $"{Capped(tPred, tCmd, "F0")}); "
            + (rMeas is double rm && rPred is double rp
                ? $"rotation predicted {rp:F1} Nm/rad vs measured {rm:F1} Nm/rad ({RelDiff(rp, rm)}{Capped(rp, rCmd, "F1")}); "
                : "no rotation command or measurement; ")
            + $"3x3 model: measured K diag {meas[0, 0]:F0} {meas[1, 1]:F0} {meas[2, 2]:F0}, predicted "
            + $"{pred[0, 0]:F0} {pred[1, 1]:F0} {pred[2, 2]:F0} N/m, relative error {err:F2}");
    }

    /// <summary>Overall stiffness of a trial: mean of the diagonal of K - K_empty.</summary>
    private static double Overall(double[,] k, double[,] empty) =>
        (k[0, 0] - empty[0, 0] + k[1, 1] - empty[1, 1] + k[2, 2] - empty[2, 2]) / 3.0;

    private static string RelDiff(double predicted, double measured) =>
        $"{(predicted - measured) / Math.Max(Math.Abs(measured), 1e-9) * 100:+0;-0} %";

    /// <summary>Least-squares line c0 + c1 * x through the trials; null with fewer than two distinct x.</summary>
    private static StiffnessProjectionModel.CommandMap? FitCommand(IReadOnlyList<double> x, IReadOnlyList<double> y,
        double[] range)
    {
        if (x.Count < 2)
            return null;
        double mx = x.Average(), my = y.Average(), sxx = 0, sxy = 0;
        for (int i = 0; i < x.Count; i++)
        {
            sxx += (x[i] - mx) * (x[i] - mx);
            sxy += (x[i] - mx) * (y[i] - my);
        }
        if (sxx < 1e-9)
            return null;
        double c1 = sxy / sxx;
        return new StiffnessProjectionModel.CommandMap(my - c1 * mx, c1, range[0], range[1]);
    }

    private static double[,] MeanMatrix(IReadOnlyList<double[,]> ks)
    {
        var m = new double[3, 3];
        foreach (var k in ks)
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    m[r, c] += k[r, c] / ks.Count;
        return m;
    }

    // --- fit ---

    public StiffnessProjectionModel? Fit()
    {
        if (_trialCode is not null)
            CloseTrial();
        var withK = Trials.Where(t => t.K is not null && !_validationTrials.Contains(t.Number)).ToList();
        var empty = withK.Where(t => t.Code < -0.5).ToList();
        var muscle = withK.Where(t => t.Code > -0.5).ToList();
        var relaxed = muscle.Where(t => Math.Abs(t.Code) < 1e-9).ToList();
        if (relaxed.Count == 0 || muscle.Count < 2)
        {
            Log("fit: needs at least one relaxed trial and one effort trial with a stiffness");
            return null;
        }
        var kEmpty = new double[3, 3];
        foreach (var t in empty)
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    kEmpty[r, c] += t.K![r, c] / empty.Count;
        _kEmpty = kEmpty;

        var index = new StiffnessProjectionModel.IndexDefinition
        {
            Kind = _o.IndexKind, DriveHz = _o.DriveHz, SpacingM = _o.SpacingM, LagOffsetDeg = _o.LagOffsetDeg,
            SingleIndex = _o.SingleIndex,
        };
        var previous = TryLoad(_o.ModelPath);
        if (previous is not null)
            index.Direction = (double[])previous.Index.Direction.Clone();
        double strongest = muscle.Max(t => t.Code);
        var msg = new List<string>();
        for (int p = 0; p < 2; p++)
        {
            var rel = StiffnessProjectionModel.CircularMeanDeg(relaxed.Where(t => t.Phase[p] is not null)
                .Select(t => t.Phase[p]!.Value));
            var tense = StiffnessProjectionModel.CircularMeanDeg(muscle
                .Where(t => t.Code >= strongest - 1e-9 && t.Phase[p] is not null).Select(t => t.Phase[p]!.Value));
            if (rel is null)
            {
                msg.Add($"{StiffnessProjectionModel.Pairs[p]}: no relaxed phase");
                continue;
            }
            index.RelaxedPhaseDeg[p] = rel.Value;
            if (tense is double ts && Math.Abs(StiffnessProjectionModel.WrapDeg(ts - rel.Value))
                >= StiffnessProjectionModel.MinDirectionChangeDeg)
                index.Direction[p] = TrendDirection(muscle.Where(t => t.Phase[p] is not null)
                    .Select(t => (t.Code, t.Phase[p]!.Value)).ToList(), rel.Value);
            msg.Add($"{StiffnessProjectionModel.Pairs[p]}: relaxed {rel.Value:F1} deg, strongest {Deg(tense)}, "
                    + $"direction {index.Direction[p]:+0;-0}");
        }
        Log("fit: " + string.Join("; ", msg));

        int single = _o.SingleIndex is null ? -1 : StiffnessProjectionModel.PairIndex(_o.SingleIndex);
        var usable = muscle.Where(t => (single >= 0 ? t.Phase[single] is not null
            : t.Phase[0] is not null && t.Phase[1] is not null)).ToList();
        if (usable.Count < 2)
        {
            Log("fit: fewer than two trials with the needed phases");
            return null;
        }
        var raw = usable.Select(t => Enumerable.Range(0, 2).Select(p =>
            StiffnessProjectionModel.RawIndex(index, p, t.Phase[single >= 0 ? single : p]!.Value)).ToArray()).ToList();
        if (single >= 0)
        {
            index.Direction[1 - single] = index.Direction[single];
            index.RelaxedPhaseDeg[1 - single] = index.RelaxedPhaseDeg[single];
        }
        for (int p = 0; p < 2; p++)
        {
            index.Scale[p] = Math.Max(raw.Max(r => r[p]), 1e-9);
            if (index.Scale[p] <= 1e-9)
                Log($"fit: warning, the {StiffnessProjectionModel.Pairs[p]} index never rises above relaxed");
        }

        var weight = new double[3, 3];
        foreach (char a in _o.Axes)
            foreach (char b in _o.Axes)
                weight["xyz".IndexOf(a), "xyz".IndexOf(b)] = 1.0;
        var fitTrials = usable.Select((t, i) =>
        {
            var k = new double[3, 3];
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    k[r, c] = t.K![r, c] - kEmpty[r, c];
            var sigma = raw[i].Select((v, p) => Math.Max(v, 0.0) / index.Scale[p]).ToArray();
            return new FitTrial(k, sigma, _arm.Geometry(t.Posture), weight);
        }).ToList();

        var model = new StiffnessProjectionModel(_arm, StiffnessFit.PriorGroups(_arm), index, _o.UpperArmDeg, _o.ElbowDeg);
        var res = StiffnessFit.Fit(model, fitTrials);
        model.SetGroupParameters(Enumerable.Range(0, model.Groups.Length).Select(g => (res.X[2 * g], res.X[2 * g + 1]))
            .ToList());

        // the two commands: overall stiffness (mean of the diagonal) per trial vs mean sigma
        _kRotEmpty = MeanMatrix(Trials.Where(t => t.Code < -0.5 && t.KRot is not null
                                                  && !_validationTrials.Contains(t.Number)).Select(t => t.KRot!).ToList());
        var sbar = fitTrials.Select(f => 0.5 * (f.Sigma[0] + f.Sigma[1])).ToList();
        var kt = usable.Select(t => Overall(t.K!, kEmpty)).ToList();
        model.TranslationCommand = FitCommand(sbar, kt, _o.TranslationCommandRange);
        var rotIdx = Enumerable.Range(0, usable.Count).Where(i => usable[i].KRot is not null).ToList();
        var kr = rotIdx.Select(i => Overall(usable[i].KRot!, _kRotEmpty)).ToList();
        model.RotationCommand = FitCommand(rotIdx.Select(i => sbar[i]).ToList(), kr, _o.RotationCommandRange);
        string Line(string name, StiffnessProjectionModel.CommandMap? c, List<double> x, List<double> y, string unit,
            string fmt) => c is null
            ? $"fit: {name} command: not enough trials with a {name}al stiffness"
            : $"fit: {name} command = {c.C0.ToString(fmt, CultureInfo.InvariantCulture)} + "
              + $"{c.C1.ToString(fmt, CultureInfo.InvariantCulture)} * mean sigma {unit}, clipped to {c.Min}-{c.Max}; "
              + "per trial (mean sigma: measured, line) " + string.Join(", ", x.Select((v, i) =>
                  $"{v:F2}: {y[i].ToString(fmt, CultureInfo.InvariantCulture)}, "
                  + $"{(c.C0 + c.C1 * v).ToString(fmt, CultureInfo.InvariantCulture)}"));
        Log(Line("translation", model.TranslationCommand, sbar, kt, "N/m", "F0"));
        Log(Line("rotation", model.RotationCommand, rotIdx.Select(i => sbar[i]).ToList(), kr, "Nm/rad", "F1"));
        model.FitReport = new JsonObject
        {
            ["date"] = DateTime.Now.ToString("s", CultureInfo.InvariantCulture),
            ["source"] = "PerturbationCalibration",
            ["force"] = _o.ForceMode,
            ["axes"] = _o.Axes,
            ["cost"] = res.Cost,
            ["iterations"] = res.Iterations,
            ["stiffness_trials"] = fitTrials.Count,
            ["empty_trials"] = empty.Count,
            ["trial_relative_error"] = new JsonArray(res.TrialErrors.Select(e => (JsonNode)e).ToArray()),
        };
        model.Save(_o.ModelPath);
        FittedModel = model;
        Log($"fit: {fitTrials.Count} trials, cost {res.Cost:E3}, relative error per trial "
            + string.Join(" ", res.TrialErrors.Select(e => e.ToString("F2", CultureInfo.InvariantCulture))));
        Log("fit: (a, b) " + string.Join(", ", model.Groups.Select(g => $"{g.Name} ({g.A:F3}, {g.B:F3})")));
        Log($"fit: model written to {_o.ModelPath}");
        SaveSession(model);
        return model;
    }

    /// <summary>Index direction for which the index rises with the effort code: score = concordant minus discordant
    /// trial pairs (code vs index); a tie goes to the direction with the smaller index at the strongest trials.</summary>
    private static double TrendDirection(List<(double Code, double Phase)> trials, double relaxedDeg)
    {
        double strongest = trials.Max(t => t.Code);
        (int Score, double Top) Rate(double d)
        {
            var idx = trials.Select(t => StiffnessProjectionModel.WrapIndexDeg(d * (t.Phase - relaxedDeg))).ToList();
            int score = 0;
            for (int i = 0; i < trials.Count; i++)
                for (int j = 0; j < trials.Count; j++)
                    if (trials[i].Code < trials[j].Code - 1e-9)
                        score += Math.Sign(idx[j] - idx[i]);
            return (score, idx.Where((_, i) => trials[i].Code >= strongest - 1e-9).Average());
        }
        var (up, down) = (Rate(1.0), Rate(-1.0));
        if (up.Score != down.Score)
            return up.Score > down.Score ? 1.0 : -1.0;
        return up.Top <= down.Top ? 1.0 : -1.0;
    }

    // --- session file ---

    private void SaveSession(StiffnessProjectionModel? model)
    {
        var dir = _o.LogDir ?? Path.GetDirectoryName(Path.GetFullPath(_o.ModelPath));
        if (string.IsNullOrEmpty(dir))
            return;
        try
        {
            Directory.CreateDirectory(dir);
            JsonArray Arr(IEnumerable<double> v) => new(v.Select(x => (JsonNode)x).ToArray());
            JsonArray Mat(double[,] m) => new(Enumerable.Range(0, 3)
                .Select(r => (JsonNode)Arr(Enumerable.Range(0, 3).Select(c => m[r, c]))).ToArray());
            var trials = new JsonArray(Trials.Select(t => (JsonNode)new JsonObject
            {
                ["number"] = t.Number, ["code"] = t.Code, ["label"] = Label(t.Code),
                ["K"] = t.K is null ? null : new JsonArray(Enumerable.Range(0, 3)
                    .Select(r => (JsonNode)Arr(Enumerable.Range(0, 3).Select(c => t.K[r, c]))).ToArray()),
                ["phase_deg"] = new JsonArray(t.Phase.Select(p => (JsonNode?)p).ToArray()),
                ["phase_packets"] = Arr(t.PhaseCounts.Select(c => (double)c)),
                ["imu"] = t.ImuQuaternions is null ? null : Arr(t.ImuQuaternions),
                ["K_rot"] = t.KRot is null ? null : Mat(t.KRot),
                ["validation"] = _validation.TryGetValue(t.Number, out var v) ? new JsonObject
                {
                    ["predicted_K"] = Mat(v.Predicted), ["measured_K_minus_empty"] = Mat(v.Measured),
                    ["relative_error"] = v.Error,
                    ["translation_predicted"] = v.TranslationPredicted, ["translation_command"] = v.TranslationCommand,
                    ["translation_measured"] = v.TranslationMeasured,
                    ["rotation_predicted"] = v.RotationPredicted, ["rotation_command"] = v.RotationCommand,
                    ["rotation_measured"] = v.RotationMeasured,
                } : _validationTrials.Contains(t.Number) ? new JsonObject() : null,
                ["cycles"] = new JsonArray(t.Cycles.Select(c => (JsonNode)new JsonObject
                {
                    ["dx"] = Arr(c.Dx), ["dx_d"] = Arr(c.DxDesired), ["dF"] = Arr(c.Df), ["rotation"] = c.Rotation,
                }).ToArray()),
            }).ToArray());
            var root = new JsonObject { ["trials"] = trials, ["model"] = model?.ToJson() };
            File.WriteAllText(Path.Combine(dir, $"calibration_{_sessionStamp}.json"),
                root.ToJsonString(new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception e)
        {
            Log($"could not write the session file: {e.Message}");
        }
    }

    // --- helpers ---

    private static StiffnessProjectionModel? TryLoad(string path)
    {
        try
        {
            return File.Exists(path) ? new StiffnessProjectionModel(path) : null;
        }
        catch (Exception)
        {
            return null;
        }
    }

    public static string Label(double code) => code switch
    {
        < -0.5 => "empty handle",
        _ when Math.Abs(code) < 1e-9 => "relaxed",
        >= FitCode - 0.5 => "fit",
        _ => $"effort {code * 100:F0} %",
    };

    private static double Norm4(double[] v, int s) =>
        Math.Sqrt(v[s] * v[s] + v[s + 1] * v[s + 1] + v[s + 2] * v[s + 2] + v[s + 3] * v[s + 3]);

    private static string Deg(double? d) => d is double v ? v.ToString("F1", CultureInfo.InvariantCulture) + " deg" : "none";

    private static string Fmt(double[] v, double scale) =>
        string.Join(" ", v.Select(x => (x * scale).ToString("F2", CultureInfo.InvariantCulture)));
}

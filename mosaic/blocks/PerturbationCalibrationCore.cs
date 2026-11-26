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
/// Quality check (<see cref="CheckTrials"/>): before the fit the fitting trials get a verdict GOOD / WEAK / REDO with
/// what to do; after the validation trials the validation gets PASS / FAIL / REPEAT VALIDATION
/// (<see cref="CheckValidation"/>). Both are logged and written to the session file.
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
/// then the model (<see cref="Options.ModelKind"/>):
///   hand (default, model file format 3, <see cref="HandModel"/>): drive d = mean sigma per trial; the 4 parameters
///     (a0, b, s_w, c_r) fitted with <see cref="StiffnessFit.FitHand"/> to the reduced translational and rotational
///     stiffness of each trial (K - K_empty, K_rot - K_rot,empty), the model being what this robot measures (flange
///     as reference point, <see cref="Options.HandleOffsetM"/>; the other degrees of freedom held by the robot springs
///     seen during the captures). Commands: the operator's reduced translational and rotational stiffness of the
///     projection at the mean drive of the relaxed and of the max trials span the slave's range.
///   groups (format 2): (a_g, b_g) fitted with <see cref="StiffnessFit.Fit"/>; commands: the overall translational and
///     rotational stiffness of each trial (mean of the diagonal) fitted as c0 + c1 * mean sigma, the line at the
///     relaxed and the max trials mapped onto the slave's range (StiffnessProjectionModel.CommandMap).
/// <see cref="Replay"/> runs the fit and the validation on the trials of a session file (offline check).
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
        public double[] TranslationCommandRange { get; set; } =
            (double[])StiffnessProjectionModel.SlaveTranslationRange.Clone();  // N/m
        public double[] RotationCommandRange { get; set; } =
            (double[])StiffnessProjectionModel.SlaveRotationRange.Clone();     // Nm/rad
        public int MinCycles { get; set; } = 3;
        /// <summary>"hand" (format 3, both commands from the projection) or "groups" (format 2).</summary>
        public string ModelKind { get; set; } = "hand";
        /// <summary>Hand model: "mean" (mean of the diagonal) or "volume" (cube root of the determinant).</summary>
        public string Reduction { get; set; } = "mean";
        /// <summary>Hand model: handle centre - flange of this robot, m, base frame (the robot turns its flange).</summary>
        public double[] HandleOffsetM { get; set; } = new double[3];
        public double Ridge { get; set; } = HandModel.DefaultRidge;
    }

    /// <summary>One push or turn. Hold: the robot stiffness that held the other degrees of freedom during the capture
    /// (translation: rotational K_r, Nm/rad; rotation: translational K_r, N/m), NaN if the state has none.</summary>
    public sealed record Cycle(double[] Dx, double[] DxDesired, double[] Df, bool Rotation = false,
        double Hold = double.NaN);

    /// <summary>A trial; PhaseSpread: circular standard deviation of each pair's phase over the trial (deg), Packets:
    /// feature vectors received during the trial (with or without a phase).</summary>
    public sealed record Trial(int Number, double Code, double[,]? K, List<Cycle> Cycles, double?[] Phase,
        int[] PhaseCounts, Posture Posture, double[]? ImuQuaternions, double[,]? KRot = null,
        double?[]? PhaseSpread = null, int Packets = 0);

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
    private CheckReport? _check;
    private JsonObject? _validationCheck;

    /// <summary>Validation trial: 3x3 model prediction vs measurement (translation), the two overall stiffnesses as the
    /// robot measured them and as the model predicts them (groups: the command lines, mean of the diagonal; hand: the
    /// reduced blocks of what the robot would measure), the command the slave would get, and the command the robot's
    /// measurement maps to (hand: the measurement moved to the handle centre with the model's ratio of operator to
    /// measured stiffness at the trial's drive).</summary>
    private sealed record Validation(double[,] Predicted, double[,] Measured, double Error, double TranslationMeasured,
        double TranslationPredicted, double TranslationCommand, double? RotationMeasured, double? RotationPredicted,
        double? RotationCommand, double TranslationFromMeasurement, double? RotationFromMeasurement);
    private readonly string _sessionStamp;
    private double[]? _replayDirection;  // Replay: the session's index direction instead of the previous model file's

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
                LogCheck();
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
            _cycles.Add(new Cycle(th, thd, dm, Rotation: true, Hold: _captureKr / _capture.N));
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
        _cycles.Add(new Cycle(dx, dxd, df, Hold: _capture.NRot > 0 ? _capture.KRot / _capture.NRot : double.NaN));
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
        var trial = new Trial(Trials.Count + 1, code, k, allCycles, _phases.Mean(), _phases.Counts, posture, q, kRot,
            _phases.Spread(), _phases.Total);
        Trials.Add(trial);
        Log($"trial {trial.Number} ({Label(code)}) done: {cycles.Count} cycles, "
            + (k is null ? $"NO stiffness (needs {_o.MinCycles}+ cycles in 3 directions)"
                : $"K diag {k[0, 0]:F0} {k[1, 1]:F0} {k[2, 2]:F0} N/m")
            + (rotCycles.Count == 0 ? "" : kRot is null ? $", {rotCycles.Count} turns, NO rotational stiffness"
                : $", {rotCycles.Count} turns, K_rot diag {kRot[0, 0]:F1} {kRot[1, 1]:F1} {kRot[2, 2]:F1} Nm/rad")
            + $", phase biceps {Deg(trial.Phase[0])} ({trial.PhaseCounts[0]} of {trial.Packets}), triceps "
            + $"{Deg(trial.Phase[1])} ({trial.PhaseCounts[1]} of {trial.Packets})");
        if (FittedModel is not null)
        {
            _validationTrials.Add(trial.Number);
            Validate(trial, FittedModel);
            CheckValidation(FittedModel);
        }
        else if (code >= 1.0 - 1e-9 && k is not null)
        {
            Log("full effort trial done: checking the trials and fitting the model");
            LogCheck();
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
        if (model.Hand is { } hand)
        {
            ValidateHand(trial, model, hand, sigma);
            return;
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
        _validation[trial.Number] = new Validation(pred, meas, err, tMeas, tPred, tCmd, rMeas, rPred, rCmd,
            model.TranslationCommand?.Map(tMeas) ?? double.NaN,
            rMeas is double rv ? model.RotationCommand?.Map(rv) : null);
        static string Capped(double predicted, double? command, string format) =>
            command is double c ? $", slave command {c.ToString(format)}" : "";
        Log($"validation trial {trial.Number} ({Label(trial.Code)}): sigma {sigma[0]:F2} {sigma[1]:F2}; "
            + $"translation predicted {tPred:F0} N/m vs measured {tMeas:F0} N/m ({RelDiff(tPred, tMeas)}"
            + $"{Capped(tPred, tCmd, "F0")}); "
            + (rMeas is double rm && rPred is double rp
                ? $"rotation predicted {rp:F1} Nm/rad vs measured {rm:F1} Nm/rad ({RelDiff(rp, rm)}{Capped(rp, rCmd, "F1")}); "
                : "no rotation command or measurement; ")
            + $"3x3 model: measured K diag {meas[0, 0]:F0} {meas[1, 1]:F0} {meas[2, 2]:F0}, predicted "
            + $"{pred[0, 0]:F0} {pred[1, 1]:F0} {pred[2, 2]:F0} N/m, relative error {err:F2}");
    }

    /// <summary>Validation with the hand model: drive of the trial; translational and rotational stiffness the robot
    /// measured vs what the model says it would measure; slave commands predicted vs from the measurement.</summary>
    private void ValidateHand(Trial trial, StiffnessProjectionModel model, HandModel hand, double[] sigma)
    {
        double d = StiffnessProjectionModel.Drive(sigma);
        var predicted = hand.Measured(d);  // session posture, as the fit
        var op = hand.Scalars(d);
        if (predicted is not { } pm || op is not { } o)
        {
            Log($"validation trial {trial.Number}: model not positive definite at drive {d:F2}");
            return;
        }
        var meas = Minus(trial.K!, _kEmpty);
        double num = 0, den = 0;
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                if (_o.Axes.Contains("xyz"[r]) && _o.Axes.Contains("xyz"[c]))
                {
                    num += (pm.T[r, c] - meas[r, c]) * (pm.T[r, c] - meas[r, c]);
                    den += meas[r, c] * meas[r, c];
                }
        double err = Math.Sqrt(num / Math.Max(den, 1e-12));
        double tMeas = HandModel.Reduce(meas, hand.Reduction), tPred = HandModel.Reduce(pm.T, hand.Reduction);
        double tCmd = model.TranslationCommand?.Map(o.T) ?? double.NaN;
        double tFromMeas = model.TranslationCommand?.Map(tMeas * o.T / tPred) ?? double.NaN;
        double? rMeas = trial.KRot is null ? null : HandModel.Reduce(Minus(trial.KRot, _kRotEmpty), hand.Reduction);
        double rPred = HandModel.Reduce(pm.R, hand.Reduction);
        double? rCmd = model.RotationCommand?.Map(o.R);
        double? rFromMeas = rMeas is double rm0 ? model.RotationCommand?.Map(rm0 * o.R / rPred) : null;
        _validation[trial.Number] = new Validation(pm.T, meas, err, tMeas, tPred, tCmd, rMeas, rPred, rCmd, tFromMeas,
            rFromMeas);
        string F(double? v, string f) => v is double x && double.IsFinite(x) ? x.ToString(f, CultureInfo.InvariantCulture) : "none";
        Log($"validation trial {trial.Number} ({Label(trial.Code)}): drive {d:F2}; translation measured {F(tMeas, "F0")} "
            + $"N/m, model {F(tPred, "F0")} N/m ({RelDiff(tPred, tMeas)}); rotation measured {F(rMeas, "F1")} Nm/rad, "
            + $"model {F(rPred, "F1")} Nm/rad" + (rMeas is double rm1 ? $" ({RelDiff(rPred, rm1)})" : "")
            + $"; slave commands translation {F(tCmd, "F0")} N/m (from the measurement {F(tFromMeas, "F0")}), rotation "
            + $"{F(rCmd, "F1")} Nm/rad (from the measurement {F(rFromMeas, "F1")}); 3x3: measured K diag {meas[0, 0]:F0} "
            + $"{meas[1, 1]:F0} {meas[2, 2]:F0}, model {pm.T[0, 0]:F0} {pm.T[1, 1]:F0} {pm.T[2, 2]:F0} N/m, relative "
            + $"error {err:F2}");
    }

    private static double[,] Minus(double[,] a, double[,] b)
    {
        var m = new double[3, 3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                m[r, c] = a[r, c] - b[r, c];
        return m;
    }

    /// <summary>Overall stiffness of a trial: mean of the diagonal of K - K_empty.</summary>
    private static double Overall(double[,] k, double[,] empty) =>
        (k[0, 0] - empty[0, 0] + k[1, 1] - empty[1, 1] + k[2, 2] - empty[2, 2]) / 3.0;

    private static string RelDiff(double predicted, double measured) =>
        $"{(predicted - measured) / Math.Max(Math.Abs(measured), 1e-9) * 100:+0;-0} %";

    /// <summary>Least-squares line c0 + c1 * x through the trials, with the operator's span from the line at the mean
    /// x of the relaxed (code 0) and of the max (code 1) trials; null with fewer than two distinct x.</summary>
    private static StiffnessProjectionModel.CommandMap? FitCommand(IReadOnlyList<double> x, IReadOnlyList<double> y,
        IReadOnlyList<double> codes, double[] range)
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
        double c1 = sxy / sxx, c0 = my - c1 * mx;
        double MeanX(Func<double, bool> code, double fallback)
        {
            var xs = Enumerable.Range(0, x.Count).Where(i => code(codes[i])).Select(i => x[i]).ToList();
            return xs.Count > 0 ? xs.Average() : fallback;
        }
        double low = c0 + c1 * MeanX(c => Math.Abs(c) < 1e-6, x.Min());
        double high = c0 + c1 * MeanX(c => c > 0.999, x.Max());
        return new StiffnessProjectionModel.CommandMap(c0, c1, range[0], range[1], low, high);
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
        if (_replayDirection is not null)
            index.Direction = (double[])_replayDirection.Clone();
        else if (previous is not null)
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
        // the trials' indices above use the default window; the run uses a window fitted to the calibrated change
        for (int p = 0; p < 2; p++)
            index.WindowHighDeg[p] = StiffnessProjectionModel.WindowHighFromScale(index.Scale[p]);
        Log($"fit: index scale biceps {index.Scale[0]:F1} deg, triceps {index.Scale[1]:F1} deg; window "
            + $"{index.WindowHighDeg[0] - 360:F0}..{index.WindowHighDeg[0]:F0} and "
            + $"{index.WindowHighDeg[1] - 360:F0}..{index.WindowHighDeg[1]:F0} deg");
        if (_o.ModelKind != "groups")
            return FitHand(usable, raw.Select(r => r.Select((v, p) => Math.Max(v, 0.0) / index.Scale[p]).ToArray()).ToList(),
                index, kEmpty, empty.Count);

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
        var codes = usable.Select(t => t.Code).ToList();
        model.TranslationCommand = FitCommand(sbar, kt, codes, _o.TranslationCommandRange);
        var rotIdx = Enumerable.Range(0, usable.Count).Where(i => usable[i].KRot is not null).ToList();
        var kr = rotIdx.Select(i => Overall(usable[i].KRot!, _kRotEmpty)).ToList();
        model.RotationCommand = FitCommand(rotIdx.Select(i => sbar[i]).ToList(), kr, rotIdx.Select(i => codes[i]).ToList(),
            _o.RotationCommandRange);
        string Line(string name, StiffnessProjectionModel.CommandMap? c, List<double> x, List<double> y, string unit,
            string fmt) => c is null
            ? $"fit: {name} command: not enough trials with a {name}al stiffness"
            : $"fit: {name} command = {c.C0.ToString(fmt, CultureInfo.InvariantCulture)} + "
              + $"{c.C1.ToString(fmt, CultureInfo.InvariantCulture)} * mean sigma {unit}; slave command: "
              + $"{c.HumanLow.ToString(fmt, CultureInfo.InvariantCulture)}-{c.HumanHigh.ToString(fmt, CultureInfo.InvariantCulture)}"
              + $" {unit} mapped onto {c.Min}-{c.Max}; "
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

    /// <summary>Fit of the hand model (format 3): see the class remarks.</summary>
    private StiffnessProjectionModel FitHand(List<Trial> usable, List<double[]> sigmas,
        StiffnessProjectionModel.IndexDefinition index, double[,] kEmpty, int emptyCount)
    {
        var fitted = Trials.Where(t => !_validationTrials.Contains(t.Number)).ToList();
        _kRotEmpty = MeanMatrix(fitted.Where(t => t.Code < -0.5 && t.KRot is not null).Select(t => t.KRot!).ToList());
        var drives = sigmas.Select(StiffnessProjectionModel.Drive).ToList();
        var trials = usable.Select((t, i) => new HandTrial(drives[i], Minus(t.K!, kEmpty),
            t.KRot is null ? null : Minus(t.KRot, _kRotEmpty))).ToList();
        var cycles = fitted.SelectMany(t => t.Cycles).ToList();
        double Hold(bool rotation, double fallback)
        {
            var h = cycles.Where(c => c.Rotation == rotation && double.IsFinite(c.Hold)).Select(c => c.Hold).ToList();
            return h.Count > 0 ? h.Average() : fallback;
        }
        double holdT = Hold(true, HandModel.RobotTranslationHold), holdR = Hold(false, HandModel.RobotRotationHold);
        var hand = new HandModel(_arm, _o.UpperArmDeg, _o.ElbowDeg, null, _o.Reduction, _o.HandleOffsetM, holdT, holdR);
        var res = StiffnessFit.FitHand(hand, trials, null, _o.Ridge);
        hand.SetParameters(res.Parameters);
        string Inv(double v, string f) => v.ToString(f, CultureInfo.InvariantCulture);
        Log($"fit: hand model (format 3), reduction {hand.Reduction}, ridge {_o.Ridge:G3}, handle offset "
            + $"[{Fmt(hand.HandleOffset, 1)}] m, robot holds {holdT:F0} N/m and {holdR:F1} Nm/rad"
            + (_o.ImuPosture ? "; all trials at the session posture (the hand model does not use the trial IMU postures yet)" : ""));
        Log($"fit: a0 {Inv(res.Parameters[0], "G4")}, b {Inv(res.Parameters[1], "G4")}, s_w {Inv(res.Parameters[2], "G4")}, "
            + $"c_r {Inv(res.Parameters[3], "G4")} Nm/rad (prior {string.Join(", ", HandModel.Prior.Select(v => Inv(v, "G3")))}); "
            + $"cost {res.Cost:E3}, {res.Iterations} iterations");
        foreach (int ch in new[] { 0, 1 })
        {
            var rows = res.Rows.Where(r => r.Channel == ch).ToList();
            Log($"fit: {(ch == 0 ? "translation" : "rotation")} per trial (drive: measured / model {(ch == 0 ? "N/m" : "Nm/rad")}) "
                + (rows.Count == 0 ? "no trial" : string.Join(", ", rows.Select(r =>
                    $"{drives[r.Trial]:F2}: {Inv(r.Measured, ch == 0 ? "F0" : "F1")} / "
                    + Inv(r.Measured * Math.Exp(r.LogError), ch == 0 ? "F0" : "F1")))));
        }
        if (res.Skipped.Count > 0)
            Log("fit: left out (measured stiffness not positive): " + string.Join(", ", res.Skipped.Select(r =>
                $"trial {usable[r.Trial].Number} {(r.Channel == 0 ? "translation" : "rotation")}")));
        if (!trials.Any(t => t.Rotation is not null))
            Log("fit: no trial has a rotational stiffness: s_w and c_r stay near the prior");

        double MeanDrive(Func<double, bool> code, double fallback)
        {
            var ds = Enumerable.Range(0, usable.Count).Where(i => code(usable[i].Code)).Select(i => drives[i]).ToList();
            return ds.Count > 0 ? ds.Average() : fallback;
        }
        double dLow = MeanDrive(c => Math.Abs(c) < 1e-6, drives.Min()), dHigh = MeanDrive(c => c > 0.999, drives.Max());
        var model = new StiffnessProjectionModel(hand, index);
        if (hand.Scalars(dLow) is { } lo && hand.Scalars(dHigh) is { } hi)
        {
            var rt = _o.TranslationCommandRange;
            var rr = _o.RotationCommandRange;
            model.TranslationCommand = new StiffnessProjectionModel.CommandMap(0, 0, rt[0], rt[1], lo.T, hi.T);
            model.RotationCommand = new StiffnessProjectionModel.CommandMap(0, 0, rr[0], rr[1], lo.R, hi.R);
            Log($"fit: translation command: operator {lo.T:F0}-{hi.T:F0} N/m (projection at drive {dLow:F2} and "
                + $"{dHigh:F2}) mapped onto {rt[0]}-{rt[1]} N/m; rotation command: operator {lo.R:F1}-{hi.R:F1} Nm/rad "
                + $"mapped onto {rr[0]}-{rr[1]} Nm/rad" + (hi.T <= lo.T ? "; WARNING: no span (max drive not above relaxed), "
                + "the commands stay at the minimum" : ""));
        }
        else
            Log("fit: model not positive definite at the relaxed or the max drive: no commands");
        model.FitReport = new JsonObject
        {
            ["date"] = DateTime.Now.ToString("s", CultureInfo.InvariantCulture),
            ["source"] = "PerturbationCalibration",
            ["model"] = "hand",
            ["force"] = _o.ForceMode,
            ["axes"] = _o.Axes,
            ["reduction"] = hand.Reduction,
            ["ridge"] = _o.Ridge,
            ["prior"] = new JsonArray(HandModel.Prior.Select(v => (JsonNode)v).ToArray()),
            ["cost"] = res.Cost,
            ["iterations"] = res.Iterations,
            ["stiffness_trials"] = trials.Count,
            ["empty_trials"] = emptyCount,
            ["drive_relaxed"] = dLow,
            ["drive_max"] = dHigh,
            ["log_error"] = new JsonArray(res.Rows.Select(r => (JsonNode)new JsonObject
            {
                ["trial"] = usable[r.Trial].Number, ["channel"] = r.Channel == 0 ? "translation" : "rotation",
                ["drive"] = drives[r.Trial], ["measured"] = r.Measured, ["log_error"] = r.LogError,
            }).ToArray()),
        };
        model.Save(_o.ModelPath);
        FittedModel = model;
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

    // --- quality check ---

    // Thresholds, backtested on the calibration sessions of 11-17 to 11-25 (MOSAIC logs): GOOD and PASS only for the
    // two sessions whose model worked, 11-19 17:26 and 11-23 14:05 UTC; REDO for the other five.
    public const double CheckMinPhaseShare = 0.8;          // share of a muscle trial's feature vectors with a phase
    public const double CheckMinSeparation = 2.0;          // |relaxed -> max phase change| / scatter within a trial
    public const double CheckScatterFloorDeg = 2.0;        // floor of that scatter
    public const double CheckMinEffortRatio = 1.5;         // robot: overall stiffness of max / relaxed
    public const double CheckMinValidationSpan = 0.5;      // validation: predicted command max - relaxed / slave range
    public const double CheckMaxValidationError = 0.25;    // validation: |predicted - measured command| / slave range

    public sealed record CheckReport(string Verdict, List<string> Lines, List<string> Actions);

    /// <summary>Quality check of the fitting trials: per clip the share of each muscle trial's feature vectors with a
    /// phase, and the relaxed -> max phase change against the scatter within a trial (root mean square of the
    /// circular standard deviations of the relaxed and max trials), with the medium trial between relaxed and max; the
    /// robot-measured overall stiffness (mean of the diagonal of K - K_empty) rising from relaxed over medium to max.
    /// Verdict GOOD, WEAK (a medium trial out of order) or REDO, with what to do.</summary>
    public static CheckReport CheckTrials(IReadOnlyList<Trial> fitTrials)
    {
        var lines = new List<string>();
        var actions = new List<string>();
        int worst = 0;
        var muscle = fitTrials.Where(t => t.Code > -0.5).ToList();
        var relaxed = muscle.Where(t => Math.Abs(t.Code) < 1e-9).ToList();
        double strongest = muscle.Count > 0 ? muscle.Max(t => t.Code) : 0.0;
        if (relaxed.Count == 0 || strongest <= 1e-9)
            return new CheckReport("REDO", new List<string> { "needs a relaxed trial and an effort trial" },
                new List<string> { "run the relaxed and the max trial" });
        var max = muscle.Where(t => t.Code >= strongest - 1e-9).ToList();
        var medium = muscle.Where(t => t.Code > 1e-9 && t.Code < strongest - 1e-9).ToList();
        for (int p = 0; p < 2; p++)
        {
            string name = p == 0 ? "biceps (clip A)" : "triceps (clip B)", clip = p == 0 ? "clip A" : "clip B";
            var lost = muscle.FirstOrDefault(t => PhaseShare(t, p) < CheckMinPhaseShare);
            if (lost is not null)
            {
                lines.Add($"{name}: phase in {PhaseShare(lost, p) * 100:F0} % of the {Label(lost.Code)} trial "
                          + $"({lost.PhaseCounts[p]} of {Math.Max(lost.Packets, lost.PhaseCounts[p])} packets): REDO");
                actions.Add($"{clip} loses its signal: press it on firmly or reseat it, check it with the bench test");
                worst = 2;
                continue;
            }
            double r0 = StiffnessProjectionModel.CircularMeanDeg(relaxed.Select(t => t.Phase[p]!.Value))!.Value;
            double change = StiffnessProjectionModel.WrapDeg(
                StiffnessProjectionModel.CircularMeanDeg(max.Select(t => t.Phase[p]!.Value))!.Value - r0);
            double scatter = Math.Max(Math.Sqrt(relaxed.Concat(max).Average(t => Sq(t.PhaseSpread?[p] ?? 0.0))),
                CheckScatterFloorDeg);
            double ratio = Math.Abs(change) / scatter;
            string head = $"{name}: relaxed -> max {change:+0;-0} deg, scatter within a trial {scatter:F0} deg, "
                          + $"ratio {ratio:F1}";
            if (ratio < CheckMinSeparation)
            {
                lines.Add(head + $" (needs {CheckMinSeparation:F1}): REDO");
                actions.Add($"{clip} does not separate relaxed from max: reseat {clip} or move the vibrator, check it "
                            + "with the bench test");
                worst = 2;
                continue;
            }
            if (medium.Count > 0)
            {
                double m = StiffnessProjectionModel.WrapDeg(StiffnessProjectionModel.CircularMeanDeg(
                    medium.Select(t => t.Phase[p]!.Value))!.Value - r0) * Math.Sign(change);
                if (m < -scatter || m > 1.5 * Math.Abs(change))
                {
                    lines.Add(head + $"; medium at {m:+0;-0} deg, not between relaxed and max: WEAK");
                    actions.Add("repeat the medium trial with an effort between relaxed and max");
                    worst = Math.Max(worst, 1);
                    continue;
                }
            }
            lines.Add(head + ": OK");
        }
        var kEmpty = MeanMatrix(fitTrials.Where(t => t.Code < -0.5 && t.K is not null).Select(t => t.K!).ToList());
        double? Level(List<Trial> ts)
        {
            var k = ts.Where(t => t.K is not null).Select(t => Overall(t.K!, kEmpty)).ToList();
            return k.Count > 0 ? k.Average() : null;
        }
        double? kr = Level(relaxed), kx = Level(max), km = medium.Count > 0 ? Level(medium) : null;
        if (kr is not double r || kx is not double x)
        {
            lines.Add("effort: no robot stiffness for the relaxed or the max trial: REDO");
            actions.Add("repeat that trial (it needs 3 or more pushes in 3 directions)");
            worst = 2;
        }
        else if (x < CheckMinEffortRatio * Math.Max(r, 1.0))
        {
            lines.Add($"effort: robot stiffness relaxed {r:F0} N/m, max {x:F0} N/m (max needs {CheckMinEffortRatio:F1}x "
                      + "relaxed): REDO");
            actions.Add("co-contract harder in the max trial and stay loose in the relaxed trial");
            worst = 2;
        }
        else if (km is double mid && !(r < mid && mid < x))
        {
            lines.Add($"effort: robot stiffness relaxed {r:F0}, medium {mid:F0}, max {x:F0} N/m, medium not between: WEAK");
            actions.Add("repeat the medium trial with an effort between relaxed and max");
            worst = Math.Max(worst, 1);
        }
        else
            lines.Add($"effort: robot stiffness relaxed {r:F0}" + (km is double m2 ? $", medium {m2:F0}" : "")
                      + $", max {x:F0} N/m: OK");
        return new CheckReport(new[] { "GOOD", "WEAK", "REDO" }[worst], lines, actions.Distinct().ToList());
    }

    private static double PhaseShare(Trial t, int p) => t.Packets > 0
        ? (double)t.PhaseCounts[p] / t.Packets
        : t.PhaseCounts[p] > 0 ? 1.0 : 0.0;

    private static double Sq(double v) => v * v;

    private void LogCheck()
    {
        _check = CheckTrials(Trials.Where(t => !_validationTrials.Contains(t.Number)).ToList());
        foreach (var line in _check.Lines)
            Log("CHECK " + line);
        Log($"VERDICT: {_check.Verdict} - " + _check.Verdict switch
        {
            "GOOD" => "both clips separate relaxed from max and the effort rose; run the validation trials (relaxed, "
                      + "medium, max)",
            "WEAK" => string.Join("; ", _check.Actions) + "; or run the validation trials and go by their check",
            _ => string.Join("; ", _check.Actions) + "; then reopen session_calibration.json and repeat the "
                 + "calibration (the model file below is written anyway: do not use it)",
        });
    }

    /// <summary>Validation check, after each validation trial once a relaxed and a max validation trial exist (the
    /// latest of each): PASS if the robot measured the max trial at CheckMinEffortRatio times the relaxed one, the
    /// predicted slave command of max lies at least CheckMinValidationSpan of the slave range above relaxed, and both
    /// predictions lie within CheckMaxValidationError of the range of the command the robot's measurement maps to;
    /// REPEAT VALIDATION if the robot saw no stiffening; FAIL otherwise.</summary>
    private void CheckValidation(StiffnessProjectionModel model)
    {
        if (model.TranslationCommand is not { } map)
            return;
        Validation? Latest(Func<double, bool> select) => Trials
            .Where(t => select(t.Code) && _validation.ContainsKey(t.Number)).Select(t => _validation[t.Number])
            .LastOrDefault();
        var vr = Latest(c => Math.Abs(c) < 1e-9);
        var vx = Latest(c => c >= 1.0 - 1e-9);
        var vm = Latest(c => c > 1e-9 && c < 1.0 - 1e-9);
        if (vr is null || vx is null)
            return;
        double range = map.Max - map.Min;
        double Measured(Validation v) => v.TranslationFromMeasurement;
        double effort = vx.TranslationMeasured / Math.Max(vr.TranslationMeasured, 1.0);
        bool span = vx.TranslationCommand - vr.TranslationCommand >= CheckMinValidationSpan * range;
        bool close = Math.Abs(vr.TranslationCommand - Measured(vr)) <= CheckMaxValidationError * range
                     && Math.Abs(vx.TranslationCommand - Measured(vx)) <= CheckMaxValidationError * range;
        string result = effort < CheckMinEffortRatio ? "REPEAT VALIDATION" : span && close ? "PASS" : "FAIL";
        string Pair(string label, Validation v) =>
            $"{label} {v.TranslationCommand:F0} / {Measured(v):F0}";
        string RotPair(string label, Validation v) => v.RotationCommand is double c && v.RotationFromMeasurement is double m
            ? $"{label} {c:F1} / {m:F1}" : $"{label} none";
        string detail = "slave command predicted / from the robot's measurement: " + Pair("relaxed", vr)
                        + (vm is null ? "" : ", " + Pair("medium", vm)) + ", " + Pair("max", vx)
                        + $" N/m; robot max / relaxed {effort:F1}x"
                        + (model.RotationCommand is null ? "" : "; rotation (not part of the verdict) "
                           + RotPair("relaxed", vr) + (vm is null ? "" : ", " + RotPair("medium", vm)) + ", "
                           + RotPair("max", vx) + " Nm/rad");
        Log("VALIDATION CHECK: " + detail);
        Log($"VALIDATION: {result} - " + result switch
        {
            "PASS" => "the calibration is good; open session_run.json",
            "REPEAT VALIDATION" => $"the robot measured no stiffening in the max validation trial (needs "
                                   + $"{CheckMinEffortRatio:F1}x relaxed); repeat the validation trials with a loose "
                                   + "relaxed and a hard max",
            _ => "the model does not predict the validation trials; reopen session_calibration.json and repeat the "
                 + "calibration",
        });
        _validationCheck = new JsonObject { ["result"] = result, ["detail"] = detail };
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
                ["phase_spread_deg"] = new JsonArray((t.PhaseSpread ?? new double?[2]).Select(p => (JsonNode?)p).ToArray()),
                ["packets"] = t.Packets,
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
                    ["hold"] = double.IsFinite(c.Hold) ? c.Hold : null,
                }).ToArray()),
            }).ToArray());
            var root = new JsonObject
            {
                ["trials"] = trials, ["model"] = model?.ToJson(),
                ["check"] = _check is null ? null : new JsonObject
                {
                    ["verdict"] = _check.Verdict,
                    ["lines"] = new JsonArray(_check.Lines.Select(l => (JsonNode)l).ToArray()),
                    ["actions"] = new JsonArray(_check.Actions.Select(a => (JsonNode)a).ToArray()),
                },
                ["validation_check"] = _validationCheck?.DeepClone(),
            };
            File.WriteAllText(Path.Combine(dir, $"calibration_{_sessionStamp}.json"),
                root.ToJsonString(new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception e)
        {
            Log($"could not write the session file: {e.Message}");
        }
    }

    // --- replay ---

    /// <summary>Offline check of the fit on a logged session (calibration_*.json of this block): its trials are added in
    /// order; the trials before the first validation trial are checked and fitted (as the session did, or at the end if
    /// the session had no validation trial), the later ones validated. Trials take the session posture of the options
    /// (or their IMU posture with ImuPosture). Writes the model file and a new session file like a live session.</summary>
    public StiffnessProjectionModel? Replay(JsonElement session)
    {
        // the session started from the direction of its previous model file, which a replay does not have; the
        // session's own direction stands in for it (it is that direction wherever the trials did not set one)
        if (session.TryGetProperty("model", out var m) && m.ValueKind == JsonValueKind.Object
            && m.TryGetProperty("index", out var ix) && ix.TryGetProperty("direction", out var dir)
            && dir.ValueKind == JsonValueKind.Object)
            _replayDirection = StiffnessProjectionModel.Pairs.Select(p => dir.GetProperty(p).GetDouble()).ToArray();
        bool fitted = false;
        foreach (var t in session.GetProperty("trials").EnumerateArray())
        {
            bool validation = t.TryGetProperty("validation", out var v) && v.ValueKind == JsonValueKind.Object;
            if (validation && !fitted)
            {
                LogCheck();
                Fit();
                fitted = true;
            }
            var trial = ParseTrial(t, Trials.Count + 1);
            Trials.Add(trial);
            Log($"replay: trial {trial.Number} ({Label(trial.Code)})" + (fitted ? ", validation" : ""));
            if (fitted && FittedModel is not null)
            {
                _validationTrials.Add(trial.Number);
                Validate(trial, FittedModel);
                CheckValidation(FittedModel);
            }
        }
        if (!fitted)
        {
            LogCheck();
            Fit();
        }
        return FittedModel;
    }

    private Trial ParseTrial(JsonElement t, int number)
    {
        static double[,]? Mat(JsonElement e)
        {
            if (e.ValueKind != JsonValueKind.Array)
                return null;
            var m = new double[3, 3];
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    m[r, c] = e[r][c].GetDouble();
            return m;
        }
        static double[] Vec(JsonElement e) => e.EnumerateArray().Select(x => x.GetDouble()).ToArray();
        static double?[] Opt(JsonElement e) => e.EnumerateArray()
            .Select(x => x.ValueKind == JsonValueKind.Number ? x.GetDouble() : (double?)null).ToArray();
        var cycles = t.GetProperty("cycles").EnumerateArray().Select(c => new Cycle(Vec(c.GetProperty("dx")),
            Vec(c.GetProperty("dx_d")), Vec(c.GetProperty("dF")), c.GetProperty("rotation").GetBoolean(),
            c.TryGetProperty("hold", out var h) && h.ValueKind == JsonValueKind.Number ? h.GetDouble() : double.NaN)).ToList();
        double[]? imu = t.TryGetProperty("imu", out var q) && q.ValueKind == JsonValueKind.Array ? Vec(q) : null;
        var posture = _o.ImuPosture && imu is not null
            ? ArmModel.FromQuaternions(imu[..4], imu[4..8], ArmModel.SessionTorso)
            : ArmModel.SessionPosture(_o.UpperArmDeg, _o.ElbowDeg);
        var counts = t.GetProperty("phase_packets").EnumerateArray().Select(x => (int)x.GetDouble()).ToArray();
        return new Trial(number, t.GetProperty("code").GetDouble(), Mat(t.GetProperty("K")), cycles,
            Opt(t.GetProperty("phase_deg")), counts, posture, imu,
            t.TryGetProperty("K_rot", out var kr) ? Mat(kr) : null,
            t.TryGetProperty("phase_spread_deg", out var sp) ? Opt(sp) : null,
            t.TryGetProperty("packets", out var pk) ? pk.GetInt32() : 0);
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

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
/// (without the program's state, e.g. with a Stimulus block, stop capture ends the trial); code 9
/// ("fit") fits the model to all trials of the session and writes the model file.
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
/// A trial with three or more cycles spanning 3-D gives K = dF dx^T (dx dx^T)^-1 (symmetrised); its phases are the
/// circular means of all feature vectors during the trial; its posture is fixed or the mean IMU posture.
/// Fit: K_empty (mean of empty trials) is subtracted; relaxed reference = circular mean of the relaxed trials;
/// direction per pair = sign of the phase change from relaxed to the strongest trials; index per trial, scale = its
/// largest value, sigma = max(index, 0) / scale; (a_g, b_g) fitted with <see cref="StiffnessFit.Fit"/>.
/// Robot state vector (perturbation.cpp): t, x, y, z, x_d, y_d, z_d, F_ext x, y, z, q_w, q_x, q_y, q_z, K_r, phase,
/// trial number.
/// </remarks>
public sealed class PerturbationCalibrationCore
{
    public const int RobotStateLength = 15;
    public const int RobotStateLengthWithPhase = 17;
    public const double FitCode = 9.0;

    public sealed class Options
    {
        public string ModelPath { get; set; } = "stiffness_projection.json";
        public string? LogDir { get; set; }
        public string ForceMode { get; set; } = "robot";
        public double? ForceSign { get; set; }  // default +1 for wrench, -1 for loadcell
        public bool ImuPosture { get; set; }
        public double UpperArmDeg { get; set; }
        public double ElbowDeg { get; set; } = 90.0;
        public string Axes { get; set; } = "xyz";
        public double[] ArmLengths { get; set; } = { 0.30, 0.27, 0.07 };
        public string IndexKind { get; set; } = "phase";
        public double DriveHz { get; set; } = 45.0;
        public double SpacingM { get; set; } = 0.02;
        public double LagOffsetDeg { get; set; }
        public string? SingleIndex { get; set; }
        public double RestSettleS { get; set; } = 1.0;
        public double CaptureSettleS { get; set; } = 0.5;
        public int MinCycles { get; set; } = 3;
    }

    public sealed record Cycle(double[] Dx, double[] DxDesired, double[] Df);

    public sealed record Trial(int Number, double Code, double[,]? K, List<Cycle> Cycles, double?[] Phase,
        int[] PhaseCounts, Posture Posture, double[]? ImuQuaternions);

    private sealed class Window
    {
        public readonly double[] X = new double[3], Xd = new double[3], F = new double[3];
        public int N;

        public void Clear()
        {
            Array.Clear(X);
            Array.Clear(Xd);
            Array.Clear(F);
            N = 0;
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

    private void CloseTrial()
    {
        double code = _trialCode!.Value;
        var cycles = _cycles ?? new List<Cycle>();
        _trialCode = null;
        if (_boundRobotTrial != 0)
            _closedRobotTrial = _boundRobotTrial;
        _boundRobotTrial = 0;
        _cycles = null;
        var k = cycles.Count >= _o.MinCycles
            ? StiffnessFit.Identify(cycles.Select(c => c.Dx).ToList(), cycles.Select(c => c.Df).ToList())
            : null;
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
        var trial = new Trial(Trials.Count + 1, code, k, cycles, _phases.Mean(), _phases.Counts, posture, q);
        Trials.Add(trial);
        Log($"trial {trial.Number} ({Label(code)}) done: {cycles.Count} cycles, "
            + (k is null ? $"NO stiffness (needs {_o.MinCycles}+ cycles in 3 directions)"
                : $"K diag {k[0, 0]:F0} {k[1, 1]:F0} {k[2, 2]:F0} N/m")
            + $", phase biceps {Deg(trial.Phase[0])} ({trial.PhaseCounts[0]}), triceps {Deg(trial.Phase[1])} "
            + $"({trial.PhaseCounts[1]})");
        SaveSession(null);
    }

    // --- fit ---

    public StiffnessProjectionModel? Fit()
    {
        if (_trialCode is not null)
            CloseTrial();
        var withK = Trials.Where(t => t.K is not null).ToList();
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
                index.Direction[p] = Math.Sign(StiffnessProjectionModel.WrapDeg(ts - rel.Value));
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
            var trials = new JsonArray(Trials.Select(t => (JsonNode)new JsonObject
            {
                ["number"] = t.Number, ["code"] = t.Code, ["label"] = Label(t.Code),
                ["K"] = t.K is null ? null : new JsonArray(Enumerable.Range(0, 3)
                    .Select(r => (JsonNode)Arr(Enumerable.Range(0, 3).Select(c => t.K[r, c]))).ToArray()),
                ["phase_deg"] = new JsonArray(t.Phase.Select(p => (JsonNode?)p).ToArray()),
                ["phase_packets"] = Arr(t.PhaseCounts.Select(c => (double)c)),
                ["imu"] = t.ImuQuaternions is null ? null : Arr(t.ImuQuaternions),
                ["cycles"] = new JsonArray(t.Cycles.Select(c => (JsonNode)new JsonObject
                {
                    ["dx"] = Arr(c.Dx), ["dx_d"] = Arr(c.DxDesired), ["dF"] = Arr(c.Df),
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

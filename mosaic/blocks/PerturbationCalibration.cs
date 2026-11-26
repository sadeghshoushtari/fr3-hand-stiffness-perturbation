using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using Microsoft.Extensions.DependencyInjection;
using MOSAIC.Components.Basics;

namespace MOSAIC.Models.Elastography;

/// <summary>
/// Perturbation calibration of the projection model inside MOSAIC (see <see cref="PerturbationCalibrationCore"/>):
/// identifies the hand stiffness per trial from the perturbation program's state (pushes and their phases), records the
/// elastography phases,
/// and on "fit" fits the model and writes the model file that StiffnessProjection reads.
/// </summary>
/// <example>
/// <code>
/// "Calibration": {
///   "Type": "PerturbationCalibration",
///   "Inputs": [ "ESP32", "Robot", "LoadCell", "IMU", "Effort" ],
///   "Params": [ "/home/f_airob/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/mosaic_models/stiffness_projection.json", "force:robot", "posture:fixed;90;90" ],
///   "Path": "/home/f_airob/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/mosaic_logs/calibration/"
/// }
/// </code>
/// Params[0]: model file to write. Further Params, "key:value":
///   force:robot | force:wrench[;sign] | force:loadcell[;sign]   restoring force (default robot; sign default +1 for
///       wrench, the FR3 convention, and -1 for loadcell)
///   posture:fixed;&lt;upper arm deg&gt;;&lt;elbow deg&gt; | posture:imu;&lt;upper arm deg&gt;;&lt;elbow deg&gt;
///       (imu: trial posture from the IMU input, the angles are the session posture written to the model file)
///   axes:xyz (Cartesian axes fitted), arm:0.30;0.27;0.07 (upper arm, forearm, hand in m), index:phase|speed2,
///   drive_hz:45, spacing_m:0.02, lag_offset_deg:0, single:biceps|triceps, settle_s:0.4 (rest settling),
///   capture_settle_s:0.35 (hold settling), min_cycles:3, range_t:300;700 (N/m) and range_r:20;40 (Nm/rad) (slave
///   ranges of the translational and rotational commands: the operator's relaxed..max span is mapped onto them),
///   model:hand|groups (hand, default: the two-pair hand model, model file format 3, both commands from the
///   projection; groups: the group model of format 2 with the command lines), reduction:mean|volume (hand model, how
///   a 3x3 block becomes one number, default mean), handle_offset:x;y;z (hand model: handle centre - robot flange in m,
///   robot base frame, default 0;0;0), ridge:0.3 (hand model: pull of the parameters towards the prior)
///   esp32:, robot:, loadcell:, imu:, plan:, effort: names of the input blocks (defaults ESP32, Robot, LoadCell, IMU,
///   PerturbationPlan, Effort).
/// The cycles come from the phase in the robot state (perturbation.cpp); a Stimulus block (plan:) can drive them instead.
/// Output per finished trial: Vector [trial, code, cycles, kxx, kyy, kzz, kxy, kxz, kyz] (K zeros if not identified);
/// after a fit: Vector [-1, 9, trials, a0, b, s_w, c_r] (hand model) or [-1, 9, trials, a_1, b_1, ..., a_4, b_4]
/// (groups).
/// </example>
public sealed class PerturbationCalibration : BaseBlock
{
    private readonly object _lock = new();
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private readonly PerturbationCalibrationCore _core;
    private readonly List<object> _params;
    private readonly Dictionary<string, string> _roles;
    private int _publishedTrials;
    private StiffnessProjectionModel? _fitPublished;

    public PerturbationCalibration(string name, double desiredRate, List<object> parameters, string? logDir)
        : base(name, desiredRate)
    {
        _params = parameters;
        var opt = new PerturbationCalibrationCore.Options
        {
            ModelPath = parameters[0]?.ToString() ?? "stiffness_projection.json",
            LogDir = string.IsNullOrWhiteSpace(logDir) ? null : logDir,
        };
        _roles = new Dictionary<string, string>
        {
            ["esp32"] = "ESP32", ["robot"] = "Robot", ["loadcell"] = "LoadCell", ["imu"] = "IMU",
            ["plan"] = "PerturbationPlan", ["effort"] = "Effort",
        };
        foreach (var (key, value) in StiffnessProjection.Roles(parameters.Skip(1)))
        {
            var v = value.Split(';', StringSplitOptions.TrimEntries);
            switch (key)
            {
                case "force":
                    opt.ForceMode = v[0].ToLowerInvariant();
                    if (v.Length > 1) opt.ForceSign = Num(v[1]);
                    break;
                case "posture":
                    opt.ImuPosture = v[0].Equals("imu", StringComparison.OrdinalIgnoreCase);
                    if (v.Length > 2)
                    {
                        opt.UpperArmDeg = Num(v[1]);
                        opt.ElbowDeg = Num(v[2]);
                    }
                    break;
                case "axes": opt.Axes = value.ToLowerInvariant(); break;
                case "arm": opt.ArmLengths = v.Select(Num).ToArray(); break;
                case "index": opt.IndexKind = value; break;
                case "drive_hz": opt.DriveHz = Num(value); break;
                case "spacing_m": opt.SpacingM = Num(value); break;
                case "lag_offset_deg": opt.LagOffsetDeg = Num(value); break;
                case "single": opt.SingleIndex = value; break;
                case "settle_s": opt.RestSettleS = Num(value); break;
                case "capture_settle_s": opt.CaptureSettleS = Num(value); break;
                case "min_cycles": opt.MinCycles = (int)Num(value); break;
                case "range_t": opt.TranslationCommandRange = v.Select(Num).ToArray(); break;
                case "range_r": opt.RotationCommandRange = v.Select(Num).ToArray(); break;
                case "model": opt.ModelKind = value.ToLowerInvariant(); break;
                case "reduction": opt.Reduction = value.ToLowerInvariant(); break;
                case "handle_offset": opt.HandleOffsetM = v.Select(Num).ToArray(); break;
                case "ridge": opt.Ridge = Num(value); break;
                default:
                    if (_roles.ContainsKey(key)) _roles[key] = value;
                    else throw new ArgumentException($"[PerturbationCalibration] unknown parameter '{key}'");
                    break;
            }
        }
        if (opt.ForceMode is not ("robot" or "wrench" or "loadcell"))
            throw new ArgumentException($"[PerturbationCalibration] force must be robot, wrench or loadcell");
        if (opt.ModelKind is not ("hand" or "groups"))
            throw new ArgumentException("[PerturbationCalibration] model must be hand or groups");
        if (opt.Reduction is not ("mean" or "volume"))
            throw new ArgumentException("[PerturbationCalibration] reduction must be mean or volume");
        if (opt.HandleOffsetM.Length != 3)
            throw new ArgumentException("[PerturbationCalibration] handle_offset needs x;y;z in m");
        _core = new PerturbationCalibrationCore(opt) { Log = Report };
        Report($"model file {opt.ModelPath}, force {opt.ForceMode}, posture "
               + (opt.ImuPosture ? "IMU" : $"fixed {opt.UpperArmDeg};{opt.ElbowDeg} deg") + $", axes {opt.Axes}, model "
               + opt.ModelKind + (opt.ModelKind == "hand"
                   ? $" (reduction {opt.Reduction}, handle offset {string.Join(";", opt.HandleOffsetM)} m)" : ""));
    }

    public static PerturbationCalibration ConfigureInput(IServiceProvider sp, JsonModel m)
    {
        if (m.Params is null || m.Params.Count < 1)
            throw new ArgumentException($"[PerturbationCalibration] '{m.Name}' requires Params[0] = model file path.");
        return ActivatorUtilities.CreateInstance<PerturbationCalibration>(sp, m.Name ?? "PerturbationCalibration",
            m.DesiredRate ?? 0, m.Params.Select(p => (object)(p?.ToString() ?? "")).ToList(), m.Path ?? "");
    }

    public override int MaxInputs => 6;

    protected override string JsonTypeName => "PerturbationCalibration";

    protected override IReadOnlyList<object>? GetJsonParams() => _params;

    public PerturbationCalibrationCore Core => _core;

    protected override void OnReceive(object sender, object data)
    {
        var source = (sender as BaseBlock)?.Name ?? "";
        double t = _clock.Elapsed.TotalSeconds;
        lock (_lock)
        {
            if (source == _roles["plan"])
            {
                if (data is ValueTuple<string, Vector<double>> stim)
                    _core.OnStimulus(stim.Item1, t);
            }
            else if (source == _roles["effort"])
            {
                var v = data as Vector<double>;
                _core.OnEffort(v is null ? null : v.Count > 0 ? v[0] : 0.0);
            }
            else if (data is Vector<double> vec)
            {
                var a = vec.ToArray();
                if (source == _roles["robot"]) _core.OnRobot(a, t);
                else if (source == _roles["esp32"]) _core.OnFeatures(a);
                else if (source == _roles["imu"]) _core.OnImu(a);
                else if (source == _roles["loadcell"]) _core.OnLoadCell(a);
            }
            PublishResults();
        }
    }

    private void PublishResults()
    {
        while (_publishedTrials < _core.Trials.Count)
        {
            var tr = _core.Trials[_publishedTrials++];
            var k = tr.K ?? new double[3, 3];
            Publish(Vector<double>.Build.DenseOfArray(new[]
            {
                tr.Number, tr.Code, tr.Cycles.Count, k[0, 0], k[1, 1], k[2, 2], k[0, 1], k[0, 2], k[1, 2],
            }));
        }
        if (_core.FittedModel is { } model && !ReferenceEquals(model, _fitPublished))
        {
            Publish(Vector<double>.Build.DenseOfArray(new[] { -1.0, PerturbationCalibrationCore.FitCode, _core.Trials.Count }
                .Concat(model.Hand?.P ?? model.Groups.SelectMany(g => new[] { g.A, g.B })).ToArray()));
            _fitPublished = model;
        }
    }

    private void Report(string message)
    {
        Debug.WriteLine($"[{Name}] {message}");
        Console.WriteLine($"[{Name}] {message}");
    }

    private static double Num(string s) => double.Parse(s, CultureInfo.InvariantCulture);
}

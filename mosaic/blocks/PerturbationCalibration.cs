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
/// identifies the hand stiffness per trial from the perturbation program's state, records the elastography phases,
/// and on "fit" fits the model and writes the model file that StiffnessProjection reads.
/// </summary>
/// <example>
/// <code>
/// "Calibration": {
///   "Type": "PerturbationCalibration",
///   "Inputs": [ "ESP32", "Robot", "LoadCell", "IMU", "PerturbationPlan", "Effort" ],
///   "Params": [ "/home/f_airob/mosaic_models/stiffness_projection.json", "force:robot", "posture:fixed;0;90" ],
///   "Path": "/home/f_airob/mosaic_logs/calibration/"
/// }
/// </code>
/// Params[0]: model file to write. Further Params, "key:value":
///   force:robot | force:wrench[;sign] | force:loadcell[;sign]   restoring force (default robot; sign default +1 for
///       wrench, the FR3 convention, and -1 for loadcell)
///   posture:fixed;&lt;upper arm deg&gt;;&lt;elbow deg&gt; | posture:imu;&lt;upper arm deg&gt;;&lt;elbow deg&gt;
///       (imu: trial posture from the IMU input, the angles are the session posture written to the model file)
///   axes:xyz (Cartesian axes fitted), arm:0.30;0.27;0.07 (upper arm, forearm, hand in m), index:phase|speed2,
///   drive_hz:45, spacing_m:0.02, lag_offset_deg:0, single:biceps|triceps, settle_s:1, min_cycles:3
///   esp32:, robot:, loadcell:, imu:, plan:, effort: names of the input blocks (defaults ESP32, Robot, LoadCell, IMU,
///   PerturbationPlan, Effort).
/// Output per finished trial: Vector [trial, code, cycles, kxx, kyy, kzz, kxy, kxz, kyz] (K zeros if not identified);
/// after a fit: Vector [-1, 9, trials, a_1, b_1, ..., a_4, b_4].
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
                case "min_cycles": opt.MinCycles = (int)Num(value); break;
                default:
                    if (_roles.ContainsKey(key)) _roles[key] = value;
                    else throw new ArgumentException($"[PerturbationCalibration] unknown parameter '{key}'");
                    break;
            }
        }
        if (opt.ForceMode is not ("robot" or "wrench" or "loadcell"))
            throw new ArgumentException($"[PerturbationCalibration] force must be robot, wrench or loadcell");
        _core = new PerturbationCalibrationCore(opt) { Log = Report };
        Report($"model file {opt.ModelPath}, force {opt.ForceMode}, posture "
               + (opt.ImuPosture ? "IMU" : $"fixed {opt.UpperArmDeg};{opt.ElbowDeg} deg") + $", axes {opt.Axes}");
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
                .Concat(model.Groups.SelectMany(g => new[] { g.A, g.B })).ToArray()));
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

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using Microsoft.Extensions.DependencyInjection;
using MOSAIC.Components.Basics;

namespace MOSAIC.Models.Elastography;

/// <summary>
/// Elastography feature vector (+ IMU posture) -> hand stiffness for the robot, with the muscle-space projection
/// model (see <see cref="StiffnessProjectionModel"/>). Sets the session reference from a Trigger block.
/// </summary>
/// <example>
/// <code>
/// "Stiffness": {
///   "Type": "StiffnessProjection",
///   "Inputs": [ "ESP32", "IMU", "Session" ],
///   "Params": [ "/home/f_airob/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/mosaic_models/stiffness_projection.json", "imu:IMU", "session:Session" ],
///   "Path": "/home/f_airob/Mohammadsadegh_Shoushtaridehshal_Master_Thesis/mosaic_logs/run/"
/// }
/// </code>
/// Params[0]: model file (stiffness_projection.json, from the PerturbationCalibration block or
/// projection/fit_model_from_perturbation.py). Further Params name the blocks of the optional inputs:
/// "imu:&lt;block&gt;" (default IMU), "session:&lt;block&gt;" (default Session). Every other input is the feature vector.
/// Inputs:
///   features: Vector of the esp32_robot_stiffness output (12 doubles).
///   IMU: Vector of 8 values, unit quaternions (w, x, y, z) of the upper-arm and the forearm frame in the robot base
///     frame (sensor-to-segment alignment done upstream). Sets the posture (model format 2 only); without it the
///     session posture of the model file is used.
///   Session: Trigger block with the actions "relaxed:0" and "tense:1". Start capture records the phases, stop
///     capture averages them: "relaxed" sets the relaxed reference of each pair, "tense" sets the direction of each
///     index (the sign of the phase change on contraction). The model file is saved after each (format 2).
/// Output: Vector [kxx, kyy, kzz, kxy, kxz, kyz, k_volume, sigma_biceps, sigma_triceps, valid], stiffness in N/m in
/// the robot base frame.
/// </example>
public sealed class StiffnessProjection : BaseBlock
{
    private readonly object _lock = new();
    private readonly StiffnessProjectionModel _model;
    private readonly string _modelPath;
    private readonly string _imuName, _sessionName;
    private readonly PhaseCapture _capture = new();
    private double? _captureCode;
    private Posture? _pendingPosture;

    public StiffnessProjection(string name, double desiredRate, string modelPath, string imuName, string sessionName)
        : base(name, desiredRate)
    {
        _modelPath = modelPath;
        _imuName = imuName;
        _sessionName = sessionName;
        _model = new StiffnessProjectionModel(modelPath);
        Report($"model {modelPath} ({_model.Format}): index {_model.Kind}, {_model.MuscleCount} muscles");
    }

    public static StiffnessProjection ConfigureInput(IServiceProvider sp, JsonModel m)
    {
        if (m.Params is null || m.Params.Count < 1)
            throw new ArgumentException($"[StiffnessProjection] '{m.Name}' requires Params[0] = model file path.");
        var roles = Roles(m.Params.Skip(1));
        return ActivatorUtilities.CreateInstance<StiffnessProjection>(sp, m.Name ?? "StiffnessProjection",
            m.DesiredRate ?? 0, m.Params[0]?.ToString() ?? string.Empty,
            roles.GetValueOrDefault("imu", "IMU"), roles.GetValueOrDefault("session", "Session"));
    }

    /// <summary>"role:block" entries of Params.</summary>
    internal static Dictionary<string, string> Roles(IEnumerable<object?> parameters) => parameters
        .Select(p => p?.ToString() ?? "")
        .Where(s => s.Contains(':'))
        .Select(s => s.Split(':', 2))
        .ToDictionary(kv => kv[0].Trim().ToLowerInvariant(), kv => kv[1].Trim());

    public override int MaxInputs => 3;

    protected override string JsonTypeName => "StiffnessProjection";

    protected override IReadOnlyList<object>? GetJsonParams() =>
        new List<object> { _modelPath, $"imu:{_imuName}", $"session:{_sessionName}" };

    protected override void OnReceive(object sender, object data)
    {
        var source = (sender as BaseBlock)?.Name;
        lock (_lock)
        {
            if (source == _sessionName)
                OnSession(data as Vector<double>);
            else if (source == _imuName)
                OnImu(data as Vector<double>);
            else if (data is Vector<double> features)
                OnFeatures(features.ToArray());
        }
    }

    private void OnFeatures(double[] features)
    {
        if (_pendingPosture is not null)
        {
            _model.SetPosture(_pendingPosture);
            _pendingPosture = null;
        }
        if (_captureCode is not null)
            _capture.Add(_model.Phases(features));
        Publish(Vector<double>.Build.DenseOfArray(_model.Step(features)));
    }

    private void OnImu(Vector<double>? q)
    {
        if (q is null || q.Count < 8 || _model.Arm is null)
            return;
        var v = q.ToArray();
        double n1 = Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
        double n2 = Math.Sqrt(v[4] * v[4] + v[5] * v[5] + v[6] * v[6] + v[7] * v[7]);
        if (Math.Abs(n1 - 1) > 0.1 || Math.Abs(n2 - 1) > 0.1)
            return;  // not a valid orientation (IMU not running yet)
        _pendingPosture = ArmModel.FromQuaternions(v[..4], v[4..8], ArmModel.SessionTorso);
    }

    private void OnSession(Vector<double>? action)
    {
        if (action is not null)  // start capture
        {
            _captureCode = action.Count > 0 ? action[0] : 0.0;
            _capture.Clear();
            return;
        }
        if (_captureCode is not double code)
            return;
        _captureCode = null;
        var mean = _capture.Mean();
        var counts = _capture.Counts;
        string msg = code < 0.5 ? _model.SetRelaxed(mean) : _model.SetDirectionFromTense(mean);
        Report($"{(code < 0.5 ? "relaxed" : "tense")} capture ({counts[0]}/{counts[1]} packets): {msg}");
        if (_model.Format == StiffnessProjectionModel.Format2)
        {
            try
            {
                _model.Save(_modelPath);
            }
            catch (Exception e)
            {
                Report($"could not save {_modelPath}: {e.Message}");
            }
        }
    }

    private void Report(string message)
    {
        Debug.WriteLine($"[{Name}] {message}");
        Console.WriteLine($"[{Name}] {message}");
    }
}

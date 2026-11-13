using System;
using System.Collections.Generic;
using System.Diagnostics;
using MathNet.Numerics.LinearAlgebra;
using Microsoft.Extensions.DependencyInjection;
using MOSAIC.Components.Basics;

namespace MOSAIC.Models.Elastography;

/// <summary>
/// Elastography feature vector -> hand stiffness for the robot, with the muscle-space projection model
/// (see <see cref="StiffnessProjectionModel"/>).
/// </summary>
/// <example>
/// <code>
/// "Stiffness": {
///   "Type": "StiffnessProjection",
///   "Inputs": [ "ESP32" ],
///   "Params": [ "C:/Users/Sadegh/Desktop/mosaic_models/stiffness_projection.json" ],
///   "Path": "C:/Users/Sadegh/Desktop/mosaic_logs/"
/// }
/// </code>
/// Params[0]: model file written by projection/fit_model_from_perturbation.py (stiffness_projection.json).
/// Input: Vector of the esp32_robot_stiffness output. Output: Vector [kxx, kyy, kzz, kxy, kxz, kyz, k_volume,
/// sigma_biceps, sigma_triceps, valid], stiffness in N/m in the robot base frame.
/// </example>
public sealed class StiffnessProjection : BaseBlock
{
    private readonly StiffnessProjectionModel _model;
    private readonly string _modelPath;

    public StiffnessProjection(string name, double desiredRate, string modelPath) : base(name, desiredRate)
    {
        _modelPath = modelPath;
        _model = new StiffnessProjectionModel(modelPath);
        Debug.WriteLine($"[{Name}] model {modelPath}: index {_model.Kind}, {_model.MuscleCount} muscles");
    }

    public static StiffnessProjection ConfigureInput(IServiceProvider sp, JsonModel m)
    {
        if (m.Params is null || m.Params.Count < 1)
            throw new ArgumentException($"[StiffnessProjection] '{m.Name}' requires Params[0] = model file path.");
        var name = m.Name ?? "StiffnessProjection";
        var instance = ActivatorUtilities.CreateInstance<StiffnessProjection>(
            sp, name, m.DesiredRate ?? 0, m.Params[0]?.ToString() ?? string.Empty);
        instance.InitDumper(sp, m.Path, $"{name}_stiffness");
        return instance;
    }

    protected override string JsonTypeName => "StiffnessProjection";

    protected override IReadOnlyList<object>? GetJsonParams() => new List<object> { _modelPath };

    protected override void OnReceive(object sender, object data)
    {
        if (data is not Vector<double> features)
            return;
        Publish(Vector<double>.Build.DenseOfArray(_model.Step(features.ToArray())));
    }
}

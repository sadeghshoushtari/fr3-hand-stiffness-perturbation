using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Nodes;

namespace MOSAIC.Models.Elastography;

/// <summary>
/// Elastography features + arm posture -> Cartesian hand stiffness (N/m, robot base frame) with the muscle-space model
/// of projection/stiffness_model.py. Reads the model file written by projection/mosaic_export.py or by the
/// PerturbationCalibration block; projection/mosaic_export.py is the reference this class is checked against
/// (mosaic/blocks/test).
/// </summary>
/// <remarks>
/// Model file formats:
///   "stiffness_projection/1": geometry (J, J_m, H) evaluated at the session posture, index direction -1 for both pairs.
///   "stiffness_projection/2": arm lengths and session posture; the geometry is evaluated here with <see cref="ArmModel"/>,
///   at the session posture or at the posture set by <see cref="SetPosture"/> (IMUs); index direction per pair.
/// Per feature vector (esp32_robot_stiffness output, 12 doubles):
///   phase of each pair = atan2(sin, cos); index "phase" = direction * (phase - relaxed) wrapped to [-90, 270) deg
///   (<see cref="WrapIndexDeg"/>), "speed2" =
///   (c / c_relaxed)^2 - 1 with c = 360 f d / lag, lag = (offset - phase) mod 360;
///   sigma = clip(max(index, 0) / scale, 0, sigma_max); u_g = a_g + b_g sum_p w_gp sigma_p; f = F_max u; k = gamma f / l0;
///   K_j = J_m^T diag(k) J_m + sum_i f_i d2l_i/dq2; K_c = (J K_j^-1 J^T)^-1.
/// Output: kxx, kyy, kzz, kxy, kxz, kyz, k_volume (cube root of det), sigma_biceps, sigma_triceps, valid,
/// k_translation_cmd (N/m), k_rotation_cmd (Nm/rad). The two commands are the operator's overall translational and
/// rotational stiffness (mean of the diagonal) from the calibration's linear maps over the mean sigma,
/// clip(c0 + c1 (sigma_biceps + sigma_triceps) / 2, min, max) ("commands" of the model file); without them the
/// translational command is clip(k_volume, 50, 800) and the rotational one 0 (no command).
/// A vector without a usable phase (cos = sin = 0) or with K_j not positive definite repeats the last valid
/// output with valid = 0 (all zeros before the first valid one).
/// </remarks>
public sealed class StiffnessProjectionModel
{
    public const string Format1 = "stiffness_projection/1";
    public const string Format2 = "stiffness_projection/2";
    public const int OutputLength = 12;
    public const int ValidIndex = 9;
    public static readonly string[] Pairs = { "biceps", "triceps" };
    public static readonly string[] OutputNames =
    {
        "kxx", "kyy", "kzz", "kxy", "kxz", "kyz", "k_volume", "sigma_biceps", "sigma_triceps", "valid",
        "k_translation_cmd", "k_rotation_cmd",
    };

    /// <summary>Linear map of the mean sigma to an overall stiffness (Line), clipped to [Min, Max] for the slave
    /// (Eval).</summary>
    public sealed record CommandMap(double C0, double C1, double Min, double Max)
    {
        public double Line(double[] sigma) => C0 + C1 * 0.5 * (sigma[0] + sigma[1]);
        public double Eval(double[] sigma) => Math.Clamp(Line(sigma), Min, Max);
    }

    /// <summary>One muscle group: members (muscle indices), driver weights over <see cref="Pairs"/>, u = a + b drive.</summary>
    public sealed record Group(string Name, int[] Members, double[] Weights, double A, double B);

    /// <summary>Index definition of a session (projection/fit_model_from_perturbation.py "index").</summary>
    public sealed class IndexDefinition
    {
        public string Kind { get; set; } = "phase";
        public double DriveHz { get; set; } = 45.0;
        public double SpacingM { get; set; } = 0.02;
        public double LagOffsetDeg { get; set; }
        public string? SingleIndex { get; set; }
        public double SigmaMax { get; set; } = 1.5;
        public double[] Scale { get; set; } = { 1.0, 1.0 };
        public double[] RelaxedPhaseDeg { get; set; } = { 0.0, 0.0 };
        public double[] Direction { get; set; } = { -1.0, -1.0 };
        public int[][] Features { get; set; } = { new[] { 8, 9 }, new[] { 10, 11 } };  // biceps: clip A, triceps: clip B (11-17)
    }

    private readonly double _gamma;
    private readonly string[] _muscles;
    private readonly double[] _fMax, _l0;
    private readonly ArmModel? _arm;  // null for format 1 (fixed geometry)
    private double[,] _j;      // 3 x 4
    private double[,] _jm;     // M x 4
    private double[,,] _h;     // M x 4 x 4
    private readonly int _single;  // index into Pairs, or -1
    private double[]? _last;

    public Group[] Groups { get; private set; }
    public IndexDefinition Index { get; }
    public double SessionUpperArmDeg { get; }
    public double SessionElbowDeg { get; }
    public string Format { get; }
    public JsonNode? FitReport { get; set; }
    public CommandMap? TranslationCommand { get; set; }
    public CommandMap? RotationCommand { get; set; }

    public string Kind => Index.Kind;
    public int MuscleCount => _fMax.Length;
    public ArmModel? Arm => _arm;

    public StiffnessProjectionModel(string path) : this(JsonDocument.Parse(File.ReadAllText(path)).RootElement) { }

    public StiffnessProjectionModel(JsonElement root)
    {
        Format = root.GetProperty("format").GetString() ?? "";
        if (Format != Format1 && Format != Format2)
            throw new InvalidDataException($"model file format is neither {Format1} nor {Format2}");
        _gamma = root.GetProperty("gamma").GetDouble();
        _muscles = root.TryGetProperty("muscles", out var mus)
            ? mus.EnumerateArray().Select(m => m.GetString() ?? "").ToArray()
            : Array.Empty<string>();

        if (Format == Format1)
        {
            _fMax = Vector(root.GetProperty("f_max"));
            _l0 = Vector(root.GetProperty("l0"));
            _j = Matrix(root.GetProperty("J"), 3, 4);
            _jm = Matrix(root.GetProperty("Jm"), _fMax.Length, 4);
            _h = new double[_fMax.Length, 4, 4];
            var h = root.GetProperty("H");
            for (int i = 0; i < _fMax.Length; i++)
            {
                var hi = Matrix(h[i], 4, 4);
                for (int r = 0; r < 4; r++)
                    for (int c = 0; c < 4; c++)
                        _h[i, r, c] = hi[r, c];
            }
            if (root.TryGetProperty("posture", out var p1) && p1.ValueKind == JsonValueKind.Object)
            {
                SessionUpperArmDeg = p1.GetProperty("upper_arm_deg").GetDouble();
                SessionElbowDeg = p1.GetProperty("elbow_deg").GetDouble();
            }
        }
        else
        {
            var arm = root.GetProperty("arm");
            _arm = new ArmModel(arm.GetProperty("upper_arm_m").GetDouble(), arm.GetProperty("forearm_m").GetDouble(),
                arm.GetProperty("hand_m").GetDouble());
            if (!_muscles.SequenceEqual(_arm.Names))
                throw new InvalidDataException("muscle list of the model file does not match ArmModel.Muscles");
            _fMax = (double[])_arm.FMax.Clone();
            _l0 = (double[])_arm.L0.Clone();
            var p2 = root.GetProperty("posture");
            SessionUpperArmDeg = p2.GetProperty("upper_arm_deg").GetDouble();
            SessionElbowDeg = p2.GetProperty("elbow_deg").GetDouble();
            (_j, _jm, _h) = Unpack(_arm.Geometry(ArmModel.SessionPosture(SessionUpperArmDeg, SessionElbowDeg)));
            if (root.TryGetProperty("fit", out var fit))
                FitReport = JsonNode.Parse(fit.GetRawText());
        }

        Groups = root.GetProperty("groups").EnumerateArray().Select(g =>
        {
            var weights = new double[2];
            foreach (var d in g.GetProperty("drivers").EnumerateObject())
                weights[PairIndex(d.Name)] = d.Value.GetDouble();
            return new Group(g.TryGetProperty("name", out var n) ? n.GetString() ?? "" : "",
                g.GetProperty("members").EnumerateArray().Select(m => m.GetInt32()).ToArray(), weights,
                g.GetProperty("a").GetDouble(), g.GetProperty("b").GetDouble());
        }).ToArray();

        var idx = root.GetProperty("index");
        Index = new IndexDefinition
        {
            Kind = idx.GetProperty("kind").GetString() ?? "phase",
            DriveHz = idx.GetProperty("drive_hz").GetDouble(),
            SpacingM = idx.GetProperty("spacing_m").GetDouble(),
            LagOffsetDeg = idx.GetProperty("lag_offset_deg").GetDouble(),
            SigmaMax = idx.GetProperty("sigma_max").GetDouble(),
            SingleIndex = idx.GetProperty("single_index").ValueKind == JsonValueKind.String
                ? idx.GetProperty("single_index").GetString() : null,
        };
        for (int p = 0; p < 2; p++)
        {
            Index.Scale[p] = idx.GetProperty("scale").GetProperty(Pairs[p]).GetDouble();
            Index.RelaxedPhaseDeg[p] = idx.GetProperty("relaxed_phase_deg").GetProperty(Pairs[p]).GetDouble();
            if (idx.TryGetProperty("direction", out var dir))
                Index.Direction[p] = dir.GetProperty(Pairs[p]).GetDouble();
            var f = idx.GetProperty("features").GetProperty(Pairs[p]);
            Index.Features[p] = new[] { f[0].GetInt32(), f[1].GetInt32() };
        }
        if (Index.Kind != "phase" && Index.Kind != "speed2")
            throw new InvalidDataException($"unknown index kind '{Index.Kind}'");
        if (root.TryGetProperty("commands", out var cmds) && cmds.ValueKind == JsonValueKind.Object)
        {
            static CommandMap? Map(JsonElement c, string name) =>
                c.TryGetProperty(name, out var m) && m.ValueKind == JsonValueKind.Object
                    ? new CommandMap(m.GetProperty("c0").GetDouble(), m.GetProperty("c1").GetDouble(),
                        m.GetProperty("min").GetDouble(), m.GetProperty("max").GetDouble())
                    : null;
            TranslationCommand = Map(cmds, "translation");
            RotationCommand = Map(cmds, "rotation");
        }
        _single = Index.SingleIndex is null ? -1 : PairIndex(Index.SingleIndex);
    }

    /// <summary>New format-2 model (written by the PerturbationCalibration block).</summary>
    public StiffnessProjectionModel(ArmModel arm, IEnumerable<Group> groups, IndexDefinition index,
        double sessionUpperArmDeg, double sessionElbowDeg, double gamma = 23.4)
    {
        Format = Format2;
        _gamma = gamma;
        _arm = arm;
        _muscles = arm.Names;
        _fMax = (double[])arm.FMax.Clone();
        _l0 = (double[])arm.L0.Clone();
        Groups = groups.ToArray();
        Index = index;
        SessionUpperArmDeg = sessionUpperArmDeg;
        SessionElbowDeg = sessionElbowDeg;
        _single = index.SingleIndex is null ? -1 : PairIndex(index.SingleIndex);
        (_j, _jm, _h) = Unpack(arm.Geometry(ArmModel.SessionPosture(sessionUpperArmDeg, sessionElbowDeg)));
    }

    /// <summary>Evaluates the geometry at a measured posture (format 2 only; format 1 keeps its fixed geometry).</summary>
    public bool SetPosture(Posture posture)
    {
        if (_arm is null)
            return false;
        (_j, _jm, _h) = Unpack(_arm.Geometry(posture));
        return true;
    }

    public void SetGroupParameters(IReadOnlyList<(double A, double B)> parameters)
    {
        Groups = Groups.Select((g, i) => g with { A = parameters[i].A, B = parameters[i].B }).ToArray();
    }

    /// <summary>One feature vector in, one output vector (OutputLength values) out.</summary>
    public double[] Step(IReadOnlyList<double> features)
    {
        var sigma = Sigmas(features);
        var kc = sigma is null ? null : Stiffness(sigma);
        if (sigma is null || kc is null)
        {
            var held = _last is null ? new double[OutputLength] : (double[])_last.Clone();
            held[ValidIndex] = 0.0;
            return held;
        }
        double det = Det3(kc);
        double volume = det > 0 ? Math.Cbrt(det) : 0.0;
        _last = new[]
        {
            kc[0, 0], kc[1, 1], kc[2, 2], kc[0, 1], kc[0, 2], kc[1, 2], volume, sigma[0], sigma[1], 1.0,
            TranslationCommand?.Eval(sigma) ?? Math.Clamp(volume, 50.0, 800.0), RotationCommand?.Eval(sigma) ?? 0.0,
        };
        return (double[])_last.Clone();
    }

    /// <summary>Phase (deg) of each pair (biceps, triceps), null where the pair has no usable phase.</summary>
    public double?[] Phases(IReadOnlyList<double> features)
    {
        var phase = new double?[2];
        for (int p = 0; p < 2; p++)
        {
            int ci = Index.Features[p][0], si = Index.Features[p][1];
            if (Math.Max(ci, si) >= features.Count)
                continue;  // short vector: no phase
            double c = features[ci], s = features[si];
            if (c * c + s * s > 0.25)
                phase[p] = Math.Atan2(s, c) * 180.0 / Math.PI;
        }
        return phase;
    }

    /// <summary>sigma per pair (biceps, triceps), or null if a needed pair has no phase.</summary>
    public double[]? Sigmas(IReadOnlyList<double> features)
    {
        var phase = Phases(features);
        var raw = new double?[2];
        for (int p = 0; p < 2; p++)
            if (phase[p] is double ph)
                raw[p] = RawIndex(Index, p, ph);
        if (_single >= 0)
            raw[0] = raw[1] = raw[_single];
        if (raw[0] is null || raw[1] is null)
            return null;
        var sigma = new double[2];
        for (int p = 0; p < 2; p++)
            sigma[p] = Math.Min(Math.Max(raw[p]!.Value, 0.0) / Index.Scale[p], Index.SigmaMax);
        return sigma;
    }

    /// <summary>Raw index of pair p (0 = biceps, 1 = triceps) at a phase, relative to the relaxed reference.</summary>
    public static double RawIndex(IndexDefinition index, int p, double phaseDeg)
    {
        double relaxedDeg = index.RelaxedPhaseDeg[p];
        if (index.Kind == "phase")
            return WrapIndexDeg(index.Direction[p] * (phaseDeg - relaxedDeg));
        double lag = Mod360(index.LagOffsetDeg - phaseDeg);
        double lagRelaxed = Mod360(index.LagOffsetDeg - relaxedDeg);
        double ratio = Math.Max(lagRelaxed, 1.0) / Math.Max(lag, 1.0);  // c / c_relaxed, c = 360 f d / lag
        return ratio * ratio - 1.0;
    }

    /// <summary>K_c (3 x 3) for given sigmas, or null if K_j is not positive definite.</summary>
    public double[,]? Stiffness(double[] sigma) => Stiffness(sigma, Groups, _j, _jm, _h);

    /// <summary>K_c for given sigmas, group parameters and geometry (used by the calibration fit).</summary>
    public double[,]? Stiffness(double[] sigma, IReadOnlyList<Group> groups, double[,] j, double[,] jm, double[,,] h)
    {
        int m = _fMax.Length;
        var u = new double[m];
        foreach (var g in groups)
        {
            double level = g.A + g.B * (g.Weights[0] * sigma[0] + g.Weights[1] * sigma[1]);
            foreach (var i in g.Members) u[i] = level;
        }
        var kj = new double[4, 4];
        for (int i = 0; i < m; i++)
        {
            double f = _fMax[i] * u[i];
            double k = _gamma * f / _l0[i];
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    kj[r, c] += k * jm[i, r] * jm[i, c] + f * h[i, r, c];
        }
        for (int r = 0; r < 4; r++)
            for (int c = r + 1; c < 4; c++)
                kj[r, c] = kj[c, r] = 0.5 * (kj[r, c] + kj[c, r]);

        var l = Cholesky(kj);
        if (l is null) return null;
        // compliance at the hand: C = J K_j^-1 J^T
        var x = new double[4, 3];
        for (int col = 0; col < 3; col++)
        {
            var rhs = new double[4];
            for (int r = 0; r < 4; r++) rhs[r] = j[col, r];
            var sol = CholeskySolve(l, rhs);
            for (int r = 0; r < 4; r++) x[r, col] = sol[r];
        }
        var comp = new double[3, 3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                for (int k = 0; k < 4; k++)
                    comp[r, c] += j[r, k] * x[k, c];
        var kc = Inverse3(comp);
        if (kc is null) return null;
        for (int r = 0; r < 3; r++)
            for (int c = r + 1; c < 3; c++)
                kc[r, c] = kc[c, r] = 0.5 * (kc[r, c] + kc[c, r]);
        return kc;
    }

    // --- session reference ---

    /// <summary>Minimum phase change (deg) between the relaxed and the tense capture that sets an index direction.</summary>
    public const double MinDirectionChangeDeg = 2.0;

    /// <summary>Sets the relaxed reference phase of each pair that has one; returns what changed.</summary>
    public string SetRelaxed(double?[] meanPhaseDeg)
    {
        var msg = new List<string>();
        for (int p = 0; p < 2; p++)
        {
            if (meanPhaseDeg[p] is not double v)
            {
                msg.Add($"{Pairs[p]}: no phase, relaxed reference kept at {Index.RelaxedPhaseDeg[p]:F1} deg");
                continue;
            }
            msg.Add($"{Pairs[p]}: relaxed reference {Index.RelaxedPhaseDeg[p]:F1} -> {v:F1} deg");
            Index.RelaxedPhaseDeg[p] = v;
        }
        return string.Join("; ", msg);
    }

    /// <summary>Sets the index direction of each pair from a tense capture: the calibration direction is kept when
    /// the index rises in it (a change of up to 270 deg, see <see cref="WrapIndexDeg"/>) and reversed when the phase
    /// moved the other way (by up to 90 deg). A change below <see cref="MinDirectionChangeDeg"/> keeps the
    /// direction.</summary>
    public string SetDirectionFromTense(double?[] meanPhaseDeg)
    {
        var msg = new List<string>();
        for (int p = 0; p < 2; p++)
        {
            if (meanPhaseDeg[p] is not double v)
            {
                msg.Add($"{Pairs[p]}: no phase, direction kept at {Index.Direction[p]:+0;-0}");
                continue;
            }
            double change = WrapIndexDeg(Index.Direction[p] * (v - Index.RelaxedPhaseDeg[p]));
            if (Math.Abs(change) < MinDirectionChangeDeg)
            {
                msg.Add($"{Pairs[p]}: change {change:F1} deg too small, direction kept at {Index.Direction[p]:+0;-0}");
                continue;
            }
            if (change < 0)
                Index.Direction[p] = -Index.Direction[p];
            msg.Add($"{Pairs[p]}: index change {Math.Abs(change):F1} deg, direction {Index.Direction[p]:+0;-0}");
        }
        return string.Join("; ", msg);
    }

    // --- model file ---

    /// <summary>Model file in format 2 (format-1 models cannot be re-saved: they have no arm lengths).</summary>
    public JsonObject ToJson()
    {
        if (_arm is null)
            throw new InvalidOperationException("a format-1 model has no arm lengths; refit or re-export it as format 2");
        JsonObject PerPair(double[] v) => new() { [Pairs[0]] = v[0], [Pairs[1]] = v[1] };
        var groups = new JsonArray();
        foreach (var g in Groups)
        {
            var drivers = new JsonObject();
            for (int p = 0; p < 2; p++)
                if (g.Weights[p] != 0)
                    drivers[Pairs[p]] = g.Weights[p];
            groups.Add(new JsonObject
            {
                ["name"] = g.Name, ["members"] = new JsonArray(g.Members.Select(i => (JsonNode)i).ToArray()),
                ["drivers"] = drivers, ["a"] = g.A, ["b"] = g.B,
            });
        }
        return new JsonObject
        {
            ["format"] = Format2,
            ["output"] = new JsonArray(OutputNames.Select(n => (JsonNode)n).ToArray()),
            ["gamma"] = _gamma,
            ["muscles"] = new JsonArray(_muscles.Select(n => (JsonNode)n).ToArray()),
            ["arm"] = new JsonObject
            {
                ["upper_arm_m"] = _arm.UpperArmM, ["forearm_m"] = _arm.ForearmM, ["hand_m"] = _arm.HandM,
            },
            ["posture"] = new JsonObject { ["upper_arm_deg"] = SessionUpperArmDeg, ["elbow_deg"] = SessionElbowDeg },
            ["groups"] = groups,
            ["index"] = new JsonObject
            {
                ["kind"] = Index.Kind, ["drive_hz"] = Index.DriveHz, ["spacing_m"] = Index.SpacingM,
                ["lag_offset_deg"] = Index.LagOffsetDeg, ["single_index"] = Index.SingleIndex,
                ["scale"] = PerPair(Index.Scale), ["relaxed_phase_deg"] = PerPair(Index.RelaxedPhaseDeg),
                ["direction"] = PerPair(Index.Direction), ["sigma_max"] = Index.SigmaMax,
                ["features"] = new JsonObject
                {
                    [Pairs[0]] = new JsonArray(Index.Features[0][0], Index.Features[0][1]),
                    [Pairs[1]] = new JsonArray(Index.Features[1][0], Index.Features[1][1]),
                },
            },
            ["fit"] = FitReport?.DeepClone(),
            ["commands"] = TranslationCommand is null && RotationCommand is null ? null : new JsonObject
            {
                ["translation"] = CommandJson(TranslationCommand, "N/m"),
                ["rotation"] = CommandJson(RotationCommand, "Nm/rad"),
            },
        };
    }

    private static JsonObject? CommandJson(CommandMap? c, string unit) => c is null ? null : new JsonObject
    {
        ["c0"] = c.C0, ["c1"] = c.C1, ["min"] = c.Min, ["max"] = c.Max, ["unit"] = unit,
        ["input"] = "mean of sigma_biceps and sigma_triceps",
    };

    /// <summary>Writes the model file; an existing file is kept as &lt;path&gt;.bak.</summary>
    public void Save(string path)
    {
        var text = ToJson().ToJsonString(new JsonSerializerOptions { WriteIndented = true });
        var dir = Path.GetDirectoryName(Path.GetFullPath(path));
        if (dir is not null)
            Directory.CreateDirectory(dir);
        if (File.Exists(path))
            File.Copy(path, path + ".bak", overwrite: true);
        var tmp = path + ".tmp";
        File.WriteAllText(tmp, text);
        File.Move(tmp, path, overwrite: true);
    }

    // --- helpers ---

    public static int PairIndex(string name) => Array.IndexOf(Pairs, name) is var i and >= 0
        ? i : throw new InvalidDataException($"unknown index '{name}'");

    public static double Mod360(double a) => a - 360.0 * Math.Floor(a / 360.0);

    public static double WrapDeg(double a) => Mod360(a + 180.0) - 180.0;

    /// <summary>Lower end (deg) of the window of the phase index.</summary>
    public const double IndexWindowLowDeg = 90.0;

    /// <summary>Wraps a signed phase change into [-90, 270) deg: a change of up to 270 deg in the contraction direction
    /// counts (triceps, run of 11-17: +212 deg at max effort, read as -148 deg by <see cref="WrapDeg"/>).</summary>
    public static double WrapIndexDeg(double a) => Mod360(a + IndexWindowLowDeg) - IndexWindowLowDeg;

    /// <summary>Circular mean of angles in degrees, or null for an empty list.</summary>
    public static double? CircularMeanDeg(IEnumerable<double> deg)
    {
        double c = 0, s = 0;
        int n = 0;
        foreach (var d in deg)
        {
            c += Math.Cos(d * Math.PI / 180.0);
            s += Math.Sin(d * Math.PI / 180.0);
            n++;
        }
        return n == 0 ? null : Math.Atan2(s, c) * 180.0 / Math.PI;
    }

    public static (double[,] J, double[,] Jm, double[,,] H) Unpack(ArmGeometry g) => (g.J, g.Jm, g.H);

    private static double[] Vector(JsonElement e)
    {
        var v = new double[e.GetArrayLength()];
        for (int i = 0; i < v.Length; i++) v[i] = e[i].GetDouble();
        return v;
    }

    private static double[,] Matrix(JsonElement e, int rows, int cols)
    {
        if (e.GetArrayLength() != rows)
            throw new InvalidDataException($"expected {rows} rows, got {e.GetArrayLength()}");
        var m = new double[rows, cols];
        for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
                m[r, c] = e[r][c].GetDouble();
        return m;
    }

    private static double[,]? Cholesky(double[,] a)
    {
        int n = a.GetLength(0);
        var l = new double[n, n];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                double s = a[i, j];
                for (int k = 0; k < j; k++) s -= l[i, k] * l[j, k];
                if (i == j)
                {
                    if (s <= 0) return null;
                    l[i, i] = Math.Sqrt(s);
                }
                else
                {
                    l[i, j] = s / l[j, j];
                }
            }
        }
        return l;
    }

    private static double[] CholeskySolve(double[,] l, double[] b)
    {
        int n = b.Length;
        var y = new double[n];
        for (int i = 0; i < n; i++)
        {
            double s = b[i];
            for (int k = 0; k < i; k++) s -= l[i, k] * y[k];
            y[i] = s / l[i, i];
        }
        var x = new double[n];
        for (int i = n - 1; i >= 0; i--)
        {
            double s = y[i];
            for (int k = i + 1; k < n; k++) s -= l[k, i] * x[k];
            x[i] = s / l[i, i];
        }
        return x;
    }

    public static double Det3(double[,] m) =>
        m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1])
        - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0])
        + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);

    public static double[,]? Inverse3(double[,] m)
    {
        double det = Det3(m);
        if (det == 0 || double.IsNaN(det)) return null;
        var inv = new double[3, 3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
            {
                int r1 = (c + 1) % 3, r2 = (c + 2) % 3, c1 = (r + 1) % 3, c2 = (r + 2) % 3;
                inv[r, c] = (m[r1, c1] * m[r2, c2] - m[r1, c2] * m[r2, c1]) / det;
            }
        return inv;
    }
}

/// <summary>Collects the pair phases over a capture (session reference, calibration trial) and averages them.</summary>
public sealed class PhaseCapture
{
    private readonly List<double>[] _phases = { new(), new() };

    public int[] Counts => new[] { _phases[0].Count, _phases[1].Count };

    public void Add(double?[] phases)
    {
        for (int p = 0; p < 2; p++)
            if (phases[p] is double v)
                _phases[p].Add(v);
    }

    /// <summary>Circular mean per pair (deg), null for a pair without phases.</summary>
    public double?[] Mean() => new[]
    {
        StiffnessProjectionModel.CircularMeanDeg(_phases[0]), StiffnessProjectionModel.CircularMeanDeg(_phases[1]),
    };

    public void Clear()
    {
        _phases[0].Clear();
        _phases[1].Clear();
    }
}

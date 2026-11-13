using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;

namespace MOSAIC.Models.Elastography;

/// <summary>
/// Elastography features -> Cartesian hand stiffness (N/m, robot base frame) with the muscle-space model of
/// projection/stiffness_model.py, for the fixed session posture. Reads the model file written by
/// projection/mosaic_export.py (format "stiffness_projection/1"); projection/mosaic_export.py is the reference this
/// class is checked against (mosaic/blocks/test).
/// </summary>
/// <remarks>
/// Per feature vector (esp32_robot_stiffness output, 12 doubles):
///   phase of each pair = atan2(sin, cos); index "phase" = -wrap(phase - relaxed), "speed2" = (c / c_relaxed)^2 - 1
///   with c = 360 f d / lag, lag = (offset - phase) mod 360; sigma = clip(max(index, 0) / scale, 0, sigma_max);
///   u_g = a_g + b_g sum_p w_gp sigma_p; f = F_max u; k = gamma f / l0;
///   K_j = J_m^T diag(k) J_m + sum_i f_i d2l_i/dq2; K_c = (J K_j^-1 J^T)^-1.
/// Output: kxx, kyy, kzz, kxy, kxz, kyz, k_volume (cube root of det), sigma_biceps, sigma_triceps, valid.
/// A vector without a usable phase (cos = sin = 0) or with K_j not positive definite repeats the last valid
/// output with valid = 0 (all zeros before the first valid one).
/// </remarks>
public sealed class StiffnessProjectionModel
{
    public const string Format = "stiffness_projection/1";
    public const int OutputLength = 10;
    private static readonly string[] Pairs = { "biceps", "triceps" };

    private readonly double _gamma;
    private readonly double[] _fMax, _l0;
    private readonly double[,] _j;      // 3 x 4
    private readonly double[,] _jm;     // M x 4
    private readonly double[,,] _h;     // M x 4 x 4
    private readonly (int[] Members, double[] Weights, double A, double B)[] _groups;  // weights over Pairs
    private readonly string _kind;
    private readonly double _driveHz, _spacingM, _lagOffsetDeg, _sigmaMax;
    private readonly int _single;  // index into Pairs, or -1
    private readonly double[] _scale = new double[2], _relaxed = new double[2];
    private readonly int[] _cosIndex = new int[2], _sinIndex = new int[2];
    private double[]? _last;

    public string Kind => _kind;
    public int MuscleCount => _fMax.Length;

    public StiffnessProjectionModel(string path) : this(JsonDocument.Parse(File.ReadAllText(path)).RootElement) { }

    public StiffnessProjectionModel(JsonElement root)
    {
        if (root.GetProperty("format").GetString() != Format)
            throw new InvalidDataException($"model file format is not {Format}");
        _gamma = root.GetProperty("gamma").GetDouble();
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

        var groups = new List<(int[], double[], double, double)>();
        foreach (var g in root.GetProperty("groups").EnumerateArray())
        {
            var members = new List<int>();
            foreach (var m in g.GetProperty("members").EnumerateArray()) members.Add(m.GetInt32());
            var weights = new double[2];
            foreach (var d in g.GetProperty("drivers").EnumerateObject())
                weights[PairIndex(d.Name)] = d.Value.GetDouble();
            groups.Add((members.ToArray(), weights, g.GetProperty("a").GetDouble(), g.GetProperty("b").GetDouble()));
        }
        _groups = groups.ToArray();

        var idx = root.GetProperty("index");
        _kind = idx.GetProperty("kind").GetString() ?? "phase";
        if (_kind != "phase" && _kind != "speed2")
            throw new InvalidDataException($"unknown index kind '{_kind}'");
        _driveHz = idx.GetProperty("drive_hz").GetDouble();
        _spacingM = idx.GetProperty("spacing_m").GetDouble();
        _lagOffsetDeg = idx.GetProperty("lag_offset_deg").GetDouble();
        _sigmaMax = idx.GetProperty("sigma_max").GetDouble();
        var single = idx.GetProperty("single_index");
        _single = single.ValueKind == JsonValueKind.String ? PairIndex(single.GetString()!) : -1;
        for (int p = 0; p < 2; p++)
        {
            _scale[p] = idx.GetProperty("scale").GetProperty(Pairs[p]).GetDouble();
            _relaxed[p] = idx.GetProperty("relaxed_phase_deg").GetProperty(Pairs[p]).GetDouble();
            var f = idx.GetProperty("features").GetProperty(Pairs[p]);
            _cosIndex[p] = f[0].GetInt32();
            _sinIndex[p] = f[1].GetInt32();
        }
    }

    /// <summary>One feature vector in, one output vector (OutputLength values) out.</summary>
    public double[] Step(IReadOnlyList<double> features)
    {
        var sigma = Sigmas(features);
        var kc = sigma is null ? null : Stiffness(sigma);
        if (sigma is null || kc is null)
        {
            var held = _last is null ? new double[OutputLength] : (double[])_last.Clone();
            held[OutputLength - 1] = 0.0;
            return held;
        }
        double det = Det3(kc);
        _last = new[]
        {
            kc[0, 0], kc[1, 1], kc[2, 2], kc[0, 1], kc[0, 2], kc[1, 2],
            det > 0 ? Math.Cbrt(det) : 0.0, sigma[0], sigma[1], 1.0,
        };
        return (double[])_last.Clone();
    }

    /// <summary>sigma per pair (biceps, triceps), or null if a needed pair has no phase.</summary>
    public double[]? Sigmas(IReadOnlyList<double> features)
    {
        var raw = new double?[2];
        for (int p = 0; p < 2; p++)
        {
            if (Math.Max(_cosIndex[p], _sinIndex[p]) >= features.Count)
                continue;  // short vector: no phase
            double c = features[_cosIndex[p]], s = features[_sinIndex[p]];
            if (c * c + s * s > 0.25)
                raw[p] = RawIndex(Math.Atan2(s, c) * 180.0 / Math.PI, _relaxed[p]);
        }
        if (_single >= 0)
            raw[0] = raw[1] = raw[_single];
        if (raw[0] is null || raw[1] is null)
            return null;
        var sigma = new double[2];
        for (int p = 0; p < 2; p++)
            sigma[p] = Math.Min(Math.Max(raw[p]!.Value, 0.0) / _scale[p], _sigmaMax);
        return sigma;
    }

    private double RawIndex(double phaseDeg, double relaxedDeg)
    {
        if (_kind == "phase")
            return -Wrap(phaseDeg - relaxedDeg);
        double lag = Mod360(_lagOffsetDeg - phaseDeg);
        double lagRelaxed = Mod360(_lagOffsetDeg - relaxedDeg);
        double ratio = Math.Max(lagRelaxed, 1.0) / Math.Max(lag, 1.0);  // c / c_relaxed, c = 360 f d / lag
        return ratio * ratio - 1.0;
    }

    /// <summary>K_c (3 x 3) for given sigmas, or null if K_j is not positive definite.</summary>
    public double[,]? Stiffness(double[] sigma)
    {
        int m = _fMax.Length;
        var u = new double[m];
        foreach (var (members, weights, a, b) in _groups)
        {
            double level = a + b * (weights[0] * sigma[0] + weights[1] * sigma[1]);
            foreach (var i in members) u[i] = level;
        }
        var kj = new double[4, 4];
        for (int i = 0; i < m; i++)
        {
            double f = _fMax[i] * u[i];
            double k = _gamma * f / _l0[i];
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    kj[r, c] += k * _jm[i, r] * _jm[i, c] + f * _h[i, r, c];
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
            for (int r = 0; r < 4; r++) rhs[r] = _j[col, r];
            var sol = CholeskySolve(l, rhs);
            for (int r = 0; r < 4; r++) x[r, col] = sol[r];
        }
        var comp = new double[3, 3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                for (int k = 0; k < 4; k++)
                    comp[r, c] += _j[r, k] * x[k, c];
        var kc = Inverse3(comp);
        if (kc is null) return null;
        for (int r = 0; r < 3; r++)
            for (int c = r + 1; c < 3; c++)
                kc[r, c] = kc[c, r] = 0.5 * (kc[r, c] + kc[c, r]);
        return kc;
    }

    // --- helpers ---

    private static int PairIndex(string name) => Array.IndexOf(Pairs, name) is var i and >= 0
        ? i : throw new InvalidDataException($"unknown index '{name}'");

    private static double Mod360(double a) => a - 360.0 * Math.Floor(a / 360.0);

    private static double Wrap(double a) => Mod360(a + 180.0) - 180.0;

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

    private static double Det3(double[,] m) =>
        m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1])
        - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0])
        + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);

    private static double[,]? Inverse3(double[,] m)
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

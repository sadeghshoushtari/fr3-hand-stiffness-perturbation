using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Nodes;

namespace MOSAIC.Models.Elastography;

/// <summary>
/// Two-pair hand stiffness model, a port of projection/hand_model.py (model file format stiffness_projection/3, checked
/// against it in mosaic/blocks/test): two elastography indices -> one drive -> 6x6 hand stiffness at the handle centre
/// -> the slave's translational and rotational command.
/// </summary>
/// <remarks>
///   drive     d = (sigma_biceps + sigma_triceps) / 2
///   muscles   u_g = ratio_g (a0 + b d) for the groups of StiffnessFit.GroupSpec; f = F_max u; k = gamma f / l0
///   arm       K_arm = J_m^T diag(k) J_m + sum_i f_i d2l_i/dq2           (4 x 4: shoulder x, y, z, elbow)
///   wrist     K_w = s_w diag(passive) + c_r d I                         (3 x 3: forearm twist, wrist flexion, deviation)
///   hand      K6 = (J6 diag(K_arm, K_w)^-1 J6^T)^-1                     (6 x 6, handle centre, robot base frame)
///   operator  k_t = reduce(K6[0..3, 0..3]), k_r = reduce(K6[3..6, 3..6]); reduce = mean of the diagonal or volume
/// The calibration compares the model with what its robot measures (<see cref="MeasuredBlocks"/>): the robot moves and
/// turns its flange (handle centre - flange = <see cref="HandleOffset"/>) and holds the other degrees of freedom with
/// its springs (rotation 10 Nm/rad during the whole trial, translation 1000 N/m; perturbation.cpp).
/// Parameters P = (a0, b, s_w, c_r), fitted per session by <see cref="StiffnessFit.FitHand"/>.
/// </remarks>
public sealed class HandModel
{
    public const string Format = "stiffness_projection/3";
    public static readonly string[] ParameterNames = { "a0", "b", "s_w", "c_r" };
    /// <summary>a0, b: the elbow-group prior of the group model; s_w = 1: Formica's passive wrist values; c_r: Nm/rad
    /// per unit drive (strong wrist co-contraction 19.5-32.1 Nm/rad, Milner et al. 1995).</summary>
    public static readonly double[] Prior = { 0.01, 0.25, 1.0, 10.0 };
    public static readonly double[] Lower = { 1e-5, 1e-5, 1e-3, 1e-3 };
    public static readonly double[] Upper = { 1.0, 2.0, 1e3, 1e3 };
    /// <summary>Pull towards the prior, per parameter in log units (hand_model.RIDGE: chosen on the replayed
    /// calibrations of 11-19 and 11-25).</summary>
    public const double DefaultRidge = 0.3;
    public static readonly string[] WristJoints = { "forearm_twist", "wrist_flexion", "wrist_deviation" };
    /// <summary>Passive wrist stiffness, Nm/rad (Formica et al. 2012): pronation/supination, flexion/extension,
    /// radial/ulnar deviation.</summary>
    public static readonly double[] DefaultWristPassive = { 0.21, 0.79, 1.48 };
    /// <summary>Activation per unit of (a0 + b d) of each group relative to the elbow muscles.</summary>
    public static readonly (string Name, double Ratio)[] DefaultGroupRatio =
        { ("biceps", 1.0), ("triceps", 1.0), ("elbow_flexors", 1.0), ("shoulder", 0.6) };
    public const double RobotTranslationHold = 1000.0, RobotRotationHold = 10.0;
    /// <summary>Volume: eigenvalues below this share of the largest are raised to it (measured blocks).</summary>
    public const double VolumeFloor = 1e-3;
    public static readonly string[] Reductions = { "mean", "volume" };

    public sealed record MuscleGroup(string Name, int[] Members, double Ratio);

    private readonly double _gamma;
    private readonly double[] _rho;
    private double[,] _j6 = new double[6, 7];
    private double[,] _kArmUnit = new double[4, 4];

    public ArmModel Arm { get; }
    public double UpperArmDeg { get; }
    public double ElbowDeg { get; }
    /// <summary>a0, b, s_w, c_r.</summary>
    public double[] P { get; private set; }
    public string Reduction { get; }
    /// <summary>Handle centre - flange of the calibration robot, m, base frame.</summary>
    public double[] HandleOffset { get; }
    public double HoldTranslation { get; }
    public double HoldRotation { get; }
    public double[] WristPassive { get; }
    public MuscleGroup[] Groups { get; }
    public double Gamma => _gamma;

    public HandModel(ArmModel arm, double upperArmDeg = 90.0, double elbowDeg = 90.0, double[]? parameters = null,
        string reduction = "mean", double[]? handleOffset = null, double holdTranslation = RobotTranslationHold,
        double holdRotation = RobotRotationHold, double[]? wristPassive = null, IEnumerable<MuscleGroup>? groups = null,
        double gamma = 23.4)
    {
        if (!Reductions.Contains(reduction))
            throw new ArgumentException($"reduction must be mean or volume, not '{reduction}'");
        Arm = arm;
        UpperArmDeg = upperArmDeg;
        ElbowDeg = elbowDeg;
        P = (double[])(parameters ?? Prior).Clone();
        Reduction = reduction;
        HandleOffset = (double[])(handleOffset ?? new double[3]).Clone();
        HoldTranslation = holdTranslation;
        HoldRotation = holdRotation;
        WristPassive = (double[])(wristPassive ?? DefaultWristPassive).Clone();
        Groups = groups?.ToArray() ?? DefaultGroups(arm);
        _gamma = gamma;
        _rho = new double[arm.MuscleCount];
        foreach (var g in Groups)
            foreach (var i in g.Members)
                _rho[i] = g.Ratio;
        SetPosture(ArmModel.SessionPosture(upperArmDeg, elbowDeg));
    }

    /// <summary>The groups of the group model (StiffnessFit.GroupSpec) with <see cref="DefaultGroupRatio"/>.</summary>
    public static MuscleGroup[] DefaultGroups(ArmModel arm) => StiffnessFit.GroupSpec.Select(g => new MuscleGroup(g.Name,
        g.Members.Select(m => Array.IndexOf(arm.Names, m) is var i and >= 0
            ? i : throw new ArgumentException($"muscle {m} not in the arm model")).ToArray(),
        DefaultGroupRatio.First(r => r.Name == g.Name).Ratio)).ToArray();

    public void SetParameters(IReadOnlyList<double> p) => P = p.ToArray();

    /// <summary>Arm geometry at a posture: J6 and the arm joint stiffness per unit of (a0 + b d).</summary>
    public void SetPosture(Posture posture)
    {
        var j6 = Arm.HandJacobian(posture);
        var jm = Arm.MuscleJacobian(posture);
        var h = Arm.MuscleHessians(posture);
        var k = new double[4, 4];
        for (int i = 0; i < Arm.MuscleCount; i++)
        {
            double f = _rho[i] * Arm.FMax[i];
            double ks = _gamma * f / Arm.L0[i];
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    k[r, c] += ks * jm[i, r] * jm[i, c] + f * h[i, r, c];
        }
        for (int r = 0; r < 4; r++)
            for (int c = r + 1; c < 4; c++)
                k[r, c] = k[c, r] = 0.5 * (k[r, c] + k[c, r]);
        _j6 = j6;
        _kArmUnit = k;
    }

    // --- forward model ---

    public double[,] JointStiffness(double d, IReadOnlyList<double>? p = null)
    {
        p ??= P;
        var kj = new double[7, 7];
        double level = p[0] + p[1] * d;
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                kj[r, c] = level * _kArmUnit[r, c];
        for (int w = 0; w < 3; w++)
            kj[4 + w, 4 + w] = p[2] * WristPassive[w] + p[3] * d;
        return kj;
    }

    /// <summary>K6 (6 x 6) at the handle centre, or null if K_j is not positive definite.</summary>
    public double[,]? HandStiffness(double d, IReadOnlyList<double>? p = null)
    {
        var l = Cholesky(JointStiffness(d, p));
        if (l is null)
            return null;
        // compliance C = J6 K_j^-1 J6^T
        var x = new double[7, 6];
        for (int col = 0; col < 6; col++)
        {
            var rhs = new double[7];
            for (int r = 0; r < 7; r++) rhs[r] = _j6[col, r];
            var sol = CholeskySolve(l, rhs);
            for (int r = 0; r < 7; r++) x[r, col] = sol[r];
        }
        var comp = new double[6, 6];
        for (int r = 0; r < 6; r++)
            for (int c = 0; c < 6; c++)
                for (int k = 0; k < 7; k++)
                    comp[r, c] += _j6[r, k] * x[k, c];
        var k6 = SpdInverse(comp);
        if (k6 is null)
            return null;
        Symmetrize(k6);
        return k6;
    }

    /// <summary>(k_t N/m, k_r Nm/rad): reduced translational and rotational blocks of K6, the operator's stiffness the
    /// commands follow; null if K_j is not positive definite.</summary>
    public (double T, double R)? Scalars(double d, IReadOnlyList<double>? p = null)
    {
        var k6 = HandStiffness(d, p);
        return k6 is null ? null : (Reduce(Block(k6, 0), Reduction), Reduce(Block(k6, 3), Reduction));
    }

    /// <summary>(K_t, K_r) as the calibration robot would measure them.</summary>
    public (double[,] T, double[,] R)? Measured(double d, IReadOnlyList<double>? p = null)
    {
        var k6 = HandStiffness(d, p);
        return k6 is null ? null : MeasuredBlocks(k6, HandleOffset, HoldTranslation, HoldRotation);
    }

    public (double T, double R)? MeasuredScalars(double d, IReadOnlyList<double>? p = null)
    {
        var m = Measured(d, p);
        return m is not { } b ? null : (Reduce(b.T, Reduction), Reduce(b.R, Reduction));
    }

    /// <summary>One number for a 3x3 stiffness block (hand_model.reduce): "mean" = mean of the diagonal, "volume" =
    /// cube root of the determinant with eigenvalues below <see cref="VolumeFloor"/> of the largest raised to it; NaN
    /// for a trace &lt;= 0 (mean) or no positive eigenvalue (volume).</summary>
    public static double Reduce(double[,] k, string mode)
    {
        if (mode == "mean")
        {
            double v = (k[0, 0] + k[1, 1] + k[2, 2]) / 3.0;
            return v > 0 ? v : double.NaN;
        }
        var s = new double[3, 3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                s[r, c] = 0.5 * (k[r, c] + k[c, r]);
        var w = SymmetricEigenvalues(s);
        double max = w.Max();
        if (max <= 0)
            return double.NaN;
        return Math.Cbrt(w.Aggregate(1.0, (acc, v) => acc * Math.Max(v, VolumeFloor * max)));
    }

    /// <summary>What the calibration robot measures for a hand stiffness K6 (hand_model.measured_blocks):
    /// K_f = T^T K6 T with T = [[I, -skew(offset)], [0, I]] (flange as reference point); translational stiffness with
    /// the rotation held at holdR, K_t = K_tt - K_tr (K_rr + holdR I)^-1 K_rt; rotational stiffness with the position held
    /// at holdT, K_r = K_rr - K_rt (K_tt + holdT I)^-1 K_tr.</summary>
    public static (double[,] T, double[,] R) MeasuredBlocks(double[,] k6, IReadOnlyList<double> offset, double holdT,
        double holdR)
    {
        var t = new double[6, 6];
        for (int i = 0; i < 6; i++) t[i, i] = 1.0;
        double ox = offset[0], oy = offset[1], oz = offset[2];
        // -skew(offset) in the upper right block
        t[0, 4] = oz; t[0, 5] = -oy;
        t[1, 3] = -oz; t[1, 5] = ox;
        t[2, 3] = oy; t[2, 4] = -ox;
        var kt6 = new double[6, 6];
        for (int r = 0; r < 6; r++)
            for (int c = 0; c < 6; c++)
                for (int k = 0; k < 6; k++)
                    kt6[r, c] += k6[r, k] * t[k, c];
        var kf = new double[6, 6];
        for (int r = 0; r < 6; r++)
            for (int c = 0; c < 6; c++)
                for (int k = 0; k < 6; k++)
                    kf[r, c] += t[k, r] * kt6[k, c];
        var ktt = Block(kf, 0, 0);
        var ktr = Block(kf, 0, 3);
        var krt = Block(kf, 3, 0);
        var krr = Block(kf, 3, 3);
        var kTrans = Schur(ktt, ktr, krr, krt, holdR);
        var kRot = Schur(krr, krt, ktt, ktr, holdT);
        return (kTrans, kRot);
    }

    /// <summary>a - b (c + hold I)^-1 e, symmetrised.</summary>
    private static double[,] Schur(double[,] a, double[,] b, double[,] c, double[,] e, double hold)
    {
        var ch = (double[,])c.Clone();
        for (int i = 0; i < 3; i++) ch[i, i] += hold;
        var inv = StiffnessProjectionModel.Inverse3(ch) ?? throw new InvalidOperationException("singular hold block");
        var res = (double[,])a.Clone();
        for (int r = 0; r < 3; r++)
            for (int col = 0; col < 3; col++)
            {
                double s = 0;
                for (int k = 0; k < 3; k++)
                    for (int m = 0; m < 3; m++)
                        s += b[r, k] * inv[k, m] * e[m, col];
                res[r, col] -= s;
            }
        Symmetrize(res);
        return res;
    }

    public static double[,] Block(double[,] k6, int start) => Block(k6, start, start);

    public static double[,] Block(double[,] m, int row, int col)
    {
        var b = new double[3, 3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                b[r, c] = m[row + r, col + c];
        return b;
    }

    // --- model file sections ---

    /// <summary>The model sections of a format-3 file (hand_model.HandModel.to_dict without index, commands, fit).</summary>
    public JsonObject ToJson() => new()
    {
        ["arm"] = new JsonObject { ["upper_arm_m"] = Arm.UpperArmM, ["forearm_m"] = Arm.ForearmM, ["hand_m"] = Arm.HandM },
        ["posture"] = new JsonObject { ["upper_arm_deg"] = UpperArmDeg, ["elbow_deg"] = ElbowDeg },
        ["muscle_groups"] = new JsonArray(Groups.Select(g => (JsonNode)new JsonObject
        {
            ["name"] = g.Name, ["members"] = new JsonArray(g.Members.Select(i => (JsonNode)i).ToArray()),
            ["ratio"] = g.Ratio,
        }).ToArray()),
        ["drive"] = new JsonObject
        {
            ["a0"] = P[0], ["b"] = P[1], ["input"] = "d = mean of sigma_biceps and sigma_triceps; u = ratio * (a0 + b d)",
        },
        ["wrist"] = new JsonObject
        {
            ["joints"] = new JsonArray(WristJoints.Select(j => (JsonNode)j).ToArray()),
            ["passive"] = new JsonArray(WristPassive.Select(v => (JsonNode)v).ToArray()), ["s_w"] = P[2], ["c_r"] = P[3],
            ["input"] = "K_w = s_w diag(passive) + c_r d I, Nm/rad",
        },
        ["reduction"] = Reduction,
        ["measurement"] = new JsonObject
        {
            ["handle_offset_m"] = new JsonArray(HandleOffset.Select(v => (JsonNode)v).ToArray()),
            ["robot_translation_hold"] = HoldTranslation, ["robot_rotation_hold"] = HoldRotation,
        },
    };

    public static HandModel FromJson(JsonElement root, double gamma)
    {
        var a = root.GetProperty("arm");
        var arm = new ArmModel(a.GetProperty("upper_arm_m").GetDouble(), a.GetProperty("forearm_m").GetDouble(),
            a.GetProperty("hand_m").GetDouble());
        if (root.TryGetProperty("muscles", out var mus)
            && !mus.EnumerateArray().Select(m => m.GetString() ?? "").SequenceEqual(arm.Names))
            throw new InvalidDataException("muscle list of the model file does not match ArmModel.Muscles");
        var posture = root.GetProperty("posture");
        var drive = root.GetProperty("drive");
        var wrist = root.GetProperty("wrist");
        double[]? offset = null;
        double holdT = RobotTranslationHold, holdR = RobotRotationHold;
        if (root.TryGetProperty("measurement", out var meas) && meas.ValueKind == JsonValueKind.Object)
        {
            if (meas.TryGetProperty("handle_offset_m", out var o))
                offset = o.EnumerateArray().Select(v => v.GetDouble()).ToArray();
            if (meas.TryGetProperty("robot_translation_hold", out var ht)) holdT = ht.GetDouble();
            if (meas.TryGetProperty("robot_rotation_hold", out var hr)) holdR = hr.GetDouble();
        }
        var groups = root.GetProperty("muscle_groups").EnumerateArray().Select(g => new MuscleGroup(
            g.GetProperty("name").GetString() ?? "", g.GetProperty("members").EnumerateArray().Select(m => m.GetInt32()).ToArray(),
            g.GetProperty("ratio").GetDouble())).ToArray();
        return new HandModel(arm, posture.GetProperty("upper_arm_deg").GetDouble(), posture.GetProperty("elbow_deg").GetDouble(),
            new[] { drive.GetProperty("a0").GetDouble(), drive.GetProperty("b").GetDouble(), wrist.GetProperty("s_w").GetDouble(),
                wrist.GetProperty("c_r").GetDouble() },
            root.TryGetProperty("reduction", out var red) ? red.GetString() ?? "mean" : "mean", offset, holdT, holdR,
            wrist.TryGetProperty("passive", out var pas) ? pas.EnumerateArray().Select(v => v.GetDouble()).ToArray() : null,
            groups, gamma);
    }

    // --- small dense algebra ---

    private static void Symmetrize(double[,] m)
    {
        int n = m.GetLength(0);
        for (int r = 0; r < n; r++)
            for (int c = r + 1; c < n; c++)
                m[r, c] = m[c, r] = 0.5 * (m[r, c] + m[c, r]);
    }

    private static double[,]? Cholesky(double[,] a)
    {
        int n = a.GetLength(0);
        var l = new double[n, n];
        for (int i = 0; i < n; i++)
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

    private static double[,]? SpdInverse(double[,] a)
    {
        int n = a.GetLength(0);
        var l = Cholesky(a);
        if (l is null)
            return null;
        var inv = new double[n, n];
        for (int c = 0; c < n; c++)
        {
            var e = new double[n];
            e[c] = 1.0;
            var col = CholeskySolve(l, e);
            for (int r = 0; r < n; r++) inv[r, c] = col[r];
        }
        return inv;
    }

    /// <summary>Eigenvalues of a symmetric 3x3 matrix (cyclic Jacobi rotations).</summary>
    public static double[] SymmetricEigenvalues(double[,] m)
    {
        var a = (double[,])m.Clone();
        for (int sweep = 0; sweep < 50; sweep++)
        {
            double off = a[0, 1] * a[0, 1] + a[0, 2] * a[0, 2] + a[1, 2] * a[1, 2];
            double diag = a[0, 0] * a[0, 0] + a[1, 1] * a[1, 1] + a[2, 2] * a[2, 2];
            if (off <= 1e-30 * Math.Max(diag, 1e-300))
                break;
            for (int p = 0; p < 2; p++)
                for (int q = p + 1; q < 3; q++)
                {
                    if (a[p, q] == 0) continue;
                    double theta = 0.5 * (a[q, q] - a[p, p]) / a[p, q];
                    double t = Math.Sign(theta == 0 ? 1.0 : theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1.0));
                    double c = 1.0 / Math.Sqrt(t * t + 1.0), s = t * c;
                    for (int k = 0; k < 3; k++)  // A <- A J
                    {
                        double akp = a[k, p], akq = a[k, q];
                        a[k, p] = c * akp - s * akq;
                        a[k, q] = s * akp + c * akq;
                    }
                    for (int k = 0; k < 3; k++)  // A <- J^T A
                    {
                        double apk = a[p, k], aqk = a[q, k];
                        a[p, k] = c * apk - s * aqk;
                        a[q, k] = s * apk + c * aqk;
                    }
                }
        }
        return new[] { a[0, 0], a[1, 1], a[2, 2] };
    }
}

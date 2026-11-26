using System;
using System.Collections.Generic;

namespace MOSAIC.Models.Elastography;

/// <summary>Measured arm posture in the robot base frame (projection/arm_model.py Posture).</summary>
/// <param name="UpperArm">Rotation matrix of the upper-arm frame (x forward, y along the humerus towards the
/// shoulder, z lateral = elbow flexion axis).</param>
/// <param name="ElbowAngle">Elbow flexion, rad, 0 = straight.</param>
/// <param name="Shoulder">Glenohumeral centre, m.</param>
/// <param name="Torso">Rotation matrix of the torso frame.</param>
public sealed record Posture(double[,] UpperArm, double ElbowAngle, double[] Shoulder, double[,] Torso);

/// <summary>Arm geometry at one posture: J (3 x 4), J_m (M x 4) and the muscle Hessians H (M x 4 x 4).</summary>
public sealed record ArmGeometry(double[,] J, double[,] Jm, double[,,] H);

/// <summary>
/// Line-of-action model of the right arm, a port of projection/arm_model.py (checked against it in
/// mosaic/blocks/test): posture -> arm Jacobian J = d(handle)/dq and muscle Jacobian J_m = d(muscle length)/dq,
/// q = (small rotation of the upper arm about the world x, y, z axes, change of elbow flexion), derivatives at q = 0.
/// Muscle points, l0 and F_max are the MoBL-ARMS fit of arm_model.MUSCLES, for a 0.30 m upper arm and 0.27 m forearm,
/// scaled to the subject; the triceps wrap over a sphere at the elbow. <see cref="HandJacobian"/> adds the forearm twist
/// and the two wrist joints and the hand's rotation (6 x 7, for <see cref="HandModel"/>).
/// </summary>
public sealed class ArmModel
{
    public const double RefUpperArmM = 0.30, RefForearmM = 0.27;

    // operator facing the robot: forward = robot -x, up = robot +z, lateral = robot +y (columns)
    public static readonly double[,] SessionTorso = { { -1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 } };

    public enum Frame { Torso, Humerus, Forearm, Wrap }

    public readonly record struct PathPoint(Frame Frame, double X, double Y, double Z);

    private static PathPoint P(Frame frame, double x, double y, double z) => new(frame, x, y, z);

    private static PathPoint Wrap(double radius) => new(Frame.Wrap, radius, 0, 0);

    public static readonly (string Name, PathPoint[] Path, double L0, double FMax)[] Muscles =
    {
        ("deltoid_anterior", new[] { P(Frame.Humerus, 0.008, -0.122, 0.011), P(Frame.Humerus, 0.016, -0.086, 0.005), P(Frame.Torso, 0.045, 0.023, -0.018), P(Frame.Torso, 0.01, 0.023, -0.069) }, 0.098, 1218.9),
        ("deltoid_middle", new[] { P(Frame.Humerus, 0.003, -0.14, 0.012), P(Frame.Humerus, 0.005, -0.113, 0.025), P(Frame.Torso, 0.001, 0.025, 0.021), P(Frame.Torso, -0.008, 0.033, -0.006) }, 0.108, 1103.5),
        ("deltoid_posterior", new[] { P(Frame.Torso, -0.055, 0.023, -0.035), P(Frame.Torso, -0.048, -0.008, 0.027), P(Frame.Humerus, 0.002, -0.055, 0.01) }, 0.137, 201.6),
        ("supraspinatus", new[] { P(Frame.Humerus, 0.005, 0.012, 0.026), P(Frame.Humerus, 0.009, 0.013, 0.02), P(Frame.Torso, -0.039, 0.045, -0.046), P(Frame.Torso, -0.042, 0.005, -0.069) }, 0.068, 499.2),
        ("infraspinatus", new[] { P(Frame.Humerus, -0.004, 0.007, 0.022), P(Frame.Torso, -0.057, -0.023, -0.015), P(Frame.Torso, -0.064, -0.04, -0.052) }, 0.075, 1075.8),
        ("subscapularis", new[] { P(Frame.Humerus, 0.013, 0.008, -0.015), P(Frame.Humerus, 0.016, 0.008, -0.016), P(Frame.Torso, -0.001, -0.075, -0.084), P(Frame.Torso, -0.066, -0.026, -0.071) }, 0.087, 1306.9),
        ("teres_minor", new[] { P(Frame.Humerus, 0.0, -0.012, 0.023), P(Frame.Humerus, 0.004, -0.006, 0.023), P(Frame.Torso, -0.071, -0.092, 0.022), P(Frame.Torso, -0.081, -0.071, -0.053) }, 0.074, 269.5),
        ("teres_major", new[] { P(Frame.Humerus, 0.009, -0.056, -0.004), P(Frame.Humerus, 0.004, -0.038, -0.009), P(Frame.Torso, -0.115, -0.147, -0.048), P(Frame.Torso, -0.085, -0.103, -0.068) }, 0.162, 144.0),
        ("pectoralis_clavicular", new[] { P(Frame.Humerus, 0.012, -0.043, 0.009), P(Frame.Humerus, 0.017, -0.034, -0.001), P(Frame.Torso, 0.035, -0.005, -0.034), P(Frame.Torso, 0.031, 0.01, -0.096) }, 0.144, 444.3),
        ("pectoralis_sternal", new[] { P(Frame.Humerus, 0.013, -0.044, 0.009), P(Frame.Humerus, 0.014, -0.047, -0.006), P(Frame.Torso, 0.05, -0.049, -0.042), P(Frame.Torso, 0.057, -0.047, -0.152) }, 0.139, 658.3),
        ("pectoralis_ribs", new[] { P(Frame.Humerus, 0.013, -0.045, 0.009), P(Frame.Humerus, 0.012, -0.055, -0.009), P(Frame.Torso, 0.05, -0.073, -0.04), P(Frame.Torso, 0.087, -0.12, -0.136) }, 0.139, 498.1),
        ("latissimus_thoracic", new[] { P(Frame.Humerus, 0.01, -0.036, -0.006), P(Frame.Humerus, 0.005, -0.035, -0.011), P(Frame.Torso, -0.071, -0.094, -0.019), P(Frame.Torso, -0.071, -0.121, -0.166) }, 0.254, 290.5),
        ("latissimus_lumbar", new[] { P(Frame.Humerus, 0.009, -0.042, -0.005), P(Frame.Humerus, 0.003, -0.048, -0.011), P(Frame.Torso, -0.067, -0.132, -0.01), P(Frame.Torso, -0.046, -0.194, -0.167) }, 0.232, 317.5),
        ("latissimus_iliac", new[] { P(Frame.Humerus, 0.011, -0.041, -0.004), P(Frame.Humerus, 0.004, -0.055, -0.011), P(Frame.Torso, -0.043, -0.16, -0.01), P(Frame.Torso, -0.045, -0.257, -0.166) }, 0.279, 189.0),
        ("coracobrachialis", new[] { P(Frame.Torso, 0.022, -0.007, -0.037), P(Frame.Torso, 0.022, -0.03, -0.024), P(Frame.Humerus, 0.008, -0.194, -0.013), P(Frame.Humerus, 0.004, -0.156, -0.002) }, 0.093, 208.2),
        ("triceps_long", new[] { P(Frame.Torso, -0.024, -0.013, -0.007), P(Frame.Humerus, -0.056, -0.169, 0.014), Wrap(0.02), P(Frame.Forearm, -0.02, 0.012, -0.002) }, 0.134, 771.8),
        ("triceps_lateral", new[] { P(Frame.Humerus, -0.0, -0.182, 0.022), Wrap(0.02), P(Frame.Forearm, -0.015, 0.02, -0.005) }, 0.114, 717.5),
        ("triceps_medial", new[] { P(Frame.Humerus, -0.0, -0.193, -0.02), Wrap(0.02), P(Frame.Forearm, -0.015, 0.02, 0.005) }, 0.114, 717.5),
        ("anconeus", new[] { P(Frame.Humerus, -0.02, -0.295, 0.0), Wrap(0.02), P(Frame.Forearm, -0.012, -0.001, -0.0) }, 0.027, 283.2),
        ("biceps_long", new[] { P(Frame.Torso, -0.025, 0.004, -0.022), P(Frame.Torso, -0.028, 0.023, -0.025), P(Frame.Humerus, 0.018, 0.014, 0.002), P(Frame.Humerus, 0.027, -0.135, 0.008), P(Frame.Forearm, -0.006, -0.046, -0.01), P(Frame.Forearm, -0.005, -0.05, 0.009) }, 0.116, 525.1),
        ("biceps_short", new[] { P(Frame.Torso, 0.022, -0.005, -0.037), P(Frame.Torso, 0.018, -0.028, -0.024), P(Frame.Humerus, 0.014, -0.094, -0.012), P(Frame.Humerus, 0.034, -0.13, 0.03), P(Frame.Forearm, -0.009, -0.049, -0.031), P(Frame.Forearm, -0.005, -0.05, 0.009) }, 0.132, 316.8),
        ("brachialis", new[] { P(Frame.Humerus, 0.0, -0.231, 0.0), P(Frame.Forearm, 0.0, -0.024, 0.0) }, 0.086, 1177.4),
        ("brachioradialis", new[] { P(Frame.Humerus, -0.015, -0.207, 0.012), P(Frame.Forearm, 0.023, -0.144, 0.017), P(Frame.Forearm, 0.023, -0.236, 0.001) }, 0.173, 276.0),
        ("extensor_carpi_radialis_longus", new[] { P(Frame.Humerus, -0.006, -0.268, 0.022), P(Frame.Forearm, 0.0, -0.103, 0.021), P(Frame.Forearm, 0.015, -0.293, 0.002) }, 0.081, 337.3),
        ("pronator_teres", new[] { P(Frame.Humerus, 0.001, -0.284, -0.024), P(Frame.Forearm, 0.009, -0.017, -0.021), P(Frame.Forearm, 0.016, -0.125, 0.018) }, 0.049, 557.2),
    };

    private readonly PathPoint[][] _paths;

    public double UpperArmM { get; }
    public double ForearmM { get; }
    public double HandM { get; }
    public string[] Names { get; }
    public double[] L0 { get; }
    public double[] FMax { get; }

    /// <param name="upperArmM">shoulder -> elbow centre</param>
    /// <param name="forearmM">elbow -> wrist</param>
    /// <param name="handM">wrist -> handle centre</param>
    public ArmModel(double upperArmM = RefUpperArmM, double forearmM = RefForearmM, double handM = 0.07)
    {
        UpperArmM = upperArmM;
        ForearmM = forearmM;
        HandM = handM;
        double sUpper = upperArmM / RefUpperArmM, sFore = forearmM / RefForearmM;
        _paths = new PathPoint[Muscles.Length][];
        Names = new string[Muscles.Length];
        L0 = new double[Muscles.Length];
        FMax = new double[Muscles.Length];
        for (int i = 0; i < Muscles.Length; i++)
        {
            var (name, path, l0, fMax) = Muscles[i];
            Names[i] = name;
            L0[i] = l0;
            FMax[i] = fMax;
            _paths[i] = Array.ConvertAll(path, p => p.Frame switch
            {
                Frame.Wrap => p with { X = p.X * sFore },
                Frame.Forearm => new PathPoint(p.Frame, p.X * sFore, p.Y * sFore, p.Z * sFore),
                _ => new PathPoint(p.Frame, p.X * sUpper, p.Y * sUpper, p.Z * sUpper),
            });
        }
    }

    public int MuscleCount => _paths.Length;

    // --- postures ---

    /// <summary>Posture from the humerus direction (shoulder -> elbow) and the forearm direction (elbow -> hand),
    /// world frame (arm_model.posture_from_directions). Needs a bent elbow.</summary>
    public static Posture FromDirections(double[] humerusDown, double[] forearm, double[,] torso, double[]? shoulder = null)
    {
        var d = Normalize(humerusDown);
        var f = Normalize(forearm);
        var axis = Cross(d, f);
        if (Norm(axis) < 0.05)
            throw new ArgumentException("elbow nearly straight: flexion axis undefined");
        var z = Normalize(axis);
        var y = new[] { -d[0], -d[1], -d[2] };
        var x = Cross(y, z);
        double angle = Math.Atan2(Norm(Cross(d, f)), Dot(d, f));
        var r = new double[3, 3];
        for (int k = 0; k < 3; k++)
        {
            r[k, 0] = x[k];
            r[k, 1] = y[k];
            r[k, 2] = z[k];
        }
        return new Posture(r, angle, shoulder ?? new double[3], torso);
    }

    /// <summary>Session posture without IMUs (fit_model_from_perturbation.session_posture): upper arm tilted
    /// forward by upperArmDeg from vertical, elbow flexed by elbowDeg, operator facing the robot.</summary>
    public static Posture SessionPosture(double upperArmDeg, double elbowDeg)
    {
        double t = upperArmDeg * Math.PI / 180.0, e = elbowDeg * Math.PI / 180.0;
        return FromDirections(new[] { -Math.Sin(t), 0, -Math.Cos(t) }, new[] { -Math.Sin(t + e), 0, -Math.Cos(t + e) },
            SessionTorso);
    }

    /// <summary>Posture from the orientations of the upper-arm and forearm frames (unit quaternions w, x, y, z, robot
    /// base frame, i.e. IMU orientations already aligned to the segment frames). Elbow angle = rotation of the forearm
    /// frame relative to the upper-arm frame about its z axis.</summary>
    public static Posture FromQuaternions(IReadOnlyList<double> upperArm, IReadOnlyList<double> forearm, double[,] torso)
    {
        var rUa = QuaternionToMatrix(upperArm);
        var rFa = QuaternionToMatrix(forearm);
        // relative rotation R = rUa^T rFa; about z: angle = atan2(R[1,0], R[0,0])
        double r00 = 0, r10 = 0;
        for (int k = 0; k < 3; k++)
        {
            r00 += rUa[k, 0] * rFa[k, 0];
            r10 += rUa[k, 1] * rFa[k, 0];
        }
        return new Posture(rUa, Math.Atan2(r10, r00), new double[3], torso);
    }

    // --- forward kinematics at local joint coordinates q around the posture ---

    private (double[,] Ua, double[,] Fa, double[] Elbow) Frames(Posture posture, double[] q)
    {
        var rUa = MatMul(RotvecToMatrix(q[0], q[1], q[2]), posture.UpperArm);
        var rFa = MatMul(rUa, RotZ(posture.ElbowAngle + q[3]));
        var elbow = Add(posture.Shoulder, MatVec(rUa, 0.0, -UpperArmM, 0.0));
        return (rUa, rFa, elbow);
    }

    public double[] Handle(Posture posture, double[] q)
    {
        var (_, rFa, elbow) = Frames(posture, q);
        return Add(elbow, MatVec(rFa, 0.0, -(ForearmM + HandM), 0.0));
    }

    /// <summary>Joints of the hand model (arm_model.HAND_JOINTS): the 4 arm joints, forearm twist about the forearm
    /// axis (forearm y), wrist flexion about the twisted forearm x axis and deviation about its z axis (neutral forearm,
    /// palm facing medially). No muscle line crosses the last three.</summary>
    public static readonly string[] HandJoints =
        { "shoulder_x", "shoulder_y", "shoulder_z", "elbow", "forearm_twist", "wrist_flexion", "wrist_deviation" };

    /// <summary>Handle centre and hand rotation at the 7 joint coordinates of <see cref="HandJoints"/>
    /// (arm_model.hand_frame).</summary>
    public (double[] Position, double[,] Rotation) HandFrame(Posture posture, double[] q)
    {
        var (_, rFa, elbow) = Frames(posture, q);
        var rHand = MatMul(MatMul(MatMul(rFa, RotY(q[4])), RotX(q[5])), RotZ(q[6]));
        var wrist = Add(elbow, MatVec(rFa, 0.0, -ForearmM, 0.0));
        return (Add(wrist, MatVec(rHand, 0.0, -HandM, 0.0)), rHand);
    }

    /// <summary>J6 (6 x 7) at q = 0: handle velocity (rows 0-2) and hand angular velocity (rows 3-5, world frame) per
    /// joint velocity of <see cref="HandJoints"/> (arm_model.hand_jacobian, central differences).</summary>
    public double[,] HandJacobian(Posture posture, double h = 1e-6)
    {
        var jac = new double[6, 7];
        for (int k = 0; k < 7; k++)
        {
            var qp = new double[7];
            var qm = new double[7];
            qp[k] = h;
            qm[k] = -h;
            var (p1, r1) = HandFrame(posture, qp);
            var (p0, r0) = HandFrame(posture, qm);
            var w = new double[3, 3];  // r1 r0^T: rotation from -h to +h, small, its skew part is the rotation vector
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    w[i, j] = r1[i, 0] * r0[j, 0] + r1[i, 1] * r0[j, 1] + r1[i, 2] * r0[j, 2];
            var omega = new[] { 0.5 * (w[2, 1] - w[1, 2]), 0.5 * (w[0, 2] - w[2, 0]), 0.5 * (w[1, 0] - w[0, 1]) };
            for (int r = 0; r < 3; r++)
            {
                jac[r, k] = (p1[r] - p0[r]) / (2 * h);
                jac[3 + r, k] = omega[r] / (2 * h);
            }
        }
        return jac;
    }

    public double[] MuscleLengths(Posture posture, double[] q)
    {
        var (rUa, rFa, elbow) = Frames(posture, q);
        var lengths = new double[_paths.Length];
        var pts = new List<double[]>(8);
        var wrapBefore = new List<double>(8);  // wrap radius of the segment ending at point k (0 = straight)
        for (int i = 0; i < _paths.Length; i++)
        {
            pts.Clear();
            wrapBefore.Clear();
            double pendingWrap = 0;
            foreach (var p in _paths[i])
            {
                if (p.Frame == Frame.Wrap)
                {
                    pendingWrap = p.X;
                    continue;
                }
                var w = p.Frame switch
                {
                    Frame.Torso => Add(posture.Shoulder, MatVec(posture.Torso, p.X, p.Y, p.Z)),
                    Frame.Humerus => Add(posture.Shoulder, MatVec(rUa, p.X, p.Y, p.Z)),
                    _ => Add(elbow, MatVec(rFa, p.X, p.Y, p.Z)),
                };
                pts.Add(w);
                wrapBefore.Add(pendingWrap);
                pendingWrap = 0;
            }
            double sum = 0;
            for (int k = 1; k < pts.Count; k++)
                sum += wrapBefore[k] > 0
                    ? WrappedLength(pts[k - 1], pts[k], elbow, wrapBefore[k])
                    : Norm(Sub(pts[k], pts[k - 1]));
            lengths[i] = sum;
        }
        return lengths;
    }

    /// <summary>Shortest path from a to b outside the sphere (centre, radius) (arm_model.wrapped_length).</summary>
    public static double WrappedLength(double[] a, double[] b, double[] centre, double radius)
    {
        var u = Sub(a, centre);
        var v = Sub(b, centre);
        double du = Norm(u), dv = Norm(v);
        radius = Math.Min(radius, Math.Min(0.99 * du, 0.99 * dv));
        double angle = Math.Acos(Math.Clamp(Dot(u, v) / (du * dv), -1.0, 1.0));
        double arc = angle - Math.Acos(radius / du) - Math.Acos(radius / dv);
        if (arc <= 0)
            return Norm(Sub(b, a));
        return Math.Sqrt(du * du - radius * radius) + Math.Sqrt(dv * dv - radius * radius) + radius * arc;
    }

    // --- derivatives at q = 0 (central differences, as arm_model.py) ---

    private static double[,] Jacobian(Func<double[], double[]> fun, double[] q0, double h)
    {
        double[,]? jac = null;
        for (int k = 0; k < q0.Length; k++)
        {
            var qp = (double[])q0.Clone();
            var qm = (double[])q0.Clone();
            qp[k] += h;
            qm[k] -= h;
            var fp = fun(qp);
            var fm = fun(qm);
            jac ??= new double[fp.Length, q0.Length];
            for (int r = 0; r < fp.Length; r++)
                jac[r, k] = (fp[r] - fm[r]) / (2 * h);
        }
        return jac!;
    }

    public double[,] ArmJacobian(Posture posture, double[]? q = null, double h = 1e-6) =>
        Jacobian(x => Handle(posture, x), q ?? new double[4], h);

    public double[,] MuscleJacobian(Posture posture, double[]? q = null, double h = 1e-6) =>
        Jacobian(x => MuscleLengths(posture, x), q ?? new double[4], h);

    public double[,,] MuscleHessians(Posture posture, double h = 1e-4)
    {
        int m = _paths.Length;
        var hess = new double[m, 4, 4];
        for (int k = 0; k < 4; k++)
        {
            var dq = new double[4];
            dq[k] = h;
            var jp = MuscleJacobian(posture, dq);
            dq[k] = -h;
            var jm = MuscleJacobian(posture, dq);
            for (int i = 0; i < m; i++)
                for (int r = 0; r < 4; r++)
                    hess[i, r, k] = (jp[i, r] - jm[i, r]) / (2 * h);
        }
        for (int i = 0; i < m; i++)
            for (int r = 0; r < 4; r++)
                for (int c = r + 1; c < 4; c++)
                    hess[i, r, c] = hess[i, c, r] = 0.5 * (hess[i, r, c] + hess[i, c, r]);
        return hess;
    }

    public ArmGeometry Geometry(Posture posture) =>
        new(ArmJacobian(posture), MuscleJacobian(posture), MuscleHessians(posture));

    // --- small vector algebra ---

    private static double[,] RotvecToMatrix(double x, double y, double z)
    {
        double angle = Math.Sqrt(x * x + y * y + z * z);
        if (angle < 1e-12)
            return new[,] { { 1, -z, y }, { z, 1, -x }, { -y, x, 1 } };
        double kx = x / angle, ky = y / angle, kz = z / angle, s = Math.Sin(angle), c1 = 1 - Math.Cos(angle);
        var k = new[,] { { 0, -kz, ky }, { kz, 0, -kx }, { -ky, kx, 0 } };
        var kk = MatMul(k, k);
        var r = new double[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                r[i, j] = (i == j ? 1 : 0) + s * k[i, j] + c1 * kk[i, j];
        return r;
    }

    private static double[,] RotX(double angle)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        return new[,] { { 1, 0, 0 }, { 0, c, -s }, { 0, s, c } };
    }

    private static double[,] RotY(double angle)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        return new[,] { { c, 0, s }, { 0, 1, 0 }, { -s, 0, c } };
    }

    private static double[,] RotZ(double angle)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        return new[,] { { c, -s, 0 }, { s, c, 0 }, { 0, 0, 1 } };
    }

    public static double[,] QuaternionToMatrix(IReadOnlyList<double> q)
    {
        double n = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        double w = q[0] / n, x = q[1] / n, y = q[2] / n, z = q[3] / n;
        return new[,]
        {
            { 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y) },
            { 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x) },
            { 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y) },
        };
    }

    private static double[,] MatMul(double[,] a, double[,] b)
    {
        var r = new double[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                r[i, j] = a[i, 0] * b[0, j] + a[i, 1] * b[1, j] + a[i, 2] * b[2, j];
        return r;
    }

    private static double[] MatVec(double[,] a, double x, double y, double z) => new[]
    {
        a[0, 0] * x + a[0, 1] * y + a[0, 2] * z,
        a[1, 0] * x + a[1, 1] * y + a[1, 2] * z,
        a[2, 0] * x + a[2, 1] * y + a[2, 2] * z,
    };

    private static double[] Add(double[] a, double[] b) => new[] { a[0] + b[0], a[1] + b[1], a[2] + b[2] };

    private static double[] Sub(double[] a, double[] b) => new[] { a[0] - b[0], a[1] - b[1], a[2] - b[2] };

    private static double Dot(double[] a, double[] b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    private static double Norm(double[] a) => Math.Sqrt(Dot(a, a));

    private static double[] Normalize(double[] a)
    {
        double n = Norm(a);
        return new[] { a[0] / n, a[1] / n, a[2] / n };
    }

    private static double[] Cross(double[] a, double[] b) => new[]
    {
        a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0],
    };
}

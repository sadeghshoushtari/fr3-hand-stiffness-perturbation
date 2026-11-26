using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using MOSAIC.Models.Elastography;

// dotnet run -- <reference.json from make_arm_reference.py>
// Compares muscle lengths, J, J_m, H and the hand Jacobian J6 of ArmModel with the Python arm model; session cases also go through
// ArmModel.SessionPosture, and every case through FromQuaternions (posture -> quaternions -> posture). Exit 1 on a miss.
const double TolLength = 1e-9, TolJ = 1e-6, TolH = 1e-4;
var root = JsonDocument.Parse(File.ReadAllText(args[0])).RootElement;
var names = root.GetProperty("names").EnumerateArray().Select(n => n.GetString()).ToArray();
bool ok = names.SequenceEqual(new ArmModel().Names);
Console.WriteLine($"muscle names and order: {(ok ? "PASS" : "FAIL")}");
int index = 0;
foreach (var c in root.GetProperty("cases").EnumerateArray())
{
    var arm = new ArmModel(c.GetProperty("upper_arm_m").GetDouble(), c.GetProperty("forearm_m").GetDouble());
    var given = new Posture(Mat(c.GetProperty("upper_arm")), c.GetProperty("elbow_angle").GetDouble(),
        Vec(c.GetProperty("shoulder")), Mat(c.GetProperty("torso")));
    var postures = new (string Label, Posture P)[] { ("given", given), ("quaternions", ViaQuaternions(given)) }.ToList();
    if (c.GetProperty("kind").GetString() == "session")
    {
        var a = Vec(c.GetProperty("angles_deg"));
        postures.Add(("SessionPosture", ArmModel.SessionPosture(a[0], a[1])));
    }
    foreach (var (label, p) in postures)
    {
        var geo = arm.Geometry(p);
        double eL = MaxErr(arm.MuscleLengths(p, new double[4]), Vec(c.GetProperty("lengths")));
        double eJ = MaxErr(geo.J, Mat(c.GetProperty("J")));
        double eJm = MaxErr(geo.Jm, Mat(c.GetProperty("Jm")));
        double eH = MaxErr(geo.H, c.GetProperty("H"));
        double eJ6 = MaxErr(arm.HandJacobian(p), Mat(c.GetProperty("J6")));
        bool pass = eL <= TolLength && eJ <= TolJ && eJm <= TolJ && eH <= TolH && eJ6 <= TolJ;
        ok &= pass;
        Console.WriteLine($"case {index} ({label}): lengths {eL:E1} m, J {eJ:E1}, J_m {eJm:E1}, H {eH:E1}, J6 {eJ6:E1} "
                          + (pass ? "PASS" : "FAIL"));
    }
    index++;
}
return ok ? 0 : 1;

static Posture ViaQuaternions(Posture p)
{
    var rFa = new double[3, 3];
    double c = Math.Cos(p.ElbowAngle), s = Math.Sin(p.ElbowAngle);
    for (int i = 0; i < 3; i++)
    {
        rFa[i, 0] = p.UpperArm[i, 0] * c + p.UpperArm[i, 1] * s;
        rFa[i, 1] = -p.UpperArm[i, 0] * s + p.UpperArm[i, 1] * c;
        rFa[i, 2] = p.UpperArm[i, 2];
    }
    var q = ArmModel.FromQuaternions(Quat(p.UpperArm), Quat(rFa), p.Torso);
    return q with { Shoulder = p.Shoulder };
}

// rotation matrix -> unit quaternion (w, x, y, z), Shepperd's method
static double[] Quat(double[,] r)
{
    double tr = r[0, 0] + r[1, 1] + r[2, 2];
    if (tr > 0)
    {
        double s = 2 * Math.Sqrt(tr + 1);
        return new[] { 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s };
    }
    if (r[0, 0] > r[1, 1] && r[0, 0] > r[2, 2])
    {
        double s = 2 * Math.Sqrt(1 + r[0, 0] - r[1, 1] - r[2, 2]);
        return new[] { (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s };
    }
    if (r[1, 1] > r[2, 2])
    {
        double s = 2 * Math.Sqrt(1 + r[1, 1] - r[0, 0] - r[2, 2]);
        return new[] { (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s };
    }
    double t = 2 * Math.Sqrt(1 + r[2, 2] - r[0, 0] - r[1, 1]);
    return new[] { (r[1, 0] - r[0, 1]) / t, (r[0, 2] + r[2, 0]) / t, (r[1, 2] + r[2, 1]) / t, 0.25 * t };
}

static double[] Vec(JsonElement e) => e.EnumerateArray().Select(v => v.GetDouble()).ToArray();

static double[,] Mat(JsonElement e)
{
    var rows = e.EnumerateArray().Select(Vec).ToArray();
    var m = new double[rows.Length, rows[0].Length];
    for (int i = 0; i < rows.Length; i++)
        for (int j = 0; j < rows[0].Length; j++)
            m[i, j] = rows[i][j];
    return m;
}

static double MaxErr(Array got, object expected)
{
    double[] g = got.Cast<double>().ToArray();
    double[] x = expected switch
    {
        double[] v => v,
        double[,] m => m.Cast<double>().ToArray(),
        JsonElement j => j.EnumerateArray().SelectMany(a => a.EnumerateArray()).SelectMany(b => b.EnumerateArray())
            .Select(v => v.GetDouble()).ToArray(),
        _ => throw new ArgumentException("expected"),
    };
    if (g.Length != x.Length)
        return double.PositiveInfinity;
    return g.Zip(x, (a, b) => Math.Abs(a - b)).Max();
}

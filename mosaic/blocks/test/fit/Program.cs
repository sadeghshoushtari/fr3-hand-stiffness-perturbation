using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using MOSAIC.Models.Elastography;

// dotnet run -- <reference.json from make_fit_reference.py>
// Fits the same synthetic trials as StiffnessModel.fit and compares parameters and cost; checks the identification,
// and the trial identification of PerturbationCalibrationCore (Stimulus states + robot state stream of a hand with a
// known stiffness) with the force from the robot spring and from the FR3 wrench.
// The two optimisers (scipy trust-region reflective, LM here) stop at slightly different points of a flat minimum,
// so the cost must agree to 1e-4 relative and the parameters to 0.01 (absolute, parameters in [0, 1]).
var root = JsonDocument.Parse(File.ReadAllText(args[0])).RootElement;
var a = root.GetProperty("arm").EnumerateArray().Select(v => v.GetDouble()).ToArray();
var arm = new ArmModel(a[0], a[1], a[2]);
var model = new StiffnessProjectionModel(arm, StiffnessFit.PriorGroups(arm), new StiffnessProjectionModel.IndexDefinition(),
    0, 90);
bool ok = true;
foreach (var c in root.GetProperty("cases").EnumerateArray())
{
    var w = Mat(c.GetProperty("weight"));
    var trials = c.GetProperty("trials").EnumerateArray().Select(t =>
    {
        var ang = Vec(t.GetProperty("angles"));
        return new FitTrial(Mat(t.GetProperty("K")), Vec(t.GetProperty("sigma")),
            arm.Geometry(ArmModel.SessionPosture(ang[0], ang[1])), w);
    }).ToList();
    var res = StiffnessFit.Fit(model, trials);
    var xPy = Vec(c.GetProperty("x"));
    double costPy = c.GetProperty("cost").GetDouble();
    double dx = res.X.Zip(xPy, (p, q) => Math.Abs(p - q)).Max();
    double dc = Math.Abs(res.Cost - costPy) / costPy;
    bool pass = dx < 0.01 && (res.Cost <= costPy || dc < 1e-4);
    ok &= pass;
    Console.WriteLine($"fit {c.GetProperty("name").GetString()}: {res.Iterations} iterations, cost {res.Cost:E4} "
                      + $"(python {costPy:E4}), largest parameter difference {dx:E1} {(pass ? "PASS" : "FAIL")}");
    Console.WriteLine("  C#     " + string.Join(" ", res.X.Select(v => v.ToString("F4"))));
    Console.WriteLine("  python " + string.Join(" ", xPy.Select(v => v.ToString("F4"))));
}
var id = root.GetProperty("identify");
var k = StiffnessFit.Identify(Rows(id.GetProperty("dx")), Rows(id.GetProperty("df")))!;
var kPy = Mat(id.GetProperty("K"));
double dk = Enumerable.Range(0, 9).Max(i => Math.Abs(k[i / 3, i % 3] - kPy[i / 3, i % 3]));
ok &= dk < 1e-9;
Console.WriteLine($"identify: largest difference {dk:E1} N/m {(dk < 1e-9 ? "PASS" : "FAIL")}");
foreach (var mode in new[] { "robot", "wrench" })
{
    var kHand = new[,] { { 600.0, 40.0, -20.0 }, { 40.0, 150.0, 10.0 }, { -20.0, 10.0, 450.0 } };
    var core = new PerturbationCalibrationCore(new PerturbationCalibrationCore.Options
    {
        ModelPath = Path.Combine(Path.GetTempPath(), "fitcheck_model.json"), LogDir = Path.GetTempPath(), ForceMode = mode,
    });
    core.OnEffort(0.0);
    double t = 0;
    var rng = new Random(1);
    foreach (var (axis, sign) in new[] { (0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1) })
        foreach (var (state, dur) in new[] { ("rest", 2.0), ("rise", 0.5), ("hold", 1.0), ("capture", 1.0), ("fall", 1.0) })
        {
            core.OnStimulus(state, t);
            for (double e = 0; e < dur; e += 0.001, t += 0.001)
            {
                var o = new double[3];
                if (state is "hold" or "capture") o[axis] = 0.015 * sign;
                // quasi-static handle: (K_r I + K_h) dx = K_r o; F_ext = force of the robot on the hand = K_h dx
                var a3 = new double[3, 3];
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        a3[r, c] = kHand[r, c] + (r == c ? 1000.0 : 0.0);
                var inv = StiffnessProjectionModel.Inverse3(a3)!;
                var dxv = Enumerable.Range(0, 3).Select(r => Enumerable.Range(0, 3).Sum(c => inv[r, c] * 1000.0 * o[c])).ToArray();
                var f = Enumerable.Range(0, 3).Select(r => Enumerable.Range(0, 3).Sum(c => kHand[r, c] * dxv[c]) + 0.05 * (rng.NextDouble() - 0.5)).ToArray();
                core.OnRobot(new[] { t, 0.45 + dxv[0], dxv[1], 0.4 + dxv[2], 0.45 + o[0], o[1], 0.4 + o[2], f[0], f[1], f[2], 0, 1, 0, 0, 1000.0 }, t);
            }
        }
    core.OnStimulus("rest", t);
    core.OnEffort(null);
    var kId = core.Trials[0].K!;
    double e2 = Math.Sqrt(Enumerable.Range(0, 9).Sum(i => Math.Pow(kId[i / 3, i % 3] - kHand[i / 3, i % 3], 2)))
                / Math.Sqrt(Enumerable.Range(0, 9).Sum(i => kHand[i / 3, i % 3] * kHand[i / 3, i % 3]));
    bool pass = core.Trials[0].Cycles.Count == 6 && e2 < 0.01;
    ok &= pass;
    Console.WriteLine($"calibration core, force {mode}: {core.Trials[0].Cycles.Count} cycles, K diag {kId[0, 0]:F1} "
                      + $"{kId[1, 1]:F1} {kId[2, 2]:F1} N/m (true 600 150 450), relative error {e2:E1} {(pass ? "PASS" : "FAIL")}");
}
return ok ? 0 : 1;

static double[] Vec(JsonElement e) => e.EnumerateArray().Select(v => v.GetDouble()).ToArray();

static double[][] Rows(JsonElement e) => e.EnumerateArray().Select(Vec).ToArray();

static double[,] Mat(JsonElement e)
{
    var rows = Rows(e);
    var m = new double[rows.Length, rows[0].Length];
    for (int i = 0; i < rows.Length; i++)
        for (int j = 0; j < rows[0].Length; j++)
            m[i, j] = rows[i][j];
    return m;
}

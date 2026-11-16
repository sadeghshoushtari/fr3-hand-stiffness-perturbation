using System;
using System.Globalization;
using System.IO;
using System.Linq;
using MOSAIC.Models.Elastography;

// dotnet run -- <dir with <case>_model.json, _features.csv, _expected.csv> <case> [<case> ...]
// Runs the packets through StiffnessProjectionModel and compares every output value with the Python reference.
// Stiffness entries: relative error (to max(|expected|, 1 N/m)); sigmas and flags: absolute error. Exit 1 on a miss.
// Format-2 models evaluate the geometry in C# and in Python separately, both by finite differences (H agrees to about
// 1e-6), so they are compared at 1e-3; format 1 carries the geometry in the file and must agree to rounding.
const double Tol = 1e-9, TolFormat2 = 1e-3;
var dir = args[0];
bool ok = true;
foreach (var name in args.Skip(1))
{
    var model = new StiffnessProjectionModel(Path.Combine(dir, $"{name}_model.json"));
    var features = Read(Path.Combine(dir, $"{name}_features.csv"));
    var expected = Read(Path.Combine(dir, $"{name}_expected.csv"));
    double worst = 0;
    int worstRow = -1, valid = 0;
    for (int i = 0; i < features.Length; i++)
    {
        var got = model.Step(features[i]);
        for (int k = 0; k < got.Length; k++)
        {
            double e = expected[i][k];
            double err = k < 7 ? Math.Abs(got[k] - e) / Math.Max(Math.Abs(e), 1.0) : Math.Abs(got[k] - e);
            if (err > worst) { worst = err; worstRow = i; }
        }
        valid += got[^1] == 1.0 ? 1 : 0;
    }
    bool pass = worst <= (model.Format == StiffnessProjectionModel.Format2 ? TolFormat2 : Tol);
    ok &= pass;
    Console.WriteLine($"{name}: {features.Length} packets, {valid} valid, largest error {worst:E2} (packet {worstRow}) "
                      + (pass ? "PASS" : "FAIL"));
}
return ok ? 0 : 1;

static double[][] Read(string path) => File.ReadAllLines(path).Where(l => l.Length > 0)
    .Select(l => l.Split(',').Select(v => double.Parse(v, CultureInfo.InvariantCulture)).ToArray()).ToArray();

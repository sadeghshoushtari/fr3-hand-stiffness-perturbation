using System;
using System.Collections.Generic;
using System.Linq;

namespace MOSAIC.Models.Elastography;

/// <summary>One calibration sample: identified hand stiffness (empty-handle stiffness subtracted), the sigmas of the
/// trial, the arm geometry at its posture and the weights of the stiffness entries in the fit.</summary>
public sealed record FitTrial(double[,] Stiffness, double[] Sigma, ArmGeometry Geometry, double[,] Weight);

/// <summary>
/// Identification and fit of projection/stiffness_model.py, without MOSAIC dependencies:
/// <see cref="Identify"/> = stiffness_model.identified_stiffness, <see cref="Fit"/> = StiffnessModel.fit for
/// stiffness trials (relative Frobenius error per trial, parameters in [0, 1], pull towards the prior).
/// </summary>
public static class StiffnessFit
{
    private static readonly string[] Shoulder =
    {
        "deltoid_anterior", "deltoid_middle", "deltoid_posterior", "supraspinatus", "infraspinatus", "subscapularis",
        "teres_minor", "teres_major", "pectoralis_clavicular", "pectoralis_sternal", "pectoralis_ribs",
        "latissimus_thoracic", "latissimus_lumbar", "latissimus_iliac", "coracobrachialis",
    };

    /// <summary>stiffness_model.GROUPS: name, members, driver weights (biceps, triceps), prior (a, b).</summary>
    public static readonly (string Name, string[] Members, double[] Weights, double A, double B)[] GroupSpec =
    {
        ("biceps", new[] { "biceps_long", "biceps_short" }, new[] { 1.0, 0.0 }, 0.01, 0.25),
        ("triceps", new[] { "triceps_long", "triceps_lateral", "triceps_medial", "anconeus" }, new[] { 0.0, 1.0 }, 0.01, 0.25),
        ("elbow_flexors", new[] { "brachialis", "brachioradialis", "extensor_carpi_radialis_longus", "pronator_teres" },
            new[] { 1.0, 0.0 }, 0.01, 0.25),
        ("shoulder", Shoulder, new[] { 0.5, 0.5 }, 0.01, 0.15),
    };

    /// <summary>The groups of stiffness_model.GROUPS with the prior parameters.</summary>
    public static StiffnessProjectionModel.Group[] PriorGroups(ArmModel arm) => GroupSpec.Select(g =>
        new StiffnessProjectionModel.Group(g.Name,
            g.Members.Select(m => Array.IndexOf(arm.Names, m) is var i and >= 0
                ? i : throw new ArgumentException($"muscle {m} not in the arm model")).ToArray(),
            (double[])g.Weights.Clone(), g.A, g.B)).ToArray();

    /// <summary>K = dF dx^T (dx dx^T)^-1, symmetrised (Zeynalli 2025, eq. 2.14). dx, df: one 3-vector per
    /// perturbation; df = restoring force of the hand (K positive for a stiff arm). Null if dx does not span 3-D.</summary>
    public static double[,]? Identify(IReadOnlyList<double[]> dx, IReadOnlyList<double[]> df)
    {
        var xx = new double[3, 3];
        var fx = new double[3, 3];
        for (int n = 0; n < dx.Count; n++)
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                {
                    xx[r, c] += dx[n][r] * dx[n][c];
                    fx[r, c] += df[n][r] * dx[n][c];
                }
        double trace = xx[0, 0] + xx[1, 1] + xx[2, 2];
        if (trace <= 0 || StiffnessProjectionModel.Det3(xx) < 1e-6 * trace * trace * trace)
            return null;  // perturbations do not cover three directions
        var inv = StiffnessProjectionModel.Inverse3(xx);
        if (inv is null)
            return null;
        var k = new double[3, 3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                for (int m = 0; m < 3; m++)
                    k[r, c] += fx[r, m] * inv[m, c];
        for (int r = 0; r < 3; r++)
            for (int c = r + 1; c < 3; c++)
                k[r, c] = k[c, r] = 0.5 * (k[r, c] + k[c, r]);
        return k;
    }

    public sealed record Result(double[] X, double Cost, int Iterations, double[] TrialErrors);

    /// <summary>Fits (a_g, b_g) of every group of the model to the trials; the model keeps its groups unchanged
    /// (apply the result with <see cref="StiffnessProjectionModel.SetGroupParameters"/>).</summary>
    public static Result Fit(StiffnessProjectionModel model, IReadOnlyList<FitTrial> trials, double priorWeight = 0.05)
    {
        if (trials.Count == 0)
            throw new ArgumentException("no stiffness trial to fit");
        var groups = model.Groups;
        var prior = groups.SelectMany(g =>
        {
            var spec = GroupSpec.FirstOrDefault(s => s.Name == g.Name);
            return spec.Name is null ? new[] { 0.01, 0.25 } : new[] { spec.A, spec.B };
        }).ToArray();
        var geo = trials.Select(t => StiffnessProjectionModel.Unpack(t.Geometry)).ToArray();

        double[] Residuals(double[] x)
        {
            var gx = groups.Select((g, i) => g with { A = x[2 * i], B = x[2 * i + 1] }).ToArray();
            var r = new List<double>(trials.Count * 9 + x.Length);
            for (int t = 0; t < trials.Count; t++)
            {
                var kc = model.Stiffness(trials[t].Sigma, gx, geo[t].J, geo[t].Jm, geo[t].H);
                var w = trials[t].Weight;
                var k = trials[t].Stiffness;
                double norm = 0;
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++)
                        norm += w[a, b] * k[a, b] * w[a, b] * k[a, b];
                norm = Math.Sqrt(norm);
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++)
                        r.Add(w[a, b] * ((kc?[a, b] ?? 1e6) - k[a, b]) / norm);
            }
            for (int i = 0; i < x.Length; i++)
                r.Add(priorWeight * (x[i] - prior[i]) / Math.Max(prior[i], 1e-3));
            return r.ToArray();
        }

        var (xBest, cost, iterations) = BoundedLeastSquares(Residuals, prior, 0.0, 1.0);
        var res = Residuals(xBest);
        var errors = new double[trials.Count];
        for (int t = 0; t < trials.Count; t++)
            errors[t] = Math.Sqrt(res.Skip(9 * t).Take(9).Sum(v => v * v));
        return new Result(xBest, cost, iterations, errors);
    }

    /// <summary>Levenberg-Marquardt with box bounds: variables at a bound whose gradient points outwards are held,
    /// steps are projected into the box. Cost = 0.5 sum r^2 (as scipy.optimize.least_squares).</summary>
    public static (double[] X, double Cost, int Iterations) BoundedLeastSquares(Func<double[], double[]> residuals,
        double[] x0, double lower, double upper, int maxIterations = 500)
    {
        int n = x0.Length;
        var x = x0.Select(v => Math.Clamp(v, lower, upper)).ToArray();
        var r = residuals(x);
        double cost = 0.5 * r.Sum(v => v * v);
        double lambda = 1e-3;
        int it;
        for (it = 0; it < maxIterations; it++)
        {
            // Jacobian by central differences (one-sided at the bounds)
            var jac = new double[r.Length, n];
            for (int k = 0; k < n; k++)
            {
                double h = 1e-7 * Math.Max(1.0, Math.Abs(x[k]));
                var xp = (double[])x.Clone();
                var xm = (double[])x.Clone();
                xp[k] = Math.Min(x[k] + h, upper);
                xm[k] = Math.Max(x[k] - h, lower);
                var rp = residuals(xp);
                var rm = residuals(xm);
                for (int i = 0; i < r.Length; i++)
                    jac[i, k] = (rp[i] - rm[i]) / (xp[k] - xm[k]);
            }
            var a = new double[n, n];
            var g = new double[n];
            for (int p = 0; p < n; p++)
            {
                for (int i = 0; i < r.Length; i++)
                    g[p] += jac[i, p] * r[i];
                for (int q = 0; q < n; q++)
                    for (int i = 0; i < r.Length; i++)
                        a[p, q] += jac[i, p] * jac[i, q];
            }
            var free = Enumerable.Range(0, n)
                .Where(k => !(x[k] <= lower && g[k] > 0) && !(x[k] >= upper && g[k] < 0)).ToArray();
            double gradNorm = free.Length == 0 ? 0 : free.Max(k => Math.Abs(g[k]));
            if (gradNorm < 1e-14)
                break;

            bool accepted = false;
            while (lambda < 1e12)
            {
                var m = new double[free.Length, free.Length];
                var b = new double[free.Length];
                for (int p = 0; p < free.Length; p++)
                {
                    for (int q = 0; q < free.Length; q++)
                        m[p, q] = a[free[p], free[q]];
                    m[p, p] += lambda * Math.Max(a[free[p], free[p]], 1e-12);
                    b[p] = -g[free[p]];
                }
                var step = Solve(m, b);
                var xNew = (double[])x.Clone();
                for (int p = 0; p < free.Length; p++)
                    xNew[free[p]] = Math.Clamp(x[free[p]] + step[p], lower, upper);
                var rNew = residuals(xNew);
                double costNew = 0.5 * rNew.Sum(v => v * v);
                if (costNew < cost)
                {
                    double change = cost - costNew;
                    double stepSize = Enumerable.Range(0, n).Max(k => Math.Abs(xNew[k] - x[k]));
                    x = xNew;
                    r = rNew;
                    cost = costNew;
                    lambda = Math.Max(lambda / 3.0, 1e-12);
                    accepted = true;
                    if (change < 1e-15 * Math.Max(cost, 1e-300) || stepSize < 1e-12)
                        return (x, cost, it + 1);
                    break;
                }
                lambda *= 4.0;
            }
            if (!accepted)
                break;
        }
        return (x, cost, it);
    }

    private static double[] Solve(double[,] m, double[] b)
    {
        int n = b.Length;
        var a = (double[,])m.Clone();
        var x = (double[])b.Clone();
        for (int c = 0; c < n; c++)
        {
            int piv = c;
            for (int r = c + 1; r < n; r++)
                if (Math.Abs(a[r, c]) > Math.Abs(a[piv, c]))
                    piv = r;
            for (int k = 0; k < n; k++)
                (a[c, k], a[piv, k]) = (a[piv, k], a[c, k]);
            (x[c], x[piv]) = (x[piv], x[c]);
            if (Math.Abs(a[c, c]) < 1e-300)
                continue;
            for (int r = c + 1; r < n; r++)
            {
                double f = a[r, c] / a[c, c];
                for (int k = c; k < n; k++)
                    a[r, k] -= f * a[c, k];
                x[r] -= f * x[c];
            }
        }
        for (int c = n - 1; c >= 0; c--)
        {
            double s = x[c];
            for (int k = c + 1; k < n; k++)
                s -= a[c, k] * x[k];
            x[c] = Math.Abs(a[c, c]) < 1e-300 ? 0 : s / a[c, c];
        }
        return x;
    }
}

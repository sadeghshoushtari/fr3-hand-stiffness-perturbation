using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using Microsoft.Extensions.DependencyInjection;
using MOSAIC.Components.Basics;
using MOSAIC.Components.Factory;
using MOSAIC.Components.Interfaces;
using MOSAIC.Models.Devices;
using MOSAIC.Models.FlowControl;

// dotnet run -- <pipeline.json> <seconds> [<actions.txt>]
// Builds the pipeline like the MOSAIC GUI does (JsonParser -> BlockGraphBuilder with BlockFactory), starts the Clock
// blocks, keeps it running for the given time, then disposes the blocks. actions.txt presses the card buttons at
// given times (s after the start), one per line:
//   <t> start <Stimulus>             Start of a Stimulus card
//   <t> capture <Trigger> <action>   select the action and Start capture on a Trigger card
//   <t> stop <Trigger>               Stop capture
var services = new ServiceCollection();
services.AddSingleton<IBlockFactory, BlockFactory>();
var sp = services.BuildServiceProvider();

await using var stream = File.OpenRead(args[0]);
var models = await JsonParser.ParseStreamAsync(stream) ?? throw new InvalidDataException("pipeline not parsed");
var graph = BlockGraphBuilder.Build(models, sp.GetRequiredService<IBlockFactory>());
Console.WriteLine($"pipeline running: {string.Join(", ", graph.Instances.Keys)}");

var actions = args.Length > 2
    ? File.ReadAllLines(args[2]).Select(l => l.Split('#')[0].Trim()).Where(l => l.Length > 0)
        .Select(l => l.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries))
        .Select(p => (T: double.Parse(p[0], CultureInfo.InvariantCulture), Args: p.Skip(1).ToArray()))
        .OrderBy(a => a.T).ToList()
    : new List<(double T, string[] Args)>();
foreach (var clock in graph.Instances.Values.OfType<ClockBlock>())
    clock.Start();

var watch = Stopwatch.StartNew();
double duration = double.Parse(args[1], CultureInfo.InvariantCulture);
foreach (var (t, a) in actions)
{
    var wait = t - watch.Elapsed.TotalSeconds;
    if (wait > 0)
        Thread.Sleep(TimeSpan.FromSeconds(wait));
    var block = graph.Instances.TryGetValue(a[1], out var b) ? b : throw new ArgumentException($"no block {a[1]}");
    switch (a[0], block)
    {
        case ("start", Stimulus s):
            s.Start();
            break;
        case ("capture", Trigger tr):
            tr.SelectAction(a[2]);
            tr.StartCapture();
            break;
        case ("stop", Trigger tr):
            tr.StopCapture();
            break;
        default:
            throw new ArgumentException($"unknown action '{string.Join(' ', a)}'");
    }
    Console.WriteLine($"{watch.Elapsed.TotalSeconds,7:F2} s  {string.Join(' ', a)}");
}
var rest = duration - watch.Elapsed.TotalSeconds;
if (rest > 0)
    Thread.Sleep(TimeSpan.FromSeconds(rest));
foreach (var block in graph.Instances.Values.OfType<IDisposable>())
    block.Dispose();
Console.WriteLine("pipeline stopped");

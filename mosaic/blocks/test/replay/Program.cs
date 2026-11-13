using System;
using System.IO;
using System.Linq;
using System.Threading;
using Microsoft.Extensions.DependencyInjection;
using MOSAIC.Components.Basics;
using MOSAIC.Components.Factory;
using MOSAIC.Components.Interfaces;

// dotnet run -- <pipeline.json> <seconds>
// Builds the pipeline like the MOSAIC GUI does (JsonParser -> BlockGraphBuilder with BlockFactory), keeps it running
// for the given time, then disposes the blocks.
var services = new ServiceCollection();
services.AddSingleton<IBlockFactory, BlockFactory>();
var sp = services.BuildServiceProvider();

await using var stream = File.OpenRead(args[0]);
var models = await JsonParser.ParseStreamAsync(stream) ?? throw new InvalidDataException("pipeline not parsed");
var graph = BlockGraphBuilder.Build(models, sp.GetRequiredService<IBlockFactory>());
Console.WriteLine($"pipeline running: {string.Join(", ", graph.Instances.Keys)}");
Thread.Sleep(TimeSpan.FromSeconds(double.Parse(args[1], System.Globalization.CultureInfo.InvariantCulture)));
foreach (var block in graph.Instances.Values.OfType<IDisposable>())
    block.Dispose();
Console.WriteLine("pipeline stopped");

#!/usr/bin/env node

/**
 * Benchmark pathfinding adapters on various scenarios
 *
 * Usage:
 *   npx tsx tests/pathfinding/benchmark.ts [<scenario> [<adapter>]]
 *   npx tsx tests/pathfinding/benchmark.ts --synthetic <map-name> [<adapter>]
 *   npx tsx tests/pathfinding/benchmark.ts --synthetic --all [<adapter>]
 *
 * Examples:
 *   npx tsx tests/pathfinding/benchmark.ts
 *   npx tsx tests/pathfinding/benchmark.ts giantworldmap-lite
 *   npx tsx tests/pathfinding/benchmark.ts giantworldmap-lite NavSat
 *   npx tsx tests/pathfinding/benchmark.ts --synthetic iceland
 *   npx tsx tests/pathfinding/benchmark.ts --synthetic iceland NavSat
 *   npx tsx tests/pathfinding/benchmark.ts --synthetic --all
 *   npx tsx tests/pathfinding/benchmark.ts --synthetic --all NavSat
 */

import { dirname, join } from "path";
import { fileURLToPath } from "url";
import { readdirSync } from "fs";
import {
  type BenchmarkResult,
  getAdapter,
  measureExecutionTime,
  measurePathLength,
  calculateStats,
  printHeader,
  measureTime,
  getScenario,
  printRow,
} from './utils';

const currentFile = fileURLToPath(import.meta.url);
const pathfindingDir = dirname(currentFile);
const syntheticScenariosDir = join(pathfindingDir, "scenarios", "synthetic");

async function runScenario(adapterName: string, scenarioName: string) {
  const { game, routes, performanceIterations } = await getScenario(scenarioName);
  const adapter = getAdapter(game, adapterName);

  console.log(`Date: ${new Date().toISOString()}`);
  console.log(`Benchmarking: ${adapterName}`);
  console.log(`Scenario: ${scenarioName}`);
  console.log(`Routes: ${routes.length}`);
  console.log("");

  // =============================================================================

  printHeader("METRIC 1: INITIALIZATION TIME");

  const { time: initializationTime } = measureTime(() => adapter.initialize());

  console.log(`Initialization time: ${initializationTime.toFixed(2)}ms`);
  console.log("");

  // =============================================================================

  printHeader("METRIC 2: PATH DISTANCE");

  const results: BenchmarkResult[] = [];

  printRow(["Route", "Path Length"], [40, 12]);

  for (const route of routes) {
    const pathLength = measurePathLength(adapter, route);
    results.push({ route: route.name, pathLength, executionTime: null });
    printRow([route.name, pathLength !== null ? `${pathLength} tiles` : "FAILED"], [40, 12]);
  }

  const { totalDistance, successfulRoutes, totalRoutes } = calculateStats(results);

  console.log("");
  console.log(`Total distance: ${totalDistance} tiles`);
  console.log(`Routes completed: ${successfulRoutes} / ${totalRoutes}`);
  console.log("");

  // =============================================================================

  printHeader("METRIC 3: PATHFINDING TIME");

  printRow(["Route", "Time"], [40, 12]);

  for (const route of routes) {
    const result = results.find(r => r.route === route.name);

    if (result && result.pathLength !== null) {
      const execTime = measureExecutionTime(adapter, route, performanceIterations);
      result.executionTime = execTime;
      printRow([route.name, `${execTime!.toFixed(2)}ms`], [40, 12]);
    } else {
      printRow([route.name, "FAILED"], [40, 12]);
    }
  }

  const stats = calculateStats(results);

  console.log("");
  console.log(`Total time: ${stats.totalTime.toFixed(2)}ms`);
  console.log(`Average time: ${stats.avgTime.toFixed(2)}ms`);
  console.log(`Routes benchmarked: ${stats.timedRoutes} / ${stats.totalRoutes}`);
  console.log("");

  // =============================================================================

  printHeader("SUMMARY");

  console.log(`Adapter: ${adapter.name}`);
  console.log(`Scenario: ${scenarioName}`);
  console.log("");

  if (stats.successfulRoutes < stats.totalRoutes) {
    console.log(`Warning: Only ${stats.successfulRoutes} out of ${stats.totalRoutes} routes were completed successfully!`);
    console.log("");
  }

  console.log("Scores:");
  console.log(`  Initialization: ${initializationTime.toFixed(2)}ms`);
  console.log(`  Pathfinding: ${stats.totalTime.toFixed(2)}ms`);
  console.log(`  Distance: ${totalDistance} tiles`);
  console.log("");
}

function printUsage() {
  console.log(`
Usage:
  npx tsx tests/pathfinding/benchmark.ts [<scenario> [<adapter>]]
  npx tsx tests/pathfinding/benchmark.ts --synthetic <map-name> [<adapter>]
  npx tsx tests/pathfinding/benchmark.ts --synthetic --all [<adapter>]

Arguments:
  <scenario>    Name of the scenario to benchmark (default: giantworldmap-lite)
  <adapter>     Name of the pathfinding adapter to use (default: NavSat)
  --synthetic   Run synthetic scenarios
  --all         Run all synthetic scenarios (requires --synthetic)

Examples:
  npx tsx tests/pathfinding/benchmark.ts
  npx tsx tests/pathfinding/benchmark.ts giantworldmap-lite
  npx tsx tests/pathfinding/benchmark.ts giantworldmap-lite NavSat
  npx tsx tests/pathfinding/benchmark.ts --synthetic iceland
  npx tsx tests/pathfinding/benchmark.ts --synthetic iceland NavSat
  npx tsx tests/pathfinding/benchmark.ts --synthetic --all
  npx tsx tests/pathfinding/benchmark.ts --synthetic --all NavSat

Available synthetic scenarios:
  Run 'ls tests/pathfinding/scenarios/synthetic' to see all available scenarios
`);
}

async function main() {
  const args = process.argv.slice(2);

  if (args.includes("--help") || args.includes("-h")) {
    printUsage();
    process.exit(0);
  }

  const isSynthetic = args.includes("--synthetic");
  const isAll = args.includes("--all");
  const nonFlagArgs = args.filter((arg) => !arg.startsWith("--"));

  if (isSynthetic) {
    if (isAll) {
      // Run all synthetic scenarios
      const adapterName = nonFlagArgs[0] || "NavSat";

      // Find all synthetic scenario files
      const scenarioFiles = readdirSync(syntheticScenariosDir)
        .filter((file) => file.endsWith(".ts"))
        .map((file) => file.replace(".ts", ""))
        .sort();

      console.log(`Running ${scenarioFiles.length} synthetic scenarios with ${adapterName} adapter...`);
      console.log("");

      for (let i = 0; i < scenarioFiles.length; i++) {
        const mapName = scenarioFiles[i];
        const scenarioName = `synthetic/${mapName}`;

        console.log(`[${i + 1}/${scenarioFiles.length}] Running: ${scenarioName}`);
        console.log("=".repeat(80));
        console.log("");

        await runScenario(adapterName, scenarioName);

        if (i < scenarioFiles.length - 1) {
          console.log("");
          console.log("=".repeat(80));
          console.log("");
        }
      }
    } else if (nonFlagArgs.length >= 1) {
      // Run single synthetic scenario
      const mapName = nonFlagArgs[0];
      const adapterName = nonFlagArgs[1] || "NavSat";
      const scenarioName = `synthetic/${mapName}`;

      await runScenario(adapterName, scenarioName);
    } else {
      console.error("Error: --synthetic requires a map name or --all flag");
      printUsage();
      process.exit(1);
    }
  } else {
    // Standard mode with positional arguments
    const scenarioName = nonFlagArgs[0] || "giantworldmap-lite";
    const adapterName = nonFlagArgs[1] || "NavSat";

    await runScenario(adapterName, scenarioName);
  }
}

main().catch((error) => {
  console.error("Fatal error:", error);
  process.exit(1);
});

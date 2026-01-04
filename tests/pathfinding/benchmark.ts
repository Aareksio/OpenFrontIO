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

const scenarioName = process.argv[2] || "giantworldmap-lite";
const adapterName = process.argv[3] || "NavSat";

runScenario(adapterName, scenarioName);

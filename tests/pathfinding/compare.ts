import { PathFinderMiniAdapter, NavigationSatelliteAdapter } from "./pathfinding-interface";
import { getScenario, printHeader, measureTime, measurePathLength, BenchmarkResult, printRow, calculateStats, measureExecutionTime } from './utils';

function compare(n1: number, n2: number): string {
  const change = n2 - n1;
  const percent = (change / n1) * 100;
  const sign = percent >= 0 ? "+" : "";
  return `${sign}${percent.toFixed(2)}%`;
}

async function runScenario(scenarioName: string) {
  const { game, routes, performanceIterations } = await getScenario(scenarioName);

  console.log(`Scenario: ${scenarioName}`);
  console.log(`Routes: ${routes.length}`);
  console.log("");
  console.log(`Baseline: PathFinder.Mini`);
  console.log(`Candidate: TradeShipNavigator`);
  console.log("");

  const baseline = new PathFinderMiniAdapter(game);
  const candidate = new NavigationSatelliteAdapter(game);

  // =============================================================================

  printHeader("METRIC 1: INITIALIZATION TIME");

  const { time: baselineInitializationTime } = measureTime(() => baseline.initialize());
  console.log(`Baseline initialization time: ${baselineInitializationTime.toFixed(2)}ms`);

  const { time: candidateInitializationTime } = measureTime(() => candidate.initialize());
  console.log(`Candidate initialization time: ${candidateInitializationTime.toFixed(2)}ms`);

  console.log("");

  // =============================================================================

  printHeader("METRIC 2: PATH DISTANCE");

  const results = {
    baseline: [] as BenchmarkResult[],
    candidate: [] as BenchmarkResult[],
  }

  printRow(
    ["Route", "Baseline", "Candidate"],
    [40, 12, 12],
  );

  for (const route of routes) {
    const baselinePathLength = measurePathLength(baseline, route);
    results.baseline.push({ route: route.name, pathLength: baselinePathLength, executionTime: null });

    const candidatePathLength = measurePathLength(candidate, route);
    results.candidate.push({ route: route.name, pathLength: candidatePathLength, executionTime: null });

    printRow(
      [
        route.name,
        baselinePathLength !== null ? `${baselinePathLength} tiles` : "FAILED",
        candidatePathLength !== null ? `${candidatePathLength} tiles` : "FAILED",
      ],
      [40, 12, 12],
    );
  }

  const baselineQualityStats = calculateStats(results.baseline);
  const candidateQualityStats = calculateStats(results.candidate);

  const distanceChange = compare(baselineQualityStats.totalDistance, candidateQualityStats.totalDistance);
  const completionChange = compare(baselineQualityStats.successfulRoutes, candidateQualityStats.successfulRoutes);

  console.log("");
  console.log(`Total distance: ${baselineQualityStats.totalDistance} vs ${candidateQualityStats.totalDistance} (${distanceChange})`);
  console.log(`Routes completed: ${baselineQualityStats.successfulRoutes} / ${baselineQualityStats.totalRoutes} (${completionChange})`);
  console.log("");

  // =============================================================================

  printHeader("METRIC 3: PATHFINDING TIME");

  printRow(
    ["Route", "Baseline", "Candidate"], 
    [40, 12, 12],
  );


  for (const route of routes) {
    const baselineResult = results.baseline.find(r => r.route === route.name);
    const candidateResult = results.candidate.find(r => r.route === route.name);

    if (baselineResult && baselineResult.pathLength !== null) {
      const baselineExecTime = measureExecutionTime(baseline, route, performanceIterations);
      baselineResult.executionTime = baselineExecTime;
    }

    if (candidateResult && candidateResult.pathLength !== null) {
      const candidateExecTime = measureExecutionTime(candidate, route, performanceIterations);
      candidateResult.executionTime = candidateExecTime;
    }

    const baselineTimeStr = baselineResult && baselineResult.executionTime !== null
      ? `${baselineResult.executionTime!.toFixed(2)}ms`
      : "FAILED";

    const candidateTimeStr = candidateResult && candidateResult.executionTime !== null
      ? `${candidateResult.executionTime!.toFixed(2)}ms`
      : "FAILED";

    printRow(
      [
        route.name,
        baselineTimeStr,
        candidateTimeStr,
      ],
      [40, 12, 12]
    );
  }

  const baselineStats = calculateStats(results.baseline);
  const candidateStats = calculateStats(results.candidate);

  
  const timeChange = compare(baselineStats.totalTime, candidateStats.totalTime);
  const avgTimeChange = compare(baselineStats.avgTime, candidateStats.avgTime);
  const timedChange = compare(baselineStats.timedRoutes, candidateStats.timedRoutes);

  console.log("");
  console.log(`Total time: ${baselineStats.totalTime.toFixed(2)}ms vs ${candidateStats.totalTime.toFixed(2)}ms (${timeChange})`);
  console.log(`Average time: ${baselineStats.avgTime.toFixed(2)}ms vs ${candidateStats.avgTime.toFixed(2)}ms (${avgTimeChange})`);
  console.log(`Routes benchmarked: ${baselineStats.timedRoutes} / ${baselineStats.totalRoutes} (${timedChange})`);

  // =============================================================================

  printHeader("SUMMARY");

  console.log(`Comparing: ${baseline.name} vs ${candidate.name}`);
  console.log(`Scenario: ${scenarioName}`);
  console.log("");

  console.log("Baseline Scores:")
  console.log(`  Initialization: ${baselineInitializationTime.toFixed(2)}ms`);
  console.log(`  Pathfinding: ${baselineStats.totalTime.toFixed(2)}ms`);
  console.log(`  Distance: ${baselineQualityStats.totalDistance} tiles`);

  if (baselineStats.successfulRoutes < baselineStats.totalRoutes) {
    console.log(`  !! Warning: Only ${baselineStats.successfulRoutes} out of ${baselineStats.totalRoutes} routes were completed successfully!`);
  }

  console.log("");

  console.log("Candidate Scores:")
  console.log(`  Initialization: ${candidateInitializationTime.toFixed(2)}ms`);
  console.log(`  Pathfinding: ${candidateStats.totalTime.toFixed(2)}ms`);
  console.log(`  Distance: ${candidateQualityStats.totalDistance} tiles`);

  if (candidateStats.successfulRoutes < candidateStats.totalRoutes) {
    console.log(`  !! Warning: Only ${candidateStats.successfulRoutes} out of ${candidateStats.totalRoutes} routes were completed successfully!`);
  }

  console.log("");


  const initializationChange = compare(baselineInitializationTime, candidateInitializationTime);

  console.log("Overall Changes:");
  console.log(`  Initialization time: ${initializationChange}`);
  console.log(`  Pathfinding time: ${timeChange}`);
  console.log(`  Distance: ${distanceChange}`);
  console.log("");
}

// Get scenario from command line argument
const scenarioName = process.argv[2] || "giantworldmap-lite";
runScenario(scenarioName);

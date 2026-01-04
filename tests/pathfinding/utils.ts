import { Game } from '../../src/core/game/Game';
import { TileRef } from '../../src/core/game/GameMap';
import { PathFinderMiniAdapter, PathfindingInterface, NavigationSatelliteAdapter, VimacsTileToShoreAdapter, VimacsTileToWaterAdapter } from './pathfinding-interface';

export const DEFAULT_ITERATIONS = 100

export type BenchmarkRoute = {
  name: string,
  from: TileRef,
  to: TileRef,
}

export type BenchmarkResult = {
  route: string;
  executionTime: number | null;
  pathLength: number | null;
}

export type BenchmarkSummary = {
  totalRoutes: number;
  successfulRoutes: number;
  timedRoutes: number;
  totalDistance: number;
  totalTime: number;
  avgTime: number;
}

export function getAdapter(
  game: Game,
  name: string, 
): PathfindingInterface {
  switch (name) {
    case "PF.Mini":
      return new PathFinderMiniAdapter(game);
    case "NavSat":
      return new NavigationSatelliteAdapter(game);
    case "Vimacs.TileToShore":
      return new VimacsTileToShoreAdapter(game);
    case "Vimacs.TileToWater":
      return new VimacsTileToWaterAdapter(game);
    default:
      throw new Error(`Unknown pathfinding adapter: ${name}`);
  }
}

export async function getScenario(
  scenarioName: string,
) {
  const scenario = await import(`./scenarios/${scenarioName}.js`);

  const game = await scenario.setupGame();

  const routes = scenario.ROUTES.map(
    ([fromName, toName]: [string, string]) => {
      const fromCoord: [number, number] = scenario.PORTS[fromName];
      const toCoord: [number, number] = scenario.PORTS[toName];

      return {
        name: `${fromName} → ${toName}`,
        from: game.ref(fromCoord[0], fromCoord[1]),
        to: game.ref(toCoord[0], toCoord[1]),
      };
    }
  );

  return { 
    game, 
    routes, 
    performanceIterations: scenario.PERFORMANCE_ITERATIONS ?? DEFAULT_ITERATIONS 
  };
}

export function measurePathLength(
  adapter: PathfindingInterface,
  route: BenchmarkRoute,
): number | null {
  const path = adapter.findPath(route.from, route.to);
  return path ? path.length : null;
}

export function measureTime<T>(fn: () => T): { result: T; time: number } {
  const start = performance.now();
  const result = fn();
  const end = performance.now();
  return { result, time: end - start };
}

export function measureExecutionTime(
  adapter: PathfindingInterface, 
  route: BenchmarkRoute,
  executions: number = 100,
): number | null {
  const { time } = measureTime(() => {
    for (let i = 0; i < executions; i++) {
      adapter.findPath(route.from, route.to);
    }
  })

  return time / executions;
}

export function calculateStats(
  results: BenchmarkResult[]
): BenchmarkSummary {
  const successful = results.filter((r) => r.pathLength !== null);
  const timed = results.filter((r) => r.executionTime !== null);

  const totalDistance = successful.reduce((sum, r) => sum + r.pathLength!, 0);
  const totalTime = timed.reduce((sum, r) => sum + r.executionTime!, 0);
  const avgTime = timed.length > 0 ? totalTime / timed.length : 0;

  return {
    totalRoutes: results.length,
    successfulRoutes: successful.length,
    timedRoutes: timed.length,
    totalDistance,
    totalTime,
    avgTime,
  };
}

export function printRow(
  columns: (string | number)[], 
  widths: number[]
): void {
  const formatted = columns.map((col, i) => {
    const str = typeof col === 'number' ? col.toString() : col;
    return str.padEnd(widths[i]);
  });

  console.log(formatted.join(' '));
}

export function printSeparator(
  width: number = 80
): void {
  console.log('-'.repeat(width));
}

export function printHeader(
  title: string, 
  width: number = 80
): void {
  printSeparator(width);
  console.log(title);
  printSeparator(width);
  console.log('');
}

import Benchmark from "benchmark";
import { dirname } from "path";
import { fileURLToPath } from "url";

import { PathFinder } from "../../src/core/pathfinding/PathFinding";
import { setup } from "../util/Setup";
import { PathFindResultType } from '../../src/core/pathfinding/AStar';

const game = await setup(
  "giantworldmap",
  {},
  [],
  dirname(fileURLToPath(import.meta.url)),
);

const PORTS: { [key: string]: [number, number] } = {
  "Miami": [1014, 694],
  "Boston": [1120, 498],
  "Houston": [844, 657],
  "Chicago": [928, 508],
  "Cleveland": [995, 514],
  "Barcelona": [1943, 518],
  "Sao Paulo": [1396, 1288],
  "San Francisco": [520, 547],
  "Falkland Islands": [1259, 1616],
  "San Salvador": [901, 842],
  "Neapol": [2081, 516],
  "San Felipe (MX)": [617, 637],
  "Anchorage": [217, 281],
  "Honolulu": [156, 768],
  "Luxor": [2302, 700],
  "Venice": [2068, 468],
  "Abu Zabi": [2554, 711],
  "Gdansk": [2143, 362],
  "Shanghai": [3317, 639],
  "Tokyo": [3525, 583],
  "Kongo River": [2142, 965],
  "Ghana River": [1877, 869],
  "Guinea River": [1824, 868],
  "Peru River": [1097, 1162],
  "China Desert River": [2965, 599],
  "Australia River": [3663, 1343],
}

function testPath(from: [number, number], to: [number, number]) {
  const startTile = game.ref(from[0], from[1]);
  const endTile = game.ref(to[0], to[1]);

  const pathfinder = PathFinder.Mini(game, 2500);

  while (true) {
    const result = pathfinder.nextTile(startTile, endTile);

    if (result.type === PathFindResultType.NextTile) {
      // Recompute applies only to moving targets, ports are static
      break;
    }

    if (result.type === PathFindResultType.PathNotFound) {
      throw new Error("Path not found");
    }
  }
}

const suite = new Benchmark.Suite()

function addRoute(fromName: keyof typeof PORTS, toName: keyof typeof PORTS) {
  const from = PORTS[fromName];
  const to = PORTS[toName];
  suite.add(`${fromName} to ${toName}`, () => testPath(from, to));
}

addRoute("Miami", "Boston");
addRoute("Miami", "Houston");
addRoute("Miami", "Chicago");
addRoute("Miami", "Cleveland");
addRoute("Miami", "Barcelona");
addRoute("Miami", "Sao Paulo");
addRoute("Miami", "San Francisco");
addRoute("Miami", "Falkland Islands");
addRoute("Miami", "San Salvador");
addRoute("Luxor", "Venice");
addRoute("Luxor", "Kongo River");
addRoute("Shanghai", "Tokyo");
addRoute("Shanghai", "San Francisco");
addRoute("Abu Zabi", "Gdansk");
addRoute("Chicago", "Cleveland");
addRoute("Barcelona", "Neapol");
addRoute("San Francisco", "San Felipe (MX)");
addRoute("Anchorage", "Honolulu");
addRoute("Kongo River", "Ghana River");
addRoute("Kongo River", "Guinea River");
addRoute("Peru River", "China Desert River");
addRoute("China Desert River", "Australia River");
addRoute("Peru River", "China Desert River");

suite
  .on("cycle", (event: any) => {
    if (event.target.error) {
      console.log(`${event.target.name} failed: ${event.target.error.message}`)
      return
    }

    console.log(`${event.target}`)
  })
  .run({ maxTime: 10 });

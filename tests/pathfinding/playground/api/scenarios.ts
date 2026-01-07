import { readdirSync } from "fs";
import { join, dirname } from "path";
import { fileURLToPath } from "url";
import { Game } from "../../../../src/core/game/Game.js";
import { getScenario } from "../../utils.js";

export interface ScenarioInfo {
  name: string;
  displayName: string;
}

export interface ScenarioCache {
  game: Game;
  ports: Map<string, [number, number]>;
}

const cache = new Map<string, ScenarioCache>();

/**
 * Get list of available scenarios by reading the scenarios directory
 * Includes both root scenarios and synthetic subdirectory scenarios
 */
export function listScenarios(): ScenarioInfo[] {
  const scenariosDir = join(
    dirname(fileURLToPath(import.meta.url)),
    "../../scenarios"
  );

  const scenarios: ScenarioInfo[] = [];

  // Scan root scenarios directory
  const files = readdirSync(scenariosDir, { withFileTypes: true });

  for (const file of files) {
    if (file.isFile() && (file.name.endsWith(".ts") || file.name.endsWith(".js"))) {
      const name = file.name.replace(/\.(ts|js)$/, "");
      const displayName = name
        .split("-")
        .map((word) => word.charAt(0).toUpperCase() + word.slice(1))
        .join(" ");
      scenarios.push({ name, displayName });
    }
  }

  // Scan synthetic subdirectory
  const syntheticDir = join(scenariosDir, "synthetic");
  try {
    const syntheticFiles = readdirSync(syntheticDir);
    for (const file of syntheticFiles) {
      if (file.endsWith(".ts") || file.endsWith(".js")) {
        const name = "synthetic/" + file.replace(/\.(ts|js)$/, "");
        const baseName = file.replace(/\.(ts|js)$/, "");
        const displayName = "[Synthetic] " + baseName
          .split("-")
          .map((word) => word.charAt(0).toUpperCase() + word.slice(1))
          .join(" ");
        scenarios.push({ name, displayName });
      }
    }
  } catch (e) {
    // Synthetic directory doesn't exist, skip it
  }

  return scenarios.sort((a, b) => a.displayName.localeCompare(b.displayName));
}

/**
 * Load a scenario from cache or disk
 */
export async function loadScenario(
  scenarioName: string
): Promise<ScenarioCache> {
  // Check cache first
  if (cache.has(scenarioName)) {
    return cache.get(scenarioName)!;
  }

  // Load scenario using existing utility
  const { game } = await getScenario(scenarioName);

  // Extract ports from the scenario module
  const scenarioModule = await import(`../../scenarios/${scenarioName}.js`);
  const ports = new Map<string, [number, number]>();

  if (scenarioModule.PORTS) {
    for (const [name, coords] of Object.entries(scenarioModule.PORTS)) {
      ports.set(name, coords as [number, number]);
    }
  }

  const cacheEntry: ScenarioCache = { game, ports };

  // Store in cache
  cache.set(scenarioName, cacheEntry);

  return cacheEntry;
}

/**
 * Get scenario metadata for client
 */
export async function getScenarioMetadata(scenarioName: string) {
  const { game, ports } = await loadScenario(scenarioName);

  // Extract map data
  const mapData: number[] = [];
  for (let y = 0; y < game.height(); y++) {
    for (let x = 0; x < game.width(); x++) {
      const tile = game.ref(x, y);
      mapData.push(game.isWater(tile) ? 1 : 0);
    }
  }

  // Convert ports to array
  const portsArray = Array.from(ports.entries()).map(([name, [x, y]]) => ({
    name,
    x,
    y,
  }));

  return {
    name: scenarioName,
    width: game.width(),
    height: game.height(),
    mapData,
    ports: portsArray,
  };
}

/**
 * Clear scenario cache (useful for testing or memory management)
 */
export function clearCache() {
  cache.clear();
}

import { Game } from "../../src/core/game/Game";
import { TileRef } from "../../src/core/game/GameMap";
import { NavMesh } from "../../src/core/pathfinding/navmesh/NavMesh";
import { MiniPathFinder } from "../../src/core/pathfinding/PathFinding";
import { PathFindResultType } from "../../src/core/pathfinding/AStar";

/**
 * Common interface for pathfinding implementations.
 * Allows benchmarking different pathfinding strategies uniformly.
 */
export interface PathfindingInterface {
  /**
   * Display name for this pathfinding implementation
   */
  readonly name: string;

  /**
   * Initialize the pathfinding implementation (e.g., build preprocessing structures).
   * No-op if no initialization is required.
   */
  initialize(): void;

  /**
   * Find a path from start to end.
   * @returns Array of tiles representing the path, or null if no path exists
   */
  findPath(from: TileRef, to: TileRef): TileRef[] | null;
}

export class PathFinderMiniAdapter implements PathfindingInterface {
  readonly name = "PF.Mini";
  private readonly game: Game;
  private readonly maxIterations: number;

  constructor(game: Game, options: { maxIterations?: number } = {}) {
    this.game = game;
    this.maxIterations = options.maxIterations ?? 250000;
  }

  initialize(): void {
    // No initialization needed
  }

  findPath(from: TileRef, to: TileRef): TileRef[] | null {
    try {
      const pathfinder = new MiniPathFinder(this.game, this.maxIterations, true, 20);
      const path: TileRef[] = [from];
      let current = from;

      while (true) {
        const result = pathfinder.nextTile(current, to);

        if (result.type === PathFindResultType.NextTile) {
          path.push(result.node);
          current = result.node;
        } else if (result.type === PathFindResultType.Completed) {
          return path;
        } else if (result.type === PathFindResultType.PathNotFound) {
          return null;
        }
      }
    } catch (e) {
      return null;
    }
  }
}

export class NavigationSatelliteAdapter implements PathfindingInterface {
  readonly name = "NavSat";
  private readonly navMesh: NavMesh;

  constructor(game: Game) {
    this.navMesh = new NavMesh(game);
  }

  initialize(): void {
    this.navMesh.initialize();
  }

  findPath(from: TileRef, to: TileRef): TileRef[] | null {
    try {
      return this.navMesh.findPath(from, to);
    } catch (e) {
      return null;
    }
  }
}

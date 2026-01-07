import { Game } from "../../src/core/game/Game";
import { TileRef } from "../../src/core/game/GameMap";
import { NavigationSatellite } from "../../src/core/pathfinding/experimental/navigation-satellite/NavigationSatelitte";
import { MiniPathFinder } from "../../src/core/pathfinding/PathFinding";
import { PathFindResultType } from "../../src/core/pathfinding/AStar";
import { boatPathFromTileToShore, boatPathFromTileToWater } from '../../src/core/pathfinding/experimental/vimacs/TransportShipUtils';

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
  private readonly satellite: NavigationSatellite;

  constructor(game: Game) {
    this.satellite = new NavigationSatellite(game);
  }

  initialize(): void {
    this.satellite.initialize();
  }

  findPath(from: TileRef, to: TileRef): TileRef[] | null {
    try {
      return this.satellite.findPath(from, to);
    } catch (e) {
      return null;
    }
  }
}

export class VimacsTileToShoreAdapter implements PathfindingInterface {
  readonly name = "Vimacs.TileToShore";
  private readonly game: Game;

  constructor(game: Game) {
    this.game = game;
  }

  initialize(): void {
    // No initialization needed
  }

  findPath(from: TileRef, to: TileRef): TileRef[] | null {
    try {
      const gm = this.game.map();
      return boatPathFromTileToShore(gm, from, to);
    } catch (e) {
      return null;
    }
  }
}

export class VimacsTileToWaterAdapter implements PathfindingInterface {
  readonly name = "Vimacs.TileToWater";
  private readonly game: Game;

  constructor(game: Game) {
    this.game = game;
  }

  initialize(): void {
    // No initialization needed
  }

  findPath(from: TileRef, to: TileRef): TileRef[] | null {
    try {
      const gm = this.game.map();
      return boatPathFromTileToWater(gm, from, to);
    } catch (e) {
      return null;
    }
  }
}

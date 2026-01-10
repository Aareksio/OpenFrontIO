import { GameMap, TileRef } from "../../game/GameMap";

/**
 * SpatialQuery - interface for spatial queries on a graph.
 * Separate from PathFinder: finds single tiles, not paths.
 */
export interface SpatialQuery<T> {
  /**
   * Find nearest tile matching predicate using BFS traversal.
   * Respects terrain/traversability.
   */
  bfsNearest(from: T, maxDist: number, predicate: (t: T) => boolean): T | null;

  /**
   * Find all tiles within distance matching predicate using BFS.
   */
  bfsWithinDistance(
    from: T,
    maxDist: number,
    predicate: (t: T) => boolean,
  ): T[];

  /**
   * Find closest tile by Manhattan distance from a set.
   * Pure coordinate distance, ignores obstacles.
   */
  manhattanNearest(tiles: T[], target: T): T | null;
}

/**
 * TileSpatialQuery - SpatialQuery implementation for GameMap tiles.
 * Uses GameMap.bfs() for traversal-based queries.
 */
export class TileSpatialQuery implements SpatialQuery<TileRef> {
  constructor(private map: GameMap) {}

  bfsNearest(
    from: TileRef,
    maxDist: number,
    predicate: (t: TileRef) => boolean,
  ): TileRef | null {
    const candidates: TileRef[] = [];

    for (const tile of this.map.bfs(
      from,
      (_, t) => this.map.manhattanDist(from, t) <= maxDist,
    )) {
      if (predicate(tile)) {
        candidates.push(tile);
      }
    }

    if (candidates.length === 0) return null;

    // Sort by Manhattan distance to find actual nearest
    candidates.sort(
      (a, b) =>
        this.map.manhattanDist(from, a) - this.map.manhattanDist(from, b),
    );

    return candidates[0];
  }

  bfsWithinDistance(
    from: TileRef,
    maxDist: number,
    predicate: (t: TileRef) => boolean,
  ): TileRef[] {
    const result: TileRef[] = [];

    for (const tile of this.map.bfs(
      from,
      (_, t) => this.map.manhattanDist(from, t) <= maxDist,
    )) {
      if (predicate(tile)) {
        result.push(tile);
      }
    }

    return result;
  }

  manhattanNearest(tiles: TileRef[], target: TileRef): TileRef | null {
    if (tiles.length === 0) return null;

    return tiles.reduce((closest, current) =>
      this.map.manhattanDist(target, current) <
      this.map.manhattanDist(target, closest)
        ? current
        : closest,
    );
  }
}

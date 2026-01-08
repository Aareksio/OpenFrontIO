// Bounded A* - fully inlined grid search with bounds checking
// Generic implementation that works with any GameMap

import { GameMap, TileRef } from "../../game/GameMap";
import { AStar } from "./AStar";
import { BucketQueue } from "./PriorityQueue";

const LAND_BIT = 7;

export interface BoundedAStarConfig {
  heuristicWeight?: number;
  maxIterations?: number;
}

export interface SearchBounds {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
}

export class BoundedAStar implements AStar {
  private stamp = 1;

  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Uint32Array;
  private readonly cameFrom: Int32Array;
  private readonly queue: BucketQueue;
  private readonly terrain: Uint8Array;
  private readonly mapWidth: number;
  private readonly heuristicWeight: number;
  private readonly maxIterations: number;

  constructor(map: GameMap, maxSearchArea: number, config?: BoundedAStarConfig) {
    this.terrain = (map as any).terrain as Uint8Array;
    this.mapWidth = map.width();
    this.heuristicWeight = config?.heuristicWeight ?? 1;
    this.maxIterations = config?.maxIterations ?? 100_000;

    this.closedStamp = new Uint32Array(maxSearchArea);
    this.gScoreStamp = new Uint32Array(maxSearchArea);
    this.gScore = new Uint32Array(maxSearchArea);
    this.cameFrom = new Int32Array(maxSearchArea);

    // BucketQueue max priority: worst case is diagonal across search area
    // For 96x96 area with weight 1: maxF = 1 * (96 + 96) = 192
    // Use sqrt(maxSearchArea) * 2 * weight as upper bound
    const maxDim = Math.ceil(Math.sqrt(maxSearchArea));
    const maxF = this.heuristicWeight * maxDim * 2;
    this.queue = new BucketQueue(maxF);
  }

  // Implement AStar interface - unbounded search on full map
  search(start: number, goal: number): number[] | null {
    // For unbounded search, use full map bounds
    const startX = start % this.mapWidth;
    const startY = (start / this.mapWidth) | 0;
    const goalX = goal % this.mapWidth;
    const goalY = (goal / this.mapWidth) | 0;

    const minX = Math.min(startX, goalX);
    const maxX = Math.max(startX, goalX);
    const minY = Math.min(startY, goalY);
    const maxY = Math.max(startY, goalY);

    return this.searchBounded(
      start as TileRef,
      goal as TileRef,
      { minX, maxX, minY, maxY },
    );
  }

  searchBounded(
    start: TileRef,
    goal: TileRef,
    bounds: SearchBounds,
  ): TileRef[] | null {
    // Advance stamp (handles overflow)
    this.stamp++;
    if (this.stamp === 0) {
      this.closedStamp.fill(0);
      this.gScoreStamp.fill(0);
      this.stamp = 1;
    }

    const stamp = this.stamp;
    const mapWidth = this.mapWidth;
    const terrain = this.terrain;
    const closedStamp = this.closedStamp;
    const gScoreStamp = this.gScoreStamp;
    const gScore = this.gScore;
    const cameFrom = this.cameFrom;
    const queue = this.queue;
    const weight = this.heuristicWeight;
    const landMask = 1 << LAND_BIT;

    const { minX, maxX, minY, maxY } = bounds;
    const boundsWidth = maxX - minX + 1;

    // Goal coordinates for heuristic
    const goalX = goal % mapWidth;
    const goalY = (goal / mapWidth) | 0;

    const boundsHeight = maxY - minY + 1;
    const numLocalNodes = boundsWidth * boundsHeight;

    // Validate bounds don't exceed allocated array size
    if (numLocalNodes > this.closedStamp.length) {
      return null; // Search area exceeds allocated buffer
    }

    // Convert global tile to local index, clamping to bounds
    const toLocal = (tile: TileRef, clamp: boolean = false): number => {
      let x = tile % mapWidth;
      let y = (tile / mapWidth) | 0;

      if (clamp) {
        // Clamp to bounds (for start/goal that may be slightly outside)
        x = Math.max(minX, Math.min(maxX, x));
        y = Math.max(minY, Math.min(maxY, y));
      }

      return (y - minY) * boundsWidth + (x - minX);
    };

    // Convert local index back to global tile
    const toGlobal = (local: number): TileRef => {
      const localX = local % boundsWidth;
      const localY = (local / boundsWidth) | 0;
      return ((localY + minY) * mapWidth + (localX + minX)) as TileRef;
    };

    // Clamp start and goal to bounds (they may be slightly outside)
    const startLocal = toLocal(start, true);
    const goalLocal = toLocal(goal, true);

    // Safety check: ensure local indices are valid
    if (
      startLocal < 0 ||
      startLocal >= numLocalNodes ||
      goalLocal < 0 ||
      goalLocal >= numLocalNodes
    ) {
      return null; // Start or goal still outside after clamping (shouldn't happen)
    }

    // Initialize
    queue.clear();
    gScore[startLocal] = 0;
    gScoreStamp[startLocal] = stamp;
    cameFrom[startLocal] = -1;

    const startX = start % mapWidth;
    const startY = (start / mapWidth) | 0;
    const startH = weight * (Math.abs(startX - goalX) + Math.abs(startY - goalY));
    queue.push(startLocal, startH);

    let iterations = this.maxIterations;

    while (!queue.isEmpty()) {
      if (--iterations <= 0) {
        return null;
      }

      const currentLocal = queue.pop();

      if (closedStamp[currentLocal] === stamp) continue;
      closedStamp[currentLocal] = stamp;

      if (currentLocal === goalLocal) {
        return this.buildPath(startLocal, goalLocal, toGlobal, numLocalNodes);
      }

      const currentG = gScore[currentLocal];
      const tentativeG = currentG + 1;

      // Convert to global coords for neighbor calculation
      const current = toGlobal(currentLocal);
      const currentX = current % mapWidth;
      const currentY = (current / mapWidth) | 0;

      // Up
      if (currentY > minY) {
        const neighbor = current - mapWidth;
        const neighborLocal = currentLocal - boundsWidth;
        if (
          closedStamp[neighborLocal] !== stamp &&
          (neighbor === goal || (terrain[neighbor] & landMask) === 0)
        ) {
          if (
            gScoreStamp[neighborLocal] !== stamp ||
            tentativeG < gScore[neighborLocal]
          ) {
            cameFrom[neighborLocal] = currentLocal;
            gScore[neighborLocal] = tentativeG;
            gScoreStamp[neighborLocal] = stamp;
            const f =
              tentativeG +
              weight * (Math.abs(currentX - goalX) + Math.abs(currentY - 1 - goalY));
            queue.push(neighborLocal, f);
          }
        }
      }

      // Down
      if (currentY < maxY) {
        const neighbor = current + mapWidth;
        const neighborLocal = currentLocal + boundsWidth;
        if (
          closedStamp[neighborLocal] !== stamp &&
          (neighbor === goal || (terrain[neighbor] & landMask) === 0)
        ) {
          if (
            gScoreStamp[neighborLocal] !== stamp ||
            tentativeG < gScore[neighborLocal]
          ) {
            cameFrom[neighborLocal] = currentLocal;
            gScore[neighborLocal] = tentativeG;
            gScoreStamp[neighborLocal] = stamp;
            const f =
              tentativeG +
              weight * (Math.abs(currentX - goalX) + Math.abs(currentY + 1 - goalY));
            queue.push(neighborLocal, f);
          }
        }
      }

      // Left
      if (currentX > minX) {
        const neighbor = current - 1;
        const neighborLocal = currentLocal - 1;
        if (
          closedStamp[neighborLocal] !== stamp &&
          (neighbor === goal || (terrain[neighbor] & landMask) === 0)
        ) {
          if (
            gScoreStamp[neighborLocal] !== stamp ||
            tentativeG < gScore[neighborLocal]
          ) {
            cameFrom[neighborLocal] = currentLocal;
            gScore[neighborLocal] = tentativeG;
            gScoreStamp[neighborLocal] = stamp;
            const f =
              tentativeG +
              weight * (Math.abs(currentX - 1 - goalX) + Math.abs(currentY - goalY));
            queue.push(neighborLocal, f);
          }
        }
      }

      // Right
      if (currentX < maxX) {
        const neighbor = current + 1;
        const neighborLocal = currentLocal + 1;
        if (
          closedStamp[neighborLocal] !== stamp &&
          (neighbor === goal || (terrain[neighbor] & landMask) === 0)
        ) {
          if (
            gScoreStamp[neighborLocal] !== stamp ||
            tentativeG < gScore[neighborLocal]
          ) {
            cameFrom[neighborLocal] = currentLocal;
            gScore[neighborLocal] = tentativeG;
            gScoreStamp[neighborLocal] = stamp;
            const f =
              tentativeG +
              weight * (Math.abs(currentX + 1 - goalX) + Math.abs(currentY - goalY));
            queue.push(neighborLocal, f);
          }
        }
      }
    }

    return null;
  }

  private buildPath(
    startLocal: number,
    goalLocal: number,
    toGlobal: (local: number) => TileRef,
    maxPathLength: number,
  ): TileRef[] {
    const path: TileRef[] = [];
    let current = goalLocal;

    // Safety check to prevent infinite loops
    let iterations = 0;
    while (current !== -1 && iterations < maxPathLength) {
      path.push(toGlobal(current));
      if (current === startLocal) {
        // Reached start, we're done
        break;
      }
      current = this.cameFrom[current];
      iterations++;
    }

    path.reverse();
    return path;
  }
}

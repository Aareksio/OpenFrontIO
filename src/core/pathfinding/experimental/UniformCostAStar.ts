// Bucket queue A* with uniform cost and direct terrain access
// Optimized for naval pathfinding where all water tiles have cost = 1

import { AStar, PathFindResultType } from "../AStar";
import { BucketQueue } from "./BucketQueue";

const LAND_BIT = 7; // Bit 7 in terrain indicates land

export class UniformCostAStar implements AStar<number> {
  private stamp = 1;
  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Uint32Array; // Integer g-scores with uniform cost
  private readonly cameFrom: Int32Array;
  private readonly openQueue: BucketQueue;
  private readonly terrain: Uint8Array;

  private sources: number[] = [];
  private dst: number = 0;
  private dstX: number = 0;
  private dstY: number = 0;
  private width: number;
  private numNodes: number;
  private iterations: number;
  private maxTries: number;
  private initialMaxTries: number;

  public completed = false;
  private foundPath: number[] | null = null;

  constructor(
    terrain: Uint8Array,
    numNodes: number,
    width: number,
    iterations: number = 500_000,
    maxTries: number = 50,
  ) {
    this.terrain = terrain;
    this.width = width;
    this.numNodes = numNodes;
    this.iterations = iterations;
    this.maxTries = maxTries;
    this.initialMaxTries = maxTries;

    this.closedStamp = new Uint32Array(numNodes);
    this.gScoreStamp = new Uint32Array(numNodes);
    this.gScore = new Uint32Array(numNodes);
    this.cameFrom = new Int32Array(numNodes);

    // Max f = 15 * (width + height) with uniform cost
    const height = Math.ceil(numNodes / width);
    const maxF = 15 * (width + height);
    this.openQueue = new BucketQueue(maxF);
  }

  reset(src: number | number[], dst: number): void {
    this.sources = Array.isArray(src) ? src : [src];
    this.dst = dst;
    this.dstX = dst % this.width;
    this.dstY = (dst / this.width) | 0;
    this.completed = false;
    this.foundPath = null;
    this.maxTries = this.initialMaxTries;

    this.stamp++;
    if (this.stamp === 0) {
      this.closedStamp.fill(0);
      this.gScoreStamp.fill(0);
      this.stamp = 1;
    }

    this.openQueue.clear();
    this.initSearch();
  }

  private initSearch(): void {
    const stamp = this.stamp;

    for (const start of this.sources) {
      this.gScore[start] = 0;
      this.gScoreStamp[start] = stamp;
      this.cameFrom[start] = -1;
      this.openQueue.push(start, this.heuristic(start));
    }
  }

  private heuristic(node: number): number {
    const x = node % this.width;
    const y = (node / this.width) | 0;
    return 15 * (Math.abs(x - this.dstX) + Math.abs(y - this.dstY));
  }

  // Inline water check - bit 7 = 0 means water
  private isWater(node: number): boolean {
    return (this.terrain[node] & (1 << LAND_BIT)) === 0;
  }

  compute(): PathFindResultType {
    if (this.completed) return PathFindResultType.Completed;

    this.maxTries--;
    let iterations = this.iterations;
    const stamp = this.stamp;
    const width = this.width;
    const numNodes = this.numNodes;
    const dst = this.dst;
    const terrain = this.terrain;
    const closedStamp = this.closedStamp;
    const gScoreStamp = this.gScoreStamp;
    const gScore = this.gScore;
    const cameFrom = this.cameFrom;
    const openQueue = this.openQueue;
    const dstX = this.dstX;
    const dstY = this.dstY;
    const landMask = 1 << LAND_BIT;

    while (!openQueue.isEmpty()) {
      iterations--;
      if (iterations <= 0) {
        if (this.maxTries <= 0) {
          return PathFindResultType.PathNotFound;
        }
        return PathFindResultType.Pending;
      }

      const current = openQueue.pop();

      if (closedStamp[current] === stamp) continue;
      closedStamp[current] = stamp;

      if (current === dst) {
        this.completed = true;
        this.foundPath = this.buildPath(current);
        return PathFindResultType.Completed;
      }

      const currentG = gScore[current];
      const tentativeG = currentG + 1; // Uniform cost = 1
      const currentX = current % width;

      // Up
      if (current >= width) {
        const neighbor = current - width;
        if (
          closedStamp[neighbor] !== stamp &&
          (neighbor === dst || (terrain[neighbor] & landMask) === 0)
        ) {
          if (gScoreStamp[neighbor] !== stamp || tentativeG < gScore[neighbor]) {
            cameFrom[neighbor] = current;
            gScore[neighbor] = tentativeG;
            gScoreStamp[neighbor] = stamp;
            const nx = neighbor % width;
            const ny = (neighbor / width) | 0;
            const f = tentativeG + 15 * (Math.abs(nx - dstX) + Math.abs(ny - dstY));
            openQueue.push(neighbor, f);
          }
        }
      }

      // Down
      if (current < numNodes - width) {
        const neighbor = current + width;
        if (
          closedStamp[neighbor] !== stamp &&
          (neighbor === dst || (terrain[neighbor] & landMask) === 0)
        ) {
          if (gScoreStamp[neighbor] !== stamp || tentativeG < gScore[neighbor]) {
            cameFrom[neighbor] = current;
            gScore[neighbor] = tentativeG;
            gScoreStamp[neighbor] = stamp;
            const nx = neighbor % width;
            const ny = (neighbor / width) | 0;
            const f = tentativeG + 15 * (Math.abs(nx - dstX) + Math.abs(ny - dstY));
            openQueue.push(neighbor, f);
          }
        }
      }

      // Left
      if (currentX !== 0) {
        const neighbor = current - 1;
        if (
          closedStamp[neighbor] !== stamp &&
          (neighbor === dst || (terrain[neighbor] & landMask) === 0)
        ) {
          if (gScoreStamp[neighbor] !== stamp || tentativeG < gScore[neighbor]) {
            cameFrom[neighbor] = current;
            gScore[neighbor] = tentativeG;
            gScoreStamp[neighbor] = stamp;
            const ny = (neighbor / width) | 0;
            const f = tentativeG + 15 * (Math.abs(currentX - 1 - dstX) + Math.abs(ny - dstY));
            openQueue.push(neighbor, f);
          }
        }
      }

      // Right
      if (currentX !== width - 1) {
        const neighbor = current + 1;
        if (
          closedStamp[neighbor] !== stamp &&
          (neighbor === dst || (terrain[neighbor] & landMask) === 0)
        ) {
          if (gScoreStamp[neighbor] !== stamp || tentativeG < gScore[neighbor]) {
            cameFrom[neighbor] = current;
            gScore[neighbor] = tentativeG;
            gScoreStamp[neighbor] = stamp;
            const ny = (neighbor / width) | 0;
            const f = tentativeG + 15 * (Math.abs(currentX + 1 - dstX) + Math.abs(ny - dstY));
            openQueue.push(neighbor, f);
          }
        }
      }
    }

    return PathFindResultType.PathNotFound;
  }

  private buildPath(goal: number): number[] {
    const path: number[] = [];
    let current = goal;

    while (current !== -1) {
      path.push(current);
      current = this.cameFrom[current];
    }

    path.reverse();
    return path;
  }

  reconstructPath(): number[] {
    return this.foundPath ?? [];
  }
}

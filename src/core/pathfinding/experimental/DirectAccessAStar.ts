// Bucket queue A* with direct terrain access but VARIABLE cost (magnitude-based)
// Tests the effect of direct access alone without uniform cost

import { AStar, PathFindResultType } from "../AStar";
import { BucketQueue } from "./BucketQueue";

const LAND_BIT = 7;
const MAGNITUDE_MASK = 0x7f; // Lower 7 bits

export class DirectAccessAStar implements AStar<number> {
  private stamp = 1;
  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Float32Array;
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
    this.gScore = new Float32Array(numNodes);
    this.cameFrom = new Int32Array(numNodes);

    const height = Math.ceil(numNodes / width);
    const maxF = 15 * (width + height) * 2;
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

  // Variable cost based on magnitude (same as GameMap.cost)
  private cost(node: number): number {
    return (this.terrain[node] & MAGNITUDE_MASK) < 10 ? 2 : 1;
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
    const magMask = MAGNITUDE_MASK;

    while (!openQueue.isEmpty()) {
      iterations--;
      if (iterations <= 0) {
        if (this.maxTries <= 0) return PathFindResultType.PathNotFound;
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
      const currentX = current % width;

      // Process neighbors with variable cost
      const neighbors = [
        current >= width ? current - width : -1,
        current < numNodes - width ? current + width : -1,
        currentX !== 0 ? current - 1 : -1,
        currentX !== width - 1 ? current + 1 : -1,
      ];

      for (const neighbor of neighbors) {
        if (neighbor === -1) continue;
        if (closedStamp[neighbor] === stamp) continue;
        if (neighbor !== dst && (terrain[neighbor] & landMask) !== 0) continue;

        const cost = (terrain[neighbor] & magMask) < 10 ? 2 : 1;
        const tentativeG = currentG + cost;

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

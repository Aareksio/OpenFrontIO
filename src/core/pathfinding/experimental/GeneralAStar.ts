// General A* with bucket queue, stamp-based tracking, and reusable buffer
// Works with any graph through adapter interface

import { BucketQueue, PriorityQueue } from "./PriorityQueue";

export interface GeneralAStarAdapter {
  neighbors(node: number, buffer: Int32Array): number; // returns count
  cost(from: number, to: number): number;
  heuristic(node: number, goal: number): number;
}

export interface GeneralAStarConfig {
  adapter: GeneralAStarAdapter;
  numNodes: number;
  maxPriority: number;
  maxNeighbors?: number; // Default: 4 for grid
  maxIterations?: number; // Default: 500_000
}

export class GeneralAStar {
  private stamp = 1;
  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Uint32Array;
  private readonly cameFrom: Int32Array;
  private readonly queue: PriorityQueue;
  private readonly adapter: GeneralAStarAdapter;
  private readonly neighborBuffer: Int32Array;
  private readonly maxIterations: number;

  constructor(config: GeneralAStarConfig) {
    this.adapter = config.adapter;
    this.maxIterations = config.maxIterations ?? 500_000;
    this.neighborBuffer = new Int32Array(config.maxNeighbors ?? 4);

    this.closedStamp = new Uint32Array(config.numNodes);
    this.gScoreStamp = new Uint32Array(config.numNodes);
    this.gScore = new Uint32Array(config.numNodes);
    this.cameFrom = new Int32Array(config.numNodes);
    this.queue = new BucketQueue(config.maxPriority);
  }

  search(start: number, goal: number): number[] | null {
    // Advance stamp (handles overflow)
    this.stamp++;
    if (this.stamp === 0) {
      this.closedStamp.fill(0);
      this.gScoreStamp.fill(0);
      this.stamp = 1;
    }

    const stamp = this.stamp;
    const adapter = this.adapter;
    const closedStamp = this.closedStamp;
    const gScoreStamp = this.gScoreStamp;
    const gScore = this.gScore;
    const cameFrom = this.cameFrom;
    const queue = this.queue;
    const buffer = this.neighborBuffer;

    // Initialize
    queue.clear();
    gScore[start] = 0;
    gScoreStamp[start] = stamp;
    cameFrom[start] = -1;

    queue.push(start, adapter.heuristic(start, goal));

    let iterations = this.maxIterations;

    while (!queue.isEmpty()) {
      if (--iterations <= 0) {
        return null; // Iteration limit reached
      }

      const current = queue.pop();

      if (closedStamp[current] === stamp) continue;
      closedStamp[current] = stamp;

      if (current === goal) {
        return this.buildPath(goal);
      }

      const currentG = gScore[current];
      const count = adapter.neighbors(current, buffer);

      for (let i = 0; i < count; i++) {
        const neighbor = buffer[i];

        if (closedStamp[neighbor] === stamp) continue;

        const tentativeG = currentG + adapter.cost(current, neighbor);

        if (gScoreStamp[neighbor] !== stamp || tentativeG < gScore[neighbor]) {
          cameFrom[neighbor] = current;
          gScore[neighbor] = tentativeG;
          gScoreStamp[neighbor] = stamp;
          const f = tentativeG + adapter.heuristic(neighbor, goal);
          queue.push(neighbor, f);
        }
      }
    }

    return null; // No path found
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
}

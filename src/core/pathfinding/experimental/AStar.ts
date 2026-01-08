// Generic A* implementation with adapter interface
// Use when performance is not critical, inline otherwise
// See GridAStar.ts for fully inlined map grid A* implementation

import { GameMap } from "../../game/GameMap";
import { BucketQueue, PriorityQueue } from "./PriorityQueue";

export interface AStar {
  search(from: number, to: number): number[] | null;
}

export interface GenericAStarAdapter {
  // Important optimization: write to the buffer and return the count
  // You can do this and it will be much faster :)
  neighbors(node: number, buffer: Int32Array): number;

  cost(from: number, to: number): number;
  heuristic(node: number, goal: number): number;
  numNodes(): number;
  maxPriority(): number;
  maxNeighbors(): number;
}

export interface GenericAStarConfig {
  adapter: GenericAStarAdapter;
  maxIterations?: number;
}

export class GenericAStar implements AStar {
  private stamp = 1;

  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Uint32Array;
  private readonly cameFrom: Int32Array;
  private readonly queue: PriorityQueue;
  private readonly adapter: GenericAStarAdapter;
  private readonly neighborBuffer: Int32Array;
  private readonly maxIterations: number;

  constructor(config: GenericAStarConfig) {
    this.adapter = config.adapter;
    this.maxIterations = config.maxIterations ?? 500_000;
    this.neighborBuffer = new Int32Array(this.adapter.maxNeighbors());
    this.closedStamp = new Uint32Array(this.adapter.numNodes());
    this.gScoreStamp = new Uint32Array(this.adapter.numNodes());
    this.gScore = new Uint32Array(this.adapter.numNodes());
    this.cameFrom = new Int32Array(this.adapter.numNodes());
    this.queue = new BucketQueue(this.adapter.maxPriority());
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
        return null; // Iteration limit
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
          queue.push(neighbor, tentativeG + adapter.heuristic(neighbor, goal));
        }
      }
    }

    return null; // No path
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

export class WaterGridAdapter implements GenericAStarAdapter {
  private readonly gameMap: GameMap;
  private readonly height: number;
  private readonly width: number;
  private readonly _numNodes: number;

  constructor(
    gameMap: GameMap,
    private readonly heuristicWeight: number = 15,
  ) {
    this.gameMap = gameMap;
    this.width = gameMap.width();
    this.height = gameMap.height();
    this._numNodes = this.width * this.height;
  }

  numNodes(): number {
    return this._numNodes;
  }

  maxNeighbors(): number {
    return 4;
  }

  maxPriority(): number {
    return this.heuristicWeight * (this.width + this.height);
  }

  neighbors(node: number, buffer: Int32Array): number {
    let count = 0;
    const x = node % this.width;

    // Up
    if (node >= this.width) {
      const n = node - this.width;
      if (this.gameMap.isWater(n)) buffer[count++] = n;
    }

    // Down
    if (node < this._numNodes - this.width) {
      const n = node + this.width;
      if (this.gameMap.isWater(n)) buffer[count++] = n;
    }

    // Left
    if (x !== 0) {
      const n = node - 1;
      if (this.gameMap.isWater(n)) buffer[count++] = n;
    }

    // Right
    if (x !== this.width - 1) {
      const n = node + 1;
      if (this.gameMap.isWater(n)) buffer[count++] = n;
    }

    return count;
  }

  cost(_from: number, _to: number): number {
    return 1;
  }

  heuristic(node: number, goal: number): number {
    const nx = node % this.width;
    const ny = (node / this.width) | 0;
    const gx = goal % this.width;
    const gy = (goal / this.width) | 0;
    return this.heuristicWeight * (Math.abs(nx - gx) + Math.abs(ny - gy));
  }
}

export interface GridGenericAStarConfig {
  maxIterations?: number;
  heuristicWeight?: number;
}

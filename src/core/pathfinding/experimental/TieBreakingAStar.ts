// OptimizedAStar with tie-breaking: prefer higher g (closer to goal) when f equal

import { AStar, PathFindResultType } from "../AStar";
import { GraphAdapter } from "../SerialAStar";

// MinHeap with tie-breaking by g-score (prefer higher g)
class TieBreakingHeap {
  private heap: Int32Array;
  private fScores: Float32Array;
  private gScores: Float32Array;
  private size = 0;

  constructor(
    capacity: number,
    fScores: Float32Array,
    gScores: Float32Array,
  ) {
    this.heap = new Int32Array(capacity);
    this.fScores = fScores;
    this.gScores = gScores;
  }

  // Returns true if a is better than b (lower f, or equal f with higher g)
  private isBetter(a: number, b: number): boolean {
    const fA = this.fScores[a];
    const fB = this.fScores[b];
    if (fA !== fB) return fA < fB;
    return this.gScores[a] > this.gScores[b]; // Prefer higher g
  }

  push(node: number): void {
    let i = this.size++;
    this.heap[i] = node;

    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (!this.isBetter(this.heap[i], this.heap[parent])) break;
      const tmp = this.heap[parent];
      this.heap[parent] = this.heap[i];
      this.heap[i] = tmp;
      i = parent;
    }
  }

  pop(): number {
    const result = this.heap[0];
    this.heap[0] = this.heap[--this.size];

    let i = 0;
    while (true) {
      const left = (i << 1) + 1;
      const right = left + 1;
      let best = i;

      if (left < this.size && this.isBetter(this.heap[left], this.heap[best])) {
        best = left;
      }
      if (
        right < this.size &&
        this.isBetter(this.heap[right], this.heap[best])
      ) {
        best = right;
      }
      if (best === i) break;

      const tmp = this.heap[best];
      this.heap[best] = this.heap[i];
      this.heap[i] = tmp;
      i = best;
    }

    return result;
  }

  isEmpty(): boolean {
    return this.size === 0;
  }

  clear(): void {
    this.size = 0;
  }
}

export class TieBreakingAStar implements AStar<number> {
  private stamp = 1;
  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Float32Array;
  private readonly fScore: Float32Array;
  private readonly cameFrom: Int32Array;
  private readonly openHeap: TieBreakingHeap;

  private sources: number[] = [];
  private dst: number = 0;
  private dstX: number = 0;
  private dstY: number = 0;
  private graph: GraphAdapter<number>;
  private width: number;
  private numNodes: number;
  private iterations: number;
  private maxTries: number;
  private initialMaxTries: number;

  public completed = false;
  private foundPath: number[] | null = null;

  constructor(
    graph: GraphAdapter<number>,
    numNodes: number,
    width: number,
    iterations: number = 500_000,
    maxTries: number = 50,
  ) {
    this.graph = graph;
    this.width = width;
    this.numNodes = numNodes;
    this.iterations = iterations;
    this.maxTries = maxTries;
    this.initialMaxTries = maxTries;

    this.closedStamp = new Uint32Array(numNodes);
    this.gScoreStamp = new Uint32Array(numNodes);
    this.gScore = new Float32Array(numNodes);
    this.fScore = new Float32Array(numNodes);
    this.cameFrom = new Int32Array(numNodes);
    this.openHeap = new TieBreakingHeap(numNodes, this.fScore, this.gScore);
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

    this.openHeap.clear();
    this.initSearch();
  }

  private initSearch(): void {
    const stamp = this.stamp;

    for (const start of this.sources) {
      this.gScore[start] = 0;
      this.gScoreStamp[start] = stamp;
      this.fScore[start] = this.heuristic(start);
      this.cameFrom[start] = -1;
      this.openHeap.push(start);
    }
  }

  private heuristic(node: number): number {
    const x = node % this.width;
    const y = (node / this.width) | 0;
    return 15 * (Math.abs(x - this.dstX) + Math.abs(y - this.dstY));
  }

  compute(): PathFindResultType {
    if (this.completed) return PathFindResultType.Completed;

    this.maxTries--;
    let iterations = this.iterations;
    const stamp = this.stamp;
    const width = this.width;
    const numNodes = this.numNodes;
    const dst = this.dst;

    while (!this.openHeap.isEmpty()) {
      iterations--;
      if (iterations <= 0) {
        if (this.maxTries <= 0) {
          return PathFindResultType.PathNotFound;
        }
        return PathFindResultType.Pending;
      }

      const current = this.openHeap.pop();

      if (this.closedStamp[current] === stamp) continue;
      this.closedStamp[current] = stamp;

      if (current === dst) {
        this.completed = true;
        this.foundPath = this.buildPath(current);
        return PathFindResultType.Completed;
      }

      const currentGScore = this.gScore[current];
      const currentX = current % width;

      if (current >= width) {
        this.processNeighbor(current - width, current, currentGScore, stamp, dst);
      }
      if (current < numNodes - width) {
        this.processNeighbor(current + width, current, currentGScore, stamp, dst);
      }
      if (currentX !== 0) {
        this.processNeighbor(current - 1, current, currentGScore, stamp, dst);
      }
      if (currentX !== width - 1) {
        this.processNeighbor(current + 1, current, currentGScore, stamp, dst);
      }
    }

    return PathFindResultType.PathNotFound;
  }

  private processNeighbor(
    neighbor: number,
    current: number,
    currentGScore: number,
    stamp: number,
    dst: number,
  ): void {
    if (neighbor !== dst && !this.graph.isTraversable(current, neighbor)) {
      return;
    }

    if (this.closedStamp[neighbor] === stamp) return;

    const tentativeGScore = currentGScore + this.graph.cost(neighbor);
    const hasValidGScore = this.gScoreStamp[neighbor] === stamp;

    if (!hasValidGScore || tentativeGScore < this.gScore[neighbor]) {
      this.cameFrom[neighbor] = current;
      this.gScore[neighbor] = tentativeGScore;
      this.gScoreStamp[neighbor] = stamp;
      this.fScore[neighbor] = tentativeGScore + this.heuristic(neighbor);
      this.openHeap.push(neighbor);
    }
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

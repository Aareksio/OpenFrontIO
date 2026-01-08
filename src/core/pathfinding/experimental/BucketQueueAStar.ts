// OptimizedAStar with bucket queue for O(1) priority queue operations
// Works because f-values are integers (weight=15 * Manhattan distance)

import { AStar, PathFindResultType } from "../AStar";
import { GraphAdapter } from "../SerialAStar";
import { BucketQueue } from "./BucketQueue";

export class BucketQueueAStar implements AStar<number> {
  private stamp = 1;
  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Float32Array;
  private readonly fScore: Float32Array;
  private readonly cameFrom: Int32Array;
  private readonly openQueue: BucketQueue;

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

    // Max f = 15 * (width + height) * 2 roughly
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
      const f = this.heuristic(start);
      this.fScore[start] = f;
      this.cameFrom[start] = -1;
      this.openQueue.push(start, f);
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

    while (!this.openQueue.isEmpty()) {
      iterations--;
      if (iterations <= 0) {
        if (this.maxTries <= 0) {
          return PathFindResultType.PathNotFound;
        }
        return PathFindResultType.Pending;
      }

      const current = this.openQueue.pop();

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
      const f = tentativeGScore + this.heuristic(neighbor);
      this.fScore[neighbor] = f;
      this.openQueue.push(neighbor, f);
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

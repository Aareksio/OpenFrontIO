// Bucket queue A* with uniform cost=1 but using GraphAdapter abstraction
// Tests effect of uniform cost alone without direct terrain access

import { AStar, PathFindResultType } from "../AStar";
import { GraphAdapter } from "../SerialAStar";

class BucketQueue {
  private buckets: Int32Array[];
  private bucketSizes: Int32Array;
  private minBucket: number;
  private maxBucket: number;
  private size: number;

  constructor(maxF: number) {
    this.maxBucket = maxF + 1;
    this.buckets = new Array(this.maxBucket);
    this.bucketSizes = new Int32Array(this.maxBucket);
    this.minBucket = this.maxBucket;
    this.size = 0;
  }

  push(node: number, f: number): void {
    const bucket = Math.min(f | 0, this.maxBucket - 1);
    if (!this.buckets[bucket]) this.buckets[bucket] = new Int32Array(64);
    const size = this.bucketSizes[bucket];
    if (size >= this.buckets[bucket].length) {
      const newBucket = new Int32Array(this.buckets[bucket].length * 2);
      newBucket.set(this.buckets[bucket]);
      this.buckets[bucket] = newBucket;
    }
    this.buckets[bucket][size] = node;
    this.bucketSizes[bucket]++;
    this.size++;
    if (bucket < this.minBucket) this.minBucket = bucket;
  }

  pop(): number {
    while (this.minBucket < this.maxBucket) {
      const size = this.bucketSizes[this.minBucket];
      if (size > 0) {
        this.bucketSizes[this.minBucket]--;
        this.size--;
        return this.buckets[this.minBucket][size - 1];
      }
      this.minBucket++;
    }
    return -1;
  }

  isEmpty(): boolean { return this.size === 0; }
  clear(): void {
    this.bucketSizes.fill(0);
    this.minBucket = this.maxBucket;
    this.size = 0;
  }
}

export class BucketUniformAStar implements AStar<number> {
  private stamp = 1;
  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Uint32Array;
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
    this.gScore = new Uint32Array(numNodes);
    this.cameFrom = new Int32Array(numNodes);

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
    for (const start of this.sources) {
      this.gScore[start] = 0;
      this.gScoreStamp[start] = this.stamp;
      this.cameFrom[start] = -1;
      this.openQueue.push(start, this.heuristic(start));
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
        if (this.maxTries <= 0) return PathFindResultType.PathNotFound;
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

      const currentG = this.gScore[current];
      const tentativeG = currentG + 1; // Uniform cost = 1
      const currentX = current % width;

      // Up
      if (current >= width) {
        this.processNeighbor(current - width, current, tentativeG, stamp, dst);
      }
      // Down
      if (current < numNodes - width) {
        this.processNeighbor(current + width, current, tentativeG, stamp, dst);
      }
      // Left
      if (currentX !== 0) {
        this.processNeighbor(current - 1, current, tentativeG, stamp, dst);
      }
      // Right
      if (currentX !== width - 1) {
        this.processNeighbor(current + 1, current, tentativeG, stamp, dst);
      }
    }

    return PathFindResultType.PathNotFound;
  }

  private processNeighbor(
    neighbor: number,
    current: number,
    tentativeG: number,
    stamp: number,
    dst: number,
  ): void {
    if (neighbor !== dst && !this.graph.isTraversable(current, neighbor)) return;
    if (this.closedStamp[neighbor] === stamp) return;

    if (this.gScoreStamp[neighbor] !== stamp || tentativeG < this.gScore[neighbor]) {
      this.cameFrom[neighbor] = current;
      this.gScore[neighbor] = tentativeG;
      this.gScoreStamp[neighbor] = stamp;
      const f = tentativeG + this.heuristic(neighbor);
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

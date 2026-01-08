// Optimized Bidirectional A* using typed arrays and stamp-based tracking
// Searches from both start and goal, meeting in the middle

import { AStar, PathFindResultType } from "../AStar";
import { GraphAdapter } from "../SerialAStar";
import { MinHeap } from "./MinHeap";

export class BidirectionalAStar implements AStar<number> {
  // Forward search state
  private fwdStamp = 0;
  private readonly fwdClosedStamp: Uint32Array;
  private readonly fwdGScoreStamp: Uint32Array;
  private readonly fwdGScore: Float32Array;
  private readonly fwdFScore: Float32Array;
  private readonly fwdCameFrom: Int32Array;
  private readonly fwdOpenHeap: MinHeap;

  // Backward search state
  private bwdStamp = 0;
  private readonly bwdClosedStamp: Uint32Array;
  private readonly bwdGScoreStamp: Uint32Array;
  private readonly bwdGScore: Float32Array;
  private readonly bwdFScore: Float32Array;
  private readonly bwdCameFrom: Int32Array;
  private readonly bwdOpenHeap: MinHeap;

  // Shared state
  private stamp = 1;
  private sources: number[] = [];
  private dst: number = 0;
  private dstX: number = 0;
  private dstY: number = 0;
  private srcX: number = 0;
  private srcY: number = 0;
  private graph: GraphAdapter<number>;
  private width: number;
  private numNodes: number;
  private iterations: number;
  private maxTries: number;
  private initialMaxTries: number;

  public completed = false;
  private meetingPoint: number = -1;
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

    // Forward search arrays
    this.fwdClosedStamp = new Uint32Array(numNodes);
    this.fwdGScoreStamp = new Uint32Array(numNodes);
    this.fwdGScore = new Float32Array(numNodes);
    this.fwdFScore = new Float32Array(numNodes);
    this.fwdCameFrom = new Int32Array(numNodes);
    this.fwdOpenHeap = new MinHeap(numNodes, this.fwdFScore);

    // Backward search arrays
    this.bwdClosedStamp = new Uint32Array(numNodes);
    this.bwdGScoreStamp = new Uint32Array(numNodes);
    this.bwdGScore = new Float32Array(numNodes);
    this.bwdFScore = new Float32Array(numNodes);
    this.bwdCameFrom = new Int32Array(numNodes);
    this.bwdOpenHeap = new MinHeap(numNodes, this.bwdFScore);
  }

  reset(src: number | number[], dst: number): void {
    this.sources = Array.isArray(src) ? src : [src];
    this.dst = dst;
    this.dstX = dst % this.width;
    this.dstY = (dst / this.width) | 0;

    // Use first source for backward heuristic target
    const primarySrc = this.sources[0];
    this.srcX = primarySrc % this.width;
    this.srcY = (primarySrc / this.width) | 0;

    this.completed = false;
    this.meetingPoint = -1;
    this.foundPath = null;
    this.maxTries = this.initialMaxTries;

    // Increment stamp to invalidate previous search data
    this.stamp++;
    if (this.stamp === 0) {
      this.fwdClosedStamp.fill(0);
      this.fwdGScoreStamp.fill(0);
      this.bwdClosedStamp.fill(0);
      this.bwdGScoreStamp.fill(0);
      this.stamp = 1;
    }

    this.fwdStamp = this.stamp;
    this.bwdStamp = this.stamp;

    this.fwdOpenHeap.clear();
    this.bwdOpenHeap.clear();
    this.initSearch();
  }

  private initSearch(): void {
    // Initialize forward search from source(s)
    for (const start of this.sources) {
      this.fwdGScore[start] = 0;
      this.fwdGScoreStamp[start] = this.fwdStamp;
      this.fwdFScore[start] = this.heuristicForward(start);
      this.fwdCameFrom[start] = -1;
      this.fwdOpenHeap.push(start);
    }

    // Initialize backward search from destination
    this.bwdGScore[this.dst] = 0;
    this.bwdGScoreStamp[this.dst] = this.bwdStamp;
    this.bwdFScore[this.dst] = this.heuristicBackward(this.dst);
    this.bwdCameFrom[this.dst] = -1;
    this.bwdOpenHeap.push(this.dst);
  }

  private heuristicForward(node: number): number {
    const x = node % this.width;
    const y = (node / this.width) | 0;
    return 15 * (Math.abs(x - this.dstX) + Math.abs(y - this.dstY));
  }

  private heuristicBackward(node: number): number {
    const x = node % this.width;
    const y = (node / this.width) | 0;
    return 15 * (Math.abs(x - this.srcX) + Math.abs(y - this.srcY));
  }

  compute(): PathFindResultType {
    if (this.completed) return PathFindResultType.Completed;

    this.maxTries--;
    let iterations = this.iterations;
    const width = this.width;
    const numNodes = this.numNodes;
    const dst = this.dst;

    while (!this.fwdOpenHeap.isEmpty() && !this.bwdOpenHeap.isEmpty()) {
      iterations--;
      if (iterations <= 0) {
        if (this.maxTries <= 0) {
          return PathFindResultType.PathNotFound;
        }
        return PathFindResultType.Pending;
      }

      // Expand forward search
      const fwdCurrent = this.fwdOpenHeap.pop();

      // Skip if already processed
      if (this.fwdClosedStamp[fwdCurrent] === this.fwdStamp) continue;
      this.fwdClosedStamp[fwdCurrent] = this.fwdStamp;

      // Check if backward search has visited this node
      if (this.bwdClosedStamp[fwdCurrent] === this.bwdStamp) {
        this.meetingPoint = fwdCurrent;
        this.completed = true;
        this.foundPath = this.buildPath();
        return PathFindResultType.Completed;
      }

      this.expandForward(fwdCurrent, width, numNodes, dst);

      // Check if backward heap is empty after forward expansion
      if (this.bwdOpenHeap.isEmpty()) break;

      iterations--;
      if (iterations <= 0) {
        if (this.maxTries <= 0) {
          return PathFindResultType.PathNotFound;
        }
        return PathFindResultType.Pending;
      }

      // Expand backward search
      const bwdCurrent = this.bwdOpenHeap.pop();

      // Skip if already processed
      if (this.bwdClosedStamp[bwdCurrent] === this.bwdStamp) continue;
      this.bwdClosedStamp[bwdCurrent] = this.bwdStamp;

      // Check if forward search has visited this node
      if (this.fwdClosedStamp[bwdCurrent] === this.fwdStamp) {
        this.meetingPoint = bwdCurrent;
        this.completed = true;
        this.foundPath = this.buildPath();
        return PathFindResultType.Completed;
      }

      this.expandBackward(bwdCurrent, width, numNodes);
    }

    return PathFindResultType.PathNotFound;
  }

  private expandForward(
    current: number,
    width: number,
    numNodes: number,
    dst: number,
  ): void {
    const currentGScore = this.fwdGScore[current];
    const currentX = current % width;

    // Up
    if (current >= width) {
      this.processNeighborForward(
        current - width,
        current,
        currentGScore,
        dst,
      );
    }
    // Down
    if (current < numNodes - width) {
      this.processNeighborForward(
        current + width,
        current,
        currentGScore,
        dst,
      );
    }
    // Left
    if (currentX !== 0) {
      this.processNeighborForward(current - 1, current, currentGScore, dst);
    }
    // Right
    if (currentX !== width - 1) {
      this.processNeighborForward(current + 1, current, currentGScore, dst);
    }
  }

  private expandBackward(
    current: number,
    width: number,
    numNodes: number,
  ): void {
    const currentGScore = this.bwdGScore[current];
    const currentX = current % width;

    // Up
    if (current >= width) {
      this.processNeighborBackward(current - width, current, currentGScore);
    }
    // Down
    if (current < numNodes - width) {
      this.processNeighborBackward(current + width, current, currentGScore);
    }
    // Left
    if (currentX !== 0) {
      this.processNeighborBackward(current - 1, current, currentGScore);
    }
    // Right
    if (currentX !== width - 1) {
      this.processNeighborBackward(current + 1, current, currentGScore);
    }
  }

  private processNeighborForward(
    neighbor: number,
    current: number,
    currentGScore: number,
    dst: number,
  ): void {
    // Skip non-traversable (except destination)
    if (neighbor !== dst && !this.graph.isTraversable(current, neighbor)) {
      return;
    }

    // Skip already processed
    if (this.fwdClosedStamp[neighbor] === this.fwdStamp) return;

    const tentativeGScore = currentGScore + this.graph.cost(neighbor);
    const hasValidGScore = this.fwdGScoreStamp[neighbor] === this.fwdStamp;

    if (!hasValidGScore || tentativeGScore < this.fwdGScore[neighbor]) {
      this.fwdCameFrom[neighbor] = current;
      this.fwdGScore[neighbor] = tentativeGScore;
      this.fwdGScoreStamp[neighbor] = this.fwdStamp;
      this.fwdFScore[neighbor] = tentativeGScore + this.heuristicForward(neighbor);
      this.fwdOpenHeap.push(neighbor);
    }
  }

  private processNeighborBackward(
    neighbor: number,
    current: number,
    currentGScore: number,
  ): void {
    // For backward search, check traversability in reverse
    // Skip non-traversable (except sources)
    const isSource = this.sources.includes(neighbor);
    if (!isSource && !this.graph.isTraversable(current, neighbor)) {
      return;
    }

    // Skip already processed
    if (this.bwdClosedStamp[neighbor] === this.bwdStamp) return;

    const tentativeGScore = currentGScore + this.graph.cost(neighbor);
    const hasValidGScore = this.bwdGScoreStamp[neighbor] === this.bwdStamp;

    if (!hasValidGScore || tentativeGScore < this.bwdGScore[neighbor]) {
      this.bwdCameFrom[neighbor] = current;
      this.bwdGScore[neighbor] = tentativeGScore;
      this.bwdGScoreStamp[neighbor] = this.bwdStamp;
      this.bwdFScore[neighbor] = tentativeGScore + this.heuristicBackward(neighbor);
      this.bwdOpenHeap.push(neighbor);
    }
  }

  private buildPath(): number[] {
    if (this.meetingPoint === -1) return [];

    // Build forward path (start -> meeting point)
    const fwdPath: number[] = [];
    let current = this.meetingPoint;

    while (current !== -1 && this.fwdCameFrom[current] !== undefined) {
      fwdPath.push(current);
      const prev = this.fwdCameFrom[current];
      if (prev === -1) break;
      current = prev;
    }

    fwdPath.reverse();

    // Build backward path (meeting point -> goal)
    current = this.meetingPoint;
    const bwdPath: number[] = [];

    while (current !== -1 && this.bwdCameFrom[current] !== undefined) {
      const next = this.bwdCameFrom[current];
      if (next === -1) break;
      bwdPath.push(next);
      current = next;
    }

    return [...fwdPath, ...bwdPath];
  }

  reconstructPath(): number[] {
    return this.foundPath ?? [];
  }
}

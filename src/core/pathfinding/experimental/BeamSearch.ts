// Beam Search - keeps only top N nodes in open set
// Falls back to A* if beam search fails to find path

import { AStar, PathFindResultType } from "../AStar";
import { GraphAdapter } from "../SerialAStar";
import { MinHeap } from "./MinHeap";
import { OptimizedAStar } from "./OptimizedAStar";

export class BeamSearch implements AStar<number> {
  // Search state
  private stamp = 1;
  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Float32Array;
  private readonly fScore: Float32Array;
  private readonly cameFrom: Int32Array;
  private readonly openHeap: MinHeap;

  // Beam tracking - nodes currently in beam
  private readonly inBeamStamp: Uint32Array;
  private beamCount = 0;

  // Config
  private readonly beamWidth: number;
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

  // Fallback state
  private useFallback = false;
  private fallbackAStar: OptimizedAStar | null = null;

  public completed = false;
  private foundPath: number[] | null = null;

  constructor(
    graph: GraphAdapter<number>,
    numNodes: number,
    width: number,
    beamWidth: number = 2000,
    iterations: number = 500_000,
    maxTries: number = 50,
  ) {
    this.graph = graph;
    this.width = width;
    this.numNodes = numNodes;
    this.beamWidth = beamWidth;
    this.iterations = iterations;
    this.maxTries = maxTries;
    this.initialMaxTries = maxTries;

    // Search arrays
    this.closedStamp = new Uint32Array(numNodes);
    this.gScoreStamp = new Uint32Array(numNodes);
    this.gScore = new Float32Array(numNodes);
    this.fScore = new Float32Array(numNodes);
    this.cameFrom = new Int32Array(numNodes);
    this.openHeap = new MinHeap(numNodes, this.fScore);
    this.inBeamStamp = new Uint32Array(numNodes);
  }

  reset(src: number | number[], dst: number): void {
    this.sources = Array.isArray(src) ? src : [src];
    this.dst = dst;
    this.dstX = dst % this.width;
    this.dstY = (dst / this.width) | 0;
    this.completed = false;
    this.foundPath = null;
    this.maxTries = this.initialMaxTries;
    this.beamCount = 0;
    this.useFallback = false;

    // Increment stamp
    this.stamp++;
    if (this.stamp === 0) {
      this.closedStamp.fill(0);
      this.gScoreStamp.fill(0);
      this.inBeamStamp.fill(0);
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
      this.inBeamStamp[start] = stamp;
      this.beamCount++;
    }
  }

  private heuristic(node: number): number {
    const x = node % this.width;
    const y = (node / this.width) | 0;
    return 15 * (Math.abs(x - this.dstX) + Math.abs(y - this.dstY));
  }

  compute(): PathFindResultType {
    if (this.completed) return PathFindResultType.Completed;

    // If fallback is active, delegate to it
    if (this.useFallback && this.fallbackAStar) {
      const result = this.fallbackAStar.compute();
      if (result === PathFindResultType.Completed) {
        this.completed = true;
        this.foundPath = this.fallbackAStar.reconstructPath();
      }
      return result;
    }

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
          // Beam search exhausted, try fallback
          return this.activateFallback();
        }
        return PathFindResultType.Pending;
      }

      const current = this.openHeap.pop();

      // Track beam membership
      if (this.inBeamStamp[current] === stamp) {
        this.beamCount--;
        this.inBeamStamp[current] = 0; // Remove from beam
      }

      // Skip if already processed
      if (this.closedStamp[current] === stamp) continue;
      this.closedStamp[current] = stamp;

      // Found goal
      if (current === dst) {
        this.completed = true;
        this.foundPath = this.buildPath(current);
        return PathFindResultType.Completed;
      }

      const currentGScore = this.gScore[current];
      const currentX = current % width;

      // Expand neighbors
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

      // Prune beam if too large
      // Note: We can't easily prune the heap, so we just track count
      // and let duplicates expire naturally via closedStamp
      // The beam width acts as a soft limit via the pruning in processNeighbor
    }

    // Open set empty, beam search failed - try fallback
    return this.activateFallback();
  }

  private processNeighbor(
    neighbor: number,
    current: number,
    currentGScore: number,
    stamp: number,
    dst: number,
  ): void {
    // Skip non-traversable (except destination)
    if (neighbor !== dst && !this.graph.isTraversable(current, neighbor)) {
      return;
    }

    // Skip already processed
    if (this.closedStamp[neighbor] === stamp) return;

    const tentativeGScore = currentGScore + this.graph.cost(neighbor);
    const hasValidGScore = this.gScoreStamp[neighbor] === stamp;

    if (!hasValidGScore || tentativeGScore < this.gScore[neighbor]) {
      this.cameFrom[neighbor] = current;
      this.gScore[neighbor] = tentativeGScore;
      this.gScoreStamp[neighbor] = stamp;
      this.fScore[neighbor] = tentativeGScore + this.heuristic(neighbor);

      // Only add to beam if we have room or this is better than worst
      const isInBeam = this.inBeamStamp[neighbor] === stamp;
      if (!isInBeam) {
        if (this.beamCount < this.beamWidth) {
          this.openHeap.push(neighbor);
          this.inBeamStamp[neighbor] = stamp;
          this.beamCount++;
        }
        // If beam is full, we skip this node (beam pruning)
        // This is aggressive but fast
      } else {
        // Already in beam, update with better score
        this.openHeap.push(neighbor);
      }
    }
  }

  private activateFallback(): PathFindResultType {
    this.useFallback = true;

    // Create fallback lazily, reuse across queries
    if (!this.fallbackAStar) {
      this.fallbackAStar = new OptimizedAStar(
        this.graph,
        this.numNodes,
        this.width,
        this.iterations,
        this.initialMaxTries,
      );
    }

    this.fallbackAStar.reset(this.sources, this.dst);
    const result = this.fallbackAStar.compute();

    // Handle completion on first call
    if (result === PathFindResultType.Completed) {
      this.completed = true;
      this.foundPath = this.fallbackAStar.reconstructPath();
    }

    return result;
  }

  private buildPath(goal: number): number[] {
    const path: number[] = [];
    let current = goal;

    while (current !== -1) {
      path.push(current);
      const prev = this.cameFrom[current];
      if (prev === -1) break;
      current = prev;
    }

    path.reverse();
    return path;
  }

  reconstructPath(): number[] {
    return this.foundPath ?? [];
  }
}

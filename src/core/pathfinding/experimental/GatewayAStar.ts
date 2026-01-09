// Gateway A* - inlined A* for gateway graph routing
// Optimized for small graphs with variable edge costs

import { GatewayGraph } from "../navmesh/GatewayGraph";
import { BucketQueue, MinHeap, PriorityQueue } from "./PriorityQueue";

export interface GatewayAStarConfig {
  heuristicWeight?: number;
  maxIterations?: number;
  useMinHeap?: boolean; // Use MinHeap instead of BucketQueue (better for variable costs)
}

export class GatewayAStar {
  private stamp = 1;

  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Float32Array;
  private readonly cameFrom: Int32Array;
  private readonly queue: PriorityQueue;
  private readonly graph: GatewayGraph;
  private readonly heuristicWeight: number;
  private readonly maxIterations: number;

  constructor(graph: GatewayGraph, config?: GatewayAStarConfig) {
    this.graph = graph;
    this.heuristicWeight = config?.heuristicWeight ?? 1;
    this.maxIterations = config?.maxIterations ?? 100_000;

    const numGateways = graph.getAllGateways().length;

    this.closedStamp = new Uint32Array(numGateways);
    this.gScoreStamp = new Uint32Array(numGateways);
    this.gScore = new Float32Array(numGateways);
    this.cameFrom = new Int32Array(numGateways);

    // For gateway graphs with variable costs, MinHeap may be better
    // BucketQueue is O(1) but requires integer priorities
    if (config?.useMinHeap) {
      this.queue = new MinHeap(numGateways);
    } else {
      // Estimate max priority: weight * (mapWidth + mapHeight)
      // Use sector size * sectors as approximation
      const maxDist = graph.sectorSize * Math.max(graph.sectorsX, 10) * 2;
      const maxF = this.heuristicWeight * maxDist;
      this.queue = new BucketQueue(maxF);
    }
  }

  search(startId: number, goalId: number): number[] | null {
    // Advance stamp (handles overflow)
    this.stamp++;
    if (this.stamp === 0) {
      this.closedStamp.fill(0);
      this.gScoreStamp.fill(0);
      this.stamp = 1;
    }

    const stamp = this.stamp;
    const graph = this.graph;
    const closedStamp = this.closedStamp;
    const gScoreStamp = this.gScoreStamp;
    const gScore = this.gScore;
    const cameFrom = this.cameFrom;
    const queue = this.queue;
    const weight = this.heuristicWeight;

    // Get goal gateway for heuristic
    const goalGw = graph.getGateway(goalId);
    if (!goalGw) return null;
    const goalX = goalGw.x;
    const goalY = goalGw.y;

    // Get start gateway for initial heuristic
    const startGw = graph.getGateway(startId);
    if (!startGw) return null;

    // Initialize
    queue.clear();
    gScore[startId] = 0;
    gScoreStamp[startId] = stamp;
    cameFrom[startId] = -1;

    const startH =
      weight * (Math.abs(startGw.x - goalX) + Math.abs(startGw.y - goalY));
    queue.push(startId, startH);

    let iterations = this.maxIterations;

    while (!queue.isEmpty()) {
      if (--iterations <= 0) {
        return null;
      }

      const current = queue.pop();

      if (closedStamp[current] === stamp) continue;
      closedStamp[current] = stamp;

      if (current === goalId) {
        return this.buildPath(startId, goalId);
      }

      const currentG = gScore[current];
      const edges = graph.getEdges(current);

      // Inline neighbor iteration
      for (let i = 0; i < edges.length; i++) {
        const edge = edges[i];
        const neighbor = edge.to;

        if (closedStamp[neighbor] === stamp) continue;

        const tentativeG = currentG + edge.cost;

        if (gScoreStamp[neighbor] !== stamp || tentativeG < gScore[neighbor]) {
          cameFrom[neighbor] = current;
          gScore[neighbor] = tentativeG;
          gScoreStamp[neighbor] = stamp;

          // Inline heuristic calculation
          const neighborGw = graph.getGateway(neighbor);
          if (neighborGw) {
            const h =
              weight *
              (Math.abs(neighborGw.x - goalX) + Math.abs(neighborGw.y - goalY));
            queue.push(neighbor, tentativeG + h);
          }
        }
      }
    }

    return null;
  }

  private buildPath(startId: number, goalId: number): number[] {
    const path: number[] = [];
    let current = goalId;

    while (current !== -1) {
      path.push(current);
      current = this.cameFrom[current];
    }

    path.reverse();
    return path;
  }
}

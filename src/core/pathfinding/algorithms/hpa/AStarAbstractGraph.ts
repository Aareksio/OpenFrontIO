// Abstract Graph A* - inlined A* for abstract graph routing

import { PathFinder } from "../../types";
import { BucketQueue, MinHeap, PriorityQueue } from "../PriorityQueue";
import { AbstractGraph } from "./AbstractGraph";

export interface AbstractGraphAStarConfig {
  heuristicWeight?: number;
  maxIterations?: number;
  useMinHeap?: boolean; // Use MinHeap instead of BucketQueue (better for variable costs)
}

export class AbstractGraphAStar implements PathFinder<number> {
  private stamp = 1;

  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Float32Array;
  private readonly cameFrom: Int32Array;
  private readonly queue: PriorityQueue;
  private readonly graph: AbstractGraph;
  private readonly heuristicWeight: number;
  private readonly maxIterations: number;

  constructor(graph: AbstractGraph, config?: AbstractGraphAStarConfig) {
    this.graph = graph;
    this.heuristicWeight = config?.heuristicWeight ?? 1;
    this.maxIterations = config?.maxIterations ?? 100_000;

    const numNodes = graph.nodeCount;

    this.closedStamp = new Uint32Array(numNodes);
    this.gScoreStamp = new Uint32Array(numNodes);
    this.gScore = new Float32Array(numNodes);
    this.cameFrom = new Int32Array(numNodes);

    // For abstract graphs with variable costs, MinHeap may be better
    // BucketQueue is O(1) but requires integer priorities
    if (config?.useMinHeap) {
      this.queue = new MinHeap(numNodes);
    } else {
      // Estimate max priority: weight * (mapWidth + mapHeight)
      // Use cluster size * clusters as approximation
      const maxDist = graph.clusterSize * Math.max(graph.clustersX, 10) * 2;
      const maxF = this.heuristicWeight * maxDist;
      this.queue = new BucketQueue(maxF);
    }
  }

  findPath(start: number | number[], goal: number): number[] | null {
    if (Array.isArray(start)) {
      throw new Error(
        "AbstractGraphAStar does not support multiple start points",
      );
    }
    return this.findPathSingle(start, goal);
  }

  private findPathSingle(startId: number, goalId: number): number[] | null {
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

    // Get goal node for heuristic
    const goalNode = graph.getNode(goalId);
    if (!goalNode) return null;
    const goalX = goalNode.x;
    const goalY = goalNode.y;

    // Get start node for initial heuristic
    const startNode = graph.getNode(startId);
    if (!startNode) return null;

    // Initialize
    queue.clear();
    gScore[startId] = 0;
    gScoreStamp[startId] = stamp;
    cameFrom[startId] = -1;

    const startH =
      weight * (Math.abs(startNode.x - goalX) + Math.abs(startNode.y - goalY));
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
      const edges = graph.getNodeEdges(current);

      // Inline neighbor iteration
      for (let i = 0; i < edges.length; i++) {
        const edge = edges[i];
        const neighbor = graph.getOtherNode(edge, current);

        if (closedStamp[neighbor] === stamp) continue;

        const tentativeG = currentG + edge.cost;

        if (gScoreStamp[neighbor] !== stamp || tentativeG < gScore[neighbor]) {
          cameFrom[neighbor] = current;
          gScore[neighbor] = tentativeG;
          gScoreStamp[neighbor] = stamp;

          // Inline heuristic calculation
          const neighborNode = graph.getNode(neighbor);
          if (neighborNode) {
            const h =
              weight *
              (Math.abs(neighborNode.x - goalX) +
                Math.abs(neighborNode.y - goalY));
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

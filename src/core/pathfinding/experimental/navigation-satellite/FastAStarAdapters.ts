import { GameMap, TileRef } from '../../../game/GameMap';
import { FastAStarAdapter } from './FastAStar';
import { GatewayGraph } from './GatewayGraph';

export class FastGatewayGraphAdapter implements FastAStarAdapter {
  constructor(private graph: GatewayGraph) {}

  getNeighbors(node: number): number[] {
    const edges = this.graph.getEdges(node);
    return edges.map(edge => edge.to);
  }

  getCost(from: number, to: number): number {
    const edges = this.graph.getEdges(from);
    const edge = edges.find(edge => edge.to === to);
    return edge?.cost ?? 1;
  }

  heuristic(node: number, goal: number): number {
    const nodeGw = this.graph.getGateway(node);
    const goalGw = this.graph.getGateway(goal);
    if (!nodeGw || !goalGw) return 0;

    // Manhattan distance heuristic
    const dx = Math.abs(nodeGw.x - goalGw.x);
    const dy = Math.abs(nodeGw.y - goalGw.y);
    return dx + dy;
  }
}

export class BoundedGameMapAdapter implements FastAStarAdapter {
  private readonly minX: number;
  private readonly minY: number;
  private readonly width: number;
  private readonly height: number;
  readonly numNodes: number;
  private readonly startTile: TileRef;
  private readonly goalTile: TileRef;

  constructor(
    private miniMap: GameMap,
    startTile: TileRef,
    goalTile: TileRef,
    bounds: { minX: number; maxX: number; minY: number; maxY: number }
  ) {
    this.startTile = startTile;
    this.goalTile = goalTile;

    this.minX = bounds.minX;
    this.minY = bounds.minY;
    this.width = bounds.maxX - bounds.minX + 1;
    this.height = bounds.maxY - bounds.minY + 1;

    this.numNodes = this.width * this.height;
  }

  // Convert global TileRef to local node ID
  tileToNode(tile: TileRef): number {
    const x = this.miniMap.x(tile) - this.minX;
    const y = this.miniMap.y(tile) - this.minY;

    // Allow start and goal tiles to be outside bounds (matching graph building behavior)
    const isStartOrGoal = tile === this.startTile || tile === this.goalTile;
    if (!isStartOrGoal && (x < 0 || x >= this.width || y < 0 || y >= this.height)) {
      return -1; // Outside bounds
    }

    // Clamp coordinates for start/goal tiles that are outside bounds
    const clampedX = Math.max(0, Math.min(this.width - 1, x));
    const clampedY = Math.max(0, Math.min(this.height - 1, y));

    return clampedY * this.width + clampedX;
  }

  // Convert local node ID to global TileRef
  nodeToTile(node: number): TileRef {
    const localX = node % this.width;
    const localY = Math.floor(node / this.width);
    return this.miniMap.ref(localX + this.minX, localY + this.minY);
  }

  getNeighbors(node: number): number[] {
    const tile = this.nodeToTile(node);
    const neighbors = this.miniMap.neighbors(tile);
    const result: number[] = [];

    for (const neighborTile of neighbors) {
      if (!this.miniMap.isWater(neighborTile)) continue;

      const neighborNode = this.tileToNode(neighborTile);
      if (neighborNode !== -1) {
        result.push(neighborNode);
      }
    }

    return result;
  }

  getCost(from: number, to: number): number {
    return 1; // Uniform cost for water tiles
  }

  heuristic(node: number, goal: number): number {
    const nodeTile = this.nodeToTile(node);
    const goalTile = this.nodeToTile(goal);

    const dx = Math.abs(this.miniMap.x(nodeTile) - this.miniMap.x(goalTile));
    const dy = Math.abs(this.miniMap.y(nodeTile) - this.miniMap.y(goalTile));

    return dx + dy; // Manhattan distance
  }
}

// Source Selector for HPA* multi-source search
// Selects best source from candidates using abstract graph BFS

import { GameMap, TileRef } from "../../../game/GameMap";
import { AbstractGraph, AbstractNode } from "./AbstractGraph";
import { LAND_MARKER } from "./WaterComponents";

export class SourceSelector {
  constructor(
    private map: GameMap,
    private graph: AbstractGraph,
  ) {}

  /**
   * Select best source from candidates to reach target.
   * Uses abstract graph BFS for fast approximate distance.
   */
  selectBestSource(sources: TileRef[], target: TileRef): TileRef | null {
    // 1. Resolve target to water and get its component
    const targetWater = this.resolveToWater(target);
    if (targetWater === null) return null;

    const targetNode = this.getClusterNodeForTile(targetWater);
    if (targetNode === null) return null;

    const targetComponent = targetNode.componentId;

    // 2. Map sources to cluster nodes (filter by component)
    const sourceNodeMap = new Map<number, TileRef>(); // nodeId → sourceTile
    for (const source of sources) {
      const sourceWater = this.resolveToWater(source, targetComponent);
      if (sourceWater === null) continue;

      const sourceNode = this.getClusterNodeForTile(
        sourceWater,
        targetComponent,
      );
      if (sourceNode === null) continue;

      // Only keep first source per node (avoid duplicates)
      if (!sourceNodeMap.has(sourceNode.id)) {
        sourceNodeMap.set(sourceNode.id, source);
      }
    }

    if (sourceNodeMap.size === 0) return null;

    // 3. BFS from target node to find nearest source node
    const winnerNodeId = this.findNearestSourceNode(
      targetNode.id,
      new Set(sourceNodeMap.keys()),
    );

    if (winnerNodeId === null) return null;

    return sourceNodeMap.get(winnerNodeId) ?? null;
  }

  /**
   * Check if a tile is water using component ID (faster than isWater bit ops)
   */
  private isWaterTile(tile: TileRef): boolean {
    return this.graph.getComponentId(tile) !== LAND_MARKER;
  }

  /**
   * Resolve land/shore tile to adjacent water tile.
   * If already water, validates component if specified.
   * Works directly on map coordinates (no full↔mini conversion).
   */
  private resolveToWater(
    tile: TileRef,
    targetComponent?: number,
  ): TileRef | null {
    // If already water, validate component
    if (this.isWaterTile(tile)) {
      if (targetComponent !== undefined) {
        const component = this.graph.getComponentId(tile);
        return component === targetComponent ? tile : null;
      }
      return tile;
    }

    // Shore tile: find adjacent water tile matching component
    for (const neighbor of this.map.neighbors(tile)) {
      if (this.isWaterTile(neighbor)) {
        if (targetComponent === undefined) {
          return neighbor;
        }
        const component = this.graph.getComponentId(neighbor);
        if (component === targetComponent) {
          return neighbor;
        }
      }
    }

    return null;
  }

  /**
   * Get cluster node for a map water tile.
   * O(1) cluster lookup + O(nodes_in_cluster) component match.
   */
  private getClusterNodeForTile(
    tile: TileRef,
    filterComponent?: number,
  ): AbstractNode | null {
    const x = this.map.x(tile);
    const y = this.map.y(tile);
    const clusterX = Math.floor(x / this.graph.clusterSize);
    const clusterY = Math.floor(y / this.graph.clusterSize);

    const cluster = this.graph.getCluster(clusterX, clusterY);
    if (!cluster || cluster.nodeIds.length === 0) return null;

    // Return first node matching component (or any if no filter)
    for (const nodeId of cluster.nodeIds) {
      const node = this.graph.getNode(nodeId);
      if (!node) continue;

      if (
        filterComponent === undefined ||
        node.componentId === filterComponent
      ) {
        return node;
      }
    }

    return null;
  }

  /**
   * BFS on abstract graph to find nearest source node from target.
   * Returns first source node ID reached, or null if none reachable.
   */
  private findNearestSourceNode(
    targetNodeId: number,
    sourceNodeIds: Set<number>,
  ): number | null {
    // Early exit: target is a source
    if (sourceNodeIds.has(targetNodeId)) {
      return targetNodeId;
    }

    const visited = new Set<number>();
    const queue: number[] = [targetNodeId];
    visited.add(targetNodeId);

    while (queue.length > 0) {
      const current = queue.shift()!;

      for (const edge of this.graph.getNodeEdges(current)) {
        const neighbor = this.graph.getOtherNode(edge, current);
        if (visited.has(neighbor)) continue;

        if (sourceNodeIds.has(neighbor)) {
          return neighbor;
        }

        visited.add(neighbor);
        queue.push(neighbor);
      }
    }

    return null;
  }
}

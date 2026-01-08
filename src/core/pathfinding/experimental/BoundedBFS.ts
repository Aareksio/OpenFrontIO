// Bounded BFS for finding nearest abstract node within cluster bounds
// Specialized from navmesh/FastBFS.ts - removes generic visitor pattern

import { GameMap, TileRef } from "../../game/GameMap";
import { AbstractNode } from "./AbstractGraph";

const LAND_BIT = 7;

export interface SearchBounds {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
}

/**
 * Bounded BFS for finding nearest AbstractNode within specified bounds.
 * Uses stamp-based visited tracking and typed arrays for performance.
 */
export class BoundedBFS {
  private stamp = 1;
  private readonly visitedStamp: Uint32Array;
  private readonly queue: Int32Array;
  private readonly dist: Uint16Array;
  private readonly terrain: Uint8Array;
  private readonly mapWidth: number;
  private readonly landMask: number;

  constructor(map: GameMap, maxSearchArea: number) {
    this.terrain = (map as any).terrain as Uint8Array;
    this.mapWidth = map.width();
    this.landMask = 1 << LAND_BIT;

    this.visitedStamp = new Uint32Array(maxSearchArea);
    this.queue = new Int32Array(maxSearchArea);
    this.dist = new Uint16Array(maxSearchArea);
  }

  /**
   * Find the nearest AbstractNode from start tile within bounds.
   *
   * @param start - Starting tile reference
   * @param candidates - Array of candidate nodes to search for
   * @param bounds - Search area bounds
   * @param maxDistance - Maximum BFS distance
   * @returns The nearest AbstractNode or null if none found
   */
  findNearestNode(
    start: TileRef,
    candidates: AbstractNode[],
    bounds: SearchBounds,
    maxDistance: number,
  ): AbstractNode | null {
    if (candidates.length === 0) return null;

    const stamp = this.nextStamp();
    const { minX, maxX, minY, maxY } = bounds;
    const boundsWidth = maxX - minX + 1;
    const mapWidth = this.mapWidth;
    const terrain = this.terrain;
    const landMask = this.landMask;

    // Convert global tile to local index
    const toLocal = (tile: TileRef): number => {
      const x = tile % mapWidth;
      const y = (tile / mapWidth) | 0;
      return (y - minY) * boundsWidth + (x - minX);
    };

    // Build lookup set for candidate tiles (for O(1) check)
    const candidateByTile = new Map<number, AbstractNode>();
    for (const node of candidates) {
      candidateByTile.set(node.tile, node);
    }

    const startLocal = toLocal(start);
    let head = 0;
    let tail = 0;

    this.visitedStamp[startLocal] = stamp;
    this.dist[startLocal] = 0;
    this.queue[tail++] = start; // Store global tile ref

    while (head < tail) {
      const tile = this.queue[head++] as TileRef;
      const tileLocal = toLocal(tile);
      const currentDist = this.dist[tileLocal];

      if (currentDist > maxDistance) {
        continue;
      }

      // Check if this tile is a candidate node
      const node = candidateByTile.get(tile);
      if (node) {
        return node;
      }

      // Check bounds - if outside cluster, reject but don't fail
      const tileX = tile % mapWidth;
      const tileY = (tile / mapWidth) | 0;
      if (tileX < minX || tileX > maxX || tileY < minY || tileY > maxY) {
        continue; // Outside bounds, skip neighbors
      }

      const nextDist = currentDist + 1;

      // North
      if (tileY > minY) {
        const n = tile - mapWidth;
        const nLocal = tileLocal - boundsWidth;
        if (
          this.visitedStamp[nLocal] !== stamp &&
          (terrain[n] & landMask) === 0
        ) {
          this.visitedStamp[nLocal] = stamp;
          this.dist[nLocal] = nextDist;
          this.queue[tail++] = n;
        }
      }

      // South
      if (tileY < maxY) {
        const s = tile + mapWidth;
        const sLocal = tileLocal + boundsWidth;
        if (
          this.visitedStamp[sLocal] !== stamp &&
          (terrain[s] & landMask) === 0
        ) {
          this.visitedStamp[sLocal] = stamp;
          this.dist[sLocal] = nextDist;
          this.queue[tail++] = s;
        }
      }

      // West
      if (tileX > minX) {
        const w = tile - 1;
        const wLocal = tileLocal - 1;
        if (
          this.visitedStamp[wLocal] !== stamp &&
          (terrain[w] & landMask) === 0
        ) {
          this.visitedStamp[wLocal] = stamp;
          this.dist[wLocal] = nextDist;
          this.queue[tail++] = w;
        }
      }

      // East
      if (tileX < maxX) {
        const e = tile + 1;
        const eLocal = tileLocal + 1;
        if (
          this.visitedStamp[eLocal] !== stamp &&
          (terrain[e] & landMask) === 0
        ) {
          this.visitedStamp[eLocal] = stamp;
          this.dist[eLocal] = nextDist;
          this.queue[tail++] = e;
        }
      }
    }

    return null;
  }

  private nextStamp(): number {
    const stamp = this.stamp++;

    if (this.stamp === 0) {
      // Overflow - reset (extremely rare)
      this.visitedStamp.fill(0);
      this.stamp = 1;
    }

    return stamp;
  }
}

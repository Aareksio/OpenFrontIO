// Water A* implementations - inlined for performance + generic adapter

import { GameMap, TileRef } from "../../game/GameMap";
import { AStar, GenericAStarAdapter } from "./AStar";
import { BucketQueue, PriorityQueue } from "./PriorityQueue";

const LAND_BIT = 7; // Bit 7 in terrain indicates land

export interface GameMapAStarConfig {
  heuristicWeight?: number;
  maxIterations?: number;
}

export class GameMapAStar implements AStar {
  private stamp = 1;

  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Uint32Array;
  private readonly cameFrom: Int32Array;
  private readonly queue: PriorityQueue;
  private readonly terrain: Uint8Array;
  private readonly width: number;
  private readonly numNodes: number;
  private readonly heuristicWeight: number;
  private readonly maxIterations: number;

  constructor(map: GameMap, config?: GameMapAStarConfig) {
    this.terrain = (map as any).terrain as Uint8Array;
    this.width = map.width();
    this.numNodes = map.width() * map.height();
    this.heuristicWeight = config?.heuristicWeight ?? 15;
    this.maxIterations = config?.maxIterations ?? 1_000_000;

    this.closedStamp = new Uint32Array(this.numNodes);
    this.gScoreStamp = new Uint32Array(this.numNodes);
    this.gScore = new Uint32Array(this.numNodes);
    this.cameFrom = new Int32Array(this.numNodes);

    const maxF = this.heuristicWeight * (map.width() + map.height());
    this.queue = new BucketQueue(maxF);
  }

  search(start: number | number[], goal: number): number[] | null {
    this.stamp++;
    if (this.stamp === 0) {
      this.closedStamp.fill(0);
      this.gScoreStamp.fill(0);
      this.stamp = 1;
    }

    const stamp = this.stamp;
    const width = this.width;
    const numNodes = this.numNodes;
    const terrain = this.terrain;
    const closedStamp = this.closedStamp;
    const gScoreStamp = this.gScoreStamp;
    const gScore = this.gScore;
    const cameFrom = this.cameFrom;
    const queue = this.queue;
    const weight = this.heuristicWeight;
    const landMask = 1 << LAND_BIT;

    const goalX = goal % width;
    const goalY = (goal / width) | 0;

    queue.clear();
    const starts = Array.isArray(start) ? start : [start];
    for (const s of starts) {
      gScore[s] = 0;
      gScoreStamp[s] = stamp;
      cameFrom[s] = -1;
      const sx = s % width;
      const sy = (s / width) | 0;
      const h = weight * (Math.abs(sx - goalX) + Math.abs(sy - goalY));
      queue.push(s, h);
    }

    let iterations = this.maxIterations;

    while (!queue.isEmpty()) {
      if (--iterations <= 0) {
        return null;
      }

      const current = queue.pop();

      if (closedStamp[current] === stamp) continue;
      closedStamp[current] = stamp;

      if (current === goal) {
        return this.buildPath(goal);
      }

      const currentG = gScore[current];
      const tentativeG = currentG + 1;
      const currentX = current % width;

      if (current >= width) {
        const neighbor = current - width;
        if (
          closedStamp[neighbor] !== stamp &&
          (neighbor === goal || (terrain[neighbor] & landMask) === 0)
        ) {
          if (
            gScoreStamp[neighbor] !== stamp ||
            tentativeG < gScore[neighbor]
          ) {
            cameFrom[neighbor] = current;
            gScore[neighbor] = tentativeG;
            gScoreStamp[neighbor] = stamp;
            const nx = neighbor % width;
            const ny = (neighbor / width) | 0;
            const f =
              tentativeG +
              weight * (Math.abs(nx - goalX) + Math.abs(ny - goalY));
            queue.push(neighbor, f);
          }
        }
      }

      if (current < numNodes - width) {
        const neighbor = current + width;
        if (
          closedStamp[neighbor] !== stamp &&
          (neighbor === goal || (terrain[neighbor] & landMask) === 0)
        ) {
          if (
            gScoreStamp[neighbor] !== stamp ||
            tentativeG < gScore[neighbor]
          ) {
            cameFrom[neighbor] = current;
            gScore[neighbor] = tentativeG;
            gScoreStamp[neighbor] = stamp;
            const nx = neighbor % width;
            const ny = (neighbor / width) | 0;
            const f =
              tentativeG +
              weight * (Math.abs(nx - goalX) + Math.abs(ny - goalY));
            queue.push(neighbor, f);
          }
        }
      }

      if (currentX !== 0) {
        const neighbor = current - 1;
        if (
          closedStamp[neighbor] !== stamp &&
          (neighbor === goal || (terrain[neighbor] & landMask) === 0)
        ) {
          if (
            gScoreStamp[neighbor] !== stamp ||
            tentativeG < gScore[neighbor]
          ) {
            cameFrom[neighbor] = current;
            gScore[neighbor] = tentativeG;
            gScoreStamp[neighbor] = stamp;
            const ny = (neighbor / width) | 0;
            const f =
              tentativeG +
              weight * (Math.abs(currentX - 1 - goalX) + Math.abs(ny - goalY));
            queue.push(neighbor, f);
          }
        }
      }

      if (currentX !== width - 1) {
        const neighbor = current + 1;
        if (
          closedStamp[neighbor] !== stamp &&
          (neighbor === goal || (terrain[neighbor] & landMask) === 0)
        ) {
          if (
            gScoreStamp[neighbor] !== stamp ||
            tentativeG < gScore[neighbor]
          ) {
            cameFrom[neighbor] = current;
            gScore[neighbor] = tentativeG;
            gScoreStamp[neighbor] = stamp;
            const ny = (neighbor / width) | 0;
            const f =
              tentativeG +
              weight * (Math.abs(currentX + 1 - goalX) + Math.abs(ny - goalY));
            queue.push(neighbor, f);
          }
        }
      }
    }

    return null;
  }

  private buildPath(goal: number): TileRef[] {
    const path: TileRef[] = [];
    let current = goal;

    while (current !== -1) {
      path.push(current as TileRef);
      current = this.cameFrom[current];
    }

    path.reverse();
    return path;
  }
}

// Generic adapter for use with GenericAStar
export class WaterGridAdapter implements GenericAStarAdapter {
  private readonly gameMap: GameMap;
  private readonly height: number;
  private readonly width: number;
  private readonly _numNodes: number;

  constructor(
    gameMap: GameMap,
    private readonly heuristicWeight: number = 15,
  ) {
    this.gameMap = gameMap;
    this.width = gameMap.width();
    this.height = gameMap.height();
    this._numNodes = this.width * this.height;
  }

  numNodes(): number {
    return this._numNodes;
  }

  maxNeighbors(): number {
    return 4;
  }

  maxPriority(): number {
    return this.heuristicWeight * (this.width + this.height);
  }

  neighbors(node: number, buffer: Int32Array): number {
    let count = 0;
    const x = node % this.width;

    if (node >= this.width) {
      const n = node - this.width;
      if (this.gameMap.isWater(n)) buffer[count++] = n;
    }
    if (node < this._numNodes - this.width) {
      const n = node + this.width;
      if (this.gameMap.isWater(n)) buffer[count++] = n;
    }
    if (x !== 0) {
      const n = node - 1;
      if (this.gameMap.isWater(n)) buffer[count++] = n;
    }
    if (x !== this.width - 1) {
      const n = node + 1;
      if (this.gameMap.isWater(n)) buffer[count++] = n;
    }

    return count;
  }

  cost(_from: number, _to: number, _prev?: number): number {
    return 1;
  }

  heuristic(node: number, goal: number): number {
    const nx = node % this.width;
    const ny = (node / this.width) | 0;
    const gx = goal % this.width;
    const gy = (goal / this.width) | 0;
    return this.heuristicWeight * (Math.abs(nx - gx) + Math.abs(ny - gy));
  }
}

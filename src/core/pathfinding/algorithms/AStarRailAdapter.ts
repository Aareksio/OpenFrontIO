// Rail/land A* adapter for GenericAStar

import { GameMap } from "../../game/GameMap";
import { GenericAStarAdapter } from "./AStar";

export interface RailAdapterConfig {
  waterPenalty?: number;
  directionChangePenalty?: number;
  heuristicWeight?: number;
}

export class RailAdapter implements GenericAStarAdapter {
  private readonly gameMap: GameMap;
  private readonly width: number;
  private readonly height: number;
  private readonly _numNodes: number;
  private readonly waterPenalty: number;
  private readonly directionChangePenalty: number;
  private readonly heuristicWeight: number;

  constructor(gameMap: GameMap, config: RailAdapterConfig = {}) {
    this.gameMap = gameMap;
    this.width = gameMap.width();
    this.height = gameMap.height();
    this._numNodes = this.width * this.height;
    this.waterPenalty = config.waterPenalty ?? 3;
    this.directionChangePenalty = config.directionChangePenalty ?? 0;
    this.heuristicWeight = config.heuristicWeight ?? 15;
  }

  numNodes(): number {
    return this._numNodes;
  }

  maxNeighbors(): number {
    return 4;
  }

  maxPriority(): number {
    // Account for water and direction penalties in max priority
    const maxCost = 1 + this.waterPenalty + this.directionChangePenalty;
    return this.heuristicWeight * (this.width + this.height) * maxCost;
  }

  neighbors(node: number, buffer: Int32Array): number {
    let count = 0;
    const x = node % this.width;
    const fromShoreline = this.gameMap.isShoreline(node);

    if (node >= this.width) {
      const n = node - this.width;
      if (this.isTraversable(n, fromShoreline)) buffer[count++] = n;
    }
    if (node < this._numNodes - this.width) {
      const n = node + this.width;
      if (this.isTraversable(n, fromShoreline)) buffer[count++] = n;
    }
    if (x !== 0) {
      const n = node - 1;
      if (this.isTraversable(n, fromShoreline)) buffer[count++] = n;
    }
    if (x !== this.width - 1) {
      const n = node + 1;
      if (this.isTraversable(n, fromShoreline)) buffer[count++] = n;
    }

    return count;
  }

  private isTraversable(to: number, fromShoreline: boolean): boolean {
    const toWater = this.gameMap.isWater(to);
    if (!toWater) return true;
    return fromShoreline || this.gameMap.isShoreline(to);
  }

  cost(from: number, to: number, prev?: number): number {
    let c = this.gameMap.isWater(to) ? 1 + this.waterPenalty : 1;

    if (prev !== undefined && this.directionChangePenalty > 0) {
      const d1 = from - prev;
      const d2 = to - from;
      if (d1 !== d2) {
        c += this.directionChangePenalty;
      }
    }

    return c;
  }

  heuristic(node: number, goal: number): number {
    const nx = node % this.width;
    const ny = (node / this.width) | 0;
    const gx = goal % this.width;
    const gy = (goal / this.width) | 0;
    return this.heuristicWeight * (Math.abs(nx - gx) + Math.abs(ny - gy));
  }
}

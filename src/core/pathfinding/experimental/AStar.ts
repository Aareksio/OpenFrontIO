// Generic A* implementation with adapter interface
// Use when performance is not critical, inline otherwise
// See GameMapAStar.ts for fully inlined water A* implementation

import { Game } from "../../game/Game";
import { GameMap } from "../../game/GameMap";
import { StationManager } from "../../game/RailNetworkImpl";
import { BucketQueue, PriorityQueue } from "./PriorityQueue";

export interface AStar {
  search(from: number | number[], to: number): number[] | null;
}

export interface GenericAStarAdapter {
  // Important optimization: write to the buffer and return the count
  // You can do this and it will be much faster :)
  neighbors(node: number, buffer: Int32Array): number;

  cost(from: number, to: number, prev?: number): number;
  heuristic(node: number, goal: number): number;
  numNodes(): number;
  maxPriority(): number;
  maxNeighbors(): number;
}

export interface GenericAStarConfig {
  adapter: GenericAStarAdapter;
  maxIterations?: number;
}

export class GenericAStar implements AStar {
  private stamp = 1;

  private readonly closedStamp: Uint32Array;
  private readonly gScoreStamp: Uint32Array;
  private readonly gScore: Uint32Array;
  private readonly cameFrom: Int32Array;
  private readonly queue: PriorityQueue;
  private readonly adapter: GenericAStarAdapter;
  private readonly neighborBuffer: Int32Array;
  private readonly maxIterations: number;

  constructor(config: GenericAStarConfig) {
    this.adapter = config.adapter;
    this.maxIterations = config.maxIterations ?? 500_000;
    this.neighborBuffer = new Int32Array(this.adapter.maxNeighbors());
    this.closedStamp = new Uint32Array(this.adapter.numNodes());
    this.gScoreStamp = new Uint32Array(this.adapter.numNodes());
    this.gScore = new Uint32Array(this.adapter.numNodes());
    this.cameFrom = new Int32Array(this.adapter.numNodes());
    this.queue = new BucketQueue(this.adapter.maxPriority());
  }

  search(start: number | number[], goal: number): number[] | null {
    // Advance stamp (handles overflow)
    this.stamp++;
    if (this.stamp === 0) {
      this.closedStamp.fill(0);
      this.gScoreStamp.fill(0);
      this.stamp = 1;
    }

    const stamp = this.stamp;
    const adapter = this.adapter;
    const closedStamp = this.closedStamp;
    const gScoreStamp = this.gScoreStamp;
    const gScore = this.gScore;
    const cameFrom = this.cameFrom;
    const queue = this.queue;
    const buffer = this.neighborBuffer;

    queue.clear();
    const starts = Array.isArray(start) ? start : [start];
    for (const s of starts) {
      gScore[s] = 0;
      gScoreStamp[s] = stamp;
      cameFrom[s] = -1;
      queue.push(s, adapter.heuristic(s, goal));
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
      const prev = cameFrom[current];
      const count = adapter.neighbors(current, buffer);

      for (let i = 0; i < count; i++) {
        const neighbor = buffer[i];

        if (closedStamp[neighbor] === stamp) continue;

        const tentativeG =
          currentG +
          adapter.cost(current, neighbor, prev === -1 ? undefined : prev);

        if (gScoreStamp[neighbor] !== stamp || tentativeG < gScore[neighbor]) {
          cameFrom[neighbor] = current;
          gScore[neighbor] = tentativeG;
          gScoreStamp[neighbor] = stamp;
          queue.push(neighbor, tentativeG + adapter.heuristic(neighbor, goal));
        }
      }
    }

    return null;
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
}

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

export class StationGraphAdapter implements GenericAStarAdapter {
  private manager: StationManager;

  constructor(private game: Game) {
    this.manager = game.railNetwork().stationManager();
  }

  numNodes(): number {
    return this.manager.count();
  }

  maxNeighbors(): number {
    return 8;
  }

  maxPriority(): number {
    return this.game.map().width() + this.game.map().height();
  }

  neighbors(node: number, buffer: Int32Array): number {
    const station = this.manager.getById(node);
    if (!station) return 0;

    let count = 0;
    for (const n of station.neighbors()) {
      buffer[count++] = n.id;
    }
    return count;
  }

  cost(): number {
    return 1;
  }

  heuristic(node: number, goal: number): number {
    const a = this.manager.getById(node);
    const b = this.manager.getById(goal);
    if (!a || !b) return 0;

    const ax = this.game.x(a.tile());
    const ay = this.game.y(a.tile());
    const bx = this.game.x(b.tile());
    const by = this.game.y(b.tile());
    return Math.abs(ax - bx) + Math.abs(ay - by);
  }
}

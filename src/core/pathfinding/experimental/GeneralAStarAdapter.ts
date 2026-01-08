// PathFinder adapter for GeneralAStar

import { Cell, Game } from "../../game/Game";
import { TileRef } from "../../game/GameMap";
import { PathFinder, PathResult, PathStatus } from "../PathFinder";
import { GeneralAStar, GeneralAStarAdapter as GraphAdapter } from "./GeneralAStar";
import { fixExtremes, upscalePath } from "./PathUpscaler";

const LAND_BIT = 7;
const HEURISTIC_WEIGHT = 15;

// Grid adapter with buffer (implements GeneralAStarAdapter interface)
class GridGraphAdapter implements GraphAdapter {
  private readonly terrain: Uint8Array;
  private readonly width: number;
  private readonly numNodes: number;
  private readonly landMask: number;

  constructor(terrain: Uint8Array, width: number, height: number) {
    this.terrain = terrain;
    this.width = width;
    this.numNodes = width * height;
    this.landMask = 1 << LAND_BIT;
  }

  heuristic(node: number, goal: number): number {
    const nx = node % this.width;
    const ny = (node / this.width) | 0;
    const gx = goal % this.width;
    const gy = (goal / this.width) | 0;
    return HEURISTIC_WEIGHT * (Math.abs(nx - gx) + Math.abs(ny - gy));
  }

  neighbors(node: number, buffer: Int32Array): number {
    let count = 0;
    const x = node % this.width;

    if (node >= this.width) {
      const n = node - this.width;
      if ((this.terrain[n] & this.landMask) === 0) buffer[count++] = n;
    }
    if (node < this.numNodes - this.width) {
      const n = node + this.width;
      if ((this.terrain[n] & this.landMask) === 0) buffer[count++] = n;
    }
    if (x !== 0) {
      const n = node - 1;
      if ((this.terrain[n] & this.landMask) === 0) buffer[count++] = n;
    }
    if (x !== this.width - 1) {
      const n = node + 1;
      if ((this.terrain[n] & this.landMask) === 0) buffer[count++] = n;
    }

    return count;
  }

  cost(_from: number, _to: number): number {
    return 1;
  }
}

export interface GeneralAStarAdapterOptions {
  maxIterations?: number;
}

export class GeneralAStarPathFinder implements PathFinder {
  private game: Game;
  private aStar: GeneralAStar;
  private miniMapWidth: number;

  constructor(game: Game, options?: GeneralAStarAdapterOptions) {
    this.game = game;
    const miniMap = game.miniMap();
    this.miniMapWidth = miniMap.width();
    const height = miniMap.height();

    const terrain = (miniMap as any).terrain as Uint8Array;
    const adapter = new GridGraphAdapter(terrain, this.miniMapWidth, height);

    const maxPriority = HEURISTIC_WEIGHT * (this.miniMapWidth + height);

    this.aStar = new GeneralAStar({
      adapter,
      numNodes: this.miniMapWidth * height,
      maxPriority,
      maxNeighbors: 4,
      maxIterations: options?.maxIterations,
    });
  }

  next(from: TileRef, to: TileRef, dist?: number): PathResult {
    const path = this.findPath(from, to);
    if (!path || path.length === 0) {
      return { status: PathStatus.NOT_FOUND };
    }

    const targetDist = dist ?? 1;
    if (this.game.manhattanDist(from, to) < targetDist) {
      return { status: PathStatus.COMPLETE, node: from };
    }

    if (path.length > 1) {
      return { status: PathStatus.NEXT, node: path[1] };
    }

    return { status: PathStatus.COMPLETE, node: path[0] };
  }

  findPath(from: TileRef, to: TileRef): TileRef[] | null {
    const gameMap = this.game.map();
    const miniMap = this.game.miniMap();

    const miniFrom = miniMap.ref(
      Math.floor(gameMap.x(from) / 2),
      Math.floor(gameMap.y(from) / 2),
    );
    const miniTo = miniMap.ref(
      Math.floor(gameMap.x(to) / 2),
      Math.floor(gameMap.y(to) / 2),
    );

    const path = this.aStar.search(miniFrom as number, miniTo as number);
    if (!path || path.length === 0) {
      return null;
    }

    const cellPath = path.map(
      (ref) => new Cell(miniMap.x(ref as TileRef), miniMap.y(ref as TileRef)),
    );

    const cellFrom = new Cell(gameMap.x(from), gameMap.y(from));
    const cellTo = new Cell(gameMap.x(to), gameMap.y(to));
    const upscaled = fixExtremes(upscalePath(cellPath), cellTo, cellFrom);

    return upscaled.map((c) => gameMap.ref(c.x, c.y));
  }
}

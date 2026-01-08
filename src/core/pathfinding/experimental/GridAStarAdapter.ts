// PathFinder adapter for GridAStar with direct terrain access

import { Cell, Game } from "../../game/Game";
import { TileRef } from "../../game/GameMap";
import { PathFinder, PathResult, PathStatus } from "../PathFinder";
import { GridAStar } from "./GridAStar";
import { fixExtremes, upscalePath } from "./PathUpscaler";

export interface GridAStarOptions {
  maxIterations?: number;
}

export class GridAStarAdapter implements PathFinder {
  private game: Game;
  private aStar: GridAStar;
  private miniMapWidth: number;

  constructor(game: Game, options?: GridAStarOptions) {
    this.game = game;
    const miniMap = game.miniMap();
    this.miniMapWidth = miniMap.width();

    // Direct access to terrain array
    const terrain = (miniMap as any).terrain as Uint8Array;

    this.aStar = new GridAStar({
      terrain,
      width: this.miniMapWidth,
      height: miniMap.height(),
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

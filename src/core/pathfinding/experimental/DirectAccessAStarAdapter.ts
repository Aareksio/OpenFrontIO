// Adapter for DirectAccessAStar (direct terrain + variable cost)

import { Cell, Game } from "../../game/Game";
import { TileRef } from "../../game/GameMap";
import { PathFindResultType } from "../AStar";
import { PathFinder, PathResult, PathStatus } from "../PathFinder";
import { DirectAccessAStar } from "./DirectAccessAStar";
import { fixExtremes, upscalePath } from "./PathUpscaler";

export class DirectAccessAStarAdapter implements PathFinder {
  private game: Game;
  private aStar: DirectAccessAStar;

  constructor(game: Game, options?: { iterations?: number; maxTries?: number }) {
    this.game = game;
    const miniMap = game.miniMap();
    const width = miniMap.width();
    const numNodes = width * miniMap.height();
    const terrain = (miniMap as any).terrain as Uint8Array;

    this.aStar = new DirectAccessAStar(
      terrain, numNodes, width,
      options?.iterations ?? 500_000,
      options?.maxTries ?? 50,
    );
  }

  next(from: TileRef, to: TileRef, dist?: number): PathResult {
    const path = this.findPath(from, to);
    if (!path || path.length === 0) return { status: PathStatus.NOT_FOUND };
    const targetDist = dist ?? 1;
    if (this.game.manhattanDist(from, to) < targetDist) return { status: PathStatus.COMPLETE, node: from };
    if (path.length > 1) return { status: PathStatus.NEXT, node: path[1] };
    return { status: PathStatus.COMPLETE, node: path[0] };
  }

  findPath(from: TileRef, to: TileRef): TileRef[] | null {
    const gameMap = this.game.map();
    const miniMap = this.game.miniMap();

    const miniFrom = miniMap.ref(Math.floor(gameMap.x(from) / 2), Math.floor(gameMap.y(from) / 2));
    const miniTo = miniMap.ref(Math.floor(gameMap.x(to) / 2), Math.floor(gameMap.y(to) / 2));

    this.aStar.reset(miniFrom as number, miniTo as number);

    let result: PathFindResultType;
    do { result = this.aStar.compute(); } while (result === PathFindResultType.Pending);

    if (result === PathFindResultType.PathNotFound) return null;

    const miniPath = this.aStar.reconstructPath();
    if (miniPath.length === 0) return null;

    const cellPath = miniPath.map((ref) => new Cell(miniMap.x(ref as TileRef), miniMap.y(ref as TileRef)));
    const cellFrom = new Cell(gameMap.x(from), gameMap.y(from));
    const cellTo = new Cell(gameMap.x(to), gameMap.y(to));
    const upscaled = fixExtremes(upscalePath(cellPath), cellTo, cellFrom);

    return upscaled.map((c) => gameMap.ref(c.x, c.y));
  }
}

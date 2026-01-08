import { Game } from "../../game/Game";
import { TileRef } from "../../game/GameMap";
import { PathFinder, PathResult, PathStatus } from "../PathFinder";
import { AStar } from "./AStar";

export class AStarPathFinder implements PathFinder {
  constructor(
    private game: Game,
    private aStar: AStar,
  ) {}

  next(from: TileRef, to: TileRef, dist?: number): PathResult {
    const path = this.findPath(from, to);
    if (!path || path.length === 0) {
      return { status: PathStatus.NOT_FOUND };
    }

    const targetDist = dist ?? 0;
    if (this.game.manhattanDist(from, to) <= targetDist) {
      return { status: PathStatus.COMPLETE, node: from };
    }

    if (path.length > 1) {
      return { status: PathStatus.NEXT, node: path[1] };
    }

    return { status: PathStatus.COMPLETE, node: path[0] };
  }

  findPath(from: TileRef, to: TileRef): TileRef[] | null {
    return this.aStar.search(from, to);
  }
}

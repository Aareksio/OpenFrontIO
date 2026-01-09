import { Game } from "../game/Game";
import { TileRef } from "../game/GameMap";
import { AStarPathFinder } from "./experimental/AStarPathFinder";
import { GameMapAStar } from "./experimental/GameMapAStar";
import { MiniAStar } from "./experimental/MiniAStar";

export enum PathStatus {
  NEXT,
  PENDING,
  COMPLETE,
  NOT_FOUND,
}

export type PathResult =
  | { status: PathStatus.PENDING }
  | { status: PathStatus.NEXT; node: TileRef }
  | { status: PathStatus.COMPLETE; node: TileRef }
  | { status: PathStatus.NOT_FOUND };

export interface PathFinder {
  next(from: TileRef, to: TileRef, dist?: number): PathResult;
  findPath(from: TileRef, to: TileRef): TileRef[] | null;
}

export class PathFinders {
  static Water(game: Game): PathFinder {
    const hpa = game.waterPathfinder();

    if (!hpa) {
      // Fall back to baseline A* if HPA* is not available
      return PathFinders.WaterDirect(game);
    }

    return new AStarPathFinder(game, hpa);
  }

  static WaterDirect(game: Game): PathFinder {
    return new AStarPathFinder(
      game,
      new MiniAStar(game, (map) => new GameMapAStar(map)),
    );
  }
}

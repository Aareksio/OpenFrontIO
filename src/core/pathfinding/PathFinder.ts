import { Game } from "../game/Game";
import { TileRef } from "../game/GameMap";
import { MiniAdapter } from "./adapters/MiniAdapter";

export enum PathStatus {
  NEXT,
  PENDING,
  COMPLETE,
  NOT_FOUND,
}

export type PathResult =
  | { status: PathStatus.NEXT; node: TileRef }
  | { status: PathStatus.PENDING }
  | { status: PathStatus.COMPLETE; node: TileRef }
  | { status: PathStatus.NOT_FOUND };

export interface PathFinder {
  next(from: TileRef, to: TileRef, dist?: number): PathResult;
}

export interface PathFinderOptions {
  iterations?: number;
  maxTries?: number;
}

export class PathFinders {
  static Water(game: Game, options?: PathFinderOptions): PathFinder {
    return new MiniAdapter(game, true, options);
  }

  static Land(game: Game, options?: PathFinderOptions): PathFinder {
    return new MiniAdapter(game, false, options);
  }
}

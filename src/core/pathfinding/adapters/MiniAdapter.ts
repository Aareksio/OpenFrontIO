import { Game } from "../../game/Game";
import { TileRef } from "../../game/GameMap";
import { PathFindResultType } from "../AStar";
import { PathFinder, PathFinderOptions, PathResult, PathStatus } from "../PathFinder";
import { MiniPathFinder } from "../PathFinding";

const DEFAULT_ITERATIONS = 10_000;
const DEFAULT_MAX_TRIES = 100;

export class MiniAdapter implements PathFinder {
  private pathFinder: MiniPathFinder;

  constructor(game: Game, waterPath: boolean, options?: PathFinderOptions) {
    this.pathFinder = new MiniPathFinder(
      game,
      options?.iterations ?? DEFAULT_ITERATIONS,
      waterPath,
      options?.maxTries ?? DEFAULT_MAX_TRIES
    );
  }

  next(from: TileRef, to: TileRef, dist?: number): PathResult {
    const result = this.pathFinder.nextTile(from, to, dist);

    switch (result.type) {
      case PathFindResultType.NextTile:
        return { status: PathStatus.NEXT, node: result.node };
      case PathFindResultType.Pending:
        return { status: PathStatus.PENDING };
      case PathFindResultType.Completed:
        return { status: PathStatus.COMPLETE, node: result.node };
      case PathFindResultType.PathNotFound:
        return { status: PathStatus.NOT_FOUND };
    }
  }

}

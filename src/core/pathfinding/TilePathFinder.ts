import { Game } from "../game/Game";
import { TileRef } from "../game/GameMap";
import { PathFinderStepper } from "./PathFinderStepper";
import {
  PathFinder,
  PathResult,
  PathStatus,
  SteppingPathFinder,
} from "./types";

/**
 * TilePathFinder - wraps a PathFinder<number> for tile-based pathfinding.
 * Adds validation and distance checking on top of PathFinderStepper.
 */
export class TilePathFinder implements SteppingPathFinder<TileRef> {
  private stepper: PathFinderStepper<TileRef>;

  constructor(
    private game: Game,
    pathFinder: PathFinder<number>,
  ) {
    // TileRef is a branded number, so === works for equality
    this.stepper = new PathFinderStepper(pathFinder, (a, b) => a === b);
  }

  next(from: TileRef, to: TileRef, dist?: number): PathResult<TileRef> {
    // Validate inputs
    if (typeof from !== "number" || typeof to !== "number") {
      return { status: PathStatus.NOT_FOUND };
    }

    if (!this.game.isValidRef(from) || !this.game.isValidRef(to)) {
      return { status: PathStatus.NOT_FOUND };
    }

    // Early exit if at destination
    if (from === to) {
      return { status: PathStatus.COMPLETE, node: to };
    }

    // Distance-based early exit
    if (dist !== undefined && dist > 0) {
      const distance = this.game.manhattanDist(from, to);
      if (distance <= dist) {
        return { status: PathStatus.COMPLETE, node: from };
      }
    }

    // Delegate to stepper
    return this.stepper.next(from, to);
  }

  findPath(from: TileRef | TileRef[], to: TileRef): TileRef[] | null {
    return this.stepper.findPath(from, to);
  }

  invalidate(): void {
    this.stepper.invalidate();
  }
}

import { Game } from "../../game/Game";
import { TileRef } from "../../game/GameMap";
import { PathFinder, PathResult, PathStatus } from "../PathFinder";
import { AStar } from "./AStar";

export class AStarPathFinder implements PathFinder {
  private pathIndex = 0;
  private path: TileRef[] | null = null;
  private lastTo: TileRef | null = null;

  constructor(
    private game: Game,
    private aStar: AStar,
  ) {}

  next(from: TileRef, to: TileRef, dist?: number): PathResult {
    if (typeof from !== "number" || typeof to !== "number") {
      return { status: PathStatus.NOT_FOUND };
    }

    if (!this.game.isValidRef(from) || !this.game.isValidRef(to)) {
      return { status: PathStatus.NOT_FOUND };
    }

    if (from === to) {
      return { status: PathStatus.COMPLETE, node: to };
    }

    if (dist !== undefined && dist > 0) {
      const distance = this.game.manhattanDist(from, to);

      if (distance <= dist) {
        return { status: PathStatus.COMPLETE, node: from };
      }
    }

    // Reset path cache if destination changed
    if (this.lastTo !== to) {
      this.path = null;
      this.pathIndex = 0;
      this.lastTo = to;
    }

    // Compute path if not cached
    if (this.path === null) {
      this.cachePath(from, to);

      if (this.path === null) {
        return { status: PathStatus.NOT_FOUND };
      }
    }

    // Re-compute if unit strayed from expected position
    const expectedPos = this.path[this.pathIndex - 1];
    if (this.pathIndex > 0 && from !== expectedPos) {
      this.cachePath(from, to);

      if (this.path === null) {
        return { status: PathStatus.NOT_FOUND };
      }
    }

    if (this.pathIndex >= this.path.length) {
      return { status: PathStatus.COMPLETE, node: to };
    }

    const nextNode = this.path[this.pathIndex];
    this.pathIndex++;

    return { status: PathStatus.NEXT, node: nextNode };
  }

  findPath(from: TileRef, to: TileRef): TileRef[] | null {
    return this.aStar.search(from, to);
  }

  private cachePath(from: TileRef, to: TileRef): boolean {
    try {
      this.path = this.aStar.search(from, to);
    } catch {
      return false;
    }

    if (this.path === null) {
      return false;
    }

    this.pathIndex = 0;

    // Skip first node if it's the current position
    if (this.path.length > 0 && this.path[0] === from) {
      this.pathIndex = 1;
    }

    return true;
  }
}

import { Game } from "../game/Game";
import { TileRef } from "../game/GameMap";
import { PseudoRandom } from "../PseudoRandom";
import { PathResult, PathStatus, SteppingPathFinder } from "./types";

export class AirPathFinder implements SteppingPathFinder<TileRef> {
  private random: PseudoRandom;
  private seed: number;

  constructor(private game: Game) {
    this.seed = game.ticks();
    this.random = new PseudoRandom(this.seed);
  }

  findPath(from: TileRef | TileRef[], to: TileRef): TileRef[] | null {
    if (Array.isArray(from)) {
      throw new Error("AirPathFinder does not support multiple start points");
    }

    const random = new PseudoRandom(this.seed);
    const path: TileRef[] = [from];
    let current = from;

    while (true) {
      const result = this.computeNext(current, to, random);

      if (result.status === PathStatus.COMPLETE) {
        break;
      }

      if (result.status === PathStatus.NEXT) {
        current = result.node;
        path.push(current);
      }
    }

    return path;
  }

  next(from: TileRef, to: TileRef, _dist?: number): PathResult<TileRef> {
    return this.computeNext(from, to, this.random);
  }

  invalidate(): void {}

  private computeNext(
    from: TileRef,
    to: TileRef,
    random: PseudoRandom,
  ): PathResult<TileRef> {
    const x = this.game.x(from);
    const y = this.game.y(from);
    const dstX = this.game.x(to);
    const dstY = this.game.y(to);

    if (x === dstX && y === dstY) {
      return { status: PathStatus.COMPLETE, node: to };
    }

    let nextX = x;
    let nextY = y;
    const ratio = Math.floor(1 + Math.abs(dstY - y) / (Math.abs(dstX - x) + 1));

    if (random.chance(ratio) && x !== dstX) {
      nextX += x < dstX ? 1 : -1;
    } else {
      nextY += y < dstY ? 1 : -1;
    }

    if (nextX === x && nextY === y) {
      return { status: PathStatus.COMPLETE, node: to };
    }

    return { status: PathStatus.NEXT, node: this.game.ref(nextX, nextY) };
  }
}

import { Game } from "../game/Game";
import { GameMap, TileRef } from "../game/GameMap";
import { StationManager } from "../game/RailNetworkImpl";
import { TrainStation } from "../game/TrainStation";
import { PseudoRandom } from "../PseudoRandom";
import { within } from "../Util";
import { DistanceBasedBezierCurve } from "../utilities/Line";
import { GenericAStar } from "./algorithms/AStar";
import { MiniAStar } from "./algorithms/AStarMini";
import { RailAdapter } from "./algorithms/AStarRailAdapter";
import { ShoreCoercingAStar } from "./algorithms/AStarShoreCoercing";
import { StationGraphAdapter } from "./algorithms/AStarStationAdapter";
import { GameMapAStar } from "./algorithms/AStarWaterAdapter";
import { TilePathFinder } from "./TilePathFinder";

export enum PathStatus {
  NEXT,
  PENDING,
  COMPLETE,
  NOT_FOUND,
}

export type PathResult<T> =
  | { status: PathStatus.PENDING }
  | { status: PathStatus.NEXT; node: T }
  | { status: PathStatus.COMPLETE; node: T }
  | { status: PathStatus.NOT_FOUND };

export interface PathFinder<T> {
  findPath(from: T | T[], to: T): T[] | null;
  next(from: T, to: T, dist?: number): PathResult<T>;
  invalidate(): void;
}

export interface RailOptions {
  waterPenalty?: number;
  directionChangePenalty?: number;
}

class StationPathFinder implements PathFinder<TrainStation> {
  private path: TrainStation[] | null = null;
  private pathIndex = 0;
  private lastTo: TrainStation | null = null;
  private manager: StationManager;

  constructor(
    private game: Game,
    private aStar: GenericAStar,
  ) {
    this.manager = game.railNetwork().stationManager();
  }

  findPath(
    from: TrainStation | TrainStation[],
    to: TrainStation,
  ): TrainStation[] | null {
    // Cluster early-exit: stations in different clusters have no path
    const toCluster = to.getCluster();
    const fromArray = Array.isArray(from) ? from : [from];
    const sameCluster = fromArray.filter((s) => s.getCluster() === toCluster);
    if (sameCluster.length === 0) return null;

    const fromIds = sameCluster.map((s) => s.id);
    const path = this.aStar.search(
      fromIds.length === 1 ? fromIds[0] : fromIds,
      to.id,
    );

    if (!path) return null;
    return path.map((id) => this.manager.getById(id)!);
  }

  next(
    from: TrainStation,
    to: TrainStation,
    dist?: number,
  ): PathResult<TrainStation> {
    if (from.id === to.id) {
      return { status: PathStatus.COMPLETE, node: to };
    }

    // Cluster early-exit
    if (from.getCluster() !== to.getCluster()) {
      return { status: PathStatus.NOT_FOUND };
    }

    if (dist !== undefined) {
      const fx = this.game.x(from.tile()),
        fy = this.game.y(from.tile());
      const tx = this.game.x(to.tile()),
        ty = this.game.y(to.tile());
      if (Math.abs(fx - tx) + Math.abs(fy - ty) <= dist) {
        return { status: PathStatus.COMPLETE, node: from };
      }
    }

    if (this.lastTo !== to) {
      this.path = null;
      this.pathIndex = 0;
      this.lastTo = to;
    }

    if (!this.path) {
      this.path = this.findPath(from, to);
      if (!this.path) return { status: PathStatus.NOT_FOUND };
      this.pathIndex = this.path[0]?.id === from.id ? 1 : 0;
    }

    if (this.pathIndex >= this.path.length) {
      return { status: PathStatus.COMPLETE, node: to };
    }

    return { status: PathStatus.NEXT, node: this.path[this.pathIndex++] };
  }

  invalidate(): void {
    this.path = null;
    this.pathIndex = 0;
    this.lastTo = null;
  }
}

class AirPathFinder implements PathFinder<TileRef> {
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

export interface ParabolaOptions {
  increment?: number;
  distanceBasedHeight?: boolean;
  directionUp?: boolean;
}

const PARABOLA_MIN_HEIGHT = 50;

export class ParabolaUniversalPathFinder implements PathFinder<TileRef> {
  private curve: DistanceBasedBezierCurve | null = null;
  private lastTo: TileRef | null = null;

  constructor(
    private gameMap: GameMap,
    private options?: ParabolaOptions,
  ) {}

  private createCurve(from: TileRef, to: TileRef): DistanceBasedBezierCurve {
    const increment = this.options?.increment ?? 3;
    const distanceBasedHeight = this.options?.distanceBasedHeight ?? true;
    const directionUp = this.options?.directionUp ?? true;

    const p0 = { x: this.gameMap.x(from), y: this.gameMap.y(from) };
    const p3 = { x: this.gameMap.x(to), y: this.gameMap.y(to) };
    const dx = p3.x - p0.x;
    const dy = p3.y - p0.y;
    const distance = Math.sqrt(dx * dx + dy * dy);
    const maxHeight = distanceBasedHeight
      ? Math.max(distance / 3, PARABOLA_MIN_HEIGHT)
      : 0;
    const heightMult = directionUp ? -1 : 1;
    const mapHeight = this.gameMap.height();

    const p1 = {
      x: p0.x + dx / 4,
      y: within(p0.y + dy / 4 + heightMult * maxHeight, 0, mapHeight - 1),
    };
    const p2 = {
      x: p0.x + (dx * 3) / 4,
      y: within(p0.y + (dy * 3) / 4 + heightMult * maxHeight, 0, mapHeight - 1),
    };

    return new DistanceBasedBezierCurve(p0, p1, p2, p3, increment);
  }

  findPath(from: TileRef | TileRef[], to: TileRef): TileRef[] | null {
    if (Array.isArray(from)) {
      throw new Error(
        "ParabolaUniversalPathFinder does not support multiple start points",
      );
    }
    const curve = this.createCurve(from, to);
    return curve
      .getAllPoints()
      .map((p) => this.gameMap.ref(Math.floor(p.x), Math.floor(p.y)));
  }

  next(from: TileRef, to: TileRef, speed?: number): PathResult<TileRef> {
    if (this.lastTo !== to) {
      this.curve = this.createCurve(from, to);
      this.lastTo = to;
    }

    const nextPoint = this.curve!.increment(speed ?? 1);
    if (!nextPoint) {
      return { status: PathStatus.COMPLETE, node: to };
    }
    const tile = this.gameMap.ref(
      Math.floor(nextPoint.x),
      Math.floor(nextPoint.y),
    );
    return { status: PathStatus.NEXT, node: tile };
  }

  invalidate(): void {
    this.curve = null;
    this.lastTo = null;
  }

  currentIndex(): number {
    return this.curve?.getCurrentIndex() ?? 0;
  }
}

/**
 * Pathfinders that work with GameMap - usable in both simulation and UI layers
 */
export class UniversalPathFinding {
  static Parabola(
    gameMap: GameMap,
    options?: ParabolaOptions,
  ): ParabolaUniversalPathFinder {
    return new ParabolaUniversalPathFinder(gameMap, options);
  }
}

/**
 * Pathfinders that require Game - simulation layer only
 */
export class PathFinding {
  static Water(game: Game): PathFinder<TileRef> {
    const hpa = game.waterPathfinder();

    if (!hpa) {
      return PathFinding.WaterFallback(game);
    }

    // Wrap HPA* with shore coercing to handle shore tiles
    return new TilePathFinder(game, new ShoreCoercingAStar(game.map(), hpa));
  }

  static WaterFallback(game: Game): PathFinder<TileRef> {
    return new TilePathFinder(
      game,
      new ShoreCoercingAStar(
        game.map(),
        new MiniAStar(game, (map) => new GameMapAStar(map)),
      ),
    );
  }

  static Rail(game: Game, options?: RailOptions): PathFinder<TileRef> {
    return new TilePathFinder(
      game,
      new MiniAStar(
        game,
        (map) =>
          new GenericAStar({
            adapter: new RailAdapter(map, {
              waterPenalty: options?.waterPenalty,
              directionChangePenalty: options?.directionChangePenalty,
            }),
          }),
      ),
    );
  }

  static Stations(game: Game): PathFinder<TrainStation> {
    const adapter = new StationGraphAdapter(game);
    const aStar = new GenericAStar({ adapter });
    return new StationPathFinder(game, aStar);
  }

  static Air(game: Game): PathFinder<TileRef> {
    return new AirPathFinder(game);
  }

  /** @deprecated Use UniversalPathFinding.Parabola for UI layer compatibility */
  static Parabola(
    game: Game,
    options?: ParabolaOptions,
  ): ParabolaUniversalPathFinder {
    return new ParabolaUniversalPathFinder(game, options);
  }
}

// Legacy export for backwards compatibility
export const PathFinders = PathFinding;

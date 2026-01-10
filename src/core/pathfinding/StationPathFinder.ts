import { Game } from "../game/Game";
import { StationManager } from "../game/RailNetworkImpl";
import { TrainStation } from "../game/TrainStation";
import { GenericAStar } from "./algorithms/AStar";
import { PathResult, PathStatus, SteppingPathFinder } from "./types";

export class StationPathFinder implements SteppingPathFinder<TrainStation> {
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
    const path = this.aStar.findPath(
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

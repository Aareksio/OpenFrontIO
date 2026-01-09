// Station graph A* adapter for GenericAStar

import { Game } from "../../game/Game";
import { StationManager } from "../../game/RailNetworkImpl";
import { GenericAStarAdapter } from "./AStar";

export class StationGraphAdapter implements GenericAStarAdapter {
  private manager: StationManager;

  constructor(private game: Game) {
    this.manager = game.railNetwork().stationManager();
  }

  numNodes(): number {
    return this.manager.count();
  }

  maxNeighbors(): number {
    return 8;
  }

  maxPriority(): number {
    return this.game.map().width() + this.game.map().height();
  }

  neighbors(node: number, buffer: Int32Array): number {
    const station = this.manager.getById(node);
    if (!station) return 0;

    let count = 0;
    for (const n of station.neighbors()) {
      buffer[count++] = n.id;
    }
    return count;
  }

  cost(): number {
    return 1;
  }

  heuristic(node: number, goal: number): number {
    const a = this.manager.getById(node);
    const b = this.manager.getById(goal);
    if (!a || !b) return 0;

    const ax = this.game.x(a.tile());
    const ay = this.game.y(a.tile());
    const bx = this.game.x(b.tile());
    const by = this.game.y(b.tile());
    return Math.abs(ax - bx) + Math.abs(ay - by);
  }
}

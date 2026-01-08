// Adapter for BeamSearch implementing PathFinder interface

import { Cell, Game } from "../../game/Game";
import { GameMap, TileRef } from "../../game/GameMap";
import { PathFindResultType } from "../AStar";
import { PathFinder, PathResult, PathStatus } from "../PathFinder";
import { GraphAdapter } from "../SerialAStar";
import { BeamSearch } from "./BeamSearch";
import { fixExtremes, upscalePath } from "./PathUpscaler";

export interface BeamSearchOptions {
  beamWidth?: number;
  iterations?: number;
  maxTries?: number;
}

const DEFAULT_BEAM_WIDTH = 2000;
const DEFAULT_ITERATIONS = 500_000;
const DEFAULT_MAX_TRIES = 50;

class GameMapGraphAdapter implements GraphAdapter<number> {
  constructor(
    private gameMap: GameMap,
    private width: number,
  ) {}

  neighbors(node: number): number[] {
    return this.gameMap.neighbors(node as TileRef) as number[];
  }

  cost(node: number): number {
    return this.gameMap.cost(node as TileRef);
  }

  position(node: number): { x: number; y: number } {
    return {
      x: this.gameMap.x(node as TileRef),
      y: this.gameMap.y(node as TileRef),
    };
  }

  isTraversable(_from: number, to: number): boolean {
    return this.gameMap.isWater(to as TileRef);
  }
}

export class BeamSearchAdapter implements PathFinder {
  private game: Game;
  private graphAdapter: GameMapGraphAdapter;
  private beamSearch: BeamSearch;

  constructor(game: Game, options?: BeamSearchOptions) {
    this.game = game;
    const miniMap = game.miniMap();
    const width = miniMap.width();
    this.graphAdapter = new GameMapGraphAdapter(miniMap, width);
    const numNodes = width * miniMap.height();

    this.beamSearch = new BeamSearch(
      this.graphAdapter,
      numNodes,
      width,
      options?.beamWidth ?? DEFAULT_BEAM_WIDTH,
      options?.iterations ?? DEFAULT_ITERATIONS,
      options?.maxTries ?? DEFAULT_MAX_TRIES,
    );
  }

  next(from: TileRef, to: TileRef, dist?: number): PathResult {
    const path = this.findPath(from, to);
    if (!path || path.length === 0) {
      return { status: PathStatus.NOT_FOUND };
    }

    const targetDist = dist ?? 1;
    if (this.game.manhattanDist(from, to) < targetDist) {
      return { status: PathStatus.COMPLETE, node: from };
    }

    if (path.length > 1) {
      return { status: PathStatus.NEXT, node: path[1] };
    }

    return { status: PathStatus.COMPLETE, node: path[0] };
  }

  findPath(from: TileRef, to: TileRef): TileRef[] | null {
    const gameMap = this.game.map();
    const miniMap = this.game.miniMap();

    const miniFrom = miniMap.ref(
      Math.floor(gameMap.x(from) / 2),
      Math.floor(gameMap.y(from) / 2),
    );
    const miniTo = miniMap.ref(
      Math.floor(gameMap.x(to) / 2),
      Math.floor(gameMap.y(to) / 2),
    );

    this.beamSearch.reset(miniFrom as number, miniTo as number);

    let result: PathFindResultType;
    do {
      result = this.beamSearch.compute();
    } while (result === PathFindResultType.Pending);

    if (result === PathFindResultType.PathNotFound) {
      return null;
    }

    const miniPath = this.beamSearch.reconstructPath();
    if (miniPath.length === 0) {
      return null;
    }

    const cellPath = miniPath.map(
      (ref) => new Cell(miniMap.x(ref as TileRef), miniMap.y(ref as TileRef)),
    );

    const cellFrom = new Cell(gameMap.x(from), gameMap.y(from));
    const cellTo = new Cell(gameMap.x(to), gameMap.y(to));
    const upscaled = fixExtremes(upscalePath(cellPath), cellTo, cellFrom);

    return upscaled.map((c) => gameMap.ref(c.x, c.y));
  }
}

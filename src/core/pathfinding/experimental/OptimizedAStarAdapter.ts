// Adapter for OptimizedAStar variants implementing PathFinder interface

import { Cell, Game } from "../../game/Game";
import { GameMap, TileRef } from "../../game/GameMap";
import { PathFindResultType } from "../AStar";
import { PathFinder, PathResult, PathStatus } from "../PathFinder";
import { GraphAdapter } from "../SerialAStar";
import { OptimizedAStar } from "./OptimizedAStar";

export interface OptimizedAStarOptions {
  iterations?: number;
  maxTries?: number;
}

const DEFAULT_ITERATIONS = 500_000;
const DEFAULT_MAX_TRIES = 50;

// Adapter for GameMap to work with OptimizedAStar
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

export class OptimizedAStarAdapter implements PathFinder {
  private game: Game;
  private graphAdapter: GameMapGraphAdapter;
  private aStar: OptimizedAStar;

  constructor(game: Game, options?: OptimizedAStarOptions) {
    this.game = game;
    const miniMap = game.miniMap();
    const width = miniMap.width();
    this.graphAdapter = new GameMapGraphAdapter(miniMap, width);
    const numNodes = width * miniMap.height();

    // Create pooled A* instance
    this.aStar = new OptimizedAStar(
      this.graphAdapter,
      numNodes,
      width,
      options?.iterations ?? DEFAULT_ITERATIONS,
      options?.maxTries ?? DEFAULT_MAX_TRIES,
    );
  }

  next(from: TileRef, to: TileRef, dist?: number): PathResult {
    // Simple implementation - compute full path and return first step
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

    // Downscale to minimap coordinates
    const miniFrom = miniMap.ref(
      Math.floor(gameMap.x(from) / 2),
      Math.floor(gameMap.y(from) / 2),
    );
    const miniTo = miniMap.ref(
      Math.floor(gameMap.x(to) / 2),
      Math.floor(gameMap.y(to) / 2),
    );

    // Reset pooled instance for new search
    this.aStar.reset(miniFrom as number, miniTo as number);

    // Run to completion
    let result: PathFindResultType;
    do {
      result = this.aStar.compute();
    } while (result === PathFindResultType.Pending);

    if (result === PathFindResultType.PathNotFound) {
      return null;
    }

    // Get minimap path and upscale
    const miniPath = this.aStar.reconstructPath();
    if (miniPath.length === 0) {
      return null;
    }

    // Convert to cells for upscaling
    const cellPath = miniPath.map(
      (ref) => new Cell(miniMap.x(ref as TileRef), miniMap.y(ref as TileRef)),
    );

    // Upscale and fix extremes
    const cellFrom = new Cell(gameMap.x(from), gameMap.y(from));
    const cellTo = new Cell(gameMap.x(to), gameMap.y(to));
    const upscaled = fixExtremes(upscalePath(cellPath), cellTo, cellFrom);

    // Convert back to TileRefs
    return upscaled.map((c) => gameMap.ref(c.x, c.y));
  }
}

// Path upscaling helpers (copied from MiniAStar.ts)
function upscalePath(path: Cell[], scaleFactor: number = 2): Cell[] {
  const scaledPath = path.map(
    (point) => new Cell(point.x * scaleFactor, point.y * scaleFactor),
  );

  const smoothPath: Cell[] = [];

  for (let i = 0; i < scaledPath.length - 1; i++) {
    const current = scaledPath[i];
    const next = scaledPath[i + 1];

    smoothPath.push(current);

    const dx = next.x - current.x;
    const dy = next.y - current.y;
    const distance = Math.max(Math.abs(dx), Math.abs(dy));
    const steps = distance;

    for (let step = 1; step < steps; step++) {
      smoothPath.push(
        new Cell(
          Math.round(current.x + (dx * step) / steps),
          Math.round(current.y + (dy * step) / steps),
        ),
      );
    }
  }

  if (scaledPath.length > 0) {
    smoothPath.push(scaledPath[scaledPath.length - 1]);
  }

  return smoothPath;
}

function fixExtremes(upscaled: Cell[], cellDst: Cell, cellSrc?: Cell): Cell[] {
  if (cellSrc !== undefined) {
    const srcIndex = findCell(upscaled, cellSrc);
    if (srcIndex === -1) {
      upscaled.unshift(cellSrc);
    } else if (srcIndex !== 0) {
      upscaled = upscaled.slice(srcIndex);
    }
  }

  const dstIndex = findCell(upscaled, cellDst);
  if (dstIndex === -1) {
    upscaled.push(cellDst);
  } else if (dstIndex !== upscaled.length - 1) {
    upscaled = upscaled.slice(0, dstIndex + 1);
  }
  return upscaled;
}

function findCell(cells: Cell[], target: Cell): number {
  for (let i = 0; i < cells.length; i++) {
    if (cells[i].x === target.x && cells[i].y === target.y) {
      return i;
    }
  }
  return -1;
}

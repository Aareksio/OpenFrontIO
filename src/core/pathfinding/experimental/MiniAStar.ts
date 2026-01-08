// Minimap A* wrapper with coordinate conversion and path upscaling

import { Cell, Game } from "../../game/Game";
import { GameMap, TileRef } from "../../game/GameMap";
import { AStar } from "./AStar";

export type AStarFactory = (map: GameMap) => AStar;

export class MiniAStar implements AStar {
  private innerAStar: AStar;

  constructor(
    private game: Game,
    factory: AStarFactory,
  ) {
    this.innerAStar = factory(game.miniMap());
  }

  search(from: TileRef, to: TileRef): TileRef[] | null {
    const gameMap = this.game.map();
    const miniMap = this.game.miniMap();

    // Convert game coords → minimap coords
    const miniFrom = miniMap.ref(
      Math.floor(gameMap.x(from) / 2),
      Math.floor(gameMap.y(from) / 2),
    );
    const miniTo = miniMap.ref(
      Math.floor(gameMap.x(to) / 2),
      Math.floor(gameMap.y(to) / 2),
    );

    // Search on minimap
    const path = this.innerAStar.search(miniFrom, miniTo);
    if (!path || path.length === 0) {
      return null;
    }

    // Convert minimap TileRefs → Cells
    const cellPath = path.map(
      (ref) => new Cell(miniMap.x(ref), miniMap.y(ref)),
    );

    // Upscale and fix extremes
    const cellFrom = new Cell(gameMap.x(from), gameMap.y(from));
    const cellTo = new Cell(gameMap.x(to), gameMap.y(to));
    const upscaled = fixExtremes(upscalePath(cellPath), cellTo, cellFrom);

    // Convert back to game TileRefs
    return upscaled.map((c) => gameMap.ref(c.x, c.y));
  }
}

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

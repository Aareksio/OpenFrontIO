// Minimap transformer with coordinate conversion and path upscaling

import { Cell, Game } from "../../game/Game";
import { TileRef } from "../../game/GameMap";
import { PathFinder } from "../types";

export class MiniMapTransformer implements PathFinder<number> {
  constructor(
    private inner: PathFinder<number>,
    private game: Game,
  ) {}

  findPath(from: TileRef | TileRef[], to: TileRef): TileRef[] | null {
    const gameMap = this.game.map();
    const miniMap = this.game.miniMap();

    // Convert game coords → minimap coords (supports multi-source)
    const fromArray = Array.isArray(from) ? from : [from];
    const miniFromArray = fromArray.map((f) =>
      miniMap.ref(Math.floor(gameMap.x(f) / 2), Math.floor(gameMap.y(f) / 2)),
    );
    const miniFrom =
      miniFromArray.length === 1 ? miniFromArray[0] : miniFromArray;

    const miniTo = miniMap.ref(
      Math.floor(gameMap.x(to) / 2),
      Math.floor(gameMap.y(to) / 2),
    );

    // Search on minimap
    const path = this.inner.findPath(miniFrom, miniTo);
    if (!path || path.length === 0) {
      return null;
    }

    // Convert minimap TileRefs → Cells
    const cellPath = path.map(
      (ref) => new Cell(miniMap.x(ref), miniMap.y(ref)),
    );

    // Upscale and fix extremes
    // For multi-source, find closest source to path start
    const upscaledPath = this.upscalePath(cellPath);
    let cellFrom: Cell | undefined;
    if (Array.isArray(from)) {
      if (upscaledPath.length > 0) {
        const pathStart = upscaledPath[0];
        let minDist = Infinity;
        for (const f of from) {
          const fx = gameMap.x(f);
          const fy = gameMap.y(f);
          const dist = Math.abs(fx - pathStart.x) + Math.abs(fy - pathStart.y);
          if (dist < minDist) {
            minDist = dist;
            cellFrom = new Cell(fx, fy);
          }
        }
      }
    } else {
      cellFrom = new Cell(gameMap.x(from), gameMap.y(from));
    }
    const cellTo = new Cell(gameMap.x(to), gameMap.y(to));
    const upscaled = this.fixExtremes(upscaledPath, cellTo, cellFrom);

    // Convert back to game TileRefs
    return upscaled.map((c) => gameMap.ref(c.x, c.y));
  }

  private upscalePath(path: Cell[], scaleFactor: number = 2): Cell[] {
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

  private fixExtremes(upscaled: Cell[], cellDst: Cell, cellSrc?: Cell): Cell[] {
    if (cellSrc !== undefined) {
      const srcIndex = this.findCell(upscaled, cellSrc);
      if (srcIndex === -1) {
        upscaled.unshift(cellSrc);
      } else if (srcIndex !== 0) {
        upscaled = upscaled.slice(srcIndex);
      }
    }

    const dstIndex = this.findCell(upscaled, cellDst);
    if (dstIndex === -1) {
      upscaled.push(cellDst);
    } else if (dstIndex !== upscaled.length - 1) {
      upscaled = upscaled.slice(0, dstIndex + 1);
    }
    return upscaled;
  }

  private findCell(cells: Cell[], target: Cell): number {
    for (let i = 0; i < cells.length; i++) {
      if (cells[i].x === target.x && cells[i].y === target.y) {
        return i;
      }
    }
    return -1;
  }
}

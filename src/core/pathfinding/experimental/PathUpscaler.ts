// Shared utilities for upscaling paths from mini-map to full resolution

import { Cell } from "../../game/Game";

export function upscalePath(path: Cell[], scaleFactor: number = 2): Cell[] {
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

export function fixExtremes(
  upscaled: Cell[],
  cellDst: Cell,
  cellSrc?: Cell,
): Cell[] {
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

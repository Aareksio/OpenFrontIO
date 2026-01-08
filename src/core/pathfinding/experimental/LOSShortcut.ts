// LOS Shortcut: Check line-of-sight before A*, skip search if clear
// Falls back to OptimizedAStar when LOS blocked

import { PathFindResultType } from "../AStar";
import { GraphAdapter } from "../SerialAStar";
import { OptimizedAStar } from "./OptimizedAStar";

export class LOSShortcut {
  private readonly aStar: OptimizedAStar;
  private readonly width: number;
  private readonly graph: GraphAdapter<number>;

  public completed = false;
  private foundPath: number[] | null = null;
  private usedLOS = false;

  constructor(
    graph: GraphAdapter<number>,
    numNodes: number,
    width: number,
    iterations: number = 500_000,
    maxTries: number = 50,
  ) {
    this.graph = graph;
    this.width = width;
    this.aStar = new OptimizedAStar(graph, numNodes, width, iterations, maxTries);
  }

  reset(src: number | number[], dst: number): void {
    this.completed = false;
    this.foundPath = null;
    this.usedLOS = false;

    // For single source, try LOS first
    const sources = Array.isArray(src) ? src : [src];
    if (sources.length === 1) {
      const losPath = this.checkLOS(sources[0], dst);
      if (losPath !== null) {
        this.completed = true;
        this.foundPath = losPath;
        this.usedLOS = true;
        return;
      }
    }

    // LOS failed or multi-source, use A*
    this.aStar.reset(src, dst);
  }

  private checkLOS(src: number, dst: number): number[] | null {
    const width = this.width;

    const x0 = src % width;
    const y0 = (src / width) | 0;
    const x1 = dst % width;
    const y1 = (dst / width) | 0;

    // Try horizontal-first L-path
    const hPath = this.tryLPath(x0, y0, x1, y1, true, src, dst);
    if (hPath !== null) return hPath;

    // Try vertical-first L-path
    return this.tryLPath(x0, y0, x1, y1, false, src, dst);
  }

  private tryLPath(
    x0: number,
    y0: number,
    x1: number,
    y1: number,
    horizontalFirst: boolean,
    src: number,
    dst: number,
  ): number[] | null {
    const width = this.width;
    const path: number[] = [];

    let cx = x0;
    let cy = y0;

    const sx = x0 < x1 ? 1 : -1;
    const sy = y0 < y1 ? 1 : -1;

    if (horizontalFirst) {
      // Horizontal leg
      while (cx !== x1) {
        const node = cy * width + cx;
        if (node !== src && !this.graph.isTraversable(node, node)) {
          return null;
        }
        path.push(node);
        cx += sx;
      }
      // Vertical leg
      while (cy !== y1) {
        const node = cy * width + cx;
        if (node !== src && !this.graph.isTraversable(node, node)) {
          return null;
        }
        path.push(node);
        cy += sy;
      }
    } else {
      // Vertical leg
      while (cy !== y1) {
        const node = cy * width + cx;
        if (node !== src && !this.graph.isTraversable(node, node)) {
          return null;
        }
        path.push(node);
        cy += sy;
      }
      // Horizontal leg
      while (cx !== x1) {
        const node = cy * width + cx;
        if (node !== src && !this.graph.isTraversable(node, node)) {
          return null;
        }
        path.push(node);
        cx += sx;
      }
    }

    // Add destination (may be non-water port, so skip traversability check)
    path.push(dst);
    return path;
  }

  compute(): PathFindResultType {
    if (this.completed) return PathFindResultType.Completed;
    return this.aStar.compute();
  }

  reconstructPath(): number[] {
    if (this.usedLOS) {
      return this.foundPath ?? [];
    }
    return this.aStar.reconstructPath();
  }

  /** Returns true if last path was found via LOS (no A* needed) */
  wasLOSHit(): boolean {
    return this.usedLOS;
  }
}

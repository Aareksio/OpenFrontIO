// NavMesh with optimized inlined A* implementations
// Uses BoundedAStar and GatewayAStar instead of FastAStar + adapters

import { Game } from "../../game/Game";
import { TileRef } from "../../game/GameMap";
import { FastBFS } from "../navmesh/FastBFS";
import { Gateway, GatewayGraph, GatewayGraphBuilder } from "../navmesh/GatewayGraph";
import { BoundedAStar } from "./BoundedAStar";
import { GatewayAStar } from "./GatewayAStar";

type PathDebugInfo = {
  gatewayPath: TileRef[] | null;
  initialPath: TileRef[] | null;
  smoothPath: TileRef[] | null;
  graph: {
    sectorSize: number;
    gateways: Array<{ id: number; tile: TileRef }>;
    edges: Array<{
      fromId: number;
      toId: number;
      from: TileRef;
      to: TileRef;
      cost: number;
      path: TileRef[] | null;
    }>;
  };
  timings: { [key: string]: number };
};

export class NavMeshOptimized {
  private graph!: GatewayGraph;
  private initialized = false;
  private fastBFS!: FastBFS;
  private gatewayAStar!: GatewayAStar;
  private localAStar!: BoundedAStar;
  private localAStarMultiSector!: BoundedAStar;

  public debugInfo: PathDebugInfo | null = null;

  constructor(
    private game: Game,
    private options: {
      cachePaths?: boolean;
    } = {},
  ) {}

  initialize(debug: boolean = false) {
    const gatewayGraphBuilder = new GatewayGraphBuilder(
      this.game,
      GatewayGraphBuilder.SECTOR_SIZE,
    );
    this.graph = gatewayGraphBuilder.build(debug);

    const miniMap = this.game.miniMap();
    this.fastBFS = new FastBFS(miniMap.width() * miniMap.height());

    // Use GatewayAStar for gateway graph routing
    this.gatewayAStar = new GatewayAStar(this.graph, {
      heuristicWeight: 1,
      maxIterations: 100_000,
    });

    // BoundedAStar for sector-bounded local pathfinding
    const sectorSize = GatewayGraphBuilder.SECTOR_SIZE;

    // Single sector: 32×32 = 1,024 nodes
    const maxLocalNodes = sectorSize * sectorSize;
    this.localAStar = new BoundedAStar(miniMap, maxLocalNodes, {
      heuristicWeight: 1,
      maxIterations: 10_000,
    });

    // Multi-sector: 3×3 sectors = 96×96 = 9,216 nodes
    const multiSectorSize = sectorSize * 3;
    const maxMultiSectorNodes = multiSectorSize * multiSectorSize;
    this.localAStarMultiSector = new BoundedAStar(miniMap, maxMultiSectorNodes, {
      heuristicWeight: 1,
      maxIterations: 10_000,
    });

    this.initialized = true;
  }

  findPath(
    from: TileRef,
    to: TileRef,
    debug: boolean = false,
  ): TileRef[] | null {
    if (!this.initialized) {
      throw new Error(
        "NavMesh not initialized. Call initialize() before using findPath().",
      );
    }

    if (debug) {
      const allEdges: Array<{
        fromId: number;
        toId: number;
        from: TileRef;
        to: TileRef;
        cost: number;
        path: TileRef[] | null;
      }> = [];

      for (const [fromId, edges] of this.graph.edges.entries()) {
        const fromGw = this.graph.getGateway(fromId);
        if (!fromGw) continue;

        for (const edge of edges) {
          const toGw = this.graph.getGateway(edge.to);
          if (!toGw) continue;

          if (fromId <= edge.to) {
            allEdges.push({
              fromId: fromId,
              toId: edge.to,
              from: fromGw.tile,
              to: toGw.tile,
              cost: edge.cost,
              path: edge.path ?? null,
            });
          }
        }
      }

      this.debugInfo = {
        gatewayPath: null,
        initialPath: null,
        smoothPath: null,
        graph: {
          sectorSize: this.graph.sectorSize,
          gateways: this.graph
            .getAllGateways()
            .map((gw) => ({ id: gw.id, tile: gw.tile })),
          edges: allEdges,
        },
        timings: {
          total: 0,
        },
      };
    }

    const dist = this.game.manhattanDist(from, to);

    // Early exit for very short distances
    if (dist <= this.graph.sectorSize) {
      performance.mark("navsat:findPath:earlyExitLocalPath:start");
      const map = this.game.map();
      const startMiniX = Math.floor(map.x(from) / 2);
      const startMiniY = Math.floor(map.y(from) / 2);
      const sectorX = Math.floor(startMiniX / this.graph.sectorSize);
      const sectorY = Math.floor(startMiniY / this.graph.sectorSize);
      const localPath = this.findLocalPath(
        from,
        to,
        sectorX,
        sectorY,
        true,
      );
      performance.mark("navsat:findPath:earlyExitLocalPath:end");
      const measure = performance.measure(
        "navsat:findPath:earlyExitLocalPath",
        "navsat:findPath:earlyExitLocalPath:start",
        "navsat:findPath:earlyExitLocalPath:end",
      );

      if (debug) {
        this.debugInfo!.timings.earlyExitLocalPath = measure.duration;
        this.debugInfo!.timings.total += measure.duration;
      }

      if (localPath) {
        if (debug) {
          console.log(
            `[DEBUG] Direct local path found for dist=${dist}, length=${localPath.length}`,
          );
        }

        return localPath;
      }

      if (debug) {
        console.log(
          `[DEBUG] Direct path failed for dist=${dist}, falling back to gateway graph`,
        );
      }
    }

    performance.mark("navsat:findPath:findGateways:start");
    const startGateway = this.findNearestGateway(from);
    const endGateway = this.findNearestGateway(to);
    performance.mark("navsat:findPath:findGateways:end");
    const findGatewaysMeasure = performance.measure(
      "navsat:findPath:findGateways",
      "navsat:findPath:findGateways:start",
      "navsat:findPath:findGateways:end",
    );

    if (debug) {
      this.debugInfo!.timings.findGateways = findGatewaysMeasure.duration;
      this.debugInfo!.timings.total += findGatewaysMeasure.duration;
    }

    if (!startGateway) {
      if (debug) {
        console.log(
          `[DEBUG] Cannot find start gateway for (${this.game.x(from)}, ${this.game.y(from)})`,
        );
      }

      return null;
    }

    if (!endGateway) {
      if (debug) {
        console.log(
          `[DEBUG] Cannot find end gateway for (${this.game.x(to)}, ${this.game.y(to)})`,
        );
      }

      return null;
    }

    if (startGateway.id === endGateway.id) {
      if (debug) {
        console.log(
          `[DEBUG] Start and end gateways are the same (ID=${startGateway.id}), finding local path with multi-sector search`,
        );
      }

      performance.mark("navsat:findPath:sameGatewayLocalPath:start");
      const sectorX = Math.floor(startGateway.x / this.graph.sectorSize);
      const sectorY = Math.floor(startGateway.y / this.graph.sectorSize);
      const path = this.findLocalPath(from, to, sectorX, sectorY, true);
      performance.mark("navsat:findPath:sameGatewayLocalPath:end");
      const sameGatewayMeasure = performance.measure(
        "navsat:findPath:sameGatewayLocalPath",
        "navsat:findPath:sameGatewayLocalPath:start",
        "navsat:findPath:sameGatewayLocalPath:end",
      );

      if (debug) {
        this.debugInfo!.timings.sameGatewayLocalPath =
          sameGatewayMeasure.duration;
        this.debugInfo!.timings.total += sameGatewayMeasure.duration;
      }

      return path;
    }

    performance.mark("navsat:findPath:findGatewayPath:start");
    const gatewayPath = this.findGatewayPath(startGateway.id, endGateway.id);
    performance.mark("navsat:findPath:findGatewayPath:end");
    const findGatewayPathMeasure = performance.measure(
      "navsat:findPath:findGatewayPath",
      "navsat:findPath:findGatewayPath:start",
      "navsat:findPath:findGatewayPath:end",
    );

    if (debug) {
      this.debugInfo!.timings.findGatewayPath = findGatewayPathMeasure.duration;
      this.debugInfo!.timings.total += findGatewayPathMeasure.duration;

      this.debugInfo!.gatewayPath = gatewayPath
        ? gatewayPath
            .map((gwId) => {
              const gw = this.graph.getGateway(gwId);
              return gw ? gw.tile : -1;
            })
            .filter((tile) => tile !== -1)
        : null;
    }

    if (!gatewayPath) {
      if (debug) {
        console.log(
          `[DEBUG] No gateway path between gateways ${startGateway.id} and ${endGateway.id}`,
        );
      }

      return null;
    }

    if (debug) {
      console.log(
        `[DEBUG] Gateway path found: ${gatewayPath.length} waypoints`,
      );
    }

    const initialPath: TileRef[] = [];
    const map = this.game.map();
    const miniMap = this.game.miniMap();

    performance.mark("navsat:findPath:buildInitialPath:start");

    // 1. Find path from start to first gateway
    const firstGateway = this.graph.getGateway(gatewayPath[0])!;
    const firstGatewayTile = map.ref(
      miniMap.x(firstGateway.tile) * 2,
      miniMap.y(firstGateway.tile) * 2,
    );

    const startMiniX = Math.floor(map.x(from) / 2);
    const startMiniY = Math.floor(map.y(from) / 2);
    const startSectorX = Math.floor(startMiniX / this.graph.sectorSize);
    const startSectorY = Math.floor(startMiniY / this.graph.sectorSize);
    const startSegment = this.findLocalPath(
      from,
      firstGatewayTile,
      startSectorX,
      startSectorY,
    );

    if (!startSegment) {
      return null;
    }

    initialPath.push(...startSegment);

    // 2. Build path through gateways
    for (let i = 0; i < gatewayPath.length - 1; i++) {
      const fromGwId = gatewayPath[i];
      const toGwId = gatewayPath[i + 1];

      const edges = this.graph.getEdges(fromGwId);
      const edge = edges.find((edge) => edge.to === toGwId);

      if (!edge) {
        return null;
      }

      if (edge.path) {
        initialPath.push(...edge.path.slice(1));
        continue;
      }

      const fromGw = this.graph.getGateway(fromGwId)!;
      const toGw = this.graph.getGateway(toGwId)!;
      const fromTile = map.ref(
        miniMap.x(fromGw.tile) * 2,
        miniMap.y(fromGw.tile) * 2,
      );
      const toTile = map.ref(
        miniMap.x(toGw.tile) * 2,
        miniMap.y(toGw.tile) * 2,
      );

      const segmentPath = this.findLocalPath(
        fromTile,
        toTile,
        edge.sectorX,
        edge.sectorY,
      );

      if (!segmentPath) {
        return null;
      }

      initialPath.push(...segmentPath.slice(1));

      if (this.options.cachePaths) {
        edge.path = segmentPath;

        const reverseEdges = this.graph.getEdges(toGwId);
        const reverseEdge = reverseEdges.find((e) => e.to === fromGwId);
        if (reverseEdge) {
          reverseEdge.path = segmentPath.slice().reverse();
        }
      }
    }

    // 3. Find path from last gateway to end
    const lastGateway = this.graph.getGateway(
      gatewayPath[gatewayPath.length - 1],
    )!;
    const lastGatewayTile = map.ref(
      miniMap.x(lastGateway.tile) * 2,
      miniMap.y(lastGateway.tile) * 2,
    );

    const endMiniX = Math.floor(map.x(to) / 2);
    const endMiniY = Math.floor(map.y(to) / 2);
    const endSectorX = Math.floor(endMiniX / this.graph.sectorSize);
    const endSectorY = Math.floor(endMiniY / this.graph.sectorSize);
    const endSegment = this.findLocalPath(
      lastGatewayTile,
      to,
      endSectorX,
      endSectorY,
    );

    if (!endSegment) {
      return null;
    }

    initialPath.push(...endSegment.slice(1));

    performance.mark("navsat:findPath:buildInitialPath:end");
    const buildInitialPathMeasure = performance.measure(
      "navsat:findPath:buildInitialPath",
      "navsat:findPath:buildInitialPath:start",
      "navsat:findPath:buildInitialPath:end",
    );

    if (debug) {
      this.debugInfo!.timings.buildInitialPath =
        buildInitialPathMeasure.duration;
      this.debugInfo!.timings.total += buildInitialPathMeasure.duration;
      this.debugInfo!.initialPath = initialPath;
      console.log(`[DEBUG] Initial path: ${initialPath.length} tiles`);
    }

    performance.mark("navsat:findPath:smoothPath:start");
    const smoothedPath = this.smoothPath(initialPath);
    performance.mark("navsat:findPath:smoothPath:end");
    const smoothPathMeasure = performance.measure(
      "navsat:findPath:smoothPath",
      "navsat:findPath:smoothPath:start",
      "navsat:findPath:smoothPath:end",
    );

    if (debug) {
      this.debugInfo!.timings.buildSmoothPath = smoothPathMeasure.duration;
      this.debugInfo!.timings.total += smoothPathMeasure.duration;
      this.debugInfo!.smoothPath = smoothedPath;
      console.log(
        `[DEBUG] Smoothed path: ${initialPath.length} → ${smoothedPath.length} tiles`,
      );
    }

    return smoothedPath;
  }

  private findNearestGateway(tile: TileRef): Gateway | null {
    const map = this.game.map();
    const x = map.x(tile);
    const y = map.y(tile);

    const miniMap = this.game.miniMap();
    const miniX = Math.floor(x / 2);
    const miniY = Math.floor(y / 2);
    const miniFrom = miniMap.ref(miniX, miniY);

    const sectorX = Math.floor(miniX / this.graph.sectorSize);
    const sectorY = Math.floor(miniY / this.graph.sectorSize);

    const sectorSize = this.graph.sectorSize;
    const minX = sectorX * sectorSize;
    const minY = sectorY * sectorSize;
    const maxX = Math.min(miniMap.width() - 1, minX + sectorSize - 1);
    const maxY = Math.min(miniMap.height() - 1, minY + sectorSize - 1);

    const sector = this.graph.getSector(sectorX, sectorY);

    if (!sector) {
      return null;
    }

    const candidateGateways = sector.gateways;
    if (candidateGateways.length === 0) {
      return null;
    }

    const maxDistance = sectorSize * sectorSize;

    return this.fastBFS.search(
      miniMap.width(),
      miniMap.height(),
      miniFrom,
      maxDistance,
      (tile: TileRef) => miniMap.isWater(tile),
      (tile: TileRef, _dist: number) => {
        const tileX = miniMap.x(tile);
        const tileY = miniMap.y(tile);

        for (const gateway of candidateGateways) {
          if (gateway.x === tileX && gateway.y === tileY) {
            return gateway;
          }
        }

        if (tileX < minX || tileX > maxX || tileY < minY || tileY > maxY) {
          return null;
        }
      },
    );
  }

  private findGatewayPath(
    fromGatewayId: number,
    toGatewayId: number,
  ): number[] | null {
    return this.gatewayAStar.search(fromGatewayId, toGatewayId);
  }

  private findLocalPath(
    from: TileRef,
    to: TileRef,
    sectorX: number,
    sectorY: number,
    multiSector: boolean = false,
  ): TileRef[] | null {
    const map = this.game.map();
    const miniMap = this.game.miniMap();

    // Convert full map coordinates to miniMap coordinates
    const miniFrom = miniMap.ref(
      Math.floor(map.x(from) / 2),
      Math.floor(map.y(from) / 2),
    );

    const miniTo = miniMap.ref(
      Math.floor(map.x(to) / 2),
      Math.floor(map.y(to) / 2),
    );

    // Calculate sector bounds
    const sectorSize = this.graph.sectorSize;

    let minX: number;
    let minY: number;
    let maxX: number;
    let maxY: number;

    if (multiSector) {
      // 3×3 sectors centered on the starting sector
      minX = Math.max(0, (sectorX - 1) * sectorSize);
      minY = Math.max(0, (sectorY - 1) * sectorSize);
      maxX = Math.min(miniMap.width() - 1, (sectorX + 2) * sectorSize - 1);
      maxY = Math.min(miniMap.height() - 1, (sectorY + 2) * sectorSize - 1);
    } else {
      // Single sector
      minX = sectorX * sectorSize;
      minY = sectorY * sectorSize;
      maxX = Math.min(miniMap.width() - 1, minX + sectorSize - 1);
      maxY = Math.min(miniMap.height() - 1, minY + sectorSize - 1);
    }

    // Choose the appropriate BoundedAStar based on search area
    const selectedAStar = multiSector
      ? this.localAStarMultiSector
      : this.localAStar;

    // Run BoundedAStar on bounded region
    const path = selectedAStar.searchBounded(miniFrom, miniTo, {
      minX,
      maxX,
      minY,
      maxY,
    });

    if (!path) {
      return null;
    }

    // Upscale from miniMap to full map
    const result = this.upscalePathToFullMap(path, from, to);

    return result;
  }

  private upscalePathToFullMap(
    miniPath: TileRef[],
    from: TileRef,
    to: TileRef,
  ): TileRef[] {
    const map = this.game.map();
    const miniMap = this.game.miniMap();

    const miniCells = miniPath.map((tile) => ({
      x: miniMap.x(tile),
      y: miniMap.y(tile),
    }));

    // Scale all points (2x)
    const scaledPath = miniCells.map((point) => ({
      x: point.x * 2,
      y: point.y * 2,
    }));

    // Interpolate between scaled points
    const smoothPath: Array<{ x: number; y: number }> = [];
    for (let i = 0; i < scaledPath.length - 1; i++) {
      const current = scaledPath[i];
      const next = scaledPath[i + 1];

      smoothPath.push(current);

      const dx = next.x - current.x;
      const dy = next.y - current.y;
      const distance = Math.max(Math.abs(dx), Math.abs(dy));
      const steps = distance;

      for (let step = 1; step < steps; step++) {
        smoothPath.push({
          x: Math.round(current.x + (dx * step) / steps),
          y: Math.round(current.y + (dy * step) / steps),
        });
      }
    }

    if (scaledPath.length > 0) {
      smoothPath.push(scaledPath[scaledPath.length - 1]);
    }

    const scaledCells = smoothPath;

    // Fix extremes to ensure exact start/end
    const fromCell = { x: map.x(from), y: map.y(from) };
    const toCell = { x: map.x(to), y: map.y(to) };

    const startIdx = scaledCells.findIndex(
      (c) => c.x === fromCell.x && c.y === fromCell.y,
    );
    if (startIdx === -1) {
      scaledCells.unshift(fromCell);
    } else if (startIdx !== 0) {
      scaledCells.splice(0, startIdx);
    }

    const endIdx = scaledCells.findIndex(
      (c) => c.x === toCell.x && c.y === toCell.y,
    );
    if (endIdx === -1) {
      scaledCells.push(toCell);
    } else if (endIdx !== scaledCells.length - 1) {
      scaledCells.splice(endIdx + 1);
    }

    return scaledCells.map((cell) => map.ref(cell.x, cell.y));
  }

  private tracePath(from: TileRef, to: TileRef): TileRef[] | null {
    const x0 = this.game.x(from);
    const y0 = this.game.y(from);
    const x1 = this.game.x(to);
    const y1 = this.game.y(to);

    const tiles: TileRef[] = [];

    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = x0 < x1 ? 1 : -1;
    const sy = y0 < y1 ? 1 : -1;
    let err = dx - dy;

    let x = x0;
    let y = y0;

    const maxTiles = 100000;
    let iterations = 0;

    while (true) {
      if (iterations++ > maxTiles) {
        return null;
      }
      const tile = this.game.ref(x, y);
      if (!this.game.isWater(tile)) {
        return null;
      }

      tiles.push(tile);

      if (x === x1 && y === y1) {
        break;
      }

      const e2 = 2 * err;
      const shouldMoveX = e2 > -dy;
      const shouldMoveY = e2 < dx;

      if (shouldMoveX && shouldMoveY) {
        x += sx;
        err -= dy;

        const intermediateTile = this.game.ref(x, y);
        if (!this.game.isWater(intermediateTile)) {
          x -= sx;
          err += dy;

          y += sy;
          err += dx;

          const altTile = this.game.ref(x, y);
          if (!this.game.isWater(altTile)) {
            return null;
          }
          tiles.push(altTile);

          x += sx;
          err -= dy;
        } else {
          tiles.push(intermediateTile);

          y += sy;
          err += dx;
        }
      } else {
        if (shouldMoveX) {
          x += sx;
          err -= dy;
        }

        if (shouldMoveY) {
          y += sy;
          err += dx;
        }
      }
    }

    return tiles;
  }

  private smoothPath(path: TileRef[]): TileRef[] {
    if (path.length <= 2) {
      return path;
    }

    const smoothed: TileRef[] = [];
    let current = 0;

    while (current < path.length - 1) {
      let farthest = current + 1;
      let bestTrace: TileRef[] | null = null;

      for (
        let i = current + 2;
        i < path.length;
        i += Math.max(1, Math.floor(path.length / 20))
      ) {
        const trace = this.tracePath(path[current], path[i]);

        if (trace !== null) {
          farthest = i;
          bestTrace = trace;
        } else {
          break;
        }
      }

      if (
        farthest < path.length - 1 &&
        (path.length - 1 - current) % 10 !== 0
      ) {
        const trace = this.tracePath(path[current], path[path.length - 1]);
        if (trace !== null) {
          farthest = path.length - 1;
          bestTrace = trace;
        }
      }

      if (bestTrace !== null && farthest > current + 1) {
        smoothed.push(...bestTrace.slice(0, -1));
      } else {
        smoothed.push(path[current]);
      }

      current = farthest;
    }

    smoothed.push(path[path.length - 1]);

    return smoothed;
  }
}

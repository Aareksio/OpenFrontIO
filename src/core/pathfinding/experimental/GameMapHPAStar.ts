// GameMap HPA* - Hierarchical Pathfinding A* for GameMap

import { Game } from "../../game/Game";
import { TileRef } from "../../game/GameMap";
import { FastBFS } from "../navmesh/FastBFS";
import {
  AbstractGraph,
  AbstractGraphBuilder,
  AbstractNode,
} from "./AbstractGraph";
import { AbstractGraphAStar } from "./AbstractGraphAStar";
import { AStar } from "./AStar";
import { BoundedAStar } from "./BoundedAStar";

type PathDebugInfo = {
  nodePath: TileRef[] | null;
  initialPath: TileRef[] | null;
  smoothPath: TileRef[] | null;
  graph: {
    clusterSize: number;
    nodes: Array<{ id: number; tile: TileRef }>;
    edges: Array<{
      id: number;
      nodeA: number;
      nodeB: number;
      from: TileRef;
      to: TileRef;
      cost: number;
    }>;
  };
  timings: { [key: string]: number };
};

export class GameMapHPAStar implements AStar {
  private graph!: AbstractGraph;
  private initialized = false;
  private fastBFS!: FastBFS;
  private abstractAStar!: AbstractGraphAStar;
  private localAStar!: BoundedAStar;
  private localAStarMultiCluster!: BoundedAStar;

  public debugInfo: PathDebugInfo | null = null;

  constructor(
    private game: Game,
    private options: {
      cachePaths?: boolean;
    } = {},
  ) {}

  initialize(debug: boolean = false) {
    const miniMap = this.game.miniMap();

    const graphBuilder = new AbstractGraphBuilder(
      miniMap,
      AbstractGraphBuilder.CLUSTER_SIZE,
    );
    this.graph = graphBuilder.build(debug);

    // FastBFS for nearest node search
    this.fastBFS = new FastBFS(miniMap.width() * miniMap.height());

    const clusterSize = AbstractGraphBuilder.CLUSTER_SIZE;

    // AbstractGraphAStar for abstract graph routing
    this.abstractAStar = new AbstractGraphAStar(this.graph);

    // BoundedAStar for cluster-bounded local pathfinding
    const maxLocalNodes = clusterSize * clusterSize;
    this.localAStar = new BoundedAStar(miniMap, maxLocalNodes);

    // BoundedAStar for multi-cluster (3x3) local pathfinding
    const multiClusterSize = clusterSize * 3;
    const maxMultiClusterNodes = multiClusterSize * multiClusterSize;
    this.localAStarMultiCluster = new BoundedAStar(miniMap, maxMultiClusterNodes);

    this.initialized = true;
  }

  // AStar interface
  search(from: number, to: number): number[] | null {
    return this.findPath(from as TileRef, to as TileRef);
  }

  findPath(
    from: TileRef,
    to: TileRef,
    debug: boolean = false,
  ): TileRef[] | null {
    if (!this.initialized) {
      throw new Error(
        "HPA* not initialized. Call initialize() before using findPath().",
      );
    }

    if (debug) {
      const allEdges: Array<{
        id: number;
        nodeA: number;
        nodeB: number;
        from: TileRef;
        to: TileRef;
        cost: number;
      }> = [];

      for (let edgeId = 0; edgeId < this.graph.edgeCount; edgeId++) {
        const edge = this.graph.getEdge(edgeId);
        if (!edge) continue;

        const nodeA = this.graph.getNode(edge.nodeA);
        const nodeB = this.graph.getNode(edge.nodeB);
        if (!nodeA || !nodeB) continue;

        allEdges.push({
          id: edge.id,
          nodeA: edge.nodeA,
          nodeB: edge.nodeB,
          from: nodeA.tile,
          to: nodeB.tile,
          cost: edge.cost,
        });
      }

      this.debugInfo = {
        nodePath: null,
        initialPath: null,
        smoothPath: null,
        graph: {
          clusterSize: this.graph.clusterSize,
          nodes: this.graph
            .getAllNodes()
            .map((node) => ({ id: node.id, tile: node.tile })),
          edges: allEdges,
        },
        timings: {
          total: 0,
        },
      };
    }

    const dist = this.game.manhattanDist(from, to);

    // Early exit for very short distances
    if (dist <= this.graph.clusterSize) {
      performance.mark("hpa:findPath:earlyExitLocalPath:start");
      const map = this.game.map();
      const startMiniX = Math.floor(map.x(from) / 2);
      const startMiniY = Math.floor(map.y(from) / 2);
      const clusterX = Math.floor(startMiniX / this.graph.clusterSize);
      const clusterY = Math.floor(startMiniY / this.graph.clusterSize);
      const localPath = this.findLocalPath(from, to, clusterX, clusterY, true);
      performance.mark("hpa:findPath:earlyExitLocalPath:end");
      const measure = performance.measure(
        "hpa:findPath:earlyExitLocalPath",
        "hpa:findPath:earlyExitLocalPath:start",
        "hpa:findPath:earlyExitLocalPath:end",
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
          `[DEBUG] Direct path failed for dist=${dist}, falling back to abstract graph`,
        );
      }
    }

    performance.mark("hpa:findPath:findNodes:start");
    const startNode = this.findNearestNode(from);
    const endNode = this.findNearestNode(to);
    performance.mark("hpa:findPath:findNodes:end");
    const findNodesMeasure = performance.measure(
      "hpa:findPath:findNodes",
      "hpa:findPath:findNodes:start",
      "hpa:findPath:findNodes:end",
    );

    if (debug) {
      this.debugInfo!.timings.findNodes = findNodesMeasure.duration;
      this.debugInfo!.timings.total += findNodesMeasure.duration;
    }

    if (!startNode) {
      if (debug) {
        console.log(
          `[DEBUG] Cannot find start node for (${this.game.x(from)}, ${this.game.y(from)})`,
        );
      }
      return null;
    }

    if (!endNode) {
      if (debug) {
        console.log(
          `[DEBUG] Cannot find end node for (${this.game.x(to)}, ${this.game.y(to)})`,
        );
      }
      return null;
    }

    if (startNode.id === endNode.id) {
      if (debug) {
        console.log(
          `[DEBUG] Start and end nodes are the same (ID=${startNode.id}), finding local path with multi-cluster search`,
        );
      }

      performance.mark("hpa:findPath:sameNodeLocalPath:start");
      const clusterX = Math.floor(startNode.x / this.graph.clusterSize);
      const clusterY = Math.floor(startNode.y / this.graph.clusterSize);
      const path = this.findLocalPath(from, to, clusterX, clusterY, true);
      performance.mark("hpa:findPath:sameNodeLocalPath:end");
      const sameNodeMeasure = performance.measure(
        "hpa:findPath:sameNodeLocalPath",
        "hpa:findPath:sameNodeLocalPath:start",
        "hpa:findPath:sameNodeLocalPath:end",
      );

      if (debug) {
        this.debugInfo!.timings.sameNodeLocalPath = sameNodeMeasure.duration;
        this.debugInfo!.timings.total += sameNodeMeasure.duration;
      }

      return path;
    }

    performance.mark("hpa:findPath:findAbstractPath:start");
    const nodePath = this.findAbstractPath(startNode.id, endNode.id);
    performance.mark("hpa:findPath:findAbstractPath:end");
    const findAbstractPathMeasure = performance.measure(
      "hpa:findPath:findAbstractPath",
      "hpa:findPath:findAbstractPath:start",
      "hpa:findPath:findAbstractPath:end",
    );

    if (debug) {
      this.debugInfo!.timings.findAbstractPath = findAbstractPathMeasure.duration;
      this.debugInfo!.timings.total += findAbstractPathMeasure.duration;

      this.debugInfo!.nodePath = nodePath
        ? nodePath
            .map((nodeId) => {
              const node = this.graph.getNode(nodeId);
              return node ? node.tile : -1;
            })
            .filter((tile) => tile !== -1)
        : null;
    }

    if (!nodePath) {
      if (debug) {
        console.log(
          `[DEBUG] No abstract path between nodes ${startNode.id} and ${endNode.id}`,
        );
      }
      return null;
    }

    if (debug) {
      console.log(`[DEBUG] Abstract path found: ${nodePath.length} waypoints`);
    }

    const initialPath: TileRef[] = [];
    const map = this.game.map();
    const miniMap = this.game.miniMap();

    performance.mark("hpa:findPath:buildInitialPath:start");

    // 1. Find path from start to first node
    const firstNode = this.graph.getNode(nodePath[0])!;
    const firstNodeTile = map.ref(
      miniMap.x(firstNode.tile) * 2,
      miniMap.y(firstNode.tile) * 2,
    );

    const startMiniX = Math.floor(map.x(from) / 2);
    const startMiniY = Math.floor(map.y(from) / 2);
    const startClusterX = Math.floor(startMiniX / this.graph.clusterSize);
    const startClusterY = Math.floor(startMiniY / this.graph.clusterSize);
    const startSegment = this.findLocalPath(
      from,
      firstNodeTile,
      startClusterX,
      startClusterY,
    );

    if (!startSegment) {
      return null;
    }

    initialPath.push(...startSegment);

    // 2. Build path through abstract nodes
    for (let i = 0; i < nodePath.length - 1; i++) {
      const fromNodeId = nodePath[i];
      const toNodeId = nodePath[i + 1];

      const edge = this.graph.getEdgeBetween(fromNodeId, toNodeId);
      if (!edge) {
        return null;
      }

      // Check path cache (stored on graph, shared across all instances)
      if (this.options.cachePaths) {
        const cachedPath = this.graph.getCachedPath(edge.id);
        if (cachedPath) {
          // Path is direction-independent, just skip first tile (already in path)
          initialPath.push(...cachedPath.slice(1));
          continue;
        }
      }

      const fromNode = this.graph.getNode(fromNodeId)!;
      const toNode = this.graph.getNode(toNodeId)!;
      const fromTile = map.ref(
        miniMap.x(fromNode.tile) * 2,
        miniMap.y(fromNode.tile) * 2,
      );
      const toTile = map.ref(
        miniMap.x(toNode.tile) * 2,
        miniMap.y(toNode.tile) * 2,
      );

      const segmentPath = this.findLocalPath(
        fromTile,
        toTile,
        edge.clusterX,
        edge.clusterY,
      );

      if (!segmentPath) {
        return null;
      }

      initialPath.push(...segmentPath.slice(1));

      // Cache the path on graph (shared across all instances)
      if (this.options.cachePaths) {
        this.graph.setCachedPath(edge.id, segmentPath);
      }
    }

    // 3. Find path from last node to end
    const lastNode = this.graph.getNode(nodePath[nodePath.length - 1])!;
    const lastNodeTile = map.ref(
      miniMap.x(lastNode.tile) * 2,
      miniMap.y(lastNode.tile) * 2,
    );

    const endMiniX = Math.floor(map.x(to) / 2);
    const endMiniY = Math.floor(map.y(to) / 2);
    const endClusterX = Math.floor(endMiniX / this.graph.clusterSize);
    const endClusterY = Math.floor(endMiniY / this.graph.clusterSize);
    const endSegment = this.findLocalPath(
      lastNodeTile,
      to,
      endClusterX,
      endClusterY,
    );

    if (!endSegment) {
      return null;
    }

    initialPath.push(...endSegment.slice(1));

    performance.mark("hpa:findPath:buildInitialPath:end");
    const buildInitialPathMeasure = performance.measure(
      "hpa:findPath:buildInitialPath",
      "hpa:findPath:buildInitialPath:start",
      "hpa:findPath:buildInitialPath:end",
    );

    if (debug) {
      this.debugInfo!.timings.buildInitialPath = buildInitialPathMeasure.duration;
      this.debugInfo!.timings.total += buildInitialPathMeasure.duration;
      this.debugInfo!.initialPath = initialPath;
      console.log(`[DEBUG] Initial path: ${initialPath.length} tiles`);
    }

    performance.mark("hpa:findPath:smoothPath:start");
    const smoothedPath = this.smoothPath(initialPath);
    performance.mark("hpa:findPath:smoothPath:end");
    const smoothPathMeasure = performance.measure(
      "hpa:findPath:smoothPath",
      "hpa:findPath:smoothPath:start",
      "hpa:findPath:smoothPath:end",
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

  private findNearestNode(tile: TileRef): AbstractNode | null {
    const map = this.game.map();
    const x = map.x(tile);
    const y = map.y(tile);

    const miniMap = this.game.miniMap();
    const miniX = Math.floor(x / 2);
    const miniY = Math.floor(y / 2);
    const miniFrom = miniMap.ref(miniX, miniY);

    const clusterX = Math.floor(miniX / this.graph.clusterSize);
    const clusterY = Math.floor(miniY / this.graph.clusterSize);

    const clusterSize = this.graph.clusterSize;
    const minX = clusterX * clusterSize;
    const minY = clusterY * clusterSize;
    const maxX = Math.min(miniMap.width() - 1, minX + clusterSize - 1);
    const maxY = Math.min(miniMap.height() - 1, minY + clusterSize - 1);

    const cluster = this.graph.getCluster(clusterX, clusterY);
    if (!cluster || cluster.nodeIds.length === 0) {
      return null;
    }

    const candidateNodes = cluster.nodeIds.map((id) => this.graph.getNode(id)!);
    const maxDistance = clusterSize * clusterSize;

    return this.fastBFS.search(
      miniMap.width(),
      miniMap.height(),
      miniFrom,
      maxDistance,
      (t: TileRef) => miniMap.isWater(t),
      (t: TileRef, _dist: number) => {
        const tileX = miniMap.x(t);
        const tileY = miniMap.y(t);

        for (const node of candidateNodes) {
          if (node.x === tileX && node.y === tileY) {
            return node;
          }
        }

        if (tileX < minX || tileX > maxX || tileY < minY || tileY > maxY) {
          return null;
        }
      },
    );
  }

  private findAbstractPath(
    fromNodeId: number,
    toNodeId: number,
  ): number[] | null {
    return this.abstractAStar.search(fromNodeId, toNodeId);
  }

  private findLocalPath(
    from: TileRef,
    to: TileRef,
    clusterX: number,
    clusterY: number,
    multiCluster: boolean = false,
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

    // Calculate cluster bounds
    const clusterSize = this.graph.clusterSize;

    let minX: number;
    let minY: number;
    let maxX: number;
    let maxY: number;

    if (multiCluster) {
      // 3×3 clusters centered on the starting cluster
      minX = Math.max(0, (clusterX - 1) * clusterSize);
      minY = Math.max(0, (clusterY - 1) * clusterSize);
      maxX = Math.min(miniMap.width() - 1, (clusterX + 2) * clusterSize - 1);
      maxY = Math.min(miniMap.height() - 1, (clusterY + 2) * clusterSize - 1);
    } else {
      // Single cluster
      minX = clusterX * clusterSize;
      minY = clusterY * clusterSize;
      maxX = Math.min(miniMap.width() - 1, minX + clusterSize - 1);
      maxY = Math.min(miniMap.height() - 1, minY + clusterSize - 1);
    }

    // Choose the appropriate BoundedAStar based on search area
    const selectedAStar = multiCluster
      ? this.localAStarMultiCluster
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

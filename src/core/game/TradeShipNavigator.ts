import { Game } from 'src/core/game/Game';
import { GameMap, TileRef } from 'src/core/game/GameMap';
import { SerialAStar, GraphAdapter } from 'src/core/pathfinding/SerialAStar';
import { MiniAStar } from 'src/core/pathfinding/MiniAStar';
import { PathFindResultType } from 'src/core/pathfinding/AStar';

// Gateway represents a continuous stretch of traversable tiles on a sector edge
interface Gateway {
  id: number;           // Unique gateway identifier
  sectorX: number;      // Sector coordinates
  sectorY: number;
  edge: 'right' | 'bottom';  // Which edge of the sector
  x: number;            // Primary point (lowest x,y)
  y: number;
  length: number;       // Width (for bottom edge) or height (for right edge)
  tile: TileRef;        // The primary tile reference
}

// Connection between two gateways within the same sector or adjacent sectors
interface GatewayConnection {
  from: number;         // Gateway ID
  to: number;           // Gateway ID
  cost: number;         // Path cost/difficulty
}

// Sector contains all gateways and their connections
interface Sector {
  x: number;
  y: number;
  gateways: Gateway[];
  connections: GatewayConnection[];
}

interface DebugInfo {
  gatewayPath: number[] | null;
  gatewayWaypoints: Array<[number, number]> | null;
  initialPath: TileRef[] | null;
  smoothedPath: TileRef[] | null;
  allGateways: Array<{ x: number; y: number; edge: 'right' | 'bottom'; length: number }>;
  sectorSize: number;
  timings: { [key: string]: number };
}

function bfsSearch<T>(
  map: GameMap,
  start: TileRef,
  maxDistance: number,
  visitor: (tile: TileRef, dist: number) => T | null,
): T | null {
  const visited = new Set<TileRef>();
  const queue: { tile: TileRef; dist: number }[] = [{ tile: start, dist: 0 }];
  visited.add(start);

  while (queue.length > 0) {
    const current = queue.shift()!;

    if (current.dist > maxDistance) {
      continue;
    }

    // Call visitor - if it returns non-null, we're done
    const result = visitor(current.tile, current.dist);
    if (result !== null) {
      return result;
    }

    // Expand BFS
    const neighbors = map.neighbors(current.tile);
    for (const neighbor of neighbors) {
      if (!visited.has(neighbor) && map.isWater(neighbor)) {
        visited.add(neighbor);
        queue.push({ tile: neighbor, dist: current.dist + 1 });
      }
    }
  }

  return null;
}

class GatewayGraph {
  constructor(
    readonly sectors: ReadonlyMap<string, Sector>,
    readonly gateways: ReadonlyMap<number, Gateway>,
    readonly connections: ReadonlyMap<number, GatewayConnection[]>,
    readonly sectorSize: number,
  ) {}

  getSectorKey(sectorX: number, sectorY: number): string {
    return `${sectorX},${sectorY}`;
  }

  getSector(sectorX: number, sectorY: number): Sector | undefined {
    return this.sectors.get(this.getSectorKey(sectorX, sectorY));
  }

  getGateway(id: number): Gateway | undefined {
    return this.gateways.get(id);
  }

  getConnections(gatewayId: number): GatewayConnection[] {
    return this.connections.get(gatewayId) ?? [];
  }

  getNearbySectorGateways(sectorX: number, sectorY: number): Gateway[] {
    const nearby: Gateway[] = [];
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        const sector = this.getSector(sectorX + dx, sectorY + dy);
        if (sector) {
          nearby.push(...sector.gateways);
        }
      }
    }
    return nearby;
  }

  getAllGateways(): Gateway[] {
    return Array.from(this.gateways.values());
  }
}

// GraphAdapter for gateway graph pathfinding
class GatewayGraphAdapter implements GraphAdapter<number> {
  constructor(private graph: GatewayGraph) {}

  neighbors(node: number): number[] {
    const connections = this.graph.getConnections(node);
    return connections.map(conn => conn.to);
  }

  cost(_node: number): number {
    return 1; // Base cost, actual cost is in the connection
  }

  position(node: number): { x: number; y: number } {
    const gateway = this.graph.getGateway(node);
    if (!gateway) return { x: 0, y: 0 };
    return { x: gateway.x, y: gateway.y };
  }

  isTraversable(from: number, to: number): boolean {
    const connections = this.graph.getConnections(from);
    return connections.some(conn => conn.to === to);
  }
}

class GatewayGraphBuilder {
  private static readonly SECTOR_SIZE = 32;

  static build(game: Game, sectorSize: number = GatewayGraphBuilder.SECTOR_SIZE): GatewayGraph {
    const startTime = performance.now();

    const sectors = new Map<string, Sector>();
    const gateways = new Map<number, Gateway>();
    const gatewayConnections = new Map<number, GatewayConnection[]>();
    let nextGatewayId = 0;

    // Build the gateway graph
    const width = game.width();
    const height = game.height();
    const sectorsX = Math.ceil(width / sectorSize);
    const sectorsY = Math.ceil(height / sectorSize);

    // Phase 1: Identify all gateways
    for (let sy = 0; sy < sectorsY; sy++) {
      for (let sx = 0; sx < sectorsX; sx++) {
        GatewayGraphBuilder.processSector(
          sx, sy, sectorsX, sectorsY,
          game, sectorSize, sectors, gateways, nextGatewayId
        );
        // Update nextGatewayId based on gateways created
        nextGatewayId = gateways.size > 0 ? Math.max(...gateways.keys()) + 1 : 0;
      }
    }

    // Phase 2: Build intra-sector connections
    for (const sector of sectors.values()) {
      GatewayGraphBuilder.buildSectorConnections(
        sector, game, sectorSize, gatewayConnections
      );
    }

    const endTime = performance.now();
    console.log(`Gateway graph built in ${(endTime - startTime).toFixed(2)}ms`);
    console.log(`Total gateways: ${gateways.size}`);
    console.log(`Total sectors: ${sectors.size}`);

    return new GatewayGraph(sectors, gateways, gatewayConnections, sectorSize);
  }

  private static getSectorKey(sectorX: number, sectorY: number): string {
    return `${sectorX},${sectorY}`;
  }

  private static processSector(
    sx: number, sy: number, 
    sectorsX: number, sectorsY: number,
    game: Game, sectorSize: number,
    sectors: Map<string, Sector>,
    gateways: Map<number, Gateway>,
    nextGatewayId: number
  ): void {
    const sectorKey = GatewayGraphBuilder.getSectorKey(sx, sy);
    let sector = sectors.get(sectorKey);
    if (!sector) {
      sector = { x: sx, y: sy, gateways: [], connections: [] };
      sectors.set(sectorKey, sector);
    }

    const baseX = sx * sectorSize;
    const baseY = sy * sectorSize;
    const width = game.width();
    const height = game.height();

    // Find gateways on right edge (if not the last column)
    if (sx < sectorsX - 1) {
      const edgeX = Math.min(baseX + sectorSize - 1, width - 1);

      const newGateways = GatewayGraphBuilder.findGatewaysOnVerticalEdge(
        edgeX, baseY, sy, sectorsY, game, sectorSize, nextGatewayId
      );

      sector.gateways.push(...newGateways);

      // Register gateways
      for (const gateway of newGateways) {
        gateways.set(gateway.id, gateway);
      }

      // Also add these gateways to the adjacent sector on the right
      const rightSectorKey = GatewayGraphBuilder.getSectorKey(sx + 1, sy);
      let rightSector = sectors.get(rightSectorKey);

      if (!rightSector) {
        rightSector = { x: sx + 1, y: sy, gateways: [], connections: [] };
        sectors.set(rightSectorKey, rightSector);
      }

      rightSector.gateways.push(...newGateways);
    }

    // Find gateways on bottom edge (if not the last row)
    if (sy < sectorsY - 1) {
      const edgeY = Math.min(baseY + sectorSize - 1, height - 1);

      const newGateways = GatewayGraphBuilder.findGatewaysOnHorizontalEdge(
        edgeY, baseX, sx, sectorsX, game, sectorSize,
        gateways.size > 0 ? Math.max(...gateways.keys()) + 1 : nextGatewayId
      );

      sector.gateways.push(...newGateways);

      // Register gateways
      for (const gateway of newGateways) {
        gateways.set(gateway.id, gateway);
      }

      // Also add these gateways to the adjacent sector below
      const bottomSectorKey = GatewayGraphBuilder.getSectorKey(sx, sy + 1);
      let bottomSector = sectors.get(bottomSectorKey);

      if (!bottomSector) {
        bottomSector = { x: sx, y: sy + 1, gateways: [], connections: [] };
        sectors.set(bottomSectorKey, bottomSector);
      }

      bottomSector.gateways.push(...newGateways);
    }
  }

  private static findGatewaysOnVerticalEdge(
    x: number, baseY: number, sectorY: number, _sectorsY: number,
    game: Game, sectorSize: number, startId: number
  ): Gateway[] {
    const gateways: Gateway[] = [];
    const height = game.height();
    const maxY = Math.min(baseY + sectorSize, height);

    let gatewayStart = -1;
    let currentId = startId;

    for (let y = baseY; y < maxY; y++) {
      const tile = game.ref(x, y);
      const nextTile = x + 1 < game.width() ? game.ref(x + 1, y) : -1;
      const isGateway = game.isWater(tile) && nextTile !== -1 && game.isWater(nextTile);

      if (isGateway) {
        if (gatewayStart === -1) {
          gatewayStart = y;
        }
      } else {
        if (gatewayStart !== -1) {
          const gatewayLength = y - gatewayStart;
          const midY = gatewayStart + Math.floor(gatewayLength / 2);

          gatewayStart = -1;
          gateways.push({
            id: currentId++,
            sectorX: Math.floor(x / sectorSize),
            sectorY: sectorY,
            edge: 'right',
            x: x,
            y: midY,
            length: gatewayLength,
            tile: game.ref(x, midY)
          });
        }
      }
    }

    if (gatewayStart !== -1) {
      const gatewayLength = maxY - gatewayStart;
      const midY = gatewayStart + Math.floor(gatewayLength / 2);

      gateways.push({
        id: currentId++,
        sectorX: Math.floor(x / sectorSize),
        sectorY: sectorY,
        edge: 'right',
        x: x,
        y: midY,
        length: gatewayLength,
        tile: game.ref(x, midY)
      });
    }

    return gateways;
  }

  private static findGatewaysOnHorizontalEdge(
    y: number, baseX: number, sectorX: number, _sectorsX: number,
    game: Game, sectorSize: number, startId: number
  ): Gateway[] {
    const gateways: Gateway[] = [];
    const width = game.width();
    const maxX = Math.min(baseX + sectorSize, width);

    let gatewayStart = -1;
    let currentId = startId;

    for (let x = baseX; x < maxX; x++) {
      const tile = game.ref(x, y);
      const nextTile = y + 1 < game.height() ? game.ref(x, y + 1) : -1;
      const isGateway = game.isWater(tile) && nextTile !== -1 && game.isWater(nextTile);

      if (isGateway) {
        if (gatewayStart === -1) {
          gatewayStart = x;
        }
      } else {
        if (gatewayStart !== -1) {
          const gatewayLength = x - gatewayStart;
          const midX = gatewayStart + Math.floor(gatewayLength / 2);

          gatewayStart = -1;
          gateways.push({
            id: currentId++,
            sectorX: sectorX,
            sectorY: Math.floor(y / sectorSize),
            edge: 'bottom',
            x: midX,
            y: y,
            length: gatewayLength,
            tile: game.ref(midX, y)
          });
        }
      }
    }

    if (gatewayStart !== -1) {
      const gatewayLength = maxX - gatewayStart;
      const midX = gatewayStart + Math.floor(gatewayLength / 2);

      gateways.push({
        id: currentId++,
        sectorX: sectorX,
        sectorY: Math.floor(y / sectorSize),
        edge: 'bottom',
        x: midX,
        y: y,
        length: gatewayLength,
        tile: game.ref(midX, y)
      });
    }

    return gateways;
  }

  private static buildSectorConnections(
    sector: Sector,
    game: Game,
    sectorSize: number,
    gatewayConnections: Map<number, GatewayConnection[]>
  ): void {
    const gateways = sector.gateways;

    for (let i = 0; i < gateways.length; i++) {
      for (let j = i + 1; j < gateways.length; j++) {
        const cost = GatewayGraphBuilder.findLocalPathCost(
          gateways[i].tile, gateways[j].tile, game, sectorSize
        );

        if (cost !== -1) {
          const connection1: GatewayConnection = {
            from: gateways[i].id,
            to: gateways[j].id,
            cost: cost
          };

          const connection2: GatewayConnection = {
            from: gateways[j].id,
            to: gateways[i].id,
            cost: cost
          };

          sector.connections.push(connection1, connection2);

          if (!gatewayConnections.has(gateways[i].id)) {
            gatewayConnections.set(gateways[i].id, []);
          }

          if (!gatewayConnections.has(gateways[j].id)) {
            gatewayConnections.set(gateways[j].id, []);
          }

          gatewayConnections.get(gateways[i].id)!.push(connection1);
          gatewayConnections.get(gateways[j].id)!.push(connection2);
        }
      }
    }
  }

  private static findLocalPathCost(
    from: TileRef, to: TileRef, game: Game, sectorSize: number
  ): number {
    const miniMap = game.miniMap();

    const miniFrom = miniMap.ref(
      Math.floor(game.x(from) / 2),
      Math.floor(game.y(from) / 2)
    );

    const miniTo = miniMap.ref(
      Math.floor(game.x(to) / 2),
      Math.floor(game.y(to) / 2)
    );

    const maxDistance = sectorSize * 3;

    const result = bfsSearch(
      miniMap, miniFrom, maxDistance,
      (tile, dist) => {
        if (tile === miniTo) {
          return dist * 2;
        }

        return null;
      }
    );

    return result ?? -1;
  }
}

export class TradeShipNavigator {
  private graph!: GatewayGraph;
  private initialized = false;

  public debugInfo: DebugInfo | null = null;
  private debugTimeStart: number | null = null;

  constructor(
    private game: Game,
  ) {}

  initialize() {
    this.graph = GatewayGraphBuilder.build(this.game);
    this.initialized = true;
  }

  findPath(
    from: TileRef, 
    to: TileRef, 
    debug: boolean = false
  ): TileRef[] | null {
    if (!this.initialized) {
      this.initialize();
    }

    if (debug) {
      this.debugInfo = {
        gatewayPath: null,
        gatewayWaypoints: null,
        initialPath: null,
        smoothedPath: null,
        allGateways: this.getAllGatewaysDebugInfo(),
        sectorSize: this.graph.sectorSize,
        timings: {},
      };
    }

    const dist = this.game.manhattanDist(from, to);

    if (dist < 500) {
      if (debug) this.debugTimeStart = performance.now();
      const localPath = this.findLocalPath(from, to, 2000);
      if (debug) this.debugInfo!.timings.earlyExitLocalPath = performance.now() - this.debugTimeStart!;

      if (localPath) {
        if (debug) console.log(`  [DEBUG] Direct local path found for dist=${dist}, length=${localPath.length}`);
        return localPath;
      }
      
      if (debug) console.log(`  [DEBUG] Direct path failed for dist=${dist}, falling back to gateway graph`);
    }

    if (debug) this.debugTimeStart = performance.now();
    const startGateway = this.findNearestGateway(from);
    const endGateway = this.findNearestGateway(to);
    if (debug) this.debugInfo!.timings.findGateways = performance.now() - this.debugTimeStart!;

    if (!startGateway) {
      if (debug) console.log(`  [DEBUG] Cannot find start gateway for (${this.game.x(from)}, ${this.game.y(from)})`);
      return null;
    }

    if (!endGateway) {
      if (debug) console.log(`  [DEBUG] Cannot find end gateway for (${this.game.x(to)}, ${this.game.y(to)})`);
      return null;
    }

    if (startGateway.id === endGateway.id) {
      if (debug) console.log(`  [DEBUG] Start and end gateways are the same (ID=${startGateway.id}), finding local path`);
      return this.findLocalPath(from, to);
    }

    if (debug) this.debugTimeStart = performance.now();
    const gatewayPath = this.findGatewayPath(startGateway.id, endGateway.id);
    if (debug) this.debugInfo!.timings.findGatewayPath = performance.now() - this.debugTimeStart!;

    if (debug) this.debugInfo!.gatewayPath = gatewayPath;

    if (!gatewayPath) {
      if (debug) console.log(`  [DEBUG] No gateway path between gateways ${startGateway.id} and ${endGateway.id}`);
      return null;
    }

    if (debug) console.log(`  [DEBUG] Gateway path found: ${gatewayPath.length} waypoints`);

    const initialPath: TileRef[] = [];
    const waypoints: TileRef[] = [from, ...gatewayPath.map(gwId => this.graph.getGateway(gwId)!.tile), to];

    if (debug) this.debugInfo!.initialPath = initialPath;
    if (debug) this.debugInfo!.gatewayWaypoints = waypoints.map(tile => [this.game.x(tile), this.game.y(tile)]);

    if (debug) this.debugTimeStart = performance.now();
    for (let i = 0; i < waypoints.length - 1; i++) {
      const segment = this.findLocalPath(waypoints[i], waypoints[i + 1]);

      if (!segment) {
        return null;
      }

      if (i === 0) {
        initialPath.push(...segment);
      } else {
        // Skip first tile to avoid duplication
        initialPath.push(...segment.slice(1));
      }
    }
    if (debug) this.debugInfo!.timings.buildInitialPath = performance.now() - this.debugTimeStart!;

    if (debug) console.log(`  [DEBUG] Initial path: ${initialPath.length} tiles`);

    if (debug) this.debugTimeStart = performance.now();
    const smoothedPath = this.smoothPath(initialPath)
    if (debug) this.debugInfo!.timings.buildSmoothPath = performance.now() - this.debugTimeStart!;

    if (debug) console.log(`  [DEBUG] Smoothed path: ${initialPath.length} → ${smoothedPath.length} tiles`);
    if (debug) this.debugInfo!.smoothedPath = smoothedPath;

    return smoothedPath;
  }

  private getAllGatewaysDebugInfo(): Array<{ x: number; y: number; edge: 'right' | 'bottom'; length: number }> {
    return this.graph.getAllGateways().map(gw => ({
      x: gw.x,
      y: gw.y,
      edge: gw.edge,
      length: gw.length,
    }));
  }

  private findNearestGateway(tile: TileRef): Gateway | null {
    const x = this.game.x(tile);
    const y = this.game.y(tile);

    // Check gateways in nearby sectors (current and adjacent)
    const sectorX = Math.floor(x / this.graph.sectorSize);
    const sectorY = Math.floor(y / this.graph.sectorSize);

    // Collect all candidate gateways from nearby sectors
    const candidateGateways = this.graph.getNearbySectorGateways(sectorX, sectorY);

    if (candidateGateways.length === 0) {
      return null;
    }

    // Use BFS to find the nearest reachable gateway (by water path distance)
    // This ensures we only find gateways in the same water region
    const miniMap = this.game.miniMap();
    const miniFrom = miniMap.ref(
      Math.floor(x / 2),
      Math.floor(y / 2)
    );

    const maxDistance = this.graph.sectorSize * 3;

    return bfsSearch(miniMap, miniFrom, maxDistance, (tile, _dist) => {
      // Check if any candidate gateway is at this position
      // Gateway positions are on full map, so convert minimap coords to full coords
      const fullX = miniMap.x(tile) * 2;
      const fullY = miniMap.y(tile) * 2;

      for (const gateway of candidateGateways) {
        // Check if gateway is within 2 tiles of current position
        // (gateway could be at any position within the minimap tile's 2x2 area)
        const dx = Math.abs(gateway.x - fullX);
        const dy = Math.abs(gateway.y - fullY);

        if (dx <= 2 && dy <= 2) {
          return gateway;
        }
      }

      return null;
    });
  }

  private findGatewayPath(
    fromGatewayId: number, 
    toGatewayId: number
  ): number[] | null {
    const adapter = new GatewayGraphAdapter(this.graph);
    const aStar = new SerialAStar(fromGatewayId, toGatewayId, 10000, 1, adapter, 0)

    if (aStar.compute() === PathFindResultType.Completed) {
      return aStar.reconstructPath();
    }

    return null
  }

  private findLocalPath(
    from: TileRef, 
    to: TileRef, 
    maxIterations: number = 10000
  ): TileRef[] | null {
    const miniMap = this.game.miniMap();
    const miniAStar = new MiniAStar(this.game, miniMap, from, to, maxIterations, 1);

    if (miniAStar.compute() === PathFindResultType.Completed) {
      return miniAStar.reconstructPath();
    }

    return null
  }

  private tracePath(from: TileRef, to: TileRef): TileRef[] | null {
    const x0 = this.game.x(from);
    const y0 = this.game.y(from);
    const x1 = this.game.x(to);
    const y1 = this.game.y(to);

    const tiles: TileRef[] = [];

    // Bresenham's line algorithm - trace and collect all tiles
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = x0 < x1 ? 1 : -1;
    const sy = y0 < y1 ? 1 : -1;
    let err = dx - dy;

    let x = x0;
    let y = y0;

    while (true) {
      const tile = this.game.ref(x, y);
      if (!this.game.isWater(tile)) {
        return null; // Path blocked
      }

      tiles.push(tile);

      if (x === x1 && y === y1) {
        break;
      }

      const e2 = 2 * err;
      const shouldMoveX = e2 > -dy;
      const shouldMoveY = e2 < dx;

      if (shouldMoveX && shouldMoveY) {
        // Diagonal move - need to expand into two 4-directional moves
        // Try moving X first, then Y
        x += sx;
        err -= dy;

        const intermediateTile = this.game.ref(x, y);
        if (!this.game.isWater(intermediateTile)) {
          // X first doesn't work, try Y first instead
          x -= sx; // undo
          err += dy; // undo

          y += sy;
          err += dx;

          const altTile = this.game.ref(x, y);
          if (!this.game.isWater(altTile)) {
            return null; // Neither direction works
          }
          tiles.push(altTile);

          // Now move X
          x += sx;
          err -= dy;
        } else {
          tiles.push(intermediateTile);

          // Now move Y
          y += sy;
          err += dx;
        }
      } else {
        // Single-axis move
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
      // Look as far ahead as possible while maintaining line of sight
      let farthest = current + 1;
      let bestTrace: TileRef[] | null = null;

      for (let i = current + 2; i < path.length; i += Math.max(1, Math.floor((path.length / 20)))) {
        const trace = this.tracePath(path[current], path[i]);

        if (trace !== null) {
          farthest = i;
          bestTrace = trace;
        } else {
          break;
        }
      }

      // Also try the final tile if we haven't already
      if (farthest < path.length - 1 && (path.length - 1 - current) % 10 !== 0) {
        const trace = this.tracePath(path[current], path[path.length - 1]);
        if (trace !== null) {
          farthest = path.length - 1;
          bestTrace = trace;
        }
      }

      // Add the traced path (or just current tile if no improvement)
      if (bestTrace !== null && farthest > current + 1) {
        // Add all tiles from the trace except the last one (to avoid duplication)
        smoothed.push(...bestTrace.slice(0, -1));
      } else {
        // No LOS improvement, just add current tile
        smoothed.push(path[current]);
      }

      current = farthest;
    }

    // Add the final tile
    smoothed.push(path[path.length - 1]);

    return smoothed;
  }
}

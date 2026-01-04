import { Game, Cell } from 'src/core/game/Game';
import { GameMap, TileRef } from 'src/core/game/GameMap';
import { SerialAStar, GraphAdapter } from 'src/core/pathfinding/SerialAStar';
import { GameMapAdapter } from 'src/core/pathfinding/MiniAStar';
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

// ============================================================================
// GatewayGraph - Immutable graph data structure
// ============================================================================

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

// Helper functions for path upscaling (from MiniAStar)
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

// ============================================================================
// GatewayGraphBuilder - Builds gateway graphs from game maps
// ============================================================================

class GatewayGraphBuilder {
  private static readonly SECTOR_SIZE = 64;

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
    sx: number, sy: number, sectorsX: number, sectorsY: number,
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
          gatewayStart = -1;
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
          gatewayStart = -1;
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

    const result = GatewayGraphBuilder.bfsSearch(
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

  private static bfsSearch<T>(
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

      const result = visitor(current.tile, current.dist);
      if (result !== null) {
        return result;
      }

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
}

// ============================================================================
// TradeShipNavigator - Uses gateway graph for pathfinding
// ============================================================================

export class TradeShipNavigator {
  private graph!: GatewayGraph;
  private initialized = false;

  constructor(
    private game: Game,
  ) {}

  initialize() {
    this.graph = GatewayGraphBuilder.build(this.game);
    this.initialized = true;
  }

  // Generic BFS search on a map with distance limit and visitor pattern
  // Returns the first non-null result from the visitor, or null if search exhausted
  private bfsSearch<T>(
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

  // Find nearest gateway to a given tile
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

    return this.bfsSearch(miniMap, miniFrom, maxDistance, (tile, _dist) => {
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

  // A* search on gateway graph using SerialAStar
  private findGatewayPath(fromGatewayId: number, toGatewayId: number): number[] | null {
    const adapter = new GatewayGraphAdapter(this.graph);

    const aStar = new SerialAStar(
      fromGatewayId,
      toGatewayId,
      10000, // iterations - gateway graph pathfinding
      20,    // maxTries - same as PathFinder.Mini
      adapter,
      0      // no direction change penalty
    );

    const result = aStar.compute();
    if (result === PathFindResultType.Completed) {
      return aStar.reconstructPath();
    }

    return null;
  }

  // Smooth path using sliding window approach
  // At each position, try to find a direct path N tiles ahead and replace the segment
  // Move forward by N/2 and repeat
  private smoothPath(path: TileRef[], N: number = 100): TileRef[] {
    if (path.length <= 2) {
      return path;
    }

    // Work on a copy to avoid modifying the input
    const result = [...path];
    let currentPos = 0;

    while (currentPos < result.length - 1) {
      // Look N tiles ahead
      const targetPos = Math.min(currentPos + N, result.length - 1);

      if (targetPos <= currentPos) {
        // No room to smooth
        break;
      }

      // Find direct path between current position and target
      const directPath = this.findDetailedLocalPath(result[currentPos], result[targetPos]);

      if (directPath !== null && directPath.length > 0) {
        // Replace the segment [currentPos, targetPos] with the direct path
        const oldSegmentLength = targetPos - currentPos + 1;

        // Remove old segment and insert new one
        result.splice(currentPos, oldSegmentLength, ...directPath);

        // Move forward by min(N/2, length of new segment)
        const moveDistance = Math.min(Math.floor(N / 2), directPath.length - 1);
        currentPos += Math.max(1, moveDistance);
      } else {
        // Can't smooth this segment, move forward
        currentPos += Math.max(1, Math.floor(N / 2));
      }
    }

    return result;
  }

  // Debug information returned when debug mode is enabled
  public debugInfo: {
    gatewayPath: number[] | null;
    gatewayWaypoints: Array<[number, number]> | null;
    initialPath: TileRef[] | null;
    smoothedPath: TileRef[] | null;
    allGateways: Array<{ x: number; y: number; edge: 'right' | 'bottom'; length: number }>;
    sectorSize: number;
  } | null = null;

  // Main entry point for pathfinding
  findPath(from: TileRef, to: TileRef, debug: boolean = false): TileRef[] | null {
    if (!this.initialized) {
      this.initialize();
    }

    // Reset debug info
    this.debugInfo = null;

    // Distance-based optimization: skip gateway graph for short distances
    const dist = this.game.manhattanDist(from, to);

    // For short distances (< 500 tiles), try direct local path first
    // Gateway graph overhead dominates for distances under ~500 tiles
    if (dist < 500) {
      // Use limited iterations (2000) for quick check - if path is complex, fail fast
      // and fall back to gateway graph. Full pathfinding uses 10000 iterations.
      const localPath = this.findDetailedLocalPath(from, to, 2000);
      if (localPath) {
        if (debug) {
          this.debugInfo = {
            gatewayPath: null,
            gatewayWaypoints: null,
            initialPath: null,
            smoothedPath: null,
            allGateways: this.getAllGatewaysDebugInfo(),
            sectorSize: this.graph.sectorSize,
          };
        }
        return localPath;
      }
      // If direct path fails (e.g., complex route with obstacles),
      // fall back to gateway graph routing
      if (debug) console.log(`  [DEBUG] Direct path failed for dist=${dist}, falling back to gateway graph`);
    }

    // Find nearest gateways to start and end
    const startGateway = this.findNearestGateway(from);
    const endGateway = this.findNearestGateway(to);

    if (!startGateway) {
      if (debug) console.log(`  [DEBUG] Cannot find start gateway for (${this.game.x(from)}, ${this.game.y(from)})`);
      return null;
    }
    if (!endGateway) {
      if (debug) console.log(`  [DEBUG] Cannot find end gateway for (${this.game.x(to)}, ${this.game.y(to)})`);
      return null;
    }

    // If same gateway, just do local pathfinding
    if (startGateway.id === endGateway.id) {
      if (debug) {
        this.debugInfo = {
          gatewayPath: null,
          gatewayWaypoints: null,
          initialPath: null,
          smoothedPath: null,
          allGateways: this.getAllGatewaysDebugInfo(),
          sectorSize: this.graph.sectorSize,
        };
      }
      return this.findDetailedLocalPath(from, to);
    }

    // Find path through gateway graph
    const gatewayPath = this.findGatewayPath(startGateway.id, endGateway.id);
    if (!gatewayPath) {
      if (debug) console.log(`  [DEBUG] No gateway path between gateways ${startGateway.id} and ${endGateway.id}`);
      return null;
    }
    if (debug) console.log(`  [DEBUG] Gateway path found: ${gatewayPath.length} waypoints`);

    // Convert gateway IDs to waypoint tiles, including start and end positions
    const gatewayWaypoints: TileRef[] = [
      from,
      ...gatewayPath.map(gwId => this.graph.getGateway(gwId)!.tile),
      to
    ];

    // Build initial path by connecting gateway waypoints
    const initialPath: TileRef[] = [];

    for (let i = 0; i < gatewayWaypoints.length - 1; i++) {
      const segment = this.findDetailedLocalPath(gatewayWaypoints[i], gatewayWaypoints[i + 1]);
      if (!segment) return null;

      if (i === 0) {
        initialPath.push(...segment);
      } else {
        // Skip first tile to avoid duplication
        initialPath.push(...segment.slice(1));
      }
    }

    if (debug) console.log(`  [DEBUG] Initial path: ${initialPath.length} tiles`);

    // Apply adaptive path smoothing - larger windows for longer paths
    // Scale smoothing window based on path length for better optimization
    let N = 100; // default window
    if (initialPath.length > 3000) {
      N = 300; // large window for very long paths
    } else if (initialPath.length > 1000) {
      N = 200; // medium window for long paths
    }

    const smoothedPath = initialPath.length > 2 * N
      ? this.smoothPath(initialPath, N)
      : initialPath;

    if (debug) console.log(`  [DEBUG] Smoothed path (N=${N}): ${initialPath.length} → ${smoothedPath.length} tiles`);

    // Store debug info
    if (debug) {
      this.debugInfo = {
        gatewayPath,
        gatewayWaypoints: gatewayWaypoints.map(tile => [this.game.x(tile), this.game.y(tile)]),
        initialPath: [...initialPath],
        smoothedPath: [...smoothedPath],
        allGateways: this.getAllGatewaysDebugInfo(),
        sectorSize: this.graph.sectorSize,
      };
    }

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

  // Detailed local pathfinding using SerialAStar on minimap (like PathFinder.Mini)
  private findDetailedLocalPath(from: TileRef, to: TileRef, maxIterations: number = 10000): TileRef[] | null {
    const miniMap = this.game.miniMap();

    // Convert full-resolution coordinates to minimap coordinates (divide by 2)
    const miniFrom = miniMap.ref(
      Math.floor(this.game.x(from) / 2),
      Math.floor(this.game.y(from) / 2)
    );
    const miniTo = miniMap.ref(
      Math.floor(this.game.x(to) / 2),
      Math.floor(this.game.y(to) / 2)
    );

    // Run A* on the minimap (4x smaller search space)
    const adapter = new GameMapAdapter(miniMap, true); // waterPath = true for ships
    const aStar = new SerialAStar(miniFrom, miniTo, maxIterations, 1, adapter, 0);
    const result = aStar.compute();

    if (result !== PathFindResultType.Completed) {
      return null;
    }

    // Get the path on the minimap
    const miniPath = aStar.reconstructPath();

    // Convert minimap path to Cell array
    const miniCells = miniPath.map(tile =>
      new Cell(miniMap.x(tile), miniMap.y(tile))
    );

    // Upscale to full resolution
    const fullResCells = upscalePath(miniCells, 2);

    // Fix extremes to ensure start and end points are exact
    const fromCell = new Cell(this.game.x(from), this.game.y(from));
    const toCell = new Cell(this.game.x(to), this.game.y(to));
    const fixedCells = fixExtremes(fullResCells, toCell, fromCell);

    // Convert back to TileRefs
    return fixedCells.map(cell => this.game.ref(cell.x, cell.y));
  }
}

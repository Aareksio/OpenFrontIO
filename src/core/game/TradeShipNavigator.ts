import { Game, Cell } from 'src/core/game/Game';
import { TileRef } from 'src/core/game/GameMap';
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

// GraphAdapter for gateway graph pathfinding
class GatewayGraphAdapter implements GraphAdapter<number> {
  constructor(
    private gateways: Map<number, Gateway>,
    private gatewayConnections: Map<number, GatewayConnection[]>,
    private game: Game
  ) {}

  neighbors(node: number): number[] {
    const connections = this.gatewayConnections.get(node);
    if (!connections) return [];
    return connections.map(conn => conn.to);
  }

  cost(node: number): number {
    return 1; // Base cost, actual cost is in the connection
  }

  position(node: number): { x: number; y: number } {
    const gateway = this.gateways.get(node);
    if (!gateway) return { x: 0, y: 0 };
    return { x: gateway.x, y: gateway.y };
  }

  isTraversable(from: number, to: number): boolean {
    const connections = this.gatewayConnections.get(from);
    if (!connections) return false;
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

// Do one thing, do it well class for
// navigating trade ships across the map
export class TradeShipNavigator {
  private static readonly SECTOR_SIZE = 64;

  private sectors: Map<string, Sector> = new Map();
  private gateways: Map<number, Gateway> = new Map();
  private gatewayConnections: Map<number, GatewayConnection[]> = new Map();
  private nextGatewayId = 0;
  private initialized = false;

  constructor(
    private game: Game,
  ) {}

  initialize() {
    if (this.initialized) return;

    const startTime = performance.now();

    this.buildGatewayGraph();

    const endTime = performance.now();
    this.initialized = true;

    console.log(`Gateway graph built in ${(endTime - startTime).toFixed(2)}ms`);
    console.log(`Total gateways: ${this.gateways.size}`);
    console.log(`Total sectors: ${this.sectors.size}`);
  }

  private getSectorKey(sectorX: number, sectorY: number): string {
    return `${sectorX},${sectorY}`;
  }

  private buildGatewayGraph() {
    const width = this.game.width();
    const height = this.game.height();
    const sectorsX = Math.ceil(width / TradeShipNavigator.SECTOR_SIZE);
    const sectorsY = Math.ceil(height / TradeShipNavigator.SECTOR_SIZE);

    // Phase 1: Identify all gateways
    for (let sy = 0; sy < sectorsY; sy++) {
      for (let sx = 0; sx < sectorsX; sx++) {
        this.processSector(sx, sy, sectorsX, sectorsY);
      }
    }

    // Phase 2: Build intra-sector connections
    // Since gateways are now shared between adjacent sectors,
    // this automatically creates cross-sector connections!
    for (const sector of this.sectors.values()) {
      this.buildSectorConnections(sector);
    }
  }

  private processSector(sx: number, sy: number, sectorsX: number, sectorsY: number) {
    const sectorKey = this.getSectorKey(sx, sy);
    // Get existing sector or create new one
    let sector = this.sectors.get(sectorKey);
    if (!sector) {
      sector = {
        x: sx,
        y: sy,
        gateways: [],
        connections: []
      };
      this.sectors.set(sectorKey, sector);
    }

    const baseX = sx * TradeShipNavigator.SECTOR_SIZE;
    const baseY = sy * TradeShipNavigator.SECTOR_SIZE;
    const width = this.game.width();
    const height = this.game.height();

    // Find gateways on right edge (if not the last column)
    if (sx < sectorsX - 1) {
      const edgeX = Math.min(baseX + TradeShipNavigator.SECTOR_SIZE - 1, width - 1);
      const gateways = this.findGatewaysOnVerticalEdge(edgeX, baseY, sy, sectorsY);
      sector.gateways.push(...gateways);

      // Also add these gateways to the adjacent sector on the right
      const rightSectorKey = this.getSectorKey(sx + 1, sy);
      let rightSector = this.sectors.get(rightSectorKey);
      if (!rightSector) {
        rightSector = { x: sx + 1, y: sy, gateways: [], connections: [] };
        this.sectors.set(rightSectorKey, rightSector);
      }
      rightSector.gateways.push(...gateways);
    }

    // Find gateways on bottom edge (if not the last row)
    if (sy < sectorsY - 1) {
      const edgeY = Math.min(baseY + TradeShipNavigator.SECTOR_SIZE - 1, height - 1);
      const gateways = this.findGatewaysOnHorizontalEdge(edgeY, baseX, sx, sectorsX);
      sector.gateways.push(...gateways);

      // Also add these gateways to the adjacent sector below
      const bottomSectorKey = this.getSectorKey(sx, sy + 1);
      let bottomSector = this.sectors.get(bottomSectorKey);
      if (!bottomSector) {
        bottomSector = { x: sx, y: sy + 1, gateways: [], connections: [] };
        this.sectors.set(bottomSectorKey, bottomSector);
      }
      bottomSector.gateways.push(...gateways);
    }

    // Register all gateways (sector already in map from earlier)
    for (const gateway of sector.gateways) {
      if (!this.gateways.has(gateway.id)) {
        this.gateways.set(gateway.id, gateway);
      }
    }
  }

  private findGatewaysOnVerticalEdge(
    x: number,
    baseY: number,
    sectorY: number,
    _sectorsY: number
  ): Gateway[] {
    const gateways: Gateway[] = [];
    const height = this.game.height();
    const maxY = Math.min(baseY + TradeShipNavigator.SECTOR_SIZE, height);

    let gatewayStart = -1;

    for (let y = baseY; y < maxY; y++) {
      const tile = this.game.ref(x, y);
      const nextTile = x + 1 < this.game.width() ? this.game.ref(x + 1, y) : -1;

      // Check if both current and next tile are water (gateway condition)
      const isGateway = this.game.isWater(tile) &&
                       nextTile !== -1 &&
                       this.game.isWater(nextTile);

      if (isGateway) {
        if (gatewayStart === -1) {
          gatewayStart = y;
        }
      } else {
        // End of gateway stretch
        if (gatewayStart !== -1) {
          const gatewayLength = y - gatewayStart;
          gateways.push({
            id: this.nextGatewayId++,
            sectorX: Math.floor(x / TradeShipNavigator.SECTOR_SIZE),
            sectorY: sectorY,
            edge: 'right',
            x: x,
            y: gatewayStart,
            length: gatewayLength,
            tile: this.game.ref(x, gatewayStart)
          });
          gatewayStart = -1;
        }
      }
    }

    // Handle gateway extending to edge
    if (gatewayStart !== -1) {
      const gatewayLength = maxY - gatewayStart;
      gateways.push({
        id: this.nextGatewayId++,
        sectorX: Math.floor(x / TradeShipNavigator.SECTOR_SIZE),
        sectorY: sectorY,
        edge: 'right',
        x: x,
        y: gatewayStart,
        length: gatewayLength,
        tile: this.game.ref(x, gatewayStart)
      });
    }

    return gateways;
  }

  private findGatewaysOnHorizontalEdge(
    y: number,
    baseX: number,
    sectorX: number,
    _sectorsX: number
  ): Gateway[] {
    const gateways: Gateway[] = [];
    const width = this.game.width();
    const maxX = Math.min(baseX + TradeShipNavigator.SECTOR_SIZE, width);

    let gatewayStart = -1;

    for (let x = baseX; x < maxX; x++) {
      const tile = this.game.ref(x, y);
      const nextTile = y + 1 < this.game.height() ? this.game.ref(x, y + 1) : -1;

      // Check if both current and next tile are water (gateway condition)
      const isGateway = this.game.isWater(tile) &&
                       nextTile !== -1 &&
                       this.game.isWater(nextTile);

      if (isGateway) {
        if (gatewayStart === -1) {
          gatewayStart = x;
        }
      } else {
        // End of gateway stretch
        if (gatewayStart !== -1) {
          const gatewayLength = x - gatewayStart;
          gateways.push({
            id: this.nextGatewayId++,
            sectorX: sectorX,
            sectorY: Math.floor(y / TradeShipNavigator.SECTOR_SIZE),
            edge: 'bottom',
            x: gatewayStart,
            y: y,
            length: gatewayLength,
            tile: this.game.ref(gatewayStart, y)
          });
          gatewayStart = -1;
        }
      }
    }

    // Handle gateway extending to edge
    if (gatewayStart !== -1) {
      const gatewayLength = maxX - gatewayStart;
      gateways.push({
        id: this.nextGatewayId++,
        sectorX: sectorX,
        sectorY: Math.floor(y / TradeShipNavigator.SECTOR_SIZE),
        edge: 'bottom',
        x: gatewayStart,
        y: y,
        length: gatewayLength,
        tile: this.game.ref(gatewayStart, y)
      });
    }

    return gateways;
  }

  private buildSectorConnections(sector: Sector) {
    // Check connectivity between all pairs of gateways in this sector
    const gateways = sector.gateways;

    for (let i = 0; i < gateways.length; i++) {
      for (let j = i + 1; j < gateways.length; j++) {
        const cost = this.findLocalPathCost(gateways[i].tile, gateways[j].tile);

        if (cost !== -1) {
          // Bidirectional connection
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

          // Add to global connection map
          if (!this.gatewayConnections.has(gateways[i].id)) {
            this.gatewayConnections.set(gateways[i].id, []);
          }
          if (!this.gatewayConnections.has(gateways[j].id)) {
            this.gatewayConnections.set(gateways[j].id, []);
          }

          this.gatewayConnections.get(gateways[i].id)!.push(connection1);
          this.gatewayConnections.get(gateways[j].id)!.push(connection2);
        }
      }
    }
  }

  private findLocalPathCost(from: TileRef, to: TileRef): number {
    // Use minimap for faster connectivity checking
    const miniMap = this.game.miniMap();

    const miniFrom = miniMap.ref(
      Math.floor(this.game.x(from) / 2),
      Math.floor(this.game.y(from) / 2)
    );
    const miniTo = miniMap.ref(
      Math.floor(this.game.x(to) / 2),
      Math.floor(this.game.y(to) / 2)
    );

    // Use simple BFS on minimap with distance limit
    // Increased from SECTOR_SIZE to SECTOR_SIZE * 3 to allow cross-sector connections
    const maxDistance = TradeShipNavigator.SECTOR_SIZE * 3;
    const visited = new Set<TileRef>();
    const queue: { tile: TileRef; cost: number }[] = [{ tile: miniFrom, cost: 0 }];
    visited.add(miniFrom);

    while (queue.length > 0) {
      const current = queue.shift()!;

      if (current.tile === miniTo) {
        // Return cost scaled by 2 (approximate full-resolution distance)
        return current.cost * 2;
      }

      if (current.cost >= maxDistance) {
        continue;
      }

      const neighbors = miniMap.neighbors(current.tile);
      for (const neighbor of neighbors) {
        if (!visited.has(neighbor) && miniMap.isWater(neighbor)) {
          visited.add(neighbor);
          queue.push({ tile: neighbor, cost: current.cost + 1 });
        }
      }
    }

    return -1; // Not connected
  }

  // Find nearest gateway to a given tile
  private findNearestGateway(tile: TileRef): Gateway | null {
    const x = this.game.x(tile);
    const y = this.game.y(tile);

    // Check gateways in nearby sectors (current and adjacent)
    const sectorX = Math.floor(x / TradeShipNavigator.SECTOR_SIZE);
    const sectorY = Math.floor(y / TradeShipNavigator.SECTOR_SIZE);

    // Collect all candidate gateways with their distances
    const candidates: { gateway: Gateway; dist: number }[] = [];

    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        const sx = sectorX + dx;
        const sy = sectorY + dy;
        const sectorKey = this.getSectorKey(sx, sy);
        const sector = this.sectors.get(sectorKey);

        if (sector) {
          for (const gateway of sector.gateways) {
            const dist = this.game.manhattanDist(tile, gateway.tile);
            candidates.push({ gateway, dist });
          }
        }
      }
    }

    // Sort by distance (closest first)
    candidates.sort((a, b) => a.dist - b.dist);

    // Verify connectivity for candidates in order, return first connected one
    for (const candidate of candidates) {
      const cost = this.findLocalPathCost(tile, candidate.gateway.tile);
      if (cost !== -1) {
        return candidate.gateway;
      }
    }

    return null;
  }

  // A* search on gateway graph using SerialAStar
  private findGatewayPath(fromGatewayId: number, toGatewayId: number): number[] | null {
    const adapter = new GatewayGraphAdapter(
      this.gateways,
      this.gatewayConnections,
      this.game
    );

    const aStar = new SerialAStar(
      fromGatewayId,
      toGatewayId,
      10000, // iterations - gateway graph is much smaller
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

  // Main entry point for pathfinding
  findPath(from: TileRef, to: TileRef, debug: boolean = false): TileRef[] | null {
    if (!this.initialized) {
      this.initialize();
    }

    // Distance-based optimization: skip gateway graph for short distances
    const dist = this.game.manhattanDist(from, to);

    // For very short distances (< 100 tiles), skip gateway graph entirely
    if (dist < 100) {
      return this.findDetailedLocalPath(from, to);
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
      ...gatewayPath.map(gwId => this.gateways.get(gwId)!.tile),
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

    // Apply distance-based path smoothing
    // Skip smoothing for very short paths to avoid overhead
    const N = 100;
    const smoothedPath = initialPath.length > 2 * N
      ? this.smoothPath(initialPath, N)
      : initialPath;

    if (debug) console.log(`  [DEBUG] Smoothed path: ${initialPath.length} → ${smoothedPath.length} tiles`);

    return smoothedPath;
  }

  // Detailed local pathfinding using SerialAStar on minimap (like PathFinder.Mini)
  private findDetailedLocalPath(from: TileRef, to: TileRef): TileRef[] | null {
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

    const aStar = new SerialAStar(
      miniFrom,
      miniTo,
      10000, // iterations - increased to match PathFinder.Mini capabilities
      20,    // maxTries - same as PathFinder.Mini (allows up to 200k total iterations)
      adapter,
      0      // no direction change penalty
    );

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

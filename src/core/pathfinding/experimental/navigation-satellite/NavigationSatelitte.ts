import { Game } from '../../../game/Game';
import { GameMap, TileRef } from '../../../game/GameMap';
import { MiniAStar } from '../../MiniAStar';
import { PathFindResultType } from '../../AStar';
import { getWaterComponentId } from './WaterComponents';
import { FastBFS } from './FastBFS';
import { FastAStar, FastAStarAdapter } from './FastAStar';

interface Gateway {
  id: number;
  x: number;
  y: number;
  tile: TileRef;
  componentId: number;
}

interface Edge {
  from: number;
  to: number;
  cost: number;
  path?: TileRef[];
}

interface Sector {
  x: number;
  y: number;
  gateways: Gateway[];
  edges: Edge[];
}

interface DebugInfo {
  gatewayPath: number[] | null;
  gatewayWaypoints: Array<[number, number]> | null;
  initialPath: TileRef[] | null;
  smoothedPath: TileRef[] | null;
  allGateways: Array<{ x: number; y: number; }>;
  sectorSize: number;
  timings: { [key: string]: number };
}

class GatewayGraph {
  constructor(
    readonly sectors: ReadonlyMap<number, Sector>,
    readonly gateways: ReadonlyMap<number, Gateway>,
    readonly edges: ReadonlyMap<number, Edge[]>,
    readonly sectorSize: number,
    readonly sectorsX: number,
  ) {}

  getSectorKey(sectorX: number, sectorY: number): number {
    return sectorY * this.sectorsX + sectorX;
  }

  getSector(sectorX: number, sectorY: number): Sector | undefined {
    return this.sectors.get(this.getSectorKey(sectorX, sectorY));
  }

  getGateway(id: number): Gateway | undefined {
    return this.gateways.get(id);
  }

  getEdges(gatewayId: number): Edge[] {
    return this.edges.get(gatewayId) ?? [];
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

// FastAStarAdapter for gateway graph pathfinding
class FastGatewayGraphAdapter implements FastAStarAdapter {
  constructor(private graph: GatewayGraph) {}

  getNeighbors(node: number): number[] {
    const edges = this.graph.getEdges(node);
    return edges.map(edge => edge.to);
  }

  getCost(from: number, to: number): number {
    const edges = this.graph.getEdges(from);
    const edge = edges.find(edge => edge.to === to);
    return edge?.cost ?? 1;
  }

  heuristic(node: number, goal: number): number {
    const nodeGw = this.graph.getGateway(node);
    const goalGw = this.graph.getGateway(goal);
    if (!nodeGw || !goalGw) return 0;

    // Manhattan distance heuristic
    const dx = Math.abs(nodeGw.x - goalGw.x);
    const dy = Math.abs(nodeGw.y - goalGw.y);
    return dx + dy;
  }
}

class GatewayGraphBuilder {
  static readonly SECTOR_SIZE = 32;

  // Derived immutable state
  private readonly miniMap: GameMap;
  private readonly width: number;
  private readonly height: number;
  private readonly sectorsX: number;
  private readonly sectorsY: number;
  private readonly fastBFS: FastBFS;

  // Mutable build state
  private sectors = new Map<number, Sector>();
  private gateways = new Map<number, Gateway>();
  private edges = new Map<number, Edge[]>();
  private nextGatewayId = 0;

  constructor(
    private readonly game: Game,
    private readonly sectorSize: number,
    private readonly debug: boolean
  ) {
    this.miniMap = game.miniMap();
    this.width = this.miniMap.width();
    this.height = this.miniMap.height();
    this.sectorsX = Math.ceil(this.width / sectorSize);
    this.sectorsY = Math.ceil(this.height / sectorSize);
    this.fastBFS = new FastBFS(this.width * this.height);
  }

  build(): GatewayGraph {
    const startTime = performance.now();

    // Phase 1: Identify all gateways
    const phase1Start = performance.now();
    for (let sy = 0; sy < this.sectorsY; sy++) {
      for (let sx = 0; sx < this.sectorsX; sx++) {
        this.processSector(sx, sy);

        // Update nextGatewayId based on gateways created
        this.nextGatewayId = this.gateways.size > 0 ? Math.max(...this.gateways.keys()) + 1 : 0;
      }
    }
    const phase1End = performance.now();

    if (this.debug) console.log(`  Phase 1 (Gateway identification): ${(phase1End - phase1Start).toFixed(2)}ms`);

    // Phase 2: Build intra-sector edges
    const phase2Start = performance.now();
    let potentialBFSCalls = 0;
    let skippedByComponentFilter = 0;
    let skippedByManhattanDistance = 0;
    let successfulConnections = 0;

    for (const sector of this.sectors.values()) {
      const gws = sector.gateways;
      const numGateways = gws.length;
      potentialBFSCalls += (numGateways * (numGateways - 1)) / 2;

      // Count filter savings
      for (let i = 0; i < gws.length; i++) {
        for (let j = i + 1; j < gws.length; j++) {
          if (gws[i].componentId !== gws[j].componentId) {
            skippedByComponentFilter++;
          } else {
            // Check Manhattan distance (gateways are on miniMap)
            const dx = Math.abs(gws[i].x - gws[j].x);
            const dy = Math.abs(gws[i].y - gws[j].y);
            const manhattanDist = dx + dy;
            const maxDistance = this.sectorSize * 12;

            if (manhattanDist > maxDistance) {
              skippedByManhattanDistance++;
            }
          }
        }
      }

      this.buildSectorConnections(sector);

      successfulConnections += sector.edges.length / 2; // Divide by 2 because bidirectional
    }

    const actualBFSCalls = potentialBFSCalls - skippedByComponentFilter - skippedByManhattanDistance;
    const totalSkipped = skippedByComponentFilter + skippedByManhattanDistance;

    const phase2End = performance.now();
    if (this.debug) {
      console.log(`  Phase 2 (Connection building): ${(phase2End - phase2Start).toFixed(2)}ms`);
      console.log(`    Potential BFS calls: ${potentialBFSCalls}`);
      console.log(`    Skipped by component filter: ${skippedByComponentFilter} (${((skippedByComponentFilter / potentialBFSCalls) * 100).toFixed(1)}%)`);
      console.log(`    Skipped by Manhattan distance: ${skippedByManhattanDistance} (${((skippedByManhattanDistance / potentialBFSCalls) * 100).toFixed(1)}%)`);
      console.log(`    Total skipped: ${totalSkipped} (${((totalSkipped / potentialBFSCalls) * 100).toFixed(1)}%)`);
      console.log(`    Actual BFS calls: ${actualBFSCalls}`);
      console.log(`    Successful connections: ${successfulConnections} (${((successfulConnections / actualBFSCalls) * 100).toFixed(1)}% success rate)`);
    }

    const endTime = performance.now();
    if (this.debug) {
      console.log(`Gateway graph built in ${(endTime - startTime).toFixed(2)}ms`);
      console.log(`Total gateways: ${this.gateways.size}`);
      console.log(`Total sectors: ${this.sectors.size}`);
    }

    return new GatewayGraph(this.sectors, this.gateways, this.edges, this.sectorSize, this.sectorsX);
  }

  private getSectorKey(sectorX: number, sectorY: number): number {
    return sectorY * this.sectorsX + sectorX;
  }

  private processSector(sx: number, sy: number): void {
    const sectorKey = this.getSectorKey(sx, sy);
    let sector = this.sectors.get(sectorKey);
    if (!sector) {
      sector = { x: sx, y: sy, gateways: [], edges: [] };
      this.sectors.set(sectorKey, sector);
    }

    const baseX = sx * this.sectorSize;
    const baseY = sy * this.sectorSize;

    // Find gateways on right edge (if not the last column)
    if (sx < this.sectorsX - 1) {
      const edgeX = Math.min(baseX + this.sectorSize - 1, this.width - 1);

      const newGateways = this.findGatewaysOnVerticalEdge(edgeX, baseY);

      sector.gateways.push(...newGateways);

      // Register gateways
      for (const gateway of newGateways) {
        this.gateways.set(gateway.id, gateway);
      }

      // Also add these gateways to the adjacent sector on the right
      const rightSectorKey = this.getSectorKey(sx + 1, sy);
      let rightSector = this.sectors.get(rightSectorKey);

      if (!rightSector) {
        rightSector = { x: sx + 1, y: sy, gateways: [], edges: [] };
        this.sectors.set(rightSectorKey, rightSector);
      }

      rightSector.gateways.push(...newGateways);
    }

    // Find gateways on bottom edge (if not the last row)
    if (sy < this.sectorsY - 1) {
      const edgeY = Math.min(baseY + this.sectorSize - 1, this.height - 1);

      const newGateways = this.findGatewaysOnHorizontalEdge(
        edgeY, baseX,
        this.gateways.size > 0 ? Math.max(...this.gateways.keys()) + 1 : this.nextGatewayId
      );

      sector.gateways.push(...newGateways);

      // Register gateways
      for (const gateway of newGateways) {
        this.gateways.set(gateway.id, gateway);
      }

      // Also add these gateways to the adjacent sector below
      const bottomSectorKey = this.getSectorKey(sx, sy + 1);
      let bottomSector = this.sectors.get(bottomSectorKey);

      if (!bottomSector) {
        bottomSector = { x: sx, y: sy + 1, gateways: [], edges: [] };
        this.sectors.set(bottomSectorKey, bottomSector);
      }

      bottomSector.gateways.push(...newGateways);
    }
  }

  private findGatewaysOnVerticalEdge(
    x: number, baseY: number
  ): Gateway[] {
    const gateways: Gateway[] = [];
    const maxY = Math.min(baseY + this.sectorSize, this.height);

    let gatewayStart = -1;
    let currentId = this.nextGatewayId;

    for (let y = baseY; y < maxY; y++) {
      const tile = this.miniMap.ref(x, y);
      const nextTile = x + 1 < this.miniMap.width() ? this.miniMap.ref(x + 1, y) : -1;
      const isGateway = this.miniMap.isWater(tile) && nextTile !== -1 && this.miniMap.isWater(nextTile);

      if (isGateway) {
        if (gatewayStart === -1) {
          gatewayStart = y;
        }
      } else {
        if (gatewayStart !== -1) {
          const gatewayLength = y - gatewayStart;
          const midY = gatewayStart + Math.floor(gatewayLength / 2);

          gatewayStart = -1;
          const tile = this.miniMap.ref(x, midY);
          gateways.push({
            id: currentId++,
            x: x,
            y: midY,
            tile: tile,
            componentId: getWaterComponentId(this.miniMap, tile)
          });
        }
      }
    }

    if (gatewayStart !== -1) {
      const gatewayLength = maxY - gatewayStart;
      const midY = gatewayStart + Math.floor(gatewayLength / 2);

      const tile = this.miniMap.ref(x, midY);
      gateways.push({
        id: currentId++,
        x: x,
        y: midY,
        tile: tile,
        componentId: getWaterComponentId(this.miniMap, tile)
      });
    }

    return gateways;
  }

  private findGatewaysOnHorizontalEdge(
    y: number, baseX: number, startId: number
  ): Gateway[] {
    const gateways: Gateway[] = [];
    const maxX = Math.min(baseX + this.sectorSize, this.width);

    let gatewayStart = -1;
    let currentId = startId;

    for (let x = baseX; x < maxX; x++) {
      const tile = this.miniMap.ref(x, y);
      const nextTile = y + 1 < this.miniMap.height() ? this.miniMap.ref(x, y + 1) : -1;
      const isGateway = this.miniMap.isWater(tile) && nextTile !== -1 && this.miniMap.isWater(nextTile);

      if (isGateway) {
        if (gatewayStart === -1) {
          gatewayStart = x;
        }
      } else {
        if (gatewayStart !== -1) {
          const gatewayLength = x - gatewayStart;
          const midX = gatewayStart + Math.floor(gatewayLength / 2);

          gatewayStart = -1;
          const tile = this.miniMap.ref(midX, y);
          gateways.push({
            id: currentId++,
            x: midX,
            y: y,
            tile: tile,
            componentId: getWaterComponentId(this.miniMap, tile)
          });
        }
      }
    }

    if (gatewayStart !== -1) {
      const gatewayLength = maxX - gatewayStart;
      const midX = gatewayStart + Math.floor(gatewayLength / 2);

      const tile = this.miniMap.ref(midX, y);
      gateways.push({
        id: currentId++,
        x: midX,
        y: y,
        tile: tile,
        componentId: getWaterComponentId(this.miniMap, tile)
      });
    }

    return gateways;
  }

  private buildSectorConnections(sector: Sector): void {
    const gateways = sector.gateways;

    for (let i = 0; i < gateways.length; i++) {
      for (let j = i + 1; j < gateways.length; j++) {
        // Skip if gateways are in different water components (can't reach each other)
        if (gateways[i].componentId !== gateways[j].componentId) {
          continue;
        }

        // Skip if Manhattan distance exceeds reasonable limit
        // Gateways are already on miniMap, so use coordinates directly
        const dx = Math.abs(gateways[i].x - gateways[j].x);
        const dy = Math.abs(gateways[i].y - gateways[j].y);
        const manhattanDist = dx + dy;
        const maxDistance = this.sectorSize * 3;

        if (manhattanDist > maxDistance) {
          continue;
        }

        const cost = this.findLocalPathCost(gateways[i].tile, gateways[j].tile);

        if (cost === -1) {
          continue;
        }

        const edge1: Edge = {
          from: gateways[i].id,
          to: gateways[j].id,
          cost: cost
        };

        const edge2: Edge = {
          from: gateways[j].id,
          to: gateways[i].id,
          cost: cost
        };

        sector.edges.push(edge1, edge2);

        if (!this.edges.has(gateways[i].id)) {
          this.edges.set(gateways[i].id, []);
        }

        if (!this.edges.has(gateways[j].id)) {
          this.edges.set(gateways[j].id, []);
        }

        this.edges.get(gateways[i].id)!.push(edge1);
        this.edges.get(gateways[j].id)!.push(edge2);
      }
    }
  }

  private findLocalPathCost(from: TileRef, to: TileRef): number {
    const maxDistance = this.sectorSize * 3;

    const result = this.fastBFS.search(
      this.miniMap, from, maxDistance,
      (tile, dist) => {
        if (tile === to) {
          return dist;
        }
        return null;
      }
    );

    return result ?? -1;
  }

  // Static factory method for backward compatibility
  static build(
    game: Game,
    sectorSize: number = GatewayGraphBuilder.SECTOR_SIZE,
    debug: boolean = false
  ): GatewayGraph {
    const builder = new GatewayGraphBuilder(game, sectorSize, debug);
    return builder.build();
  }
}

export class NavigationSatellite {
  private graph!: GatewayGraph;
  private initialized = false;
  private fastBFS!: FastBFS;
  private fastAStar!: FastAStar;

  public debugInfo: DebugInfo | null = null;
  private debugTimeStart: number | null = null;

  constructor(
    private game: Game,
    private options: { cachePaths?: boolean } = {}
  ) {}

  initialize(debug: boolean = false) {
    this.graph = GatewayGraphBuilder.build(this.game, GatewayGraphBuilder.SECTOR_SIZE, debug);
    const miniMap = this.game.miniMap();
    this.fastBFS = new FastBFS(miniMap.width() * miniMap.height());

    // Size FastAStar based on number of gateways
    const maxGatewayId = Math.max(...this.graph.getAllGateways().map(gw => gw.id), 0);
    this.fastAStar = new FastAStar(maxGatewayId + 1);

    this.initialized = true;
  }

  findPath(
    from: TileRef,
    to: TileRef,
    debug: boolean = false
  ): TileRef[] | null {
    if (!this.initialized) {
      this.initialize(debug);
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
    const map = this.game.map();
    const miniMap = this.game.miniMap();

    if (debug) {
      this.debugInfo!.gatewayWaypoints = gatewayPath.map(gwId => {
        const gw = this.graph.getGateway(gwId)!;
        return [miniMap.x(gw.tile) * 2, miniMap.y(gw.tile) * 2];
      });
    }

    if (debug) this.debugTimeStart = performance.now();

    // 1. Find path from start to first gateway
    const firstGateway = this.graph.getGateway(gatewayPath[0])!;
    const firstGatewayTile = map.ref(miniMap.x(firstGateway.tile) * 2, miniMap.y(firstGateway.tile) * 2);
    const startSegment = this.findLocalPath(from, firstGatewayTile);

    if (!startSegment) {
      return null;
    }

    initialPath.push(...startSegment);

    // 2. Build path through gateways
    for (let i = 0; i < gatewayPath.length - 1; i++) {
      const fromGwId = gatewayPath[i];
      const toGwId = gatewayPath[i + 1];

      // Get the connection
      const edges = this.graph.getEdges(fromGwId);
      const edge = edges.find(edge => edge.to === toGwId);

      if (!edge) {
        return null;
      }

      if (edge.path) {
        // Use cached path if available
        initialPath.push(...edge.path.slice(1));
        continue;
      }

      const fromGw = this.graph.getGateway(fromGwId)!;
      const toGw = this.graph.getGateway(toGwId)!;
      const fromTile = map.ref(miniMap.x(fromGw.tile) * 2, miniMap.y(fromGw.tile) * 2);
      const toTile = map.ref(miniMap.x(toGw.tile) * 2, miniMap.y(toGw.tile) * 2);

      const segmentPath = this.findLocalPath(fromTile, toTile);

      if (!segmentPath) {
        return null;
      }

      // Skip first tile to avoid duplication
      initialPath.push(...segmentPath.slice(1));

      if (this.options.cachePaths) {
        // Cache the path for future reuse
        edge.path = segmentPath;
      }
    }

    // 3. Find path from last gateway to end
    const lastGateway = this.graph.getGateway(gatewayPath[gatewayPath.length - 1])!;
    const lastGatewayTile = map.ref(miniMap.x(lastGateway.tile) * 2, miniMap.y(lastGateway.tile) * 2);
    const endSegment = this.findLocalPath(lastGatewayTile, to);

    if (!endSegment) {
      return null;
    }

    // Skip first tile to avoid duplication
    initialPath.push(...endSegment.slice(1));

    if (debug) this.debugInfo!.timings.buildInitialPath = performance.now() - this.debugTimeStart!;
    if (debug) this.debugInfo!.initialPath = initialPath;

    if (debug) console.log(`  [DEBUG] Initial path: ${initialPath.length} tiles`);

    if (debug) this.debugTimeStart = performance.now();
    const smoothedPath = this.smoothPath(initialPath)
    if (debug) this.debugInfo!.timings.buildSmoothPath = performance.now() - this.debugTimeStart!;

    if (debug) console.log(`  [DEBUG] Smoothed path: ${initialPath.length} → ${smoothedPath.length} tiles`);
    if (debug) this.debugInfo!.smoothedPath = smoothedPath;

    return smoothedPath;
  }

  private getAllGatewaysDebugInfo(): Array<{ x: number; y: number; }> {
    return this.graph.getAllGateways().map(gw => ({ x: gw.x, y: gw.y }));
  }

  private findNearestGateway(tile: TileRef): Gateway | null {
    const map = this.game.map();
    const x = map.x(tile);
    const y = map.y(tile);

    // Convert to miniMap coordinates
    const miniMap = this.game.miniMap();
    const miniX = Math.floor(x / 2);
    const miniY = Math.floor(y / 2);
    const miniFrom = miniMap.ref(miniX, miniY);

    // Check gateways in nearby sectors (using miniMap coordinates)
    const sectorX = Math.floor(miniX / this.graph.sectorSize);
    const sectorY = Math.floor(miniY / this.graph.sectorSize);

    // Collect all candidate gateways from nearby sectors
    const candidateGateways = this.graph.getNearbySectorGateways(sectorX, sectorY);

    if (candidateGateways.length === 0) {
      return null;
    }

    // Use BFS to find the nearest reachable gateway (by water path distance)
    // This ensures we only find gateways in the same water region
    const maxDistance = this.graph.sectorSize * 24;

    return this.fastBFS.search(miniMap, miniFrom, maxDistance, (tile: TileRef, _dist: number) => {
      // Check if any candidate gateway is at this position
      // Gateways are now on miniMap, so compare directly
      const tileX = miniMap.x(tile);
      const tileY = miniMap.y(tile);

      for (const gateway of candidateGateways) {
        if (gateway.x === tileX && gateway.y === tileY) {
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
    const adapter = new FastGatewayGraphAdapter(this.graph);
    return this.fastAStar.search(fromGatewayId, toGatewayId, adapter, 100000);
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

    // Safety limit to prevent excessive memory allocation
    const maxTiles = 100000;
    let iterations = 0;

    while (true) {
      if (iterations++ > maxTiles) {
        return null; // Path too long
      }
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

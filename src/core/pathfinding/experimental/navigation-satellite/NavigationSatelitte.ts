import { Game } from '../../../game/Game';
import { GameMap, TileRef } from '../../../game/GameMap';
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
  sectorX: number;
  sectorY: number;
}

interface Sector {
  x: number;
  y: number;
  gateways: Gateway[];
  edges: Edge[];
}

type BuildDebugInfo = {
  sectors: number | null;
  gateways: number | null;
  edges: number | null;
  actualBFSCalls: number | null;
  potentialBFSCalls: number | null;
  skippedTotal: number | null;
  skippedByComponentFilter: number | null;
  skippedByManhattanDistance: number | null;
  timings: { [key: string]: number };
}

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

class BoundedGameMapAdapter implements FastAStarAdapter {
  private readonly minX: number;
  private readonly minY: number;
  private readonly width: number;
  private readonly height: number;
  readonly numNodes: number;
  private readonly startTile: TileRef;
  private readonly goalTile: TileRef;

  constructor(
    private miniMap: GameMap,
    startTile: TileRef,
    goalTile: TileRef,
    bounds: { minX: number; maxX: number; minY: number; maxY: number }
  ) {
    this.startTile = startTile;
    this.goalTile = goalTile;

    this.minX = bounds.minX;
    this.minY = bounds.minY;
    this.width = bounds.maxX - bounds.minX + 1;
    this.height = bounds.maxY - bounds.minY + 1;

    this.numNodes = this.width * this.height;
  }

  // Convert global TileRef to local node ID
  tileToNode(tile: TileRef): number {
    const x = this.miniMap.x(tile) - this.minX;
    const y = this.miniMap.y(tile) - this.minY;

    // Allow start and goal tiles to be outside bounds (matching graph building behavior)
    const isStartOrGoal = tile === this.startTile || tile === this.goalTile;
    if (!isStartOrGoal && (x < 0 || x >= this.width || y < 0 || y >= this.height)) {
      return -1; // Outside bounds
    }

    // Clamp coordinates for start/goal tiles that are outside bounds
    const clampedX = Math.max(0, Math.min(this.width - 1, x));
    const clampedY = Math.max(0, Math.min(this.height - 1, y));

    return clampedY * this.width + clampedX;
  }

  // Convert local node ID to global TileRef
  nodeToTile(node: number): TileRef {
    const localX = node % this.width;
    const localY = Math.floor(node / this.width);
    return this.miniMap.ref(localX + this.minX, localY + this.minY);
  }

  getNeighbors(node: number): number[] {
    const tile = this.nodeToTile(node);
    const neighbors = this.miniMap.neighbors(tile);
    const result: number[] = [];

    for (const neighborTile of neighbors) {
      if (!this.miniMap.isWater(neighborTile)) continue;

      const neighborNode = this.tileToNode(neighborTile);
      if (neighborNode !== -1) {
        result.push(neighborNode);
      }
    }

    return result;
  }

  getCost(from: number, to: number): number {
    return 1; // Uniform cost for water tiles
  }

  heuristic(node: number, goal: number): number {
    const nodeTile = this.nodeToTile(node);
    const goalTile = this.nodeToTile(goal);

    const dx = Math.abs(this.miniMap.x(nodeTile) - this.miniMap.x(goalTile));
    const dy = Math.abs(this.miniMap.y(nodeTile) - this.miniMap.y(goalTile));

    return dx + dy; // Manhattan distance
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

  // Programatically accessible debug info
  public debugInfo: BuildDebugInfo | null = null;

  constructor(
    private readonly game: Game,
    private readonly sectorSize: number,
  ) {
    this.miniMap = game.miniMap();
    this.width = this.miniMap.width();
    this.height = this.miniMap.height();
    this.sectorsX = Math.ceil(this.width / sectorSize);
    this.sectorsY = Math.ceil(this.height / sectorSize);
    this.fastBFS = new FastBFS(this.width * this.height);
  }

  build(debug: boolean): GatewayGraph {
    performance.mark('navsat:build:start');

    if (debug) {
      console.log(`[DEBUG] Building gateway graph with sector size ${this.sectorSize} (${this.sectorsX}x${this.sectorsY} sectors)`);

      this.debugInfo = {
        sectors: null,
        gateways: null,
        edges: null,
        actualBFSCalls: null,
        potentialBFSCalls: null,
        skippedTotal: null,
        skippedByComponentFilter: null,
        skippedByManhattanDistance: null,
        timings: {},
      }
    }

    performance.mark('navsat:build:phase1:start');
    for (let sy = 0; sy < this.sectorsY; sy++) {
      for (let sx = 0; sx < this.sectorsX; sx++) {
        this.processSector(sx, sy);

        // Update nextGatewayId based on gateways created
        this.nextGatewayId = this.gateways.size > 0 ? Math.max(...this.gateways.keys()) + 1 : 0;
      }
    }
    performance.mark('navsat:build:phase1:end');
    const phase1Measure = performance.measure(
      'navsat:build:phase1',
      'navsat:build:phase1:start',
      'navsat:build:phase1:end'
    );

    if (debug) {
      console.log(`[DEBUG] Phase 1 (Gateway identification): ${phase1Measure.duration.toFixed(2)}ms`);

      this.debugInfo!.edges = 0;
      this.debugInfo!.potentialBFSCalls = 0;
      this.debugInfo!.skippedByComponentFilter = 0;
      this.debugInfo!.skippedByManhattanDistance = 0;
    }

    performance.mark('navsat:build:phase2:start');
    for (const sector of this.sectors.values()) {
      const gws = sector.gateways;
      const numGateways = gws.length;

      if (debug) {
        this.debugInfo!.potentialBFSCalls! += (numGateways * (numGateways - 1)) / 2;

        for (let i = 0; i < gws.length; i++) {
          for (let j = i + 1; j < gws.length; j++) {
            if (gws[i].componentId !== gws[j].componentId) {
              this.debugInfo!.skippedByComponentFilter!++;
            } else {
              const dx = Math.abs(gws[i].x - gws[j].x);
              const dy = Math.abs(gws[i].y - gws[j].y);
              const manhattanDist = dx + dy;
              const maxDistance = this.sectorSize * 12;

              if (manhattanDist > maxDistance) {
                this.debugInfo!.skippedByManhattanDistance!++;
              }
            }
          }
        }
      }

      this.buildSectorConnections(sector);

      if (debug) {
        // Divide by 2 because bidirectional
        this.debugInfo!.edges! += sector.edges.length / 2;
      }
    }

    if (debug) {
      this.debugInfo!.actualBFSCalls = this.debugInfo!.potentialBFSCalls! - this.debugInfo!.skippedByComponentFilter! - this.debugInfo!.skippedByManhattanDistance!;
      this.debugInfo!.skippedTotal = this.debugInfo!.skippedByComponentFilter! + this.debugInfo!.skippedByManhattanDistance!;
    }

    performance.mark('navsat:build:phase2:end');
    const phase2Measure = performance.measure(
      'navsat:build:phase2',
      'navsat:build:phase2:start',
      'navsat:build:phase2:end'
    );

    if (debug) {
      console.log(`[DEBUG] Phase 2 (Connection building): ${phase2Measure.duration.toFixed(2)}ms`);
      console.log(`[DEBUG]   Potential BFS calls: ${this.debugInfo!.potentialBFSCalls}`);
      console.log(`[DEBUG]   Skipped by component filter: ${this.debugInfo!.skippedByComponentFilter} (${((this.debugInfo!.skippedByComponentFilter! / this.debugInfo!.potentialBFSCalls!) * 100).toFixed(1)}%)`);
      console.log(`[DEBUG]   Skipped by Manhattan distance: ${this.debugInfo!.skippedByManhattanDistance} (${((this.debugInfo!.skippedByManhattanDistance! / this.debugInfo!.potentialBFSCalls!) * 100).toFixed(1)}%)`);
      console.log(`[DEBUG]   Total skipped: ${this.debugInfo!.skippedTotal} (${((this.debugInfo!.skippedTotal! / this.debugInfo!.potentialBFSCalls!) * 100).toFixed(1)}%)`);
      console.log(`[DEBUG]   Actual BFS calls: ${this.debugInfo!.actualBFSCalls}`);
      console.log(`[DEBUG]   Edges Found: ${this.debugInfo!.edges} (${((this.debugInfo!.edges! / this.debugInfo!.actualBFSCalls!) * 100).toFixed(1)}% success rate)`);
    }

    performance.mark('navsat:build:end');
    const totalMeasure = performance.measure(
      'navsat:build:total',
      'navsat:build:start',
      'navsat:build:end'
    );

    if (debug) {
      console.log(`[DEBUG] Gateway graph built in ${totalMeasure.duration.toFixed(2)}ms`);
      console.log(`[DEBUG] Gateways: ${this.gateways.size}`);
      console.log(`[DEBUG] Sectors: ${this.sectors.size}`);
    }

    return new GatewayGraph(
      this.sectors,
      this.gateways,
      this.edges,
      this.sectorSize,
      this.sectorsX
    );
  }

  private getSectorKey(sectorX: number, sectorY: number): number {
    return sectorY * this.sectorsX + sectorX;
  }

  private addGatewayToSector(sector: Sector, gateway: Gateway): void {
    // Check if a gateway already exists at this position in the sector
    for (const existingGw of sector.gateways) {
      if (existingGw.x === gateway.x && existingGw.y === gateway.y) {
        // Gateway already exists in this sector, don't add duplicate
        return;
      }
    }

    // Gateway doesn't exist in sector yet, add it
    sector.gateways.push(gateway);
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

    if (sx < this.sectorsX - 1) {
      const edgeX = Math.min(baseX + this.sectorSize - 1, this.width - 1);
      const newGateways = this.findGatewaysOnVerticalEdge(edgeX, baseY);

      for (const gateway of newGateways) {
        this.gateways.set(gateway.id, gateway);
        this.addGatewayToSector(sector, gateway);

        const rightSectorKey = this.getSectorKey(sx + 1, sy);
        let rightSector = this.sectors.get(rightSectorKey);

        if (!rightSector) {
          rightSector = { x: sx + 1, y: sy, gateways: [], edges: [] };
          this.sectors.set(rightSectorKey, rightSector);
        }

        this.addGatewayToSector(rightSector, gateway);
      }

      // Update nextGatewayId to prevent ID collision with horizontal edge
      if (newGateways.length > 0) {
        this.nextGatewayId = Math.max(...this.gateways.keys()) + 1;
      }
    }

    if (sy < this.sectorsY - 1) {
      const edgeY = Math.min(baseY + this.sectorSize - 1, this.height - 1);
      const newGateways = this.findGatewaysOnHorizontalEdge(edgeY, baseX);

      for (const gateway of newGateways) {
        this.gateways.set(gateway.id, gateway);
        this.addGatewayToSector(sector, gateway);

        const bottomSectorKey = this.getSectorKey(sx, sy + 1);
        let bottomSector = this.sectors.get(bottomSectorKey);

        if (!bottomSector) {
          bottomSector = { x: sx, y: sy + 1, gateways: [], edges: [] };
          this.sectors.set(bottomSectorKey, bottomSector);
        }

        this.addGatewayToSector(bottomSector, gateway);
      }
    }
  }

  private findGatewaysOnVerticalEdge(
    x: number, baseY: number
  ): Gateway[] {
    const gateways: Gateway[] = [];
    const maxY = Math.min(baseY + this.sectorSize, this.height);

    let gatewayStart = -1;
    let currentId = this.nextGatewayId;

    const tryAddGateway = (y: number) => {
      if (gatewayStart === -1) return

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

    for (let y = baseY; y < maxY; y++) {
      const tile = this.miniMap.ref(x, y);
      const nextTile = x + 1 < this.miniMap.width() ? this.miniMap.ref(x + 1, y) : -1;
      const isGateway = this.miniMap.isWater(tile) && nextTile !== -1 && this.miniMap.isWater(nextTile);

      if (isGateway) {
        if (gatewayStart === -1) {
          gatewayStart = y;
        }
      } else {
        tryAddGateway(y)
      }
    }

    tryAddGateway(maxY);

    return gateways;
  }

  private findGatewaysOnHorizontalEdge(
    y: number, baseX: number
  ): Gateway[] {
    const gateways: Gateway[] = [];
    const maxX = Math.min(baseX + this.sectorSize, this.width);

    let gatewayStart = -1;
    let currentId = this.nextGatewayId;

    const tryAddGateway = (x: number) => {
      if (gatewayStart === -1) return

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

    for (let x = baseX; x < maxX; x++) {
      const tile = this.miniMap.ref(x, y);
      const nextTile = y + 1 < this.miniMap.height() ? this.miniMap.ref(x, y + 1) : -1;
      const isGateway = this.miniMap.isWater(tile) && nextTile !== -1 && this.miniMap.isWater(nextTile);

      if (isGateway) {
        if (gatewayStart === -1) {
          gatewayStart = x;
        }
      } else {
        tryAddGateway(x);
      }
    }

    tryAddGateway(maxX);

    return gateways;
  }

  private buildSectorConnections(sector: Sector): void {
    const gateways = sector.gateways;

    // Calculate bounding box once for this sector
    const sectorMinX = sector.x * this.sectorSize;
    const sectorMinY = sector.y * this.sectorSize;
    const sectorMaxX = Math.min(this.width - 1, sectorMinX + this.sectorSize - 1);
    const sectorMaxY = Math.min(this.height - 1, sectorMinY + this.sectorSize - 1);

    for (let i = 0; i < gateways.length; i++) {
      const fromGateway = gateways[i];

      // Build list of target gateways (only those we haven't processed yet)
      const targetGateways: Gateway[] = [];
      for (let j = i + 1; j < gateways.length; j++) {
        // Skip if gateways are in different water components
        if (gateways[i].componentId !== gateways[j].componentId) {
          continue;
        }

        targetGateways.push(gateways[j]);
      }

      if (targetGateways.length === 0) {
        continue;
      }

      // Single BFS to find all reachable target gateways
      const reachableGateways = this.findAllReachableGatewaysInBounds(
        fromGateway.tile,
        targetGateways,
        sectorMinX,
        sectorMaxX,
        sectorMinY,
        sectorMaxY
      );

      // Create edges for all reachable gateways
      for (const [targetId, cost] of reachableGateways.entries()) {
        if (!this.edges.has(fromGateway.id)) {
          this.edges.set(fromGateway.id, []);
        }

        if (!this.edges.has(targetId)) {
          this.edges.set(targetId, []);
        }

        // Check for existing edges - gateways may live in 2 sectors, keep only cheaper connection
        const existingEdgeFromI = this.edges.get(fromGateway.id)!.find(e => e.to === targetId);
        const existingEdgeFromJ = this.edges.get(targetId)!.find(e => e.to === fromGateway.id);

        // If edge doesn't exist or new cost is cheaper, update it
        if (!existingEdgeFromI || cost < existingEdgeFromI.cost) {
          const edge1: Edge = {
            from: fromGateway.id,
            to: targetId,
            cost: cost,
            sectorX: sector.x,
            sectorY: sector.y
          };

          const edge2: Edge = {
            from: targetId,
            to: fromGateway.id,
            cost: cost,
            sectorX: sector.x,
            sectorY: sector.y
          };

          // Add to sector edges for tracking
          sector.edges.push(edge1, edge2);

          if (existingEdgeFromI) {
            const idx1 = this.edges.get(fromGateway.id)!.indexOf(existingEdgeFromI);
            this.edges.get(fromGateway.id)![idx1] = edge1;

            const idx2 = this.edges.get(targetId)!.indexOf(existingEdgeFromJ!);
            this.edges.get(targetId)![idx2] = edge2;
          } else {
            this.edges.get(fromGateway.id)!.push(edge1);
            this.edges.get(targetId)!.push(edge2);
          }
        }
      }
    }
  }

  private findAllReachableGatewaysInBounds(
    from: TileRef,
    targetGateways: Gateway[],
    minX: number,
    maxX: number,
    minY: number,
    maxY: number
  ): Map<number, number> {
    const fromX = this.miniMap.x(from);
    const fromY = this.miniMap.y(from);

    // Create a map of tile positions to gateway IDs for fast lookup
    const tileToGateway = new Map<TileRef, number>();
    let maxManhattanDist = 0;

    for (const gateway of targetGateways) {
      tileToGateway.set(gateway.tile, gateway.id);
      const dx = Math.abs(gateway.x - fromX);
      const dy = Math.abs(gateway.y - fromY);
      maxManhattanDist = Math.max(maxManhattanDist, dx + dy);
    }

    const maxDistance = maxManhattanDist * 3; // Allow path deviation
    const reachable = new Map<number, number>();
    let foundCount = 0;

    this.fastBFS.search(
      this.miniMap, from, maxDistance,
      (tile, dist) => {
        const x = this.miniMap.x(tile);
        const y = this.miniMap.y(tile);

        // Reject if outside of bounding box
        const isStartOrEnd = tile === from || tileToGateway.has(tile);
        if (!isStartOrEnd && (x < minX || x > maxX || y < minY || y > maxY)) {
          return null;
        }

        // Check if this tile is one of our target gateways
        const gatewayId = tileToGateway.get(tile);

        if (gatewayId !== undefined) {
          reachable.set(gatewayId, dist);
          foundCount++;

          // Early exit if we've found all target gateways
          if (foundCount === targetGateways.length) {
            return dist; // Return to stop BFS
          }
        }
      }
    );

    return reachable;
  }
}

export class NavigationSatellite {
  private graph!: GatewayGraph;
  private initialized = false;
  private fastBFS!: FastBFS;
  private gatewayAStar!: FastAStar;
  private localAStar!: FastAStar;
  private localAStarMultiSector!: FastAStar;

  public debugInfo: PathDebugInfo | null = null;

  constructor(
    private game: Game,
    private options: {
      cachePaths?: boolean,
    } = {}
  ) {}

  initialize(debug: boolean = false) {
    const gatewayGraphBuilder = new GatewayGraphBuilder(this.game, GatewayGraphBuilder.SECTOR_SIZE);
    this.graph = gatewayGraphBuilder.build(debug);

    const miniMap = this.game.miniMap();
    this.fastBFS = new FastBFS(miniMap.width() * miniMap.height());

    const gatewayCount = Math.max(this.graph.getAllGateways().length);
    this.gatewayAStar = new FastAStar(gatewayCount);

    // Fixed-size FastAStar for sector-bounded local pathfinding
    // Single sector: 32×32 = 1,024 nodes
    const sectorSize = GatewayGraphBuilder.SECTOR_SIZE;
    const maxLocalNodes = sectorSize * sectorSize; // 1,024 nodes
    this.localAStar = new FastAStar(maxLocalNodes);

    // Multi-sector FastAStar for cross-sector pathfinding (same gateway, different sectors)
    // 3×3 sectors: 96×96 = 9,216 nodes
    const multiSectorSize = sectorSize * 3;
    const maxMultiSectorNodes = multiSectorSize * multiSectorSize;
    this.localAStarMultiSector = new FastAStar(maxMultiSectorNodes);

    this.initialized = true;
  }

  findPath(
    from: TileRef,
    to: TileRef,
    debug: boolean = false
  ): TileRef[] | null {
    if (!this.initialized) {
      throw new Error('NavigationSatellite not initialized. Call initialize() before using findPath().');
    }

    if (debug) {
      // Collect all edges with their paths for visualization
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

          // Only add each edge once (not both directions)
          // Include self-loops (fromId === edge.to) for debugging
          if (fromId <= edge.to) {
            allEdges.push({
              fromId: fromId,
              toId: edge.to,
              from: fromGw.tile,
              to: toGw.tile,
              cost: edge.cost,
              path: edge.path ?? null
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
          gateways: this.graph.getAllGateways().map(gw => ({ id: gw.id, tile: gw.tile })),
          edges: allEdges
        },
        timings: {},
      };
    }

    const dist = this.game.manhattanDist(from, to);

    // Early exit for very short distances that fit within multi-sector range
    if (dist <= this.graph.sectorSize) {
      performance.mark('navsat:findPath:earlyExitLocalPath:start');
      const map = this.game.map();
      const startMiniX = Math.floor(map.x(from) / 2);
      const startMiniY = Math.floor(map.y(from) / 2);
      const sectorX = Math.floor(startMiniX / this.graph.sectorSize);
      const sectorY = Math.floor(startMiniY / this.graph.sectorSize);
      const localPath = this.findLocalPath(from, to, sectorX, sectorY, 2000, true);
      performance.mark('navsat:findPath:earlyExitLocalPath:end');
      const measure = performance.measure(
        'navsat:findPath:earlyExitLocalPath',
        'navsat:findPath:earlyExitLocalPath:start',
        'navsat:findPath:earlyExitLocalPath:end'
      );

      if (debug) {
        this.debugInfo!.timings.earlyExitLocalPath = measure.duration;
      }

      if (localPath) {
        if (debug) {
          console.log(`[DEBUG] Direct local path found for dist=${dist}, length=${localPath.length}`);
        }

        return localPath;
      }

      if (debug) {
        console.log(`[DEBUG] Direct path failed for dist=${dist}, falling back to gateway graph`);
      }
    }

    performance.mark('navsat:findPath:findGateways:start');
    const startGateway = this.findNearestGateway(from);
    const endGateway = this.findNearestGateway(to);
    performance.mark('navsat:findPath:findGateways:end');
    const findGatewaysMeasure = performance.measure(
      'navsat:findPath:findGateways',
      'navsat:findPath:findGateways:start',
      'navsat:findPath:findGateways:end'
    );

    if (debug) {
      this.debugInfo!.timings.findGateways = findGatewaysMeasure.duration;
    }

    if (!startGateway) {
      if (debug) {
        console.log(`[DEBUG] Cannot find start gateway for (${this.game.x(from)}, ${this.game.y(from)})`);
      }

      return null;
    }

    if (!endGateway) {
      if (debug) {
        console.log(`[DEBUG] Cannot find end gateway for (${this.game.x(to)}, ${this.game.y(to)})`);
      }

      return null;
    }

    if (startGateway.id === endGateway.id) {
      if (debug) {
        console.log(`[DEBUG] Start and end gateways are the same (ID=${startGateway.id}), finding local path with multi-sector search`);
      }

      const sectorX = Math.floor(startGateway.x / this.graph.sectorSize);
      const sectorY = Math.floor(startGateway.y / this.graph.sectorSize);
      return this.findLocalPath(from, to, sectorX, sectorY, 10000, true);
    }

    performance.mark('navsat:findPath:findGatewayPath:start');
    const gatewayPath = this.findGatewayPath(startGateway.id, endGateway.id);
    performance.mark('navsat:findPath:findGatewayPath:end');
    const findGatewayPathMeasure = performance.measure(
      'navsat:findPath:findGatewayPath',
      'navsat:findPath:findGatewayPath:start',
      'navsat:findPath:findGatewayPath:end'
    );

    if (debug) {
      this.debugInfo!.timings.findGatewayPath = findGatewayPathMeasure.duration;

      this.debugInfo!.gatewayPath = gatewayPath ? gatewayPath.map(gwId => {
        const gw = this.graph.getGateway(gwId);
        return gw ? gw.tile : -1;
      }).filter(tile => tile !== -1) : null;
    }

    if (!gatewayPath) {
      if (debug) {
        console.log(`[DEBUG] No gateway path between gateways ${startGateway.id} and ${endGateway.id}`);
      }
      
      return null;
    }

    if (debug) {
      console.log(`[DEBUG] Gateway path found: ${gatewayPath.length} waypoints`);
    }

    const initialPath: TileRef[] = [];
    const map = this.game.map();
    const miniMap = this.game.miniMap();

    performance.mark('navsat:findPath:buildInitialPath:start');

    // 1. Find path from start to first gateway
    const firstGateway = this.graph.getGateway(gatewayPath[0])!;
    const firstGatewayTile = map.ref(miniMap.x(firstGateway.tile) * 2, miniMap.y(firstGateway.tile) * 2);

    // Use start position's sector with multi-sector search (gateway may be on border)
    const startMiniX = Math.floor(map.x(from) / 2);
    const startMiniY = Math.floor(map.y(from) / 2);
    const startSectorX = Math.floor(startMiniX / this.graph.sectorSize);
    const startSectorY = Math.floor(startMiniY / this.graph.sectorSize);
    const startSegment = this.findLocalPath(from, firstGatewayTile, startSectorX, startSectorY);

    if (!startSegment) {
      return null;
    }

    initialPath.push(...startSegment);

    // 2. Build path through gateways
    for (let i = 0; i < gatewayPath.length - 1; i++) {
      const fromGwId = gatewayPath[i];
      const toGwId = gatewayPath[i + 1];

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

      const segmentPath = this.findLocalPath(fromTile, toTile, edge.sectorX, edge.sectorY);

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

    // Use end position's sector with multi-sector search (gateway may be on border)
    const endMiniX = Math.floor(map.x(to) / 2);
    const endMiniY = Math.floor(map.y(to) / 2);
    const endSectorX = Math.floor(endMiniX / this.graph.sectorSize);
    const endSectorY = Math.floor(endMiniY / this.graph.sectorSize);
    const endSegment = this.findLocalPath(lastGatewayTile, to, endSectorX, endSectorY);

    if (!endSegment) {
      return null;
    }

    // Skip first tile to avoid duplication
    initialPath.push(...endSegment.slice(1));

    performance.mark('navsat:findPath:buildInitialPath:end');
    const buildInitialPathMeasure = performance.measure(
      'navsat:findPath:buildInitialPath',
      'navsat:findPath:buildInitialPath:start',
      'navsat:findPath:buildInitialPath:end'
    );

    if (debug) {
      this.debugInfo!.timings.buildInitialPath = buildInitialPathMeasure.duration;
      this.debugInfo!.initialPath = initialPath;
      console.log(`[DEBUG] Initial path: ${initialPath.length} tiles`);
    }

    performance.mark('navsat:findPath:smoothPath:start');
    const smoothedPath = this.smoothPath(initialPath);
    performance.mark('navsat:findPath:smoothPath:end');
    const smoothPathMeasure = performance.measure(
      'navsat:findPath:smoothPath',
      'navsat:findPath:smoothPath:start',
      'navsat:findPath:smoothPath:end'
    );

    if (debug) {
      this.debugInfo!.timings.buildSmoothPath = smoothPathMeasure.duration
      this.debugInfo!.smoothPath = smoothedPath;
      console.log(`[DEBUG] Smoothed path: ${initialPath.length} → ${smoothedPath.length} tiles`);
    }

    return smoothedPath;
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

    // Check gateways in the tile's own sector (using miniMap coordinates)
    const sectorX = Math.floor(miniX / this.graph.sectorSize);
    const sectorY = Math.floor(miniY / this.graph.sectorSize);

    // Calculate single sector bounds
    const sectorSize = this.graph.sectorSize;
    const minX = sectorX * sectorSize;
    const minY = sectorY * sectorSize;
    const maxX = Math.min(miniMap.width() - 1, minX + sectorSize - 1);
    const maxY = Math.min(miniMap.height() - 1, minY + sectorSize - 1);

    // Get gateways from the tile's own sector only (includes border gateways)
    const sector = this.graph.getSector(sectorX, sectorY);

    if (!sector) {
      return null;
    }

    const candidateGateways = sector.gateways;
    if (candidateGateways.length === 0) {
      return null;
    }

    // Use BFS to find the nearest reachable gateway (by water path distance)
    // Search space is bounded by sector bounds, so maxDistance can be large
    const maxDistance = sectorSize * sectorSize;

    return this.fastBFS.search(miniMap, miniFrom, maxDistance, (tile: TileRef, _dist: number) => {
      const tileX = miniMap.x(tile);
      const tileY = miniMap.y(tile);

      // Check if any candidate gateway is at this position first
      for (const gateway of candidateGateways) {
        if (gateway.x === tileX && gateway.y === tileY) {
          return gateway;
        }
      }

      // Reject non-gateway tiles outside the sector bounds
      if (tileX < minX || tileX > maxX || tileY < minY || tileY > maxY) {
        return null;
      }
    });
  }

  private findGatewayPath(
    fromGatewayId: number,
    toGatewayId: number
  ): number[] | null {
    const adapter = new FastGatewayGraphAdapter(this.graph);
    return this.gatewayAStar.search(fromGatewayId, toGatewayId, adapter, 100000);
  }

  private findLocalPath(
    from: TileRef,
    to: TileRef,
    sectorX: number,
    sectorY: number,
    maxIterations: number = 10000,
    multiSector: boolean = false
  ): TileRef[] | null {
    const map = this.game.map();
    const miniMap = this.game.miniMap();

    // Convert full map coordinates to miniMap coordinates
    const miniFrom = miniMap.ref(
      Math.floor(map.x(from) / 2),
      Math.floor(map.y(from) / 2)
    );

    const miniTo = miniMap.ref(
      Math.floor(map.x(to) / 2),
      Math.floor(map.y(to) / 2)
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

    const adapter = new BoundedGameMapAdapter(miniMap, miniFrom, miniTo, { minX, maxX, minY, maxY });

    // Convert to local node IDs
    const startNode = adapter.tileToNode(miniFrom);
    const goalNode = adapter.tileToNode(miniTo);

    if (startNode === -1 || goalNode === -1) {
      return null; // Start or goal outside bounds
    }

    // Choose the appropriate FastAStar buffer based on search area
    const selectedAStar = multiSector ? this.localAStarMultiSector : this.localAStar;

    // Run FastAStar on bounded region
    const path = selectedAStar.search(startNode, goalNode, adapter, maxIterations);

    if (!path) {
      return null;
    }

    // Convert path from local node IDs back to miniMap TileRefs
    const miniPath = path.map((node: number) => adapter.nodeToTile(node));

    // Upscale from miniMap to full map (same logic as MiniAStar)
    const result = this.upscalePathToFullMap(miniPath, from, to);

    return result;
  }

  private upscalePathToFullMap(miniPath: TileRef[], from: TileRef, to: TileRef): TileRef[] {
    const map = this.game.map();
    const miniMap = this.game.miniMap();

    // Convert miniMap path to cells
    const miniCells = miniPath.map(tile => ({
      x: miniMap.x(tile),
      y: miniMap.y(tile)
    }));

    // FIRST: Scale all points (2x)
    const scaledPath = miniCells.map(point => ({
      x: point.x * 2,
      y: point.y * 2
    }));

    // SECOND: Interpolate between scaled points
    const smoothPath: Array<{ x: number; y: number }> = [];
    for (let i = 0; i < scaledPath.length - 1; i++) {
      const current = scaledPath[i];
      const next = scaledPath[i + 1];

      // Add the current point
      smoothPath.push(current);

      // Calculate dx/dy from SCALED coordinates
      const dx = next.x - current.x;
      const dy = next.y - current.y;
      const distance = Math.max(Math.abs(dx), Math.abs(dy));
      const steps = distance;

      // Add intermediate points
      for (let step = 1; step < steps; step++) {
        smoothPath.push({
          x: Math.round(current.x + (dx * step) / steps),
          y: Math.round(current.y + (dy * step) / steps)
        });
      }
    }

    // Add last point
    if (scaledPath.length > 0) {
      smoothPath.push(scaledPath[scaledPath.length - 1]);
    }

    const scaledCells = smoothPath;

    // Fix extremes to ensure exact start/end
    const fromCell = { x: map.x(from), y: map.y(from) };
    const toCell = { x: map.x(to), y: map.y(to) };

    // Ensure start is correct
    const startIdx = scaledCells.findIndex(c => c.x === fromCell.x && c.y === fromCell.y);
    if (startIdx === -1) {
      scaledCells.unshift(fromCell);
    } else if (startIdx !== 0) {
      scaledCells.splice(0, startIdx);
    }

    // Ensure end is correct
    const endIdx = scaledCells.findIndex(c => c.x === toCell.x && c.y === toCell.y);
    if (endIdx === -1) {
      scaledCells.push(toCell);
    } else if (endIdx !== scaledCells.length - 1) {
      scaledCells.splice(endIdx + 1);
    }

    // Convert back to TileRefs
    return scaledCells.map(cell => map.ref(cell.x, cell.y));
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

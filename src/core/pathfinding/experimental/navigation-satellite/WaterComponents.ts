import { GameMap, TileRef } from '../../../game/GameMap';

/**
 * Manages water component identification using flood-fill.
 * Pre-allocates buffers and provides explicit initialization.
 */
export class WaterComponent {
  private readonly map: GameMap;
  private readonly width: number;
  private readonly height: number;
  private readonly numTiles: number;
  private readonly lastRowStart: number;
  private readonly queue: Int32Array;
  private componentIds: Uint32Array | null = null;

  constructor(map: GameMap) {
    this.map = map;
    this.width = map.width();
    this.height = map.height();
    this.numTiles = this.width * this.height;
    this.lastRowStart = (this.height - 1) * this.width;
    this.queue = new Int32Array(this.numTiles);
  }

  initialize(): void {
    const ids = new Uint32Array(this.numTiles);
    let nextId = 0;

    // Scan all tiles and flood-fill each unvisited water component
    for (let start = 0; start < this.numTiles; start++) {
      if (ids[start] !== 0 || !this.map.isWater(start)) continue;

      nextId++;
      this.floodFillComponent(ids, start, nextId);
    }

    this.componentIds = ids;
  }

  /**
   * Flood-fill a single connected water component starting from the given tile.
   * Uses BFS to mark all connected water tiles with the same component ID.
   */
  private floodFillComponent(ids: Uint32Array, start: number, componentId: number): void {
    ids[start] = componentId;

    let head = 0;
    let tail = 0;
    this.queue[tail++] = start;

    while (head < tail) {
      const node = this.queue[head++]!;
      const x = node % this.width;

      // North
      if (node >= this.width) {
        const neighbor = node - this.width;
        if (ids[neighbor] === 0 && this.map.isWater(neighbor)) {
          ids[neighbor] = componentId;
          this.queue[tail++] = neighbor;
        }
      }

      // South
      if (node < this.lastRowStart) {
        const neighbor = node + this.width;
        if (ids[neighbor] === 0 && this.map.isWater(neighbor)) {
          ids[neighbor] = componentId;
          this.queue[tail++] = neighbor;
        }
      }

      // West
      if (x !== 0) {
        const neighbor = node - 1;
        if (ids[neighbor] === 0 && this.map.isWater(neighbor)) {
          ids[neighbor] = componentId;
          this.queue[tail++] = neighbor;
        }
      }

      // East
      if (x !== this.width - 1) {
        const neighbor = node + 1;
        if (ids[neighbor] === 0 && this.map.isWater(neighbor)) {
          ids[neighbor] = componentId;
          this.queue[tail++] = neighbor;
        }
      }
    }
  }

  /**
   * Get the component ID for a tile.
   * Returns 0 for land tiles or if not initialized.
   */
  getComponentId(tile: TileRef): number {
    if (!this.componentIds) return 0;
    if (!this.map.isWater(tile)) return 0;
    return this.componentIds[tile] ?? 0;
  }
}


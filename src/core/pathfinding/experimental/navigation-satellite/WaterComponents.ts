import { GameMap, TileRef } from '../../../game/GameMap';

const LAND_MARKER = 0xFF; // Must fit in Uint8Array

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
  private componentIds: Uint8Array | Uint16Array | null = null;

  // HACK: Direct access to terrain data for performance
  private readonly terrain: Uint8Array;

  constructor(map: GameMap) {
    this.map = map;
    this.width = map.width();
    this.height = map.height();
    this.numTiles = this.width * this.height;
    this.lastRowStart = (this.height - 1) * this.width;
    this.queue = new Int32Array(this.numTiles);

    // HACK: Access private terrain field for performance
    this.terrain = (map as any).terrain;
  }

  initialize(): void {
    let ids: Uint8Array | Uint16Array = new Uint8Array(this.numTiles);

    // Pre-mark all land tiles to optimize flood-fill
    this.premarkLandTiles(ids);

    let nextId = 0;

    // Scan all tiles and flood-fill each unvisited water component
    for (let start = 0; start < this.numTiles; start++) {
      const value = ids[start];

      // Skip if already visited (land=0xFF or water component >0)
      if (value === LAND_MARKER || value > 0) {
        continue;
      }

      nextId++;

      // Dynamically upgrade to Uint16Array when we hit component 254
      if (nextId === 254 && ids instanceof Uint8Array) {
        ids = this.upgradeToUint16Array(ids);
      }

      this.floodFillComponent(ids, start, nextId);
    }

    this.componentIds = ids;
  }

  /**
   * Pre-mark all land tiles in the ids array.
   * Processes 4 bytes at a time for better performance.
   * Land tiles are marked with 0xFF, water tiles remain 0.
   *
   * Always called with Uint8Array (before potential upgrade to Uint16Array).
   */
  private premarkLandTiles(ids: Uint8Array): void {
    // Write 4 bytes at once using Uint32Array view for better performance
    const numChunks = Math.floor(this.numTiles / 4);
    const terrain32 = new Uint32Array(this.terrain.buffer, this.terrain.byteOffset, numChunks);
    const ids32 = new Uint32Array(ids.buffer, ids.byteOffset, numChunks);

    for (let i = 0; i < numChunks; i++) {
      const chunk = terrain32[i];

      // Extract bit 7 from each byte, negate, and combine into single 32-bit write
      // bit 7 = 0 (water) → -(0) = 0x00
      // bit 7 = 1 (land)  → -(1) = 0xFF (truncated to 8 bits)
      const b0 = -((chunk >> 7) & 1) & 0xFF;
      const b1 = -((chunk >> 15) & 1) & 0xFF;
      const b2 = -((chunk >> 23) & 1) & 0xFF;
      const b3 = -((chunk >> 31) & 1);  // Upper byte, no mask needed

      ids32[i] = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
    }

    // Handle remaining tiles (when numTiles not divisible by 4)
    for (let i = numChunks * 4; i < this.numTiles; i++) {
      ids[i] = -(this.terrain[i] >> 7);
    }
  }

  /**
   * Upgrade from Uint8Array to Uint16Array when we exceed 254 components.
   * Direct copy works because both use 0xFF for land marker.
   */
  private upgradeToUint16Array(ids: Uint8Array): Uint16Array {
    const newIds = new Uint16Array(this.numTiles);
    for (let i = 0; i < this.numTiles; i++) {
      newIds[i] = ids[i];
    }
    return newIds;
  }

  /**
   * Flood-fill a single connected water component using scan-line algorithm.
   * Processes horizontal spans of tiles for better memory locality and cache performance.
   *
   * Note: Land tiles are pre-marked, so ids[x] === 0 guarantees water tile.
   */
  private floodFillComponent(ids: Uint8Array | Uint16Array, start: number, componentId: number): void {
    let head = 0;
    let tail = 0;
    this.queue[tail++] = start;

    while (head < tail) {
      const seed = this.queue[head++]!;

      // Skip if already processed
      if (ids[seed] !== 0) continue;

      // Scan left to find the start of this horizontal water span
      // No isWaterFast check needed - ids[x] === 0 guarantees water
      let left = seed;
      const rowStart = seed - (seed % this.width);
      while (left > rowStart && ids[left - 1] === 0) {
        left--;
      }

      // Scan right to find the end of this horizontal water span
      let right = seed;
      const rowEnd = rowStart + this.width - 1;
      while (right < rowEnd && ids[right + 1] === 0) {
        right++;
      }

      // Fill the entire horizontal span and check above/below for new spans
      for (let x = left; x <= right; x++) {
        ids[x] = componentId;

        // Check tile above (if not in first row)
        if (x >= this.width) {
          const above = x - this.width;
          if (ids[above] === 0) {
            this.queue[tail++] = above;
          }
        }

        // Check tile below (if not in last row)
        if (x < this.lastRowStart) {
          const below = x + this.width;
          if (ids[below] === 0) {
            this.queue[tail++] = below;
          }
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


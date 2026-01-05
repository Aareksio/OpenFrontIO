import { GameMap, TileRef } from '../../../game/GameMap';

// Optimized BFS using stamp-based visited tracking and typed array queue
export class FastBFS {
  private stamp = 1;
  private readonly visitedStamp: Uint32Array;
  private readonly queue: Int32Array;
  private readonly dist: Uint16Array;

  constructor(numTiles: number) {
    this.visitedStamp = new Uint32Array(numTiles);
    this.queue = new Int32Array(numTiles);
    this.dist = new Uint16Array(numTiles);
  }

  private nextStamp(): number {
    const stamp = this.stamp++;
    if (this.stamp === 0) {
      // Overflow - reset (extremely rare)
      this.visitedStamp.fill(0);
      this.stamp = 1;
    }
    return stamp;
  }

  search<T>(
    map: GameMap,
    start: TileRef,
    maxDistance: number,
    visitor: (tile: TileRef, dist: number) => T | null | undefined
  ): T | null {
    const stamp = this.nextStamp();
    const w = map.width();
    const h = map.height();
    const lastRowStart = (h - 1) * w;

    let head = 0;
    let tail = 0;

    this.visitedStamp[start] = stamp;
    this.dist[start] = 0;
    this.queue[tail++] = start;

    while (head < tail) {
      const node = this.queue[head++] as TileRef;
      const currentDist = this.dist[node];

      if (currentDist > maxDistance) {
        continue;
      }

      // Call visitor:
      // - Returns T: Found target, return immediately
      // - Returns null: Reject tile, don't explore neighbors
      // - Returns undefined: Valid tile, explore neighbors
      const result = visitor(node, currentDist);
      if (result !== null && result !== undefined) {
        return result;
      }

      // If visitor returned null, reject this tile and don't explore neighbors
      if (result === null) {
        continue;
      }

      const nextDist = currentDist + 1;
      const x = node % w;

      // North
      if (node >= w) {
        const n = node - w;
        if (this.visitedStamp[n] !== stamp && map.isWater(n)) {
          this.visitedStamp[n] = stamp;
          this.dist[n] = nextDist;
          this.queue[tail++] = n;
        }
      }

      // South
      if (node < lastRowStart) {
        const s = node + w;
        if (this.visitedStamp[s] !== stamp && map.isWater(s)) {
          this.visitedStamp[s] = stamp;
          this.dist[s] = nextDist;
          this.queue[tail++] = s;
        }
      }

      // West
      if (x !== 0) {
        const wv = node - 1;
        if (this.visitedStamp[wv] !== stamp && map.isWater(wv)) {
          this.visitedStamp[wv] = stamp;
          this.dist[wv] = nextDist;
          this.queue[tail++] = wv;
        }
      }

      // East
      if (x !== w - 1) {
        const ev = node + 1;
        if (this.visitedStamp[ev] !== stamp && map.isWater(ev)) {
          this.visitedStamp[ev] = stamp;
          this.dist[ev] = nextDist;
          this.queue[tail++] = ev;
        }
      }
    }

    return null;
  }
}

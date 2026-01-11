import { describe, expect, it } from "vitest";
import {
  BresenhamPathSmoother,
  BresenhamSmoothingTransformer,
} from "../../../src/core/pathfinding/smoothing/BresenhamPathSmoother";
import { PathFinder } from "../../../src/core/pathfinding/types";
import { createGameMap, createIslandMap, L, ref, W } from "./fixtures";

/**
 * 10x10 map: larger map for integration tests
 */
// prettier-ignore
const largeIslandMapData = {
  width: 10, height: 10, grid: [
    W, W, W, W, W, W, W, W, W, W,
    W, W, W, W, W, W, W, W, W, W,
    W, W, L, L, L, L, L, W, W, W,
    W, W, L, L, L, L, L, W, W, W,
    W, W, L, L, L, L, L, W, W, W,
    W, W, L, L, L, L, L, W, W, W,
    W, W, L, L, L, L, L, W, W, W,
    W, W, W, W, W, W, W, W, W, W,
    W, W, W, W, W, W, W, W, W, W,
    W, W, W, W, W, W, W, W, W, W,
  ],
};

describe("BresenhamPathSmoother", () => {
  describe("smooth", () => {
    it("returns empty path unchanged", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const smoother = new BresenhamPathSmoother(map, () => true);

      expect(smoother.smooth([])).toEqual([]);
    });

    it("returns single-element path unchanged", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const smoother = new BresenhamPathSmoother(map, () => true);

      const path = [ref(map, 0, 0)];
      expect(smoother.smooth(path)).toEqual(path);
    });

    it("returns two-element path unchanged", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const smoother = new BresenhamPathSmoother(map, () => true);

      const path = [ref(map, 0, 0), ref(map, 1, 1)];
      expect(smoother.smooth(path)).toEqual(path);
    });

    it("smooths straight horizontal path using direct Bresenham line", () => {
      const mapData = largeIslandMapData;
      const map = createGameMap(mapData);
      // All water traversable
      const smoother = new BresenhamPathSmoother(map, (t) => map.isWater(t));

      // Straight horizontal path along top row (all water)
      const path = [
        ref(map, 0, 0),
        ref(map, 1, 0),
        ref(map, 2, 0),
        ref(map, 3, 0),
        ref(map, 4, 0),
      ];

      const smoothed = smoother.smooth(path);

      // Bresenham traces full line - preserves all points along horizontal
      expect(smoothed[0]).toBe(path[0]);
      expect(smoothed[smoothed.length - 1]).toBe(path[path.length - 1]);
      // All tiles in smoothed path should be traversable
      smoothed.forEach((t) => expect(map.isWater(t)).toBe(true));
      // For horizontal line, should produce exactly the same path
      expect(smoothed).toEqual(path);
    });

    it("keeps path when obstacles block direct line", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const smoother = new BresenhamPathSmoother(map, (t) => map.isWater(t));

      // Path around island - cannot be simplified due to land
      // (0,0) → (0,4) → (4,4) - L-shaped path
      const path = [
        ref(map, 0, 0),
        ref(map, 0, 1),
        ref(map, 0, 2),
        ref(map, 0, 3),
        ref(map, 0, 4),
        ref(map, 1, 4),
        ref(map, 2, 4),
        ref(map, 3, 4),
        ref(map, 4, 4),
      ];

      const smoothed = smoother.smooth(path);

      // Start and end preserved
      expect(smoothed[0]).toBe(path[0]);
      expect(smoothed[smoothed.length - 1]).toBe(path[path.length - 1]);
    });

    it("uses traversability function", () => {
      const mapData = largeIslandMapData;
      const map = createGameMap(mapData);

      // Only even X coordinates are traversable
      const smoother = new BresenhamPathSmoother(
        map,
        (t) => map.x(t) % 2 === 0,
      );

      const path = [
        ref(map, 0, 0),
        ref(map, 2, 0),
        ref(map, 4, 0),
        ref(map, 6, 0),
        ref(map, 8, 0),
      ];

      const smoothed = smoother.smooth(path);

      // All smoothed tiles should have even X
      smoothed.forEach((t) => {
        expect(map.x(t) % 2).toBe(0);
      });
    });

    it("smooths straight vertical path using direct Bresenham line", () => {
      const mapData = largeIslandMapData;
      const map = createGameMap(mapData);
      // All water traversable
      const smoother = new BresenhamPathSmoother(map, (t) => map.isWater(t));

      // Vertical path down first column (all water)
      const path = [
        ref(map, 0, 0),
        ref(map, 0, 1),
        ref(map, 0, 2),
        ref(map, 0, 3),
        ref(map, 0, 4),
      ];

      const smoothed = smoother.smooth(path);

      // Bresenham traces full line - preserves all points along vertical
      expect(smoothed[0]).toBe(path[0]);
      expect(smoothed[smoothed.length - 1]).toBe(path[path.length - 1]);
      // All tiles in smoothed path should be traversable
      smoothed.forEach((t) => expect(map.isWater(t)).toBe(true));
      // For vertical line, should produce exactly the same path
      expect(smoothed).toEqual(path);
    });

    it("handles diagonal collision: X blocked, Y succeeds", () => {
      const mapData = largeIslandMapData;
      const map = createGameMap(mapData);

      // Create traversability that blocks specific X moves but allows Y
      // Block (3, 0) and (3, 1) to force Y-first movement
      const blockedTiles = new Set([ref(map, 3, 0), ref(map, 3, 1)]);
      const smoother = new BresenhamPathSmoother(
        map,
        (t) => !blockedTiles.has(t),
      );

      // Path that would normally try to go through (3, 0)
      const path = [
        ref(map, 0, 0),
        ref(map, 1, 1),
        ref(map, 2, 2),
        ref(map, 4, 2),
        ref(map, 5, 2),
      ];

      const smoothed = smoother.smooth(path);

      // Should preserve start and end
      expect(smoothed[0]).toBe(path[0]);
      expect(smoothed[smoothed.length - 1]).toBe(path[path.length - 1]);

      // Smoothed path should not contain blocked tiles
      smoothed.forEach((t) => {
        expect(blockedTiles.has(t)).toBe(false);
      });
    });

    it("tracePath returns null when direct line blocked (graceful fallback)", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);

      // Use water traversability - land blocks direct line
      const smoother = new BresenhamPathSmoother(map, (t) => map.isWater(t));

      // Path that goes around island: (0,0) -> (0,4) -> (4,4)
      // Direct line from (0,0) to (4,4) crosses land, so trace fails
      const path = [
        ref(map, 0, 0),
        ref(map, 0, 2),
        ref(map, 0, 4),
        ref(map, 2, 4),
        ref(map, 4, 4),
      ];

      const smoothed = smoother.smooth(path);

      // Should still produce valid path (may not be simplified much due to blocking)
      expect(smoothed[0]).toBe(path[0]);
      expect(smoothed[smoothed.length - 1]).toBe(path[path.length - 1]);
      // Path should be valid (all tiles traversable)
      smoothed.forEach((t) => {
        expect(map.isWater(t)).toBe(true);
      });
    });
  });
});

describe("BresenhamSmoothingTransformer", () => {
  function createMockPathFinder(): PathFinder<number> & {
    returnPath: number[] | null | undefined;
  } {
    const mock = {
      returnPath: undefined as number[] | null | undefined,
      findPath(from: number | number[], to: number): number[] | null {
        if (mock.returnPath !== undefined) return mock.returnPath;
        const start = Array.isArray(from) ? from[0] : from;
        return [start, to];
      },
    };
    return mock;
  }

  describe("findPath", () => {
    it("returns null when inner returns null", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();
      inner.returnPath = null;

      const transformer = new BresenhamSmoothingTransformer(inner, map);
      const result = transformer.findPath(ref(map, 0, 0), ref(map, 4, 4));

      expect(result).toBeNull();
    });

    it("smooths path from inner pathfinder", () => {
      const mapData = largeIslandMapData;
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();

      // Straight water path
      inner.returnPath = [
        ref(map, 0, 0),
        ref(map, 1, 0),
        ref(map, 2, 0),
        ref(map, 3, 0),
        ref(map, 4, 0),
      ];

      const transformer = new BresenhamSmoothingTransformer(inner, map);
      const result = transformer.findPath(ref(map, 0, 0), ref(map, 4, 0));

      expect(result).not.toBeNull();
      expect(result![0]).toBe(ref(map, 0, 0));
      expect(result![result!.length - 1]).toBe(ref(map, 4, 0));
    });

    it("defaults to water traversability", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();

      // Path that tries to cross land should not smooth through it
      const transformer = new BresenhamSmoothingTransformer(inner, map);

      // Set path that goes around island
      inner.returnPath = [ref(map, 0, 0), ref(map, 0, 4), ref(map, 4, 4)];

      const result = transformer.findPath(ref(map, 0, 0), ref(map, 4, 4));

      expect(result).not.toBeNull();
    });

    it("allows custom traversability function", () => {
      const mapData = largeIslandMapData;
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();

      // Custom: all tiles traversable
      const transformer = new BresenhamSmoothingTransformer(
        inner,
        map,
        () => true,
      );

      inner.returnPath = [ref(map, 0, 0), ref(map, 5, 5), ref(map, 9, 9)];

      const result = transformer.findPath(ref(map, 0, 0), ref(map, 9, 9));

      expect(result).not.toBeNull();
    });
  });

  describe("path shapes", () => {
    it("smooths diagonal path on open water", () => {
      const mapData = largeIslandMapData;
      const map = createGameMap(mapData);
      // All tiles traversable for this test
      const smoother = new BresenhamPathSmoother(map, () => true);

      // Diagonal path (0,0) → (9,9) with intermediate points
      const path = [
        ref(map, 0, 0),
        ref(map, 1, 1),
        ref(map, 2, 2),
        ref(map, 3, 3),
        ref(map, 4, 4),
        ref(map, 5, 5),
        ref(map, 6, 6),
        ref(map, 7, 7),
        ref(map, 8, 8),
        ref(map, 9, 9),
      ];

      const smoothed = smoother.smooth(path);

      // Smoother preserves start and end
      expect(smoothed[0]).toBe(path[0]);
      expect(smoothed[smoothed.length - 1]).toBe(path[path.length - 1]);
      // Path is fully traversable, so should be able to smooth
      // Note: Bresenham may interpolate between waypoints, so length may vary
    });

    it("smooths staircase pattern using Bresenham diagonal", () => {
      const mapData = largeIslandMapData;
      const map = createGameMap(mapData);
      const smoother = new BresenhamPathSmoother(map, () => true);

      // Staircase: (0,0) → (1,0) → (1,1) → (2,1) → (2,2)
      // Bresenham trace from (0,0) to (2,2) produces same 5 tiles
      // (moves X then Y at each diagonal step)
      const path = [
        ref(map, 0, 0),
        ref(map, 1, 0),
        ref(map, 1, 1),
        ref(map, 2, 1),
        ref(map, 2, 2),
      ];

      const smoothed = smoother.smooth(path);

      // Bresenham trace for diagonal (0,0)→(2,2) produces:
      // (0,0) → (1,0) → (1,1) → (2,1) → (2,2)
      // Same tiles as input staircase
      expect(smoothed).toEqual([
        ref(map, 0, 0),
        ref(map, 1, 0),
        ref(map, 1, 1),
        ref(map, 2, 1),
        ref(map, 2, 2),
      ]);
    });
  });

  describe("determinism", () => {
    it("same path produces same smoothed result", () => {
      const mapData = largeIslandMapData;
      const map = createGameMap(mapData);
      const smoother1 = new BresenhamPathSmoother(map, () => true);
      const smoother2 = new BresenhamPathSmoother(map, () => true);

      const path = [
        ref(map, 0, 0),
        ref(map, 2, 1),
        ref(map, 4, 2),
        ref(map, 6, 3),
        ref(map, 8, 4),
      ];

      const smoothed1 = smoother1.smooth(path);
      const smoothed2 = smoother2.smooth(path);

      expect(smoothed1).toEqual(smoothed2);
    });
  });
});

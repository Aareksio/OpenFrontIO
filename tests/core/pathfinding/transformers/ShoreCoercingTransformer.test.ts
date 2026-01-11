import { describe, expect, it } from "vitest";
import { ShoreCoercingTransformer } from "../../../../src/core/pathfinding/transformers/ShoreCoercingTransformer";
import { PathFinder } from "../../../../src/core/pathfinding/types";
import { createGameMap, createIslandMap, L, ref, W } from "../_fixtures";

describe("ShoreCoercingTransformer", () => {
  // Mock PathFinder that records calls and returns configurable path
  function createMockPathFinder(): PathFinder<number> & {
    calls: Array<{ from: number | number[]; to: number }>;
    returnPath: number[] | null | undefined;
  } {
    const mock = {
      calls: [] as Array<{ from: number | number[]; to: number }>,
      returnPath: undefined as number[] | null | undefined,
      findPath(from: number | number[], to: number): number[] | null {
        mock.calls.push({ from, to });
        // If returnPath explicitly set (including null), use it
        if (mock.returnPath !== undefined) return mock.returnPath;
        // Default: return straight path from first source to dest
        const start = Array.isArray(from) ? from[0] : from;
        return [start, to];
      },
    };
    return mock;
  }

  describe("findPath", () => {
    it("passes water tiles unchanged", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();
      const transformer = new ShoreCoercingTransformer(inner, map);

      const water1 = ref(map, 0, 0);
      const water2 = ref(map, 4, 0);
      inner.returnPath = [water1, water2];

      const result = transformer.findPath(water1, water2);

      expect(result).toEqual([water1, water2]);
      expect(inner.calls).toHaveLength(1);
      expect(inner.calls[0].from).toBe(water1);
      expect(inner.calls[0].to).toBe(water2);
    });

    it("coerces shore start to water and prepends original", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();
      const transformer = new ShoreCoercingTransformer(inner, map);

      const shore = ref(map, 1, 1); // shore tile
      const water = ref(map, 4, 4); // water tile

      // Shore (1,1) has water neighbors at (1,0) and (0,1); first is (1,0)
      const shoreWaterNeighbor = ref(map, 1, 0);
      inner.returnPath = [shoreWaterNeighbor, water];

      const result = transformer.findPath(shore, water);

      expect(result).not.toBeNull();
      expect(result![0]).toBe(shore);
      expect(result![1]).toBe(shoreWaterNeighbor);
    });

    it("coerces shore destination to water and appends original", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();
      const transformer = new ShoreCoercingTransformer(inner, map);

      const water = ref(map, 0, 0); // water tile
      const shore = ref(map, 1, 1); // shore tile

      // Shore (1,1) → first water neighbor is (1,0)
      const shoreWaterNeighbor = ref(map, 1, 0);
      inner.returnPath = [water, shoreWaterNeighbor];

      const result = transformer.findPath(water, shore);

      expect(result).not.toBeNull();
      expect(result![result!.length - 1]).toBe(shore);
    });

    it("coerces both shore start and destination", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();
      const transformer = new ShoreCoercingTransformer(inner, map);

      const shore1 = ref(map, 1, 1); // shore tile
      const shore2 = ref(map, 3, 3); // another shore tile

      // Shore (1,1) → first water is (1,0); shore (3,3) → first water is (3,4)
      const water1 = ref(map, 1, 0);
      const water2 = ref(map, 3, 4);
      inner.returnPath = [water1, water2];

      const result = transformer.findPath(shore1, shore2);

      expect(result).not.toBeNull();
      expect(result![0]).toBe(shore1);
      expect(result![result!.length - 1]).toBe(shore2);
    });

    it("returns null when source has no water neighbor", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();
      const transformer = new ShoreCoercingTransformer(inner, map);

      // Center land tile (2,2) has no water neighbors
      const land = ref(map, 2, 2);
      const water = ref(map, 0, 0);

      const result = transformer.findPath(land, water);

      expect(result).toBeNull();
      expect(inner.calls).toHaveLength(0);
    });

    it("returns null when destination has no water neighbor", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();
      const transformer = new ShoreCoercingTransformer(inner, map);

      const water = ref(map, 0, 0);
      const land = ref(map, 2, 2); // center land, no water neighbor

      const result = transformer.findPath(water, land);

      expect(result).toBeNull();
      expect(inner.calls).toHaveLength(0);
    });

    it("returns null when inner pathfinder returns null", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();
      inner.returnPath = null;
      const transformer = new ShoreCoercingTransformer(inner, map);

      const result = transformer.findPath(ref(map, 0, 0), ref(map, 4, 4));

      expect(result).toBeNull();
    });

    it("returns null when inner pathfinder returns empty path", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();
      inner.returnPath = [];
      const transformer = new ShoreCoercingTransformer(inner, map);

      const result = transformer.findPath(ref(map, 0, 0), ref(map, 4, 4));

      expect(result).toBeNull();
    });

    it("handles multiple sources, filters invalid ones", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();
      const transformer = new ShoreCoercingTransformer(inner, map);

      const water1 = ref(map, 0, 0);
      const shore = ref(map, 1, 1);
      const land = ref(map, 2, 2); // no water neighbor - should be filtered
      const waterDest = ref(map, 4, 4);

      inner.returnPath = [water1, waterDest];

      const result = transformer.findPath([water1, shore, land], waterDest);

      expect(result).not.toBeNull();
      // Inner should receive only valid sources
      expect(inner.calls).toHaveLength(1);
      const fromArg = inner.calls[0].from;
      expect(Array.isArray(fromArg)).toBe(true);
      // Should have 2 valid sources (water1 and coerced shore)
      expect((fromArg as number[]).length).toBe(2);
    });

    it("returns null when all sources are invalid", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();
      const transformer = new ShoreCoercingTransformer(inner, map);

      const land = ref(map, 2, 2); // no water neighbor

      const result = transformer.findPath([land], ref(map, 0, 0));

      expect(result).toBeNull();
      expect(inner.calls).toHaveLength(0);
    });
  });

  describe("determinism", () => {
    it("shore with multiple water neighbors selects consistently", () => {
      // 5x5 map: channel between land masses
      // L L W W W   (y=0)
      // L S W W W   (y=1)
      // S S W S S   (y=2)  ← (1,2) has 2 water neighbors: (1,3) and (2,2)
      // W W W S L   (y=3)
      // W W W L L   (y=4)
      // prettier-ignore
      const map = createGameMap({
        width: 5, height: 5, grid: [
          L, L, W, W, W,
          L, L, W, W, W,
          L, L, W, L, L,
          W, W, W, L, L,
          W, W, W, L, L,
        ],
      });
      const shoreWithMultipleWater = ref(map, 1, 2);
      // First water neighbor (deterministic order) is (1,3)
      const expectedWaterNeighbor = ref(map, 1, 3);

      const inner1 = createMockPathFinder();
      const inner2 = createMockPathFinder();
      const transformer1 = new ShoreCoercingTransformer(inner1, map);
      const transformer2 = new ShoreCoercingTransformer(inner2, map);

      const waterDest = ref(map, 2, 4); // water tile in channel
      inner1.returnPath = [waterDest];
      inner2.returnPath = [waterDest];

      transformer1.findPath(shoreWithMultipleWater, waterDest);
      transformer2.findPath(shoreWithMultipleWater, waterDest);

      // Both select the same water neighbor: (1,3)
      expect(inner1.calls[0].from).toBe(expectedWaterNeighbor);
      expect(inner2.calls[0].from).toBe(expectedWaterNeighbor);
    });

    it("corner shore with water neighbors works correctly", () => {
      const mapData = createIslandMap();
      const map = createGameMap(mapData);
      const inner = createMockPathFinder();
      const transformer = new ShoreCoercingTransformer(inner, map);

      // Shore (1,1) has 2 water neighbors: (1,0) and (0,1); first is (1,0)
      const cornerShore = ref(map, 1, 1);
      const waterNeighbor = ref(map, 1, 0);
      const waterDest = ref(map, 4, 4);

      inner.returnPath = [waterNeighbor, waterDest];

      const result = transformer.findPath(cornerShore, waterDest);

      // Exact expected path: [cornerShore, waterNeighbor, waterDest]
      expect(result).not.toBeNull();
      expect(result).toEqual([cornerShore, waterNeighbor, waterDest]);
    });
  });
});

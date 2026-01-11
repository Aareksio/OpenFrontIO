import { describe, expect, it } from "vitest";
import {
  GameMapWaterComponents,
  LAND_MARKER,
} from "../../../src/core/pathfinding/algorithms/hpa/WaterComponents";
import { createGameMap, createIslandMap, L, ref, W } from "./_fixtures";

/**
 * 7x5 map: two separate water bodies (components)
 * Left water (cols 0-1) and right water (cols 5-6) are disconnected.
 */
// prettier-ignore
const twoComponentsMapData = {
  width: 7, height: 5, grid: [
    W, W, L, L, L, W, W,
    W, W, L, L, L, W, W,
    W, W, L, L, L, W, W,
    W, W, L, L, L, W, W,
    W, W, L, L, L, W, W,
  ],
};

describe("GameMapWaterComponents", () => {
  describe("getComponentId", () => {
    it("returns 0 before initialization", () => {
      const map = createGameMap(createIslandMap());
      const wc = new GameMapWaterComponents(map);

      // Water tile at (0,0) - should return 0 (not initialized)
      const waterTile = ref(map, 0, 0);
      expect(wc.getComponentId(waterTile)).toBe(0);
    });

    it("returns 0 for land tiles after initialization", () => {
      const map = createGameMap(createIslandMap());
      const wc = new GameMapWaterComponents(map);
      wc.initialize();

      // Land tile at (2,2) - should return 0 (land marker)
      const landTile = ref(map, 2, 2);
      expect(map.isLand(landTile)).toBe(true);
      expect(wc.getComponentId(landTile)).toBe(LAND_MARKER);
    });

    it("returns same component ID for all water tiles in single connected area", () => {
      const map = createGameMap(createIslandMap());
      const wc = new GameMapWaterComponents(map);
      wc.initialize();

      // IslandMap: all water tiles are connected (one component)
      // Water tiles: (0,0), (1,0), (2,0), (3,0), (4,0), etc.
      const water1 = ref(map, 0, 0);
      const water2 = ref(map, 4, 0);
      const water3 = ref(map, 0, 4);
      const water4 = ref(map, 4, 4);

      expect(map.isWater(water1)).toBe(true);
      expect(map.isWater(water2)).toBe(true);
      expect(map.isWater(water3)).toBe(true);
      expect(map.isWater(water4)).toBe(true);

      const id1 = wc.getComponentId(water1);
      const id2 = wc.getComponentId(water2);
      const id3 = wc.getComponentId(water3);
      const id4 = wc.getComponentId(water4);

      // All should have same component ID (1, since first component)
      expect(id1).toBe(1);
      expect(id2).toBe(id1);
      expect(id3).toBe(id1);
      expect(id4).toBe(id1);
    });

    it("returns different component IDs for disconnected water areas", () => {
      const map = createGameMap(twoComponentsMapData);
      const wc = new GameMapWaterComponents(map);
      wc.initialize();

      // TwoComponentsMap (7x5):
      // W W L L L W W
      // Left water: cols 0-1
      // Right water: cols 5-6

      const leftWater1 = ref(map, 0, 0);
      const leftWater2 = ref(map, 1, 2);
      const rightWater1 = ref(map, 5, 0);
      const rightWater2 = ref(map, 6, 4);

      expect(map.isWater(leftWater1)).toBe(true);
      expect(map.isWater(leftWater2)).toBe(true);
      expect(map.isWater(rightWater1)).toBe(true);
      expect(map.isWater(rightWater2)).toBe(true);

      const leftId1 = wc.getComponentId(leftWater1);
      const leftId2 = wc.getComponentId(leftWater2);
      const rightId1 = wc.getComponentId(rightWater1);
      const rightId2 = wc.getComponentId(rightWater2);

      // Left water tiles should have same ID
      expect(leftId1).toBe(leftId2);

      // Right water tiles should have same ID
      expect(rightId1).toBe(rightId2);

      // Left and right should have DIFFERENT IDs
      expect(leftId1).not.toBe(rightId1);

      // Both should be non-zero (not land)
      expect(leftId1).toBeGreaterThan(0);
      expect(leftId1).not.toBe(LAND_MARKER);
      expect(rightId1).toBeGreaterThan(0);
      expect(rightId1).not.toBe(LAND_MARKER);
    });

    it("returns LAND_MARKER for land tiles in TwoComponentsMap", () => {
      const map = createGameMap(twoComponentsMapData);
      const wc = new GameMapWaterComponents(map);
      wc.initialize();

      // Land at cols 2-4
      const landTile1 = ref(map, 2, 0);
      const landTile2 = ref(map, 3, 2);
      const landTile3 = ref(map, 4, 4);

      expect(map.isLand(landTile1)).toBe(true);
      expect(map.isLand(landTile2)).toBe(true);
      expect(map.isLand(landTile3)).toBe(true);

      expect(wc.getComponentId(landTile1)).toBe(LAND_MARKER);
      expect(wc.getComponentId(landTile2)).toBe(LAND_MARKER);
      expect(wc.getComponentId(landTile3)).toBe(LAND_MARKER);
    });
  });

  describe("determinism", () => {
    it("produces same component IDs on repeated initialization", () => {
      const map = createGameMap(twoComponentsMapData);
      const wc1 = new GameMapWaterComponents(map);
      const wc2 = new GameMapWaterComponents(map);

      wc1.initialize();
      wc2.initialize();

      // Check all tiles have same component ID
      for (let y = 0; y < 5; y++) {
        for (let x = 0; x < 7; x++) {
          const tile = ref(map, x, y);
          expect(wc1.getComponentId(tile)).toBe(wc2.getComponentId(tile));
        }
      }
    });
  });

  describe("direct terrain access optimization", () => {
    it("produces same results with accessTerrainDirectly=false", () => {
      const map = createGameMap(twoComponentsMapData);
      const wcDirect = new GameMapWaterComponents(map, true);
      const wcIndirect = new GameMapWaterComponents(map, false);

      wcDirect.initialize();
      wcIndirect.initialize();

      // Check all tiles have same component ID
      for (let y = 0; y < 5; y++) {
        for (let x = 0; x < 7; x++) {
          const tile = ref(map, x, y);
          expect(wcDirect.getComponentId(tile)).toBe(
            wcIndirect.getComponentId(tile),
          );
        }
      }
    });
  });
});

import { describe, expect, it } from "vitest";
import { GameMapImpl } from "../../../src/core/game/GameMap";
import { MiniMapTransformer } from "../../../src/core/pathfinding/transformers/MiniMapTransformer";
import { PathFinder } from "../../../src/core/pathfinding/types";

describe("MiniMapTransformer", () => {
  // Create test maps: main map is 10x10, minimap is 5x5 (2x downscale)
  function createTestMaps() {
    const W = 0x20; // Water
    const mainTerrain = new Uint8Array(100).fill(W); // 10x10 all water
    const miniTerrain = new Uint8Array(25).fill(W); // 5x5 all water

    const mainMap = new GameMapImpl(10, 10, mainTerrain, 0);
    const miniMap = new GameMapImpl(5, 5, miniTerrain, 0);

    return { mainMap, miniMap };
  }

  // Mock Game that provides map and miniMap
  function createMockGame(mainMap: GameMapImpl, miniMap: GameMapImpl) {
    return {
      map: () => mainMap,
      miniMap: () => miniMap,
    } as any; // Cast to any since we're only implementing needed methods
  }

  function createMockPathFinder(): PathFinder<number> & {
    calls: Array<{ from: number | number[]; to: number }>;
    returnPath: number[] | null | undefined;
  } {
    const mock = {
      calls: [] as Array<{ from: number | number[]; to: number }>,
      returnPath: undefined as number[] | null | undefined,
      findPath(from: number | number[], to: number): number[] | null {
        mock.calls.push({ from, to });
        if (mock.returnPath !== undefined) return mock.returnPath;
        const start = Array.isArray(from) ? from[0] : from;
        return [start, to];
      },
    };
    return mock;
  }

  describe("findPath", () => {
    it("converts coordinates to minimap scale", () => {
      const { mainMap, miniMap } = createTestMaps();
      const game = createMockGame(mainMap, miniMap);
      const inner = createMockPathFinder();
      const transformer = new MiniMapTransformer(inner, game);

      // Main map coords (4, 6) → minimap coords (2, 3)
      const from = mainMap.ref(4, 6);
      // Main map coords (8, 2) → minimap coords (4, 1)
      const to = mainMap.ref(8, 2);

      // Set minimap path
      const miniFrom = miniMap.ref(2, 3);
      const miniTo = miniMap.ref(4, 1);
      inner.returnPath = [miniFrom, miniTo];

      transformer.findPath(from, to);

      // Verify inner was called with minimap coords
      expect(inner.calls).toHaveLength(1);
      expect(inner.calls[0].from).toBe(miniFrom);
      expect(inner.calls[0].to).toBe(miniTo);
    });

    it("upscales minimap path back to full resolution", () => {
      const { mainMap, miniMap } = createTestMaps();
      const game = createMockGame(mainMap, miniMap);
      const inner = createMockPathFinder();
      const transformer = new MiniMapTransformer(inner, game);

      const from = mainMap.ref(0, 0);
      const to = mainMap.ref(8, 0);

      // Minimap path: (0,0) → (4,0) - straight horizontal
      inner.returnPath = [
        miniMap.ref(0, 0),
        miniMap.ref(1, 0),
        miniMap.ref(2, 0),
        miniMap.ref(3, 0),
        miniMap.ref(4, 0),
      ];

      const result = transformer.findPath(from, to);

      expect(result).not.toBeNull();
      // Path should be in full-resolution coords
      expect(result![0]).toBe(from);
      expect(result![result!.length - 1]).toBe(to);
    });

    it("returns null when inner returns null", () => {
      const { mainMap, miniMap } = createTestMaps();
      const game = createMockGame(mainMap, miniMap);
      const inner = createMockPathFinder();
      inner.returnPath = null;
      const transformer = new MiniMapTransformer(inner, game);

      const result = transformer.findPath(mainMap.ref(0, 0), mainMap.ref(8, 8));

      expect(result).toBeNull();
    });

    it("returns null when inner returns empty path", () => {
      const { mainMap, miniMap } = createTestMaps();
      const game = createMockGame(mainMap, miniMap);
      const inner = createMockPathFinder();
      inner.returnPath = [];
      const transformer = new MiniMapTransformer(inner, game);

      const result = transformer.findPath(mainMap.ref(0, 0), mainMap.ref(8, 8));

      expect(result).toBeNull();
    });

    it("handles multiple sources", () => {
      const { mainMap, miniMap } = createTestMaps();
      const game = createMockGame(mainMap, miniMap);
      const inner = createMockPathFinder();
      const transformer = new MiniMapTransformer(inner, game);

      const from1 = mainMap.ref(0, 0);
      const from2 = mainMap.ref(2, 0);
      const to = mainMap.ref(8, 0);

      inner.returnPath = [miniMap.ref(0, 0), miniMap.ref(4, 0)];

      const result = transformer.findPath([from1, from2], to);

      // Inner should receive array of minimap coords
      expect(inner.calls).toHaveLength(1);
      expect(Array.isArray(inner.calls[0].from)).toBe(true);

      expect(result).not.toBeNull();
    });

    it("fixes path extremes to match original from/to", () => {
      const { mainMap, miniMap } = createTestMaps();
      const game = createMockGame(mainMap, miniMap);
      const inner = createMockPathFinder();
      const transformer = new MiniMapTransformer(inner, game);

      // From odd coords (won't exactly map to minimap)
      const from = mainMap.ref(1, 1);
      const to = mainMap.ref(9, 9);

      inner.returnPath = [miniMap.ref(0, 0), miniMap.ref(4, 4)];

      const result = transformer.findPath(from, to);

      expect(result).not.toBeNull();
      // Should include original from/to even if not on scaled grid
      expect(result![0]).toBe(from);
      expect(result![result!.length - 1]).toBe(to);
    });
  });

  describe("coordinate mapping", () => {
    it("maps main coords (0,0) to mini coords (0,0)", () => {
      const { mainMap, miniMap } = createTestMaps();
      const game = createMockGame(mainMap, miniMap);
      const inner = createMockPathFinder();
      const transformer = new MiniMapTransformer(inner, game);

      inner.returnPath = [miniMap.ref(0, 0)];

      transformer.findPath(mainMap.ref(0, 0), mainMap.ref(0, 0));

      expect(inner.calls[0].from).toBe(miniMap.ref(0, 0));
      expect(inner.calls[0].to).toBe(miniMap.ref(0, 0));
    });

    it("maps main coords (1,1) to mini coords (0,0) (floor division)", () => {
      const { mainMap, miniMap } = createTestMaps();
      const game = createMockGame(mainMap, miniMap);
      const inner = createMockPathFinder();
      const transformer = new MiniMapTransformer(inner, game);

      inner.returnPath = [miniMap.ref(0, 0)];

      transformer.findPath(mainMap.ref(1, 1), mainMap.ref(1, 1));

      expect(inner.calls[0].from).toBe(miniMap.ref(0, 0));
    });

    it("maps main coords (2,2) to mini coords (1,1)", () => {
      const { mainMap, miniMap } = createTestMaps();
      const game = createMockGame(mainMap, miniMap);
      const inner = createMockPathFinder();
      const transformer = new MiniMapTransformer(inner, game);

      inner.returnPath = [miniMap.ref(1, 1)];

      transformer.findPath(mainMap.ref(2, 2), mainMap.ref(2, 2));

      // 2/2 = 1
      expect(inner.calls[0].from).toBe(miniMap.ref(1, 1));
    });
  });

  describe("boundary handling", () => {
    it("handles coords at max boundary", () => {
      const { mainMap, miniMap } = createTestMaps();
      const game = createMockGame(mainMap, miniMap);
      const inner = createMockPathFinder();
      const transformer = new MiniMapTransformer(inner, game);

      // Max coords: (9,9) on 10x10 main map → (4,4) on 5x5 minimap
      const from = mainMap.ref(9, 9);
      const to = mainMap.ref(0, 0);

      inner.returnPath = [miniMap.ref(4, 4), miniMap.ref(0, 0)];

      const result = transformer.findPath(from, to);

      expect(result).not.toBeNull();
      expect(result![0]).toBe(from);
      expect(result![result!.length - 1]).toBe(to);
    });

    it("upscaled path stays within map bounds", () => {
      const { mainMap, miniMap } = createTestMaps();
      const game = createMockGame(mainMap, miniMap);
      const inner = createMockPathFinder();
      const transformer = new MiniMapTransformer(inner, game);

      // Path along edge of minimap
      inner.returnPath = [
        miniMap.ref(0, 0),
        miniMap.ref(1, 0),
        miniMap.ref(2, 0),
        miniMap.ref(3, 0),
        miniMap.ref(4, 0),
      ];

      const from = mainMap.ref(0, 0);
      const to = mainMap.ref(9, 0);

      const result = transformer.findPath(from, to);

      expect(result).not.toBeNull();
      // Verify all tiles in result are valid (within bounds)
      for (const t of result!) {
        const x = mainMap.x(t);
        const y = mainMap.y(t);
        expect(x).toBeGreaterThanOrEqual(0);
        expect(x).toBeLessThan(10);
        expect(y).toBeGreaterThanOrEqual(0);
        expect(y).toBeLessThan(10);
      }
    });
  });

  describe("determinism", () => {
    it("same inputs produce identical paths", () => {
      const { mainMap, miniMap } = createTestMaps();
      const game = createMockGame(mainMap, miniMap);
      const inner1 = createMockPathFinder();
      const inner2 = createMockPathFinder();
      const transformer1 = new MiniMapTransformer(inner1, game);
      const transformer2 = new MiniMapTransformer(inner2, game);

      const miniPath = [
        miniMap.ref(0, 0),
        miniMap.ref(1, 1),
        miniMap.ref(2, 2),
        miniMap.ref(3, 3),
        miniMap.ref(4, 4),
      ];
      inner1.returnPath = miniPath;
      inner2.returnPath = miniPath;

      const from = mainMap.ref(0, 0);
      const to = mainMap.ref(9, 9);

      const result1 = transformer1.findPath(from, to);
      const result2 = transformer2.findPath(from, to);

      expect(result1).toEqual(result2);
    });
  });
});

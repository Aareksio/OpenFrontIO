import { beforeAll, describe, expect, it } from "vitest";
import { Game } from "../../../src/core/game/Game";
import { TileRef } from "../../../src/core/game/GameMap";
import { PathFinding } from "../../../src/core/pathfinding/PathFinder";
import { PathStatus } from "../../../src/core/pathfinding/types";
import { setup } from "../../util/Setup";

/**
 * Integration tests for the full pathfinding transformer chain.
 * These tests verify the complete PathFinding.Water() pipeline:
 * HPA* → ComponentCheck → BresenhamSmoothing → ShoreCoercing → MiniMap
 */
describe("PathFinding Integration", () => {
  let game: Game;

  beforeAll(async () => {
    game = await setup("ocean_and_land");
  });

  // Map coordinates (ocean_and_land 16x16):
  // Water: 122 tiles, first at (8,0), distant (8,0)↔(15,4) dist=11
  // Shore: 21 tiles, first at (7,0), distant (7,0)↔(7,10) dist=10
  // Land: 113 tiles, first at (0,0)

  describe("PathFinding.Water full chain", () => {
    it("produces valid water path with all transformers", () => {
      const pathFinder = PathFinding.Water(game);
      const map = game.map();

      // Distant water tiles: (8,0) → (15,4), distance = 11
      const from = map.ref(8, 0);
      const to = map.ref(15, 4);

      expect(map.isWater(from)).toBe(true);
      expect(map.isWater(to)).toBe(true);
      expect(map.manhattanDist(from, to)).toBe(11);

      const path = pathFinder.findPath(from, to);

      // Connected water tiles - path MUST exist
      expect(path).not.toBeNull();
      expect(path![0]).toBe(from);
      expect(path![path!.length - 1]).toBe(to);

      // All tiles in path should be water or shore
      for (const t of path!) {
        expect(map.isWater(t) || map.isShore(t)).toBe(true);
      }

      // Path should be connected (gap <= 2 due to minimap upscaling)
      for (let i = 1; i < path!.length; i++) {
        const dist = map.manhattanDist(path![i - 1], path![i]);
        expect(dist).toBeLessThanOrEqual(2);
      }
    });

    it("returns null for land tile without water neighbor", () => {
      const pathFinder = PathFinding.Water(game);
      const map = game.map();

      // Land tile at (0,0) has no water neighbors
      const landTile = map.ref(0, 0);
      // Water tile at (8,0)
      const waterTile = map.ref(8, 0);

      expect(map.isLand(landTile)).toBe(true);
      expect(map.isShore(landTile)).toBe(false);
      expect(map.isWater(waterTile)).toBe(true);

      // Should return null - shore coercing transformer rejects land without water neighbor
      const path = pathFinder.findPath(landTile, waterTile);
      expect(path).toBeNull();
    });

    it("stepping interface works with full chain", () => {
      const pathFinder = PathFinding.Water(game);
      const map = game.map();

      // Water tiles at distance 6: (8,0) → (14,0)
      const from = map.ref(8, 0);
      const to = map.ref(14, 0);

      expect(map.isWater(from)).toBe(true);
      expect(map.isWater(to)).toBe(true);
      expect(map.manhattanDist(from, to)).toBe(6);

      // Step through the path
      let current = from;
      let steps = 0;
      const maxSteps = 20;

      while (steps < maxSteps && current !== to) {
        const result = pathFinder.next(current, to);

        if (result.status === PathStatus.COMPLETE) {
          // COMPLETE means we're at or near destination
          current = (result as { node: TileRef }).node;
          break;
        }

        expect(result.status).toBe(PathStatus.NEXT);
        current = (result as { node: TileRef }).node;
        steps++;
      }

      // Should reach destination or be very close (within minimap gap)
      expect(map.manhattanDist(current, to)).toBeLessThanOrEqual(2);
      // Should terminate in reasonable steps
      expect(steps).toBeLessThan(maxSteps);
    });
  });
});

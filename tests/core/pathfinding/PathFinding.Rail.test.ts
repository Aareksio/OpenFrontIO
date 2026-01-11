import { beforeAll, describe, expect, it } from "vitest";
import { Game } from "../../../src/core/game/Game";
import { TileRef } from "../../../src/core/game/GameMap";
import { PathFinding } from "../../../src/core/pathfinding/PathFinder";
import {
  PathStatus,
  SteppingPathFinder,
} from "../../../src/core/pathfinding/types";
import { setup } from "../../util/Setup";

describe("PathFinding.Rail", () => {
  let game: Game;
  let pathFinder: SteppingPathFinder<TileRef>;

  beforeAll(async () => {
    game = await setup("ocean_and_land");
    pathFinder = PathFinding.Rail(game);
  });

  // Map coordinates (ocean_and_land 16x16):
  // Land: 113 tiles, first at (0,0), adjacent (0,0)↔(1,0)
  // All land tiles in cols 0-6 are connected

  describe("findPath", () => {
    it("finds path on land tiles", () => {
      const map = game.map();

      // Adjacent land tiles: (0,0) and (1,0)
      const from = map.ref(0, 0);
      const to = map.ref(1, 0);

      expect(map.isLand(from)).toBe(true);
      expect(map.isLand(to)).toBe(true);

      const path = pathFinder.findPath(from, to);

      expect(path).not.toBeNull();
      expect(path!.length).toBe(2);
      expect(path![0]).toBe(from);
      expect(path![1]).toBe(to);
    });
  });

  describe("options", () => {
    it("accepts waterPenalty option", () => {
      const finderWithPenalty = PathFinding.Rail(game, { waterPenalty: 10 });
      expect(finderWithPenalty).toBeDefined();
    });

    it("accepts directionChangePenalty option", () => {
      const finderWithPenalty = PathFinding.Rail(game, {
        directionChangePenalty: 5,
      });
      expect(finderWithPenalty).toBeDefined();
    });

    it("accepts both options", () => {
      const finderWithOptions = PathFinding.Rail(game, {
        waterPenalty: 10,
        directionChangePenalty: 5,
      });
      expect(finderWithOptions).toBeDefined();
    });

    it("directionChangePenalty affects path selection", () => {
      // With high direction change penalty, path should prefer straight lines
      const finderNoPenalty = PathFinding.Rail(game);
      const finderHighPenalty = PathFinding.Rail(game, {
        directionChangePenalty: 100,
      });

      const map = game.map();

      // Land tiles at distance 4: (0,0) → (4,0)
      const from = map.ref(0, 0);
      const to = map.ref(4, 0);

      expect(map.isLand(from)).toBe(true);
      expect(map.isLand(to)).toBe(true);
      expect(map.manhattanDist(from, to)).toBe(4);

      const pathNoPenalty = finderNoPenalty.findPath(from, to);
      const pathHighPenalty = finderHighPenalty.findPath(from, to);

      // Connected tiles: paths MUST exist
      expect(pathNoPenalty).not.toBeNull();
      expect(pathHighPenalty).not.toBeNull();

      // Helper to count direction changes in a path
      const countDirectionChanges = (path: TileRef[]): number => {
        if (path.length < 3) return 0;
        let changes = 0;
        let prevDx = map.x(path[1]) - map.x(path[0]);
        let prevDy = map.y(path[1]) - map.y(path[0]);
        for (let i = 2; i < path.length; i++) {
          const dx = map.x(path[i]) - map.x(path[i - 1]);
          const dy = map.y(path[i]) - map.y(path[i - 1]);
          if (dx !== prevDx || dy !== prevDy) {
            changes++;
          }
          prevDx = dx;
          prevDy = dy;
        }
        return changes;
      };

      expect(pathNoPenalty![0]).toBe(from);
      expect(pathNoPenalty![pathNoPenalty!.length - 1]).toBe(to);
      expect(pathHighPenalty![0]).toBe(from);
      expect(pathHighPenalty![pathHighPenalty!.length - 1]).toBe(to);

      const changesNoPenalty = countDirectionChanges(pathNoPenalty!);
      const changesHighPenalty = countDirectionChanges(pathHighPenalty!);
      expect(changesNoPenalty).toBe(0);
      expect(changesHighPenalty).toBe(0);
    });
  });

  describe("stepping interface", () => {
    it("invalidate does not throw", () => {
      expect(() => pathFinder.invalidate()).not.toThrow();
    });

    it("returns NEXT when path exists to distant land tile", () => {
      const map = game.map();

      // Land tiles at distance 2: (0,0) → (2,0)
      const from = map.ref(0, 0);
      const to = map.ref(2, 0);

      expect(map.isLand(from)).toBe(true);
      expect(map.isLand(to)).toBe(true);
      expect(map.manhattanDist(from, to)).toBe(2);

      const result = pathFinder.next(from, to);

      // Connected land tiles at dist > 0: must return NEXT
      expect(result.status).toBe(PathStatus.NEXT);
      const node = (result as { node: TileRef }).node;
      // Next node should be (1,0) - the intermediate tile
      expect(node).toBe(map.ref(1, 0));
    });

    it("returns COMPLETE when at destination", () => {
      const map = game.map();

      // Land tile at (0,0)
      const tile = map.ref(0, 0);
      expect(map.isLand(tile)).toBe(true);

      const result = pathFinder.next(tile, tile);

      expect(result.status).toBe(PathStatus.COMPLETE);
      expect((result as { node: TileRef }).node).toBe(tile);
    });
  });
});

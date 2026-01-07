import { describe, test, expect, vi } from "vitest";
import { PathFinder } from "../../../src/core/pathfinding/PathFinding";
import { PathFindResultType } from "../../../src/core/pathfinding/AStar";
import { TileRef } from "../../../src/core/game/GameMap";
import { mapFromString } from "./utils";

const DEFAULT_ITERATIONS = 10_000;
const DEFAULT_TRIES = 1;

function navigateTo(
  pathFinder: PathFinder,
  from: TileRef,
  to: TileRef,
  maxIter = 100
): { reached: boolean; notFound: boolean; pos: TileRef; steps: number; path: TileRef[] } {
  const status = { 
    reached: false, 
    notFound: false, 
    pos: from, 
    steps: 0, 
    path: [] as TileRef[] 
  };

  for (let i = 0; i < maxIter; i++) {
    const result = pathFinder.nextTile(status.pos, to);

    if (result.type === PathFindResultType.NextTile) {
      status.path.push(result.node);
      status.pos = result.node;
      status.steps++;
    } else if (result.type === PathFindResultType.Completed) {
      status.path.push(result.node);
      status.reached = true;
      return status;
    } else if (result.type === PathFindResultType.PathNotFound) {
      status.notFound = true;
      return status;
    }
  }

  return status;
}

describe("PathFinder state machine tests", () => {
  describe("nextTile() basic behavior", () => {
    test("returns NextTile on first call", async () => {
      const game = await mapFromString(["WWWW"]);
      const pathFinder = PathFinder.Mini(game, DEFAULT_ITERATIONS, true, DEFAULT_TRIES);
      const src = game.map().ref(0, 0);
      const dst = game.map().ref(3, 0);

      const result = pathFinder.nextTile(src, dst);
      expect(result.type).toBe(PathFindResultType.NextTile);
    });

    test("returns Completed when destination reached", async () => {
      const game = await mapFromString(["WWWW"]);
      const pathFinder = PathFinder.Mini(game, DEFAULT_ITERATIONS, true, DEFAULT_TRIES);
      const src = game.map().ref(0, 0);
      const dst = game.map().ref(3, 0);

      const result = navigateTo(pathFinder, src, dst);
      expect(result.reached).toBe(true);
    });

    test("returns Completed immediately when already at destination", async () => {
      const game = await mapFromString(["WWWW"]);
      const pathFinder = PathFinder.Mini(game, DEFAULT_ITERATIONS, true, DEFAULT_TRIES);
      const src = game.map().ref(0, 0);

      const result = pathFinder.nextTile(src, src, 1);
      expect(result.type).toBe(PathFindResultType.Completed);

      if (result.type === PathFindResultType.Completed) {
        expect(result.node).toBe(src);
      }
    });

    test("subsequent calls continue path", async () => {
      const game = await mapFromString(["WWWW"]);
      const pathFinder = PathFinder.Mini(game, DEFAULT_ITERATIONS, true, DEFAULT_TRIES);
      const src = game.map().ref(0, 0);
      const dst = game.map().ref(3, 0);
      
      const result = navigateTo(pathFinder, src, dst);
      expect(result.reached).toBe(true);
      expect(result.pos).toBe(dst);
      expect(result.steps).toBeGreaterThan(0);
    });
  });

  describe("Destination changes", () => {
    test("reaches new destination when dest changes", async () => {
      const game = await mapFromString([
        "WWWWWWWW", // 8 wide
      ]);

      const pathFinder = PathFinder.Mini(game, DEFAULT_ITERATIONS, true, DEFAULT_TRIES);
      const src = game.map().ref(0, 0);
      const dst1 = game.map().ref(4, 0);
      const dst2 = game.map().ref(7, 0);

      const first = navigateTo(pathFinder, src, dst1);
      expect(first.reached).toBe(true);

      const second = navigateTo(pathFinder, first.pos, dst2);
      expect(second.reached).toBe(true);
      expect(second.pos).toBe(dst2);
    });

    test("recomputes path when destination significantly changes", async () => {
      const game = await mapFromString([
        "WWWWWWWWWWWWWWWWWWWW", // 20 wide
      ]);

      const pathFinder = PathFinder.Mini(game, DEFAULT_ITERATIONS, true, DEFAULT_TRIES);
      const src = game.map().ref(0, 0);
      const dst1 = game.map().ref(10, 0);
      const dst2 = game.map().ref(19, 0);

      // Start pathing to dst1
      const result1 = pathFinder.nextTile(src, dst1);
      expect(result1.type).toBe(PathFindResultType.NextTile);

      // Change to far destination (should trigger recompute)
      const result2 = pathFinder.nextTile(src, dst2);
      expect(result2.type).toBe(PathFindResultType.NextTile);

      // Eventually should reach dst2
      const nav = navigateTo(pathFinder, src, dst2);
      expect(nav.reached).toBe(true);
    });
  });

  describe("Error handling", () => {
    test("returns PathNotFound for null source", async () => {
      const game = await mapFromString(["WWWW"]);
      const pathFinder = PathFinder.Mini(game, DEFAULT_ITERATIONS, true, DEFAULT_TRIES);
      const dst = game.map().ref(0, 0);

      const consoleSpy = vi.spyOn(console, 'error').mockImplementation(() => {});

      const result = pathFinder.nextTile(null, dst);
      expect(result.type).toBe(PathFindResultType.PathNotFound);

      consoleSpy.mockRestore();
    });

    test("returns PathNotFound for null destination", async () => {
      const game = await mapFromString(["WWWW"]);
      const pathFinder = PathFinder.Mini(game, DEFAULT_ITERATIONS, true, DEFAULT_TRIES);
      const src = game.map().ref(0, 0);

      const consoleSpy = vi.spyOn(console, 'error').mockImplementation(() => {});

      const result = pathFinder.nextTile(src, null);
      expect(result.type).toBe(PathFindResultType.PathNotFound);

      consoleSpy.mockRestore();
    });
  });

  describe("Bugs", () => {
    test.skip("returns PathNotFound when no path exists", async () => {
      // Expected to fail until we implement pathing that
      // is aware of upscaling from miniMap to main map.

      const game = await mapFromString(["WLLW"]);
      const pathFinder = PathFinder.Mini(game, DEFAULT_ITERATIONS, true, DEFAULT_TRIES);
      const src = game.map().ref(0, 0);
      const dst = game.map().ref(3, 0);

      const result = navigateTo(pathFinder, src, dst);
      expect(result.notFound).toBe(true);
    });
  })
});

import { describe, expect, test, vi } from "vitest";
import { TileRef } from "../../../src/core/game/GameMap";
import {
  PathFinder,
  PathFinders,
  PathStatus,
} from "../../../src/core/pathfinding/PathFinder";
import { setup } from "../../util/Setup";
import { mapFromString } from "./utils";

function navigateTo(
  pathFinder: PathFinder,
  from: TileRef,
  to: TileRef,
  maxIter = 100,
): {
  reached: boolean;
  notFound: boolean;
  pos: TileRef;
  steps: number;
  path: TileRef[];
} {
  const status = {
    reached: false,
    notFound: false,
    pos: from,
    steps: 0,
    path: [] as TileRef[],
  };

  for (let i = 0; i < maxIter; i++) {
    const result = pathFinder.next(status.pos, to);

    if (result.status === PathStatus.NEXT) {
      status.path.push(result.node);
      status.pos = result.node;
      status.steps++;
    } else if (result.status === PathStatus.COMPLETE) {
      status.path.push(result.node);
      status.reached = true;
      return status;
    } else if (result.status === PathStatus.NOT_FOUND) {
      status.notFound = true;
      return status;
    }
  }

  return status;
}

describe("PathFinder state machine tests", () => {
  describe("next() basic behavior", () => {
    test("returns next on first call", async () => {
      const game = await mapFromString(["WWWW"]);
      const pathFinder = PathFinders.Water(game);
      const src = game.map().ref(0, 0);
      const dst = game.map().ref(3, 0);

      const result = pathFinder.next(src, dst);
      expect(result.status).toBe(PathStatus.NEXT);
    });

    test("returns complete when destination reached", async () => {
      const game = await mapFromString(["WWWW"]);
      const pathFinder = PathFinders.Water(game);
      const src = game.map().ref(0, 0);
      const dst = game.map().ref(3, 0);

      const result = navigateTo(pathFinder, src, dst);
      expect(result.reached).toBe(true);
    });

    test("returns complete immediately when already at destination", async () => {
      const game = await mapFromString(["WWWW"]);
      const pathFinder = PathFinders.Water(game);
      const src = game.map().ref(0, 0);

      const result = pathFinder.next(src, src, 1);
      expect(result.status).toBe(PathStatus.COMPLETE);

      if (result.status === PathStatus.COMPLETE) {
        expect(result.node).toBe(src);
      }
    });

    test("subsequent calls continue path", async () => {
      const game = await mapFromString(["WWWW"]);
      const pathFinder = PathFinders.Water(game);
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

      const pathFinder = PathFinders.Water(game);
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

      const pathFinder = PathFinders.Water(game);
      const src = game.map().ref(0, 0);
      const dst1 = game.map().ref(10, 0);
      const dst2 = game.map().ref(19, 0);

      // Start pathing to dst1
      const result1 = pathFinder.next(src, dst1);
      expect(result1.status).toBe(PathStatus.NEXT);

      // Change to far destination (should trigger recompute)
      const result2 = pathFinder.next(src, dst2);
      expect(result2.status).toBe(PathStatus.NEXT);

      // Eventually should reach dst2
      const nav = navigateTo(pathFinder, src, dst2);
      expect(nav.reached).toBe(true);
    });
  });

  describe("Error handling", () => {
    test("returns not-found for null source", async () => {
      const game = await mapFromString(["WWWW"]);
      const pathFinder = PathFinders.Water(game);
      const dst = game.map().ref(0, 0);

      const consoleSpy = vi
        .spyOn(console, "error")
        .mockImplementation(() => {});

      const result = pathFinder.next(null as unknown as TileRef, dst);
      expect(result.status).toBe(PathStatus.NOT_FOUND);

      consoleSpy.mockRestore();
    });

    test("returns not-found for null destination", async () => {
      const game = await mapFromString(["WWWW"]);
      const pathFinder = PathFinders.Water(game);
      const src = game.map().ref(0, 0);

      const consoleSpy = vi
        .spyOn(console, "error")
        .mockImplementation(() => {});

      const result = pathFinder.next(src, null as unknown as TileRef);
      expect(result.status).toBe(PathStatus.NOT_FOUND);

      consoleSpy.mockRestore();
    });
  });

  describe("Bugs", () => {
    test.fails("returns PathNotFound when no path exists", async () => {
      // Expected to fail until we implement pathing that
      // is aware of upscaling from miniMap to main map.

      const game = await mapFromString(["WLLW"]);
      const pathFinder = PathFinders.Water(game);
      const src = game.map().ref(0, 0);
      const dst = game.map().ref(3, 0);

      const result = navigateTo(pathFinder, src, dst);
      expect(result.notFound).toBe(true);
    });
  });
});

describe("PathFinder world map tests", () => {
  // Ocean shoreline coordinates:
  // Spain east coast: [926, 283], France south coast: [950, 257]
  // Poland north coast: [1033, 175], Miami: [488, 355], Rio: [680, 658]

  test("finds path Spain to France (Mediterranean)", async () => {
    const game = await setup("world");
    const pathFinder = PathFinders.Water(game);

    const src = game.ref(926, 283); // Spain east coast
    const dst = game.ref(950, 257); // France south coast

    const result = navigateTo(pathFinder, src, dst, 500);
    expect(result.reached).toBe(true);
    expect(result.steps).toBeGreaterThan(0);
  });

  test("finds path Miami to Rio (Atlantic)", async () => {
    const game = await setup("world");
    const pathFinder = PathFinders.Water(game);

    const src = game.ref(488, 355); // Miami
    const dst = game.ref(680, 658); // Rio

    const result = navigateTo(pathFinder, src, dst, 2000);
    expect(result.reached).toBe(true);
    expect(result.steps).toBeGreaterThan(100);
  });

  test("finds path France to Poland (around Europe)", async () => {
    const game = await setup("world");
    const pathFinder = PathFinders.Water(game);

    const src = game.ref(950, 257); // France south coast
    const dst = game.ref(1033, 175); // Poland north coast

    const result = navigateTo(pathFinder, src, dst, 2000);
    expect(result.reached).toBe(true);
    expect(result.steps).toBeGreaterThan(50);
  });

  test("finds path Miami to Spain (transatlantic)", async () => {
    const game = await setup("world");
    const pathFinder = PathFinders.Water(game);

    const src = game.ref(488, 355); // Miami
    const dst = game.ref(926, 283); // Spain east coast

    const result = navigateTo(pathFinder, src, dst, 3000);
    expect(result.reached).toBe(true);
    expect(result.steps).toBeGreaterThan(200);
  });

  test("finds path Rio to Poland (South Atlantic to Baltic)", async () => {
    const game = await setup("world");
    const pathFinder = PathFinders.Water(game);

    const src = game.ref(680, 658); // Rio
    const dst = game.ref(1033, 175); // Poland north coast

    const result = navigateTo(pathFinder, src, dst, 5000);
    expect(result.reached).toBe(true);
    expect(result.steps).toBeGreaterThan(300);
  });
});

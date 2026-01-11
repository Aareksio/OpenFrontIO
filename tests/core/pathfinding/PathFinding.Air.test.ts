import { beforeAll, describe, expect, it } from "vitest";
import { Game } from "../../../src/core/game/Game";
import { TileRef } from "../../../src/core/game/GameMap";
import { PathFinding } from "../../../src/core/pathfinding/PathFinder";
import {
  PathStatus,
  SteppingPathFinder,
} from "../../../src/core/pathfinding/types";
import { setup } from "../../util/Setup";

describe("PathFinding.Air", () => {
  let game: Game;

  function createPathFinder(): SteppingPathFinder<TileRef> {
    return PathFinding.Air(game);
  }

  beforeAll(async () => {
    game = await setup("ocean_and_land");
  });

  describe("findPath", () => {
    it("returns path between any two points (ignores terrain)", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      // Air pathfinder ignores terrain, so can go anywhere
      // (2,2) → (14,14): manhattan = 24, path length = 25
      const from = map.ref(2, 2);
      const to = map.ref(14, 14);

      const path = pathFinder.findPath(from, to);

      expect(path).not.toBeNull();
      expect(path!.length).toBe(25);
      expect(path![0]).toBe(from);
      expect(path![path!.length - 1]).toBe(to);
    });

    it("throws error for multiple start points", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      const from = [map.ref(2, 2), map.ref(4, 4)];
      const to = map.ref(14, 14);

      expect(() => pathFinder.findPath(from, to)).toThrow(
        "does not support multiple start points",
      );
    });

    it("returns single-tile path when from equals to", () => {
      const pathFinder = createPathFinder();
      const map = game.map();
      const tile = map.ref(8, 8);

      const path = pathFinder.findPath(tile, tile);

      expect(path).not.toBeNull();
      expect(path![0]).toBe(tile);
    });
  });

  describe("next", () => {
    it("returns COMPLETE when at destination", () => {
      const pathFinder = createPathFinder();
      const map = game.map();
      const tile = map.ref(8, 8);

      const result = pathFinder.next(tile, tile);

      expect(result.status).toBe(PathStatus.COMPLETE);
      expect((result as { node: TileRef }).node).toBe(tile);
    });

    it("returns NEXT with adjacent tile when not at destination", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      const from = map.ref(2, 2);
      const to = map.ref(14, 14);

      const result = pathFinder.next(from, to);

      expect(result.status).toBe(PathStatus.NEXT);
      const node = (result as { node: TileRef }).node;
      expect(map.manhattanDist(from, node)).toBe(1);
    });

    it("eventually reaches destination through repeated stepping", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      // (2,2) → (10,10): manhattan = 16, so exactly 16 steps
      const from = map.ref(2, 2);
      const to = map.ref(10, 10);

      let current = from;
      let steps = 0;

      while (current !== to) {
        const result = pathFinder.next(current, to);

        if (result.status === PathStatus.COMPLETE) {
          break;
        }

        current = (result as { node: TileRef }).node;
        steps++;
      }

      // Should take exactly 16 steps (manhattan distance)
      expect(steps).toBe(16);
    });

    it("ignores dist parameter", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      const from = map.ref(10, 10);
      const to = map.ref(13, 10);
      const actualDist = map.manhattanDist(from, to);
      expect(actualDist).toBe(3);

      // Air pathfinder does NOT respect dist parameter; always steps 1 tile
      const result = pathFinder.next(from, to, 10);

      expect(result.status).toBe(PathStatus.NEXT);
      const node = (result as { node: TileRef }).node;
      expect(map.manhattanDist(from, node)).toBe(1);
      expect(map.manhattanDist(node, to)).toBe(2);
    });
  });

  describe("path validity", () => {
    it("all consecutive tiles in path are adjacent (Manhattan distance 1)", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      // (2,2) → (14,14): manhattan = 24, path length = 25
      const from = map.ref(2, 2);
      const to = map.ref(14, 14);

      const path = pathFinder.findPath(from, to);

      expect(path).not.toBeNull();
      expect(path!.length).toBe(25);

      // Verify every consecutive pair is adjacent
      for (let i = 1; i < path!.length; i++) {
        const dist = map.manhattanDist(path![i - 1], path![i]);
        expect(dist).toBe(1);
      }
    });

    it("path ends at exact destination", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      const from = map.ref(5, 5);
      const to = map.ref(10, 12);

      const path = pathFinder.findPath(from, to);

      expect(path).not.toBeNull();
      expect(path![path!.length - 1]).toBe(to);
    });
  });

  describe("path shapes", () => {
    it("diagonal path has equal X and Y movement", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      // Equal X and Y offset: (0,0) → (10,10)
      const from = map.ref(0, 0);
      const to = map.ref(10, 10);

      const path = pathFinder.findPath(from, to);
      expect(path).not.toBeNull();

      let xMoves = 0;
      let yMoves = 0;
      for (let i = 1; i < path!.length; i++) {
        const dx = map.x(path![i]) - map.x(path![i - 1]);
        const dy = map.y(path![i]) - map.y(path![i - 1]);
        if (dx !== 0) xMoves++;
        if (dy !== 0) yMoves++;
      }

      expect(xMoves).toBe(10);
      expect(yMoves).toBe(10);
    });

    it("horizontal path has only X movement", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      // Pure horizontal: (0,5) → (15,5)
      const from = map.ref(0, 5);
      const to = map.ref(15, 5);

      const path = pathFinder.findPath(from, to);
      expect(path).not.toBeNull();

      let xMoves = 0;
      let yMoves = 0;
      for (let i = 1; i < path!.length; i++) {
        const dx = map.x(path![i]) - map.x(path![i - 1]);
        const dy = map.y(path![i]) - map.y(path![i - 1]);
        if (dx !== 0) xMoves++;
        if (dy !== 0) yMoves++;
      }

      expect(xMoves).toBe(15);
      expect(yMoves).toBe(0);
    });

    it("vertical path has only Y movement", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      // Pure vertical: (5,0) → (5,15)
      const from = map.ref(5, 0);
      const to = map.ref(5, 15);

      const path = pathFinder.findPath(from, to);
      expect(path).not.toBeNull();

      let xMoves = 0;
      let yMoves = 0;
      for (let i = 1; i < path!.length; i++) {
        const dx = map.x(path![i]) - map.x(path![i - 1]);
        const dy = map.y(path![i]) - map.y(path![i - 1]);
        if (dx !== 0) xMoves++;
        if (dy !== 0) yMoves++;
      }

      expect(xMoves).toBe(0);
      expect(yMoves).toBe(15);
    });

    it("adjacent tiles produce minimal path", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      const from = map.ref(5, 5);
      const to = map.ref(6, 5);

      const path = pathFinder.findPath(from, to);

      expect(path).not.toBeNull();
      expect(path!.length).toBe(2);
      expect(path![0]).toBe(from);
      expect(path![1]).toBe(to);
    });
  });
});

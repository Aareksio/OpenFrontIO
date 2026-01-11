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
      expect(path!.length).toBe(25); // manhattan + 1
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

    it("creates paths with optimal length (Bresenham)", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      const from = map.ref(0, 0);
      const to = map.ref(15, 15);

      const path = pathFinder.findPath(from, to);

      expect(path).not.toBeNull();

      const manhattanDist = map.manhattanDist(from, to);
      expect(path!.length).toBe(manhattanDist + 1);

      for (let i = 1; i < path!.length; i++) {
        const prevDist = map.manhattanDist(path![i - 1], to);
        const currDist = map.manhattanDist(path![i], to);
        expect(currDist).toBe(prevDist - 1);
      }
    });
  });

  describe("next (stepping)", () => {
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

      // Tiles at distance 3 (not adjacent)
      const from = map.ref(10, 10);
      const to = map.ref(13, 10);
      const actualDist = map.manhattanDist(from, to);
      expect(actualDist).toBe(3);

      // With dist=10 (larger than actual distance), Water/Rail would return COMPLETE
      // But Air ignores dist entirely and continues stepping toward target
      const result = pathFinder.next(from, to, 10);

      // Air pathfinder: always NEXT until at destination, ignores dist threshold
      // Unlike Water/Rail which would return COMPLETE when actualDist <= threshold
      expect(result.status).toBe(PathStatus.NEXT);
      const node = (result as { node: TileRef }).node;
      // Should have moved exactly 1 tile toward destination
      expect(map.manhattanDist(from, node)).toBe(1);
      expect(map.manhattanDist(node, to)).toBe(2);
    });

    it("always moves exactly one tile per step", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      const from = map.ref(2, 2);
      const to = map.ref(12, 12);

      const result = pathFinder.next(from, to);
      expect(result.status).toBe(PathStatus.NEXT);

      const nextTile = (result as { node: TileRef }).node;
      expect(map.manhattanDist(from, nextTile)).toBe(1);
    });
  });

  describe("invalidate", () => {
    it("does not throw", () => {
      const pathFinder = createPathFinder();
      expect(() => pathFinder.invalidate()).not.toThrow();
    });
  });

  describe("determinism", () => {
    it("produces same path for same seed", () => {
      const finder1 = createPathFinder();
      const finder2 = createPathFinder();

      const map = game.map();
      const from = map.ref(2, 2);
      const to = map.ref(14, 14);

      const path1 = finder1.findPath(from, to);
      const path2 = finder2.findPath(from, to);

      expect(path1).not.toBeNull();
      expect(path2).not.toBeNull();
      expect(path1!.length).toBe(path2!.length);
      expect(path1).toEqual(path2);
    });

    it("next() is deterministic with fresh finder", () => {
      const finder1 = createPathFinder();
      const finder2 = createPathFinder();
      const map = game.map();

      const from = map.ref(2, 2);
      const to = map.ref(14, 14);

      // Same sequence of steps should produce same results
      const result1 = finder1.next(from, to);
      const result2 = finder2.next(from, to);

      expect(result1.status).toBe(result2.status);
      // Both NEXT and COMPLETE have nodes - compare them
      if (
        result1.status === PathStatus.NEXT ||
        result1.status === PathStatus.COMPLETE
      ) {
        expect((result1 as { node: TileRef }).node).toBe(
          (result2 as { node: TileRef }).node,
        );
      }
    });
  });

  describe("multi-step movement", () => {
    it("consecutive next calls advance n tiles", () => {
      const pathFinder = createPathFinder();
      const map = game.map();

      // (0,0) → (10,10): manhattan = 20, so 3 steps won't complete
      const from = map.ref(0, 0);
      const to = map.ref(10, 10);

      // Simulate 3 steps (like ShellExecution)
      let current = from;
      const positions: TileRef[] = [from];

      for (let i = 0; i < 3; i++) {
        const result = pathFinder.next(current, to);
        if (result.status === PathStatus.COMPLETE) break;
        current = (result as { node: TileRef }).node;
        positions.push(current);
      }

      // Should have exactly 4 positions (start + 3 steps)
      expect(positions.length).toBe(4);
      // Each step should be adjacent to previous
      for (let i = 1; i < positions.length; i++) {
        expect(map.manhattanDist(positions[i - 1], positions[i])).toBe(1);
      }
      // After 3 steps, should be 3 tiles closer
      expect(map.manhattanDist(positions[3], to)).toBe(17);
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

      // Count X and Y movements
      let xMoves = 0;
      let yMoves = 0;
      for (let i = 1; i < path!.length; i++) {
        const dx = map.x(path![i]) - map.x(path![i - 1]);
        const dy = map.y(path![i]) - map.y(path![i - 1]);
        if (dx !== 0) xMoves++;
        if (dy !== 0) yMoves++;
      }

      // Bresenham: exactly 10 X moves and 10 Y moves for (0,0)→(10,10)
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

      // Count X and Y movements
      let xMoves = 0;
      let yMoves = 0;
      for (let i = 1; i < path!.length; i++) {
        const dx = map.x(path![i]) - map.x(path![i - 1]);
        const dy = map.y(path![i]) - map.y(path![i - 1]);
        if (dx !== 0) xMoves++;
        if (dy !== 0) yMoves++;
      }

      // Pure horizontal: 15 X moves, 0 Y moves
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

      // Count X and Y movements
      let xMoves = 0;
      let yMoves = 0;
      for (let i = 1; i < path!.length; i++) {
        const dx = map.x(path![i]) - map.x(path![i - 1]);
        const dy = map.y(path![i]) - map.y(path![i - 1]);
        if (dx !== 0) xMoves++;
        if (dy !== 0) yMoves++;
      }

      // Pure vertical: 0 X moves, 15 Y moves
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
      // Path should be just [from, to]
      expect(path!.length).toBe(2);
      expect(path![0]).toBe(from);
      expect(path![1]).toBe(to);
    });
  });
});

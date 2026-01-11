import { describe, expect, it } from "vitest";
import { GameMapImpl } from "../../../src/core/game/GameMap";
import { UniversalPathFinding } from "../../../src/core/pathfinding/PathFinder";
import { PathStatus } from "../../../src/core/pathfinding/types";

describe("UniversalPathFinding.Parabola", () => {
  // Create a larger map for parabola tests (need space for arcs)
  function createLargeMap() {
    const W = 0x20; // Water
    const terrain = new Uint8Array(10000).fill(W); // 100x100 all water
    return new GameMapImpl(100, 100, terrain, 0);
  }

  describe("findPath", () => {
    it("returns parabolic arc between two points", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map);

      const from = map.ref(10, 50);
      const to = map.ref(90, 50);

      const path = finder.findPath(from, to);

      expect(path).not.toBeNull();
      expect(path!.length).toBe(39);
      expect(path![0]).toBe(from);
      expect(path![path!.length - 1]).toBe(to);
    });

    it("throws error for multiple start points", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map);

      const from = [map.ref(10, 50), map.ref(20, 50)];
      const to = map.ref(90, 50);

      expect(() => finder.findPath(from, to)).toThrow(
        "does not support multiple start points",
      );
    });

    it("handles same start and end point", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map);

      const tile = map.ref(50, 50);

      const path = finder.findPath(tile, tile);

      expect(path).not.toBeNull();
      // Parabola still creates arc even for same point (uses fixed height)
      expect(path!.length).toBe(26);
    });

    it("creates arc across map", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map);

      const from = map.ref(0, 50);
      const to = map.ref(99, 50);

      const path = finder.findPath(from, to);

      expect(path).not.toBeNull();
      expect(path!.length).toBe(43);
      expect(path![0]).toBe(from);
      expect(path![path!.length - 1]).toBe(to);
    });
  });

  describe("next (stepping)", () => {
    it("returns NEXT with node when not at destination", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map);

      const from = map.ref(10, 50);
      const to = map.ref(90, 50);

      const result = finder.next(from, to);

      expect(result.status).toBe(PathStatus.NEXT);
      expect("node" in result).toBe(true);
    });

    it("eventually returns COMPLETE", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map, { increment: 10 });

      const from = map.ref(10, 50);
      const to = map.ref(90, 50);

      // Keep stepping until complete
      let result = finder.next(from, to);
      let steps = 0;
      const maxSteps = 1000;

      while (result.status !== PathStatus.COMPLETE && steps < maxSteps) {
        expect(result.status).toBe(PathStatus.NEXT);
        const node = (
          result as { status: typeof PathStatus.NEXT; node: number }
        ).node;
        result = finder.next(node, to);
        steps++;
      }

      expect(result.status).toBe(PathStatus.COMPLETE);
      const completeNode = (
        result as { status: typeof PathStatus.COMPLETE; node: number }
      ).node;
      expect(completeNode).toBe(to);
    });

    it("respects speed parameter (higher speed = further movement)", () => {
      const map = createLargeMap();
      const finder1 = UniversalPathFinding.Parabola(map);
      const finder2 = UniversalPathFinding.Parabola(map);

      const from = map.ref(10, 50);
      const to = map.ref(90, 50);

      // Step with speed 1
      const result1 = finder1.next(from, to, 1);

      // Step with speed 5
      const result2 = finder2.next(from, to, 5);

      // Both should be NEXT (not at destination yet)
      expect(result1.status).toBe(PathStatus.NEXT);
      expect(result2.status).toBe(PathStatus.NEXT);

      const node1 = (
        result1 as { status: typeof PathStatus.NEXT; node: number }
      ).node;
      const node2 = (
        result2 as { status: typeof PathStatus.NEXT; node: number }
      ).node;

      // Speed 5 should move strictly further than speed 1
      const dist1 = map.manhattanDist(from, node1);
      const dist2 = map.manhattanDist(from, node2);
      expect(dist2).toBeGreaterThan(dist1);

      // Verify index progression confirms speed affects curve traversal
      expect(finder2.currentIndex()).toBeGreaterThan(finder1.currentIndex());
    });
  });

  describe("options", () => {
    it("increment option affects path density", () => {
      const map = createLargeMap();
      const finder1 = UniversalPathFinding.Parabola(map, { increment: 1 });
      const finder2 = UniversalPathFinding.Parabola(map, { increment: 10 });

      const from = map.ref(10, 50);
      const to = map.ref(90, 50);

      const path1 = finder1.findPath(from, to);
      const path2 = finder2.findPath(from, to);

      expect(path1).not.toBeNull();
      expect(path2).not.toBeNull();
      // Smaller increment = more points
      expect(path1!.length).toBeGreaterThan(path2!.length);
    });

    it("distanceBasedHeight option affects arc height", () => {
      const map = createLargeMap();
      const finder1 = UniversalPathFinding.Parabola(map, {
        distanceBasedHeight: true,
      });
      const finder2 = UniversalPathFinding.Parabola(map, {
        distanceBasedHeight: false,
      });

      const from = map.ref(10, 50);
      const to = map.ref(90, 50);

      const path1 = finder1.findPath(from, to);
      const path2 = finder2.findPath(from, to);

      expect(path1).not.toBeNull();
      expect(path2).not.toBeNull();

      // With distanceBasedHeight=true, path should have Y deviation
      // With distanceBasedHeight=false, path should be more direct
      const getMaxYDeviation = (path: number[]) => {
        const midY = map.y(from);
        return Math.max(...path.map((t) => Math.abs(map.y(t) - midY)));
      };

      const dev1 = getMaxYDeviation(path1!);
      const dev2 = getMaxYDeviation(path2!);
      expect(dev1).toBeGreaterThan(dev2);
    });

    it("directionUp option affects arc direction", () => {
      const map = createLargeMap();
      const finderUp = UniversalPathFinding.Parabola(map, {
        directionUp: true,
      });
      const finderDown = UniversalPathFinding.Parabola(map, {
        directionUp: false,
      });

      const from = map.ref(10, 50);
      const to = map.ref(90, 50);

      const pathUp = finderUp.findPath(from, to);
      const pathDown = finderDown.findPath(from, to);

      expect(pathUp).not.toBeNull();
      expect(pathDown).not.toBeNull();

      // Get midpoint Y values
      const midIdx = Math.floor(pathUp!.length / 2);
      const midY_Up = map.y(pathUp![midIdx]);
      const midY_Down = map.y(pathDown![midIdx]);
      const startY = map.y(from);

      // directionUp=true means Y decreases (goes "up" on screen)
      // directionUp=false means Y increases (goes "down" on screen)
      expect(midY_Up).toBeLessThan(startY);
      expect(midY_Down).toBeGreaterThan(startY);
    });
  });

  describe("invalidate", () => {
    it("resets curve state to initial", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map);

      const from = map.ref(10, 50);
      const to = map.ref(90, 50);

      // Build curve and step with high speed to advance index
      finder.next(from, to, 10);
      finder.next(from, to, 10);
      finder.next(from, to, 10);
      const indexBefore = finder.currentIndex();
      // After multiple high-speed steps, index should have advanced
      expect(indexBefore).toBeGreaterThan(0);

      // Invalidate
      finder.invalidate();

      // After invalidation, index should be 0
      expect(finder.currentIndex()).toBe(0);

      // Can still use finder after invalidation - should return NEXT since not at dest
      const result = finder.next(from, to);
      expect(result.status).toBe(PathStatus.NEXT);
    });

    it("clears curve so new curve is created for same destination", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map);

      const from = map.ref(10, 50);
      const to = map.ref(90, 50);

      // Get first result
      const result1 = finder.next(from, to);
      expect(result1.status).toBe(PathStatus.NEXT);
      const tile1 = (
        result1 as { status: typeof PathStatus.NEXT; node: number }
      ).node;

      // Invalidate
      finder.invalidate();

      // Same call should return same tile (fresh curve from same start)
      const result2 = finder.next(from, to);
      expect(result2.status).toBe(PathStatus.NEXT);
      const tile2 = (
        result2 as { status: typeof PathStatus.NEXT; node: number }
      ).node;

      expect(tile1).toBe(tile2);
    });
  });

  describe("currentIndex", () => {
    it("returns 0 when no curve", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map);

      expect(finder.currentIndex()).toBe(0);
    });

    it("increments as path is stepped", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map);

      const from = map.ref(10, 50);
      const to = map.ref(90, 50);

      // Step multiple times to build up index
      finder.next(from, to);
      finder.next(from, to);
      const idx1 = finder.currentIndex();

      finder.next(from, to);
      finder.next(from, to);
      const idx2 = finder.currentIndex();

      // Index should increase after more steps
      expect(idx2).toBeGreaterThan(idx1);
    });

    it("higher speed increments index faster", () => {
      const map = createLargeMap();
      const finder1 = UniversalPathFinding.Parabola(map);
      const finder2 = UniversalPathFinding.Parabola(map);

      const from = map.ref(10, 50);
      const to = map.ref(90, 50);

      // Step with speed 1
      finder1.next(from, to, 1);
      const idx1 = finder1.currentIndex();

      // Step with speed 5
      finder2.next(from, to, 5);
      const idx2 = finder2.currentIndex();

      // Higher speed should result in higher index after same number of steps
      expect(idx2).toBeGreaterThan(idx1);
    });
  });

  describe("short distances", () => {
    it("creates valid arc for distance < 50 (PARABOLA_MIN_HEIGHT)", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map, {
        distanceBasedHeight: true,
      });

      // Distance of 30 is less than PARABOLA_MIN_HEIGHT (50)
      const from = map.ref(50, 50);
      const to = map.ref(80, 50);

      const path = finder.findPath(from, to);

      expect(path).not.toBeNull();
      expect(path!.length).toBe(28);
      expect(path![0]).toBe(from);
      expect(path![path!.length - 1]).toBe(to);
    });

    it("creates valid path for adjacent tiles (distance=1)", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map);

      const from = map.ref(50, 50);
      const to = map.ref(51, 50);

      const path = finder.findPath(from, to);

      expect(path).not.toBeNull();
      // Parabola creates arc even for adjacent tiles (uses fixed height)
      expect(path!.length).toBe(26);
      expect(path![0]).toBe(from);
      expect(path![path!.length - 1]).toBe(to);
    });

    it("creates valid path for very short distance (distance=5)", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map, {
        distanceBasedHeight: true,
      });

      const from = map.ref(50, 50);
      const to = map.ref(55, 50);

      const path = finder.findPath(from, to);

      expect(path).not.toBeNull();
      expect(path![0]).toBe(from);
      expect(path![path!.length - 1]).toBe(to);
    });
  });

  describe("map boundary clipping", () => {
    it("arc clipped at map top boundary (directionUp near y=0)", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map, {
        directionUp: true,
        distanceBasedHeight: true,
      });

      // Start near top of map
      const from = map.ref(10, 5);
      const to = map.ref(90, 5);

      const path = finder.findPath(from, to);

      expect(path).not.toBeNull();
      // All points should be within map bounds (y >= 0)
      for (const t of path!) {
        expect(map.y(t)).toBeGreaterThanOrEqual(0);
      }
    });

    it("arc clipped at map bottom boundary (directionDown near y=max)", () => {
      const map = createLargeMap();
      const finder = UniversalPathFinding.Parabola(map, {
        directionUp: false,
        distanceBasedHeight: true,
      });

      // Start near bottom of map (map is 100x100, so y=99 is max)
      const from = map.ref(10, 95);
      const to = map.ref(90, 95);

      const path = finder.findPath(from, to);

      expect(path).not.toBeNull();
      // All points should be within map bounds (y < 100)
      for (const t of path!) {
        expect(map.y(t)).toBeLessThan(100);
      }
    });
  });

  describe("determinism", () => {
    it("same inputs produce identical paths", () => {
      const map = createLargeMap();
      const finder1 = UniversalPathFinding.Parabola(map, {
        distanceBasedHeight: true,
      });
      const finder2 = UniversalPathFinding.Parabola(map, {
        distanceBasedHeight: true,
      });

      const from = map.ref(10, 50);
      const to = map.ref(90, 50);

      const path1 = finder1.findPath(from, to);
      const path2 = finder2.findPath(from, to);

      expect(path1).toEqual(path2);
    });
  });
});

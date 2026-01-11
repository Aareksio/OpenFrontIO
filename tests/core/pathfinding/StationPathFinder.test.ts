import { describe, expect, it } from "vitest";
import { TileRef } from "../../../src/core/game/GameMap";
import { Cluster, TrainStation } from "../../../src/core/game/TrainStation";
import { StationPathFinder } from "../../../src/core/pathfinding/StationPathFinder";
import { PathStatus } from "../../../src/core/pathfinding/types";

/**
 * StationPathFinder tests with mock infrastructure.
 *
 * Test topology:
 *   Cluster A: stations 0, 1, 2 (connected: 0-1-2)
 *   Cluster B: stations 3, 4 (connected: 3-4)
 *
 * Tile positions:
 *   Station 0: (0, 0)
 *   Station 1: (5, 0)
 *   Station 2: (10, 0)
 *   Station 3: (0, 10)
 *   Station 4: (5, 10)
 */

function createMockCluster(): Cluster {
  const stations = new Set<TrainStation>();
  const cluster = {
    stations,
    has: (s: TrainStation) => stations.has(s),
    addStation: (s: TrainStation) => stations.add(s),
    removeStation: () => {},
    addStations: () => {},
    merge: () => {},
    availableForTrade: () => new Set<TrainStation>(),
    size: () => stations.size,
    clear: () => stations.clear(),
  } as unknown as Cluster;
  return cluster;
}

// Mock TrainStation
function createMockStation(
  id: number,
  cluster: Cluster,
  tileX: number,
  tileY: number,
): TrainStation {
  const tile = (tileY * 100 + tileX) as TileRef; // Encode as single number
  const station = {
    id,
    getCluster: () => cluster,
    tile: () => tile,
    neighbors: () => [] as TrainStation[], // Set after creation
  } as TrainStation;
  cluster.addStation(station);
  return station;
}

// Mock Game with StationManager
function createMockGame(stations: Map<number, TrainStation>) {
  return {
    x: (t: TileRef) => t % 100,
    y: (t: TileRef) => Math.floor(t / 100),
    railNetwork: () => ({
      stationManager: () => ({
        getById: (id: number) => stations.get(id),
        count: () => stations.size,
      }),
    }),
  };
}

// Mock GenericAStar
function createMockAStar(pathMap: Map<string, number[] | null>) {
  return {
    findPath: (from: number | number[], to: number): number[] | null => {
      const fromKey = Array.isArray(from)
        ? from.sort().join(",")
        : String(from);
      const key = `${fromKey}->${to}`;
      return pathMap.get(key) ?? null;
    },
  };
}

describe("StationPathFinder", () => {
  // Setup test topology
  const clusterA = createMockCluster();
  const clusterB = createMockCluster();

  const station0 = createMockStation(0, clusterA, 0, 0);
  const station1 = createMockStation(1, clusterA, 5, 0);
  const station2 = createMockStation(2, clusterA, 10, 0);
  const station3 = createMockStation(3, clusterB, 0, 10);
  const station4 = createMockStation(4, clusterB, 5, 10);

  // Set up neighbor relationships
  (station0 as { neighbors: () => TrainStation[] }).neighbors = () => [
    station1,
  ];
  (station1 as { neighbors: () => TrainStation[] }).neighbors = () => [
    station0,
    station2,
  ];
  (station2 as { neighbors: () => TrainStation[] }).neighbors = () => [
    station1,
  ];
  (station3 as { neighbors: () => TrainStation[] }).neighbors = () => [
    station4,
  ];
  (station4 as { neighbors: () => TrainStation[] }).neighbors = () => [
    station3,
  ];

  const stations = new Map<number, TrainStation>([
    [0, station0],
    [1, station1],
    [2, station2],
    [3, station3],
    [4, station4],
  ]);

  // Pre-computed paths (A* returns station IDs)
  const pathMap = new Map<string, number[] | null>([
    ["0->1", [0, 1]],
    ["0->2", [0, 1, 2]],
    ["1->0", [1, 0]],
    ["1->2", [1, 2]],
    ["2->0", [2, 1, 0]],
    ["2->1", [2, 1]],
    ["3->4", [3, 4]],
    ["4->3", [4, 3]],
    // Multi-source paths
    ["0,1->2", [0, 1, 2]],
    ["0,2->1", [0, 1]], // Closest source to 1
  ]);

  const mockGame = createMockGame(stations);
  const mockAStar = createMockAStar(pathMap);

  function createPathFinder(): StationPathFinder {
    return new StationPathFinder(mockGame as any, mockAStar as any);
  }

  describe("findPath", () => {
    it("finds path between stations in same cluster", () => {
      const finder = createPathFinder();
      const path = finder.findPath(station0, station2);

      expect(path).not.toBeNull();
      expect(path!.length).toBe(3);
      expect(path![0]).toBe(station0);
      expect(path![1]).toBe(station1);
      expect(path![2]).toBe(station2);
    });

    it("returns null for stations in different clusters", () => {
      const finder = createPathFinder();
      const path = finder.findPath(station0, station3);

      expect(path).toBeNull();
    });

    it("finds path for adjacent stations", () => {
      const finder = createPathFinder();
      const path = finder.findPath(station0, station1);

      expect(path).not.toBeNull();
      expect(path!.length).toBe(2);
      expect(path![0]).toBe(station0);
      expect(path![1]).toBe(station1);
    });

    it("filters multi-source by cluster", () => {
      const finder = createPathFinder();
      // station0 and station3 are in different clusters, station2 is target in clusterA
      const path = finder.findPath([station0, station3], station2);

      expect(path).not.toBeNull();
      // Only station0 is in same cluster as station2
      expect(path![0]).toBe(station0);
      expect(path![path!.length - 1]).toBe(station2);
    });

    it("returns null when all sources in different cluster", () => {
      const finder = createPathFinder();
      const path = finder.findPath([station3, station4], station0);

      expect(path).toBeNull();
    });
  });

  describe("next", () => {
    it("returns COMPLETE when at destination", () => {
      const finder = createPathFinder();
      const result = finder.next(station0, station0);

      expect(result.status).toBe(PathStatus.COMPLETE);
      expect((result as { node: TrainStation }).node).toBe(station0);
    });

    it("returns NOT_FOUND for different clusters", () => {
      const finder = createPathFinder();
      const result = finder.next(station0, station3);

      expect(result.status).toBe(PathStatus.NOT_FOUND);
    });

    it("returns NEXT with correct next station", () => {
      const finder = createPathFinder();
      const result = finder.next(station0, station2);

      expect(result.status).toBe(PathStatus.NEXT);
      expect((result as { node: TrainStation }).node).toBe(station1);
    });

    it("steps through path correctly", () => {
      const finder = createPathFinder();

      // Step 1: 0 -> 2, should return 1
      const r1 = finder.next(station0, station2);
      expect(r1.status).toBe(PathStatus.NEXT);
      expect((r1 as { node: TrainStation }).node).toBe(station1);

      // Step 2: 1 -> 2, should return 2
      const r2 = finder.next(station1, station2);
      expect(r2.status).toBe(PathStatus.NEXT);
      expect((r2 as { node: TrainStation }).node).toBe(station2);

      // Step 3: 2 -> 2, should be COMPLETE
      const r3 = finder.next(station2, station2);
      expect(r3.status).toBe(PathStatus.COMPLETE);
    });

    it("dist threshold returns COMPLETE early", () => {
      const finder = createPathFinder();
      // station0 at (0,0), station1 at (5,0), manhattan dist = 5
      // With dist=5, should return COMPLETE from station0
      const result = finder.next(station0, station1, 5);

      expect(result.status).toBe(PathStatus.COMPLETE);
      expect((result as { node: TrainStation }).node).toBe(station0);
    });

    it("dist threshold allows navigation when above threshold", () => {
      const finder = createPathFinder();
      // station0 at (0,0), station1 at (5,0), manhattan dist = 5
      // With dist=4, should navigate (5 > 4)
      const result = finder.next(station0, station1, 4);

      expect(result.status).toBe(PathStatus.NEXT);
    });
  });

  describe("invalidate", () => {
    it("clears cached path", () => {
      const finder = createPathFinder();

      // Build up cache
      finder.next(station0, station2);

      // Invalidate
      finder.invalidate();

      // After invalidate, path should be recomputed
      // (We can't directly test cache, but ensure no errors)
      const result = finder.next(station0, station2);
      expect(result.status).toBe(PathStatus.NEXT);
    });

    it("handles destination change correctly", () => {
      const finder = createPathFinder();

      // Navigate to station2
      const r1 = finder.next(station0, station2);
      expect(r1.status).toBe(PathStatus.NEXT);
      expect((r1 as { node: TrainStation }).node).toBe(station1);

      // Change destination to station1 (no explicit invalidate needed)
      const r2 = finder.next(station0, station1);
      expect(r2.status).toBe(PathStatus.NEXT);
      expect((r2 as { node: TrainStation }).node).toBe(station1);
    });
  });

  describe("determinism", () => {
    it("produces identical results on repeated calls", () => {
      const finder1 = createPathFinder();
      const finder2 = createPathFinder();

      const path1 = finder1.findPath(station0, station2);
      const path2 = finder2.findPath(station0, station2);

      expect(path1).toEqual(path2);
    });

    it("next() produces identical results", () => {
      const finder1 = createPathFinder();
      const finder2 = createPathFinder();

      const r1 = finder1.next(station0, station2);
      const r2 = finder2.next(station0, station2);

      expect(r1.status).toBe(r2.status);
      expect((r1 as { node: TrainStation }).node).toBe(
        (r2 as { node: TrainStation }).node,
      );
    });
  });
});

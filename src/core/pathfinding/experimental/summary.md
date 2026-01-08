# Pathfinding Algorithm Exploration

Naval pathfinding on ~4000x2000 grid. 4-direction, no diagonals. Online only.

## Benchmark

```
npx tsx ./tests/pathfinding/benchmark/run.ts --synthetic --all [adapter]
```

| Adapter      | Init   | Path  | Notes                                       |
| ------------ | ------ | ----- | ------------------------------------------- |
| `hpa.cached` | 217ms  | 233ms | Production default. Requires preprocessing. |
| `hpa`        | 212ms  | 604ms | No caching. Requires preprocessing.         |
| `a.baseline` | ~100ms | 480ms | Best online. Fully inlined.                 |
| `a.general`  | ~100ms | 560ms | Generic interface. 17% slower.              |

## Code Organization

| File                 | Purpose                                    |
| -------------------- | ------------------------------------------ |
| `AStar.ts`           | `AStar` interface, `GenericAStar`, adapters |
| `GameMapAStar.ts`    | Fully inlined A\* for performance          |
| `AStarPathFinder.ts` | PathFinder adapter                         |
| `MiniAStar.ts`       | Minimap conversion + path upscaling        |
| `PriorityQueue.ts`   | MinHeap + BucketQueue                      |

### Architecture

```
AStarPathFinder
    └── MiniAStar (game↔minimap, factory: (map) => AStar)
        └── GameMapAStar / GenericAStar (implements AStar, takes GameMap)
```

## Optimization Results

From binary heap (1294ms) to bucket queue + inlining (480ms) = 62% speedup.

| Technique             | Speedup |
| --------------------- | ------- |
| Bucket queue          | 39%     |
| Direct terrain access | 24%     |
| Uniform cost=1        | 19%     |

## Rejected Algorithms

| Algorithm           | Reason                   |
| ------------------- | ------------------------ |
| JPS                 | Open water symmetry      |
| Bidirectional       | 12% slower overall       |
| Beam Search         | 17-44x slower on failure |
| Theta\*/LOS         | Too expensive per node   |
| IDA\*               | Re-expansion overhead    |
| RSR, ALT, Block A\* | Require preprocessing    |

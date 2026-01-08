# Pathfinding Algorithm Exploration

**Goal:** Learn from online A* optimizations to potentially speed up HPA further.

**Problem:** Naval pathfinding on ~4000x2000 tile world map. 4-direction movement, no diagonals.

**Constraints:** Online only (no preprocessing). Single-threaded JS.

## Available Adapters

Benchmark: `npx tsx ./tests/pathfinding/benchmark/run.ts --synthetic --all [adapter]`

| Adapter | Init | Path | Notes |
|---------|------|------|-------|
| `hpa.cached` | 217ms | 233ms | HPA with path caching. Requires preprocessing. Production default. |
| `hpa` | 212ms | 604ms | HPA without caching. Requires preprocessing. |
| `a.baseline` | ~100ms | 484ms | Bucket queue + direct terrain + cost=1. Best online. |
| `a.general` | ~100ms | 557ms | Generic graph interface. 15% slower than baseline. |

**Note:** `a.legacy` exists but is 50x slower (65s) - don't benchmark.

## Optimization Journey

Starting from binary heap A* (1294ms), applied incremental optimizations:

| Step | Technique | Time | Speedup | Mechanism |
|------|-----------|------|---------|-----------|
| 1 | Binary heap baseline | 1294ms | - | MinHeap + GraphAdapter + 15x weighted heuristic |
| 2 | + Bucket queue | 790ms | 39% | O(1) vs O(log n) heap ops. f-values are integers. |
| 3 | + Direct terrain | 600ms | 24% | Inline bit check vs function calls (isWater, cost). |
| 4 | + Uniform cost=1 | 489ms | 19% | Skip magnitude lookup. Simpler g-score math. |

Effects are roughly additive. Total speedup: 62% (1294ms → 489ms).

## Rejected

### Tested

| Idea | Result |
|------|--------|
| Tie-breaking by g-value | 22% slower. Comparison overhead > reduced expansions. |
| LOS Shortcut | 1.7-4.5% faster. Rarely hits on world maps. Not worth complexity. |
| Beam Search | 17-44x slower when beam fails (requires full A* restart). |
| JPS | Open water has too much symmetry. |
| Lazy Theta* | No improvement. |
| Cache x,y coords | Modulo/division not expensive. |
| Bidirectional | 28% faster on long routes, 12% slower overall. No clear use case. |

### Considered

| Idea | Reason |
|------|--------|
| Theta* | LOS check per node too expensive |
| Focal Search | 15x weight already greedy; zigzag inherent to 4-dir |
| Corridor Search | Fails at coastlines; failures waste coarse work |
| IDA* | Re-expansion overhead on large grids |
| RSR, ALT, Block A* | Require preprocessing |
| D*/LPA* | For replanning, not single queries |
| SMA*, Fringe Search | Memory-focused, not faster |

## Future Ideas

| Idea | Effort | Potential |
|------|--------|-----------|
| Incremental heuristic | Medium | Small - compute h delta from direction |
| WASM hot loop | High | Unknown - JS engines already optimized |
| Memory layout tuning | Medium | Small - interleave arrays for cache |

## Code Organization

**Current structure (2026-01-08):**
- `PriorityQueue.ts` - interface + MinHeap + BucketQueue implementations
- `GridAStar.ts` - optimized grid A* with inlined neighbors, direct terrain access
- `GridAStarAdapter.ts` - PathFinder adapter for GridAStar
- `GeneralAStar.ts` - generic A* with adapter interface (neighbors, cost, heuristic)
- `GeneralAStarAdapter.ts` - PathFinder adapter + GridGraphAdapter for benchmarking

**Philosophy:**
- `FastAStar` (navmesh) = generic adapter-based graph A* with MinHeap
- `GridAStar` (experimental) = specialized version with inlined ops for grids
- `GeneralAStar` (experimental) = generic graph A* with BucketQueue + stamp arrays
- GridAStar is hand-optimized specialization of GeneralAStar (15% faster)

## Generalization Cost

`a.general` vs `a.baseline`: 557ms vs 484ms = **15% slower**

| Approach | Time | vs Baseline |
|----------|------|-------------|
| Array alloc per expansion | 643ms | +33% |
| Callback per neighbor | 700ms | +45% (worse!) |
| Reusable `number[]` buffer | 581ms | +20% |
| Reusable `Int32Array` buffer | 563ms | +16% |
| **+ Goal coord caching** | 557ms | **+15%** (best) |

Findings:
- Callback overhead > array GC pressure
- TypedArray buffer faster than JS array
- Caching goal coords saves 2 modulos per heuristic call
- Branch-based caching slower than explicit setGoal()

Kept: BucketQueue, stamp arrays, Int32Array buffer, goal caching via setGoal().

## Future Unification with NavMesh

| Task | Effort | Potential |
|------|--------|-----------|
| BucketQueue in FastAStar | Low | ~39% speedup on local searches |

**Note:** NavMesh local searches (BoundedGameMapAdapter) use uniform cost on grids.
BucketQueue could replace MinHeap for ~39% speedup (O(1) vs O(log n) heap ops).

## Missing from exploration
- [ ] Profile memory allocation (GC pressure during search)
- [ ] Measure actual cache miss rates
- [ ] Benchmark BucketQueue in NavMesh local searches

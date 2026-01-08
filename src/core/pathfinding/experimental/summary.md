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
| `a.baseline` | ~100ms | 489ms | Bucket queue + direct terrain + cost=1. Best online. |

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

**Philosophy:**
- `FastAStar` (navmesh) = generic adapter-based graph A*
- `GridAStar` (experimental) = specialized version with inlined ops for grids
- Both conceptually same algorithm; GridAStar is hand-optimized specialization

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

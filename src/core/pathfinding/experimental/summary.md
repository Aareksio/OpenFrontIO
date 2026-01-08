# Pathfinding Algorithm Exploration

Naval pathfinding on ~4000x2000 tile world map. 4-direction movement, no diagonals.

**Constraints:** Online only (no preprocessing). Single-threaded JS.

> **Keep this doc current.** Update after each experiment with results and rationale.

### Missing from exploration
- [ ] Profile memory allocation (GC pressure during search)
- [ ] Test typed arrays (Uint32Array) for open/closed sets
- [ ] Measure actual cache miss rates

## Available Adapters

Benchmark: `npx tsx ./tests/pathfinding/benchmark/run.ts --synthetic --all [adapter]`

**Note:** Never benchmark `a.legacy` - it's 50x slower than baseline (65s vs 1.3s).

| Adapter | Init | Path | Notes |
|---------|------|------|-------|
| `hpa.cached` | 217ms | 233ms | HPA with path caching. Requires preprocessing. Production default. |
| `hpa` | 212ms | 604ms | HPA without caching. Requires preprocessing. |
| `a.uniform` | ~90ms | 480ms | Bucket queue + direct terrain + cost=1. Best online. |
| `a.bucket` | ~90ms | 790ms | Bucket queue + GraphAdapter + variable cost. |
| `a.optimized` | ~90ms | 1294ms | Binary heap baseline. 15x weighted heuristic. |
| `a.bidirectional` | ~90ms | - | 28% faster on long routes, 12% slower overall. Situational. |

## Speedup Breakdown

Individual optimizations measured against `a.bucket` baseline:

| Optimization | Speedup | Mechanism |
|--------------|---------|-----------|
| Bucket queue | 39% | O(1) vs O(log n) heap ops. f-values are integers. |
| Direct terrain access | 24% | Inline bit check vs function calls (isWater, cost). |
| Uniform cost=1 | 19% | Skip magnitude lookup. Simpler g-score math. |

Effects are roughly additive.

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

import { Game } from '../../../game/Game';
import { GameMap, TileRef } from '../../../game/GameMap';
import { findWaterPathFromSeedsCoarseToFine } from './CoarseToFineWaterPath';
import { getWaterComponentIds } from './WaterComponents';

export function boatPathFromTileToShore(
  gm: GameMap,
  startTile: TileRef,
  dstShore: TileRef,
): TileRef[] | null {
  if (!gm.isValidRef(startTile) || !gm.isValidRef(dstShore)) return null;
  
  // BENCHMARK: We allow navigating to any water tile!
  // if (!gm.isShore(dstShore)) return null;

  let seedNodes: TileRef[] = [];
  let seedOrigins: TileRef[] = [];
  if (gm.isWater(startTile)) {
    seedNodes = [startTile];
    seedOrigins = [startTile];
  } else if (gm.isShore(startTile)) {
    const adj = adjacentWaterTiles(gm, startTile);
    if (adj.length === 0) return null;
    seedNodes = adj;
    seedOrigins = adj.map(() => startTile);
  } else {
    return null;
  }

  const targetWaterAll = adjacentWaterTiles(gm, dstShore);
  if (targetWaterAll.length === 0) return null;

  // Avoid impossible searches: restrict to the same connected water component.
  const ids = getWaterComponentIds(gm);
  const targetComps = new Set<number>();
  for (const t of targetWaterAll) {
    const id = ids[t] ?? 0;
    if (id !== 0) targetComps.add(id);
  }
  if (targetComps.size === 0) return null;

  const seedNodesFiltered: TileRef[] = [];
  const seedOriginsFiltered: TileRef[] = [];
  const seedComps = new Set<number>();
  for (let i = 0; i < seedNodes.length; i++) {
    const n = seedNodes[i]!;
    const id = ids[n] ?? 0;
    if (id === 0) continue;
    if (!targetComps.has(id)) continue;
    seedNodesFiltered.push(n);
    seedOriginsFiltered.push(seedOrigins[i]!);
    seedComps.add(id);
  }
  if (seedNodesFiltered.length === 0) return null;

  const targetWater = targetWaterAll.filter((t) => seedComps.has(ids[t] ?? 0));
  if (targetWater.length === 0) return null;

  // const startTime = performance.now();
  const result = findWaterPathFromSeedsCoarseToFine(
    gm,
    seedNodesFiltered,
    seedOriginsFiltered,
    targetWater,
    {
      kingMoves: true,
      noCornerCutting: true,
    },
    coarseBoatMapOrNull(gm),
  );
  // const duration = performance.now() - startTime;
  if (result === null) return null;

  if (gm.isWater(startTile)) {
    const path = [...result.path, dstShore];
    // console.log(
    //   `boatPathFromTileToShore: ${duration.toFixed(2)}ms, steps=${Math.max(
    //     0,
    //     path.length - 1,
    //   )}`,
    // );
    return path;
  }
  const path = [startTile, ...result.path, dstShore];
  // console.log(
  //   `boatPathFromTileToShore: ${duration.toFixed(2)}ms, steps=${Math.max(
  //     0,
  //     path.length - 1,
  //   )}`,
  // );
  return path;
}


export function boatPathFromTileToWater(
  gm: GameMap,
  startTile: TileRef,
  dstWater: TileRef,
): TileRef[] | null {
  if (!gm.isValidRef(startTile) || !gm.isValidRef(dstWater)) return null;
  if (!gm.isWater(dstWater)) return null;

  let seedNodes: TileRef[] = [];
  let seedOrigins: TileRef[] = [];
  if (gm.isWater(startTile)) {
    seedNodes = [startTile];
    seedOrigins = [startTile];
  } else if (gm.isShore(startTile)) {
    const adj = adjacentWaterTiles(gm, startTile);
    if (adj.length === 0) return null;
    seedNodes = adj;
    seedOrigins = adj.map(() => startTile);
  } else {
    return null;
  }

  // Avoid impossible searches: restrict seeds to the same connected water component.
  const ids = getWaterComponentIds(gm);
  const dstComp = ids[dstWater] ?? 0;
  if (dstComp === 0) return null;

  const seedNodesFiltered: TileRef[] = [];
  const seedOriginsFiltered: TileRef[] = [];
  for (let i = 0; i < seedNodes.length; i++) {
    const n = seedNodes[i]!;
    if ((ids[n] ?? 0) !== dstComp) continue;
    seedNodesFiltered.push(n);
    seedOriginsFiltered.push(seedOrigins[i]!);
  }
  if (seedNodesFiltered.length === 0) return null;

  // const startTime = performance.now();
  const result = findWaterPathFromSeedsCoarseToFine(
    gm,
    seedNodesFiltered,
    seedOriginsFiltered,
    [dstWater],
    {
      kingMoves: true,
      noCornerCutting: true,
    },
    coarseBoatMapOrNull(gm),
  );
  // const duration = performance.now() - startTime;
  if (result === null) return null;

  if (gm.isWater(startTile)) {
    // console.log(
    //   `boatPathFromTileToWater: ${duration.toFixed(2)}ms, steps=${Math.max(
    //     0,
    //     result.path.length - 1,
    //   )}`,
    // );
    return result.path;
  }
  const path = [startTile, ...result.path];
  // console.log(
  //   `boatPathFromTileToWater: ${duration.toFixed(2)}ms, steps=${Math.max(
  //     0,
  //     path.length - 1,
  //   )}`,
  // );
  return path;
}

function adjacentWaterTiles(gm: GameMap, shore: TileRef): TileRef[] {
  const out: TileRef[] = [];
  for (const n of gm.neighbors(shore)) {
    if (gm.isWater(n)) out.push(n);
  }
  return out;
}

function coarseBoatMapOrNull(gm: GameMap | Game): GameMap | null {
  return microMapOrNull(gm) ?? miniMapOrNull(gm);
}

function miniMapOrNull(gm: GameMap): GameMap | null {
  const mm = (gm as any).miniMap;
  if (typeof mm === "function") return mm.call(gm) as GameMap;
  return null;
}

function microMapOrNull(gm: GameMap): GameMap | null {
  const mm = (gm as any).microMap;
  if (typeof mm === "function") return mm.call(gm) as GameMap;
  return null;
}

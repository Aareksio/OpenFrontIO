import { PathFinding } from "../pathfinding/PathFinder";
import { Game, Player, UnitType } from "./Game";
import { andFN, GameMap, manhattanDistFN, TileRef } from "./GameMap";

export function canBuildTransportShip(
  game: Game,
  player: Player,
  tile: TileRef,
): TileRef | false {
  if (
    player.unitCount(UnitType.TransportShip) >= game.config().boatMaxNumber()
  ) {
    return false;
  }

  const dst = targetTransportTile(game, tile);
  if (dst === null) {
    return false;
  }

  const other = game.owner(tile);
  if (other === player) {
    return false;
  }
  if (other.isPlayer() && !player.canAttackPlayer(other)) {
    return false;
  }

  // Get destination water component
  const dstComponent = game.getWaterComponent(dst);
  if (dstComponent === null) return false;

  // Find player shore tiles on same component
  const validShores = Array.from(player.borderTiles()).filter(
    (t) => game.isShore(t) && game.hasWaterComponent(t, dstComponent),
  );
  if (validShores.length === 0) return false;

  // Find closest valid shore as spawn (manhattan)
  return validShores.reduce((closest, current) =>
    game.manhattanDist(dst, current) < game.manhattanDist(dst, closest)
      ? current
      : closest,
  );
}

export function targetTransportTile(gm: Game, tile: TileRef): TileRef | null {
  const owner = gm.playerBySmallID(gm.ownerID(tile));

  if (owner.isPlayer()) {
    // Find closest shore of target player to clicked tile (manhattan)
    const shoreTiles = Array.from((owner as Player).borderTiles()).filter((t) =>
      gm.isShore(t),
    );
    if (shoreTiles.length === 0) return null;

    return shoreTiles.reduce((closest, current) =>
      gm.manhattanDist(tile, current) < gm.manhattanDist(tile, closest)
        ? current
        : closest,
    );
  } else {
    // Terra nullius: BFS for nearby unowned shore tiles
    return closestShoreTN(gm, tile, 50);
  }
}

export function bestShoreDeploymentSource(
  gm: Game,
  player: Player,
  dst: TileRef,
): TileRef | false {
  const candidates = candidateShoreTiles(gm, player, dst);
  if (candidates.length === 0) return false;

  const path = PathFinding.Water(gm).findPath(candidates, dst);
  if (!path || path.length === 0) {
    console.warn(`bestShoreDeploymentSource: path not found`);
    return false;
  }
  // ShoreCoercingAStar prepends the original shore tile to path
  return path[0];
}

export function candidateShoreTiles(
  gm: Game,
  player: Player,
  target: TileRef,
): TileRef[] {
  const targetComponent = gm.getWaterComponent(target);
  if (targetComponent === null) return [];

  let closestManhattanDistance = Infinity;
  let minX = Infinity,
    minY = Infinity,
    maxX = -Infinity,
    maxY = -Infinity;

  let bestByManhattan: TileRef | null = null;
  const extremumTiles: Record<string, TileRef | null> = {
    minX: null,
    minY: null,
    maxX: null,
    maxY: null,
  };

  // Pre-filter to shores on same component
  const borderShoreTiles = Array.from(player.borderTiles()).filter(
    (t) => gm.isShore(t) && gm.hasWaterComponent(t, targetComponent),
  );

  for (const tile of borderShoreTiles) {
    const distance = gm.manhattanDist(tile, target);
    const cell = gm.cell(tile);

    // Manhattan-closest tile
    if (distance < closestManhattanDistance) {
      closestManhattanDistance = distance;
      bestByManhattan = tile;
    }

    // Extremum tiles
    if (cell.x < minX) {
      minX = cell.x;
      extremumTiles.minX = tile;
    } else if (cell.y < minY) {
      minY = cell.y;
      extremumTiles.minY = tile;
    } else if (cell.x > maxX) {
      maxX = cell.x;
      extremumTiles.maxX = tile;
    } else if (cell.y > maxY) {
      maxY = cell.y;
      extremumTiles.maxY = tile;
    }
  }

  // Calculate sampling interval to ensure we get at most 50 tiles
  const samplingInterval = Math.max(
    10,
    Math.ceil(borderShoreTiles.length / 50),
  );
  const sampledTiles = borderShoreTiles.filter(
    (_, index) => index % samplingInterval === 0,
  );

  const candidates = [
    bestByManhattan,
    extremumTiles.minX,
    extremumTiles.minY,
    extremumTiles.maxX,
    extremumTiles.maxY,
    ...sampledTiles,
  ].filter(Boolean) as number[];

  return candidates;
}

function closestShoreTN(
  gm: GameMap,
  tile: TileRef,
  searchDist: number,
): TileRef | null {
  const tn = Array.from(
    gm.bfs(
      tile,
      andFN((_, t) => !gm.hasOwner(t), manhattanDistFN(tile, searchDist)),
    ),
  )
    .filter((t) => gm.isShore(t))
    .sort((a, b) => gm.manhattanDist(tile, a) - gm.manhattanDist(tile, b));
  if (tn.length === 0) {
    return null;
  }
  return tn[0];
}

import { PathFinding } from "../pathfinding/PathFinder";
import { TileSpatialQuery } from "../pathfinding/spatial/SpatialQuery";
import { Game, Player, UnitType } from "./Game";
import { TileRef } from "./GameMap";

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
  const spatial = new TileSpatialQuery(game.map());
  return spatial.manhattanNearest(validShores, dst) ?? false;
}

export function targetTransportTile(gm: Game, tile: TileRef): TileRef | null {
  const owner = gm.playerBySmallID(gm.ownerID(tile));
  const spatial = new TileSpatialQuery(gm.map());

  if (owner.isPlayer()) {
    // Find closest shore of target player to clicked tile (manhattan)
    const shoreTiles = Array.from((owner as Player).borderTiles()).filter((t) =>
      gm.isShore(t),
    );
    if (shoreTiles.length === 0) return null;

    return spatial.manhattanNearest(shoreTiles, tile);
  } else {
    // Terra nullius: BFS for nearby unowned shore tiles
    return spatial.bfsNearest(
      tile,
      50,
      (t) => !gm.hasOwner(t) && gm.isShore(t),
    );
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
  // ShoreCoercingTransformer prepends the original shore tile to path
  return path[0];
}

export function candidateShoreTiles(
  gm: Game,
  player: Player,
  target: TileRef,
): TileRef[] {
  const targetComponent = gm.getWaterComponent(target);
  if (targetComponent === null) return [];

  const spatial = new TileSpatialQuery(gm.map());

  // Pre-filter to shores on same component
  const borderShoreTiles = Array.from(player.borderTiles()).filter(
    (t) => gm.isShore(t) && gm.hasWaterComponent(t, targetComponent),
  );

  if (borderShoreTiles.length === 0) return [];

  // Manhattan-closest tile
  const bestByManhattan = spatial.manhattanNearest(borderShoreTiles, target);

  // Extremum tiles
  let minX = Infinity,
    minY = Infinity,
    maxX = -Infinity,
    maxY = -Infinity;
  const extremumTiles: Record<string, TileRef | null> = {
    minX: null,
    minY: null,
    maxX: null,
    maxY: null,
  };

  for (const tile of borderShoreTiles) {
    const cell = gm.cell(tile);
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

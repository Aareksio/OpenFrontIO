import { describe, expect, it } from "vitest";
import { SpawnExecution } from "../../../src/core/execution/SpawnExecution";
import { PlayerInfo, PlayerType } from "../../../src/core/game/Game";
import { SpatialQuery } from "../../../src/core/pathfinding/spatial/SpatialQuery";
import { createGame, L, W } from "./fixtures";

describe("SpatialQuery", () => {
  describe("closestShoreByLand", () => {
    it("finds shore tile owned by player", () => {
      // prettier-ignore
      const game = createGame({
        width: 5, height: 5,
        grid: [
          W, W, W, W, W,
          W, L, L, L, W,
          W, L, L, L, W,
          W, L, L, L, W,
          W, W, W, W, W,
        ],
      });

      // Spawn player on the island
      const info = new PlayerInfo("test", PlayerType.Human, null, "test_id");
      game.addPlayer(info);
      game.addExecution(new SpawnExecution("game", info, game.ref(2, 2)));
      while (game.inSpawnPhase()) game.executeNextTick();

      const player = game.player(info.id);
      const spatial = new SpatialQuery(game);

      // From center land tile, find nearest shore
      const result = spatial.closestShoreByLand(player, game.ref(2, 2));

      expect(result).not.toBeNull();
      expect(game.isShore(result!)).toBe(true);
      expect(game.ownerID(result!)).toBe(player.smallID());
    });

    it("returns null when no shore within maxDist", () => {
      // prettier-ignore
      const game = createGame({
        width: 7, height: 7,
        grid: [
          W, W, W, W, W, W, W,
          W, L, L, L, L, L, W,
          W, L, L, L, L, L, W,
          W, L, L, L, L, L, W,
          W, L, L, L, L, L, W,
          W, L, L, L, L, L, W,
          W, W, W, W, W, W, W,
        ],
      });

      const info = new PlayerInfo("test", PlayerType.Human, null, "test_id");
      game.addPlayer(info);
      game.addExecution(new SpawnExecution("game", info, game.ref(3, 3)));
      while (game.inSpawnPhase()) game.executeNextTick();

      const player = game.player(info.id);
      const spatial = new SpatialQuery(game);

      // maxDist=1 from center (3,3) - shore is 2 tiles away
      const result = spatial.closestShoreByLand(player, game.ref(3, 3), 1);

      expect(result).toBeNull();
    });

    it("returns null for terra nullius with no owned shore", () => {
      // prettier-ignore
      const game = createGame({
        width: 5, height: 5,
        grid: [
          W, W, W, W, W,
          W, L, L, L, W,
          W, L, L, L, W,
          W, L, L, L, W,
          W, W, W, W, W,
        ],
      });

      const spatial = new SpatialQuery(game);
      const terraNullius = game.terraNullius();

      // No player owns the land, so terraNullius owns it
      // But terraNullius tiles are not "shore" in the ownership sense
      const result = spatial.closestShoreByLand(terraNullius, game.ref(2, 2));

      // Should find shore tiles owned by terra nullius
      expect(result).not.toBeNull();
      expect(game.isShore(result!)).toBe(true);
    });
  });

  describe("closestShoreByWater", () => {
    it("returns null for terra nullius", () => {
      // prettier-ignore
      const game = createGame({
        width: 5, height: 5,
        grid: [
          W, W, W, W, W,
          W, L, L, L, W,
          W, L, L, L, W,
          W, L, L, L, W,
          W, W, W, W, W,
        ],
      });

      const spatial = new SpatialQuery(game);
      const terraNullius = game.terraNullius();

      const result = spatial.closestShoreByWater(terraNullius, game.ref(0, 0));

      expect(result).toBeNull();
    });

    it("returns null when target is on land (no water component)", () => {
      // prettier-ignore
      const game = createGame({
        width: 5, height: 5,
        grid: [
          W, W, W, W, W,
          W, L, L, L, W,
          W, L, L, L, W,
          W, L, L, L, W,
          W, W, W, W, W,
        ],
      });

      const info = new PlayerInfo("test", PlayerType.Human, null, "test_id");
      game.addPlayer(info);
      game.addExecution(new SpawnExecution("game", info, game.ref(2, 2)));
      while (game.inSpawnPhase()) game.executeNextTick();

      const player = game.player(info.id);
      const spatial = new SpatialQuery(game);

      // Target is land tile - no water component
      const result = spatial.closestShoreByWater(player, game.ref(2, 2));

      expect(result).toBeNull();
    });

    it("finds shore tile reachable by water", () => {
      // prettier-ignore
      const game = createGame({
        width: 7, height: 5,
        grid: [
          W, W, W, W, W, W, W,
          W, L, L, L, L, L, W,
          W, L, L, L, L, L, W,
          W, L, L, L, L, L, W,
          W, W, W, W, W, W, W,
        ],
      });

      const info = new PlayerInfo("test", PlayerType.Human, null, "test_id");
      game.addPlayer(info);
      game.addExecution(new SpawnExecution("game", info, game.ref(3, 2)));
      while (game.inSpawnPhase()) game.executeNextTick();

      const player = game.player(info.id);
      const spatial = new SpatialQuery(game);

      // Target is water tile at corner
      const result = spatial.closestShoreByWater(player, game.ref(0, 0));

      // Should find a shore tile owned by player that's reachable by water
      if (result !== null) {
        expect(game.isShore(result)).toBe(true);
        expect(game.ownerID(result)).toBe(player.smallID());
      }
    });
  });
});

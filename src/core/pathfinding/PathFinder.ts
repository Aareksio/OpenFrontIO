import { Game } from "../game/Game";
import { GameMap, TileRef } from "../game/GameMap";
import { TrainStation } from "../game/TrainStation";
import { AirPathFinder } from "./AirPathFinder";
import { GenericAStar } from "./algorithms/AStar";
import { RailAdapter } from "./algorithms/AStarRailAdapter";
import { StationGraphAdapter } from "./algorithms/AStarStationAdapter";
import { GameMapAStar } from "./algorithms/AStarWaterAdapter";
import {
  ParabolaOptions,
  ParabolaUniversalPathFinder,
} from "./ParabolaPathFinder";
import { PathFinderBuilder } from "./PathFinderBuilder";
import { BresenhamSmoothingTransformer } from "./smoothing/BresenhamPathSmoother";
import { StationPathFinder } from "./StationPathFinder";
import { TilePathFinder } from "./TilePathFinder";
import { ComponentCheckTransformer } from "./transformers/ComponentCheckTransformer";
import { MiniMapTransformer } from "./transformers/MiniMapTransformer";
import { ShoreCoercingTransformer } from "./transformers/ShoreCoercingTransformer";
import { SteppingPathFinder } from "./types";

export interface RailOptions {
  waterPenalty?: number;
  directionChangePenalty?: number;
}

/**
 * Pathfinders that work with GameMap - usable in both simulation and UI layers
 */
export class UniversalPathFinding {
  static Parabola(
    gameMap: GameMap,
    options?: ParabolaOptions,
  ): ParabolaUniversalPathFinder {
    return new ParabolaUniversalPathFinder(gameMap, options);
  }
}

/**
 * Pathfinders that require Game - simulation layer only
 */
export class PathFinding {
  static Water(game: Game): SteppingPathFinder<TileRef> {
    const hpa = game.miniWaterHPA();
    const graph = game.miniWaterGraph();

    if (!hpa || !graph) {
      return PathFinding.WaterFallback(game);
    }

    const miniMap = game.miniMap();
    const finder = PathFinderBuilder.create(hpa)
      .wrap(
        (pf) =>
          new ComponentCheckTransformer(pf, (t) => graph.getComponentId(t)),
      )
      .wrap((pf) => new BresenhamSmoothingTransformer(pf, miniMap))
      .wrap((pf) => new ShoreCoercingTransformer(pf, miniMap))
      .wrap((pf) => new MiniMapTransformer(pf, game))
      .build();

    return new TilePathFinder(game, finder);
  }

  static WaterFallback(game: Game): SteppingPathFinder<TileRef> {
    const miniMap = game.miniMap();

    const finder = PathFinderBuilder.create(new GameMapAStar(miniMap))
      .wrap((pf) => new ShoreCoercingTransformer(pf, miniMap))
      .wrap((pf) => new MiniMapTransformer(pf, game))
      .build();

    return new TilePathFinder(game, finder);
  }

  static Rail(game: Game, options?: RailOptions): SteppingPathFinder<TileRef> {
    const miniMap = game.miniMap();
    const finder = PathFinderBuilder.create(
      new GenericAStar({
        adapter: new RailAdapter(miniMap, {
          waterPenalty: options?.waterPenalty,
          directionChangePenalty: options?.directionChangePenalty,
        }),
      }),
    )
      .wrap((pf) => new MiniMapTransformer(pf, game))
      .build();

    return new TilePathFinder(game, finder);
  }

  static Stations(game: Game): SteppingPathFinder<TrainStation> {
    const adapter = new StationGraphAdapter(game);
    const aStar = new GenericAStar({ adapter });
    return new StationPathFinder(game, aStar);
  }

  static Air(game: Game): SteppingPathFinder<TileRef> {
    return new AirPathFinder(game);
  }
}

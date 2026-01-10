import { PathFinder } from "./types";

type WrapFactory<T> = (pf: PathFinder<T>) => PathFinder<T>;

/**
 * PathFinderBuilder - fluent builder for composing PathFinder transformers.
 *
 * Usage:
 *   const finder = PathFinderBuilder.create(corePathFinder)
 *     .wrap((pf) => new SomeTransformer(pf, deps))
 *     .wrap((pf) => new AnotherTransformer(pf, deps))
 *     .build();
 */
export class PathFinderBuilder<T> {
  private wrappers: WrapFactory<T>[] = [];

  private constructor(private core: PathFinder<T>) {}

  static create<T>(core: PathFinder<T>): PathFinderBuilder<T> {
    return new PathFinderBuilder(core);
  }

  wrap(factory: WrapFactory<T>): this {
    this.wrappers.push(factory);
    return this;
  }

  build(): PathFinder<T> {
    return this.wrappers.reduce(
      (pf, wrapper) => wrapper(pf),
      this.core as PathFinder<T>,
    );
  }
}

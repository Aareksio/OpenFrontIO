import { PathFinder, PathResult, PathStatus } from "./types";

/**
 * PathFinderStepper - wraps a PathFinder and provides step-by-step traversal
 *
 * Handles path caching, invalidation, and incremental movement.
 * Generic over any PathFinder<T> implementation.
 */
export class PathFinderStepper<T> {
  private path: T[] | null = null;
  private pathIndex = 0;
  private lastTo: T | null = null;

  constructor(
    private finder: PathFinder<T>,
    private equals: (a: T, b: T) => boolean = (a, b) => a === b,
  ) {}

  /**
   * Get the next step on the path from `from` to `to`.
   * Returns PathResult with status and optional next node.
   */
  next(from: T, to: T): PathResult<T> {
    if (this.equals(from, to)) {
      return { status: PathStatus.COMPLETE, node: to };
    }

    // Invalidate cache if destination changed
    if (this.lastTo === null || !this.equals(this.lastTo, to)) {
      this.path = null;
      this.pathIndex = 0;
      this.lastTo = to;
    }

    // Compute path if not cached
    if (this.path === null) {
      try {
        this.path = this.finder.findPath(from, to);
      } catch {
        return { status: PathStatus.NOT_FOUND };
      }

      if (this.path === null) {
        return { status: PathStatus.NOT_FOUND };
      }

      this.pathIndex = 0;
      // Skip start if it matches current position
      if (this.path.length > 0 && this.equals(this.path[0], from)) {
        this.pathIndex = 1;
      }
    }

    // Validate we're still on path
    const expectedPos = this.path[this.pathIndex - 1];
    if (this.pathIndex > 0 && !this.equals(from, expectedPos)) {
      // Recompute path from current position
      try {
        this.path = this.finder.findPath(from, to);
      } catch {
        return { status: PathStatus.NOT_FOUND };
      }

      if (this.path === null) {
        return { status: PathStatus.NOT_FOUND };
      }

      this.pathIndex = 0;
      if (this.path.length > 0 && this.equals(this.path[0], from)) {
        this.pathIndex = 1;
      }
    }

    // Check if we've reached the end
    if (this.pathIndex >= this.path.length) {
      return { status: PathStatus.COMPLETE, node: to };
    }

    // Return next step
    const nextNode = this.path[this.pathIndex];
    this.pathIndex++;

    return { status: PathStatus.NEXT, node: nextNode };
  }

  /**
   * Clear cached path. Call when target changes or path becomes invalid.
   */
  invalidate(): void {
    this.path = null;
    this.pathIndex = 0;
    this.lastTo = null;
  }

  /**
   * Compute full path without stepping
   */
  findPath(from: T | T[], to: T): T[] | null {
    return this.finder.findPath(from, to);
  }
}

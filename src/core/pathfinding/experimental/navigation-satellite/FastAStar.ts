// Optimized A* using stamp-based visited tracking and typed arrays
// Works with node IDs represented as integers (0 to numNodes-1)

export interface FastAStarAdapter {
  // Get neighbors of a node
  getNeighbors(node: number): number[];

  // Get cost to move from one node to another
  getCost(from: number, to: number): number;

  // Heuristic estimate from node to goal
  heuristic(node: number, goal: number): number;
}

// Simple binary min-heap for open set using typed arrays
class MinHeap {
  private heap: Int32Array;
  private scores: Float32Array;
  private size = 0;

  constructor(capacity: number, scores: Float32Array) {
    this.heap = new Int32Array(capacity);
    this.scores = scores;
  }

  push(node: number): void {
    let i = this.size++;
    this.heap[i] = node;

    // Bubble up
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (this.scores[this.heap[parent]] <= this.scores[this.heap[i]]) {
        break;
      }
      // Swap
      const tmp = this.heap[parent];
      this.heap[parent] = this.heap[i];
      this.heap[i] = tmp;
      i = parent;
    }
  }

  pop(): number {
    const result = this.heap[0];
    this.heap[0] = this.heap[--this.size];

    // Bubble down
    let i = 0;
    while (true) {
      const left = (i << 1) + 1;
      const right = left + 1;
      let smallest = i;

      if (left < this.size && this.scores[this.heap[left]] < this.scores[this.heap[smallest]]) {
        smallest = left;
      }
      if (right < this.size && this.scores[this.heap[right]] < this.scores[this.heap[smallest]]) {
        smallest = right;
      }

      if (smallest === i) {
        break;
      }

      // Swap
      const tmp = this.heap[smallest];
      this.heap[smallest] = this.heap[i];
      this.heap[i] = tmp;
      i = smallest;
    }

    return result;
  }

  isEmpty(): boolean {
    return this.size === 0;
  }

  clear(): void {
    this.size = 0;
  }
}

export class FastAStar {
  private stamp = 1;
  private readonly visitedStamp: Uint32Array;
  private readonly gScore: Float32Array;
  private readonly fScore: Float32Array;
  private readonly cameFrom: Int32Array;
  private readonly openHeap: MinHeap;

  constructor(numNodes: number) {
    this.visitedStamp = new Uint32Array(numNodes);
    this.gScore = new Float32Array(numNodes);
    this.fScore = new Float32Array(numNodes);
    this.cameFrom = new Int32Array(numNodes);
    this.openHeap = new MinHeap(numNodes, this.fScore);
  }

  private nextStamp(): number {
    const stamp = this.stamp++;
    if (this.stamp === 0) {
      // Overflow - reset (extremely rare)
      this.visitedStamp.fill(0);
      this.stamp = 1;
    }
    return stamp;
  }

  search(
    start: number,
    goal: number,
    adapter: FastAStarAdapter,
    maxIterations: number = 100000
  ): number[] | null {
    const stamp = this.nextStamp();

    this.openHeap.clear();
    this.gScore[start] = 0;
    this.fScore[start] = adapter.heuristic(start, goal);
    this.cameFrom[start] = -1;
    this.openHeap.push(start);
    this.visitedStamp[start] = stamp;

    let iterations = 0;

    while (!this.openHeap.isEmpty() && iterations < maxIterations) {
      iterations++;

      const current = this.openHeap.pop();

      // Found goal
      if (current === goal) {
        return this.reconstructPath(start, goal);
      }

      const neighbors = adapter.getNeighbors(current);
      const currentGScore = this.gScore[current];

      for (const neighbor of neighbors) {
        const tentativeGScore = currentGScore + adapter.getCost(current, neighbor);

        // If we haven't visited this node, or found a better path
        if (this.visitedStamp[neighbor] !== stamp || tentativeGScore < this.gScore[neighbor]) {
          this.cameFrom[neighbor] = current;
          this.gScore[neighbor] = tentativeGScore;
          this.fScore[neighbor] = tentativeGScore + adapter.heuristic(neighbor, goal);

          if (this.visitedStamp[neighbor] !== stamp) {
            this.visitedStamp[neighbor] = stamp;
            this.openHeap.push(neighbor);
          }
        }
      }
    }

    return null; // No path found
  }

  private reconstructPath(start: number, goal: number): number[] {
    const path: number[] = [];
    let current = goal;

    while (current !== start) {
      path.push(current);
      current = this.cameFrom[current];

      // Safety check
      if (current === -1) {
        return [];
      }
    }

    path.push(start);
    path.reverse();
    return path;
  }
}

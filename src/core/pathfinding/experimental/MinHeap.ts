// Binary min-heap using typed arrays for efficient priority queue operations

export class MinHeap {
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
      if (this.scores[this.heap[parent]] <= this.scores[this.heap[i]]) break;
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

      if (
        left < this.size &&
        this.scores[this.heap[left]] < this.scores[this.heap[smallest]]
      ) {
        smallest = left;
      }
      if (
        right < this.size &&
        this.scores[this.heap[right]] < this.scores[this.heap[smallest]]
      ) {
        smallest = right;
      }
      if (smallest === i) break;

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

// Bucket queue: O(1) push/pop when keys are bounded integers
// Used for A* where f-values are integers (weight * Manhattan distance)

export class BucketQueue {
  private buckets: Int32Array[];
  private bucketSizes: Int32Array;
  private minBucket: number;
  private maxBucket: number;
  private size: number;

  constructor(maxF: number) {
    this.maxBucket = maxF + 1;
    this.buckets = new Array(this.maxBucket);
    this.bucketSizes = new Int32Array(this.maxBucket);
    this.minBucket = this.maxBucket;
    this.size = 0;
  }

  push(node: number, f: number): void {
    const bucket = Math.min(f | 0, this.maxBucket - 1);

    if (!this.buckets[bucket]) {
      this.buckets[bucket] = new Int32Array(64);
    }

    const size = this.bucketSizes[bucket];
    if (size >= this.buckets[bucket].length) {
      const newBucket = new Int32Array(this.buckets[bucket].length * 2);
      newBucket.set(this.buckets[bucket]);
      this.buckets[bucket] = newBucket;
    }

    this.buckets[bucket][size] = node;
    this.bucketSizes[bucket]++;
    this.size++;

    if (bucket < this.minBucket) {
      this.minBucket = bucket;
    }
  }

  pop(): number {
    while (this.minBucket < this.maxBucket) {
      const size = this.bucketSizes[this.minBucket];
      if (size > 0) {
        this.bucketSizes[this.minBucket]--;
        this.size--;
        return this.buckets[this.minBucket][size - 1];
      }
      this.minBucket++;
    }
    return -1;
  }

  isEmpty(): boolean {
    return this.size === 0;
  }

  clear(): void {
    this.bucketSizes.fill(0);
    this.minBucket = this.maxBucket;
    this.size = 0;
  }
}

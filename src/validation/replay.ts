import type { ValidationFrame } from "./types";

/** Recorded-state playback; this does not rerun dynamics or claim resimulation equivalence. */
export class ReplayPlayer {
  private frames: ValidationFrame[];
  private cursor = 0;
  private active = false;
  private time = 0;
  constructor(frames: readonly ValidationFrame[]) {
    if (
      frames.some(
        (f, i) =>
          !Number.isFinite(f.time) || (i > 0 && f.time < frames[i - 1].time),
      )
    )
      throw new Error("Replay timestamps must be finite and ordered");
    this.frames = structuredClone([...frames]);
    this.time = this.frames[0]?.time ?? 0;
  }
  get frame(): ValidationFrame | null {
    return this.frames[this.cursor] ?? null;
  }
  get index(): number {
    return this.cursor;
  }
  get playing(): boolean {
    return this.active;
  }
  get duration(): number {
    return this.frames.at(-1)?.time ?? 0;
  }
  play(): void {
    this.active = this.frames.length > 1 && this.time < this.duration;
  }
  pause(): void {
    this.active = false;
  }
  seek(time: number): ValidationFrame | null {
    if (!Number.isFinite(time)) throw new Error("Replay time must be finite");
    this.time = Math.max(
      this.frames[0]?.time ?? 0,
      Math.min(this.duration, time),
    );
    let lo = 0,
      hi = this.frames.length;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (this.frames[mid].time <= this.time) lo = mid + 1;
      else hi = mid;
    }
    this.cursor = Math.max(0, lo - 1);
    if (this.time >= this.duration) this.active = false;
    return this.frame;
  }
  seekIndex(index: number): ValidationFrame | null {
    if (!Number.isFinite(index)) throw new Error("Replay index must be finite");
    return this.seek(
      this.frames[
        Math.max(0, Math.min(this.frames.length - 1, Math.floor(index)))
      ]?.time ?? 0,
    );
  }
  advance(delta: number): ValidationFrame | null {
    if (!Number.isFinite(delta) || delta < 0)
      throw new Error("Playback delta must be finite and non-negative");
    return this.active ? this.seek(this.time + delta) : this.frame;
  }
}

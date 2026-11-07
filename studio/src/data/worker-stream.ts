import type {Snapshot} from '../core';

// One batch in flight; the render cache and this staging ring share the same
// retention bound. History lost to a stalled UI is explicit, never queued forever.
export const UPDATE_CAPACITY = 30000;
export class WorkerUpdateStream {
  private frames: Snapshot[] = [];
  private latest = new Map<string, object>();
  private reset = false;
  private inFlight = false;
  dropped = 0;
  coalesced = 0;

  constructor(private readonly send: (message: object) => void) {}

  push<T extends {type: string; frames?: Snapshot[]; reset?: boolean}>(message: T) {
    if (message.type === 'frames') {
      if (message.reset) {
        this.frames = [];
        this.reset = true;
      }
      this.frames.push(...message.frames!);
      if (this.frames.length > UPDATE_CAPACITY) {
        const excess = this.frames.length - UPDATE_CAPACITY;
        this.frames.splice(0, excess);
        this.dropped += excess;
      }
    } else {
      if (this.latest.has(message.type)) this.coalesced++;
      this.latest.set(message.type, message);
    }
  }

  ack() { this.inFlight = false; }

  flush() {
    if (this.inFlight || (!this.reset && !this.frames.length && !this.latest.size)) return;
    const messages = [...this.latest.values()];
    this.latest.clear();
    if (this.reset || this.frames.length) {
      messages.unshift({type: 'frames', frames: this.frames.splice(0, 2000), reset: this.reset});
      this.reset = false;
    }
    this.inFlight = true;
    this.send({type: 'updates', messages, dropped: this.dropped, coalesced: this.coalesced});
  }
}

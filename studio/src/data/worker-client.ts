import type {Snapshot} from '../core';
import type {PlaybackState, SystemState} from './types';
import type {SessionManifest} from './manifest';

export type WorkerMessage =
    {type: 'frames'; frames: Snapshot[]; reset: boolean} |
    {type: 'playback'; state: PlaybackState} |
    ({type: 'system'} & SystemState) |
    {type: 'manifest'; manifest: SessionManifest|null; error: string} |
    {type: 'export'; buffer: ArrayBuffer} |
    {type: 'updates'; messages: WorkerMessage[]; dropped: number; coalesced: number};

export class WorkerRpcError extends Error {
  constructor(public readonly code: 'crashed'|'messageerror'|'timeout'|'terminated', message: string) {
    super(message);
    this.name = 'WorkerRpcError';
  }
}

export type WorkerPort = Pick<Worker, 'postMessage'|'terminate'|'onmessage'|'onerror'|'onmessageerror'>;
export const RPC_TIMEOUT_MS = 10000;
export const WORKER_HEARTBEAT_MS = 5000;

export class WorkerClient {
  private worker: WorkerPort|null = null;
  private nextId = 1;
  private pending = new Map<number, {
    resolve(value: unknown): void; reject(error: WorkerRpcError): void;
    timer: ReturnType<typeof setTimeout>;
  }>();
  private heartbeat?: ReturnType<typeof setInterval>;
  private lastPong = Date.now();

  constructor(private readonly factory: () => WorkerPort,
              private readonly receive: (data: WorkerMessage) => void,
              private readonly failed: (error: WorkerRpcError) => void,
              private readonly timeoutMs = RPC_TIMEOUT_MS) {
    this.restart();
  }

  restart() {
    if (this.worker) return;
    try {
      const worker = this.factory();
      this.worker = worker;
      this.lastPong = Date.now();
      worker.onmessage = event => {
        if (worker !== this.worker) return;
        const data = event.data;
        if (data.type === 'pong') this.lastPong = Date.now();
        else if (data.type === 'rpc') {
          const call = this.pending.get(data.id);
          if (call) {
            clearTimeout(call.timer);
            this.pending.delete(data.id);
            call.resolve(data.response);
          }
        } else this.receive(data);
      };
      worker.onerror = event => {
        if (worker !== this.worker) return;
        event.preventDefault();
        this.fail(new WorkerRpcError('crashed', event.message || 'Telemetry worker crashed'));
      };
      worker.onmessageerror = () => {
        if (worker === this.worker)
          this.fail(new WorkerRpcError('messageerror', 'Telemetry worker message could not be decoded'));
      };
      this.heartbeat = setInterval(() => {
        if (Date.now() - this.lastPong >= WORKER_HEARTBEAT_MS * 2) {
          this.fail(new WorkerRpcError('terminated', 'Telemetry worker stopped responding; reconnect to restart'));
        } else this.post({type: 'ping'});
      }, WORKER_HEARTBEAT_MS);
    } catch (error) {
      this.fail(new WorkerRpcError('crashed', String(error)));
    }
  }

  fail(error: WorkerRpcError) {
    clearInterval(this.heartbeat);
    if (this.worker) {
      this.worker.onmessage = null;
      this.worker.onerror = null;
      this.worker.onmessageerror = null;
      this.worker.terminate();
    }
    this.worker = null;
    for (const call of this.pending.values()) {
      clearTimeout(call.timer);
      call.reject(error);
    }
    this.pending.clear();
    this.failed(error);
  }

  acknowledger() {
    const worker = this.worker;
    return () => {
      if (worker && worker === this.worker) this.post({type: 'ack'});
    };
  }

  post(data: object, transfer: Transferable[] = []) {
    if (!this.worker) throw new WorkerRpcError('terminated', 'Telemetry worker unavailable; reconnect to restart');
    try { this.worker.postMessage(data, transfer); }
    catch (error) { this.fail(new WorkerRpcError('messageerror', String(error))); }
  }

  rpc(method: string, params: unknown): Promise<unknown> {
    return new Promise((resolve, reject) => {
      if (!this.worker) {
        reject(new WorkerRpcError('terminated', 'Telemetry worker unavailable; reconnect to restart'));
        return;
      }
      const id = this.nextId++;
      const timer = setTimeout(() => {
        this.fail(new WorkerRpcError('timeout', `Telemetry worker RPC ${method} timed out`));
      }, this.timeoutMs);
      this.pending.set(id, {resolve, reject, timer});
      this.post({type: 'rpc', id, request: {jsonrpc: '2.0', id, method, params}});
    });
  }
}

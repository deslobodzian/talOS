// Live reloading of /drive/layout.json on WebSocket reconnect or
// mid-connection session/dictionary changes (REQ-OPS-34, REQ-OPS-36,
// REQ-OPS-39). No DOM and no timers of its own: tests pass a fake fetcher and
// injected timers.
//
// Fail-closed contract:
//   - If a reload fetch or layout validation fails, `error` holds a clear
//     reason and the previously loaded layout stays intact (never replaced by
//     null or partial state), so Painter keeps the top status line live,
//     displays LAYOUT ERROR and marks the grid errored until a reload succeeds.
//   - A monotonic request id (and AbortController) discards out-of-order
//     responses/errors from superseded fetches.
//   - On a WebSocket reconnect, any hung/in-flight fetch from an older
//     connection is aborted and a fresh reload starts immediately; on a
//     mid-connection session/dictionary change while a fetch is already in
//     flight, at most one coalesced follow-up fetch runs afterwards.
//   - Retries while the socket remains open follow RECONNECT_MS, backing off
//     to DECODE_RETRY_MS after DECODE_RETRIES consecutive failures, and stop
//     while disconnected or replaced (close 4000).

import {DECODE_RETRIES, DECODE_RETRY_MS, type LayoutReloadReason, RECONNECT_MS} from './feed';
import {type Layout, layoutsEqual, parseLayout} from './layout';

export type LayoutFetchResult =
    {ok: true; status: number; json(): Promise<unknown>}|
    {ok: false; status: number};

export type LayoutFetcher =
    (url: string, init: {cache: 'no-store'; signal?: AbortSignal}) => Promise<LayoutFetchResult>;

export interface LayoutReloaderTimers {
  setTimeout(fn: () => void, ms: number): number;
  clearTimeout(handle: number): void;
}

export interface LayoutReloaderCallbacks {
  isConnected(): boolean;
  onLayout(next: Layout, changed: boolean): void;
  onError(message: string): void;
}

export class LayoutReloadError extends Error {}

function errorText(error: unknown): string {
  return error instanceof Error && error.message ? error.message : String(error);
}

// Fetches and validates /drive/layout.json with cache: 'no-store'. Throws
// LayoutReloadError with a clear, operator-readable reason on any network,
// HTTP, JSON parse or layout validation failure.
export async function fetchAndParseLayout(
    fetcher: LayoutFetcher,
    url = '/drive/layout.json',
    signal?: AbortSignal): Promise<Layout> {
  let response: LayoutFetchResult;
  try {
    response = await fetcher(url, {cache: 'no-store', signal});
  } catch (error) {
    throw new LayoutReloadError(
        `Cannot reload dashboard layout: ${errorText(error)}; retrying`);
  }
  if (!response.ok) {
    throw new LayoutReloadError(
        `Cannot reload dashboard layout (HTTP ${response.status}); retrying`);
  }
  let json: unknown;
  try {
    json = await response.json();
  } catch (error) {
    throw new LayoutReloadError(
        `Dashboard layout is not valid JSON: ${errorText(error)}; retrying`);
  }
  try {
    return parseLayout(json);
  } catch (error) {
    throw new LayoutReloadError(
        `Invalid dashboard layout: ${errorText(error)}; retrying`);
  }
}

export class LayoutReloader {
  private current: Layout;
  private reloadError: string|null = null;
  private requestId = 0;
  private inFlight = false;
  private pending = false;
  private consecutiveFailures = 0;
  private retryHandle: number|null = null;
  private abort: AbortController|null = null;

  constructor(
      initialLayout: Layout,
      private readonly load: (signal?: AbortSignal) => Promise<Layout>,
      private readonly timers: LayoutReloaderTimers,
      private readonly callbacks: LayoutReloaderCallbacks) {
    this.current = initialLayout;
  }

  get layout(): Layout {
    return this.current;
  }

  get error(): string|null {
    return this.reloadError;
  }

  // Requests a layout reload. A `reconnect` aborts any in-flight fetch from an
  // older connection and fetches immediately; a mid-connection `session` or
  // `dictionary` trigger coalesces behind an in-flight fetch.
  trigger(reason: LayoutReloadReason) {
    this.clearRetry();
    if (this.inFlight) {
      if (reason === 'reconnect') {
        this.abortInFlight();
        this.pending = false;
      } else {
        this.pending = true;
        return;
      }
    }
    this.startFetch();
  }

  // Cancels any in-flight fetch and armed retry timer (called on socket close,
  // error, or close 4000 replacement). Preserves any existing `error` so a
  // previously failed reload stays fail-closed until a reload succeeds.
  cancelPending() {
    this.clearRetry();
    this.pending = false;
    this.abortInFlight();
  }

  private clearRetry() {
    if (this.retryHandle !== null) {
      this.timers.clearTimeout(this.retryHandle);
      this.retryHandle = null;
    }
  }

  private abortInFlight() {
    if (!this.inFlight) return;
    this.requestId++;
    this.inFlight = false;
    const controller = this.abort;
    this.abort = null;
    try {
      controller?.abort();
    } catch {
      // ignore abort errors
    }
  }

  private startFetch() {
    const id = ++this.requestId;
    this.inFlight = true;
    const controller = typeof AbortController === 'function' ? new AbortController() : null;
    this.abort = controller;
    void this.load(controller?.signal).then(
        next => {
          if (id !== this.requestId) return;
          this.inFlight = false;
          this.abort = null;
          const hadError = this.reloadError !== null;
          this.reloadError = null;
          this.consecutiveFailures = 0;
          const changed = !layoutsEqual(this.current, next);
          if (changed) this.current = next;
          if (changed || hadError) this.callbacks.onLayout(next, changed);
          if (this.pending) {
            this.pending = false;
            if (this.callbacks.isConnected()) this.startFetch();
          }
        },
        error => {
          if (id !== this.requestId) return;
          this.inFlight = false;
          this.abort = null;
          this.consecutiveFailures++;
          this.reloadError = errorText(error);
          this.callbacks.onError(this.reloadError);
          if (this.pending) {
            this.pending = false;
            if (this.callbacks.isConnected()) {
              this.startFetch();
              return;
            }
          }
          if (this.callbacks.isConnected()) {
            const delay = this.consecutiveFailures >= DECODE_RETRIES ?
                DECODE_RETRY_MS : RECONNECT_MS;
            this.retryHandle = this.timers.setTimeout(() => {
              this.retryHandle = null;
              if (this.callbacks.isConnected() && !this.inFlight) this.startFetch();
            }, delay);
          }
        });
  }
}

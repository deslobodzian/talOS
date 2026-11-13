// The driver dashboard's connection to the bridge's /drive feed
// (studio/bridge/README.md "Driver dashboard feed"), as a model with no DOM
// and no timers of its own: the page calls tick(now) on an interval and
// passes `now` (milliseconds, any monotonic origin) to every handler, so
// tests drive it with an injected clock and a fake socket.
//
// It holds (REQ-OPS-36):
//   - One socket at a time. On any close or error, except close 4000
//     (`replaced by a newer driver dashboard`), the next attempt starts
//     RECONNECT_MS after it. After 4000 it stays closed, reports `replaced`,
//     and connects again only on reconnect(), a user action: a page that
//     reconnected by itself would take the slot back from the newer page.
//     A connect that has not opened after CONNECT_TIMEOUT_MS, or an open
//     socket silent for DEAD_AFTER_MS, is closed and counts as an error, so
//     a dead network path does not wait for the OS to time the socket out.
//   - Freshness. The telemetry is current while a tick newer than the one
//     held (a sequence_id that advanced, or a new stream) arrived within
//     STALE_MS on this socket, after the first tick it brought. That first
//     tick is the bridge's connect-time keyframe, which repeats the newest
//     tick the bridge holds however old (the producer may have died minutes
//     ago), so it sets the values, shown stale, and never makes them current.
//     The bridge's empty update every 100 ms re-sends the held tick when
//     nothing changed, so a sequence_id that stops means the producer
//     stopped. With the page ticking every RENDER_MS, a value is shown stale
//     at most STALE_MS + RENDER_MS = 400 ms after its last update, inside
//     REQ-OPS-36's 500 ms.
//   - The failed link, upstream first: dashboard to bridge (no socket, or no
//     message, heartbeat included, for SILENCE_MS: the bridge sends one at
//     least every 100 ms), then telemetry (messages arrive but no new tick),
//     then the RoboRIO link (`hw/status/alert_error_link_lost`). Right after
//     a socket opens the bridge may still be refilling the slot's bucket
//     before the dictionary (README: at most about 0.8 s on the 2026 config):
//     for CONNECTING_GRACE_MS that is `connecting`, not a lost link. After
//     the keyframe, a live producer's next tick follows within the update
//     interval or the 100 ms heartbeat; until STALE_MS after the keyframe
//     that is still `connecting`, then stale telemetry.
//   - Robustness. A binary or text message that does not decode is counted
//     and ignored; nothing it carries reaches the views.

import {type Snapshot, TelemetryDecoder} from '../core';
import {AlertTracker} from '../data/alerts';

export const RECONNECT_MS = 1000;
export const STALE_MS = 300;
export const RENDER_MS = 100;
export const SILENCE_MS = 500;
export const CONNECTING_GRACE_MS = 1500;
export const CONNECT_TIMEOUT_MS = 2000;
export const DEAD_AFTER_MS = 2000;
export const REPLACED_CODE = 4000;
export const LINK_LOST_CHANNEL = 'hw/status/alert_error_link_lost';
// The longest history a graph or a field trail can ask for (README: 30 s),
// and a cap on points per channel whatever the update rate.
const HISTORY_S = 30;
const HISTORY_POINTS = 4096;

// The part of a browser WebSocket the model uses; tests pass a fake.
export interface DriveSocket {
  binaryType: string;
  onopen: (() => void)|null;
  onmessage: ((event: {data: unknown}) => void)|null;
  onclose: ((event: {code: number; reason: string}) => void)|null;
  onerror: (() => void)|null;
  close(): void;
}
export type SocketFactory = (url: string) => DriveSocket;

export type SummaryNode = {
  name: string;
  alive: boolean;
  critical: boolean;
  restarts: number;
  quarantined: boolean;
  drives: string|null;
};
// talos.drive.summary (README): `registry` false means `alive` and `drives`
// are unknown, `supervised` false that `restarts` and `quarantined` are.
export type DriveSummary = {
  session: string;
  supervised: boolean;
  registry: boolean;
  nodes: SummaryNode[];
  omitted: number;
};

export type Connection = 'connecting'|'open'|'waiting'|'replaced';
export type Link = 'ok'|'connecting'|'bridge'|'telemetry'|'roborio'|'replaced';
export type LinkStatus = {link: Link; text: string};

export type Point = {t_s: number; v: number};

export type Counters = {
  connects: number;
  malformed_binary: number;
  malformed_text: number;
  ignored_text: number;
};

const seconds = (ms: number) => (ms / 1000).toFixed(1) + ' s';

export function parseSummary(v: unknown): DriveSummary|null {
  if (typeof v !== 'object' || v === null) return null;
  const s = v as Record<string, unknown>;
  if (typeof s.registry !== 'boolean' || typeof s.supervised !== 'boolean' ||
      !Array.isArray(s.nodes))
    return null;
  const nodes: SummaryNode[] = [];
  for (const n of s.nodes) {
    if (typeof n !== 'object' || n === null) return null;
    const node = n as Record<string, unknown>;
    if (typeof node.name !== 'string') return null;
    nodes.push({
      name: node.name,
      alive: node.alive === true,
      critical: node.critical !== false,
      restarts: typeof node.restarts === 'number' && Number.isFinite(node.restarts) ?
          node.restarts : 0,
      quarantined: node.quarantined === true,
      drives: typeof node.drives === 'string' ? node.drives : null
    });
  }
  return {
    session: typeof s.session === 'string' ? s.session : '',
    supervised: s.supervised,
    registry: s.registry,
    nodes,
    omitted: typeof s.omitted === 'number' && s.omitted > 0 ? s.omitted : 0
  };
}

function bytesOf(data: unknown): Uint8Array|null {
  if (data instanceof ArrayBuffer) return new Uint8Array(data);
  if (ArrayBuffer.isView(data))
    return new Uint8Array(data.buffer, data.byteOffset, data.byteLength);
  return null;
}

export class DriveModel {
  connection: Connection = 'waiting';
  // The newest tick held, kept across reconnects so the page shows the last
  // values (stale) rather than nothing.
  latest: Snapshot|null = null;
  summary: DriveSummary|null = null;
  readonly alerts = new AlertTracker();
  readonly counters: Counters =
      {connects: 0, malformed_binary: 0, malformed_text: 0, ignored_text: 0};

  private socket: DriveSocket|null = null;
  private decoder = new TelemetryDecoder();
  private everOpened = false;
  private attemptAt = 0;     // when the current socket was created
  private openedAt = 0;      // when it opened
  private nextAttemptAt = 0; // while waiting
  private lastMessageAt = 0; // any message on the current socket
  // A tick newer than the held one, on this socket after its keyframe.
  private lastTickAt: number|null = null;
  private summaryAt: number|null = null;
  private tickThisSocket = false;  // the connect-time keyframe arrived
  private keyframeAt = 0;
  private readonly tracked: Set<string>;
  private readonly histories = new Map<string, Point[]>();

  // `tracked`: the channels whose history a graph or a field trail draws.
  constructor(private readonly url: string,
              private readonly openSocket: SocketFactory,
              tracked: Iterable<string> = []) {
    this.tracked = new Set(tracked);
  }

  // Opens the first socket.
  start(now: number) {
    if (this.connection === 'waiting' && !this.socket) this.connect(now);
  }

  // A user action (the "reconnect" button): the only way out of `replaced`.
  reconnect(now: number) {
    if (this.connection === 'replaced' || this.connection === 'waiting')
      this.connect(now);
  }

  tick(now: number) {
    if (this.connection === 'waiting' && now >= this.nextAttemptAt) {
      this.connect(now);
    } else if (this.connection === 'connecting' &&
               now - this.attemptAt >= CONNECT_TIMEOUT_MS) {
      this.fail(now);
    } else if (this.connection === 'open' &&
               now - this.lastMessageAt >= DEAD_AFTER_MS) {
      this.fail(now);
    }
  }

  // Whether the values held are current: a new tick within STALE_MS on an
  // open socket.
  fresh(now: number): boolean {
    return this.connection === 'open' && this.lastTickAt !== null &&
        now - this.lastTickAt <= STALE_MS &&
        now - this.lastMessageAt <= SILENCE_MS;
  }

  // Whether the summary was sent on this socket, which is still heard.
  summaryFresh(now: number): boolean {
    return this.connection === 'open' && this.summaryAt !== null &&
        this.summaryAt >= this.openedAt && now - this.lastMessageAt <= SILENCE_MS;
  }

  history(channel: string): readonly Point[] {
    return this.histories.get(channel) ?? [];
  }

  // The one status line: which link failed, upstream first.
  status(now: number): LinkStatus {
    switch (this.connection) {
      case 'replaced':
        return {link: 'replaced',
                text: 'Replaced by another dashboard: press Reconnect to take the slot back'};
      case 'waiting':
        return {link: 'bridge', text: 'Dashboard to bridge link down: reconnecting'};
      case 'connecting':
        if (!this.everOpened && now - this.attemptAt < CONNECTING_GRACE_MS)
          return {link: 'connecting', text: 'Connecting to the robot'};
        return {link: 'bridge', text: 'Dashboard to bridge link down: reconnecting'};
      case 'open':
        break;
    }
    if (now - this.lastMessageAt > SILENCE_MS)
      return {link: 'bridge', text: 'Dashboard to bridge link: nothing heard for ' +
                                    seconds(now - this.lastMessageAt)};
    if (!this.tickThisSocket) {
      if (now - this.openedAt <= CONNECTING_GRACE_MS)
        return {link: 'connecting', text: 'Connected, waiting for telemetry'};
      return {link: 'telemetry',
              text: 'Telemetry stale: bridge connected, no telemetry for ' +
                  seconds(now - this.openedAt)};
    }
    if (this.lastTickAt === null) {
      if (now - this.keyframeAt <= STALE_MS)
        return {link: 'connecting', text: 'Connected, waiting for a new tick'};
      return {link: 'telemetry', text: 'Telemetry stale: no new tick in ' +
                                       seconds(now - this.keyframeAt) + ' since connecting'};
    }
    if (now - this.lastTickAt > STALE_MS)
      return {link: 'telemetry',
              text: 'Telemetry stale: no new tick for ' + seconds(now - this.lastTickAt)};
    const s = this.latest;
    if (s?.alerts?.[LINK_LOST_CHANNEL] !== undefined) {
      const lost = s.channels[LINK_LOST_CHANNEL];
      if (lost === undefined)
        return {link: 'roborio', text: 'RoboRIO link unknown: no hardware status'};
      if (lost !== 0) return {link: 'roborio', text: 'RoboRIO link lost'};
    }
    return {link: 'ok', text: 'Connected: telemetry current'};
  }

  private connect(now: number) {
    this.detach();
    this.decoder.reset();
    this.tickThisSocket = false;
    this.lastTickAt = null;
    this.connection = 'connecting';
    this.attemptAt = now;
    this.counters.connects++;
    let socket: DriveSocket;
    try {
      socket = this.openSocket(this.url);
    } catch {
      this.wait(now);
      return;
    }
    this.socket = socket;
    socket.binaryType = 'arraybuffer';
    // The handlers take their time from the clock the page installs; a
    // fake socket in a test calls them through deliver*().
    socket.onopen = () => this.onOpen(this.clock());
    socket.onmessage = event => this.onMessage(event.data, this.clock());
    socket.onclose = event => this.onClose(event.code, this.clock());
    socket.onerror = () => this.fail(this.clock());
  }

  // The time handlers use; the page sets it to performance.now, a test to
  // its own clock.
  clock: () => number = () => 0;

  private onOpen(now: number) {
    this.connection = 'open';
    this.everOpened = true;
    this.openedAt = now;
    this.lastMessageAt = now;
  }

  private onMessage(data: unknown, now: number) {
    if (this.connection !== 'open') return;
    this.lastMessageAt = now;
    if (typeof data === 'string') {
      this.onText(data, now);
      return;
    }
    const bytes = bytesOf(data);
    if (!bytes) {
      this.counters.malformed_binary++;
      return;
    }
    let snapshots: Snapshot[];
    try {
      snapshots = this.decoder.decodeAll(bytes);
    } catch {
      this.counters.malformed_binary++;
      return;
    }
    for (const s of snapshots) this.onSnapshot(s, now);
  }

  private onText(text: string, now: number) {
    let message: unknown;
    try {
      message = JSON.parse(text);
    } catch {
      this.counters.malformed_text++;
      return;
    }
    const kind = typeof message === 'object' && message !== null ?
        (message as {kind?: unknown}).kind : undefined;
    if (kind === 'talos.drive.heartbeat') return;
    if (kind === 'talos.drive.summary') {
      const summary = parseSummary(message);
      if (!summary) {
        this.counters.malformed_text++;
        return;
      }
      this.summary = summary;
      this.summaryAt = now;
      return;
    }
    if (typeof kind === 'string') this.counters.ignored_text++;
    else this.counters.malformed_text++;
  }

  private onSnapshot(s: Snapshot, now: number) {
    const held = this.latest;
    const newer = held === null || s.stream_id !== held.stream_id ||
        BigInt(s.sequence_id) > BigInt(held.sequence_id);
    const keyframe = !this.tickThisSocket;
    this.latest = s;
    if (keyframe) {
      this.tickThisSocket = true;
      this.keyframeAt = now;
    }
    if (!newer) return;
    this.alerts.push(s);
    this.record(s);
    // The connect-time keyframe is as old as the bridge's newest tick, which
    // the page cannot tell: only a tick after it on this socket is current.
    if (!keyframe) this.lastTickAt = now;
  }

  private record(s: Snapshot) {
    const t_s = Number(BigInt(s.timestamp_ns) / 1000n) / 1e6;
    for (const channel of this.tracked) {
      let points = this.histories.get(channel);
      if (!points) this.histories.set(channel, points = []);
      // A stream restart can move the clock back: start the trace over.
      if (points.length && t_s < points[points.length - 1].t_s) points.length = 0;
      points.push({t_s, v: s.channels[channel] ?? NaN});
      let drop = 0;
      while (drop < points.length && t_s - points[drop].t_s > HISTORY_S) drop++;
      drop = Math.max(drop, points.length - HISTORY_POINTS);
      if (drop > 0) points.splice(0, drop);
    }
  }

  private onClose(code: number, now: number) {
    this.detach();
    if (code === REPLACED_CODE) {
      this.connection = 'replaced';
      return;
    }
    this.wait(now);
  }

  private fail(now: number) {
    this.detach();
    this.wait(now);
  }

  private wait(now: number) {
    this.connection = 'waiting';
    this.nextAttemptAt = now + RECONNECT_MS;
  }

  // Drops the current socket, so nothing it does later reaches the model and
  // at most one socket is ever open.
  private detach() {
    const socket = this.socket;
    if (!socket) return;
    this.socket = null;
    socket.onopen = socket.onmessage = socket.onclose = socket.onerror = null;
    try {
      socket.close();
    } catch {
      // already closed
    }
  }
}

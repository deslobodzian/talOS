/// <reference lib="webworker" />
import {Coordinator, decode, JitterBuffer, Timeline} from './core';
import {demoPacket} from './demo';

const ctx = self as unknown as DedicatedWorkerGlobalScope;
let coordinator = new Coordinator(), jitter = new JitterBuffer(10);
let socket: WebSocket|null = null, demo = false, sequence = 0n,
            status = 'Disconnected';
let packets = new Map<string, Uint8Array>(),
    pendingBytes = new Map<string, Uint8Array>();
let nextDemo = 0, lastTick = performance.now(), lastView = 0;
let retry: ReturnType<typeof setTimeout>|undefined;
let generation = 0;
function reset() {
  coordinator = new Coordinator();
  jitter = new JitterBuffer(10);
  packets.clear();
  pendingBytes.clear();
}
function close() {
  generation++;
  clearTimeout(retry);
  if (socket) {
    socket.onclose = null;
    socket.close();
    socket = null;
  }
  demo = false;
}
function ingest(bytes: Uint8Array, now: number) {
  try {
    const frame = decode(bytes);
    jitter.push(frame, now);
    pendingBytes.set(frame.sequence_id, bytes);
    if (pendingBytes.size > 4096)
      pendingBytes.delete(pendingBytes.keys().next().value!);
  } catch {
    jitter.stats.invalid++;
  }
}
function connect(url: string, gen: number) {
  try {
    const u = new URL(url);
    if (!['ws:', 'wss:'].includes(u.protocol))
      throw Error('Use ws:// or wss://');
    socket = new WebSocket(u);
    socket.binaryType = 'arraybuffer';
    status = 'Connecting';
    socket.onopen = () => {
      status = 'WebSocket connected';
    };
    socket.onmessage = e => {
      if (e.data instanceof ArrayBuffer)
        ingest(new Uint8Array(e.data), performance.now());
    };
    socket.onclose = () => {
      status = 'Disconnected · retrying';
      if (gen === generation) retry = setTimeout(() => connect(url, gen), 1000);
    };
    socket.onerror = () => {
      status = 'WebSocket connection failed';
    };
  } catch (e) {
    status = String(e);
  }
}
ctx.onmessage = ({data}) => {
  try {
    switch (data.type) {
      case 'connect':
        close();
        reset();
        connect(data.url, generation);
        break;
      case 'demo':
        close();
        reset();
        demo = true;
        sequence = 0n;
        nextDemo = performance.now();
        status = 'Deterministic demo · 50 Hz';
        break;
      case 'packet':
        if (demo || socket) {
          close();
          reset();
        }
        status = 'Desktop UDP';
        ingest(new Uint8Array(data.buffer), performance.now());
        break;
      case 'mode':
        if (['PAUSED', 'LIVE_STREAMING', 'REPLAY_PLAYING'].includes(data.mode))
          coordinator.mode = data.mode;
        break;
      case 'seek':
        coordinator.seek(BigInt(data.timestamp_ns));
        break;
      case 'step':
        coordinator.step(BigInt(data.dt_ns));
        break;
      case 'export': {
        const chunks =
            coordinator.timeline.frames.map(f => packets.get(f.sequence_id))
                .filter((x): x is Uint8Array => !!x);
        const out = new Uint8Array(chunks.reduce((n, c) => n + c.length, 0));
        let p = 0;
        for (const c of chunks) {
          out.set(c, p);
          p += c.length;
        }
        ctx.postMessage({type: 'export', buffer: out.buffer}, [out.buffer]);
        break;
      }
      case 'import': {
        const bytes = new Uint8Array(data.buffer);
        if (bytes.length > 256 * 1024 * 1024)
          throw Error('Log exceeds 256 MiB');
        const imported = new Timeline();
        const raw = new Map<string, Uint8Array>();
        let p = 0;
        while (p < bytes.length) {
          if (p + 4 > bytes.length) throw Error('Truncated log');
          const size = new DataView(bytes.buffer, bytes.byteOffset + p, 4)
                           .getUint32(0, true) +
              4;
          if (size < 16 || p + size > bytes.length)
            throw Error('Truncated log frame');
          const chunk = bytes.slice(p, p + size);
          const f = decode(chunk);
          if (!imported.append(f))
            throw Error('Log sequence/timestamps must strictly increase');
          raw.set(f.sequence_id, chunk);
          p += size;
        }
        // Commit only after the entire log validates. Deterministic import
        // ignores arrival time.
        close();
        reset();
        coordinator = new Coordinator(imported);
        packets = raw;
        coordinator.seek(BigInt(imported.bounds()?.start ?? '0'));
        status = 'Replay log loaded';
        break;
      }
    }
  } catch (e) {
    status = String(e);
  }
};
setInterval(() => {
  const now = performance.now();
  if (demo) {
    let n = 0;
    while (now >= nextDemo && n++ < 10) {
      ingest(demoPacket(sequence++), now);
      nextDemo += 20;
    }
  }
  for (const f of jitter.flush(now)) {
    if (!coordinator.timeline.append(f)) {
      jitter.stats.invalid++;
      continue;
    }
    const raw = pendingBytes.get(f.sequence_id);
    if (raw) packets.set(f.sequence_id, raw);
    pendingBytes.delete(f.sequence_id);
  }
  const first = coordinator.timeline.frames[0];
  if (first) {
    for (const key of packets.keys()) {
      if (BigInt(key) < BigInt(first.sequence_id))
        packets.delete(key);
      else
        break;
    }
  }
  coordinator.tick(BigInt(Math.max(0, Math.round((now - lastTick) * 1e6))));
  lastTick = now;
  if (now - lastView >= 1000 / 30) {
    lastView = now;
    const snapshot = coordinator.snapshot();
    const t = BigInt(snapshot?.timestamp_ns ?? '0');
    const frames = coordinator.timeline.frames;
    let end = frames.length;
    while (end > 0 && BigInt(frames[end - 1].timestamp_ns) > t) end--;
    ctx.postMessage({
      type: 'view',
      snapshot,
      history: frames.slice(Math.max(0, end - 600), end),
      mode: coordinator.mode,
      bounds: coordinator.timeline.bounds(),
      stats: jitter.stats,
      status
    });
  }
}, 4);

// Built-in event detectors. Every one is edge-triggered: it compares a frame
// against the previous frame of the same stream (or a graph against the
// previous graph) and reports only a change, never a steady state. Nothing is
// rate-limited silently -- a detector that could fire on every frame is kept
// edge-triggered so it only reports transitions.
//
// Detectors are generic by channel-name pattern, so Studio stays
// robot-agnostic: no 2026 node names appear here. The reserved `hw` owner is
// part of the telemetry schema, not a robot name.

import type {Snapshot} from '../core';
import type {SystemGraph, SystemSelection} from '../system';
import type {
  EventCategory,
  EventDraft,
  FrameDetector,
  Severity,
  SystemDetector
} from './types';
import type {SessionManifest} from './manifest';
import {driverStationMode} from './robot';

const DS = 'hw/state/driver_station/';

const val = (s: Snapshot|null, name: string): number|undefined => s?.channels[name];

function enumLabel(s: Snapshot|null, name: string, value: number): string {
  const labels = s?.labels[name];
  if (labels && Number.isInteger(value) && value >= 0 && value < labels.length)
    return labels[value];
  return String(value);
}

// A set bit list for a fault/status mask, "cleared" when the mask is zero.
function maskDetail(value: number): string {
  const bits: number[] = [];
  let v = Math.trunc(value);
  for (let i = 0; i < 53 && v !== 0; i++) {
    if (v & 1) bits.push(i);
    v = Math.floor(v / 2);
  }
  return bits.length ? `bits: ${bits.join(', ')}` : 'cleared';
}

function categoryFor(name: string): EventCategory {
  if (name.startsWith(DS)) return 'driver_station';
  if (name.startsWith('hw/status/')) {
    const rest = name.slice('hw/status/'.length);
    if (rest.startsWith('battery') || rest.startsWith('brownout') || rest.startsWith('can_'))
      return 'power';
    return 'hardware';
  }
  if (name.startsWith('hw/')) return 'hardware';
  return 'mechanism';
}

// A boolean edge on one channel, both directions.
function boolEdge(id: string, channel: string, category: EventCategory,
                  onTitle: string, offTitle: string, onSeverity: Severity,
                  offSeverity: Severity): FrameDetector {
  return {
    id,
    detect(prev, next) {
      const a = val(prev, channel), b = val(next, channel);
      if (a === undefined || b === undefined || a === b) return [];
      if (b === 1) return [{severity: onSeverity, category, title: onTitle, channel}];
      return [{severity: offSeverity, category, title: offTitle, channel}];
    }
  };
}

// A counter detector that reports RUNS rather than increments: a steady
// increase is one event when it starts and one when it has stopped advancing
// for a second, never one per increment. A counter that ticks at 80 Hz must
// not flood the event log while a robot sits disabled refusing commands.
const STOP_AFTER_NS = 1_000_000_000n;
const CLASSIFY_AFTER_NS = 500_000_000n;

type Run = {
  severity: Severity;
  startValue: number;
  startTime_ns: bigint;
  lastValue: number;
  lastAdvance_ns: bigint;
  // The start event has not been emitted yet: the detector is still observing
  // the run to classify it (see `defer`/`classifyStart`).
  pending: boolean;
  // Every frame since the run began advanced the counter; a steady frame
  // breaks the streak, so a deferred run only classifies "warn" while this
  // stays true.
  stillIncreasing: boolean;
  enabledSince_ns: bigint|null;
  sawKnownMode: boolean;
};

// An event draft carrying its own stamp. A deferred start event is classified
// after the run begins but belongs at the run's true start time, so the engine
// stamps it there rather than at the frame that decided it.
export type StampedDraft = EventDraft & {t0_ns?: string};

type IncrementOptions = {
  id: string;
  match: (name: string) => boolean;
  severity: Severity;
  category: EventCategory;
  // Matched channels whose start event is deferred until the run is
  // classified. Used by the gateway's rejected-command counter: a disable
  // transition refuses commands in flight, and the run's first frame still
  // reads an enabled Driver Station mode.
  defer?: (name: string) => boolean;
  // Decides the start event from the run observed so far. Returns a stamped
  // draft once the run is classifiable, or null to keep observing.
  classifyStart?: (name: string, run: Run, frame: Snapshot) => StampedDraft|null;
};

function incrementDetector(options: IncrementOptions): FrameDetector & {reset(): void} {
  const runs = new Map<string, Run>();
  // A run ending is news, not a fault: the run's start event already carried
  // its severity.
  const stopDraft = (name: string, run: Run, ts: bigint): EventDraft => ({
    severity: 'info',
    category: options.category,
    title: `${name} stopped increasing`,
    detail: `+${run.lastValue - run.startValue} over ${
        (Number(ts - run.startTime_ns) / 1e9).toFixed(1)} s`,
    channel: name
  });
  return {
    id: options.id,
    detect(prev, next) {
      const drafts: StampedDraft[] = [];
      const ts = BigInt(next.timestamp_ns);
      for (const name of Object.keys(next.channels)) {
        if (!options.match(name)) continue;
        const value = next.channels[name];
        const previous = prev?.channels[name];
        let run = runs.get(name);
        if (!run) {
          if (previous === undefined || value <= previous) continue;
          run = {
            severity: options.severity,
            startValue: previous,
            startTime_ns: ts,
            lastValue: value,
            lastAdvance_ns: ts,
            pending: options.defer?.(name) ?? false,
            stillIncreasing: true,
            enabledSince_ns: ['autonomous', 'teleop', 'test'].includes(
                driverStationMode(next) ?? '') ? ts : null,
            sawKnownMode: driverStationMode(next) !== null
          };
          runs.set(name, run);
          if (!run.pending)
            drafts.push({
              severity: run.severity,
              category: options.category,
              title: `${name} increasing`,
              channel: name
            });
          continue;
        }

        const advanced = value > run.lastValue;
        if (advanced) {
          run.lastValue = value;
          run.lastAdvance_ns = ts;
        } else {
          run.stillIncreasing = false;
          if (value < run.lastValue) {
            run.lastValue = value;
            run.lastAdvance_ns = ts;
          }
        }

        if (run.pending) {
          const classified = options.classifyStart?.(name, run, next) ?? null;
          if (classified) {
            drafts.push(classified);
            run.pending = false;
            run.severity = classified.severity;
          } else if (!advanced && ts - run.lastAdvance_ns >= STOP_AFTER_NS) {
            // A pending run never accumulated enough observed enabled time
            // to justify a warning; unknown state is not evidence of a fault.
            drafts.push({
              severity: 'info',
              category: options.category,
              title: `${name} increasing`,
              detail: run.sawKnownMode ? 'insufficient continuously enabled observations' :
                  'the robot state was unknown throughout the run',
              channel: name,
              t0_ns: run.startTime_ns.toString()
            });
            drafts.push(stopDraft(name, run, ts));
            runs.delete(name);
            continue;
          }
        }

        if (!run.pending && !advanced && ts - run.lastAdvance_ns >= STOP_AFTER_NS) {
          drafts.push(stopDraft(name, run, ts));
          runs.delete(name);
        }
      }
      return drafts;
    },
    reset() {
      runs.clear();
    }
  };
}

// Fresh detector instances per engine, so the stateful counter detectors do
// not leak run state between a store and a test, or between two stores.
export function createFrameDetectors(): FrameDetector[] {
  return [
  // The one Driver Station detector. Every DS state change is a transition of
  // driverStationMode, so a single detector reports disconnected, e-stop,
  // disabled and enabled without duplicating a transition across four.
  {
    id: 'ds-mode',
    detect(prev, next) {
      if (prev === null) return [];  // no previous mode at a stream start
      const a = driverStationMode(prev), b = driverStationMode(next);
      if (a === b || a === null || b === null) return [];
      const draft: EventDraft = {
        severity: 'info',
        category: 'driver_station',
        title: b === 'disconnected' ? 'Driver Station disconnected' :
               b === 'estop'         ? 'E-stop' :
               b === 'disabled'      ? 'Disabled' :
                                       `Enabled · ${b}`
      };
      if (b === 'disconnected') draft.severity = 'warn';
      else if (b === 'estop') draft.severity = 'error';
      if (a === 'estop' || a === 'disconnected') draft.detail = `from ${a}`;
      return [draft];
    }
  },
  // FMS attach/detach.
  boolEdge('ds-fms', DS + 'fms_attached', 'driver_station',
           'FMS attached', 'FMS detached', 'info', 'info'),
  // Brownout: start is an error, recovery is info.
  boolEdge('power-brownout', 'hw/status/brownout', 'power',
           'Brownout', 'Brownout cleared', 'error', 'info'),
  // Link configuration lost/regained.
  boolEdge('link-configured', 'hw/status/configured', 'hardware',
           'Link configured', 'Link not configured', 'info', 'warn'),
  // Motor faults distinguish newly set bits from resolved bits.
  {
    id: 'link-fault-mask',
    detect(prev, next) {
      const a = val(prev, 'hw/status/motor_fault_mask'),
            b = val(next, 'hw/status/motor_fault_mask');
      if (a === undefined || b === undefined || a === b) return [];
      const before = BigInt(Math.trunc(a)), after = BigInt(Math.trunc(b));
      const added = after & ~before, cleared = before & ~after;
      return [
        ...(added ? [{
          severity: 'warn' as const, category: 'hardware' as const,
          title: 'Motor fault bits set', detail: maskDetail(Number(added)),
          channel: 'hw/status/motor_fault_mask'
        }] : []),
        ...(cleared ? [{
          severity: 'info' as const, category: 'hardware' as const,
          title: 'Motor fault bits resolved', detail: maskDetail(Number(cleared)),
          channel: 'hw/status/motor_fault_mask'
        }] : [])
      ];
    }
  },
  {
    id: 'link-timeout-mask',
    detect(prev, next) {
      const a = val(prev, 'hw/status/timed_out_mask'),
            b = val(next, 'hw/status/timed_out_mask');
      if (a === undefined || b === undefined || a === b) return [];
      return [{
        severity: 'warn',
        category: 'hardware',
        title: 'Timed-out mask changed',
        detail: maskDetail(b),
        channel: 'hw/status/timed_out_mask'
      }];
    }
  },
  // Link counters, reported as runs: gateway counts and identity rejections
  // (warn: an obsolete request from before a reconfiguration or epoch change
  // is dropped by design), ownership rejections (error: a subsystem
  // addressed a device it does not own), clock resets (info). `gateway_rejected_command_count`
  // is deferred and classified: while the robot is not enabled the gateway
  // refuses commands as designed, so the run is only a warning when the Driver
  // Station stays enabled for half a second of continuous increments.
  incrementDetector({
    id: 'counter-gateway',
    match: name => /^hw\/status\/gateway_[^/]*_count$/.test(name),
    severity: 'warn',
    category: 'hardware',
    defer: name => name === 'hw/status/gateway_rejected_command_count',
    classifyStart(name, run, frame) {
      const mode = driverStationMode(frame);
      if (mode === null) {
        run.enabledSince_ns = null;
        return null;
      }
      run.sawKnownMode = true;
      const enabled =
          mode === 'autonomous' || mode === 'teleop' || mode === 'test';
      if (!enabled) {
        return {
          severity: 'info',
          category: 'hardware',
          title: 'Commands refused while not enabled',
          detail: 'the gateway is refusing commands while the robot is not enabled',
          t0_ns: run.startTime_ns.toString()
        };
      }
      if (run.enabledSince_ns === null)
        run.enabledSince_ns = BigInt(frame.timestamp_ns);
      if (run.stillIncreasing &&
          BigInt(frame.timestamp_ns) - run.enabledSince_ns >= CLASSIFY_AFTER_NS) {
        return {
          severity: 'warn',
          category: 'hardware',
          title: `${name} increasing`,
          t0_ns: run.startTime_ns.toString()
        };
      }
      return null;
    }
  }),
  incrementDetector({
    id: 'counter-identity',
    match: name => /(^|\/)identity_rejected_count$/.test(name),
    severity: 'warn',
    category: 'hardware'
  }),
  incrementDetector({
    id: 'counter-ownership',
    match: name => /(^|\/)ownership_rejected_count$/.test(name),
    severity: 'error',
    category: 'hardware'
  }),
  incrementDetector({
    id: 'counter-clock-resets',
    match: name => /(^|\/)clock_resets_count$/.test(name),
    severity: 'info',
    category: 'hardware'
  }),
  // Any `*/status/healthy` edge: warn when it turns false.
  {
    id: 'health-edge',
    detect(prev, next) {
      const drafts: EventDraft[] = [];
      for (const name of Object.keys(next.channels)) {
        if (!/\/status\/healthy$/.test(name)) continue;
        const a = val(prev, name), b = val(next, name);
        if (a === undefined || b === undefined || a === b || b !== 0) continue;
        drafts.push({severity: 'warn', category: categoryFor(name), title: `${name} unhealthy`, channel: name});
      }
      return drafts;
    }
  },
  // Any estimator's `*/state/valid` edge: warn when it turns false.
  {
    id: 'estimator-valid-edge',
    detect(prev, next) {
      const drafts: EventDraft[] = [];
      for (const name of Object.keys(next.channels)) {
        const m = /^(.+)\/state\/valid$/.exec(name);
        if (!m) continue;
        const owner = m[1];
        if (!Object.hasOwn(next.channels, `${owner}/state/observation_accepted_count`))
          continue;
        const a = val(prev, name), b = val(next, name);
        if (a === undefined || b === undefined || a === b || b !== 0) continue;
        drafts.push({severity: 'warn', category: 'estimator', title: `${owner} invalid`, channel: name});
      }
      return drafts;
    }
  },
  // Any enum-labelled channel changing value, with the old and new labels.
  {
    id: 'enum-change',
    detect(prev, next) {
      const drafts: EventDraft[] = [];
      const seen = new Set<string>();
      for (const s of [prev, next]) {
        if (!s) continue;
        for (const name of Object.keys(s.labels)) seen.add(name);
      }
      for (const name of seen) {
        const a = val(prev, name), b = val(next, name);
        if (a === undefined || b === undefined || a === b) continue;
        drafts.push({
          severity: 'info',
          category: categoryFor(name),
          title: `${name} changed`,
          detail: `${enumLabel(prev, name, a)} → ${enumLabel(next, name, b)}`,
          channel: name
        });
      }
      return drafts;
    }
  }
  ];
}

export const FRAME_DETECTORS: FrameDetector[] = createFrameDetectors();

function selectionKey(selection: SystemSelection): string {
  return [...selection.sources].sort().join('\u0000');
}

export const SYSTEM_DETECTORS: SystemDetector[] = [
  // Node liveness: alive -> dead is an error, dead -> alive is info.
  {
    id: 'node-alive',
    detect(prev, next) {
      const drafts: EventDraft[] = [];
      for (const node of next.nodes) {
        const before = prev?.nodes.find(n => n.name === node.name);
        if (!before || before.alive === node.alive) continue;
        drafts.push(node.alive ? {
          severity: 'info', category: 'system', title: `${node.name} alive`
        } : {
          severity: 'error', category: 'system', title: `${node.name} stopped responding`
        });
      }
      return drafts;
    }
  },
  // pid or generation change: the node was replaced.
  {
    id: 'node-restarted',
    detect(prev, next) {
      const drafts: EventDraft[] = [];
      for (const node of next.nodes) {
        const before = prev?.nodes.find(n => n.name === node.name);
        if (!before) continue;
        if (before.pid !== node.pid || before.generation !== node.generation) {
          drafts.push({
            severity: 'warn',
            category: 'system',
            title: `${node.name} restarted`,
            detail: `pid ${before.pid} → ${node.pid}`
          });
        }
      }
      return drafts;
    }
  },
  // Target selection change: who drives a subsystem.
  {
    id: 'target-selection',
    detect(prev, next) {
      const drafts: EventDraft[] = [];
      for (const node of next.nodes) {
        const before = prev?.nodes.find(n => n.name === node.name);
        if (!before) continue;
        const beforeSelections = new Map(before.selections.map(
            (s, i) => [`${i}:${selectionKey(s)}`, s]));
        for (const [i, selection] of node.selections.entries()) {
          const prior = beforeSelections.get(`${i}:${selectionKey(selection)}`);
          if (prior && prior.selected !== selection.selected) {
            drafts.push({
              severity: 'info',
              category: 'system',
              title: `${node.name} now driven by ${selection.selected ?? 'none'}`
            });
          }
        }
      }
      return drafts;
    }
  }
];

export type ManifestDetector = {
  id: string;
  detect(prev: SessionManifest|null, next: SessionManifest): EventDraft[];
};

// Detectors over session-manifest updates: the launcher restarts a node and
// flips its failure flag, and the manifest is the only place either is visible.
export const MANIFEST_DETECTORS: ManifestDetector[] = [
  {
    id: 'manifest-node',
    detect(prev, next) {
      const drafts: EventDraft[] = [];
      for (const node of next.nodes) {
        const before = prev?.nodes.find(n => n.name === node.name);
        if (!before) continue;
        if (node.restart_count > before.restart_count) {
          drafts.push({
            severity: 'warn',
            category: 'system',
            title: `${node.name} restarted (${node.restart_count})`
          });
        }
        if (!before.recorder_failed && node.recorder_failed) {
          drafts.push({
            severity: 'error',
            category: 'system',
            title: `${node.name} failed`
          });
        }
      }
      return drafts;
    }
  }
];

import {Builder} from 'flatbuffers';

import {ChannelInfo} from './generated/talos/telemetry/channel-info';
import {Frame} from './generated/talos/telemetry/frame';
import {Pose} from './generated/talos/telemetry/pose';
import {
  type DeclaredGraph,
  type DeclaredNode,
  type SourceKind,
  SYSTEM_GRAPH_VERSION,
  type SystemGraph,
  type SystemNode,
  type SystemSource,
  type SystemTopic,
} from './system';

// One stream for the whole demo run; a bridge restart would pick a new one,
// which is how the viewer knows to throw the old timeline away.
export const DEMO_STREAM_ID = 42n;
export const DEMO_DICTIONARY_ID = 1;

const DEMO_CHANNELS: {name: string; unit: string; labels?: string[]}[] = [
  {name: 'velocity', unit: 'm/s'},
  {name: 'heading', unit: 'rad'},
  {name: 'voltage', unit: 'V'},
  {name: 'setpoint', unit: 'm/s'},
  {name: 'error', unit: 'm/s'},
  {name: 'vx', unit: 'm/s'},
  {name: 'vy', unit: 'm/s'},
  {name: 'mode', unit: '', labels: ['disabled', 'teleop', 'auto']},
];

export function demoPacket(sequence: bigint): Uint8Array {
  const b = new Builder(2048);
  const t = Number(sequence % 100000n) * .02;
  const x = 8 + 5 * Math.cos(t * .35), y = 4 + 2.5 * Math.sin(t * .35),
        yaw = t * .35 + Math.PI / 2;
  // The dictionary rides the first frame of each hundred, as the bridge's
  // cached dictionary frame does on connect; the rest are bare values.
  let dictionary: ReturnType<typeof Frame.createDictionaryVector>|undefined;
  if (sequence % 100n === 0n) {
    const entries = DEMO_CHANNELS.map(channel => {
      // Strings and vectors first: nothing may be built while a table is open.
      const name = b.createString(channel.name);
      const unit = channel.unit ? b.createString(channel.unit) : undefined;
      const labels = channel.labels ?
          ChannelInfo.createLabelsVector(
              b, channel.labels.map(label => b.createString(label))) :
          undefined;
      ChannelInfo.startChannelInfo(b);
      ChannelInfo.addName(b, name);
      if (unit !== undefined) ChannelInfo.addUnit(b, unit);
      if (labels !== undefined) ChannelInfo.addLabels(b, labels);
      return ChannelInfo.endChannelInfo(b);
    });
    dictionary = Frame.createDictionaryVector(b, entries);
  }
  const values = Frame.createValuesVector(b, [
    1.75, yaw, 12.4 - .8 * Math.sin(t * .7), 2, .2 * Math.sin(t * 2),
    -1.75 * Math.sin(t * .35), .875 * Math.cos(t * .35),
    Number(sequence / 50n % 3n)
  ]);
  Frame.startFrame(b);
  Frame.addStreamId(b, DEMO_STREAM_ID);
  Frame.addSequenceId(b, sequence);
  Frame.addTimestampNs(b, 1000000000n + sequence * 20000000n);
  Frame.addDictionaryId(b, DEMO_DICTIONARY_ID);
  Frame.addValues(b, values);
  if (dictionary !== undefined) Frame.addDictionary(b, dictionary);
  Frame.addChassis(b, Pose.createPose(b, x, y, .15, 0, 0, yaw));
  Frame.finishSizePrefixedFrameBuffer(b, Frame.endFrame(b));
  return b.asUint8Array();
}

// --- system graph ----------------------------------------------------------
//
// A stand-in for what the bridge reads out of the live node registry, so every
// system view is explorable with no robot attached. The node and topic set is
// the real one from 2026-robot/main_processor/configuration/robot.toml and the
// node headers, because a demo that shows a system nobody runs teaches the
// wrong thing.

type DemoSource = {
  kind: SourceKind;
  name: string;
  bytes: number;

  // Events per second while the owning node is healthy. Zero means the source
  // is registered but nothing ever arrives on it, which is a state worth being
  // able to see rather than one to hide.
  hz: number;

  // Only the one topic the demo deliberately loses on; see DEMO_NODES.
  dropsPerSecond?: number;

  // What this end says about its far end. `external` means the peer is outside
  // talOS; `optional` means it is allowed not to exist yet. Both are carried
  // through to the graph the viewer parses, so the demo can show a half-wired
  // topic that is not a fault next to one that is.
  external?: boolean;
  optional?: boolean;
};

type DemoNode = {
  name: string;
  target: string;
  pid: number;
  periodUs: number;

  // How long this node runs before it stops, in seconds. Undefined means it
  // never stops. A node that has stopped keeps its slot and its counters
  // frozen where they were, exactly as the registry leaves it -- so its rates
  // fall to zero and its heartbeat ages, rather than a dead node appearing to
  // still publish.
  stopsAfterSeconds?: number;

  // True for a node the launcher probed with `--describe` and that never
  // claimed a registry slot: it is in the declared graph and absent from the
  // live one, which is what a crash at startup or a missing binary looks like
  // from the outside. Its sources still describe what it would have held, so
  // the topics its peers wait on are the real ones.
  neverStarts?: boolean;
  sources: DemoSource[];
};

// introspect::kLivenessTimeout. Liveness is derived here the same way
// RegistryReader derives it, so the demo cannot disagree with a live robot.
const LIVENESS_SECONDS = 5;

const runtimeOf = (node: DemoNode, elapsed: number) =>
    node.stopsAfterSeconds === undefined ?
    elapsed :
    Math.min(elapsed, node.stopsAfterSeconds);

// hardware::Packet is one fixed slot regardless of payload; the rest are the
// flatbuffer structs the nodes exchange.
const HW_PACKET = 4104;
const STUDIO_SLOT = 16384;

// The demo carries a fault of every kind the viewer can name, on purpose.
// Every health state has to be reachable without breaking a robot to see it,
// and each of these is a shape that actually occurs:
//
//   orphaned     /shooter/state and /hw/request/shooter -- the shooter node was
//                declared and never claimed a registry slot, so odometry waits
//                on a topic nothing publishes and so does the hardware node.
//                From inside either one this is indistinguishable from a quiet
//                topic, which is exactly why it belongs in a system view. The
//                declared graph is the other half of the answer: it names the
//                node that was supposed to be here, which is the difference
//                between a crash at startup and a topic nobody ever wrote.
//   unread       /shooter/target -- the arbiter publishes it and the node that
//                would read it is the one that never started. Also
//                /talos/telemetry, whose reader is the Studio bridge: it holds
//                an RTMS reader slot but claims no registry slot, so from the
//                registry's side the feed has no subscriber.
//   bridged      /hw/command -- its consumer is the RoboRIO across the UDP
//                link, so a shared-memory subscriber will never exist. The
//                hardware node declares that end external, and the viewer says
//                so rather than counting it as unread.
//   unconnected  /drivetrain/target/auto and /shooter/target/auto -- ends
//                declared optional. Autonomous is not written, so a missing
//                peer is the design rather than a fault.
//   lossy        /hw/state -- a 200 Hz consumer occasionally lapped by a 200 Hz
//                producer.
//   idle         every topic, in the first moments of a session: both ends
//                wired and nothing published yet. The demo starts there rather
//                than mid-flight, so this is what the first frame shows.
//   stale        operator_interface -- it stops two seconds in, then holds its
//                registry slot with frozen counters. Watch it: it stays "alive"
//                until its heartbeat passes the five-second timeout, then goes
//                stale while its publish rates fall to zero.
const DEMO_NODES: DemoNode[] = [
  {
    name: 'hardware_node',
    target: '//talOS/bridge:hardware_node',
    pid: 4131,
    // Drives its own 1 kHz tick rather than an event loop, so unlike every
    // other node it declares no timer source.
    periodUs: 1000,
    // Same order as the node's own manifest: state, command, driver station,
    // then one request topic per subsystem.
    sources: [
      {kind: 'SENDER', name: '/hw/state', bytes: HW_PACKET, hz: 200},
      {
        kind: 'SENDER',
        name: '/hw/command',
        bytes: HW_PACKET,
        hz: 200,
        external: true
      },
      {
        kind: 'SENDER',
        name: '/hw/state/driver_station',
        bytes: HW_PACKET,
        hz: 50
      },
      {kind: 'FETCHER', name: '/hw/request/drivetrain', bytes: HW_PACKET, hz: 200},
      // The shooter never started, so nothing arrives on its request topic.
      {kind: 'FETCHER', name: '/hw/request/shooter', bytes: HW_PACKET, hz: 0}
    ]
  },
  {
    name: 'drivetrain',
    target: '//2026-robot/main_processor/drivetrain:node',
    pid: 4132,
    periodUs: 5000,
    sources: [
      {kind: 'TIMER', name: 'drivetrain', bytes: 0, hz: 200},
      // Two lost messages a second out of 200: enough that the lossy state is
      // visible within the first second of the demo rather than four seconds
      // in, and a rate a loaded 200 Hz consumer really does lap at.
      {
        kind: 'WATCHER',
        name: '/hw/state',
        bytes: HW_PACKET,
        hz: 200,
        dropsPerSecond: 2
      },
      {kind: 'FETCHER', name: '/drivetrain/target', bytes: 32, hz: 50},
      {kind: 'SENDER', name: '/hw/request/drivetrain', bytes: HW_PACKET, hz: 200},
      {kind: 'SENDER', name: '/drivetrain/state', bytes: 48, hz: 200}
    ]
  },
  {
    name: 'odometry',
    target: '//2026-robot/main_processor/odometry:node',
    pid: 4133,
    periodUs: 5000,
    sources: [
      {kind: 'TIMER', name: 'odometry', bytes: 0, hz: 200},
      {kind: 'WATCHER', name: '/drivetrain/state', bytes: 48, hz: 200},
      // Waiting on the node that never started.
      {kind: 'WATCHER', name: '/shooter/state', bytes: 24, hz: 0},
      {kind: 'SENDER', name: '/odometry/state', bytes: 64, hz: 200}
    ]
  },
  {
    name: 'shooter',
    target: '//2026-robot/main_processor/shooter:node',
    pid: 4134,
    periodUs: 5000,
    // Declared by the launcher, never in the registry. This is the case the
    // live graph alone cannot name: absent because it died on the way up, not
    // absent because nobody asked for it.
    neverStarts: true,
    sources: [
      {kind: 'TIMER', name: 'shooter', bytes: 0, hz: 200},
      {kind: 'WATCHER', name: '/hw/state', bytes: HW_PACKET, hz: 200},
      {kind: 'FETCHER', name: '/shooter/target', bytes: 16, hz: 50},
      {kind: 'SENDER', name: '/hw/request/shooter', bytes: HW_PACKET, hz: 200},
      {kind: 'SENDER', name: '/shooter/state', bytes: 24, hz: 200}
    ]
  },
  {
    name: 'arbiter',
    target: '//2026-robot/main_processor/arbiter:node',
    pid: 4135,
    periodUs: 20000,
    sources: [
      {kind: 'TIMER', name: 'arbiter', bytes: 0, hz: 50},
      {kind: 'FETCHER', name: '/drivetrain/target/teleop', bytes: 32, hz: 50},
      // No autonomous publisher exists yet, and the arbiter says so on the end
      // it holds: both auto topics have a reader, no writer, and no fault.
      {
        kind: 'FETCHER',
        name: '/drivetrain/target/auto',
        bytes: 32,
        hz: 0,
        optional: true
      },
      {kind: 'FETCHER', name: '/shooter/target/teleop', bytes: 16, hz: 50},
      {
        kind: 'FETCHER',
        name: '/shooter/target/auto',
        bytes: 16,
        hz: 0,
        optional: true
      },
      {kind: 'SENDER', name: '/drivetrain/target', bytes: 32, hz: 50},
      {kind: 'SENDER', name: '/shooter/target', bytes: 16, hz: 50}
    ]
  },
  {
    name: 'operator_interface',
    target: '//2026-robot/main_processor/operator_interface:node',
    pid: 4137,
    periodUs: 20000,
    stopsAfterSeconds: 2,
    sources: [
      {kind: 'TIMER', name: 'operator_interface', bytes: 0, hz: 50},
      {kind: 'WATCHER', name: '/hw/state/driver_station', bytes: HW_PACKET, hz: 50},
      {kind: 'FETCHER', name: '/drivetrain/state', bytes: 48, hz: 50},
      {kind: 'SENDER', name: '/drivetrain/target/teleop', bytes: 32, hz: 50},
      {kind: 'SENDER', name: '/shooter/target/teleop', bytes: 16, hz: 50}
    ]
  },
  {
    name: 'telemetry',
    target: '//2026-robot/main_processor/telemetry:node',
    pid: 4138,
    periodUs: 10000,
    sources: [
      {kind: 'TIMER', name: 'telemetry', bytes: 0, hz: 100},
      {kind: 'WATCHER', name: '/odometry/state', bytes: 64, hz: 200},
      {kind: 'WATCHER', name: '/drivetrain/state', bytes: 48, hz: 200},
      {kind: 'WATCHER', name: '/hw/state/driver_station', bytes: HW_PACKET, hz: 50},
      {kind: 'SENDER', name: '/talos/telemetry', bytes: STUDIO_SLOT, hz: 100}
    ]
  }
];

// Fixed origin, so two runs of the demo produce identical documents.
const DEMO_START_WALL_NS = 1735689600000000000n;
const DEMO_SESSION = '7351220489113141';

function demoSource(node: DemoNode, source: DemoSource, id: number,
                    elapsed: number): SystemSource {
  const live = runtimeOf(node, elapsed);
  const events = Math.floor(live * source.hz);

  // Only a dispatch measures latency: the loop records it in invoke(), which
  // runs for timers and watchers. A sender or a fetcher never sets it, so
  // reporting a number here would be inventing one.
  const dispatched = source.kind === 'TIMER' || source.kind === 'WATCHER';

  return {
    id,
    kind: source.kind,
    name: source.name,
    message_bytes: source.bytes,
    alignment: 8,
    period_ns: source.kind === 'TIMER' ? String(node.periodUs * 1000) : '0',
    events: String(events),
    dropped: String(Math.floor(live * (source.dropsPerSecond ?? 0))),
    // A sender reports the sequence it last wrote, a subscriber the cursor it
    // last read; both track this source's own event count.
    sequence: String(events),
    last_monotonic_ns: String(Math.round(live * 1e9)),
    last_latency_ns: String(dispatched ? 40000 + (id % 5) * 3000 : 0),
    max_latency_ns: String(dispatched ? 900000 + (id % 7) * 50000 : 0),
    external: source.external === true,
    optional: source.optional === true
  };
}

function demoNode(node: DemoNode, slot: number, elapsed: number,
                  wallNs: bigint): SystemNode {
  const sources =
      node.sources.map((source, i) => demoSource(node, source, i, elapsed));
  const live = runtimeOf(node, elapsed);

  // Only timers and watchers reach invoke(), so only they advance the loop's
  // dispatch index.
  const dispatches = sources.reduce(
      (total, source) =>
          source.kind === 'TIMER' || source.kind === 'WATCHER' ?
          total + BigInt(source.events) :
          total,
      0n);

  return {
    name: node.name,
    target: node.target,
    slot,
    pid: String(node.pid),
    session_id: DEMO_SESSION,
    generation: '1',
    start_wall_ns: String(DEMO_START_WALL_NS),
    // It last checked in when it last ran.
    heartbeat_wall_ns:
        String(DEMO_START_WALL_NS + BigInt(Math.round(live * 1e9))),
    dispatch_count: String(dispatches),
    declared_sources: sources.length,
    alive: elapsed - live < LIVENESS_SECONDS,
    // Nothing here is driving real hardware, and saying so is better than
    // letting the demo pass for a robot.
    simulation: true,
    replay: false,
    sources
  };
}

// Both ends of every topic, gathered across nodes. Deliberately the same
// derivation as TopicsOf() in studio/bridge/system.h -- timers excluded,
// counters summed, rows sorted by name -- so the demo and a live bridge hand
// the viewer the same shape.
function deriveTopics(nodes: SystemNode[]): SystemTopic[] {
  const rows = new Map<string, SystemTopic>();

  for (const node of nodes) {
    for (const source of node.sources) {
      // A timer has a name but nothing on the other end.
      if (source.kind === 'TIMER') continue;

      let row = rows.get(source.name);
      if (!row) {
        row = {
          name: source.name,
          message_bytes: 0,
          publishers: [],
          subscribers: [],
          published: '0',
          received: '0',
          dropped: '0'
        };
        rows.set(source.name, row);
      }

      row.message_bytes = Math.max(row.message_bytes, source.message_bytes);
      row.dropped = String(BigInt(row.dropped) + BigInt(source.dropped));
      if (source.kind === 'SENDER') {
        row.publishers.push(node.name);
        row.published = String(BigInt(row.published) + BigInt(source.events));
      } else {
        row.subscribers.push(node.name);
        row.received = String(BigInt(row.received) + BigInt(source.events));
      }
    }
  }

  return [...rows.values()].sort(
      (a, b) => a.name < b.name ? -1 : a.name > b.name ? 1 : 0);
}

// `sequence` is the demo frame counter, which advances at 50 Hz alongside
// demoPacket(), so counters here move at the rates the real nodes run at.
export function demoSystemGraph(sequence: bigint = 0n): string {
  const elapsed = Number(sequence % 100000n) * .02;
  const wallNs = DEMO_START_WALL_NS + BigInt(Math.round(elapsed * 1e9));
  // A node that never started holds no slot, so it is not in this document at
  // all -- the registry has nothing to say about it. Only the declared graph
  // does, which is the whole point of having one.
  const nodes = DEMO_NODES.filter(node => !node.neverStarts)
                    .map((node, i) => demoNode(node, i, elapsed, wallNs));

  const graph: SystemGraph&{kind: string} = {
    kind: 'talos.system_graph',
    // The demo states endpoint attributes on every end, so it is a version-2
    // document; claiming version 1 while carrying them would make the demo the
    // one graph the viewer cannot trust about them.
    version: SYSTEM_GRAPH_VERSION,
    wall_ns: String(wallNs),
    registry: {
      available: true,
      node_capacity: 32,
      source_capacity: 64,
      liveness_timeout_ns: '5000000000'
    },
    bridge: {
      topic: '/talos/telemetry',
      clients: 1,
      frames_forwarded: String(Math.floor(elapsed * 100)),
      invalid: '0',
      ws_drops: '0'
    },
    nodes,
    topics: deriveTopics(nodes)
  };
  return JSON.stringify(graph);
}

// --- declared graph ---------------------------------------------------------
//
// What the launcher would have written after probing each of these binaries
// with `--describe`, before it spawned any of them. Built from the same table
// as the live graph, so the two cannot disagree about what a node declares --
// only about which of them started, which is the difference worth showing.

// The launcher's lint, in the launcher's own wording (naming::CheckGraph).
//
// One warning, and no errors, because a graph with an error is one the launcher
// refuses to launch: a demo whose declared graph would have stopped the robot
// from starting would be showing an impossible session. The telemetry feed's
// reader is the Studio bridge, which holds an RTMS reader slot and claims no
// registry slot, so nothing the linter can see subscribes to it.
const DEMO_DIAGNOSTICS = [{
  severity: 'warning' as const,
  subject: '/talos/telemetry',
  message: 'is published but nothing subscribes to it'
}];

// Every end the node's constructor registered, timers included: `--describe`
// prints the manifest, and the manifest is where a timer lives too.
const declaredNode = (node: DemoNode): DeclaredNode => ({
  name: node.name,
  target: node.target,
  sources: node.sources.map(source => ({
                             kind: source.kind,
                             name: source.name,
                             message_bytes: source.bytes,
                             external: source.external === true,
                             optional: source.optional === true
                           }))
});

export function demoDeclaredGraph(): string {
  const graph: DeclaredGraph&{kind: string} = {
    kind: 'talos.declared_graph',
    version: 1,
    nodes: DEMO_NODES.map(declaredNode),
    diagnostics: DEMO_DIAGNOSTICS
  };
  return JSON.stringify(graph);
}

# Studio

talOS Studio is a browser telemetry viewer with panel modules and a persistent
workspace.

## Pages and panels

The first-launch pages are defined in [`src/layout/presets.ts`](src/layout/presets.ts):

| Page | Panels |
|---|---|
| Robot | Robot, Field, Events |
| Vision | Vision, Field |
| Mechanisms | Subsystems, Plot |
| Hardware | Hardware link, Events |
| System | System, Session, Health, Timing |
| Signals | Plot, Inspector |

Spatial (Field, 3D pose) and Tools (Agent, Frames) are additional presets.
The complete panel list is [`src/panels/registry.ts`](src/panels/registry.ts).

## Frontend structure

- [`src/worker.ts`](src/worker.ts) owns telemetry decoding, the jitter buffer,
  timeline, playback cursor and queries through the [agent RPC surface](agent/README.md).
- [`src/data/store.ts`](src/data/store.ts) holds the columnar render cache.
- [`src/data/studio.ts`](src/data/studio.ts) provides the worker-backed or static
  Studio context; [`src/data/hooks.ts`](src/data/hooks.ts) exposes React bindings.
- Panels read through a [`DataSource`](src/data/source.ts): `LiveSource` (the
  worker and store above) or, while a recording is open,
  [`RecordedSource`](src/data/recorded.ts) over the
  [review API](src/data/review.ts). One source is shown at a time.
- [`src/shell/`](src/shell/) renders the workspace and connection controls.
- [`src/layout/tree.ts`](src/layout/tree.ts) implements workspace operations.
  [`src/layout/workspace.ts`](src/layout/workspace.ts) persists the workspace in
  `localStorage` under `talos-studio-workspace`.

### Freshness

[`src/data/freshness.ts`](src/data/freshness.ts) marks a reading stale at age
>= 1 second, or when its sample timestamp or reading clock is unknown.
The system graph is stale while disconnected or when the bridge has said
nothing about it (a whole graph, an update, or a notice that the radio budget
deferred one) for 1 second.
[`src/data/studio.ts`](src/data/studio.ts) also invalidates the live clock and
system graph when the worker has posted nothing for one second.

### Producer restarts

A new telemetry `stream_id` is a restarted producer. Within one launcher
session the history continues across it, in the worker and in the headless
agent alike ([`src/history.ts`](src/history.ts)): channels are matched by name,
each frame keeps the dictionary it was decoded against, and an export carries
each stream's dictionary. A "Producer restarted" event marks the boundary,
plot lines and Driver Station mode bands break there, and detected events and
the session manifest baseline are kept, so a restart counted in the manifest
is still reported. The session is the one the system graph's live registry
row for the publisher of the bridge's telemetry topic carries
(`telemetrySession` in [`src/system.ts`](src/system.ts)); the manifest cannot
say, because a bridge serves the manifest file it was started with. A graph
naming another session replaces the history, keeping only a stream that
started after the old session was last seen; frames of a stream retired that
way are ignored until another stream appears. A new stream whose timestamps
start behind the history (a simulated or replayed clock) also replaces it.
With no graph naming the session (no registry), a restart always continues it.

## Add a panel

1. Write `src/panels/<id>/index.tsx`, exporting a
   [`PanelDefinition`](src/panels/types.ts). Keep persisted configuration as plain
   JSON; `parseConfig` must accept unknown input and return a valid configuration.

   ```tsx
   import {useChannelValue} from '../../data/hooks';
   import type {PanelDefinition} from '../types';

   function VoltagePanel() {
     const volts = useChannelValue('voltage');
     return <div className="voltage-panel">{volts ?? '—'} V</div>;
   }

   export const voltagePanel: PanelDefinition = {
     id: 'voltage',
     title: 'Voltage',
     category: 'Data',
     description: 'The robot bus voltage.',
     defaultConfig: {},
     parseConfig: () => ({}),
     component: VoltagePanel
   };
   ```

2. Add its import and array entry to `src/panels/registry.ts`. Set
   `recorded: true` once the panel renders from a recorded session (below);
   until then it shows that it is not available there.
3. Import panel-specific CSS from the module. Scope selectors under the panel's
   root class to avoid affecting other panels.

### Exports

An exported log is the telemetry format (`TLM3` pages) of the Studio that
wrote it, and only a Studio of the same format version imports it; an older
export is refused by name. A match from before a format change stays
readable through the nodes' event logs (`log_dump`), which are the complete
record, unlike the live view.

## Reviewing a recorded session

Studio opens a session the launcher recorded, through the review server
(`studio_review` in `studio/review`, which reads the session's event logs
directly; no conversion step). One command builds Studio and the server and
serves a log root (`./sim.sh` records into `/tmp/talos_logs`, one directory
per session):

```sh
bazel run -c opt //studio/review:studio_review -- --library /tmp/talos_logs
```

Then open `http://127.0.0.1:5805`. A Studio the review server served opens
straight into its session list (start time, length and class) and offers no
live connection; open a session from it, or later from **Open recording…**
in the header. A Studio served by the live bridge reaches a review server
through **Open recording…** too, with the server's address in the picker
(default `http://127.0.0.1:5805`, remembered in this browser).

While a recording is open it replaces the live view on every page, and the
footer becomes its transport: step back and forward by the step size, play
and pause at 1x, 2x or 4x, a slider over the whole session, and the playhead
as session time (from the session's start) and wall time. **Close recording**,
going live (`L`) or connecting returns to live data. A failed request (the
server unreachable, an HTTP error, a malformed answer) pauses playback and is
shown in the footer with **Retry**; panels keep what they last showed.

| Panel | On a recording |
|---|---|
| Plot | Each channel decimated to the plot's pixel width: the line is each bucket's last sample, at that sample's own publication time, and a bucket of more than one sample also draws its min-to-max range. The window ends at the playhead; "All history" is the session from its start to the playhead. |
| Field | The pose estimator's pose at the playhead (no pose while its `valid` is false), the trail and camera observations over the window. |
| Robot | Every reading at the playhead, stale as live when its sample is a second or more older than the playhead; the system graph, events and node selections are live-only and show as absent. |
| Every other panel | "Not available for recorded sessions yet." |

Values carry the channel names live telemetry uses. The chassis pose differs:
live, it is the topic `[studio] pose_topic` names; on a recording, it is the
estimator that the dictionary shows (the one with
`<owner>/state/observation_accepted_count`).

## Run

From `studio/`:

```sh
pnpm install
pnpm dev                          # Vite, bound to 127.0.0.1
pnpm build                        # tsc --noEmit + vite build
pnpm test                         # node:test via tsx (tests/*.test.ts*)
```

From the repository root:

```sh
bazel run //studio:dev
bazel build //studio:web_dist
```

See the [bridge README](bridge/README.md) for serving the frontend with a live
feed, the [agent README](agent/README.md) for headless queries, and
[DATA_FLOW](../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary) for the
transport and frame budgets.

## Reviewing a session

`studio_review` serves recorded sessions, read straight from their `.tlog`
event logs with no conversion step:

```sh
bazel run -c opt //studio/review:studio_review -- --library /tmp/talos_logs [--port 5805] [--cache-mb 256] [--webroot DIR]
```

`--library` is a log root as the launcher writes it: one directory per
session holding its `manifest.json` and event logs, rotated segments and
restarts included ([session segments and logs](../talOS/launcher/README.md#session-segments-and-logs)).
Relative paths are taken from where `bazel run` was invoked. The server
listens on 127.0.0.1 only (there is no flag to change that). It serves the
review API under `/review/`: `sessions.json`, and per session
`channels.json`, `bounds.json`, `snapshot?t_ns=` and
`series?c=a,b&start_ns=&end_ns=&points=N`, whose shapes are in
[`review/review.h`](review/review.h). Every other path is a file of the built
Studio (`//studio:web_dist`, from the binary's runfiles, or `--webroot`), so a
Studio opened there reads the API from its own origin; a path that leaves
the webroot is refused. A session id is one directory name of the library; a
parent, hidden or symlinked one is refused.

A request whose `Host` is not `127.0.0.1`, `localhost` or `[::1]` with the
server's port is refused (403): a web page cannot reach the server by
resolving its own name to this machine. A request from another loopback
origin (`http://127.0.0.1:<port>`, `http://localhost:<port>`,
`http://[::1]:<port>`, such as a Studio the live bridge served) gets that
origin back in `Access-Control-Allow-Origin`; any other origin gets none, so
its page cannot read a session.

The server never writes into the library, which may be a read-only copy: a
session's class (`match`, `other`, `unknown`) is the launcher's
`session_class` file when that is current, else read from its logs and
remembered by the server until the session's files change size, never
written back.

Values are what `log_dump --format jsonl` decodes from the same records,
named as live telemetry names them (`talOS/telemetry:channels`): each
talos_messages struct by field path, a target lane's envelope as
`<lane>/mode_since_us`, `<lane>/active` and `<lane>/message/...`, the Driver
Station packet as its live `hw/state/driver_station/*` channels, and the other
hardware packets (`/hw/state`, `/hw/command`, `/hw/request/<node>`) by
`log_dump`'s own decoder, a device by its logical id
(`hw/request/shooter/motors/1/demand`). A topic is read from its writer's
SEND records, else from the first reader's MESSAGE records. Every time is
nanoseconds on the recording's monotonic clock (the merge's event time), and
every value carries its publication time; `origin_wall_ns + t_ns` is the
wall-clock time. A damaged log is read and warned about as `log_dump` reads
it, a record claiming more payload than its source declares being damage
([dispatch log](../talOS/events/README.md)); a topic whose recorded schema
the channel model refuses ([telemetry](../talOS/telemetry/README.md#automatic-channels-and-units))
is not reviewed and is named on stderr; a log of another format version, or
a session with no intact record, is refused (422) naming the file, and only
that session's requests fail.

Opening a session streams it once, merged, and keeps per log file one
24-byte checkpoint per second of event time that holds records (where that
second starts, and which sources recorded a value in it, a SEND or MESSAGE
record). A series seeks to the checkpoints it needs and reads those seconds
only, one second either side of its range by event time: a message received
or sent more than a second from its publication time is not found. A
snapshot at `t` reads, per log file and per topic source in it, the latest
second (up to the one after `t`'s) that holds a value of that source, each
such second once, and the source's next earlier such second only when that
one yields none (every value in it published after `t`): normally one second
per source, however long ago the topic was last published. Source ids from 63 on
share one bit of the checkpoint, so such a source may also read seconds that
hold only another's values. No log file is read whole.
Series buckets (min, max, last, count) are kept for repeated requests in an
LRU of at most `--cache-mb`. So the heap is the index, plus that cache, plus
one read window per open log (REQ-OPS-42). On a generated 203 MiB session
(8 logs, 2.75 million records; `bazel run -c opt //studio/review:review_bench`,
Linux x86-64, a loaded host) the index pass took 0.49 s and kept 66,240 B
(2,760 log-file seconds), a series of 8 channels over the whole session
0.46 s, and peak resident memory was 5.7 MiB.

The API's shapes are pinned by golden responses for the review tests'
fixture session, [`review/testdata/golden`](review/testdata/golden):
`//studio/review:review_test` requires the server to reproduce them byte for
byte, and `//studio:review_golden_test` decodes them with
`src/data/review.ts`, so a shape change on one side fails the other's test.
For an intended change, regenerate them and change the other side in the
same commit:

```sh
TALOS_UPDATE_GOLDEN=1 bazel run //studio/review:review_test -- \
    --gtest_filter=ReviewTest.ResponsesEqualTheGoldenFiles
```

## Tests

Every `tests/*.test.ts*` file runs under `pnpm test`, and under Bazel through
the `STUDIO_TESTS` table in `BUILD.bazel` (add one line per new test file).
Panels are tested through their pure derivations, plus react-dom/server renders
against `createStaticStudio` only where the rendered output carries a behaviour
(staleness, unknown versus absent readings, parse errors). The data layer
(store, events, discovery, robot status, manifest, hooks), the layout tree,
the command palette and the registry have their own tests.

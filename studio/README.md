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
directly; no conversion step). With the server running on the same machine
(default `http://127.0.0.1:5805`), choose **Open recording…** in the header,
list the sessions (start time, length and class), and open one. The address
is remembered in this browser.

While a recording is open it replaces the live view on every page, and the
footer becomes its transport: step back and forward by the step size, play
and pause at 1x, 2x or 4x, a slider over the whole session, and the playhead
as session time (from the session's start) and wall time. **Close recording**,
going live (`L`) or connecting returns to live data. A failed request (the
server unreachable, an HTTP error, a malformed answer) pauses playback and is
shown in the footer with **Retry**; panels keep what they last showed.

| Panel | On a recording |
|---|---|
| Plot | Each channel decimated to the plot's pixel width: the line is each bucket's last sample, and a bucket of more than one sample also draws its min-to-max range. The window ends at the playhead; "All history" is the session from its start to the playhead. |
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

## Tests

Every `tests/*.test.ts*` file runs under `pnpm test`, and under Bazel through
the `STUDIO_TESTS` table in `BUILD.bazel` (add one line per new test file).
Panels are tested through their pure derivations, plus react-dom/server renders
against `createStaticStudio` only where the rendered output carries a behaviour
(staleness, unknown versus absent readings, parse errors). The data layer
(store, events, discovery, robot status, manifest, hooks), the layout tree,
the command palette and the registry have their own tests.

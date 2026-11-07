# Studio

talOS Studio is the robot's control-room viewer: a browser app that shows
exactly what the robot is doing, from Driver Station state to the live node
graph. It is built as a shell around a set of self-contained panel modules, so
a new way of reading telemetry is one new panel: no changes to the shell, the
worker, or any other panel.

## What you see

- **Status strip** (always visible, under the header): Driver Station mode,
  enabled, match time, alliance, battery, brownout, link age, nodes alive and
  the estimator, each shown as unknown when the data does not carry it.
- **Timeline** (footer): play/pause, step, live, a scrub track drawing Driver
  Station mode bands and event ticks (click a tick to seek), exact seek, and
  log import/export. Every panel shows the robot at this one cursor.
- **Pages** (tabs), opened from presets on first launch:

  | Page | Panels | Answers |
  |---|---|---|
  | Robot | Robot, Field, Events | What is the robot doing right now? |
  | Vision | Vision, Field | Are the cameras healthy, and do they agree with the estimate? |
  | Mechanisms | Subsystems, Plot | What is each mechanism asked to do, who is asking, and is it doing it? |
  | Hardware | Hardware link, Events | Power, the RoboRIO link, its counters and faults. |
  | System | System, Health, Timing | Which processes run, what they publish, how late they are. |
  | Signals | Plot, Inspector | Any channel, plotted or searched. |

  Spatial (Field, 3D pose) and Tools (Agent, Frames) are available from a
  page's preset menu or the command palette.
- **Panels**: Robot, Subsystems, Vision, Events, Health (Robot); Field, 3D pose
  (Spatial); Plot, Inspector (Data); Hardware link, Session, System, Timing
  (System); Agent, Frames (Tools).
- **Keyboard**: Space play/pause, Left/Right step, L live, Ctrl/Cmd+K command
  palette (pages, panels, presets, events, transport, workspace), ? shortcuts.

### Events

Events are facts read off the data at the instant they changed, never inferred
faults. Detectors (`data/detectors.ts`) are edge-triggered and robot-agnostic:
Driver Station mode transitions, brownout, link configuration, fault and
timeout masks, counter runs (one event when a counter starts increasing and one
when it stops, never one per increment), unhealthy cameras, invalid
estimators, enum-labelled channels changing, node restarts and target-source
changes from the system graph, and restarts or recorder failures from the
session manifest. A detector compares against the last known value of each
channel, so a change during a telemetry gap is still reported.

## Architecture

```
worker.ts ── decode / jitter / timeline / cursor / RPC ──┐
                                                         ▼
data/store.ts ── columnar render cache ──> data/hooks.ts ──> panels/<id>/index.tsx
                                                         ▲
data/studio.ts ── StudioProvider (worker or static) ─────┘
```

- **Worker** (`src/worker.ts`) is the single owner of the decoder, jitter
  buffer, timeline and playback cursor, and of the JSON-RPC query surface
  (`agent/rpc.ts`). It posts incremental updates to the main thread.
- **Store** (`data/store.ts`) keeps a columnar render cache fed by the worker,
  so a panel can read the whole retained history without the worker re-posting
  it.
- **Hooks** (`data/hooks.ts`) are the only way a panel or the shell reads data:
  `useSnapshot`, `useChannelValue`, `useSeries`, `usePlayback`, `useSystem`,
  `useEvents`, `useModeBands`, `useActions`, `useRobotStatus`, and friends.
- **Panels** (`panels/<id>/index.tsx`) each export a `PanelDefinition` and read
  data only through those hooks. They never import the shell.
- **Shell** (`shell/`) owns the workspace layout, page tabs, connection
  control, the status strip and the timeline. It renders panels through the
  registry and gives them navigation via a small context.

### Layout

A workspace is a JSON tree persisted in `localStorage` (`talos-studio-workspace`):

```
Workspace { version: 1, activePage, pages: Page[] }
Page { id, title, root: LayoutNode }
LayoutNode =
  | { type: 'split', direction: 'row'|'column', children, sizes }
  | { type: 'panel', instanceId, panelId, config }
```

The pure, unit-tested operations in `layout/tree.ts` (split, close, replace,
setConfig, resize, `parseWorkspace`) are the only code that shapes a page.
`layout/presets.ts` is data: each preset is a nested row/column arrangement of
panels, and `DEFAULT_PAGES` lists the presets a first launch opens. A panel's
`config` is plain JSON and its `parseConfig` must accept anything, falling back
to its default field by field.

### Registry

`panels/registry.ts` is an explicit array of `AnyPanelDefinition`, one import
per panel. The shell resolves a workspace's `panelId` through it; an id that is
not registered renders an "Unknown panel" placeholder that preserves its config
for a later upgrade.

## Data contract

Panels read only these facts (see `data/types.ts` for the exact types):

- `PlaybackState` — mode, cursor (`cursor_ns`), retained `bounds`, stream
  `stats`, and the connection `status`.
- `SystemState` — the live `SystemGraph`, measured `rates`, and the optional
  `declared` graph.
- `SessionManifest` (`useManifest()`) — the launcher's session manifest from
  the bridge's `/manifest.json`: config, output directory, and per-process
  restarts, exit codes and recorder state. Null when not offered.
- `ChannelInfo` — a dictionary channel: `{name, unit, labels}`.
- `Series` — columnar history (`times` in seconds since `t0_ns`, one
  `Float64Array` per channel, `NaN` where there was no reading).
- `RobotEvent` — a fact read off the data, never an inferred fault.
- `ModeBand` — a contiguous stretch of history in one Driver Station mode.

Discovery (`data/discovery.ts`) finds subsystems, cameras and estimators by
the shape of their channels and the system graph, so the same panels serve any
robot. Shared atoms (`ui/`): panel chrome, stats and badges, `Sparkline`,
`TimeSeriesChart`, `DataTable`, `ChannelPicker`, channel-name helpers and
formatters.

Telemetry names a channel without its leading slash and joins nested fields
with `/` (`pose_estimator/state/pose/x_m`). A `NaN`/absent sample is no
reading, not zero. Studio stays robot-agnostic: no robot's channel names are
hard-coded anywhere.

## Add a panel in three steps

1. **Write the module** `src/panels/<id>/index.tsx`:

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

2. **Register it** — add one import and one array entry to
   `src/panels/registry.ts`.

3. **Style it** — either in the module's own `panels/<id>/<id>.css` (imported
   by the module, so global CSS is never edited) or, for shared atoms, in
   `src/ui/`.

The panel now appears in the panel picker (grouped by category), in presets if
you add it there, and persists its config across reloads.

## CSS conventions

`style.css` holds only the design tokens and the shell chrome, plus shared
`ui/` primitives, and those global classes are **namespaced**: no bare generic
semantic words. An alert box is `.alert-error`, a red text utility is
`.text-warn`, a badge is `.ui-badge`, the status dot is `.status-dot`, the empty
state is `.empty-state`. A panel may freely reuse the words `error`, `warn`,
`ok`, `info`, `empty` and `badge` in its own class names without picking up a
global layout/background rule.

Panel CSS lives in `panels/<id>/<id>.css`, imported by the panel module, and
every selector is scoped under the panel's root class (`.system`, `.health`,
`.robot`, …), so one panel's styles can never leak into another.

Layout trap to avoid: a scrolling panel root that is a flex column lets its
sections shrink (default `flex-shrink: 1`) instead of scrolling. A scrolling
panel body is either a normal block container, or its sections set
`flex-shrink: 0`; only fill-the-panel views (plot, field, 3D) rely on flex
growth.

## Run

```sh
pnpm install
pnpm dev                          # Vite dev server on http://127.0.0.1:5173
pnpm build                        # tsc --noEmit + vite build
pnpm test                         # node:test via tsx (tests/*.test.ts*)
```

With Bazel from the repository root:

```sh
bazel run //studio:dev
bazel build //studio:web_dist
```

With the simulator (also starts the bridge and Studio):

```sh
./sim.sh
```

The bridge serves plain JSON for quick inspection:

```sh
curl localhost:5800/telemetry.json
curl localhost:5800/channels.json
curl localhost:5800/system.json
```

The headless JSON-RPC agent (same code path as the Agent panel) runs at
`bazel run //studio:agent` on `http://127.0.0.1:5804/rpc`.

## Tests

Every `tests/*.test.ts*` file runs under `pnpm test`, and under Bazel through
the `STUDIO_TESTS` table in `BUILD.bazel` (add one line per new test file).
Panels are tested with `createStaticStudio` and react-dom/server: an empty
studio and one shaped like a real robot's dictionary, asserting what a person
would read, plus unit tests for each panel's pure derivations. The data layer
(store, events, discovery, robot status, manifest, hooks), the layout tree,
the command palette and the registry have their own tests.

# talOS telemetry wire format (v3)

The Studio telemetry feed: one framework node (`//talOS/telemetry:node`)
observes state and status topics in the declared graph and publishes
size-prefixed FlatBuffer pages on the byte topic configured by `[studio] topic`
(default `/talos/telemetry`); the Studio bridge (`//studio/bridge`) assembles
them and sends each browser or agent its own change-only updates. This package
owns the framework node, `telemetry.fbs` (the page schema), `frames.h` (page
encoding, used by the producer, the bridge and the mock), `wire.h` (slot
geometry, page and dictionary limits, header validation) and `channels.h`
(`:channels`, the channel model below).

## Pages

Every message on the topic, and every binary WebSocket message the bridge
sends, is one FlatBuffers size-prefixed buffer:

```
[4-byte little-endian size n][Frame buffer, n bytes, identifier "TLM3"]
```

`talos::telemetry_wire::payload(slot)` validates that header (prefix present,
`8 <= n <= slot_bytes - 4`, `TLM3` at bytes `[8, 12)`) and returns prefix plus
buffer with the slot's padding stripped.

A tick is one or more **value pages** sharing `sequence_id`; a dictionary is
one or more **dictionary pages**. `page` counts from 0 to `page_count - 1`
within the tick (or the dictionary), pages arrive in order, and a reader
applies a tick when its last page arrives. A tick takes as many value pages as
its channels need, each at most 1200 B; a dictionary takes as many pages as it
needs, each at most one slot. A dictionary holds at most `kMaxChannels`
(`wire.h`) channels, so that the largest update, a keyframe of every channel,
fits the Studio bridge's egress burst and can be sent to a client whole.

`Frame` fields:

| Field | Meaning |
| --- | --- |
| `stream_id` | Random per producer process (fixed for its lifetime). A change means the producer restarted. |
| `sequence_id` | The producer's tick, incremented once per tick; every page of a tick carries it. |
| `timestamp_ns` | The tick's producer monotonic time, nanoseconds. |
| `dictionary_id` | FNV-1a 32 over the dictionary (names, units, kinds, alert levels, labels, in order). Identifies which dictionary channel indices refer to. |
| `page`, `page_count` | This page's place among the tick's value pages, or the dictionary's pages. |
| `channel_count` | Dictionary size; every channel index is below it. |
| `chassis` | Robot `Pose` (metres, radians), on every value page; zeros until `pose/valid` reads 1. |
| `keyframe` | Value pages: the update carries every channel, and a channel it does not carry is absent. Otherwise only the carried and `absent` channels change. |
| `channels`, `values` | Value pages: `values[i]` is the raw 64-bit sample of channel `channels[i]`, read per the channel's kind. |
| `absent` | Value pages (change-only updates): channels that lost their sample. |
| `first_channel`, `dictionary` | Dictionary pages: `dictionary[i]` describes channel `first_channel + i`. |

`ChannelInfo` is `{name, unit, labels, kind, alert}`. `unit` is `""` for unitless
channels. `kind` says how a sample reads: `Double` (IEEE-754 bits, NaN and
the infinities included), `Int`
(two's complement), `UInt`, or `Bool` (0 or 1). Integer fields travel as their
exact value, never through a double, so 64-bit ids and counters arrive exact.
A non-empty `labels` makes an integer channel an *enum channel*: `labels[value]`
names its value (for example `intake/inhibit` carries the `intake::Inhibit`
names). Out-of-range values have no label. A non-`None` `alert` (`Info`,
`Warning`, `Error`) makes a `Bool` channel an *alert* of that level, active
while 1 ([NAMING.md](../NAMING.md#topics) states which fields are alerts);
`None` is every other channel. Every string is written once,
never shared between offsets: Studio refuses a dictionary page whose strings,
counted per reference, outgrow the page.

A channel whose topic was not published this tick (stale or never sent) is
*absent*: a producer tick, a keyframe, does not carry it. Absence is never a
frozen last value. A NaN or infinite reading is not absence: it travels as its
bits, the bridge's JSON spells it `"NaN"`, `"Infinity"` or `"-Infinity"`, and
Studio shows it as such and plots it as a gap.

## Dictionary protocol

The framework node builds its dictionary once at construction from reflected
SENDER topics with role `state` or `status` matching include/exclude globs,
the matching Driver Station topic, and `pose/valid`. Schema-less topics other
than Driver Station contribute no channels. Dictionary pages precede a
tick's value pages on the cadence in
[DATA_FLOW.md](../../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary),
so a reader that attaches mid-stream assembles the dictionary.

Consumers cache the latest complete dictionary keyed by
`(stream_id, dictionary_id)`. A value page is usable only when its
`stream_id`, `dictionary_id` and `channel_count` match the cached dictionary;
until then the consumer waits (the bridge answers `503` on `/telemetry.json`).

## The live view and the record

The Studio bridge (`studio::ClientFeed` in
[`studio/bridge/telemetry.h`](../../studio/bridge/telemetry.h)) keeps, per
client, the values it last sent. A joining client gets the dictionary pages
and then a keyframe of the latest tick; after that each update carries only
the channels whose value or presence changed, at their exact current value,
with the tick's `sequence_id` and `timestamp_ns` (so the client knows the
age). With nothing changed it still sends an empty update every 100 ms. When
the egress budget cannot afford an update, the update is deferred, not
dropped: the next one carries every change since what the client last
received. A client therefore never shows a wrong or stale value as current,
and shows every channel's latest value within the budgeted latency.

**The live view can skip intermediate samples under budget pressure**, and
does so whenever several ticks pass between affordable updates. It is a viewer
feed, not a record. The complete per-tick record is each node's event log; the
telemetry node's own log holds every input it observed.

## Transport budget

Page caps, dictionary limits, shared-memory geometry, dictionary cadence,
configured rates and the radio budget live in
[DATA_FLOW.md](../../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary).
The topic overwrites (`kTopicOptions` in `node.h`): a reader that stalls is
lapped and counts what it missed, and never stops publication for other
readers; the bridge never applies a tick it lost a page of. Construction
refuses a dictionary past `kMaxChannels`, naming the topics with the most
channels to exclude; pages the dictionary once and refuses, by name, a channel
whose entry alone cannot fit one dictionary page; and for a graph reports the
pages a tick takes. Studio
refuses a whole dictionary over one channel it cannot decode, so construction
also refuses, by name, a channel whose name, unit or label exceeds the
dictionary string limit, or whose name starts with `derived/` (Studio's own
computed channels); exclude it (below) or rename the field. A WebSocket message
boundary does not guarantee a TCP/IP packet boundary.

## Automatic channels and units

Channels are discovered automatically from the graph descriptors. Rules 1 to
5 are `//talOS/telemetry:channels` (`channels.h`), the one implementation of
the channel model: `FlattenTopic(topic, schema_name, bfbs)` lists a topic's
channels (name, kind, unit, labels, alert level, and the scalar's offset
and type) from
its schema, refusing a schema that names an object it does not hold, holds
a struct that contains itself, nests structs more than 16 deep, or expands
to more than 4096 struct instances (`channels.h`; a recorded log's schema
may be damaged or crafted). The review server also refuses a topic of more
than 2048 channels; the node flattens only the channels its `exclude` keeps,
so a large message can be brought within the dictionary cap below by
excluding part of it. `ReadChannel` reads one
channel's raw sample from a message's
bytes (absent when the message ends before it), and `DriverStationChannels`
and `ReadDriverStation` do the same for the Driver Station packet.
`EnvelopeChannels(topic, inner, bfbs)` lists a target lane's: its header's
`<lane>/mode_since_us` and `<lane>/active`, then the inner struct under
`<lane>/message/`, each at its offset in the whole envelope. The node
builds its dictionary and reads every tick through it, so anything that
names or decodes a recorded message with it uses the live names. Rules 6
to 8 are the node's own.

1. **Naming**: Channel names follow wire conventions `<owner>/<role>/<field>`
   (for example `drivetrain/state/vx_mps`, `pose/valid`).
2. **Kinds from type**: `bool` fields are `Bool`; signed integer fields
   (`byte` to `long`) are `Int`; unsigned ones (`ubyte` to `ulong`) are
   `UInt`; `float` and `double` are `Double`.
3. **Units from suffix**: Numeric units are derived directly from field name
   suffixes (via `talOS/schema/unit_suffix.h`), such as `_mps` (`m/s`),
   `_rad` (`rad`), `_s` (`s`). Fields without recognized unit suffixes have empty units.
4. **Enum labels**: Enum fields extract string labels automatically from FlatBuffers
   schema reflection (`reflection::Schema`). An enum whose values would need
   more labels than the dictionary label limit, or that has no non-negative
   value, carries no labels and reads as a plain number.
5. **Alert levels**: a `bool` field (not an array element) whose name
   `naming::ParseAlertField` accepts (`alert_<error|warning|info>_<name>`,
   [NAMING.md](../NAMING.md#topics)) carries that level in `alert`; every
   other channel carries `None`. Its value is read like any bool, so a stale
   alert topic is absent, never a frozen "active" or "clear".
6. **Include/exclude globs**: In `telemetry.toml`, `include` selects whole
   topics only (default `["*"]`). `exclude` takes precedence and matches both
   whole topics and individual dictionary channel names: excluding a topic
   removes all its channels; excluding a channel leaves its siblings intact.
   Names match with or without a leading `/`, using shell-style globs (`*`
   also matches `/`). For example,
   `exclude = ["*_camera/status/frames_captured_count"]` removes that counter
   from every camera without hiding its other status fields. Automatic
   channels such as `pose/valid` can also be excluded. Only retained channels
   enter the dictionary and the pages.
7. **Pose extraction**: `[studio] pose_topic` selects a topic containing a
   `geometry.Pose2d` struct. Telemetry extracts `x_m`, `y_m`, and `yaw_rad` via
   reflection. Missing, stale, invalid or nonfinite pose data makes `pose/valid`
   and `chassis` zero.
8. **Labels from configuration**: an integer channel whose values index
   another subsystem's string-array parameter is named by it, at
   construction, with one `[[subsystem.labels]]` table per channel in
   `telemetry.toml`:

   ```toml
   [[subsystem.labels]]
   channel = "autonomous/state/routine_index"  # an integer channel of this dictionary
   from = "autonomous.routines"                # <subsystem>.<key>: an array of strings
   ```

   The labels travel as the channel's `labels`, like an enum's, so Studio
   and the dashboard show the routine's name and never read TOML. The node
   reads `from` with `param_of`; construction is refused, naming the key
   (`labels[<i>].channel` or `labels[<i>].from`), for a missing key, a
   `from` that is not `<subsystem>.<key>`, a subsystem the config lacks, a
   missing parameter, a value that is not a non-empty array of strings,
   more than 1024 labels or a label over the dictionary string limit, and,
   once the graph is known, a channel absent from the dictionary (excluded,
   or in no state or status topic), not an integer, or already labelled.
   The labels cost dictionary bytes only, nothing per tick.

## Dashboard layout

The driver dashboard's layout is the file `[studio] dashboard` in `robot.toml`
names, relative to the file that sets it. Config load refuses a missing file, and the file is in the config's
`CONFIG_FILES`, so `talos_bundle` ships it
([ARCHITECTURE.md](../../ARCHITECTURE.md#configuration)). A consumer reads the
resolved path from `config::RobotConfig::studio.dashboard` (empty when unset).
`//talOS/telemetry:dashboard_layout` (`dashboard_layout.h`) is the one parser:
`ParseDashboardLayout(file)` checks everything the file alone decides, and
`DashboardLayoutJson(layout)` is the layout as the dashboard page reads it
(`/drive/layout.json`), so no TOML parser is needed in TypeScript. What needs
the telemetry dictionary takes its inputs from the layout: the names that must
be in it (`RequiredChannels`), whether the feed also carries every alert
channel (`CarriesAlertChannels`), and the budget for the feed's full channel
count (`CheckDashboardBudget`).

```toml
[dashboard]                 # optional; every key optional, defaults shown
title = "Drive"             # at most 64 bytes
columns = 12                # grid cells, 1..24
rows = 8                    # 1..24
update_hz = 20              # drive feed updates a second, 1..50
budget_bytes_per_s = 24000  # each drive client's egress budget
drive_clients = 2           # 1..4; drive_clients x budget_bytes_per_s <= 80000

[[widgets]]                 # 1 to 64 widgets, drawn in file order
kind = "number"             # required
at = [0, 0, 2, 1]           # required: column, row, width, height
title = "Flywheel"          # optional, at most 64 bytes; "" uses the kind's heading
channel = "shooter/state/flywheel_velocity_rps"
```

A channel is a dictionary name, `<owner>/<role>/<field path>` with no leading
`/` (letters, digits, `_` and `/`), at most 256 bytes. A layout reads at most
256 distinct channels. Each kind takes only its own keys:

| `kind` | Keys (defaults) | Channels it reads |
| --- | --- | --- |
| `match` | none | `hw/state/driver_station/{connected, enabled, autonomous, teleop, test, estop, fms_attached, alliance, match_time_s}` |
| `power` | `warn_below_v` (11.5; in (0, 20]) | `hw/status/{battery_v, brownout, health_valid}` |
| `routine` | `channel` (required), `phase` | both |
| `alerts` | none | every alert channel of the dictionary |
| `subsystems` | `nodes`: node names to show (all) | none: the bridge's drive summary |
| `field` | `pose` (required: the prefix of `x_m`, `y_m`, `yaw_rad`), `valid`, `localized`, `trail_s` (5; [0, 30]) | the three pose fields, `valid`, `localized` |
| `number` | `channel` (required), `label` (""), `precision` (2; 0..6), `warn_below`, `warn_above` (below < above) | `channel` |
| `boolean` | `channel` (required), `label` (""), `good` (true: the value drawn green) | `channel` |
| `text` | `channel` (required), `label` ("") | `channel`, shown by its enum label |
| `graph` | `channels` (required, 1 to 8), `window_s` (10; (0, 30]), `min`, `max` (min < max) | `channels` |
| `checks` | `items` (required, 1 to 16) of `{ label, channel, equals }` or `{ label, channel, at_least, at_most }` (either bound, or both) | each item's `channel` |

A check passes while its channel's value meets every condition: `equals`
takes a bool (a Bool channel), a number, or a string (an enum channel's
label). `camera`, `mechanism`, `chooser` and `tunable` are reserved kinds,
and a `field` widget's `trajectory` a reserved key: each is refused by name
until its slice lands.

Parsing refuses, naming the file, then `[dashboard].<key>` or
`widgets[<index>].<key>` (index from 0; for example `widgets[3].at`,
`widgets[1].items[0].equals`): a TOML syntax error (with its line and
column); an unknown top-level key, `[dashboard]` key or widget key; a missing,
unknown or reserved `kind`; a missing required key; a value of the wrong type
or outside its range; a cell that is not four integers, is empty, lies
outside the grid or overlaps an earlier widget; a malformed channel or node
name, or one listed twice in a widget; a check with no condition or with
`equals` beside a bound; more widgets, graph channels, check items or
distinct channels than the limits above; drive reservations over 80,000 B/s;
and a budget below the worst case.

**Worst case.** One drive client can be sent, in a second,
`update_hz` keyframes of the feed's channels plus two summaries and ten
heartbeats:

```
update_hz x (12 x channels + pages x 210) + 2 x (4096 + 10) + 10 x 210
pages = max(1, ceil(channels / 83))
```

210 B is a value page's overhead bound plus a WebSocket header, 83 the
samples a 1200 B page always holds, 4096 B the drive summary's cap at most
every 500 ms, and an empty update goes out every 100 ms (`wire.h`).
Parsing requires `budget_bytes_per_s` to cover the named channels;
`CheckDashboardBudget(layout, count)` checks it for a feed of `count`
channels, such as the named ones plus the dictionary's alert channels.
At 20 Hz a 24,000 B/s budget covers 39 channels.

**`/drive/layout.json`.** One object; every key present, defaults filled
in, an unset optional number `null`, `"channels"` the sorted distinct named
channels (the named part of the drive feed's channel mask), `"alerts"`
whether the feed also carries every alert channel, `"file"` the layout's file
name. Each widget has `kind`, `at`, `title` and a `channels` object from role
to channel name (a widget's single channels; for `match` and `power` the
roles are the field names above, for `field` `x`, `y`, `yaw`, `valid`,
`localized`), then its options: `power` `warn_below_v`; `subsystems`
`nodes`; `field` `trail_s`; `number` `label`, `precision`, `warn_below`,
`warn_above`; `boolean` `label`, `good`; `text` `label`; `graph` `series`
(its channels), `window_s`, `min`, `max`; `checks` `items`, each
`{label, channel, equals, at_least, at_most}`.

```json
{"version":1,"file":"dashboard.toml","title":"Drive","columns":12,"rows":8,
 "update_hz":20,"budget_bytes_per_s":24000,"drive_clients":2,"alerts":false,
 "channels":["autonomous/state/phase","autonomous/state/routine_index"],
 "widgets":[{"kind":"routine","at":[0,0,4,2],"title":"",
   "channels":{"channel":"autonomous/state/routine_index",
               "phase":"autonomous/state/phase"}}]}
```

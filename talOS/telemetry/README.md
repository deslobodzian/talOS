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
| `dictionary_id` | FNV-1a 32 over the dictionary (names, units, kinds, labels, in order). Identifies which dictionary channel indices refer to. |
| `page`, `page_count` | This page's place among the tick's value pages, or the dictionary's pages. |
| `channel_count` | Dictionary size; every channel index is below it. |
| `chassis` | Robot `Pose` (metres, radians), on every value page; zeros until `pose/valid` reads 1. |
| `keyframe` | Value pages: the update carries every channel, and a channel it does not carry is absent. Otherwise only the carried and `absent` channels change. |
| `channels`, `values` | Value pages: `values[i]` is the raw 64-bit sample of channel `channels[i]`, read per the channel's kind. |
| `absent` | Value pages (change-only updates): channels that lost their sample. |
| `first_channel`, `dictionary` | Dictionary pages: `dictionary[i]` describes channel `first_channel + i`. |

`ChannelInfo` is `{name, unit, labels, kind}`. `unit` is `""` for unitless
channels. `kind` says how a sample reads: `Double` (IEEE-754 bits, NaN and
the infinities included), `Int`
(two's complement), `UInt`, or `Bool` (0 or 1). Integer fields travel as their
exact value, never through a double, so 64-bit ids and counters arrive exact.
A non-empty `labels` makes an integer channel an *enum channel*: `labels[value]`
names its value (for example `intake/inhibit` carries the `intake::Inhibit`
names). Out-of-range values have no label. Every string is written once,
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
4 are `//talOS/telemetry:channels` (`channels.h`), the one implementation of
the channel model: `FlattenTopic(topic, schema_name, bfbs)` lists a topic's
channels (name, kind, unit, labels, and the scalar's offset and type) from
its schema, `ReadChannel` reads one channel's raw sample from a message's
bytes (absent when the message ends before it), and `DriverStationChannels`
and `ReadDriverStation` do the same for the Driver Station packet. The node
builds its dictionary and reads every tick through it, so anything that
names or decodes a recorded message with it uses the live names. Rules 5
and 6 are the node's own.

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
5. **Include/exclude globs**: In `telemetry.toml`, `include` selects whole
   topics only (default `["*"]`). `exclude` takes precedence and matches both
   whole topics and individual dictionary channel names: excluding a topic
   removes all its channels; excluding a channel leaves its siblings intact.
   Names match with or without a leading `/`, using shell-style globs (`*`
   also matches `/`). For example,
   `exclude = ["*_camera/status/frames_captured_count"]` removes that counter
   from every camera without hiding its other status fields. Automatic
   channels such as `pose/valid` can also be excluded. Only retained channels
   enter the dictionary and the pages.
6. **Pose extraction**: `[studio] pose_topic` selects a topic containing a
   `geometry.Pose2d` struct. Telemetry extracts `x_m`, `y_m`, and `yaw_rad` via
   reflection. Missing, stale, invalid or nonfinite pose data makes `pose/valid`
   and `chassis` zero.

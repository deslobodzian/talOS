# talOS telemetry wire format (v2)

The Studio telemetry feed: one framework node (`//talOS/telemetry:node`)
observes state and status topics in the declared graph and publishes
size-prefixed FlatBuffer `Frame`s on the byte topic configured by `[studio] topic`
(default `/talos/telemetry`); the Studio bridge (`//studio/bridge`)
forwards them to browsers and agents. This package owns the framework node,
`telemetry.fbs` (the frame schema), and `wire.h` (slot geometry, frame and
dictionary limits, header validation).

## Frame format

Every message on the topic, and every binary WebSocket frame the bridge
forwards, is a FlatBuffers size-prefixed buffer:

```
[4-byte little-endian size n][Frame buffer, n bytes, identifier "TLM2"]
```

`talos::telemetry_wire::payload(slot)` validates that header (prefix present,
`8 <= n <= slot_bytes - 4`, `TLM2` at bytes `[8, 12)`) and returns prefix plus
buffer with the slot's padding stripped.

`Frame` fields:

| Field | Meaning |
| --- | --- |
| `stream_id` | Random per producer process (fixed for its lifetime). A change means the producer restarted. |
| `sequence_id` | Increments per frame, including frames DROP_NEWEST shed, so gaps are visible downstream. |
| `timestamp_ns` | Producer monotonic clock, nanoseconds. |
| `dictionary_id` | FNV-1a 32 over the dictionary (names, units, labels, in order). Identifies which dictionary `values` aligns with. |
| `chassis` | Robot `Pose` (metres, radians); zeros until `pose/valid` reads 1. |
| `values` | One double per dictionary entry, index-aligned. `NaN` means the source was absent or stale this tick — never a frozen last value. |
| `dictionary` | `[ChannelInfo]`; present only on dictionary frames (below). |

`ChannelInfo` is `{name, unit, labels}`. `unit` is `""` for unitless channels.
A non-empty `labels` makes the channel an *enum channel*: its value is an
integral index and `labels[value]` names it (for example `intake/inhibit`
carries the `intake::Inhibit` names). Out-of-range values have no label.
Every string is written once, never shared between offsets: Studio refuses a
dictionary whose strings, counted per reference, outgrow the frame.

## Dictionary protocol

The framework node builds its dictionary once at construction from reflected
SENDER topics with role `state` or `status` matching include/exclude globs,
the matching Driver Station topic, and `pose/valid`. Schema-less topics other
than Driver Station contribute no channels. Dictionary frames repeat as
specified in [DATA_FLOW.md](../../docs/DATA_FLOW.md).

Consumers cache the latest dictionary keyed by `(stream_id, dictionary_id)`.
A value-only frame is usable only when its `dictionary_id` matches the cached
dictionary for its `stream_id`; until then the consumer waits (the bridge
answers `503` on `/telemetry.json`).

## Transport budget

Frame caps, dictionary limits, shared-memory geometry, dictionary cadence and
configured rates live in [DATA_FLOW.md](../../docs/DATA_FLOW.md). Construction
measures both worst-case value frames and dictionary frames and throws if either
exceeds its budget, naming the largest sources or dictionary channels. Studio
refuses a whole dictionary over one channel it cannot decode, so construction
also refuses, by name, a channel whose name, unit or label exceeds the
dictionary string limit, or whose name starts with `derived/` (Studio's own
computed channels); exclude it (below) or rename the field. A WebSocket message
boundary does not guarantee a TCP/IP packet boundary.

## Automatic channels and units

Channels are discovered automatically from the graph descriptors:

1. **Naming**: Channel names follow wire conventions `<owner>/<role>/<field>`
   (for example `drivetrain/state/vx_mps`, `pose/valid`).
2. **Units from suffix**: Numeric units are derived directly from field name
   suffixes (via `talOS/schema/unit_suffix.h`), such as `_mps` (`m/s`),
   `_rad` (`rad`), `_s` (`s`). Fields without recognized unit suffixes have empty units.
3. **Enum labels**: Enum fields extract string labels automatically from FlatBuffers
   schema reflection (`reflection::Schema`). An enum whose values would need
   more labels than the dictionary label limit, or that has no non-negative
   value, carries no labels and reads as a plain number.
4. **Include/exclude globs**: In `telemetry.toml`, `include` selects whole
   topics only (default `["*"]`). `exclude` takes precedence and matches both
   whole topics and individual dictionary channel names: excluding a topic
   removes all its channels; excluding a channel leaves its siblings intact.
   Names match with or without a leading `/`, using shell-style globs (`*`
   also matches `/`). For example,
   `exclude = ["*_camera/status/frames_captured_count"]` removes that counter
   from every camera without hiding its other status fields. Automatic
   channels such as `pose/valid` can also be excluded. Only retained channels
   contribute to the dictionary, values vector and construction-time budget.
5. **Pose extraction**: `[studio] pose_topic` selects a topic containing a
   `geometry.Pose2d` struct. Telemetry extracts `x_m`, `y_m`, and `yaw_rad` via
   reflection. Missing, stale, invalid or nonfinite pose data makes `pose/valid`
   and `chassis` zero.

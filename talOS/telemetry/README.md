# talOS telemetry wire format (v2)

The Studio telemetry feed: one framework node (`//talOS/telemetry:node`)
observes state and status topics in the declared graph and publishes
size-prefixed FlatBuffer `Frame`s on the `/talos/telemetry` byte topic
(configured via `[studio] topic`); the Studio bridge (`//studio/bridge`)
forwards them to browsers and agents. This package owns the framework node,
`telemetry.fbs` (the frame schema), and `wire.h` (slot geometry and header validation).

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

## Dictionary protocol

The framework node builds its dictionary **once at construction** from the
graph descriptors (all SENDER topics with role `state` or `status` matching
include/exclude globs, plus Driver Station and `pose/valid`), and never
changes it afterwards. It sends the dictionary:

- on the **first** frame of the stream,
- on **every 100th** frame after it (sequence 100, 200, ...),
- **never otherwise**.

Consumers cache the latest dictionary keyed by `(stream_id, dictionary_id)`.
A value-only frame is usable only when its `dictionary_id` matches the cached
dictionary for its `stream_id`; until then the consumer waits (the bridge
answers `503` on `/telemetry.json`). Because the dictionary repeats every 100
frames (once per second at 100 Hz), a consumer that joins mid-stream or loses
a dictionary frame recovers within a second, and a producer restart (new
`stream_id`) re-sends one immediately.

## Size and rate budget

Value-only frames are capped at `kMaxValueFrameBytes = 1200` bytes **including
the 4-byte prefix**, enforced by the producer at construction: it builds a
worst-case all-channels-present frame, measures it, and throws rather than
start if the frame would exceed the cap.

- One frame fits a 1500-byte MTU Ethernet frame with IPv4/TCP and WebSocket
  framing overhead to spare, so a frame is never fragmented across packets.
- At the 100 Hz telemetry rate: `1200 B x 8 bit/B x 100 /s = 0.96 Mbit/s` per
  connected client — under 25% of the 4 Mbit/s FRC on-field radio floor that
  rule R704 budgets around.
- Dictionary frames are larger (they carry every name, unit and label) but
  bounded by the 16384-byte slot, and at one frame per second their amortised
  cost is negligible next to the value stream.
- The topic ring is 64 slots x 16384 B, so the bridge can lag over half a
  second at 100 Hz before DROP_NEWEST starts shedding the newest frames.

If the channel count grows enough to threaten the 1200-byte cap, the
construction-time check fails loudly at startup instead of the feed silently
blowing the radio budget on the field.

## Automatic channels and units

Channels are discovered automatically from the graph descriptors:

1. **Naming**: Channel names follow wire conventions `<owner>/<role>/<field>`
   (for example `drivetrain/state/vx_mps`, `pose/valid`).
2. **Units from suffix**: Numeric units are derived directly from field name
   suffixes (via `talOS/schema/unit_suffix.h`), such as `_mps` (`m/s`),
   `_rad` (`rad`), `_s` (`s`). Fields without recognized unit suffixes have empty units.
3. **Enum labels**: Enum fields extract string labels automatically from FlatBuffers
   schema reflection (`reflection::Schema`).
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
5. **Pose extraction**: When `[studio] pose_topic` is set, telemetry extracts
   `x_m`, `y_m`, and `yaw_rad` from the `geometry.Pose2d` struct inside that topic via
   reflection and sets `chassis`. When stale, `pose/valid` is 0 and chassis is 0.
6. **Budget enforcement**: The 1200-byte worst-case frame budget is verified at
   construction. If exceeded, construction throws naming the largest topics.

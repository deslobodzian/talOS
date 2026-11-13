# Names

`talOS/introspection/names.h` is the authority; the launcher checks every name
before it spawns anything (`--describe-only` runs the same check and exits),
and `talos_messages` refuses a malformed topic when its `BUILD` file loads
(`contract.bzl`, held to `names.h` by `//talOS/build:topic_grammar_test`).

## Topics

`/owner/role[/qualifier[/qualifier]]`: 2 to 4 segments, at most 31 bytes in
total (the macOS shared-memory name limit), each segment at most 24 bytes.
A segment is lowercase letters, digits and `_`, starts with a letter, and has no
doubled or trailing `_`.

- **owner**: the subsystem the topic is about (the node name), or a reserved
  owner: `hw` (the hardware bridge), `talos` (framework), `sim`, `replay`,
  `test` (tests only; refused in a launch).
- **role**, one of:

  | role | published by | meaning |
  |---|---|---|
  | `state` | the owner | what it is now |
  | `status` | the owner | health and liveness |
  | `event` | the owner | something that happened once |
  | `telemetry` | the owner | observation feed for recording/display |
  | `target` | anyone but the owner | what it is asked to become |
  | `request` | anyone but the owner | actuator instruction awaiting resolution |
  | `command` | anyone but the owner | actuator instruction, already resolved |

- **qualifier**: the source or variant, e.g. `teleop`, `auto`. Target sources
  in `[robot.targets]` and `[targets]` follow the segment rules. A target
  lane is exactly `/<subsystem>/target/<source>`, a valid name of three
  segments (`naming::IsTargetLane`). Producers build it with
  `naming::TargetLaneTopic`; the consumer subscribes to the relative
  `target/<source>`, resolved by its node. See
  [target lanes](../ARCHITECTURE.md#targets).

- **alerts** (role `status`): a `bool` field of a node's own `status`
  message named `alert_<level>_<name>`, where `<level>` is `error`,
  `warning` or `info` and `<name>` is a lowercase letter, then lowercase
  letters, digits and single underscores, is an alert, active while true.
  The node reports the condition and does not latch it.
  `naming::ParseAlertField` is the only reader of this grammar: the
  telemetry dictionary carries each alert's level (`ChannelInfo.alert`), so
  no viewer parses names. The graph lint refuses an `alert_` field that is
  not a `bool` or not of these levels (nested structs included), and warns
  on an alert outside a `status` topic.

Abbreviations are rejected (`tgt`, `cmd`, `req`, `ds`, `dt`, `drive`, `odom`,
`pos`, `vel`, …; use the full word), and so are direction suffixes (`_in`,
`_out`, `_pub`, `_sub`, `_tx`, `_rx`, `_topic`, `_send`, `_recv`).

The graph lint also rejects two writers on one topic, more than 32 declared
readers per topic (including observers), a read topic nobody writes (unless an
endpoint declares it `optional` or `external`), conflicting schemas, message
sizes or ring layouts, a target lane whose source is in no mode of that
subsystem's target policy, and an `hw` topic written by anything but the
hardware bridge (the launcher's `hardware_node` roster entry, whatever binary
`[robot] hardware_node` names and whatever name it describes), except
`/hw/request/<node>`, which only `<node>`'s devices-layer flush writes (its
endpoint carries the `hardware_request` attribute; RULES.md 2). Independently
of the lint, which `--allow-graph-errors` overrides, a node's `publish` and
`publish_bytes` (C++, or Python through `talOS/node_api`) refuse any topic
under `/hw/request`, remapped or not, at construction. A topic nobody reads, a
policy lane nobody writes yet, or a topic whose owner is not a node in the
session, is a warning.

Examples: `/drivetrain/state`, `/drivetrain/target/teleop` (written by the
operator interface), `/drivetrain/target/auto`, `/hw/state`,
`/hw/state/driver_station`, `/hw/request/<node>`, `/talos/telemetry`.

## Nodes

A node name follows the segment rules, is not a reserved owner, and is the
`[subsystems.<name>]` key, the `node_name` in `talos_cc_node`/`talos_py_node`,
the log file stem and the registry row. The node's Bazel label must match the
name in its package leaf, its repository (at a repository root), or its rule:
`//2026-robot/main_processor/shooter:node`, `@wrist//:node`,
`//examples/superstructure/cpp:elevator`.

Message types are PascalCase in the package's FlatBuffers namespace
(`shooter::ShooterTarget`); fields are `snake_case` with the unit in the name
(`target_velocity_rps`, `period_us`).

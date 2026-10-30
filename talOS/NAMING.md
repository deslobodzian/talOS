# Names

`talOS/introspection/names.h` is the authority; the launcher checks every name
before it spawns anything (`--describe-only` runs the same check and exits).

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
  | `request` | anyone but the owner | actuator instruction awaiting arbitration |
  | `command` | anyone but the owner | actuator instruction, already resolved |

- **qualifier**: the source or variant, e.g. `teleop`, `auto`.

Abbreviations are rejected (`tgt`, `cmd`, `req`, `ds`, `dt`, `drive`, `odom`,
`pos`, `vel`, …; use the full word), and so are direction suffixes (`_in`,
`_out`, `_pub`, `_sub`, `_tx`, `_rx`, `_topic`, `_send`, `_recv`).

The graph lint also rejects two writers on one topic, a read topic nobody
writes (unless the subscription is `optional`), and one topic used with two
message sizes. A topic nobody reads, or whose owner is not a node in the
session, is a warning.

Examples: `/drivetrain/state`, `/drivetrain/target` (written by the arbiter),
`/drivetrain/target/teleop`, `/driver_station/state`, `/hw/state`,
`/hw/request/<node>`, `/talos/telemetry`.

## Nodes

A node name follows the segment rules, is not a reserved owner, and is the
`[subsystems.<name>]` key, the `node_name` in `talos_cc_node`/`talos_py_node`,
the log file stem and the registry row. The node's Bazel label must match the
name in its package leaf, its repository (at a repository root), or its rule:
`//2026-robot/main_processor/shooter:node`, `@wrist//:node`,
`//examples/superstructure/cpp:wrist`.

Message types are PascalCase in the package's FlatBuffers namespace
(`shooter::ShooterTarget`); fields are `snake_case` with the unit in the name
(`target_velocity_rps`, `period_us`).

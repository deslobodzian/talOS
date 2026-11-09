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
  | `request` | anyone but the owner | actuator instruction awaiting resolution |
  | `command` | anyone but the owner | actuator instruction, already resolved |

- **qualifier**: the source or variant, e.g. `teleop`, `auto`. Target sources
  in `[robot.targets]` and `[targets]` follow the segment rules; see
  [target lanes](../ARCHITECTURE.md#targets).

Abbreviations are rejected (`tgt`, `cmd`, `req`, `ds`, `dt`, `drive`, `odom`,
`pos`, `vel`, …; use the full word), and so are direction suffixes (`_in`,
`_out`, `_pub`, `_sub`, `_tx`, `_rx`, `_topic`, `_send`, `_recv`).

The graph lint also rejects two writers on one topic, more than eight declared
readers per topic (including observers), a read topic nobody writes (unless an
endpoint declares it `optional` or `external`), conflicting schemas, message
sizes or ring layouts, and a target lane whose source is in no mode of that
subsystem's target policy. A topic nobody reads, a policy lane nobody writes
yet, or a topic whose owner is not a node in the session, is a warning.

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

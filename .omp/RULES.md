# talos hard rules

- Layers depend only downward (rtms → events → node → devices → robot code);
  `talOS/` never depends on a robot, an example or `studio/`. See ARCHITECTURE.md.
- A node-API feature lands in one change in `talOS/node`, `talOS/node_api` (C ABI)
  and `talOS/python/talos`. A device type touches `talOS/hardware`,
  `talOS/devices`, `talOS/python/talos/devices.py` and the RoboRIO program.
- One implementation per concept: framework features (`Node`, `StateMachine`,
  devices, `Sim`) are implemented once in C++ and wrapped in Python via
  `talOS/node_api`; shared C++ nodes exposed to Python use `TALOS_EXPORT_NODE`
  and `talos.cpp_node` rather than a second Python implementation.
- Language independence: adding or modifying a node in one language never
  requires editing code in another language (e.g. a Python subsystem is wired
  into `arbiter`, `operator_interface` and `telemetry` through its TOML config,
  never by editing C++ files).
- Messages are FlatBuffers structs only. Topic names come from the package's
  `talos_messages(topics = ...)` and follow talOS/NAMING.md. One writer per topic;
  extra producers publish `/<sub>/target/<source>` for an arbiter.
- Actuation is gated on `robot_enabled(node)` and a `fresh()` target; never write
  `/hw/request/<node>` directly.
- Never set `commissioned = true` or change physical CAN ids in a robot config.
- New node behaviour gets a `Sim`/`SimHardware` (or `talos.testing`) test that
  covers the disabled and stale-target cases. Finish every change with
  `bazel build //...` and `bazel test --test_output=errors //...`, plus launcher
  `--describe-only` for graph/config changes.

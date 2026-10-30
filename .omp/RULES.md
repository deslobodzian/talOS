# talos hard rules

- Layers depend only downward (rtms → events → node → devices → robot code);
  `talOS/` never depends on a robot, an example or `studio/`. See ARCHITECTURE.md.
- A node-API feature lands in one change in `talOS/node`, `talOS/node_api` (C ABI)
  and `talOS/python/talos`. A device type touches `talOS/hardware`,
  `talOS/devices`, `talOS/python/talos/devices.py` and the RoboRIO program.
- Messages are FlatBuffers structs only. Topic names come from the package's
  `talos_messages(topics = ...)` and follow talOS/NAMING.md. One writer per topic;
  extra producers publish `/<sub>/target/<source>` for an arbiter.
- Actuation is gated on `robot_enabled(node)` and a `fresh()` target; never write
  `/hw/request/<node>` directly.
- Never set `commissioned = true` or change physical CAN ids in a robot config.
- New node behaviour gets a `Sim`/`SimHardware` (or `talos.testing`) test that
  covers the disabled and stale-target cases. Finish with `bazel test` on the
  touched packages, plus launcher `--describe-only` for graph/config changes.

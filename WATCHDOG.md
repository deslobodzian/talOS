# talos review priorities

Architecture, names and build commands are in the project AGENTS.md you already
receive; enforce them. These are the review-specific traps.

## Robot safety (raise as blocker when violated)

- Any path where a motor can keep driving after the robot is disabled, the
  hardware state is stale (`command_timeout_us`), or a target subscription is
  not `fresh()`. Actuation must be gated on `robot_enabled(node)` and freshness.
- Bypassing the devices-layer flush (writing `/hw/request/<node>` directly,
  sending hardware requests from outside `update()`).
- Edits to physical CAN ids or `commissioned = true` in a robot config.
- Blocking, allocating-in-a-loop, or unbounded work inside `update()` that can
  overrun `period_us`.

## Contract drift (concern)

- A node-API change missing its `talOS/node_api` C ABI or `talOS/python/talos`
  mirror, or a device type missing one of hardware / devices / devices.py / RIO.
- Two writers on one topic instead of `/<sub>/target/<source>` plus an arbiter.
- A message changed in size or layout without every C++/Python/Studio consumer
  updated (Studio TS bindings come from `pnpm generate`).
- `talOS/` gaining a dependency on a robot, an example, or `studio/`.

## Verification gaps (concern)

- New node behaviour without a `Sim`/`SimHardware` test covering disabled and
  stale-target cases.
- Topic/node/config changes not checked with launcher `--describe-only`.
- RoboRIO program edits without `./gradlew phoenixTest` / `buildRio`.

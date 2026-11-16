# Bench simulation: soak and chaos test

```sh
bazel test --test_output=errors --test_env=TALOS_SOAK_SECONDS=180 \
  //bench/main_processor/simulation:soak_test
```

`soak_test` is tagged `manual` (`bazel test //...` and `:all` skip it); the
[soak workflow](../../../.github/workflows/soak.yaml) runs it for 180 s of
teleop on Linux for `hardening/**` and `feature/**` pushes (seed 0) and for
pushes to `main` (seed 1), and on macOS nightly (seed 1, `nightly.yaml`).

Its graph is `soak.toml`: the bench's
[`robot_physics_sim.toml`](../configuration/robot_physics_sim.toml) graph
(the C++ `swerve_module` owner at 5 ms, the Python `motor` owner at 20 ms, the
Python `gyro` observer, the framework `teleop` producing both lanes, and
`telemetry`) with a `[studio]` table, so the launcher also supervises the
Studio bridge. It runs against the physics simulator
([`sim_gateway --physics`](../../../talOS/simulation/README.md)) with
ephemeral ports and its own `TALOS_SHM_NAMESPACE`, which its first line
prints. It holds teleop for `TALOS_SOAK_SECONDS` (3 to 600; the test refuses
more, so a run fits Bazel's 900 s `long` timeout) with the stick centred for
the first 0.5 s of every 3 s cycle and driving the rest (the swerve module's
hold-to-run button and drive axis, the motor's voltage axis). Each cycle,
1.8 s in, injects one fault. The first is always an owner restart under a
frozen producer: `teleop` is `SIGSTOP`ped, `swerve_module` `SIGKILL`ed, and
`teleop` resumed 300 ms later. Every later cycle either `SIGKILL`s a node
(`swerve_module`, `motor`, `teleop`, `telemetry`, `hardware_node`) or
`SIGSTOP`s one for 300 ms (`teleop`, `swerve_module`, `motor`, `telemetry`,
`hardware_node`, Studio bridge), in an order shuffled by `TALOS_SOAK_SEED`
(default 0); 36 s covers every kind once. From the recording it checks, over
the whole run:

- each actuator owner's request gaps stay within its bridge owner timeout
  except across a fault on that node, and a killed owner sends again within
  5 s;
- no request actuates under a disabled Driver Station view, and no
  `swerve_module` request actuates without a received active teleop target
  published after its latest crash and younger than `command_timeout_us`;
- across a stalled `teleop`, every `swerve_module` request is neutral from
  `command_timeout_us` after the newest active target it received (across a
  stalled `hardware_node`, the newest `/hw/state`), not after the `SIGSTOP`,
  and the first of them was due (`event_ms`) before that expiry plus its
  `period_us`; a stalled `swerve_module` is neutralized in `/hw/command`
  within its owner timeout plus a period of its last request;
- `swerve_module`, restarted under the frozen producer, sends more than five
  requests before `teleop` resumes, all neutral from the first and at most
  30 ms apart. The restart is faster than `command_timeout_us`, so the lane's
  newest target is still young enough for `Target::get` at the first
  request; the requests are neutral because a restarted node's subscription
  receives only what is published after it attaches. The run prints the
  restart latency and that target's age;
- the launcher exits 0, nothing is quarantined, and each node's
  `restart_count` equals the number of times it was killed.

It prints each owner's request-gap p99 and maximum. Failure artifacts are
retained in the test's undeclared outputs, and a failure prints the tails of
the launcher's and the Studio bridge's logs (the bridge's own exit reason is
only in its log). The CI job also records the host once a second during the
test (`vm_stat`/`vmstat`, `top`, and on macOS the busiest threads' priority
and scheduling policy from `ps -M`) into `host/` beside the test's logs, kept
with a failed session.

`--sim` does not request realtime scheduling, so on macOS the soak sets
`TALOS_REALTIME=1`, the production policy (see the node development guide's
realtime scheduling paragraph), and fails if the launcher log reports
`talos: realtime scheduling incomplete`, so a silent fallback cannot pass.
Linux runs keep timeshare: they pass without it, and `SCHED_FIFO` has no
per-thread fail-safe.

After editing `soak.toml`, regenerate its manifest:
`bazel run //tools:sync_manifest -- bench/main_processor/simulation/soak.toml`.

# Bench simulation: soak and chaos test

```sh
bazel test --test_output=errors --test_env=TALOS_SOAK_SECONDS=180 \
  //bench/main_processor/simulation:soak_test
```

`soak_test` is tagged `manual` (`bazel test //...` and `:all` skip it); the
[soak workflow](../../../.github/workflows/soak.yaml) runs it for 180 s of
enabled cycles on Linux for `hardening/**` and `feature/**` pushes (seed 0) and
for pushes to `main` (seed 1), and on macOS nightly (seed 1, `nightly.yaml`).
`robot_soak_graph_test` runs `--describe-only` on the soak config in regular
`bazel test //...`.

Its graph is [`testdata/robot_soak.toml`](testdata/robot_soak.toml) (including
[`robot.toml`](../configuration/robot.toml) with `simulation_only = true` and
`[robot.targets] autonomous = ["auto"]`): two 5 ms C++ actuator owners
(`swerve_module` and `motor`), the Python `gyro` observer, the framework
`teleop` producing both teleop lanes, the test-only 20 ms C++ `auto_producer`
([`auto_producer.cc`](auto_producer.cc)) producing both autonomous lanes, and
`telemetry`, launched with `--start-studio-bridge`, so the launcher also
supervises the Studio bridge with every `[studio]` key at its default. It runs
against the physics simulator
([`sim_gateway --physics`](../../../talOS/simulation/README.md)) with
ephemeral ports and its own `TALOS_SHM_NAMESPACE`, which its first line
prints.

It runs `TALOS_SOAK_SECONDS` (3 to 600; the test refuses more, so a run fits
Bazel's 900 s `long` timeout) of enabled 3 s cycles framed by 2 s disabled
lead/tail legs. For runs of four or more cycles (including 42 s in prepush and
180 s in CI), the schedule passes through an initial teleop leg, a 1 s disabled
transition, an autonomous leg (~20% of enabled time, at least 6 s, or 9 s at
36 s+), a 1 s disabled transition, and the remaining teleop leg. During teleop
cycles the stick is centred for the first 0.5 s and driving the rest (the
swerve module's hold-to-run button and drive axis for ~5.56 rps, the motor's
voltage axis for ~1.11 V after deadband rescaling); during autonomous
`auto_producer` drives `swerve_module` at 4.0 rps and `motor` in velocity mode
at 8.0 rps.

Each cycle, 1.8 s in, injects one fault. The first is always an owner restart
under a frozen producer: `teleop` is `SIGSTOP`ped, `swerve_module` `SIGKILL`ed,
and `teleop` resumed 300 ms later. Every later cycle either `SIGKILL`s a node
(`swerve_module`, `motor`, `teleop`, `auto_producer`, `telemetry`,
`hardware_node`) or `SIGSTOP`s one for 300 ms (`teleop`, `auto_producer`,
`swerve_module`, `motor`, `telemetry`, `hardware_node`, Studio bridge), in an
order shuffled by `TALOS_SOAK_SEED` (default 0), with both killing and stopping
`auto_producer` guaranteed during the autonomous leg. From the recording it
checks, over the whole run, reading each owner's runs from their own event logs
(restart `k` writes `<name>.restart<k>.tlog`), so a request is judged by the
inputs and crash of the process that sent it and never by when the harness read
its clock:

- both 5 ms C++ actuator owners' request gaps stay within their bridge owner
  timeout except across a fault on that node, and a killed or stalled owner
  sends again within 5 s;
- `swerve_module`, restarted under the frozen producer, sends more than five
  requests before `teleop` resumes, all neutral from the first and at most
  30 ms apart. It asserts the regime that makes this evidence: the lane's
  newest target, the frozen producer's last and an active one, is at most
  `command_timeout_us` less four of the owner's 5 ms periods old at the
  restarted process's first request with an enabled Driver Station view, so
  `Target::get` would still accept it by its age for at least those four
  ticks. The requests are then neutral only because a subscription delivers
  only what is published after it attached
  ([ARCHITECTURE.md](../../../ARCHITECTURE.md#targets)). A restart too slow
  for that regime fails the run as no evidence, naming the restart latency
  and the target's age, which a passing run prints;
- across the mode-generation handoffs (`teleop` → `disabled` → `autonomous` →
  `disabled` → `teleop`), both `swerve_module` and `motor` produce active
  autonomous-driven requests during autonomous (with no teleop leakage) and
  active teleop-driven requests in both teleop legs (with no autonomous
  leakage);
- the launcher exits 0, nothing is quarantined, and each node's
  `restart_count` equals the number of times it was killed. Per-tick
  disabled-neutral, stall-to-neutral and fresh-target bounds are proven at
  component tier (`talOS/devices`, `talOS/bridge`, `talOS/hardware`) and are
  not re-asserted here.

It prints each owner's request-gap p99 and maximum and the mode-handoff active
request counts. Failure artifacts are retained in the test's undeclared outputs,
and a failure prints the tails of the launcher's and the Studio bridge's logs
(the bridge's own exit reason is only in its log). The CI job also records the
host once a second during the test (`vm_stat`/`vmstat`, `top`, and on macOS the
busiest threads' priority and scheduling policy from `ps -M`) into `host/`
beside the test's logs, kept with a failed session.

`--sim` does not request realtime scheduling, so on macOS the soak sets
`TALOS_REALTIME=1`, the production policy (see the node development guide's
realtime scheduling paragraph), and fails if the launcher log reports
`talos: realtime scheduling incomplete`, so a silent fallback cannot pass.
Linux runs keep timeshare: they pass without it, and `SCHED_FIFO` has no
per-thread fail-safe.

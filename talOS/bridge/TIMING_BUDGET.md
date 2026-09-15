# Bridge Timing Budget and Performance Notes

Perf items 41/44/45/46/47/48. All rates below are read from the code and
config cited, not assumed. The hardware core and backend files
(`talOS/hardware/`) are read-only for this item; backend changes are listed
as follow-ups, not made here.

## 41. Timing budget

| Stage | Rate / deadline | Source |
|---|---|---|
| Gateway sampling (`period_us`) | 200 Hz / 5 ms default, range 1–20 ms | `talOS/hardware/config.h`, `talOS/configuration/config_parser.h` (`period_us` in `[1000, 20000]`) |
| Gateway command expiry (`command_timeout_us`) | 100 ms default, range `[2 * period_us, 500000]` | `config_parser.h` (parser enforces the floor) |
| Bridge tick | 1 kHz explicit deadline (`kTickPeriodUs`) | `talOS/bridge/node.h`, `Run()` in `node.cc` |
| Bridge command publication | change-driven + heartbeat `command_timeout_us / 5` clamped to 5–20 ms (20 ms at defaults) | `node.cc` (`ShouldSendCommand`) |
| Drivetrain / odometry / shooter requests | 200 Hz / 5 ms | `2026-robot/main_processor/configuration/subsystems/{drivetrain,odometry,shooter}.toml` |
| Telemetry watch | 100 Hz / 10 ms | `.../subsystems/telemetry.toml` |
| Arbiter / intake / operator input / driver station | 50 Hz / 20 ms | `.../subsystems/{arbiter,intake,operator_interface,driver_station}.toml` |
| Subsystem timeout (bridge neutralizes) | 2x subsystem period | `node.cc` (`timeout_us = 2 * tracker.period_us`) |
| Controller-transition invalidation | reboot, config change, epoch bump, enabled/fault edge | `node.cc` (`transition` check; gateway advances epoch on disable/re-enable/fault recovery in `gateway.cc`) |
| Introspection publish | 4 Hz / 250 ms | `node.cc` (`Run()`) |

Budget reasoning: the fastest control path is gateway sample (5 ms) →
subsystem request (5 ms) → bridge merge (1 ms tick) → UDP command. The
gateway expiry (100 ms) is 20x the sample period, so one missed sample never
expires a live command; the bridge heartbeat (20 ms) refreshes inside the
expiry with 5x margin. Operator input at 50 Hz is the slowest control input
and still 2x inside the expiry.

## 42. Measuring the command path

`talOS/bridge/command_path_metrics.h` records, per bridge:

- `tick_us`: Tick() wall processing time (p50/p99/max over a 256 window).
- `state_gap_us`: bridge-clock inter-arrival of accepted states.
- `request_gap_us`: bridge-clock inter-arrival of decoded subsystem requests.
- `send_gap_us`: bridge-clock intervals between command sends.
- `signal_age_us`: Phoenix per-signal ages, verbatim.
- `missed_deadlines`, `suppressed_sends` counters; `tick_overruns()` on the node.

Cross-clock rule: the gateway clock (`State::sample_time_us`,
`Command::observed_time_us`) and the bridge clock (`Tick(now_us)`) have no
known relationship. Nothing in the bridge differences them. Signal ages are
already durations at poll time, so recording them is not a cross-clock
subtraction; the request's `observed_time_us` is adopted verbatim into the
merge and expiry is checked on the gateway clock only (`Gateway::Accept`).
Publish-to-merge latency is therefore bounded, not measured: with LATEST
reads every 1 ms tick and subsystem periods >= 5 ms it is at most one tick
period plus jitter, and the components of that bound are the histograms
above. Read them via `HardwareNode::metrics()` or the benchmark printout.

## 43. Bridge scheduling

Before: `Run()` did work then `sleep_for(1 ms)` — correct on average but
drifting (work time accumulates) and sending every tick while active
(~1000 Hz of UDP commands against a 200 Hz gateway).

After: `Run()` schedules against an explicit per-tick deadline
(`sleep_until(next_tick += 1 ms)`); an overrun skips one sleep, re-anchors
the deadline (no catch-up spiral), and is counted, not hidden. Publication
is change-driven with the bounded heartbeat from the table above.

Preserved: subsystem 2x-period timeouts and neutralization (a timed-out
merge differs, so it always sends immediately), controller-transition
invalidation (resets the send state so the first post-transition merge
always goes out), gateway expiry semantics (untouched, read-only), and the
introspection cadence. A live link sends at the sample rate (200 Hz, one
send per new observation) instead of the tick rate (1 kHz); a stalled link
falls back to the heartbeat (50 Hz at defaults). The benchmark's
`suppressed` vs `commands` columns show the measured ratio (about 16:1:
every sample sends once, the four idle ticks per sample suppress).

## 44. CAN frequencies and freshness

Per [CTRE status signals (stable)](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html),
every signal rate is set with `setUpdateFrequency()` /
`SetUpdateFrequencyForAll()`, `0 Hz` disables a signal, and bus utilization
above ~90% causes indeterminate behavior (less of a concern on CANivore CAN
FD). Control feedback is fast, diagnostics slow:

- Fast (control feedback): drive/steer position and velocity at the gateway
  sample rate (200 Hz default, `status_hz`), matching the repo's current
  "setpoints and telemetry at 200 Hz" operating point (`hardware/README.md`).
  The backend configures these rates explicitly and uses one-shot control
  requests.
- Slow (diagnostics): supply voltage, currents, and other health signals at
  a fraction of that, down to 0 Hz when unneeded (the CTR doc's own example
  disables supply-voltage reporting).

Freshness rules, same source: prefer `refresh()` / `refreshAll()` over
repeated getters; use `waitForUpdate(timeout)` instead of polling when the
loop must block for fresh data; use `getTimestamp()` / `getAllTimestamps()`
for the Phoenix-best timestamp and `getLatencyCompensatedValue()` for
compensation (inputs must be refreshed first); sample time-critical signals
together and `waitForAll()` to cut multi-device (e.g. swerve odometry)
latency.

What the gateway already carries: per-signal `position_age_us` /
`velocity_age_us` / `age_us` on every sample, plus gateway-clock
`sample_time_us`; `State` documents that samples are cached Phoenix
measurements at poll time, not a synchronized CAN capture, and that clock
sync with vision stays an estimator task (`hardware/README.md`, runtime
contract). Follow-up (backend-owned, not done here): audit the Phoenix
backend's per-signal rates against the fast/slow split, confirm measured
bus utilization on the real bus, and confirm `ConfigurationId`-gated
commands still meet the budget after any rate change.

## 45. Bounded work under overload

- Bridge UDP receive: at most 32 frames per tick (`node.cc`). A burst
  delays the merge; it cannot stall it.
- Gateway endpoint UDP receive: at most 16 frames per tick
  (`hardware/endpoint.h`, read-only).
- Subsystem IPC: one read per tracker per tick with
  `OVERWRITE_OLDEST`/`LATEST` (`node.cc` `Open()`), so a fast publisher
  overwrites rather than queues; a stopped subscriber is lapped, never
  blocks the publisher (see `talOS/ipc/README.md`).
- Ring capacity: `RTMS MAX_SLOTS` is 1024 (`talOS/rtms/rtms.h`);
  `Subscriber::read_next()` reports per-message `sequence`/`dropped` so a
  consumer can detect laps deterministically.
- Log buffers / slow consumers: log and Studio readers use the same
  overwrite-oldest topics (`/hw/command`, `/hw/state` in the benchmark), so
  a slow logger or viewer degrades its own freshness, not the control path.
  No unbounded in-memory log queue exists on the bridge send path.

## 46. Profile before optimizing

No optimization in this item beyond removing needless sends, and that change
is justified by counting (1000 Hz sends vs 200 Hz consumer, both in this
file). Small fixed-size copies stay simple on purpose: `Command` arrays are
bounded (`kMaxMotors` 16, digital outputs 4, PWM 4) and merged with plain
loops; the histogram is a 256-ring plus copy-and-sort. If a future profile
(`bridge_perf_test` benchmark row) shows Tick cost matters, optimize from
that number, not before.

## 47. Realistic-load benchmark

`bazel test //talOS/bridge:bridge_perf_test --test_output=all` runs, among
unit tests, `RealisticLoadBenchmark`: all actuator nodes publishing at their
configured periods (5 ms drive, 20 ms roller), gateway states at 200 Hz over
loopback UDP, plus logging-style and Studio-style shared-memory readers, for
400 cycles over 2 synthetic seconds. Each cycle spins Ticks at fixed
synthetic time until the state is ingested before any request cites it, so
requests cite only observed samples even though the synthetic loop outruns
the kernel (see the test comment). It prints one `[benchmark]` line: config fingerprint
(`ConfigurationId`), subsystem periods, states/commands/suppressed counts,
tick/state-gap/signal-age p50/p99/max, and reader counts. File that line
with the build identity for the run:

- `git rev-parse --short HEAD`
- `bazel version` (plus exact test invocation and flags)

Coverage map: companion load = the bridge tick loop itself; all nodes = one
publisher per actuator subsystem at its real period; logging = the command
reader; Studio = the state reader. The generous `tick max < 20 ms` assertion
is a stall tripwire, not a performance claim; compare rows across runs
instead of asserting absolute numbers in CI.

## 48. Scheduling privileges

None requested, none present: a repo-wide search finds no
`sched_setscheduler` / `SCHED_FIFO` / `SCHED_RR` / `setpriority` / `mlock`
in `talOS/bridge`, `talOS/hardware`, or `talOS/process`. The bridge runs at
normal priority and stays there. The evidence bar for ever changing that is
a benchmark row showing sustained `missed_deadlines` / `tick_overruns` with
p99 tick time near the 1 ms period on the target host — until then,
privileges would only hide the scheduling facts the metrics above expose.

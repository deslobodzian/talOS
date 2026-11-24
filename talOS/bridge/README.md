# Hardware Bridge (`talOS/bridge`)

`//talOS/bridge:hardware_node` is the main-processor courier between the robot's
nodes (shared-memory topics) and the controller processor (RoboRIO or
`sim_gateway`, over UDP). It merges each actuator-owning subsystem's
`/hw/request/<node>` into one `hardware::Command`, enforces identity, ownership,
and per-owner timeout neutralization (`talOS/hardware/merge.h`), forwards `/hw/state` and
`/hw/state/driver_station`, maintains the gateway-to-node clock mapping
(`talOS/hardware/clock_offset.h`), and publishes link and controller health on `/hw/status`
(`talOS/hardware/status/bridge_status.fbs`).

## Why `HardwareNode` Composes `WakeChannelWait` Instead of `RealtimeEventLoop`

Both `HardwareNode` and `RealtimeEventLoop` sleep on an OS `Poller` waiting for
cross-process `WakeReceiver` notifications from RTMS topics, and both need the
exact same wake-receiver initialization, polling fallback, per-reader
`StallGuard`, `drain()`, and `WakeLoss` accounting. That shared logic lives once
in [`talOS/events/wake_channel_wait.h`](../events/wake_channel_wait.h), composed
by both loops.

`HardwareNode` deliberately keeps its own outer loop (`Run()` / `Pass()` /
`Tick()`) rather than running on `RealtimeEventLoop` for four architectural
reasons:

1. **Dynamic UDP descriptor lifecycle.** `RealtimeEventLoop::watch_fd()` fixes
   its watched file descriptors once before `run()` starts. `HardwareNode`
   defers opening a pinned `local_address` that is not yet on the host, closes
   its UDP socket on permanent send/receive errors
   (`SendErrorNeedsNewSocket` / `ReceiveErrorNeedsNewSocket`), removes the dead
   fd from `poller_`, and reopens and re-adds a new fd on the `kReopenRetryUs`
   cadence without restarting the loop.
2. **Single-pass merge and seven dynamic deadlines.** Every pass runs one
   ordered `Tick()` (receive UDP frames up to `min(kReceiveBudget, remaining
   window datagram budget)`, read every request topic, enforce trust gates and
   owner timeouts, send one merged command if due, publish `/hw/status`). Rather
   than arming and re-arming separate event-loop timers on every state change,
   `NextDutyUs()` solves the exact next wake instant directly from current state
   across seven dynamic deadlines: (1) immediate completion of a pending echoed
   clock probe, (2) closed-socket reopen retry, (3) next periodic clock probe,
   (4) unanswered clock-probe retry, (5) `/hw/status` period, (6) each merged
   owner's `ExpiresAtUs()`, and (7) `CommandDueUs()` (immediate on merge change,
   else command heartbeat), with `kRetryUs` backoff on failed sends and `Pass()`
   bounding waits by liveness, introspection, and `max_wait_us_`. On clean loop
   exit, `Run()` calls `SendShutdownNeutral()` before `CloseWaits()` so a
   gateway with a fresh ready State matching `config_id_` immediately receives
   a neutral command stamped with the latest observed identity and sample time.
3. **`kBridge` CPU-budget throttling.** Because `hardware_node` runs in the
   highest realtime tier (`LoopTier::kBridge`, above control nodes on Linux),
   a UDP or request flood would starve control loops if the poller stayed
   immediately readable. `Pass()` resets the `kCpuWindowUs` (1 ms) window before
   each tick, caps `Tick()`'s UDP receive loop to the remaining `kDatagramBudget`
   (64) so no 1 ms window exceeds 64 received datagrams, and enforces
   `kPassBudget` (8); when either is spent, `Run()` uses `sleep_for`
   (`wait.ignore_input`) to sleep through input until the window ends or a timed
   duty falls due—a mode `RealtimeEventLoop` does not have.
4. **Selective `.tlog` recording and synthetic-time unit tests.**
   `HardwareNode` records only its outbound `/hw/state`, `/hw/command`, and
   `/hw/status` `SEND`s (omitting `/hw/state/driver_station` and raw UDP
   reads), and its component and integration tests (`trust_test.cc`,
   `hardware_node_test.cc`) drive `Tick(now_us)` and `Pass(now_us)` directly
   with deterministic synthetic microsecond timestamps and an in-process
   gateway (`in_process_gateway.h`), avoiding any wall-clock or event-loop
   scheduling overhead.

## `State` Ordering and Forward-Jump Gate

Every decoded `kHardwareState` frame with a nonzero `boot_id` passes through
`ClockOffsetEstimator::Observe(boot_id, sample_time_us, now_us)`
([`talOS/hardware/clock_offset.h`](../hardware/clock_offset.h)) before
`HardwareNode::HandleGatewayFrame` updates identity, freshness, active merges,
configuration pushes, or `/hw/state` (`REQ-RT-27`):

- **Retired boot or non-increasing `sample_time_us`**: dropped whole and counted
  in `/hw/status` `clock_ignored_count`.
- **Forward-jump bound and two-frame re-sync within an active boot**: once a
  boot's first `State` establishes its clock baseline
  `(anchor_sample_us, anchor_receipt_us)`, a subsequent `State` of that boot is
  accepted directly when
  `sample_time_us - anchor_sample_us <= receipt_delta + receipt_delta / 500 + 1_000_000`
  (saturating unsigned arithmetic, with
  `receipt_delta = max(0, now_us - anchor_receipt_us)`): at most
  `kMaxSampleJumpMarginUs` (1 s) of jitter / burst catch-up plus 2000 ppm
  (`1 / kMaxClockDriftDivisor`, twice the estimator's 1000 ppm relative
  oscillator drift assumption) of elapsed bridge time. A `State` that exceeds
  this bound is held in a single-slot forward-jump candidate, dropped whole,
  and counted in `clock_ignored_count` without advancing `last_sample_time_us`,
  invalidating active subsystems, or refreshing `state_age_us`. Any normal
  accepted `State` or boot change clears the candidate (a duplicate or reordered
  `State` with `sample_time_us <= last_sample_time_us` returns early before
  touching it). Only the next `State` past the bound strictly advancing past the
  candidate with non-decreasing bridge receipt time and a symmetrically
  consistent physical clock rate (both `cand_sample_delta` against
  `cand_receipt_delta` and `cand_receipt_delta` against `cand_sample_delta`
  within the same 2000 ppm + 1 s bound) re-syncs the anchor and is accepted,
  recovering an unseen-boot `SIGSTOP` / `SO_RCVBUF` tail-drop on the second live
  frame while rejecting isolated far-future timestamps (including `UINT64_MAX`)
  and inconsistent jump pairs.
- **Residual risk**:
  - A single corrupt `State` *inside* the bound (`<= 1 s + 0.2% * receipt_delta`
    ahead of the prompt anchor — e.g. up to ~1 s while streaming, ~2.2 s on the
    first frame after a 600 s outage, ~8.2 s after a 1 h outage) is accepted
    directly and blanks the link until real gateway time catches up
    (`sample_time_us > last_sample_time_us`).
  - Two far-future `State` frames past the bound that agree with each other
    within 2000 ppm + 1 s (e.g. `+1 h` followed by `+1 h + 5 ms`, with no normal
    accepted `State` or boot change between them) confirm the jump candidate and
    lock out the boot until real gateway time catches up or the gateway reboots
    — the accepted trade-off for recovering unseen-boot `SIGSTOP` / `SO_RCVBUF`
    tail-drop on the second live frame.
- **Prompt-anchor tracking**: on a normal accepted `State`, the anchor advances
  immediately only when the sample is at least as prompt as it
  (`sample_delta >= receipt_delta`), so a delayed buffered frame or drained
  burst after a pause or >500 s outage cannot pull the anchor back and reject
  live frames behind it. When sustained bridge-fast oscillator drift (or an
  unusually prompt early sample) keeps accepted frames slower than the anchor,
  the estimator tracks the most prompt accepted sample of the spell and
  promotes it once at least `kConvergedSamples` (8) accepted frames span
  `kAnchorRefreshUs` (1 s) of bridge receipt time, keeping the drift allowance
  bounded over long runs. Arbitrarily long link outages recover on the same
  boot because both clocks advance together.
- **First frame of an unseen boot**: gateway and bridge clocks are independent
  monotonic clocks with an arbitrary constant offset, so a new boot's first
  `State` establishes its baseline and resets the guard. Strict monotonicity
  within a boot never rolls back (`REQ-SAFE-11`), so a corrupt first frame of a
  brand-new boot fails safe (link goes stale, robot stays neutral) until the
  gateway reboots.

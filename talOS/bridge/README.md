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
   exit, `Run()` calls `SendShutdownNeutral()` before `CloseWaits()` so an
   active gateway immediately receives a neutral command stamped with the latest
   observed identity and sample time.
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

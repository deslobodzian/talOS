# Measurement results

This file is evidence, not history of the code: what was measured, when, on
what hardware and host, in what state, at which revision, and which
requirement or [TODO.md](../TODO.md) item it bears on. The procedures are in
[BENCH.md](BENCH.md) and [BRINGUP.md](BRINGUP.md); why the code changed is in
`git log`; what is still open is in TODO.md, whose entries link here.

Rules for an entry:

- It is added only with its date, the revision (the RoboRIO build and the
  main processor's talOS commit; `NOT RECORDED` when unknown, never guessed),
  the host and hardware, and the state (enabled or disabled, commissioned or
  not, load). Newest date first.
- It names the requirement or TODO id it is evidence for.
- It says when it is **not acceptance evidence** (BENCH.md's rules: realtime
  refused, `Source revision: NOT RECORDED`, a busy host, a session not kept).
  A busy or sleeping host's timing is never acceptance evidence.
- An entry is not rewritten later; a new measurement is a new entry.

Hosts named below: the **bench** is BENCH.md's RoboRIO (image
FRC_roboRIO_2026_v1.2, two cores) and CAN devices; the **Linux workstation**
has 8 cores; the **Mac mini** is the deployment host (M4 Pro, 12 cores).

## 2026-11-22

Mac mini (M4 Pro, 12 cores, macOS 26.3 / Darwin 25.3.0 arm64), commit
`820a06d5`, no hardware attached (`TODO.md` `MAC-1`, `INC-3`, `DEPLOY-10`):

- **MAC-1: `tools/mac_qualify.sh` full build, suite, configs, and warm spawns.**
  - `bazel build //...`: PASS (41 s); `bazel test --test_output=errors //...`:
    PASS (118 s, 100% green); all 6 `--describe-only` configs (`robot.toml`,
    `robot_py.toml`, `robot_bench.toml`, `robot_physics_sim.toml`,
    `examples/superstructure/config/{robot,robot_py}.toml`): PASS.
  - `tools/deploy.sh boot-check`: FAIL as expected under `DEPLOY-8` (`FileVault
    is On` and `pmset autorestart 0` failed; `pmset sleep 0` ok; automatic macOS
    updates enabled warning).
  - Warm Python node spawn → event-loop origin (no `sudo purge`, so warm page
    cache; startup grace 5000 ms): `gyro` 106.0 ms first launch / 131.2–133.7 ms
    restarts; `motor` 107.3 ms first launch / 120.0–121.0 ms restarts.
- **INC-3: `//talOS/launcher:restart_latency` on `swerve_module` (10/10 SIGKILLs).**
  - Idle: total **26.4 ms** median (range **25.2–31.1 ms**; `SIGKILL → origin`
    18.8 ms median, 18.0–19.9 ms).
  - Under `--disk-load 8` (8 writers rewriting and syncing 256 MiB files): total
    **11.1 ms** median (range **10.4–357.7 ms**; per kill: 15.8, **357.7**,
    10.6, 10.8, 10.4, 10.6, 36.9, 77.4, 11.3, 10.8 ms). The single **357.7 ms**
    outlier (`SIGKILL → origin` 351.0 ms) exceeds the 300 ms frozen-driver
    window even on M4 Pro, keeping `INC-3` open.
- **DEPLOY-10: `//bench/main_processor:bench_bundle` (with Studio) `SIGTERM` stop.**
  - Five consecutive runs of `bench_bundle/run.sh --sim --remote 127.0.0.1
    --studio-http-port 0` stopped with `SIGTERM`: 5/5 exited 0 in **79.7–88.9 ms**
    (median **84.2 ms**), against explicit launchd `ExitTimeOut = 90` s.

Linux workstation (8 cores), commit `820a06d5`, loaded (concurrent worker Bazel
builds running in sibling worktrees; `TODO.md` `TEST-24`):

- **TEST-24: `//studio:drive_browser_test` 200/200 PASS on loaded Linux.**
  - `bazel test --runs_per_test=200 --test_output=errors
    --test_env=TALOS_REQUIRE_BROWSER=1 //studio:drive_browser_test`: **200/200
    PASSED** (min 2.3 s, avg 2.5 s, max 3.3 s, dev 0.2 s; elapsed 506.4 s),
    zero timeouts waiting for the Studio bridge's `listening` line.

## 2026-11-21

Mac mini (M4 Pro, 12 cores, Darwin 25.3.0 arm64), commits `d942b0b8` and
`6037f7ff`, no hardware attached, idle. Measurements for `TODO.md` `LOG-8`,
`TRACE-1`, `TEST-10`, and `TEST-22`:

- **LOG-8: enabled simulation time-cut log window stays within 8/7 of the
  printed window (`REQ-OPS-32`).** At `d942b0b8`, 60 s enabled teleop physics
  simulation (`robot_physics_sim.toml` with `[robot] log_max_bytes = 33554432`,
  `--mode-schedule teleop:60`, active stick demand from t=2 s): printed
  `Event-log window 20331671 us` (`20.332 s`); all 6 logging members exited 0
  with `log_capped = true`, `log_gap = false`, `recorder_failed = false` and
  kept spans `hardware_node` `19.307 s` (`W/span = 1.0531`), `swerve_module`
  `19.273 s` (`1.0549`), `teleop` `19.268 s` (`1.0552`), `telemetry` `19.185 s`
  (`1.0597`), `gyro` `19.103 s` (`1.0643`), `motor` `18.938 s` (`1.0736`); worst
  ratio `1.0736 <= 8/7 (1.1429)`.
- **TRACE-1: macOS death and idle-timeout checks pass on the Mac mini.** At
  `d942b0b8`, `bazel test --test_output=errors //talOS/launcher:launcher_death_test //studio/bridge:client_timeout_test`
  passed (`launcher_death_test` 1.3 s, `client_timeout_test` 9.1 s). In Linux
  CI run `44855147331` (`d942b0b8`), `//talOS/protocol:udp_address_change_test`
  ran under `TALOS_REQUIRE_NETNS=1` and passed in 0.2 s (0 skips).
- **TEST-10 & TEST-22: repeated macOS runs pass with 0 failures.** At
  `6037f7ff`, `bazel test --nocache_test_results --runs_per_test=5` passed
  `//talOS/launcher:launcher_test` (20/20 shard runs, max 51.2 s),
  `//talOS/launcher:supervision_test` (15/15 shard runs, max 71.9 s), and
  `//talOS/bridge:run_timing_test` (5/5 runs, max 0.1 s).

## 2026-11-20

Mac mini (M4 Pro, 12 cores, Darwin 25.3.0 arm64), commit `1218376d`, no
hardware attached. Three consecutive uncached full suites
(`bazel test --nocache_test_results --test_output=errors //...`, 251 tests)
each started immediately after driving 1-minute `vm.loadavg` to >= 9.0
(`TODO.md` `MAC-2`, `TEST-10`):

- **MAC-2: `//bench/main_processor:bundle_test` passes 3 of 3 full suites
  started at load >= 9.**
  - Run 1 (start load `9.01 5.18 3.54`, elapsed 166 s):
    `//bench/main_processor:bundle_test` PASSED in 33.8 s (`install_check_test`
    27.1 s, `warm_up_test` 18.1 s). Two unrelated tests failed under the initial
    load spike: `//talOS/launcher:supervision_test`
    (`Supervision.HungStudioBridgeIsKilledAndRestartedWithoutDisturbingNodes`,
    6430 ms; real wall-clock deadlines, `TEST-10`) and `//tools:prepush_test`
    (TIMEOUT in 60.0 s).
  - Run 2 (start load `9.16 6.88 4.63`, elapsed 128 s):
    `//bench/main_processor:bundle_test` PASSED in 50.8 s (`install_check_test`
    42.3 s, `warm_up_test` 14.6 s); 251/251 tests passed.
  - Run 3 (start load `9.12 6.91 4.99`, elapsed 139 s):
    `//bench/main_processor:bundle_test` PASSED in 55.6 s (`install_check_test`
    26.8 s, `warm_up_test` 15.4 s); 251/251 tests passed.

Mac mini (M4 Pro, 12 cores, Darwin 25.3.0 arm64), commit `0cebbd0d`, no
hardware attached, idle (load 1.2). Measurements for `TODO.md` `SCALE-1`,
`RT-7`, `STUDIO-2`, `MAC-2`, and `INC-3`:

- **SCALE-1: `wake_fanout_bench` at 32 armed readers on macOS.**
  `bazel run -c opt //talOS/rtms:wake_fanout_bench` (2000 publications after
  200 warm-up, `MAX_READERS` 32, segment version 9): 0 armed / 32 held:
  0.00 us median, 1.00 us p99, 3.00 us max; 1 armed / 1 held: 2.00 us median,
  5.00 us p99, 16.00 us max; 8 armed / 8 held: 16.00 us median, 30.00 us p99,
  62.00 us max (2.00 us/armed); 8 armed / 32 held: 15.00 us median, 26.00 us
  p99, 29.00 us max (1.88 us/armed); 32 armed / 32 held: 71.00 us median,
  93.00 us p99, 97.00 us max (2.22 us/armed); `walk` 32/32: 0.00 us median,
  1.00 us p99, 4.00 us max.
- **RT-7: descriptor count on macOS enabled in simulation (`REQ-OPS-43`).**
  Physics simulation (`robot_physics_sim.toml`, `--sim-gateway --physics`,
  15 s teleop with stick demand, `ulimit -n` soft 256 / hard unlimited),
  sampled mid-run via `lsof -a -p <pid> -d 0-9999`: `launcher` 36 open FDs,
  `hardware_node` 9 (7 ipc/sock/kqueue), `swerve_module` 8 (6), `motor` 8 (6),
  `gyro` 7 (5), `telemetry` 7 (5), `teleop` 7 (5), `sim_gateway` 6 (5).
  `session_report --strict` passed all verdicts (first accepted State 4.16 ms,
  0 enabled gaps >= 100 ms, worst target age 2.20 ms <= 25 ms,
  `alert_warning_owner_timed_out` false across all 749 samples).
- **STUDIO-2: real bench sessions reviewed on the Mac mini (`REQ-OPS-41`).**
  `bazel run -c opt //studio/review:studio_review` against the 2026-11-17 bench
  logs (`~/talos-bench/logs/U`, `V`, `W`):
  - Session `U` (`7798953721310323380`, 300.0 s, 25 files, 182.8 MiB):
    `bounds.json` (cold `Open()`) 392.0 ms, `channels.json` 0.4 ms (243
    channels), `timeline.json` 0.3 ms (3 modes, 3 alerts, 6 runs),
    `declared.json` 0.3 ms, `snapshot` 4.2 ms (19.3 KB), `series` (1 ch, 400
    pts) 54.7 ms. Headless Google Chrome rendered `index.html` (`talOS ·
    Telemetry Studio`, HTTP 200) against `127.0.0.1` serving `U`.
  - Session `V` (`17289576640307273572`, 1094.6 s, 66 files, 580.1 MiB,
    `log_capped`): `bounds.json` 1302.3 ms, `channels.json` 0.4 ms (243 ch),
    `timeline.json` 0.2 ms (23 modes, 6 alerts, 7 runs), `snapshot` 11.8 ms,
    `series` 164.2 ms.
  - Session `W` (`1111857340715075397`, 1178.1 s, 68 files, 720.5 MiB,
    `log_capped`): `bounds.json` 1626.7 ms, `channels.json` 0.4 ms (231 ch),
    `timeline.json` 0.3 ms (1 mode, 2 alerts, 7 runs), `snapshot` 14.2 ms,
    `series` 217.4 ms.
- **MAC-2: fresh `bench_bundle` install and first-launch timing (`REQ-OPS-47`).**
  Fresh `tools/deploy.sh install` of `//bench/main_processor:bench_bundle` into
  a temporary prefix took 4.05 s (including warm-up); first
  `run.sh --sim --describe-only` took 0.13 s (against the 15 s probe deadline);
  first `run.sh --sim --duration-s 5` took 5.22 s total (0.22 s startup
  overhead).
- **INC-3: `restart_latency` baseline on Mac mini (`swerve_module`, physics sim).**
  Idle (10 kills): SIGKILL to first `/hw/request/swerve_module` median 26.2 ms
  (range 24.8–31.2 ms; Python `gyro`/`motor` warm spawn-to-origin 84.9–123.9 ms).
  Under `--disk-load 8` (10 kills): median 10.6 ms (range 10.1–40.5 ms; Python
  79.3–124.2 ms).

Mac mini (M4 Pro, 12 cores, macOS arm64), commit `1e3bb1c8`, no hardware
attached, idle. Verification of `TODO.md` STUDIO-1 macOS bridge listener and
idle-timeout tests:

- **STUDIO-1: exclusive port bind and client idle-timeout tests pass on macOS.**
  `bazel test --nocache_test_results --test_output=errors //studio/bridge:listen_test //studio/bridge:client_timeout_test`
  passed with 0 skips (`listen_test` 0.2 s, confirming a second listener on an
  in-use port fails on macOS; `client_timeout_test` 8.9 s, confirming an
  unresponsive client frees its slot within the 12–16 s idle-timeout window).

Mac mini (M4 Pro, 12 cores, macOS arm64), commit `6dd79706`, no hardware
attached. Investigation of nightly `Bazel (tsan, macOS)` failures (`TODO.md`
CI-13) and kept CI logs from Nightly runs `43877022665` (`6c701709`) and
`44122679143` (`d702c99d`).

- **CI-13: zero ThreadSanitizer race reports; mechanism is ~100x macOS TSan
  fork/exec kernel overhead.** Neither nightly run's logs contain any
  `WARNING: ThreadSanitizer` finding. Standalone Apple clang measurements on
  the Mac mini: `fork()` from a 3-thread process took 120 ms under TSan vs
  0.5 ms plain; `fork` + `exec /usr/bin/true` took 213 ms vs 3 ms; 10 empty
  `main` starts took 1.94 s (mostly kernel CPU) vs 0.02 s; 24 concurrent empty
  `main` starts consumed 10.7 s kernel CPU in 0.96 s wall. Starting
  `no_state_sim` took 0.9 s plain vs 2.9 s TSan idle, and 1.9 s plain vs
  12.6 s TSan for 3 concurrent sessions pinned to efficiency cores
  (`taskpolicy -c background`).
- **CI-13: 3-of-3 reproduction of Nightly `44122679143` failures.** Running
  `bazel test --config=tsan --nocache_test_results --runs_per_test=3
  --local_test_jobs=3 '--run_under=taskpolicy -c background'` reproduced the
  exact CI failure messages 3 of 3 on `//talOS/build:sim_check_no_state_test`
  (14 s ready bound exceeded after the case's 4 s sleep) and
  `//tools:scaffold_sim_test` (15 s ready wait exceeded across 6 members), and
  pushed `//talOS/launcher:log_dump_test` (18 subprocess spawns, 60 s small
  limit) to 48 s. On the idle 12-core Mac mini all three passed
  (15.6 s, 24.2 s, 16.0 s). Because the Linux sanitizer lanes still run all
  three targets, they take `no_macos_sanitizer` alongside the other
  session-spawning tests in `talOS/launcher/BUILD`.

## 2026-11-17

Bench, RoboRIO build c6c04a67 with companion 10.0.0.37, Mac mini (12 cores,
Darwin 25.3.0, load 1.3 to 1.6, two users logged in) as main processor over
Ethernet, bench bundle of c6c04a67 (`git_dirty = false`), launched over ssh,
uncommissioned throughout. Driver Station on the Linux workstation, program
NOT RECORDED, gamepad in slot 0. Sessions
`~/talos-bench/logs/U/7798953721310323380` (300 s, no Studio) and
`~/talos-bench/logs/V/17289576640307273572` (with the Studio bridge). Not
acceptance evidence for BENCH-8: uncommissioned, launched over ssh, and not
the head revision.

- **BENCH-11: enabled while uncommissioned, nothing drives.** Session U,
  Teleoperated, enabled from 131.7 s to 169.9 s (7649 States). The teleop
  lanes went active (`/motor/target/teleop` `voltage_frac` -1 to 1,
  `velocity_rps` to 30, `torque_current_a` to 2). `/motor/state` stayed
  `NEUTRAL`, `source: NONE`, `applied_voltage_v` 0 in all 59999 records and
  no `/motor/state` or `/swerve_module/state` record shows a drive mode. The
  RoboRIO console counts 0 accepted commands in every 10 s report and about
  500 refused. `alert_error_uncommissioned` was active for the whole
  session. The person at the bench saw no motion.
- **BENCH-11: joystick numbering as the RoboRIO reports it.** Session U,
  joystick 0: axis 1 ranged -1 to 1; buttons 1 to 10 were each seen.
- **BENCH-3: a stick held through a disable and enable.** Session V, 796.4 s
  to 813.1 s: six enables with axis 1 already held (five at -1, one at 1),
  each 1.1 to 2.0 s, with a disable between. The teleop node published
  nothing on `/motor/target/teleop` in any of them and `/motor/state` stayed
  `NEUTRAL`, `NONE`. In the next enable (814.5 s to 815.7 s), entered with
  the stick at neutral, the lane was written again (253 sends, none active:
  the stick stayed inside the deadband). A lane going active on a
  deflection after a neutral reading was seen earlier in the same session
  (28.5 s to 33.4 s), not after the held spells.
- **BENCH-3 (a): the persisted config after a reboot.** One minute after a
  RoboRIO boot, `/home/lvuser/talos_config.bin` was present (1009 bytes)
  and the console printed `hardware gateway config=9594024380816546967
  commissioned=0 (waiting for config push)`.
- **BENCH-3 (c): console.** Over session U the RoboRIO console shows no
  WPILib joystick warning; `companion connected` and `companion lost` carry
  the `[s.ms]` prefix. What the Driver Station's own message window showed
  was not recorded.
- **Link and realtime.** Session U: Realtime `full` on all six nodes and no
  grant warning; 60000 States, `/hw/state` interval median 4.999 ms, p99
  5.168 ms, max 6.283 ms (enabled: 4.999, 5.171, 5.354 ms); no recording
  loss; `timeouts=0` for both subsystems.
- **RESET-3: no bus silence in 300 s.** Session U: feedback age max 8.7 ms
  for the three Talon FX (firmware 26.3.1.0) and 3.9 ms for the CANcoder
  and Pigeon 2, over 60000 States each. The silence was seen every few
  minutes on 2026-11-11, so one 5-minute session does not show it is gone.
- **TIME-4: the refused-command age moved by one tick within a session.**
  Session U, RoboRIO console `[rio] latency:` lines, robot disabled: from
  `[102]` to `[182]` refused command age p50 10.7 to 10.8 ms, p99 15.8 to
  16.1 ms; from `[192]` on p50 5.8 ms, p99 10.8 to 11.0 ms, max up to
  15.7 ms; no restart, no mode change at that time. Session V at `[1213]`:
  5.8 and 10.9 ms. REQ-RT-26's bound is on accepted commands (11 ms p99,
  15 ms max); none was accepted here.
- **TIME-4: the two bands alternate with the clock drift.** The same
  RoboRIO boot's console over 70 minutes (sessions V, W and two short
  ones), every 10 s line: refused command age p50 is either 5.8 to 6.0 ms
  or 10.7 to 10.8 ms. In session W it read 10.7 ms from `[751]` to
  `[1512]` (761 s), 5.8 ms to `[1712]` (200 s), then alternated window by
  window to `[1812]` and stayed at 10.7 ms: one cycle of 961 s. The
  measured clock drift (-5.17 ppm, session U) sweeps one 5 ms period in
  967 s, so the band follows the node timers' phase against the RoboRIO
  tick, as TIME-4 derived. p99 in the 10.7 ms band was 10.9 to 11.1 ms.
  Disabled, the bridge sends on its 20 ms heartbeat (about 500 commands
  per 10 s), so the share of time in each band here is not the enabled
  path's.
- **A command 11.6 s old after the cable returned.** Session W's window at
  `[741]`, after the reconnect at `[739.2]`: refused command age max
  11555743 us, 119 commands in the window. Refused here as every command
  was (uncommissioned); what an enabled, commissioned RoboRIO does with it
  was not exercised.
- **E-stop and a RoboRIO reboot under a running session.** Session V: the
  Driver Station showed `estop` from 518.4 s, robot disabled. A RoboRIO
  reboot at 569.5 s (`/sbin/reboot` over ssh): the next boot's first packet
  read `estop` false and the one 30 ms later true again, the Driver Station
  still sending it. The bridge accepted the new boot (boot id
  15957309938990792307) with no session restart; `configured` true again.
  `estop` read false after the Driver Station program was restarted
  (detached 727.0 s to 763.5 s) and the RoboRIO was rebooted a second
  time; which of the two cleared it was not separated.
- **RT-17, BENCH.md "The Mini's link": a pulled companion cable with the
  Mini's Wi-Fi on does not recover.** Session V, Wi-Fi `en1` on
  (10.0.0.25, same subnet). First pull: RoboRIO console `companion lost`
  at `[401.9]`, `companion connected` at `[412.4]` (10.4 s). Second loss at
  `[507.3]`: no recovery in over 80 s with the cable back. The bridge's
  socket then read `10.0.0.25:5803->10.0.0.72:5802` (`lsof`), its sends
  succeeded, `/hw/status` `state_age_us` grew past 82 s with
  `alert_error_link_lost`, and the RoboRIO (built with companion
  10.0.0.37) accepted nothing. Only stopping the session ended it.
- **The same pull with the Mini's Wi-Fi off recovers.** Session
  `~/talos-bench/logs/W/1111857340715075397` (1800 s, Studio bridge,
  disabled, uncommissioned, same builds), Wi-Fi off
  (`networksetup -setairportpower en1 off`): `companion lost` at `[728.0]`,
  `companion connected` at `[739.2]` (11.1 s), the bridge's new socket on
  `10.0.0.37` (`socket_reopens=1`, `send_failures=1`), no node restart.
- **BENCH-3: the Driver Station's network pulled while enabled.** Session V:
  twice (832.9 s and 866.9 s) the Driver Station view went from enabled to
  detached, mode none, disabled, and returned attached and disabled (1.7 s
  and 20.7 s later). The instant of each pull was not recorded, so no
  detection time is stated.
- **RESET-3: no bus silence in the 19.6 minutes session W kept.** Every
  event log reached its byte budget (`log_capped`), so the report covers
  235530 States, not the whole 30 minutes: feedback age max 10.4 ms over
  the three Talon FX, 6.1 ms for the CANcoder and Pigeon 2. Realtime
  `full` on the six nodes. `/hw/state` interval median 5.001 ms, p99
  5.220 ms. Load 1.9 to 3.1 at the end: timing not acceptance evidence.
- **RT-28: 81 consecutive States missing with both ends running.** Session
  W, one interval of 410.2 ms (the only one over 20 ms in the kept
  window, 767.7 s into it, about RoboRIO `[2083]`, some 22 minutes after
  the cable pull): `sample_time_us` stepped 410004 us, so the States were
  never published, not delayed. The RoboRIO's `tick phases` lines show
  2001 to 2003 ticks in each 10 s window around it, no `loop timing`
  report and no send failure; on the Mini every node timer's max lateness
  was 0.38 ms or less (telemetry 1.24 ms), the bridge kept publishing
  `/hw/status` (`state_age_us` max 391 ms) and counted no send failure.
  Below the 0.5 s link-loss bound, so no `companion lost`. Cause not
  established; nobody was at the bench as far as recorded.
- **TIME-5: the report's Driver Station to lane row.** Session U: median
  46742.6 ms, p99 129937 ms, 7500 samples. `teleop` published the motor's
  lane only from 131.7 s to 169.9 s (7647 sends), so every packet before
  the enable is paired with the first send at 131.7 s.
- **RT-5: RoboRIO thread scheduling, read from `/proc`, disabled.**
  `sched_rt_runtime_us` -1, 2 CPUs. In the robot program:
  `talos_watchdog` `SCHED_FIFO` 30, the loop (main thread) `SCHED_FIFO`
  10, `talos-ds` `SCHED_FIFO` 5, all three last run on CPU 1; five
  threads `SCHED_RR` 1 to 3; `talos-companion`, `talos-canbus`,
  `talos-config`, `talos-log`, and the vendor's `can_recv`, `can_psend`
  and `can_stat` `SCHED_OTHER`. Kernel: interrupt threads `SCHED_FIFO` 15
  (`irq/54-eth0`, `irq/53` and `irq/76` USB, which carries the CANivore),
  `ksoftirqd` 8, `ktimersoftd` 1. The eth0 and USB interrupts were all
  counted on CPU 0.
- **RT-5: tick cost at two status rates.** Disabled, same bundle and
  RoboRIO build; `[robot] status_hz` 50.0 from a scratch copy of the
  bundle's config (session `~/talos-bench/logs/X1/13312505921970813578`,
  60 s), then the bundle's own 200.0 again (session
  `~/talos-bench/logs/X2/6288709441872347922`, 45 s). `can_buses[1]`
  (`Shooter`) `utilization_frac` after the first 15 s: 0.08 to 0.11
  (median 0.09) at 50, 0.16 to 0.18 (median 0.17) at 200. Three
  consecutive `tick phases` lines each, mean/max µs: at 50, refresh
  170.2/290.2, 167.8/309.6, 169.8/328.8 and devices 209.6/322.9,
  204.3/362.7, 207.6/333.2; at 200, refresh 169.7/272.3, 167.9/288.9,
  170.6/293.4 and devices 206.7/385.2, 203.6/389.0, 210.5/330.5. The
  status rate changes the bus load, not the tick's cost.
- **Studio from another build.** A dev Studio page built from 6a4bbd88
  against session V's bridge (c6c04a67) showed every node stale; the
  bridge's `/system.json` listed all seven alive and it sent a
  `talos.system_graph.update` every 0.5 s (STUDIO-12).
- **RT-28: the hole read from the kept log and the RoboRIO, the same
  evening.** Session W's `/hw/state` on each side of the 410.2 ms
  interval: `sequence` 410256 then 410338 (a step of 82: the gateway
  ticked through and 81 States are missing), `sample_time_us` 2082239941
  then 2082649945, same boot and epoch, feedback ages 4.5 to 5.3 ms after
  it (not the CAN silence of RESET-3), `counters.rejected_command_count`
  1 on both (2 from the next `/hw/status`, 20 ms later). `/hw/status` kept
  its 20 ms cadence across it with `clock_ignored_count` 0; from 767.033 s
  to 767.693 s it counted 33 commands sent, 51 States received (132 due)
  and 13 Driver Station packets (33 due). RoboRIO console: 483
  refused commands in the window ending `[2082.591]` and 496 in the next
  (about 500 each otherwise), the second with a max age of 415658 us: about
  21 commands never arrived and the first after the hole carried the
  frozen observation, so both directions were out. RoboRIO, 96 minutes
  after boot, not rebooted since: `dmesg` shows eth0 link down at
  `[401.4]` and up at `[407.95]` (the first cable pull) and no link event
  after; `Udp: SndbufErrors 0`, `RcvbufErrors 83`, `InErrors 83`; eth0 TX
  errors and dropped 0, RX errors 1. Mini: `netstat -I en0` errors 0 in and
  out, `netstat -s -p udp` 0 dropped for full socket buffers; its unified
  log returned nothing over ssh. The hole was at about 17:08:57 EST. Not
  placed further than the switch or the Mini's Ethernet side.
- **RT-27: both pulls reopened the bridge's socket.** Session V's bridge
  exit line: `send_failures=2 socket_reopens=2`; its log has one
  `cannot send (Can't assign requested address)` line and
  `sends again (2 reopened so far)`. The first reopen took 10.0.0.37 and
  the link returned; the second took 10.0.0.25.
- **TIME-5: session U re-read with the corrected row** (`session_report` as of the commit "launcher: take
  Driver Station to lane only from a producer that answers", run on the
  Linux workstation on a copy of the session): Driver Station
  to lane, teleop: 1685 samples, median 2.929 ms, p99 3.075 ms, max
  7.547 ms; 0 enabled packets unanswered, 11546 disabled.

## 2026-11-16

- **TEST-9, TEST-10, MAC-2: launcher and installing tests on macOS.** Mac mini,
  macOS, 2026-11-16. `launcher_test` alone, uncached, three runs: 10.3, 10.0,
  9.8 s; `supervision_test`: 13.5, 13.4, 13.1 s. `--runs_per_test=10` at load
  about 3: `launcher_test` 40 of 40 shard runs (max 27.9 s), `supervision_test`
  30 of 30 (max 50.7 s). Three full uncached `bazel test //...` runs after the
  installing tests stopped overlapping: 234 of 234 each, `bundle_test` 29.4,
  44.3, 42.0 s, `install_check_test` 13.4, 19.3, 21.2 s, `warm_up_test` 19.6,
  14.3, 14.4 s; one run before the change: 234 of 234, 45.2, 46.2, 38.9 s.
  `install_check_test` alone: 17.7 and 17.6 s before, 12.0 and 11.9 s after.
  The earlier timeouts at load about 9 were not reproduced.

- **RT-22: a Python worker thread delays `update()`.** Mac mini, macOS
  (Darwin 25.3), idle (load 1.25 to 1.75), talOS f8d6dcad, Python 3.11.14,
  `sys.getswitchinterval()` 0.005 s. A scratch `talos_py_node` (not kept)
  owning no output, so an observer: timeshare, not the time-constraint
  policy of a control node. Live run, `--duration-s 32`, the first 2 s
  dropped; `update()` start-to-start intervals from `time.monotonic_ns()`
  at its first line, against a worker thread started in the constructor:

  | period | worker | p50 | p99 | max | over 1 ms late |
  |---|---|---|---|---|---|
  | 5 ms | none | 5.00 ms | 5.01 ms | 5.04 ms | 0 of 5998 |
  | 5 ms | pure-Python loop | 7.51 ms | 11.91 ms | 12.51 ms | 2999 of 5988 |
  | 5 ms | blocked in `socket.recv` | 5.00 ms | 5.03 ms | 5.07 ms | 0 of 5998 |
  | 5 ms | `sorted()` of 2 M floats (224 ms) | 224.1 ms | 225.5 ms | 225.8 ms | 158 of 166 |
  | 20 ms | none | 20.00 ms | 20.02 ms | 20.06 ms | 0 of 1498 |
  | 20 ms | pure-Python loop | 20.00 ms | 21.31 ms | 22.28 ms | 20 of 1498 |
  | 20 ms | blocked in `socket.recv` | 20.00 ms | 20.02 ms | 20.05 ms | 0 of 1498 |
  | 20 ms | `sorted()` of 2 M floats (224 ms) | 224.3 ms | 225.8 ms | 226.0 ms | 183 of 189 |

  A pure-Python worker costs `update()` up to the switch interval plus part
  of a period; one C call that keeps the interpreter lock costs the whole
  call; a worker waiting in a call that releases the lock costs nothing
  measurable.

- **RIO-1: stale-FaultField flake, mechanism observed.** Mac mini, macOS
  (Darwin 25.3), desktop Phoenix simulation, load 0.5 to 1.5, talOS
  6acdcf9f; one `StaleFaultFieldClearsLimitsAndKeepsFaults` per process.
  Before the fix 8 of 40 Release and 2 of 20 Debug runs failed, every one
  at the boot row's wait (`fault_flags 0`, after the 10 s bound); at
  841a0fc0, before the torque-current work, 1 of 20 Release. Instrumented
  reads of the boot row's TalonFX: it read valid with BootDuringEnable
  from about 0.11 s after the backend was constructed until 0.51 to 0.53 s,
  then valid without it, whether driven at 2 V or neutral; the 30-read
  pump before the wait ended 0.46 to 0.53 s after construction, and the
  four instrumented failures were exactly the runs whose pump ended after
  the bit cleared. After the fix 0 of 40 Release, 0 of 20 Debug,
  `--gtest_repeat=10` of the test 10 of 10 in each, and
  `--gtest_filter='PhoenixBackend*' --gtest_repeat=5` 125 of 125 in each.

- **CI-1, CI-2, CI-9, CI-10, INC-6: first green nightly.** GitHub hosted
  macOS runners, `main` at 95f4878e, Nightly run 43905118119 (started by
  hand): Bazel under AddressSanitizer, Bazel under ThreadSanitizer and the
  macOS soak (seed 1) all passed. The ThreadSanitizer and soak jobs passed
  on a rerun: on the first attempt no hosted runner picked them up
  ("The job was not acquired by Runner of type hosted even after multiple
  attempts"), which is no result of the code (TODO.md CI-11).
- **BENCH-3: RoboRIO tests under ThreadSanitizer.** Linux workstation, gcc
  16, talOS 2e6d720c (not pushed), `./gradlew --offline phoenixTest -Ptsan`:
  the build compiles and 83 of 83 tests pass. ThreadSanitizer reported
  nothing in talOS code; it reported three data races inside CTRE's
  uninstrumented `libCTRE_PhoenixTools_Sim.so`, reached from a Pigeon 2's
  construction in `PhoenixBackend::Configure`, and two thread leaks at
  exit, so the Gradle task fails.
- **BENCH-11: disabled dry run on the Mac mini.** Bench, RoboRIO build
  6acdcf9f with companion 10.0.0.37, Mac mini (12 cores) as main
  processor over Ethernet, bundle of 6acdcf9f (`git_dirty = false`),
  disabled, uncommissioned, no Driver Station attached, launched over
  ssh, 45 s, session 124490254188690328. Controller link up, 8995 States,
  `send_failures=0`; Realtime `full` on all six nodes and no realtime
  warning; `/hw/state` interval median 4.999 ms, p99 5.182 ms, max
  7.434 ms; `/hw/status` `configured` true, `commissioned` false. Load
  before the run 4.1 (a build had just finished), so the timing is not
  acceptance evidence.

## 2026-11-15

- **BENCH-5: CAN bus status slots on the bench.** Bench, RoboRIO build
  87ce6efa, disabled, 60 s; main processor revision NOT RECORDED. Not
  acceptance evidence (BENCH-8). The bridge's slot table read `0 rio,
  1 Shooter`; slot 1 read OK at 0.18 utilization with every counter 0; slot 0
  (the RoboRIO's own bus, no devices) read OK at 0 with a transmit error count
  of 128 and one transmit-buffer-full; `alert_warning_can_bus_unread` stayed
  false. The first deploy (before 87ce6efa) reported zero buses: the gateway
  lost the count when an asynchronous configure committed, which no simulated
  backend could show.
- **TEST-26: ring tests on ARM.** Mac mini (ARM), revision NOT RECORDED:
  `//talOS/rtms:ring_handoff_test` and `:ring_lap_test` passed 20 of 20. The
  mutation that removes the torn-read guard was seen failing only on x86-64.
- **INC-7: headless Chromium gave no DevTools port.** Linux workstation,
  `tools/prepush.sh` run of 7facd73f, several agents running, load about 8:
  `//studio:drive_browser_test` failed in `setUpClass` with `TimeoutError:
  /usr/bin/chromium DevTools port: not within 60 s`; every session member
  exited 0; no `browser.log` was kept. Ten repeats with the browser required
  passed in 2.3 to 3.1 s. inotify limits 524288 watches, 1024 instances (not
  near). Busy host.

## 2026-11-14

The bench sessions of this date and 2026-11-15 are **not acceptance
evidence** (BENCH-8): the Linux workstation's bridge and control loops ran
with realtime refused (`mlockall`, `SCHED_FIFO` not permitted),
`session_report` printed `Source revision: NOT RECORDED`, and the sessions
were kept only in a scratch directory. The RoboRIO-side figures (its own log)
do not depend on that; the State-interval and log-window figures do.

- **LOG-2, LOG-8 (REQ-OPS-32): event-log window, 600 s bench session.** Bench,
  RoboRIO build 261c190e, five devices on CANivore `Shooter`, disabled; Linux
  workstation as main processor, revision NOT RECORDED; `log_max_bytes =
  33554432`. The launcher printed `Event-log window 19483373 us` (19.48 s); the five
  nodes kept 19.2 to 19.3 s and the bridge 17.25 s, so the window over the
  shortest span is 1.13 (bound 8/7) and the 2026-11-11 outlier is gone; seven
  segments per member, every member `log_capped`, none `recorder_failed`;
  11.6 MB on disk of the 32 MiB budget (35%). The host was building at the
  same time, so this run says nothing about a control loop stalling at a
  rotation.
- **BENCH-3, RT-21 (REQ-RT-24): Driver Station read with no Driver Station.**
  Bench, no Driver Station attached, 3 to 100 min uptime; revision NOT
  RECORDED. Before 7e46011e the loop called `HAL_RefreshDSData` every 5 ms, a
  blocking round trip to `FRC_NetCommDaemon` (`SCHED_OTHER`): `driver
  station` phase 0.70 to 0.88 ms mean, up to 13.5 ms, hundreds of overruns;
  about 0.12 ms with a Driver Station attached. With the `talos-ds` thread
  making those calls (7e46011e, same bench): phase 9.2 to 9.7 µs mean, 38 to
  63 µs max, no `loop timing` line, State interval max 5.34 ms over 23,999
  States.
- **BENCH-4: swerve module devices read.** Bench, RoboRIO build 7920f947,
  disabled, uncommissioned, 30 s; Linux workstation as main processor,
  revision NOT RECORDED. All five devices on CANivore `Shooter` reported
  connected (Talon FX 1, 5, 6, CANcoder 3, Pigeon 2 id 0); the steer position
  followed the CANcoder (-0.0754 against -0.0752 rot); the yaw read live; the
  RoboRIO tick with 3 motors and 2 sensors cost refresh 145 µs, devices
  205 µs, control 218 µs mean. Nothing was enabled.
- **VIS-1 (REQ-OPS-35, -36): driver dashboard list rows.** Mac mini, headless
  Chrome at 1366x768, 1280x720 and 1366x500, page after the second round of
  fixes (revision NOT RECORDED): whole rows only; every state word whole in
  19 real and 8 injected worst-case rows (the word no longer shrinks; the
  narrowest name, 68.6 px, still reads `front_rig…`); a list scrolled to its
  bottom or middle keeps its position when a row changes.

## 2026-11-13

- **BENCH-3: config persistence and State cadence.** Bench, RoboRIO build
  30d9f76, disabled, uncommissioned; Linux workstation as main processor,
  revision NOT RECORDED. Pushing a changed config and then the original
  rewrote `talos_config.bin` both times with no `[rio] loop timing` line and
  no rejected frame; the `[s.ms]` prefix shows on the companion-link console
  messages; a 60 s session delivered 12000 States (interval median 5.00 ms,
  p99 5.21, max 7.27).
- **BENCH-3 (REQ-RT-22): RoboRIO address removed, first run.** Bench, RoboRIO
  build 6cae6d0, disabled, uncommissioned; Linux workstation. `ip addr del` on
  the RoboRIO's `eth0` for 6.07 s under a 45 s session lost the link for that
  time and reconnected 47 ms after the address returned, with no rejected
  frame and no bridge send failure. The RoboRIO printed neither `hardware
  gateway socket cannot send` nor `replaced` (that build did not count or
  print other send failures). The tick after the address returned reported
  `2 overruns`, `max work 9419 us`, which none of the five tick phases (each
  under 0.3 ms) accounted for.
- **BENCH-3 (REQ-RT-22): RoboRIO address removed, rerun.** Bench, the tree of
  4473d5d, which the RoboRIO reports as build 65602fd; disabled,
  uncommissioned; Linux workstation; address removed for 6.07 s under a 45 s
  session. The RoboRIO printed `send failed (errno 22); keeping the socket`
  and, after the address returned, `sending again: 1471 sends failed over
  6.025 s (errno 22)`: the send error is `EINVAL`, which keeps the socket, and
  the link came back on the same socket. The bridge counted no send failure
  and no reopen; nothing was rejected; no `loop timing` line. In the window
  holding the gap the `driver station` phase was the long one (mean 326.5 us,
  max 4095.3 us, against about 110 us and 0.6 to 2.3 ms in the other
  windows); `link` stayed under 0.31 ms. So the earlier 9.4 ms tick was time
  inside the Driver Station read (WPILib/HAL), not the socket.
- **RT-12 (REQ-OPS-20): counters at the enable edge.** Mac mini, head
  47f6ead, ten driven runs of `./sim.sh --sim-gateway --no-studio`, disabled
  then teleop: `gateway_rejected_command_count` and `identity_rejected_count`
  stayed 0 and `timed_out_mask` stayed 0 within a second of the enable edge in
  10 of 10 (2026-11-11: 1 of 10 and 3 to 5 of 10). The intake's bit was
  raised once at every start-up (a Python node's first request arrives after
  its owner timeout), which REQ-OPS-20's later wording no longer reports.
- **RT-12 (REQ-OPS-20): Linux simulated sessions.** Linux workstation, three
  15 s sessions of `robot_physics_sim.toml`, revision NOT RECORDED: each ended
  `timeouts=0 timed_out=0` for every subsystem, with `identity_rejected=0
  ownership_rejected=0` and no owner alert at stop.
- **RT-12 (REQ-OPS-20): owner alert at start-up (wave 17).** Mac mini,
  revision NOT RECORDED: 0 of 11 sessions (the set that had shown the alert 11
  of 11) ever showed the alert or a nonzero mask; no first request was
  refused; both counters stayed flat across each enable edge.
- **CI-8: `tools/prepush.sh` on macOS.** Mac mini, wave 16, head 4a84ce36
  (before the review fixes): every step passed with no portability change
  (313 s with gradle and Studio skipped on a cold output base, 200 s with
  neither skipped; 815 `.d` files, no host header outside the SDK);
  `/usr/bin/env bash` there is Homebrew bash 5.3, so bash 3.2 is untested;
  its `./gradlew` reused a daemon already running. Wave 17, with the review
  fixes (revision NOT RECORDED): every step passed in 277 s, the
  network-namespace line read "may skip" (no `unshare`), and `git worktree
  list` was identical before and after.
- **VIS-1 (REQ-OPS-34 to -39): driver dashboard served.** Linux simulated
  session from the source tree, revision NOT RECORDED: `studio/drive.html`
  served with its assets and the 13-widget 2026 layout; `/drive` 200.
- **VIS-1: driver dashboard in a browser (wave 16).** Mac mini, the page as
  pushed in 85dc2a78, headless Chrome at 1366x768, field config from an
  installed bundle: `/drive`, its three hashed assets and `/drive/layout.json`
  served with the right cache headers; the page fit the screen in dark and
  light with status `OK Connected: telemetry current` and every widget showing
  a value; SUBSYSTEMS hid 4 of 13 rows (shooter among them) and PRE-MATCH half
  of its last row behind `overflow: hidden`; the chart legend's unit text was
  8.8 px. Chrome's own `--screenshot` captures before data arrives; the check
  needs the DevTools protocol and a real wait.
- **VIS-1: driver dashboard in a browser (wave 19).** Mac mini, the page as
  pushed in 9a3d36cd, Chrome at 1366x768, dark and light: SUBSYSTEMS 13 of 13
  rows visible in two columns and PRE-MATCH 6 of 6, as the fit model
  computed; smallest text 12 px; on load 12 of 13 widgets read STALE under
  CONNECTING for about 100 ms, then OK; a stopped session reads BRIDGE LINK
  within 75 ms; a new session is picked up 0.94 s after the bridge listens.
  At 1280x720 SUBSYSTEMS fell to one column showing 9 of 13 with the last row
  cut in half, nothing said it scrolls, and the session label squeezed its
  heading (addressed with REQ-OPS-49).
- **DEPLOY-4 (REQ-OPS-34, REQ-OPS-20): installed bundle with Studio.** Mac
  mini, an installed bundle run by hand as the boot service runs it
  (`robot.toml`, `--sim`, the repository's `sim_gateway`, 30 s), revision NOT
  RECORDED: all 14 members started and stopped clean; port 5800 served `/`
  200, `/drive/layout.json` 200 (32 channels, all in `/channels.json`),
  `/channels.json` 200 (176 channels, 6 alerts); `/drive` 404 (that bundle did
  not carry `drive.html`); `--no-studio` left nothing listening; no response
  carried `Cache-Control`; `session_report` listed
  `alert_warning_owner_timed_out` active once at start-up for 151 ms.
- **DEPLOY-7: installed bundle against a checkout's simulator.** Mac mini,
  revision NOT RECORDED: `run.sh --sim --remote 127.0.0.1 --describe-only`
  from an installed bundle described 14/14 on the field config, and a 30 s run
  against the checkout's `sim_gateway` exited 0 with every member 0.
- **DEPLOY-8: boot settings.** Mac mini, revision NOT RECORDED: `deploy.sh enable-boot --dry-run`
  printed `would refuse: FileVault is On.: the LaunchDaemon could not start
  before a login; turn FileVault off first`; `pmset` showed `autorestart 0`.
- **DEPLOY-10 (REQ-OPS-48, REQ-OPS-47): bundle on macOS (wave 16).** Mac
  mini, the no-symlink bundle, revision NOT RECORDED: 0 links; installed with
  a 4.9 s warm-up and no warning; a 30 s simulated run exited 0 with all 14
  members 0; each Python node's process is the bundled interpreter (the
  `/bin/sh` script execs it and keeps the pid and process group); `bundle_test`
  passed with the bundle restored from a disk cache after `bazel clean`. The
  warm-up's native-library pass runs under the host `python3` and took the
  install warm-up from about 4.5 s to about 11 s (58 libraries).
- **DEPLOY-10 (REQ-OPS-48): one-interpreter bundle on macOS (wave 17).** Mac
  mini, revision NOT RECORDED: 363 MB with the runtime once (`python3` there
  is a 50 kB launcher finding `libpython3.11.dylib` by `@executable_path`,
  which resolves from `python/`); installed with a 5.3 s warm-up and no
  warning; a 30 s simulated run exited 0 with all 14 members 0; `bundle_test`
  passed restored from a disk cache.
- **MAC-2: first-exec charge.** Mac mini, revision NOT RECORDED: 104 ms for
  one fresh script, 702 ms median with 8 started at once, 2.9 s with 32; the
  second exec takes 10 ms. Launcher tests that write stub scripts and probe
  them within 1 to 4 s failed there once several test processes ran
  (`launcher_test` 0 of 10 at full parallelism,
  `NonCriticalDescribeTimeoutAllowsPeersToLaunch` 5 of 10 at four jobs), and
  `bundle_test` missed the 15 s probe deadline once on `pose_estimator` right
  after a build.
- **MAC-2: suite at wave 14.** Mac mini, the head before c1668b8: build and
  212/212 fresh, gradle 54 tests, launcher stress 10/10 and 30/30 with the
  stub warm-up; `launcher_test` (then unsharded) took 54 to 58 s of its 60 s
  at ten parallel runs and timed out 9 of 10 (59.6 to 64.4 s) with bundle
  installs running beside it.
- **MAC-2 (REQ-OPS-47): install warm-up.** Mac mini, head 47f6ead: a fresh
  install's warm-up took 4.6 to 5.4 s idle and 30 to 38 s under the
  launcher-test load (bound 60 s per member, none reached); the first
  `--describe-only` after it took 0.4 to 0.5 s idle and 1.5 to 1.7 s under
  load, 14/14, against 17.4 s under load without the warm-up; the warm-up
  added no file to the bundle.
- **TEST-9: sharded `launcher_test` on macOS.** Mac mini, head 47f6ead, ten
  runs at default jobs: `launcher_test`'s four shards took 13.1 to 15.8, 19.4
  to 25.6, 24.9 to 27.5 and 17.5 to 24.2 s of the 60 s limit (the unsharded
  binary took 54 to 58 s), 40 of 40 passing.
- **TEST-22: bridge test host check on macOS CI.** GitHub macOS runner, CI run
  43375149698, head 47f6ead: `HardwareNodeTest.
  RunSendsHeartbeatsAndTheTimeoutNeutralOnItsOwn` failed its host check: "host
  unqualified: an ordinary thread's 1 ms sleep overran by 26411 us" against
  the 25000 µs it allows. First occurrence recorded.
- **BENCH-1: bench velocity buttons past the voltage cap (computed, not
  measured).** Revision NOT RECORDED. With the slot 0 gains the Talon asks for about 5.85 V from
  rest for button 2 (30 rps), clamped to its 4 V peak, and settles near
  2.85 V; `max_voltage = 2.0` would cap button 2 near 21 rps.

## 2026-11-12

- **CAM-1: simulated camera CPU.** Linux workstation (8 cores), load under
  1.1, revision NOT RECORDED: four cameras at 1280×720 and 60 fps use about
  2.6 cores; a frame takes 4.6 to 5.6 ms at the median, but about a quarter
  take 12 to 14 ms because exposure gain lets sensor noise through as speckle
  the detector then examines.
- **TEST-16: `accuracy_test` fusion check evaluated.** Linux workstation, load
  0.8 to 1.9, after 0a98851: three runs kept 534 to 539 of 539 frames per
  camera; the check was evaluated and passed (fused 0.045 to 0.064 m against
  0.094 to 0.095 m odometry). Before 0a98851 the camera node's single result
  slot lost 5 to 7% of computed frames and the check was skipped on every
  workstation run.
- **STUDIO-1: a client that never reads.** Mac mini loopback, revision NOT
  RECORDED: held only after about 327 KB, 8.5 s at the 40 kB/s share (nothing
  held at 8 s, 3,295 holds by 12 s).
- **CI-1, CI-2, CI-3: Linux CI legs.** GitHub CI on the pushed branch: the
  Linux sanitizer legs (asan, tsan) and the bindings diff were green in run
  43184148264 (c725fd3); the Linux soak (seed 0) and match rehearsal were
  green in runs 43178185836 and 43184148264. Run 43178185836's first
  merge-base tidy surfaced nine findings in three files (`v4l2_source.h`, two
  `studio/review` tests), fixed in 014beaa and c725fd3; run 43184148264 was
  clean over the files it diffed.

## 2026-11-11

- **MAC-1: full suite on the Mac mini.** `bazel test //...` 186 of 186 at
  d53393e (149 s wall, warm cache).
- **LOG-2: log window before the time cut.** Bench, revision NOT RECORDED:
  kept spans 48.3 to 120.4 s, longest over shortest 2.49.
- **RT-5: RoboRIO loop at default scheduling.** Bench, image
  FRC_roboRIO_2026_v1.2, two cores, 5 ms loop, one Talon FX, loop at default
  scheduling; revision NOT RECORDED. The gateway's State sample intervals were
  p50 5.002 ms, p90 5.44, p99 6.66, max 9.01 with nothing else running, the
  main processor's publish intervals mirroring them. The loop reported up to
  `max work 11506 us, max interval 11510 us`, with 8 lifetime overruns and 15
  late ticks in about 10 minutes, the worst coinciding with ssh/top sessions
  on the RoboRIO (inference). A later run with no shell attached: 5 overruns
  and 2 late ticks in 10 min, max work 8.3 ms. Default-priority busy loops on
  the RoboRIO kept the worst State interval under 7.6 ms.
- **RT-5: RoboRIO thread table.** Same bench, same day. Every kernel
  interrupt thread `SCHED_FIFO` 15 (`irq/54-eth0`, `irq/64-e000b000`, ...)
  except the FPGA's `irq/61-atomiczy` at 96; no kernel CAN interrupt (CAN goes
  through the FPGA interrupt and user-space threads). In the robot program
  `can_recv`, `can_psend`, `can_stat`, `frc_evnt_client`, `nirio::DeviceNo`,
  `atomiczynqu` (two), `niriodevenum` and `frc_server_mon` are `SCHED_OTHER`
  nice 0; three unnamed threads `SCHED_RR` 1, 1 and 2 (unidentified); all 22
  `FRC_NetCommDaemon` threads `SCHED_OTHER`; `sched_rt_runtime_us` -1
  (throttling off). So a loop that never sleeps holds off, on its core, every
  `SCHED_OTHER` thread including `can_psend` and `can_recv`, and the
  `SCHED_RR` threads.
- **RT-5, RESET-2, RESET-3, RT-15: evening session, loop at `SCHED_FIFO` 10.**
  Bench, RoboRIO build ba81a01, loop `SCHED_FIFO` 10, Kraken X44 on CANivore
  `Shooter`, firmware 26.3.0.0, disabled; Mac mini main processor, every node
  with its full realtime grant; main-processor revision NOT RECORDED.
  - RT-5: with no shell on the RoboRIO, 600 s had 0 overruns and 0 late ticks
    (no `loop timing` line printed); `/hw/state` intervals median 4.999 ms,
    p99 5.225, max 7.526 over 120000 States; tick phase maxima refresh 191,
    devices 274, control 291, health 215 µs; main-processor timer lateness p99
    at most 0.19 ms, max 1.13 ms.
  - RESET-2: not reproduced in two config applies (session start and after a
    RoboRIO reboot): applied voltage and stator current exactly 0 in the first
    100 States and throughout.
  - RESET-3: four whole-feedback gaps of 60.1, 64.2, 63.3 and 67.3 ms in 600 s
    on the CANivore, each matched by a RoboRIO `feedback LATE` line (peak age
    62.6 to 67.8 ms); the State stream itself had no gap over 20 ms. So the
    silences are not specific to the RoboRIO bus.
  - RT-15: `reboot` over ssh took 48.24 s from the last State to the first:
    about 16.6 s to kernel start, 30.9 s from kernel start to program start
    (eth0 link came up three times, at 11.0, 12.8 and 26.7 s), 0.83 s from
    program start to the first State (7 failed socket attempts, errno 101).
    The link recovered by itself (new boot id followed, clock valid after
    about 8 status samples).
- **RESET-2: idle Kraken reports current after config apply.** Bench, one
  Kraken X44, Talon FX id 1, firmware 26.3.0, config uncommissioned, no Driver
  Station; revision NOT RECORDED. For the first 77 States after the config was
  applied the undriven motor reported applied voltage 0.08 V and stator
  current up to 15.4 A, then exactly 0. A later reading showed stator current
  -4.80 A right after config apply, the same window.
- **RESET-3: whole-bus CAN silences on the RoboRIO bus.** Bench, one Kraken
  X44 Talon FX id 1, firmware 26.3.0.0, disabled, uncommissioned; Mac mini
  main processor with the realtime grant; revision NOT RECORDED. Every Talon
  frame id stops together for a constant 64-66 ms (the 200 Hz frame
  0x02042DC1, the 20 ms and 250 ms frames, the multi-frame responses on
  0x0204F946), in bursts at random intervals of 32-479 s (one run had none for
  19.6 min, then three within five minutes; 11 events in 2722 s of one boot, 7
  with no main processor attached; 3 more captured by a diagnostic build with
  an independent HAL CAN stream). The RoboRIO's `txerr` goes 0 to 128 within
  5-14 ms of the start and decays; `rxerr`, bus-off and tx-full stay 0; the
  5 ms loop keeps ticking; the missed frames never arrive later; `can_util`
  drops to its floor. Not a RoboRIO scheduling or talOS defect. The far-end
  terminator is an unpowered REV PDH, so the Talon is the only live node.
  Disabling the Phoenix diagnostics server stops all Phoenix CAN traffic on
  this image, so that experiment cannot be run.
- **RT-15: RoboRIO reboot and Ethernet unplug.** Bench, revision NOT
  RECORDED. After `reboot`, State returned 46.8 s after the last one, about
  18 s after the RoboRIO first answered ping; the network flapped (ping gaps
  of 3.1 s and 4.5 s) until State returned. The socket retry (100 ms)
  explains at most 1 s (measured 0.6 s). A later measurement puts the delay in
  NI's boot: the program starts about 10 s after the first ping answer. A
  software Ethernet unplug (`ifconfig eth0 down` for 3 s) gave a 6.9 s State
  gap (link and DHCP), the node's `connected` false 101 ms after the last
  State, and recovery complete 39 ms after the first State.
- **RT-13: clock round trip after the bridge's wake-on-input.** macOS 26.3,
  Apple M4 Pro, `./sim.sh --sim-gateway --no-studio`, revision: before and
  after the commit "bridge, ipc, protocol: wake the bridge on input instead of
  a 1 ms poll". After: `clock_min_round_trip_us` p99 7.25-8.77 ms and max up
  to 8.97 ms in 11 runs; before: p99 6.90-6.99 ms and max <= 7.0 ms in 3 runs.
  The median fell from 6.0 ms to 4.5-6.0 ms; `state_age_us` median rose from
  2.0 ms to 2.3-3.3 ms (possibly only its sampling instant). No timeouts,
  rejects or late ticks.
- **RT-12: counters at the enable edge.** Mac mini, driven simulation runs,
  revision NOT RECORDED: the check the 2026-11-13 rerun passed 10 of 10
  (`gateway_rejected_command_count`, `identity_rejected_count` and
  `timed_out_mask` 0 within a second of the enable edge) was recorded as
  "1 of 10 and 3 to 5 of 10" (which figure belongs to which counter was not
  recorded).
- **RIO-1: stale-FaultField flake.** Load average about 13:
  `StaleFaultFieldClearsLimitsAndKeepsFaults` once read `fault_flags 0` before
  its stale step. The old test then passed 120 runs at load 7 to 24 (date not
  recorded).

## 2026-11-10

- **CI-1: sanitizers on macOS.** Mac mini, revision NOT RECORDED: no ASan,
  UBSan or TSan report anywhere in `//...`. The remaining failures were
  platform cost, not races: one `fork()` takes about 30 ms under ASan and
  375 ms under TSan (1.5 ms plain, `posix_spawn` 0.7–2.6 ms).

## Date not recorded

Measurements moved here from TODO.md on 2026-11-15 whose entries gave no
date. The revision is given where the entry gave one. Nothing new is added to
this section.

- **MAC-1: Python node spawn and first exec.** Linux: a Python node's
  spawn-to-registry-row time 152–169 ms cold, 77–82 ms warm for
  `pose_estimator`. A development Mac, in Bazel's darwin sandbox: the first
  exec of a freshly written stub script takes 95–106 ms, a re-exec 5–15 ms
  (the launcher_test describe-timeout case failed at HEAD with a 100 ms probe
  deadline).
- **MAC-2: stub warm-up and install.** Mac mini, after the launcher tests'
  stub warm-up (`talOS/launcher/testdata/first_exec.h`), ten parallel runs:
  `launcher_test` 10 of 10 (from 0 of 10) and `supervision_test` 30 of 30
  shards (from 8 of 30), no deadline changed. The charge is per file, paid only
  by an exec (XprotectService); a hard link to a paid file costs 3 ms. A fresh
  `deploy.sh install` of the 2026 bundle answered `--describe-only` in 3.7 s
  cold (0.3 s warm; `pose_estimator` 1.66 s cold), idle host.
- **CI-2: reporter starvation not reproduced.** 12-core development Mac:
  time-constraint load up to 40 threads never delayed a timeshare 10 ms
  sleeper past 27 ms. macOS CI: the soak at 2e10d65, 8c999f6 and 9fdf947 and
  `match_rehearsal_test` at 45dac98 killed healthy event-loop nodes
  (`autonomous`, never signalled; `driver`, `shooter`, `telemetry`) as
  "heartbeat/dispatch stalled" while their logs show no loop gap over 40 ms
  outside the harness's SIGSTOP windows, on a 3-vCPU runner at about 254% CPU.
  Soak run 42906922745: eleven event-loop nodes froze together for about
  105 ms while the bridge and `sim_gateway` kept running. Locally the 180 s
  soak passed with seed 0 and failed with seed 1 (a 300 ms SIGSTOPped
  `hardware_node` killed as stalled, heartbeat refreshed every 250 ms).
- **LOG-5: session reader memory.** Linux, `//talOS/launcher:log_memory_bench`
  on a 210 MiB session: `log_dump` peaks at 5.6 MiB (was 632 MiB);
  `tlog_to_wpilog` peaks at 525 MiB (was 681 MiB), about 300 B per
  publication.
- **LOG-13: converted session sizes.** A 45 s bench session of 27 MB converts
  to 19 MB and 251 entries; a 60 s, 12-motor fixture of 13 MB to 38 MB.
- **LOG-8: time-cut window in simulation.** Bench config, disabled, 12 MiB
  cap: kept spans went from 19.5, 22.6, 20.0 and 41.3 s (ratio 2.12) to 16.4,
  16.3, 16.1 and 16.3 s (1.017) before the formula's last change. Sessions use
  roughly 70 to 93% of the byte budget.
- **OBS-1: RoboRIO loop cost per tick.** Bench, one motor: refresh 74 µs,
  devices 57 µs, control 27 µs, health 23 µs mean.
- **RIO-2: refresh phase on one bus.** Single-bus bench: refresh phase 147 µs
  mean, unchanged by the per-bus `RefreshAll`.
- **RT-7: wake-socket descriptor peak.** Mac mini, 951c87f, disabled
  simulation: 11 of 256. Linux, enabled: 27.
- **RT-19: Phoenix desktop-simulation lock.** Desktop simulation: a 200 ms
  hold of the `PhoenixTools_Sim` mutex stalled the RoboRIO loop 198 ms and
  stopped all frames.
- **BENCH-2: bench motor readings.** Bench: applied voltage read -24.64 V for
  one 20 ms frame when a duty-cycle demand was released to coast (-2.0 x
  12.32 V). slot0 kV 0.095 V/rps was set from two points near 10 rps; a
  30 rps request reached 35.5 rps, implying about 0.086.
- **VIS-2: pose estimator tick with every drivetrain sample.** Linux
  workstation, busy host: factor-graph median tick 0.38 ms to 0.88 ms (kalman
  0.09 to 0.14 ms). Mac mini (M4 Pro, 60 s benchmark): 0.50 ms median, 0.69 ms
  at p99 (kalman 0.11 and 0.16 ms).
- **SCALE-1: publish cost per armed reader.** Linux,
  `//talOS/rtms:wake_fanout_bench`: about 1 µs per armed reader idle, up to
  6 µs loaded; walking unused slots not measurable.
- **STUDIO-2: recorded-session review.** By hand on a 20 s simulated session:
  438 channels, a snapshot in 7.7 ms, the picker listing the session in
  headless Chromium.
- **STUDIO-4: large-message wait (derived, not measured, 62f29fa).** About
  16 s at 4 clients for a 64 KB message, about 55 min at 64 clients, no bound
  near the 64,000 B cap.
- **DEPLOY-10 (REQ-OPS-48): one-interpreter bundle on Linux.** 749 to 407 MB
  (tar 781 to 424 MB, gzip 240 to 131 MB); a 20 s simulated run from an
  installed bundle exited 0 with all 14 members 0.
- **TEST-9: launcher process tests, Linux against macOS.** c725fd3: Linux
  `launcher_test` 4.2 s, `supervision_test` 3.6 s (largest of 3 shards); idle
  Mac mini `launcher_test` 20.0 s for 57 cases (35 s under load),
  `supervision_test` 20.8 s. On the Mac the first exec of a newly written
  `#!/bin/sh` stub takes 126 ms (median of 20, max 187), a second exec 13 ms,
  `/bin/sh <stub>` 9 ms; about 100 fresh stubs per run is about 13 s of the
  20 s.
- **TEST-10: launcher tests under load on macOS.** Mac mini (12 cores),
  89ebbf1, `--runs_per_test=20` at full parallelism: `launcher_test` timed out
  20 of 20 at 75 s (after 30 to 40 of its 57 cases, 4 to 9 already failed);
  `supervision_test` failed 14 of 60 shard runs; `continuity_test` passed 20
  of 20 (3 of 20 failed before). Nearly every failure is a `--describe` probe
  missing its deadline, so the launcher refuses the graph (`did not answer
  --describe (no answer within 4000ms; the probe was killed)`; 668 misses at
  4000 ms, 63 at 250 ms, 61 at 1000 ms). Most-failed cases:
  `NonCriticalDescribeTimeoutAllowsPeersToLaunch`,
  `DurationDeadlineIsNotDelayedByFailingBridge`, `ProbesRunInParallel`;
  `HeartbeatingConstructorStillHasAFiniteDeadline`,
  `CriticalCleanExitRestarts`, `RequestedStopDoesNotRestartCleanExits`. A
  single run of each passes.
- **TEST-17: real-time end-to-end tests.** `accuracy_test` 22 s,
  `match_rehearsal_test` 19.6 s, `startup_test` 9.7 s;
  `//talOS/python:python_startup_test` 30 s in an older run;
  `//tools:scaffold_sim_test` 8 s.
- **TEST-24: browser test timeout.** Linux workstation, `--runs_per_test=10`
  while other tests ran: one run in 30 timed out waiting for the Studio
  bridge's `listening` line; not seen in 80 runs since.
- **LANE-1: fast-lane targets over 5 s on hosted runners.** CI run
  43357369179, Linux: `//2026-robot/main_processor:bundle_test` 6.8 s,
  `pose_estimator:node_test` 5.6 s, `//studio:typecheck` 5.6 s; six on macOS,
  `bundle_test` 19.5 s; all under 5 s on the workstation.
- **INC-3: macOS restart path under disk load.** Mac mini, syscall shim under
  8 disk writers: three disk waits on the SIGKILL-to-first-request path; the
  restarted node's event-log open and header write on its loop thread
  (145–528 ms), the manifest writer's create stalling the restart's directory
  reads and fork (7–160 ms), and the killed node's reap (SIGKILL → `waitpid`
  up to 250–1254 ms). Kill-and-reap A/B with a program mimicking `LogWriter`
  (4 × 256 KiB pool, 280 KiB/s, writer thread; 135 kills per policy over three
  runs, 1-minute load 1.5–5.5 with other builds at times), kills slower than
  100 ms / 250 ms / max: once-a-second `fsync` (current) 24 / 10 / 579 ms;
  `fdatasync` after each chunk 15 / 7 / 660 ms (per run 7, 7, 1 against 9, 9,
  6 slower than 100 ms, so run-to-run spread exceeds the gain); with 32 KiB
  writes 4 / 2 / 627 ms of 45, but a 6.6 s sync stall filled the pool and
  dropped records; per-chunk `fsync` 10 / 7 / 797 ms and `F_BARRIERFSYNC` 22 /
  11 / 1097 ms of 45; `F_NOCACHE` 13 / 7 / 1269 ms of 45. Per-chunk sync
  removes the slow reaps of an idle writer holding dirty pages (max 484 ms →
  9 ms), but a kill then waits for the sync or write in flight, a quarter to a
  third of the writer's time under that load (`sync_file_range` is
  Linux-only).
- **INC-5: `setpgid` refusal not reproduced.** macOS 26.3: 56,000 spawns of
  the launcher's sequence from 8 to 16 threads, both inside and as a session
  leader, never reproduced `EPERM`.
- **INC-6: hardware-request gaps at timeshare.** Mac mini with three free
  cores: A/B runs failed 3 of 3 at timeshare and showed no gap over 15 ms with
  `TALOS_REALTIME=1`. The graph alone uses about 254% CPU median (463% p90).
- **VIS-1: the driver dashboard page in tests.** `studio/drive.html` is plain
  TypeScript, about 47 kB, in every bundle's webroot. `//studio:drive_browser_test`
  (headless Chromium on Linux, every run) sees the lists and headings whole at
  1366x768 and 1280x720, the out-of-view count after a scroll, a scrolled list
  keeping its place through a redraw, and the stale draw 302 to 410 ms after
  the producer stops.
- **MECH-8: travel past a limit switch in the physics test.** 0.18 rot past a
  switch reached at about 8 rps once the mechanism settles in neutral before
  its first move (REQ-SAFE-36), where it used to run 11 rot past.
- **REPO-5: the 2026 robot repository (stage A).** Linux workstation, downloads
  cached: `bazel test //...` 23 of 23, `//:sim_smoke_test` running all 14
  members to exit 0 with State received; cold build 246 s and 7,504 actions.
- **AUTO-4: autonomous translation gain in simulation.**
  `//2026-robot/main_processor/integration:closed_loop_test`, ideal motors:
  0.068 m from the path 1 s after a gradual 0.3 m shove at gain 2.0, against
  0.154 m at 1.0.

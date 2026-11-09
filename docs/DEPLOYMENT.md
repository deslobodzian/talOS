# Deployment

The main processor runs a **bundle**: one directory with every node binary, the
launcher, the config and the runtime libraries. `tools/deploy.sh` installs
bundles, starts one on boot, shows logs and rolls back.

## Build a bundle

```sh
bazel build //2026-robot/main_processor:robot_bundle
```

`talos_bundle` (`talOS/build/contract.bzl`) takes `nodes`, `config`
(`robot_field.toml`) and `config_files`; the robot's BUILD passes `NODES` and
`CONFIG_FILES` from the generated `configuration/robot_field_manifest.bzl`
(`bazel run //tools:sync_manifest -- <robot_field.toml>`). It writes
`bazel-bin/2026-robot/main_processor/robot_bundle/`:

| Entry | What |
|---|---|
| `run.sh` | runs `./launcher --bundle <dir> --config config/robot_field.toml "$@"` |
| `launcher` | `//talOS/launcher` |
| `bin/` | node binaries; a Python node's runfiles beside it (its wheels, e.g. `robotpy-wpimath` and `gtsam`, and `libtalos_node.so`) |
| `lib/` | any shared libraries C++ nodes link, one copy per name; no C++ WPILib libraries are staged. `talos_cc_node` binaries carry the rpath `@loader_path/../lib` (macOS) or `$ORIGIN/../lib` (Linux) |
| `config/` | `robot_field.toml` and every file it reads (`include`, `path`, `defaults`), at the same paths relative to each other as in the source tree |
| `bundle_manifest.toml` | node names, targets, binaries, languages, config SHA-256 |

The config SHA-256 (`config_sha256`, copied into every session manifest) is
the identity of the staged config: every file under `config/`, the robot
config included, contributes its path relative to `config/` and its bytes.
Absolute and workspace locations never participate, so moving the whole
config tree or copying the bundle elsewhere keeps the identity; editing,
renaming or moving any one file relative to the others (the root config too)
changes it.

The bundle uses `2026-robot/main_processor/configuration/robot_field.toml`,
which includes `robot.toml` and overrides only `[robot] controller_address`
to the RoboRIO's `10.56.87.2`. Simulation and its tests keep using
`robot.toml` with loopback (`127.0.0.1`); no source edits are needed when
switching between simulation and the field bundle. The overlay does not
change CAN ids or commissioning: hardware must still be checked and
commissioned through the normal bring-up process.

To bring up the bundle against a different controller address without editing
the config, run `./run.sh --remote <ip>` (for example,
`./run.sh --remote 10.56.87.3`). This overrides the effective address for that
launch; use `--remote-port` and `--local-port` if the ports also differ.

## On the main processor

```sh
./tools/deploy.sh install <bundle> [--prefix /opt/talos]
./tools/deploy.sh enable-boot [--prefix /opt/talos] [--user talos] [--dry-run]
./tools/deploy.sh logs [--prefix /opt/talos] [LOG_ROOT]
./tools/deploy.sh fetch [--prefix /opt/talos] DEST
./tools/deploy.sh rollback [--prefix /opt/talos]
```

- **install** copies the bundle to `<prefix>/bundles/<stamp>/` (read-only,
  symlinks kept), retains `previous`, and atomically replaces `current` using
  a temporary sibling symlink and rename (Linux and macOS). This selects the
  next bundle; restart the service to activate it.
- **enable-boot** starts `<prefix>/current/run.sh` at boot as `--user`
  (default `talos`; the account must exist). Linux: writes and enables
  `/etc/systemd/system/talos.service` (`Type=notify`, `Restart=always`, a
  three-second supervision watchdog). macOS: writes the
  LaunchDaemon `/Library/LaunchDaemons/org.talos.robot.plist` (`KeepAlive`)
  and bootstraps it into the system domain, so it starts without a login.
  Both use sudo. Before enabling, it checks bundle read/traversal access as
  the selected account and creates `<prefix>/logs` owned by that account,
  mode 0755. Bundles stay read-only. An inaccessible bundle fails clearly;
  fix ancestor traversal/read permissions rather than granting bundle writes.
  `--dry-run` prints these provisioning/check commands and the unit or plist.
- **logs** lists session directories under `<prefix>/logs` (or `LOG_ROOT`) and
  summarizes the newest manifest by modification time: session id, config path
  and hash, each node's exit code and log path. No installed bundle is needed
  to inspect retained sessions.
- **fetch** copies every session folder from `<prefix>/logs` into `DEST`,
  preserving its manifest and all node logs, including restarted attempts.
  Run it on the main processor, for example
  `./tools/deploy.sh fetch /media/usb/robot-sessions`, then transfer that
  directory to the analysis machine. Existing destination session names are
  rejected rather than merged; choose a fresh destination for each collection.
- **rollback** swaps `current` and `previous`: the whole previous bundle, with
  its own config. Selection does not change already-running processes.
  With the robot disabled, activate install/rollback with
  `sudo systemctl restart talos` on Linux, or
  `sudo launchctl kickstart -k system/org.talos.robot` on macOS.

Run by hand: `/opt/talos/current/run.sh --duration-s 30`.

Boot services set `TALOS_LOG_DIR=<prefix>/logs`. The generated `run.sh` passes
that directory as the launcher's output **root**, unless an explicit
`--output-dir ROOT` is supplied. The launcher appends a unique session id on
every invocation, so recordings live in `<prefix>/logs/<session-id>/` and
restarts of the service never reuse a previous session folder. For a manual
run to use the same root, set `TALOS_LOG_DIR=/opt/talos/logs` or pass
`--output-dir /opt/talos/logs`; `logs LOG_ROOT` can inspect another output root.

## Supervision and service recovery

Boot probes each node before spawning the graph. `[subsystem] critical = true`
is the fail-closed default; failed probes refuse startup. Observation-only
nodes may explicitly set `critical = false`: a timeout, crash or import failure
then logs a warning without blocking the rest of the valid graph, and the
node is launched under restart supervision. Motors, digital outputs, PWM
outputs and described target-lane senders cannot be non-critical; the hardware
bridge is always critical. Naming and graph errors remain fatal, so `run.sh`
does not weaken the graph checks with `--allow-graph-errors`.
`--describe-timeout-ms MS` adjusts the existing 15000 ms per-probe deadline
and can be passed through `run.sh`.

The Linux unit sets `TimeoutStartSec=1200` (20 minutes), independently of
the running watchdog. Budget startup as at most two sequential probes per
roster member (initial description plus graph-reader re-probe), each with the
default 15-second deadline: up to 32 members take at most 960 seconds of probe
waiting, leaving 240 seconds for config/graph setup and process spawning.
Larger graphs or `--describe-timeout-ms` overrides must fit this same finite
budget or explicitly increase the installed unit's timeout. A genuinely hung
startup is killed by systemd at the deadline; watchdog notifications only begin
after READY, from the supervision loop, never an independent heartbeat.

On Linux the launcher sends `READY=1` when the graph has started and enters
supervision, then `WATCHDOG=1` on each supervision iteration (normally every
20 ms). `WatchdogSec=3` leaves substantial scheduling and manifest-write
headroom while recovering a hung launcher during a match; systemd expects a
notification within half that interval. A stalled loop cannot be masked by
an independent watchdog thread. `Restart=always` starts a fresh session after
either a watchdog failure or a launcher exit; `systemctl stop talos` still
stops the service deliberately. Notifications use `NOTIFY_SOCKET` directly,
including Linux abstract (`@...`) Unix datagram sockets, without libsystemd.

Each node (including description probes and the Studio bridge) owns a process
group. Restart, stop and shutdown signal that whole group, not just its
leader. A per-node guardian watches a launcher-owned lifetime pipe and kills
the group when the launcher dies, even on SIGKILL; Linux additionally sets
`PR_SET_PDEATHSIG` on the node. macOS uses the same lifetime-pipe protection,
but has no `PR_SET_PDEATHSIG` or systemd watchdog: launchd's `KeepAlive`
restarts an exited launcher, not a hung one. Nodes and helpers must remain in
their assigned process group rather than daemonizing into another session.

Critical nodes never enter quarantine: crashes and heartbeat stalls retry with
exponential backoff from 50 ms capped at 1 s, even after exhausting the rolling
budget. Non-critical nodes retain the circuit breaker: exhausting their budget
quarantines them until the oldest restart ages out, plus normal backoff.
Exactly one retry is pending, so clearing a transient fault can recover without a service
restart. The session manifest records `quarantined` and
`retry_at_monotonic_ns`; a stopping session cancels pending retries.
`max_restarts = 0` never restarts a node. Registry version 3
(`/talos_registry.3`) uses host-monotonic heartbeat/liveness timestamps, so
wall-clock corrections cannot falsely evict healthy nodes; deploy all node
and viewer binaries together.

## RoboRIO

`2026-robot/controller_processor/rio` is deployed with GradleRIO
(`./gradlew deploy` in that directory). Set `kCompanionAddress` in
`src/main/cpp/Robot.cpp` to the main processor's address first.
Robot bring-up checklist: [BRINGUP.md](BRINGUP.md).

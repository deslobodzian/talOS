# Deployment

The main processor runs a **bundle**: one directory with every node binary, the
launcher, the config and the runtime libraries. `tools/deploy.sh` installs
bundles, starts one on boot, shows logs, rolls back and prunes old bundles.

## Build a bundle

```sh
bazel build --config=release //2026-robot/main_processor:robot_bundle
```

`--config=release` (`.bazelrc`) stamps the build: the bundle records the git
revision it was built from and whether the tree had uncommitted changes
(`tools/workspace_status.sh`). Without it the bundle builds the same but
records no revision.

`talos_bundle` (`talOS/build/contract.bzl`) takes `nodes`, `config`
(`robot_field.toml`) and `config_files`; the robot's BUILD passes the hardware
node plus `NODES`, and `CONFIG_FILES`, from the generated [field manifest](../2026-robot/main_processor/configuration/robot_field_manifest.bzl)
(`bazel run //tools:sync_manifest -- 2026-robot/main_processor/configuration/robot_field.toml`).
The build fails unless `nodes` is exactly what the config launches: its
subsystems' nodes and its hardware node, `[robot] hardware_node` or by default
`//talOS/bridge:hardware_node`. A robot viewed with Studio also sets
`studio_bridge` (`//studio/bridge:studio_bridge`) and `studio_webroot`
(`//studio:web_dist`, the built site), both or neither; the build fails when
`studio_bridge` is not the config's `[studio] bridge` (when it sets one) or
the site has no `index.html`. The 2026 robot, the bench rig and
`tools/new_robot`'s scaffold set both, which add about 2.5 MB. It writes
`bazel-bin/2026-robot/main_processor/robot_bundle/`:

| Entry | What |
|---|---|
| `run.sh` | runs `./launcher --bundle <dir> --config config/robot_field.toml "$@"` from `<dir>`, its own directory with symlinks resolved, adding `--output-dir "$TALOS_LOG_DIR"` when that is set; a `--config`/`--output-dir` argument replaces the default |
| `launcher` | `//talOS/launcher` |
| `bin/` | node binaries; a Python node's runfiles beside it (its wheels, e.g. `robotpy-wpimath` and `gtsam`, and `libtalos_node.so`) |
| `lib/` | any shared libraries C++ nodes link, one copy per name; no C++ WPILib libraries are staged. `talos_cc_node` binaries carry the rpath `@loader_path/../lib` (macOS) or `$ORIGIN/../lib` (Linux) |
| `studio/` | with `studio_bridge` and `studio_webroot` only: the bridge binary (`studio/studio_bridge`) and the site it serves (`studio/webroot/`) |
| `config/` | `robot_field.toml` and every file it reads (`include`, `path`, `defaults`), at the same paths relative to each other as in the source tree |
| `bundle_manifest.toml` | node names, targets, binaries, each binary's SHA-256 (`binary_sha256`), languages, config SHA-256; in a stamped build `git_revision` and `git_dirty`; with Studio, a `[studio]` table naming its bridge (path, SHA-256, label) and webroot |

The launcher copies `config_sha256`, `git_revision` and `git_dirty` into every
session manifest, and records each member's binary SHA-256 again when the
session starts ([session manifest](../talOS/launcher/README.md#session-manifest)).
A digest covers only the file under `bin/`: not a C++ node's shared libraries
in `lib/`, nor a Python node's runfiles beside its entry script. The bundle's
revision covers those.

The config SHA-256 is
the identity of the staged config: every file under `config/`, the robot
config included, contributes its path relative to `config/` and its bytes.
Absolute and workspace locations never participate, so moving the whole
config tree or copying the bundle elsewhere keeps the identity; editing,
renaming or moving any one file relative to the others (the root config too)
changes it.

The bundle uses `2026-robot/main_processor/configuration/robot_field.toml`,
which includes the base [robot config](../2026-robot/main_processor/configuration/robot.toml)
and changes only `[robot] controller_address`, to the RoboRIO's `10.56.87.2` (it also restates the base's `require_target_coverage = true`).
Simulation and its tests keep using that base config with loopback (`127.0.0.1`);
no source edits are needed when switching between
simulation and the field bundle. The overlay does not
change CAN ids or commissioning: hardware must still be checked and
commissioned through the normal bring-up process.

`./run.sh --start-studio-bridge` serves Studio from the bundle on
`[studio] http_port` (5800): the launcher runs the bundle's bridge for
`[studio] bridge` and serves the bundle's webroot unless `[studio] webroot`
is an absolute path ([Studio bridge](../talOS/launcher/README.md#studio-bridge)).
The boot service starts `run.sh` without it. From a bundle built without
Studio, the flag starts nothing and the session runs without Studio.

To bring up the bundle against a different controller address without editing
the config, run `./run.sh --remote <ip>` (for example,
`./run.sh --remote 10.56.87.3`). This overrides the effective address for that
launch; use `--remote-port` and `--local-port` if the ports also differ.

## On the main processor

```sh
./tools/deploy.sh install <bundle> [--prefix /opt/talos]
./tools/deploy.sh enable-boot [--prefix /opt/talos] [--user talos] [--dry-run]
./tools/deploy.sh boot-check
./tools/deploy.sh logs [--prefix /opt/talos] [LOG_ROOT]
./tools/deploy.sh fetch [--prefix /opt/talos] DEST
./tools/deploy.sh rollback [--prefix /opt/talos]
./tools/deploy.sh prune [--prefix /opt/talos] [--keep 3]
```

A relative `--prefix` is resolved against the working directory.

- **install** copies the bundle to `<prefix>/bundles/<stamp>/` (read-only,
  symlinks kept), checks that the copy has an executable `run.sh`, runs
  `sync`, retains `previous`, and atomically replaces `current` using
  a temporary sibling symlink and rename (Linux and macOS). This selects the
  next bundle; restart the service to activate it.
- **enable-boot** starts `<prefix>/current/run.sh` at boot as `--user`
  (default `talos`; the account must exist). Linux: writes and enables
  `/etc/systemd/system/talos.service` (`Type=notify`, `Restart=always`, a
  three-second supervision watchdog, `LimitRTPRIO=49` and
  `LimitMEMLOCK=infinity`, so the unprivileged service gets the
  [realtime scheduling](NODE_DEVELOPMENT_GUIDE.md#realtime-scheduling) its
  nodes request). macOS: refuses while FileVault is on (`fdesetup status`
  is anything but `FileVault is Off.`), since the daemon then cannot start
  until someone logs in; sets `pmset -a autorestart 1 sleep 0` (power back on
  after a power loss, never sleep while idle); writes the
  LaunchDaemon `/Library/LaunchDaemons/org.talos.robot.plist` (`KeepAlive`,
  `ProcessType` `Interactive`, stderr to `/Library/Logs/talos/launchd.stderr.log`,
  [below](#boot-service-output)) and the rotation rule
  `/etc/newsyslog.d/org.talos.robot.conf`, bootstraps the daemon into the
  system domain, so it starts without a login, and ends with `boot-check`.
  `ProcessType` `Interactive` gives the launcher an app's resource limits,
  that is none; without the key launchd applies light limits that throttle
  the job's CPU and I/O (`man launchd.plist`), `Background` throttles harder,
  and `Adaptive` follows XPC activity the launcher has none of.
  Both use sudo. Before enabling, it checks bundle read/traversal access as
  the selected account and creates `<prefix>/logs` owned by that account,
  mode 0755. Bundles stay read-only. An inaccessible bundle fails clearly;
  fix ancestor traversal/read permissions rather than granting bundle writes.
  `--dry-run` prints these provisioning/check commands and the unit, or the
  `pmset` command, plist and rotation rule (on macOS reporting, not refusing,
  FileVault).
- **boot-check** (macOS; elsewhere it only says so) checks, read-only and
  without sudo, what a robot that boots with nobody logged in needs: FileVault
  off and `pmset -g` showing `autorestart 1` and `sleep 0`; it exits 1 naming
  each one that does not hold. It also reports the automatic-update settings
  in `/Library/Preferences/com.apple.SoftwareUpdate`, warning when macOS
  updates install on their own (which restarts the Mini); `enable-boot`
  changes neither FileVault nor updates, which a person sets in System
  Settings. `tools/mac_qualify.sh` runs it as a step.
- **logs** lists session directories under `<prefix>/logs` (or `LOG_ROOT`),
  oldest first by the `start_wall_ns` their manifest records (the directory's
  modification time without one; session ids are not ordered in time), each
  with its start, `git_revision` (and `dirty`), the launcher's match
  classification from its `session_class` file (`match`, `other`, `unknown`,
  or `unclassified` before retention has read it; a `--sim` session is never
  classified, so it stays `unclassified`) and its runs counted by
  `reason`. It then summarizes the newest manifest: session id, start, config
  path and hash, revision, and per node its exit code (and the signal that
  ended it, if one did), last run's reason, restarts, event log path and
  output log. It says when there are no sessions yet or the newest session
  directory has no manifest, and prints the last lines of
  `/Library/Logs/talos/launchd.stderr.log` when that file was written after
  the newest session started, as by a launch that failed before
  recording anything. No installed bundle is needed to inspect retained
  sessions.
- **fetch** copies every session folder from `<prefix>/logs` into `DEST`,
  preserving its manifest, every node's event logs (restarted attempts
  included) and output logs.
  Run it on the main processor, for example
  `./tools/deploy.sh fetch /media/usb/robot-sessions`, then transfer that
  directory to the analysis machine. Existing destination session names are
  rejected rather than merged; choose a fresh destination for each collection.
- **rollback** swaps `current` and `previous`: the whole previous bundle, with
  its own config. It refuses when `current` is missing or `previous` no longer
  holds a bundle. Selection does not change already-running processes: a
  running launcher restarts nodes from the bundle it started in.
  With the robot disabled, activate install/rollback with
  `sudo systemctl restart talos` on Linux, or
  `sudo launchctl kickstart -k system/org.talos.robot` on macOS.
- **prune** deletes installed bundles except the newest `--keep N` (default 3)
  and the ones `current` and `previous` select, however old. Prune after
  activating the selection, so the running bundle is among those kept.

Run by hand: `/opt/talos/current/run.sh --duration-s 30`. Stop the boot
service first (`sudo systemctl stop talos`; on macOS
`sudo launchctl bootout system/org.talos.robot`, and `enable-boot` again
afterwards): only one launcher may run a session in a shared-memory namespace,
so a second is refused with the running one's pid
([one session per namespace](../talOS/launcher/README.md#one-session-per-namespace)).

Boot services set `TALOS_LOG_DIR=<prefix>/logs`. The generated `run.sh` passes
that directory as the launcher's output **root**, unless an explicit
`--output-dir ROOT` is supplied. The launcher appends a unique session id on
every invocation, so recordings live in `<prefix>/logs/<session-id>/` and
restarts of the service never reuse a previous session folder. For a manual
run to use the same root, set `TALOS_LOG_DIR=/opt/talos/logs` or pass
`--output-dir /opt/talos/logs`; `logs LOG_ROOT` can inspect another output root.

Each node's stdout and stderr, every run of it, land in
`<prefix>/logs/<session-id>/<name>.log`, and the launcher's own lines in
`<prefix>/logs/<session-id>/talos.log` ([node output](../talOS/launcher/README.md#node-output)).

For event-log byte budgets, retention, capped recordings, manifest-write
recovery and what is synced to disk when, see [session segments and logs](../talOS/launcher/README.md#session-segments-and-logs).

### Boot service output

The launcher's own output is the record of a launch that fails before it
creates its session directory (a bad config, a held session lock); from that
directory's creation on, a refused graph included, its lines are also in
`<session>/talos.log`. Its errors go to stderr. On Linux both streams go to the journal (`journalctl -u talos`). The
macOS LaunchDaemon sends stderr to `/Library/Logs/talos/launchd.stderr.log`
and stdout (progress lines, already in the manifest, and the uncapped echo
of node output, already in each `<name>.log`) to `/dev/null`. Node output,
including a node's `realtime scheduling incomplete` line, is in its
`<session>/<name>.log`, not this file; the launcher's own warning for a
bridge or control loop without its realtime grant is here (and in
`<session>/talos.log`), and the outcome is
in the [session manifest](../talOS/launcher/README.md#session-manifest). launchd appends to it across
launches without marking where one starts. newsyslog, which macOS runs
hourly as root, renames it once it passes 1024 KiB and keeps four older
copies (`launchd.stderr.log.0` to `.3`), adding no line of its own, so a
crash loop, which launchd restarts at most every 10 s, cannot fill the disk;
a running launcher keeps appending to the copy it opened until it restarts.
newsyslog sets each new file's owner and mode by path, so the file lives in
a root-owned directory (`enable-boot` creates it `root:wheel` 0755 and the
file owned by the service account), never in the account's own log root,
where the account could put a link to a root file in its place.
`deploy.sh logs` shows the file's tail when it is newer than the newest
session.

## Supervision and service recovery

Boot probes each node before spawning the graph. `[subsystem] critical = true`
is the fail-closed default; failed probes refuse startup. Observation-only
nodes may explicitly set `critical = false`: a timeout, crash or import failure
then logs a warning without blocking the rest of the valid graph, and the
node is launched under restart supervision. Motors, digital outputs, PWM
outputs and described target-lane senders cannot be non-critical; the hardware
bridge is always critical. Naming and graph errors remain fatal, as does a
critical node's first run whose
[live endpoints](../talOS/launcher/README.md#live-endpoints) differ from its
probe (a non-critical node's is killed alone), so `run.sh` does not weaken the
graph checks with `--allow-graph-errors`.
`--describe-timeout-ms MS` adjusts the existing 15000 ms per-probe deadline
and can be passed through `run.sh`.

The Linux unit sets `TimeoutStartSec=1200` (20 minutes), independently of
the running watchdog. Description probes use the per-probe deadline above;
larger graphs or deadline overrides must fit the unit's startup budget.
Watchdog notifications begin only after READY, from the supervision loop,
never an independent heartbeat.

On Linux the launcher sends `READY=1` when the graph has started and enters
supervision, then `WATCHDOG=1` on each supervision iteration, which runs at
least every half of the unit's watchdog interval (1.5 s) and, while nodes
run, at least every quarter of their shortest heartbeat timeout (125 ms at
its 500 ms floor;
[supervision passes](../talOS/launcher/README.md#supervision-passes)).
With `WatchdogSec=3`, systemd treats three seconds without a
notification as a hung launcher and restarts it, leaving ample scheduling
headroom: the loop never waits for a disk sync. A stalled loop cannot be
masked by an independent watchdog thread. `Restart=always` starts a fresh
session after either a watchdog failure or a launcher exit, however often:
`StartLimitIntervalSec=0` lifts systemd's start-rate limit (five starts in ten
seconds by default, manual restarts included). `systemctl stop talos` still
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

Critical nodes never enter quarantine: crashes and stalls keep
restarting on the capped [restart backoff](../talOS/launcher/README.md#restarts),
even after exhausting the rolling budget, and only a critical node's first
crash or stall in the rolling window restarts it at once. Non-critical nodes
retain the circuit breaker: exhausting their budget quarantines them until the
oldest restart ages out, plus normal backoff.
Exactly one retry is pending, so clearing a transient fault can recover without a service
restart. The session manifest records `quarantined` and
`retry_at_monotonic_ns`; a stopping session cancels pending retries.
`--max-restarts 0` never restarts a node; `[robot] max_restarts` must be at
least 1. Registry version 6
(`/talos_registry.6`) uses host-monotonic heartbeat/liveness timestamps, so
wall-clock corrections cannot falsely evict healthy nodes; deploy all node
and viewer binaries together. Run one session's nodes in one pid namespace: a
node in another (a container sharing `/dev/shm` with `--ipc=host`) cannot be
restarted, because its dead topic claims stay until the session ends. Keep
them in one time namespace too: Linux shows process start times shifted by the
reader's time namespace, so a node in another would judge live claims dead.

## RoboRIO

`2026-robot/controller_processor/rio` is deployed with GradleRIO
(`./gradlew deploy` in that directory). It sends to the companion address it
was built with, port 5803: `10.56.87.11` unless the main processor's address
differs, then `./gradlew deploy -Pcompanion=<ipv4>`
([RoboRIO](../talOS/hardware/README.md#roborio)).
Deploy it from the same checkout as the main-processor bundle: each side
rejects frames of another protocol version (currently 7; see
[DATA_FLOW.md](DATA_FLOW.md)), and `hardware_node` logs both versions once
when that happens; the RoboRIO reports it to the Driver Station
(`companion/protocol_ok`, [RoboRIO](../talOS/hardware/README.md#roborio)).
Robot bring-up checklist: [BRINGUP.md](BRINGUP.md); before each match:
[PRE_MATCH.md](PRE_MATCH.md).

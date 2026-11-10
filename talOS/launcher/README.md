# Launching a robot

The launcher probes configured binaries with `--describe`, lints their declared
graph, creates session-owned topics, and supervises the roster.
Architecture and configuration invariants live in
[ARCHITECTURE.md](../../ARCHITECTURE.md); graph naming rules live in
[NAMING.md](../NAMING.md).
## Two phases

**Phase one asks.** Every binary in the config is run with `--describe`, up
to one probe per hardware thread at a time, so the roster takes about as long
as its slowest probe (graph readers, described again, as long as theirs). A node
given that flag builds itself on a `SimulatedEventLoop`, prints the manifest its
constructor registered as JSON, and exits zero. The simulated loop's channels
live inside the process, so this touches no shared memory, no hardware and no
network: probing a robot's roster is safe on a laptop with nothing plugged in.
The launcher parses those manifests, assembles the graph they describe, and runs
`naming::LintGraph` over it. Every finding is printed. If any of them is an
error, nothing is spawned and the launcher exits non-zero.

Subsystems default to `critical = true`. A failed probe (timeout, crash or
import error) is fatal for a critical node. Set `[subsystem] critical = false`
(inline: `[subsystems.<name>]`) only when the robot may run without that node.
Its probe failure becomes a loud warning: the remaining graph starts and the
node runs under normal restart supervision. The hardware bridge is always
critical. A non-critical node owning motors, digital outputs or PWM outputs,
or successfully describing a target-lane sender, is a graph error. Missing
descriptions cannot prove a node's outputs, so unknown nodes fail closed;
`critical = false` is an explicit deployment safety assertion, not inferred
from a name. Naming and graph violations still refuse startup.
`--describe-timeout-ms MS` sets each probe's deadline (default 15000 ms).

**Phase two starts.** Only then does the launcher fork the roster and supervise
it. It persists the session manifest before spawning and atomically replaces it
as nodes start, restart or exit, report a stopped event log, or run endpoints
that differ from their probe's ([live endpoints](#live-endpoints)); normal
shutdown adds final log counts.

`--describe-only` stops after phase one, which makes the check useful on its own
— in CI, in a pit before a match, or after a rename that touched two packages.

`--allow-graph-errors` overrides graph-error refusal, and the refusal of a
[live endpoint mismatch](#live-endpoints). Missing descriptions cannot create
segments for undeclared topics.

## One session per namespace

A launch that starts the roster (anything but `--describe-only` and
`--replay-inputs`, which use no shared memory) first takes the session lock of
its shared-memory namespace (`session_lock.h`), before it writes anything. A
session owns its namespace: it replaces every declared topic's segment at
startup and unlinks them at its end, so a second launcher there would remove a
running session's topics, and that session's restarted nodes would attach to
segments its other nodes no longer share. The second launch is refused instead:

    launcher: another launcher (pid 4242) is running a session in this shared-memory namespace (...)

The namespace is `ShmObjectName`'s ([ARCHITECTURE.md](../../ARCHITECTURE.md#topics-and-messages)):
sessions and test runs under their own `TALOS_SHM_NAMESPACE` or Bazel
`TEST_TMPDIR` do not exclude one another. The lock is a POSIX record lock on
`<lock-dir>/talos-<mapped name of /talos_session>.lock`, in the registry lock's
[directory](../introspection/README.md#the-segment); the kernel releases it
when the launcher exits or is killed, and nodes never inherit it. The launcher
touches the file hourly, so macOS's daily cleanup of `/tmp` files unused for
three days never deletes it under a long session, and reports it if the path
stops naming the locked file; acquiring it checks that it locked the file the
path still names.

Next, before any probe, the launcher prepares the
[live registry](../introspection/README.md#prepared-by-the-launcher), so every
node it starts can register, or refuses the launch naming the segment or lock
file that its nodes could not open.

## When a binary does not answer

A failed critical probe is an error; a failed non-critical probe is a warning.
`--describe-timeout-ms MS` sets the per-binary deadline. A binary that cannot
be run at all is reported with the step and errno that stopped it, as a spawn
is ([restarts](#restarts)): `did not answer --describe (execv <path>: Exec
format error)`, not an exit code 127. A probe is asked once, so a spawn
refused for want of a process slot or memory (`EAGAIN` or `ENOMEM` at `fork`
or `fork (guardian)`, as when the parallel probes reach the per-user process
limit) is retried, after 5 ms and then twice as long each time up to 200 ms,
until the describe deadline; past it the finding names that last cause and the
attempts made, and a probe that spawns only after a retry prints `[launcher]
probe of <binary> spawned on attempt N (after <step>: <errno>)`. Any other spawn failure is reported at once. Missing
descriptions remain recorded in `graph.json` and the session manifest.

## Live endpoints

The lint checks the graph the probes declared, so a node must declare the same
endpoints whether or not `Node::describing()`
([node development guide](../../docs/NODE_DEVELOPMENT_GUIDE.md)). The launcher
checks that each run did. Once per run of every member that answered
`--describe`, in the first supervision pass that finds the run's registry row
active with its sources published, it compares the row's topic ends with the
probe's: kind, topic, message size, alignment and the `external`, `optional`
and `observer` attributes, as multisets. Timers are not endpoints, and the
loop's latest-only flag is not declared by a probe. The schema hash is not in
the registry; it needs no comparison, because every declared topic's
session-owned segment was created with its declared schema and an end with
another one fails at attach, while an undeclared topic differs by name. A row
publishes its sources once per claim, before it clears `kFlagConstructing`
(or, for the hardware bridge and the simulation gateway, which never set it,
before any source shows), so the comparison never sees a half-written table;
a run that registers no source at all is not compared.

A difference is one error line naming the member, its pid and each end only
the live run or only the probe holds:

    [launcher] error: alpha (pid 4242): live endpoints differ from its --describe probe, the graph the lint checked: live only: SENDER /alpha/event (8 B, align 8); stopping the session. ...

and marks the member `endpoint_mismatch` in the live manifest (Studio's
session panel shows *endpoint mismatch*). The run's graph was never checked
(a second writer of a topic, a reader nothing writes), so it is then refused
the way the critical split already treats a failed probe
([two phases](#two-phases)) and a crash ([restarts](#restarts)):

- A non-critical member is killed (SIGKILL to its group) and not restarted
  for the rest of the session, which runs on without it, as it would after
  that member's failed probe. A session that may be driving never stops for
  an observation-only node.
- A critical member's first run stops the session, as a lint error refuses to
  start one: every member gets SIGINT, as on a requested stop, no restart is
  scheduled, and the launcher exits 1. A first run activates while the
  session starts.
- A critical member's restart that differs is only reported: the session may
  be driving, the launcher does not read the Driver Station to know, and
  stopping it or that member would take the robot down mid-match.

`--allow-graph-errors` keeps every member running with the mismatch reported
and recorded.

## Controller addressing

The hardware bridge talks to the controller processor over UDP, so a real-robot
launch has to say where that processor is. The address lives in the deployment
configuration, with a CLI override for one-off runs:

```toml
[robot]
controller_address = "10.56.87.2"  # RoboRIO as seen from this companion
controller_port = 5802             # gateway port on the controller
local_port = 5803                  # bind port on the companion
```


```sh
bazel run //talOS/launcher:launcher -- --config 2026-robot/main_processor/configuration/robot.toml
bazel run //talOS/launcher:launcher -- --config 2026-robot/main_processor/configuration/robot.toml --remote 10.56.87.2
```

`--remote` overrides `[robot] controller_address`; `--remote-port` and
`--local-port` override their config counterparts the same way. The address
must be an IPv4 literal (the bridge's socket resolves no hostname), and a port,
in the config or a flag, is an integer in [1, 65535]; config load or flag
parsing refuses anything else. The launcher
resolves the effective endpoint once per launch, prints it
(`[launcher] Controller <addr>:<port> ...`), and always starts
`hardware_node` with an explicit `--remote/--remote-port/--local-port` triple.
Running the bridge standalone resolves identically: its own `--remote` wins,
otherwise its `--config` supplies the address. Simulation requires the loopback
remote. On hardware, commissioning gates commands; see
[hardware configuration](../hardware/README.md#configuration).

## Lightweight simulation gateway

`--sim --start-sim-gateway` starts the binary named by `[robot] sim_gateway`
as a roster member. Repeat `--sim-gateway-arg ARG` to forward individual
arguments verbatim, in order, to **both** its `--describe` probe and runtime.
The forwarding flag requires `--start-sim-gateway`. For example:

```sh
bazel run //talOS/launcher -- \
  --config "$PWD/2026-robot/main_processor/configuration/robot.toml" \
  --sim --start-sim-gateway --sim-gateway-arg --physics \
  --sim-gateway-arg --wheel-radius-error --sim-gateway-arg 0.1
```

A `--sim` session runs its nodes and the bridge without
[realtime scheduling](../../docs/NODE_DEVELOPMENT_GUIDE.md#realtime-scheduling)
unless the launcher's environment sets `TALOS_REALTIME=1`, which every child
inherits: then the simulated gateway, commissioned in memory under `--sim`,
is driven under the robot's scheduling policy.


## Studio bridge

Sim sessions can put the Studio viewer bridge under launcher supervision with
`--start-studio-bridge` (what `sim.sh` passes unless `--no-studio`), so the
bridge is a declared node in `graph.json` instead of a process running behind
the launcher's back:

```toml
[studio]
bridge = "//studio/bridge:studio_bridge"  # binary target to supervise
http_port = 5800                          # HTTP/WebSocket port
webroot = "bazel-bin/studio/dist"         # static web directory, relative to the launcher's working directory
```

Every key shown is optional. The watched topic comes from `[studio] topic`,
the same setting the telemetry node reads. `--studio-http-port PORT`
overrides `[studio] http_port` for one run. Supervision is sim-only: without
`--sim` the flag has no effect, since a deployed session has no build tree
serving the webroot and no browser to open. The bridge answers `--describe`
like any other node, and the launcher starts it with the session's `graph.json`
as `--declared` and the session's `manifest.json` as `--manifest`; its stdout
and stderr land in `<output_dir>/<session_id>/studio_bridge.log`
(plaintext, not a `.tlog`) beside the manifest, like every member's
[output](#node-output), but are not echoed to the launcher's stdout.

The bridge is the one roster member supervised differently (`StudioBridge` in
`launcher.h`); it waits for the telemetry segment itself (see the
[bridge README](../../studio/bridge/README.md#publisher-and-serving-constraints)). Its
first attempt spawns with the nodes; a quick exit is retried at most once a
second for up to 60 seconds, inside the supervision loop, so it never delays the
session or its `--duration-s` deadline. While it is still starting it does not
hold the session open. Once it has survived a second it is supervised like any
node, except that it is never restarted under the policy below. If it never
comes up it is marked failed in the manifest and the session runs on without
Studio.

## Node output

Every roster member's stdout and stderr go to one pipe per run, which the
launcher drains into `<output_dir>/<session_id>/<name>.log`: every run of the
member appended to that one file after a line naming it, such as
`[launcher] alpha run 1, pid 4242` for the run that writes
`alpha.restart1.tlog`. Except for the Studio bridge, the output is also
echoed to the launcher's stdout. A deployed robot therefore keeps its nodes'
output beside their event logs, whatever happens to the launcher's own.

A print never waits for a disk or for the launcher's stdout: a one-line write
to a file on a loaded disk can take hundreds of milliseconds, far past a
node's `command_timeout_us`. One launcher
thread only reads the pipes into memory; others write the logs and the echo.
While a writer is behind, up to 2 MiB per member (and 2 MiB of echo) waits in
memory: a 1 MiB batch the writer has taken and a refilled 1 MiB pending buffer; beyond that output is dropped, and the log (or the echo) says how many
bytes where they were lost, as does a launcher line at the end of the session.
A member's log keeps at most 64 MiB per session, then says so and drops the
rest. The child's end of the pipe stays blocking: a nonblocking one would make
a full pipe an error inside the node (lost data and an error flag in stdio, a
silenced `std::cout`/`std::cerr` in iostreams, `BlockingIOError` from Python's
`print`). A node can still block if the launcher's reading thread is not
scheduled while it fills its pipe (64 KiB on macOS, 256 KiB requested on
Linux).

stdout is a pipe, not a terminal, so C stdio and Python buffer it in blocks
and a killed run loses what it had not flushed; write diagnostics to stderr.
`--replay-inputs` runs its one node with the launcher's own stdout and stderr.

## Restarts

A roster member — `hardware_node`, every subsystem node, `sim_gateway` — that
exits with a nonzero status or by a signal while the session is not stopping is
restarted. So is a critical member that exits cleanly (status 0) while the
session is not stopping: its peers still run, so it did not finish anything.
A run that cannot be spawned, a first run included, is retried the same way
and marks the member `recorder_failed`: a pipe or the fork failing (as under
`EAGAIN` or `EMFILE`), or the child failing before or at its exec (its process
group, guardian or output redirection, or `execv` itself: `ENOENT`, `EACCES`,
`ENOEXEC`, `ETXTBSY`). The child reports such a failure through a
close-on-exec pipe, which the launcher reads until the exec closes it, so the
line names the step and its errno where an exit status would only say 127:

    [launcher] cannot spawn alpha (execv /opt/talos/current/bin/alpha: Permission denied); retry in 50 ms

Every restart attempt counts against the budget below, spawned or not; the
retry keeps that run's log name and restart number.
The first restart waits 50 ms, each later one twice as long, capped at 1 s,
except that a critical member's first crash in the restart window restarts it
at once (its next restart there waits 100 ms). A clean exit always waits the
backoff, which gives a roster that is completing the time to finish (below).
Critical nodes (including every actuator owner and the hardware bridge) never
quarantine: they keep retrying with that capped backoff. Non-critical nodes may
restart at most `max_restarts` times within a rolling 60-second window (not
over the whole session):

```toml
[robot]
max_restarts = 5  # per node per rolling restart window
```

The key is optional (default 5) and must be an integer >= 1; `--max-restarts N`
overrides it for one run, and only `--max-restarts 0` disables every restart,
critical members' included, so a deployed config cannot leave the robot
without its drivetrain after one crash. A non-critical member's clean exit is a node that
finished and is not restarted. When a critical member's clean-exit restart
falls due and every other member's latest run also exited 0 (the whole roster
completed, as one-shot fixtures do), it is dropped and the session ends.
Nothing is restarted once a signal (requested stop, launcher shutdown) or the
`--duration-s` deadline has started stopping the session; each restart
decision reads the stop flag again, so a node killed by the same `systemctl
stop` that is stopping the launcher is not restarted. The backoff is
scheduled inside the supervision loop, like the Studio bridge's retries, so a
restarting node never delays another node or the deadline. Each restart prints
one line:

    [launcher] alpha exited (signal 11 SIGSEGV); restart 1 in 0 ms

When a non-critical node's rolling budget is exhausted, it is **quarantined**, not
permanently abandoned. Exactly one retry is scheduled for the oldest restart
plus the window and normal backoff. Quarantine and retry transitions are
logged; the live manifest exposes `quarantined` and
`retry_at_monotonic_ns` (zero with no pending restart). Stopping cancels the
retry; `--max-restarts 0` never schedules one. A persistent fault remains
rate-limited, while a cleared fault can recover later in the same session.

Every run writes its own event log rather than truncating the last one: the
first run `<name>.tlog`, restart `k` `<name>.restart<k>.tlog`. The session
manifest records each node's `restart_count` and every log in `log_paths`
(`log_path` stays the first run's), and its `record_count` sums all of them.
A log that reaches its byte share keeps its newest records: it is written in
segments, the one being written at its own path and older ones beside it as
`<stem>.segment<k>.tlog`, the oldest deleted to stay within the share
(`LogWriterOptions::max_bytes`); `record_count` sums the kept segments.
`log_dump` on the session directory merges every `*.tlog`, restarts and
segments included.
A node's `exit_code`, `exit_signal` and `pid` are its last run's (every
run's are in its `runs`, with the reason it ended; see
[session manifest](#session-manifest)):
`exit_code` is -1 while a run is live and after a signal ended it, and
`exit_signal` is that signal's number (0 for none), which the end-of-session
summary also names (`exit=-1 (signal 2 SIGINT)`); Studio's session panel
shows it in the exit column and marks the node *killed by signal*.

A run whose event log stops early says so in its registry row while it runs
(see [event log state](../introspection/README.md#event-log-state)). A node
creates its log on the writer thread once its loop starts, never on the loop
thread, so one whose log cannot be created (a missing or unwritable session
directory) keeps running with its row flagged, and exits 1 when stopped; the
hardware bridge also prints the reason once. The launcher prints one line per member and kind, and marks the member in the live
manifest: `recorder_failed` for a failed log, `log_capped` for one whose byte
share cost it records (its oldest segment deleted, or, with a share too small
for two segments, capture stopped at the share). At normal shutdown it also
marks a truncated, damaged or unreadable log `recorder_failed` and a capped one
(`<log>.capped`) `log_capped`; a log whose capture stopped at a record
boundary looks complete there, so only the live row reports it.

`--heartbeat-periods N` defaults to 10 periods, with a minimum timeout of
500 ms. `--heartbeat-startup-grace-ms MS` defaults to 5000 ms for processes
that have not registered. Such processes are killed once both that grace and
the heartbeat timeout have elapsed; registration is required even when
`--allow-graph-errors` permits a failed description. The launcher prepared the
registry before starting them ([one session per namespace](#one-session-per-namespace)),
and rows of killed runs are freed by the next claim. Rows held by live
processes outside the session are not, so a launch whose roster (every member
but Studio's bridge, which takes no row) does not fit beside them in the 32
rows is refused before anything starts, naming the holders.

Construction is visible in the registry before endpoints are available and has
a fixed deadline from process spawn. See the
[registry lifecycle](../introspection/README.md). Construction failures and
timeouts enter the ordinary restart/quarantine policy.
`--restart-window-ms MS` overrides the default 60000 ms rolling budget window.

Every spawned child has its own process group; restart, stop and shutdown
signal the group, and even a clean exit removes remaining helpers. A guardian
in each group watches a launcher-owned lifetime pipe and kills the group on
EOF, including after launcher SIGKILL. It never execs, so it closes every
other descriptor it inherited: a lock or pipe end the launcher's process held
at the spawn would otherwise stay held for as long as the node runs. Linux also
uses `PR_SET_PDEATHSIG`; macOS relies on the pipe. Children must not daemonize
out of their group.

The requested-stop supervision test acknowledges installed signal handlers in
all three stub nodes before signaling the launcher. Every stub stays alive until
that stop, and the test checks zero restarts and clean exits for the whole roster.
An immediately exiting hardware stub instead exercises an unrelated restart/stop
race: a signal before its executable starts can legitimately terminate it.

With `NOTIFY_SOCKET` set, the launcher sends `READY=1` on entry to supervision
and `WATCHDOG=1` on every loop iteration, using nonblocking Unix datagrams
(including Linux `@` abstract names). A hung loop therefore stops feeding the
service watchdog. The loop waits on no lock a node can hold: its registry
attach is read-only, which [takes no lock](../introspection/README.md#the-segment).
See [deployment](../../docs/DEPLOYMENT.md) for the generated systemd watchdog
policy and macOS's launchd limitations.

### Restart latency

From SIGKILL of a critical member to its first hardware request, measured on
a Linux x86-64 desktop (8 threads) by killing `drivetrain` 20 times in a
`--sim` session of `robot_physics_sim.toml` with the physics gateway under
`--restart-window-ms 1500`, so each is a first restart, session directory on
btrfs. The stages come from `CLOCK_MONOTONIC` stamps: the killer's, the
launcher's `waitpid` and `fork` and the restarted node's first constructor (an
`LD_PRELOAD` shim), and the restarted run's `.tlog` origin, first tick deadline
less its offset (`run()`) and first `/hw/request/drivetrain` send:

| Stage | ms, median (range) | Set by |
|---|---|---|
| SIGKILL → launcher reaps the exit | not yet measured | the SIGCHLD wakeup of the supervision loop |
| reap → `fork` of the restart | not yet measured | restart bookkeeping (`RestartLogBudget` lists the session directory); the pass posts its manifest version to the writer thread after its restarts |
| `fork` → first constructor | 1.2 (1.1–1.2) | group and guardian setup, exec, dynamic loading |
| first constructor → event-loop origin | 0.4 (0.4–0.5) | flags; the event-log file is created on the log's writer thread, not here |
| origin → `run()` | 1.5 (1.4–1.6) | robot config parse, topic attach |
| `run()` → first tick | 5.0 | one `period_us` |
| first tick → first `/hw/request/drivetrain` | 0.0 | that tick's flush, which needs a `/hw/state` received since the node attached |
| **total** | not yet measured | |

`restart_latency` repeats the `.tlog` part of this measurement on any host:
SIGKILL → origin (the first four rows together), origin → `run()`, `run()` →
first tick, first tick → first request and the total, as median (range). The
config's nodes come from `bazel-bin`, so build them first:

```sh
bazel run //talOS/launcher:restart_latency -- \
  --config 2026-robot/main_processor/configuration/robot_physics_sim.toml \
  --start-sim-gateway --sim-gateway-arg=--physics \
  --sim-gateway-arg=--port --sim-gateway-arg={remote_port} \
  --sim-gateway-arg=--remote-port --sim-gateway-arg={local_port}
```

It runs the session on loopback ports it picks (`{remote_port}` and
`{local_port}` in gateway arguments) under its own `TALOS_SHM_NAMESPACE`,
kills `--node` (default `drivetrain`) `--kills` times (default 10), spaced
500 ms past `--restart-window-ms` (default 1500), and removes the session and
its segments afterwards (`--keep` keeps the directory). The session lives in
`TMPDIR`, which selects the disk (a tmpfs `/tmp` is not one). `--disk-load N`
adds N processes rewriting and fsyncing 256 MiB files beside the session
during the kills. It then kills each Python node `--python-kills` times
(default 3) and reports its spawn (the process start time; 10 ms resolution
on Linux, 1 µs on macOS) to the restarted run's event-loop origin, after
which the node claims its registry row; the first launch is reported too,
which follows its `--describe` probe. `--evict-cache` drops the node's files
from the page cache between the kill and the restart (macOS: `sudo purge`);
pages another live process maps stay cached.

## Replaying a session into one node

`--replay-inputs <session dir> --only <node>` runs one roster member of the
config, and nothing else, on the topics a recorded session logged: no other
node, no shared-memory segment, no hardware bridge, no gateway.

```sh
./sim.sh --sim-gateway --no-studio --no-open --duration-s 15   # /tmp/talos_logs/<id>
bazel run //talOS/launcher -- \
  --config "$PWD/2026-robot/main_processor/configuration/robot.toml" \
  --replay-inputs /tmp/talos_logs/<id> --only pose_estimator \
  --output-dir /tmp/pose_estimator_replay
bazel run //talOS/launcher:log_dump -- /tmp/pose_estimator_replay/<new-session-id>
```

Only the selected node starts; its output is a new log, not a comparison with
the recorded outputs. The output directory must not be inside the input
session. See [input replay](../events/README.md#replaying-another-sessions-inputs)
for timing and schema checks.

## graph.json

Phase one writes `<output_dir>/<session_id>/graph.json` beside the session's
`manifest.json`: the session id, the config it came from, every node with its
name, target, whether it answered, and each source's kind, name, message size
and endpoint attributes, followed by every diagnostic. Studio's bridge serves it
so a viewer can show the declared graph next to the live one and mark the
difference.
It is written to a temporary file and renamed into place, as is the
`declared_graph.json` beside it that graph-reading nodes are described and
started with as `--graph`. A launch that cannot write either exits 1 before
anything starts.


## Session manifest

`<output_dir>/<session_id>/manifest.json` says what ran and how each run
ended (when it is written: [session segments and logs](#session-segments-and-logs)).

- `config_path`: the config, made absolute against the launcher's working
  directory (also in `graph.json`, and passed to every member as `--config`).
- `config_sha256`, `git_revision`, `git_dirty`: the bundle's config identity
  and the source revision of a `--config=release` build
  ([deployment](../../docs/DEPLOYMENT.md#build-a-bundle)); empty (and
  `false`) outside a bundle or for an unstamped one. The RoboRIO program's
  build is not recorded.
- per node, `binary_sha256`: the SHA-256 of `binary_path` as the session
  starts, empty if it cannot be read. It covers that file only: not the
  shared libraries a C++ node loads, nor a Python node's runfiles.
- per node, `runs`: one entry per run, spawn failures included: `pid` (-1
  when the spawn failed), `start_wall_ns`, `end_wall_ns` (0 while it runs),
  `exit_code`, `exit_signal` (as for the node's last run, above) and `reason`.
  At most 32 are kept, the first run and the newest; `runs_dropped` counts
  those dropped between them.

| `reason` | the run |
|---|---|
| `running` | has not ended |
| `exit` | exited 0 on its own |
| `crash` | exited nonzero or by a signal, the session stopping or not, and is none of the rows below |
| `stalled` | was killed (SIGKILL) because its registry heartbeat or dispatch count stopped |
| `heartbeat_missing` | was killed because no registry row of it was found once the startup grace and its heartbeat timeout had passed |
| `construction_deadline` | was killed for constructing longer than the deadline |
| `endpoint_mismatch` | was killed, or its session stopped, for [live endpoints](#live-endpoints) that differ from its probe |
| `stop` | ended as the session's stop (signal, `--duration-s`) asks: exit 0, SIGINT or SIGTERM |
| `stop_timeout` | outlived the 2 s after the stop's SIGINT and was killed (SIGKILL) |
| `spawn_failed` | could not be spawned (`exit_code` -1, `exit_signal` 0) |

A kill of the launcher's own (`stalled`, `heartbeat_missing`,
`construction_deadline`, `endpoint_mismatch`) names its run whatever exit or
signal is then observed, which `exit_code` and `exit_signal` still record.

## Building

```sh
bazel test //talOS/launcher/...
bazel run //talOS/launcher:launcher -- --config <path-to>/robot.toml --describe-only
```

Node targets resolve to `bazel-bin/<package>/<target>`, unless an override
names the binary. A bundle's `bundle_manifest.toml` (written by
`talOS/build/bundle_builder.py`, loaded by `--bundle` or found in the working
directory) lists each node's name, targets, `bin/` copy, its SHA-256 and
language, plus the config path and its SHA-256, and in a stamped build the
source revision ([session manifest](#session-manifest)). The launcher also reads a
workspace-relative config, so outside a bundle it only makes sense with the
workspace as its working directory; `bazel run` starts a binary in its
runfiles tree instead, and `main.cc` chases `BUILD_WORKSPACE_DIRECTORY` back so
both spellings behave the same.


## Session segments and logs

`--output-dir DIR` always names a root: each launch writes to
`DIR/<session_id>/`, including deployed `TALOS_LOG_DIR` roots. Without the flag,
the root is `/tmp/talos_logs`. Use persistent storage on a deployed robot.
`manifest.json` is written before nodes start, then replaced atomically
(written beside it, then renamed over it) at each process lifecycle change,
when a node reports a stopped event log and when a run's endpoints differ from
its probe: a reader always sees one complete version. `graph.json`,
`declared_graph.json` and every manifest version from the roster about to
start on are synced (`talos::process::DurableSync`, file
then directory: `fsync`, and on macOS `F_FULLFSYNC`, which also flushes the
drive's write cache), so a power loss mid-match keeps the last lifecycle
change whose sync finished. The supervision loop never touches the disk for
them: a write or sync can take hundreds of milliseconds while nodes write to
the same disk, so the loop hands each version to the manifest writer's thread
and moves on, and that thread writes the newest one (skipping versions
superseded while it was busy). A reader therefore sees a version shortly
after the change it records, not before. A killed launcher leaves its last
written version (`end_wall_ns = 0` means no normal shutdown). Each node's
event log file is synced with `DurableSync` when its loop ends (the file, not
its directory), and with a plain `fsync` once a second while it runs; a
capped log's rotation also syncs the finished segment and the next one's
header with `DurableSync`, then the directory, so a power loss leaves whole
segments (`LogWriter`).

At startup, retention removes older sessions: directories holding a
`manifest.json`, or, named by a session id, an event log whose header carries
that id (what `--replay-inputs` leaves). `[robot] log_keep_sessions` defaults
to 50, including the new session; `log_max_bytes` defaults to 2 GiB. Both must
be positive integers; config load refuses another type or value. Half the byte budget is reserved for the new session's event logs
and divided among logging members: a quarter of it equally, the rest by each
member's expected record rate (one record per declared source per
`period_us` tick, byte topics excluded since they are never logged;
`SplitLogBudget`). Restarts share their member's remaining budget, but never
get less than half of its share: to make that room the launcher deletes the
earlier runs' oldest log files (never the newest file of the run that just
ended) and marks those runs `log_capped`. Older sessions are retained within the other half.

Only a launch that starts the roster, and so holds the session lock,
removes anything; `--describe-only` and `--replay-inputs` launches, which run
beside a live session, remove nothing. Holding the lock is what tells a
session whose manifest has no `end_wall_ns` (its launcher lost power) from a
live one in the same shared-memory namespace (`TALOS_SHM_NAMESPACE`); a
launcher in another namespace must not share the log root. The newest 16
sessions (`kProtectedMatches`, `session_retention.h`) that ran a match or
that this build cannot classify are protected. Sessions are removed in this
order, each group oldest first by the `start_wall_ns` its manifest records
(the directory's modification time when it has none, as for
`--replay-inputs` output), never by the modification time of a session that
has one: sessions that ran no match, together with unprotected
unclassifiable ones; then unprotected matches; then the protected. The
protected are kept even beyond `log_keep_sessions`; one is removed only while
the older sessions still exceed their half of `log_max_bytes` once nothing
else is left, so that half stays the hard bound on retained event logs.
Removing a match or an unclassifiable session prints a `WARNING` to the
launcher's stderr naming it, its size and the limit it exceeded. Start times
are the main processor's wall clock, so a coprocessor without a real-time
clock or time sync orders sessions only as well as its clock survives a
reboot.

A session is a match when it has a launch manifest (so not `--replay-inputs`
output, even of a match), its launch was not `--sim`, and any node's event
log holds a received `/hw/state/driver_station` packet with the FMS
attached; nodes are read smallest first, each log streamed with
`event::log::VisitLog`, until one does. A session none of whose logs does,
but with a log of another format version or a packet this build cannot
decode, cannot be classified. The verdict is cached in the session's
`session_class` file and read before any log is opened: `match` for good,
`other` until the session's event log bytes change, `unknown` until they
change or this build's log format, `Packet` size or Driver Station codec
differs. A session's logs are therefore read once while that file can be
written.

A directory named
by a session id that holds nothing, or only what a launch writes before it
starts its nodes (`graph.json`, `declared_graph.json`, the `.tmp` of an
interrupted write, and the empty `.tlog` a node leaves when it dies between
its log writer creating the file and writing the header), is a launch that
recorded nothing:
`--describe-only`, a refused graph, or a launch that failed or died early. It
never counts against `log_keep_sessions` or the byte budget, and it is removed
once another session directory is newer: one newer than every other (a
`--describe-only` result just written, or a launch still describing) survives
the next launch, and a crash loop, refused graphs included, leaves one such
directory instead of evicting recorded sessions. Symlinks, other directories
(including one named by a session id that holds another session's log or any
other file) and the current session are not deleted. A session retention
cannot read or remove (another user's, unreadable, removed meanwhile) is
reported and passed over; it never stops a launch. Graph/manifest metadata, non-event captures and the members' output
logs (`<name>.log`, [node output](#node-output)) are outside the event-writer
caps: each member's output log can reach 64 MiB per session, so reserve
64 MiB per roster member per retained session beside `log_max_bytes`.

Simulation is only permitted with effective remote `127.0.0.1`. `--sim` with a
different configured or overridden remote is refused before any process starts.


Before spawning nodes, phase two calls `CreateSessionSegments` (`launcher.h`),
which creates every declared topic as a session-owned RTMS segment with its
schema name, schema hash, alignment, and slots. Nodes only attach. When the
session ends, the launcher destroys its queues and unlinks all session segments.

`log_dump` reads embedded topic descriptors and decodes recorded payloads
without the original robot build. Log format details live in
[events](../events/README.md#log-format) and byte sizes in
[DATA_FLOW.md](../../docs/DATA_FLOW.md#dispatch-logs).

### Filtering and exporting logs

Passing only file or session-directory paths retains the human-readable dump,
decoded payloads, and summary. Paths can be combined with:

- `--node name`: match a process name exactly.
- `--topic pattern`: match a topic exactly, or match its prefix when the pattern
  ends with `*` (quote wildcards to prevent shell expansion).
- `--since-ms ms`, `--until-ms ms`: inclusive event-time bounds in milliseconds
  relative to the earliest record in the entire input session, **before**
  filtering. Values must be finite numbers.
- `--format text|jsonl|csv`: default `text`; `jsonl` emits one JSON object per
  matching record; `csv` emits a header and RFC-4180 rows. Machine-readable
  formats omit the human-readable preamble and summary.
- `--config robot.toml`: the config whose device names label the hardware
  packets (below). Without it, `log_dump` reads the `config_path` of the
  `manifest.json` in a session directory it was given, when that file still
  parses; otherwise devices are identified by logical id only. The launcher
  records it absolute; a relative path (from a manifest written before it
  did, or given here) is resolved against `BUILD_WORKING_DIRECTORY` under
  `bazel run`, else the working directory.

Repeat `--node` or `--topic`, or separate their values with commas, to select
alternatives within that filter. Different filters are combined with AND.
Use `--` before a path beginning with `--`. Unknown flags and invalid values
are usage errors.

A damaged log does not stop the dump: its intact records are merged (the
reader skips a damaged record or a tail with no intact record, as a power loss
can leave; see [log format](../events/README.md#log-format)), and once it ends
`log_dump` prints `warning: damaged log <path>: <what was skipped, with byte
offsets>` on stderr, which `session_report` reads.

```sh
bazel run //talOS/launcher:log_dump -- /tmp/logs/session \
  --node drivetrain,pose_estimator --topic '/drivetrain/*' \
  --since-ms 1000 --until-ms 5000 --format jsonl
```

Both export formats contain `process`, `dispatch_index`, `kind`, `source_id`,
`topic`, `event_ms`, `now_ms`, `publish_ms`, `sequence`, `aux`,
`payload_bytes`, `schema`, `decoded` and `hardware`; JSONL also carries
`payload_hex`, the complete payload. Times keep the same session baseline
regardless of filters. `schema` is empty when the source has no schema;
`decoded` is empty without a schema or payload, otherwise it contains the
descriptor-decoded payload text (for a schema-less hardware Packet, a
truncated hex preview). JSON strings are escaped; CSV fields containing
commas, quotes, or newlines are quoted and embedded quotes are doubled.

The hardware packet topics (`/hw/state`, `/hw/command`, `/hw/request/<node>`,
`/hw/state/driver_station`) have no FlatBuffers schema, so `hardware` holds
them decoded with the wire codecs themselves (`hardware_record.h`): a JSON
object in JSONL, its text in CSV and the text dump, absent (JSONL) or empty
(CSV) for every other record. `type` is `state`, `command` or
`driver_station`:

- `command` (requests and the merged command): `config_id`, `boot_id`,
  `epoch`, `observed_time_us`, and per motor `id`, `name`, `mode`
  (`neutral`, `duty_cycle`, `voltage`, `velocity`, `position`,
  `motion_magic`), `slot`, `demand`, `feedforward_v`; digital and PWM outputs.
- `state`: the flags (`configured`, `enabled`, `command_active`,
  `hardware_fault`), and per motor `valid`, position, velocity, applied
  `voltage`, stator/supply current, temperature, sample ages, `fault_flags`,
  `faulted` (a fault other than an asserted limit), `forward_limit` and
  `reverse_limit` (hard limit switches); every other device's samples, health
  and gateway counters.
- `driver_station`: `mode` (`teleop`, `autonomous`, `test`, `none`),
  `enabled`, `estop`, attachment, alliance, station and match fields, and each
  connected joystick's `port`, `name`, `axes`, `buttons` (bit i is button
  i + 1) and `povs`.

64-bit ids are decimal strings. A device's `name` appears only when the
packet's `config_id` is that config's (as written, or commissioned as `--sim`
runs it), so a config edited after the match never mislabels a device. A
payload that will not decode gives `{"type": ..., "error": "..."}` and the
export continues. Driver Station strings are cut at a fixed length, so a byte
that does not begin a complete UTF-8 character is written as `\u00XX`. The
codecs are the reading build's: the Packet's schema identity hashes only its
size, so a log recorded before a wire change that kept that size decodes with
the new layout, wrongly or as an error; `payload_hex` keeps the bytes.

### Session measurements

```sh
bazel run //talOS/launcher:session_report -- --strict /tmp/logs/session
bazel run //talOS/launcher:session_report -- --json /tmp/logs/session
```

The offline Python tool reuses `log_dump` and its events reader/embedded
descriptor decoder, without robot-specific schemas. It reports manifest config
identity and source revision, node exits/restarts/quarantine/recorder
failures/caps, each node's binary digest and runs with the reason each ended
([session manifest](#session-manifest)), an unfinished
session, state inter-arrival median/nearest-rank p99/max,
enabled gaps of at least 100 ms, target MESSAGE receipt ages, timer dispatch
lateness, and every scalar `/hw/status` field.
Counter deltas sum nonnegative increments and separately count resets.
`clock_offset_valid` includes its observed valid fraction. JSONL `log_dump`
exports also carry `ds_enabled`, the Driver Station `hardware.enabled` bit,
for each Driver Station packet that decodes.

State timing selects one publisher stream (bridge IPC publications), or one
consumer stream if there is no publisher recording; duplicate readers are never
pooled. Enabled intervals hold the logged DS enabled bit until its next sample,
not a reconstructed safety gate. A gap overlapping an enabled interval counts.
Median “about 5 ms” uses an explicitly labeled 4–6 ms analysis tolerance; p99
must be below 10 ms. `--strict` exits 2 unless every required verdict is PASS
and the manifest shows a finished session (`end_wall_ns` nonzero) with no
recording node (non-empty `log_path`) `quarantined`, `recorder_failed` or
`log_capped`; host processes without an event log (`studio_bridge`,
`sim_gateway`) are not acceptance evidence and are exempt, and with no log
`log_dump` reports damaged (above), each named with its file and byte offsets.
It names each reason on stderr. It exits 1 for input/reader errors (including a missing `log_dump`
runfile) and 0 otherwise. NOT MEASURED is never a pass; only the timer
lateness row is informational and not required.

Target receipt age is `now - publish_time` of a target MESSAGE dispatch, not
target age at a control tick. Target age at the tick comes from the READ record
`Target::get` logs for the lane it first selects in each `update()` (`now - publish_time`
per `process:/<sub>/target/<source>`). Its required verdict interprets "about
25 ms at most" as a 25 ms maximum (an analysis interpretation) and names the
worst lane. It is NOT MEASURED when no target lane was dispatched or lanes were
dispatched only while disabled or released, and FAIL when a process was
dispatched active target lanes (envelope `active` with a nonzero
`mode_since_us`) while enabled but never selected one (for example a mode
generation mismatch). An age at or past `command_timeout_us` is
never recorded, because `Target` never selects such a lane; a producer stall
shows as ages rising toward the timeout. Timer lateness has no specified
acceptance threshold. Capped, truncated, unfinished recordings,
DS silence, missing topics and unknown intervals limit acceptance; retain the
raw session. Clock offset ranges measure the estimator, not absolute clock
synchronization accuracy. See `docs/BRINGUP.md` for the bench checks.

## If you change the shape of `graph.json`

Studio parses this document in TypeScript (`studio/src/system.ts`), and neither
language can check the other. `studio/tests/launcher_graph.json` and
`launcher_graph_faults.json` are real output from

    bazel run //talOS/launcher:launcher -- --describe-only \
      --config 2026-robot/main_processor/configuration/robot.toml

with only `session_id` pinned, and `//studio:declared_test` parses both. So a
change to the envelope here fails there rather than reaching a viewer as a
silently empty panel. Regenerate the fixtures and fix the parser in the same
commit; the faults fixture is the same robot with `drivetrain` publishing the
misspelled `/hw/req/drive` instead of `/hw/request/drivetrain`, which is where
the diagnostic wording is pinned.

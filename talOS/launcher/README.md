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
[live endpoint mismatch](#live-endpoints), but not the refusal of a misnamed
roster member ([node output](#node-output)). Missing descriptions cannot create
segments for undeclared topics. Since it overrides the safety refusals too, it
is accepted only with `--sim`, whose remote is loopback
([controller addressing](#controller-addressing)): without it the launcher
refuses the flag before it reads the config, so nothing is probed, written or
spawned, `--describe-only` included:

    launcher: --allow-graph-errors is accepted only with --sim; nothing was described or started

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

Locking an existing lock file allocates nothing on disk; creating an absent
one does. macOS empties `/tmp` at boot and keeps it on the same container as
a log root on the internal disk, so on a full disk the first launch after a
reboot cannot create the file (`ENOSPC`). When creating the absent file fails
with `ENOSPC` or `EDQUOT` and the lock directory is on the log root's
filesystem (same `st_dev`; never Linux's `/dev/shm`, a tmpfs log retention
cannot free), no launcher holds the lock that launch would take: it tries the
lock again once the config is read, runs
[retention](#session-segments-and-logs) without it if there is still no
room, then takes it, and is refused if it cannot. Another launch in the same
state may take the lock and start recording while that retention runs, so
retention without the lock removes only sessions whose manifest records an
`end_wall_ns` (their launcher shut down) and no launch that recorded
nothing. Any other lock failure refuses the launch before retention.

Next, before any probe, the launcher prepares the
[live registry](../introspection/README.md#prepared-by-the-launcher), so every
node it starts can register, or refuses the launch naming the segment or lock
file that its nodes could not open.

## When a binary does not answer

A failed critical probe is an error; a failed non-critical probe is a warning.
A member with no file at its binary path, as before a build, is not probed:
all such members make one finding that names them and ends with the command
that builds them, `bazel build` of their labels (in a bundle, it says the
bundle lacks them); it is an error unless every one of them is non-critical.
The [Studio bridge](#studio-bridge) is neither: a probe of it that does not
answer, its binary missing or not, is one warning of its own (with the same
build command or bundle note), and the session runs without it.
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

## Parameter keys

Each node's describe also lists every parameter key its constructor read with
`param` or `param_of`, set or not
([introspection](../introspection/README.md#describe)). After phase one the
launcher checks every subsystem table against them (`LintParameterKeys`): a
key that is not a framework key (`config::kFrameworkSubsystemKeys`) and that
no node read is an error naming the subsystem, the key as `param` names it
(`detector.sigma`, `geometry.modules[0].x_m`), the files that set it and,
when one is close, the nearest key a node read there:

    error: elevator: parameter 'homing_vv' (.../subsystems/elevator.toml) is read by no node's constructor (Node::param); did you mean 'homing_v'?

A key counts as read when a node read it, an element of it when the config
holds an array (`names[1]` reads `names`), or a key inside it when the config
holds an empty table (`routine.none.x` reads `routine.none`); a read under a
scalar (`recording.enabled` where `recording = false`) reads nothing the
config set. An empty table is one key, so an unknown table is refused too.
A subsystem whose own node did not answer its probe is not checked, and a
finding elsewhere stays an error that names that node (it may read the key
with `param_of`; fix its probe). A subsystem whose node answered without a
params field (a hand-written describe, or a binary built before the field)
is not checked either, which is a warning, and while any such node exists
every finding is a warning. Probes run with the session's `--sim`, so a key a
node reads only in simulation is unread on the robot.

## Live endpoints

The lint checks the graph the probes declared, so a node must declare the same
endpoints whether or not `Node::describing()`
([node development guide](../../docs/NODE_DEVELOPMENT_GUIDE.md)). The launcher
checks that each run did. Once per run of every member that answered
`--describe`, in the first supervision pass that finds the run's registry row
active with its sources published, it compares the row's topic ends with the
probe's: kind, topic, message size, alignment and the `external`, `optional`
and `observer` attributes, as multisets. Timers are not endpoints, and the
loop's latest-only flag, which a probe reports, is a delivery detail, not an
endpoint attribute. The schema hash is not in
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

Any session, with `--sim` or against the real controller, can put the Studio
viewer bridge under launcher supervision with `--start-studio-bridge` (what
`sim.sh` passes unless `--no-studio`), so the bridge is a declared node in
`graph.json` and a member in the manifest instead of a process running behind
the launcher's back; without the flag it is not started:

```toml
[studio]
bridge = "//studio/bridge:studio_bridge"  # binary target to supervise
http_port = 5800                          # HTTP/WebSocket port
webroot = "bazel-bin/studio/dist"         # static web directory, relative to the launcher's working directory
dashboard = "dashboard.toml"              # driver dashboard layout, relative to this file; no default
```

Every key shown is optional. `dashboard` names the driver dashboard's
layout ([talOS/telemetry](../telemetry/README.md#dashboard-layout)); config
load refuses a missing file, and the file is in the config's
`CONFIG_FILES` and its bundle. The watched topic comes from `[studio] topic`,
the same setting the telemetry node reads. `--studio-http-port PORT`
overrides `[studio] http_port` for one run. The default webroot exists only
in a checkout that has run `bazel build //studio:web_dist`; elsewhere set
`[studio] webroot`. The bridge needs both the binary (`[studio] bridge`,
resolved under `bazel-bin/` like a node's) and the webroot. A bridge whose
`--describe` probe does not answer (not built, absent from a bundle, or
unable to run: a missing library, a crash) is one warning in the graph,
naming the `bazel build` command or the bundle, and the launcher leaves it
out of the session, which runs without Studio, `--allow-graph-errors` or
not; a missing webroot lets it start and fail (below). This holds for the
member the flag adds only, never for a robot's own subsystem named
`studio_bridge`, which is an ordinary node; with the flag, such a subsystem
would share the bridge's name, and the launch is refused naming its
`[subsystems.studio_bridge]` key. A bundle built with `talos_bundle`'s
`studio_bridge` and `studio_webroot` carries both
([DEPLOYMENT.md](../../docs/DEPLOYMENT.md#build-a-bundle)): its manifest's
`[studio]` table registers the bundled bridge under its label and the name
`studio_bridge` (the name only for the member the flag adds: a subsystem
named `studio_bridge` whose node does not resolve is missing, never the
bundled bridge), and the bundle's webroot replaces a relative
`[studio] webroot` (an absolute one is kept). A bundle built without them has
no bridge: the flag adds none to the roster and the launcher says the bundle
carries no Studio. The bridge answers `--describe`
like any other node, and the launcher starts it with the session's `graph.json`
as `--declared` and the session's `manifest.json` as `--manifest`; its stdout
and stderr land in `<output_dir>/<session_id>/studio_bridge.log`
(plaintext, not a `.tlog`) beside the manifest, like every member's
[output](#node-output), but are not echoed to the launcher's stdout.

The bridge is the one roster member supervised differently (`StudioBridge` in
`studio_bridge.h`); it waits for the telemetry segment itself (see the
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
echoed to the launcher's stdout. Every line the launcher prints itself, from
the session directory's creation on, also goes to that directory's
`talos.log` (a line printed before it exists, such as a refused config, goes
to the terminal only). A roster member whose name breaks the
[node-name rule](../NAMING.md#nodes) is refused before anything is written,
even with `--allow-graph-errors`: its name is its file names, and `talos`
(or `Talos`, on a case-insensitive file system such as macOS's default APFS)
would write `talos.log`. A deployed robot
therefore keeps its nodes' output and the launcher's beside their event logs, whatever happens to the launcher's
stdout and stderr.

Neither a print nor the launcher waits for a disk or for the launcher's
stdout or stderr: a one-line write to a file on a loaded disk can take
hundreds of milliseconds, far past a node's `command_timeout_us`, and a
terminal or pipe nobody reads (a paused pager, an ssh session sshd has not
yet found dead) can take a write forever. One launcher thread only reads
the members' pipes into memory; others write the logs, `talos.log`, and the
launcher's stdout and stderr, and only that last one ever waits on them.
The launcher's own lines (`std::cout`, `std::cerr`) go into memory as well,
so supervision (restarts, stall kills, the stop) goes on whatever its
stdout does. While a writer is behind, up to 2 MiB per member, 2 MiB for
`talos.log` and 2 MiB for each of the launcher's stdout and stderr (one
buffer when they are the same file, as with `2>&1` or a terminal) wait in
memory: a 1 MiB batch the writer has taken and a refilled 1 MiB pending
buffer. Beyond that output is dropped, and the log (or the stdout or
stderr) says how many bytes where they were lost, as does a launcher line
at the end of the session for a member's log, and a `talos.log` line, at
the drain that finds them, for the stdout or stderr. A stdout or stderr whose
write fails for good (its reader closed it: EPIPE, as the launcher ignores
SIGPIPE) is not written again; a line on the other one and in `talos.log`
says so. At the end the launcher gives its stdout and stderr 1 s to take
what is left, then drops it and says how much in `talos.log` (an upper
bound: the part a write is still inside may yet arrive), and exits:
a write still in progress then (a terminal nobody reads takes part of it
and holds the rest) is abandoned 100 ms later, not waited for. Each such
note about the stdout or stderr is a line of its own in `talos.log`: one
that comes while a launcher line is half written there waits for that
line to end, or, at the latest, for the session's end. A member's
log, and `talos.log`, keep at most 64 MiB per session, then say so and drop
the rest. The child's end of the pipe stays blocking: a nonblocking one
would make a full pipe an error inside the node (lost data and an error flag
in stdio, a silenced `std::cout`/`std::cerr` in iostreams, `BlockingIOError`
from Python's `print`). A node can still block if the launcher's reading
thread is not scheduled while it fills its pipe (64 KiB on macOS, 256 KiB
requested on Linux).

Every process the launcher spawns starts with every signal at its default
disposition and none blocked, whatever the launcher inherited (`nohup`'s
ignored SIGHUP, `&`'s ignored SIGINT and SIGQUIT, its own ignored SIGPIPE).

A terminal hangup (an interactive ssh session or a terminal window closing
sends SIGHUP) stops the session in order, as SIGTERM does, unless the
launcher started with SIGHUP ignored (`nohup`), which it then keeps
ignoring. A closed pipe costs only the echo; a session that must outlive its
terminal runs under the boot service ([deployment](../../docs/DEPLOYMENT.md))
or `nohup`.

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

When the session ends, the launcher exits with 0, or with the last nonzero
`exit_code` among the members (-1, so 255, for one a signal ended), or 1 for
refused [live endpoints](#live-endpoints). A member whose last run ended by
the stop's own signal does not count: one that was not yet handling a stop
(between its exec and `process::InstallStopHandlers()`, a C++ node's first
lines or a Python node's whole interpreter start) when the stop's SIGINT
reached it, so an operator's Ctrl-C during startup ends the session with 0.
Its run still records the signal: `exit_code` -1, `exit_signal` 2, `reason`
`stop`. Only a SIGINT the launcher sent that run counts so, which it knows
from its own record of the stop it sent before it reaped the run, never from
the stop flag. The launcher sends it only after the spawn has returned, which
is after the member's exec. A terminal's Ctrl-C reaches only the launcher's
process group, never a member's (each has its own). Still counted: a
signal before the stop, another signal (a SIGTERM that systemd's default
`KillMode=control-group` sends every process of the unit along with the
launcher), and the stop timeout's SIGKILL.
A SIGINT from elsewhere that ends the run after the launcher's last look at
it and before its own SIGINT is indistinguishable from it and counts as the
stop's.

A run whose event log stops early says so in its registry row while it runs
(see [event log state](../introspection/README.md#event-log-state)). A node
creates its log on the writer thread once its loop starts, never on the loop
thread, so one whose log cannot be created (a missing or unwritable session
directory) keeps running with its row flagged, and exits 1 when stopped; the
hardware bridge also prints the reason once. The launcher prints one line per member and kind (for a capped log, whether it records on, deleting its oldest records, or stopped, which it tells by the segments beside the log), and marks the member in the live
manifest: `recorder_failed` for a failed log, `log_capped` for one whose byte
share cost it records (its oldest segment deleted, or, with a share too small
for two segments, capture stopped at the share). At normal shutdown it also
marks a truncated, damaged or unreadable log `recorder_failed` and a capped one
(`<log>.capped`) `log_capped`; a log whose capture stopped at a record
boundary looks complete there, so only the live row reports it.

A run reports in its registry row the realtime scheduling its loop asked for
and got ([realtime scheduling](../introspection/README.md#realtime-scheduling)).
For a bridge or control loop without its full grant (the policy or a memory
step refused, or, outside `--sim`, never requested) the launcher prints one
warning per run, naming the member, its tier, its grant and the refused steps
(the reasons are in the node's `<name>.log`), and the live manifest records
the worst outcome any of the member's runs reported (`loop_tier`, `realtime`,
`realtime_refused`; [session manifest](#session-manifest)). It does not stop
or restart the node.

A registered node is killed as stalled when its registry row shows no progress
for its heartbeat timeout. Once running, progress is the row's dispatch count,
which an event-loop node's loop thread stores at every dispatch (and the
hardware bridge from its own loop), never the heartbeat: the reporting thread
that heartbeats runs below every realtime loop, so a host busy with them can
starve it for longer than the timeout while the loop keeps time. While a node
constructs it has no loop yet, so its heartbeat (or a first dispatch) counts.
The launcher measures that interval between its own reads of the row: progress
is stamped with the time it reads just after the read that saw it move, and
judged at the time it reads just before each later read, so its own stall
anywhere in a supervision pass (its output blocked, a page fault, the host
descheduling it) can delay a kill but never causes one.
The kill's stderr line names the signal that stopped: the dispatch count, and
whether the heartbeat still moved or was unchanged too, or the construction
heartbeat. The heartbeat still bounds a running node at 5 s
(`introspect::kLivenessTimeout`): the next registry claim (a restart, or any
node starting) frees an active row whose heartbeat is that old, so a node whose
reporting thread is starved for 5 s loses its row, its loop stops storing the
dispatch count, and it is killed as `heartbeat_missing`.
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
out of their group. A child whose `setpgid` is refused still runs if it already
leads its own group, or else, after three refusals, as the leader of a new
session (and so of its own group). For such a spawn the launcher prints one
line: the errno and attempt number of the child's last refusal, the child's
pid, process group, session and parent, and the result of the launcher's own
`setpgid`; a spawn that fails there names the same in its error.

The requested-stop supervision test acknowledges installed signal handlers in
all three stub nodes before signaling the launcher. Every stub stays alive until
that stop, and the test checks zero restarts and clean exits for the whole roster.
An immediately exiting hardware stub instead exercises an unrelated restart/stop
race: the stop's signal reaching a restart before its trap is set ends it by
that signal.

With `NOTIFY_SOCKET` set, the launcher sends `READY=1` on entry to supervision
and `WATCHDOG=1` on every loop iteration, using nonblocking Unix datagrams
(including Linux `@` abstract names), and, when systemd sets `WATCHDOG_USEC`,
runs an iteration at least every half of it ([supervision passes](#supervision-passes)).
A hung loop therefore stops feeding the service watchdog. The loop waits on no
lock a node can hold: its registry attach is read-only, which
[takes no lock](../introspection/README.md#the-segment).
See [deployment](../../docs/DEPLOYMENT.md) for the generated systemd watchdog
policy and macOS's launchd limitations.

### Supervision passes

The supervision loop does not poll. After each pass it sleeps until the
earliest time a pass has something to decide, and a child's exit (`SIGCHLD`)
or a stop signal (`SIGINT`, `SIGTERM`, `SIGHUP`) starts a pass at once. Both
handlers write one byte to a self-pipe the loop sleeps on
(`SupervisionWakeup` in `spawn.h`), on Linux and macOS alike, so a signal
that lands between a pass's last look and its sleep is not lost, whichever
thread runs the handler. The sleep adds no thread, takes no lock a node can
hold and touches no disk. The times it sleeps until:

| Deadline | Decides |
|---|---|
| each running member's next look at its registry row (`RunLiveness::NextLook`): one look interval (its heartbeat timeout / 4) after the last, its stall deadline (the startup grace's end or one heartbeat timeout after the progress last seen, whichever is later), and its construction deadline until a look sees it constructed | `stalled`, `heartbeat_missing`, `construction_deadline`; the row's endpoints, recording and realtime flags and delivery counts, read at each look |
| each pending restart's start | the restart backoff, a quarantined member's retry |
| one retry interval (1 s) after the Studio bridge's last spawn, while it starts | its stability, retry or abandonment ([Studio bridge](#studio-bridge)) |
| `--duration-s`'s end, until the stop is sent | the stop |
| the stop timeout (2 s after the stop), until its SIGKILLs are sent | `stop_timeout` |
| half of `WATCHDOG_USEC`, when systemd sets it | `WATCHDOG=1` |
| the hourly refresh of the session lock and of the session directory's `session.lock` | [one session per namespace](#one-session-per-namespace), [retention](#session-segments-and-logs) |

Everything else a pass does follows an event or another pass: reaping exits
(with one more read of the run's row for its final delivery counts),
sending the stop, retrying a registry attach (at the next look), and handing
the manifest writer a version. A member's look interval bounds how late a
stall is seen: a run whose dispatch count stops at time S is killed by S plus
its heartbeat timeout plus one look interval (625 ms after S for the 500 ms
floor), plus however late the launcher wakes, and never before one heartbeat
timeout after the last look that saw it move. An idle session therefore
makes one pass per shortest look interval among its members, 8 a second when
every timeout is the 500 ms floor, whatever the number of members, since each
pass looks at every one.

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
session. The node runs under `--sim` exactly when the session did (its
`manifest.json`'s `simulation`; a session without one follows this launch's
`--sim`): under `--sim` the devices layer treats the config as commissioned,
which is part of the config id every recorded State carries, so a replay that
differed from the session would accept no State. A node binary run directly
with `--replay-inputs` takes `--sim` only as given. A topic the node did not
watch live (all of them, for a node the session did not run) is fed from
every log that recorded it, the writer's `SEND`s (the hardware bridge's
`--log` included) and readers' receipts, each publication once, with the
sequence and publication time live readers saw. A node the session ran
replays on its own recorded timeline: its
timers fire at the deadlines they fired at live, and it is handed what it
received live, at the ticks it received it, so an unchanged node sends what it
sent, except for what its loop's wake-up latency put into it (`now()`, and
ages measured from it). See
[input replay](../events/README.md#replaying-another-sessions-inputs) for
timing and schema checks.

A node that reads the declared graph (`telemetry`, `teleop`) replays on the
one the session was launched with, the session's `declared_graph.json` (a
node binary run directly with `--replay-inputs` also takes `--graph PATH`
instead). The launcher writes that file
only into a session that ran a graph reader, so replaying such a node from a
session without one fails, naming the missing file. The replayed node has no
session directory (`Node::session_dir()` is null, although the launcher
passes it `--log`), so state a node keeps per session or across restarts,
such as the autonomous node's routine selection and progress record, is
neither read nor written.

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
  build is not here: every `/hw/state` record of the bridge's log carries it
  (`log_dump`'s `hardware` field,
  [filtering and exporting logs](#filtering-and-exporting-logs)).
- per node, `binary_sha256`: the SHA-256 of `binary_path` as the session
  starts, empty if it cannot be read. It covers that file only: not the
  shared libraries a C++ node loads, nor a Python node's runfiles.
- per node, `loop_tier` (`observer`, `control` or `bridge`), `realtime` (the
  worst grant any run reported: `full`, `partial`, `not_requested` or
  `refused`) and `realtime_refused` (that run's refused steps, comma
  separated); all empty until a run reports
  ([realtime scheduling](../introspection/README.md#realtime-scheduling)).
- per node, `runs`: one entry per run, spawn failures included: `pid` (-1
  when the spawn failed), `start_wall_ns`, `end_wall_ns` (0 while it runs),
  `exit_code`, `exit_signal` (as for the node's last run, above) and `reason`.
  At most 32 are kept, the first run and the newest; `runs_dropped` counts
  those dropped between them.
- per node and per run, `wake_failures`, `stalled_arms`, `wake_fallbacks`
  and `requests_skipped`: what the run's registry row last showed it lost
  ([wake loss](../introspection/README.md#wake-loss),
  `introspect::DeliveryLossOf`), each normally 0. The launcher reads them
  in the supervision pass that already reads the row for liveness, and
  once more when it reaps the run: from the row the run released as it
  exited, which keeps its last sample until the next claim, or the row a
  killed run left. A row counts as the run's only while it carries the
  run's pid and session and a heartbeat no older than the run's spawn, so
  an earlier run's row whose pid the run reuses is never taken for it. So
  a run's values are its last sample, increases while the session stopped
  included (the pass stops reading rows for liveness then), unless another
  process claims the row it released before the launcher reaps it: the
  run's values are then what the last supervision pass read, and an
  increase in its final sample is lost. The node's fields sum every run, dropped ones included, and
  `requests_skipped_by_subsystem` splits the bridge's `requests_skipped` by
  the subsystem whose `/hw/request/<subsystem>` it passed over (only
  nonzero ones). A run, or a node, whose row was never seen (a spawn that
  failed, Studio's bridge) has none of these fields. A count leaving 0
  posts a manifest version of its own; a later increase waits for the next
  version the launcher posts anyway, at the latest the one the run's end
  posts, so a healthy session adds no version, and a killed launcher loses
  only the increases after a count's first.
- `launcher_log`: the launcher's own `talos.log` ([node output](#node-output)).
  A member's output log is not listed: it is always
  `<output_dir>/<session_id>/<name>.log`, and the launcher's end-of-session
  note of the bytes one lost is in `talos.log`.

| `reason` | the run |
|---|---|
| `running` | has not ended |
| `exit` | exited 0 on its own |
| `crash` | exited nonzero or by a signal, the session stopping or not, and is none of the rows below |
| `stalled` | was killed (SIGKILL) because its registry row stopped showing progress: its dispatch count once running, its heartbeat while constructing ([restarts](#restarts)) |
| `heartbeat_missing` | was killed because no registry row of it was found once the startup grace and its heartbeat timeout had passed |
| `construction_deadline` | was killed for constructing longer than the deadline |
| `endpoint_mismatch` | was killed, or its session stopped, for [live endpoints](#live-endpoints) that differ from its probe |
| `stop` | ended as the session's stop (signal, `--duration-s`) asks: exit 0, SIGINT or SIGTERM (which of these leave the launcher's exit status 0: [restarts](#restarts)) |
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

The sources are split into libraries along these seams, each with its own
header:

| Library | Files | Holds |
|---|---|---|
| `:session` | `session.h`, `session_lock.h`, `session_retention.h`, `file_digest.h` | manifest.json and run history, atomic writes, the session lock and session directory lock, what retention reads of a recorded session, the log budget split |
| `:spawn` | `spawn.h` | `SpawnProcess` (process group, guardian, child reports), `SupervisionWakeup` |
| `:child_output` | `child_output.h` | `ChildOutput`, the drain behind [node output](#node-output), and `LauncherOutput`, the launcher's own stdout, stderr and `talos.log` |
| `:roster` | `roster.h` | `LauncherOptions`, the bundle manifest, `DiscoverNodes` and binary resolution |
| `:describe_graph` | `describe_graph.h` | phase one: `ProbeDescribe`, `DescribeGraph`, the parameter lints, `EndpointMismatch`, session segments, `graph.json` |
| `:log_inputs` | `log_inputs.h` | `LogInputFiles`, the log files `log_dump`'s and `tlog_to_wpilog`'s inputs name, each once |
| `:log_merge` | `log_merge.h` | `MergeLogs`, a session's records streamed to a visitor in time order; `log_dump` and `tlog_to_wpilog` link it, not the supervisor |
| `:supervision_policy` | `supervision_policy.h` | the [restart](#restarts) and kill decisions as functions of the supervision clock: `RestartSchedule` (backoff, rolling budget, quarantine) at the time a pass sampled, `RunLiveness` (stall, missing row, construction deadline) at the times read around each row read, and when to look at the row next ([supervision passes](#supervision-passes)) |
| `:launcher_lib` | `launcher.h`, `studio_bridge.h`, `service_notifier.h` | the supervisor (`Launcher`; retention in `launcher_retention.cc`), which samples the clock, reads the registry, spawns, reaps and kills, and asks `:supervision_policy` what to do; and the Studio bridge |


## Session segments and logs

`--output-dir DIR` always names a root: each launch writes to
`DIR/<session_id>/`, including deployed `TALOS_LOG_DIR` roots. Without the flag,
the root is `/tmp/talos_logs`. Use persistent storage on a deployed robot.
`manifest.json` is written before nodes start, then replaced atomically
(written beside it, then renamed over it) at each process lifecycle change,
when a node reports a stopped event log, when a run's endpoints differ from
its probe and when a run's delivery-loss count first leaves 0
([session manifest](#session-manifest)): a reader always sees one complete
version. `graph.json`,
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
its directory), and with a plain `fsync` once a second while it runs (a
killed node loses only about its last 10 ms, which its writer had not yet
written: [log format](../events/README.md#log-format)); a
capped log's rotation also syncs the finished segment and the next one's
header with `DurableSync`, then the directory, so a power loss leaves whole
segments (`LogWriter`). The output logs, each member's `<name>.log` and the
launcher's `talos.log` ([node output](#node-output)), are never synced: a
power loss can cut the end of either. The launcher's lines before the
session directory exists (the session lock, the config, retention) go to
its terminal only.

At startup, retention removes older sessions: directories holding a
`manifest.json`, or, named by a session id, an event log whose header carries
that id (what `--replay-inputs` leaves). `[robot] log_keep_sessions` defaults
to 50, including the new session; `log_max_bytes` defaults to 2 GiB. Both must
be positive integers; config load refuses another type or value. Half the byte budget is reserved for the new session's event logs
and divided among logging members in proportion to the bytes each member's
log is expected to store per second (REQ-OPS-32; `log_budget.h`). From the
member's `--describe` and the config: a timer or sender records once per
`period_us` tick; a sequence watcher (a subscription with a callback)
records every message at its writer's rate, and a latest-only watcher at
most once per tick (`--describe`'s `latest_only`); a target-lane reader adds
one payload-free READ per tick. Each record is its 64 B header plus the
payload without trailing zeros: a user topic's message size, and for a
hardware packet its size word and encoded bytes from the config's devices,
not the 1436 B `hardware::Packet` (`/hw/state` once per gateway tick, the
Driver Station and `/hw/status` at 50 Hz, `/hw/request/<node>` at its node's
rate). The hardware bridge logs only the State it publishes and the commands
it sends, so only those count for it, the commands at one per actuator
owner's request: the rate while owners change their demands. Byte topics are never logged. A member that did not answer `--describe`
weighs as the mean of the others, and every member gets at least the
smallest share at which its log rotates (two segments of its prefix and
largest record) while those fit in the budget.

The shares give the session's log window, printed as the `Event-log window`
line. T is the budget left after the floors over the summed rates of the
members not held at a floor: what each of their shares holds at its rate.
A member's prefix cost is the time its eight segments' prefixes (header,
manifest and schemas, repeated per segment) take of its share at its rate.
The window is T less the largest prefix cost, but never less than 7/8 of T,
so a larger budget never shortens it and one member with large prefixes
cannot shrink every member's window. Every logging member is started with
`--log-window-us <window>` beside `--log-max-bytes <share>`, and its capped
log also ends a segment once its records span window / n
(`LogWriterOptions::window`; n = 8 segments while the share holds them;
never sooner than the writer's 1 s sync interval, so a window under 8 s
keeps more than the window where the bytes hold it). So in every
robot state a member that stores no more than its expected rate, and whose
prefix cost is within T less the window, keeps between 7/8 of the window
and the window of record time, whatever it
actually stores (a disabled `teleop` that sends nothing, a bridge whose
owners hold their demands), and leaves the rest of its share unused; a
member that stores more ends its segments on bytes and keeps a shorter tail
in proportion, and no other member's tail depends on it; a member that
records nothing for a while keeps a longer one. A member whose prefix cost
is larger ends its segments on bytes and keeps less. That includes every
member held at its floor: its share is two segments of its prefix and
largest record, so each segment holds about one largest record after its
prefix, and the log keeps about two records' worth of time (a 20 KB prefix
with 100 B records at 100 Hz keeps about 20 ms). No window is passed
(segments end on bytes only) when the floors do not fit or no member's rate
is known. Time-ended segments are smaller, never more numerous: a log still
holds at most n segment files of at most share / n bytes. Restarts share their member's remaining budget, but never
get less than half of its share: to make that room the launcher deletes the
earlier runs' oldest log files (never the newest file of the run that just
ended) and marks those runs `log_capped`. A restarted run gets the same
window; with less than its member's share, its segments may end on bytes
first. Older sessions are retained within
the other half, every file in them counted (event logs, output logs, camera
frames, metadata), except the protected matches below.

Retention runs once the config and roster are read and before the launch
writes anything to the log root, its session directory included: a flag or
config refusal removes nothing, and a launch on a full log disk frees room
before it needs any. The launch then creates `DIR/<session_id>/` and takes
its lock (below); when it cannot, it is refused, and the retention record
goes to stderr. When the
disk is still full once retention removed what it could, the refusal names
the free space and the free floor.

Only a launch that starts the roster, and so holds the session lock (or
finds its lock file absent with no room to create it, with the limits
[above](#one-session-per-namespace)), removes anything; `--describe-only` and `--replay-inputs` launches, which run
beside a live session, remove nothing.

Whether a session is still being written is read from its own directory,
not from the session lock, which covers one shared-memory namespace
(`TALOS_SHM_NAMESPACE`): launches in different namespaces, such as a
simulation beside a bench session or two developers' simulations, may share
a log root, the default `/tmp/talos_logs` included. Every launch,
`--describe-only` and `--replay-inputs` included, holds an exclusive `flock`
on its session directory's `session.lock` (`SessionDirectoryLock`,
`session_lock.h`) from before it writes anything there until it exits, and
the kernel releases it when the launcher exits or is killed. The descriptor
is close-on-exec and a guardian closes every descriptor, so no node or
helper holds it: a node that outlives its launcher (its guardian kills its
process group at once) or a process it daemonized never keeps a dead
session. Retention passes over every directory whose lock is held, without
counting or reading it, and says so on stderr. So a launch running in
another namespace in the same log root counts toward neither this launch's
`log_keep_sessions` nor its byte budget, and the free floor keeps room for
this session alone: two sessions running at once there may together hold
twice `log_max_bytes`, so size a shared log root for both. A session
whose manifest has no `end_wall_ns` and whose lock nobody holds was left by
a launcher that died (a power loss, a kill) and is retained like any other.
To remove a directory, retention takes its lock (creating `session.lock`
when it is absent), renames it to `.talos-removing-<session_id>` while
holding the lock, and removes it there; a launch creating that path
meanwhile finds the lock held or the directory gone and creates it again,
so no launch writes into a directory being removed. A
`.talos-removing-<session_id>` directory a removal cut short goes at the
next launch's retention. The supervision loop touches its `session.lock`
hourly, with the session lock (one `futimens` on the held descriptor, which
writes and syncs nothing itself), so a cleaner of `/tmp` files unused for
days (macOS's after 3, systemd-tmpfiles' after 10) never deletes it under a
long session in the default `/tmp/talos_logs`. If the path stops naming the
locked file (removed by hand, or by a cleaner before a refresh), retention
could take the directory from then on; the next refresh locks the file the
path names (creating it, never the directory) and says so once on stderr. That adds
no window: it and a retention lock the same file, so either the launch
holds it first and every later retention passes the session over, or a
retention holds it and is removing the directory, which is reported too. A
running launcher that holds no `session.lock` (one built before it existed)
is protected only from launches in its own namespace. So is one whose log
root's filesystem takes no advisory locks (NFS without a lock daemon,
macOS's msdos and exFAT volumes): its launch warns once and runs without
the lock rather than being refused, and retention there removes sessions as
it did before launches locked them. Such a launch tries the lock again at
each hourly refresh, and once it takes it (the lock and the probe failed in
one lock-daemon hiccup, and the daemon answers again) says so once on stderr
and is passed over by retention from then on. Whether a log root takes no locks is
decided once by retention, and once by the session directory's creation,
by a shared lock of the root's `.talos-lock-probe` file, never by one
failing call: on a root whose probe locked, a session whose `session.lock`
cannot be locked for such a reason (an NFS lock daemon that stops
answering) is passed over, with a warning, and a launch whose own lock
fails so tries again rather than running without it: up to 100 attempts
1 ms apart (about 0.1 s), after which the launch is refused (`cannot create
the session directory ...; nothing was started`, exit 1).

The newest 16
sessions (`kProtectedMatches`, `session_retention.h`) that ran a match or
that this build cannot classify are protected. Sessions are removed in this
order, each group oldest first by the `start_wall_ns` its manifest records
(the directory's modification time when it has none, as for
`--replay-inputs` output), never by the modification time of a session that
has one: sessions that ran no match, together with unprotected
unclassifiable ones; then unprotected matches; then the protected. The
unprotected go while there are more than `log_keep_sessions` sessions or
they hold more than their half of `log_max_bytes`. The protected count
toward `log_keep_sessions` but are never removed for it, and do not count
toward the byte budget, so a match larger than that half survives the next
launch. They go only when the filesystem holding the log root (`statvfs`)
has less free than the free floor once those limits are met: then sessions
continue in the same order, protected last, each removal counted as freeing
its bytes, until the floor is free. The floor is `log_max_bytes` plus
64 MiB per roster member plus 64 MiB: the new session's event-log half, an
output log of 64 MiB per member and `talos.log`'s 64 MiB, and the other
half for its camera frames and metadata. The launcher reads no node's parameters, so a session whose frame
recording (each camera's `[recording] max_bytes`, summed) exceeds half of
`log_max_bytes` is not covered by the floor.

Each launch records in its own session directory's `retention.txt` (written
once that directory exists, synced with `DurableSync`) every session it
removed, with its size, its class and the limit it exceeded; a launch that
removed nothing writes none. A directory holding a `retention.txt` is
never removed as a launch that recorded nothing (below), so the record
outlives a launch that died before its manifest. The removal of a match or
an unclassifiable session is also a warning on the launcher's stderr, and
the rest retention has to say goes only there: every running session it
passed over, every session it could not read or remove, a free floor it
could not reach, and free space it could not read. So a launch that stops
before its manifest beside a running session (a refused graph, a missing
binary, a service restarting it) still leaves an abandoned launch, not a
record to keep. Start times are the main processor's wall
clock, so a coprocessor without a real-time clock or time sync orders
sessions only as well as its clock survives a reboot.

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
starts its nodes (`graph.json`, `declared_graph.json`, `talos.log`,
`session.lock`, the
`.tmp` of an interrupted write, and the empty `.tlog` a node leaves when it dies between
its log writer creating the file and writing the header), is a launch that
recorded nothing:
`--describe-only`, a refused graph, or a launch that failed or died early. It
never counts against `log_keep_sessions` or the byte budget, and it is removed
once another session directory is newer: one newer than every other (a
`--describe-only` result just written, or a launch still describing) survives
the next launch, and a crash loop, refused graphs included, leaves one such
directory instead of evicting recorded sessions. Symlinks, other directories
(including one named by a session id that holds another session's log or any
other file), a running session and the current session are not deleted. A session retention
cannot read or remove (another user's, unreadable, removed meanwhile) is
reported and passed over; it never stops a launch.

Sizing: the new session's event logs stay within half of `log_max_bytes`,
but its members' output logs (`<name>.log`) and the launcher's
`talos.log` ([node output](#node-output), up to 64 MiB each), its camera
frames (each camera's `[recording] max_bytes`)
and its metadata are outside the event-writer caps. Older unprotected
sessions, everything counted, stay within the other half. The 16 protected
sessions are bounded only by their number, each up to one session's size.
Reserve the free floor (which covers the new session only while its frames
and metadata fit in half of `log_max_bytes`), the unprotected half, and 16
sessions of that size; when the disk holds less, the floor removes the
oldest sessions, matches last. At the default 2 GiB, four cameras recording
256 MiB each fill exactly the half the floor reserves for frames, and one
such pit session fills the unprotected half, so it goes at the next launch.

Simulation is only permitted with effective remote `127.0.0.1`. `--sim` with a
different configured or overridden remote is refused before any process starts.


Before spawning nodes, phase two calls `CreateSessionSegments` (`describe_graph.h`),
which creates every declared topic as a session-owned RTMS segment with its
schema name, schema hash, alignment, and slots. Nodes only attach. When the
session ends, the launcher destroys its queues and unlinks all session segments.

`log_dump` reads embedded topic descriptors and decodes recorded payloads
without the original robot build. Log format details live in
[events](../events/README.md#log-format) and byte sizes in
[DATA_FLOW.md](../../docs/DATA_FLOW.md#dispatch-logs).

### Filtering and exporting logs

Passing only file or session-directory paths retains the human-readable dump,
decoded payloads, and summary. A directory is every `*.tlog` in it, a file
is that log with its kept segments, and a log named more than once (its
session directory and the file, or the file twice) is merged once
(`LogInputFiles`, `log_inputs.h`). Paths can be combined with:

- `--node name`: match a process name exactly.
- `--topic pattern`: match a topic exactly, or match its prefix when the pattern
  ends with `*` (quote wildcards to prevent shell expansion).
- `--since-ms ms`, `--until-ms ms`: inclusive event-time bounds in milliseconds
  relative to the earliest record in the entire input session, **before**
  filtering. Values must be finite numbers.
- `--format text|jsonl|csv|summary`: default `text`; `jsonl` emits one JSON
  object per matching record; `csv` emits a header and RFC-4180 rows.
  Machine-readable formats omit the human-readable preamble and summary.
  `summary` prints one block per topic instead of the records (below).
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

Records are printed in merge order: by event time, then the record's `now`,
then its dispatch index, and on a full tie in the order the files are listed
(sorted by path within a directory, so a rotated log's older segments come
first). Every log is read through a fixed window (`WindowedLogReader`), never
whole, so memory grows with the number of logs, not with their length. Each
mode reads the session twice: once for the earliest event time every printed
time is relative to and for the record count, printing the damage warnings
then, before any output; once to print. On a generated 210 MiB session (8
logs, 1.72 million 128-byte records; `bazel run -c opt
//talOS/launcher:log_memory_bench`, Linux x86-64) `log_dump`'s peak resident
memory is 5.6 MiB, text or JSONL, where reading every log whole and holding
each record took 632 MiB. Each open log costs about 128 KiB (its window and
header buffer) plus its manifest and schemas, and keeps a file descriptor
open for the whole read.

```sh
bazel run //talOS/launcher:log_dump -- /tmp/logs/session \
  --node drivetrain,pose_estimator --topic '/drivetrain/*' \
  --since-ms 1000 --until-ms 5000 --format jsonl
```

`--format summary` answers "did it move" without reading every record. Each
matching topic (or timer) is one block, topics sorted by name and blocks
separated by a blank line:

```text
topic /gamma/state continuity_test.Count
  records gamma TIMER 3
  first_ms 2.000
  last_ms 22.000
  latest {run_index: 4, sequence_count: 9}
  range run_index 2 11
  range sequence_count 9 9
```

`records` counts the matching records per process and kind (a topic read by
two nodes is counted in each log). `first_ms` and `last_ms` are the earliest
and latest publication times, on the same baseline as the other formats (a
timer's event time). `latest` is the last merged record's decoded value
(`hardware` too for a hardware packet), and each `range` line gives a
numeric or bool field's smallest and largest value over the matching
records, as decoded, in field order (`message.<field>` inside a target
envelope, `<field>[i]` for an array element). The summary is folded in
during the streaming pass, so its memory grows with the topics and their
fields, not the records.

Both export formats contain `process`, `dispatch_index`, `kind`, `source_id`,
`topic`, `event_ms`, `now_ms`, `publish_ms`, `sequence`, `aux`,
`payload_bytes`, `schema`, `decoded` and `hardware`; JSONL also carries
`payload_hex`, the complete payload. Times keep the same session baseline
regardless of filters. `schema` is empty when the source has no schema;
`decoded` is empty without a schema or payload, otherwise it contains the
descriptor-decoded payload text (for a schema-less hardware Packet, a
truncated hex preview). JSON strings are escaped; CSV fields containing
commas, quotes, or newlines are quoted and embedded quotes are doubled.

A JSONL record of a `status` topic (a topic whose role segment is `status`)
also carries `alerts`, each alert field of its message in field order:
`{"name": <field path>, "level": "error"|"warning"|"info", "active": <bool>}`,
where an alert is a `bool` field whose own name
`naming::ParseAlertField` reads ([NAMING.md](../NAMING.md#topics)) and a
nested one is named by its path (`health.alert_warning_hot`). A record with
no alert field, of any other topic, or an `alert_` field that names no level
gets none; CSV and the text dump carry the state in `decoded` only.

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
- `state`: the gateway build that sent it (`gateway_program`: `roborio`,
  `roborio_simulation`, `sim_gateway` or `unknown`; `gateway_build_id`, the
  first 16 hex digits of its git commit, null when unknown;
  `gateway_build_dirty`), the flags (`configured`, `enabled`,
  `command_active`, `hardware_fault`), and per motor `valid`, position,
  velocity, applied `voltage`, stator/supply current, temperature, sample
  ages, `fault_flags`, `faulted` (a fault other than an asserted limit or
  an active current limit),
  `forward_limit` and `reverse_limit` (hard limit switches); every other
  device's samples, health and gateway counters.
- `driver_station`: `mode` (`teleop`, `autonomous`, `test`, `none`; `none`
  while no Driver Station is attached), `enabled`, `estop`, attachment, alliance, station and match fields, and each
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
failures/caps, each node's loop tier and realtime outcome, each node's
delivery loss (its nonzero `wake_failures`, `stalled_arms`, `wake_fallbacks`
and `requests_skipped`, the bridge's split per subsystem, as the manifest's
per-node fields of those names carry them; "none" when all are 0, NOT
RECORDED without them; [wake loss](../introspection/README.md#wake-loss)),
each node's binary
digest and runs with the reason each ended
([session manifest](#session-manifest)), each output whose bytes the
launcher's `talos.log` says were dropped (a member's `<name>.log`,
`talos.log` itself, the launcher's stdout or stderr, with the count; NOT
RECORDED for a manifest without `launcher_log`; informational, since the
output logs are not the measured record), an unfinished
session, state inter-arrival median/nearest-rank p99/max,
enabled gaps of at least 100 ms, target MESSAGE receipt ages, timer dispatch
lateness, the bridge's request-to-command delay, every scalar
`/hw/status` field, and the alerts.
The Alerts section lists each alert that was ever active, as
`<topic>/<field path>` with its level, from the `alerts` of its status
topic's records (above; the publisher's stream, else one reader's): its
first activation (milliseconds from the start of the log), activation count,
and longest and total active time. Times are publication times: an alert is
active from the first record showing it true to the first showing it false,
or, still active at the end, to its topic's last record, so they resolve to
that topic's publication period. The section is informational: `--strict`
does not judge alerts, at any level.
The bridge has no timer (it sleeps until a datagram, a request or a duty is
due), so it is measured instead by request to command: for each observation
(`boot_id`, `observed_time_us`) a `/hw/command` carries, its first command's
time minus the first `/hw/request/<node>` carrying it; a heartbeat repeating
an observation, or a command no request carried by then (such as an owner
timeout's neutral command, even when the node's next request observes the
same State), is not counted, so no value is negative. The request's time is
its tick's, stamped before that tick's work; the command's is its publication
on `/hw/command`, after the bridge's merge. So the row measures request tick
to command publication, the requesting node's work included, not delivery to
the RoboRIO: a command is logged when published whether or not its UDP send
succeeded (its record's status is the shared-memory write's), so after a
failed send, and while a socket is being replaced, the row still counts a
command the RoboRIO received later or never; the bridge's `send_failures`
(exit line, below) says whether that happened. With several owners the
merged observation can be an older request's, overstating it. The row is
informational: REQ-RT-20's 1 ms bound is verified by the bridge's component
test. The bridge's wake counters (`passes`, `throttled`, `wait_fallbacks`)
and socket counters (`send_failures`, `socket_reopens`) are only in its exit
lines on stdout, in its [node output](#node-output) log, not in the report.
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
`sim_gateway`) are not acceptance evidence and are exempt, with no log
`log_dump` reports damaged (above), each named with its file and byte offsets,
and with every bridge and control loop holding its full realtime grant. Off
`--sim` a `partial`, `refused` or `not_requested` outcome fails; under `--sim`
only a loop that asked (`TALOS_REALTIME=1`) and was refused a step does.
Observers, nodes that never reported, and manifests that predate the fields
(*NOT RECORDED*) are listed but not judged. It also fails on any node's
nonzero `wake_failures` (a reader left asleep until its backstop),
`requests_skipped` (a request its subsystem never merged) or
`wake_fallbacks` (a run that drained on a poll tick instead of on
publication, so its prompt handlers' latency and the no-polling guarantee
did not hold); `stalled_arms` (the guard against a transport misreporting
unread data, which still drains that watcher on timers and the backstop)
is shown but not judged, and neither is a manifest without the fields.
It names each reason on stderr. It exits 1 for input/reader errors (including a missing `log_dump`
runfile) and 0 otherwise. NOT MEASURED is never a pass; only the timer
lateness and bridge request-to-command rows are informational and not
required. The enabled-only verdicts (enabled state median, p99 and gaps,
target age at tick) cannot be measured in a session that never enabled, and
the report says so in one line; `--strict` still fails them, so acceptance
needs an enabled session.

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

### Converting a session to WPILOG

`tlog_to_wpilog` (`//talOS/launcher:tlog_to_wpilog`) writes a session's
`.tlog`s as one WPILib DataLog (`.wpilog`, format 1.0), the file
AdvantageScope and WPILib's DataLogTool open. It streams the logs through
the same `MergeLogs` as `log_dump`, decodes payloads from the
schemas the logs carry (no recording build needed), and writes the file with
its own small writer (`wpilog.h`; no WPILib dependency). Offline only.
REQ-OPS-26 states what it guarantees.

```sh
bazel run //talOS/launcher:tlog_to_wpilog -- /tmp/talos_logs/<session> \
  -o /tmp/match.wpilog
bazel run //talOS/launcher:tlog_to_wpilog -- a.tlog b.tlog -o /tmp/ab.wpilog
```

Inputs are `.tlog` files or session directories, all from one session,
read as [`log_dump`](#filtering-and-exporting-logs) reads them: kept
segments and restarts included, and a log named twice read once. Then open the file in AdvantageScope
(File > Open Log(s)...): topics appear under their names, and a pose entry
(below) can be dragged onto the 2D Field view as a robot pose.

Mapping:

| Recorded | WPILOG entry | Type |
|---|---|---|
| a scalar field of a struct message | `<topic>/<field>`, nested structs joined with `/` (`/drivetrain/state/module0/angle_rad`) | from the FlatBuffers base type: `double`, `float`, `boolean`; every integer and enum `int64` |
| a fixed array of scalars | `<topic>/<field>` | `double[]`, `float[]`, `boolean[]`, `int64[]` |
| a fixed array of structs | `<topic>/<field>/<index>/<subfield>` | per element field, as above |
| a target lane (`TargetEnvelope<M>`) | `<topic>/mode_since_us`, `<topic>/active`, `<topic>/message/<field>` | `int64`, `boolean`, as above |
| a `geometry.Pose2d` (talOS/msgs), as well as its three fields | `<topic>/<path to it>`, or `<topic>/pose` when the Pose2d is the whole message | `double[]` `[x_m, y_m, yaw_rad]`, AdvantageScope's legacy number-array pose |
| a reader's dropped count (`aux` of a MESSAGE record) | `/talos_log/dropped/<node><topic>`, cumulative | `int64` |

Field names keep their unit suffix (`position_m`), and each entry's metadata
is JSON naming the schema and, from the suffix, the unit
(`{"source":"talos.tlog","schema":"geometry.PoseEstimate","unit":"m"}`).
Pose2d is the only extra mapping; other messages (a swerve module position,
an odometry sample) appear only as their fields.

An entry name belongs to the first topic to publish a value under it. A
topic whose field path lands on a name another topic owns (`/arm/state`
field `left.angle_rad` and topic `/arm/state/left` field `angle_rad` are
both `/arm/state/left/angle_rad`) gets `<topic>:<field path>` instead
(`/arm/state/left:angle_rad`), which no topic or field name can spell, and
the converter prints one warning naming both.

Which records, and when:

- Each publication is written once. A publication is its topic, RTMS
  sequence and publish time, which its writer's `SEND` record and every
  reader's `MESSAGE` record of it share. Its values come from the `SEND`
  when the writer's log holds it, otherwise from the first `MESSAGE` of it
  in merge order, from any reader. So a reader's copy fills what the
  writer's log lacks (the oldest segments a byte cap deleted, a tail cut by
  a crash or a stopped capture), and a writer that was not logged at all is
  rebuilt from everything its readers received. A send the transport
  refused (`aux` not OK) was never published and is skipped.
- Each value is stamped with the message's publish time, the record's
  `publish_time_ns` (a SEND's is the publishing dispatch's `now`; a
  MESSAGE's is the writer's, carried with the payload), so a writer's copy
  and a reader's copy carry the same time. That is the recording machine's
  monotonic clock (`CLOCK_MONOTONIC` in a live session, the simulated clock
  under `--sim` or replay), in microseconds after the earliest log start
  (`FileHeader::start_monotonic_ns`) of the input. A dropped count is
  stamped with the publish time of the message after the gap. Wall-clock
  time is not used.
- Timer, exit, scheduling and READ records carry no message and are skipped. Byte
  topics (`talos.Bytes`, such as `/talos/telemetry`), the hardware packet, and
  any topic whose schema cannot be read are skipped, each named once on
  stderr.

Caveats: a `ulong` field becomes `int64` with its bits kept, so a value above
2^63 - 1 reads negative (a `sample_time_ns` never gets there); enums are their
numeric value, not their name; hardware packets (`/hw/state`,
`/hw/request/<node>`), which `log_dump` decodes with the wire codecs, are not
converted. Damage is read as `log_dump` reads it: a damaged record or a cut
tail is skipped with a warning naming the log, and a log cut inside its
header is skipped. A file that is not a `.tlog`, a manifest that fails its
checksum, an unsupported format version, a mismatched session id, or no
readable log is an error: the converter prints `tlog_to_wpilog: error: ...`
naming the file and exits 1 without writing the output. Usage errors also
exit 1.

The output is written to a temporary file in its directory
(`.<name>.tmp.XXXXXX`) and renamed over the output only once it is complete,
so a write that fails (a full disk) exits 1, removes the temporary file and
leaves no partial output and any previous output unchanged. The output path
is checked before any input is read: one that names something other than a
regular file (a device, a directory), or a directory that cannot take a new
file, fails with `cannot write <output>`. A symlink at the output path is
replaced by the new file, not written through.

Memory: the merge itself holds a window per log, as `log_dump`'s does
([filtering and exporting](#filtering-and-exporting-logs)), and reads the
session twice: once for the publications a writer logged, once to select.
The converter keeps, per publication, its key in the dedup sets and, until
it has sorted them by publish time, each selected value's payload and
source. On the same generated 210 MiB session (1.72 million publications of
64 bytes, each a value; `log_memory_bench`) its peak resident memory is
525 MiB, where holding the whole merged session took 681 MiB. Convert on a
development machine, not the robot.

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

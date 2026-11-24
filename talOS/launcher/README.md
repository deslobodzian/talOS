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

Beside the graph, `naming::CheckHardwareConfig` checks the hardware config
for advisory limits, warnings that never refuse a launch: a motor whose
`max_velocity_rps * sensor_to_mechanism_ratio` passes 105 rps, and each CAN
bus whose estimated utilization passes 70 %. The estimate is per bus, from
that bus's Talon FX, CANcoder and Pigeon 2 counts at the config's
`status_hz`, with CTR Electronics' per-device figures (classic CAN on
`"rio"`, CAN FD on a CANivore) plus 5 % for the vendor's diagnostic server,
calibrated to the bench's measured CANivore utilization
(`EstimateCanBusLoads` in `talOS/introspection/hardware_lint.h`); the warning names
the bus, its estimate and its device counts.

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

`--duration-s N` bounds the session: N seconds after the launcher starts
spawning the roster, it stops the session as a signal does
([restarts](#restarts)). Members are started without a `--duration-s` of
their own, so only that stop ends a member that runs well; a member whose
launcher dies is killed by its guardian ([restarts](#restarts)). A launcher
that is alive but stuck sends no stop and its guardians do not act: its
members run on until the launcher is killed.

A launch that starts the roster handles a stop (SIGINT, SIGTERM, or a
[hangup](#node-output)) in order from its first step, before it takes the
[session lock](#one-session-per-namespace), so a stop at any point leaves
nothing behind that a normal end would not. Before phase one ends, the stop
kills every probe still running, with its process group, at once rather
than at its describe deadline, and starts no other; the launch then writes
no manifest or segment and exits 0 (a stop that lands while the described
graph is being linted still leaves its `graph.json`, and a graph with errors
is still refused with status 1), its session directory a
launch that recorded nothing ([session segments and logs](#session-segments-and-logs)).
Once phase two has created the session's topic segments, a stop starts no
member that has not started yet (the manifest's `stopped_before_start`),
stops those that have as every stop does ([restarts](#restarts)), writes
the final manifest and unlinks the segments. The launch keeps its stop
handling until every member is reaped and the segments are unlinked, and
then puts back the dispositions it found. `--describe-only` and
`--replay-inputs` keep the default action: a stop ends the launcher at
once, and each probe's or the replayed node's guardian kills its group.

`--describe-only` stops after phase one, which makes the check useful on its own
— in CI, in a pit before a match, or after a rename that touched two packages.

`--describe-only` judges the graph, not whether this launch could start: it
takes none of the refusals a launch makes before its probes that depend on
the launch's own flags or on the host. `--describe-as-launch`, accepted only
with `--describe-only` (alone it would start the roster, and is refused
before the config is read), adds those that need nothing a running session
holds; `deploy.sh`'s [graph check](../../docs/DEPLOYMENT.md#graph-check)
passes it. Of a launch's refusals before its probes:

| Refusal | `--describe-only` | with `--describe-as-launch` |
|---|---|---|
| `--allow-graph-errors` without `--sim`; `--sim-gateway-arg` without `--start-sim-gateway` or naming a port; an unloadable bundle or config; an invalid `--remote` or `--local-address`; `--sim` with a remote other than `127.0.0.1` or a `local_address` other than unset/`127.0.0.1`; a bad restart setting; a duplicate or misnamed member | refused | refused |
| [`[robot] simulation_only`](#lightweight-simulation-gateway) without `--sim` | accepted | refused |
| the [session lock](#one-session-per-namespace) held by another launch | not checked | not checked: it is held by the session a check runs beside |
| the [live registry](../introspection/README.md#prepared-by-the-launcher) cannot be prepared (registry capacity is checked after probes, once any unanswered Studio bridge is dropped, and is skipped by both modes too) | not checked | not checked: it is shared memory, and its rows are the running session's |

A port the bridges bind is not checked either way: it is bound only by a
member that has started (the hardware bridge's link, the Studio bridge's
HTTP server).

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
startup ([segment lifetime](../rtms/README.md#segment-lifetime)) and unlinks
them at its end, so a second launcher there would remove a
running session's topics, and that session's restarted nodes would attach to
segments its other nodes no longer share. The second launch is refused instead:

    launcher: another launcher (pid 4242) is running a session in this shared-memory namespace (...)

`--check-session-lock` runs that lock check on its own without a config,
exiting 0 if the namespace's session lock is free and 1 naming the holder on
stderr if it is held (`sim.sh` calls it before starting WPILib simulation, and
before `bazel build` when the launcher binary is already built).

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

In a namespace (`TALOS_SHM_NAMESPACE`, or Bazel's `TEST_TMPDIR`) a launch
that started the roster removes, once every member is reaped, what it leaves
besides its topic segments: the session lock file, while it still names the
locked file, and the namespace's registry segment with its writer lock file,
unless the registry's writer lock is held at that instant or a row is held
by a running process outside the session (a node or tool run without the
launcher; a row whose process is gone, such as a SIGKILLed node's of an
earlier session, does not count). A namespace is rarely used twice, so the
segment would otherwise stay in shared memory (`/dev/shm` on Linux) until
reboot, and so would the lock files in the lock directory, except under
Bazel, whose `TEST_TMPDIR` holds them and is removed with the test.
Outside a namespace the ordinary names, one set per host that tools and
hand-run nodes share, are left in place.

The session owns, creates and unlinks only the topics its probes declared
(`DescribeGraph`). A run that opens an undeclared topic (a member whose live
endpoints differ from its probe, or one with no description under
`--allow-graph-errors`) creates that segment itself, as a node run without
the launcher does ([ARCHITECTURE.md](../../ARCHITECTURE.md#topics-and-messages)):
its last queue to close unlinks it, but a run killed while it is the last
holder (the SIGKILL of a [live endpoint](#live-endpoints) mismatch, a stall,
the stop timeout) leaves it in shared memory until the next queue to open
that name reuses it, someone unlinks it, or the host restarts; neither the
session's end nor a namespace's cleanup removes it.

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
when one is close, the nearest key a node read there, one error per key with
the remedy (delete the key, or read it with `param`):

    error: elevator: parameter 'homing_vv' (.../subsystems/elevator.toml) is read by no node's constructor; delete it from the config, or read it in the node's constructor with Node::param (node.param in Python); did you mean 'homing_v'?

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

## Python actuator periods

A Python node adds its interpreter's and garbage collector's jitter to every
tick, so one that owns actuators runs at 20 ms or slower
([METHODOLOGY.md §10](../../docs/METHODOLOGY.md#10-performance)). After phase
one the launcher checks every subsystem whose own node's describe reports
`abi_update` (its `update()` is a callback through the C ABI: a Python node's;
a `talos.cpp_node`'s `update()` is C++ and is not one) and whose devices own a
motor, digital output or PWM output (`LintPythonActuatorPeriods`): a
`period_us`, its own or else `[robot] period_us`, below 20000 is an error
naming the subsystem, its period and the minimum:

    error: motor: a Python node owning a motor, digital output or PWM output runs at period_us 5000, below the minimum of 20000 us for one (docs/METHODOLOGY.md §10); set its period_us to 20000 or slower, or write the node in C++

Since an actuator owner's `period_us` is also at most a quarter of
`command_timeout_us` ([the tick](../../ARCHITECTURE.md#the-tick)), a config
with a Python actuator owner needs a `command_timeout_us` of at least 80000.

## Telemetry labels and dashboard layout

After phase one the launcher runs `LintTelemetryConfig` over every telemetry
subsystem's `[[subsystem.labels]]` entries and any `[studio] dashboard` layout
using the same validator the telemetry node runs at construction
([talOS/telemetry](../telemetry/README.md#dashboard-layout)). An invalid
`[[subsystem.labels]]` entry (bad structure, unknown key, missing/non-string
field, bad `from`, missing/non-string-array source, over 1024 labels or a
label past the string limit) or a `[studio] dashboard` layout the file alone
refuses (syntax, schema, grid or budget) is a phase-1 error naming the
subsystem or layout file and key. It refuses `--describe-only`,
`deploy.sh install`, and launch even when `telemetry` is marked
`critical = false` (overridden in simulation only with
`--sim --allow-graph-errors`). A finding checked against the declared graph's
dictionary (a label's channel absent from it, not an integer, or already
labelled; a layout channel absent from or excluded from it; a dictionary or
drive feed over its limit once labels and alerts are counted) is that error
too when every node other than telemetry and the Studio bridge answered
`--describe`. When one did not (a `critical = false` node that does not
answer only warns), its channels are missing from the dictionary, so such a
finding is a warning that says the declared graph is incomplete and names the
nodes that did not answer, and the launch is not refused for it. With no
such node answering at all, nothing is checked against the dictionary.

## Live endpoints

The lint checks the graph the probes declared, so a node must declare the same
endpoints whether or not `Node::describing()`
([node development guide](../../docs/NODE_DEVELOPMENT_GUIDE.md)). Two checks
enforce that:

- **Targeted-outputs construction gate (REQ-SAFE-42).** When a member's
  `--describe` probe answered and declared a `hardware_request` endpoint without
  `untargeted_outputs`, and `--allow-graph-errors` is not set, the launcher
  passes `--targeted-outputs` to that member on its first run and on every
  restart (`Restart` reuses `launch.args`). A node started with
  `--targeted-outputs` refuses `talos::AllowUntargetedOutputs`
  (`talos.allow_untargeted_outputs` in Python) at construction with
  `std::logic_error` (`TalosError`) and exits non-zero before its loop starts,
  so an opt-out hidden behind `!Node::describing()` can never tick untargeted
  on a first run or after a critical restart. Non-actuator nodes,
  `hardware_node`, `sim_gateway`, `studio_bridge`, nodes whose probe declared
  `untargeted_outputs`, unanswered probes and `--allow-graph-errors` launches do
  not receive `--targeted-outputs`.
- **Live endpoint comparison.** Once per run of every member that answered
  `--describe`, in the first supervision pass that finds the run's registry row
  active with its sources published, the launcher compares the row's topic ends
  with the probe's: kind, topic, message size, alignment and the `external`,
  `optional`, `observer`, `untargeted_outputs`, `operator_arming` and
  `internal_driver_station` attributes, as multisets.
  Timers are not endpoints, and the
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
local_address = "10.56.87.5"       # optional companion interface IPv4; unset binds 0.0.0.0
controller_port = 5802             # gateway port on the controller
local_port = 5803                  # bind port on the companion
```


```sh
bazel run //talOS/launcher:launcher -- --config bench/main_processor/configuration/robot.toml
bazel run //talOS/launcher:launcher -- --config bench/main_processor/configuration/robot.toml --remote 10.56.87.2 --local-address 10.56.87.5
```

`--remote` overrides `[robot] controller_address`, `--local-address` overrides
`[robot] local_address`, and `--remote-port` and `--local-port` override their
config counterparts the same way. Every address must be an IPv4 literal without
leading zeros (the bridge's socket resolves no hostname), and a port, in the
config or a flag, is an integer in [1, 65535]; config load or flag validation
refuses anything else before probing or spawning. The launcher resolves the
effective endpoint once per launch, prints it (`[launcher] Controller
<addr>:<port> (local <ip>:<port>)` when `local_address` is set, or `(local port
<port>)` when unset), and always starts `hardware_node` with an explicit
`--remote/--remote-port/--local-port` triple plus `--local-address <ip>` when
`local_address` is non-empty, which its `--describe` probe gets too
(`ControllerArgs`): the bridge refuses `--sim` without the loopback remote, so
a probe left to the config's address would refuse `--sim --remote 127.0.0.1` on
a config naming the RoboRIO.

When `[robot] local_address` (or `--local-address`) is set, `hardware_node`
binds that IPv4 interface address on its first open and on every socket reopen,
preventing a multi-homed companion (such as a Mac mini with Ethernet and Wi-Fi
on the same subnet) from drifting onto another source IP after a cable pull or
route change (`REQ-RT-22`). When unset on a non-`--sim`, non-loopback
(`!= 127.0.0.1`) controller target, phase one (and `--describe-only`) emits a
non-fatal warning (`warning: hardware_node: [robot] local_address is unset;
...`) so field configs catch an unpinned bind before deployment (`REQ-OPS-25`);
loopback (`127.0.0.1`) and `--sim` launches emit no warning.

Running the bridge standalone resolves identically: its own `--remote` and
`--local-address` win, otherwise its `--config` supplies them. Simulation
requires the loopback remote (`127.0.0.1`) and either an unset or `127.0.0.1`
local address (a field config that sets `[robot] local_address` can be run
under `--sim` by also passing `--local-address 127.0.0.1`). On hardware,
commissioning gates commands; see
[hardware configuration](../hardware/README.md#configuration).

## Lightweight simulation gateway

`--sim --start-sim-gateway` starts the binary named by `[robot] sim_gateway`
(`//talOS/simulation:sim_gateway`, [its README](../simulation/README.md)) as
a roster member, under the name `sim_gateway`. It is started with
`--config` and `--session-id` and no `--log`, so it writes no event log, and
`--replay-inputs --only` refuses it. These hold for the member the launcher
adds only, which it does when `--sim`, `--start-sim-gateway` and a
`[robot] sim_gateway` key are all present: a robot's own subsystem named
`sim_gateway` is otherwise an ordinary node, and with that member the launch
is refused naming its `[subsystems.sim_gateway]` key. The launcher passes the gateway the bridge's
ports (`SimulatorArgs`): `--port` is the effective controller port the bridge
sends to (`[robot] controller_port`, or `--remote-port`) and `--remote-port` the
effective local port the bridge listens on (`[robot] local_port`, or
`--local-port`), so the two always agree. Unless `--sim-gateway-arg` already
supplies `--schedule-start-file`, the launcher also passes
`--schedule-start-file <output_dir>/<session_id>/sim_schedule_start` at runtime
and writes the node-clock timestamp there atomically once every non-observer
member's loop has started (its live registry row is active without
`kFlagConstructing` and has reported its realtime outcome or a dispatch; the
Studio bridge, an observer, never delays it, nor does a member that has exited
without a pending non-quarantined restart). Until then `sim_gateway` holds its
schedules' time-zero state, so a slow-starting node never misses its opening
mode or joystick window, and a restarted gateway rejoins the same schedule
partway; passing `--sim-gateway-arg --schedule-start-file` leaves coordination
to the caller. Repeat `--sim-gateway-arg ARG` to
forward further arguments verbatim, in order, after those, to **both** its
`--describe` probe and runtime. The forwarding flag requires
`--start-sim-gateway`, and a `--port` or `--remote-port` among its values is
refused before anything is probed: set the config's ports or the launcher's
flags instead. For example:

```sh
bazel run //talOS/launcher -- \
  --config "$PWD/bench/main_processor/configuration/robot.toml" \
  --sim --start-sim-gateway --sim-gateway-arg --physics \
  --sim-gateway-arg --wheel-radius-error --sim-gateway-arg 0.1
```

A config whose settings are for simulation only (simulated gains, bindings,
wide travel) says so with `[robot] simulation_only = true` (default `false`),
and the launcher then refuses to start it without `--sim`, right after
loading the config, before anything is probed or spawned:

    launcher: /path/robot_sim.toml sets [robot] simulation_only = true; run it with --sim (or --describe-only); nothing was started

`--describe-only` and `--replay-inputs`, which touch no hardware, still run
it, except `--describe-only --describe-as-launch`, which refuses it as the
launch would ([two phases](#two-phases)), saying what that means for an
install:

    launcher: /path/robot_sim.toml sets [robot] simulation_only = true; a launch of it without --sim is refused, so it cannot be installed as the robot's; nothing was started

The key passes through `include` like every `[robot]` key, so a
simulation config that includes `robot.toml` sets it in its own `[robot]`;
config load refuses a file that sets it `false` over an included file that
sets it `true`, naming both files.
`hardware_node` started by hand refuses such a config the same way
([hardware README, "Configuration"](../hardware/README.md#configuration)).

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
load refuses a missing file, phase one (`LintTelemetryConfig`) checks the
layout ([telemetry labels and dashboard layout](#telemetry-labels-and-dashboard-layout)), and
the file is in the config's `CONFIG_FILES` and its bundle. The watched topic
comes from `[studio] topic`,
the same setting the telemetry node reads. `--studio-http-port PORT`
overrides `[studio] http_port` for one run; `--studio-http-port 0` asks the
OS for a free port (`[studio] http_port` in the config still requires
`[1, 65535]`). The launcher captures the bridge's `listening <port>` line
from its output pipe, records the bound port as `http_port` on the
`studio_bridge` entry in `manifest.json`, and prints that port in
`Started studio_bridge ... on http://localhost:<port>` (never port 0). The default webroot exists only
in a checkout that has run `bazel build //studio:web_dist`; elsewhere set
`[studio] webroot` (a robot repository's: [built pages](../../studio/README.md#built-pages)).
The bridge needs the binary (`[studio] bridge`, resolved under `bazel-bin/`
like a node's); without the webroot's pages it still runs, and answers `/`
and `/drive` with how to build them. A bridge whose
`--describe` probe does not answer (not built, absent from a bundle, or
unable to run: a missing library, a crash) is one warning in the graph,
naming the `bazel build` command or the bundle, and the launcher leaves it
out of the session, which runs without Studio, `--allow-graph-errors` or
not. This holds for the
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
like any other node, and the launcher starts it with `--session-id`, the
session's `graph.json` as `--declared` and the session's `manifest.json` as
`--manifest`; its stdout and stderr land in
`<output_dir>/<session_id>/studio_bridge.log` (plaintext, not a `.tlog`) beside
the manifest, like every member's [output](#node-output), but are not echoed to
the launcher's stdout.

The bridge is the one roster member supervised differently (`StudioBridge` in
`studio_bridge.h`); it waits for the telemetry segment itself (see the
[bridge README](../../studio/bridge/README.md#publisher-and-serving-constraints)).
It is an observer: its spawns, exits, liveness kills, restarts and final
give-up never start, stop, restart or delay another member, and never change
the launcher's exit status. Its first run spawns with the nodes, and it claims
a live registry row (observer tier, updating `heartbeat_monotonic_ns` and
`dispatch_count` on every loop pass, at least every 250 ms). The launcher
watches that row with `RunLiveness` (1 s period, `kStudioBridgePeriod`, with
the session's `--heartbeat-periods` and `--heartbeat-startup-grace-ms`): a
bridge whose loop stops making progress is SIGKILLed as `stalled` (or
`heartbeat_missing` if it never registers past the grace and timeout) and
restarted on the bridge's schedule below. Every ending of a run while the
session is not stopping is a failure, exit 0 included (a server that ends
before the stop has failed), and so is a spawn that fails; each is restarted
on a non-critical member's schedule ([restarts](#restarts)): 50 ms, doubling
to 1 s, at most `max_restarts` starts (default 5) in any restart window
(default 60 s), inside the supervision loop, so a restart never delays the
session, another member or the `--duration-s` deadline. Each restart prints
the line a node's does:

    [launcher] studio_bridge exited (signal 11 SIGSEGV); restart 1 in 50 ms

Where a node would be quarantined, the bridge is left for the rest of the
session instead: one line names it and says it will not be restarted again,
its manifest entry is marked `recorder_failed`, and the session runs on
without Studio. The bound counts starts inside the rolling restart window,
not over the session: a bridge that dies or hangs fewer than `max_restarts`
times per window is restarted every time, for the whole session. A bridge that
fails at once on every run therefore costs at
most `max_restarts` + 1 spawns, 6 by default, all within about 1.6 s of the
first (50 + 100 + 200 + 400 + 800 ms of backoff), then none: no busy loop
and no fork storm.
`--max-restarts 0` leaves it at its first ending. A stop cancels a pending
restart, so no bridge is spawned once the stop has begun, and the session
ends without waiting out the backoff. A run counts as up once it has run
for a second (`kStudioBridgeSettleInterval`, printed as `Started
studio_bridge ... on http://localhost:PORT`); a run not yet up and a pending
restart do not hold the session open, and a run not yet up when the session
ends is SIGKILLed once the other members have ended. A bridge none of whose
runs came up is marked failed when the session ends on its deadline or its
roster's completion. Its restarts are in its manifest entry
(`restart_count`, which counts every restart attempt, one whose spawn
failed included, as its restart lines number them;
`retry_at_monotonic_ns` while one is pending; and each
run's `runs[]` entry with the reason it ended, `stalled` and
`heartbeat_missing` included), and so in
[`session_report`](#session-measurements)'s node and run tables and Studio's
session panel; how it ended (a kill, a crash, its exit code) is never in
the launcher's exit status: an operator's stop of a session whose bridge
died still exits 0.

A restart binds the same HTTP port: the configured port, or the port learned
from the first run's `listening <port>` line when started with
`--studio-http-port 0`. The bridge holds its port
exclusively (`SO_REUSEADDR`, never `SO_REUSEPORT`;
[studio/bridge](../../studio/bridge/README.md)), and the launcher kills a dead
or hung run's process group before it schedules the restart, so the port is
free again for the restart. A port another listener holds, such as another
launcher's bridge on the same port (a developer's `./sim.sh` on the robot
computer), is refused: every run prints `cannot listen on port <port>` and
exits, which the bound above ends, with the reason in `studio_bridge.log`.

### Refusal visibility (`--serve-refusal`)

On any phase-1 refusal after the launcher acquires the namespace `SessionLock` (a bad
config, a graph or naming error, `simulation_only` without `--sim`, a full node
registry, or a failed probe; never on `--describe-only`, `--replay-inputs`, a
held session lock, or a pre-lock CLI/bundle argument refusal so another live
session's log root and `/drive` port are never disturbed), the launcher
atomically writes `<output_dir>/last_refusal.json` (`{"refused": true,
"reason": "...", "config_path": "...", "wall_ns": ...}`) in the log root (when
writable or creatable).

Without `--serve-refusal`, the launcher still exits 1 immediately. With both
`--serve-refusal` and `--start-studio-bridge` (`tools/deploy.sh enable-boot`
passes `--serve-refusal` to the boot service's `run.sh`, which adds
`--start-studio-bridge` for bundles carrying Studio), the launcher starts the
Studio bridge in refusal mode (`--refusal-file <output_dir>/last_refusal.json`,
using `--studio-http-port`, `[studio] http_port`, or default `5800` and the
bundle's or configured webroot so even a TOML parse error still serves
`/drive`), notifies systemd (`READY=1` and `WATCHDOG=1`) once the bridge
announces its listening port, and waits until a stop signal, `--duration-s` (if
> 0), or the bridge's own exit (e.g. `EADDRINUSE` when another process outside
the lock already holds port 5800), then stops the bridge cleanly and still
exits 1. While it runs, `/drive` renders **LAUNCH REFUSED** with the exact
refusal reason instead of **BRIDGE LINK**
([PRE_MATCH.md](../../docs/PRE_MATCH.md#driver-dashboard-top-to-bottom)).

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
stdout does. Each launcher line goes into that memory in one piece
(`ConsoleLine`, `console_line.h`), so a member's echoed output never lands
inside it. The echo of a member's output goes in whole lines too: a line
that reaches its pipe in more than one write (stdio to a pipe is fully
buffered) waits for its newline, the pipe's end (once every process holding
it has closed it, or at the session's end), or more than 64 KiB held
(`kChildOutputReadBytes`), past which a line with no newline is shown split.
A partial line shown at the pipe's end or at that bound is ended with a
newline on the console, so it is a line of its own; a member's last partial
line can therefore show after the launcher's "exited" line for that member.
The member's `<name>.log` takes every byte as it is read, and no added
newline. While a writer is behind, up to 2 MiB per member, 2 MiB for
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
ignored SIGHUP, `&`'s ignored SIGINT and SIGQUIT, its own ignored SIGPIPE),
with one exception: every run of a member whose `--describe` answer says
`holds_stop` (each binary built on `talos::Run` or `talos.run`;
[describe](../introspection/README.md#describe)) starts with SIGINT, SIGTERM
and SIGHUP blocked. They stay blocked through every exec of its start-up (a
bundle's `/bin/sh` script execing the interpreter, rules_python's first
interpreter execing the second) until the node framework installs its stop
handlers, which unblock them: a stop sent earlier is held, not met by the
runtime's own handling (CPython would raise `KeyboardInterrupt` and exit 1),
and the node takes it as a stop during construction and exits 0, provided
it reaches its stop handlers within the stop timeout (`kStopTimeout`, 2 s)
of the launcher's stop. One still starting then is SIGKILLed and recorded
`stop_timeout`, as a stuck constructor is, and the session ends nonzero.
On shutdown, the launcher stops `hardware_node` first (`kStopSignal`, up to
`kBridgeStopTimeout`, 500 ms, before SIGKILLing a bridge that has neither
exited nor quiesced) and signals the remaining members as soon as
`hardware_node` has either exited or marked its registry row `kFlagQuiesced`
(after sending its shutdown neutral, closing its waits and UDP socket, and
publishing introspection, before flushing its `.tlog`). A quiesced bridge still
flushing its event log shares `kStopTimeout` (2 s) with the remaining members
before SIGKILL; an un-quiesced bridge that hits `kBridgeStopTimeout` is
SIGKILLed and reaped before other members are signaled.
Probes, group guardians, `--replay-inputs` runs, members that do not say so
(`hardware_node`, `sim_gateway`, the Studio bridge, foreign binaries) and
members whose `--describe` probe did not answer (a non-critical member the
launch went ahead without) start with none blocked.

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
The [Studio bridge](#studio-bridge) restarts on the non-critical schedule
below after any ending, exit 0 included, and is left for the rest of the
session where a non-critical node would be quarantined.
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
scheduled inside the supervision loop, so a restarting node never delays
another node or the deadline. Each restart prints
one line when it is scheduled and one, with the new run's pid, when it
starts (a first run prints `Started` in the same form):

    [launcher] alpha exited (signal 11 SIGSEGV); restart 1 in 0 ms
    [launcher] Restarted alpha (pid 4243, target //example/alpha:node, restart 1)

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
shows it in the exit column and marks the node *killed by signal*, except
a last run with `stopped_by_session` true, which it shows as stopped (for a
manifest without that field, written by an older launcher: a run with reason
`stop` and SIGINT).

When the session ends, the launcher exits with 0, or with the last nonzero
`exit_code` among the members (-1, so 255, for one a signal ended), or 1 for
refused [live endpoints](#live-endpoints). The [Studio bridge](#studio-bridge)
never counts, however it ended, nor does a member a stop left unstarted
([two phases](#two-phases)). Only a member's last run counts: a critical
member that crashed, was restarted, and whose last run then ended at the
session's stop (exiting 0, or by the stop's own signal) leaves the status
0, although the session lost it mid-run; the crash is in its `runs`
(reason `crash`). A member whose last run ended by
the stop's own signal does not count: one that was not yet handling a stop
(between its exec and its stop handlers, which a member started with the stop
held never is: [node output](#node-output)) when the stop's SIGINT
reached it, so an operator's Ctrl-C during startup ends the session with 0.
Its run still records the signal: `exit_code` -1, `exit_signal` 2, `reason`
`stop`, `stopped_by_session` true, the field the exit status reads. Only
a SIGINT the launcher sent that run counts so, which it knows
from its own record of the stop it sent before it reaped the run, never from
the stop flag. The launcher sends it only after the spawn has returned, which
is after the member's exec. A terminal's Ctrl-C reaches only the launcher's
process group, never a member's (each has its own). Still counted: a
signal before the stop, another signal (a SIGTERM that systemd's default
`KillMode=control-group` sends every process of the unit along with the
launcher; the unit `deploy.sh` writes sets `KillMode=mixed`, which signals
the launcher alone), and the stop timeout's SIGKILL.
A SIGINT from elsewhere that ends the run after the launcher's last look at
it and before its own SIGINT is indistinguishable from it and counts as the
stop's.

A run whose event log stops early says so in its registry row while it runs
(see [event log state](../introspection/README.md#event-log-state)). A node
creates its log on the writer thread once its loop starts, never on the loop
thread, so one whose log cannot be created (a missing or unwritable session
directory) keeps running with its row flagged, and exits 1 when stopped; the
hardware bridge also prints the reason once. A node or the bridge whose log
has a gap (records dropped and marked while capture went on) also exits 1
when stopped, and prints `recording_gaps=` and `records_lost=` on its exit
line. The launcher prints one line per member and kind (for a capped log,
whether it records on, deleting its oldest records, or stopped, which it
tells by the segments beside the log), and one per run whose log has a gap
(`[launcher] <member> event log <log> lost records (a gap)`); it reads the
row in each supervision pass and once more when it reaps the run, from the
row the run left, so a run killed during a gap is marked too, once a
sample of its row (every 250 ms, from a node and from the bridge) has shown
the gap, unless a restart storm reused that row first
([session manifest](#session-manifest)). It marks the
member in the live manifest: `recorder_failed` for a failed
log, `log_gap` and `recorder_failed` for one with a gap (a gapped log is
incomplete evidence), `log_capped` for one whose byte share cost it records
(its oldest segment deleted, or, with a share too small for two segments,
capture stopped at the share). At normal shutdown it also marks a truncated,
damaged or unreadable log `recorder_failed`, one holding a GAP record
`log_gap` and `recorder_failed` (with the gap line, if the row had not
shown it), and
a capped one (`<log>.capped`) `log_capped`.

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
processes outside the session are not, so a launch whose roster (every member,
including `sim_gateway` and `studio_bridge`) does not fit beside them in the 32
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
A hung loop therefore stops feeding the service watchdog. That watchdog is
the only bound on a deployed session whose launcher hangs: nothing but the
launcher's stop ends a member that runs well, and a member's guardian acts
only when the launcher's process ends, so systemd's `WatchdogSec=` (set by
the unit `deploy.sh` writes) kills the hung launcher and with it, through
the guardians, every member. A session run by hand, or under launchd, which
has no watchdog, has no such bound: its members run until the launcher is
killed. The loop waits on no
lock a node can hold: its registry attach is read-only, which
[takes no lock](../introspection/README.md#the-segment).
See [deployment](../../docs/DEPLOYMENT.md) for the generated systemd watchdog
policy and macOS's launchd limitations.

### Sleep prevention

While a session runs (phase two, from starting the roster until the session
ends; not `--describe-only`, `--replay-inputs`, or `--check-session-lock`), the
launcher holds a macOS IOKit `kIOPMAssertionTypePreventSystemSleep` assertion
(`SleepGuard` in `sleep_guard.h`, equivalent to `caffeinate -s`) and releases it
cleanly via RAII when the session exits. This is defense-in-depth independent of
`pmset sleep 0` ([deployment](../../docs/DEPLOYMENT.md#on-the-main-processor))
against idle or DarkWake maintenance sleep on a Mac mini. If IOKit refuses the
assertion, the launcher logs a non-fatal warning to stderr (and `talos.log`) and
continues the session. On Linux, `SleepGuard` is a silent no-op.

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
| each running member's next look at its registry row (`RunLiveness::NextLook`, the Studio bridge's included): one look interval (its heartbeat timeout / 4) after the last, its stall deadline (the startup grace's end or one heartbeat timeout after the progress last seen, whichever is later), and its construction deadline until a look sees it constructed | `stalled`, `heartbeat_missing`, `construction_deadline`; the row's endpoints, recording and realtime flags and delivery counts, read at each look |
| each pending restart's start, the Studio bridge's included | the restart backoff, a quarantined member's retry |
| one settle interval (1 s) after the Studio bridge's last spawn, until that run is up | whether it holds the session open ([Studio bridge](#studio-bridge)) |
| 20 ms startup polls while waiting to write `sim_schedule_start` (`kSimScheduleStartPoll`, until written or `sim_gateway` exits with no pending restart) or for the bridge run's first `/hw/status` (`kBridgeProtocolPoll`, until its protocol version is checked) | starting `sim_gateway`'s schedule; bridge protocol check |
| `--duration-s`'s end, until the stop is sent | the stop |
| two-phase stop timeouts: first `hardware_node` quiescence (5 ms `kBridgeQuiescePoll` on its registry row for `kFlagQuiesced`, bounded by 500 ms `kBridgeStopTimeout`), then the remaining members (and any quiesced bridge still flushing its log; 2 s, `kStopTimeout`), until their SIGKILLs are sent | `stop_timeout` |
| half of `WATCHDOG_USEC`, when systemd sets it | `WATCHDOG=1` |
| the hourly refresh of the session lock and of the session directory's `session.lock` and quiet files | [one session per namespace](#one-session-per-namespace), [retention](#session-segments-and-logs) |

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

No dated measurement of this path is recorded
([BENCH_RESULTS.md](../../docs/BENCH_RESULTS.md#date-not-recorded) has only
its macOS stalls under disk load, INC-3). What the code fixes is the
backoff before the `fork` ([restarts](#restarts): none for a critical member's
first crash in the window, else 50 ms doubling to 1 s) and one `period_us`
from the restarted run's `run()` to its first tick, whose flush sends the
first `/hw/request/<node>` once a `/hw/state` has arrived since the node
attached. The rest (the SIGCHLD wakeup and reap, restart bookkeeping, `fork`,
exec and dynamic loading, config parse and topic attach) depends on the host.

`restart_latency` measures it from the restarted run's `.tlog` on any host:
SIGKILL → event-loop origin, origin → `run()`, `run()` → first tick, first
tick → first request and the total, as median (range). The
config's nodes come from `bazel-bin`, so build them first:

```sh
bazel run //talOS/launcher:restart_latency -- \
  --config bench/main_processor/configuration/robot_physics_sim.toml \
  --node swerve_module --start-sim-gateway --sim-gateway-arg=--physics
```

It runs the session on loopback ports it picks (the launcher passes them to
the simulator it starts) under its own
`TALOS_SHM_NAMESPACE`,
kills `--node` (required: a critical node that sends its
`/hw/request/<node>`) `--kills` times (default 10), spaced
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
  --config "$PWD/bench/main_processor/configuration/robot_physics_sim.toml" \
  --replay-inputs /tmp/talos_logs/<id> --only swerve_module \
  --output-dir /tmp/swerve_module_replay
bazel run //talOS/launcher:log_dump -- /tmp/swerve_module_replay/<new-session-id>
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
sequence and publication time live readers saw. A node the session ran replays on its own recorded timeline: its
timers fire at the deadlines they fired at live (a node that fired none
starts its run at its first dispatch and fires none up to that run's last
record; for a single-run log with no own-log gap, damaged stretch or broken
segment carry, `loop_end()` is clamped before the earliest armed timer deadline
strictly after the last recorded `TIMER` or `MESSAGE` dispatch so a late `EXIT`
or an offset-0 exit-only run does not fabricate a firing, except for the
`ARM_TIMER` live-latency and backlog-drain-exit cases in
[input replay](../events/README.md#replaying-another-sessions-inputs);
a restarted node replays as one run), and it is handed what it
received live, at the ticks it received it, so an unchanged node sends what it
sent, except for what its loop's wake-up latency put into it (`now()`, and
ages measured from it). The node is never fed across a log gap: the run ends
just before the earliest gap (GAP record) or damaged stretch in a log it is
fed a watched topic from (its own logs for a topic it watched live, otherwise
every log that recorded the topic), the node prints the log, the topic and
the time, and the launcher exits 1 with it. A log it would be fed from that
may have lost records no GAP record marks is refused before the node runs,
naming the log and the sign, and the launcher exits 1: one with a
`<log>.capped` marker or of a member the session manifest marks
`log_capped`, a run's log without its EXIT record named in the manifest's
`gap_logs` or `recorder_failed_logs` (or, for legacy manifests without those
arrays, any run of a member marked `log_gap` or `recorder_failed`;
[session manifest](#session-manifest)), one
with a missing or incomplete-header segment (including a missing first segment
without a marker when a later numbered segment remains or the unnumbered active
file's first intact record has `dispatch_index > 1`, or a segment missing
before the active file detected by a `dispatch_index > prev + 1` or
repeated-sender accepted-`SEND` sequence `> prev + 1` jump; see
[input replay](../events/README.md#replaying-another-sessions-inputs) for the
unnumbered-file limits), and one whose damaged
records nothing bounds the time of. Passing `--allow-capped-replay` (accepted
only with `--replay-inputs`; forwarded to the replayed node) overrides the
`<log>.capped` / `log_capped` refusal and replays from the start of the kept
window, printing a warning to stderr naming the capped log(s) and noting that
pre-window history was lost; uncapped `log_gap`, `recorder_failed`, incomplete
headers, missing non-head segments, and unbounded damage are still refused. The
node's own first run's log is
refused the same way, whatever it feeds, when the node had a timer and
that log's first segment, or a segment before its first firing read, is
gone or unreadable (the byte cap deleted it, or its header is incomplete):
it may have held the firing its recorded timeline starts from (with
`--allow-capped-replay`, a byte-capped log whose first segment was rotated out
starts the timed node at the kept window's earliest recorded firing or
dispatch). So is a
timed node's log with no such firing that holds no record of its run, or
a gap or damaged stretch that may hold its firings. A writer's or another
reader's damaged stretch is no loss when an intact accepted send or receipt
follows it in the same file or a contiguous later segment and the session's
other logs hold every message in that sequence range, and reader-only caps,
marks, gaps or damage on a topic the replayed node did not watch live are
ignored when every writer log of that topic is intact and exited and the
session holds every sequence from `0` through the maximum recorded sequence
([the rule](../events/README.md#replaying-another-sessions-inputs)). See
[input replay](../events/README.md#replaying-another-sessions-inputs) for
timing, duplicate-sequence and schema checks.

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
`manifest.json`: the session id, the config it came from, the configuration's
CAN bus slot names (`can_buses`, `hardware::CanBuses`, slot 0 `"rio"` then each
extra bus in byte order), every node with its
name, target, whether it answered, and each source's kind, name, message size
and endpoint attributes, followed by every diagnostic. Studio's bridge serves it
so a viewer can show the declared graph next to the live one, mark the
difference, and label `/hw/status` CAN bus slots by bus name.
It is written to a temporary file and renamed into place, as is the
`declared_graph.json` beside it that graph-reading nodes are described and
started with as `--graph`. A launch that cannot write either exits 1 before
anything starts.


## Session manifest

`<output_dir>/<session_id>/manifest.json` says what ran and how each run
ended (when it is written: [session segments and logs](#session-segments-and-logs)).

- `config_path`: the config, made absolute against the launcher's working
  directory (also in `graph.json`, and passed to every member as `--config`).
- `hardware_protocol_version`: the hardware protocol of the launcher's build
  (`protocol::kProtocolVersion`); absent in a manifest written before the
  field. It is the launcher's constant: the bridge and node binaries it ran
  are of another build when they were not rebuilt with it (`bazel run
  //talOS/launcher` rebuilds the launcher only; a bundle builds them all
  from one checkout), so the log tools read a session under the version its
  bridge recorded on `/hw/status`, and under this one only when no bridge
  recorded one
  ([which logs a build can read](#which-logs-a-build-can-read)).
- `period_us`, `command_timeout_us`: the config's `[robot] period_us` and
  `command_timeout_us`, the bounds `session_report` judges the `/hw/state`
  cadence against ([session measurements](#session-measurements)); absent
  in a manifest written before the fields.
- `config_sha256`, `git_revision`, `git_dirty` (and, when a separate robot
  repository was stamped against a talOS git checkout, `talos_git_revision`,
  `talos_git_dirty`, omitted when empty): the bundle's config identity
  and the source revision(s) of a `--config=release` build
  ([deployment](../../docs/DEPLOYMENT.md#build-a-bundle)); empty (and
  `false`) outside a bundle or for an unstamped one. `session_report` prints
  `talOS revision: <rev>[ (dirty)]` after `Source revision:` when present. The
  RoboRIO program's build is not here: every `/hw/state` record of the bridge's
  log carries it (`log_dump`'s `hardware` field,
  [filtering and exporting logs](#filtering-and-exporting-logs)).
- per node, `critical` and `studio_bridge`: `critical` is `true` for the
  hardware bridge, the simulation gateway and critical subsystems, and `false`
  for non-critical subsystems (`[subsystems.<name>] critical = false`) and the
  launcher-added Studio bridge; `studio_bridge` is `true` only for the
  launcher-added Studio bridge (`--start-studio-bridge`), never for a robot
  subsystem named `studio_bridge`.
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
  `exit_code`, `exit_signal` (as for the node's last run, above), `reason`
  and `stopped_by_session`: true only for a run the stop signal the
  launcher sent ended (reason `stop`, SIGINT), which leaves the exit status
  0 ([restarts](#restarts)); false for every other run, one that handled
  the stop and exited 0 or that a signal from elsewhere ended during the
  stop (also reason `stop`) included. At most 32 are kept, the first run
  and the newest; `runs_dropped` counts those dropped between them.
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
  increase in its final sample is lost (its event log flags too, so a run
  killed during a log gap is then not marked `log_gap`). A claim takes a
  row released in the last five seconds only when every other of the
  registry's 32 rows is held or was released after it
  ([claiming a slot](../introspection/README.md#claiming-a-slot-and-liveness)):
  in a restart storm, where held rows plus rows released in the last five
  seconds reach 32 before the launcher reaps a run, that run's row can be
  reused first. The node's fields sum every run, dropped ones included, and
  `requests_skipped_by_subsystem` splits the bridge's `requests_skipped` by
  the subsystem whose `/hw/request/<subsystem>` it passed over (only
  nonzero ones). A run, or a node, whose row was never seen (a spawn that
  failed) or that reports no delivery-loss counts (Studio's bridge) has none of
  these fields. A count leaving 0
  posts a manifest version of its own; a later increase waits for the next
  version the launcher posts anyway, at the latest the one the run's end
  posts, so a healthy session adds no version, and a killed launcher loses
  only the increases after a count's first.
- per node, `recorder_failed`, `recorder_failed_logs`, `log_gap`, `gap_logs`
  and `log_capped`: what its runs' event logs lost ([restarts](#restarts)).
  `gap_logs` and `recorder_failed_logs` list the per-run event log paths that
  suffered a gap or a recording failure (live `kFlagRecorderFailed`, or
  damaged/unreadable at shutdown; a failed spawn sets `recorder_failed` without
  adding a log path), so input replay scopes missing-`EXIT` refusals to the
  affected run(s) while falling back to member-wide marks on legacy manifests
  without those arrays.
- on `studio_bridge`, `http_port`: the HTTP port the bridge bound (captured
  from its `listening <port>` line); absent for other members and until the
  bridge reports it.
- `stopped_before_start`: a stop request arrived before every member had
  started ([two phases](#two-phases)); the members without `runs` were
  never started.
- `session_lock_lost`, `directory_lock_lost`, `directory_lock_retaken`:
  whether the hourly lock refresh found the namespace session lock file removed
  (`session_lock_lost`), found `session.lock` removed and could not lock it
  again (`directory_lock_lost`), or found `session.lock` removed and locked it
  again on a new file (`directory_lock_retaken`); each defaults to `false` and
  persists into the live manifest as soon as observed
  ([session segments and logs](#session-segments-and-logs)).
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
| `stop_timeout` | outlived the stop timeout after its SIGINT (500 ms `kBridgeStopTimeout` for an un-quiesced `hardware_node`, 2 s `kStopTimeout` for a quiesced `hardware_node` still flushing its log and for other members) and was killed (SIGKILL) |
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
| `:session` | `session.h`, `session_lock.h`, `file_digest.h` | manifest.json and run history, atomic writes, the session lock and session directory lock |
| `:session_retention` | `session_retention.h` | session classification and what retention reads of a recorded session |
| `:log_budget` | `log_budget.h` | each member's share of the session's event-log budget |
| `:spawn` | `spawn.h` | `SpawnProcess` (process group, guardian, child reports), `SupervisionWakeup` |
| `:child_output` | `child_output.h` | `ChildOutput`, the drain behind [node output](#node-output), and `LauncherOutput`, the launcher's own stdout, stderr and `talos.log` |
| `:roster` | `roster.h` | `LauncherOptions`, the bundle manifest, `DiscoverNodes` and binary resolution |
| `:describe_graph` | `describe_graph.h` | phase one: `ProbeDescribe`, `DescribeGraph`, the parameter, Python-period and telemetry/dashboard lints, `EndpointMismatch`, session segments, `graph.json` |
| `:log_inputs` | `log_inputs.h` | `LogInputFiles`, the log files `log_dump`'s and Studio's review's inputs name, each once |
| `:hardware_record` | `hardware_record.h` | the hardware packets decoded with the wire codecs (`HardwareRecordOf`, `HardwareRecordJson`), the check that a session's packets are of this build's protocol (`RecordedProtocolVersion`, `HardwareProtocolCheck`) and the run config's device names (`SessionConfigPath`, `LoadDeviceNames`), for `log_dump` and Studio's review |
| `:log_merge` | `log_merge.h` | `MergeLogs`, a session's records streamed to a visitor in time order; `log_dump` and Studio's review link it, not the supervisor |
| `:supervision_policy` | `supervision_policy.h` | the [restart](#restarts) and kill decisions as functions of the supervision clock: `RestartSchedule` (backoff, rolling budget, quarantine) at the time a pass sampled, `RunLiveness` (stall, missing row, construction deadline) at the times read around each row read, and when to look at the row next ([supervision passes](#supervision-passes)) |
| `:sleep_guard` | `sleep_guard.h` | `SleepGuard`: macOS IOKit `PreventSystemSleep` RAII assertion held for a running session (silent no-op on Linux; [sleep prevention](#sleep-prevention)) |
| `:launcher_lib` | `launcher.h`, `studio_bridge.h`, `service_notifier.h` | the supervisor (`Launcher`; retention in `launcher_retention.cc`), which samples the clock, reads the registry, spawns, reaps and kills, and asks `:supervision_policy` what to do; and the Studio bridge |


## Session segments and logs

`--output-dir DIR` always names a root: each launch writes to
`DIR/<session_id>/`, including deployed `TALOS_LOG_DIR` roots. Without the flag,
the root is `/tmp/talos_logs`. Use persistent storage on a deployed robot.
`manifest.json` is written before nodes start, then replaced atomically
(written beside it, then renamed over it) at each process lifecycle change,
when `studio_bridge` reports its bound `http_port`, when a node reports a
stopped event log, when a run's endpoints differ from its probe, when a run's
delivery-loss count first leaves 0, and when an hourly lock refresh observes a
lost or retaken session or directory lock
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
rate). The hardware bridge logs only the State it publishes, the commands
it sends and its `/hw/status`, so only those count for it, the commands at
one per actuator owner's request: the rate while owners change their
demands. Byte topics are never logged. A member that did not answer `--describe`
weighs as the mean of the others, and every member gets at least the
smallest share at which its log rotates (two segments of its prefix and
largest record, at least a 128 B GAP record) while those fit in the budget.

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
next launch's retention; if a concurrent sweep removes the tree first
(`ENOENT`), both `remove_idle` and the leftover sweep treat it cleanly as
removed without reporting an error (`launcher_retention_test.cc`). The supervision loop touches its `session.lock`
hourly, with the session lock (one `futimens` on the held descriptor, which
writes and syncs nothing itself), and with it the session's files a launch
writes once and may leave for days (`manifest.json` in a quiet session,
`graph.json`, `declared_graph.json`, `retention.txt`, named by
`kSessionQuietFiles`; one `utimensat` each, never creating an absent one),
so a cleaner of `/tmp` files unused for
days (macOS's after 3, systemd-tmpfiles' after 10) never deletes them under a
long session in the default `/tmp/talos_logs`. A member's `<name>.log`, a
finished run's event log and `talos.log` are not touched: a cleaner can still
delete one that has been quiet that long. The touches, and the `fstat` and
`lstat` that check the lock file, run one after another on the supervision
thread; on a network log root each is a call to the server, which a dead
server blocks for as long as the mount retries it (without end on a hard
mount), and with it supervision and the watchdog's ping. If the path stops naming the
locked file (removed by hand, or by a cleaner before a refresh), retention
could take the directory from then on; the next refresh locks the file the
path names (creating it, never the directory), says so once on stderr, and
records `directory_lock_retaken: true` in `manifest.json`. That adds
no window: it and a retention lock the same file, so either the launch
holds it first and every later retention passes the session over, or a
retention holds it and is removing the directory, which is reported on stderr
and recorded as `directory_lock_lost: true` in `manifest.json` (similarly, a
removed namespace session lock is reported on stderr and recorded as
`session_lock_lost: true`). A
running launcher that holds no `session.lock` (one built before it existed)
is protected only from launches in its own namespace. So is one whose log
root's filesystem takes no advisory locks (`EOPNOTSUPP`, `ENOTSUP`, `EINVAL`,
`ENOSYS`, such as macOS's msdos and exFAT volumes): its launch warns once and
runs without the lock rather than being refused, and retention there removes
sessions as it did before launches locked them. `ENOLCK` is separated from
`FlockUnsupported`: when probing `.talos-lock-probe` on the log root returns
`ENOLCK` (such as a network lock daemon being down), retention prints a warning
naming `ENOLCK` and refuses/skips all deletions, leaving every existing session
untouched so a running session in another namespace can never be deleted
(`REQ-OPS-11`, `launcher_retention_test.cc`). Such a launch (started unlocked
while the lock daemon was down) tries the lock again at each hourly refresh,
and once it takes it says so once on stderr and is passed over by retention
from then on. Until that refresh it is not: once the daemon answers again, a
retention from another namespace whose probe now locks can take the unheld lock
and remove the running session, a window of up to one refresh period (an hour).
The retry is a `flock` on the
supervision thread, against a lock service that has just failed: on a
network filesystem (a hard-mounted NFS root) that call can block through its
RPC retries, and with it supervision and the service watchdog's ping, so the
log root should be on a local disk. Whether a log root supports advisory locks,
takes none, or has an unavailable lock service (`ENOLCK`) is decided once by
retention, and once by the session directory's creation, by a shared lock of
the root's `.talos-lock-probe` file (`ProbeLogRootLocks`), never by one failing
call. On a root whose probe locked, a session whose `session.lock`
cannot be locked for such a reason (an NFS lock daemon that stops
answering) is passed over, with a warning, and a launch whose own lock
fails so tries again rather than running without it: up to 100 attempts
1 ms apart (about 0.1 s), after which the launch is refused (`cannot create
the session directory ...; nothing was started`, exit 1). NFSv4 is reported
to refuse an exclusive `flock` on a descriptor opened read-only, so every
exclusive lock of a `session.lock` is taken on a read-write one (the shared
locks, `Held` and the root's probe, stay read-only); whether NFSv4 does so
is unverified, since no NFS root has been tested.

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
`session_class` file and read before any log is opened: `match` for good
(dropping one could only let a match go first), `other` and `unknown` until
the session's event log bytes change or this build's log format, `Packet`
size, Driver Station codec sizes or hardware protocol differs. A session
recorded under a hardware protocol other than this build's (its bridge's
version, else its manifest's) is `unknown` without its Driver Station
packets being read
([which logs a build can read](#which-logs-a-build-can-read)). A session's
logs are therefore read once while that file can be written.

A directory named
by a session id that holds nothing, or only what a launch writes before it
starts its nodes (`graph.json`, `declared_graph.json`, `talos.log`,
`session.lock`, the
`.tmp` of an interrupted write, and the empty `.tlog` a node leaves when it dies between
its log writer creating the file and writing the header), is a launch that
recorded nothing:
`--describe-only`, a refused graph, a launch stopped while it described its
roster, or a launch that failed or died early. It
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
  `summary` prints one block per topic, and one per process with a log gap,
  instead of the records (below).
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

A log gap, the records an event-log writer dropped and capture resumed after
(a GAP record; [log format](../events/README.md#log-format)), is a record of
its process with no topic (`source_id` 65535 and topic `?` in the exports,
`-` in the text dump; its `publish_ms` means nothing), merged at its start (`event_ms`, the earliest time a dropped
record names) and ending at `now_ms`, when capture resumed (for a gap the
writer's stop closed, the stop). It concerns every topic of its log, so no
`--topic` pattern hides it; `--node` selects it by its process, and the time
window by its whole span: a gap overlapping the window is shown. The text
dump prints `      gap: <sentence>` under its record line, CSV puts the
same sentence in `decoded`, and JSONL puts it in `decoded` and adds a `gap`
object (below). Once a log with gaps ends, `log_dump` prints one line for
it on stderr, however many it holds: `warning: log gap in <path>: <the first
gap's sentence>[ (and <n> more)]`. The sentence is `DescribeGap`'s
(`log_reader.h`): what was lost, over how long, which dispatches, the cause,
and whether the first dispatch was partly recorded or the stop closed it.

Records are printed in merge order: by event time, then the record's `now`,
then its dispatch index, and on a full tie in the order the files are listed
(sorted by path within a directory, so a rotated log's older segments come
first). Every log is read through a fixed window (`WindowedLogReader`), never
whole, so memory grows with the number of logs, not with their length. Each
mode reads the session twice: once for the earliest event time every printed
time is relative to and for the record count, printing the gap and damage
warnings then, before any output; once to print. That baseline is the
earliest event time over every record of the logs named, before any
`--node`, `--topic`, `--since-ms` or `--until-ms` filter, so a filter never
moves it: a log file named alone is timed from its own earliest record, so
its times are not session-relative; name the session directory (with
`--node` to keep the one process) for times from the session's earliest
record. The text dump names the baseline under its file list
(`times: ms after event time <ns> ns, the earliest record of these logs`),
so two dumps' baselines can be compared. On a generated 210 MiB session (8
logs, 1.72 million 128-byte records; `bazel run -c opt
//talOS/launcher:log_memory_bench`, Linux x86-64) `log_dump`'s peak resident
memory is 5.6 MiB, text or JSONL, where reading every log whole and holding
each record took 632 MiB. Each open log costs about 128 KiB (its window and
header buffer) plus its manifest and schemas. A session may hold more logs
than `RLIMIT_NOFILE` allows open files: readers keep at most 64 descriptors
open between them ([windowed reader](../events/README.md)).

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
timer's event time). `latest` is the last merged record's decoded value (a
record its schema cannot decode gives why, `<...>`, and no field value); a
hardware packet `log_dump` decodes has a `hardware` line in its place. Each
`range` line gives a numeric or bool field's smallest and largest value
over the matching records, as decoded, in field order (`message.<field>`
inside a target envelope, `<field>[i]` for an array element), and each
`values` line an enum field's names, each once in the order first seen
(`values phase IDLE HOMED FAILED`), so a state change shows though
`latest` reads only the last. The summary is folded in
during the streaming pass, so its memory grows with the topics and their
fields, not the records.

After the topics, each process with a matching log gap gets one block,
sorted by process: how many gaps, the records, dispatches and sends they
lost, the earliest start and the latest end on the same baseline. GAP
records are not a topic. The block holds totals, not one line per gap, so a
session with any number of gaps costs a fixed size per process; the text
dump and JSONL list each gap.

```text
gaps gamma
  count 2
  records 4
  dispatches 4
  sends 0
  first_ms 15.000
  last_ms 55.000
```

Both export formats contain `process`, `dispatch_index`, `kind`, `source_id`,
`topic`, `event_ms`, `now_ms`, `publish_ms`, `sequence`, `aux`,
`payload_bytes`, `schema`, `decoded` and `hardware`; JSONL also carries
`payload_hex`, the complete payload. Times keep the same session baseline
regardless of filters. `schema` is empty when the source has no schema;
`decoded` is empty without a schema or payload, otherwise it contains the
descriptor-decoded payload text (for a schema-less hardware Packet, a
truncated hex preview; for a GAP record, the gap's sentence). JSON strings
are escaped; CSV fields containing commas, quotes, or newlines are quoted
and embedded quotes are doubled.

A JSONL GAP record (`"kind":"GAP"`) also carries `gap`, which
`session_report` reads:

```json
"gap":{"start_ms":15,"end_ms":35,"first_lost_ms":15,"last_lost_ms":25,
       "first_dispatch_index":2,"last_dispatch_index":3,"records":2,
       "bytes":144,"dispatches":2,"sends":0,"cause":"pool_full",
       "splits_dispatch":false,"closed_at_flush":false}
```

`start_ms` and `end_ms` are the record's `event_ms` and `now_ms` (the
dropped records lie in [`start_ms`, `last_lost_ms`]; capture stopped at
`first_lost_ms`, the first dropped record's `now`, and resumed at
`end_ms`); `first_dispatch_index` to `last_dispatch_index` are the dispatches
it touched; `records`, `bytes`, `dispatches` (TIMER and MESSAGE records)
and `sends` count what it dropped; `cause` is `pool_full` (the writer's
buffers were all handed over), `record_too_large` or `unknown`;
`splits_dispatch` says the first dispatch had records before the gap;
`closed_at_flush` that capture never resumed and the writer's stop closed
it. A GAP record without its detail payload, which this build never writes,
has null `first_lost_ms` and `last_lost_ms`, zero counts and cause
`unknown`.

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
  `motion_magic`, `torque_current`), `slot`, `demand`, `feedforward_v`;
  digital outputs; per PWM output `mode` (`off`, no pulse, or `pulse`) and
  `output`.
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
runs it), so a config edited after the match never mislabels a device.
Driver Station strings are cut at a fixed length, so a byte that does not
begin a complete UTF-8 character is written as `\u00XX`.

### Which logs a build can read

A build reads every topic with a schema in any log of a log format version
it reads ([events](../events/README.md#log-format)), versions 7 and 8,
whatever build recorded it: the schema travels in the log. A build from
before version 8 refuses a version-8 log, naming both versions, so a
rolled-back bundle (`tools/deploy.sh rollback`) cannot dump or replay the
sessions a newer build recorded, and its retention classifies them
`unknown` ([session segments and logs](#session-segments-and-logs)). The
hardware packets carry no schema, and a build decodes them with its own
wire codecs, so it decodes only the hardware packets of a session recorded
under its own hardware protocol (`protocol::kProtocolVersion`;
[DATA_FLOW.md](../../docs/DATA_FLOW.md)), a version of its own: a log format
change leaves it as it is.

<a id="replay-after-a-schema-change"></a>Reading is not replaying:
`--replay-inputs` feeds a node a topic only when the node reads it as the
schema the session recorded, hash included, and otherwise refuses the
session naming the topic and both schemas
([input replay](../events/README.md#replaying-another-sessions-inputs)).
So adding a field to any message a replayed node subscribes to makes every
session recorded before the change unreplayable into that node by a later
build; replay such a session with the build that recorded it (a stamped
bundle's session names its `git_revision`). For `/hw/status`
(`bridge.BridgeStatus`), which every node with devices reads through
`HardwareLink`, that is every node with devices.

The packets do not say which version they are of (a recorded Packet holds
a frame's payload without its header, and its schema identity hashes only
its size). A session records it twice: the bridge puts its own build's
version on every `/hw/status` (`bridge.BridgeStatus` `protocol_version_id`),
and the launcher puts its build's in `manifest.json`, as
`hardware_protocol_version` ([session manifest](#session-manifest)). The
tools read a session under the bridge's: the first `/hw/status` the bridge
sent in each of its logs (each run and segment), resolving `protocol_version_id`
by field name from the recorded `.bfbs` schema so a `BridgeStatus` layout change
does not hide it. They use the manifest's only when no log records a `/hw/status`
whose schema carries an integer `protocol_version_id`: a session from before
the field, one with no bridge, or a bridge whose schema lacks the field
(`RecordedProtocolVersion`, `hardware_record.h`). While a session runs, the
launcher also opens `/hw/status` as an observer reader (using the bridge's
declared shape and resolving `protocol_version_id` from its `.bfbs`, counted as
one observer reader slot on `/hw/status` during phase-one graph lint) and warns
once per bridge run (to stderr and `talos.log`, re-warning if the bridge
restarts) when the live `protocol_version_id` differs from the launcher build's
`protocol::kProtocolVersion`. Finding the bridge's
costs a read of each log's header, and of the bridge's logs up to their
first status, never past their first second of records: the bridge's
`--log` records every status it publishes, on its first pass and every
20 ms after, so each segment holds one within its first 20 ms unless the
bridge stalled, or unless the segment holds almost no records because the
bridge's log share is at its floor: such a segment can hold no status and
records no version, so a session none of whose logs records one is read
under the manifest's. A bridge log with none in
its first second (one from before the bridge logged its statuses) records
no version. A session is a
directory's own `manifest.json` and logs, or for a `.tlog` file named on
its own, the `manifest.json` beside it and that
session's logs beside it when the manifest's `session_id` is the log
header's (otherwise the file alone: a manifest of another session, or
without a `session_id`, records nothing for it). When the bridge's and the
launcher's versions differ, each tool says so in one line and uses the
bridge's:

```
log_dump: warning: the session's bridge recorded hardware protocol <B> (/hw/status protocol_version_id), its launcher <L> (manifest.json hardware_protocol_version): read under the bridge's
```

`log_dump` and
`session_report` (through `log_dump`) decide once per session, before
decoding any hardware packet (`HardwareProtocolCheck`,
`hardware_record.h`), and when they refuse, print one line on stderr and
decode none of its hardware packets, the ones that happen to decode
included:

- **Another version is recorded.** No packet is tried; `<where>` is
  `/hw/status protocol_version_id, the bridge's` or `manifest.json
  hardware_protocol_version`:

  ```
  log_dump: warning: hardware packets not decoded: recorded under hardware protocol <R>; this build's is <N> (<where>)
  ```

- **The bridge recorded two versions**, by runs of two builds (a bridge
  restarted from another build, which one bundle cannot do). No packet is
  tried:

  ```
  log_dump: warning: hardware packets not decoded: recorded under hardware protocols <A> and <B> by bridge runs of different builds (/hw/status protocol_version_id); this build's is <N>
  ```

- **No version is recorded** (a session from before the manifest field
  with no bridge version, or a `.tlog` file with neither its session's
  manifest nor a bridge's status). Every
  hardware packet is tried with this build's codecs. Every one decoded
  under the build that recorded it, since the bridge publishes only a State
  or Driver Station packet its codec accepted and every request and command
  is its codec's encoding, so one this build rejects means another
  protocol:

  ```
  log_dump: warning: hardware packets not decoded: recorded under a hardware protocol other than this build's protocol <N> (the session records no protocol version); this build's codecs reject <k> of the <n> tried, the first on <topic>: <reason>
  ```

  A session of another protocol whose hardware packets all happen to
  decode, because the change kept every length, cannot be told apart this
  way and decodes with this build's layout, possibly wrongly.

- **This build's version is recorded.** Every packet is still tried, since
  the version is one binary's: one this build rejects was written by
  another codec (a node binary of another build, or, with only the
  launcher's version recorded, the bridge's too) or by hand. `<whose>` and
  `<what>` are `the bridge's` and `nodes` when the bridge recorded the
  version, else `the launcher's` and `bridge or nodes`:

  ```
  log_dump: warning: hardware packets not decoded: recorded under this build's hardware protocol <N> (<whose>), yet this build's codecs reject <k> of the <n> tried, the first on <topic>: <reason>; the session's <what> may be of another build
  ```

`session_report` prints `log_dump`'s disagreement line prefixed
`session_report:`. A record
damaged on disk is not among them: the log reader skips it by its checksum
and names the log (above). When `log_dump` refuses, it exports no `hardware`
and no `ds_enabled` member (JSONL), an empty `hardware` field (CSV) and no
hardware line (text, summary); `payload_hex` keeps the bytes. Every other
record is exported as before, and the exit status is 0. Read a session's
hardware packets with a build of the protocol the line names; a stamped
bundle's session also names its source revision (`git_revision`).

Each publication's bytes are the same in every log that recorded it, so the
check tries one copy: the writer's SEND records, and readers' copies only of
a topic whose SEND it has not seen. `log_dump`, which streams the session
in bounded memory, decodes a printed packet a second time.

`--replay-inputs` refuses a session recorded under another hardware
protocol (the bridge's version, else the manifest's, as above), or whose
bridge recorded two, before it starts the node, with one line,
`launcher: --replay-inputs <session> was <refusal>: not replayed`, where
`<refusal>` is the text after `hardware packets not decoded: ` above, and
exit status 1: the node would get States of another layout, and its run
could not match the recording. It prints the disagreement line above,
prefixed `launcher: --replay-inputs <session>: `, when the bridge's and the
launcher's versions differ. A session with no recorded version is
replayed: the node's devices layer decodes each `/hw/state`
with this build's codec, so a State it rejects is no State,
`robot_enabled` stays false and every output is neutral, but a State of
another protocol that kept its length is decoded, wrongly.

Retention's match classification ([above](#session-segments-and-logs))
reads Driver Station packets the same way: a session recorded under
another protocol (the bridge's version, else the manifest's), or whose
bridge recorded two, is `unknown` (kept like a match) without its packets
being read, and one with no recorded version is read with this build's
codec, `unknown` when that rejects a packet.

### Session measurements

```sh
bazel run //talOS/launcher:session_report -- --strict /tmp/logs/session
bazel run //talOS/launcher:session_report -- --json /tmp/logs/session
```

A repository that depends on talOS runs `@talos//talOS/launcher:session_report`.

`bazel run //talOS/launcher:stop_paths -- <session> [--config robot.toml]`
reads the bench's stop-path trials from a session's logs (`stop_paths.py`; see
[BENCH.md "Reading a trial"](../../docs/BENCH.md)).

The offline Python tool reuses `log_dump` and its events reader/embedded
descriptor decoder, without robot-specific schemas. It reports manifest config
identity and source revision, node exits/restarts/quarantine/recorder
failures/caps, every log gap, each node's loop tier and realtime outcome, each node's
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
session, state inter-arrival min/median/nearest-rank p99/max, missed
`/hw/state` sequence counts per boot (`state_sequence_gaps`),
enabled gaps of at least the manifest's `command_timeout_us`
(`enabled_gaps` and its bound `enabled_gap_bound_ms` in `--json`, which also gives the p99 rows' bound as `state_p99_bound_ms`), target MESSAGE receipt ages, timer dispatch
lateness, the bridge's request-to-command delay in enabled and disabled
time, the [latency rows](#latency-rows), every scalar `/hw/status` field,
and the alerts.
The required "controller link" verdict is whether the bridge ever accepted
a State from the controller, which a launched session's exit status cannot
say (the launcher ends a bounded session by stopping every member, and a
stopped bridge exits 0 whether or not a State arrived; it prints
`hardware_node warning: ended without receiving a State from the controller
at <address>:<port> (ran <s> s)` on stderr instead). PASS names the first
accepted State's time from the start of the log (`first_state_ms` in
`--json`), from the bridge's first `/hw/state` record (it publishes only a
State it accepted), else from the first `/hw/status` record with a nonzero
`state_age_us` or a clear `alert_error_link_lost`, less that record's
`state_age_us` (the record follows the State by up to one status period;
clamped at 0); FAIL is `/hw/status`
records and none of either; NOT MEASURED is neither topic logged.
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
to command publication, the requesting node's work included, not arrival at
the RoboRIO: `/hw/command` is published and logged only after the bridge's
UDP send succeeds (`socket_send_failures_count` and
`socket_send_would_block_count` on `/hw/status`, and `send_failures` /
`send_would_block` in the exit line, count sends that did not go out), though
UDP delivery across the wire
is still unacknowledged. With several owners the
merged observation can be an older request's, overstating it. Each sample
is counted in enabled or disabled time (`bridge_request_to_command`
`enabled` and `disabled` in `--json`, each with the bridge's `process`) as
the bridge decides its send: "enabled" means the State the bridge held, the
last of the selected `/hw/state` stream, was configured, enabled and not
faulted (`hardware::GatewayReady`) from the request to the command, and
"disabled" that it was not ready throughout; a sample across a change, or
that starts before the first State, meets that stream's log gap or holds a
State that did not decode, is in neither. Not ready, the gateway cannot
take a command, and a merge whose only change is its observation (a
disabled node's requests are neutral) waits for the bridge's heartbeat
(`command_timeout_us` / 5, clamped to 5 to 20 ms), so only the enabled row
measures its reaction. The rows are informational: REQ-RT-20's 1 ms bound is verified by
the bridge's component test. The bridge's `wait_fallbacks` and stalled request topics reach the
report as its row's `wake_fallbacks` and `stalled_arms` (delivery loss,
above). Its five socket and UDP counters (`send_failures`, `socket_reopens`,
`sequence_gaps`, `decode_failures`, `send_would_block`) are published on
`/hw/status` and surfaced in `session_report` (`bridge_socket_counters` in
`--json`, `Bridge UDP/State loss counters:` in Markdown) as well as its exit
lines; only its other wake counters (`passes`, `udp`, `request`, `deadline`,
`throttled`) are only in its exit lines on stdout, in its
[node output](#node-output) log, not in the report.
Counter deltas sum nonnegative increments and separately count resets.
`clock_offset_valid` includes its observed valid fraction. JSONL `log_dump`
exports also carry `ds_enabled`, the Driver Station `hardware.enabled` bit,
for each Driver Station packet that decodes.

State timing selects one publisher stream (bridge IPC publications), or one
consumer stream if there is no publisher recording; duplicate readers are never
pooled. Enabled intervals hold the logged DS enabled bit until its next sample,
not a reconstructed safety gate. A `/hw/state` interval overlapping an
enabled interval counts.

A log gap (a GAP record, `log_dump`'s JSONL `kind` `GAP` with its `gap`
member) is a stretch of records its log's writer dropped. The report lists
each one as a session issue, `<process>: log gap from <start_ms> to <end_ms>
ms, <records> records lost (<cause>[, closed at the stop])`, and in `--json`
as `log_gaps` (each `gap` member with its `process`), so `--strict` fails a
session with one, naming it, even when every verdict passes; the manifest's
`log_gap` is a session issue as `recorder_failed` is. An interval is measured
only if no record it depends on could be missing inside it. Every time a
dropped record named (its now, event and publish times) lies in the gap's
lost span, `[min(start_ms, first_lost_ms), last_lost_ms]`, or `[start_ms,
end_ms]` for a GAP without its detail; the record capture resumed with was
kept, so the span stops at the last record lost. An interval `[a, b]` is not
measured when it meets (closed at both ends) a lost span of a log it reads:

| Measurement | Interval | Logs whose gaps make it unmeasured |
|---|---|---|
| `/hw/state` overall and enabled | two consecutive records of the selected stream | that stream's process |
| enabled or not (enabled rows, and an active lane "while enabled" for target age at tick) | from the Driver Station record whose bit is held (from the log's start before the first) to the interval's end or the lane's time | the Driver Station stream's process: such an interval is neither enabled nor disabled |
| bridge request to command | from the request before the first one carrying the observation in that node's `/hw/request/<node>` stream, or from the log's start before that stream's first request (where an earlier request carrying it could be missing), to the command | the bridge's, and that of every node that sent a request |
| state to request | from the State's publication to the owner's first request carrying it | the owner's |
| Driver Station to lane | from the packet's publication to the producer's next lane publication, or to the stream's next packet when that comes first (a gap leaves the packet in neither the samples nor either unanswered count) | the producer's, and the Driver Station stream's process (a lost packet would widen the window) |
| Driver Station to command | from the packet's publication to the command | the bridge's, and that of every node that selected a lane or sent a request |
| alert active time | an alert's active span (publication times) | its status stream's process; the span still counts as an activation, but is not timed (`spans_across_gap` in `--json`, NOT MEASURED when no span is left) |

Target receipt age, target age at the tick and timer lateness each read one
record, so no value of theirs spans a gap, but the samples a gap dropped are
missing: their worst value is the worst the log kept, not the worst the run
had. For a process whose log has a gap, the target age at tick and timer
lateness details end with `; <process> lost <n> records in <m> log gaps`,
and a line under the timing table says its receipt, target at tick and timer
rows lack those samples (`records_lost` in `--json`). The `/hw/status`
counter deltas difference cumulative counts, so a gap leaves them whole but
for a counter reset inside one, which goes uncounted; the report counts no
missed ticks or overruns. A row whose intervals all meet a gap is NOT
MEASURED, never a pass, and its detail (or, for request to command and the
latency rows, a line under their table) names how many intervals were left
out; `--json` gives the counts in `intervals_across_gaps`
(`state_overall`, `state_enabled`, `state_sequence`,
`bridge_request_to_command`, `state_to_request`, `driver_station_to_lane`,
`driver_station_to_command`). A session without a gap reports
exactly as before, with none of these keys.

Median “about 5 ms” uses an explicitly labeled 4–6 ms analysis tolerance; p99
must be below twice the manifest's `period_us` (the gateway's tick; 10 ms at
the bench's 5000 µs), and an enabled interval at or past its
`command_timeout_us` (100 ms at the bench's) is a gap. A manifest without
`period_us` (written before the launcher recorded it, or not a positive 32-bit
integer) leaves `overall state p99` and `enabled state p99` NOT MEASURED,
naming `period_us`, and `state_p99_bound_ms` null; without
`command_timeout_us`, `enabled state gaps` is NOT MEASURED, naming
`command_timeout_us`, and `enabled_gaps` and `enabled_gap_bound_ms` are null.
`--strict` exits 2 unless every required verdict is PASS
and the manifest shows a finished session (`end_wall_ns` nonzero) with no
recording node (non-empty `log_path`) `quarantined`, `recorder_failed`,
`log_capped` or `log_gap`, and no log gap (above); host processes without
an event log (`studio_bridge`, `sim_gateway`) are not acceptance evidence and are exempt, with no log
`log_dump` reports damaged (above), each named with its file and byte offsets,
and with every bridge and control loop holding its full realtime grant. Off
`--sim` a `partial`, `refused` or `not_requested` outcome fails; under `--sim`
only a loop that asked (`TALOS_REALTIME=1`) and was refused a step does.
Observers, nodes that never reported, and manifests that predate the fields
(*NOT RECORDED*) are listed but not judged. It also fails on any node's
nonzero `wake_failures` (a reader left asleep until its backstop),
`requests_skipped` (a request its subsystem never merged), `stalled_arms`
(a watcher whose failed arms stopped holding its loop awake, so its
publications waited for the loop's timers or backstop) or `wake_fallbacks`
(a run that drained on a poll tick instead of on publication, so its
prompt handlers' latency and the no-polling guarantee did not hold),
naming the node and the count (`<node>: stalled_arms=1`); a manifest
without the fields is not judged. It names each reason on stderr. It exits 1 for input/reader errors (including a missing `log_dump`
runfile) and 0 otherwise. NOT MEASURED is never a pass; only the timer
lateness, bridge request-to-command and latency rows are informational and
not required. The enabled-only verdicts (enabled state median, p99 and gaps,
target age at tick) cannot be measured in a session that never enabled, and
the report says so in one line; `--strict` still fails them, so acceptance
needs an enabled session. In a session whose hardware packets `log_dump`
does not decode ([which logs a build can read](#which-logs-a-build-can-read):
it records another hardware protocol, or this build's codecs reject one
of its packets, whatever version it records), `log_dump`
decodes no Driver Station packet, so the same verdicts are NOT
MEASURED with `log_dump`'s line in their detail (target age at tick too:
without the enabled bit a lane `Target` never selected cannot be found),
the line is a session issue (`hardware_not_decoded` in `--json`), and
`--strict` names it and exits 2. The `/hw/state` cadence rows (overall
median and p99) and the controller link use only record times and are
still judged. Read such a session with a build of its protocol for
acceptance.

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

#### Latency rows

The Latency section (REQ-RT-25) measures the path from a sample or a Driver
Station packet to a command, from records a session already logs. Each
value is the difference of two readings of one clock: the report subtracts
only main-processor times (a record's `now_ms` or `publish_ms`) from each
other. A RoboRIO time (`sample_time_us`, `observed_time_us`) is only matched
or ordered against another of the same `boot_id`, never subtracted from a
main-processor one, whose mapping to it is uncertain by milliseconds
([gateway clock accuracy](../../docs/DATA_FLOW.md#gateway-clock-accuracy)).
Each row is min, p50 (median), nearest-rank p99
and max with its count, as the timing rows are, and NOT MEASURED when it has
no sample (never 0). The rows are informational: `--strict` judges none of
them, since REQ-RT-26's bound is judged at the bench.

| Row (`--json` key) | What it measures | Records | Clock |
|---|---|---|---|
| feedback age (`feedback_age_us`, per `"motor <id>"` and `"sensor <id>"`, µs) | the older of a motor's or CAN sensor's `position_age_us` and `velocity_age_us` in each State (a sample marked invalid is skipped). A sample with an age of 4294967295 (UINT32_MAX, the RoboRIO's value when Phoenix gave no timestamp) is left out of the figures and counted in `sentinel` (and named in the text row). A simulation session's row reads 0 by construction: the simulated backends report age 0, except for a device a fault injection froze | the selected `/hw/state` stream's decoded `hardware` | the RoboRIO's: each age is its own reading of the signal's age at the read |
| state to request (`state_to_request_ms`, per actuator owner) | a State's publication to the tick of the owner's first request carrying it, joined (`boot_id`, `sample_time_us`) to (`boot_id`, `observed_time_us`) as request to command joins | the State stream's `publish_ms`; each `/hw/request/<node>` SEND's `now_ms` | the main processor's |
| Driver Station to lane (`driver_station_to_lane_ms`, per producer) | a Driver Station packet's publication to the producer's next lane publication at or after it, a release included, taken only when that publication comes before the stream's next packet (the last packet's window has no end): the reaction of a producer that is publishing. A packet the producer did not answer before the next one gives no sample and is counted per producer in `driver_station_to_lane_unanswered`, as `{"total": t, "enabled": n, "other": m}` by the packet's own enabled bit (the one the enabled rows hold; `other` is disabled), and a line under the table names nonzero counts, the enabled one first. In a session whose packets were not decoded (another hardware protocol) there is no enabled bit: `enabled` and `other` are null, as every row needing the bit is, and the line gives the total with the split NOT MEASURED. So a producer that is silent, as `teleop` is while disabled, shows as an `other` count, not as a wait spanning the silence, and one that stalls while enabled, which the row's samples cannot show since none exceeds one packet gap, shows as an `enabled` count. The `enabled` count is not purely stalls: an operator producer is silent by design while enabled until its `OperatorArming` arms (a stick held through the enable), and in a mode it does not serve (`teleop` in autonomous), and the report cannot tell these from a stall. A producer slower than the 20 ms packets answers only the packets in its last window before each send, so its row is at most one packet gap and its counts are not zero. It lists every process that publishes a target lane, whatever its source (`teleop`, `auto`, `align`, ...), so a producer whose lanes do not derive from the packet is listed too | `/hw/state/driver_station`'s `publish_ms` (a reader's copy carries the bridge's); each target-lane SEND's `publish_ms` | the main processor's |
| bridge request to command (`bridge_request_to_command` `enabled`, `disabled`) | above | above | the main processor's |
| Driver Station to command (`driver_station_to_command_ms`) | for each packet showing the robot enabled in teleop (the packet's `mode`; autonomous and test give no sample), its publication to the first command built from it: the earliest tick in which a `Target` selected a lane published at or after this packet and before the next one (its READ) and its node sent a request (the same dispatch of the same run: a restarted node counts its dispatches from 0 again, so a process's records start a new run after its EXIT or where its dispatch index falls), then the bridge's first command at or after that tick, of the same boot, whose observation is no older than the request's; the robot still enabled at the command. A merged command carries its oldest owner's observation, so with two actuator owners the row waits for the other owner's next request and overstates by up to that owner's period; a one-owner config gives an exact figure | the packet's `publish_ms`; READ and request records; `/hw/command` SENDs' `now_ms` | the main processor's |
| clock drift (`clock_drift_ppm`) | the least-squares slope of the valid `clock_offset_us` (`clock_offset_valid` true) against `/hw/status` publication time, fitted within each estimator (one `clock_boot_id` and `clock_resets_count`, split at the status log's gaps) and pooled; null without two such samples | `/hw/status` records | both readings map the gateway's clock to the main processor's at one instant: their slope is the oscillators' relative drift |

State to request ends at the tick's start, where request to command begins,
and Driver Station to lane ends at the lane's publication, where target age
at the tick begins, so the rows add up along each path. The spread of state
to request over a session shows whether the node's timer phase against the
RoboRIO tick swept the whole period; a session shorter than one sweep (one
period divided by the drift) shows only its own band. Driver Station to
lane is mostly the producer's timer phase against the RoboRIO's 20 ms
Driver Station forward: `teleop` does not wake on a packet, it drains the
Driver Station topic before its own timer and sends then. Driver Station to
command follows only an operator path through a target lane into an
actuator owner's request; while the lane is released (sticks neutral, or
before `OperatorArming` arms after an enable) `Target` selects nothing and
logs no READ, so those packets give no sample.

A disabled bench exercises part of the path: the CAN read and feedback
ages, the RoboRIO's read and State send, the bridge's receive and publish,
each owner's timer, `update()` and flush (its requests are neutral but carry
the observation, so state to request is exact), the bridge's request read
and merge, and the Driver Station chain up to the producer. It does not
exercise Driver Station to lane: `teleop` releases its lanes once when it
stops commanding and then publishes nothing while disabled, unarmed or
without a connected stick (`talOS/teleop/node.h`); only while it is armed
in an enabled teleop does it publish on every tick (a target, or a release
with the sticks neutral). A disabled session's packets are therefore
unanswered and counted under `other`; the row's samples, and the
`enabled` count that shows a stall, come from its enabled spells. Nor does it
exercise the bridge's change-driven send (disabled, it waits for the heartbeat, so the disabled
request-to-command row is no reaction time), the RoboRIO's acceptance and
apply, or `Target` selection: bridge request to command enabled and Driver
Station to command need an enabled session. With two actuator owners a
merged command carries the older owner's observation, so a per-owner figure
on the RoboRIO side needs a one-owner config.

The RoboRIO's half of the path is in no session record: the age of a
command's observation when the RoboRIO takes it, and a Driver Station
refresh to its send, are printed on the RoboRIO's console as its
`[rio] latency` line, on the RoboRIO's clock
([RoboRIO](../hardware/README.md#roborio)).

## If you change the shape of `graph.json`

Studio parses this document in TypeScript (`studio/src/system.ts`), and neither
language can check the other. `studio/tests/launcher_graph.json` and
`launcher_graph_faults.json` are real output from

    bazel run @talos//talOS/launcher:launcher -- --describe-only \
      --config $PWD/main_processor/configuration/robot.toml

in the 2026 robot's repository, with `session_id` and `config_path` pinned,
and `//studio:declared_test` parses both. A talOS type's schema hash and size
can be refreshed from this repository's own `--describe-only` graph (the
bench's); the robot-defined types come only from a run there (the header
comment of `studio/tests/declared.test.ts`). So a
change to the envelope here fails there rather than reaching a viewer as a
silently empty panel. Regenerate the fixtures and fix the parser in the same
commit; the faults fixture is the same robot with `drivetrain` publishing the
misspelled `/hw/req/drive` instead of `/hw/request/drivetrain`, which is where
the diagnostic wording is pinned.

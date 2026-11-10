# Launching a robot

The launcher probes configured binaries with `--describe`, lints their declared
graph, creates session-owned topics, and supervises the roster.
Architecture and configuration invariants live in
[ARCHITECTURE.md](../../ARCHITECTURE.md); graph naming rules live in
[NAMING.md](../NAMING.md).
## Two phases

**Phase one asks.** Every binary in the config is run with `--describe`. A node
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
as nodes start, restart or exit, or report a stopped event log; normal shutdown
adds final log counts.

`--describe-only` stops after phase one, which makes the check useful on its own
— in CI, in a pit before a match, or after a rename that touched two packages.

`--allow-graph-errors` overrides graph-error refusal. Missing descriptions
cannot create segments for undeclared topics.

## When a binary does not answer

A failed critical probe is an error; a failed non-critical probe is a warning.
`--describe-timeout-ms MS` sets the per-binary deadline. Missing descriptions
remain recorded in `graph.json` and the session manifest.

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
`--local-port` override their config counterparts the same way. The launcher
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
lands in `<output_dir>/<session_id>/studio_bridge.log`
(plaintext, not a `.tlog`) beside the manifest.

The bridge refuses to start before the telemetry publisher exists, so it is the
one roster member supervised differently (`StudioBridge` in `launcher.h`). Its
first attempt spawns with the nodes; a quick exit is retried at most once a
second for up to 60 seconds, inside the supervision loop, so it never delays the
session or its `--duration-s` deadline. While it is still starting it does not
hold the session open. Once it has survived a second it is supervised like any
node, except that it is never restarted under the policy below. If it never
comes up it is marked failed in the manifest and the session runs on without
Studio.

## Restarts

A roster member — `hardware_node`, every subsystem node, `sim_gateway` — that
exits with a nonzero status or by a signal while the session is not stopping is
restarted. So is a critical member that exits cleanly (status 0) while the
session is not stopping: its peers still run, so it did not finish anything.
A run that cannot be spawned (fork or pipe failing, as under `EAGAIN` or
`EMFILE`), a first run included, is retried the same way and marks the member
`recorder_failed`. Every restart attempt counts against the budget below,
spawned or not; the retry keeps that run's log name and restart number.
The first restart waits 50 ms, each later one twice as long, capped
at 1 s. Critical nodes (including every actuator owner and the hardware bridge)
never quarantine: they keep retrying with that capped backoff. Non-critical
nodes may restart at most `max_restarts` times within a rolling 60-second
window (not over the whole session):

```toml
[robot]
max_restarts = 5  # per node per rolling restart window; 0 disables restarts
```

The key is optional (default 5) and must be an integer >= 0; `--max-restarts N`
overrides it for one run. A non-critical member's clean exit is a node that
finished and is not restarted. When a critical member's clean-exit restart
falls due and every other member's latest run also exited 0 (the whole roster
completed, as one-shot fixtures do), it is dropped and the session ends.
Nothing is restarted once a signal (requested stop, launcher shutdown) or the
`--duration-s` deadline has started stopping the session. The backoff is
scheduled inside the supervision loop, like the Studio bridge's retries, so a
restarting node never delays another node or the deadline. Each restart prints
one line:

    [launcher] alpha exited (signal 11); restart 1 in 50 ms

When a non-critical node's rolling budget is exhausted, it is **quarantined**, not
permanently abandoned. Exactly one retry is scheduled for the oldest restart
plus the window and normal backoff. Quarantine and retry transitions are
logged; the live manifest exposes `quarantined` and
`retry_at_monotonic_ns` (zero with no pending restart). Stopping cancels the
retry; `max_restarts = 0` never schedules one. A persistent fault remains
rate-limited, while a cleared fault can recover later in the same session.

Every run writes its own event log rather than truncating the last one: the
first run `<name>.tlog`, restart `k` `<name>.restart<k>.tlog`. The session
manifest records each node's `restart_count` and every log in `log_paths`
(`log_path` stays the first run's), and its `record_count` sums all of them.
`log_dump` on the session directory merges every `*.tlog`, restarts included.
A node's `exit_code` and `pid` are its last run's.

A run whose event log stops early says so in its registry row while it runs
(see [event log state](../introspection/README.md#event-log-state)). The
launcher prints one line per member and kind, and marks the member in the live
manifest: `recorder_failed` for a failed log, `log_capped` for one at its byte
budget. At normal shutdown it also marks a truncated or unreadable log
`recorder_failed` and a capped one (`<log>.capped`) `log_capped`; a log whose
capture stopped at a record boundary looks complete there, so only the live
row reports it.

`--heartbeat-periods N` defaults to 10 periods, with a minimum timeout of
500 ms. `--heartbeat-startup-grace-ms MS` defaults to 5000 ms for processes
that have not registered. Such processes are killed once both that grace and
the heartbeat timeout have elapsed; registration is required even when
`--allow-graph-errors` permits a failed description.

Construction is visible in the registry before endpoints are available and has
a fixed deadline from process spawn. See the
[registry lifecycle](../introspection/README.md). Construction failures and
timeouts enter the ordinary restart/quarantine policy.
`--restart-window-ms MS` overrides the default 60000 ms rolling budget window.

Every spawned child has its own process group; restart, stop and shutdown
signal the group, and even a clean exit removes remaining helpers. A guardian
in each group watches a launcher-owned lifetime pipe and kills the group on
EOF, including after launcher SIGKILL. Linux also uses `PR_SET_PDEATHSIG`;
macOS relies on the pipe. Children must not daemonize out of their group.

The requested-stop supervision test acknowledges installed signal handlers in
all three stub nodes before signaling the launcher. Every stub stays alive until
that stop, and the test checks zero restarts and clean exits for the whole roster.
An immediately exiting hardware stub instead exercises an unrelated restart/stop
race: a signal before its executable starts can legitimately terminate it.

With `NOTIFY_SOCKET` set, the launcher sends `READY=1` on entry to supervision
and `WATCHDOG=1` on every loop iteration, using nonblocking Unix datagrams
(including Linux `@` abstract names). A hung loop therefore stops feeding the
service watchdog. See [deployment](../../docs/DEPLOYMENT.md) for the generated
systemd watchdog policy and macOS's launchd limitations.

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


## Building

```sh
bazel test //talOS/launcher/...
bazel run //talOS/launcher:launcher -- --config <path-to>/robot.toml --describe-only
```

Node targets resolve to `bazel-bin/<package>/<target>`, unless an override
names the binary. A bundle's `bundle_manifest.toml` (written by
`talOS/build/bundle_builder.py`, loaded by `--bundle` or found in the working
directory) lists each node's name, targets, `bin/` copy and language,
plus the config path and its SHA-256. The launcher also reads a
workspace-relative config, so outside a bundle it only makes sense with the
workspace as its working directory; `bazel run` starts a binary in its
runfiles tree instead, and `main.cc` chases `BUILD_WORKSPACE_DIRECTORY` back so
both spellings behave the same.


## Session segments and logs

`--output-dir DIR` always names a root: each launch writes to
`DIR/<session_id>/`, including deployed `TALOS_LOG_DIR` roots. Without the flag,
the root is `/tmp/talos_logs`. Use persistent storage on a deployed robot.
`manifest.json` is written before nodes start, flushed and synced, then replaced
atomically at each process lifecycle change and when a node reports a stopped
event log; an interrupted session retains its last complete manifest
(`end_wall_ns = 0` means no normal shutdown).

At startup, retention removes the oldest sessions: directories holding a
`manifest.json`, or, named by a session id, an event log whose header carries
that id (what `--replay-inputs` leaves). `[robot] log_keep_sessions` defaults
to 50, including the new session; `log_max_bytes` defaults to 2 GiB. Both must
be positive. Half the byte budget is reserved for the new session's event logs
and divided among logging members; restarts share their member's remaining
budget. Older sessions are retained within the other half. A directory named
by a session id that holds nothing, or only what a launch writes before it
starts its nodes (`graph.json`, `declared_graph.json`, the `.tmp` of an
interrupted write, empty `.tlog` files), is a launch that recorded nothing:
`--describe-only`, a refused graph, or a launch that failed or died early. It
never counts against `log_keep_sessions` or the byte budget, and it is removed
once another session directory is newer: one newer than every other (a
`--describe-only` result just written, or a launch still describing) survives
the next launch, and a crash loop, refused graphs included, leaves one such
directory instead of evicting recorded sessions. Symlinks, other directories
(including one named by a session id that holds another session's log or any
other file), the current session and the session `--replay-inputs` reads are
not deleted. A session retention cannot read or remove (another user's,
unreadable, removed meanwhile) is reported and passed over; it never stops a
launch. Graph/manifest metadata and non-event captures are outside the
event-writer caps.

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

Repeat `--node` or `--topic`, or separate their values with commas, to select
alternatives within that filter. Different filters are combined with AND.
Use `--` before a path beginning with `--`. Unknown flags and invalid values
are usage errors.

```sh
bazel run //talOS/launcher:log_dump -- /tmp/logs/session \
  --node drivetrain,pose_estimator --topic '/drivetrain/*' \
  --since-ms 1000 --until-ms 5000 --format jsonl
```

Both export formats contain `process`, `dispatch_index`, `kind`, `source_id`,
`topic`, `event_ms`, `now_ms`, `sequence`, `aux`, `payload_bytes`, `schema`, and
`decoded`. Times keep the same session baseline regardless of filters.
`schema` is empty when the source has no schema; `decoded` is empty without a
schema or payload, otherwise it contains the descriptor-decoded payload text.
JSON strings are escaped; CSV fields containing commas, quotes, or newlines
are quoted and embedded quotes are doubled.

### Session measurements

```sh
bazel run //talOS/launcher:session_report -- --strict /tmp/logs/session
bazel run //talOS/launcher:session_report -- --json /tmp/logs/session
```

The offline Python tool reuses `log_dump` and its events reader/embedded
descriptor decoder, without robot-specific schemas. It reports manifest config
identity, node exits/restarts/quarantine/recorder failures/caps, an unfinished
session, state inter-arrival median/nearest-rank p99/max,
enabled gaps of at least 100 ms, target MESSAGE receipt ages, timer dispatch
lateness, and every scalar `/hw/status` field.
Counter deltas sum nonnegative increments and separately count resets.
`clock_offset_valid` includes its observed valid fraction. JSONL `log_dump`
exports additionally include `ds_enabled` for successfully decoded Driver Station
packets, using the framework wire codec rather than guessed byte offsets.

State timing selects one publisher stream (bridge IPC publications), or one
consumer stream if there is no publisher recording; duplicate readers are never
pooled. Enabled intervals hold the logged DS enabled bit until its next sample,
not a reconstructed safety gate. A gap overlapping an enabled interval counts.
Median “about 5 ms” uses an explicitly labeled 4–6 ms analysis tolerance; p99
must be below 10 ms. `--strict` exits 2 unless every required verdict is PASS
and the manifest shows a finished session (`end_wall_ns` nonzero) with no
recording node (non-empty `log_path`) `quarantined`, `recorder_failed` or
`log_capped`; host processes without an event log (`studio_bridge`,
`sim_gateway`) are not acceptance evidence and are exempt. It names each reason
on stderr. It exits 1 for input/reader errors (including a missing `log_dump`
runfile) and 0 otherwise. NOT MEASURED is never a pass; only the tick-time
target age and timer lateness rows are informational and not required.

Target receipt age is `now - publish_time`, not target age at a control tick.
Polled subscriptions and the selected Target value are not recorded as tick
inputs, so tick-age/timeout acceptance remains NOT MEASURED. Timer lateness has
no specified acceptance threshold. Capped, truncated, unfinished recordings,
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

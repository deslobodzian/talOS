# Launching a robot

A robot program is a set of processes that have to meet on a set of names. Each
one is written on its own, builds on its own and runs on its own, and none of
them can see the others: an RTMS topic is a POSIX shared-memory object addressed
by exact string, so a publisher on `/drivetrain/target` and a subscriber on
`/drivetrain/tgt` are both working perfectly. They simply never meet. Nothing in
either process is in a position to notice, because from the inside there is no
difference between a topic nobody has published to yet and a topic nobody will
ever publish to.

The launcher is the one place that can notice. It reads the roster out of
`robot.toml` and starts every process in it, so for one moment it holds the
whole graph — and it holds it before any of it exists, which is the only moment
when refusing to continue is cheap.

## Two phases

**Phase one asks.** Every binary in the config is run with `--describe`. A node
given that flag builds itself on a `SimulatedEventLoop`, prints the manifest its
constructor registered as JSON, and exits zero. The simulated loop's channels
live inside the process, so this touches no shared memory, no hardware and no
network: probing a robot's roster is safe on a laptop with nothing plugged in.
The launcher parses those manifests, assembles the graph they describe, and runs
`naming::LintGraph` over it. Every finding is printed. If any of them is an
error, nothing is spawned and the launcher exits non-zero.

**Phase two starts.** Only then does the launcher fork the roster and supervise
it. It persists the session manifest before spawning and atomically replaces it
as nodes start, restart, or exit; normal shutdown adds final log counts.

`--describe-only` stops after phase one, which makes the check useful on its own
— in CI, in a pit before a match, or after a rename that touched two packages.

## Why the description comes from the binary

The obvious alternative is to write the topology in the config file the launcher
already reads. Each subsystem would list what it publishes and subscribes to,
and phase one would be a loop over a table instead of eight forks.

That would be a second declaration of the same fact. The topics are already
declared, in each node's constructor, in the `make_sender` and `watch` calls
that actually create the endpoints — and a second copy of a fact is a copy to
forget to update. The failure mode of a stale topology table is precisely the
failure the check exists to catch: someone renames a topic in one node, the
table still says the old name, and the launcher confirms a graph that will not
wire up. A check that is wrong in exactly the situation it was written for is
worse than no check, because it is trusted.

So the launcher asks the binary, and the binary answers from the constructor
that builds the real loop. The only difference between the described graph and
the running graph is which loop the node was handed. When they disagree, it is
because the node behaves differently in simulation, which is an interesting fact
about the node rather than a bookkeeping error about a file.

The same reasoning is why identity comes from the binary too. Each node's `main`
holds its name and its Bazel target in one pair of constants, used by both
`--describe` and the `Reporter` that publishes into the live registry. The
declared graph and the running graph therefore cannot disagree about who a node
is, and a viewer can join one to the other on a name it knows is the same name.
If the config's roster key and the binary's own name differ, the launcher says
so — as a warning, since a confusing session is not a broken one.

## What refuses a launch, and what only gets mentioned

The severities come from `naming::LintGraph`, not from here; the launcher's only
policy is that an error refuses.

An error is something that cannot be right in any configuration: two writers on
one topic, which makes the dispatch log unreplayable; one topic used with two
message sizes; a subscriber whose topic nothing publishes; a target lane
(`/<sub>/target/<source>`) whose source is in no mode of that subsystem's
target policy, so nothing will ever read it. A warning is a state that is
normal while a robot is being built, such as a feed nobody has subscribed to
yet, or a policy lane nobody writes yet (`/drivetrain/target/auto` until
autonomous exists). The launcher hands the lint every subsystem's policy
sources from the config (`[robot.targets]` plus each subsystem's `[targets]`).

Two things keep that distinction honest, and both of them matter more than they
look:

- A node declares endpoint attributes for the ends whose far side is not a node
  in this session. A `talos::Target` subscribes to its policy lanes optional,
  because a lane with no producer is normal (the lint still warns about it,
  once, by name); the hardware bridge marks `/hw/command` external, because the
  RoboRIO consumes it over UDP and no shared-memory subscriber will ever
  appear. Without these the report would flag a known design decision as an
  error on every single launch, and a report that always complains is a report
  nobody reads — which costs exactly the finding it was built to deliver.

- Timers are described but not linted. A timer is an event source with no far
  end, and its name is a label like `swerve`, not a path, so measuring it
  against the topic grammar would call every periodic node malformed.

`--allow-graph-errors` launches anyway. It exists because the right answer in a
competition queue is not the right answer at a desk: a robot that has to move in
four minutes cannot be blocked by a linter, and the person launching it can see
the errors printed above the decision. The default is still refusal, because the
error being reported is silent at runtime — no log line, no exception, no
timeout, just a topic with one end.

## When a binary does not answer

A binary that ignores `--describe` produces a warning and the launch proceeds.
This is deliberate: a node someone is halfway through writing must stay
launchable, and a framework that refuses to start what it cannot describe is a
framework people route around. Its topics are missing from the declared graph,
which is recorded per node in `graph.json` and in the session manifest, so a
graph that passed because half of it went unread does not look like a graph that
passed.

The probe has a timeout — fifteen seconds by default,
`--describe-timeout-ms` to change it — after which the probe is killed and
reported. A node that blocks in its constructor waiting for a device is a bug
worth seeing, and it must cost the launch seconds rather than the session. The
default leaves room for a Python node's first run from a freshly deployed
bundle, when macOS checks each native library a wheel brings (several seconds
for `robotpy-wpimath`; under one second afterwards).

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

All three keys default to the loopback contract (`127.0.0.1:5802`, local
`5803`), so simulation and loopback tests need no extra configuration.

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
otherwise its `--config` supplies the address. No `--sim` on a real robot;
under `--sim` (or against loopback) the bridge treats the config as
commissioned, while on hardware the config's `commissioned` flag gates
commands.

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

Without `--physics`, the gateway uses ideal actuators and declares no truth
sender; with it, the launcher creates `/sim/state` and telemetry observes it.
The gateway registers its live identity with the session ID like other roster
members, but heartbeat/stall supervision remains disabled for this process.
`sim.sh --sim-gateway` supplies `--physics`; its repeatable `--gateway-arg ARG`
forwards additional settings through the same launcher interface.

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

Every key is optional; an absent `[studio]` table leaves these defaults. The
watched topic is not repeated here: it comes from `[subsystems.telemetry]`
topic, the same key the telemetry node reads. `--studio-http-port PORT`
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
restarted. The first restart waits 50 ms, each later one twice as long, capped
at 1 s. Each node may restart at most `max_restarts` times within a rolling
60-second window (not over the whole session):

```toml
[robot]
max_restarts = 5  # per node per rolling restart window; 0 disables restarts
```

The key is optional (default 5) and must be an integer >= 0; `--max-restarts N`
overrides it for one run. A clean exit (status 0) is a node that finished and
is never restarted, and nothing is restarted once a signal or the
`--duration-s` deadline has started stopping the session. The backoff is
scheduled inside the supervision loop, like the Studio bridge's retries, so a
restarting node never delays another node or the deadline. Each restart prints
one line:

    [launcher] alpha exited (signal 11); restart 1/5 in 50 ms

Every run writes its own event log rather than truncating the last one: the
first run `<name>.tlog`, restart `k` `<name>.restart<k>.tlog`. The session
manifest records each node's `restart_count` and every log in `log_paths`
(`log_path` stays the first run's), and its `record_count` sums all of them.
`log_dump` on the session directory merges every `*.tlog`, restarts included.
A node's `exit_code` and `pid` are its last run's.

Heartbeat supervision reads the introspection registry without modifying it.
`--heartbeat-periods N` defaults to 10 periods, with a minimum timeout of
500 ms. `--heartbeat-startup-grace-ms MS` defaults to 5000 ms to allow node
initialization. Stalled nodes are killed and enter the same restart policy.
`--restart-window-ms MS` overrides the default 60000 ms rolling budget window.

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

The launcher starts the node's binary with `--sim --replay-inputs <dir>
--log <output_dir>/<node>.tlog --config <config>`, the flag every node binary
takes (C++ and Python alike: both run `talos::Run`). The node builds as it
would live, then runs on a simulated loop whose clock starts at the session's
origin; every message the session logged on a topic it subscribes to arrives
at its recorded time (`talOS/events/README.md`, "Replaying another session's
inputs"). It prints what it was fed, from whose log, and the topics it
subscribes to that nothing recorded. A topic recorded with another schema
fails before the node runs, naming both types. The node may be one the
session never ran, or a changed version of one that it did: its new outputs
are its own log, not compared with anything, and two replays of one session
write identical records. Replay feeds every input from the session origin,
including messages the live node missed while initializing at startup, so its
output can legitimately differ from the live log at the beginning of the run.
Pass `--session-id <id>` if you need the output files to match in their
`session_id` header field (otherwise only the random session id and host
wall-clock start time in the 120-byte file header differ; all records and
payloads are bit-identical). The output directory must not be the session's,
whose logs are being read.

## graph.json

Phase one writes `<output_dir>/<session_id>/graph.json` beside the session's
`manifest.json`: the session id, the config it came from, every node with its
name, target, whether it answered, and each source's kind, name, message size
and endpoint attributes, followed by every diagnostic. Studio's bridge serves it
so a viewer can show the declared graph next to the live one and mark the
difference.

`uint64` values are written as decimal strings, which is the rule everywhere in
this repo that emits JSON. A JSON number is a double, so a session id above 2⁵³
comes back a different number, and an id that changes is not an id.

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

`:launcher_lib` depends on `//talOS/introspection:describe` and `:names` — the
format and the naming rules.

## Session segments and logs

`--output-dir DIR` always names a root: each launch writes to
`DIR/<session_id>/`, including deployed `TALOS_LOG_DIR` roots. Without the flag,
the root is `/tmp/talos_logs`. Use persistent storage on a deployed robot.
`manifest.json` is written before nodes start, flushed and synced, then replaced
atomically at each process lifecycle change; an interrupted session retains its
last complete manifest (`end_wall_ns = 0` means no normal shutdown).

At startup, retention removes the oldest recognized sessions (directories with
a `manifest.json`) until `[robot] log_keep_sessions` (default 50, including the
current session) and `log_max_bytes` (default 2 GiB) are satisfied. Both must be
positive. The current session, symlinks, and unrelated directories are never
deleted. The current session can itself grow beyond the budget; retention runs
on the next launch. Event writers sync approximately once a second and at close.

Simulation is only permitted with effective remote `127.0.0.1`. `--sim` with a
different configured or overridden remote is refused before any process starts.


Before spawning nodes, phase two calls `CreateSessionSegments` (`launcher.h`),
which creates every declared topic as a session-owned RTMS segment with its
schema name, schema hash, alignment, and slots. Nodes only attach. When the
session ends, the launcher destroys its queues and unlinks all session segments.

Recorded logs (`.tlog` format version 5) embed descriptors for each logged topic.
`log_dump` (`//talOS/launcher:log_dump`) reads these descriptors and decodes
recorded message payloads into named fields without requiring the original build
environment.

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

## If you change the shape of `graph.json`

Studio parses this document in TypeScript (`studio/src/system.ts`), and neither
language can check the other. `studio/tests/launcher_graph.json` and
`launcher_graph_faults.json` are real output from

    bazel run //talOS/launcher:launcher -- --describe-only \
      --config 2026-robot/main_processor/configuration/robot.toml

with only `session_id` pinned, and `//studio:declared_test` parses both. So a
change to the envelope here fails there rather than reaching a viewer as a
silently empty panel. Regenerate the fixtures and fix the parser in the same
commit; the faults fixture is the same robot with the historical
`/hw/req/drive` misspelling put back, which is where the diagnostic wording is
pinned.

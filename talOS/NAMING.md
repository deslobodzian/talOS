# Naming protocol

This is normative. `talOS/introspection/names.h` is the same rules as code, and
where the two disagree the header wins — the launcher and the test suite read
the header, and nothing reads this file.

## Why there is a protocol at all

A talOS topic is a POSIX shared-memory object addressed by exact string. There
is no name service to resolve against, no type check at the transport, and no
error for a name nobody else uses. So two spellings of one idea do not fail:
they produce a publisher and a subscriber that are both individually correct,
both report healthy, and never meet. The robot then does nothing, or does
something with stale data, and the only evidence is a topic with traffic on one
end.

This has happened here repeatedly:

- `/hw/req/drive` against `/hw/req/drivetrain`. The drivetrain package spelled
  its request topic one way; the hardware bridge derived the other from the
  subsystem name in `robot.toml`. It was papered over with a second subscriber
  and an `if (name == "drivetrain")` in the bridge, which meant the bug was
  permanent and invisible rather than fixed.
- `/hw/cmd` against `/hw/cmd_in`. Two topics that read as one concept with a
  direction, so the graph reported a command topic nobody consumed.
- `/odometry` against everything else's `/<owner>/state`. A different shape for
  no reason, so there was no rule left to apply to the next topic.

None of these was caught by a compiler, a test, or review, because a
convention that lives only in a document is not enforced by anything. So the
convention lives in a header, the launcher checks the whole graph before it
spawns a single process, and `//talOS/introspection:names_test` checks it on
every build.

## Topics

```
topic      = "/" owner "/" role [ "/" qualifier [ "/" qualifier ] ]
owner      = node-name | reserved-namespace
role       = "state" | "target" | "command" | "request"
           | "status" | "event"  | "telemetry"
qualifier  = segment
segment    = lowercase-letter *( lowercase-letter / digit / "_" )
```

- Two to four segments, at most 24 characters each.
- Lowercase, digits and single underscores. No hyphens, no camelCase, no
  leading digit, no trailing or doubled underscore.
- **The whole name, including the leading slash, is at most 31 characters.**

That last limit is the transport's, and it is tighter than anything else in the
chain. A topic is a POSIX shared-memory object; macOS caps a shm name at
`PSHMNAMLEN`, and `rtms::ValidatePath` enforces 30 characters after the leading
slash and throws otherwise. The event loop would allow 63
(`event::MAX_SOURCE_NAME`), and the first version of `names.h` used that number
— which made the check worse than useless, because it accepted names that pass
review and then throw inside a node's constructor at startup. A validator has
to enforce the tightest limit in the chain, not its own.

The budget is genuinely tight, and it is the one rule here you are likely to
hit by accident. The longest name in the robot today is
`/drivetrain/target/teleop`, at 25 — six characters of headroom.
`/operator_interface/target/teleop` is 33 and does not fit. When a name will
not fit, **shorten the qualifier**, not the owner or the role: those two are
what make the graph readable, and a qualifier only has to be unique among its
siblings.

### Segment 1 — owner

The **subject**: the node the topic is *about*, spelled exactly as that node's
name (see below), or one of the reserved namespaces:

| namespace | meaning |
|---|---|
| `hw` | the hardware bridge's namespace: the shared-memory side of the controller processor, which is not a talOS node |
| `talos` | framework feeds owned by no subsystem |
| `sim` | simulation-only topics, never present on a real robot |
| `replay` | topics fed from a log rather than from a peer |
| `test` | tests only. The launcher refuses to spawn a node that uses one |

"The owner is the node that publishes it" is the obvious rule, and it is
wrong. The arbiter publishes `/drivetrain/target` and the drivetrain consumes
it — that is the whole point of arbitration — so a rule that made the publisher
the owner would forbid the architecture this repo actually has. It was the
first draft of this protocol, and `names_test` rejected it.

The owner is the subject, and the **role** says which way the data moves
relative to it. That is what is checked:

| roles | direction | rule |
|---|---|---|
| `state`, `status`, `event`, `telemetry` | out of the owner | the owner **must** be the publisher. A node reporting another node's state is either misnamed or reaching into something that is not its business |
| `target`, `request`, `command` | into the owner | the owner must **not** be the publisher. A node publishing its own target is talking to itself |

Reserved namespaces are exempt from both, because their subject is not a talOS
node: the bridge publishes `/hw/state` on behalf of a controller processor, and
neither direction applies.

A third check is a warning rather than an error: an owner segment that names no
node in the session and is not reserved. That is the shape a typo takes —
`/drivetraim/state`, or a name left behind by a half-finished rename — but a
partial launch, with one subsystem disabled in `robot.toml`, is a legitimate
session in which the owner really is absent.

All of this is checked on the writing end only. A subscriber has no claim on
the name of what it reads.

### Segment 2 — role

A closed vocabulary, so that a reader who has never seen a topic before still
knows what is on it and which way it flows:

| role | direction | meaning |
|---|---|---|
| `state` | out | what a thing is now, published by the thing itself |
| `status` | out | health and liveness about a thing, not its physics |
| `event` | out | something that happened once, not a level |
| `telemetry` | out | an observation feed, for recording and display only |
| `target` | in | what a thing is being asked to become |
| `request` | in | an actuator-level instruction awaiting arbitration |
| `command` | in | an actuator-level instruction, already resolved |

"Direction" is relative to the owner, and it is what the owner rule below is
checked against. Every role must appear in `kOwnerPublishedRoles` or
`kOwnerConsumedRoles` in `names.h` — a static assertion enforces that, so a new
role cannot silently opt itself out of the check.

Closed is the point. A vocabulary anyone may extend at a call site cannot make
that promise, and the moment it has twenty entries it is not a vocabulary.
Adding a role is a deliberate edit to `kRoles` in `names.h` and to this table,
in one commit, with a reason.

### Segments 3–4 — qualifier

Free-form, same character rules. A qualifier says *which* — which producer,
which subsystem, which subject:

```
/drivetrain/target/teleop      which producer  (arbitration)
/hw/request/drivetrain         which subsystem
/hw/state/driver_station       which subject
```

### Prefix families

`/drivetrain/target` may exist beside `/drivetrain/target/teleop`. This is the
sanctioned arbitration shape: producers publish to their own qualified topic,
one arbiter reads them all and publishes the winner to the unqualified topic,
and the actuator subscribes only to that. Nothing does prefix matching — RTMS
lookups are exact — so the shape is unambiguous.

No further rule applies to a family, and there is deliberately no check for
one. The owner *is* the first segment, so a family shares its owner by
construction; a lint rule saying so could never fail, and a check that cannot
fire is worse than no check because it implies coverage that does not exist.

### No abbreviations

`tgt`, `cmd`, `req`, `ds`, `odom`, `pos`, `vel`, `cfg`, `pkt`, `msg`, and the
rest in `kAbbreviations` are rejected, with the long form named in the error.

The cost of an abbreviation is not brevity. It is that the long form is still
available to whoever writes the other end of the topic, and there is nothing to
stop them using it. `drive` is on the list for exactly that reason: it cost a
special case in the bridge.

### No direction suffixes

`_in`, `_out`, `_pub`, `_sub`, `_tx`, `_rx`, `_send`, `_recv`, `_topic` are
rejected.

Direction is a property of an endpoint, not of a topic. The same topic is
outbound to its publisher and inbound to every subscriber, so a name that picks
one side is wrong from the other — and naming one side invites a second topic
for the other direction, when what was wanted was one topic with two ends.

### One writer

Exactly one node may publish a given topic. This is not a style preference: the
event loop records a dispatch log and replay refuses a differently-shaped
manifest, so two producers racing on one topic would resolve differently on
replay than they did on the field. The linter reports two writers as an error
and names both.

Fan-in is spelled with an arbiter and a prefix family, as above.

### Endpoint attributes

Some ends are *supposed* to have no peer, and a graph that cannot say so reports
designs as faults — which trains people to ignore the report, a worse outcome
than not having one. Two flags, declared by the owning node and carried in
`SourceRecord::flags`:

| flag | meaning | example |
|---|---|---|
| `kSourceFlagExternal` | the far end is outside talOS and a shared-memory peer will never exist | `/hw/command`, consumed by the RoboRIO over UDP |
| `kSourceFlagOptional` | this end may legitimately have no peer yet | `/drivetrain/target/auto`, because autonomous is not written |

Use them sparingly and for the stated reason. A flag on a topic that really is
misconnected hides exactly what this protocol exists to surface.

### Declaring a topic

Each topic is declared **once**, in the owning package's header, as

```cpp
inline constexpr const char* kTargetTopic = "/drivetrain/target";
```

Consumers include that header and alias the constant; `arbiter/node.h` is the
worked example. A bare topic string literal at a call site is a defect even
when it is spelled correctly, because it is a second place for the name to live
and the linter cannot see it before it runs.

## Nodes

A node's name is one string used in five places, and they must all be that
string:

1. the `[subsystems.<name>]` key in `robot.toml`
2. the `name` passed to `introspect::Reporter`
3. the row the registry shows and the Studio System tab labels
4. the log file stem, `<output_dir>/<name>.tlog`
5. the owner segment of every topic the node is the *subject* of — the ones it
   publishes state on, and the ones addressed to it

Character rules are a segment's, and a node may not be named after a reserved
namespace.

### Names and build targets

A robot's node is built by `//<robot>/<processor>/<name>:node`, so the package's
last path segment carries the name. A framework node has no subsystem package,
so there the name matches the rule name after the colon —
`//talOS/bridge:hardware_node` is the node `hardware_node`.

`CheckNodeTarget` accepts either, and one of the two must hold. The reason is
traceability: the launcher resolves a node from the config and a viewer reports
it by name, and a name that no build target produces cannot be traced back to
source.

## Other names

- **A topic name is not the shared-memory name.** POSIX allows exactly one
  slash in an shm object name, at the front; glibc returns `EINVAL` for an
  interior one, while macOS's flat namespace accepts it. Every name in this
  protocol has two or three segments, so `ShmObjectName`
  (`talOS/memory/shared_memory_ptr.h`) maps the interior slashes to `.` at the
  one boundary that opens a segment — `/hw/state/driver_station` becomes
  `/hw.state.driver_station`. The separator is `.` and not `_` because the
  mapping has to be injective: segments may contain underscores, so `_` would
  make `/hw/state/driver_station` and `/hw/state_driver/station` one segment,
  silently joining two unrelated streams. The grammar allows no `.` inside a
  segment, so `.` cannot collide.

  Anything that opens a segment directly — a stale-segment reclaim, a test, a
  tool listing `/dev/shm` — must call `ShmObjectName` and not use the topic.
  Skipping it is a bug that hides on macOS, where the unmapped name happens to
  be legal: the bridge's own publisher pre-check did exactly this and reported a
  healthy publisher as missing.
- **Shared-memory segments** that are not topics carry their layout version in
  the path: `/talos_registry.1`. The version is in the *name* so a build whose
  records have a different shape lands in a different segment instead of
  misreading someone else's. Bump it whenever a record changes shape.
- **Log files**: `<output_dir>/<node-name>.tlog`, one per node, plus
  `manifest.json` and `graph.json` for the session.
- **Message types** are `PascalCase` C++/FlatBuffers types in the owning
  package's namespace (`talos::drive::ChassisTarget`); fields are
  `snake_case`, and a field carrying a unit says so in the name
  (`max_velocity_rps`, `wheel_radius_m`, `period_us`). A unit in the name is
  the only place a unit can be checked at all, since the wire carries a number.
- **Topic constants** are `k<Role>Topic`, qualified when needed:
  `kTargetTopic`, `kTeleopTargetTopic`, `kDriveRequestTopic`.

## How this is enforced

| when | what |
|---|---|
| every build | `//talOS/introspection:names_test` checks the grammar and the graph rules, including against this robot's real declared graph |
| every launch | the launcher probes each binary with `--describe`, assembles the declared graph, runs `naming::LintGraph`, and **refuses to spawn** on an error. `--allow-graph-errors` overrides it, because a robot at a competition must be able to run past a lint failure — but refusal is the default, since the failure this catches is silent at runtime |
| every run | `NodeRegistration::publish` warns on stderr for a malformed name. A warning, never a throw: a node that refused to run because a topic is misnamed would be a worse failure than the misnamed topic |
| in a viewer | Studio's System tab classifies every topic end, and the Overview lists the ones that need attention |

## Changing a name

Renaming a topic changes every manifest that mentions it, and replay compares
manifests, so **existing recordings will refuse to replay against the renamed
build**. That is the correct behaviour — the alternative is silently misrouting
every record — but it means a rename is a real cost, paid at the moment you do
it. Rename early, and rename both ends in one commit.

The mechanical part:

1. Change the constant's value in the owning package's header. Do not change
   the C++ identifier at the same time; that turns one reviewable diff into two.
2. `grep -rn '<old-name>' --include='*.cc' --include='*.h' --include='*.ts' --include='*.tsx' --include='*.py' --include='*.toml' .`
   Topic names reach TypeScript, Python integration tests, and `robot.toml`
   comments, and none of those break at compile time.
3. `bazel run //talOS/launcher:launcher -- --describe-only` and read the
   diagnostics before anything runs.
4. `bazel test //...`.

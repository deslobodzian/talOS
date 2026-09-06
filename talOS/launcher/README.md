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

**Phase two starts.** Only then does the launcher fork the roster, wait for it,
read back the dispatch logs and write the session manifest, exactly as it did
before.

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
message sizes; a subscriber whose topic nothing publishes. A warning is a state
that is normal while a robot is being built, such as a feed nobody has
subscribed to yet.

Two things keep that distinction honest, and both of them matter more than they
look:

- A node declares endpoint attributes for the ends whose far side is not a node
  in this session. The arbiter marks `/drivetrain/target/auto` and
  `/shooter/target/auto` optional, because autonomous is not written and those
  subscriptions are deliberately unfed; the hardware bridge marks `/hw/command`
  external, because the RoboRIO consumes it over UDP and no shared-memory
  subscriber will ever appear. Without these the report would flag a known
  design decision on every single launch, and a report that always complains is
  a report nobody reads — which costs exactly the finding it was built to
  deliver.

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

The probe has a short timeout — five seconds by default,
`--describe-timeout-ms` to change it — after which the probe is killed and
reported. A node that blocks in its constructor waiting for a device is a bug
worth seeing, and it must cost the launch a few seconds rather than the session.

## graph.json

Phase one writes `<output_dir>/graph.json` beside the session's
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
bazel run //talOS/launcher:launcher -- --describe-only
```

The launcher resolves node targets to `bazel-bin/...` paths and reads a
workspace-relative config, so it only makes sense with the workspace as its
working directory; `bazel run` starts a binary in its runfiles tree instead, and
`main.cc` chases `BUILD_WORKSPACE_DIRECTORY` back so both spellings behave the
same.

`:launcher_lib` depends on `//talOS/introspection:describe` and `:names` — the
format and the naming rules, neither of which pulls in an event loop or shared
memory. The launcher never opens a topic; it starts processes that do.

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

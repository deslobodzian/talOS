# Live node registry

`registry.h` exposes a shared-memory registry of node identities, endpoints,
heartbeats, source counters and Target selections. `RegistryReader` maps it
read-only; `Reporter` publishes a node's event-loop data.

## The segment

The logical path is `/talos_registry.3`, and the trailing number is
`kRegistryVersion`. Registry opens and explicit unlinks apply `ShmObjectName`,
just like topics: `TALOS_SHM_NAMESPACE` (or Bazel's `TEST_TMPDIR` fallback)
isolates tests and their children, while production retains the ordinary name.
The header validates version and record sizes before a reader proceeds.
Heartbeats and stale-slot checks use host monotonic time; `start_wall_ns` is
human-facing wall time.

Openers serialize segment creation, sizing and initialization with an exclusive
`flock` on a regular file, `/tmp/talos-registry-<mapped-name>.lock` (the mapped
name without its leading slash). The name is the exact `ShmObjectName` result
used for the segment, so `TALOS_SHM_NAMESPACE` and `TEST_TMPDIR` isolate the lock
and the segment together; no second namespace or temp-directory rule applies.
The lock is opened independently for each acquisition, serializing threads as
well as processes, and is close-on-exec. The kernel releases it on close or
process death, including SIGKILL. A normal file works on Linux and macOS:
[Darwin's `flock` requires a vnode](https://github.com/apple-oss-distributions/xnu/blob/main/bsd/kern/kern_descrip.c),
but [its shared-memory descriptors are `DTYPE_PSXSHM`](https://github.com/apple-oss-distributions/xnu/blob/main/bsd/kern/posix_shm.c),
not vnodes; locking the shm descriptor itself is not portable.

The writer resets any uncommitted segment under this lock, fills the header,
then release-stores the magic word as the final commit. An initializer killed
at any earlier instruction leaves an uncommitted segment: the next writer
reinitializes it, without a timeout or manual `shm_unlink`. A committed but
incompatible layout is still rejected, never reset. A read-only opener takes
the startup lock and validates the commit; it cannot repair an incomplete
segment and reports that a writer must initialize it. Polling an open reader
never locks. Lock files deliberately persist, even across explicit segment
unlinks: unlinking a lock file could split waiters between two inodes.


`RegistryMapping` owns its mapping but does not unlink the registry on
destruction. `RegistryMapping::unlink` explicitly removes it.

## Endpoint attributes

`external` marks an endpoint whose far side is not a shared-memory peer;
`optional` permits a missing peer. These are per-endpoint attributes, not
per-topic attributes. Graph lint rules are documented in
[NAMING.md](../NAMING.md).

The declaration comes from the node rather than the event loop, because the loop
cannot know: a sender whose consumer is off-box is registered exactly like a
sender whose consumer is the node next door. `Reporter::Options::endpoints`
takes them keyed by topic name for the loop's own sources, and
`ExtraSource::flags` carries one inline for a topic the node owns outside the
loop. The same attributes appear in `--describe` output, so the declared graph
the launcher checks and the live graph a viewer shows agree about which
dead ends are designed.


`publish` checks node and topic names against `names.h` and prints findings to
stderr without refusing publication. Timer labels are not checked as topics.

## Who drives a subsystem

Each `SelectionRecord` describes one Target's policy source names and its
selected index, or -1 for none. `SelectionSnapshot::selected_source()` returns
the name or `nullopt`.

The hot path never touches the segment. `get()` stores the index into the
node's own atomic (`NodeAccess::set_selected`: one relaxed load, and a store
only when the answer changed), and the `Reporter` copies it into the slot
inside each refresh sample through `Reporter::Options::selections`. A slot
holds `kMaxSelectionsPerNode` Targets and `kMaxSelectionSources` names each;
the declared counts say when a node or a policy had more, and a selected source
past the name table reads as `#<index>` rather than as nobody.


## Claiming a slot, and liveness

A slot is FREE, CLAIMING or ACTIVE. `NodeRegistration` holds the same kernel
lock from the stale-slot sweep through the claim's ACTIVE commit. It takes a
FREE slot and writes identity, counts, heartbeat and generation before
release-storing ACTIVE. A claimant killed at any instruction before this
commit cannot keep the lock held. Any CLAIMING slot observed by the next sweep
under the lock is therefore abandoned, and is freed immediately, even if the
claimant died before recording a pid or heartbeat. This also recovers a sweeper
killed during its own temporary CLAIMING transition. Recovery takes one sweep
(a fixed scan of 32 slots), not 32 restarts or a heartbeat timeout.

Every new claim also sweeps ACTIVE slots whose host-monotonic heartbeat is at
least `kLivenessTimeout`, five seconds old. The heartbeat is rechecked after
the ACTIVE-to-CLAIMING transition so a refresh observed during the sweep can
keep the row.

Each claim bumps a generation counter before the slot goes ACTIVE. A reader
snapshots the generation, copies the identity fields, and re-reads it; a change
means the slot was recycled mid-copy and the read is retried.
The source records are published the other way round: they are written first and
the count is stored with release ordering afterwards, so a reader that can see a
count can see the records it describes.

A registration releases its slot in its destructor, so a clean shutdown leaves
no ghost row and a viewer sees the node disappear immediately. A node that is
killed leaves its slot ACTIVE until the heartbeat goes stale.

## Observing must not perturb

Writers only ever touch their own slot, which is padded to a cache line so two
nodes updating counters do not contend on the same line. Every field a reader
observes while a node runs is a lock-free atomic — there are static assertions
to that effect, because a registry that fell back to a lock would be a registry
that could block a control loop. `RegistryReader` writes nothing and locks only
while opening, never while polling, so it is safe to poll at any rate.
`RegistryReader::open` returns `nullopt` instead of throwing when the segment
does not exist: no nodes running is an ordinary viewer startup state.

The reporter claims its slot synchronously before user construction, setting
`kFlagConstructing` in the existing atomic flags field (no layout change).
`SlotState::ACTIVE` means the slot is owned; the lifecycle flag distinguishes
construction from dispatching. A bookkeeping thread heartbeats identity only.
`RegistryReader` includes these rows with `alive = false`, so Studio can show
initialization without treating it as an active node.

After construction, `Reporter::start(loop, ...)` waits for `loop.running()`
while continuing identity heartbeats. The loop allocates its counters array
before storing `running_`, synchronizing the reporter's load. The reporter
publishes endpoints and clears the constructing flag only when the manifest
and counters exist. The launcher bounds construction from process spawn;
see [launcher supervision](../launcher/README.md#restarts).
The reporter samples full source counters and selections every 250 ms and
heartbeat/dispatch progress every 10 ms. Full counters can lag the progress
pair until the next full sample.
Short sleeps also let a node which exits leave the viewer promptly; a final
full sample preserves its last counters.

It must be constructed after the loop so that reverse destruction order joins
the thread before the counters it samples are destroyed. `talos::Run`
(`talOS/node`) does this for every node.

A registry that cannot be opened is printed to stderr once and then ignored, and
`Reporter::error()` keeps the message. A node that refused to run because a
telemetry viewer might not see it would be a worse failure than not being seen.

## The counters

`EventLoopBase` keeps a `SourceCounters` array, one entry per registered
source, allocated when registration closes and never reallocated — so the
reporter can hold the pointer for the life of the run. Each entry tracks events,
dropped messages, the last transport sequence, the last dispatch time, the last
dispatch latency and the worst latency seen.

"Events" means whatever that source doing its job looks like: dispatches for a
timer or a watcher, and messages accepted by the transport for a sender.
Publishes the transport refused count as dropped, alongside messages a writer
lapped before a subscriber read them and the realtime loop's failed reads (a
read the writer tore, or one that found the subscriber's reader slot freed).

These are atomics so another thread can read them mid-run without a data race.

## Limits

The capacities are fixed at 32 nodes and 64 sources each. A node that registers
more sources than fit still reports the true count in
`declared_source_count`, so the truncation is visible rather than silent, but
the extra sources are not described. A 33rd node is refused with an error rather
than displacing a live one.

`HardwareNode::BuildIntrospectionManifest` supplies the hardware bridge's
manifest explicitly and its run loop refreshes the registry.

Refreshes are bracketed by a sample sequence. Readers retry inconsistent
copies a bounded number of times rather than waiting for a stalled writer;
these snapshots are observational, not control inputs.

The mapping is requested with mode `0666` (subject to the process umask).
It provides no authentication.

Naming rules and whole-graph linting live in [NAMING.md](../NAMING.md).

## Building

```sh
bazel test //talOS/introspection/...
```

`registry_test` uses forked children stopped at startup-only injection hooks,
then SIGKILLed before the initialization or ACTIVE commit. `waitpid(WUNTRACED)`
synchronizes the kill without sleeps. The tests verify initialization recovery,
all 32 slots remaining usable after an abandoned claim, and eight concurrent
processes holding distinct registrations across the entire registry.

`:registry` is the segment and both ends of it, and depends only on
`//talOS/events:events_core` and `:names` — a reader such as Studio's bridge
does not pull in an event loop. It does not depend on `:describe` either, and
`publish`'s attribute overload is a template on the container for that reason:
the record of what is running should not have to include the description of what
would run. `:reporter` adds the sampling thread and is needed only by processes
that publish.

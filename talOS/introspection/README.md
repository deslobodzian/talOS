# Live node registry

RTMS topics are POSIX shared-memory objects. There is no portable way to list
them, and a process holding one open is invisible to everyone else, so nothing
can discover the shape of a running robot by looking at the transport. Asking
"what is running, and who publishes what" from the outside is not a query that
has an answer — the nodes have to say so.

This is where they say it. One shared-memory segment holds a fixed array of
slots. A node claims a slot at startup, writes its event-loop manifest into it —
every timer, watcher, fetcher and sender, by name, kind and message size — and
then keeps a heartbeat, a set of counters and the lane each of its Targets
selected current for as long as it runs. Studio's bridge and any agent map the
segment read-only and get the whole system: which processes exist, which topics
each one is on and at which end, how much traffic has actually moved, and who
drives each subsystem.

## The segment

The logical path is `/talos_registry.3`, and the trailing number is
`kRegistryVersion`. Registry opens and explicit unlinks apply `ShmObjectName`,
just like topics: `TALOS_SHM_NAMESPACE` (or Bazel's `TEST_TMPDIR` fallback)
isolates tests and their children, while production retains the ordinary name.
The version is part of the name on purpose. The records below are a frozen
binary layout shared between processes that were compiled separately, so a build
whose records have a different shape must not be able to read a segment written
by another one. Putting the version in the name means it does not try: it lands
in a different segment. `RegistryHeader` also carries `node_record_bytes` and
`source_record_bytes`, and a reader that attaches to a segment whose sizes
disagree with its own reports that rather than proceeding, which catches a
layout change that someone forgot to bump the version for.

Version 3 changes heartbeat timestamps to host monotonic nanoseconds. Heartbeat
publication, stale-slot sweeping and reader liveness all use `steady_clock`,
whose epoch is shared across processes on the same host. NTP and manual wall
clock changes cannot evict healthy nodes. `start_wall_ns` remains human-facing
wall time; the Studio bridge converts heartbeat age back into a display wall
timestamp at serialization. Version-2 binaries must be rebuilt together: the
old wall-time epoch is deliberately isolated in its old segment.

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

The record layout, registry name/version and monotonic heartbeat epoch are
unchanged. All writers sharing a registry must use this locking protocol;
rebuild them together rather than mixing old lock-free claimers with new ones.

`RegistryMapping` does its own `shm_open` and `mmap` rather than using
`SharedMemoryPtr`, because `SharedMemoryPtr` unlinks on destruction. That is
right for a topic owned by one publisher and wrong here: the registry outlives
any single node. Nothing unlinks it. A segment left behind by a previous run is
harmless, because every slot in it is either FREE or held by a node that stopped
heartbeating, and both are reclaimable. `RegistryMapping::unlink` exists for
tests, which want a clean segment per case.

## Endpoint attributes

A topic with a publisher and no subscriber, or a subscriber and no publisher, is
usually a bug — and it is the specific bug this whole directory exists to catch,
because two spellings of one name produce exactly that shape. But it is not
always a bug, and the cases where it is not are permanent.

`/hw/command` is published by the hardware bridge and consumed by the RoboRIO,
which reads it over UDP. It will never have a shared-memory subscriber, and
there is no version of this robot in which it would. `/drivetrain/target/auto`
is subscribed to by the drivetrain's `talos::Target` and published by nobody,
because autonomous is not written yet; it will be, and until then the topic is
deliberately half-connected (the lint mentions it as a warning, never as an
error).

Reported as faults, those two are noise on every run, and noise on every run is
how a report stops being read. The next real dangling topic then arrives into a
list that already has two entries everyone has learned to scroll past, which
leaves the tool worse than useless — it looks like it is working. So the node
that owns the endpoint declares what it knows: `external` means the far end is
not a shared-memory peer and should not be looked for, `optional` means the far
end is expected eventually but its absence is not a fault today. Both are per
endpoint, not per topic, because only one end of a topic knows.

The declaration comes from the node rather than the event loop, because the loop
cannot know: a sender whose consumer is off-box is registered exactly like a
sender whose consumer is the node next door. `Reporter::Options::endpoints`
takes them keyed by topic name for the loop's own sources, and
`ExtraSource::flags` carries one inline for a topic the node owns outside the
loop. The same attributes appear in `--describe` output, so the declared graph
the launcher checks and the live graph a viewer shows agree about which
dead ends are designed.

They ride in `SourceRecord::flags`, and the segment version stayed at 1. That
word was `reserved`, and every build that has ever written this layout wrote it
as zero: no field moves, nothing changes size, and a writer that predates
attributes stores a 0 that reads as "declared none" — which is the right answer
for a node that declared none. Both mixed cases are therefore correct, and
bumping would have cost real availability, because the version is in the segment
name: every node not yet rebuilt would land in a different segment and vanish
from the viewer. A version bump protects against a layout a reader would
misinterpret, and this is not one.

`publish` also checks the node's own name and each of its topic names against
the naming rules in `names.h`, once, and prints what it finds to stderr. A
warning, never a refusal. A node that would not start because one of its topics
is misspelled takes a subsystem off the robot to fix a report, which is a worse
failure than the misspelling — and the launcher already checks the same rules
before it spawns anything, so it can refuse where refusing is cheap. What this
adds is that a node started by hand still says so out loud, because a
misspelling nobody is ever told about is how the two-spelling bug survives. A
timer's name is skipped: it is a label, not an address, and nothing subscribes
to it.

## Who drives a subsystem

A subsystem's `talos::Target` (`talOS/devices/target.h`) picks, on every
`get()`, the first live lane of its policy — `teleop`, `auto`, `align` — or
none. No manifest entry or counter says which one won, and "who is driving the
drivetrain right now" is the first question asked of a robot that moves when it
should not, or does not when it should. So each slot also holds one
`SelectionRecord` per Target the node opened, in opening order: the policy's
source names (`TargetPolicy::Sources()` order), written once and published with
a release-ordered `selection_count` the way the sources are, and a
`std::atomic<int32_t> selected` — the index of the selected lane, or -1 for
none. `SelectionSnapshot::selected_source()` turns that back into the name, or
`nullopt` when nobody drives.

The hot path never touches the segment. `get()` stores the index into the
node's own atomic (`NodeAccess::set_selected`: one relaxed load, and a store
only when the answer changed), and the `Reporter` copies it into the slot
inside each refresh sample through `Reporter::Options::selections`. A slot
holds `kMaxSelectionsPerNode` Targets and `kMaxSelectionSources` names each;
the declared counts say when a node or a policy had more, and a selected source
past the name table reads as `#<index>` rather than as nobody.

This one did bump the version, to 2. There was no reserved word left to reuse,
and a name table per Target would not have fit in one: the records sit at the
end of `NodeRecord`, which changes its size and the offset of every slot after
the first. That is a layout an older reader would misinterpret — the header's
record sizes would make it refuse the segment outright — so the two builds now
use different segments, and a viewer not yet rebuilt sees no nodes rather than
an error.

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
keep the row. Five seconds is deliberately generous: a process stopped in a
debugger is not a dead process, and the cost of waiting is one stale viewer row.

Each claim bumps a generation counter before the slot goes ACTIVE. A reader
snapshots the generation, copies the identity fields, and re-reads it; a change
means the slot was recycled mid-copy and the read is retried. Two attempts is
enough, because claiming happens once per process start and not once per read.
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

## Why the reporter is a thread and not an event source

`Reporter` runs on its own `std::jthread`. The obvious alternative — register a
timer on the node's event loop and publish from its handler — is wrong, and
wrong in a way that matters more than the convenience.

Registering a timer puts introspection into the loop's manifest. The manifest is
the log's description of the program, and replay compares the two and refuses to
run on a mismatch, so every recording made with introspection on would refuse to
replay against a build with it off, and vice versa. Worse, ids are handed out in
registration order, so an extra source shifts the id of everything registered
after it and invalidates existing logs outright. Observability must not change
the program being observed.

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
and counters exist. The launcher bounds this entire phase to 30 seconds from
spawn, even if heartbeats keep arriving; see its supervision documentation.
It then publishes the manifest once and samples source counters and selections
every 250 ms. Between full samples it updates only heartbeat and dispatch
progress every 10 ms, bracketed together by the sample sequence. The full
display refresh cadence must not consume the launcher's unchanged 500 ms
minimum stall window: previously a 300 ms process pause plus an already
200–250 ms-old sample could trigger a restart before `SIGCONT`. With the new
progress cadence, a pause plus the 10 ms reporting interval below that floor
does not itself trigger a stall restart (subject to host scheduling delay).
Full source counters can lag the progress pair until the next full sample.
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
lapped before a subscriber read them.

These are atomics so another thread can read them mid-run without a data race.
The cost is six relaxed stores per dispatch and no clock reads beyond the ones
the dispatch had already made, which is cheap enough to leave on in a 1 kHz
loop.

## Limits

The capacities are fixed at 32 nodes and 64 sources each. A node that registers
more sources than fit still reports the true count in
`declared_source_count`, so the truncation is visible rather than silent, but
the extra sources are not described. A 33rd node is refused with an error rather
than displacing a live one.

The hardware node predates the event loop and drives its own tick, so it has no
manifest to hand over and declares one by hand in
`HardwareNode::BuildIntrospectionManifest`, in the same order `Open()` creates
the transports, and refreshes it from its run loop. Its per-topic counts come
from the counters it already kept, and its latency fields are zero, because it
does not measure them.

Counters are relaxed atomics written one source at a time, so a reader can
observe a set that is slightly inconsistent across sources within one node — two
counters a few microseconds apart, never a torn value. That is the right trade
for traffic counters and would not be for anything a control decision depended
on.

Nothing here is authenticated or access-controlled. The segment is world-
readable and world-writable, on the assumption already made everywhere else in
this tree: the robot and the development machine are a trusted local network.

## Naming rules

`names.h` is the topic grammar (`/owner/role[/qualifier[/qualifier]]`, 31
bytes, lowercase segments, no abbreviations or direction suffixes), the closed
role vocabulary and which side of its owner each role flows, the node-name and
node-target checks (a target is a main-repository label, `//package:rule`, whose
package leaf or rule is the node's name), and `LintGraph`, the whole-graph check
the launcher runs before it spawns anything. NAMING.md is the prose.

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

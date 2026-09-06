# Live node registry

RTMS topics are POSIX shared-memory objects. There is no portable way to list
them, and a process holding one open is invisible to everyone else, so nothing
can discover the shape of a running robot by looking at the transport. Asking
"what is running, and who publishes what" from the outside is not a query that
has an answer — the nodes have to say so.

This is where they say it. One shared-memory segment holds a fixed array of
slots. A node claims a slot at startup, writes its event-loop manifest into it —
every timer, watcher, fetcher and sender, by name, kind and message size — and
then keeps a heartbeat and a set of counters current for as long as it runs.
Studio's bridge and any agent map the segment read-only and get the whole
system: which processes exist, which topics each one is on and at which end, and
how much traffic has actually moved.

## The segment

The path is `/talos_registry.1`, and the trailing number is `kRegistryVersion`.
The version is part of the name on purpose. The records below are a frozen
binary layout shared between processes that were compiled separately, so a build
whose records have a different shape must not be able to read a segment written
by another one. Putting the version in the name means it does not try: it lands
in a different segment. `RegistryHeader` also carries `node_record_bytes` and
`source_record_bytes`, and a reader that attaches to a segment whose sizes
disagree with its own reports that rather than proceeding, which catches a
layout change that someone forgot to bump the version for.

A fresh segment is zero-filled, which is what makes initialization safe without
a lock: the first process to attach wins a compare-exchange on the header's
magic word from 0 to `kInitializing`, fills in the capacities, and then stores
the real magic. Everyone else spins on the magic for a bounded 50 ms and fails
with a message naming the segment if it never appears — a process that died
between the two stores would otherwise hang every node that followed it.

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

`/hw/command` is published by the arbiter and consumed by the RoboRIO, which
reads it over UDP through the hardware bridge. It will never have a
shared-memory subscriber, and there is no version of this robot in which it
would. `/drivetrain/target/auto` is subscribed to by the drivetrain and
published by nobody, because autonomous is not written yet; it will be, and
until then the topic is deliberately half-connected.

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

## Claiming a slot, and liveness

A slot is FREE, CLAIMING or ACTIVE, and a node moves it with a compare-exchange
— the same pattern RTMS already uses for reader slots. `NodeRegistration` takes
a FREE slot if there is one. Only when every slot is taken does it consider
stealing one, and then only from a node whose last heartbeat is older than
`kLivenessTimeout`, five seconds.

Five seconds is deliberately generous. A process stopped in a debugger is not a
dead process, and the cost of waiting is one stale row in a viewer, whereas the
cost of being impatient is evicting a node that was about to publish. For the
same reason staleness is checked only on the fallback path: a stale slot is left
alone while any slot is genuinely free, so a registry that is not under pressure
never evicts anyone.

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
that could block a control loop. `RegistryReader` writes nothing and takes
nothing, so it is safe to poll at any rate, and `RegistryReader::open` returns
`nullopt` instead of throwing when the segment does not exist: no nodes running
is an ordinary state for a viewer to start up in, not an error.

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

So the reporter reads and never registers. It waits for `loop.running()`, which
is safe to wait on: the loop allocates its counters array and then stores
`running_`, and that store synchronizes with the reporter's load, so a visible
`true` means both the frozen manifest and the counters exist. It then publishes
the manifest once and samples the counters every 250 ms, well inside the
liveness timeout. It sleeps in short slices rather than one long one so that a
node which exits leaves the viewer promptly, and takes a final sample on the way
out so the last cycle's counts are not lost.

It must be constructed after the loop so that reverse destruction order joins
the thread before the counters it samples are destroyed. Every node main follows
that order:

```cpp
talos::event::RealtimeEventLoop<talos::event::log::LogWriter> loop{...};
talos::odometry::OdometryNode node{loop};
talos::introspect::Reporter reporter{
    loop,
    {.name = "odometry",
     .target = "//2026-robot/main_processor/odometry:node",
     .session_id = session_id,
     .simulation = simulation}};
loop.run();
```

A registry that cannot be opened is printed to stderr once and then ignored, and
`Reporter::error()` keeps the message. A node that refused to run because a
telemetry viewer might not see it would be a worse failure than not being seen.

## The counters

`EventLoopBase` gained a `SourceCounters` array, one entry per registered
source, allocated when registration closes and never reallocated — so the
reporter can hold the pointer for the life of the run. Each entry tracks events,
dropped messages, the last transport sequence, the last dispatch time, the last
dispatch latency and the worst latency seen.

"Events" means whatever that source doing its job looks like: dispatches for a
timer or a watcher, messages accepted by the transport for a sender, and values
actually received for a fetcher. An empty fetch is not an event, or a handler
that samples a topic every cycle would report traffic that never arrived.
Publishes the transport refused count as dropped, alongside messages a writer
lapped before a subscriber read them.

This is not a replacement for `LoopMetrics`. That keeps whole latency
distributions in plain arrays for the owning thread to report at the end of a
run; these are atomics specifically so another thread can read them mid-run
without a data race. The cost is six relaxed stores per dispatch and no clock
reads beyond the ones the dispatch had already made, which is cheap enough to
leave on in a 1 kHz loop — and leaving it on is the point, because observability
that has to be switched on is observability you do not have when you need it.

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

## Building

```sh
bazel test //talOS/introspection:registry_test
```

`:registry` is the segment and both ends of it, and depends only on
`//talOS/events:events_core` and `:names` — a reader such as Studio's bridge
does not pull in an event loop. It does not depend on `:describe` either, and
`publish`'s attribute overload is a template on the container for that reason:
the record of what is running should not have to include the description of what
would run. `:reporter` adds the sampling thread and is needed only by processes
that publish.

# Live node registry

`registry.h` exposes a shared-memory registry of node identities, endpoints,
heartbeats, source counters, Target selections, event log state and wake
loss.
`RegistryReader` maps it read-only; `Reporter` publishes a node's event-loop
data.

## The segment

The logical path is `/talos_registry.6`, and the trailing number is
`kRegistryVersion`. Registry opens and explicit unlinks apply `ShmObjectName`,
just like topics: `TALOS_SHM_NAMESPACE` (or Bazel's `TEST_TMPDIR` fallback)
isolates tests and their children, while production retains the ordinary name.
The header validates version and record sizes before a reader proceeds.
Heartbeats and stale-slot checks use host monotonic time; `start_wall_ns` is
human-facing wall time.

Writers serialize segment creation, sizing and initialization with an exclusive
`flock` on a regular file, `<lock-dir>/talos-<mapped-name>.lock` (`lock_file.h`;
the mapped name without its leading slash). The name is the exact
`ShmObjectName` result used for the segment, so `TALOS_SHM_NAMESPACE` and
`TEST_TMPDIR` isolate the lock and the segment together; no second namespace or
temp-directory rule applies. The directory is the test's `TEST_TMPDIR` under
Bazel's namespace (no `TALOS_SHM_NAMESPACE`), so a test run's lock files go
with it instead of accumulating until reboot; otherwise `/dev/shm` on Linux,
beside the segments, which a systemd `PrivateTmp=` service shares with
everyone else, and `/tmp` on macOS. `/dev/shm` and `/tmp` are sticky and
world-writable, so an existing lock file is opened without `O_CREAT`: Linux's
`fs.protected_regular` refuses `O_CREAT` on another user's file there, even to root. Its creator gives it mode `0666`
whatever its umask. A writer that locked a file something unlinked meanwhile
(a `/tmp` cleaner) locks the file the path now names instead.
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
incompatible layout is still rejected, never reset. A read-only opener takes no
lock: it validates the commit, so a segment a writer is still creating, sizing
or initializing at that moment opens as absent, too small or uncommitted, for
the opener to retry. It cannot repair an incomplete segment and reports that a
writer must initialize it. A process holding the lock, even one stopped while
holding it, therefore cannot block a reader such as the launcher's supervision
loop. Lock files deliberately persist, even across explicit segment
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

`kSourceFlagLatestOnly` is the one source flag the node does not declare: the
loop sets it (`LoopSourceFlags`, from `EventLoopBase::latest_only`) for each
watcher that delivers only the newest message, so a reader can tell that
watcher's `dropped` includes skips by design. The `Reporter` publishes it in
the registry, and `--describe` writes it as `"latest_only": true` on that
watcher alone (absent means a sequence watcher), which the launcher's log
budget reads ([log budget](../launcher/README.md#session-segments-and-logs)).
It is a bit of the existing `flags` word, needing no layout or version
change: an older writer leaves it clear, and an older reader ignores it.

`kSourceFlagHardwareRequest` marks the publisher the devices layer's flush
registers through `detail::NodeAccess::add_hardware_request`, the only way a
node opens `/hw/request/<node>`. Node code does not declare it: the node layer
sets it from that registration, and `--describe` writes it as
`"hardware_request": true` on that endpoint alone (absent means false, so the
format version is unchanged). `LintGraph` refuses a `/hw/request` writer
without it. Like `kSourceFlagLatestOnly` it is a bit of the existing `flags`
word with no registry layout or version change.


`publish` checks node and topic names against `names.h` and prints findings to
stderr without refusing publication. Timer labels are not checked as topics.

## Describe

`describe.h` writes and reads a node's `--describe` answer
(`DescribeToJson`, `DescribeReader`; format version 1): its name, target,
`reads_graph`, `params` and `sources`. `params` lists every parameter key the
node's constructor read (`Node::param`, `param_of`; set in the config or not),
in first-read order, as `{"subsystem": ..., "key": ...}`; the launcher refuses
a subsystem key no node lists
([parameter keys](../launcher/README.md#parameter-keys)). It is optional, so
the version is unchanged: a describe without it (Studio's bridge writes its
own, and a binary built before the field) reads as `params` nullopt, "not
reported", which the launcher does not judge, while `[]` reports none.

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

## Event log state

A node's event log can stop early, and capture never resumes: `LogWriter`
reports `failed()` for a disk error or for capture abandoned (the writer fell
behind, or a record did not fit), and `capped()` for a log whose byte budget
cost it records (its oldest segment deleted, or capture stopped at the cap). The `Reporter` copies both into the row's `flags` in each full
sample, when the loop's recorder reports them, as `kFlagRecorderFailed` and
`kFlagLogCapped`; the hardware bridge does the same for its own log
(`NodeRegistration::set_recording`). They are bits of the existing `flags`
word, which needs no layout or version change: an older writer leaves them
clear, and an older reader ignores them. Once set they stay set until the slot
is claimed again. The launcher records them in the session manifest while the
node runs ([launcher supervision](../launcher/README.md#restarts)).

## Realtime scheduling

A node's loop asks for realtime scheduling once, just before it runs
(`process::ConfigureRealtime`; [realtime scheduling](../../docs/NODE_DEVELOPMENT_GUIDE.md#realtime-scheduling)),
and the OS can refuse any step of it. The node then stores the outcome in its
row with `NodeRegistration::set_realtime` (`Reporter::set_realtime` for an
event-loop node; the hardware bridge calls it on its own registration),
whether or not it asked. It is bits 5 to 12 of `flags` (`kFlagRealtimeMask`):
the loop tier plus one (0 until the node reports, which a reader sees as
unreported, never as a grant), whether it asked, and the `process::kStep*`
steps the OS refused. `RealtimeOf(flags)` and `NodeSnapshot::realtime()`
decode it to a `process::RealtimeOutcome`, whose `grant()` is `full`,
`partial` (only a memory step refused), `refused` (the scheduling policy
refused: the loop runs at timeshare) or `not_requested`. Like the event log
flags, these are bits every earlier writer left clear and every earlier reader
ignores, so the layout and version are unchanged. A ring whose lock is
refused after the loop starts is not in the row (stderr only).

## Wake loss

A node sleeps until a publication wakes it ([wake channel](../rtms/README.md#wake-channel)),
so a lost wake costs a reader its loop's backstop with nothing in the
process to show it but one stderr line. The row counts it, in three
`uint64` fields at the end of `NodeRecord`, cumulative since the claim and
each normally 0: `wake_failures`, the notifications this node's writers
could not send for a reason other than a gone receiver
(`RTMSQueue::wake_failures()` summed over the loop's senders and the node's
byte topics, `Node::publish_bytes`, which its loop does not write, such as
`/talos/telemetry`: no descriptor left, say); `stalled_arms`, prompt
watchers whose failed arms stopped holding its loop awake; and
`wake_fallbacks`, runs that could not build their wake receiver and drain
on a poll tick instead. The `Reporter` copies
`RealtimeEventLoop::wake_loss()` in each full sample, adding each
`ExtraSource::wake_failures` (`NodeRegistration::set_wake_loss`); the
hardware bridge, whose loop is its own, stores its publishers' failures and its `OpenWaits()` fallbacks, and
no stalled arms. A wake to the bridge that is lost shows as the requests its
latest-value read of `/hw/request/<node>` then passes over: each counts in
`dropped` of that subsystem's request source in the bridge's row, which is
not latest-only, so Studio calls the topic `lossy`.
`NodeSnapshot::delivery_loss()` (and `DeliveryLossOf(const NodeRecord&)`,
for a reader that copies no snapshot, such as the launcher's supervision
pass, which keeps them per run in the
[session manifest](../launcher/README.md#session-manifest)) gathers them
with `requests_skipped`, the `dropped` of the node's readers of
`/hw/request/<node>` summed. The fields took the record's tail
padding, so `sizeof(NodeRecord)` stayed 11136 B, but a claim by an older
build would not have cleared them, so they are registry version 6
([data flow](../../docs/DATA_FLOW.md#main-processor-shared-memory-slots)).

How late the loop runs is in its timer sources: a periodic timer's worst
dispatch lateness and the periods it missed ([the counters](#the-counters)),
so an overrunning loop is visible even while its dispatch count keeps moving.

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

Every new claim also sweeps ACTIVE slots whose claimant is gone: its process
no longer runs, or its host-monotonic heartbeat is at least
`kLivenessTimeout`, five seconds old. A row records its claimant as RTMS
records a claim, the pid, the process start time and the pid namespace
(`pid_start`; `//talOS/rtms:process_liveness`), so a killed node's row is freed
by the next claim, and a pid reused since does not keep it. A claimant in
another pid namespace sharing `/dev/shm` cannot be looked up from this one, so
its row is freed only once its heartbeat is `kLivenessTimeout` old. Both are
rechecked after the ACTIVE-to-CLAIMING transition so a refresh observed during
the sweep can keep the row.

Each claim bumps a generation counter before the slot goes ACTIVE. A reader
snapshots the generation, copies the identity fields, and re-reads it; a change
means the slot was recycled mid-copy and the read is retried.
The source records are published the other way round: they are written first and
the count is stored with release ordering afterwards, so a reader that can see a
count can see the records it describes.

A registration releases its slot in its destructor, so a clean shutdown leaves
no ghost row and a viewer sees the node disappear immediately. A node that is
killed leaves its slot ACTIVE until the next claim, or until the heartbeat goes
stale; a crash-looping roster therefore cannot fill the registry.

## Observing must not perturb

Writers only ever touch their own slot, which is padded to a cache line so two
nodes updating counters do not contend on the same line. Every field a reader
observes while a node runs is a lock-free atomic — there are static assertions
to that effect, because a registry that fell back to a lock would be a registry
that could block a control loop. `RegistryReader` writes nothing and takes no
lock, opening included ([the segment](#the-segment)), so it is safe to poll at
any rate.
`RegistryReader::open` returns `nullopt` instead of throwing when the segment
does not exist: no nodes running is an ordinary viewer startup state.

The reporter claims its slot synchronously before user construction, setting
`kFlagConstructing` in the existing atomic flags field (no layout change).
`SlotState::ACTIVE` means the slot is owned; the lifecycle flag distinguishes
construction from dispatching. A bookkeeping thread heartbeats identity only,
every 10 ms while the node constructs, which the launcher bounds.
`RegistryReader` includes these rows with `alive = false`, so Studio can show
initialization without treating it as an active node.

After construction, `Reporter::start(loop, ...)` waits for `loop.running()`
while continuing identity heartbeats. The loop allocates its counters array
before storing `running_`, synchronizing the reporter's load. The reporter
publishes endpoints and clears the constructing flag only when the manifest
and counters exist. The launcher bounds construction from process spawn;
see [launcher supervision](../launcher/README.md#restarts).
The reporter samples the heartbeat, source counters, selections and event log
state every 250 ms (`kRefreshPeriod`, a twentieth of `kLivenessTimeout`, the
one deadline the heartbeat must meet once the loop runs), and while the loop
runs that refresh is the reporting thread's only wake. The dispatch count is not the
reporter's: `start` attaches an `event::DispatchObserver` to the loop, and the
loop thread stores the count
in the row at every dispatch, outside the sample sequence. It is the launcher's
evidence of progress ([supervision](../launcher/README.md#restarts)), and the
reporting thread runs below every realtime loop, so a loaded host can starve it
for longer than the stall timeout while the loop keeps time. Source counters
can lag the dispatch count until the next full sample.
It does not poll for the loop's end: the row stays until the reporter is
destroyed, which `talos::Run` does as soon as its loop returns, and stopping it
ends the refresh wait at once. A final full sample preserves the last counters.

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
A sender's dropped count is the publishes the transport refused, and a
periodic timer's is the periods it missed; what a watcher's dropped count
includes, intentional latest-only skips among it, is in
[the event system's Lateness section](../events/README.md#lateness). A source
flagged `kSourceFlagLatestOnly` has those skips in its count.

These are atomics so another thread can read them mid-run without a data race.

## Limits

The capacities are fixed at 32 nodes and 64 sources each. A node that registers
more sources than fit still reports the true count in
`declared_source_count`, so the truncation is visible rather than silent, but
the extra sources are not described. A 33rd node is refused with an error rather
than displacing a live one.

`HardwareNode::BuildIntrospectionManifest` supplies the hardware bridge's
manifest explicitly and its run loop refreshes the registry.

Refreshes are bracketed by a sample sequence (an event-loop node's dispatch
count, stored by its loop thread, is not). Readers retry inconsistent
copies a bounded number of times rather than waiting for a stalled writer;
these snapshots are observational, not control inputs.

A writer that owns the segment gives it mode `0666` (`kRegistryMode`)
whatever its umask; on macOS `shm_open` applies none. It provides no
authentication.

## Prepared by the launcher

A node runs on when it cannot register (above), but the launcher kills a node
that has no row once its startup grace ends, so a registry its nodes cannot
open would restart the whole roster forever. Before it starts any node, the
launcher therefore calls `PrepareRegistry`: it opens the segment read-write,
creating and initializing it when absent or uncommitted, gives the segment and
its lock file their modes, and opens the lock file as every writer must. Nodes
run as the launcher's user, so what it can open they can. It waits for the
lock only to create or initialize the segment, and at most
`kRegistryLockWait` (5 s). Anything it cannot open -- typically a segment or
lock file another user created with a narrower mode -- refuses the launch with
a message naming it, its owner and mode, and how to remove it.

Naming rules and whole-graph linting live in [NAMING.md](../NAMING.md).

## Building

```sh
bazel test //talOS/introspection/...
```

`registry_test` uses forked children stopped at startup-only injection hooks,
then SIGKILLed before the initialization or ACTIVE commit. `waitpid(WUNTRACED)`
synchronizes the kill without sleeps. The tests verify initialization recovery,
all 32 slots remaining usable after an abandoned claim, and eight concurrent
processes holding distinct registrations across the entire registry. A
SIGKILLed claimant's row is freed by the next claim, before its heartbeat ages
out; on Linux, where the test may create one, a claimant running as pid 1 of
its own pid namespace keeps its row through that claim; and `PrepareRegistry`
gives the segment and lock file their modes under a restrictive umask and
refuses a segment it cannot open read-write.

`:registry` is the segment and both ends of it, and depends only on
`//talOS/events:events_core`, `:names`, `:lock_file` and
`//talOS/rtms:process_liveness` — a reader such as Studio's bridge does not
pull in an event loop. It does not depend on `:describe` either, and
`publish`'s attribute overload is a template on the container for that reason:
the record of what is running should not have to include the description of what
would run. `:reporter` adds the sampling thread and is needed only by processes
that publish.

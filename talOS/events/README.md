# Event System

Periodic and message callbacks share an event loop. The package provides real,
simulated and recorded-input execution through the same registration API.
Node-level usage belongs in the
[node development guide](../../docs/NODE_DEVELOPMENT_GUIDE.md).

## Dispatch and replay

Replay requires handlers to derive their behavior from recorded inputs and the
loop clock:

1. **Single-threaded dispatch.** Callbacks run one at a time on the loop
   thread, never re-entrant. Sources are frozen once `run()` starts.
2. **Time comes only from the loop.** `monotonic_now()` is sampled once per
   dispatch and frozen for the whole callback, so two reads inside one handler
   cannot disagree. Handlers never touch `std::chrono` clocks directly.
3. **Every input is recorded.** Timer firings (deadline plus how many periods
   elapsed), message payloads with their sequence, drop count and writer
   publication time, the start time and the exit.
4. **Every output is recorded.** Each `send` is logged with its bytes, the
   transport's verdict and the sequence it got, tagged with the dispatch that
   produced it. Timer setup/disable and handler-requested exit are validated too.
5. **Stable identities.** Sources are numbered in registration order, and the
   log header carries a manifest of them. Replay refuses a log whose manifest
   does not match the program.
6. **Fixed dispatch order.** Expired timers first, ordered by (deadline, id),
   then watched topics in registration order. The id tiebreak matters: two
   timers due in the same iteration must never swap.
7. **Shared handlers.** The loop implementations use CRTP and callback thunks;
   a program templated on the loop can register the same handlers for real,
   simulated and replay execution.

Register and configure sources before starting the loop. A timer change made
while the loop is running must come from inside a callback: anything else is
another thread, which races the scheduler and has no reproducible position in
the log, so it throws `std::logic_error`. To change a schedule from outside,
send the loop a message and change it in the handler, where it is recorded.

A loop that is stopped is not running, so the harness may re-arm timers on a
`SimulatedEventLoop` between `run_for` calls. That is the outside world acting,
exactly like `inject()`, and it is deliberately not recorded: its only effect is
the firings it produces, and those are recorded.

`exit()` and `running()` are thread-safe; other API calls and lifecycle
operations require external serialization. `exit()` is not a signal-handler API.
What must reach a `RealtimeEventLoop` from outside the program, a signal
above all, goes through `watch_fd(fd, handler)`, set before the run: a wait
that finds `fd` readable runs the handler on the loop
thread, outside any dispatch and unrecorded, and a readable `fd` alone drains
nothing. The watch is level-triggered, so the handler reads `fd` empty or
calls `exit()`. A loop behind its timers never waits, so a stop also needs
`Options::stop`, a flag the loop reads before each timer, each message and
each wait, exiting once it is true: one relaxed load per dispatch
(`RealtimeEventLoop.AStopFlagEndsTheRunAfterTheCurrentDispatch`). A node
stops with both: SIGINT and SIGTERM set `process::stop_requested`, its flag,
and make `process::StopFd()` readable, whose handler calls `exit()`
(`RealtimeEventLoop.AWatchedFdEndsTheWaitOnTheLoopThread`).
Realtime and replay loop instances run once.

## The three loops

| Loop | Clock | Transport | Use |
|---|---|---|---|
| `RealtimeEventLoop` | OS monotonic clock via the poller | RTMS shared memory | On the robot |
| `SimulatedEventLoop` | Virtual, jumps to the next deadline or prompt publication | In-memory channels | Deterministic tests |
| `ReplayEventLoop` | Timestamps from the log | The log itself | Reproducing a run |

`RealtimeEventLoop` waits on kqueue plus an absolute-deadline `EVFILT_TIMER`
with `NOTE_CRITICAL` (macOS; a relative `kevent()` timeout is coalesced, waking
1-2 ms late) or `epoll_pwait2` given the time left to the deadline (Linux).
On PREEMPT_RT that wait is a sleeper hrtimer, expired in hard interrupt
context for a realtime thread, where a timerfd's expiry waits in `ktimers/N`
at `SCHED_FIFO` 1 behind any loop on that CPU. A kernel before 5.11
(`ENOSYS`) or a seccomp filter refusing the call (`EPERM`) falls back to
epoll plus an absolute-deadline timerfd, with the same deadlines, and the
poller says so once on stderr (`epoll_pwait2 unavailable`).
Every loop drains immediately before each timer dispatch, so a timer handler
sees what was published by its deadline under every delivery.
`Options::delivery` (or `set_delivery()` before the run) picks whether, and
when, it also drains between timers:

- `Delivery::POLL` (the default) also drains on a fixed tick, 1 ms by default,
  so the tick bounds message latency. A tick due at a timer's deadline adds
  nothing to that timer: the timer's own drain ran first.
- `Delivery::BEFORE_TIMERS` arms no poll timer, so a loop with only periodic
  work wakes once per period. `activity()` counts wakeups and drains; over
  20 ticks at 100 Hz that is about 20 of each against about 200 wakeups and
  220 drains under `POLL`
  (`RealtimeEventLoop.OnlyPollWakesBetweenTimersForAQuietTopic`).
- `Delivery::ON_PUBLISH` also drains when a publication on a *prompt* watcher
  (`set_prompt(id)` before the run) wakes the loop, and at no other time. The
  loop owns one `WakeReceiver` (`talOS/rtms/wake.h`, the transport's [wake
  channel](../rtms/README.md#wake-channel)) whose id each prompt watcher's
  reader slot carries (`RTMSQueue::set_reader_wake`, again on a slot claimed
  after another process freed the old one). Before it blocks the loop arms
  every prompt watcher (`RTMSQueue::arm_wake`) and blocks only when all of
  them are caught up; otherwise it drains and arms again. A publication after
  a successful arm makes the receiver's fd readable, and the poller returns.
  A wake drains every watcher, prompt or not, in registration order, as a
  poll tick does, so a callback that reads a latest-value subscription sees
  current data. An idle loop sleeps until its next timer: a prompt watcher
  nobody publishes on costs no wakeups
  (`RealtimeEventLoop.OnPublishSleepsWhileNothingIsPublished`), and a
  publication on a watcher that is not prompt waits for the next timer's
  drain. A notification is only a hint, and a publisher that dies between
  publishing and notifying sends none, so while it has prompt watchers the
  loop never blocks longer than `Options::wake_backstop` (1 s) without
  draining (`RealtimeEventLoop.TheBackstopDrainsWhatNoNotificationAnnounced`);
  a loop whose next timer is nearer never reaches it. If three
  drains in a row, each run because one watcher's arm reported something to
  read, deliver nothing from it, that watcher's failed arm stops keeping the
  loop awake until a drain delivers from it again (counted once in
  `activity().stalled_arms`), so a transport that misreports unread data
  cannot make the loop spin
  (`RealtimeEventLoop.AnArmThatKeepsFailingWithNothingToReadStopsSpinning`). With no prompt watcher it runs as
  `BEFORE_TIMERS` and holds no receiver. A loop that cannot build its
  receiver (no descriptor left, say) runs that run under `POLL` instead,
  says so once on stderr, and counts it in `activity().wake_fallbacks`
  (`RealtimeEventLoop.WithoutAWakeReceiverTheLoopFallsBackToPolling`).
  `wake_loss()` returns both counts with the notifications its senders
  failed to send (`RTMSQueue::wake_failures()`), from any thread once the
  run has started; the node's registry row carries them
  ([wake loss](../introspection/README.md#wake-loss)).

The macOS wait also passes `kevent()` that coalesced relative timeout, as a
backstop: XNU delivers every `EVFILT_TIMER` through one shared kernel worker
thread (priority 93, below time-constraint threads), and while it does not
run every kqueue timer on the machine is late together, without bound. The
timeout expires the waiting thread's own wait instead, so such a stall costs
a wait at most the timeout's leeway, about a fifth of the wait.

Each dispatch samples its own `now`, frozen for that callback; a slow timer
callback cannot hide elapsed time from the next due timer. Messages retain the
writer's publication time separately from their drain/dispatch time. Message
age is `now - publish_time`, including any wait before draining, while
`Context::latency()` is `now - event_time`. A message's `event_time` is the
drain that delivered it: a timer's or poll tick's deadline, or the moment a
woken loop saw its wake, or the backstop's deadline. The exception is a
prompt watcher's message in a drain the loop ran because it was woken (a
wake, an arm that found it unread, or the backstop): the loop owed it a
dispatch from its publication, so its `event_time` is its publication time
and `latency()`, and the source's max latency, include the whole wait from
publication. A watcher that is not prompt in that same drain waited by
design and keeps the drain's time
(`RealtimeEventLoop.AWakeStampsOnlyPromptMessagesWithTheirPublication`).

Subscriptions without a per-message callback skip directly to the newest
available value once per drain, counting skipped messages as dropped. Callback
subscriptions retain sequence delivery with `max_messages_per_tick` (64 by
default) bounding each drain. A drain that stops at that cap is followed at
once by another, never by a wait: a prompt watcher's failed arm drains again
under `ON_PUBLISH`, and under `BEFORE_TIMERS` and `ON_PUBLISH` a sequence
watcher that is not prompt makes the loop drain again before it waits
(`POLL`'s next tick drains it anyway). Each drain stays capped and due timers
run between them, so a burst costs one drain per 64 messages per watcher, and
a watcher fed faster than 64 per drain never falls behind by a batch per
timer (`RealtimeEventLoop.ADrainCappedOnAQuietWatcherDrainsAgainAtOnce`).
`SimulatedEventLoop` makes the loop due again at that instant
(`next_deadline()`); such drains count toward `kMaxWokenDrainsPerInstant`.
The node's internal storage callback does not turn a plain latest-value
subscription into sequence delivery.

A watcher whose reader slot another process frees (having judged this one dead)
claims one again when a drain finds it gone, and at each later drain while none
is free; messages published while it had no slot count in the next delivery's
`Context::dropped`. That failed read, and any read the writer tears, also count
in the source's `SourceCounters::dropped`.

The log records dispatch and publication times and the resulting order, so
replay reproduces every delivery policy and ages. A drain leaves no record of
its own, only the messages it dispatched, each with its `event_time` (above)
and its own `now`, so a wake drain needs no new record kind and a replay,
which takes both from the log, re-records it byte for byte
(`RealtimeEventLoop.RealtimeRunReplaysForEveryDeliveryPolicy`).
`SimulatedEventLoop` takes the same options and stamps messages at simulated
send time. Under `ON_PUBLISH` it has no wake channel and needs none: a prompt
watcher's unread message is itself an event at its publication time, which
`next_deadline()` reports and `run_until()` handles in time order. With no
timer due by then, it is a wake drain as above (every watcher, in
registration order; a prompt watcher's message stamped with its publication
time, so `latency()` is 0), so a simulated callback runs at the instant of
the publication and a watcher that is not prompt is drained only before a
timer or by such a drain
(`SimulatedEventLoop.OnPublishRunsAPromptHandlerAtThePublicationInstant`,
`SimulatedEventLoop.OnPublishDrainsAQuietWatcherOnlyBeforeATimerOrWithAWake`).
At one instant it runs every due timer first, as the realtime loop's timer
pass does before it waits: a publication at a timer's deadline is delivered
by that timer's drain, stamped with the deadline (the same instant), in one
drain, and the timer's handler sees it
(`SimulatedEventLoop.APublicationAtATimersDeadlineIsDeliveredByThatTimersDrain`).
A message a handler publishes on a prompt watcher of the same loop drains at
that instant too, the 64-message cap included (the rest is due again at
once), and `run_until()` throws `std::runtime_error`, naming the topics,
after `kMaxWokenDrainsPerInstant` (1000) such drains at one instant, a
zero-time publish cycle that would never let time advance. Loops sharing an
environment see each other's prompt publications through `next_deadline()`:
a loop already run to an instant is due there again when another publishes
to it at that instant (`talos::testing::Sim` runs it again). The node layer
chooses delivery timing and which watchers are prompt: see
`NodeAccess::require_immediate_delivery` in `talOS/node/access.h`. A
`Node::subscribe` callback opened with `talos::Wake::kAtDrain` is a sequence
watcher that is not prompt: its messages wait for a timer's drain or another
watcher's wake, and a backlog over the cap drains again at once, as above.


## Recording and replaying

```cpp
RealtimeEventLoop<log::LogWriter> loop{log::LogWriter{"/tmp/run.tlog", "arm"}};
Arm<decltype(loop)> arm{loop, 2.0F};
arm.start(loop.monotonic_now() + 1ms);
loop.run();
```

```cpp
log::LogReader reader{"/tmp/run.tlog"};
ReplayEventLoop<> replay{reader};
Arm<decltype(replay)> arm{replay, 2.0F};
arm.start(replay.monotonic_now() + 1ms);  // the identical setup line
replay.run();

if (replay.diverged()) { /* the program no longer does what it did */ }
```

Replay verifies as it goes. Every send must match the recorded bytes, in order.
A changed gain, a reordered branch or a missing send is reported as a
divergence naming the dispatch and the topic. Set `Options{false}` to collect
every divergence in one pass instead of throwing at the first. Only the loop's
typed sends, timer changes, recorded reads and exit are verified: byte topics
(`Node::publish_bytes`) are outside the loop and never logged, so a node
binary's `--replay` (`talOS/node/node.cc`) discards them, exactly as
`--replay-inputs` does below. Replay is offline either way: it never attaches
to or writes the shared memory of a robot that may be running on this machine.

Log versions and byte sizes are recorded in
[DATA_FLOW.md](../../docs/DATA_FLOW.md#dispatch-logs).

Note that both snippets set the schedule up the same way. `monotonic_now()`
before `run()` returns the loop's *origin*, which the realtime loop fixes at
construction and the replay loop takes from the log, so a program that builds
its initial schedule out of the loop's clock lands on the same numbers in both.
The manifest stores each timer's initial armed state, period and first deadline
**relative to that origin**, never an absolute one: an absolute deadline is a
fact about when the machine booted, and recording one would force every replay
to be handed the original run's numbers by hand. Missing setup, or a changed
period or phase, is rejected before any dispatch.

The realtime loop runs that schedule from the moment `run()` starts: a timer
armed for the origin plus 1 ms first comes due 1 ms into the run. Whatever ran
between construction and `run()` (a node's config parsing, imports, device
setup) is therefore not reported as the first dispatch's `latency()`, missed
periods or `SourceCounters::max_latency_ns`. The log records each firing at
the deadline the loop used, and replay takes firings from the log, so replay is
unaffected.

After `run()` starts, `monotonic_now()` outside a dispatch returns the loop's
current time instead, so a harness stepping a simulation sees time where it left
it. Inside a dispatch it is always the frozen recorded `now`, which is the only
form handlers ever see.


During callbacks, `ARM_TIMER` and `DISARM_TIMER` records validate scheduling
calls and `HANDLER_EXIT` validates explicit shutdown. A `READ` record
(`record_read()`) notes that the dispatch used a watcher's current value: its
`now`, that value's publication time and sequence, and no payload. It changes
nothing the program sees, and replay checks the source and value read. Outside
a dispatch `record_read()` records nothing. An early shutdown stops
further dispatch even when collecting divergences. External shutdown is
represented by the terminal `EXIT` record.

Inspect a log with `bazel run //talOS/launcher:log_dump -- /tmp/run.tlog`, or
convert it for AdvantageScope with `bazel run //talOS/launcher:tlog_to_wpilog --
/tmp/run.tlog -o /tmp/run.wpilog` (talOS/launcher/README.md).

## Replaying another session's inputs

`ReplayEventLoop` answers "does this program still do what it did?" and
refuses a program whose manifest differs from the log's. Offline tuning asks
the other question: what would a *new* program -- one that was not in the
session, or a changed version of one that was -- have done in that match?
`input_replay.h` answers it without comparing anything:

```cpp
// Every *.tlog; "estimator" is the replayed program's process name.
const RecordedSession session{"/tmp/talos_logs/<session>", "estimator"};
SimulationEnvironment environment;
SimulatedEventLoop<log::LogWriter> loop{
    environment, log::LogWriter{"/tmp/out/estimator.tlog", "estimator"},
    {.start = session.loop_start(), .run_start = session.run_start()}};
Estimator estimator{loop};                 // registers its watchers
const InputReplayReport report = ReplayInputs(loop, environment, session);
```

- `RecordedSession` reads every log of a session directory. A topic's
  messages come from whichever logs recorded them: the writer's accepted
  `SEND`s or readers' receipts. A writer's `SEND` (an event loop's, or the
  hardware bridge's `--log` of `/hw/state` and `/hw/command`) and its
  readers' receipts carry the same transport sequence and publication time,
  so a program that did not watch a topic live is fed each of its
  publications once, at its publication time. One message seen
  in several logs is kept once, by topic and transport sequence, at the
  earliest time recorded. Logs from two sessions, or a topic recorded with
  two schemas, are refused.
- A program the session ran (logs carrying its process name) replays on its
  own recorded timeline. Each process's log has its own origin, and its
  loop's schedule counts from when its `run()` began, after construction
  (`RealtimeEventLoop`), so the loop starts at its first run's origin
  (`loop_start()`) with its run starting where that run's did (`run_start()`:
  its first timer firing's deadline minus the timer's manifest offset): its
  timers fire at the recorded deadlines. A topic it watched live is fed from
  its own receipts alone, each at the drain that delivered it live (its
  `MESSAGE` record's event time), with its recorded publication time; the
  replayed drain runs at its deadline, without the live loop's wake-up
  latency, so a message published during that latency is stamped with the
  deadline. A message one live drain missed, and the next took, is taken by
  that next drain again. A watcher that took only the newest message sees no
  skipped message, so its dropped count is zero. The replay ends at the program's
  last record (`loop_end()`).
- `ReplayInputs` runs the loop on its virtual clock to `loop_end()`, the end
  of the session for a program it did not run. Each recorded message on a
  topic the program watches is published with its publication time at its
  delivery instant (`RecordedMessage::delivered`: its publication time, or
  its live drain), so the drain there delivers it with its original age.
  Topics the program sends are its own output and are not fed; what it sends
  goes to its recorder, a new log.
- A watched topic recorded with another schema (name or hash in the
  per-topic descriptors) throws `ReplayError` before anything runs, naming
  both types and who recorded it. Watched topics no log recorded are listed in
  `InputReplayReport::unrecorded`; nothing arrives on them.

Replay feeds a topic the program did not watch live from the session origin,
including messages it would have missed during startup; its output may differ
from the live log at the beginning. An unchanged program replayed from its
own session sends what it sent live, at the same deadlines, except for what
the live loop's wake-up latency put into it (`now()` at a dispatch is the
deadline, not the deadline plus that latency).
Pass an explicit `--session-id` to keep that header field stable. The log's
wall-clock start timestamp remains informational.

Node binaries expose this as
`--replay-inputs <session dir>` (`talOS/node/node.cc`, Python nodes included
through the C ABI); the launcher as `--replay-inputs <dir> --only <node>`
(`talOS/launcher/README.md`). Byte topics (`Node::publish_bytes`) are
discarded there, as under `--replay`: the loop never logs them, and an offline
run must not write into a live robot's shared memory.

## Log format

`[FileHeader][ManifestEntry × n][schema block][Record ...]`, little-endian,
append-only, with a CRC on the manifest and schema blocks, and on every record.
The schema block carries each manifest entry's topic schema
(`talOS/schema/identity.h`): its name and embedded `.bfbs` reflection schema
bytes. Stored payloads omit trailing zero bytes; readers restore the declared
payload size with zero padding. Complete records before a truncated final
record remain readable. A record whose magic, lengths or checksum are wrong is
skipped: the reader scans to the next record magic whose header and checksum
validate, and counts what it skipped (`LogReader::damage()`), so one damaged
record, or a garbage or zero-filled tail left by a power loss, costs only
itself. `--replay-inputs` replays the intact records and warns with the file
and byte offsets; exact replay (`ReplayEventLoop`, `--replay`) refuses a log
with records missing between intact ones, and replays a damaged tail's prefix
as it does a truncated log's.

`log::LogReader` holds the whole file; `log::WindowedLogReader` (and
`VisitLog` on it) reads the same records, damage skipped the same way,
through a fixed 64 KiB window, so a reader of a whole session (`log_dump`,
`tlog_to_wpilog`) holds memory per log, not per record.

Every record a reader returns carries its byte offset in its file
(`Record::offset`; a rotated log's segment is a file of its own, and the
session merge passes it on as `MergedRecord::offset`). An index of a log
keeps those offsets with the file's path and reads from one later: a
`WindowedLogReader` opened on the file reads its header, manifest and schema
block, and `seek(offset)` (or `VisitLog` with `VisitOptions::start_offset`)
continues at that record, so the manifest and schemas are available and no
earlier record is read. From there it returns exactly the records,
and skips exactly the damage, that a read from the start does, and counts
records and damage from the offset. It refuses, with
`std::invalid_argument` naming the file and the offset, an offset inside
the header, manifest or schema block, or one where no intact record starts
(magic, lengths and checksum). The checksum cannot tell a record a read from
the start returns from another intact record: one nested whole in another
record's payload, one inside a span a read skipped as damage (from another
log, or larger than the scan accepts), or a record boundary of another file
with the same layout. So an offset is only meaningful for the file it came
from, and only one a reader returned.

Format details live in `log/format.h`. The version field is checked on read;
readers reject a version they do not know rather than guessing.
Readers reject unsupported versions with both recorded and supported versions
in the error.
Records are copied into preallocated chunks and handed to a background thread
through an atomic ring. Dispatch does not lock, wait for disk, or resize buffers.
The writer writes a record to the file within `write_interval` (10 ms) of
committing it, chunk full or not, so a node killed by a signal (the launcher's
stall kill, a segfault, an abort), which runs no flush, loses only its last
10 ms or so plus the writer's scheduling delay: the kernel keeps what was
written. While records arrive closer together than `write_interval` the
writer wakes and writes once per `write_interval` (at most 100 times a
second, whatever the tick rate); a record that finds it idle is written at
once, one wakeup per record (50 a second for a 50 Hz node). Once a
`write_interval` passes with no record, and after a disk error, it sleeps on
a semaphore until a chunk is handed over, a record finds it idle, or its
once-per-second `fsync` (what a power loss can lose) is due. A full chunk
never waits for `write_interval`: handing it over costs the loop thread an
atomic increment and one semaphore post whether or not the writer sleeps, so
a flood wakes the writer once per chunk. Committing a record costs one
atomic store and load, plus, when the writer is idle, one semaphore post (`sem_post` on Linux, `dispatch_semaphore_signal` on
macOS): it may enter the kernel to wake the writer, but never blocks or takes
a lock, and a post made before the writer waits is kept. A node logging
more often than every 10 ms posts a record only when its stream starts.
The writer runs under the normal scheduling
policy (`SCHED_OTHER` on Linux, the standard policy on macOS) even when the
thread that started it runs realtime. If the pool is exhausted,
capture stops permanently: `failed()` becomes true and `error()` explains why.
Previously accepted records are drained in order, leaving a prefix without a
clean `EXIT`, rather than a log with holes.

Capture never resumes, because resuming would put a hole in the middle of the
log and replay would skip it without saying so. **Treat `failed()` as a robot
fault**, next to a brownout or a lost CAN device: it means the rest of the match
is not replayable. `error()` says why, and `capture_stopped()` distinguishes
"the loop outran the writer" from a disk error.

`start()` sizes every chunk for the manifest's largest payload. A record larger
than that capacity also stops capture without allocating. Header writing at
startup, explicit `flush()`, shutdown, and the opt-in synchronous writer can
still block on disk; do not call `flush()` from a control callback.

## Lateness

A handler can react to its own lateness straight from its context:
`context.latency()` is how late the dispatch ran, `context.sequence > 1` on a
timer means periods were missed, and `context.dropped` on a message counts
messages this subscription skipped without observing. On a sequence watcher
(not `latest_only`) a non-zero `dropped` means data was lost rather than merely
delayed: the writer lapped the reader, or the reader slot was freed and claimed
again. A latest-only watcher skips all but the newest message by design and
counts those skips as dropped too, so its `dropped` is not a fault signal.
Treat missed timer periods and a sequence watcher's drops as faults.

The poll tick controls drain frequency under `Delivery::POLL`, and a wake
starts a drain under `Delivery::ON_PUBLISH`; neither is a hard end-to-end
latency bound: callback execution and host scheduling can delay it.

Each source also keeps lock-free `SourceCounters` (events, drops, last
sequence, last and max latency) that the introspection reporter publishes
while the loop runs. A watcher's drop count is the sum of its `context.dropped`
values plus reads the writer tore and reads that found the reader slot freed,
so a latest-only watcher's count grows with every intentional skip;
`latest_only(id)` says which watchers those are, and the reporter publishes it
so Studio does not call those skips loss. A periodic timer's drop count is the
periods it missed, `context.sequence - 1` summed over its dispatches, and its
max latency the worst dispatch lateness, so an overrunning loop shows in the
registry while its dispatch count still moves.

Message conventions live in [ARCHITECTURE.md](../../ARCHITECTURE.md).

## Overflow

Publishers default to `OVERWRITE_OLDEST`, so a subscriber that stops reading
gets lapped rather than stalling the publisher. A stuck logging process must
never be able to freeze a control loop. Readers find out through
`Context::dropped`; [Lateness](#lateness) says which drops are faults.

At the RTMS layer, `DROP_NEWEST` refuses a write with `BUFFER_FULL` when a reader
falls behind; it does not block or retry. The application must handle that
outcome. Realtime loop topics use `OVERWRITE_OLDEST`.

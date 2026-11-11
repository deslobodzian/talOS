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
Realtime and replay loop instances run once.

## The three loops

| Loop | Clock | Transport | Use |
|---|---|---|---|
| `RealtimeEventLoop` | OS monotonic clock via the poller | RTMS shared memory | On the robot |
| `SimulatedEventLoop` | Virtual, jumps to the next deadline | In-memory channels | Deterministic tests |
| `ReplayEventLoop` | Timestamps from the log | The log itself | Reproducing a run |

`RealtimeEventLoop` waits on kqueue plus an absolute-deadline `EVFILT_TIMER`
with `NOTE_CRITICAL` (macOS; a relative `kevent()` timeout is coalesced, waking
1-2 ms late) or `epoll_pwait2` given the time left to the deadline (Linux).
On PREEMPT_RT that wait is a sleeper hrtimer, expired in hard interrupt
context for a realtime thread, where a timerfd's expiry waits in `ktimers/N`
at `SCHED_FIFO` 1 behind any loop on that CPU. A kernel before 5.11
(`ENOSYS`) or a seccomp filter refusing the call (`EPERM`) falls back to
epoll plus an absolute-deadline timerfd, with the same deadlines, and the
poller says so once on stderr (`epoll_pwait2 unavailable`). Messages are
drained rather than woken by the transport: RTMS carries no notification
channel and adding one would change its shared-memory layout.
Every loop drains immediately before each timer dispatch, so a timer handler
sees what was published by its deadline under either delivery.
`Options::delivery` (or `set_delivery()` before the run) picks whether it also
drains between timers:

- `Delivery::POLL` (the default) also drains on a fixed tick, 1 ms by default,
  so the tick bounds message latency. A tick due at a timer's deadline adds
  nothing to that timer: the timer's own drain ran first.
- `Delivery::BEFORE_TIMERS` arms no poll timer, so a loop with only periodic
  work wakes once per period. `activity()` counts wakeups and drains; at
  100 Hz that is about 20 of each over 200 ms against about 200 wakeups and
  220 drains under `POLL` (`RealtimeEventLoop.BeforeTimersWakesOnlyForTimers`).

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
`Context::latency()` remains drain scheduling latency.

Subscriptions without a per-message callback skip directly to the newest
available value once per drain, counting skipped messages as dropped. Callback
subscriptions retain sequence delivery with `max_messages_per_tick` (64 by
default) bounding each drain. The node's internal storage callback does not
turn a plain latest-value subscription into sequence delivery.

A watcher whose reader slot another process frees (having judged this one dead)
claims one again when a drain finds it gone, and at each later drain while none
is free; messages published while it had no slot count in the next delivery's
`Context::dropped`. That failed read, and any read the writer tears, also count
in the source's `SourceCounters::dropped`.

The log records dispatch and publication times and the resulting order, so
replay reproduces both delivery policies and ages. `SimulatedEventLoop` takes
the same options and stamps messages at simulated send time. The node layer
chooses delivery timing: see `NodeAccess::require_immediate_delivery` in
`talOS/node/access.h`.


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

Inspect a log with `bazel run //talOS/launcher:log_dump -- /tmp/run.tlog`.

## Replaying another session's inputs

`ReplayEventLoop` answers "does this program still do what it did?" and
refuses a program whose manifest differs from the log's. Offline tuning asks
the other question: what would a *new* program -- one that was not in the
session, or a changed version of one that was -- have done in that match?
`input_replay.h` answers it without comparing anything:

```cpp
const RecordedSession session{"/tmp/talos_logs/<session>"};  // every *.tlog
SimulationEnvironment environment;
SimulatedEventLoop<log::LogWriter> loop{
    environment, log::LogWriter{"/tmp/out/estimator.tlog", "estimator"},
    {.start = session.start()}};
Estimator estimator{loop};                 // registers its watchers
const InputReplayReport report = ReplayInputs(loop, environment, session);
```

- `RecordedSession` reads every log of a session directory. A topic's
  messages come from whichever logs recorded them: the writer's accepted
  `SEND`s or readers' receipts. Both preserve the writer's publication time.
  One message seen in several logs is kept once, by topic and transport
  sequence. Logs from two sessions, or a topic recorded with two schemas,
  are refused.
- `ReplayInputs` runs the loop on its virtual clock from the session's origin
  to its last recorded time. Each recorded message on a topic the program
  watches is published at the writer's publication time, so the next drain
  delivers it with its original age. Topics the program sends are its own
  output and are not fed; what it sends goes to its recorder, a new log.
- A watched topic recorded with another schema (name or hash in the
  per-topic descriptors) throws `ReplayError` before anything runs, naming
  both types and who recorded it. Watched topics no log recorded are listed in
  `InputReplayReport::unrecorded`; nothing arrives on them.

Replay feeds inputs from the session origin, including messages a live node
missed during startup; its output may differ from the live log at the beginning.
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

Format details live in `log/format.h`. The version field is checked on read;
readers reject a version they do not know rather than guessing.
Readers reject unsupported versions with both recorded and supported versions
in the error.
Records are copied into preallocated chunks and handed to a background thread
through an atomic ring. Dispatch does not lock, wait for disk, or resize buffers.
The writer sleeps on a semaphore until a chunk is handed over or its
once-per-second sync is due. Handing over a chunk costs the loop thread an
atomic increment and load, plus, when the writer is idle, one semaphore post
(`sem_post` on Linux, `dispatch_semaphore_signal` on macOS): it may enter the
kernel to wake the writer, but never blocks or takes a lock, and a post made
before the writer waits is kept. The writer runs under the normal scheduling
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

The poll tick controls drain frequency under `Delivery::POLL`, not a hard
end-to-end latency bound: callback execution and host scheduling can delay it.

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

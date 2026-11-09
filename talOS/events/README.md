# Event System

## Purpose

Robot logic does not happen in sequence. A motor is read continuously, a command
is sent occasionally, a sensor fault has to interrupt whatever else was
happening. Polling every condition burns cycles; the event loop waits instead,
and runs a callback when something actually happens.

There are two kinds of event:

* **Periodic** — a callback every N microseconds. Control loops live here.
* **Message** — a callback when a topic publishes. Sensor data, setpoints,
  faults.

Both are dispatched by the same loop, on one thread, in a defined order.

## The real goal: replay

Anything that goes wrong on the field has to be reproducible at the bench. So
the loop is built so that a recorded run can be replayed *exactly* — same
handlers, same inputs, same outputs, byte for byte. That is a stronger property
than logging, and it constrains the whole design.

Replay is perfect only if the handlers are a pure function of what the loop
gave them. Seven rules make that true:

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
7. **The same code replays.** Robot code is a template on the loop type, so the
   realtime, simulated and replay loops run identical handlers. There are no
   virtual functions anywhere in the dispatch path.

Point 7 is also a latency requirement. The loops are CRTP, the recorder is a
template policy (`NullRecorder` compiles to nothing), the platform backend is
chosen at compile time, and callbacks are `{object, thunk}` pairs rather than
`std::function`. The realtime scheduler and background recorder preallocate
before dispatch begins. User callbacks must manage their own allocation budget;
simulation and replay are not allocation-free.

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

`RealtimeEventLoop` waits on kqueue (macOS) or epoll plus an absolute-deadline
timerfd (Linux). Messages are drained rather than woken by the transport: RTMS
carries no notification channel and adding one would change its shared-memory
layout. `Options::delivery` (or `set_delivery()` before the run) picks when:

- `Delivery::POLL` (the default) drains on a fixed tick, 1 ms by default, so
  the tick bounds message latency.
- `Delivery::BEFORE_TIMERS` arms no poll timer and drains immediately before
  each timer dispatch, so a loop with only periodic work wakes once per period.
  `activity()` counts wakeups and drains; at 100 Hz that is about 20 of each
  over 200 ms against about 200 under `POLL`
  (`RealtimeEventLoop.BeforeTimersWakesOnlyForTimers`).

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

The log records dispatch and publication times and the resulting order, so
replay reproduces both delivery policies and ages. `SimulatedEventLoop` takes
the same options and stamps messages at simulated send time. The node layer
chooses delivery timing: see `NodeAccess::require_immediate_delivery` in
`talOS/node/access.h`.

## Writing a handler

```cpp
template <typename Loop>
class Arm {
 public:
  Arm(Loop& loop, float gain) : gain_{gain} {
    watch<SensorMessage, &Arm::on_sensor>(loop, "/arm/sensor", this);
    command_ = make_sender<CommandMessage>(loop, "/arm/command");
    control_ = make_timer<&Arm::on_control>(loop, "control", this);
  }

  void start(MonotonicTime first_deadline) {
    control_.setup_periodic(first_deadline, 1ms);
  }

  std::uint16_t control_id() const { return control_.id(); }

 private:
  void on_sensor(const Context& context, const SensorMessage& message) {
    position_ = message.position();
    if (context.dropped > 0) { /* data was lost, not just delayed */ }
  }

  void on_control(const Context& context) {
    command_.send(CommandMessage{gain_ * (target_ - position_)});
  }
};
```

Registration order defines the source ids, so keep it stable: reordering these
lines invalidates existing logs.

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
typed sends, timer changes and exit are verified: byte topics
(`Node::publish_bytes`) are outside the loop and never logged, so a node
binary's `--replay` (`talOS/node/node.cc`) discards them, exactly as
`--replay-inputs` does below. Replay is offline either way: it never attaches
to or writes the shared memory of a robot that may be running on this machine.

Dispatch logs use `.tlog` version 6: 120 B file header, 112 B manifest entries,
64 B record headers plus stored payloads. The extra publication timestamp is
required to reproduce ages; older versions are rejected with both version
numbers in the error. Input replay injects messages at publication, not receipt.

Note that both snippets set the schedule up the same way. `monotonic_now()`
before `run()` returns the loop's *origin*, which the realtime loop fixes at
construction and the replay loop takes from the log, so a program that builds
its initial schedule out of the loop's clock lands on the same numbers in both.
The manifest stores each timer's initial armed state, period and first deadline
**relative to that origin**, never an absolute one: an absolute deadline is a
fact about when the machine booted, and recording one would force every replay
to be handed the original run's numbers by hand. Missing setup, or a changed
period or phase, is rejected before any dispatch.

After `run()` starts, `monotonic_now()` outside a dispatch returns the loop's
current time instead, so a harness stepping a simulation sees time where it left
it. Inside a dispatch it is always the frozen recorded `now`, which is the only
form handlers ever see.

One consequence worth knowing: the realtime loop's origin is the moment the loop
object is constructed, so a first deadline of `monotonic_now() + 1ms` is one
millisecond after *construction*, not after `run()`. If setup between the two
takes longer than that, the first cycle is already late and fires once with a
cycle count instead of bursting. That is deterministic and it replays, but if
you want the first tick to land a period after the run begins, either construct
the loop last or arm the timer inside the first dispatch.

During callbacks, `ARM_TIMER` and `DISARM_TIMER` records validate scheduling
calls and `HANDLER_EXIT` validates explicit shutdown. An early shutdown stops
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

The loop is simulated and its inputs come from a file, so two replays of one
session produce identical records. Replay feeds every input from the session
origin (monotonic t = 0), including any messages the live node missed during
process startup or native module imports, so replayed output can legitimately
differ from the live log at the beginning of the run. To make the output
`.tlog` files byte-identical in their headers across runs, pass an explicit
`--session-id <id>` (otherwise only the generated `session_id` and the
informational wall-clock `start_wall_ns` in `FileHeader` differ, with all
records, schemas, and payloads 100% bit-identical).

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
bytes. A record is a 64-byte header plus its payload up to the last non-zero
byte: fixed-size messages are mostly trailing zero padding, so the header keeps
the payload's full size and the reader zero-fills it back, handing out exactly
the bytes that were recorded. A log that ends mid-record is not corruption:
everything before the cut still replays, which matters when a robot loses
power. A record whose checksum is wrong *is* corruption and is refused.

Format details live in `log/format.h`. The version field is checked on read;
readers reject a version they do not know rather than guessing.
The current format is version 6, which adds each message's writer publication
time to the schema identities introduced in version 5. Other versions are
refused with both recorded and supported versions in the error.
Records are copied into preallocated chunks and handed to a background thread
through an atomic ring. Dispatch does not lock, wait for disk, or resize buffers.
The writer checks for new chunks every millisecond. If the pool is exhausted,
capture stops permanently: `failed()` becomes true and `error()` explains why.
Previously accepted records are drained in order, leaving a prefix without a
clean `EXIT`, rather than a log with holes.

Capture never resumes, because resuming would put a hole in the middle of the
log and replay would skip it without saying so. **Treat `failed()` as a robot
fault**, next to a brownout or a lost CAN device: it means the rest of the match
is not replayable. `error()` says why, and `capture_stopped()` distinguishes
"the loop outran the writer" from a disk error. `//2026-robot/main_processor/drivetrain:node` exits
non-zero and prints the reason; do the same in any process you add.

`start()` sizes every chunk for the manifest's largest payload. A record larger
than that capacity also stops capture without allocating. Header writing at
startup, explicit `flush()`, shutdown, and the opt-in synchronous writer can
still block on disk; do not call `flush()` from a control callback.

## Lateness

A handler can react to its own lateness straight from its context:
`context.latency()` is how late the dispatch ran, `context.sequence > 1` on a
timer means periods were missed, and `context.dropped > 0` on a message means
data was lost rather than merely delayed. Treat both as faults.

Message latency is bounded by the drain tick: worst case one tick, mean half a
tick. Shorten the tick to trade CPU for message latency.

Each source also keeps lock-free `SourceCounters` (events, drops, last
sequence, last and max latency) that the introspection reporter publishes
while the loop runs.

## Message rules

Messages are flatbuffer **structs**, never tables: a slot has to have a size
known at compile time, and replay has to compare payloads byte for byte.
`LoopMessage` enforces this.

## Overflow

Publishers default to `OVERWRITE_OLDEST`, so a subscriber that stops reading
gets lapped rather than stalling the publisher. A stuck logging process must
never be able to freeze a control loop. Readers find out through
`Context::dropped`, which counts messages that were skipped, not merely
delayed. Treat a non-zero value as a fault.

At the RTMS layer, `DROP_NEWEST` refuses a write with `BUFFER_FULL` when a reader
falls behind; it does not block or retry. The application must handle that
outcome. Realtime loop topics use `OVERWRITE_OLDEST`.

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
   elapsed), message payloads with their sequence and drop count, fetch
   results including "there was nothing", the start time and the exit.
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
timerfd (Linux). Message delivery is a fixed-rate drain, 1 ms by default, rather
than a wakeup from the transport: RTMS carries no notification channel and
adding one would change its shared-memory layout. The tick therefore bounds
message latency, and each drained message is stamped with the tick that saw it.

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
every divergence in one pass instead of throwing at the first.

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
the loop last or arm the timer inside the first dispatch. A benchmark that is
never replayed can simply arm off `Poller::now()` immediately before `run()`,
which is what `tools/loop_perf.cc` does.

During callbacks, `ARM_TIMER` and `DISARM_TIMER` records validate scheduling
calls and `HANDLER_EXIT` validates explicit shutdown. An early shutdown stops
further dispatch even when collecting divergences. External shutdown is
represented by the terminal `EXIT` record.

Inspect a log with `bazel run //talOS/events:log_dump -- /tmp/run.tlog`.

## Log format

`[FileHeader][ManifestEntry × n][Record ...]`, little-endian, append-only, with
a CRC on the manifest block and on every record. A record is a 56-byte header
plus its payload. A log that ends mid-record is not corruption: everything
before the cut still replays, which matters when a robot loses power. A record
whose checksum is wrong *is* corruption and is refused.

Format details live in `log/format.h`. The version field is checked on read;
readers reject a version they do not know rather than guessing.
The current format is version 2; version 1 logs are rejected because they do
not contain the timer and shutdown information needed by these checks.

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
"the loop outran the writer" from a disk error. `//talOS/drivetrain:node` exits
non-zero and prints the reason; do the same in any process you add.

`start()` sizes every chunk for the manifest's largest payload. A record larger
than that capacity also stops capture without allocating. Header writing at
startup, explicit `flush()`, shutdown, and the opt-in synchronous writer can
still block on disk; do not call `flush()` from a control callback.

## Measuring latency and jitter

A realtime claim you cannot measure is a guess. Any loop can be instrumented by
naming a metrics policy, which is a second template parameter alongside the
recorder:

```cpp
RealtimeEventLoop<NullRecorder, LoopMetrics> loop;
...
loop.run_for(10s);
std::printf("%s", loop.metrics().report(loop.manifest()).c_str());
```

Per source it records three distributions, each as a full histogram rather than
an average, because the tail is the part that hurts:

* **dispatch latency**, `now - event_time`. How late the loop was to service
  the event. For a periodic timer this is the wake-up delay, and its spread is
  what a control engineer means by jitter. For a message it is how late the
  drain tick ran, not the age of the message: the shared-memory layout carries
  no publish timestamp, so end-to-end transport latency has to be measured by
  stamping the send time into the message, which is what `loop_perf` does.
* **interval**, the wall gap between consecutive dispatches. For a 1 kHz timer
  this should sit on 1 ms; drift or spread means the loop is not holding rate.
* **handler time**, how long the callback ran. This is the budget. When it
  approaches the period, overruns are about to start.

Plus counters for **overruns** (periods that elapsed unserviced) and **dropped
messages**, and a loop-wide **utilization** figure: the share of wall time
spent inside handlers.

`LoopMetrics` costs two clock reads and a few histogram increments per
dispatch, measured at 40 ns on an M-series Mac, which is 0.004% of a 1 ms
cycle. `NullMetrics` is the default and compiles away completely, clock reads
included, so an uninstrumented loop pays nothing at all.

The histogram is fixed-size and allocation-free: logarithmic buckets with 16
linear steps per octave, exact below 16 ns and 6.25% worst-case error above it,
covering nanoseconds to centuries in 4 KiB. Percentiles report the bucket's
upper bound, so a reported latency is never optimistic. Count, minimum, maximum
and mean are tracked exactly alongside it, so bucketing only ever affects
percentiles.

A handler can also react to its own lateness without any of this, straight from
its context: `context.sequence > 1` on a timer means periods were missed, and
`context.dropped > 0` on a message means data was lost rather than merely
delayed. Treat both as faults.

To benchmark rather than monitor, use the bundled tool:

```
bazel run //talOS/events:loop_perf -- --rate-hz 1000 --duration-ms 5000
```

It reports the loop-side distributions above plus end-to-end IPC latency,
measured by stamping the publish time into the message and comparing it against
the subscriber's reading of the same clock. Run it with and without `--log` to
price the logging.

### What the numbers mean

Read a measurement against the platform's floor, not against zero. On a
development Mac a bare `kevent` asked to wait 1 ms returns about 140 us late at
the median, and the loop measures roughly the same, so the loop is adding
essentially nothing on top of the kernel. macOS coalesces timers aggressively;
the Linux target with `timerfd` does considerably better, and that is the number
that matters for the robot.

Two macOS-specific notes, both measured rather than assumed. `SCHED_FIFO` makes
no difference. The mach time-constraint policy leaves the median alone but cuts
the tail hard, taking the worst case of a 1 ms wait from 1475 us to 219 us. If
development-machine tail latency ever starts mattering, that is the lever, and
it is not wired up today.

End-to-end IPC latency is bounded by the drain tick: worst case one tick, mean
half a tick. Measured on an idle Mac, publishing at 777 Hz so the rates do not
divide evenly:

| tick | mean | p99 | max |
|---|---|---|---|
| 1000 us | 494 us | 1016 us | 1098 us |
| 500 us | 261 us | 524 us | 597 us |
| 200 us | 104 us | 221 us | 220 us |

Shorten the tick to trade CPU for message latency; the relationship is linear
and holds to within a couple of percent.

Watch out for harmonic rates. Publishing at 1000 Hz against a 1000 us tick
phase-locks the two grids, and the mean rises from the expected 494 us to
835 us because the phase offset never averages out. A mean near a full tick in
the report is that, not a slow transport. Benchmark with rates that do not
divide evenly, or you will measure the phase rather than the system.

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

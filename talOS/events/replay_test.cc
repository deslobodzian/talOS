#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "events_test/events_test_message_generated.h"
#include "talOS/events/handles.h"
#include "talOS/events/log/gap_testing.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/events/test_robot.h"

namespace talos::event {
namespace {

using namespace std::chrono_literals;
using testing::TestRobot;
using testing::Topics;

Topics test_topics() {
  return Topics{"/replay/sensor", "/replay/setpoint", "/replay/command"};
}

std::string temp_path(std::string_view name) {
  return ::testing::TempDir() + "/talos_" + std::string{name} + "_" +
         std::to_string(::getpid()) + ".tlog";
}

std::vector<char> read_file(const std::string& path) {
  std::ifstream file{path, std::ios::binary};
  return std::vector<char>{std::istreambuf_iterator<char>{file},
                           std::istreambuf_iterator<char>{}};
}

// Byte offset of FileHeader::start_wall_ns. It is the one field that legally
// differs between a recording and a replay of it: wall time is informational,
// and the replay happens later than the run it reproduces.
const std::size_t WALL_CLOCK_OFFSET = offsetof(log::FileHeader, start_wall_ns);
constexpr std::size_t WALL_CLOCK_BYTES = 8;

// Drives a robot through a fixed script of inputs. Everything the handlers see
// is injected here, so the run is reproducible by construction.
std::vector<float> record_run(const std::string& path, float gain,
                              Delivery delivery = Delivery::POLL,
                              const log::LogWriterOptions& options = {}) {
  SimulationEnvironment environment;
  SimulatedEventLoop<log::LogWriter> loop{
      environment, log::LogWriter{path, "test_robot", options}};
  loop.set_delivery(delivery);

  TestRobot<SimulatedEventLoop<log::LogWriter>> robot{loop, test_topics(),
                                                      gain};
  // Under ON_PUBLISH the sensor wakes the loop, so each reading is delivered
  // by a drain of its own at its publication instant, ahead of the control
  // timer, and recorded with its publication time as its event time.
  if (delivery == Delivery::ON_PUBLISH) {
    for (const Registration& source : loop.manifest()) {
      if (source.name == test_topics().sensor) loop.set_prompt(source.id);
    }
  }

  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);

  for (int i = 0; i < 30; ++i) {
    loop.inject(test_topics().sensor,
                EventsTest::SensorMessage{i, static_cast<float>(i) * 0.25F,
                                          static_cast<float>(i) * 0.05F});

    if (i % 3 == 0) {
      loop.inject(test_topics().setpoint,
                  EventsTest::SetpointMessage{i, static_cast<float>(i)});
    }

    loop.run_for(1ms);
  }

  loop.finish();
  EXPECT_FALSE(loop.recorder().failed()) << loop.recorder().error();
  return robot.efforts();
}

struct MessageTimes {
  std::vector<Context> contexts;

  void receive(const Context& context, const EventsTest::SensorMessage&) {
    contexts.push_back(context);
  }
};

TEST(Replay, PreservesPublishTimeAndDispatchAge) {
  const auto path = temp_path("dispatch_age");
  constexpr auto origin = MonotonicTime::from_nanos(7'000'000'000);
  MessageTimes recorded;
  {
    SimulationEnvironment environment;
    using Loop = SimulatedEventLoop<log::LogWriter>;
    Loop loop{environment, log::LogWriter{path, "dispatch_age"},
              Loop::Options{.tick_period = 10ms, .start = origin}};
    watch<EventsTest::SensorMessage, &MessageTimes::receive>(
        loop, test_topics().sensor, &recorded);
    loop.run_for(2ms);
    loop.inject(test_topics().sensor, EventsTest::SensorMessage{1, 2.0F, 3.0F});
    loop.run_until(origin + 10ms);
    loop.finish();
    ASSERT_FALSE(loop.recorder().failed()) << loop.recorder().error();
  }
  ASSERT_EQ(recorded.contexts.size(), 1u);
  EXPECT_EQ(recorded.contexts[0].publish_time, origin + 2ms);
  EXPECT_EQ(recorded.contexts[0].now - recorded.contexts[0].publish_time, 8ms);

  log::LogReader reader{path};
  ReplayEventLoop<> loop{reader};
  MessageTimes replayed;
  watch<EventsTest::SensorMessage, &MessageTimes::receive>(
      loop, test_topics().sensor, &replayed);
  loop.run();
  EXPECT_FALSE(loop.diverged());
  EXPECT_TRUE(loop.reached_exit());
  ASSERT_EQ(replayed.contexts.size(), 1u);
  EXPECT_EQ(replayed.contexts[0].event_time, recorded.contexts[0].event_time);
  EXPECT_EQ(replayed.contexts[0].now, recorded.contexts[0].now);
  EXPECT_EQ(replayed.contexts[0].publish_time,
            recorded.contexts[0].publish_time);
  EXPECT_EQ(replayed.contexts[0].now - replayed.contexts[0].publish_time, 8ms);
  std::remove(path.c_str());
}

// The core guarantee: replaying a log and recording that replay produces the
// same log again, byte for byte. If any input were left out of the log, or any
// handler read something the loop did not record, this test would fail. The
// delivery policy only moves where the drains fall in the recorded run, so it
// must not matter.
void ExpectReRecordingIsByteIdentical(Delivery delivery) {
  const std::string original_path = temp_path("identical_a");
  const std::string replayed_path = temp_path("identical_b");

  record_run(original_path, 1.75F, delivery);

  {
    log::LogReader reader{original_path};
    ReplayEventLoop<log::LogWriter> loop{
        reader, log::LogWriter{replayed_path, "test_robot"}};

    TestRobot<ReplayEventLoop<log::LogWriter>> robot{loop, test_topics(),
                                                     1.75F};
    robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                         1ms);
    loop.run();
    ASSERT_FALSE(loop.diverged());
  }

  std::vector<char> original = read_file(original_path);
  std::vector<char> replayed = read_file(replayed_path);

  ASSERT_FALSE(original.empty());
  ASSERT_EQ(original.size(), replayed.size());

  // Wall clock is expected to differ; blank it in both before comparing.
  for (std::size_t i = 0; i < WALL_CLOCK_BYTES; ++i) {
    original[WALL_CLOCK_OFFSET + i] = 0;
    replayed[WALL_CLOCK_OFFSET + i] = 0;
  }

  EXPECT_EQ(original, replayed);

  std::remove(original_path.c_str());
  std::remove(replayed_path.c_str());
}

TEST(Replay, ReRecordingIsByteIdenticalForEveryDeliveryPolicy) {
  for (const auto delivery :
       {Delivery::POLL, Delivery::BEFORE_TIMERS, Delivery::ON_PUBLISH}) {
    ExpectReRecordingIsByteIdentical(delivery);
  }
}

// REQ-OPS-13
TEST(Replay, DetectsChangedBehaviour) {
  const std::string path = temp_path("divergence");
  record_run(path, 2.5F);

  // Locate the second control timer dispatch: the first runs with zero error
  // (position == target == 0), and the second acts on the first non-zero error.
  std::uint64_t expected_dispatch = 0;
  {
    log::LogReader inspect{path};
    log::LogReader::Record record;
    int control_ticks = 0;
    while (inspect.next(record)) {
      if (static_cast<EventKind>(record.header.kind) == EventKind::TIMER &&
          ++control_ticks == 2) {
        expected_dispatch = record.header.dispatch_index;
        break;
      }
    }
  }
  ASSERT_GT(expected_dispatch, 0u);

  log::LogReader reader{path};
  ReplayEventLoop<> loop{reader};

  // Same program, different gain: the first control cycle that acts on a
  // non-zero error must produce different bytes, naming dispatch and topic.
  TestRobot<ReplayEventLoop<>> robot{loop, test_topics(), 9.0F};
  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);

  try {
    loop.run();
    ADD_FAILURE() << "replay accepted divergent control output";
  } catch (const ReplayDivergence& e) {
    EXPECT_EQ(e.divergence().dispatch_index, expected_dispatch);
    const std::string what = e.what();
    EXPECT_NE(what.find("dispatch " + std::to_string(expected_dispatch)),
              std::string::npos)
        << what;
    EXPECT_NE(what.find(test_topics().command), std::string::npos) << what;
  }

  std::remove(path.c_str());
}

TEST(Replay, CollectsEveryDivergenceWhenAsked) {
  const std::string path = temp_path("collect");
  record_run(path, 2.5F);

  log::LogReader reader{path};
  ReplayEventLoop<> loop{reader, ReplayEventLoop<>::Options{false}};
  TestRobot<ReplayEventLoop<>> robot{loop, test_topics(), 9.0F};
  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);

  loop.run();

  EXPECT_TRUE(loop.diverged());
  EXPECT_GT(loop.divergences().size(), 1u);

  std::remove(path.c_str());
}

// REQ-OPS-13
TEST(Replay, RejectsALogFromADifferentProgram) {
  const std::string path = temp_path("manifest");
  record_run(path, 2.5F);

  log::LogReader reader{path};
  ReplayEventLoop<> loop{reader};
  TestRobot<ReplayEventLoop<>> robot{loop, test_topics(), 2.5F};
  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);

  // One extra source: the log can no longer describe this program.
  loop.register_timer("extra", Thunk{});

  // Refused before the first dispatch, naming the log.
  try {
    loop.run();
    ADD_FAILURE() << "replay ran a program its log does not describe";
  } catch (const ReplayError& e) {
    EXPECT_NE(std::string{e.what()}.find(path), std::string::npos) << e.what();
  }
  EXPECT_EQ(robot.efforts().size(), 0u);

  std::remove(path.c_str());
}

template <typename Loop>
struct TimerProgram {
  Loop& loop;
  Duration period{2ms};
  bool disable{true};
  bool stop{true};
  std::uint16_t id;
  int calls{0};

  explicit TimerProgram(Loop& event_loop) : loop{event_loop} {
    id = loop.register_timer("control", make_thunk<&TimerProgram::tick>(this));
    loop.arm_timer(id, MonotonicTime::from_nanos(1'000'000), {});
  }
  void tick(const Context& context) {
    ++calls;
    loop.arm_timer(id, context.now + period, period);
    if (disable) loop.disarm_timer(id);
    if (stop) loop.exit();
  }
};

void record_timer_program(const std::string& path) {
  SimulationEnvironment environment;
  SimulatedEventLoop<log::LogWriter> loop{environment,
                                          log::LogWriter{path, "timers"}};
  TimerProgram program{loop};
  loop.run_for(3ms);
  loop.finish();
  ASSERT_FALSE(loop.recorder().failed());
}

// A timer program that re-arms, disarms and exits from its handler replays
// without divergence, reaches the recorded exit and re-records identically.
TEST(Replay, TimerOperationsAndHandlerExitReRecordIdentically) {
  const auto original_path = temp_path("timer_identical_a");
  const auto replayed_path = temp_path("timer_identical_b");
  record_timer_program(original_path);
  {
    log::LogReader reader{original_path};
    ReplayEventLoop<log::LogWriter> loop{
        reader, log::LogWriter{replayed_path, "timers"}};
    TimerProgram program{loop};
    loop.run();
    ASSERT_FALSE(loop.diverged());
    EXPECT_TRUE(loop.reached_exit());
    EXPECT_EQ(program.calls, 1);
  }
  auto original = read_file(original_path);
  auto replayed = read_file(replayed_path);
  ASSERT_EQ(original.size(), replayed.size());
  ASSERT_GT(original.size(), WALL_CLOCK_OFFSET + WALL_CLOCK_BYTES);
  for (std::size_t i = 0; i < WALL_CLOCK_BYTES; ++i) {
    original[WALL_CLOCK_OFFSET + i] = 0;
    replayed[WALL_CLOCK_OFFSET + i] = 0;
  }
  EXPECT_EQ(original, replayed);
  std::remove(original_path.c_str());
  std::remove(replayed_path.c_str());
}

// Replays the recorded timer program with `change` applied to the replaying
// copy; returns the divergence replay reported, if any.
template <typename Change>
std::optional<Divergence> replay_changed_timer_program(const char* name,
                                                       Change change) {
  const auto path = temp_path(name);
  record_timer_program(path);
  std::optional<Divergence> diverged;
  {
    log::LogReader reader{path};
    ReplayEventLoop<> loop{reader};
    TimerProgram program{loop};
    change(program);
    try {
      loop.run();
    } catch (const ReplayDivergence& divergence) {
      diverged = divergence.divergence();
    }
  }
  std::remove(path.c_str());
  return diverged;
}

// The handler re-arms its timer with a different period than it recorded.
// REQ-OPS-13
TEST(Replay, DetectsAChangedTimerPeriod) {
  const auto divergence = replay_changed_timer_program(
      "timer_period", [](auto& program) { program.period = 3ms; });
  ASSERT_TRUE(divergence.has_value());
  EXPECT_EQ(divergence->dispatch_index, 1u);
}

// The handler no longer disarms its timer.
// REQ-OPS-13
TEST(Replay, DetectsASkippedDisarm) {
  const auto divergence = replay_changed_timer_program(
      "timer_disarm", [](auto& program) { program.disable = false; });
  ASSERT_TRUE(divergence.has_value());
  EXPECT_EQ(divergence->dispatch_index, 1u);
}

// The handler no longer exits the loop.
// REQ-OPS-13
TEST(Replay, DetectsAMissingExit) {
  const auto divergence = replay_changed_timer_program(
      "timer_exit", [](auto& program) { program.stop = false; });
  ASSERT_TRUE(divergence.has_value());
  EXPECT_EQ(divergence->dispatch_index, 1u);
}

// REQ-OPS-13
TEST(Replay, RejectsChangedOrMissingInitialSchedule) {
  const auto path = temp_path("initial_schedule");
  record_timer_program(path);
  for (int change = 0; change < 3; ++change) {
    SCOPED_TRACE(change);
    log::LogReader reader{path};
    ReplayEventLoop<> loop{reader};
    TimerProgram program{loop};
    if (change == 0) {
      loop.arm_timer(program.id, MonotonicTime::from_nanos(1'000'000), 2ms);
    }
    if (change == 1) {
      loop.arm_timer(program.id, MonotonicTime::from_nanos(2'000'000), {});
    }
    if (change == 2) loop.disarm_timer(program.id);
    try {
      loop.run();
      ADD_FAILURE() << "replay accepted mismatched initial schedule";
    } catch (const ReplayError& e) {
      EXPECT_NE(std::string{e.what()}.find(path), std::string::npos)
          << e.what();
    }
  }
  std::remove(path.c_str());
}

// A stepped recording, where the harness re-arms a timer between steps the way
// the outside world would. The replayed program does none of that: the firings
// the change produced are in the log, which is all replay needs.
TEST(Replay, ReplaysARunWhoseScheduleWasChangedBetweenSteps) {
  const auto path = temp_path("stepped_schedule");
  std::vector<float> recorded;

  {
    SimulationEnvironment environment;
    SimulatedEventLoop<log::LogWriter> loop{environment,
                                            log::LogWriter{path, "test_robot"}};
    TestRobot<SimulatedEventLoop<log::LogWriter>> robot{loop, test_topics(),
                                                        2.0F};
    robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                         1ms);

    for (int i = 0; i < 10; ++i) {
      loop.inject(test_topics().sensor,
                  EventsTest::SensorMessage{i, static_cast<float>(i) * 0.25F,
                                            static_cast<float>(i) * 0.05F});
      loop.run_for(1ms);

      // Halfway through, the harness slows the control timer down.
      if (i == 4) {
        robot.control_timer().setup_periodic(loop.monotonic_now() + 2ms, 2ms);
      }
    }

    loop.finish();
    ASSERT_FALSE(loop.recorder().failed()) << loop.recorder().error();
    recorded = robot.efforts();
  }

  ASSERT_FALSE(recorded.empty());

  log::LogReader reader{path};
  ReplayEventLoop<> loop{reader};
  TestRobot<ReplayEventLoop<>> robot{loop, test_topics(), 2.0F};
  robot.control_timer().setup_periodic(loop.monotonic_now() + 1ms, 1ms);

  loop.run();

  EXPECT_FALSE(loop.diverged());
  EXPECT_TRUE(loop.reached_exit());
  EXPECT_EQ(robot.efforts(), recorded);
  std::remove(path.c_str());
}

TEST(Replay, EarlyExitStopsDispatchEvenWhenCollectingDivergences) {
  const auto path = temp_path("early_exit");
  {
    SimulationEnvironment environment;
    SimulatedEventLoop<log::LogWriter> loop{environment,
                                            log::LogWriter{path, "exit"}};
    const auto id = loop.register_timer("tick", {});
    loop.arm_timer(id, MonotonicTime::from_nanos(1'000'000), 1ms);
    loop.run_for(3ms);
    loop.finish();
  }
  log::LogReader reader{path};
  ReplayEventLoop<> loop{reader, ReplayEventLoop<>::Options{false}};
  int calls = 0;
  auto handler = [&](const Context&) {
    ++calls;
    loop.exit();
  };
  const auto id = loop.register_timer("tick", make_thunk(&handler));
  loop.arm_timer(id, MonotonicTime::from_nanos(1'000'000), 1ms);
  loop.run();
  EXPECT_TRUE(loop.diverged());
  EXPECT_FALSE(loop.reached_exit());
  EXPECT_EQ(calls, 1);
  std::remove(path.c_str());
}

// Reads one of two watchers from a timer, the way a node's update() reads its
// target, and records which value it used.
template <typename Loop>
struct ReadProgram {
  struct Read {
    Context tick;
    Context used;
  };

  Loop& loop;
  bool read_setpoint{false};
  std::uint16_t sensor;
  std::uint16_t setpoint;
  std::uint16_t timer;
  std::optional<Context> last_sensor;
  std::optional<Context> last_setpoint;
  std::vector<Read> reads;

  explicit ReadProgram(Loop& event_loop) : loop{event_loop} {
    sensor = watch<EventsTest::SensorMessage, &ReadProgram::on_sensor>(
        loop, test_topics().sensor, this);
    setpoint = watch<EventsTest::SetpointMessage, &ReadProgram::on_setpoint>(
        loop, test_topics().setpoint, this);
    timer =
        loop.register_timer("control", make_thunk<&ReadProgram::tick>(this));
    loop.arm_timer(timer, MonotonicTime::from_nanos(2'000'000), 1ms);
  }
  void on_sensor(const Context& context, const EventsTest::SensorMessage&) {
    last_sensor = context;
  }
  void on_setpoint(const Context& context, const EventsTest::SetpointMessage&) {
    last_setpoint = context;
  }
  void tick(const Context& context) {
    const std::optional<Context>& used =
        read_setpoint ? last_setpoint : last_sensor;
    if (!used) return;
    loop.record_read(read_setpoint ? setpoint : sensor, context.now,
                     used->publish_time, used->sequence);
    reads.push_back(Read{context, *used});
  }
};

std::vector<ReadProgram<SimulatedEventLoop<log::LogWriter>>::Read>
record_read_program(const std::string& path) {
  SimulationEnvironment environment;
  SimulatedEventLoop<log::LogWriter> loop{environment,
                                          log::LogWriter{path, "reads"}};
  ReadProgram program{loop};
  // Published after the origin, so a read's publish time differs from its
  // timer dispatch's.
  loop.run_for(1ms);
  loop.inject(test_topics().sensor, EventsTest::SensorMessage{1, 2.0F, 3.0F});
  loop.inject(test_topics().setpoint, EventsTest::SetpointMessage{1, 4.0F});
  loop.run_for(3ms);
  loop.finish();
  EXPECT_FALSE(loop.recorder().failed()) << loop.recorder().error();
  return program.reads;
}

// A READ record carries the reading dispatch and the value it used, so a
// session report can measure the value's age at that dispatch from the log.
TEST(Replay, ReadRecordRoundTripsWithItsFields) {
  const auto path = temp_path("read_fields");
  const auto reads = record_read_program(path);
  ASSERT_GE(reads.size(), 2u);

  log::LogReader reader{path};
  log::LogReader::Record record;
  std::size_t index = 0;
  while (reader.next(record)) {
    if (static_cast<EventKind>(record.header.kind) != EventKind::READ) continue;
    ASSERT_LT(index, reads.size());
    const auto& [tick, used] = reads[index++];
    EXPECT_EQ(record.header.source_id, 0u);  // the sensor watcher
    EXPECT_EQ(record.header.dispatch_index, tick.dispatch_index);
    EXPECT_EQ(record.header.event_time_ns, tick.event_time.nanos());
    EXPECT_EQ(record.header.now_ns, tick.now.nanos());
    EXPECT_EQ(record.header.publish_time_ns, used.publish_time.nanos());
    EXPECT_NE(used.publish_time, tick.publish_time);
    EXPECT_EQ(record.header.sequence, used.sequence);
    EXPECT_EQ(record.header.aux, 0u);
    EXPECT_EQ(record.header.payload_bytes, 0u);
  }
  EXPECT_EQ(index, reads.size());
  std::remove(path.c_str());
}

// Exact replay holds a program to the reads it recorded: the unchanged program
// replays cleanly, and one that reads another source diverges.
TEST(Replay, DetectsAReadOfADifferentSource) {
  const auto path = temp_path("read_source");
  ASSERT_FALSE(record_read_program(path).empty());
  for (const bool read_setpoint : {false, true}) {
    log::LogReader reader{path};
    ReplayEventLoop<> loop{reader};
    ReadProgram program{loop};
    program.read_setpoint = read_setpoint;
    if (read_setpoint) {
      EXPECT_THROW(loop.run(), ReplayDivergence);
    } else {
      loop.run();
      EXPECT_FALSE(loop.diverged());
      EXPECT_TRUE(loop.reached_exit());
    }
  }
  std::remove(path.c_str());
}

struct LogRecords {
  std::vector<log::RecordHeader> headers;
  std::vector<std::uint64_t> offsets;
};

LogRecords read_records(const std::string& path) {
  log::LogReader reader{path};
  LogRecords records;
  log::LogReader::Record record;
  while (reader.next(record)) {
    records.headers.push_back(record.header);
    records.offsets.push_back(record.offset);
  }
  return records;
}

// A log with a GAP record replays every dispatch before the gap's first
// dropped dispatch and stops there: no divergence, no clean exit, the gap
// named, the controller's efforts the recorded run's up to there, and a
// re-recording of the replay is the log's bytes before that dispatch, then
// the replay's own EXIT. Rows: the gap starts a dispatch; it splits one, whose
// recorded part is not replayed as if whole; it lasts to the stop (closed by
// it, the EXIT after it).
// REQ-OPS-13
TEST(Replay, StopsBeforeTheDispatchAGapStartsIn) {
  const std::string original_path = temp_path("gap_original");
  const std::vector<float> recorded = record_run(original_path, 1.75F);
  const LogRecords original = read_records(original_path);
  const std::size_t n = original.headers.size();

  std::uint16_t control = 0;
  {
    log::LogReader reader{original_path};
    for (const Registration& source : reader.manifest()) {
      if (source.name == "control") control = source.id;
    }
  }
  const auto is = [&](std::size_t i, EventKind kind) {
    return original.headers[i].kind == static_cast<std::uint16_t>(kind);
  };
  // A control dispatch past the middle, its send, and the record three
  // dispatches later, where capture resumes.
  std::size_t timer = n / 2;
  while (timer < n && !(is(timer, EventKind::TIMER) &&
                        original.headers[timer].source_id == control)) {
    ++timer;
  }
  ASSERT_LT(timer + 1, n);
  ASSERT_TRUE(is(timer + 1, EventKind::SEND));
  ASSERT_EQ(original.headers[timer + 1].dispatch_index,
            original.headers[timer].dispatch_index);
  std::size_t resume = timer + 1;
  for (int dispatches = 0; resume < n; ++resume) {
    if (original.headers[resume].dispatch_index !=
            original.headers[resume - 1].dispatch_index &&
        ++dispatches == 3) {
      break;
    }
  }
  ASSERT_LT(resume, n - 1);
  const std::uint64_t dispatch = original.headers[timer].dispatch_index;
  std::size_t controls_before = 0;
  for (std::size_t i = 0; i < timer; ++i) {
    if (is(i, EventKind::TIMER) && original.headers[i].source_id == control) {
      ++controls_before;
    }
  }
  ASSERT_GT(controls_before, 0u);

  struct Row {
    const char* name;
    std::size_t first;
    std::size_t end;
    std::uint16_t flags;
  };
  const Row rows[] = {
      {"starts_a_dispatch", timer, resume, 0},
      {"splits_a_dispatch", timer + 1, resume, log::GAP_SPLITS_DISPATCH},
      {"lasts_to_the_stop", timer, n, log::GAP_CLOSED_AT_FLUSH},
  };
  for (const Row& row : rows) {
    SCOPED_TRACE(row.name);
    const std::string gapped_path =
        temp_path(std::string{"gapped_"} + row.name);
    const std::string replayed_path =
        temp_path(std::string{"gap_replayed_"} + row.name);
    const log::LogGap gap =
        log::CopyWithGap(original_path, gapped_path, row.first, row.end);
    ASSERT_EQ(gap.detail.flags, row.flags);
    ASSERT_EQ(gap.first_dispatch_index, dispatch);

    std::vector<float> efforts;
    {
      log::LogReader reader{gapped_path};
      ReplayEventLoop<log::LogWriter> loop{
          reader, log::LogWriter{replayed_path, "test_robot"}};
      TestRobot<ReplayEventLoop<log::LogWriter>> robot{loop, test_topics(),
                                                       1.75F};
      robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                           1ms);
      loop.run();
      EXPECT_FALSE(loop.diverged());
      EXPECT_FALSE(loop.reached_exit());
      ASSERT_TRUE(loop.stopped_at_gap().has_value());
      const log::LogGap& stopped = loop.stopped_at_gap().value();
      EXPECT_EQ(stopped.first_dispatch_index, gap.first_dispatch_index);
      EXPECT_EQ(stopped.start_ns, gap.start_ns);
      EXPECT_EQ(stopped.resumed_ns, gap.resumed_ns);
      EXPECT_EQ(stopped.detail.records, gap.detail.records);
      EXPECT_EQ(stopped.detail.last_dispatch_index,
                gap.detail.last_dispatch_index);
      EXPECT_EQ(stopped.detail.flags, row.flags);
      efforts = robot.efforts();
    }
    EXPECT_EQ(efforts, std::vector<float>(
                           recorded.begin(),
                           recorded.begin() +
                               static_cast<std::ptrdiff_t>(controls_before)));

    // The bytes before the dispatch the gap starts in, then one EXIT.
    const std::uint64_t cut = original.offsets[timer];
    std::vector<char> gapped = read_file(gapped_path);
    std::vector<char> replayed = read_file(replayed_path);
    ASSERT_EQ(replayed.size(), cut + sizeof(log::RecordHeader));
    for (std::size_t i = 0; i < WALL_CLOCK_BYTES; ++i) {
      gapped[WALL_CLOCK_OFFSET + i] = 0;
      replayed[WALL_CLOCK_OFFSET + i] = 0;
    }
    EXPECT_TRUE(std::equal(replayed.begin(),
                           replayed.begin() + static_cast<std::ptrdiff_t>(cut),
                           gapped.begin()));
    const LogRecords rerecorded = read_records(replayed_path);
    ASSERT_EQ(rerecorded.headers.size(), timer + 1);
    EXPECT_EQ(rerecorded.headers.back().kind,
              static_cast<std::uint16_t>(EventKind::EXIT));
    EXPECT_EQ(rerecorded.headers.back().dispatch_index, dispatch);

    std::remove(gapped_path.c_str());
    std::remove(replayed_path.c_str());
  }
  std::remove(original_path.c_str());
}

// A log that ends without its EXIT (a power loss cut it inside a record, or
// its writer was killed) may end inside its last dispatch, as a gap may split
// one: every dispatch before that one replays and matches the recorded run,
// that dispatch's recorded part does not replay, and the replay names it,
// with no divergence for the sends the cut lost.
// REQ-OPS-13
TEST(Replay, ALogCutInsideItsLastDispatchReplaysTheDispatchesBeforeIt) {
  const std::string path = temp_path("truncated");
  const std::string cut_path = temp_path("truncated_cut");
  const std::vector<float> recorded = record_run(path, 2.5F);
  const LogRecords original = read_records(path);
  const auto is = [&](std::size_t i, EventKind kind) {
    return original.headers[i].kind == static_cast<std::uint16_t>(kind);
  };

  // The last control send: the cut falls inside it, after its dispatch's
  // TIMER record.
  std::size_t send = original.headers.size() - 1;
  while (send > 0 && !is(send, EventKind::SEND)) --send;
  ASSERT_GT(send, 0u);
  ASSERT_TRUE(is(send - 1, EventKind::TIMER));
  const std::uint64_t dispatch = original.headers[send].dispatch_index;
  ASSERT_EQ(original.headers[send - 1].dispatch_index, dispatch);
  std::size_t controls_before = 0;
  for (std::size_t i = 0; i + 1 < send; ++i) {
    if (is(i, EventKind::TIMER)) ++controls_before;
  }
  ASSERT_GT(controls_before, 0u);
  {
    const std::vector<char> bytes = read_file(path);
    std::ofstream out{cut_path, std::ios::binary};
    out.write(bytes.data(),
              static_cast<std::streamsize>(original.offsets[send] + 10));
  }

  log::LogReader reader{cut_path};
  ReplayEventLoop<> loop{reader};
  TestRobot<ReplayEventLoop<>> robot{loop, test_topics(), 2.5F};
  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);
  loop.run();

  EXPECT_TRUE(reader.truncated());
  EXPECT_FALSE(loop.diverged());
  EXPECT_FALSE(loop.reached_exit());
  EXPECT_FALSE(loop.stopped_at_gap().has_value());
  ASSERT_TRUE(loop.ended_in_dispatch().has_value());
  EXPECT_EQ(loop.ended_in_dispatch().value(), dispatch);
  EXPECT_EQ(robot.efforts(),
            std::vector<float>(recorded.begin(),
                               recorded.begin() + static_cast<std::ptrdiff_t>(
                                                      controls_before)));

  std::remove(path.c_str());
  std::remove(cut_path.c_str());
}

// A later segment of a rotated log starts mid-run: replayed as a run, it
// would hand the program inputs out of their context. Exact replay refuses it
// before anything runs, naming the log and the dispatch it starts at.
// REQ-OPS-13
TEST(Replay, RefusesALogThatDoesNotStartAtItsFirstDispatch) {
  const std::string path = temp_path("rotated");
  // Room for a few dozen records in each of eight segments.
  const std::uint64_t prefix = [&] {
    const std::string whole = temp_path("rotated_whole");
    record_run(whole, 2.5F);
    const std::uint64_t first = read_records(whole).offsets.front();
    std::remove(whole.c_str());
    return first;
  }();
  record_run(path, 2.5F, Delivery::POLL,
             log::LogWriterOptions{.background = false,
                                   .max_bytes = 8 * (prefix + 2048)});
  const std::vector<std::string> segments = log::LogSegmentFiles(path);
  ASSERT_GE(segments.size(), 2u);
  ASSERT_EQ(segments.back(), path);

  std::uint64_t first_dispatch = 0;
  for (const log::RecordHeader& header : read_records(path).headers) {
    if (header.dispatch_index != 0) {
      first_dispatch = header.dispatch_index;
      break;
    }
  }
  ASSERT_GT(first_dispatch, 1u);

  log::LogReader reader{path};
  try {
    ReplayEventLoop<> loop{reader};
    ADD_FAILURE() << "replay accepted a log starting at dispatch "
                  << first_dispatch;
  } catch (const ReplayError& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find(path), std::string::npos) << what;
    EXPECT_NE(what.find("dispatch " + std::to_string(first_dispatch)),
              std::string::npos)
        << what;
  }

  for (const std::string& segment : segments) std::remove(segment.c_str());
  std::remove(log::CappedMarkerPath(path).c_str());
}

// Sends three commands from its constructor, outside any dispatch, as a
// node announcing its initial state does, then ticks.
template <typename Loop>
struct StartupSender {
  Sender<Loop, EventsTest::CommandMessage> command;
  Timer<Loop> tick;
  int ticks{0};

  explicit StartupSender(Loop& loop)
      : command{make_sender<EventsTest::CommandMessage>(loop,
                                                        "/replay/startup")},
        tick{make_timer<&StartupSender::on_tick>(loop, "tick", this)} {
    for (int i = 0; i < 3; ++i) {
      command.send(EventsTest::CommandMessage{i, static_cast<float>(i), 1});
    }
    tick.setup_periodic(MonotonicTime::from_nanos(1'000'000), 1ms);
  }
  void on_tick(const Context&) { ++ticks; }
};

// A gap that starts outside any dispatch (a constructor's sends dropped) is a
// gap like any: the replayed constructor's sends past it are not divergences,
// no dispatch replays, and the stop names the gap.
// REQ-OPS-13
TEST(Replay, AGapOutsideAnyDispatchStopsBeforeTheFirstDispatch) {
  const std::string original_path = temp_path("startup_original");
  const std::string gapped_path = temp_path("startup_gapped");
  {
    SimulationEnvironment environment;
    SimulatedEventLoop<log::LogWriter> loop{
        environment, log::LogWriter{original_path, "startup"}};
    StartupSender program{loop};
    loop.run_for(3ms);
    loop.finish();
    ASSERT_FALSE(loop.recorder().failed()) << loop.recorder().error();
    ASSERT_GT(program.ticks, 0);
  }
  const LogRecords original = read_records(original_path);
  ASSERT_GE(original.headers.size(), 4u);
  for (std::size_t i = 0; i < 3; ++i) {
    ASSERT_EQ(original.headers[i].kind,
              static_cast<std::uint16_t>(EventKind::SEND));
    ASSERT_EQ(original.headers[i].dispatch_index, 0u);
  }
  // The second send dropped; capture resumes at the third.
  const log::LogGap gap = log::CopyWithGap(original_path, gapped_path, 1, 2);
  ASSERT_EQ(gap.first_dispatch_index, 0u);

  log::LogReader reader{gapped_path};
  ReplayEventLoop<> loop{reader};
  std::optional<StartupSender<ReplayEventLoop<>>> program;
  EXPECT_NO_THROW(program.emplace(loop));
  ASSERT_TRUE(program.has_value());
  EXPECT_NO_THROW(loop.run());
  EXPECT_FALSE(loop.diverged());
  EXPECT_FALSE(loop.reached_exit());
  ASSERT_TRUE(loop.stopped_at_gap().has_value());
  EXPECT_EQ(loop.stopped_at_gap().value().start_ns, gap.start_ns);
  EXPECT_EQ(program->ticks, 0);
  std::remove(original_path.c_str());
  std::remove(gapped_path.c_str());
}

// A log cut while its program was constructed (inside a send from its
// constructor, so with no record at all or some of those sends) ends before
// the first dispatch, as a gap there does: the replayed constructor's sends
// past the cut are not divergences, no dispatch replays, and the replay
// names the cut.
// REQ-OPS-13
TEST(Replay, ALogCutDuringConstructionEndsBeforeTheFirstDispatch) {
  const std::string original_path = temp_path("startup_cut_original");
  {
    SimulationEnvironment environment;
    SimulatedEventLoop<log::LogWriter> loop{
        environment, log::LogWriter{original_path, "startup"}};
    StartupSender program{loop};
    loop.run_for(3ms);
    loop.finish();
    ASSERT_FALSE(loop.recorder().failed()) << loop.recorder().error();
  }
  const LogRecords original = read_records(original_path);
  ASSERT_GE(original.headers.size(), 4u);
  ASSERT_EQ(original.headers[1].dispatch_index, 0u);
  const std::vector<char> bytes = read_file(original_path);
  // Cut inside the first send, then inside the second.
  for (const std::size_t send : {std::size_t{0}, std::size_t{1}}) {
    SCOPED_TRACE(send == 0 ? "no record" : "one of three sends");
    const std::string cut_path = temp_path("startup_cut");
    {
      std::ofstream out{cut_path, std::ios::binary | std::ios::trunc};
      out.write(bytes.data(),
                static_cast<std::streamsize>(original.offsets[send] + 10));
    }
    log::LogReader reader{cut_path};
    ReplayEventLoop<> loop{reader};
    std::optional<StartupSender<ReplayEventLoop<>>> program;
    EXPECT_NO_THROW(program.emplace(loop));
    ASSERT_TRUE(program.has_value());
    EXPECT_NO_THROW(loop.run());
    EXPECT_FALSE(loop.diverged());
    EXPECT_FALSE(loop.reached_exit());
    ASSERT_TRUE(loop.ended_in_dispatch().has_value());
    EXPECT_EQ(loop.ended_in_dispatch().value(), 0u);
    EXPECT_EQ(program->ticks, 0);
    std::remove(cut_path.c_str());
  }
  std::remove(original_path.c_str());
}

// Records lost between intact ones would replay a run the program never
// made, so exact replay refuses the log (input replay stops at them or
// refuses them).
// REQ-OPS-13
TEST(Replay, RefusesALogMissingADamagedRecord) {
  const std::string path = temp_path("damaged");
  record_run(path, 2.5F);

  std::vector<char> bytes = read_file(path);
  std::array<char, sizeof(log::RECORD_MAGIC)> magic{};
  std::memcpy(magic.data(), &log::RECORD_MAGIC, magic.size());
  std::vector<std::size_t> records;
  for (auto at = bytes.begin();
       (at = std::search(at, bytes.end(), magic.begin(), magic.end())) !=
       bytes.end();
       ++at) {
    records.push_back(static_cast<std::size_t>(at - bytes.begin()));
  }
  ASSERT_GT(records.size(), 4u);
  bytes[records[records.size() / 2] +
        offsetof(log::RecordHeader, dispatch_index)] ^= 1;
  {
    std::ofstream out{path, std::ios::binary | std::ios::trunc};
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }

  log::LogReader reader{path};
  try {
    ReplayEventLoop<> loop{reader};
    ADD_FAILURE() << "replay accepted a log missing damaged records";
  } catch (const ReplayError& e) {
    EXPECT_NE(std::string{e.what()}.find(path), std::string::npos) << e.what();
  }
  EXPECT_EQ(reader.damage().spans, 1u);

  std::remove(path.c_str());
}

// Mostly trailing zeros, like a hardware packet: the log stores only the
// leading bytes of each one. A framework-style byte layout, so it carries a
// hand-made identity rather than a generated descriptor.
struct Padded {
  std::array<std::byte, 256> bytes{};
};

const talos::schema::Identity kPaddedSchema{"replay_test.Padded", 0x9add};

// Echoes every message it receives, with `mark` in the last byte.
template <typename Loop>
struct PaddedEcho {
  Loop& loop;
  std::byte mark{0};
  std::uint16_t sender{0};
  std::vector<std::vector<std::byte>> received;

  explicit PaddedEcho(Loop& event_loop) : loop{event_loop} {
    loop.register_watcher("/replay/padded_in", sizeof(Padded), alignof(Padded),
                          kPaddedSchema, make_thunk<&PaddedEcho::echo>(this));
    sender = loop.register_sender("/replay/padded_out", sizeof(Padded),
                                  alignof(Padded), kPaddedSchema);
  }

  void echo(const Context&, std::span<const std::byte> payload) {
    received.emplace_back(payload.begin(), payload.end());
    // A Padded, whatever the payload's size: a fixed-size copy has a last
    // byte to mark, where a copy of an empty payload would not.
    Padded out{};
    std::copy_n(payload.begin(), std::min(payload.size(), out.bytes.size()),
                out.bytes.begin());
    out.bytes.back() = mark;
    loop.send_message(sender, std::as_bytes(std::span{&out, std::size_t{1}}));
  }
};

// Replay restores the padding the log left out: handlers see the full
// recorded bytes, sends are checked against all of them, and the replay
// re-records the identical log.
TEST(Replay, RestoresTrailingZerosTheLogDoesNotStore) {
  const auto original_path = temp_path("padded_a");
  const auto replayed_path = temp_path("padded_b");

  std::vector<std::vector<std::byte>> recorded;
  {
    SimulationEnvironment environment;
    SimulatedEventLoop<log::LogWriter> loop{
        environment, log::LogWriter{original_path, "padded"}};
    PaddedEcho echo{loop};
    for (int i = 0; i < 4; ++i) {
      Padded message{};
      message.bytes[0] = static_cast<std::byte>(i);  // a leading zero at i == 0
      message.bytes[2] = std::byte{9};
      if (i == 3) message.bytes[200] = std::byte{7};
      loop.inject("/replay/padded_in", message);
      loop.run_for(1ms);
    }
    loop.finish();
    ASSERT_FALSE(loop.recorder().failed()) << loop.recorder().error();
    recorded = echo.received;
  }
  ASSERT_EQ(recorded.size(), 4u);

  {
    log::LogReader reader{original_path};
    log::LogReader::Record record;
    std::size_t trimmed = 0;
    while (reader.next(record)) {
      if (record.header.stored_bytes < record.header.payload_bytes) ++trimmed;
    }
    EXPECT_EQ(trimmed, 8u) << "every message and echo should drop padding";
  }

  {
    log::LogReader reader{original_path};
    ReplayEventLoop<log::LogWriter> loop{
        reader, log::LogWriter{replayed_path, "padded"}};
    PaddedEcho echo{loop};
    loop.run();
    EXPECT_FALSE(loop.diverged());
    EXPECT_EQ(echo.received, recorded);
  }

  auto original = read_file(original_path);
  auto replayed = read_file(replayed_path);
  ASSERT_EQ(original.size(), replayed.size());
  for (std::size_t i = 0; i < WALL_CLOCK_BYTES; ++i) {
    original[WALL_CLOCK_OFFSET + i] = 0;
    replayed[WALL_CLOCK_OFFSET + i] = 0;
  }
  EXPECT_EQ(original, replayed);

  // A byte the log left out is still compared: writing into the padding is a
  // different send.
  log::LogReader reader{original_path};
  ReplayEventLoop<> loop{reader};
  PaddedEcho echo{loop};
  echo.mark = std::byte{1};
  EXPECT_THROW(loop.run(), ReplayDivergence);

  std::remove(original_path.c_str());
  std::remove(replayed_path.c_str());
}

// Watches a sequence topic, then on its timer tick publishes a summary of the
// messages drained before the tick and exits.
template <typename Loop>
struct BacklogProgram {
  Loop& loop;
  std::uint16_t sender{0};
  int received{0};
  std::int32_t last_value{0};
  int observed_count{0};
  std::int32_t observed_value{0};

  BacklogProgram(Loop& event_loop, std::string_view in_topic,
                 std::string_view out_topic)
      : loop{event_loop} {
    watch<EventsTest::SensorMessage, &BacklogProgram::on_sensor>(loop, in_topic,
                                                                 this);
    sender =
        loop.register_sender(out_topic, sizeof(EventsTest::CommandMessage),
                             alignof(EventsTest::CommandMessage),
                             talos::schema::Of<EventsTest::CommandMessage>());
    const auto timer =
        loop.register_timer("tick", make_thunk<&BacklogProgram::on_tick>(this));
    loop.arm_timer(timer, loop.monotonic_now(), Duration::zero());
  }

  void on_sensor(const Context&, const EventsTest::SensorMessage& message) {
    last_value = message.sensor_id();
    ++received;
  }

  void on_tick(const Context&) {
    observed_count = received;
    observed_value = last_value;
    const EventsTest::CommandMessage command{
        observed_value, static_cast<float>(observed_count), 1};
    loop.send_message(sender,
                      std::as_bytes(std::span{&command, std::size_t{1}}));
    loop.exit();
  }
};

// A .tlog recorded by RealtimeEventLoop when > 64 queued sequence messages are
// drained across multiple passes before a single timer tick replays cleanly
// through ReplayEventLoop and re-records byte-identically.
// REQ-OPS-13
// REQ-SAFE-01
// REQ-RT-04
TEST(Replay, MultiPassPreTimerBacklogRecordedOnRealtimeLoopReplaysCleanly) {
  constexpr int kQueued = 80;  // over default max_messages_per_tick (64)
  constexpr auto kHangBound = 10s;
  const std::string pid = std::to_string(::getpid());
  const std::string in_topic = "/rpb_" + pid;
  const std::string out_topic = "/rpo_" + pid;
  const std::string original_path = temp_path("rt_backlog_a");
  const std::string replayed_path = temp_path("rt_backlog_b");

  {
    using RecordingLoop = RealtimeEventLoop<log::LogWriter>;
    RecordingLoop::Options options;
    options.delivery = Delivery::BEFORE_TIMERS;
    RecordingLoop loop{log::LogWriter{original_path, "backlog"}, options};
    BacklogProgram program{loop, in_topic, out_topic};

    RTMSQueue publisher{in_topic,
                        sizeof(EventsTest::SensorMessage),
                        alignof(EventsTest::SensorMessage),
                        talos::schema::Of<EventsTest::SensorMessage>(),
                        MAX_SLOTS,
                        RTMSOptions{.publisher = true}};
    for (int i = 1; i <= kQueued; ++i) {
      const EventsTest::SensorMessage message{i, 0.0F, 0.0F};
      ASSERT_EQ(
          publisher
              .write(std::as_bytes(
                  std::span<const EventsTest::SensorMessage, 1>{&message, 1}))
              .result,
          WriteResult::SUCCESS);
    }

    loop.run_for(kHangBound);  // Bound only; on_tick exits.
    ASSERT_FALSE(loop.recorder().failed()) << loop.recorder().error();
    EXPECT_EQ(loop.activity().drains, 2u);  // 64 + 16 before the single tick
    EXPECT_EQ(program.observed_count, kQueued);
    EXPECT_EQ(program.observed_value, kQueued);
  }

  // Verify the recorded log captured all 80 MESSAGE dispatches ahead of the
  // single TIMER dispatch.
  {
    const LogRecords recorded = read_records(original_path);
    int messages_before_timer = 0;
    int timer_count = 0;
    for (const log::RecordHeader& header : recorded.headers) {
      const auto kind = static_cast<EventKind>(header.kind);
      if (kind == EventKind::MESSAGE && timer_count == 0) {
        ++messages_before_timer;
      } else if (kind == EventKind::TIMER) {
        ++timer_count;
      }
    }
    EXPECT_EQ(messages_before_timer, kQueued);
    EXPECT_EQ(timer_count, 1);
  }

  {
    log::LogReader reader{original_path};
    ReplayEventLoop<log::LogWriter> replay{
        reader, log::LogWriter{replayed_path, "backlog"}};
    BacklogProgram program{replay, in_topic, out_topic};
    replay.run();
    EXPECT_FALSE(replay.diverged());
    EXPECT_TRUE(replay.reached_exit());
    EXPECT_EQ(program.observed_count, kQueued);
    EXPECT_EQ(program.observed_value, kQueued);
    ASSERT_FALSE(replay.recorder().failed()) << replay.recorder().error();
  }

  std::vector<char> original = read_file(original_path);
  std::vector<char> replayed = read_file(replayed_path);
  ASSERT_FALSE(original.empty());
  ASSERT_EQ(original.size(), replayed.size());
  for (std::size_t i = 0; i < WALL_CLOCK_BYTES; ++i) {
    original[WALL_CLOCK_OFFSET + i] = 0;
    replayed[WALL_CLOCK_OFFSET + i] = 0;
  }
  EXPECT_EQ(original, replayed);

  std::remove(original_path.c_str());
  std::remove(replayed_path.c_str());
}

}  // namespace
}  // namespace talos::event

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "talOS/events/events_test_message_generated.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
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
std::vector<float> record_run(const std::string& path, float gain) {
  SimulationEnvironment environment;
  SimulatedEventLoop<log::LogWriter> loop{environment,
                                          log::LogWriter{path, "test_robot"}};

  TestRobot<SimulatedEventLoop<log::LogWriter>> robot{loop, test_topics(),
                                                      gain};

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

TEST(Replay, ReproducesRecordedBehaviour) {
  const std::string path = temp_path("reproduce");
  const std::vector<float> recorded = record_run(path, 2.5F);
  ASSERT_FALSE(recorded.empty());

  log::LogReader reader{path};
  ReplayEventLoop<> loop{reader};
  TestRobot<ReplayEventLoop<>> robot{loop, test_topics(), 2.5F};
  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);

  loop.run();

  EXPECT_FALSE(loop.diverged());
  EXPECT_TRUE(loop.reached_exit());
  EXPECT_EQ(robot.efforts(), recorded);
  EXPECT_EQ(robot.control_count(), recorded.size());

  std::remove(path.c_str());
}

// The core guarantee: replaying a log and recording that replay produces the
// same log again, byte for byte. If any input were left out of the log, or any
// handler read something the loop did not record, this test would fail.
TEST(Replay, ReRecordingIsByteIdentical) {
  const std::string original_path = temp_path("identical_a");
  const std::string replayed_path = temp_path("identical_b");

  record_run(original_path, 1.75F);

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

TEST(Replay, DetectsChangedBehaviour) {
  const std::string path = temp_path("divergence");
  record_run(path, 2.5F);

  log::LogReader reader{path};
  ReplayEventLoop<> loop{reader};

  // Same program, different gain: the first control cycle that acts on a
  // non-zero error must produce different bytes.
  TestRobot<ReplayEventLoop<>> robot{loop, test_topics(), 9.0F};
  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);

  EXPECT_THROW(loop.run(), ReplayDivergence);

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
  EXPECT_NE(loop.divergences().front().detail.find("sent different bytes"),
            std::string::npos);

  std::remove(path.c_str());
}

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

  EXPECT_THROW(loop.run(), ReplayError);

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

TEST(Replay, ValidatesTimerOperationsAndHandlerExit) {
  const auto path = temp_path("timer_operations");
  record_timer_program(path);
  log::LogReader reader{path};
  ReplayEventLoop<> loop{reader};
  TimerProgram program{loop};
  EXPECT_NO_THROW(loop.run());
  EXPECT_FALSE(loop.diverged());
  EXPECT_TRUE(loop.reached_exit());
  EXPECT_EQ(program.calls, 1);
  std::remove(path.c_str());
}

TEST(Replay, TimerOperationsReRecordIdentically) {
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

TEST(Replay, DetectsChangedPeriodSkippedDisableAndMissingExit) {
  const auto path = temp_path("timer_changes");
  record_timer_program(path);
  for (int change = 0; change < 3; ++change) {
    log::LogReader reader{path};
    ReplayEventLoop<> loop{reader};
    TimerProgram program{loop};
    if (change == 0) program.period = 3ms;
    if (change == 1) program.disable = false;
    if (change == 2) program.stop = false;
    EXPECT_THROW(loop.run(), ReplayDivergence) << "change " << change;
  }
  std::remove(path.c_str());
}

TEST(Replay, RejectsChangedOrMissingInitialSchedule) {
  const auto path = temp_path("initial_schedule");
  record_timer_program(path);
  for (int change = 0; change < 3; ++change) {
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
    EXPECT_THROW(loop.run(), ReplayError) << "change " << change;
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

// A log that stops mid-record still replays everything it did capture. A robot
// that lost power should not cost you the whole match.
TEST(Replay, ReplaysTheCompletePrefixOfATruncatedLog) {
  const std::string path = temp_path("truncated");
  const std::vector<float> recorded = record_run(path, 2.5F);

  std::vector<char> bytes = read_file(path);
  ASSERT_GT(bytes.size(), 200u);

  const std::string truncated_path = temp_path("truncated_cut");
  {
    std::ofstream out{truncated_path, std::ios::binary};
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size() - 37));
  }

  log::LogReader reader{truncated_path};
  ReplayEventLoop<> loop{reader};
  TestRobot<ReplayEventLoop<>> robot{loop, test_topics(), 2.5F};
  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);

  loop.run();

  EXPECT_FALSE(loop.diverged());
  EXPECT_FALSE(loop.reached_exit());
  EXPECT_TRUE(reader.truncated());

  ASSERT_FALSE(robot.efforts().empty());
  EXPECT_LE(robot.efforts().size(), recorded.size());

  // Everything that did replay matches what was originally computed.
  for (std::size_t i = 0; i < robot.efforts().size(); ++i) {
    EXPECT_FLOAT_EQ(robot.efforts()[i], recorded[i]) << "at cycle " << i;
  }

  std::remove(path.c_str());
  std::remove(truncated_path.c_str());
}

}  // namespace
}  // namespace talos::event

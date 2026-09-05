#include "talOS/events/simulated_event_loop.h"

#include <gtest/gtest.h>

#include <type_traits>
#include <vector>

#include "talOS/events/events_test_message_generated.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/test_robot.h"

namespace talos::event {
namespace {

using namespace std::chrono_literals;
using testing::TestRobot;
using testing::Topics;

Topics simulated_topics() {
  return Topics{"/sim/sensor", "/sim/setpoint", "/sim/command"};
}

// The user's latency requirement, enforced by the compiler rather than by
// review: none of the loops may grow a vtable.
TEST(LoopShape, NoVirtualDispatch) {
  EXPECT_FALSE(std::is_polymorphic_v<SimulatedEventLoop<>>);
  EXPECT_FALSE(std::is_polymorphic_v<RealtimeEventLoop<>>);
  EXPECT_FALSE(std::is_polymorphic_v<ReplayEventLoop<>>);
  EXPECT_FALSE(std::is_polymorphic_v<Thunk>);
  EXPECT_FALSE(std::is_polymorphic_v<Scheduler>);
}

TEST(SimulatedEventLoop, RegistrationAssignsIdsInOrder) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  TestRobot<SimulatedEventLoop<>> robot{loop, simulated_topics(), 2.0F};

  const Manifest& manifest = loop.manifest();
  ASSERT_EQ(manifest.size(), 4u);

  EXPECT_EQ(manifest[0].kind, SourceKind::WATCHER);
  EXPECT_EQ(manifest[0].name, "/sim/sensor");
  EXPECT_EQ(manifest[1].kind, SourceKind::FETCHER);
  EXPECT_EQ(manifest[2].kind, SourceKind::SENDER);
  EXPECT_EQ(manifest[3].kind, SourceKind::TIMER);
  EXPECT_EQ(manifest[3].name, "control");

  for (std::size_t i = 0; i < manifest.size(); ++i) {
    EXPECT_EQ(manifest[i].id, i);
  }
}

TEST(SimulatedEventLoop, RegistrationAfterRunIsRejected) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  TestRobot<SimulatedEventLoop<>> robot{loop, simulated_topics(), 1.0F};

  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);
  loop.run_for(5ms);

  EXPECT_THROW(loop.register_timer("late", Thunk{}), RegistrationError);
}

TEST(SimulatedEventLoop, PeriodicTimerFiresOnSchedule) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  TestRobot<SimulatedEventLoop<>> robot{loop, simulated_topics(), 1.0F};

  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);
  loop.run_for(10ms);

  // Fires at 1ms through 10ms inclusive.
  EXPECT_EQ(robot.control_count(), 10u);

  // Virtual time never overruns, so every cycle is exactly on time.
  for (const float effort : robot.efforts()) {
    EXPECT_FALSE(std::isnan(effort));
  }
}

TEST(SimulatedEventLoop, WatcherReceivesEveryInjectedMessage) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  TestRobot<SimulatedEventLoop<>> robot{loop, simulated_topics(), 1.0F};

  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);

  constexpr int messages = 25;
  for (int i = 0; i < messages; ++i) {
    loop.inject(simulated_topics().sensor,
                EventsTest::SensorMessage{i, static_cast<float>(i), 0.0F});
  }

  loop.run_for(10ms);

  EXPECT_EQ(robot.sensor_count(), messages);
  EXPECT_EQ(robot.dropped_sensors(), 0u);
}

TEST(SimulatedEventLoop, FetcherTakesTheNewestValueOnly) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  TestRobot<SimulatedEventLoop<>> robot{loop, simulated_topics(), 1.0F};

  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);

  for (int i = 1; i <= 5; ++i) {
    loop.inject(simulated_topics().setpoint,
                EventsTest::SetpointMessage{i, static_cast<float>(i)});
  }

  loop.run_for(2ms);

  ASSERT_FALSE(robot.efforts().empty());

  // Position is zero and no sensor arrived, so effort is gain * target. The
  // fetcher must have skipped straight to the last setpoint, 5.
  EXPECT_FLOAT_EQ(robot.efforts().front(), 5.0F);
}

TEST(SimulatedEventLoop, ExitStopsTheLoop) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};

  int calls = 0;
  auto handler = [&loop, &calls](const Context&) {
    ++calls;
    if (calls == 3) {
      loop.exit();
    }
  };

  const std::uint16_t id = loop.register_timer("stopper", make_thunk(&handler));
  Timer<SimulatedEventLoop<>>{&loop, id}.setup_periodic(
      MonotonicTime::from_nanos(1'000'000), 1ms);

  loop.run_for(100ms);
  EXPECT_EQ(calls, 3);
}

// On the virtual clock every deadline is met exactly, which makes this the one
// place the metrics can be asserted against known-good numbers instead of
// whatever the machine happened to do.
TEST(SimulatedEventLoop, MetricsRecordAnIdealSchedule) {
  using Loop = SimulatedEventLoop<NullRecorder, LoopMetrics>;

  SimulationEnvironment environment;
  Loop loop{environment};
  TestRobot<Loop> robot{loop, simulated_topics(), 1.0F};

  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);

  for (int i = 0; i < 20; ++i) {
    loop.inject(simulated_topics().sensor,
                EventsTest::SensorMessage{i, static_cast<float>(i), 0.0F});
    loop.run_for(1ms);
  }
  loop.finish();

  const std::uint16_t timer_id = robot.control_timer().id();
  const SourceMetrics& timer = loop.metrics().source(timer_id);

  EXPECT_EQ(timer.dispatches, 20u);

  // Virtual time jumps exactly to each deadline, so there is no latency and no
  // jitter at all. Any non-zero value here would mean the simulated clock had
  // developed an opinion of its own.
  EXPECT_EQ(timer.latency.max_ns(), 0);
  EXPECT_EQ(timer.latency.jitter_ns(), 0);
  EXPECT_EQ(timer.overruns, 0u);

  // Intervals are exactly the 1 ms period, one fewer sample than dispatches.
  EXPECT_EQ(timer.interval.count(), timer.dispatches - 1);
  EXPECT_EQ(timer.interval.min_ns(), 1'000'000);
  EXPECT_EQ(timer.interval.max_ns(), 1'000'000);

  const SourceMetrics& watcher = loop.metrics().source(0);
  EXPECT_EQ(watcher.dispatches, 20u);
  EXPECT_EQ(watcher.dropped, 0u);
}

// Same program, same injected inputs, same everything. This is the property
// the whole logging design rests on.
TEST(SimulatedEventLoop, TwoRunsProduceIdenticalBehaviour) {
  const auto run = [] {
    SimulationEnvironment environment;
    SimulatedEventLoop<> loop{environment};
    TestRobot<SimulatedEventLoop<>> robot{loop, simulated_topics(), 3.5F};

    robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                         1ms);

    for (int i = 0; i < 40; ++i) {
      loop.inject(simulated_topics().sensor,
                  EventsTest::SensorMessage{i, static_cast<float>(i) * 0.5F,
                                            static_cast<float>(i) * 0.1F});
      loop.inject(simulated_topics().setpoint,
                  EventsTest::SetpointMessage{i, static_cast<float>(i)});
      loop.run_for(1ms);
    }

    loop.finish();
    return robot.efforts();
  };

  const std::vector<float> first = run();
  const std::vector<float> second = run();

  ASSERT_FALSE(first.empty());
  EXPECT_EQ(first, second);
}

}  // namespace
}  // namespace talos::event

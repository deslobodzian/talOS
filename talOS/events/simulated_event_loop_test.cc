#include "talOS/events/simulated_event_loop.h"

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
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
  EXPECT_EQ(manifest[1].kind, SourceKind::WATCHER);
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

// The simulated loop is quiescent between run_for calls, so the harness may
// re-arm a timer there in the same way it injects a message: it is the outside
// world acting, not the program, and only the resulting firings are recorded.
TEST(SimulatedEventLoop, StepsAllowTimerChangesBetweenRuns) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  TestRobot<SimulatedEventLoop<>> robot{loop, simulated_topics(), 1.0F};

  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);
  loop.run_for(5ms);
  EXPECT_EQ(robot.control_count(), 5u);

  // Slow the timer down from outside, then step again.
  robot.control_timer().setup_periodic(loop.monotonic_now() + 2ms, 2ms);
  loop.run_for(10ms);
  EXPECT_EQ(robot.control_count(), 10u);

  robot.control_timer().disable();
  loop.run_for(10ms);
  EXPECT_EQ(robot.control_count(), 10u);
}

// A timer change from another thread has no reproducible position in the log
// and races the scheduler, so it is refused rather than silently accepted.
TEST(SimulatedEventLoop, TimerChangeFromAHandlerlessThreadDuringRunIsRejected) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};

  std::optional<std::logic_error> caught;
  auto handler = [&](const Context&) {
    std::thread other{[&] {
      try {
        loop.arm_timer(0, loop.monotonic_now() + 1ms, 1ms);
      } catch (const std::logic_error& error) {
        caught.emplace(error);
      }
    }};
    other.join();
    loop.exit();
  };

  const auto id = loop.register_timer("tick", make_thunk(&handler));
  loop.arm_timer(id, MonotonicTime::from_nanos(1'000'000), 1ms);
  loop.run_for(5ms);

  if (!caught.has_value()) {
    FAIL() << "no logic_error caught";
  }
  EXPECT_NE(std::string{caught->what()}.find("loop callback"),
            std::string::npos);
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

TEST(SimulatedEventLoop, SetpointWatcherKeepsTheNewestValue) {
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

  // Position is zero and no sensor arrived, so effort is gain * target, and
  // the target is the last setpoint delivered, 5.
  EXPECT_FLOAT_EQ(robot.efforts().back(), 5.0F);
}

// Under BEFORE_TIMERS a message injected before a timer's deadline reaches
// the handler that timer runs, and nothing drains in between; under POLL the
// timer at the same instant runs first and the drain follows it.
TEST(SimulatedEventLoop, BeforeTimersDrainsRightBeforeEachTimer) {
  const auto first_effort = [](Delivery delivery) {
    SimulationEnvironment environment;
    SimulatedEventLoop<> loop{environment};
    loop.set_delivery(delivery);
    TestRobot<SimulatedEventLoop<>> robot{loop, simulated_topics(), 1.0F};
    robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                         10ms);
    loop.inject(simulated_topics().setpoint,
                EventsTest::SetpointMessage{1, 5.0F});
    loop.run_for(25ms);
    EXPECT_EQ(robot.control_count(), 3u);
    if (delivery == Delivery::BEFORE_TIMERS) {
      EXPECT_EQ(loop.drains(), robot.control_count());
    } else {
      EXPECT_EQ(loop.drains(), 25u);
    }
    return robot.efforts().front();
  };
  EXPECT_FLOAT_EQ(first_effort(Delivery::BEFORE_TIMERS), 5.0F);
  EXPECT_FLOAT_EQ(first_effort(Delivery::POLL), 0.0F);
}

TEST(SimulatedEventLoop, DeliveryIsFixedOnceTheRunStarts) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  loop.run_for(1ms);
  EXPECT_THROW(loop.set_delivery(Delivery::BEFORE_TIMERS), std::logic_error);
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

#include "talOS/events/simulated_event_loop.h"

#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "events_test/events_test_message_generated.h"
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

TEST(SimulatedEventLoop, RegistrationAfterRunIsRejected) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  TestRobot<SimulatedEventLoop<>> robot{loop, simulated_topics(), 1.0F};

  robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                       1ms);
  loop.run_for(5ms);

  EXPECT_THROW(loop.register_timer("late", Thunk{}), RegistrationError);
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
}

TEST(SimulatedEventLoop, LatestSkipsBacklogButCallbacksKeepCappedSequence) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  std::vector<Context> latest;
  std::vector<std::int64_t> ordered;
  auto on_latest = [&](const Context& context) { latest.push_back(context); };
  auto on_message = [&](const Context&, std::span<const std::byte> bytes) {
    const auto* message =
        reinterpret_cast<const EventsTest::SensorMessage*>(bytes.data());
    ordered.push_back(message->sensor_id());
  };
  const auto schema = talos::schema::Of<EventsTest::SensorMessage>();
  const auto latest_id = loop.register_watcher(
      "/backlog", sizeof(EventsTest::SensorMessage),
      alignof(EventsTest::SensorMessage), schema, make_thunk(&on_latest), true);
  loop.register_watcher("/backlog", sizeof(EventsTest::SensorMessage),
                        alignof(EventsTest::SensorMessage), schema,
                        make_thunk(&on_message));
  loop.run_for(10ms);
  for (int i = 0; i < 130; ++i) {
    loop.inject("/backlog", EventsTest::SensorMessage{i, 0.0F, 0.0F});
  }

  loop.run_for(1ms);
  ASSERT_EQ(latest.size(), 1u);
  EXPECT_EQ(latest.front().sequence, 129u);
  EXPECT_EQ(latest.front().dropped, 129u);
  EXPECT_EQ(latest.front().now - latest.front().publish_time, 1ms);
  EXPECT_EQ(loop.source_counters()[latest_id].dropped.load(), 129u);
  ASSERT_EQ(ordered.size(), 64u);
  loop.run_for(1ms);
  EXPECT_EQ(ordered.size(), 128u);
  loop.run_for(1ms);
  ASSERT_EQ(ordered.size(), 130u);
  EXPECT_EQ(latest.size(), 1u);
  for (int i = 0; i < 130; ++i) EXPECT_EQ(ordered[i], i);
}

// Loops sharing an environment may be stepped to different times. A drain
// delivers only what was published by the loop's own now: a latest-only
// watcher takes the newest visible message (the skipped ones count as
// dropped), an ordered one stops at the first message from its future.
TEST(SimulatedEventLoop, DrainNeverDeliversAPublicationFromTheFuture) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> ahead{environment};
  SimulatedEventLoop<> behind{environment};
  std::vector<Context> latest;
  std::vector<Context> ordered;
  auto on_latest = [&](const Context& context) { latest.push_back(context); };
  auto on_message = [&](const Context& context, std::span<const std::byte>) {
    ordered.push_back(context);
  };
  const auto schema = talos::schema::Of<EventsTest::SensorMessage>();
  const auto latest_id = behind.register_watcher(
      "/future", sizeof(EventsTest::SensorMessage),
      alignof(EventsTest::SensorMessage), schema, make_thunk(&on_latest), true);
  behind.register_watcher("/future", sizeof(EventsTest::SensorMessage),
                          alignof(EventsTest::SensorMessage), schema,
                          make_thunk(&on_message));

  const MonotonicTime origin{};
  ahead.run_until(origin + 1ms);
  ahead.inject("/future", EventsTest::SensorMessage{0, 0.0F, 0.0F});
  ahead.inject("/future", EventsTest::SensorMessage{1, 0.0F, 0.0F});
  ahead.run_until(origin + 3ms);
  ahead.inject("/future", EventsTest::SensorMessage{2, 0.0F, 0.0F});

  behind.run_until(origin + 2ms);
  ASSERT_EQ(latest.size(), 1u);
  EXPECT_EQ(latest[0].sequence, 1u);
  EXPECT_EQ(latest[0].dropped, 1u);
  EXPECT_EQ(latest[0].now, origin + 1ms);
  ASSERT_EQ(ordered.size(), 2u);
  EXPECT_EQ(ordered[1].sequence, 1u);
  for (const Context& context : ordered) {
    EXPECT_LE(context.publish_time, context.now);
  }

  behind.run_until(origin + 3ms);
  ASSERT_EQ(latest.size(), 2u);
  EXPECT_EQ(latest[1].sequence, 2u);
  EXPECT_EQ(latest[1].dropped, 0u);
  EXPECT_EQ(latest[1].now, latest[1].publish_time);
  ASSERT_EQ(ordered.size(), 3u);
  EXPECT_EQ(ordered[2].sequence, 2u);
  EXPECT_EQ(ordered[2].dropped, 0u);
  EXPECT_EQ(behind.source_counters()[latest_id].dropped.load(), 1u);
}

TEST(SimulatedEventLoop, OverflowMatchesRtmsRetainedWindowAndDropCounts) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  std::vector<Context> received;
  std::vector<std::int64_t> values;
  auto on_message = [&](const Context& context,
                        std::span<const std::byte> bytes) {
    received.push_back(context);
    values.push_back(
        reinterpret_cast<const EventsTest::SensorMessage*>(bytes.data())
            ->sensor_id());
  };
  const auto id = loop.register_watcher(
      "/overflow", sizeof(EventsTest::SensorMessage),
      alignof(EventsTest::SensorMessage),
      talos::schema::Of<EventsTest::SensorMessage>(), make_thunk(&on_message));
  constexpr std::size_t count = MAX_SLOTS + 100;
  for (std::size_t i = 0; i < count; ++i) {
    loop.inject("/overflow",
                EventsTest::SensorMessage{static_cast<std::int32_t>(i), 0, 0});
  }
  loop.run_for(1ms);
  ASSERT_EQ(received.size(), 64u);
  EXPECT_EQ(received.front().sequence, 101u);
  EXPECT_EQ(received.front().dropped, 101u);
  EXPECT_EQ(loop.source_counters()[id].dropped.load(), 101u);
  loop.run_for(20ms);
  ASSERT_EQ(values.size(), MAX_SLOTS - 1);
  for (std::size_t i = 0; i < values.size(); ++i) {
    EXPECT_EQ(values[i], static_cast<std::int64_t>(i + 101));
    EXPECT_EQ(received[i].publish_time, MonotonicTime{});
    if (i != 0) {
      EXPECT_EQ(received[i].dropped, 0u);
    }
  }
  auto& channel =
      environment.channel("/overflow", sizeof(EventsTest::SensorMessage));
  EXPECT_THROW(channel.publish_time(100), std::out_of_range);
}

TEST(SimulatedEventLoop, PausedClockCoalescesMissedTimersOnOriginalPhase) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  std::vector<Context> ticks;
  auto on_tick = [&](const Context& context) { ticks.push_back(context); };
  const auto id = loop.register_timer("tick", make_thunk(&on_tick));
  loop.arm_timer(id, MonotonicTime{} + 5ms, 5ms);
  loop.run_for(5ms);
  loop.advance_until(MonotonicTime{} + 28ms);
  loop.run_for(0ms);
  ASSERT_EQ(ticks.size(), 2u);
  EXPECT_EQ(ticks.back().now, MonotonicTime{} + 28ms);
  EXPECT_EQ(ticks.back().event_time, MonotonicTime{} + 10ms);
  EXPECT_EQ(ticks.back().sequence, 4u);
  loop.run_for(2ms);
  ASSERT_EQ(ticks.size(), 3u);
  EXPECT_EQ(ticks.back().now, MonotonicTime{} + 30ms);
  EXPECT_EQ(ticks.back().sequence, 1u);
}

// Under either delivery a message injected before a timer's deadline reaches
// the handler that timer runs, even when a POLL tick is due at the same
// instant (the tick loses the tie to the timer). BEFORE_TIMERS
// drains only there; POLL also drains on every 1 ms tick.
TEST(SimulatedEventLoop, EveryDeliveryDrainsRightBeforeEachTimer) {
  for (const auto delivery : {Delivery::BEFORE_TIMERS, Delivery::POLL}) {
    SCOPED_TRACE(delivery == Delivery::POLL ? "POLL" : "BEFORE_TIMERS");
    SimulationEnvironment environment;
    SimulatedEventLoop<> loop{environment};
    loop.set_delivery(delivery);
    TestRobot<SimulatedEventLoop<>> robot{loop, simulated_topics(), 1.0F};
    robot.control_timer().setup_periodic(MonotonicTime::from_nanos(1'000'000),
                                         10ms);
    loop.inject(simulated_topics().setpoint,
                EventsTest::SetpointMessage{1, 5.0F});
    loop.run_for(10ms);
    loop.inject(simulated_topics().setpoint,
                EventsTest::SetpointMessage{2, 7.0F});
    loop.run_for(15ms);
    ASSERT_EQ(robot.control_count(), 3u);
    EXPECT_EQ(loop.drains(), delivery == Delivery::POLL ? 25u + 3u : 3u);
    EXPECT_FLOAT_EQ(robot.efforts()[0], 5.0F);
    EXPECT_FLOAT_EQ(robot.efforts()[1], 7.0F);
  }
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

}  // namespace
}  // namespace talos::event

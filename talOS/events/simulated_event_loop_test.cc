#include "talOS/events/simulated_event_loop.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
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

// Under every delivery a message injected before a timer's deadline reaches
// the handler that timer runs, even when a poll tick is due at the same
// instant (the tick loses the tie to the timer). BEFORE_TIMERS drains only
// there; POLL also drains on every 1 ms tick. ON_PUBLISH with no prompt
// watcher (TestRobot marks none) has nothing to wake it and drains as
// BEFORE_TIMERS does, as the realtime loop would.
TEST(SimulatedEventLoop, EveryDeliveryDrainsRightBeforeEachTimer) {
  for (const auto delivery :
       {Delivery::BEFORE_TIMERS, Delivery::POLL, Delivery::ON_PUBLISH}) {
    SCOPED_TRACE(static_cast<int>(delivery));
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

// --- Delivery::ON_PUBLISH ---------------------------------------------------

using Sensor = EventsTest::SensorMessage;

std::uint16_t WatchSensor(SimulatedEventLoop<>& loop, std::string_view topic,
                          Thunk callback, bool latest_only = false) {
  return loop.register_watcher(topic, sizeof(Sensor), alignof(Sensor),
                               talos::schema::Of<Sensor>(), callback,
                               latest_only);
}

std::uint16_t SendSensor(SimulatedEventLoop<>& loop, std::string_view topic) {
  return loop.register_sender(topic, sizeof(Sensor), alignof(Sensor),
                              talos::schema::Of<Sensor>());
}

void Send(SimulatedEventLoop<>& loop, std::uint16_t sender, int id) {
  const Sensor message{id, 0.0F, 0.0F};
  loop.send_message(sender, std::as_bytes(std::span{&message, 1}));
}

// What one handler saw, and on which topic.
struct Heard {
  std::string topic;
  Context context;
};

// The realtime loop is woken by a prompt publication; the simulated one
// treats it as an event at its publication time. Its handler runs at that
// instant with nothing to wait for (latency 0), not on a 1 ms poll tick, and
// a loop nobody publishes to drains nothing.
// REQ-RT-19
TEST(SimulatedEventLoop, OnPublishRunsAPromptHandlerAtThePublicationInstant) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  loop.set_delivery(Delivery::ON_PUBLISH);
  std::vector<Context> heard;
  auto on_message = [&](const Context& context, std::span<const std::byte>) {
    heard.push_back(context);
  };
  loop.set_prompt(WatchSensor(loop, "/prompt", make_thunk(&on_message)));

  const MonotonicTime published = MonotonicTime{} + 2500us;
  loop.run_until(published);
  EXPECT_EQ(loop.drains(), 0u) << "drained with nothing published";
  loop.inject("/prompt", Sensor{1, 0.0F, 0.0F});
  EXPECT_EQ(loop.next_deadline(), published);
  loop.run_until(published + 50ms);

  ASSERT_EQ(heard.size(), 1u);
  EXPECT_EQ(heard[0].now, published);
  EXPECT_EQ(heard[0].event_time, published);
  EXPECT_EQ(heard[0].latency(), 0ns);
  EXPECT_EQ(heard[0].now - heard[0].publish_time, 0ns);
  EXPECT_EQ(loop.drains(), 1u);
  EXPECT_EQ(loop.next_deadline(), MonotonicTime::max());
}

// A watcher that is not prompt does not wake the loop: its message waits for
// the next timer's drain, stamped with that timer's deadline, or for a
// drain a prompt publication causes, where it keeps the drain's time.
// REQ-RT-02
TEST(SimulatedEventLoop,
     OnPublishDrainsAQuietWatcherOnlyBeforeATimerOrWithAWake) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  loop.set_delivery(Delivery::ON_PUBLISH);
  std::vector<Context> quiet;
  std::vector<Context> prompt;
  auto on_quiet = [&](const Context& context) { quiet.push_back(context); };
  auto on_prompt = [&](const Context& context, std::span<const std::byte>) {
    prompt.push_back(context);
  };
  WatchSensor(loop, "/quiet", make_thunk(&on_quiet), true);
  loop.set_prompt(WatchSensor(loop, "/wake", make_thunk(&on_prompt)));
  auto on_tick = [](const Context&) {};
  const auto tick = loop.register_timer("tick", make_thunk(&on_tick));
  const MonotonicTime origin{};
  loop.arm_timer(tick, origin + 10ms, 10ms);

  loop.run_until(origin + 2ms);
  loop.inject("/quiet", Sensor{1, 0.0F, 0.0F});
  loop.run_until(origin + 9ms);
  EXPECT_TRUE(quiet.empty()) << "a quiet publication woke the loop";
  loop.run_until(origin + 10ms);
  ASSERT_EQ(quiet.size(), 1u);
  EXPECT_EQ(quiet[0].now, origin + 10ms);
  EXPECT_EQ(quiet[0].event_time, origin + 10ms);

  loop.inject("/quiet", Sensor{2, 0.0F, 0.0F});
  loop.run_until(origin + 13ms);
  EXPECT_EQ(quiet.size(), 1u);
  loop.inject("/wake", Sensor{3, 0.0F, 0.0F});
  loop.run_until(origin + 15ms);
  ASSERT_EQ(prompt.size(), 1u);
  EXPECT_EQ(prompt[0].event_time, origin + 13ms);
  ASSERT_EQ(quiet.size(), 2u);
  EXPECT_EQ(quiet[1].now, origin + 13ms);
  EXPECT_EQ(quiet[1].event_time, origin + 13ms);
  EXPECT_EQ(quiet[1].publish_time, origin + 10ms);
  EXPECT_EQ(loop.drains(), 2u);
}

// Loops sharing an environment are stepped one after another. A loop already
// run to an instant is due there again when another publishes to its prompt
// watcher at that instant, and its handler runs at the publication's time,
// not at the reader's next event (talos::testing::Sim runs it again).
// REQ-RT-05
TEST(SimulatedEventLoop,
     APromptPublicationAtAnInstantTheReaderHasRunIsDueThere) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> reader{environment};
  SimulatedEventLoop<> writer{environment};
  reader.set_delivery(Delivery::ON_PUBLISH);
  std::vector<Context> heard;
  auto on_message = [&](const Context& context, std::span<const std::byte>) {
    heard.push_back(context);
  };
  reader.set_prompt(WatchSensor(reader, "/handoff", make_thunk(&on_message)));
  const std::uint16_t sender = SendSensor(writer, "/handoff");
  auto on_tick = [&](const Context&) { Send(writer, sender, 1); };
  const auto tick = writer.register_timer("tick", make_thunk(&on_tick));
  const MonotonicTime at = MonotonicTime{} + 5ms;
  writer.arm_timer(tick, at, {});

  reader.run_until(at);  // the reader runs first: nothing yet
  writer.run_until(at);
  EXPECT_TRUE(heard.empty());
  EXPECT_EQ(reader.next_deadline(), at);
  reader.run_until(at);
  ASSERT_EQ(heard.size(), 1u);
  EXPECT_EQ(heard[0].now, at);
  EXPECT_EQ(heard[0].publish_time, at);
  EXPECT_EQ(heard[0].latency(), 0ns);
}

// A prompt publication made at a reader's timer deadline is delivered by the
// drain before that timer, so the timer's handler sees it, in one drain: the
// realtime loop runs its due timers before it waits for a wake.
// REQ-RT-19
TEST(SimulatedEventLoop,
     APublicationAtATimersDeadlineIsDeliveredByThatTimersDrain) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> reader{environment};
  SimulatedEventLoop<> writer{environment};
  reader.set_delivery(Delivery::ON_PUBLISH);
  std::vector<Context> heard;
  auto on_message = [&](const Context& context, std::span<const std::byte>) {
    heard.push_back(context);
  };
  reader.set_prompt(WatchSensor(reader, "/handoff", make_thunk(&on_message)));
  std::size_t heard_by_tick = 0;
  auto on_reader_tick = [&](const Context&) { heard_by_tick = heard.size(); };
  const MonotonicTime at = MonotonicTime{} + 5ms;
  reader.arm_timer(reader.register_timer("tick", make_thunk(&on_reader_tick)),
                   at, {});
  const std::uint16_t sender = SendSensor(writer, "/handoff");
  auto on_writer_tick = [&](const Context&) { Send(writer, sender, 1); };
  writer.arm_timer(writer.register_timer("tick", make_thunk(&on_writer_tick)),
                   at, {});

  writer.run_until(at);
  reader.run_until(at);
  ASSERT_EQ(heard.size(), 1u);
  EXPECT_EQ(heard[0].event_time, at);
  EXPECT_EQ(heard_by_tick, 1u) << "the timer ran before its drain delivered";
  EXPECT_EQ(reader.drains(), 1u);
}

// A prompt handler that publishes on another prompt watcher of its loop
// hands over at the same instant, whatever the registration order: the
// first drain cannot see a publication its own later watcher makes, so the
// loop drains again at once.
// REQ-RT-19
TEST(SimulatedEventLoop, PromptHandlersChainAtOneInstant) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  loop.set_delivery(Delivery::ON_PUBLISH);
  std::vector<Heard> heard;
  auto on_second = [&](const Context& context, std::span<const std::byte>) {
    heard.push_back({"second", context});
  };
  loop.set_prompt(WatchSensor(loop, "/second", make_thunk(&on_second)));
  const std::uint16_t sender = SendSensor(loop, "/second");
  auto on_first = [&](const Context& context, std::span<const std::byte>) {
    heard.push_back({"first", context});
    Send(loop, sender, 2);
  };
  loop.set_prompt(WatchSensor(loop, "/first", make_thunk(&on_first)));

  const MonotonicTime at = MonotonicTime{} + 3ms;
  loop.run_until(at);
  loop.inject("/first", Sensor{1, 0.0F, 0.0F});
  loop.run_until(at + 10ms);
  ASSERT_EQ(heard.size(), 2u);
  EXPECT_EQ(heard[0].topic, "first");
  EXPECT_EQ(heard[1].topic, "second");
  for (const Heard& h : heard) {
    SCOPED_TRACE(h.topic);
    EXPECT_EQ(h.context.now, at);
    EXPECT_EQ(h.context.event_time, at);
  }
  EXPECT_EQ(loop.drains(), 2u);
}

// The per-drain cap holds under ON_PUBLISH, but a prompt watcher it leaves
// with unread messages is due again at once: a backlog past the cap is
// delivered in order at its publication instant, none lost.
// REQ-RT-02
TEST(SimulatedEventLoop, PromptBacklogPastTheCapDrainsAtItsInstant) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  loop.set_delivery(Delivery::ON_PUBLISH);
  std::vector<Context> heard;
  auto on_message = [&](const Context& context, std::span<const std::byte>) {
    heard.push_back(context);
  };
  loop.set_prompt(WatchSensor(loop, "/backlog", make_thunk(&on_message)));
  const MonotonicTime at = MonotonicTime{} + 1ms;
  loop.run_until(at);
  for (int i = 0; i < 130; ++i) loop.inject("/backlog", Sensor{i, 0.0F, 0.0F});
  loop.run_until(at + 5ms);
  ASSERT_EQ(heard.size(), 130u);
  for (std::size_t i = 0; i < heard.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(heard[i].sequence, i);
    EXPECT_EQ(heard[i].dropped, 0u);
    EXPECT_EQ(heard[i].now, at);
  }
  EXPECT_EQ(loop.drains(), 3u);  // 64 + 64 + 2
}

// A prompt handler that answers every message on its own prompt topic never
// lets time advance. The loop gives up after its bound, naming the topic,
// instead of hanging the test.
TEST(SimulatedEventLoop, AZeroTimePublishCycleThrowsNamingItsTopic) {
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{environment};
  loop.set_delivery(Delivery::ON_PUBLISH);
  const std::uint16_t sender = SendSensor(loop, "/echo");
  auto on_message = [&](const Context&, std::span<const std::byte>) {
    Send(loop, sender, 0);
  };
  loop.set_prompt(WatchSensor(loop, "/echo", make_thunk(&on_message)));
  loop.run_until(MonotonicTime{});
  loop.inject("/echo", Sensor{0, 0.0F, 0.0F});
  try {
    loop.run_until(MonotonicTime{} + 1ms);
    FAIL() << "a zero-time cycle returned";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string{error.what()}.find("/echo"), std::string::npos)
        << error.what();
  }
  EXPECT_EQ(loop.drains(), SimulatedEventLoop<>::kMaxWokenDrainsPerInstant);
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

#include "talOS/events/realtime_event_loop.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "events_test/events_test_message_generated.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/test_robot.h"
#include "talOS/ipc/publisher.h"

namespace talos::event {
namespace {

using namespace std::chrono_literals;
using testing::TestRobot;
using testing::Topics;

// macOS caps shared-memory object names at 31 characters, so these stay short.
// The pid keeps concurrent test runs from colliding on the same objects.
Topics realtime_topics() {
  const std::string pid = std::to_string(::getpid());
  return Topics{"/ts_" + pid, "/tp_" + pid, "/tc_" + pid};
}

std::string temp_path(std::string_view name) {
  return ::testing::TempDir() + "/talos_rt_" + std::string{name} + "_" +
         std::to_string(::getpid()) + ".tlog";
}

using RecordingLoop = RealtimeEventLoop<log::LogWriter>;

std::uint16_t register_sensor_watcher(RealtimeEventLoop<>& loop,
                                      const std::string& topic) {
  return loop.register_watcher(topic, sizeof(EventsTest::SensorMessage),
                               alignof(EventsTest::SensorMessage),
                               talos::schema::Of<EventsTest::SensorMessage>(),
                               Thunk{});
}

TEST(RealtimeEventLoop, PeriodicTimerRunsAtRate) {
  RealtimeEventLoop<> loop;

  std::uint64_t fired = 0;
  std::uint64_t total_cycles = 0;

  auto handler = [&fired, &total_cycles](const Context& context) {
    ++fired;
    total_cycles += context.sequence;
  };

  const std::uint16_t id = loop.register_timer("tick", make_thunk(&handler));
  Timer<RealtimeEventLoop<>> timer{&loop, id};
  timer.setup_periodic(Poller::now() + 1ms, 1ms);

  loop.run_for(200ms);

  // A 1 kHz timer over 200 ms. Loaded CI can steal a lot of time, so assert
  // the shape rather than an exact count: it must actually be periodic, and it
  // must not fire more often than the period allows.
  EXPECT_GT(fired, 50u);
  EXPECT_LE(fired, 210u);

  // Every period is accounted for, whether it was serviced on time or
  // collapsed into an overrun.
  EXPECT_GE(total_cycles, fired);
  EXPECT_LE(total_cycles, 220u);
}

TEST(RealtimeEventLoop, ResamplesClockAfterEachDueTimerCallback) {
  RealtimeEventLoop<> loop;
  loop.set_delivery(Delivery::BEFORE_TIMERS);
  MonotonicTime first_finished{};
  MonotonicTime second_observed{};
  auto first = [&](const Context&) {
    std::this_thread::sleep_for(10ms);
    first_finished = Poller::now();
  };
  auto second = [&](const Context& context) {
    second_observed = context.now;
    loop.exit();
  };
  const auto first_id = loop.register_timer("first", make_thunk(&first));
  const auto second_id = loop.register_timer("second", make_thunk(&second));
  const auto deadline = loop.monotonic_now();
  loop.arm_timer(first_id, deadline, {});
  loop.arm_timer(second_id, deadline, {});
  loop.run_for(100ms);
  EXPECT_GT(first_finished, deadline);
  EXPECT_GE(second_observed, first_finished);
}

// A node builds its loop, then spends a while in setup (config, imports,
// devices) before run(). Its schedule counts from the run's start, so the
// worst lateness the loop reports is the run's own, not that setup.
TEST(RealtimeEventLoop, SetupBeforeTheRunIsNotReportedAsLateness) {
  RealtimeEventLoop<> loop;
  std::vector<Context> ticks;
  auto handler = [&](const Context& context) {
    ticks.push_back(context);
    if (ticks.size() == 3) loop.exit();
  };
  const auto id = loop.register_timer("tick", make_thunk(&handler));
  loop.arm_timer(id, loop.monotonic_now() + 5ms, 5ms);
  std::this_thread::sleep_for(100ms);
  const MonotonicTime run_start = Poller::now();
  loop.run_for(5s);  // Bound only; exits after three ticks.
  const Duration ran = Poller::now() - run_start;
  ASSERT_EQ(ticks.size(), 3u);
  EXPECT_GE(ticks.front().event_time, run_start + 5ms);
  EXPECT_LT(loop.source_counters()[id].max_latency_ns.load(), ran.count());
}

TEST(RealtimeEventLoop, LatestBacklogDispatchesNewestWithPublicationAge) {
  const std::string topic = "/latest_" + std::to_string(::getpid());
  RealtimeEventLoop<> loop;
  loop.set_delivery(Delivery::BEFORE_TIMERS);
  std::vector<Context> observed;
  auto on_message = [&](const Context& context) {
    observed.push_back(context);
  };
  const auto watcher =
      loop.register_watcher(topic, sizeof(EventsTest::SensorMessage),
                            alignof(EventsTest::SensorMessage),
                            talos::schema::Of<EventsTest::SensorMessage>(),
                            make_thunk(&on_message), true);
  RTMSQueue publisher{topic, sizeof(EventsTest::SensorMessage),
                      alignof(EventsTest::SensorMessage),
                      talos::schema::Of<EventsTest::SensorMessage>()};
  const auto published = Poller::now() - 100ms;
  for (int i = 0; i < 130; ++i) {
    const EventsTest::SensorMessage message{i, 0.0F, 0.0F};
    ASSERT_EQ(
        publisher.write(
            std::span<const std::byte>{
                reinterpret_cast<const std::byte*>(&message), sizeof(message)},
            published.nanos()),
        WriteResult::SUCCESS);
  }
  auto stop = [&](const Context&) { loop.exit(); };
  const auto timer = loop.register_timer("stop", make_thunk(&stop));
  loop.arm_timer(timer, loop.monotonic_now(), {});
  loop.run_for(100ms);
  ASSERT_EQ(observed.size(), 1u);
  EXPECT_EQ(observed.front().sequence, 129u);
  EXPECT_EQ(observed.front().publish_time, published);
  EXPECT_GE(observed.front().now - observed.front().publish_time, 100ms);
  EXPECT_EQ(loop.source_counters()[watcher].dropped.load(), 129u);
}

TEST(RealtimeEventLoop, ExitFromAnotherThreadReturnsPromptly) {
  RealtimeEventLoop<> loop;

  // Nothing but the internal poll timer is armed, so the loop is asleep in the
  // poller almost all of the time.
  std::atomic<bool> started{false};
  auto handler = [&started](const Context&) { started.store(true); };

  const std::uint16_t id = loop.register_timer("tick", make_thunk(&handler));
  Timer<RealtimeEventLoop<>> timer{&loop, id};
  timer.setup_periodic(Poller::now() + 1ms, 50ms);

  std::thread stopper{[&loop, &started] {
    while (!started.load()) {
      std::this_thread::yield();
    }
    loop.exit();
  }};

  const MonotonicTime start = Poller::now();
  loop.run_for(10s);
  const Duration elapsed = Poller::now() - start;

  stopper.join();

  EXPECT_LT(elapsed, Duration{std::chrono::seconds{5}});
}

TEST(RealtimeEventLoop, RejectsTimerChangesFromAnotherThread) {
  RealtimeEventLoop<> loop;
  std::atomic<bool> started{false};
  auto handler = [&](const Context&) { started.store(true); };
  const auto id = loop.register_timer("tick", make_thunk(&handler));
  loop.arm_timer(id, loop.monotonic_now() + 1ms, 1ms);
  std::thread other{[&] {
    // Bounded, so a loop that never dispatches fails the test instead of
    // hanging until the harness kills it.
    const auto give_up = std::chrono::steady_clock::now() + 5s;
    while (!started.load() && std::chrono::steady_clock::now() < give_up) {
      std::this_thread::yield();
    }
    EXPECT_TRUE(started.load()) << "the timer never fired";
    EXPECT_THROW(loop.arm_timer(id, Poller::now(), 1ms), std::logic_error);
    EXPECT_THROW(loop.disarm_timer(id), std::logic_error);
    loop.exit();
  }};
  loop.run_for(1s);
  other.join();
}

TEST(RealtimeEventLoop, ReaderExhaustionFailsRegistrationAndAllowsRetry) {
  const std::string topic = "/full_" + std::to_string(::getpid());
  RTMSQueue keeper{topic, sizeof(EventsTest::SensorMessage),
                   alignof(EventsTest::SensorMessage),
                   talos::schema::Of<EventsTest::SensorMessage>(), MAX_SLOTS};
  std::vector<std::size_t> readers;
  for (std::size_t i = 0; i < MAX_READERS; ++i) {
    auto reader = keeper.register_reader();
    ASSERT_TRUE(reader);
    readers.push_back(reader.value_or(std::size_t{0}));
  }
  {
    RealtimeEventLoop<> loop;
    EXPECT_THROW(register_sensor_watcher(loop, topic), RegistrationError);
    EXPECT_TRUE(loop.manifest().empty());
    keeper.release_reader(readers.back());
    readers.pop_back();
    EXPECT_EQ(register_sensor_watcher(loop, topic), 0);
  }
  const auto recycled = keeper.register_reader();
  ASSERT_TRUE(recycled);
  keeper.release_reader(recycled.value_or(std::size_t{0}));
  for (auto reader : readers) keeper.release_reader(reader);
}

// Another process can free a watcher's reader slot, having judged this one
// dead. The subscription must claim a slot again rather than go quiet, and
// count the failed read and what was published while it had no slot.
TEST(RealtimeEventLoop, AWatcherWhoseSlotIsFreedClaimsOneAgain) {
  const std::string topic = "/freed_" + std::to_string(::getpid());
  RealtimeEventLoop<> loop;
  loop.set_delivery(Delivery::BEFORE_TIMERS);
  std::vector<Context> delivered;
  auto on_message = [&](const Context& context) {
    delivered.push_back(context);
  };
  const auto watcher = loop.register_watcher(
      topic, sizeof(EventsTest::SensorMessage),
      alignof(EventsTest::SensorMessage),
      talos::schema::Of<EventsTest::SensorMessage>(), make_thunk(&on_message));
  RTMSQueue publisher{topic, sizeof(EventsTest::SensorMessage),
                      alignof(EventsTest::SensorMessage),
                      talos::schema::Of<EventsTest::SensorMessage>()};
  SharedMemoryPtr mapping{topic, sizeof(RTMSHeader)};
  auto* header = static_cast<RTMSHeader*>(mapping.ptr());
  ASSERT_NE(header, nullptr);
  const auto publish = [&publisher](int value) {
    const EventsTest::SensorMessage message{value, 0.0F, 0.0F};
    return publisher.write(std::span<const std::byte>{
               reinterpret_cast<const std::byte*>(&message),
               sizeof(message)}) == WriteResult::SUCCESS;
  };
  ASSERT_TRUE(publish(0));
  // Each timer runs after the drain that precedes it.
  auto free_slot = [&](const Context&) {
    for (auto& reader : header->readers) reader.state.store(0);
    EXPECT_TRUE(publish(1));
    EXPECT_TRUE(publish(2));
  };
  auto publish_more = [&](const Context&) { EXPECT_TRUE(publish(3)); };
  auto stop = [&](const Context&) { loop.exit(); };
  const auto first = loop.register_timer("free", make_thunk(&free_slot));
  const auto second = loop.register_timer("more", make_thunk(&publish_more));
  const auto third = loop.register_timer("stop", make_thunk(&stop));
  const auto origin = loop.monotonic_now();
  loop.arm_timer(first, origin, {});
  loop.arm_timer(second, origin + 1ms, {});
  loop.arm_timer(third, origin + 2ms, {});
  loop.run_for(1s);
  ASSERT_EQ(delivered.size(), 2u);
  EXPECT_EQ(delivered[0].sequence, 0u);
  EXPECT_EQ(delivered[0].dropped, 0u);
  EXPECT_EQ(delivered[1].sequence, 3u);
  EXPECT_EQ(delivered[1].dropped, 2u);  // 1 and 2, published without a slot
  // That failed read, plus the two messages it never saw.
  EXPECT_EQ(loop.source_counters()[watcher].dropped.load(), 3u);
}

// The point of the whole design: the realtime run is full of jitter, dropped
// messages and scheduling noise, and replaying its log still reproduces every
// output exactly, whichever way the run delivered its messages.
void ExpectRealtimeRunReplays(Delivery delivery) {
  const Topics topics = realtime_topics();
  const std::string path = temp_path("run");

  std::vector<float> recorded;
  std::uint64_t sensor_count = 0;
  std::uint64_t dropped = 0;

  {
    RecordingLoop loop{log::LogWriter{path, "realtime_robot"}};
    loop.set_delivery(delivery);
    TestRobot<RecordingLoop> robot{loop, topics, 3.0F};

    // Derived from the loop's clock, not the OS clock, which is what lets the
    // replay below set up identically without being handed this number.
    robot.control_timer().setup_periodic(loop.monotonic_now() + 1ms, 2ms);

    std::atomic<bool> stop{false};

    // Publish at a deliberately uneven rate so the loop sees bursts, gaps and
    // varying latency. None of that may leak into the replayed result.
    std::thread publisher{[&topics, &stop] {
      ipc::Publisher<EventsTest::SensorMessage> sensor{topics.sensor};
      ipc::Publisher<EventsTest::SetpointMessage> setpoint{topics.setpoint};

      std::mt19937 generator{1234};
      std::uniform_int_distribution<int> jitter{0, 900};

      for (int i = 0; !stop.load(); ++i) {
        sensor.write(
            EventsTest::SensorMessage{i, static_cast<float>(i % 50) * 0.1F,
                                      static_cast<float>(i % 7) * 0.01F});

        if (i % 5 == 0) {
          setpoint.write(
              EventsTest::SetpointMessage{i, static_cast<float>(i % 20)});
        }

        std::this_thread::sleep_for(
            std::chrono::microseconds{jitter(generator)});
      }
    }};

    loop.run_for(400ms);
    stop.store(true);
    publisher.join();

    ASSERT_FALSE(loop.recorder().failed()) << loop.recorder().error();

    recorded = robot.efforts();
    sensor_count = robot.sensor_count();
    dropped = robot.dropped_sensors();
  }

  ASSERT_FALSE(recorded.empty());
  EXPECT_GT(sensor_count, 0u);

  log::LogReader reader{path};
  ReplayEventLoop<> replay{reader};
  TestRobot<ReplayEventLoop<>> replayed_robot{replay, topics, 3.0F};

  // The identical setup line, with no knowledge of the recording run: the
  // replay loop's clock already reads the recorded origin.
  replayed_robot.control_timer().setup_periodic(replay.monotonic_now() + 1ms,
                                                2ms);

  replay.run();

  EXPECT_FALSE(replay.diverged());
  EXPECT_TRUE(replay.reached_exit());
  EXPECT_EQ(replayed_robot.efforts(), recorded);
  EXPECT_EQ(replayed_robot.sensor_count(), sensor_count);

  // Drop counts are an input the handlers can act on, so they replay too.
  EXPECT_EQ(replayed_robot.dropped_sensors(), dropped);

  std::remove(path.c_str());
}

TEST(RealtimeEventLoop, RealtimeRunReplaysForBothDeliveryPolicies) {
  for (const auto delivery : {Delivery::POLL, Delivery::BEFORE_TIMERS}) {
    ExpectRealtimeRunReplays(delivery);
  }
}

struct Wakeups {
  std::uint64_t fired{0};
  RealtimeEventLoop<>::Activity activity{};
};

// A 100 Hz program with one watched topic, run for 200 ms.
Wakeups RunPeriodicProgram(Delivery delivery, const std::string& topic) {
  RealtimeEventLoop<> loop;
  loop.set_delivery(delivery);
  register_sensor_watcher(loop, topic);
  Wakeups result;
  auto handler = [&result](const Context&) { ++result.fired; };
  const std::uint16_t id = loop.register_timer("tick", make_thunk(&handler));
  Timer<RealtimeEventLoop<>> timer{&loop, id};
  timer.setup_periodic(loop.monotonic_now() + 10ms, 10ms);
  loop.run_for(200ms);
  result.activity = loop.activity();
  return result;
}

// What the delivery policy is for: a periodic program that only reads its
// topics at its own tick sleeps between ticks instead of waking every
// millisecond to drain.
TEST(RealtimeEventLoop, BeforeTimersWakesOnlyForTimers) {
  const std::string pid = std::to_string(::getpid());
  const Wakeups poll = RunPeriodicProgram(Delivery::POLL, "/twp_" + pid);
  const Wakeups before =
      RunPeriodicProgram(Delivery::BEFORE_TIMERS, "/twb_" + pid);
  std::printf(
      "100 Hz program over 200 ms: POLL %llu wakeups, %llu drains; "
      "BEFORE_TIMERS %llu wakeups, %llu drains, %llu timer firings\n",
      static_cast<unsigned long long>(poll.activity.wakeups),
      static_cast<unsigned long long>(poll.activity.drains),
      static_cast<unsigned long long>(before.activity.wakeups),
      static_cast<unsigned long long>(before.activity.drains),
      static_cast<unsigned long long>(before.fired));

  // Loaded CI steals time, so these bound shapes rather than exact counts.
  EXPECT_GT(poll.activity.drains, 50u);
  EXPECT_GT(poll.activity.wakeups, 50u);

  EXPECT_GT(before.fired, 0u);
  EXPECT_EQ(before.activity.drains, before.fired);  // one per timer dispatch
  EXPECT_LE(before.activity.wakeups, 2 * before.fired + 2);
  EXPECT_LT(before.activity.wakeups, 50u);
}

// Under POLL a poll tick due at a timer's deadline loses the tie to the timer
// (ties break on id), yet the timer must not run without a drain. The control
// timer here is due on the first poll tick, after a setpoint publication; it
// must see that setpoint.
TEST(RealtimeEventLoop, PollDeliveryDrainsBeforeATimerDueWithTheTick) {
  RealtimeEventLoop<> loop;
  ASSERT_EQ(loop.delivery(), Delivery::POLL);
  const Topics topics = realtime_topics();
  TestRobot<RealtimeEventLoop<>> robot{loop, topics, 1.0F};
  auto stop = [&loop](const Context&) { loop.exit(); };
  const auto stop_id = loop.register_timer("stop", make_thunk(&stop));
  // The run moves the schedule's origin to its start and arms the poll tick a
  // tick_period later, so origin + 1 ms is that tick's deadline. The stop
  // timer's higher id runs it after the control timer at the same instant.
  robot.control_timer().setup_periodic(loop.start_time() + 1ms, 10ms);
  loop.arm_timer(stop_id, loop.start_time() + 1ms, Duration::zero());
  ipc::Publisher<EventsTest::SetpointMessage> setpoint{topics.setpoint};
  setpoint.write(EventsTest::SetpointMessage{1, 5.0F});
  loop.run_for(5s);  // Bound only; the stop timer exits.
  ASSERT_EQ(robot.control_count(), 1u);
  // 5 less the dispatch latency in seconds; 0 less it without the drain.
  EXPECT_GT(robot.efforts().front(), 4.0F);
}

TEST(RealtimeEventLoop, DeliveryIsFixedOnceTheRunStarts) {
  RealtimeEventLoop<> loop;
  auto handler = [&loop](const Context&) {
    EXPECT_THROW(loop.set_delivery(Delivery::POLL), std::logic_error);
    loop.exit();
  };
  const std::uint16_t id = loop.register_timer("tick", make_thunk(&handler));
  loop.arm_timer(id, loop.monotonic_now() + 1ms, Duration::zero());
  loop.set_delivery(Delivery::BEFORE_TIMERS);
  loop.run_for(1s);
  EXPECT_EQ(loop.delivery(), Delivery::BEFORE_TIMERS);
}

}  // namespace
}  // namespace talos::event

#include "talOS/events/realtime_event_loop.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <optional>
#include <random>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "events_test/events_test_message_generated.h"
#include "talOS/events/log/format.h"
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
                                      const std::string& topic,
                                      Thunk callback = Thunk{}) {
  return loop.register_watcher(topic, sizeof(EventsTest::SensorMessage),
                               alignof(EventsTest::SensorMessage),
                               talos::schema::Of<EventsTest::SensorMessage>(),
                               callback);
}

// A loop that sleeps until a publication on a prompt watcher wakes it. The
// backstop is far beyond every test's bound, so a delivery that waited for
// it fails the test instead of passing late.
RealtimeEventLoop<>::Options on_publish() {
  RealtimeEventLoop<>::Options options;
  options.delivery = Delivery::ON_PUBLISH;
  options.wake_backstop = 30s;
  return options;
}

// The writer end of a sensor topic, for tests that publish from outside the
// loop. It claims the topic when constructed, so the wake a writer taking
// over a topic sends its readers happens then, not at a publication a test
// times.
class SensorWriter {
 public:
  explicit SensorWriter(const std::string& topic)
      : queue_{topic,
               sizeof(EventsTest::SensorMessage),
               alignof(EventsTest::SensorMessage),
               talos::schema::Of<EventsTest::SensorMessage>(),
               MAX_SLOTS,
               RTMSOptions{.publisher = true}} {}
  bool publish(int value) {
    const EventsTest::SensorMessage message{value, 0.0F, 0.0F};
    return queue_.write(std::span<const std::byte>{
               reinterpret_cast<const std::byte*>(&message),
               sizeof(message)}) == WriteResult::SUCCESS;
  }

 private:
  RTMSQueue queue_;
};

// Spins (yielding) until `ready`, for at most five seconds; false on giving
// up, so a loop that never gets there fails its test instead of hanging it.
template <typename Ready>
bool await(Ready ready) {
  const auto give_up = std::chrono::steady_clock::now() + 5s;
  while (!ready()) {
    if (std::chrono::steady_clock::now() > give_up) return false;
    std::this_thread::yield();
  }
  return true;
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

// A tick that overruns its period coalesces the periods it missed into one
// late dispatch; the timer's live `dropped` counts them, so the registry
// shows an overrunning control loop while its dispatch count still moves.
TEST(RealtimeEventLoop, OverrunPeriodsCountAsTheTimersDrops) {
  RealtimeEventLoop<> loop;
  std::vector<Context> ticks;
  auto handler = [&](const Context& context) {
    ticks.push_back(context);
    if (ticks.size() == 1) std::this_thread::sleep_for(22ms);
    if (ticks.size() == 3) loop.exit();
  };
  const auto id = loop.register_timer("tick", make_thunk(&handler));
  loop.arm_timer(id, loop.monotonic_now() + 5ms, 5ms);
  loop.run_for(5s);  // Bound only; exits after three ticks.
  ASSERT_EQ(ticks.size(), 3u);
  std::uint64_t missed = 0;
  for (const Context& tick : ticks) missed += tick.sequence - 1;
  EXPECT_GE(missed, 3u);  // 22 ms of a 5 ms period, at least
  EXPECT_EQ(loop.source_counters()[id].dropped.load(), missed);
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

// Under POLL the loop sleeps between poll ticks; under ON_PUBLISH it sleeps
// with its prompt watcher armed, on a topic nobody publishes.
TEST(RealtimeEventLoop, ExitFromAnotherThreadReturnsPromptly) {
  for (const auto delivery : {Delivery::POLL, Delivery::ON_PUBLISH}) {
    SCOPED_TRACE(static_cast<int>(delivery));
    RealtimeEventLoop<> loop{on_publish()};
    loop.set_delivery(delivery);
    loop.set_prompt(register_sensor_watcher(
        loop, "/texit_" + std::to_string(::getpid()) + "_" +
                  std::to_string(static_cast<int>(delivery))));

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
}

// What talOS/node stops a node with, so that no thread has to poll for a
// stop: a watched fd turning readable ends the loop's wait, and its handler
// runs on the loop thread, whether the byte was there before the run or
// arrives while the loop runs, under each delivery an idle node can run
// with. The loop has no timer and a 30 s backstop, so nothing but the fd
// ends its one wait before the run's 20 s end; and the fd is no publication,
// so nothing drains.
// REQ-OPS-06
TEST(RealtimeEventLoop, AWatchedFdEndsTheWaitOnTheLoopThread) {
  int index = 0;
  for (const auto delivery : {Delivery::BEFORE_TIMERS, Delivery::ON_PUBLISH}) {
    for (const bool before_run : {true, false}) {
      SCOPED_TRACE(
          ::testing::Message()
          << "delivery " << static_cast<int>(delivery)
          << (before_run ? ", byte before the run" : ", byte while it runs"));
      int fds[2];
      ASSERT_EQ(::pipe(fds), 0);
      RealtimeEventLoop<> loop{on_publish()};
      loop.set_delivery(delivery);
      if (delivery == Delivery::ON_PUBLISH) {
        loop.set_prompt(
            register_sensor_watcher(loop, "/tfd_" + std::to_string(::getpid()) +
                                              "_" + std::to_string(index++)));
      }
      int calls = 0;
      std::thread::id ran_on;
      loop.watch_fd(fds[0], [&] {
        ++calls;
        ran_on = std::this_thread::get_id();
        loop.exit();
      });
      const char byte = 1;
      if (before_run) ASSERT_EQ(::write(fds[1], &byte, 1), 1);
      std::thread writer{[&] {
        if (before_run) return;
        if (await([&] { return loop.running(); })) {
          EXPECT_EQ(::write(fds[1], &byte, 1), 1);
        }
      }};

      const MonotonicTime start = Poller::now();
      loop.run_for(20s);  // Bound only; the handler exits.
      const Duration elapsed = Poller::now() - start;
      writer.join();
      ::close(fds[0]);
      ::close(fds[1]);

      EXPECT_EQ(calls, 1);
      EXPECT_EQ(ran_on, std::this_thread::get_id());
      EXPECT_LT(elapsed, Duration{10s}) << "the wait ended at the run's end";
      EXPECT_EQ(loop.activity().wakeups, 1u);
      EXPECT_EQ(loop.activity().drains, 0u);
    }
  }
}

// A loop behind its timers never waits, so a stop it learned of only in a
// wait would never land: a node whose update() overran its period would run
// on until the launcher's SIGKILL. With a stop flag the run ends after the
// dispatch during which the stop came (here the handler requests it, as a
// signal landing mid-handler does), for a 1 ms timer whose handler takes
// 2 ms, and for a burst of callback messages delivered by woken drains back
// to back. Each handler exits at its 100th call, so a loop that ignored the
// flag fails instead of hanging.
// REQ-OPS-06
TEST(RealtimeEventLoop, AStopFlagEndsTheRunAfterTheCurrentDispatch) {
  constexpr int kStopAt = 3;
  constexpr int kGiveUpAt = 100;
  {
    SCOPED_TRACE("overrunning timer");
    std::atomic<bool> stop{false};
    RealtimeEventLoop<>::Options options;
    options.stop = &stop;
    RealtimeEventLoop<> loop{options};
    loop.set_delivery(Delivery::BEFORE_TIMERS);
    int calls = 0;
    auto handler = [&](const Context&) {
      const MonotonicTime until = Poller::now() + 2ms;
      while (Poller::now() < until) {
      }
      if (++calls == kStopAt) stop.store(true);
      if (calls == kGiveUpAt) loop.exit();
    };
    const std::uint16_t id = loop.register_timer("tick", make_thunk(&handler));
    Timer<RealtimeEventLoop<>> timer{&loop, id};
    timer.setup_periodic(Poller::now(), 1ms);
    loop.run_for(20s);  // Bound only.
    EXPECT_EQ(calls, kStopAt);
    // At most the wait for the first deadline: never one once behind.
    EXPECT_LE(loop.activity().wakeups, 1u);
  }
  {
    SCOPED_TRACE("message burst");
    constexpr int kBurst = 200;  // over max_messages_per_tick, under the ring
    const std::string topic = "/tstop_" + std::to_string(::getpid());
    std::atomic<bool> stop{false};
    RealtimeEventLoop<>::Options options = on_publish();
    options.stop = &stop;
    RealtimeEventLoop<> loop{options};
    int calls = 0;
    auto on_message = [&](const Context&) {
      if (++calls == kStopAt) stop.store(true);
      if (calls == kGiveUpAt) loop.exit();
    };
    loop.set_prompt(
        register_sensor_watcher(loop, topic, make_thunk(&on_message)));
    SensorWriter writer{topic};
    for (int i = 0; i < kBurst; ++i) ASSERT_TRUE(writer.publish(i));
    loop.run_for(20s);  // Bound only.
    EXPECT_EQ(calls, kStopAt);
  }
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

std::vector<char> read_file(const std::string& path) {
  std::ifstream file{path, std::ios::binary};
  return std::vector<char>{std::istreambuf_iterator<char>{file},
                           std::istreambuf_iterator<char>{}};
}

// The point of the whole design: the realtime run is full of jitter, dropped
// messages and scheduling noise, and replaying its log still reproduces every
// output exactly, whichever way the run delivered its messages. Recording the
// replay gives the same log again, byte for byte: a drain is recorded only as
// the messages it dispatched, whether a poll tick, a timer or a publication
// woke the loop, so replay needs nothing more.
void ExpectRealtimeRunReplays(Delivery delivery) {
  const Topics topics = realtime_topics();
  const std::string path = temp_path("run");
  const std::string replayed_path = temp_path("rerun");

  std::vector<float> recorded;
  std::uint64_t sensor_count = 0;
  std::uint64_t dropped = 0;

  {
    RecordingLoop loop{log::LogWriter{path, "realtime_robot"}};
    loop.set_delivery(delivery);
    TestRobot<RecordingLoop> robot{loop, topics, 3.0F};
    // Both watchers, so that under ON_PUBLISH each publication wakes the
    // loop; the other policies ignore it.
    for (const Registration& source : loop.manifest()) {
      if (source.kind == SourceKind::WATCHER) loop.set_prompt(source.id);
    }

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

  {
    log::LogReader reader{path};
    using Replay = ReplayEventLoop<log::LogWriter>;
    Replay replay{reader, log::LogWriter{replayed_path, "realtime_robot"}};
    TestRobot<Replay> replayed_robot{replay, topics, 3.0F};

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
    ASSERT_FALSE(replay.recorder().failed()) << replay.recorder().error();
  }

  std::vector<char> original = read_file(path);
  std::vector<char> replayed = read_file(replayed_path);
  ASSERT_FALSE(original.empty());
  ASSERT_EQ(original.size(), replayed.size());
  // Wall time is informational, and the replay ran later.
  const std::size_t wall = offsetof(log::FileHeader, start_wall_ns);
  for (std::size_t i = 0; i < sizeof(log::FileHeader::start_wall_ns); ++i) {
    original[wall + i] = 0;
    replayed[wall + i] = 0;
  }
  EXPECT_EQ(original, replayed);

  std::remove(path.c_str());
  std::remove(replayed_path.c_str());
}

// REQ-RT-02
TEST(RealtimeEventLoop, RealtimeRunReplaysForEveryDeliveryPolicy) {
  for (const auto delivery :
       {Delivery::POLL, Delivery::BEFORE_TIMERS, Delivery::ON_PUBLISH}) {
    SCOPED_TRACE(static_cast<int>(delivery));
    ExpectRealtimeRunReplays(delivery);
  }
}

struct Wakeups {
  std::uint64_t fired{0};
  RealtimeEventLoop<>::Activity activity{};
};

constexpr std::uint64_t kPeriodicFirings = 20;

// A 100 Hz program with one watched topic, prompt, that nobody publishes on,
// run until its timer has fired kPeriodicFirings times. Ending on the count,
// not the clock, keeps a descheduled host from changing what it measures.
Wakeups RunPeriodicProgram(Delivery delivery, const std::string& topic) {
  RealtimeEventLoop<> loop;
  loop.set_delivery(delivery);
  loop.set_prompt(register_sensor_watcher(loop, topic));
  Wakeups result;
  auto handler = [&](const Context&) {
    if (++result.fired == kPeriodicFirings) loop.exit();
  };
  const std::uint16_t id = loop.register_timer("tick", make_thunk(&handler));
  Timer<RealtimeEventLoop<>> timer{&loop, id};
  timer.setup_periodic(loop.monotonic_now() + 10ms, 10ms);
  loop.run_for(30s);  // Bound only; the timer exits.
  result.activity = loop.activity();
  return result;
}

// What the delivery policy is for: a periodic program sleeps
// between its ticks instead of waking every millisecond to drain, both when
// it only reads its topics at its tick and when it sleeps armed for a
// publication that does not come; its timer still fires on time.
// REQ-RT-02
TEST(RealtimeEventLoop, OnlyPollWakesBetweenTimersForAQuietTopic) {
  const std::string pid = std::to_string(::getpid());
  const Wakeups poll = RunPeriodicProgram(Delivery::POLL, "/twp_" + pid);
  for (const auto delivery : {Delivery::BEFORE_TIMERS, Delivery::ON_PUBLISH}) {
    const Wakeups quiet = RunPeriodicProgram(
        delivery,
        "/tw" + std::to_string(static_cast<int>(delivery)) + "_" + pid);
    std::printf(
        "100 Hz program, 20 ticks: POLL %llu wakeups, %llu drains; "
        "delivery %d %llu wakeups, %llu drains, %llu timer firings\n",
        static_cast<unsigned long long>(poll.activity.wakeups),
        static_cast<unsigned long long>(poll.activity.drains),
        static_cast<int>(delivery),
        static_cast<unsigned long long>(quiet.activity.wakeups),
        static_cast<unsigned long long>(quiet.activity.drains),
        static_cast<unsigned long long>(quiet.fired));

    // The timer fires while the loop sleeps armed; each firing is one drain
    // and about one wait, however late the host ran it.
    EXPECT_EQ(quiet.fired, kPeriodicFirings);
    EXPECT_EQ(quiet.activity.drains, quiet.fired);  // one per timer dispatch
    EXPECT_LE(quiet.activity.wakeups, 2 * quiet.fired + 2);
  }
  // POLL also wakes for ticks between the timers (about ten per firing).
  EXPECT_EQ(poll.fired, kPeriodicFirings);
  EXPECT_GT(poll.activity.drains, poll.fired);
  EXPECT_GT(poll.activity.wakeups, poll.fired);
}

// With nothing but a prompt watcher and nobody publishing, the
// loop sleeps for the whole run: a 1 ms poll would wake about 200 times.
// REQ-RT-19
TEST(RealtimeEventLoop, OnPublishSleepsWhileNothingIsPublished) {
  RealtimeEventLoop<> loop{on_publish()};
  loop.set_prompt(
      register_sensor_watcher(loop, "/tidle_" + std::to_string(::getpid())));
  loop.run_for(200ms);
  EXPECT_LE(loop.activity().wakeups, 2u);
  EXPECT_LE(loop.activity().drains, 1u);
}

// A loop with no timer is woken by each publication on its prompt
// watcher, and delivers every message in order without a drop: one at a time
// while it sleeps, then a burst past the per-drain cap. The 30 s backstop and
// 5 s bound mean no delivery may wait for anything but the publication.
// REQ-RT-19
TEST(RealtimeEventLoop, OnPublishDeliversEveryPublicationInOrder) {
  constexpr int kOneByOne = 20;
  constexpr int kBurst = 200;  // over max_messages_per_tick, under the ring
  const std::string topic = "/tord_" + std::to_string(::getpid());
  RealtimeEventLoop<> loop{on_publish()};
  std::vector<Context> delivered;
  std::atomic<int> count{0};
  auto on_message = [&](const Context& context) {
    delivered.push_back(context);
    if (count.fetch_add(1) + 1 == kOneByOne + kBurst) loop.exit();
  };
  loop.set_prompt(
      register_sensor_watcher(loop, topic, make_thunk(&on_message)));
  SensorWriter writer{topic};

  std::thread publisher{[&] {
    for (int i = 0; i < kOneByOne; ++i) {
      // Each after the previous one arrived, so the loop is asleep for it.
      if (!await([&] { return count.load() == i; })) return;
      EXPECT_TRUE(writer.publish(i));
    }
    if (!await([&] { return count.load() == kOneByOne; })) return;
    for (int i = kOneByOne; i < kOneByOne + kBurst; ++i) {
      EXPECT_TRUE(writer.publish(i));
    }
  }};
  loop.run_for(5s);
  publisher.join();

  ASSERT_EQ(delivered.size(), static_cast<std::size_t>(kOneByOne + kBurst));
  for (std::size_t i = 0; i < delivered.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(delivered[i].sequence, i);
    EXPECT_EQ(delivered[i].dropped, 0u);
    // Woken for a prompt watcher: latency() runs from the publication.
    EXPECT_EQ(delivered[i].event_time, delivered[i].publish_time);
    EXPECT_GE(delivered[i].now, delivered[i].event_time);
  }
  // Every failed arm here was a real backlog, which the drain delivered.
  EXPECT_EQ(loop.activity().stalled_arms, 0u);
}

// A publication on a watcher that is not prompt does not wake the
// loop, and still reaches the next timer: it is drained right before it.
// REQ-RT-01
TEST(RealtimeEventLoop, OnPublishDrainsAQuietWatcherOnlyBeforeTimers) {
  const std::string pid = std::to_string(::getpid());
  const std::string latched = "/tlat_" + pid;
  RealtimeEventLoop<> loop{on_publish()};
  loop.set_prompt(register_sensor_watcher(loop, "/tprm_" + pid));
  std::vector<Context> delivered;
  auto on_latched = [&](const Context& context) {
    delivered.push_back(context);
  };
  register_sensor_watcher(loop, latched, make_thunk(&on_latched));
  SensorWriter writer{latched};
  auto publish = [&](const Context&) { EXPECT_TRUE(writer.publish(1)); };
  MonotonicTime read_at{};
  auto read = [&](const Context& context) {
    read_at = context.event_time;
    loop.exit();
  };
  const auto first = loop.register_timer("publish", make_thunk(&publish));
  const auto second = loop.register_timer("read", make_thunk(&read));
  loop.arm_timer(first, loop.monotonic_now() + 10ms, {});
  loop.arm_timer(second, loop.monotonic_now() + 60ms, {});
  loop.run_for(5s);  // Bound only; the second timer exits.

  ASSERT_EQ(delivered.size(), 1u);
  EXPECT_EQ(delivered[0].event_time, read_at);  // the second timer's drain
  EXPECT_EQ(loop.activity().drains, 2u);        // one per timer, no more
}

// In a drain the loop was woken for, a prompt watcher's message counts its
// latency from publication; a watcher that is not prompt waited for the
// drain by design and is stamped with the drain.
// REQ-RT-19
TEST(RealtimeEventLoop, AWakeStampsOnlyPromptMessagesWithTheirPublication) {
  const std::string pid = std::to_string(::getpid());
  const std::string prompt = "/tlp_" + pid;
  const std::string quiet = "/tlq_" + pid;
  RealtimeEventLoop<> loop{on_publish()};
  std::optional<Context> woke;
  std::optional<Context> waited;
  auto on_prompt = [&](const Context& context) { woke = context; };
  auto on_quiet = [&](const Context& context) {
    waited = context;
    loop.exit();
  };
  loop.set_prompt(
      register_sensor_watcher(loop, prompt, make_thunk(&on_prompt)));
  register_sensor_watcher(loop, quiet, make_thunk(&on_quiet));
  SensorWriter prompt_writer{prompt};
  SensorWriter quiet_writer{quiet};
  ASSERT_TRUE(quiet_writer.publish(1));  // waits: nothing wakes the loop

  std::thread publisher{[&] { EXPECT_TRUE(prompt_writer.publish(2)); }};
  loop.run_for(5s);  // Bound only; the quiet watcher's handler exits.
  publisher.join();

  ASSERT_TRUE(woke.has_value());
  ASSERT_TRUE(waited.has_value());
  EXPECT_EQ(woke->event_time, woke->publish_time);
  EXPECT_LT(waited->publish_time, woke->publish_time);
  EXPECT_GE(waited->event_time, woke->publish_time);  // the drain's time
  EXPECT_GE(waited->now, waited->event_time);
  EXPECT_EQ(loop.activity().drains, 1u);
}

// A message no notification announces (a publisher that died between
// publishing and notifying; here, a watcher that is not prompt) is drained
// once the backstop elapses, and only then: one wait, one drain, at the
// backstop's deadline.
// REQ-RT-19
TEST(RealtimeEventLoop, TheBackstopDrainsWhatNoNotificationAnnounced) {
  const std::string pid = std::to_string(::getpid());
  const std::string unannounced = "/tbs_" + pid;
  RealtimeEventLoop<>::Options options = on_publish();
  options.wake_backstop = 50ms;
  RealtimeEventLoop<> loop{options};
  loop.set_prompt(register_sensor_watcher(loop, "/tbp_" + pid));
  std::vector<Context> delivered;
  auto on_message = [&](const Context& context) {
    delivered.push_back(context);
    loop.exit();
  };
  register_sensor_watcher(loop, unannounced, make_thunk(&on_message));
  SensorWriter writer{unannounced};
  ASSERT_TRUE(writer.publish(1));

  const MonotonicTime start = Poller::now();
  loop.run_for(5s);  // Bound only; the handler exits.

  ASSERT_EQ(delivered.size(), 1u);
  EXPECT_GE(delivered[0].event_time, start + options.wake_backstop);
  EXPECT_GE(delivered[0].now, delivered[0].event_time);
  EXPECT_EQ(loop.activity().drains, 1u);
  EXPECT_EQ(loop.activity().wakeups, 1u);
}

// The stall guard's seam: arms the watcher as RTMS does, then reports unread
// data that read_next() never returns, as a transport whose reads keep
// tearing would.
bool ArmButReportUnread(RTMSQueue& queue, std::size_t reader_id) {
  (void)queue.arm_wake(reader_id);
  return false;
}

// A failed arm drains instead of sleeping; when the drains it causes find
// nothing, the loop must stop spinning on them: after kMaxStalledArms (3)
// empty drains it sleeps, and says so once in stalled_arms.
// REQ-RT-19
TEST(RealtimeEventLoop, AnArmThatKeepsFailingWithNothingToReadStopsSpinning) {
  RealtimeEventLoop<>::Options options = on_publish();
  options.arm_wake = &ArmButReportUnread;
  RealtimeEventLoop<> loop{options};
  loop.set_prompt(
      register_sensor_watcher(loop, "/tstl_" + std::to_string(::getpid())));
  loop.run_for(100ms);
  EXPECT_EQ(loop.activity().stalled_arms, 1u);
  EXPECT_EQ(loop.activity().drains, 3u);
  EXPECT_LE(loop.activity().wakeups, 2u);
}

WakeReceiver NoDescriptorsLeft() {
  throw std::system_error(EMFILE, std::generic_category(),
                          "talos wake receiver: socket");
}

// A node that cannot build its wake receiver (out of descriptors) still
// runs: under POLL for that run, said once on stderr, and counted.
// REQ-RT-19
TEST(RealtimeEventLoop, WithoutAWakeReceiverTheLoopFallsBackToPolling) {
  const std::string topic = "/tfb_" + std::to_string(::getpid());
  RealtimeEventLoop<>::Options options = on_publish();
  options.make_wake_receiver = &NoDescriptorsLeft;
  RealtimeEventLoop<> loop{options};
  std::vector<Context> delivered;
  auto on_message = [&](const Context& context) {
    delivered.push_back(context);
    loop.exit();
  };
  loop.set_prompt(
      register_sensor_watcher(loop, topic, make_thunk(&on_message)));
  SensorWriter writer{topic};
  ASSERT_TRUE(writer.publish(1));

  ::testing::internal::CaptureStderr();
  loop.run_for(5s);  // Bound only; the handler exits.
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();

  ASSERT_EQ(delivered.size(), 1u);  // by a poll tick: nothing else drains
  EXPECT_EQ(loop.delivery(), Delivery::POLL);
  EXPECT_EQ(loop.activity().wake_fallbacks, 1u);
  EXPECT_NE(stderr_text.find("cannot be woken on publish"), std::string::npos)
      << stderr_text;
  EXPECT_NE(stderr_text.find(std::strerror(EMFILE)), std::string::npos)
      << stderr_text;
}

// Another process can free a prompt watcher's reader slot. The slot it
// claims again must wake the loop too, or every later message would wait
// for the backstop. The publisher starts once the new slot names the loop's
// receiver, then goes one message at a time, each while the loop sleeps.
// REQ-RT-19
TEST(RealtimeEventLoop, APromptWatcherWhoseSlotIsFreedIsWokenOnItsNewSlot) {
  constexpr int kAfter = 10;
  const std::string topic = "/trej_" + std::to_string(::getpid());
  RealtimeEventLoop<> loop{on_publish()};
  std::optional<SensorWriter> writer;  // opened once the watcher has the topic
  RTMSHeader* header = nullptr;
  std::vector<Context> delivered;
  std::atomic<int> count{0};
  std::atomic<bool> freed{false};
  auto on_message = [&](const Context& context) {
    delivered.push_back(context);
    if (delivered.size() == 1) {
      for (auto& reader : header->readers) reader.state.store(0);
      EXPECT_TRUE(writer->publish(1));  // while it holds no slot
      freed.store(true);
    }
    if (count.fetch_add(1) + 1 == 1 + kAfter) loop.exit();
  };
  loop.set_prompt(
      register_sensor_watcher(loop, topic, make_thunk(&on_message)));
  writer.emplace(topic);
  SharedMemoryPtr mapping{topic, sizeof(RTMSHeader)};
  header = static_cast<RTMSHeader*>(mapping.ptr());
  ASSERT_NE(header, nullptr);
  ASSERT_TRUE(writer->publish(0));

  const auto rearmed = [header] {
    for (const auto& reader : header->readers) {
      if ((reader.state.load() & 3) ==
              static_cast<std::uint64_t>(ReaderState::ACTIVE) &&
          reader.wake_id.load() != 0) {
        return true;
      }
    }
    return false;
  };
  std::thread publisher{[&] {
    if (!await([&] { return freed.load() && rearmed(); })) return;
    for (int i = 0; i < kAfter; ++i) {
      if (!await([&] { return count.load() == 1 + i; })) return;
      EXPECT_TRUE(writer->publish(2 + i));
    }
  }};
  loop.run_for(5s);
  publisher.join();

  ASSERT_EQ(delivered.size(), static_cast<std::size_t>(1 + kAfter));
  EXPECT_EQ(delivered[1].sequence, 2u);
  EXPECT_EQ(delivered[1].dropped, 1u);  // published while it had no slot
  for (std::size_t i = 2; i < delivered.size(); ++i) {
    EXPECT_EQ(delivered[i].sequence, i + 1);
    EXPECT_EQ(delivered[i].dropped, 0u);
  }
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

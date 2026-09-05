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

#include "talOS/events/events_test_message_generated.h"
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

// Instrumentation has to work on the loop that actually has jitter to report.
TEST(RealtimeEventLoop, MetricsMeasureLatencyAndJitter) {
  RealtimeEventLoop<NullRecorder, LoopMetrics> loop;

  auto handler = [](const Context&) {};
  const std::uint16_t id = loop.register_timer("tick", make_thunk(&handler));
  Timer<RealtimeEventLoop<NullRecorder, LoopMetrics>> timer{&loop, id};
  timer.setup_periodic(Poller::now() + 1ms, 1ms);

  loop.run_for(200ms);

  const SourceMetrics& metrics = loop.metrics().source(id);

  ASSERT_GT(metrics.dispatches, 10u);
  EXPECT_EQ(metrics.dispatches, metrics.latency.count());

  // A real loop is always at least slightly late, and never early: the
  // dispatch happens after the deadline by definition.
  EXPECT_GE(metrics.latency.min_ns(), 0);
  EXPECT_GE(metrics.latency.max_ns(), metrics.latency.min_ns());
  EXPECT_GE(metrics.latency.percentile_ns(0.99), metrics.latency.mean_ns() / 2);

  // Intervals cluster around the 1 ms period.
  ASSERT_GT(metrics.interval.count(), 0u);
  EXPECT_GT(metrics.interval.percentile_ns(0.50), 500'000);
  EXPECT_LT(metrics.interval.percentile_ns(0.50), 3'000'000);

  // An empty handler is fast, and utilization is therefore tiny.
  EXPECT_LT(metrics.handler_time.percentile_ns(0.50), 100'000);
  EXPECT_LT(loop.metrics().utilization(), 0.5);

  EXPECT_FALSE(loop.metrics().report(loop.manifest()).empty());
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

TEST(RealtimeEventLoop, WatcherReceivesPublishedMessages) {
  const Topics topics = realtime_topics();
  RealtimeEventLoop<> loop;
  TestRobot<RealtimeEventLoop<>> robot{loop, topics, 2.0F};

  robot.control_timer().setup_periodic(Poller::now() + 1ms, 2ms);

  std::atomic<bool> stop{false};
  std::thread publisher{[&topics, &stop] {
    ipc::Publisher<EventsTest::SensorMessage> sensor{topics.sensor};

    for (int i = 0; !stop.load(); ++i) {
      sensor.write(EventsTest::SensorMessage{i, static_cast<float>(i), 0.0F});
      std::this_thread::sleep_for(500us);
    }
  }};

  loop.run_for(300ms);
  stop.store(true);
  publisher.join();

  EXPECT_GT(robot.sensor_count(), 0u);
  EXPECT_GT(robot.control_count(), 0u);
}

// The point of the whole design: the realtime run is full of jitter, dropped
// messages and scheduling noise, and replaying its log still reproduces every
// output exactly.
TEST(RealtimeEventLoop, ReplayOfARealtimeRunDoesNotDiverge) {
  const Topics topics = realtime_topics();
  const std::string path = temp_path("run");

  std::vector<float> recorded;
  std::uint64_t sensor_count = 0;
  std::uint64_t dropped = 0;

  {
    RecordingLoop loop{log::LogWriter{path, "realtime_robot"}};
    TestRobot<RecordingLoop> robot{loop, topics, 3.0F};

    robot.control_timer().setup_periodic(Poller::now() + 1ms, 2ms);

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

  replay.run();

  EXPECT_FALSE(replay.diverged());
  EXPECT_TRUE(replay.reached_exit());
  EXPECT_EQ(replayed_robot.efforts(), recorded);
  EXPECT_EQ(replayed_robot.sensor_count(), sensor_count);

  // Drop counts are an input the handlers can act on, so they replay too.
  EXPECT_EQ(replayed_robot.dropped_sensors(), dropped);

  std::remove(path.c_str());
}

}  // namespace
}  // namespace talos::event

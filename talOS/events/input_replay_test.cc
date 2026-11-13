// Input replay (input_replay.h): a program that was not in a recorded
// session, driven from the topics the session logged, computes what it would
// have computed had it been there -- and computes it the same way every time.

#include "talOS/events/input_replay.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "events_test/events_test_message_generated.h"
#include "talOS/events/handles.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/events/test_robot.h"

namespace talos::event {
namespace {

using namespace std::chrono_literals;
using testing::TestRobot;
using testing::Topics;
using Loop = SimulatedEventLoop<log::LogWriter>;

// Not zero, so a replay that starts its clock anywhere but the recorded
// origin shows.
constexpr MonotonicTime kOrigin = MonotonicTime::from_nanos(7'000'000'000);
constexpr int kSessionMs = 60;
constexpr float kGain = 2.5F;

const Topics kTopics{"/input/sensor", "/input/setpoint", "/consumer/command"};

std::string TempDir(const std::string& name) {
  const std::filesystem::path dir =
      std::filesystem::path{::testing::TempDir()} /
      ("talos_input_replay_" + name + "_" + std::to_string(::getpid()));
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir.string();
}

// The session's writer of /input/sensor: one reading every 2 ms.
class Producer {
 public:
  explicit Producer(Loop& loop)
      : sensor_{make_sender<EventsTest::SensorMessage>(loop, kTopics.sensor)},
        tick_{make_timer<&Producer::on_tick>(loop, "tick", this)} {
    tick_.setup_periodic(loop.monotonic_now() + 2ms, 2ms);
  }

 private:
  void on_tick(const Context&) {
    ++count_;
    sensor_.send(EventsTest::SensorMessage{count_,
                                           0.25F * static_cast<float>(count_),
                                           0.05F * static_cast<float>(count_)});
  }

  Sender<Loop, EventsTest::SensorMessage> sensor_;
  Timer<Loop> tick_;
  int count_{0};
};

// Reads both inputs and nothing else. It is the only log of
// /input/setpoint, whose writer (the harness) keeps none.
class Observer {
 public:
  explicit Observer(Loop& loop) {
    watch<EventsTest::SensorMessage, &Observer::on_sensor>(loop, kTopics.sensor,
                                                           this);
    watch<EventsTest::SetpointMessage, &Observer::on_setpoint>(
        loop, kTopics.setpoint, this);
  }

 private:
  void on_sensor(const Context&, const EventsTest::SensorMessage&) {}
  void on_setpoint(const Context&, const EventsTest::SetpointMessage&) {}
};

struct Sent {
  std::int64_t now_ns;
  std::vector<std::byte> payload;
  bool operator==(const Sent&) const = default;
};

// Every message `log_path`'s program sent, with when.
std::vector<Sent> Sends(const std::string& log_path) {
  log::LogReader reader{log_path};
  std::vector<Sent> sent;
  log::LogReader::Record record;
  while (reader.next(record)) {
    if (record.header.kind == static_cast<std::uint16_t>(EventKind::SEND)) {
      sent.push_back({record.header.now_ns,
                      {record.payload.begin(), record.payload.end()}});
    }
  }
  return sent;
}

// Records a session of a producer and an observer into `session_dir`, with
// the consumer running live beside them into `live_log` (outside the
// session). Returns the live consumer's efforts.
std::vector<float> RecordSession(const std::string& session_dir,
                                 const std::string& live_log) {
  SimulationEnvironment environment;
  const Loop::Options options{.start = kOrigin};
  Loop producer_loop{environment,
                     log::LogWriter{session_dir + "/producer.tlog", "producer"},
                     options};
  Loop observer_loop{environment,
                     log::LogWriter{session_dir + "/observer.tlog", "observer"},
                     options};
  Loop consumer_loop{environment, log::LogWriter{live_log, "consumer"},
                     options};
  Producer producer{producer_loop};
  Observer observer{observer_loop};
  TestRobot<Loop> consumer{consumer_loop, kTopics, kGain};
  consumer.control_timer().setup_periodic(kOrigin + 1ms, 1ms);

  for (int ms = 1; ms <= kSessionMs; ++ms) {
    const MonotonicTime now = kOrigin + std::chrono::milliseconds{ms};
    if (ms % 10 == 5) {
      // Publish between drains: ordering within one exact tick is not part of
      // input replay, which feeds each publication before that tick's drain.
      producer_loop.run_until(now - 500us);
      producer_loop.inject(
          kTopics.setpoint,
          EventsTest::SetpointMessage{ms, 0.1F * static_cast<float>(ms)});
    }
    producer_loop.run_until(now);
    observer_loop.run_until(now);
    consumer_loop.run_until(now);
  }
  producer_loop.finish();
  observer_loop.finish();
  consumer_loop.finish();
  return consumer.efforts();
}

struct Replayed {
  std::vector<float> efforts;
  InputReplayReport report;
};

// The consumer, which the session never ran, fed from it.
Replayed ReplayConsumer(const RecordedSession& session,
                        const std::string& log_path) {
  SimulationEnvironment environment;
  Loop loop{environment, log::LogWriter{log_path, "consumer"},
            Loop::Options{.start = session.start()}};
  TestRobot<Loop> consumer{loop, kTopics, kGain};
  consumer.control_timer().setup_periodic(session.start() + 1ms, 1ms);
  InputReplayReport report = ReplayInputs(loop, environment, session);
  EXPECT_FALSE(loop.recorder().failed()) << loop.recorder().error();
  return {consumer.efforts(), std::move(report)};
}

TEST(InputReplay, ANewConsumerComputesWhatItWouldHaveLiveAndDeterministically) {
  const std::string dir = TempDir("session");
  const std::string out = TempDir("out");
  const std::vector<float> live = RecordSession(dir, out + "/live.tlog");

  const RecordedSession session{dir};
  EXPECT_EQ(session.log_paths().size(), 2u);
  EXPECT_EQ(session.start(), kOrigin);
  EXPECT_EQ(session.end(), kOrigin + std::chrono::milliseconds{kSessionMs});
  const RecordedTopic* sensor = session.find(kTopics.sensor);
  const RecordedTopic* setpoint = session.find(kTopics.setpoint);
  ASSERT_NE(sensor, nullptr);
  ASSERT_NE(setpoint, nullptr);
  // Sent by the producer and received by the observer: kept once.
  EXPECT_EQ(sensor->recorded_by, "producer");
  EXPECT_EQ(sensor->message_count, static_cast<std::size_t>(kSessionMs / 2));
  // Only a reader logged it.
  EXPECT_EQ(setpoint->recorded_by, "observer");
  EXPECT_EQ(setpoint->message_count, 6u);
  EXPECT_EQ(session.find(kTopics.command), nullptr);

  const Replayed first = ReplayConsumer(session, out + "/first.tlog");
  const Replayed second = ReplayConsumer(session, out + "/second.tlog");

  ASSERT_EQ(first.report.fed.size(), 2u);
  EXPECT_EQ(first.report.fed[0].topic, kTopics.sensor);
  EXPECT_EQ(first.report.fed[0].messages, sensor->message_count);
  EXPECT_EQ(first.report.fed[1].topic, kTopics.setpoint);
  EXPECT_EQ(first.report.fed[1].messages, setpoint->message_count);
  EXPECT_TRUE(first.report.unrecorded.empty());

  // The inputs reached it: the error moves as readings and setpoints arrive.
  ASSERT_EQ(first.efforts.size(), static_cast<std::size_t>(kSessionMs));
  EXPECT_NE(first.efforts.front(), first.efforts.back());
  // Delivered when a live reader would have had them.
  EXPECT_EQ(first.efforts, live);
  EXPECT_EQ(Sends(out + "/first.tlog"), Sends(out + "/live.tlog"));
  // And the same every time.
  EXPECT_EQ(second.efforts, first.efforts);
  const std::vector<Sent> sends = Sends(out + "/first.tlog");
  EXPECT_EQ(sends.size(), static_cast<std::size_t>(kSessionMs));
  EXPECT_EQ(Sends(out + "/second.tlog"), sends);

  std::filesystem::remove_all(dir);
  std::filesystem::remove_all(out);
}

struct MessageTimes {
  std::vector<Context> contexts;

  void receive(const Context& context, const EventsTest::SensorMessage&) {
    contexts.push_back(context);
  }
};

TEST(InputReplay, ReaderOnlyLogFeedsAtPublicationRatherThanDelayedReceipt) {
  const std::string dir = TempDir("delayed_reader");
  {
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{dir + "/reader.tlog", "reader"},
              Loop::Options{.tick_period = 10ms, .start = kOrigin}};
    MessageTimes observed;
    watch<EventsTest::SensorMessage, &MessageTimes::receive>(
        loop, kTopics.sensor, &observed);
    loop.run_for(2ms);
    loop.inject(kTopics.sensor, EventsTest::SensorMessage{1, 2.0F, 3.0F});
    loop.run_until(kOrigin + 10ms);
    loop.finish();
    ASSERT_FALSE(loop.recorder().failed()) << loop.recorder().error();
    ASSERT_EQ(observed.contexts.size(), 1u);
    EXPECT_EQ(observed.contexts[0].now, kOrigin + 10ms);
    EXPECT_EQ(observed.contexts[0].publish_time, kOrigin + 2ms);
  }
  const RecordedSession session{dir};
  ASSERT_EQ(session.messages().size(), 1u);
  EXPECT_EQ(session.messages()[0].time, kOrigin + 2ms);

  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{
      environment, SimulatedEventLoop<>::Options{.start = session.start()}};
  MessageTimes replayed;
  watch<EventsTest::SensorMessage, &MessageTimes::receive>(loop, kTopics.sensor,
                                                           &replayed);
  const auto report = ReplayInputs(loop, environment, session);
  ASSERT_EQ(report.fed.size(), 1u);
  EXPECT_EQ(report.fed[0].messages, 1u);
  ASSERT_EQ(replayed.contexts.size(), 1u);
  EXPECT_EQ(replayed.contexts[0].now, kOrigin + 2ms);
  EXPECT_EQ(replayed.contexts[0].publish_time, kOrigin + 2ms);
  std::filesystem::remove_all(dir);
}

// Reads /input/sensor as a type of the same size that is not the one the
// session recorded.
class Misreader {
 public:
  explicit Misreader(Loop& loop) {
    watch<EventsTest::OtherSensorMessage, &Misreader::on_sensor>(
        loop, kTopics.sensor, this);
  }

 private:
  void on_sensor(const Context&, const EventsTest::OtherSensorMessage&) {}
};

TEST(InputReplay, RefusesATopicRecordedWithAnotherSchemaBeforeDispatch) {
  static_assert(sizeof(EventsTest::OtherSensorMessage) ==
                sizeof(EventsTest::SensorMessage));
  const std::string dir = TempDir("mismatch");
  RecordSession(dir, TempDir("mismatch_live") + "/live.tlog");
  const RecordedSession session{dir};

  SimulationEnvironment environment;
  Loop loop{
      environment,
      log::LogWriter{TempDir("mismatch_out") + "/misreader.tlog", "misreader"},
      Loop::Options{.start = session.start()}};
  Misreader misreader{loop};
  EXPECT_THROW(ReplayInputs(loop, environment, session), ReplayError);
  // Refused before anything ran.
  EXPECT_EQ(loop.dispatch_count(), 0u);
  std::filesystem::remove_all(dir);
}

// Reads the recorded sensor and a topic the session never had.
class Wanderer {
 public:
  explicit Wanderer(Loop& loop) {
    watch<EventsTest::SensorMessage, &Wanderer::on_sensor>(loop, kTopics.sensor,
                                                           this);
    watch<EventsTest::SensorMessage, &Wanderer::on_sensor>(
        loop, "/elsewhere/sensor", this);
  }
  int readings{0};

 private:
  void on_sensor(const Context&, const EventsTest::SensorMessage&) {
    ++readings;
  }
};

TEST(InputReplay, ReportsTopicsTheSessionNeverRecorded) {
  const std::string dir = TempDir("unrecorded");
  RecordSession(dir, TempDir("unrecorded_live") + "/live.tlog");
  const RecordedSession session{dir};

  SimulationEnvironment environment;
  Loop loop{
      environment,
      log::LogWriter{TempDir("unrecorded_out") + "/wanderer.tlog", "wanderer"},
      Loop::Options{.start = session.start()}};
  Wanderer wanderer{loop};
  const InputReplayReport report = ReplayInputs(loop, environment, session);
  ASSERT_EQ(report.unrecorded.size(), 1u);
  EXPECT_EQ(report.unrecorded[0], "/elsewhere/sensor");
  ASSERT_EQ(report.fed.size(), 1u);
  EXPECT_EQ(wanderer.readings, kSessionMs / 2);
  std::filesystem::remove_all(dir);
}

}  // namespace
}  // namespace talos::event

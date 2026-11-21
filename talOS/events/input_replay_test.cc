// Input replay (input_replay.h): a program that was not in a recorded
// session, driven from the topics the session logged, computes what it would
// have computed had it been there -- and computes it the same way every time.

#include "talOS/events/input_replay.h"

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "events_test/events_test_message_generated.h"
#include "talOS/events/handles.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/gap_testing.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/session_manifest.h"
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

// Sends one reading from its constructor, before its loop's first dispatch.
class InitialSender {
 public:
  InitialSender(Loop& loop, int value)
      : sensor_{make_sender<EventsTest::SensorMessage>(loop, kTopics.sensor)} {
    sensor_.send(EventsTest::SensorMessage{value, 1.0F, 1.0F});
  }

 private:
  Sender<Loop, EventsTest::SensorMessage> sensor_;
};

// A send made before the first dispatch (a constructor's initial state) is
// published at its loop's origin, which is what its readers saw; a node the
// session did not run is fed it then, so a restarted writer's initial
// message arrives at the restart, not at the session's start.
// REQ-OPS-14
TEST(InputReplay, FeedsAConstructorSendAtItsRunsOrigin) {
  const std::string dir = TempDir("constructor_send");
  const MonotonicTime restart = kOrigin + 50ms;
  {
    // One environment: the restarted writer continues the topic's sequence,
    // as it does on a session's shared memory.
    SimulationEnvironment environment;
    for (const auto& [name, start, value] :
         {std::tuple{"/writer.tlog", kOrigin, 1},
          std::tuple{"/writer_restarted.tlog", restart, 2}}) {
      Loop loop{environment, log::LogWriter{dir + name, "writer"},
                Loop::Options{.start = start}};
      const InitialSender sender{loop, value};
      loop.run_until(start + 10ms);
      loop.finish();
      ASSERT_FALSE(loop.recorder().failed()) << loop.recorder().error();
    }
  }
  const RecordedSession session{dir};
  SimulationEnvironment environment;
  SimulatedEventLoop<> loop{
      environment, SimulatedEventLoop<>::Options{.start = session.start()}};
  MessageTimes replayed;
  watch<EventsTest::SensorMessage, &MessageTimes::receive>(loop, kTopics.sensor,
                                                           &replayed);
  ReplayInputs(loop, environment, session);
  ASSERT_EQ(replayed.contexts.size(), 2u);
  EXPECT_EQ(replayed.contexts[0].publish_time, kOrigin);
  EXPECT_EQ(replayed.contexts[1].publish_time, restart);
  EXPECT_EQ(replayed.contexts[1].now, restart);
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

// Replaces `path`'s records from the first dispatch that starts at or after
// `at` through the next two with the GAP record the writer would write for
// them (log/gap_testing.h).
log::LogGap MarkGap(const std::string& path, MonotonicTime at) {
  std::vector<log::RecordHeader> headers;
  {
    log::LogReader reader{path};
    log::LogReader::Record record;
    while (reader.next(record)) headers.push_back(record.header);
  }
  const auto starts = [&](std::size_t i) {
    const auto kind = static_cast<EventKind>(headers[i].kind);
    return (kind == EventKind::TIMER || kind == EventKind::MESSAGE) &&
           (i == 0 ||
            headers[i].dispatch_index != headers[i - 1].dispatch_index);
  };
  std::size_t first = 0;
  while (first < headers.size() &&
         !(starts(first) && headers[first].now_ns >= at.nanos())) {
    ++first;
  }
  std::size_t end = first + 1;
  for (int dispatches = 0; end < headers.size(); ++end) {
    if (starts(end) && ++dispatches == 3) break;
  }
  if (end >= headers.size()) {
    throw std::invalid_argument("MarkGap: " + path + " ends before the gap");
  }
  const std::string copy = path + ".gap";
  const log::LogGap gap = log::CopyWithGap(path, copy, first, end);
  std::filesystem::rename(copy, path);
  return gap;
}

// The consumer's control ticks (one per millisecond from the origin) before
// `at`.
std::size_t TicksBefore(MonotonicTime at) {
  std::size_t ticks = 0;
  for (int ms = 1; ms <= kSessionMs; ++ms) {
    if (kOrigin + std::chrono::milliseconds{ms} < at) ++ticks;
  }
  return ticks;
}

// How long the consumer's construction took live in RecordLateConsumer: its
// run, and so its control timer's schedule, begins this long after its
// origin.
constexpr Duration kLateRun = 5ms;

// Records the producer and the consumer into `session_dir`, the consumer's
// run starting kLateRun after its origin, as a slow constructor makes it.
void RecordLateConsumer(const std::string& session_dir) {
  SimulationEnvironment environment;
  Loop producer_loop{environment,
                     log::LogWriter{session_dir + "/producer.tlog", "producer"},
                     Loop::Options{.start = kOrigin}};
  Loop consumer_loop{
      environment, log::LogWriter{session_dir + "/consumer.tlog", "consumer"},
      Loop::Options{.start = kOrigin, .run_start = kOrigin + kLateRun}};
  Producer producer{producer_loop};
  TestRobot<Loop> consumer{consumer_loop, kTopics, kGain};
  consumer.control_timer().setup_periodic(kOrigin + 1ms, 1ms);
  for (int ms = 1; ms <= kSessionMs; ++ms) {
    const MonotonicTime now = kOrigin + std::chrono::milliseconds{ms};
    producer_loop.run_until(now);
    consumer_loop.run_until(now);
  }
  producer_loop.finish();
  consumer_loop.finish();
}

// Reads the session's setpoints and nothing else.
class SetpointWatcher {
 public:
  explicit SetpointWatcher(Loop& loop) {
    watch<EventsTest::SetpointMessage, &SetpointWatcher::on_setpoint>(
        loop, kTopics.setpoint, this);
  }

 private:
  void on_setpoint(const Context&, const EventsTest::SetpointMessage&) {}
};

// A run fed from a log with a GAP record ends before the gap's start, the
// earliest time it names, and reports the topic, the log and that time: what
// came before is fed as without the gap. Which logs count: for a program the
// session did not run, every log that recorded a topic it watches (the
// observer, the only log of /input/setpoint; the producer, the writer of
// /input/sensor); for one it ran, its own logs for a topic it watched live, so
// the producer's earlier gap does not stop the observer. A gap in a log that
// recorded nothing it watches does not stop it; of two gaps, in two logs or
// in one, the earlier. A program whose own log has a gap before its first
// timer firing still replays on its recorded timeline.
// REQ-OPS-14
TEST(InputReplay, EndsBeforeTheFirstGapOfALogItIsFedFrom) {
  {
    SCOPED_TRACE("another process's log");
    const std::string dir = TempDir("gap_observer");
    const std::string out = TempDir("gap_observer_out");
    const std::vector<float> live = RecordSession(dir, out + "/live.tlog");
    const log::LogGap gap = MarkGap(dir + "/observer.tlog", kOrigin + 20ms);
    const MonotonicTime at = MonotonicTime::from_nanos(gap.start_ns);
    const RecordedSession session{dir};
    const Replayed replayed = ReplayConsumer(session, out + "/replayed.tlog");
    ASSERT_TRUE(replayed.report.stopped_at_gap.has_value());
    const InputReplayReport::Gap& stopped =
        replayed.report.stopped_at_gap.value();
    EXPECT_EQ(stopped.log, dir + "/observer.tlog");
    EXPECT_EQ(stopped.at.nanos(), gap.start_ns);
    // producer.tlog holds /input/sensor in full (intact and exited), so only
    // /input/setpoint (recorded solely by observer.tlog) stops at the gap.
    EXPECT_EQ(stopped.topic, kTopics.setpoint);
    const std::size_t ticks = TicksBefore(at);
    ASSERT_GT(ticks, 0u);
    ASSERT_LT(ticks, live.size());
    EXPECT_EQ(
        replayed.efforts,
        std::vector<float>(live.begin(),
                           live.begin() + static_cast<std::ptrdiff_t>(ticks)));
    const std::vector<Sent> live_sends = Sends(out + "/live.tlog");
    EXPECT_EQ(Sends(out + "/replayed.tlog"),
              std::vector<Sent>(
                  live_sends.begin(),
                  live_sends.begin() + static_cast<std::ptrdiff_t>(ticks)));
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("a log that feeds it nothing");
    const std::string dir = TempDir("gap_unfed");
    const std::string out = TempDir("gap_unfed_out");
    RecordSession(dir, out + "/live.tlog");
    MarkGap(dir + "/producer.tlog", kOrigin + 20ms);
    const RecordedSession session{dir};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/setpoints.tlog", "setpoints"},
              Loop::Options{.start = session.start()}};
    SetpointWatcher watcher{loop};
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    EXPECT_FALSE(report.stopped_at_gap.has_value());
    ASSERT_EQ(report.fed.size(), 1u);
    EXPECT_EQ(report.fed[0].messages, 6u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("its own log");
    const std::string dir = TempDir("gap_own");
    const std::string out = TempDir("gap_own_out");
    RecordSession(dir, out + "/live.tlog");
    MarkGap(dir + "/producer.tlog", kOrigin + 10ms);
    const log::LogGap gap = MarkGap(dir + "/observer.tlog", kOrigin + 30ms);
    // What it received live, at drains before the gap's start.
    std::size_t received = 0;
    {
      log::LogReader reader{dir + "/observer.tlog"};
      log::LogReader::Record record;
      while (reader.next(record)) {
        if (record.header.kind ==
                static_cast<std::uint16_t>(EventKind::MESSAGE) &&
            record.header.event_time_ns < gap.start_ns) {
          ++received;
        }
      }
    }
    ASSERT_GT(received, 0u);
    const RecordedSession session{dir, "observer"};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/observer.tlog", "observer"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    Observer observer{loop};
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    ASSERT_TRUE(report.stopped_at_gap.has_value());
    EXPECT_EQ(report.stopped_at_gap.value().log, dir + "/observer.tlog");
    EXPECT_EQ(report.stopped_at_gap.value().at.nanos(), gap.start_ns);
    ASSERT_EQ(report.fed.size(), 2u);
    EXPECT_EQ(report.fed[0].messages + report.fed[1].messages, received);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("two gaps");
    const std::string dir = TempDir("gap_two");
    const std::string out = TempDir("gap_two_out");
    RecordSession(dir, out + "/live.tlog");
    // The later gap is in the log read first.
    MarkGap(dir + "/observer.tlog", kOrigin + 30ms);
    const log::LogGap gap = MarkGap(dir + "/producer.tlog", kOrigin + 14ms);
    const RecordedSession session{dir};
    const Replayed replayed = ReplayConsumer(session, out + "/replayed.tlog");
    ASSERT_TRUE(replayed.report.stopped_at_gap.has_value());
    EXPECT_EQ(replayed.report.stopped_at_gap.value().log,
              dir + "/producer.tlog");
    EXPECT_EQ(replayed.report.stopped_at_gap.value().topic, kTopics.sensor);
    EXPECT_EQ(replayed.report.stopped_at_gap.value().at.nanos(), gap.start_ns);
    EXPECT_EQ(replayed.efforts.size(),
              TicksBefore(MonotonicTime::from_nanos(gap.start_ns)));
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("two gaps in one log");
    const std::string dir = TempDir("gap_two_one_log");
    const std::string out = TempDir("gap_two_one_log_out");
    RecordSession(dir, out + "/live.tlog");
    MarkGap(dir + "/observer.tlog", kOrigin + 30ms);
    const log::LogGap gap = MarkGap(dir + "/observer.tlog", kOrigin + 14ms);
    const RecordedSession session{dir};
    const Replayed replayed = ReplayConsumer(session, out + "/replayed.tlog");
    ASSERT_TRUE(replayed.report.stopped_at_gap.has_value());
    EXPECT_EQ(replayed.report.stopped_at_gap.value().log,
              dir + "/observer.tlog");
    EXPECT_EQ(replayed.report.stopped_at_gap.value().at.nanos(), gap.start_ns);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    // Its timers fire where they fired live, even with no firing to read
    // before the gap: the stop comes before its first firing, as live.
    SCOPED_TRACE("its own log, a gap before its first timer firing");
    const std::string dir = TempDir("gap_own_first");
    const std::string out = TempDir("gap_own_first_out");
    RecordLateConsumer(dir);
    const log::LogGap gap = MarkGap(dir + "/consumer.tlog", kOrigin);
    const RecordedSession session{dir, "consumer"};
    EXPECT_EQ((session.run_start() - kOrigin).count(),
              std::chrono::nanoseconds{kLateRun}.count());
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/consumer.tlog", "consumer"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    TestRobot<Loop> consumer{loop, kTopics, kGain};
    consumer.control_timer().setup_periodic(kOrigin + 1ms, 1ms);
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    ASSERT_TRUE(report.stopped_at_gap.has_value());
    EXPECT_EQ(report.stopped_at_gap.value().at.nanos(), gap.start_ns);
    ASSERT_LT(gap.start_ns, (kOrigin + kLateRun + 1ms).nanos());
    EXPECT_TRUE(consumer.efforts().empty())
        << consumer.efforts().size() << " control firings before the stop";
    EXPECT_TRUE(Sends(out + "/consumer.tlog").empty());
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// A program with a timer every millisecond and no input.
class Ticker {
 public:
  explicit Ticker(Loop& loop)
      : tick_{make_timer<&Ticker::on_tick>(loop, "tick", this)} {
    tick_.setup_periodic(loop.monotonic_now() + 1ms, 1ms);
  }

 private:
  void on_tick(const Context&) {}

  Timer<Loop> tick_;
};

// A gap in a log a run is fed from that starts after the replayed program's
// recorded exit (loop_end()) is never reached: the run neither stops at it
// nor runs past its exit. Here the consumer ran live as a ticker that read
// nothing and exited at kExit; the producer ran on and has a gap after that.
// Replayed, the consumer reads the producer's sensor, every reading before
// kExit, and ticks to kExit.
// REQ-OPS-14
TEST(InputReplay, AGapAfterTheReplayedRunsExitDoesNotStopIt) {
  constexpr MonotonicTime kExit = kOrigin + 30ms;
  const std::string dir = TempDir("gap_after_exit");
  const std::string out = TempDir("gap_after_exit_out");
  {
    SimulationEnvironment environment;
    Loop producer_loop{environment,
                       log::LogWriter{dir + "/producer.tlog", "producer"},
                       Loop::Options{.start = kOrigin}};
    Loop consumer_loop{environment,
                       log::LogWriter{dir + "/consumer.tlog", "consumer"},
                       Loop::Options{.start = kOrigin}};
    Producer producer{producer_loop};
    Ticker ticker{consumer_loop};
    consumer_loop.run_until(kExit);
    consumer_loop.finish();
    producer_loop.run_until(kOrigin + std::chrono::milliseconds{kSessionMs});
    producer_loop.finish();
  }
  const log::LogGap gap = MarkGap(dir + "/producer.tlog", kOrigin + 40ms);
  ASSERT_GT(gap.start_ns, kExit.nanos());
  const RecordedSession session{dir, "consumer"};
  ASSERT_EQ(session.loop_end(), kExit);
  const RecordedTopic* sensor = session.find(kTopics.sensor);
  ASSERT_NE(sensor, nullptr);
  ASSERT_TRUE(sensor->first_gap.has_value());
  EXPECT_EQ(sensor->first_gap->nanos(), gap.start_ns);

  SimulationEnvironment environment;
  Loop loop{environment, log::LogWriter{out + "/consumer.tlog", "consumer"},
            Loop::Options{.start = session.loop_start(),
                          .run_start = session.run_start()}};
  TestRobot<Loop> consumer{loop, kTopics, kGain};
  consumer.control_timer().setup_periodic(kOrigin + 1ms, 1ms);
  const InputReplayReport report = ReplayInputs(loop, environment, session);
  EXPECT_FALSE(report.stopped_at_gap.has_value());
  ASSERT_EQ(report.fed.size(), 1u);
  EXPECT_EQ(report.fed[0].topic, kTopics.sensor);
  // The readings every 2 ms up to the exit, the one at it included.
  EXPECT_EQ(report.fed[0].messages, 15u);
  EXPECT_EQ(consumer.efforts().size(), TicksBefore(kExit + 1ns));
  EXPECT_EQ(loop.monotonic_now(), kExit);
  std::filesystem::remove_all(dir);
  std::filesystem::remove_all(out);
}

// Flips one bit of the header of the record at `offset`, so a reader skips
// it as damage and resumes at the next.
void DamageRecord(const std::string& path, std::uint64_t offset) {
  std::fstream file{path, std::ios::binary | std::ios::in | std::ios::out};
  const auto at = static_cast<std::streamoff>(
      offset + offsetof(log::RecordHeader, dispatch_index));
  char byte = 0;
  file.seekg(at);
  file.read(&byte, 1);
  byte = static_cast<char>(byte ^ 1);
  file.seekp(at);
  file.write(&byte, 1);
}

std::vector<std::uint64_t> RecordOffsets(const std::string& path) {
  log::LogReader reader{path};
  std::vector<std::uint64_t> offsets;
  log::LogReader::Record record;
  while (reader.next(record)) offsets.push_back(record.offset);
  return offsets;
}

void CreateEmptyFile(const std::string& path) {
  const std::ofstream file{path};
  ASSERT_TRUE(file.good()) << path;
}

// Splits the log at `path` as a byte cap rotates one, once per entry of
// `cuts` (ascending): its records before the first at or after cuts[0] (its
// now_ns) go to `<stem>.segment0.tlog`, those from there to before cuts[1] to
// `<stem>.segment1.tlog`, and so on; the rest stay at `path`, each behind the
// log's header, manifest and schema block.
void RotateAt(const std::string& path, const std::vector<MonotonicTime>& cuts) {
  std::vector<std::uint64_t> ends;
  {
    log::LogReader reader{path};
    log::LogReader::Record record;
    while (ends.size() < cuts.size() && reader.next(record)) {
      while (ends.size() < cuts.size() &&
             record.header.now_ns >= cuts[ends.size()].nanos()) {
        ends.push_back(record.offset);
      }
    }
  }
  ASSERT_EQ(ends.size(), cuts.size()) << path << " ends before a cut";
  const std::uint64_t prefix = RecordOffsets(path).front();
  std::vector<char> bytes;
  {
    std::ifstream in{path, std::ios::binary};
    bytes.assign(std::istreambuf_iterator<char>{in},
                 std::istreambuf_iterator<char>{});
  }
  const auto write = [&](const std::string& to, std::uint64_t from,
                         std::uint64_t end) {
    std::ofstream out{to, std::ios::binary | std::ios::trunc};
    out.write(bytes.data(), static_cast<std::streamsize>(prefix));
    out.write(bytes.data() + from, static_cast<std::streamsize>(end - from));
  };
  std::uint64_t from = prefix;
  for (std::size_t k = 0; k < ends.size(); ++k) {
    ASSERT_GT(ends[k], from) << "segment " << k << " would be empty";
    write(log::LogSegmentPath(path, k), from, ends[k]);
    from = ends[k];
  }
  ASSERT_LT(from, bytes.size());
  write(path, from, bytes.size());
}

// Counts the readings it receives.
class SensorWatcher {
 public:
  explicit SensorWatcher(Loop& loop) {
    watch<EventsTest::SensorMessage, &SensorWatcher::on_sensor>(
        loop, kTopics.sensor, this);
  }
  int readings{0};

 private:
  void on_sensor(const Context&, const EventsTest::SensorMessage&) {
    ++readings;
  }
};

// Registers what WriterLog's log records: a sender of /input/sensor (id 0)
// and a timer (id 1).
struct SensorWriter {
  explicit SensorWriter(Loop& loop) {
    make_sender<EventsTest::SensorMessage>(loop, kTopics.sensor);
    loop.register_timer("tick", Thunk{});
  }
};

// The manifest `Program`'s registrations make, without running it (its
// loop's log, at `unused`, is never started, so never created).
template <typename Program>
Manifest ManifestOf(const std::string& unused) {
  SimulationEnvironment environment;
  Loop loop{environment, log::LogWriter{unused, "manifest"}};
  Program program{loop};
  return loop.manifest();
}

const MonotonicTime kDrain = kOrigin + 10ms;

// A reader's log, as a realtime loop writes one: one drain at kDrain hands
// its watcher four readings published 1 to 4 ms after the origin, the
// topic's sequences 0 to 3 (a topic's first message is sequence 0), each
// dispatch sampling its own now 5 us after the last, so every dispatch's now
// is after its event time. Returns each record's offset.
std::vector<std::uint64_t> ReaderLog(const std::string& dir) {
  const std::string path = dir + "/reader.tlog";
  log::LogWriter writer{path, "reader",
                        log::LogWriterOptions{.background = false}};
  writer.start(ManifestOf<SensorWatcher>(dir + "/unused.tlog"), kOrigin);
  for (int i = 1; i <= 4; ++i) {
    const EventsTest::SensorMessage reading{i, 1.0F, 1.0F};
    writer.dispatch(
        Context{.kind = EventKind::MESSAGE,
                .source_id = 0,
                .dispatch_index = static_cast<std::uint64_t>(i),
                .event_time = kDrain,
                .now = kDrain + std::chrono::microseconds{5 * i},
                .publish_time = kOrigin + std::chrono::milliseconds{i},
                .sequence = static_cast<std::uint64_t>(i - 1)},
        std::as_bytes(std::span{&reading, 1}));
  }
  writer.finish(Context{.kind = EventKind::EXIT,
                        .dispatch_index = 5,
                        .event_time = kDrain + 25us,
                        .now = kDrain + 25us});
  writer.flush();
  return RecordOffsets(path);
}

// A writer's log: a tick 1 to 4 ms after the origin, each sending a reading,
// the sequences ReaderLog's receipts carry. Returns each record's offset
// (tick, send, ..., EXIT).
std::vector<std::uint64_t> WriterLog(const std::string& dir) {
  const std::string path = dir + "/writer.tlog";
  log::LogWriter writer{path, "writer",
                        log::LogWriterOptions{.background = false}};
  writer.start(ManifestOf<SensorWriter>(dir + "/unused.tlog"), kOrigin);
  for (int i = 1; i <= 4; ++i) {
    const Context tick{.kind = EventKind::TIMER,
                       .source_id = 1,
                       .dispatch_index = static_cast<std::uint64_t>(i),
                       .event_time = kOrigin + std::chrono::milliseconds{i},
                       .now = kOrigin + std::chrono::milliseconds{i},
                       .sequence = 1};
    writer.dispatch(tick, {});
    const EventsTest::SensorMessage reading{i, 1.0F, 1.0F};
    writer.send(tick, 0, std::as_bytes(std::span{&reading, 1}), 0,
                static_cast<std::uint64_t>(i - 1));
  }
  writer.finish(Context{.kind = EventKind::EXIT,
                        .dispatch_index = 5,
                        .event_time = kOrigin + 5ms,
                        .now = kOrigin + 5ms});
  writer.flush();
  return RecordOffsets(path);
}

// A damaged stretch (records lost between intact ones, which no GAP record
// marks) ends a run fed from its log as a gap does, at the earliest time a
// record lost there could be fed at, or refuses the log where nothing bounds
// that. Rows: a reader's receipts, fed at their publication times, are
// bounded by the last intact receipt's, not by when its dispatch ran (a
// drain's dispatches run after the drain's time), and by nothing when the
// lost one was its first; a writer's sends, fed at their records' now, by
// the last intact record's; the replayed program's own receipts, fed at
// their drain's time, which need not follow the log's order, by nothing.
// Nothing bounding them, the log is refused, naming it and the byte offset.
// REQ-OPS-14
TEST(InputReplay, StopsAtOrRefusesDamageByWhenItsLostRecordsWereFed) {
  {
    SCOPED_TRACE("a reader's receipts");
    const std::string dir = TempDir("damage_reader");
    const std::string out = TempDir("damage_reader_out");
    const std::vector<std::uint64_t> offsets = ReaderLog(dir);
    DamageRecord(dir + "/reader.tlog", offsets[2]);  // the third reading
    const RecordedSession session{dir};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/watcher.tlog", "watcher"},
              Loop::Options{.start = session.start()}};
    SensorWatcher watcher{loop};
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    ASSERT_TRUE(report.stopped_at_gap.has_value());
    EXPECT_EQ(report.stopped_at_gap->log, dir + "/reader.tlog");
    // The second reading's publication: the third's is no earlier.
    EXPECT_EQ(report.stopped_at_gap->at, kOrigin + 2ms);
    ASSERT_TRUE(report.stopped_at_gap->damage_offset.has_value());
    EXPECT_EQ(report.stopped_at_gap->damage_offset.value(), offsets[2]);
    // The first only: neither the second, at the stop, nor the fourth,
    // past the lost third.
    EXPECT_EQ(watcher.readings, 1);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    // Its first receipt lost: a first receipt can carry any earlier
    // publication time, so nothing bounds it.
    SCOPED_TRACE("a reader's receipts, its first one lost");
    const std::string dir = TempDir("damage_first");
    const std::string out = TempDir("damage_first_out");
    const std::vector<std::uint64_t> offsets = ReaderLog(dir);
    DamageRecord(dir + "/reader.tlog", offsets[0]);
    const RecordedSession session{dir};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/watcher.tlog", "watcher"},
              Loop::Options{.start = session.start()}};
    SensorWatcher watcher{loop};
    try {
      ReplayInputs(loop, environment, session);
      ADD_FAILURE() << "fed a reader's receipts after its lost first one";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(dir + "/reader.tlog"), std::string::npos) << what;
      EXPECT_NE(what.find("byte " + std::to_string(offsets[0])),
                std::string::npos)
          << what;
    }
    EXPECT_EQ(watcher.readings, 0);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("a writer's sends");
    const std::string dir = TempDir("damage_writer");
    const std::string out = TempDir("damage_writer_out");
    const std::vector<std::uint64_t> offsets = WriterLog(dir);
    DamageRecord(dir + "/writer.tlog", offsets[5]);  // the third send
    const RecordedSession session{dir};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/watcher.tlog", "watcher"},
              Loop::Options{.start = session.start()}};
    SensorWatcher watcher{loop};
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    ASSERT_TRUE(report.stopped_at_gap.has_value());
    // The third tick's now: the third send's publication is no earlier.
    EXPECT_EQ(report.stopped_at_gap->at, kOrigin + 3ms);
    ASSERT_TRUE(report.stopped_at_gap->damage_offset.has_value());
    EXPECT_EQ(report.stopped_at_gap->damage_offset.value(), offsets[5]);
    EXPECT_EQ(watcher.readings, 2);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    // The bound carries across the segments of one log: damage right after
    // a later segment's header is bounded by the previous segment's last
    // intact record, as it would be in one file.
    SCOPED_TRACE("a writer's sends, at a later segment's head");
    const std::string dir = TempDir("damage_segment");
    const std::string out = TempDir("damage_segment_out");
    WriterLog(dir);
    const std::string path = dir + "/writer.tlog";
    RotateAt(path, {kOrigin + 3ms});  // the third tick starts a segment
    const std::vector<std::uint64_t> offsets = RecordOffsets(path);
    const std::uint64_t head = offsets[0];
    // Damage both the tick and its send so a send is actually lost in the
    // stretch starting at `head` (damaging only the tick leaves all sends
    // contiguous in sequence order, which LOG-29(a) proves lossless).
    DamageRecord(path, offsets[0]);
    DamageRecord(path, offsets[1]);
    const RecordedSession session{dir};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/watcher.tlog", "watcher"},
              Loop::Options{.start = session.start()}};
    SensorWatcher watcher{loop};
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    ASSERT_TRUE(report.stopped_at_gap.has_value());
    EXPECT_EQ(report.stopped_at_gap->log, path);
    // The second send's now, the previous segment's last record.
    EXPECT_EQ(report.stopped_at_gap->at, kOrigin + 2ms);
    ASSERT_TRUE(report.stopped_at_gap->damage_offset.has_value());
    EXPECT_EQ(report.stopped_at_gap->damage_offset.value(), head);
    EXPECT_EQ(watcher.readings, 1);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("its own receipts");
    const std::string dir = TempDir("damage_own");
    const std::string out = TempDir("damage_own_out");
    const std::vector<std::uint64_t> offsets = ReaderLog(dir);
    DamageRecord(dir + "/reader.tlog", offsets[2]);
    const RecordedSession session{dir, "reader"};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/reader.tlog", "reader"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    SensorWatcher watcher{loop};
    try {
      ReplayInputs(loop, environment, session);
      ADD_FAILURE() << "fed its own receipts across damage";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(dir + "/reader.tlog"), std::string::npos) << what;
      EXPECT_NE(what.find("byte " + std::to_string(offsets[2])),
                std::string::npos)
          << what;
    }
    EXPECT_EQ(watcher.readings, 0);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

struct WatcherRun {
  InputReplayReport report;
  int readings{0};
};

// A watcher of the sensor, which the session never ran, fed from `session`.
WatcherRun ReplayWatcher(const RecordedSession& session,
                         const std::string& out) {
  SimulationEnvironment environment;
  Loop loop{environment, log::LogWriter{out + "/watcher.tlog", "watcher"},
            Loop::Options{.start = session.start()}};
  SensorWatcher watcher{loop};
  InputReplayReport report = ReplayInputs(loop, environment, session);
  return {std::move(report), watcher.readings};
}

// A reader's damaged stretch lost receipts whose sequences lie between its
// intact receipts around it (a watcher receives in sequence order; from the
// topic's first, 0, when none precedes it). Where the session's other logs
// (the writer's sends) hold every one of those, the stretch lost nothing the
// feed lacks, and a node that did not watch the topic live is fed it in
// full; where they miss one, even with no loss of their own to show (a
// damaged tail), or no intact receipt follows the stretch to cap its range,
// the reader's damage still stops the run, naming its log and that
// stretch's offset.
// REQ-OPS-14
TEST(InputReplay, AReadersDamageDoesNotStopATopicTheOtherLogsHoldInFull) {
  {
    SCOPED_TRACE("the writer holds every send");
    const std::string dir = TempDir("held_reader");
    const std::string out = TempDir("held_reader_out");
    WriterLog(dir);
    DamageRecord(dir + "/reader.tlog", ReaderLog(dir)[2]);  // the third
    const RecordedSession session{dir};
    const WatcherRun run = ReplayWatcher(session, out);
    EXPECT_FALSE(run.report.stopped_at_gap.has_value());
    ASSERT_EQ(run.report.fed.size(), 1u);
    EXPECT_EQ(run.report.fed[0].recorded_by, "writer");
    EXPECT_EQ(run.report.fed[0].messages, 4u);
    EXPECT_EQ(run.readings, 4);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("the writer holds every send, the reader's first lost");
    const std::string dir = TempDir("held_first");
    const std::string out = TempDir("held_first_out");
    WriterLog(dir);
    DamageRecord(dir + "/reader.tlog", ReaderLog(dir)[0]);
    const RecordedSession session{dir};
    try {
      const WatcherRun run = ReplayWatcher(session, out);
      EXPECT_FALSE(run.report.stopped_at_gap.has_value());
      EXPECT_EQ(run.readings, 4);
    } catch (const ReplayError& e) {
      ADD_FAILURE() << e.what();
    }
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("the writer lost the same send");
    const std::string dir = TempDir("held_both");
    const std::string out = TempDir("held_both_out");
    DamageRecord(dir + "/writer.tlog", WriterLog(dir)[5]);  // the third send
    const std::uint64_t damaged = ReaderLog(dir)[2];
    DamageRecord(dir + "/reader.tlog", damaged);
    const RecordedSession session{dir};
    const WatcherRun run = ReplayWatcher(session, out);
    ASSERT_TRUE(run.report.stopped_at_gap.has_value());
    // The reader's bound, the second reading's publication, comes before
    // the writer's, the third tick's now.
    EXPECT_EQ(run.report.stopped_at_gap->log, dir + "/reader.tlog");
    EXPECT_EQ(run.report.stopped_at_gap->at, kOrigin + 2ms);
    EXPECT_EQ(run.report.stopped_at_gap->damage_offset, damaged);
    EXPECT_EQ(run.readings, 1);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    // Cut at its third send: a log that only ends early shows no loss of
    // its own, but does not hold the send the reader's stretch lost.
    SCOPED_TRACE("the writer's log ends before the stretch");
    const std::string dir = TempDir("held_short");
    const std::string out = TempDir("held_short_out");
    std::filesystem::resize_file(dir + "/writer.tlog", WriterLog(dir)[5]);
    const std::uint64_t damaged = ReaderLog(dir)[2];
    DamageRecord(dir + "/reader.tlog", damaged);
    const RecordedSession session{dir};
    const WatcherRun run = ReplayWatcher(session, out);
    ASSERT_TRUE(run.report.stopped_at_gap.has_value());
    EXPECT_EQ(run.report.stopped_at_gap->log, dir + "/reader.tlog");
    EXPECT_EQ(run.report.stopped_at_gap->at, kOrigin + 2ms);
    EXPECT_EQ(run.report.stopped_at_gap->damage_offset, damaged);
    EXPECT_EQ(run.readings, 1);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    // Its last receipt lost, only its EXIT after: nothing in the reader's log
    // caps the sequences the stretch lost. When the writer's log lacks EXIT
    // (incomplete writer), the topic cannot be proven complete and still stops;
    // when the writer's log is intact and exited, the complete writer proves
    // the topic whole and replays all 4 readings.
    SCOPED_TRACE("no receipt follows the reader's stretch, incomplete writer");
    const std::string dir = TempDir("held_last_incomplete_writer");
    const std::string out = TempDir("held_last_incomplete_writer_out");
    const std::vector<std::uint64_t> writer_offsets = WriterLog(dir);
    // Drop the writer's EXIT so the writer does not prove completeness.
    std::filesystem::resize_file(dir + "/writer.tlog", writer_offsets.back());
    const std::uint64_t damaged = ReaderLog(dir)[3];
    DamageRecord(dir + "/reader.tlog", damaged);
    const RecordedSession session{dir};
    const WatcherRun run = ReplayWatcher(session, out);
    ASSERT_TRUE(run.report.stopped_at_gap.has_value());
    EXPECT_EQ(run.report.stopped_at_gap->log, dir + "/reader.tlog");
    // The third reading's publication.
    EXPECT_EQ(run.report.stopped_at_gap->at, kOrigin + 3ms);
    EXPECT_EQ(run.report.stopped_at_gap->damage_offset, damaged);
    EXPECT_EQ(run.readings, 2);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("no receipt follows the reader's stretch, complete writer");
    const std::string dir = TempDir("held_last_complete_writer");
    const std::string out = TempDir("held_last_complete_writer_out");
    WriterLog(dir);
    const std::uint64_t damaged = ReaderLog(dir)[3];
    DamageRecord(dir + "/reader.tlog", damaged);
    const RecordedSession session{dir};
    const WatcherRun run = ReplayWatcher(session, out);
    EXPECT_FALSE(run.report.stopped_at_gap.has_value());
    EXPECT_EQ(run.readings, 4);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    // The first stretch (the first receipt) is held; the second (the
    // third) is not, the writer's log ending before it: the stop names the
    // second's offset.
    SCOPED_TRACE("a held stretch, then one not held");
    const std::string dir = TempDir("held_second");
    const std::string out = TempDir("held_second_out");
    std::filesystem::resize_file(dir + "/writer.tlog", WriterLog(dir)[5]);
    const std::vector<std::uint64_t> offsets = ReaderLog(dir);
    DamageRecord(dir + "/reader.tlog", offsets[0]);
    DamageRecord(dir + "/reader.tlog", offsets[2]);
    const RecordedSession session{dir};
    const WatcherRun run = ReplayWatcher(session, out);
    ASSERT_TRUE(run.report.stopped_at_gap.has_value());
    EXPECT_EQ(run.report.stopped_at_gap->log, dir + "/reader.tlog");
    EXPECT_EQ(run.report.stopped_at_gap->at, kOrigin + 2ms);
    EXPECT_EQ(run.report.stopped_at_gap->damage_offset, offsets[2]);
    EXPECT_EQ(run.readings, 1);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// Writes the session manifest the launcher would, naming each member's log
// by the path it had when recorded (elsewhere: matched by file name), with
// `observer` (JSON members) for the observer.
void WriteManifest(const std::string& dir, const std::string& observer) {
  const std::uint64_t id = log::LogReader{dir + "/observer.tlog"}.session_id();
  std::ofstream{dir + "/manifest.json"}
      << "{\n  \"session_id\": " << id << ",\n  \"nodes\": [\n"
      << "    {\"name\": \"producer\", \"log_paths\": "
         "[\"/robot/logs/7/producer.tlog\"], \"log_gap\": false, "
         "\"log_capped\": false},\n"
      << "    {\"name\": \"observer\", \"log_paths\": "
         "[\"/robot/logs/7/observer.tlog\"], "
      << observer << "}\n  ]\n}\n";
}

// A log the byte cap cost records (its `<log>.capped` marker, beside the log
// a segment belongs to, or the session manifest's log_capped for its member)
// may have lost a GAP record with them, and a run's log without its EXIT
// record, of a member the manifest marks log_gap, may have lost one to its
// process's end: input replay refuses, before anything runs, to feed a
// watched topic from either, naming the log and the sign it found. A
// log_gap member's log with its EXIT holds a GAP record for every gap, so it
// is fed up to its first, its segments a byte cap rotated out included (the
// run's EXIT is in its newest file). A capped log that feeds the program
// nothing does not stop it.
// REQ-OPS-14
TEST(InputReplay, RefusesToFeedFromACappedLogOrALogGapRunCutShort) {
  struct Row {
    const char* name;
    const char* refused;  // the log named, in the session directory
    const char* sign;
  };
  const Row rows[] = {
      {"capped", "observer.tlog", "observer.tlog.capped"},
      {"a capped log's segment", "observer.segment2.tlog",
       "observer.tlog.capped"},
      {"log_capped", "observer.tlog", "log_capped"},
      {"log_gap, without its EXIT", "observer.tlog", "log_gap"},
  };
  for (const Row& row : rows) {
    SCOPED_TRACE(row.name);
    const std::string dir = TempDir("refused");
    const std::string out = TempDir("refused_out");
    RecordSession(dir, out + "/live.tlog");
    const std::string sign{row.sign};
    if (sign == "log_capped") {
      WriteManifest(dir, "\"log_gap\": false, \"log_capped\": true");
    } else if (sign == "log_gap") {
      WriteManifest(dir, "\"log_gap\": true, \"log_capped\": false");
      // Its last record is its EXIT.
      std::filesystem::resize_file(
          dir + "/observer.tlog", RecordOffsets(dir + "/observer.tlog").back());
    } else {
      if (std::string{row.refused} != "observer.tlog") {
        std::filesystem::rename(dir + "/observer.tlog",
                                dir + "/" + row.refused);
      }
      CreateEmptyFile(dir + "/" + sign);
    }
    const RecordedSession session{dir};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/consumer.tlog", "consumer"},
              Loop::Options{.start = session.start()}};
    TestRobot<Loop> consumer{loop, kTopics, kGain};
    consumer.control_timer().setup_periodic(session.start() + 1ms, 1ms);
    try {
      ReplayInputs(loop, environment, session);
      ADD_FAILURE() << "fed from " << row.refused;
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(dir + "/" + row.refused), std::string::npos) << what;
      EXPECT_NE(what.find(sign), std::string::npos) << what;
    }
    EXPECT_EQ(loop.dispatch_count(), 0u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  for (const bool rotated : {false, true}) {
    // Rotated: its run's EXIT is in its newest file, not in the segment.
    SCOPED_TRACE(rotated ? "log_gap, rotated, with its GAP record and its EXIT"
                         : "log_gap, with its GAP record and its EXIT");
    const std::string dir = TempDir("log_gap_exit");
    const std::string out = TempDir("log_gap_exit_out");
    RecordSession(dir, out + "/live.tlog");
    const log::LogGap gap = MarkGap(dir + "/observer.tlog", kOrigin + 20ms);
    if (rotated) RotateAt(dir + "/observer.tlog", {kOrigin + 10ms});
    WriteManifest(dir, "\"log_gap\": true, \"log_capped\": false");
    const RecordedSession session{dir};
    const Replayed replayed = ReplayConsumer(session, out + "/replayed.tlog");
    ASSERT_TRUE(replayed.report.stopped_at_gap.has_value());
    EXPECT_EQ(replayed.report.stopped_at_gap->log, dir + "/observer.tlog");
    EXPECT_EQ(replayed.report.stopped_at_gap->at.nanos(), gap.start_ns);
    EXPECT_FALSE(replayed.report.stopped_at_gap->damage_offset.has_value());
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("a capped log that feeds it nothing");
    const std::string dir = TempDir("capped_unfed");
    const std::string out = TempDir("capped_unfed_out");
    RecordSession(dir, out + "/live.tlog");
    CreateEmptyFile(dir + "/producer.tlog.capped");
    const RecordedSession session{dir};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/setpoints.tlog", "setpoints"},
              Loop::Options{.start = session.start()}};
    SetpointWatcher watcher{loop};
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    ASSERT_EQ(report.fed.size(), 1u);
    EXPECT_EQ(report.fed[0].messages, 6u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// A byte cap that rotated a log ten times or more leaves segment files
// whose names sort as text out of segment order (segment10 before segment2).
// The replayed program's run start still comes from its oldest segment, so a
// late-starting run's timers fire at the deadlines they fired at live, and it
// is fed every receipt of every segment at the drain that delivered it.
// REQ-OPS-14
TEST(InputReplay, ARunWhoseLogRotatedPastTenSegmentsKeepsItsTimeline) {
  const std::string dir = TempDir("rotated_eleven");
  const std::string out = TempDir("rotated_eleven_out");
  RecordLateConsumer(dir);
  const std::string path = dir + "/consumer.tlog";
  std::size_t receipts = 0;
  {
    log::LogReader reader{path};
    log::LogReader::Record record;
    while (reader.next(record)) {
      if (record.header.kind ==
          static_cast<std::uint16_t>(EventKind::MESSAGE)) {
        ++receipts;
      }
    }
  }
  const std::vector<Sent> live = Sends(path);
  std::vector<MonotonicTime> cuts;
  cuts.reserve(11);
  for (int k = 0; k < 11; ++k) {
    cuts.push_back(kOrigin + 10ms + std::chrono::milliseconds{4 * k});
  }
  RotateAt(path, cuts);
  ASSERT_TRUE(std::filesystem::exists(log::LogSegmentPath(path, 10)));

  const RecordedSession session{dir, "consumer"};
  EXPECT_EQ((session.run_start() - kOrigin).count(),
            std::chrono::nanoseconds{kLateRun}.count());
  SimulationEnvironment environment;
  Loop loop{environment, log::LogWriter{out + "/consumer.tlog", "consumer"},
            Loop::Options{.start = session.loop_start(),
                          .run_start = session.run_start()}};
  TestRobot<Loop> consumer{loop, kTopics, kGain};
  consumer.control_timer().setup_periodic(kOrigin + 1ms, 1ms);
  const InputReplayReport report = ReplayInputs(loop, environment, session);
  EXPECT_FALSE(report.stopped_at_gap.has_value());
  std::size_t fed = 0;
  for (const InputReplayReport::Fed& topic : report.fed) fed += topic.messages;
  EXPECT_EQ(fed, receipts);
  EXPECT_EQ(Sends(out + "/consumer.tlog"), live);
  std::filesystem::remove_all(dir);
  std::filesystem::remove_all(out);
}

// The deadline of every timer firing the log at `path` recorded.
std::vector<std::int64_t> TimerFirings(const std::string& path) {
  log::LogReader reader{path};
  std::vector<std::int64_t> firings;
  log::LogReader::Record record;
  while (reader.next(record)) {
    if (record.header.kind == static_cast<std::uint16_t>(EventKind::TIMER)) {
      firings.push_back(record.header.event_time_ns);
    }
  }
  return firings;
}

// Records a Ticker, its run starting kLateRun after its origin, into
// `<dir>/ticker.tlog` up to kSessionMs after the origin.
void RecordLateTicker(const std::string& dir) {
  SimulationEnvironment environment;
  Loop loop{environment, log::LogWriter{dir + "/ticker.tlog", "ticker"},
            Loop::Options{.start = kOrigin, .run_start = kOrigin + kLateRun}};
  const Ticker ticker{loop};
  loop.run_until(kOrigin + std::chrono::milliseconds{kSessionMs});
  loop.finish();
}

// A replayed program's run start is its first timer firing less that
// timer's offset, read from its first run's log. A byte cap that deleted
// that log's first segment (its `<log>.capped` marker says so) took that
// firing with it: the oldest kept segment's first firing is whole periods
// later. A program that watches no topic its log recorded, so that no fed
// topic refuses the log, would replay every timer that many periods late;
// input replay refuses, before anything runs, to read the run start from
// such a log, naming the file it would read it from and the sign. A capped
// log that kept its first segment (capture stopped at the cap) still gives
// the run start: its timers fire at their live deadlines.
// REQ-OPS-14
TEST(InputReplay, RefusesARunStartFromALogWhoseFirstSegmentIsGone) {
  {
    SCOPED_TRACE("its first segment deleted by the byte cap");
    const std::string dir = TempDir("head_gone");
    const std::string out = TempDir("head_gone_out");
    RecordLateTicker(dir);
    const std::string path = dir + "/ticker.tlog";
    const std::vector<std::int64_t> live = TimerFirings(path);
    ASSERT_FALSE(live.empty());
    RotateAt(path, {kOrigin + 20ms, kOrigin + 40ms});
    std::filesystem::remove(log::LogSegmentPath(path, 0));
    CreateEmptyFile(log::CappedMarkerPath(path));
    const RecordedSession session{dir, "ticker"};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/ticker.tlog", "ticker"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    const Ticker ticker{loop};
    try {
      ReplayInputs(loop, environment, session);
      const std::vector<std::int64_t> replayed =
          TimerFirings(out + "/ticker.tlog");
      ADD_FAILURE() << "replayed from " << log::LogSegmentPath(path, 1)
                    << ": first firing "
                    << (replayed.empty() ? -1
                                         : replayed.front() - kOrigin.nanos())
                    << " ns after the origin, live "
                    << live.front() - kOrigin.nanos() << " ns";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(log::LogSegmentPath(path, 1)), std::string::npos)
          << what;
      EXPECT_NE(what.find(log::CappedMarkerPath(path)), std::string::npos)
          << what;
      EXPECT_EQ(loop.dispatch_count(), 0u);
    }
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("capture stopped at the cap, its first segment kept");
    const std::string dir = TempDir("head_kept");
    const std::string out = TempDir("head_kept_out");
    RecordLateTicker(dir);
    const std::string path = dir + "/ticker.tlog";
    const std::size_t live = TimerFirings(path).size();
    std::filesystem::resize_file(path, RecordOffsets(path)[20]);
    CreateEmptyFile(log::CappedMarkerPath(path));
    const std::vector<std::int64_t> kept = TimerFirings(path);
    ASSERT_LT(kept.size(), live);
    const RecordedSession session{dir, "ticker"};
    EXPECT_EQ(session.run_start(), kOrigin + kLateRun);
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/ticker.tlog", "ticker"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    const Ticker ticker{loop};
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    EXPECT_FALSE(report.stopped_at_gap.has_value());
    EXPECT_EQ(TimerFirings(out + "/ticker.tlog"), kept);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// RecordedSession takes one log as well as a session directory. A log the
// byte cap rotated is all its kept files (log::LogSegmentFiles), whichever
// of them is named, as a directory reads them: its run start comes from
// its first segment's first firing, not the newest file's, so its timers
// replay at their live deadlines.
// REQ-OPS-14
TEST(InputReplay, OneFileOfARotatedLogReadsTheWholeLog) {
  const std::string dir = TempDir("one_file");
  const std::string out = TempDir("one_file_out");
  RecordLateTicker(dir);
  const std::string path = dir + "/ticker.tlog";
  const std::vector<std::int64_t> live = TimerFirings(path);
  RotateAt(path, {kOrigin + 20ms, kOrigin + 40ms});
  for (const std::string& named : {path, log::LogSegmentPath(path, 1)}) {
    SCOPED_TRACE(named);
    const RecordedSession session{named, "ticker"};
    EXPECT_EQ(session.log_paths(), log::LogSegmentFiles(path));
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/ticker.tlog", "ticker"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    const Ticker ticker{loop};
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    EXPECT_FALSE(report.stopped_at_gap.has_value());
    EXPECT_EQ(TimerFirings(out + "/ticker.tlog"), live);
  }
  EXPECT_EQ(
      RecordedSession(dir + "//ticker.segment1.tlog", "ticker").log_paths(),
      log::LogSegmentFiles(path));
  std::filesystem::remove_all(dir);
  std::filesystem::remove_all(out);
}

// The deadline of every timer firing in every kept file of the log at
// `path`, oldest segment first.
std::vector<std::int64_t> LogTimerFirings(const std::string& path) {
  std::vector<std::int64_t> firings;
  for (const std::string& file : log::LogSegmentFiles(path)) {
    const std::vector<std::int64_t> more = TimerFirings(file);
    firings.insert(firings.end(), more.begin(), more.end());
  }
  return firings;
}

// Reads the sensor as each reading is published (a prompt watcher, run
// with Delivery::ON_PUBLISH as a node that watches a topic is), and ticks
// every millisecond from 20 ms into its run; given `rearm`, its first
// reading re-arms the tick to `rearm` after it.
class SlowTicker {
 public:
  explicit SlowTicker(Loop& loop, std::optional<Duration> rearm = {})
      : tick_{make_timer<&SlowTicker::on_tick>(loop, "tick", this)},
        rearm_{rearm} {
    loop.set_prompt(watch<EventsTest::SensorMessage, &SlowTicker::on_sensor>(
        loop, kTopics.sensor, this));
    tick_.setup_periodic(loop.monotonic_now() + 20ms, 1ms);
  }

 private:
  void on_tick(const Context&) {}
  void on_sensor(const Context& context, const EventsTest::SensorMessage&) {
    if (rearm_) tick_.setup_periodic(context.now + *rearm_, 1ms);
    rearm_.reset();
  }

  Timer<Loop> tick_;
  std::optional<Duration> rearm_;
};

// Records the producer to kSessionMs and a SlowTicker (`slow`), its run
// starting `late` after the origin, to `exit`, into `dir`.
void RecordSlowTicker(const std::string& dir, MonotonicTime exit,
                      std::optional<Duration> rearm = {},
                      Duration late = kLateRun) {
  SimulationEnvironment environment;
  Loop producer_loop{environment,
                     log::LogWriter{dir + "/producer.tlog", "producer"},
                     Loop::Options{.start = kOrigin}};
  Loop slow_loop{environment, log::LogWriter{dir + "/slow.tlog", "slow"},
                 Loop::Options{.delivery = Delivery::ON_PUBLISH,
                               .start = kOrigin,
                               .run_start = kOrigin + late}};
  const Producer producer{producer_loop};
  SlowTicker slow{slow_loop, rearm};
  for (int ms = 1; ms <= kSessionMs; ++ms) {
    const MonotonicTime now = kOrigin + std::chrono::milliseconds{ms};
    producer_loop.run_until(now);
    if (now <= exit) slow_loop.run_until(now);
  }
  producer_loop.finish();
  slow_loop.finish();
}

// Replays `slow` from the session in `dir` into `out`/slow.tlog: what
// ReplayInputs throws, or none.
std::optional<std::string> ReplaySlowTicker(
    const std::string& dir, const std::string& out,
    std::optional<Duration> rearm = {}) {
  const RecordedSession session{dir, "slow"};
  SimulationEnvironment environment;
  Loop loop{environment, log::LogWriter{out + "/slow.tlog", "slow"},
            Loop::Options{.delivery = Delivery::ON_PUBLISH,
                          .start = session.loop_start(),
                          .run_start = session.run_start()}};
  SlowTicker slow{loop, rearm};
  try {
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    EXPECT_FALSE(report.stopped_at_gap.has_value());
  } catch (const ReplayError& e) {
    EXPECT_EQ(loop.dispatch_count(), 0u);
    return e.what();
  }
  return std::nullopt;
}

// Each receipt every kept file of the log at `path` holds: when its drain
// ran (now_ns), its event time, publication time and sequence; oldest
// segment first.
using Receipt =
    std::tuple<std::int64_t, std::int64_t, std::int64_t, std::uint64_t>;
std::vector<Receipt> Receipts(const std::string& path) {
  std::vector<Receipt> receipts;
  for (const std::string& file : log::LogSegmentFiles(path)) {
    log::LogReader reader{file};
    log::LogReader::Record record;
    while (reader.next(record)) {
      if (record.header.kind ==
          static_cast<std::uint16_t>(EventKind::MESSAGE)) {
        receipts.emplace_back(record.header.now_ns, record.header.event_time_ns,
                              record.header.publish_time_ns,
                              record.header.sequence);
      }
    }
  }
  return receipts;
}

// Zero-fills the file header of the log file at `path`, as a crash while
// it was being created leaves one: a reader skips the file.
void ZeroFileHeader(const std::string& path) {
  std::fstream file{path, std::ios::binary | std::ios::in | std::ios::out};
  const std::array<char, sizeof(log::FileHeader::magic)> zeros{};
  file.write(zeros.data(), zeros.size());
}

// A replayed program's run start is read from its first timer firing, in
// whichever segment of its first run's log holds it: a byte cap that
// rotated the log before that firing (the program drained inputs first)
// leaves its first segment with none, and the run start is not the
// origin, but the first firing's deadline less its offset, so its timers
// replay at their live deadlines. A timer the run re-armed before its
// first firing gives no run start, in a later segment too (each repeats
// the log's manifest). Only a log read from its first segment on, each
// segment continuing the one before it, proves which firing is first: one
// whose first segment, or a segment before the first firing read, could
// not be read is refused, naming the file. A run that fired no timer that
// gives its run start (it exited before its first deadline, or re-armed
// its timer first) starts at its first dispatch, which live made inside its
// run: it is fed what it received, at the same drains, and fires none up to
// that run's last record, where the origin would fire one its construction
// time early (a program that ran once ends there, unless its loop was late
// when it stopped; a restarted program replays as one run). Its log is
// refused when it holds no record of its run, or when a loss in it may
// hold the firings.
// REQ-OPS-14
TEST(InputReplay, ARunStartIsReadFromTheFirstSegmentWithAFiring) {
  {
    SCOPED_TRACE("its first firing in its second segment");
    const std::string dir = TempDir("late_firing");
    const std::string out = TempDir("late_firing_out");
    RecordSlowTicker(dir, kOrigin + std::chrono::milliseconds{kSessionMs});
    const std::string path = dir + "/slow.tlog";
    const std::vector<std::int64_t> live = TimerFirings(path);
    ASSERT_FALSE(live.empty());
    RotateAt(path, {kOrigin + 15ms, kOrigin + 40ms});
    ASSERT_TRUE(TimerFirings(log::LogSegmentPath(path, 0)).empty());
    ASSERT_EQ(LogTimerFirings(path), live);
    EXPECT_EQ(ReplaySlowTicker(dir, out), std::nullopt);
    EXPECT_EQ(TimerFirings(out + "/slow.tlog"), live);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("re-armed in its first segment, fired in its second");
    const std::string dir = TempDir("rearmed_firing");
    const std::string out = TempDir("rearmed_firing_out");
    RecordSlowTicker(dir, kOrigin + std::chrono::milliseconds{kSessionMs},
                     30ms);
    const std::string path = dir + "/slow.tlog";
    const std::vector<std::int64_t> live = TimerFirings(path);
    ASSERT_FALSE(live.empty());
    RotateAt(path, {kOrigin + 15ms});
    ASSERT_TRUE(TimerFirings(log::LogSegmentPath(path, 0)).empty());
    EXPECT_EQ(ReplaySlowTicker(dir, out, 30ms), std::nullopt);
    EXPECT_EQ(TimerFirings(out + "/slow.tlog"), live);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("its first segment's header incomplete");
    const std::string dir = TempDir("head_unread");
    const std::string out = TempDir("head_unread_out");
    RecordSlowTicker(dir, kOrigin + std::chrono::milliseconds{kSessionMs});
    const std::string path = dir + "/slow.tlog";
    RotateAt(path, {kOrigin + 30ms, kOrigin + 45ms});
    ZeroFileHeader(log::LogSegmentPath(path, 0));
    const std::optional<std::string> refused = ReplaySlowTicker(dir, out);
    EXPECT_NE(refused.value_or("").find(log::LogSegmentPath(path, 0)),
              std::string::npos)
        << refused.value_or("replayed, not refused");
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("the segment holding its first firing unread");
    const std::string dir = TempDir("first_firing_unread");
    const std::string out = TempDir("first_firing_unread_out");
    RecordSlowTicker(dir, kOrigin + std::chrono::milliseconds{kSessionMs});
    const std::string path = dir + "/slow.tlog";
    RotateAt(path, {kOrigin + 15ms, kOrigin + 30ms, kOrigin + 45ms});
    ASSERT_TRUE(TimerFirings(log::LogSegmentPath(path, 0)).empty());
    ASSERT_FALSE(TimerFirings(log::LogSegmentPath(path, 1)).empty());
    ZeroFileHeader(log::LogSegmentPath(path, 1));
    const std::optional<std::string> refused = ReplaySlowTicker(dir, out);
    EXPECT_NE(refused.value_or("").find(log::LogSegmentPath(path, 2)),
              std::string::npos)
        << refused.value_or("replayed, not refused");
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("exited before its first deadline");
    const std::string dir = TempDir("no_firing");
    const std::string out = TempDir("no_firing_out");
    RecordSlowTicker(dir, kOrigin + 21ms);
    const std::string path = dir + "/slow.tlog";
    ASSERT_TRUE(TimerFirings(path).empty());
    RotateAt(path, {kOrigin + 10ms});
    EXPECT_EQ(ReplaySlowTicker(dir, out), std::nullopt);
    EXPECT_EQ(TimerFirings(out + "/slow.tlog"), std::vector<std::int64_t>{});
    EXPECT_EQ(Receipts(out + "/slow.tlog"), Receipts(path));
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("no record of its run");
    const std::string dir = TempDir("no_record");
    const std::string out = TempDir("no_record_out");
    RecordSlowTicker(dir, kOrigin + std::chrono::milliseconds{kSessionMs});
    const std::string path = dir + "/slow.tlog";
    std::filesystem::resize_file(path, RecordOffsets(path).front());
    const std::optional<std::string> refused = ReplaySlowTicker(dir, out);
    EXPECT_NE(refused.value_or("").find(path), std::string::npos)
        << refused.value_or("replayed, not refused");
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("every firing in a gap");
    const std::string dir = TempDir("firings_lost");
    const std::string out = TempDir("firings_lost_out");
    RecordLateTicker(dir);
    const std::string path = dir + "/ticker.tlog";
    ASSERT_FALSE(TimerFirings(path).empty());
    log::CopyWithGap(path, path + ".gap", 0, RecordOffsets(path).size());
    std::filesystem::rename(path + ".gap", path);
    ASSERT_TRUE(TimerFirings(path).empty());
    const RecordedSession session{dir, "ticker"};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/ticker.tlog", "ticker"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    const Ticker ticker{loop};
    try {
      ReplayInputs(loop, environment, session);
      ADD_FAILURE() << "replayed " << TimerFirings(out + "/ticker.tlog").size()
                    << " firings from a log whose gap holds every one";
    } catch (const ReplayError& e) {
      EXPECT_NE(std::string{e.what()}.find(path), std::string::npos)
          << e.what();
      EXPECT_EQ(loop.dispatch_count(), 0u);
    }
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// Closes this process's descriptor of the file at `path`, as a disk that
// refuses every later write leaves its writer.
bool CloseLogDescriptor(const std::string& path) {
  struct stat target {};
  if (::stat(path.c_str(), &target) != 0) return false;
  const auto limit = ::sysconf(_SC_OPEN_MAX);
  for (int fd = 0; fd < limit; ++fd) {
    struct stat candidate {};
    if (::fstat(fd, &candidate) == 0 && candidate.st_dev == target.st_dev &&
        candidate.st_ino == target.st_ino) {
      return ::close(fd) == 0;
    }
  }
  return false;
}

// A recorder whose disk fails ends capture for good with no GAP record, so
// its log holds neither the records after the failure nor its EXIT, and the
// launcher marks its member recorder_failed without log_gap. Input replay
// refuses, before anything runs, to feed a watched topic from such a log,
// naming it and the sign, as it does a log_gap member's run cut short.
// REQ-OPS-14
TEST(InputReplay, RefusesALogWhoseRecorderFailedBeforeItsExit) {
  const std::string dir = TempDir("recorder_failed");
  const std::string out = TempDir("recorder_failed_out");
  const std::string path = dir + "/reader.tlog";
  {
    log::LogWriter writer{path, "reader",
                          log::LogWriterOptions{.background = false}};
    writer.start(ManifestOf<SensorWatcher>(dir + "/unused.tlog"), kOrigin);
    for (int i = 1; i <= 4; ++i) {
      if (i == 3) {
        writer.flush();
        ASSERT_TRUE(CloseLogDescriptor(path));
      }
      const EventsTest::SensorMessage reading{i, 1.0F, 1.0F};
      writer.dispatch(
          Context{.kind = EventKind::MESSAGE,
                  .source_id = 0,
                  .dispatch_index = static_cast<std::uint64_t>(i),
                  .event_time = kOrigin + std::chrono::milliseconds{i},
                  .now = kOrigin + std::chrono::milliseconds{i},
                  .publish_time = kOrigin + std::chrono::milliseconds{i},
                  .sequence = static_cast<std::uint64_t>(i)},
          std::as_bytes(std::span{&reading, 1}));
    }
    writer.finish(Context{.kind = EventKind::EXIT,
                          .dispatch_index = 5,
                          .event_time = kOrigin + 5ms,
                          .now = kOrigin + 5ms});
    writer.flush();
    ASSERT_TRUE(writer.failed());
    EXPECT_EQ(writer.gaps(), 0u);
  }
  // What the failed recorder left: the two readings before the failure.
  std::size_t receipts = 0;
  {
    log::LogReader reader{path};
    log::LogReader::Record record;
    while (reader.next(record)) {
      const auto kind = static_cast<EventKind>(record.header.kind);
      EXPECT_FALSE(log::GapOf(record.header, record.payload).has_value());
      EXPECT_NE(kind, EventKind::EXIT);
      if (kind == EventKind::MESSAGE) ++receipts;
    }
  }
  EXPECT_EQ(receipts, 2u);
  std::ofstream{dir + "/manifest.json"}
      << "{\"session_id\": " << log::LogReader{path}.session_id()
      << ", \"nodes\": [{\"name\": \"reader\", \"log_paths\": "
         "[\"/robot/logs/7/reader.tlog\"], \"recorder_failed\": true, "
         "\"log_gap\": false, \"log_capped\": false}]}\n";

  const RecordedSession session{dir};
  SimulationEnvironment environment;
  Loop loop{environment, log::LogWriter{out + "/watcher.tlog", "watcher"},
            Loop::Options{.start = session.start()}};
  SensorWatcher watcher{loop};
  try {
    ReplayInputs(loop, environment, session);
    ADD_FAILURE() << "fed from a log whose recorder failed before its EXIT";
  } catch (const ReplayError& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find(path), std::string::npos) << what;
    EXPECT_NE(what.find("recorder_failed"), std::string::npos) << what;
  }
  EXPECT_EQ(watcher.readings, 0);
  std::filesystem::remove_all(dir);
  std::filesystem::remove_all(out);
}

// A timer armed at offset 0 (immediately due at run_start).
class ImmediateTicker {
 public:
  explicit ImmediateTicker(Loop& loop, bool watch_sensor = false)
      : tick_{make_timer<&ImmediateTicker::on_tick>(loop, "tick", this)} {
    if (watch_sensor) {
      loop.set_prompt(
          watch<EventsTest::SensorMessage, &ImmediateTicker::on_sensor>(
              loop, kTopics.sensor, this));
    }
    tick_.setup_periodic(loop.monotonic_now(), 1ms);
  }

  int ticks{0};
  int readings{0};

 private:
  void on_tick(const Context&) { ++ticks; }
  void on_sensor(const Context&, const EventsTest::SensorMessage&) {
    ++readings;
  }

  Timer<Loop> tick_;
};

// A live loop that was late when it stopped can record its EXIT after a due
// timer deadline it never fired, and an offset-0 timer whose run's only
// dispatch record is its EXIT sits right at run_start == loop_end: replay must
// not fabricate a timer firing in either case.
// REQ-OPS-14
TEST(InputReplay, LateExitAndOffsetZeroExitOnlyDoNotFabricateTimerFirings) {
  {
    SCOPED_TRACE("late EXIT after a timer that fired");
    const std::string dir = TempDir("late_exit_fired");
    const std::string out = TempDir("late_exit_fired_out");
    const std::string path = dir + "/ticker.tlog";
    {
      log::LogWriter writer{path, "ticker",
                            log::LogWriterOptions{.background = false}};
      writer.start(ManifestOf<Ticker>(dir + "/unused.tlog"), kOrigin);
      for (int i = 1; i <= 10; ++i) {
        const MonotonicTime deadline =
            kOrigin + kLateRun + std::chrono::milliseconds{i};
        writer.dispatch(Context{.kind = EventKind::TIMER,
                                .source_id = 0,
                                .dispatch_index = static_cast<std::uint64_t>(i),
                                .event_time = deadline,
                                .now = deadline,
                                .sequence = 1},
                        {});
      }
      // Next deadline is kLateRun + 11ms, never fired; EXIT recorded late at
      // kLateRun + 11.5ms.
      const MonotonicTime late_exit = kOrigin + kLateRun + 11ms + 500us;
      writer.finish(Context{.kind = EventKind::EXIT,
                            .dispatch_index = 11,
                            .event_time = late_exit,
                            .now = late_exit});
      writer.flush();
    }
    const std::vector<std::int64_t> live = TimerFirings(path);
    ASSERT_EQ(live.size(), 10u);
    const RecordedSession session{dir, "ticker"};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/ticker.tlog", "ticker"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    const Ticker ticker{loop};
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    EXPECT_FALSE(report.stopped_at_gap.has_value());
    EXPECT_EQ(TimerFirings(out + "/ticker.tlog"), live);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("late EXIT after no timer fired, with a message dispatch");
    const std::string dir = TempDir("late_exit_unfired");
    const std::string out = TempDir("late_exit_unfired_out");
    const std::string path = dir + "/slow.tlog";
    {
      log::LogWriter writer{path, "slow",
                            log::LogWriterOptions{.background = false}};
      writer.start(ManifestOf<SlowTicker>(dir + "/unused.tlog"), kOrigin);
      const EventsTest::SensorMessage reading{1, 1.0F, 1.0F};
      writer.dispatch(Context{.kind = EventKind::MESSAGE,
                              .source_id = 1,
                              .dispatch_index = 1,
                              .event_time = kOrigin + 10ms,
                              .now = kOrigin + 10ms,
                              .publish_time = kOrigin + 10ms,
                              .sequence = 0},
                      std::as_bytes(std::span{&reading, 1}));
      // Timer offset is 20ms: replay run_start is 10ms (first dispatch), so
      // replay deadline is 30ms; EXIT recorded late at 31ms without firing it.
      writer.finish(Context{.kind = EventKind::EXIT,
                            .dispatch_index = 2,
                            .event_time = kOrigin + 31ms,
                            .now = kOrigin + 31ms});
      writer.flush();
    }
    EXPECT_EQ(ReplaySlowTicker(dir, out), std::nullopt);
    EXPECT_EQ(TimerFirings(out + "/slow.tlog"), std::vector<std::int64_t>{});
    EXPECT_EQ(Receipts(out + "/slow.tlog"), Receipts(path));
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("offset-0 timer whose only dispatch record is its EXIT");
    const std::string dir = TempDir("offset_zero_exit_only");
    const std::string out = TempDir("offset_zero_exit_only_out");
    // Also put a producer log with messages before EXIT to ensure pre-run
    // messages on a newly-watched prompt topic cannot trigger the offset-0
    // timer either.
    WriterLog(dir);
    const std::string path = dir + "/immediate.tlog";
    {
      log::LogWriter writer{path, "immediate",
                            log::LogWriterOptions{.background = false}};
      writer.start(ManifestOf<ImmediateTicker>(dir + "/unused.tlog"), kOrigin);
      const MonotonicTime exit_at = kOrigin + kLateRun;
      writer.finish(Context{.kind = EventKind::EXIT,
                            .dispatch_index = 1,
                            .event_time = exit_at,
                            .now = exit_at});
      writer.flush();
    }
    for (const bool watch_sensor : {false, true}) {
      SCOPED_TRACE(watch_sensor ? "watching another log's topic"
                                : "no watched topics");
      const RecordedSession session{dir, "immediate"};
      SimulationEnvironment environment;
      Loop loop{environment,
                log::LogWriter{out + "/immediate.tlog", "immediate"},
                Loop::Options{.delivery = Delivery::ON_PUBLISH,
                              .start = session.loop_start(),
                              .run_start = session.run_start()}};
      const ImmediateTicker ticker{loop, watch_sensor};
      const InputReplayReport report = ReplayInputs(loop, environment, session);
      EXPECT_FALSE(report.stopped_at_gap.has_value());
      EXPECT_EQ(ticker.ticks, 0);
      EXPECT_EQ(ticker.readings, 0);
      // Only the terminal EXIT record advances dispatch_count.
      EXPECT_EQ(loop.dispatch_count(), 1u);
      EXPECT_EQ(TimerFirings(out + "/immediate.tlog"),
                std::vector<std::int64_t>{});
    }
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// A segment skipped as an incomplete header in the middle of a log lost its
// records: input replay refuses any topic fed from that log (naming the log
// and the unread segment), rather than silently dropping those records with
// only a warning. A log that feeds the program nothing does not stop it.
// REQ-OPS-14
TEST(InputReplay, RefusesTopicFeedsAcrossAnIncompleteHeaderMiddleSegment) {
  {
    SCOPED_TRACE("incomplete middle segment in a log feeding a watched topic");
    const std::string dir = TempDir("mid_incomplete");
    const std::string out = TempDir("mid_incomplete_out");
    RecordSession(dir, out + "/live.tlog");
    const std::string path = dir + "/observer.tlog";
    RotateAt(path, {kOrigin + 20ms, kOrigin + 40ms});
    const std::string mid = log::LogSegmentPath(path, 1);
    ZeroFileHeader(mid);

    const RecordedSession session{dir};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/consumer.tlog", "consumer"},
              Loop::Options{.start = session.start()}};
    TestRobot<Loop> consumer{loop, kTopics, kGain};
    consumer.control_timer().setup_periodic(session.start() + 1ms, 1ms);
    try {
      ReplayInputs(loop, environment, session);
      ADD_FAILURE() << "fed from " << path << " across incomplete " << mid;
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(path), std::string::npos) << what;
      EXPECT_NE(what.find(mid), std::string::npos) << what;
    }
    EXPECT_EQ(loop.dispatch_count(), 0u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("incomplete middle segment in a log that feeds nothing");
    const std::string dir = TempDir("mid_incomplete_unfed");
    const std::string out = TempDir("mid_incomplete_unfed_out");
    RecordSession(dir, out + "/live.tlog");
    const std::string path = dir + "/producer.tlog";
    RotateAt(path, {kOrigin + 20ms, kOrigin + 40ms});
    ZeroFileHeader(log::LogSegmentPath(path, 1));

    const RecordedSession session{dir};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/setpoints.tlog", "setpoints"},
              Loop::Options{.start = session.start()}};
    SetpointWatcher watcher{loop};
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    EXPECT_FALSE(report.stopped_at_gap.has_value());
    ASSERT_EQ(report.fed.size(), 1u);
    EXPECT_EQ(report.fed[0].messages, 6u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// A log whose first segment is gone with no .capped marker and no manifest
// mark (e.g. a copied session missing segment0) is refused for any topic fed
// from it, naming the log, whether multiple segments remain or only the
// active unnumbered file remains (whose first dispatch index is > 1).
// REQ-OPS-14
TEST(InputReplay, RefusesTopicFeedsFromALogMissingItsFirstSegmentWithoutMarks) {
  {
    SCOPED_TRACE("segment0 deleted, segment1 and active file remain");
    const std::string dir = TempDir("head_gone_unmarked");
    const std::string out = TempDir("head_gone_unmarked_out");
    RecordSession(dir, out + "/live.tlog");
    const std::string path = dir + "/observer.tlog";
    RotateAt(path, {kOrigin + 20ms, kOrigin + 40ms});
    const std::string head = log::LogSegmentPath(path, 0);
    std::filesystem::remove(head);

    const RecordedSession session{dir};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/consumer.tlog", "consumer"},
              Loop::Options{.start = session.start()}};
    TestRobot<Loop> consumer{loop, kTopics, kGain};
    consumer.control_timer().setup_periodic(session.start() + 1ms, 1ms);
    try {
      ReplayInputs(loop, environment, session);
      ADD_FAILURE() << "fed from " << path << " with " << head << " missing";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(log::LogSegmentPath(path, 1)), std::string::npos)
          << what;
      EXPECT_NE(what.find(head), std::string::npos) << what;
    }
    EXPECT_EQ(loop.dispatch_count(), 0u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("segment0 deleted, only active unnumbered file remains");
    const std::string dir = TempDir("head_gone_only_active");
    const std::string out = TempDir("head_gone_only_active_out");
    RecordSession(dir, out + "/live.tlog");
    const std::string path = dir + "/observer.tlog";
    RotateAt(path, {kOrigin + 20ms});
    std::filesystem::remove(log::LogSegmentPath(path, 0));

    const RecordedSession session{dir};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/consumer.tlog", "consumer"},
              Loop::Options{.start = session.start()}};
    TestRobot<Loop> consumer{loop, kTopics, kGain};
    consumer.control_timer().setup_periodic(session.start() + 1ms, 1ms);
    try {
      ReplayInputs(loop, environment, session);
      ADD_FAILURE() << "fed from " << path << " starting past dispatch 1";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(path), std::string::npos) << what;
    }
    EXPECT_EQ(loop.dispatch_count(), 0u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("segment0 deleted, only active file remains, run_start check");
    const std::string dir = TempDir("head_gone_only_active_run_start");
    const std::string out = TempDir("head_gone_only_active_run_start_out");
    RecordLateTicker(dir);
    const std::string path = dir + "/ticker.tlog";
    RotateAt(path, {kOrigin + 20ms});
    std::filesystem::remove(log::LogSegmentPath(path, 0));

    const RecordedSession session{dir, "ticker"};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/ticker.tlog", "ticker"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    const Ticker ticker{loop};
    try {
      ReplayInputs(loop, environment, session);
      ADD_FAILURE() << "took run_start from " << path
                    << " after segment0 was deleted";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(path), std::string::npos) << what;
    }
    EXPECT_EQ(loop.dispatch_count(), 0u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// A segment deleted right before the log's unnumbered active file (segment0
// present, segment1 deleted, active .tlog present) cannot be seen from file
// names alone, but leaves a dispatch_index jump (event-loop logs) or an
// accepted-SEND transport sequence jump (bridge-style dispatch_index == 0
// logs): input replay detects the discontinuity and refuses both topic feeds
// and run start, naming the log.
// REQ-OPS-14
TEST(InputReplay, RefusesALogMissingTheSegmentBeforeItsUnnumberedFile) {
  {
    SCOPED_TRACE("event-loop log feeding a watched topic");
    const std::string dir = TempDir("missing_before_active_feed");
    const std::string out = TempDir("missing_before_active_feed_out");
    RecordSession(dir, out + "/live.tlog");
    const std::string path = dir + "/observer.tlog";
    RotateAt(path, {kOrigin + 20ms, kOrigin + 40ms});
    std::filesystem::remove(log::LogSegmentPath(path, 1));

    const RecordedSession session{dir};
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/consumer.tlog", "consumer"},
              Loop::Options{.start = session.start()}};
    TestRobot<Loop> consumer{loop, kTopics, kGain};
    consumer.control_timer().setup_periodic(session.start() + 1ms, 1ms);
    try {
      ReplayInputs(loop, environment, session);
      ADD_FAILURE() << "fed from " << path << " with segment1 deleted";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(path), std::string::npos) << what;
    }
    EXPECT_EQ(loop.dispatch_count(), 0u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("event-loop log run_start across missing segment1");
    const std::string dir = TempDir("missing_before_active_run_start");
    const std::string out = TempDir("missing_before_active_run_start_out");
    RecordSlowTicker(dir, kOrigin + std::chrono::milliseconds{kSessionMs});
    const std::string path = dir + "/slow.tlog";
    RotateAt(path, {kOrigin + 15ms, kOrigin + 30ms});
    ASSERT_TRUE(TimerFirings(log::LogSegmentPath(path, 0)).empty());
    ASSERT_FALSE(TimerFirings(log::LogSegmentPath(path, 1)).empty());
    std::filesystem::remove(log::LogSegmentPath(path, 1));

    const std::optional<std::string> refused = ReplaySlowTicker(dir, out);
    EXPECT_NE(refused.value_or("").find(path), std::string::npos)
        << refused.value_or("replayed, not refused");
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("dispatch_index == 0 bridge-style log (sequence jump)");
    const std::string dir = TempDir("missing_before_active_bridge");
    const std::string out = TempDir("missing_before_active_bridge_out");
    const std::string path = dir + "/bridge.tlog";
    {
      log::LogWriter writer{path, "hardware_node",
                            log::LogWriterOptions{.background = false}};
      writer.start(ManifestOf<SensorWriter>(dir + "/unused.tlog"), kOrigin);
      for (int i = 1; i <= 12; ++i) {
        const MonotonicTime at = kOrigin + std::chrono::milliseconds{i};
        const EventsTest::SensorMessage reading{i, 1.0F, 1.0F};
        writer.send(Context{.event_time = at, .now = at}, 0,
                    std::as_bytes(std::span{&reading, 1}), 0,
                    static_cast<std::uint64_t>(i - 1));
      }
      writer.finish(Context{.kind = EventKind::EXIT,
                            .event_time = kOrigin + 13ms,
                            .now = kOrigin + 13ms});
      writer.flush();
    }
    RotateAt(path, {kOrigin + 5ms, kOrigin + 9ms});
    std::filesystem::remove(log::LogSegmentPath(path, 1));

    const RecordedSession session{dir};
    try {
      ReplayWatcher(session, out);
      ADD_FAILURE() << "fed from " << path << " with segment1 deleted";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(path), std::string::npos) << what;
    }
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// A program with a periodic timer that disarms itself on its first firing,
// and optionally watches /input/sensor during replay.
class DisarmingTicker {
 public:
  explicit DisarmingTicker(Loop& loop, bool watch_sensor = false)
      : tick_{make_timer<&DisarmingTicker::on_tick>(loop, "tick", this)} {
    if (watch_sensor) {
      watch<EventsTest::SensorMessage, &DisarmingTicker::on_sensor>(
          loop, kTopics.sensor, this);
    }
    tick_.setup_periodic(loop.monotonic_now() + 5ms, 5ms);
  }

  int ticks{0};
  int readings{0};

 private:
  void on_tick(const Context&) {
    ++ticks;
    tick_.disable();
  }
  void on_sensor(const Context&, const EventsTest::SensorMessage&) {
    ++readings;
  }

  Timer<Loop> tick_;
};

// A program whose initial one-shot timer arms a second, initially disarmed
// one-shot timer 5 ms later.
class ArmingTicker {
 public:
  explicit ArmingTicker(Loop& loop)
      : first_{make_timer<&ArmingTicker::on_first>(loop, "first", this)},
        second_{make_timer<&ArmingTicker::on_second>(loop, "second", this)} {
    first_.setup(loop.monotonic_now() + 5ms);
  }

  int first_ticks{0};
  int second_ticks{0};

 private:
  void on_first(const Context& context) {
    ++first_ticks;
    second_.setup(context.now + 5ms);
  }
  void on_second(const Context&) { ++second_ticks; }

  Timer<Loop> first_;
  Timer<Loop> second_;
};

// An own-log GAP record before EXIT drops records (which may include TIMER
// firings or DISARM_TIMER operations), invalidating tracked timer state so a
// stale pre-gap deadline never clamps loop_end() and drops firings that ran
// live.
// REQ-OPS-14
TEST(InputReplay, OwnLogGapBeforeExitDoesNotClampAwayRealFirings) {
  const std::string dir = TempDir("own_gap_before_exit");
  const std::string out = TempDir("own_gap_before_exit_out");
  const std::string path = dir + "/ticker.tlog";
  const MonotonicTime exit_at = kOrigin + kLateRun + 10ms;
  {
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{path, "ticker"},
              Loop::Options{.start = kOrigin, .run_start = kOrigin + kLateRun}};
    const Ticker ticker{loop};
    loop.run_until(exit_at);
    loop.finish();
  }
  const std::vector<std::int64_t> live = TimerFirings(path);
  ASSERT_EQ(live.size(), 10u);
  // Replace firings 6..10 with a GAP closed at flush, leaving EXIT held at
  // exit_at after the GAP.
  const std::string gapped = path + ".gap";
  log::CopyWithGap(path, gapped, 5, RecordOffsets(path).size());
  std::filesystem::rename(gapped, path);
  ASSERT_EQ(TimerFirings(path).size(), 5u);

  const RecordedSession session{dir, "ticker"};
  EXPECT_EQ(session.loop_end(), exit_at);
  SimulationEnvironment environment;
  Loop loop{environment, log::LogWriter{out + "/ticker.tlog", "ticker"},
            Loop::Options{.start = session.loop_start(),
                          .run_start = session.run_start()}};
  const Ticker ticker{loop};
  const InputReplayReport report = ReplayInputs(loop, environment, session);
  EXPECT_FALSE(report.stopped_at_gap.has_value());
  EXPECT_EQ(TimerFirings(out + "/ticker.tlog"), live);
  std::filesystem::remove_all(dir);
  std::filesystem::remove_all(out);
}

// Late-EXIT clamping tracks ARM_TIMER and DISARM_TIMER: disarming a periodic
// timer prevents its old next deadline from clamping away later time/inputs,
// and arming a previously disarmed one-shot timer clamps a late EXIT before
// that new deadline so no firing is fabricated. Crafted extreme offsets and
// deadlines saturate without signed-overflow UB.
// REQ-OPS-14
TEST(InputReplay, ArmAndDisarmTimerTrackingBeforeLateExit) {
  {
    SCOPED_TRACE("DISARM_TIMER clears the timer so later inputs are not cut");
    const std::string dir = TempDir("disarm_before_exit");
    const std::string out = TempDir("disarm_before_exit_out");
    const MonotonicTime exit_at = kOrigin + 20ms;
    {
      SimulationEnvironment environment;
      Loop producer_loop{environment,
                         log::LogWriter{dir + "/producer.tlog", "producer"},
                         Loop::Options{.start = kOrigin}};
      Loop ticker_loop{environment,
                       log::LogWriter{dir + "/ticker.tlog", "ticker"},
                       Loop::Options{.start = kOrigin}};
      const Producer producer{producer_loop};
      DisarmingTicker ticker{ticker_loop};
      for (int ms = 1; ms <= 20; ++ms) {
        const MonotonicTime now = kOrigin + std::chrono::milliseconds{ms};
        producer_loop.run_until(now);
        ticker_loop.run_until(now);
      }
      producer_loop.finish();
      ticker_loop.finish();
      ASSERT_EQ(ticker.ticks, 1);
    }
    const RecordedSession session{dir, "ticker"};
    // Without DISARM_TIMER tracking, loop_end would be clamped to 10ms - 1ns.
    EXPECT_EQ(session.loop_end(), exit_at);
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/ticker.tlog", "ticker"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    DisarmingTicker ticker{loop, /*watch_sensor=*/true};
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    EXPECT_FALSE(report.stopped_at_gap.has_value());
    EXPECT_EQ(ticker.ticks, 1);
    // Producer sends every 2 ms from 2 ms to 20 ms -> 10 readings.
    EXPECT_EQ(ticker.readings, 10);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("ARM_TIMER clamps a late EXIT before the armed one-shot");
    const std::string dir = TempDir("arm_before_late_exit");
    const std::string out = TempDir("arm_before_late_exit_out");
    const std::string path = dir + "/armer.tlog";
    {
      log::LogWriter writer{path, "armer",
                            log::LogWriterOptions{.background = false}};
      writer.start(ManifestOf<ArmingTicker>(dir + "/unused.tlog"), kOrigin);
      const MonotonicTime first_at = kOrigin + 5ms;
      const MonotonicTime second_at = kOrigin + 10ms;
      const Context first_tick{.kind = EventKind::TIMER,
                               .source_id = 0,
                               .dispatch_index = 1,
                               .event_time = first_at,
                               .now = first_at,
                               .sequence = 1};
      writer.dispatch(first_tick, {});
      // Handler armed timer 1 as a one-shot for second_at (period = 0).
      writer.dispatch(Context{.kind = EventKind::ARM_TIMER,
                              .source_id = 1,
                              .dispatch_index = 1,
                              .event_time = second_at,
                              .now = first_at,
                              .sequence = 0},
                      {});
      // Late EXIT after second_at without timer 1 ever firing.
      const MonotonicTime late_exit = kOrigin + 12ms;
      writer.finish(Context{.kind = EventKind::EXIT,
                            .dispatch_index = 2,
                            .event_time = late_exit,
                            .now = late_exit});
      writer.flush();
    }
    const RecordedSession session{dir, "armer"};
    // Without ARM_TIMER tracking, loop_end would remain 12ms and timer 1 would
    // fabricate a firing at 10ms.
    EXPECT_EQ(session.loop_end(), kOrigin + 10ms - 1ns);
    SimulationEnvironment environment;
    Loop loop{environment, log::LogWriter{out + "/armer.tlog", "armer"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    ArmingTicker armer{loop};
    const InputReplayReport report = ReplayInputs(loop, environment, session);
    EXPECT_FALSE(report.stopped_at_gap.has_value());
    EXPECT_EQ(armer.first_ticks, 1);
    EXPECT_EQ(armer.second_ticks, 0);
    EXPECT_EQ(TimerFirings(out + "/armer.tlog"),
              (std::vector<std::int64_t>{(kOrigin + 5ms).nanos()}));
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("extreme crafted offset and deadline saturate cleanly");
    const std::string dir = TempDir("extreme_timer_bounds");
    const std::string path = dir + "/crafted.tlog";
    {
      Manifest manifest;
      manifest.push_back(
          Registration{.id = 0,
                       .kind = SourceKind::TIMER,
                       .name = "huge_offset",
                       .period_ns = 0,
                       .offset_ns = std::numeric_limits<std::int64_t>::max(),
                       .armed = true});
      manifest.push_back(Registration{.id = 1,
                                      .kind = SourceKind::TIMER,
                                      .name = "min_deadline",
                                      .period_ns = 0,
                                      .offset_ns = 0,
                                      .armed = false});
      log::LogWriter writer{path, "crafted",
                            log::LogWriterOptions{.background = false}};
      writer.start(manifest, kOrigin);
      writer.dispatch(Context{.kind = EventKind::ARM_TIMER,
                              .source_id = 1,
                              .dispatch_index = 1,
                              .event_time = MonotonicTime::from_nanos(
                                  std::numeric_limits<std::int64_t>::min()),
                              .now = kOrigin + 1ms,
                              .sequence = 0},
                      {});
      writer.finish(Context{.kind = EventKind::EXIT,
                            .dispatch_index = 2,
                            .event_time = kOrigin + 2ms,
                            .now = kOrigin + 2ms});
      writer.flush();
    }
    const RecordedSession session{dir, "crafted"};
    EXPECT_EQ(
        session.loop_end(),
        MonotonicTime::from_nanos(std::numeric_limits<std::int64_t>::min()));
    std::filesystem::remove_all(dir);
  }
}

// With InputReplayOptions::allow_capped, a byte-capped session (`<log>.capped`
// or manifest `log_capped`) replays deterministically from the start of the
// kept window and warns on stderr naming the capped log(s) and pre-window
// loss; default behavior (allow_capped = false) remains refusal.
// REQ-OPS-14
TEST(InputReplay, AllowCappedReplaysFromKeptWindowAndWarns) {
  {
    SCOPED_TRACE("rotated capped feed log (.capped marker)");
    const std::string dir = TempDir("allow_capped_feed");
    const std::string out = TempDir("allow_capped_feed_out");
    RecordSession(dir, out + "/live.tlog");
    const std::string observer = dir + "/observer.tlog";
    RotateAt(observer, {kOrigin + 20ms, kOrigin + 40ms});
    std::filesystem::remove(log::LogSegmentPath(observer, 0));
    CreateEmptyFile(log::CappedMarkerPath(observer));

    // Default refuses.
    {
      const RecordedSession session{dir};
      SimulationEnvironment env;
      Loop loop{env, log::LogWriter{out + "/refused.tlog", "consumer"},
                Loop::Options{.start = session.start()}};
      TestRobot<Loop> consumer{loop, kTopics, kGain};
      consumer.control_timer().setup_periodic(session.start() + 1ms, 1ms);
      EXPECT_THROW(ReplayInputs(loop, env, session), ReplayError);
      EXPECT_EQ(loop.dispatch_count(), 0u);
    }

    // allow_capped replays deterministically and warns.
    const InputReplayOptions opts{.allow_capped = true};
    const RecordedSession session{dir, {}, opts};
    auto run_once = [&](const std::string& log_path, std::string& stderr_out) {
      SimulationEnvironment env;
      Loop loop{env, log::LogWriter{log_path, "consumer"},
                Loop::Options{.start = session.loop_start(),
                              .run_start = session.run_start()}};
      TestRobot<Loop> consumer{loop, kTopics, kGain};
      consumer.control_timer().setup_periodic(session.loop_start() + 1ms, 1ms);
      ::testing::internal::CaptureStderr();
      const InputReplayReport report = ReplayInputs(loop, env, session, opts);
      stderr_out = ::testing::internal::GetCapturedStderr();
      return std::pair{consumer.efforts(), report};
    };
    std::string err1;
    std::string err2;
    const auto [efforts1, report1] = run_once(out + "/first.tlog", err1);
    const auto [efforts2, report2] = run_once(out + "/second.tlog", err2);
    EXPECT_FALSE(report1.stopped_at_gap.has_value());
    EXPECT_EQ(report1.capped_logs, std::vector<std::string>{observer});
    EXPECT_NE(err1.find(observer), std::string::npos) << err1;
    EXPECT_NE(err1.find("kept window"), std::string::npos) << err1;
    EXPECT_EQ(efforts1, efforts2);
    EXPECT_EQ(Sends(out + "/first.tlog"), Sends(out + "/second.tlog"));
    // Observer kept setpoints from 20..60 ms (25, 35, 45, 55 -> 4 of 6).
    ASSERT_EQ(report1.fed.size(), 2u);
    EXPECT_EQ(report1.fed[1].topic, kTopics.setpoint);
    EXPECT_EQ(report1.fed[1].messages, 4u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("manifest log_capped without .capped file");
    const std::string dir = TempDir("allow_capped_manifest");
    const std::string out = TempDir("allow_capped_manifest_out");
    RecordSession(dir, out + "/live.tlog");
    const std::string observer = dir + "/observer.tlog";
    RotateAt(observer, {kOrigin + 20ms});
    std::filesystem::remove(log::LogSegmentPath(observer, 0));
    WriteManifest(dir, "\"log_gap\": false, \"log_capped\": true");

    const RecordedSession session{dir};
    SimulationEnvironment env;
    Loop loop{env, log::LogWriter{out + "/consumer.tlog", "consumer"},
              Loop::Options{.start = session.start()}};
    TestRobot<Loop> consumer{loop, kTopics, kGain};
    consumer.control_timer().setup_periodic(session.start() + 1ms, 1ms);
    ::testing::internal::CaptureStderr();
    const InputReplayReport report = ReplayInputs(
        loop, env, session, InputReplayOptions{.allow_capped = true});
    const std::string err = ::testing::internal::GetCapturedStderr();
    EXPECT_FALSE(report.stopped_at_gap.has_value());
    EXPECT_EQ(report.capped_logs, std::vector<std::string>{observer});
    EXPECT_NE(err.find(observer), std::string::npos) << err;
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("timed node whose segment0 was rotated out by byte cap");
    const std::string dir = TempDir("allow_capped_timed");
    const std::string out = TempDir("allow_capped_timed_out");
    RecordLateTicker(dir);
    const std::string path = dir + "/ticker.tlog";
    RotateAt(path, {kOrigin + 20ms, kOrigin + 40ms});
    std::filesystem::remove(log::LogSegmentPath(path, 0));
    CreateEmptyFile(log::CappedMarkerPath(path));
    const std::vector<std::int64_t> kept_firings = LogTimerFirings(path);
    ASSERT_FALSE(kept_firings.empty());

    const InputReplayOptions opts{.allow_capped = true};
    const RecordedSession session{dir, "ticker", opts};
    SimulationEnvironment env;
    Loop loop{env, log::LogWriter{out + "/ticker.tlog", "ticker"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    const Ticker ticker{loop};
    ::testing::internal::CaptureStderr();
    const InputReplayReport report = ReplayInputs(loop, env, session, opts);
    const std::string err = ::testing::internal::GetCapturedStderr();
    EXPECT_FALSE(report.stopped_at_gap.has_value());
    EXPECT_EQ(report.capped_logs, std::vector<std::string>{path});
    EXPECT_NE(err.find(path), std::string::npos) << err;
    EXPECT_NE(err.find("kept window"), std::string::npos) << err;
    EXPECT_EQ(TimerFirings(out + "/ticker.tlog"), kept_firings);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE(
        "timed node with no firing in kept window starts at earliest dispatch");
    const std::string dir = TempDir("allow_capped_dispatch_only");
    const std::string out = TempDir("allow_capped_dispatch_only_out");
    RecordSlowTicker(dir, kOrigin + 21ms);
    const std::string path = dir + "/slow.tlog";
    ASSERT_TRUE(TimerFirings(path).empty());
    RotateAt(path, {kOrigin + 10ms});
    std::filesystem::remove(log::LogSegmentPath(path, 0));
    CreateEmptyFile(log::CappedMarkerPath(path));
    const std::vector<Receipt> kept_receipts = Receipts(path);
    ASSERT_FALSE(kept_receipts.empty());

    const InputReplayOptions opts{.allow_capped = true};
    const RecordedSession session{dir, "slow", opts};
    EXPECT_EQ(session.run_start(), kOrigin + 10ms);
    SimulationEnvironment env;
    Loop loop{env, log::LogWriter{out + "/slow.tlog", "slow"},
              Loop::Options{.delivery = Delivery::ON_PUBLISH,
                            .start = session.loop_start(),
                            .run_start = session.run_start()}};
    SlowTicker slow{loop};
    ::testing::internal::CaptureStderr();
    const InputReplayReport report = ReplayInputs(loop, env, session, opts);
    const std::string err = ::testing::internal::GetCapturedStderr();
    EXPECT_FALSE(report.stopped_at_gap.has_value());
    EXPECT_EQ(report.capped_logs, std::vector<std::string>{path});
    EXPECT_NE(err.find(path), std::string::npos) << err;
    EXPECT_EQ(TimerFirings(out + "/slow.tlog"), std::vector<std::int64_t>{});
    // The replayed SimulationEnvironment numbers messages from 0, while the
    // kept window's timestamps (now, event_time, publish_time) match live.
    const std::vector<Receipt> replayed = Receipts(out + "/slow.tlog");
    ASSERT_EQ(replayed.size(), kept_receipts.size());
    for (std::size_t i = 0; i < kept_receipts.size(); ++i) {
      EXPECT_EQ(std::get<0>(replayed[i]), std::get<0>(kept_receipts[i]));
      EXPECT_EQ(std::get<1>(replayed[i]), std::get<1>(kept_receipts[i]));
      EXPECT_EQ(std::get<2>(replayed[i]), std::get<2>(kept_receipts[i]));
    }
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// Even with allow_capped = true, uncapped log_gap, recorder_failed, incomplete
// headers, unmarked missing segment0, and unbounded damage are still refused,
// and GAP records in the kept window still stop the replay.
// REQ-OPS-14
TEST(InputReplay, AllowCappedStillRefusesUncappedGapsDamageAndBrokenSegments) {
  const InputReplayOptions allow{.allow_capped = true};
  {
    SCOPED_TRACE("capped AND log_gap without EXIT");
    const std::string dir = TempDir("allow_capped_log_gap");
    const std::string out = TempDir("allow_capped_log_gap_out");
    RecordSession(dir, out + "/live.tlog");
    CreateEmptyFile(dir + "/observer.tlog.capped");
    WriteManifest(dir, "\"log_gap\": true, \"log_capped\": true");
    std::filesystem::resize_file(dir + "/observer.tlog",
                                 RecordOffsets(dir + "/observer.tlog").back());
    const RecordedSession session{dir, {}, allow};
    SimulationEnvironment env;
    Loop loop{env, log::LogWriter{out + "/consumer.tlog", "consumer"},
              Loop::Options{.start = session.start()}};
    TestRobot<Loop> consumer{loop, kTopics, kGain};
    consumer.control_timer().setup_periodic(session.start() + 1ms, 1ms);
    EXPECT_THROW(ReplayInputs(loop, env, session, allow), ReplayError);
    EXPECT_EQ(loop.dispatch_count(), 0u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("capped AND recorder_failed without EXIT");
    const std::string dir = TempDir("allow_capped_rec_failed");
    const std::string out = TempDir("allow_capped_rec_failed_out");
    RecordSession(dir, out + "/live.tlog");
    CreateEmptyFile(dir + "/observer.tlog.capped");
    WriteManifest(
        dir,
        "\"log_gap\": false, \"log_capped\": true, \"recorder_failed\": true");
    std::filesystem::resize_file(dir + "/observer.tlog",
                                 RecordOffsets(dir + "/observer.tlog").back());
    const RecordedSession session{dir, {}, allow};
    SimulationEnvironment env;
    Loop loop{env, log::LogWriter{out + "/consumer.tlog", "consumer"},
              Loop::Options{.start = session.start()}};
    TestRobot<Loop> consumer{loop, kTopics, kGain};
    consumer.control_timer().setup_periodic(session.start() + 1ms, 1ms);
    EXPECT_THROW(ReplayInputs(loop, env, session, allow), ReplayError);
    EXPECT_EQ(loop.dispatch_count(), 0u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("capped with incomplete header on segment0");
    const std::string dir = TempDir("allow_capped_incomplete_head");
    const std::string out = TempDir("allow_capped_incomplete_head_out");
    RecordLateTicker(dir);
    const std::string path = dir + "/ticker.tlog";
    RotateAt(path, {kOrigin + 20ms, kOrigin + 40ms});
    ZeroFileHeader(log::LogSegmentPath(path, 0));
    CreateEmptyFile(log::CappedMarkerPath(path));
    const RecordedSession session{dir, "ticker", allow};
    SimulationEnvironment env;
    Loop loop{env, log::LogWriter{out + "/ticker.tlog", "ticker"},
              Loop::Options{.start = session.loop_start(),
                            .run_start = session.run_start()}};
    const Ticker ticker{loop};
    EXPECT_THROW(ReplayInputs(loop, env, session, allow), ReplayError);
    EXPECT_EQ(loop.dispatch_count(), 0u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("capped with unbounded damage");
    const std::string dir = TempDir("allow_capped_damage");
    const std::string out = TempDir("allow_capped_damage_out");
    const std::vector<std::uint64_t> offsets = ReaderLog(dir);
    DamageRecord(dir + "/reader.tlog", offsets[0]);
    CreateEmptyFile(dir + "/reader.tlog.capped");
    const RecordedSession session{dir, {}, allow};
    SimulationEnvironment env;
    Loop loop{env, log::LogWriter{out + "/watcher.tlog", "watcher"},
              Loop::Options{.start = session.start()}};
    SensorWatcher watcher{loop};
    EXPECT_THROW(ReplayInputs(loop, env, session, allow), ReplayError);
    EXPECT_EQ(watcher.readings, 0);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("missing segment0 without .capped or manifest log_capped");
    const std::string dir = TempDir("allow_capped_unmarked_missing");
    const std::string out = TempDir("allow_capped_unmarked_missing_out");
    RecordSession(dir, out + "/live.tlog");
    const std::string observer = dir + "/observer.tlog";
    RotateAt(observer, {kOrigin + 20ms, kOrigin + 40ms});
    std::filesystem::remove(log::LogSegmentPath(observer, 0));
    const RecordedSession session{dir, {}, allow};
    SimulationEnvironment env;
    Loop loop{env, log::LogWriter{out + "/consumer.tlog", "consumer"},
              Loop::Options{.start = session.start()}};
    TestRobot<Loop> consumer{loop, kTopics, kGain};
    consumer.control_timer().setup_periodic(session.start() + 1ms, 1ms);
    EXPECT_THROW(ReplayInputs(loop, env, session, allow), ReplayError);
    EXPECT_EQ(loop.dispatch_count(), 0u);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE(
        "capped log with a GAP record in the kept window still stops at gap");
    const std::string dir = TempDir("allow_capped_gap_in_window");
    const std::string out = TempDir("allow_capped_gap_in_window_out");
    RecordSession(dir, out + "/live.tlog");
    const std::string observer = dir + "/observer.tlog";
    const log::LogGap gap = MarkGap(observer, kOrigin + 45ms);
    RotateAt(observer, {kOrigin + 20ms});
    std::filesystem::remove(log::LogSegmentPath(observer, 0));
    CreateEmptyFile(log::CappedMarkerPath(observer));
    const RecordedSession session{dir, {}, allow};
    SimulationEnvironment env;
    Loop loop{env, log::LogWriter{out + "/consumer.tlog", "consumer"},
              Loop::Options{.start = session.start()}};
    TestRobot<Loop> consumer{loop, kTopics, kGain};
    consumer.control_timer().setup_periodic(session.start() + 1ms, 1ms);
    const InputReplayReport report = ReplayInputs(loop, env, session, allow);
    ASSERT_TRUE(report.stopped_at_gap.has_value());
    EXPECT_EQ(report.stopped_at_gap->log, observer);
    EXPECT_EQ(report.stopped_at_gap->at.nanos(), gap.start_ns);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// session_manifest::Parse reads optional per-run gap_logs and
// recorder_failed_logs arrays, leaving them absent in legacy manifests and on
// malformed non-string arrays.
// REQ-OPS-14
TEST(InputReplay, SessionManifestParsesPerRunLogArrays) {
  const std::string json = R"({
    "session_id": 42,
    "nodes": [
      {
        "name": "modern",
        "log_paths": ["/s/modern.tlog", "/s/modern.restart1.tlog"],
        "log_gap": true,
        "gap_logs": ["/s/modern.tlog"],
        "recorder_failed": true,
        "recorder_failed_logs": ["/s/modern.restart1.tlog"],
        "log_capped": false
      },
      {
        "name": "empty_arrays",
        "log_paths": ["/s/empty.tlog"],
        "log_gap": false,
        "gap_logs": [],
        "recorder_failed": true,
        "recorder_failed_logs": [],
        "log_capped": false
      },
      {
        "name": "legacy",
        "log_paths": ["/s/legacy.tlog"],
        "log_gap": true,
        "recorder_failed": true,
        "log_capped": false
      },
      {
        "name": "malformed",
        "log_paths": ["/s/malformed.tlog"],
        "log_gap": true,
        "gap_logs": [123],
        "recorder_failed": true,
        "recorder_failed_logs": "not_an_array",
        "log_capped": false
      }
    ]
  })";
  const session_manifest::Fields fields = session_manifest::Parse(json);
  ASSERT_EQ(fields.session_id, 42u);
  ASSERT_EQ(fields.nodes.size(), 4u);

  ASSERT_TRUE(fields.nodes[0].gap_logs.has_value());
  EXPECT_EQ(*fields.nodes[0].gap_logs,
            std::vector<std::string>{"/s/modern.tlog"});
  ASSERT_TRUE(fields.nodes[0].recorder_failed_logs.has_value());
  EXPECT_EQ(*fields.nodes[0].recorder_failed_logs,
            std::vector<std::string>{"/s/modern.restart1.tlog"});

  ASSERT_TRUE(fields.nodes[1].gap_logs.has_value());
  EXPECT_TRUE(fields.nodes[1].gap_logs->empty());
  ASSERT_TRUE(fields.nodes[1].recorder_failed_logs.has_value());
  EXPECT_TRUE(fields.nodes[1].recorder_failed_logs->empty());

  EXPECT_FALSE(fields.nodes[2].gap_logs.has_value());
  EXPECT_FALSE(fields.nodes[2].recorder_failed_logs.has_value());

  EXPECT_FALSE(fields.nodes[3].gap_logs.has_value());
  EXPECT_FALSE(fields.nodes[3].recorder_failed_logs.has_value());
}

// LOG-29(a): a writer's damaged stretch is no loss when an intact accepted
// send follows it (ignoring refused sends with aux != 0) and readers' logs
// hold every sequence in [from, to).
// REQ-OPS-14
TEST(InputReplay, WritersDamageCoveredByReadersReplaysInFull) {
  {
    SCOPED_TRACE("middle send damaged in writer, held by reader");
    const std::string dir = TempDir("writer_dmg_mid");
    const std::string out = TempDir("writer_dmg_mid_out");
    const std::vector<std::uint64_t> offsets = WriterLog(dir);
    ReaderLog(dir);
    DamageRecord(dir + "/writer.tlog", offsets[5]);  // 3rd send (seq 2)
    const RecordedSession session{dir};
    const WatcherRun run = ReplayWatcher(session, out);
    EXPECT_FALSE(run.report.stopped_at_gap.has_value());
    EXPECT_EQ(run.readings, 4);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE("first send damaged in writer (from=0), held by reader");
    const std::string dir = TempDir("writer_dmg_first");
    const std::string out = TempDir("writer_dmg_first_out");
    const std::vector<std::uint64_t> offsets = WriterLog(dir);
    ReaderLog(dir);
    DamageRecord(dir + "/writer.tlog", offsets[1]);  // 1st send (seq 0)
    const RecordedSession session{dir};
    const WatcherRun run = ReplayWatcher(session, out);
    EXPECT_FALSE(run.report.stopped_at_gap.has_value());
    EXPECT_EQ(run.readings, 4);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    // A refused send (aux != 0, sequence = 0) right after damage must not
    // close the writer's stretch prematurely: when the reader lacks seq 2, the
    // stretch still stops; when the reader holds seq 2, the next accepted send
    // (seq 3) closes [2, 3) and replay completes.
    SCOPED_TRACE("refused send (aux != 0) after damage does not close stretch");
    for (const bool reader_has_all : {false, true}) {
      SCOPED_TRACE(reader_has_all ? "reader holds seq 2"
                                  : "reader lacks seq 2");
      const std::string dir = TempDir("writer_dmg_refused");
      const std::string out = TempDir("writer_dmg_refused_out");
      const std::string wpath = dir + "/writer.tlog";
      std::uint64_t third_send_offset = 0;
      {
        log::LogWriter writer{wpath, "writer",
                              log::LogWriterOptions{.background = false}};
        writer.start(ManifestOf<SensorWriter>(dir + "/unused.tlog"), kOrigin);
        for (int i = 1; i <= 4; ++i) {
          const Context tick{
              .kind = EventKind::TIMER,
              .source_id = 1,
              .dispatch_index = static_cast<std::uint64_t>(i),
              .event_time = kOrigin + std::chrono::milliseconds{i},
              .now = kOrigin + std::chrono::milliseconds{i},
              .sequence = 1};
          writer.dispatch(tick, {});
          const EventsTest::SensorMessage reading{i, 1.0F, 1.0F};
          if (i == 4) {
            // Refused send (aux = 1, sequence = 0) before accepted seq 3.
            writer.send(tick, 0, std::as_bytes(std::span{&reading, 1}),
                        /*status=*/1, /*sequence=*/0);
          }
          writer.send(tick, 0, std::as_bytes(std::span{&reading, 1}),
                      /*status=*/0, static_cast<std::uint64_t>(i - 1));
        }
        writer.finish(Context{.kind = EventKind::EXIT,
                              .dispatch_index = 5,
                              .event_time = kOrigin + 5ms,
                              .now = kOrigin + 5ms});
        writer.flush();
      }
      third_send_offset =
          RecordOffsets(wpath)[5];  // tick1,s0,tick2,s1,tick3,s2
      DamageRecord(wpath, third_send_offset);
      if (reader_has_all) {
        ReaderLog(dir);
      }
      const RecordedSession session{dir};
      const WatcherRun run = ReplayWatcher(session, out);
      if (reader_has_all) {
        EXPECT_FALSE(run.report.stopped_at_gap.has_value());
        EXPECT_EQ(run.readings, 4);
      } else {
        ASSERT_TRUE(run.report.stopped_at_gap.has_value());
        EXPECT_EQ(run.report.stopped_at_gap->log, wpath);
        EXPECT_EQ(run.report.stopped_at_gap->damage_offset, third_send_offset);
        EXPECT_EQ(run.readings, 2);
      }
      std::filesystem::remove_all(dir);
      std::filesystem::remove_all(out);
    }
  }
}

// LOG-29(b): an open damaged stretch in a reader or writer log carries across
// contiguous segment boundaries and closes on the next intact receipt/send in
// a later segment; a missing middle segment refuses rather than closing it.
// REQ-OPS-14
TEST(InputReplay, DamagedStretchClosesAcrossContiguousSegmentBoundary) {
  {
    // Writer lacks EXIT (so case (c) does not apply): recovery depends solely
    // on closing reader.segment0.tlog's open stretch on the receipt in
    // reader.tlog.
    SCOPED_TRACE("reader stretch at segment0 tail closed in active segment");
    for (const bool writer_holds_seq1 : {true, false}) {
      SCOPED_TRACE(writer_holds_seq1 ? "writer holds seq 1"
                                     : "writer lacks seq 1");
      const std::string dir = TempDir("seg_cross_reader");
      const std::string out = TempDir("seg_cross_reader_out");
      const std::vector<std::uint64_t> woff = WriterLog(dir);
      // Drop EXIT (and seq 1..3 when !writer_holds_seq1).
      std::filesystem::resize_file(dir + "/writer.tlog",
                                   writer_holds_seq1 ? woff.back() : woff[2]);
      ReaderLog(dir);
      const std::string rpath = dir + "/reader.tlog";
      // Split between receipt 2 (seq 1, now=kDrain+10us) and receipt 3 (seq 2,
      // now=kDrain+15us) after adding a trailing non-receipt record (or split
      // so seq 1 is damaged with an intact record after it? Wait: LogReader
      // only counts a stretch when an intact record follows it in that file!).
      // Put receipts 0, 1, 2 in segment0 and receipt 3 in active; write an
      // extra TIMER/EXIT? Instead, have reader watch two sources or put a
      // second intact record after the damaged receipt in segment0!
      std::filesystem::remove(rpath);
      {
        log::LogWriter writer{rpath, "reader",
                              log::LogWriterOptions{.background = false}};
        // SensorWatcher + timer so segment0 has an intact TIMER after the
        // damaged receipt (making it a damaged stretch, not a tail) while the
        // next intact receipt of /input/sensor is in the active segment.
        SimulationEnvironment env;
        Loop tmpl{env, log::LogWriter{dir + "/unused.tlog", "tmpl"}};
        SensorWatcher w{tmpl};
        tmpl.register_timer("tick", Thunk{});
        writer.start(tmpl.manifest(), kOrigin);
        for (int i = 1; i <= 2; ++i) {
          const EventsTest::SensorMessage reading{i, 1.0F, 1.0F};
          writer.dispatch(
              Context{.kind = EventKind::MESSAGE,
                      .source_id = 0,
                      .dispatch_index = static_cast<std::uint64_t>(i),
                      .event_time = kDrain,
                      .now = kDrain + std::chrono::microseconds{5 * i},
                      .publish_time = kOrigin + std::chrono::milliseconds{i},
                      .sequence = static_cast<std::uint64_t>(i - 1)},
              std::as_bytes(std::span{&reading, 1}));
        }
        // Intact timer at end of segment0 after damaged receipt seq 1.
        writer.dispatch(Context{.kind = EventKind::TIMER,
                                .source_id = 1,
                                .dispatch_index = 3,
                                .event_time = kDrain + 12us,
                                .now = kDrain + 12us,
                                .sequence = 1},
                        {});
        for (int i = 3; i <= 4; ++i) {
          const EventsTest::SensorMessage reading{i, 1.0F, 1.0F};
          writer.dispatch(
              Context{.kind = EventKind::MESSAGE,
                      .source_id = 0,
                      .dispatch_index = static_cast<std::uint64_t>(i + 1),
                      .event_time = kDrain,
                      .now = kDrain + std::chrono::microseconds{5 * i},
                      .publish_time = kOrigin + std::chrono::milliseconds{i},
                      .sequence = static_cast<std::uint64_t>(i - 1)},
              std::as_bytes(std::span{&reading, 1}));
        }
        writer.finish(Context{.kind = EventKind::EXIT,
                              .dispatch_index = 6,
                              .event_time = kDrain + 25us,
                              .now = kDrain + 25us});
        writer.flush();
      }
      RotateAt(rpath, {kDrain + 15us});
      const std::string seg0 = log::LogSegmentPath(rpath, 0);
      const std::vector<std::uint64_t> seg0_offsets = RecordOffsets(seg0);
      ASSERT_EQ(seg0_offsets.size(), 3u);   // msg0, msg1, timer
      DamageRecord(seg0, seg0_offsets[1]);  // damage msg1 (seq 1) in segment0

      const RecordedSession session{dir};
      const WatcherRun run = ReplayWatcher(session, out);
      if (writer_holds_seq1) {
        EXPECT_FALSE(run.report.stopped_at_gap.has_value());
        EXPECT_EQ(run.readings, 4);
      } else {
        ASSERT_TRUE(run.report.stopped_at_gap.has_value());
        EXPECT_EQ(run.report.stopped_at_gap->log, seg0);
        EXPECT_EQ(run.report.stopped_at_gap->damage_offset, seg0_offsets[1]);
      }
      std::filesystem::remove_all(dir);
      std::filesystem::remove_all(out);
    }
  }
  {
    SCOPED_TRACE("writer stretch at segment0 tail closed in active segment");
    const std::string dir = TempDir("seg_cross_writer");
    const std::string out = TempDir("seg_cross_writer_out");
    ReaderLog(dir);
    // Drop reader EXIT so writer recovery is exercised directly.
    std::filesystem::resize_file(dir + "/reader.tlog",
                                 RecordOffsets(dir + "/reader.tlog").back());
    const std::string wpath = dir + "/writer.tlog";
    {
      log::LogWriter writer{wpath, "writer",
                            log::LogWriterOptions{.background = false}};
      writer.start(ManifestOf<SensorWriter>(dir + "/unused.tlog"), kOrigin);
      for (int i = 1; i <= 2; ++i) {
        const Context tick{.kind = EventKind::TIMER,
                           .source_id = 1,
                           .dispatch_index = static_cast<std::uint64_t>(i),
                           .event_time = kOrigin + std::chrono::milliseconds{i},
                           .now = kOrigin + std::chrono::milliseconds{i},
                           .sequence = 1};
        writer.dispatch(tick, {});
        const EventsTest::SensorMessage reading{i, 1.0F, 1.0F};
        writer.send(tick, 0, std::as_bytes(std::span{&reading, 1}), 0,
                    static_cast<std::uint64_t>(i - 1));
      }
      // Extra intact timer at end of segment0 so damaged send seq 1 is a
      // stretch (not a tail), with next accepted send (seq 2) in active file.
      writer.dispatch(Context{.kind = EventKind::TIMER,
                              .source_id = 1,
                              .dispatch_index = 3,
                              .event_time = kOrigin + 2500us,
                              .now = kOrigin + 2500us,
                              .sequence = 1},
                      {});
      for (int i = 3; i <= 4; ++i) {
        const Context tick{.kind = EventKind::TIMER,
                           .source_id = 1,
                           .dispatch_index = static_cast<std::uint64_t>(i + 1),
                           .event_time = kOrigin + std::chrono::milliseconds{i},
                           .now = kOrigin + std::chrono::milliseconds{i},
                           .sequence = 1};
        writer.dispatch(tick, {});
        const EventsTest::SensorMessage reading{i, 1.0F, 1.0F};
        writer.send(tick, 0, std::as_bytes(std::span{&reading, 1}), 0,
                    static_cast<std::uint64_t>(i - 1));
      }
      writer.finish(Context{.kind = EventKind::EXIT,
                            .dispatch_index = 6,
                            .event_time = kOrigin + 5ms,
                            .now = kOrigin + 5ms});
      writer.flush();
    }
    RotateAt(wpath, {kOrigin + 3ms});
    const std::string seg0 = log::LogSegmentPath(wpath, 0);
    const std::vector<std::uint64_t> seg0_offsets = RecordOffsets(seg0);
    ASSERT_EQ(seg0_offsets.size(), 5u);   // t1, s0, t2, s1, t_extra
    DamageRecord(seg0, seg0_offsets[3]);  // damage send seq 1 in segment0

    const RecordedSession session{dir};
    const WatcherRun run = ReplayWatcher(session, out);
    EXPECT_FALSE(run.report.stopped_at_gap.has_value());
    EXPECT_EQ(run.readings, 4);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
  {
    SCOPED_TRACE(
        "missing middle segment refuses and does not close carried stretch");
    const std::string dir = TempDir("seg_cross_broken");
    const std::string out = TempDir("seg_cross_broken_out");
    const std::vector<std::uint64_t> woff = WriterLog(dir);
    std::filesystem::resize_file(dir + "/writer.tlog", woff.back());
    const std::string rpath = dir + "/reader.tlog";
    {
      log::LogWriter writer{rpath, "reader",
                            log::LogWriterOptions{.background = false}};
      SimulationEnvironment env;
      Loop tmpl{env, log::LogWriter{dir + "/unused.tlog", "tmpl"}};
      SensorWatcher w{tmpl};
      tmpl.register_timer("tick", Thunk{});
      writer.start(tmpl.manifest(), kOrigin);
      for (int i = 1; i <= 4; ++i) {
        const EventsTest::SensorMessage reading{i, 1.0F, 1.0F};
        writer.dispatch(
            Context{.kind = EventKind::MESSAGE,
                    .source_id = 0,
                    .dispatch_index = static_cast<std::uint64_t>(2 * i - 1),
                    .event_time = kDrain,
                    .now = kDrain + std::chrono::microseconds{10 * i},
                    .publish_time = kOrigin + std::chrono::milliseconds{i},
                    .sequence = static_cast<std::uint64_t>(i - 1)},
            std::as_bytes(std::span{&reading, 1}));
        writer.dispatch(
            Context{
                .kind = EventKind::TIMER,
                .source_id = 1,
                .dispatch_index = static_cast<std::uint64_t>(2 * i),
                .event_time = kDrain + std::chrono::microseconds{10 * i + 2},
                .now = kDrain + std::chrono::microseconds{10 * i + 2},
                .sequence = 1},
            {});
      }
      writer.finish(Context{.kind = EventKind::EXIT,
                            .dispatch_index = 9,
                            .event_time = kDrain + 50us,
                            .now = kDrain + 50us});
      writer.flush();
    }
    RotateAt(rpath, {kDrain + 20us, kDrain + 30us});
    const std::string seg0 = log::LogSegmentPath(rpath, 0);
    const std::string seg1 = log::LogSegmentPath(rpath, 1);
    DamageRecord(seg0, RecordOffsets(seg0)[0]);  // damage msg0 in segment0
    std::filesystem::remove(seg1);               // missing middle segment1

    const RecordedSession session{dir};
    EXPECT_THROW(ReplayWatcher(session, out), ReplayError);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// LOG-29(c): a complete, exited writer log holding [0, max_seq] rescues a
// capped, manifest-marked, or gapped reader log for non-replayed topics, while
// an incomplete writer (missing EXIT, truncated, gapped, capped, or missing a
// sequence) or a topic watched live by the replayed program still
// refuses/stops. REQ-OPS-14
TEST(InputReplay, CompleteWriterRescuesCappedMarkedOrGappedReader) {
  enum class ReaderFault {
    kCappedMarker,
    kManifestCapped,
    kManifestLogGapNoExit,
    kManifestRecorderFailedNoExit,
    kGapRecord,
  };
  for (const ReaderFault fault :
       {ReaderFault::kCappedMarker, ReaderFault::kManifestCapped,
        ReaderFault::kManifestLogGapNoExit,
        ReaderFault::kManifestRecorderFailedNoExit, ReaderFault::kGapRecord}) {
    const std::string dir = TempDir("writer_rescues_reader");
    const std::string out = TempDir("writer_rescues_reader_out");
    WriterLog(dir);
    ReaderLog(dir);
    const std::string rpath = dir + "/reader.tlog";
    const std::uint64_t id = log::LogReader{rpath}.session_id();
    switch (fault) {
      case ReaderFault::kCappedMarker:
        CreateEmptyFile(log::CappedMarkerPath(rpath));
        break;
      case ReaderFault::kManifestCapped:
        std::ofstream{dir + "/manifest.json"}
            << "{\"session_id\": " << id
            << ", \"nodes\": [{\"name\": \"reader\", \"log_paths\": "
               "[\"/s/reader.tlog\"], \"log_capped\": true, "
               "\"log_gap\": false, \"recorder_failed\": false}]}\n";
        break;
      case ReaderFault::kManifestLogGapNoExit:
        std::filesystem::resize_file(rpath, RecordOffsets(rpath).back());
        std::ofstream{dir + "/manifest.json"}
            << "{\"session_id\": " << id
            << ", \"nodes\": [{\"name\": \"reader\", \"log_paths\": "
               "[\"/s/reader.tlog\"], \"log_capped\": false, "
               "\"log_gap\": true, \"gap_logs\": [\"/s/reader.tlog\"], "
               "\"recorder_failed\": true, \"recorder_failed_logs\": []}]}\n";
        break;
      case ReaderFault::kManifestRecorderFailedNoExit:
        std::filesystem::resize_file(rpath, RecordOffsets(rpath).back());
        std::ofstream{dir + "/manifest.json"}
            << "{\"session_id\": " << id
            << ", \"nodes\": [{\"name\": \"reader\", \"log_paths\": "
               "[\"/s/reader.tlog\"], \"log_capped\": false, "
               "\"log_gap\": false, \"gap_logs\": [], "
               "\"recorder_failed\": true, "
               "\"recorder_failed_logs\": [\"/s/reader.tlog\"]}]}\n";
        break;
      case ReaderFault::kGapRecord:
        MarkGap(rpath, kDrain + 5us);
        break;
    }

    // Non-replayed consumer: writer holds /input/sensor in full -> succeeds!
    {
      const RecordedSession session{dir};
      const WatcherRun run = ReplayWatcher(session, out);
      EXPECT_FALSE(run.report.stopped_at_gap.has_value());
      EXPECT_TRUE(run.report.capped_logs.empty());
      EXPECT_EQ(run.readings, 4);
    }

    // Replayed program watching the topic live ("reader"): reader's own
    // cap/mark/gap still applies (never rescued by writer).
    {
      const RecordedSession session{dir, "reader"};
      SimulationEnvironment env;
      Loop loop{env, log::LogWriter{out + "/reader.tlog", "reader"},
                Loop::Options{.start = session.loop_start(),
                              .run_start = session.run_start()}};
      SensorWatcher watcher{loop};
      if (fault == ReaderFault::kGapRecord) {
        const InputReplayReport report = ReplayInputs(loop, env, session);
        EXPECT_TRUE(report.stopped_at_gap.has_value());
      } else {
        EXPECT_THROW(ReplayInputs(loop, env, session), ReplayError);
      }
    }

    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }

  // Incomplete writer still refuses capped reader.
  enum class WriterDefect {
    kNoExit,
    kTruncated,
    kGapped,
    kCapped,
    kMissingSeq0
  };
  for (const WriterDefect defect :
       {WriterDefect::kNoExit, WriterDefect::kTruncated, WriterDefect::kGapped,
        WriterDefect::kCapped, WriterDefect::kMissingSeq0}) {
    const std::string dir = TempDir("incomplete_writer_refuses");
    const std::string out = TempDir("incomplete_writer_refuses_out");
    const std::string wpath = dir + "/writer.tlog";
    if (defect == WriterDefect::kMissingSeq0) {
      // Writer starts at sequence 1 (missing sequence 0), reader also starts
      // at sequence 1 -> held(topic, 0, max_seq + 1) is false.
      {
        log::LogWriter writer{wpath, "writer",
                              log::LogWriterOptions{.background = false}};
        writer.start(ManifestOf<SensorWriter>(dir + "/unused.tlog"), kOrigin);
        const Context tick{.kind = EventKind::TIMER,
                           .source_id = 1,
                           .dispatch_index = 1,
                           .event_time = kOrigin + 2ms,
                           .now = kOrigin + 2ms,
                           .sequence = 1};
        writer.dispatch(tick, {});
        const EventsTest::SensorMessage reading{2, 1.0F, 1.0F};
        writer.send(tick, 0, std::as_bytes(std::span{&reading, 1}), 0, 1);
        writer.finish(Context{.kind = EventKind::EXIT,
                              .dispatch_index = 2,
                              .event_time = kOrigin + 3ms,
                              .now = kOrigin + 3ms});
        writer.flush();
      }
      {
        log::LogWriter writer{dir + "/reader.tlog", "reader",
                              log::LogWriterOptions{.background = false}};
        writer.start(ManifestOf<SensorWatcher>(dir + "/unused.tlog"), kOrigin);
        const EventsTest::SensorMessage reading{2, 1.0F, 1.0F};
        writer.dispatch(Context{.kind = EventKind::MESSAGE,
                                .source_id = 0,
                                .dispatch_index = 1,
                                .event_time = kDrain,
                                .now = kDrain + 5us,
                                .publish_time = kOrigin + 2ms,
                                .sequence = 1},
                        std::as_bytes(std::span{&reading, 1}));
        writer.finish(Context{.kind = EventKind::EXIT,
                              .dispatch_index = 2,
                              .event_time = kDrain + 10us,
                              .now = kDrain + 10us});
        writer.flush();
      }
    } else {
      const std::vector<std::uint64_t> woff = WriterLog(dir);
      ReaderLog(dir);
      if (defect == WriterDefect::kNoExit) {
        std::filesystem::resize_file(wpath, woff.back());
      } else if (defect == WriterDefect::kTruncated) {
        std::ofstream append{wpath, std::ios::binary | std::ios::app};
        const char garbage[16] = {1, 2, 3, 4};
        append.write(garbage, sizeof(garbage));
      } else if (defect == WriterDefect::kGapped) {
        MarkGap(wpath, kOrigin + 1ms);
      } else if (defect == WriterDefect::kCapped) {
        CreateEmptyFile(log::CappedMarkerPath(wpath));
      }
    }
    CreateEmptyFile(log::CappedMarkerPath(dir + "/reader.tlog"));
    const RecordedSession session{dir};
    EXPECT_THROW(ReplayWatcher(session, out), ReplayError);
    std::filesystem::remove_all(dir);
    std::filesystem::remove_all(out);
  }
}

// LOG-29(d): merging by (topic, sequence) fails closed with ReplayError naming
// the topic and sequence when two records share (topic, sequence) with
// differing payload or publish_time_ns, or when a single log observes a
// non-monotonic sequence (e.g. a segment re-created outside a launcher session
// restarting at 0).
// REQ-OPS-14
TEST(InputReplay, FailClosedOnConflictingOrNonMonotonicSequences) {
  {
    SCOPED_TRACE("conflicting payload across logs on same (topic, sequence)");
    const std::string dir = TempDir("conflict_payload");
    WriterLog(dir);
    const std::string rpath = dir + "/reader.tlog";
    {
      log::LogWriter writer{rpath, "reader",
                            log::LogWriterOptions{.background = false}};
      writer.start(ManifestOf<SensorWatcher>(dir + "/unused.tlog"), kOrigin);
      const EventsTest::SensorMessage different{999, 9.0F, 9.0F};
      writer.dispatch(Context{.kind = EventKind::MESSAGE,
                              .source_id = 0,
                              .dispatch_index = 1,
                              .event_time = kDrain,
                              .now = kDrain + 5us,
                              .publish_time = kOrigin + 1ms,
                              .sequence = 0},
                      std::as_bytes(std::span{&different, 1}));
      writer.finish(Context{.kind = EventKind::EXIT,
                            .dispatch_index = 2,
                            .event_time = kDrain + 10us,
                            .now = kDrain + 10us});
      writer.flush();
    }
    try {
      const RecordedSession session{dir};
      ADD_FAILURE() << "accepted conflicting payloads for sequence 0";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(kTopics.sensor), std::string::npos) << what;
      EXPECT_NE(what.find("sequence 0"), std::string::npos) << what;
    }
    // Also fails when replaying "reader" (own_log vs writer cross-check).
    try {
      const RecordedSession session{dir, "reader"};
      ADD_FAILURE() << "accepted conflicting own_log vs writer sequence 0";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(kTopics.sensor), std::string::npos) << what;
      EXPECT_NE(what.find("sequence 0"), std::string::npos) << what;
    }
    std::filesystem::remove_all(dir);
  }
  {
    SCOPED_TRACE(
        "conflicting publish_time_ns across logs on same (topic, sequence)");
    const std::string dir = TempDir("conflict_time");
    WriterLog(dir);
    const std::string rpath = dir + "/reader.tlog";
    {
      log::LogWriter writer{rpath, "reader",
                            log::LogWriterOptions{.background = false}};
      writer.start(ManifestOf<SensorWatcher>(dir + "/unused.tlog"), kOrigin);
      const EventsTest::SensorMessage same{1, 1.0F, 1.0F};
      writer.dispatch(
          Context{.kind = EventKind::MESSAGE,
                  .source_id = 0,
                  .dispatch_index = 1,
                  .event_time = kDrain,
                  .now = kDrain + 5us,
                  .publish_time = kOrigin + 9ms,  // differs from 1ms
                  .sequence = 0},
          std::as_bytes(std::span{&same, 1}));
      writer.finish(Context{.kind = EventKind::EXIT,
                            .dispatch_index = 2,
                            .event_time = kDrain + 10us,
                            .now = kDrain + 10us});
      writer.flush();
    }
    try {
      const RecordedSession session{dir};
      ADD_FAILURE() << "accepted conflicting publish_time_ns for sequence 0";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(kTopics.sensor), std::string::npos) << what;
      EXPECT_NE(what.find("sequence 0"), std::string::npos) << what;
    }
    std::filesystem::remove_all(dir);
  }
  {
    SCOPED_TRACE(
        "non-monotonic sequence in single sender log (restarted at 0)");
    const std::string dir = TempDir("nonmono_sender");
    const std::string wpath = dir + "/writer.tlog";
    {
      log::LogWriter writer{wpath, "writer",
                            log::LogWriterOptions{.background = false}};
      writer.start(ManifestOf<SensorWriter>(dir + "/unused.tlog"), kOrigin);
      const std::uint64_t seqs[] = {0, 1, 0};
      for (int i = 0; i < 3; ++i) {
        const Context tick{
            .kind = EventKind::TIMER,
            .source_id = 1,
            .dispatch_index = static_cast<std::uint64_t>(i + 1),
            .event_time = kOrigin + std::chrono::milliseconds{i + 1},
            .now = kOrigin + std::chrono::milliseconds{i + 1},
            .sequence = 1};
        writer.dispatch(tick, {});
        const EventsTest::SensorMessage reading{i + 1, 1.0F, 1.0F};
        writer.send(tick, 0, std::as_bytes(std::span{&reading, 1}), 0, seqs[i]);
      }
      writer.finish(Context{.kind = EventKind::EXIT,
                            .dispatch_index = 4,
                            .event_time = kOrigin + 5ms,
                            .now = kOrigin + 5ms});
      writer.flush();
    }
    try {
      const RecordedSession session{dir};
      ADD_FAILURE() << "accepted non-monotonic sender sequence 0 after 1";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(kTopics.sensor), std::string::npos) << what;
      EXPECT_NE(what.find("sequence 0"), std::string::npos) << what;
    }
    std::filesystem::remove_all(dir);
  }
  {
    SCOPED_TRACE(
        "non-monotonic sequence in single reader log across segment boundary");
    const std::string dir = TempDir("nonmono_reader_seg");
    const std::string rpath = dir + "/reader.tlog";
    {
      log::LogWriter writer{rpath, "reader",
                            log::LogWriterOptions{.background = false}};
      writer.start(ManifestOf<SensorWatcher>(dir + "/unused.tlog"), kOrigin);
      const std::uint64_t seqs[] = {0, 1, 0};
      for (int i = 0; i < 3; ++i) {
        const EventsTest::SensorMessage reading{i + 1, 1.0F, 1.0F};
        writer.dispatch(
            Context{.kind = EventKind::MESSAGE,
                    .source_id = 0,
                    .dispatch_index = static_cast<std::uint64_t>(i + 1),
                    .event_time = kDrain,
                    .now = kDrain + std::chrono::microseconds{5 * (i + 1)},
                    .publish_time = kOrigin + std::chrono::milliseconds{i + 1},
                    .sequence = seqs[i]},
            std::as_bytes(std::span{&reading, 1}));
      }
      writer.finish(Context{.kind = EventKind::EXIT,
                            .dispatch_index = 4,
                            .event_time = kDrain + 25us,
                            .now = kDrain + 25us});
      writer.flush();
    }
    RotateAt(rpath, {kDrain + 15us});
    try {
      const RecordedSession session{dir};
      ADD_FAILURE() << "accepted non-monotonic reader sequence across segment";
    } catch (const ReplayError& e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(kTopics.sensor), std::string::npos) << what;
      EXPECT_NE(what.find("sequence 0"), std::string::npos) << what;
    }
    std::filesystem::remove_all(dir);
  }
}

// Wave 9 B1 regression test: under allow_capped with segment 0 rotated out,
// late-EXIT clamping still suppresses a fabricated firing for timers observed
// in the kept window, while an unobserved timer (e.g. disarmed in rotated-out
// segment 0) does not clamp loop_end() early.
// REQ-OPS-14
TEST(InputReplay, AllowCappedClampsLateExitWhenSegmentZeroRotatedOut) {
  struct TwoTimers {
    explicit TwoTimers(Loop& loop)
        : fast{make_timer<&TwoTimers::on_fast>(loop, "fast", this)},
          disarmed_early{
              make_timer<&TwoTimers::on_early>(loop, "early", this)} {
      fast.setup_periodic(loop.monotonic_now() + 1ms, 1ms);
      // Offset 6.5 ms: if unobserved disarmed_early participated in clamping,
      // run_start (4 ms) + 6.5 ms = 10.5 ms would clamp earlier than fast's
      // unfired 11 ms deadline.
      disarmed_early.setup_periodic(loop.monotonic_now() + 6500us, 6500us);
    }
    void on_fast(const Context&) {}
    void on_early(const Context&) {}
    Timer<Loop> fast;
    Timer<Loop> disarmed_early;
  };
  const std::string dir = TempDir("allow_capped_late_exit");
  const std::string out = TempDir("allow_capped_late_exit_out");
  const std::string path = dir + "/two_timers.tlog";
  {
    log::LogWriter writer{path, "two_timers",
                          log::LogWriterOptions{.background = false}};
    writer.start(ManifestOf<TwoTimers>(dir + "/unused.tlog"), kOrigin);
    // In segment 0: timer 1 ("early") fires once and disarms itself; timer 0
    // ("fast") fires at 1..10 ms.
    std::uint64_t dispatch = 0;
    const MonotonicTime first = kOrigin + kLateRun + 1ms;
    writer.dispatch(Context{.kind = EventKind::TIMER,
                            .source_id = 0,
                            .dispatch_index = ++dispatch,
                            .event_time = first,
                            .now = first,
                            .sequence = 1},
                    {});
    const Context early_ctx{.kind = EventKind::TIMER,
                            .source_id = 1,
                            .dispatch_index = ++dispatch,
                            .event_time = first,
                            .now = first,
                            .sequence = 1};
    writer.dispatch(early_ctx, {});
    writer.dispatch(Context{.kind = EventKind::DISARM_TIMER,
                            .source_id = 1,
                            .dispatch_index = dispatch,
                            .event_time = first,
                            .now = first},
                    {});
    for (int i = 2; i <= 10; ++i) {
      const MonotonicTime deadline =
          kOrigin + kLateRun + std::chrono::milliseconds{i};
      writer.dispatch(Context{.kind = EventKind::TIMER,
                              .source_id = 0,
                              .dispatch_index = ++dispatch,
                              .event_time = deadline,
                              .now = deadline,
                              .sequence = 1},
                      {});
    }
    // Late EXIT at 11.5 ms, after fast's unfired 11 ms deadline.
    const MonotonicTime late_exit = kOrigin + kLateRun + 11ms + 500us;
    writer.finish(Context{.kind = EventKind::EXIT,
                          .dispatch_index = ++dispatch,
                          .event_time = late_exit,
                          .now = late_exit});
    writer.flush();
  }
  RotateAt(path, {kOrigin + kLateRun + 5ms});
  std::filesystem::remove(log::LogSegmentPath(path, 0));
  CreateEmptyFile(log::CappedMarkerPath(path));

  const std::vector<std::int64_t> kept_firings = LogTimerFirings(path);
  ASSERT_EQ(kept_firings.size(), 6u);  // 5..10 ms of fast only
  const InputReplayOptions opts{.allow_capped = true};
  const RecordedSession session{dir, "two_timers", opts};
  // Clamped to 1 ns before fast's unfired 11 ms deadline (not clamped early at
  // 10.5 ms by unobserved disarmed_early, and not left unclamped at 11.5 ms).
  EXPECT_EQ(session.loop_end(), kOrigin + kLateRun + 11ms - Duration{1});

  SimulationEnvironment env;
  Loop loop{env, log::LogWriter{out + "/two_timers.tlog", "two_timers"},
            Loop::Options{.start = session.loop_start(),
                          .run_start = session.run_start()}};
  TwoTimers prog{loop};
  ::testing::internal::CaptureStderr();
  const InputReplayReport report = ReplayInputs(loop, env, session, opts);
  static_cast<void>(::testing::internal::GetCapturedStderr());
  EXPECT_FALSE(report.stopped_at_gap.has_value());
  // Replayed fast fires at 5..10 ms (no fabricated 11 ms firing).
  std::vector<std::int64_t> fast_firings;
  {
    log::LogReader reader{out + "/two_timers.tlog"};
    log::LogReader::Record record;
    while (reader.next(record)) {
      if (record.header.kind == static_cast<std::uint16_t>(EventKind::TIMER) &&
          record.header.source_id == 0) {
        fast_firings.push_back(record.header.event_time_ns);
      }
    }
  }
  EXPECT_EQ(fast_firings, kept_firings);
  std::filesystem::remove_all(dir);
  std::filesystem::remove_all(out);
}

// LOG-29(c): an unconsumed IncompleteLogHeader file or a manifest-listed
// missing/unexited/capped/failed writer run must prevent writer_complete from
// falsely rescuing a capped or gapped reader. REQ-OPS-14
TEST(InputReplay, MissingOrIncompleteRestartedWriterDoesNotRescueReader) {
  {
    SCOPED_TRACE(
        "unconsumed IncompleteLogHeader prevents writer rescue (capped & "
        "gapped)");
    for (const bool gapped_reader : {false, true}) {
      SCOPED_TRACE(gapped_reader ? "gapped reader" : "capped reader");
      const std::string dir = TempDir("incomplete_restart_no_rescue");
      const std::string out = TempDir("incomplete_restart_no_rescue_out");
      WriterLog(dir);
      ReaderLog(dir);
      if (gapped_reader) {
        MarkGap(dir + "/reader.tlog", kDrain + 5us);
      } else {
        CreateEmptyFile(log::CappedMarkerPath(dir + "/reader.tlog"));
      }
      // Truncated-header restarted writer log: skipped on IncompleteLogHeader
      // with no later readable segment to consume it.
      CreateEmptyFile(dir + "/writer.restart1.tlog");

      ::testing::internal::CaptureStderr();
      const RecordedSession session{dir};
      static_cast<void>(::testing::internal::GetCapturedStderr());
      if (gapped_reader) {
        const WatcherRun run = ReplayWatcher(session, out);
        EXPECT_TRUE(run.report.stopped_at_gap.has_value());
      } else {
        EXPECT_THROW(ReplayWatcher(session, out), ReplayError);
      }
      std::filesystem::remove_all(dir);
      std::filesystem::remove_all(out);
    }
  }
  {
    SCOPED_TRACE(
        "manifest listing missing restarted writer log, log_capped, or "
        "recorder_failed/log_gap on missing log prevents writer rescue");
    enum class WriterManifestFault {
      kMissingRestartLog,
      kWriterLogCapped,
      kWriterRecorderFailedMissing,
      kWriterLogGapMissing,
    };
    for (const WriterManifestFault fault :
         {WriterManifestFault::kMissingRestartLog,
          WriterManifestFault::kWriterLogCapped,
          WriterManifestFault::kWriterRecorderFailedMissing,
          WriterManifestFault::kWriterLogGapMissing}) {
      const std::string dir = TempDir("manifest_writer_no_rescue");
      const std::string out = TempDir("manifest_writer_no_rescue_out");
      WriterLog(dir);
      ReaderLog(dir);
      const std::uint64_t id =
          log::LogReader{dir + "/writer.tlog"}.session_id();
      CreateEmptyFile(log::CappedMarkerPath(dir + "/reader.tlog"));
      std::ofstream manifest{dir + "/manifest.json"};
      switch (fault) {
        case WriterManifestFault::kMissingRestartLog:
          manifest << "{\"session_id\": " << id
                   << ", \"nodes\": [{\"name\": \"writer\", \"log_paths\": "
                      "[\"/s/writer.tlog\", \"/s/writer.restart1.tlog\"], "
                      "\"log_capped\": false, \"log_gap\": false, "
                      "\"gap_logs\": [], \"recorder_failed\": false, "
                      "\"recorder_failed_logs\": []}]}\n";
          break;
        case WriterManifestFault::kWriterLogCapped:
          manifest << "{\"session_id\": " << id
                   << ", \"nodes\": [{\"name\": \"writer\", \"log_paths\": "
                      "[\"/s/writer.tlog\"], \"log_capped\": true, "
                      "\"log_gap\": false, \"gap_logs\": [], "
                      "\"recorder_failed\": false, "
                      "\"recorder_failed_logs\": []}]}\n";
          break;
        case WriterManifestFault::kWriterRecorderFailedMissing:
          manifest << "{\"session_id\": " << id
                   << ", \"nodes\": [{\"name\": \"writer\", \"log_paths\": "
                      "[\"/s/writer.tlog\"], \"log_capped\": false, "
                      "\"log_gap\": false, \"gap_logs\": [], "
                      "\"recorder_failed\": true, \"recorder_failed_logs\": "
                      "[\"/s/writer.restart1.tlog\"]}]}\n";
          break;
        case WriterManifestFault::kWriterLogGapMissing:
          manifest << "{\"session_id\": " << id
                   << ", \"nodes\": [{\"name\": \"writer\", \"log_paths\": "
                      "[\"/s/writer.tlog\"], \"log_capped\": false, "
                      "\"log_gap\": true, \"gap_logs\": "
                      "[\"/s/writer.restart1.tlog\"], \"recorder_failed\": "
                      "false, \"recorder_failed_logs\": []}]}\n";
          break;
      }
      manifest.close();

      const RecordedSession session{dir};
      EXPECT_THROW(ReplayWatcher(session, out), ReplayError);
      std::filesystem::remove_all(dir);
      std::filesystem::remove_all(out);
    }
  }
}

// LOG-28: an inconsistent manifest with log_gap: true and gap_logs: [] still
// fails closed as a member-wide log_gap mark on an unexited log. REQ-OPS-14
TEST(InputReplay, InconsistentLogGapTrueWithEmptyGapLogsFailsClosed) {
  const std::string dir = TempDir("inconsistent_empty_gap_logs");
  const std::string out = TempDir("inconsistent_empty_gap_logs_out");
  const std::vector<std::uint64_t> woff = WriterLog(dir);
  const std::string wpath = dir + "/writer.tlog";
  const std::uint64_t id = log::LogReader{wpath}.session_id();
  // Drop EXIT so manifest log_gap refuses the log.
  std::filesystem::resize_file(wpath, woff.back());
  std::ofstream{dir + "/manifest.json"}
      << "{\"session_id\": " << id
      << ", \"nodes\": [{\"name\": \"writer\", \"log_paths\": "
         "[\"/s/writer.tlog\"], \"log_capped\": false, \"log_gap\": true, "
         "\"gap_logs\": [], \"recorder_failed\": false, "
         "\"recorder_failed_logs\": []}]}\n";

  const RecordedSession session{dir};
  try {
    ReplayWatcher(session, out);
    ADD_FAILURE() << "accepted unexited log with log_gap:true and gap_logs:[]";
  } catch (const ReplayError& e) {
    EXPECT_NE(std::string{e.what()}.find("log_gap"), std::string::npos)
        << e.what();
  }
  std::filesystem::remove_all(dir);
  std::filesystem::remove_all(out);
}

// LOG-29(a)/(b): a MESSAGE/SEND record with a mismatched payload size must not
// close an open damaged stretch. REQ-OPS-14
TEST(InputReplay, WrongPayloadSizeRecordDoesNotCloseOpenStretch) {
  const std::string dir = TempDir("wrong_payload_stretch");
  const std::string out = TempDir("wrong_payload_stretch_out");
  const std::vector<std::uint64_t> woff = WriterLog(dir);
  // Drop writer EXIT so reader's damaged stretch is judged directly (not
  // bypassed by writer_complete).
  std::filesystem::resize_file(dir + "/writer.tlog", woff.back());
  const std::string rpath = dir + "/reader.tlog";
  {
    log::LogWriter writer{rpath, "reader",
                          log::LogWriterOptions{.background = false}};
    writer.start(ManifestOf<SensorWatcher>(dir + "/unused.tlog"), kOrigin);
    for (int i = 1; i <= 2; ++i) {
      const EventsTest::SensorMessage reading{i, 1.0F, 1.0F};
      writer.dispatch(
          Context{.kind = EventKind::MESSAGE,
                  .source_id = 0,
                  .dispatch_index = static_cast<std::uint64_t>(i),
                  .event_time = kDrain,
                  .now = kDrain + std::chrono::microseconds{5 * i},
                  .publish_time = kOrigin + std::chrono::milliseconds{i},
                  .sequence = static_cast<std::uint64_t>(i - 1)},
          std::as_bytes(std::span{&reading, 1}));
    }
    // Receipt for sequence 2 with a 1-byte payload (wrong size for
    // SensorMessage): must be ignored and must NOT close the damaged stretch
    // on sequence 1.
    const std::byte bogus{0x42};
    writer.dispatch(Context{.kind = EventKind::MESSAGE,
                            .source_id = 0,
                            .dispatch_index = 3,
                            .event_time = kDrain,
                            .now = kDrain + 15us,
                            .publish_time = kOrigin + 3ms,
                            .sequence = 2},
                    std::span{&bogus, 1});
    writer.finish(Context{.kind = EventKind::EXIT,
                          .dispatch_index = 4,
                          .event_time = kDrain + 20us,
                          .now = kDrain + 20us});
    writer.flush();
  }
  const std::vector<std::uint64_t> roff = RecordOffsets(rpath);
  ASSERT_EQ(roff.size(), 4u);  // seq 0, seq 1, bogus seq 2, EXIT
  DamageRecord(rpath, roff[1]);

  ::testing::internal::CaptureStderr();
  const RecordedSession session{dir};
  static_cast<void>(::testing::internal::GetCapturedStderr());
  const WatcherRun run = ReplayWatcher(session, out);
  ASSERT_TRUE(run.report.stopped_at_gap.has_value());
  EXPECT_EQ(run.report.stopped_at_gap->damage_offset, roff[1]);
  std::filesystem::remove_all(dir);
  std::filesystem::remove_all(out);
}

}  // namespace
}  // namespace talos::event

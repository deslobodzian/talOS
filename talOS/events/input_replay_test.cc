// Input replay (input_replay.h): a program that was not in a recorded
// session, driven from the topics the session logged, computes what it would
// have computed had it been there -- and computes it the same way every time.

#include "talOS/events/input_replay.h"

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "events_test/events_test_message_generated.h"
#include "talOS/events/handles.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/gap_testing.h"
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
    // Both topics are fed from the observer's log: the first fed is named.
    EXPECT_EQ(stopped.topic, kTopics.sensor);
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
// its watcher four readings published 1 to 4 ms after the origin, each
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
                .sequence = static_cast<std::uint64_t>(i)},
        std::as_bytes(std::span{&reading, 1}));
  }
  writer.finish(Context{.kind = EventKind::EXIT,
                        .dispatch_index = 5,
                        .event_time = kDrain + 25us,
                        .now = kDrain + 25us});
  writer.flush();
  return RecordOffsets(path);
}

// A writer's log: a tick 1 to 4 ms after the origin, each sending a reading.
// Returns each record's offset (tick, send, ..., EXIT).
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
                static_cast<std::uint64_t>(i));
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

}  // namespace
}  // namespace talos::event

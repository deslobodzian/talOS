#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/events/recorder.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/events/time.h"
#include "talOS/schema/identity.h"

// Input replay: drive a program that was not in a recorded session (or a
// changed version of one that was) from the topics that session logged.
//
// ReplayEventLoop re-runs a program against its own log and fails at the
// first difference. This is the other question -- "what would this program
// have done in that match?" -- so nothing is compared: the program runs on a
// SimulatedEventLoop on the recorded clock, every recorded message on a topic
// it watches is published at its recorded publish time, and whatever it
// sends is its own new output. A program that was in the session (its logs
// carry its process name) runs on its own recorded timeline: its loop's
// origin and run start, so its timers fire at the recorded deadlines, and it
// is handed what it received live, at the drains it received it in.
namespace talos::event {

// A topic as a session's logs recorded it.
struct RecordedTopic {
  std::string name;
  std::uint32_t message_bytes{0};
  schema::Identity schema;
  // Whose log its messages come from: the replayed program's own when it
  // watched the topic live, else its writer's when the writer logged it,
  // otherwise the first reader's that did.
  std::string recorded_by;
  std::size_t message_count{0};
};

struct RecordedMessage {
  MonotonicTime time;  // producer's publish time, not a reader's receipt
  // When the replayed program is to see it: `time`, or for its own receipt
  // the deadline of the live drain that delivered it.
  MonotonicTime delivered;
  std::uint32_t topic{0};  // index into RecordedSession::topics()
  std::uint64_t sequence{0};
  std::size_t offset{0};  // of its payload in the session's byte store
};

// Every message one session's logs recorded, by topic.
//
// A topic the replayed program watched live is fed from its own logs'
// receipts alone: what it saw, when it saw it. Each is delivered at the
// drain that delivered it live (its MESSAGE record's event time), with the
// publish time the transport stamped. The replayed loop runs every drain at
// its deadline, without the live loop's wake-up latency, so a receipt
// published after its drain's deadline (during that latency) is published
// at that deadline.
//
// Every other topic is fed from all logs: a message's time is its producer's
// publish time, preserved in both SEND and MESSAGE records even when only a
// delayed reader logged it, and one message seen in several logs is kept
// once, by topic and transport sequence.
class RecordedSession {
 public:
  // `path` is a session directory (every *.tlog in it, restarts included)
  // or one log; `replayed` is the process name of the program to be
  // replayed, or empty for none. Throws std::runtime_error on an unreadable
  // log, on logs from two sessions, and on a topic the logs record with two
  // schemas. A log's damaged records are skipped (log::LogReader), and what
  // was skipped is printed to stderr as a warning naming the file and byte
  // offsets.
  explicit RecordedSession(const std::string& path,
                           std::string_view replayed = {});

  const std::vector<std::string>& log_paths() const { return log_paths_; }
  std::uint64_t session_id() const { return session_id_; }

  // The earliest log origin, and the latest time any log recorded.
  MonotonicTime start() const { return start_; }
  MonotonicTime end() const { return end_; }

  // Where the replayed program's loop starts (SimulatedEventLoop::Options):
  // its first recorded run's loop origin, and the instant that run's loop
  // began running, which its construction put after the origin (the first
  // timer firing's deadline minus its manifest offset; the origin when it
  // had no timer). For a program the session did not log, start() for both.
  // Where the replay ends: the latest time the program's own logs recorded
  // (its exit), or end().
  MonotonicTime loop_start() const { return loop_start_; }
  MonotonicTime run_start() const { return run_start_; }
  MonotonicTime loop_end() const { return loop_end_; }

  const std::vector<RecordedTopic>& topics() const { return topics_; }
  // The topic called `name`, or null when no log recorded it.
  const RecordedTopic* find(std::string_view name) const;

  // In feed order: delivered, then time, then topic name, then sequence.
  const std::vector<RecordedMessage>& messages() const { return messages_; }
  std::span<const std::byte> payload(const RecordedMessage& message) const {
    return {bytes_.data() + message.offset,
            topics_[message.topic].message_bytes};
  }

 private:
  std::vector<std::string> log_paths_;
  std::uint64_t session_id_{0};
  MonotonicTime start_;
  MonotonicTime end_;
  MonotonicTime loop_start_;
  MonotonicTime run_start_;
  MonotonicTime loop_end_;
  std::vector<RecordedTopic> topics_;
  std::vector<RecordedMessage> messages_;
  std::vector<std::byte> bytes_;
};

struct InputReplayReport {
  struct Fed {
    std::string topic;
    std::string recorded_by;
    std::size_t messages{0};
  };
  // Topics the program watches that the session recorded, in registration
  // order, with how many messages each was fed.
  std::vector<Fed> fed;
  // Topics the program watches that no log recorded: nothing arrives on
  // them, as if their writer had never run.
  std::vector<std::string> unrecorded;
};

// The topic `registration` (a watcher of the program) reads, matched against
// what the session recorded: null when nothing was recorded, the recorded
// topic when the schemas agree. Throws ReplayError naming both types when
// they do not.
const RecordedTopic* MatchRecordedTopic(const RecordedSession& session,
                                        const Registration& registration);

// Runs `loop` to session.loop_end(), publishing every recorded message on a
// topic the program watches with its recorded publish time, just before the
// first drain at or after its `delivered` time, which delivers it. A topic
// the program sends itself is its own output and is not fed. `loop` must
// have finished registration, not have run, and start at
// session.loop_start() with its run starting at session.run_start()
// (SimulatedEventLoop::Options::start, run_start). Finishes the loop, which
// closes its recorder's log. Throws ReplayError before running on a schema
// mismatch.
template <RecorderPolicy Recorder>
InputReplayReport ReplayInputs(SimulatedEventLoop<Recorder>& loop,
                               SimulationEnvironment& environment,
                               const RecordedSession& session) {
  if (loop.start_time() != session.loop_start()) {
    throw std::logic_error(
        "input replay needs a loop that starts at the recorded origin "
        "(RecordedSession::loop_start)");
  }

  InputReplayReport report;
  std::vector<SimulatedChannel*> channels(session.topics().size(), nullptr);
  std::vector<std::size_t> fed_index(session.topics().size(), 0);
  const Manifest& manifest = loop.manifest();
  const auto sends = [&](std::string_view topic) {
    for (const Registration& registration : manifest) {
      if (registration.kind == SourceKind::SENDER &&
          registration.name == topic) {
        return true;
      }
    }
    return false;
  };

  for (const Registration& registration : manifest) {
    if (registration.kind != SourceKind::WATCHER || sends(registration.name)) {
      continue;
    }
    const RecordedTopic* topic = MatchRecordedTopic(session, registration);
    if (topic == nullptr) {
      report.unrecorded.push_back(registration.name);
      continue;
    }
    const auto index =
        static_cast<std::size_t>(topic - session.topics().data());
    if (channels[index] != nullptr) continue;  // watched twice
    channels[index] = &environment.channel(
        registration.name, registration.message_bytes, registration.schema);
    fed_index[index] = report.fed.size();
    report.fed.push_back({topic->name, topic->recorded_by, 0});
  }

  std::optional<MonotonicTime> instant;
  for (const RecordedMessage& message : session.messages()) {
    if (message.delivered > session.loop_end()) break;
    SimulatedChannel* channel = channels[message.topic];
    if (channel == nullptr) continue;
    // Everything due before the message's delivery happens first; then the
    // clock stands at it, so the drain there, a timer's or a wake's, is the
    // first to see it. Every message of one instant goes out before that
    // drain runs.
    if (instant != message.delivered) {
      loop.run_until(message.delivered - Duration{1});
      if (loop.monotonic_now() < message.delivered) {
        loop.advance_until(message.delivered);
      }
      instant = message.delivered;
    }
    channel->publish(session.payload(message), message.time);
    ++report.fed[fed_index[message.topic]].messages;
  }
  loop.run_until(session.loop_end());
  loop.finish();
  return report;
}

}  // namespace talos::event

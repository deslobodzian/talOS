#pragma once

#include <cstddef>
#include <cstdint>
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
// SimulatedEventLoop whose clock starts at the session's origin, every
// recorded message on a topic it watches is published at its producer's
// recorded publish time, and whatever it sends is its own new output.
namespace talos::event {

// A topic as a session's logs recorded it.
struct RecordedTopic {
  std::string name;
  std::uint32_t message_bytes{0};
  schema::Identity schema;
  // Whose log its messages come from: its writer's when the writer logged
  // it, otherwise the first reader's that did.
  std::string recorded_by;
  std::size_t message_count{0};
};

struct RecordedMessage {
  MonotonicTime time;      // producer's publish time, not a reader's receipt
  std::uint32_t topic{0};  // index into RecordedSession::topics()
  std::uint64_t sequence{0};
  std::size_t offset{0};  // of its payload in the session's byte store
};

// Every message one session's logs recorded, by topic.
//
// A message's time is its producer's publish time, preserved in both SEND and
// MESSAGE records even when only a delayed reader logged it. One message seen
// in several logs is kept once, by topic and transport sequence.
class RecordedSession {
 public:
  // `path` is a session directory (every *.tlog in it, restarts included)
  // or one log. Throws std::runtime_error on an unreadable log, on logs from
  // two sessions, and on a topic the logs record with two schemas.
  explicit RecordedSession(const std::string& path);

  const std::vector<std::string>& log_paths() const { return log_paths_; }
  std::uint64_t session_id() const { return session_id_; }

  // The earliest log origin, and the latest time any log recorded.
  MonotonicTime start() const { return start_; }
  MonotonicTime end() const { return end_; }

  const std::vector<RecordedTopic>& topics() const { return topics_; }
  // The topic called `name`, or null when no log recorded it.
  const RecordedTopic* find(std::string_view name) const;

  // In feed order: time, then topic name, then sequence.
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

// Runs `loop` from the session's start to its end, publishing every recorded
// message on a topic the program watches at its producer's publish time, so
// the first drain at or after it delivers it with its original timestamp.
// A topic the program sends itself is its own output and is not fed. `loop`
// must have finished registration, not have run, and start at session.start()
// (SimulatedEventLoop::Options::start). Finishes the loop, which closes its
// recorder's log. Throws ReplayError before running on a schema mismatch.
template <RecorderPolicy Recorder>
InputReplayReport ReplayInputs(SimulatedEventLoop<Recorder>& loop,
                               SimulationEnvironment& environment,
                               const RecordedSession& session) {
  if (loop.start_time() != session.start()) {
    throw std::logic_error(
        "input replay needs a loop that starts at the session's origin");
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

  for (const RecordedMessage& message : session.messages()) {
    SimulatedChannel* channel = channels[message.topic];
    if (channel == nullptr) continue;
    // Everything due before the message happens first; the drain at its
    // time or after it delivers it.
    loop.run_until(message.time - Duration{1});
    channel->publish(session.payload(message), message.time);
    ++report.fed[fed_index[message.topic]].messages;
  }
  loop.run_until(session.end());
  loop.finish();
  return report;
}

}  // namespace talos::event

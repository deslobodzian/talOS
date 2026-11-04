#include "talOS/events/input_replay.h"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"

namespace talos::event {

namespace {

std::string Shape(const schema::Identity& schema, std::uint32_t bytes) {
  return schema::Describe(schema) + ", " + std::to_string(bytes) + " bytes";
}

}  // namespace

RecordedSession::RecordedSession(const std::string& path)
    : log_paths_{log::FindLogFiles(path)} {
  if (log_paths_.empty()) {
    throw std::runtime_error("no .tlog files in " + path);
  }

  std::map<std::string, std::uint32_t, std::less<>> topic_index;
  // (topic, sequence) -> index in messages_.
  std::map<std::pair<std::uint32_t, std::uint64_t>, std::size_t> seen;
  // Whether a topic's recorded_by is its writer, which a reader never
  // replaces.
  std::vector<bool> by_writer;
  bool first = true;

  for (const std::string& log_path : log_paths_) {
    log::LogReader reader{log_path};
    if (first) {
      session_id_ = reader.session_id();
      start_ = reader.start_time();
      end_ = reader.start_time();
      first = false;
    } else if (reader.session_id() != session_id_) {
      throw std::runtime_error(log_path + " is from session " +
                               std::to_string(reader.session_id()) + ", not " +
                               std::to_string(session_id_));
    }
    start_ = std::min(start_, reader.start_time());
    end_ = std::max(end_, reader.start_time());

    // This log's topic sources, by source id: where their messages go.
    constexpr std::uint32_t kNone = 0xFFFFFFFF;
    std::vector<std::uint32_t> topic_of(reader.manifest().size(), kNone);
    for (const Registration& registration : reader.manifest()) {
      if (registration.kind != SourceKind::SENDER &&
          registration.kind != SourceKind::WATCHER) {
        continue;
      }
      const bool writer = registration.kind == SourceKind::SENDER;
      auto [it, inserted] = topic_index.try_emplace(
          registration.name, static_cast<std::uint32_t>(topics_.size()));
      if (inserted) {
        topics_.push_back({registration.name, registration.message_bytes,
                           registration.schema, reader.process_name(), 0});
        by_writer.push_back(writer);
      } else {
        RecordedTopic& topic = topics_[it->second];
        if (!(topic.schema == registration.schema) ||
            topic.message_bytes != registration.message_bytes) {
          throw std::runtime_error(
              "the session records topic '" + registration.name + "' as " +
              Shape(topic.schema, topic.message_bytes) + " (" +
              topic.recorded_by + ") and as " +
              Shape(registration.schema, registration.message_bytes) + " (" +
              reader.process_name() + ")");
        }
        if (writer && !by_writer[it->second]) {
          topic.recorded_by = reader.process_name();
          by_writer[it->second] = true;
        }
      }
      if (registration.id < topic_of.size()) {
        topic_of[registration.id] = it->second;
      }
    }

    log::LogReader::Record record;
    while (reader.next(record)) {
      const log::RecordHeader& header = record.header;
      end_ = std::max(end_, MonotonicTime::from_nanos(header.now_ns));
      const auto kind = static_cast<EventKind>(header.kind);
      // A watcher's receipt, or a writer's accepted send.
      const bool receipt = kind == EventKind::MESSAGE;
      const bool send = kind == EventKind::SEND && header.aux == 0;
      if ((!receipt && !send) || header.source_id >= topic_of.size() ||
          topic_of[header.source_id] == kNone) {
        continue;
      }
      const std::uint32_t topic = topic_of[header.source_id];
      if (record.payload.size() != topics_[topic].message_bytes) continue;
      const MonotonicTime time = MonotonicTime::from_nanos(header.now_ns);
      const auto [it, inserted] =
          seen.try_emplace({topic, header.sequence}, messages_.size());
      if (!inserted) {
        RecordedMessage& known = messages_[it->second];
        if (time < known.time) {
          known.time = time;
          std::copy(record.payload.begin(), record.payload.end(),
                    bytes_.begin() + static_cast<std::ptrdiff_t>(known.offset));
        }
        continue;
      }
      messages_.push_back({time, topic, header.sequence, bytes_.size()});
      bytes_.insert(bytes_.end(), record.payload.begin(), record.payload.end());
      ++topics_[topic].message_count;
    }
  }

  std::sort(messages_.begin(), messages_.end(),
            [this](const RecordedMessage& a, const RecordedMessage& b) {
              return std::tie(a.time, topics_[a.topic].name, a.sequence) <
                     std::tie(b.time, topics_[b.topic].name, b.sequence);
            });
}

const RecordedTopic* RecordedSession::find(std::string_view name) const {
  for (const RecordedTopic& topic : topics_) {
    if (topic.name == name) return &topic;
  }
  return nullptr;
}

const RecordedTopic* MatchRecordedTopic(const RecordedSession& session,
                                        const Registration& registration) {
  const RecordedTopic* topic = session.find(registration.name);
  if (topic == nullptr) return nullptr;
  if (!(topic->schema == registration.schema) ||
      topic->message_bytes != registration.message_bytes) {
    throw ReplayError("topic '" + registration.name + "' was recorded as " +
                      Shape(topic->schema, topic->message_bytes) + " by " +
                      topic->recorded_by + "; this program reads it as " +
                      Shape(registration.schema, registration.message_bytes));
  }
  return topic;
}

}  // namespace talos::event

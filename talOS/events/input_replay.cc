#include "talOS/events/input_replay.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <optional>
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

RecordedSession::RecordedSession(const std::string& path,
                                 std::string_view replayed)
    : log_paths_{log::FindLogFiles(path)} {
  if (log_paths_.empty()) {
    throw std::runtime_error("no .tlog files in " + path);
  }

  std::map<std::string, std::uint32_t, std::less<>> topic_index;
  // (topic, sequence) -> index in messages_, every log's.
  std::map<std::pair<std::uint32_t, std::uint64_t>, std::size_t> seen;
  // The same for the replayed program's own receipts, kept apart in `own`.
  std::map<std::pair<std::uint32_t, std::uint64_t>, std::size_t> own_seen;
  std::vector<RecordedMessage> own;
  // Whether a topic's recorded_by is its writer, which a reader never
  // replaces.
  std::vector<bool> by_writer;
  // Whether the replayed program watched the topic live: fed from `own` alone.
  std::vector<bool> watched_by_replayed;
  // The replayed program's first recorded run: its log's origin and run start.
  std::optional<std::pair<MonotonicTime, MonotonicTime>> replayed_run;
  // The latest time any of its logs recorded: its last run's exit.
  std::optional<MonotonicTime> replayed_end;
  bool first = true;

  for (const std::string& log_path : log_paths_) {
    std::optional<log::LogReader> opened;
    try {
      opened.emplace(log_path);
    } catch (const log::IncompleteLogHeader& e) {
      // A log cut while it was being created holds no records.
      std::cerr << "warning: skipping " << log_path << ": " << e.what() << "\n";
      continue;
    }
    log::LogReader& reader = *opened;
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
    // A log of the program being replayed: what it saw, and when it ran.
    const bool own_log = !replayed.empty() && reader.process_name() == replayed;

    // This log's topic sources, by source id: where their messages go.
    constexpr std::uint32_t kNone = 0xFFFFFFFF;
    std::vector<std::uint32_t> topic_of(reader.manifest().size(), kNone);
    // The manifest's armed timers' offsets from the origin, until a record
    // re-arms or disarms one, by source id: a timer's first firing at its
    // manifest deadline tells how far the live loop's run() began after
    // its origin (RealtimeEventLoop shifts its schedule by that).
    std::vector<std::optional<std::int64_t>> timer_offset(
        reader.manifest().size());
    std::optional<MonotonicTime> run_start;
    for (const Registration& registration : reader.manifest()) {
      if (registration.kind == SourceKind::TIMER && registration.armed &&
          registration.id < timer_offset.size()) {
        timer_offset[registration.id] = registration.offset_ns;
      }
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
        watched_by_replayed.push_back(false);
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
      if (own_log && !writer) watched_by_replayed[it->second] = true;
      if (registration.id < topic_of.size()) {
        topic_of[registration.id] = it->second;
      }
    }

    log::LogReader::Record record;
    while (reader.next(record)) {
      const log::RecordHeader& header = record.header;
      end_ = std::max(end_, MonotonicTime::from_nanos(header.now_ns));
      if (own_log) {
        replayed_end = std::max(replayed_end.value_or(reader.start_time()),
                                MonotonicTime::from_nanos(header.now_ns));
      }
      const auto kind = static_cast<EventKind>(header.kind);
      if (own_log && !run_start && header.source_id < timer_offset.size()) {
        if (kind == EventKind::ARM_TIMER || kind == EventKind::DISARM_TIMER) {
          timer_offset[header.source_id].reset();
        } else if (kind == EventKind::TIMER && timer_offset[header.source_id]) {
          run_start = MonotonicTime::from_nanos(
              header.event_time_ns - *timer_offset[header.source_id]);
        }
      }
      // A watcher's receipt, or a writer's accepted send.
      const bool receipt = kind == EventKind::MESSAGE;
      const bool send = kind == EventKind::SEND && header.aux == 0;
      if ((!receipt && !send) || header.source_id >= topic_of.size() ||
          topic_of[header.source_id] == kNone) {
        continue;
      }
      const std::uint32_t topic = topic_of[header.source_id];
      if (record.payload.size() != topics_[topic].message_bytes) continue;
      if (own_log && receipt) {
        // Delivered by the drain at its event time; published no later, as
        // the replayed drain runs at its deadline.
        const MonotonicTime delivered =
            MonotonicTime::from_nanos(header.event_time_ns);
        const MonotonicTime time = std::min(
            delivered, MonotonicTime::from_nanos(header.publish_time_ns));
        // A topic watched twice receives each message twice: the earlier
        // delivery is when the program first saw it.
        const auto [it, inserted] =
            own_seen.try_emplace({topic, header.sequence}, own.size());
        if (!inserted) {
          RecordedMessage& known = own[it->second];
          if (delivered < known.delivered) {
            known.delivered = delivered;
            known.time = time;
          }
          continue;
        }
        own.push_back({time, delivered, topic, header.sequence, bytes_.size()});
        bytes_.insert(bytes_.end(), record.payload.begin(),
                      record.payload.end());
        continue;
      }
      const MonotonicTime time =
          MonotonicTime::from_nanos(header.publish_time_ns);
      const auto [it, inserted] =
          seen.try_emplace({topic, header.sequence}, messages_.size());
      if (!inserted) {
        RecordedMessage& known = messages_[it->second];
        if (time < known.time) {
          known.time = time;
          known.delivered = time;
          std::copy(record.payload.begin(), record.payload.end(),
                    bytes_.begin() + static_cast<std::ptrdiff_t>(known.offset));
        }
        continue;
      }
      messages_.push_back({time, time, topic, header.sequence, bytes_.size()});
      bytes_.insert(bytes_.end(), record.payload.begin(), record.payload.end());
    }
    // The intact records still replay; say what was skipped.
    if (const std::string damage = log::DescribeDamage(reader.damage());
        !damage.empty()) {
      std::cerr << "warning: damaged log " << log_path << ": " << damage
                << "; replaying the intact records\n";
    }
    // Its first run: the earliest origin among its logs (restarts included).
    if (own_log &&
        (!replayed_run || reader.start_time() < replayed_run->first)) {
      replayed_run.emplace(reader.start_time(),
                           std::max(reader.start_time(),
                                    run_start.value_or(reader.start_time())));
    }
  }
  if (first) throw std::runtime_error("no readable .tlog files in " + path);

  // A topic the replayed program watched is what it received, and nothing
  // another log adds.
  std::erase_if(messages_, [&](const RecordedMessage& message) {
    return watched_by_replayed[message.topic];
  });
  messages_.insert(messages_.end(), own.begin(), own.end());
  for (std::size_t topic = 0; topic < topics_.size(); ++topic) {
    if (watched_by_replayed[topic]) topics_[topic].recorded_by = replayed;
  }
  for (const RecordedMessage& message : messages_) {
    ++topics_[message.topic].message_count;
  }
  loop_start_ = replayed_run ? replayed_run->first : start_;
  run_start_ = replayed_run ? replayed_run->second : start_;
  loop_end_ = replayed_end.value_or(end_);

  std::sort(messages_.begin(), messages_.end(),
            [this](const RecordedMessage& a, const RecordedMessage& b) {
              return std::tie(a.delivered, a.time, topics_[a.topic].name,
                              a.sequence) < std::tie(b.delivered, b.time,
                                                     topics_[b.topic].name,
                                                     b.sequence);
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

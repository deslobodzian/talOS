#include "talOS/events/input_replay.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"

namespace talos::event {

namespace {

std::string Shape(const schema::Identity& schema, std::uint32_t bytes) {
  return schema::Describe(schema) + ", " + std::to_string(bytes) + " bytes";
}

// The log a file of it belongs to: `<stem>.tlog` for a segment the byte cap
// rotated out (`<stem>.segment<k>.tlog`, log::LogSegmentPath), else the file.
std::string LogOfFile(const std::string& file) {
  constexpr std::string_view kExtension = ".tlog";
  constexpr std::string_view kSegment = ".segment";
  std::string_view stem = file;
  if (!stem.ends_with(kExtension)) return file;
  stem.remove_suffix(kExtension.size());
  std::size_t digits = 0;
  while (digits < stem.size() && std::isdigit(static_cast<unsigned char>(
                                     stem[stem.size() - 1 - digits])) != 0) {
    ++digits;
  }
  if (digits == 0) return file;
  stem.remove_suffix(digits);
  if (!stem.ends_with(kSegment)) return file;
  stem.remove_suffix(kSegment.size());
  return std::string{stem} + std::string{kExtension};
}

// What the session manifest says a member's logs lost.
struct ManifestMarks {
  bool log_gap{false};
  bool log_capped{false};
  bool recorder_failed{false};
};

// By file name, the logs of every member the session manifest beside the
// logs (`<directory>/manifest.json`, talOS/launcher/session.cc) marks
// `log_gap`, `log_capped` or `recorder_failed`, with those marks; empty when
// there is no manifest, or it is another session's or has no session_id.
// Throws std::runtime_error naming a manifest that is not JSON, or whose
// `log_gap`, `log_capped`, `recorder_failed` or `log_paths` is not what the
// launcher writes.
std::map<std::string, ManifestMarks> MarkedLogNames(
    const std::filesystem::path& directory, std::uint64_t session_id) {
  const std::filesystem::path path = directory / "manifest.json";
  std::ifstream stream{path};
  if (!stream) return {};
  std::map<std::string, ManifestMarks> names;
  try {
    const nlohmann::json manifest = nlohmann::json::parse(stream);
    if (!manifest.is_object()) return {};
    const auto id = manifest.find("session_id");
    if (id == manifest.end() || !id->is_number_unsigned() ||
        id->get<std::uint64_t>() != session_id) {
      return {};
    }
    const auto nodes = manifest.find("nodes");
    if (nodes == manifest.end()) return {};
    for (const nlohmann::json& node :
         nodes->get_ref<const nlohmann::json::array_t&>()) {
      const ManifestMarks marks{
          .log_gap = node.value("log_gap", false),
          .log_capped = node.value("log_capped", false),
          .recorder_failed = node.value("recorder_failed", false)};
      if (!marks.log_gap && !marks.log_capped && !marks.recorder_failed) {
        continue;
      }
      for (const nlohmann::json& log :
           node.at("log_paths").get_ref<const nlohmann::json::array_t&>()) {
        names[std::filesystem::path{log.get<std::string>()}
                  .filename()
                  .string()] = marks;
      }
    }
  } catch (const nlohmann::json::exception& e) {
    throw std::runtime_error("cannot read the session manifest " +
                             path.string() + ": " + e.what());
  }
  return names;
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
  // Each log read: the earliest time a GAP record of it names; its first
  // damaged stretch's offset, and for each topic it recorded the earliest
  // time a record lost there could be fed at (none where nothing bounds it);
  // whether it holds its EXIT; and the topics it recorded (with whether the
  // replayed program watched them in it).
  struct ReadLog {
    std::string path;
    std::optional<MonotonicTime> gap;
    std::optional<std::uint64_t> damage_offset;
    std::map<std::uint32_t, std::optional<MonotonicTime>> damage_bound;
    bool exit{false};
    std::vector<std::pair<std::uint32_t, bool>> topics;
  };
  std::vector<ReadLog> logs;
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
    // The topics this log recorded, and whether the replayed program
    // watched each in it.
    std::vector<std::pair<std::uint32_t, bool>> recorded;
    // The earliest time a GAP record of this log names (gap), and the
    // earliest loss of its timeline (gap_start: that, or the now_ns of the
    // last intact record before a damaged stretch), which a run start taken
    // from a timer firing after it must not precede.
    std::optional<MonotonicTime> gap;
    std::optional<MonotonicTime> gap_start;
    // The now_ns of the last intact record, and by source id the
    // publication time of a watcher's last intact receipt: what bounds the
    // records a damaged stretch after them lost.
    std::optional<std::int64_t> last_now;
    std::vector<std::optional<std::int64_t>> last_receipt(
        reader.manifest().size());
    ReadLog read{.path = log_path};
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
      recorded.emplace_back(it->second, own_log && !writer);
    }

    log::LogReader::Record record;
    while (reader.next(record)) {
      const log::RecordHeader& header = record.header;
      // The reader skipped a damaged stretch just before this record: the
      // first one bounds what each topic's records lost there can be fed
      // at (a later one bounds them no earlier).
      if (reader.damage().spans != 0 && !read.damage_offset.has_value()) {
        read.damage_offset = reader.damage().first_offset;
        // Every record's now_ns is its dispatch's sample of the loop clock,
        // which never decreases along the log, except a send made before
        // the first dispatch, whose now_ns is 0.
        const MonotonicTime since_last =
            last_now
                ? MonotonicTime::from_nanos(*last_now)
                : std::min(MonotonicTime::from_nanos(0), reader.start_time());
        gap_start = std::min(gap_start.value_or(since_last), since_last);
        for (const Registration& source : reader.manifest()) {
          if (source.id >= topic_of.size() || topic_of[source.id] == kNone) {
            continue;
          }
          // A writer's send is fed at its publication time, which is its
          // record's now_ns (LogWriter::send). A reader's receipt is fed at
          // its publication time, and a watcher receives its topic in
          // sequence order, whose publication times never decrease (one
          // writer per topic, stamped from the monotonic clock); before its
          // first receipt nothing bounds it. The replayed program's own
          // receipt is fed at its drain's event time (a tick's deadline, or
          // a prompt watcher's publication time), which need not follow the
          // log's order: nothing bounds it.
          std::optional<MonotonicTime> bound;
          if (source.kind == SourceKind::SENDER) {
            bound = since_last;
          } else if (!own_log && last_receipt[source.id].has_value()) {
            bound = MonotonicTime::from_nanos(*last_receipt[source.id]);
          }
          const std::uint32_t topic = topic_of[source.id];
          const auto [it, inserted] =
              read.damage_bound.try_emplace(topic, bound);
          if (!inserted) {
            it->second = it->second && bound
                             ? std::optional{std::min(*it->second, *bound)}
                             : std::nullopt;
          }
        }
      }
      last_now = header.now_ns;
      end_ = std::max(end_, MonotonicTime::from_nanos(header.now_ns));
      if (own_log) {
        replayed_end = std::max(replayed_end.value_or(reader.start_time()),
                                MonotonicTime::from_nanos(header.now_ns));
      }
      if (const auto marked = log::GapOf(header, record.payload)) {
        const MonotonicTime start = MonotonicTime::from_nanos(marked->start_ns);
        gap = std::min(gap.value_or(start), start);
        gap_start = std::min(gap_start.value_or(start), start);
        continue;
      }
      const auto kind = static_cast<EventKind>(header.kind);
      if (kind == EventKind::EXIT) read.exit = true;
      if (kind == EventKind::MESSAGE &&
          header.source_id < last_receipt.size()) {
        last_receipt[header.source_id] = header.publish_time_ns;
      }
      // The first firing of a timer the run had not re-armed is at the run
      // start plus its manifest offset. A firing after a gap is one the live
      // loop made, so it fixes that timer's phase, but not the run start:
      // the gap may have dropped earlier firings (the start is then whole
      // periods earlier) or a re-arm. It is still a sound start before the
      // gap: every live firing before the gap was recorded and none was, and
      // the start it gives (never before the gap less the offset) fires no
      // replayed timer before the gap either, where a run fed from this log
      // stops. A run nothing stops at the gap replays that timer at its live
      // times from this firing on, and the others that many periods late.
      if (own_log && !run_start && header.source_id < timer_offset.size()) {
        const std::optional<std::int64_t>& offset =
            timer_offset[header.source_id];
        if (kind == EventKind::ARM_TIMER || kind == EventKind::DISARM_TIMER) {
          timer_offset[header.source_id].reset();
        } else if (kind == EventKind::TIMER && offset.has_value()) {
          std::int64_t start_ns = header.event_time_ns - *offset;
          if (gap_start.has_value()) {
            start_ns = std::max(start_ns, gap_start->nanos() - *offset);
          }
          run_start = MonotonicTime::from_nanos(start_ns);
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
    // Say what was skipped: a damaged tail only shortens the log, but a run
    // fed from it stops at a damaged stretch.
    if (const std::string damage = log::DescribeDamage(reader.damage());
        !damage.empty()) {
      std::cerr << "warning: damaged log " << log_path << ": " << damage
                << (reader.damage().spans != 0
                        ? "; a replay fed from it stops before the first "
                          "damaged stretch, or is refused\n"
                        : "; its intact records replay\n");
    }
    read.gap = gap;
    read.topics = std::move(recorded);
    logs.push_back(std::move(read));
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
  // Logs that lost records nothing in them marks: a capped one (its marker
  // beside the log a segment belongs to, or the session manifest's
  // log_capped for its member), and a run's log without its EXIT of a member
  // the manifest marks log_gap or recorder_failed. A writer that reached its
  // EXIT wrote a GAP record for every gap before it (LogWriter::gaps), so
  // such a log is judged by its GAP records; one without it may have lost a
  // gap's mark to its process's end, or its tail to a disk that failed (a
  // failed recorder writes no GAP record). A run's EXIT is in its newest
  // file, the log's path.
  const std::filesystem::path directory =
      std::filesystem::path{log_paths_.front()}.parent_path();
  const std::map<std::string, ManifestMarks> marked = MarkedLogNames(
      directory.empty() ? std::filesystem::path{"."} : directory, session_id_);
  std::set<std::string> exited;
  for (const ReadLog& log : logs) {
    if (log.exit) exited.insert(log.path);
  }
  const auto refusal = [&](const std::string& file) -> std::string {
    const std::string log = LogOfFile(file);
    if (std::string marker = log::CappedMarkerPath(log);
        std::filesystem::exists(marker)) {
      return marker + " exists: no GAP record marks what the byte cap cost it";
    }
    const auto marks =
        marked.find(std::filesystem::path{log}.filename().string());
    if (marks == marked.end()) return {};
    if (marks->second.log_capped) {
      return "the session manifest marks its member log_capped: no GAP "
             "record marks what the byte cap cost it";
    }
    if (marks->second.log_gap && !exited.contains(log)) {
      return "the session manifest marks its member log_gap, and its run's "
             "log has no EXIT record: a gap its process's end cut short has "
             "no GAP record";
    }
    if (marks->second.recorder_failed && !exited.contains(log)) {
      return "the session manifest marks its member recorder_failed, and its "
             "run's log has no EXIT record: a recorder that failed (a disk "
             "error) ends capture with no GAP record";
    }
    return {};
  };
  // Where each topic's feed ends, and whether it is refused: the earliest
  // loss, and the first refused log, among the logs it is fed from, which
  // for a topic the program watched live are its own logs that watched it,
  // and otherwise every log that recorded it.
  for (const ReadLog& log : logs) {
    const std::string sign = refusal(log.path);
    for (const auto& [topic, watched_here] : log.topics) {
      if (watched_by_replayed[topic] && !watched_here) continue;
      RecordedTopic& fed = topics_[topic];
      const auto loss = [&](MonotonicTime at,
                            std::optional<std::uint64_t> damage_offset) {
        if (at < fed.first_gap.value_or(MonotonicTime::max())) {
          fed.first_gap = at;
          fed.first_gap_log = log.path;
          fed.first_gap_damage_offset = damage_offset;
        }
      };
      if (log.gap.has_value()) loss(*log.gap, std::nullopt);
      std::string reason = sign;
      if (log.damage_offset.has_value()) {
        const auto bound = log.damage_bound.find(topic);
        if (bound != log.damage_bound.end() && bound->second.has_value()) {
          loss(*bound->second, log.damage_offset);
        } else if (reason.empty()) {
          reason = "records are lost between intact ones at byte " +
                   std::to_string(*log.damage_offset) + ", and nothing " +
                   "bounds when they were to be fed (" +
                   (watched_here ? "the program's own receipts, fed at their "
                                   "drains' times"
                                 : "no intact receipt of the topic precedes "
                                   "them") +
                   ")";
        }
      }
      if (!reason.empty() && fed.refused_log.empty()) {
        fed.refused_log = log.path;
        fed.refused_reason = reason;
      }
    }
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

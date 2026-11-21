#include "talOS/events/input_replay.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <tuple>
#include <utility>

#include "talOS/events/context.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/session_manifest.h"

namespace talos::event {

namespace {

std::string Shape(const schema::Identity& schema, std::uint32_t bytes) {
  return schema::Describe(schema) + ", " + std::to_string(bytes) + " bytes";
}

// The log a file of it belongs to, and the file's place in it: `<stem>.tlog`
// and k for a segment the byte cap rotated out (`<stem>.segment<k>.tlog`,
// log::LogSegmentPath), else the file and a place after every segment.
std::pair<std::string, std::uint64_t> LogAndPlace(const std::string& file) {
  constexpr std::uint64_t kLast = std::numeric_limits<std::uint64_t>::max();
  constexpr std::string_view kExtension = ".tlog";
  constexpr std::string_view kSegment = ".segment";
  std::string_view stem = file;
  if (!stem.ends_with(kExtension)) return {file, kLast};
  stem.remove_suffix(kExtension.size());
  std::size_t digits = 0;
  while (digits < stem.size() && std::isdigit(static_cast<unsigned char>(
                                     stem[stem.size() - 1 - digits])) != 0) {
    ++digits;
  }
  if (digits == 0) return {file, kLast};
  std::uint64_t k = 0;
  const char* const end = stem.data() + stem.size();
  if (const auto [at, error] = std::from_chars(end - digits, end, k);
      error != std::errc{} || at != end) {
    return {file, kLast};
  }
  stem.remove_suffix(digits);
  if (!stem.ends_with(kSegment)) return {file, kLast};
  stem.remove_suffix(kSegment.size());
  return {std::string{stem} + std::string{kExtension}, k};
}

// `a + b`, saturated to [int64_t::min, int64_t::max] on signed overflow.
std::int64_t SaturatingAdd(std::int64_t a, std::int64_t b) {
  constexpr std::int64_t kMax = std::numeric_limits<std::int64_t>::max();
  constexpr std::int64_t kMin = std::numeric_limits<std::int64_t>::min();
  if (b > 0 && a > kMax - b) return kMax;
  if (b < 0 && a < kMin - b) return kMin;
  return a + b;
}

// `a - b`, saturated to [int64_t::min, int64_t::max] on signed overflow.
std::int64_t SaturatingSub(std::int64_t a, std::int64_t b) {
  constexpr std::int64_t kMax = std::numeric_limits<std::int64_t>::max();
  constexpr std::int64_t kMin = std::numeric_limits<std::int64_t>::min();
  if (b < 0 && a > kMax + b) return kMax;
  if (b > 0 && a < kMin + b) return kMin;
  return a - b;
}

// `deadline_ns + period_ns * max(cycles, 1)`, saturated to int64_t::max on
// overflow.
std::int64_t AdvanceDeadline(std::int64_t deadline_ns, std::int64_t period_ns,
                             std::uint64_t cycles) {
  const std::uint64_t steps = std::max<std::uint64_t>(cycles, 1);
  if (period_ns <= 0) return deadline_ns;
  const std::int64_t max_ns = std::numeric_limits<std::int64_t>::max();
  const std::int64_t base = std::max<std::int64_t>(deadline_ns, 0);
  const auto max_steps =
      static_cast<std::uint64_t>((max_ns - base) / period_ns);
  if (steps > max_steps) return max_ns;
  return deadline_ns + period_ns * static_cast<std::int64_t>(steps);
}

// The log a file of it belongs to (LogAndPlace).
std::string LogOfFile(const std::string& file) {
  return LogAndPlace(file).first;
}

// The files RecordedSession reads for `path`: every *.tlog of a directory
// (log::FindLogFiles), or every kept file of the log a file belongs to
// (log::LogSegmentFiles, oldest segment first): a log the byte cap rotated
// is all of them, whichever is named, as a directory reads it. A file of no
// such log (missing, say) is read alone, so its reader names it. Each log's
// files are read together, oldest segment first (FindLogFiles' order by name
// puts segment10 before segment2), so what one segment carries to the next
// is read before it.
std::vector<std::string> SessionFiles(const std::string& path) {
  std::error_code ec;
  if (std::filesystem::is_directory(path, ec)) {
    std::vector<std::string> files = log::FindLogFiles(path);
    std::ranges::stable_sort(files, {}, LogAndPlace);
    return files;
  }
  // LogSegmentFiles names a segment by its directory, "." for a bare name,
  // joined with one separator.
  std::filesystem::path file = std::filesystem::path{path}.lexically_normal();
  if (!file.has_parent_path()) file = std::filesystem::path{"."} / file;
  std::vector<std::string> files =
      log::LogSegmentFiles(LogOfFile(file.string()));
  if (std::find(files.begin(), files.end(), file.string()) == files.end()) {
    return {path};
  }
  return files;
}

// What the session manifest says a member's logs lost.
struct ManifestMarks {
  bool log_gap{false};
  bool log_capped{false};
  bool recorder_failed{false};
};

// By file name, the logs of every member the session manifest beside the
// logs (`<directory>/manifest.json`, talOS/launcher/session.cc, read by
// session_manifest::Parse) marks `log_gap`, `log_capped` or
// `recorder_failed`, with those marks; empty when there is no manifest, or
// it is another session's or has no session_id. When a member carries per-run
// `gap_logs` or `recorder_failed_logs` arrays, only the basenames listed in
// each array receive that mark (legacy manifests without those arrays fall
// back to marking every basename in `log_paths` when the member's boolean is
// true). Throws std::runtime_error naming a manifest that is not JSON, or
// whose `log_gap`, `log_capped`, `recorder_failed` or required `log_paths` is
// not what the launcher writes.
std::map<std::string, ManifestMarks> MarkedLogNames(
    const std::filesystem::path& directory, std::uint64_t session_id) {
  const std::optional<std::string> text = session_manifest::ReadText(directory);
  if (!text) return {};
  const auto error = [&](std::string_view what) {
    return std::runtime_error(
        "cannot read the session manifest " +
        (directory / session_manifest::kFileName).string() + ": " +
        std::string{what});
  };
  session_manifest::Fields manifest;
  try {
    manifest = session_manifest::Parse(*text);
  } catch (const session_manifest::ManifestError& e) {
    throw error(e.what());
  }
  if (manifest.session_id != session_id) return {};
  std::map<std::string, ManifestMarks> names;
  for (const session_manifest::Member& node : manifest.nodes) {
    const bool legacy_gap = node.log_gap && !node.gap_logs.has_value();
    const bool legacy_failed =
        node.recorder_failed && !node.recorder_failed_logs.has_value();
    const bool needs_log_paths = node.log_capped || legacy_gap || legacy_failed;
    if (needs_log_paths) {
      if (!node.log_paths) {
        throw error("a marked member's log_paths is not an array of strings");
      }
      for (const std::string& log : *node.log_paths) {
        ManifestMarks& marks =
            names[std::filesystem::path{log}.filename().string()];
        if (legacy_gap) marks.log_gap = true;
        if (node.log_capped) marks.log_capped = true;
        if (legacy_failed) marks.recorder_failed = true;
      }
    }
    if (node.gap_logs) {
      for (const std::string& log : *node.gap_logs) {
        names[std::filesystem::path{log}.filename().string()].log_gap = true;
      }
    }
    if (node.recorder_failed_logs) {
      for (const std::string& log : *node.recorder_failed_logs) {
        names[std::filesystem::path{log}.filename().string()].recorder_failed =
            true;
      }
    }
  }
  return names;
}

}  // namespace

RecordedSession::RecordedSession(const std::string& path,
                                 std::string_view replayed,
                                 InputReplayOptions options)
    : allow_capped_{options.allow_capped}, log_paths_{SessionFiles(path)} {
  if (log_paths_.empty()) {
    throw std::runtime_error("no .tlog files in " + path);
  }

  std::map<std::string, std::uint32_t, std::less<>> topic_index;
  // (topic, sequence) -> index in messages_, every log's.
  std::map<std::pair<std::uint32_t, std::uint64_t>, std::size_t> seen;
  // The same for the replayed program's own receipts, kept apart in `own`,
  // along with each receipt's raw transport publish_time_ns (before clamping
  // to its drain's delivered time) for duplicate-sequence conflict checks.
  std::map<std::pair<std::uint32_t, std::uint64_t>, std::size_t> own_seen;
  std::vector<RecordedMessage> own;
  std::vector<std::int64_t> own_publish_ns;
  // Whether a topic's recorded_by is its writer, which a reader never
  // replaces.
  std::vector<bool> by_writer;
  // Whether the replayed program watched the topic live: fed from `own` alone.
  std::vector<bool> watched_by_replayed;
  // The replayed program's first recorded run: its log's origin and run start.
  std::optional<std::pair<MonotonicTime, MonotonicTime>> replayed_run;
  // Every distinct run log of the replayed program (LogOfFile): size 1 means
  // it ran once without restarting.
  std::set<std::string> replayed_logs;
  // The log of that run; the first file of it read; the file the run start
  // was read from (that first file until a firing gives it); whether a
  // firing gave it; the first file of the log read that did not continue
  // the segment before it while none had (log::LogSegmentFiles' next one,
  // read, of the same origin), after which a firing proves nothing; and
  // whether that log's manifest arms a timer (the run start is then read
  // from a firing).
  std::string replayed_run_log;
  std::string replayed_run_first_read;
  std::string replayed_run_skipped_incomplete;
  std::string replayed_run_file;
  bool replayed_run_fired = false;
  std::string replayed_run_broken;
  // The dispatch_index of the first intact record read in that run's first
  // file (before any loss), so an active file whose segment0 was deleted
  // without a marker is refused when it starts past dispatch 1.
  std::optional<std::uint64_t> replayed_run_first_dispatch_index;
  // The now_ns of that run's first record of a dispatch (a TIMER, MESSAGE
  // or EXIT record), and the earliest loss of its timeline (gap_start), read
  // while the log continued from its first segment: a dispatch, or the EXIT
  // that ends the run, is made only inside run().
  std::optional<MonotonicTime> replayed_run_dispatch;
  std::optional<MonotonicTime> replayed_run_loss;
  bool replayed_run_timed = false;
  // Per-timer armed state of the replayed program's first run, updated from
  // its manifest and every TIMER, ARM_TIMER and DISARM_TIMER record across
  // its segments, and the latest event_time_ns of a TIMER or MESSAGE dispatch
  // in that run: used to avoid fabricating a firing after a late EXIT or for
  // an offset-0 timer whose only dispatch record is its EXIT.
  struct TimerState {
    bool armed{false};
    std::optional<std::int64_t> deadline_ns;
    std::int64_t offset_ns{0};
    std::int64_t period_ns{0};
  };
  std::vector<TimerState> replayed_run_timers;
  bool replayed_run_timers_uncertain = false;
  std::optional<MonotonicTime> replayed_run_last_event;
  // The latest time any of its logs recorded: its last run's exit.
  std::optional<MonotonicTime> replayed_end;
  // What one damaged stretch of a log cost one of its topic sources: the
  // earliest time a record lost there could be fed at (none where nothing
  // bounds it) and the stretch's byte offset. For a writer's accepted sends
  // or a non-replayed watcher's receipts, also the sequences its lost records
  // lie in: both advance in transport sequence order, so after the last intact
  // record before the stretch (from 0 when none precedes it) and before the
  // next intact record after it in the same file or a contiguous later segment
  // (`to`, unknown until read). The session's logs holding every message in
  // `[from, *to)`, the stretch lost nothing the feed lacks.
  struct Stretch {
    std::uint32_t topic{0};
    std::optional<MonotonicTime> bound;
    std::uint64_t offset{0};
    bool recoverable{false};
    bool is_sender{false};
    std::uint64_t from{0};
    std::optional<std::uint64_t> to;
  };
  struct OpenStretch {
    std::size_t log_idx{0};
    std::size_t stretch_idx{0};
  };
  struct LogTopic {
    std::uint32_t topic{0};
    bool writer{false};
    bool watched_by_replayed{false};
  };
  // Each log read: the earliest time a GAP record of it names; what each of
  // its damaged stretches cost each topic source; whether it holds its EXIT;
  // whether it ended with a truncated/damaged tail; any missing or unreadable
  // segment detected when reading this file; and the topics it recorded (with
  // whether it wrote each and whether the replayed program watched it there).
  struct ReadLog {
    std::string path;
    std::optional<MonotonicTime> gap;
    std::vector<Stretch> damage;
    bool exit{false};
    bool truncated{false};
    std::string segment_refusal;
    // True when `segment_refusal` is solely a missing first segment (no
    // incomplete header, no middle-segment gap, no dispatch/sequence jump),
    // which a byte-capped log explains when allow_capped is set.
    bool head_missing_only{false};
    std::vector<LogTopic> topics;
  };
  std::vector<ReadLog> logs;
  // Each file read, by path: its origin, then the now_ns of its last intact
  // record and by source id its watchers' last intact receipts' publication
  // times and sequences, per-source monotonic sequence watermarks, open
  // damaged stretches, the earliest loss of its timeline and its timers'
  // offsets and armed states (gap_start, timer_offset, timer_state). A
  // contiguous segment of the same log starts from them, so damage right
  // after its header is bounded, open stretches close on the next intact
  // record in a later segment, and a run start read from its first firing is
  // bounded and offset, as it would be in one file.
  struct Carried {
    MonotonicTime origin;
    std::optional<std::int64_t> last_now;
    std::vector<std::optional<std::int64_t>> last_receipt;
    std::vector<std::optional<std::uint64_t>> last_sequence;
    std::vector<std::optional<std::uint64_t>> last_monotonic_seq;
    std::vector<std::optional<OpenStretch>> open_stretches;
    std::optional<MonotonicTime> gap_start;
    std::vector<std::optional<std::int64_t>> timer_offset;
    std::vector<TimerState> timer_state;
    bool timer_state_uncertain{false};
  };
  std::map<std::string, Carried> carried;
  // Per-log segment continuity across all files of one base log (LogOfFile),
  // which SessionFiles orders consecutively from segment 0 to the unnumbered
  // active file. Catches incomplete-header segments in the middle or at the
  // head, missing first segments without a marker, gaps in segment numbers,
  // and missing segments before the unnumbered active file (via dispatch
  // index or accepted-SEND sequence jumps).
  struct LogContinuity {
    std::string skipped_incomplete;
    bool has_readable{false};
    std::string prev_read;
    std::uint64_t prev_place{0};
    MonotonicTime prev_origin;
    std::size_t prev_manifest_size{0};
    bool prev_truncated{false};
    std::optional<std::uint64_t> prev_last_dispatch;
    std::vector<std::optional<std::uint64_t>> prev_last_sent_seq;
  };
  std::map<std::string, LogContinuity> log_continuity;
  bool first = true;

  for (const std::string& log_path : log_paths_) {
    constexpr std::uint64_t kLast = std::numeric_limits<std::uint64_t>::max();
    const auto [base_log, place] = LogAndPlace(log_path);
    LogContinuity& cont = log_continuity[base_log];

    std::optional<log::LogReader> opened;
    try {
      opened.emplace(log_path);
    } catch (const log::IncompleteLogHeader& e) {
      // A log cut while it was being created holds no records; if other
      // segments of the same log are readable, note this file so any topic
      // feed and run start across it are refused.
      std::cerr << "warning: skipping " << log_path << ": " << e.what() << "\n";
      if (cont.skipped_incomplete.empty()) cont.skipped_incomplete = log_path;
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
    if (own_log) replayed_logs.insert(base_log);

    // This log's topic sources, by source id: where their messages go.
    constexpr std::uint32_t kNone = 0xFFFFFFFF;
    std::vector<std::uint32_t> topic_of(reader.manifest().size(), kNone);
    // The manifest's armed timers' offsets from the origin, until a record
    // re-arms or disarms one, by source id: a timer's first firing at its
    // manifest deadline tells how far the live loop's run() began after
    // its origin (RealtimeEventLoop shifts its schedule by that).
    std::vector<std::optional<std::int64_t>> timer_offset(
        reader.manifest().size());
    std::vector<TimerState> timer_state(reader.manifest().size());
    bool timer_state_uncertain = false;
    std::optional<MonotonicTime> run_start;
    bool timed = false;
    // The now_ns of this file's first record of a dispatch, and the latest
    // event_time_ns of a TIMER or MESSAGE dispatch in this file.
    std::optional<MonotonicTime> first_dispatch;
    std::optional<MonotonicTime> last_event;
    // The topics this log recorded, whether it wrote each, and whether the
    // replayed program watched each in it.
    std::vector<LogTopic> recorded;
    // The earliest time a GAP record of this log names (gap), and the
    // earliest loss of its timeline (gap_start: that, or the now_ns of the
    // last intact record before a damaged stretch), which a run start taken
    // from a timer firing after it must not precede.
    std::optional<MonotonicTime> gap;
    std::optional<MonotonicTime> gap_start;
    // The now_ns of the last intact record, and by source id the
    // publication time and sequence of a watcher's last intact receipt:
    // what bounds the records a damaged stretch after them lost.
    std::optional<std::int64_t> last_now;
    std::vector<std::optional<std::int64_t>> last_receipt(
        reader.manifest().size());
    std::vector<std::optional<std::uint64_t>> last_sequence(
        reader.manifest().size());
    // Per source_id monotonic sequence watermark (carried across contiguous
    // segments, never cleared by gap or damage) to fail closed if a topic
    // segment was re-created outside a launcher session and restarted at 0.
    std::vector<std::optional<std::uint64_t>> last_monotonic_seq(
        reader.manifest().size());
    // By source id, its open stretch (`{log_idx, stretch_idx}` into `logs` or
    // the current `read.damage`), carried across contiguous segments so an
    // intact receipt or accepted send in a later segment closes `to`.
    std::vector<std::optional<OpenStretch>> open(reader.manifest().size());
    // Check structural continuity with the previously read file of `base_log`.
    bool header_continues = false;
    std::string segment_refusal;
    bool head_missing_only = false;
    const std::string skipped_incomplete_before = cont.skipped_incomplete;
    if (!cont.has_readable) {
      if (!cont.skipped_incomplete.empty()) {
        segment_refusal = "its earlier segment " + cont.skipped_incomplete +
                          " of " + base_log +
                          " could not be read (its header is incomplete)";
      } else if (place != 0 && place != kLast) {
        segment_refusal = "its first segment " +
                          log::LogSegmentPath(base_log, 0) + " of " + base_log +
                          " is missing";
        head_missing_only = true;
      }
    } else {
      if (!cont.skipped_incomplete.empty()) {
        segment_refusal = "segment " + cont.skipped_incomplete + " of " +
                          base_log +
                          " before it could not be read (its header is "
                          "incomplete)";
      } else if (place != kLast && place != cont.prev_place + 1) {
        segment_refusal = "segment " +
                          log::LogSegmentPath(base_log, cont.prev_place + 1) +
                          " of " + base_log + " before it is missing";
      } else if (reader.start_time() != cont.prev_origin ||
                 reader.manifest().size() != cont.prev_manifest_size) {
        segment_refusal = "it does not continue " + cont.prev_read + " in " +
                          base_log + " (different origin or manifest)";
      } else if (cont.prev_truncated) {
        segment_refusal = "segment " + cont.prev_read + " of " + base_log +
                          " before it ended with a damaged tail";
      } else {
        header_continues = true;
      }
    }
    cont.skipped_incomplete.clear();

    const Carried* continued = nullptr;
    bool next_segment = false;
    if (header_continues && segment_refusal.empty()) {
      const auto previous = carried.find(cont.prev_read);
      if (previous != carried.end()) {
        continued = &previous->second;
        next_segment = true;
        last_now = continued->last_now;
        last_receipt = continued->last_receipt;
        last_sequence = continued->last_sequence;
        last_monotonic_seq = continued->last_monotonic_seq;
        open = continued->open_stretches;
        gap_start = continued->gap_start;
      }
    }
    ReadLog read{.path = log_path};
    const auto drop_carried_open = [&] {
      for (auto& slot : open) {
        if (slot.has_value() && slot->log_idx != logs.size()) slot.reset();
      }
    };
    const auto close_open_stretch = [&](std::uint32_t source_id, bool is_sender,
                                        std::uint64_t seq) {
      if (source_id >= open.size() || !open[source_id].has_value()) return;
      const OpenStretch ref = *open[source_id];
      if (ref.log_idx != logs.size() && !segment_refusal.empty()) return;
      Stretch& s = ref.log_idx == logs.size()
                       ? read.damage[ref.stretch_idx]
                       : logs[ref.log_idx].damage[ref.stretch_idx];
      if (s.recoverable && s.is_sender == is_sender) {
        s.to = seq;
        open[source_id].reset();
      }
    };
    // Whether a record of a dispatch has been read from this file.
    bool dispatched = false;
    // The damaged stretches counted so far, and where the last intact
    // record ends: where a stretch found after it starts.
    std::size_t spans = 0;
    std::uint64_t record_end = 0;
    for (const Registration& registration : reader.manifest()) {
      if (registration.kind == SourceKind::TIMER &&
          registration.id < timer_offset.size()) {
        timer_state[registration.id] =
            TimerState{.armed = registration.armed,
                       .offset_ns = registration.offset_ns,
                       .period_ns = registration.period_ns};
        if (registration.armed) {
          timer_offset[registration.id] = registration.offset_ns;
          timed = true;
        }
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
      recorded.push_back(LogTopic{.topic = it->second,
                                  .writer = writer,
                                  .watched_by_replayed = own_log && !writer});
    }
    // Every segment repeats the log's manifest: a timer the segment before
    // re-armed or disarmed is as that one left it.
    if (continued != nullptr) {
      timer_offset = continued->timer_offset;
      timer_state = continued->timer_state;
      timer_state_uncertain = continued->timer_state_uncertain;
    }

    // Record-level continuity across segment boundaries (catches a deleted
    // segment right before the unnumbered active file, or a rotated active
    // file whose segment0 was deleted with no marker).
    bool seen_loss_in_file = false;
    bool seen_record_in_file = false;
    std::optional<std::uint64_t> first_file_dispatch;
    std::optional<std::uint64_t> last_dispatch =
        header_continues ? cont.prev_last_dispatch : std::nullopt;
    std::vector<std::optional<std::uint64_t>> last_sent_seq(
        reader.manifest().size());
    if (header_continues &&
        cont.prev_last_sent_seq.size() == last_sent_seq.size()) {
      last_sent_seq = cont.prev_last_sent_seq;
    }
    std::vector<bool> checked_first_send(reader.manifest().size(), false);

    log::LogReader::Record record;
    while (reader.next(record)) {
      const log::RecordHeader& header = record.header;
      // The reader skipped a damaged stretch just before this record: what
      // it lost of each topic source, by when that could be fed at.
      if (reader.damage().spans != spans) {
        seen_loss_in_file = true;
        if (own_log) {
          timer_state_uncertain = true;
          for (TimerState& timer : timer_state) {
            timer.armed = false;
            timer.deadline_ns.reset();
          }
        }
        const std::vector<std::optional<std::uint64_t>> prev_sent_seq =
            last_sent_seq;
        last_dispatch.reset();
        std::fill(last_sent_seq.begin(), last_sent_seq.end(), std::nullopt);
        const std::uint64_t offset =
            spans == 0 ? reader.damage().first_offset : record_end;
        spans = reader.damage().spans;
        // Every record's now_ns is its dispatch's sample of the loop clock,
        // which never decreases along the log, except a send made before
        // the first dispatch, whose now_ns is 0.
        const MonotonicTime since_last =
            last_now
                ? MonotonicTime::from_nanos(*last_now)
                : std::min(MonotonicTime::from_nanos(0), reader.start_time());
        gap_start = std::min(gap_start.value_or(since_last), since_last);
        for (const Registration& source : reader.manifest()) {
          if (source.id >= topic_of.size() || topic_of[source.id] == kNone ||
              open[source.id].has_value()) {
            continue;
          }
          // A writer's send is fed at its publication time, which is its
          // record's now_ns (LogWriter::send), and accepted sends advance in
          // transport sequence order (refused sends carry aux != 0 and are
          // ignored). A reader's receipt is fed at its publication time, and
          // a watcher receives its topic in sequence order, whose publication
          // times never decrease (one writer per topic, stamped from the
          // monotonic clock); before its first receipt nothing bounds it. The
          // replayed program's own receipt is fed at its drain's event time
          // (a tick's deadline, or a prompt watcher's publication time),
          // which need not follow the log's order: nothing bounds it.
          Stretch lost{.topic = topic_of[source.id], .offset = offset};
          if (source.kind == SourceKind::SENDER) {
            lost.bound = since_last;
            lost.recoverable = true;
            lost.is_sender = true;
            lost.from = (source.id < prev_sent_seq.size() &&
                         prev_sent_seq[source.id].has_value())
                            ? *prev_sent_seq[source.id] + 1
                            : 0;
          } else if (!own_log) {
            if (last_receipt[source.id].has_value()) {
              lost.bound = MonotonicTime::from_nanos(*last_receipt[source.id]);
            }
            lost.recoverable = true;
            lost.is_sender = false;
            lost.from = last_sequence[source.id].has_value()
                            ? *last_sequence[source.id] + 1
                            : 0;
          }
          open[source.id] = OpenStretch{.log_idx = logs.size(),
                                        .stretch_idx = read.damage.size()};
          read.damage.push_back(lost);
        }
      }
      if (!seen_record_in_file) {
        seen_record_in_file = true;
        if (!seen_loss_in_file) {
          first_file_dispatch = header.dispatch_index;
          if (!cont.has_readable && place == kLast &&
              header.dispatch_index > 1 && segment_refusal.empty()) {
            segment_refusal = "its first dispatch is " +
                              std::to_string(header.dispatch_index) +
                              ", not 1: its first segment " +
                              log::LogSegmentPath(base_log, 0) + " of " +
                              base_log + " is missing";
            head_missing_only = true;
          } else if (header_continues && cont.prev_last_dispatch.has_value() &&
                     header.dispatch_index > *cont.prev_last_dispatch + 1) {
            next_segment = false;
            if (segment_refusal.empty()) {
              segment_refusal = "dispatch index jumps from " +
                                std::to_string(*cont.prev_last_dispatch) +
                                " in " + cont.prev_read + " to " +
                                std::to_string(header.dispatch_index) + " in " +
                                log_path + ": a segment of " + base_log +
                                " between them is missing";
            }
            drop_carried_open();
          }
        }
      }
      record_end =
          record.offset + sizeof(log::RecordHeader) + header.stored_bytes;
      last_now = header.now_ns;
      if (header.dispatch_index != 0) dispatched = true;
      end_ = std::max(end_, MonotonicTime::from_nanos(header.now_ns));
      if (own_log) {
        replayed_end = std::max(replayed_end.value_or(reader.start_time()),
                                MonotonicTime::from_nanos(header.now_ns));
      }
      if (const auto marked = log::GapOf(header, record.payload)) {
        seen_loss_in_file = true;
        if (own_log) {
          // Records dropped in the gap may hold TIMER firings or
          // ARM_TIMER/DISARM_TIMER calls; invalidate tracked timer state so a
          // stale pre-gap deadline never clamps loop_end() below real firings.
          timer_state_uncertain = true;
          for (TimerState& timer : timer_state) {
            timer.armed = false;
            timer.deadline_ns.reset();
          }
        }
        if (marked->detailed) {
          last_dispatch = marked->detail.last_dispatch_index;
        } else {
          last_dispatch.reset();
        }
        std::fill(last_sent_seq.begin(), last_sent_seq.end(), std::nullopt);
        const MonotonicTime start = MonotonicTime::from_nanos(marked->start_ns);
        gap = std::min(gap.value_or(start), start);
        gap_start = std::min(gap_start.value_or(start), start);
        continue;
      }
      last_dispatch = header.dispatch_index;
      const auto kind = static_cast<EventKind>(header.kind);
      if (kind == EventKind::EXIT) read.exit = true;
      if (kind == EventKind::TIMER || kind == EventKind::MESSAGE) {
        const MonotonicTime event_time =
            MonotonicTime::from_nanos(header.event_time_ns);
        last_event =
            last_event ? std::max(*last_event, event_time) : event_time;
      }
      // A dispatch's own record, or the run's EXIT; not a GAP record (above),
      // which carries a dropped record's dispatch index and the resuming
      // record's now_ns, nor an operation record of a dispatch already seen.
      if (!first_dispatch && header.dispatch_index != 0 &&
          (kind == EventKind::TIMER || kind == EventKind::MESSAGE ||
           kind == EventKind::EXIT)) {
        first_dispatch = MonotonicTime::from_nanos(header.now_ns);
      }
      if (kind == EventKind::MESSAGE &&
          header.source_id < last_receipt.size()) {
        last_receipt[header.source_id] = header.publish_time_ns;
        last_sequence[header.source_id] = header.sequence;
        close_open_stretch(header.source_id, /*is_sender=*/false,
                           header.sequence);
      }
      if (own_log && header.source_id < timer_state.size()) {
        TimerState& timer = timer_state[header.source_id];
        if (kind == EventKind::ARM_TIMER) {
          timer.armed = true;
          timer.deadline_ns = header.event_time_ns;
          timer.period_ns = static_cast<std::int64_t>(header.sequence);
        } else if (kind == EventKind::DISARM_TIMER) {
          timer.armed = false;
          timer.deadline_ns.reset();
        } else if (kind == EventKind::TIMER) {
          if (timer.period_ns > 0) {
            timer.armed = true;
            timer.deadline_ns = AdvanceDeadline(
                header.event_time_ns, timer.period_ns, header.sequence);
          } else {
            timer.armed = false;
            timer.deadline_ns.reset();
          }
        }
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
          std::int64_t start_ns = SaturatingSub(header.event_time_ns, *offset);
          if (gap_start.has_value()) {
            start_ns =
                std::max(start_ns, SaturatingSub(gap_start->nanos(), *offset));
          }
          run_start = MonotonicTime::from_nanos(start_ns);
        }
      }
      // A watcher's receipt, or a writer's accepted send.
      const bool receipt = kind == EventKind::MESSAGE;
      const bool send = kind == EventKind::SEND && header.aux == 0;
      if (send && header.source_id < last_sent_seq.size()) {
        if (header_continues && !seen_loss_in_file &&
            !checked_first_send[header.source_id]) {
          checked_first_send[header.source_id] = true;
          if (cont.prev_last_sent_seq.size() == last_sent_seq.size() &&
              cont.prev_last_sent_seq[header.source_id].has_value() &&
              header.sequence >
                  *cont.prev_last_sent_seq[header.source_id] + 1) {
            next_segment = false;
            if (segment_refusal.empty()) {
              const std::string topic_name =
                  header.source_id < topic_of.size() &&
                          topic_of[header.source_id] != kNone
                      ? topics_[topic_of[header.source_id]].name
                      : std::to_string(header.source_id);
              segment_refusal =
                  "topic '" + topic_name + "' sequence jumps from " +
                  std::to_string(*cont.prev_last_sent_seq[header.source_id]) +
                  " in " + cont.prev_read + " to " +
                  std::to_string(header.sequence) + " in " + log_path +
                  ": a segment of " + base_log + " between them is missing";
            }
            drop_carried_open();
          }
        }
        last_sent_seq[header.source_id] = header.sequence;
        close_open_stretch(header.source_id, /*is_sender=*/true,
                           header.sequence);
      }
      if ((!receipt && !send) || header.source_id >= topic_of.size() ||
          topic_of[header.source_id] == kNone) {
        continue;
      }
      const std::uint32_t topic = topic_of[header.source_id];
      if (record.payload.size() != topics_[topic].message_bytes) continue;
      if (header.source_id < last_monotonic_seq.size()) {
        if (last_monotonic_seq[header.source_id].has_value() &&
            header.sequence <= *last_monotonic_seq[header.source_id]) {
          throw ReplayError(
              "non-monotonic sequence on topic '" + topics_[topic].name +
              "' in " + log_path + ": sequence " +
              std::to_string(header.sequence) + " after " +
              std::to_string(*last_monotonic_seq[header.source_id]) +
              " (a segment may have been re-created outside a launcher "
              "session)");
        }
        last_monotonic_seq[header.source_id] = header.sequence;
      }
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
          const std::span<const std::byte> known_payload{
              bytes_.data() + known.offset, topics_[topic].message_bytes};
          if (header.publish_time_ns != own_publish_ns[it->second] ||
              !std::equal(record.payload.begin(), record.payload.end(),
                          known_payload.begin())) {
            throw ReplayError(
                "conflicting records for topic '" + topics_[topic].name +
                "' sequence " + std::to_string(header.sequence) + " in " +
                log_path +
                " (different publish time or payload; a segment may have been "
                "re-created outside a launcher session)");
          }
          if (delivered < known.delivered) {
            known.delivered = delivered;
            known.time = time;
          }
          continue;
        }
        own.push_back({time, delivered, topic, header.sequence, bytes_.size()});
        own_publish_ns.push_back(header.publish_time_ns);
        bytes_.insert(bytes_.end(), record.payload.begin(),
                      record.payload.end());
        continue;
      }
      // A send made before the loop's first dispatch (a constructor's) is
      // recorded with no clock (LogWriter::send from an empty Context:
      // now_ns and publish_time_ns 0), but the transport stamped it with
      // the loop's origin (monotonic_now() before the run), as its readers
      // recorded: a restarted writer's initial message is fed at its
      // restart, not at the session's start.
      const bool before_first_dispatch = send && !dispatched &&
                                         header.dispatch_index == 0 &&
                                         header.now_ns == 0;
      const MonotonicTime time =
          before_first_dispatch
              ? reader.start_time()
              : MonotonicTime::from_nanos(header.publish_time_ns);
      const auto [it, inserted] =
          seen.try_emplace({topic, header.sequence}, messages_.size());
      if (!inserted) {
        const RecordedMessage& known = messages_[it->second];
        const std::span<const std::byte> known_payload{
            bytes_.data() + known.offset, topics_[topic].message_bytes};
        if (time != known.time ||
            !std::equal(record.payload.begin(), record.payload.end(),
                        known_payload.begin())) {
          throw ReplayError(
              "conflicting records for topic '" + topics_[topic].name +
              "' sequence " + std::to_string(header.sequence) + " in " +
              log_path +
              " (different publish time or payload; a segment may have been "
              "re-created outside a launcher session)");
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
                        ? "; a replay fed from it stops before a damaged "
                          "stretch, or is refused, unless the session's logs "
                          "hold every message the stretch could have lost\n"
                        : "; its intact records replay\n");
    }
    if (!cont.has_readable && place == 0 && !seen_record_in_file &&
        !seen_loss_in_file) {
      // An intact empty segment0 starts before dispatch 1, so a jump past 1
      // in a following segment is still detected.
      last_dispatch = 0;
    }
    cont.has_readable = true;
    cont.prev_read = log_path;
    cont.prev_place = place;
    cont.prev_origin = reader.start_time();
    cont.prev_manifest_size = reader.manifest().size();
    cont.prev_truncated = reader.truncated();
    cont.prev_last_dispatch = last_dispatch;
    cont.prev_last_sent_seq = std::move(last_sent_seq);

    carried[log_path] = Carried{.origin = reader.start_time(),
                                .last_now = last_now,
                                .last_receipt = last_receipt,
                                .last_sequence = last_sequence,
                                .last_monotonic_seq = last_monotonic_seq,
                                .open_stretches = open,
                                .gap_start = gap_start,
                                .timer_offset = timer_offset,
                                .timer_state = timer_state,
                                .timer_state_uncertain = timer_state_uncertain};
    read.gap = gap;
    read.truncated = reader.truncated();
    read.segment_refusal = std::move(segment_refusal);
    read.head_missing_only = head_missing_only;
    read.topics = std::move(recorded);
    logs.push_back(std::move(read));
    // Its first run: the earliest origin among its logs (restarts included).
    // Its run start is read from the first firing in any file of that run's
    // log: a later segment's, when the ones before it (read before it, with
    // the same origin) held none, as when the byte cap rotated the log
    // before the first deadline, but only while each file read continued
    // the one before it: a segment missing or unread between them may have
    // held the first firing. With no firing in any, it is the first dispatch
    // (below).
    if (own_log &&
        (!replayed_run || reader.start_time() < replayed_run->first)) {
      replayed_run.emplace(reader.start_time(),
                           std::max(reader.start_time(),
                                    run_start.value_or(reader.start_time())));
      replayed_run_log = base_log;
      replayed_run_first_read = log_path;
      replayed_run_skipped_incomplete = skipped_incomplete_before;
      replayed_run_file = log_path;
      replayed_run_fired = run_start.has_value();
      replayed_run_broken.clear();
      replayed_run_first_dispatch_index = first_file_dispatch;
      replayed_run_dispatch = first_dispatch;
      replayed_run_loss = gap_start;
      replayed_run_timed = timed;
      replayed_run_timers = timer_state;
      replayed_run_timers_uncertain = timer_state_uncertain;
      replayed_run_last_event = last_event;
    } else if (own_log && replayed_run &&
               reader.start_time() == replayed_run->first &&
               base_log == replayed_run_log) {
      replayed_run_timers = timer_state;
      replayed_run_timers_uncertain = replayed_run_timers_uncertain ||
                                      timer_state_uncertain ||
                                      continued == nullptr || !next_segment;
      if (last_event) {
        replayed_run_last_event =
            replayed_run_last_event
                ? std::max(*replayed_run_last_event, *last_event)
                : *last_event;
      }
      if (!replayed_run_fired) {
        if (continued == nullptr || !next_segment) {
          if (replayed_run_broken.empty()) replayed_run_broken = log_path;
        } else if (replayed_run_broken.empty()) {
          if (!replayed_run_dispatch) replayed_run_dispatch = first_dispatch;
          // Carried from the segment before: the loss so far.
          replayed_run_loss = gap_start;
          if (run_start) {
            replayed_run->second = std::max(reader.start_time(), *run_start);
            replayed_run_file = log_path;
            replayed_run_fired = true;
          }
        }
      }
    }
  }
  // A run whose log arms a timer and holds no firing that gives its run
  // start (none recorded, or only of timers it re-armed first) starts at its
  // first record of a dispatch. That record is made inside run(), so at or
  // after the live run start: every timer armed before the run, at the run
  // start plus its offset, then falls no earlier than it did live, and live
  // fired none up to that run's last record, so neither does the replay up
  // to there; a restarted program replays as one run. A timer it re-armed in
  // a dispatch is re-armed at that dispatch's time, as live. The origin would
  // fire such a timer the whole construction time early, at deadlines live
  // never reached. Its log is refused (below) when a loss in it may hold its
  // firings, or when it holds no such record.
  if (replayed_run && replayed_run_timed && !replayed_run_fired &&
      !replayed_run_loss && replayed_run_dispatch) {
    replayed_run->second =
        std::max(replayed_run->first, *replayed_run_dispatch);
  }
  // Whether the replayed program's first run was read from its first segment.
  // When segment 0 was rotated out (e.g. by the byte cap under
  // --allow-capped-replay), an ARM_TIMER or DISARM_TIMER in segment 0 is
  // unseen, so clamping below only uses timers whose deadline was observed in
  // the kept window (or offset-0 timers when no firing was kept at all).
  bool replayed_run_from_head = true;
  if (replayed_run) {
    const std::vector<std::string> files =
        log::LogSegmentFiles(replayed_run_log);
    const bool starts_mid_run = replayed_run_first_dispatch_index.has_value() &&
                                *replayed_run_first_dispatch_index > 1;
    replayed_run_from_head =
        !starts_mid_run &&
        (LogAndPlace(replayed_run_first_read).second == 0 ||
         (files.size() == 1 && files.front() == replayed_run_first_read));
  }
  // A single-run live loop that was late when it stopped can record its EXIT
  // after a due timer deadline it never fired, and an offset-0 timer whose
  // run's only dispatch record is its EXIT sits right at
  // run_start == replayed_end. When the run's kept records have no GAP,
  // damaged stretch, or broken segment carry, end the replay one nanosecond
  // before the earliest armed timer deadline strictly after any recorded TIMER
  // or MESSAGE dispatch (or any deadline when none ran), avoiding fabricated
  // firings in those cases. When segment 0 was rotated out, only timers with a
  // deadline observed in the kept window (or any armed timer when no firing
  // was kept) participate, so a timer disarmed in segment 0 cannot clamp early.
  if (replayed_run && replayed_end && replayed_logs.size() == 1 &&
      !replayed_run_timers_uncertain) {
    std::optional<MonotonicTime> earliest_unfired;
    for (const TimerState& timer : replayed_run_timers) {
      if (!timer.armed) continue;
      if (!replayed_run_from_head && !timer.deadline_ns.has_value() &&
          replayed_run_fired) {
        continue;
      }
      const MonotonicTime deadline =
          timer.deadline_ns.has_value()
              ? MonotonicTime::from_nanos(*timer.deadline_ns)
              : MonotonicTime::from_nanos(SaturatingAdd(
                    replayed_run->second.nanos(), timer.offset_ns));
      if (!replayed_run_last_event.has_value() ||
          deadline > *replayed_run_last_event) {
        earliest_unfired =
            std::min(earliest_unfired.value_or(deadline), deadline);
      }
    }
    if (earliest_unfired && *earliest_unfired <= *replayed_end) {
      replayed_end = MonotonicTime::from_nanos(
          SaturatingSub(earliest_unfired->nanos(), 1));
    }
  }
  if (first) throw std::runtime_error("no readable .tlog files in " + path);

  // Cross-check the replayed program's own receipts against other logs sharing
  // (topic, sequence) before replacing those topics' messages with `own`.
  for (const auto& [key, own_idx] : own_seen) {
    const auto other = seen.find(key);
    if (other == seen.end()) continue;
    const RecordedMessage& a = own[own_idx];
    const RecordedMessage& b = messages_[other->second];
    const std::size_t n = topics_[a.topic].message_bytes;
    if (own_publish_ns[own_idx] != b.time.nanos() ||
        !std::equal(bytes_.begin() + static_cast<std::ptrdiff_t>(a.offset),
                    bytes_.begin() + static_cast<std::ptrdiff_t>(a.offset + n),
                    bytes_.begin() + static_cast<std::ptrdiff_t>(b.offset))) {
      throw ReplayError(
          "conflicting records for topic '" + topics_[a.topic].name +
          "' sequence " + std::to_string(a.sequence) +
          " across session logs (different publish time or payload; a segment "
          "may have been re-created outside a launcher session)");
    }
  }
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
  const auto cap_refusal = [&](const std::string& file) -> std::string {
    const std::string log = LogOfFile(file);
    if (std::string marker = log::CappedMarkerPath(log);
        std::filesystem::exists(marker)) {
      return marker + " exists: no GAP record marks what the byte cap cost it";
    }
    const auto marks =
        marked.find(std::filesystem::path{log}.filename().string());
    if (marks != marked.end() && marks->second.log_capped) {
      return "the session manifest marks its member log_capped: no GAP "
             "record marks what the byte cap cost it";
    }
    return {};
  };
  const auto hard_refusal = [&](const std::string& file) -> std::string {
    const std::string log = LogOfFile(file);
    const auto marks =
        marked.find(std::filesystem::path{log}.filename().string());
    if (marks == marked.end()) return {};
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
  // Whether the session's logs hold every message of `topic` with a
  // sequence from `from` to before `to`. One writer per topic: a session's
  // transport numbers a topic's messages 0, 1, 2, ..., no two alike, as the
  // merge by sequence above assumes.
  const auto held = [&](std::uint32_t topic, std::uint64_t from,
                        std::uint64_t to) {
    if (to < from) return false;
    const auto first = seen.lower_bound({topic, from});
    const auto last = seen.lower_bound({topic, to});
    return static_cast<std::uint64_t>(std::distance(first, last)) == to - from;
  };
  // Precompute per-log cap/hard/segment refusals once so writer-completeness
  // and the topic-feed loop judge every log identically.
  struct LogVerdict {
    std::string base_log;
    std::string cap_sign;
    std::string hard_sign;
  };
  std::vector<LogVerdict> verdicts;
  verdicts.reserve(logs.size());
  for (const ReadLog& log : logs) {
    const std::string base_log = LogOfFile(log.path);
    const std::string cap_sign = cap_refusal(log.path);
    std::string hard_sign = hard_refusal(log.path);
    if (hard_sign.empty() && !log.segment_refusal.empty() &&
        !(log.head_missing_only && !cap_sign.empty())) {
      hard_sign = log.segment_refusal;
    }
    verdicts.push_back({base_log, cap_sign, hard_sign});
  }
  // For any topic the replayed program did not watch live: if at least one
  // writer log exists for the topic, every writer log of that topic is
  // complete (no cap/hard/segment refusal, no GAP, no unrecovered damage on
  // that topic, not truncated, and its run reached EXIT), and the session
  // holds every sequence in [0, max_seq], then the writer holds the topic in
  // full and reader-only caps, marks, gaps and damage cost the feed nothing.
  std::vector<bool> writer_complete(topics_.size(), false);
  for (std::size_t topic = 0; topic < topics_.size(); ++topic) {
    if (watched_by_replayed[topic]) continue;
    bool has_writer = false;
    bool all_writers_complete = true;
    for (std::size_t i = 0; i < logs.size(); ++i) {
      const ReadLog& log = logs[i];
      const bool is_writer = std::any_of(
          log.topics.begin(), log.topics.end(), [&](const LogTopic& entry) {
            return entry.topic == topic && entry.writer;
          });
      if (!is_writer) continue;
      has_writer = true;
      const LogVerdict& v = verdicts[i];
      if (!v.cap_sign.empty() || !v.hard_sign.empty() ||
          !log.segment_refusal.empty() || log.gap.has_value() ||
          log.truncated || !exited.contains(v.base_log)) {
        all_writers_complete = false;
        break;
      }
      for (const Stretch& stretch : log.damage) {
        if (stretch.topic != topic) continue;
        if (!(stretch.recoverable && stretch.to.has_value() &&
              held(static_cast<std::uint32_t>(topic), stretch.from,
                   *stretch.to))) {
          all_writers_complete = false;
          break;
        }
      }
      if (!all_writers_complete) break;
    }
    if (!has_writer || !all_writers_complete) continue;
    const auto t = static_cast<std::uint32_t>(topic);
    const auto first_msg = seen.lower_bound({t, 0});
    const auto last_msg =
        seen.upper_bound({t, std::numeric_limits<std::uint64_t>::max()});
    if (first_msg == last_msg) {
      writer_complete[topic] = true;
    } else {
      const std::uint64_t max_seq = std::prev(last_msg)->first.second;
      writer_complete[topic] =
          max_seq < std::numeric_limits<std::uint64_t>::max() &&
          held(t, 0, max_seq + 1);
    }
  }
  // Where each topic's feed ends, and whether it is refused: the earliest
  // loss, and the first refused log, among the logs it is fed from, which
  // for a topic the program watched live are its own logs that watched it,
  // and otherwise every log that recorded it (excluding reader-only logs of a
  // topic proven writer-complete above). A writer's or reader's damaged
  // stretch whose lost sequences `[from, *to)` the session's logs all hold
  // lost nothing the feed lacks, and is no loss.
  for (std::size_t i = 0; i < logs.size(); ++i) {
    const ReadLog& log = logs[i];
    const std::string& base_log = verdicts[i].base_log;
    const std::string& cap_sign = verdicts[i].cap_sign;
    const std::string& hard_sign = verdicts[i].hard_sign;
    for (const LogTopic& entry : log.topics) {
      const std::uint32_t topic = entry.topic;
      const bool watched_here = entry.watched_by_replayed;
      if (watched_by_replayed[topic] && !watched_here) continue;
      const bool log_writes_topic = std::any_of(
          log.topics.begin(), log.topics.end(), [&](const LogTopic& other) {
            return other.topic == topic && other.writer;
          });
      if (!log_writes_topic && writer_complete[topic]) continue;
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
      std::string reason = hard_sign;
      for (const Stretch& stretch : log.damage) {
        if (stretch.topic != topic ||
            (stretch.recoverable && stretch.to.has_value() &&
             held(topic, stretch.from, *stretch.to))) {
          continue;
        }
        if (stretch.bound.has_value()) {
          loss(*stretch.bound, stretch.offset);
        } else if (reason.empty()) {
          reason = "records are lost between intact ones at byte " +
                   std::to_string(stretch.offset) + ", and nothing " +
                   "bounds when they were to be fed (" +
                   (watched_here ? "the program's own receipts, fed at their "
                                   "drains' times"
                                 : "no intact receipt of the topic precedes "
                                   "them, and the session's logs do not hold "
                                   "every message they could be") +
                   ")";
        }
      }
      if (!reason.empty() && fed.refused_log.empty()) {
        fed.refused_log = log.path;
        fed.refused_reason = reason;
      }
      if (!cap_sign.empty()) {
        if (fed.capped_log.empty()) {
          fed.capped_log = log.path;
          fed.capped_reason = cap_sign;
        }
        if (std::find(fed.capped_logs.begin(), fed.capped_logs.end(),
                      base_log) == fed.capped_logs.end()) {
          fed.capped_logs.push_back(base_log);
        }
      }
    }
  }
  // The replayed program's run start is its first timer firing less that
  // timer's offset, which only its first run's log read from its first
  // segment on is sure to hold. Once that segment was not read (the byte cap
  // deleted it, or its header is incomplete: the first file read is neither
  // LogSegmentPath(log, 0) nor the log of a run never rotated), the first
  // firing read may be whole periods after the run's, and every timer would
  // replay that late: the log is refused, as a fed topic's capped log is,
  // whether or not a topic is fed from it, unless it is byte-capped and
  // InputReplayOptions::allow_capped is set. So is a log in which a file read
  // before any firing did not continue the one before it (a segment between
  // them missing or unread). With no armed timer the run start is the log's
  // origin, which every segment holds.
  if (replayed_run_timed) {
    const std::string& log = replayed_run_log;
    const std::string head = log::LogSegmentPath(log, 0);
    if (!replayed_run_from_head) {
      const std::string marker = log::CappedMarkerPath(log);
      const auto marks =
          marked.find(std::filesystem::path{log}.filename().string());
      const bool capped = std::filesystem::exists(marker) ||
                          (marks != marked.end() && marks->second.log_capped);
      if (!replayed_run_skipped_incomplete.empty()) {
        run_start_refused_log_ = replayed_run_first_read;
        run_start_refused_reason_ =
            "its earlier segment could not be read (its header is "
            "incomplete): " +
            replayed_run_skipped_incomplete +
            ", which may have held the first timer firing the run start is "
            "read from, and a later segment's first firing may be whole "
            "periods after it";
      } else if (std::filesystem::exists(head)) {
        run_start_refused_log_ = replayed_run_first_read;
        run_start_refused_reason_ =
            "its first segment could not be read (its header is "
            "incomplete): " +
            head +
            ", which may have held the first timer firing the run start is "
            "read from, and a later segment's first firing may be whole "
            "periods after it";
      } else if (capped) {
        const std::string sign =
            std::filesystem::exists(marker)
                ? marker + " exists: the byte cap deleted its first segment "
                : "the session manifest marks its member log_capped: the byte "
                  "cap deleted its first segment ";
        run_start_capped_log_ = replayed_run_first_read;
        run_start_capped_base_log_ = log;
        run_start_capped_reason_ =
            sign + head +
            ", which may have held the first timer firing the run start is "
            "read from, and a later segment's first firing may be whole "
            "periods after it";
      } else {
        run_start_refused_log_ = replayed_run_first_read;
        run_start_refused_reason_ =
            "its first segment is gone: " + head +
            ", which may have held the first timer firing the run start is "
            "read from, and a later segment's first firing may be whole "
            "periods after it";
      }
    }
    if (run_start_refused_log_.empty()) {
      if (!replayed_run_fired && !replayed_run_broken.empty()) {
        run_start_refused_log_ = replayed_run_broken;
        run_start_refused_reason_ =
            "it does not continue the segment of " + log +
            " read before it (a segment between them is missing or could not "
            "be read), which may have held the first timer firing the run "
            "start is read from, and none was read before it";
      } else if (!replayed_run_fired && !replayed_run_dispatch) {
        // Its run began (its writer starts with run()) and left no record: a
        // replay would run its timers from the origin to the session's end.
        run_start_refused_log_ = replayed_run_first_read;
        run_start_refused_reason_ =
            "it holds no record of its run: nothing gives its start or its end";
      } else if (!replayed_run_fired && replayed_run_loss) {
        // The firings that give the run start may be in the loss; its first
        // dispatch would then replay none of them.
        run_start_refused_log_ = replayed_run_first_read;
        run_start_refused_reason_ =
            "it holds no timer firing that gives its run start, and a loss in "
            "it (a GAP record or damaged stretch, from " +
            std::to_string(replayed_run_loss->nanos()) +
            " ns) may hold the run's firings";
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

void WarnCappedReplay(const std::vector<std::string>& capped_logs) {
  if (capped_logs.empty()) return;
  std::cerr << "warning: --allow-capped-replay: replaying capped log"
            << (capped_logs.size() == 1 ? "" : "s") << " ";
  for (std::size_t i = 0; i < capped_logs.size(); ++i) {
    if (i > 0) std::cerr << ", ";
    std::cerr << capped_logs[i];
  }
  std::cerr << " from the start of the kept window; pre-window history was "
               "lost to the byte cap\n";
}

}  // namespace talos::event

#pragma once

// Merging one session's event logs into one time-ordered stream (log_dump,
// tlog_to_wpilog), read through a fixed window per log: memory grows with the
// number of logs, not their records.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/log/format.h"
#include "talOS/events/manifest.h"

namespace talos::launcher {

// One log of a merge: its file, process and manifest, read once when the
// merge opens it. Every record of the log names its process and source
// through it, so no name is copied per record.
struct MergedLog {
  std::string path;
  std::string process_name;
  event::Manifest manifest;
  // `source_id`'s entry in the manifest; null if none.
  const event::Registration* source(std::uint16_t source_id) const;
};

// One record of the merge, as the visitor sees it.
struct MergedRecord {
  // The record's log, shared so a caller may keep it past the merge.
  std::shared_ptr<const MergedLog> log;
  // The record's source in its log's manifest, schema included (format.h);
  // null for an EXIT record or an id the manifest lacks.
  const event::Registration* source{nullptr};
  event::log::RecordHeader header{};
  // The full header.payload_bytes; valid only during the visit.
  std::span<const std::byte> payload;
  // Where the record starts in `log->path`: a WindowedLogReader of that file
  // resumes there (seek).
  std::uint64_t offset{0};

  const std::string& process_name() const { return log->process_name; }
  std::string_view source_name() const {
    return source != nullptr ? std::string_view{source->name} : "?";
  }
};

// Whether a merge prints its warnings on stderr: a log skipped for a header
// cut while it was created, and, once it ends, a log the reader skipped
// damage in ("warning: damaged log <path>: ...", which session_report
// parses). A caller that merges a session twice warns on one pass.
enum class MergeWarnings { kPrint, kQuiet };

// Hands every intact record of `log_paths` to `visit`, ordered by event
// time, then now, then dispatch index, and on a tie by the order of
// `log_paths` (each log's own records keep their order). Every log is
// opened, and its session id checked against the first's, before the first
// record is visited. False, with `error` set, on no paths, a log that cannot
// be opened, or a session id mismatch; a visitor returning false stops the
// merge, which then returns true.
bool MergeLogs(const std::vector<std::string>& log_paths,
               const std::function<bool(const MergedRecord&)>& visit,
               std::string& error,
               MergeWarnings warnings = MergeWarnings::kPrint);

}  // namespace talos::launcher

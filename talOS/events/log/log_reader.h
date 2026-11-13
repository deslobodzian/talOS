#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/context.h"
#include "talOS/events/log/format.h"
#include "talOS/events/manifest.h"
#include "talOS/events/time.h"

namespace talos::event::log {

// Thrown (by LogReader and SummarizeLog) for a file that ends, or reads back
// zero-filled, before its header, manifest and schema block are complete: a
// log cut by a power loss while it was being created. It holds no records,
// so a reader of a whole session skips it with a warning rather than failing.
class IncompleteLogHeader : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// What a reader skipped because it was not an intact record (format.h): a
// power loss can leave a garbage or zero-filled tail, and a damaged disk
// block can break a record anywhere. Offsets are byte offsets in the file.
struct LogDamage {
  // Damaged stretches skipped because an intact record follows them. Each
  // one lost at least one record; how many is unknowable.
  std::size_t spans{0};
  std::uint64_t skipped_bytes{0};  // in those stretches
  std::uint64_t first_offset{0};   // where the first one starts
  // Bytes after the last intact record that hold no intact record: a record
  // cut mid-write, or a garbage or zero-filled tail.
  std::uint64_t tail_bytes{0};
  std::uint64_t tail_offset{0};  // where they start

  bool clean() const noexcept { return spans == 0 && tail_bytes == 0; }
};

// The most bytes any record's payload may claim, whatever its manifest
// declares. No talOS source carries a message near it: the largest logged
// today is the 1436 B hardware::Packet (docs/DATA_FLOW.md), and RTMS keeps up
// to 1024 slots of a topic in shared memory. It bounds what one crafted or
// damaged record, whose header checksum its author can compute, makes a
// reader allocate.
inline constexpr std::uint64_t kMaxRecordPayloadBytes = std::uint64_t{1} << 20;

namespace detail {
// The most bytes each record may claim: a record carrying its topic's
// message (a MESSAGE of a watcher or fetcher, a SEND of a sender) that
// topic's declared message size; any other record (a timer's, an id the
// manifest does not list) the larger of 64 KiB and the manifest's largest
// message; never more than kMaxRecordPayloadBytes. A record claiming more is
// damage (LogReader).
class PayloadBounds {
 public:
  void add(const ManifestEntry& entry);
  std::uint64_t operator()(const RecordHeader& header) const noexcept;

 private:
  static constexpr std::uint64_t kMinOtherBytes = std::uint64_t{64} << 10;
  struct Topic {
    EventKind carries;  // the record kind that carries its message
    std::uint64_t bytes;
  };
  std::vector<std::optional<Topic>> topic_;  // by source id
  std::uint64_t other_{kMinOtherBytes};
};
}  // namespace detail

// Whether `header` is a SEND its transport accepted: LogWriter::send logs a
// refused or failed send too, its aux the loop's SendStatus (0 OK; the
// bridge logs its own write the same way). Only an accepted send's payload
// is a value a reader could have received.
inline bool IsAcceptedSend(const RecordHeader& header) noexcept {
  return header.kind == static_cast<std::uint16_t>(EventKind::SEND) &&
         header.aux == 0;
}

// One line naming what `damage` skipped, offsets included; empty if clean.
std::string DescribeDamage(const LogDamage& damage);

// Reads a dispatch log written by LogWriter.
//
// The whole file is read into memory up front: these logs are meant to be
// replayed on the same class of machine that could run the loop in real
// time, so they are small enough that mmap would only add complexity.
//
// Records are resynchronised past damage. Where the next record's magic,
// lengths or checksum are wrong, or it runs past the end of the file, the
// reader scans forward for the next RECORD_MAGIC whose header and checksum
// validate and whose dispatch index and time can follow the last record it
// returned (one from another log, in disk blocks a power loss exposed,
// cannot), resumes there, and counts what it skipped in damage(). Bytes
// with no intact record after them end the log (truncated()). A record whose
// payload_bytes exceeds what its source allows (detail::PayloadBounds: its
// topic's declared message size, else the larger of the manifest's largest
// message and 64 KiB, never more than kMaxRecordPayloadBytes) is damage too,
// so one false magic costs at most that much checksumming and one crafted
// header at most that much memory. A caller that must not see a gap (exact
// replay) refuses a log whose damage().spans is nonzero.
class LogReader {
 public:
  struct Record {
    RecordHeader header{};
    // The full header.payload_bytes, zero-filled past what the file stores.
    // Owned by the reader: valid until the next call to next() or rewind().
    std::span<const std::byte> payload;
    // Where the record's header starts in its file (a rotated log's segment
    // is a file of its own): what WindowedLogReader::seek resumes at.
    std::uint64_t offset{0};
  };

  // Reads and validates the header and manifest block. Throws
  // std::runtime_error on a bad magic, an unsupported format version (naming
  // actual and supported versions), a checksum mismatch, or a short file.
  explicit LogReader(std::string_view path);

  const Manifest& manifest() const noexcept { return manifest_; }
  MonotonicTime start_time() const noexcept { return start_time_; }
  std::string process_name() const { return process_name_; }
  std::int64_t start_wall_ns() const noexcept { return start_wall_ns_; }
  std::uint32_t format_version() const noexcept { return format_version_; }
  std::uint64_t session_id() const noexcept { return session_id_; }

  // Advances to the next intact record, skipping damage (see above). Returns
  // false once no intact record is left.
  bool next(Record& out);

  void rewind() noexcept;

  // True once next() has stopped on bytes that hold no intact record
  // (damage().tail_bytes). A robot that lost power mid-write leaves a log
  // like this; everything before the cut still replays.
  bool truncated() const noexcept { return damage_.tail_bytes != 0; }

  // What next() has skipped since construction or the last rewind().
  const LogDamage& damage() const noexcept { return damage_; }

  // Number of records next() has returned since construction or the last
  // rewind(); call after iterating to completion for the file's total.
  std::size_t record_count() const noexcept { return record_count_; }

 private:
  std::vector<std::byte> data_;
  std::vector<std::byte> payload_;  // zero-filled copy of the last payload
  Manifest manifest_;
  MonotonicTime start_time_;
  std::int64_t start_wall_ns_{0};
  std::uint32_t format_version_{0};
  std::uint64_t session_id_{0};
  std::string process_name_;
  std::size_t records_offset_{0};
  std::size_t cursor_{0};
  detail::PayloadBounds bounds_;
  LogDamage damage_;
  std::optional<RecordHeader> last_;  // the last record next() returned
  std::size_t record_count_{0};
};

// Reads a log as LogReader does, but through a fixed window instead of the
// whole file: it holds the header, manifest and schema block, a 64 KiB
// window and the current record's payload, however long the log. It
// validates exactly what LogReader does, throws the same errors, and skips
// damage exactly as LogReader::next does. Each reader keeps its file open.
// A reader of many logs at once (a session's merge) uses one per log.
class WindowedLogReader {
 public:
  explicit WindowedLogReader(std::string_view path);
  ~WindowedLogReader();
  WindowedLogReader(WindowedLogReader&&) noexcept;
  WindowedLogReader& operator=(WindowedLogReader&&) noexcept;
  WindowedLogReader(const WindowedLogReader&) = delete;
  WindowedLogReader& operator=(const WindowedLogReader&) = delete;

  const Manifest& manifest() const noexcept;
  // The manifest block's entries as stored, in order.
  std::span<const ManifestEntry> manifest_entries() const noexcept;
  MonotonicTime start_time() const noexcept;
  const std::string& process_name() const noexcept;
  std::uint64_t session_id() const noexcept;

  // As LogReader::next. The payload is valid until the next call.
  bool next(LogReader::Record& out);

  // Continues at `offset`, the Record::offset of a record a reader of this
  // same file returned: next() then returns that record and every record
  // after it exactly as a reader from the start does, damage skipped alike,
  // and damage() and record_count() count from there. The header, manifest
  // and schemas stay as the constructor read them, so resuming costs one
  // header read and no record before `offset`. Throws std::invalid_argument
  // naming the file and the offset where no intact record (magic, lengths
  // and checksum) starts at `offset`, or `offset` is inside the header,
  // manifest or schema block. A check cannot tell an intact record a full
  // read would not have returned from one it would: a record nested whole in
  // another's payload, an intact record inside a span a full read skipped as
  // damage (one from another log, or past the scan's size limit), or a
  // boundary of another file of the same layout. An offset is therefore kept
  // with the path of the file it came from.
  void seek(std::uint64_t offset);

  const LogDamage& damage() const noexcept;
  bool truncated() const noexcept { return damage().tail_bytes != 0; }
  std::size_t record_count() const noexcept;

 private:
  struct State;
  std::unique_ptr<State> state_;
};

// What LogReader would report after reading a log to its end -- record
// count, truncation and damage -- without holding the file: it reads through
// a fixed buffer, so memory does not grow with the log. It validates exactly
// what LogReader does and throws the same errors.
struct LogSummary {
  std::size_t record_count{0};
  bool truncated{false};
  LogDamage damage;
};
LogSummary SummarizeLog(std::string_view path);

// What SummarizeLog walks, handed to a caller as it goes, through a
// WindowedLogReader: the manifest block once its checksum validates, then
// each intact record (resynchronised past damage exactly as LogReader::next
// is).
struct LogVisitor {
  // The manifest's entries, before any record; false skips the records.
  std::function<bool(std::span<const ManifestEntry>)> manifest;
  // One record, its payload zero-filled to payload_bytes as LogReader's;
  // valid only during the call. False stops the walk.
  std::function<bool(const LogReader::Record&)> record;
};
struct VisitOptions {
  // Where the records start: 0 for the first, else a Record::offset a reader
  // of this file returned (WindowedLogReader::seek, which refuses any other
  // before the manifest callback runs).
  std::uint64_t start_offset{0};
};
// Throws what SummarizeLog throws, and what seek throws. The summary counts
// the records walked, and the damage skipped, from `start_offset`: fewer
// than the log holds when a callback stopped the walk.
LogSummary VisitLog(std::string_view path, const LogVisitor& visitor,
                    const VisitOptions& options = {});

// A session directory's event logs: every `*.tlog` in it (the launcher
// writes one per node run, restarts included), sorted by path. A path that is
// not a directory is returned as the one log it names.
std::vector<std::string> FindLogFiles(const std::string& path_or_dir);

// The files the log written at `log_path` is kept in, oldest first: the
// segments a byte cap rotated out (LogSegmentPath) and still kept, then
// `log_path` itself; only those that exist.
std::vector<std::string> LogSegmentFiles(const std::string& log_path);

}  // namespace talos::event::log

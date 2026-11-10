#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

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
// with no intact record after them end the log (truncated()). A record found
// by the scan may claim at most the larger of the manifest's largest message
// and 64 KiB, so one false magic costs at most that much checksumming. A
// caller that must not see a gap (exact replay) refuses a log whose
// damage().spans is nonzero.
class LogReader {
 public:
  struct Record {
    RecordHeader header{};
    // The full header.payload_bytes, zero-filled past what the file stores.
    // Owned by the reader: valid until the next call to next() or rewind().
    std::span<const std::byte> payload;
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
  std::uint64_t scan_payload_bytes_{0};
  LogDamage damage_;
  std::optional<RecordHeader> last_;  // the last record next() returned
  std::size_t record_count_{0};
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

// A session directory's event logs: every `*.tlog` in it (the launcher
// writes one per node run, restarts included), sorted by path. A path that is
// not a directory is returned as the one log it names.
std::vector<std::string> FindLogFiles(const std::string& path_or_dir);

// The files the log written at `log_path` is kept in, oldest first: the
// segments a byte cap rotated out (LogSegmentPath) and still kept, then
// `log_path` itself; only those that exist.
std::vector<std::string> LogSegmentFiles(const std::string& log_path);

}  // namespace talos::event::log

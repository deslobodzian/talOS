#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/log/format.h"
#include "talOS/events/manifest.h"
#include "talOS/events/time.h"

namespace talos::event::log {

// Reads a dispatch log written by LogWriter.
//
// The whole file is read into memory up front: these logs are meant to be
// replayed on the same class of machine that could run the loop in real
// time, so they are small enough that mmap would only add complexity.
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

  // Advances to the next record. Returns false once there is nothing left to
  // read, whether that is a clean end of file or a file that ends mid
  // record (see truncated()). Throws std::runtime_error, naming the record
  // index, if a record's magic or checksum is wrong: that is corruption, not
  // an ordinary end of file, and must not be replayed silently.
  bool next(Record& out);

  void rewind() noexcept;

  // True once next() has stopped because the file ended without enough
  // bytes left for a full record. A robot that lost power mid-write leaves
  // a log like this; everything before the cut still replays.
  bool truncated() const noexcept { return truncated_; }

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
  bool truncated_{false};
  std::size_t record_count_{0};
};

// What LogReader would report after reading a log to its end -- record count
// and whether the file ends mid record -- without holding the file: it
// streams through a fixed buffer, so memory does not grow with the log. It
// validates exactly what LogReader does and throws the same errors.
struct LogSummary {
  std::size_t record_count{0};
  bool truncated{false};
};
LogSummary SummarizeLog(std::string_view path);

// A session directory's event logs: every `*.tlog` in it (the launcher
// writes one per node run, restarts included), sorted by path. A path that is
// not a directory is returned as the one log it names.
std::vector<std::string> FindLogFiles(const std::string& path_or_dir);

}  // namespace talos::event::log

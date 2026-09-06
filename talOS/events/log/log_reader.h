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
    std::span<const std::byte> payload;
  };

  // Reads and validates the header and manifest block. Throws
  // std::runtime_error on a bad magic, an unknown format version, a manifest
  // whose checksum does not match, or a file too short to hold either.
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

}  // namespace talos::event::log

#include "talOS/events/log/log_reader.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace talos::event::log {

namespace {

std::vector<std::byte> read_whole_file(std::string_view path) {
  const int fd = ::open(std::string{path}.c_str(), O_RDONLY);
  if (fd < 0) {
    throw std::system_error(errno, std::generic_category(),
                            "LogReader: open failed for " + std::string{path});
  }

  struct stat st {};
  if (::fstat(fd, &st) != 0) {
    const int saved_errno = errno;
    ::close(fd);
    throw std::system_error(saved_errno, std::generic_category(),
                            "LogReader: fstat failed for " + std::string{path});
  }

  std::vector<std::byte> data(static_cast<std::size_t>(st.st_size));
  std::size_t read_bytes = 0;
  while (read_bytes < data.size()) {
    const ssize_t n =
        ::read(fd, data.data() + read_bytes, data.size() - read_bytes);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      const int saved_errno = errno;
      ::close(fd);
      throw std::system_error(
          saved_errno, std::generic_category(),
          "LogReader: read failed for " + std::string{path});
    }
    if (n == 0) {
      break;  // File shrank under us; treat what we got as the whole file.
    }
    read_bytes += static_cast<std::size_t>(n);
  }
  data.resize(read_bytes);

  ::close(fd);
  return data;
}

// Magic first, then version, so an older log -- whose header is shorter --
// is reported as one, not as a truncated file.
void check_identity(const FileHeader& header) {
  // A file whose length reached the disk before its bytes did reads back
  // zero-filled: cut while it was being created, not damaged.
  constexpr char kZeros[sizeof(header.magic)] = {};
  if (std::memcmp(header.magic, kZeros, sizeof(header.magic)) == 0) {
    throw IncompleteLogHeader("LogReader: file header is zero-filled");
  }
  if (std::memcmp(header.magic, FILE_MAGIC, sizeof(header.magic)) != 0) {
    throw std::runtime_error("LogReader: bad file magic");
  }
  if (header.version != FORMAT_VERSION) {
    throw std::runtime_error(
        "LogReader: unsupported format version " +
        std::to_string(header.version) + " (supported version " +
        std::to_string(FORMAT_VERSION) +
        "); dump or replay it with the build that recorded it");
  }
}

void check_record_magic(const RecordHeader& header, std::size_t index) {
  if (header.magic != RECORD_MAGIC) {
    throw std::runtime_error("LogReader: record " + std::to_string(index) +
                             ": bad magic");
  }
}

// The CRC of a record's header with its crc32 field zeroed: the seed its
// stored bytes are folded into.
std::uint32_t record_header_crc(RecordHeader header) {
  header.crc32 = 0;
  return crc32(std::span<const std::byte>{
      reinterpret_cast<const std::byte*>(&header), sizeof(header)});
}

void check_record(const RecordHeader& header, std::uint32_t crc,
                  std::size_t index) {
  if (crc != header.crc32) {
    throw std::runtime_error("LogReader: record " + std::to_string(index) +
                             ": checksum mismatch");
  }
  if (header.stored_bytes > header.payload_bytes) {
    throw std::runtime_error("LogReader: record " + std::to_string(index) +
                             ": stores more bytes than its payload");
  }
}

// A log file read front to back through one fixed buffer.
class LogStream {
 public:
  explicit LogStream(std::string_view path)
      : fd_{::open(std::string{path}.c_str(), O_RDONLY | O_CLOEXEC)} {
    if (fd_ < 0) {
      throw std::system_error(
          errno, std::generic_category(),
          "LogReader: open failed for " + std::string{path});
    }
  }
  ~LogStream() { ::close(fd_); }
  LogStream(const LogStream&) = delete;
  LogStream& operator=(const LogStream&) = delete;

  // Fills `out`; fewer bytes only at the end of the file.
  std::size_t read(std::span<std::byte> out) {
    std::size_t got = 0;
    while (got < out.size()) {
      const ssize_t n = ::read(fd_, out.data() + got, out.size() - got);
      if (n < 0) {
        if (errno == EINTR) continue;
        throw std::system_error(errno, std::generic_category(),
                                "LogReader: read failed");
      }
      if (n == 0) break;
      got += static_cast<std::size_t>(n);
    }
    return got;
  }

  template <typename T>
  std::size_t read(T& out) {
    return read(
        std::span<std::byte>{reinterpret_cast<std::byte*>(&out), sizeof(out)});
  }

  // Reads the next `size` bytes, folding them into `crc`. False if the file
  // ends first.
  bool fold(std::uint64_t size, std::uint32_t& crc) {
    while (size > 0) {
      const auto want =
          static_cast<std::size_t>(std::min<std::uint64_t>(size, kBuffer));
      const std::size_t got = read(std::span{buffer_}.first(want));
      crc = crc32(std::span<const std::byte>{buffer_}.first(got), crc);
      if (got < want) return false;
      size -= got;
    }
    return true;
  }

 private:
  static constexpr std::size_t kBuffer = 64u << 10;
  int fd_;
  std::vector<std::byte> buffer_ = std::vector<std::byte>(kBuffer);
};

}  // namespace

LogReader::LogReader(std::string_view path) : data_{read_whole_file(path)} {
  constexpr std::size_t kVersionEnd = offsetof(FileHeader, header_bytes);
  if (data_.size() < kVersionEnd) {
    throw IncompleteLogHeader("LogReader: file too short for a header");
  }

  FileHeader header{};
  std::memcpy(&header, data_.data(), std::min(data_.size(), sizeof(header)));
  check_identity(header);
  if (header.header_bytes < sizeof(FileHeader) ||
      data_.size() < header.header_bytes) {
    throw IncompleteLogHeader("LogReader: file too short for its header");
  }

  const std::size_t manifest_bytes_size =
      static_cast<std::size_t>(header.manifest_count) * sizeof(ManifestEntry);
  const std::size_t manifest_offset = header.header_bytes;
  const std::size_t schema_offset = manifest_offset + manifest_bytes_size;

  if (data_.size() < schema_offset ||
      data_.size() - schema_offset < header.schema_bytes) {
    throw IncompleteLogHeader("LogReader: file too short for its manifest");
  }

  const std::span<const std::byte> manifest_bytes{
      data_.data() + manifest_offset, manifest_bytes_size};
  const std::span<const std::byte> schema_bytes{
      data_.data() + schema_offset,
      static_cast<std::size_t>(header.schema_bytes)};
  if (crc32(schema_bytes, crc32(manifest_bytes)) != header.crc32) {
    throw std::runtime_error("manifest checksum mismatch");
  }

  manifest_.reserve(header.manifest_count);
  std::size_t schema_at = 0;
  for (std::uint32_t i = 0; i < header.manifest_count; ++i) {
    ManifestEntry entry{};
    std::memcpy(&entry,
                data_.data() + manifest_offset + i * sizeof(ManifestEntry),
                sizeof(entry));

    Registration reg{};
    reg.id = entry.id;
    reg.kind = static_cast<SourceKind>(entry.kind);
    reg.name = read_name(entry.name);
    reg.message_bytes = entry.message_bytes;
    reg.alignment = entry.alignment;
    reg.period_ns = entry.period_ns;
    reg.offset_ns = entry.offset_ns;
    reg.armed = entry.armed != 0;

    const std::size_t schema_size = std::size_t{entry.schema_name_bytes} +
                                    std::size_t{entry.schema_bfbs_bytes};
    if (schema_bytes.size() - schema_at < schema_size) {
      throw std::runtime_error("LogReader: source " + std::to_string(i) +
                               "'s schema runs past the schema block");
    }
    const char* schema =
        reinterpret_cast<const char*>(schema_bytes.data()) + schema_at;
    reg.schema.name.assign(schema, entry.schema_name_bytes);
    reg.schema.hash = entry.schema_hash;
    reg.schema.bfbs.assign(schema + entry.schema_name_bytes,
                           entry.schema_bfbs_bytes);
    schema_at += schema_size;
    manifest_.push_back(std::move(reg));
  }
  if (schema_at != schema_bytes.size()) {
    throw std::runtime_error(
        "LogReader: schema block longer than its manifest's schemas");
  }

  start_time_ = MonotonicTime::from_nanos(header.start_monotonic_ns);
  start_wall_ns_ = header.start_wall_ns;
  format_version_ = header.version;
  session_id_ = header.session_id;
  process_name_ = read_name(header.process_name);

  records_offset_ = schema_offset + schema_bytes.size();
  cursor_ = records_offset_;
}

void LogReader::rewind() noexcept {
  cursor_ = records_offset_;
  truncated_ = false;
  record_count_ = 0;
}

bool LogReader::next(Record& out) {
  const std::size_t remaining = data_.size() - cursor_;
  if (remaining < sizeof(RecordHeader)) {
    if (remaining > 0) {
      truncated_ = true;
    }
    return false;
  }

  RecordHeader header{};
  std::memcpy(&header, data_.data() + cursor_, sizeof(header));
  check_record_magic(header, record_count_);

  const std::size_t total = sizeof(RecordHeader) + header.stored_bytes;
  if (remaining < total) {
    truncated_ = true;
    return false;
  }

  const std::span<const std::byte> stored{
      data_.data() + cursor_ + sizeof(RecordHeader), header.stored_bytes};
  check_record(header, crc32(stored, record_header_crc(header)), record_count_);

  out.header = header;
  if (header.stored_bytes == header.payload_bytes) {
    out.payload = stored;
  } else {
    payload_.assign(header.payload_bytes, std::byte{0});
    if (!stored.empty()) {
      std::memcpy(payload_.data(), stored.data(), stored.size());
    }
    out.payload = payload_;
  }
  cursor_ += total;
  ++record_count_;
  return true;
}

LogSummary SummarizeLog(std::string_view path) {
  LogStream stream{path};
  constexpr std::size_t kVersionEnd = offsetof(FileHeader, header_bytes);
  FileHeader header{};
  const std::size_t header_read = stream.read(header);
  if (header_read < kVersionEnd) {
    throw IncompleteLogHeader("LogReader: file too short for a header");
  }
  check_identity(header);
  std::uint32_t ignored = 0;
  if (header.header_bytes < sizeof(FileHeader) ||
      header_read < sizeof(FileHeader) ||
      !stream.fold(header.header_bytes - sizeof(FileHeader), ignored)) {
    throw IncompleteLogHeader("LogReader: file too short for its header");
  }

  // The manifest entries stream one at a time, the schema block in chunks.
  std::uint32_t crc = 0;
  std::uint64_t schemas = 0;
  std::optional<std::uint32_t> overrun;
  for (std::uint32_t i = 0; i < header.manifest_count; ++i) {
    ManifestEntry entry{};
    if (stream.read(entry) < sizeof(entry)) {
      throw IncompleteLogHeader("LogReader: file too short for its manifest");
    }
    crc = crc32(
        std::span<const std::byte>{reinterpret_cast<const std::byte*>(&entry),
                                   sizeof(entry)},
        crc);
    schemas += std::uint64_t{entry.schema_name_bytes} + entry.schema_bfbs_bytes;
    if (!overrun && schemas > header.schema_bytes) overrun = i;
  }
  if (!stream.fold(header.schema_bytes, crc)) {
    throw IncompleteLogHeader("LogReader: file too short for its manifest");
  }
  if (crc != header.crc32) {
    throw std::runtime_error("manifest checksum mismatch");
  }
  if (overrun) {
    throw std::runtime_error("LogReader: source " + std::to_string(*overrun) +
                             "'s schema runs past the schema block");
  }
  if (schemas != header.schema_bytes) {
    throw std::runtime_error(
        "LogReader: schema block longer than its manifest's schemas");
  }

  LogSummary summary;
  for (;;) {
    RecordHeader record{};
    const std::size_t got = stream.read(record);
    if (got < sizeof(record)) {
      summary.truncated = got > 0;
      return summary;
    }
    check_record_magic(record, summary.record_count);
    std::uint32_t record_crc = record_header_crc(record);
    if (!stream.fold(record.stored_bytes, record_crc)) {
      summary.truncated = true;
      return summary;
    }
    check_record(record, record_crc, summary.record_count);
    ++summary.record_count;
  }
}

std::vector<std::string> FindLogFiles(const std::string& path_or_dir) {
  std::error_code ec;
  if (!std::filesystem::is_directory(path_or_dir, ec)) return {path_or_dir};
  std::vector<std::string> result;
  for (const auto& entry :
       std::filesystem::directory_iterator(path_or_dir, ec)) {
    if (entry.path().extension() == ".tlog") {
      result.push_back(entry.path().string());
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

std::vector<std::string> LogSegmentFiles(const std::string& log_path) {
  // A segment's name is LogSegmentPath(log_path, k): split it around k.
  const std::string probe = LogSegmentPath(log_path, 0);
  const std::filesystem::path probe_path{probe};
  const std::string probe_name = probe_path.filename().string();
  const std::string head = probe_name.substr(0, probe_name.size() - 6);
  const std::string tail = ".tlog";
  std::filesystem::path directory = probe_path.parent_path();
  if (directory.empty()) directory = ".";

  std::vector<std::pair<std::uint64_t, std::string>> segments;
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
    const std::string name = entry.path().filename().string();
    if (name.size() <= head.size() + tail.size() || !name.starts_with(head) ||
        !name.ends_with(tail))
      continue;
    const std::string_view digits{name.data() + head.size(),
                                  name.size() - head.size() - tail.size()};
    std::uint64_t k = 0;
    const auto [end, error] =
        std::from_chars(digits.data(), digits.data() + digits.size(), k);
    if (error != std::errc{} || end != digits.data() + digits.size()) continue;
    segments.emplace_back(k, entry.path().string());
  }
  std::sort(segments.begin(), segments.end());
  std::vector<std::string> result;
  result.reserve(segments.size() + 1);
  for (auto& [k, path] : segments) result.push_back(std::move(path));
  if (std::filesystem::exists(log_path, ec)) result.push_back(log_path);
  return result;
}

}  // namespace talos::event::log

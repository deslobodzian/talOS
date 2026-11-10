#include "talOS/events/log/log_reader.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
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

// The CRC of a record's header with its crc32 field zeroed: the seed its
// stored bytes are folded into.
std::uint32_t record_header_crc(RecordHeader header) {
  header.crc32 = 0;
  return crc32(std::span<const std::byte>{
      reinterpret_cast<const std::byte*>(&header), sizeof(header)});
}

// The least payload a record found by a resynchronising scan may claim
// (LogReader); the manifest's largest message raises it. It bounds the
// checksumming one false magic can cost.
constexpr std::uint64_t kMinScanPayloadBytes = 64u << 10;
constexpr std::uint64_t kAnyPayload = std::numeric_limits<std::uint64_t>::max();

// A log's bytes, read through view(): a whole file in memory (SpanSource) or
// an open file through a fixed window (FileSource).
class SpanSource {
 public:
  static constexpr std::size_t kChunk = 1u << 20;
  explicit SpanSource(std::span<const std::byte> data) : data_{data} {}
  std::uint64_t size() const noexcept { return data_.size(); }
  // Up to `n` (at most kChunk) bytes at `offset`; fewer only at the end.
  std::span<const std::byte> view(std::uint64_t offset, std::size_t n) const {
    if (offset >= data_.size()) return {};
    const auto at = static_cast<std::size_t>(offset);
    return data_.subspan(at, std::min(n, data_.size() - at));
  }

 private:
  std::span<const std::byte> data_;
};

class FileSource {
 public:
  static constexpr std::size_t kChunk = 64u << 10;
  explicit FileSource(int fd) : fd_{fd} {
    struct stat st {};
    if (::fstat(fd_, &st) != 0) {
      throw std::system_error(errno, std::generic_category(),
                              "LogReader: fstat failed");
    }
    size_ = static_cast<std::uint64_t>(st.st_size);
  }
  std::uint64_t size() const noexcept { return size_; }
  // Up to `n` (at most kChunk) bytes at `offset`, valid until the next call;
  // fewer only at the end of the file.
  std::span<const std::byte> view(std::uint64_t offset, std::size_t n) {
    if (offset < window_offset_ ||
        offset + n > window_offset_ + window_.size()) {
      fill(offset);
    }
    if (offset < window_offset_ || offset >= window_offset_ + window_.size()) {
      return {};
    }
    const auto at = static_cast<std::size_t>(offset - window_offset_);
    return std::span<const std::byte>{window_}.subspan(
        at, std::min(n, window_.size() - at));
  }

 private:
  void fill(std::uint64_t offset) {
    window_offset_ = offset;
    window_.resize(static_cast<std::size_t>(
        std::min<std::uint64_t>(kChunk, size_ - std::min(offset, size_))));
    std::size_t got = 0;
    while (got < window_.size()) {
      const ssize_t n = ::pread(fd_, window_.data() + got, window_.size() - got,
                                static_cast<off_t>(offset + got));
      if (n < 0) {
        if (errno == EINTR) continue;
        throw std::system_error(errno, std::generic_category(),
                                "LogReader: read failed");
      }
      if (n == 0) break;  // Shrank under us: what is there is the file.
      got += static_cast<std::size_t>(n);
    }
    window_.resize(got);
  }

  int fd_;
  std::uint64_t size_{0};
  std::uint64_t window_offset_{0};
  std::vector<std::byte> window_;
};

// Folds the `size` bytes at `offset` into `crc`. False if the file ends
// first.
template <typename Source>
bool fold(Source& source, std::uint64_t offset, std::uint64_t size,
          std::uint32_t& crc) {
  while (size > 0) {
    const auto want =
        static_cast<std::size_t>(std::min<std::uint64_t>(size, Source::kChunk));
    const std::span<const std::byte> bytes = source.view(offset, want);
    crc = crc32(bytes, crc);
    if (bytes.size() < want) return false;
    offset += want;
    size -= want;
  }
  return true;
}

// The offset of the first RECORD_MAGIC at or after `from`; size() if none.
template <typename Source>
std::uint64_t find_magic(Source& source, std::uint64_t from) {
  std::array<std::byte, sizeof(RECORD_MAGIC)> magic{};
  std::memcpy(magic.data(), &RECORD_MAGIC, magic.size());
  while (from < source.size() && source.size() - from >= magic.size()) {
    const std::span<const std::byte> bytes = source.view(from, Source::kChunk);
    const auto found =
        std::search(bytes.begin(), bytes.end(), magic.begin(), magic.end());
    if (found != bytes.end()) {
      return from + static_cast<std::uint64_t>(found - bytes.begin());
    }
    if (bytes.size() < magic.size()) break;
    // The next window overlaps this one by a partial magic's length.
    from += bytes.size() - (magic.size() - 1);
  }
  return source.size();
}

// Whether an intact record -- magic, lengths and checksum -- starts at
// `offset` and ends within the file, claiming at most `max_payload` bytes.
template <typename Source>
bool intact_at(Source& source, std::uint64_t offset, std::uint64_t max_payload,
               RecordHeader& header) {
  const std::span<const std::byte> bytes =
      source.view(offset, sizeof(RecordHeader));
  if (bytes.size() < sizeof(RecordHeader)) return false;
  std::memcpy(&header, bytes.data(), sizeof(header));
  if (header.magic != RECORD_MAGIC ||
      header.stored_bytes > header.payload_bytes ||
      header.payload_bytes > max_payload) {
    return false;
  }
  const std::uint64_t body = offset + sizeof(RecordHeader);
  if (source.size() - body < header.stored_bytes) return false;
  std::uint32_t crc = record_header_crc(header);
  return fold(source, body, header.stored_bytes, crc) && crc == header.crc32;
}

// Whether a record a scan found can follow `last`, the record accepted
// before the damage, in the log that wrote it: dispatch indices never go
// back, and a later dispatch was not dispatched earlier. Within one dispatch
// a READ carries the time it was taken, so a later record of that dispatch
// may carry an earlier one. An intact record that fails this belongs to
// another log (old disk blocks a power loss exposed) and is skipped.
bool can_follow(const RecordHeader& record,
                const std::optional<RecordHeader>& last) {
  if (!last || record.dispatch_index == last->dispatch_index) return true;
  return record.dispatch_index > last->dispatch_index &&
         record.now_ns >= last->now_ns;
}

// The intact record at `cursor`, or the next one a scan finds that can
// follow `last`, moving `cursor` to it, setting `last` to it and counting
// what was skipped in `damage`. Empty, with `cursor` at the end, once no
// intact record is left.
template <typename Source>
std::optional<RecordHeader> locate(Source& source, std::uint64_t& cursor,
                                   std::uint64_t scan_payload_bytes,
                                   LogDamage& damage,
                                   std::optional<RecordHeader>& last) {
  if (cursor >= source.size()) return std::nullopt;
  RecordHeader header{};
  if (intact_at(source, cursor, kAnyPayload, header)) {
    last = header;
    return header;
  }

  // A damaged length cannot be trusted to skip by: scan from the next byte.
  const std::uint64_t start = cursor;
  for (std::uint64_t at = find_magic(source, start + 1); at < source.size();
       at = find_magic(source, at + 1)) {
    if (intact_at(source, at, scan_payload_bytes, header) &&
        can_follow(header, last)) {
      if (damage.spans == 0) damage.first_offset = start;
      ++damage.spans;
      damage.skipped_bytes += at - start;
      cursor = at;
      last = header;
      return header;
    }
  }
  damage.tail_offset = start;
  damage.tail_bytes = source.size() - start;
  cursor = source.size();
  return std::nullopt;
}

// A log file's header, manifest and schema block, read front to back
// through one fixed buffer.
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

  int fd() const noexcept { return fd_; }

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

  scan_payload_bytes_ = kMinScanPayloadBytes;
  for (const Registration& reg : manifest_) {
    scan_payload_bytes_ =
        std::max<std::uint64_t>(scan_payload_bytes_, reg.message_bytes);
  }
  records_offset_ = schema_offset + schema_bytes.size();
  cursor_ = records_offset_;
}

void LogReader::rewind() noexcept {
  cursor_ = records_offset_;
  damage_ = LogDamage{};
  last_.reset();
  record_count_ = 0;
}

bool LogReader::next(Record& out) {
  SpanSource source{data_};
  std::uint64_t cursor = cursor_;
  const std::optional<RecordHeader> header =
      locate(source, cursor, scan_payload_bytes_, damage_, last_);
  cursor_ = static_cast<std::size_t>(cursor);
  if (!header) return false;

  const std::span<const std::byte> stored{
      data_.data() + cursor_ + sizeof(RecordHeader), header->stored_bytes};
  out.header = *header;
  if (header->stored_bytes == header->payload_bytes) {
    out.payload = stored;
  } else {
    payload_.assign(header->payload_bytes, std::byte{0});
    if (!stored.empty()) {
      std::memcpy(payload_.data(), stored.data(), stored.size());
    }
    out.payload = payload_;
  }
  cursor_ += sizeof(RecordHeader) + header->stored_bytes;
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
  std::uint64_t scan_payload_bytes = kMinScanPayloadBytes;
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
    scan_payload_bytes =
        std::max<std::uint64_t>(scan_payload_bytes, entry.message_bytes);
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

  // The records, through a fixed window that a resynchronising scan can
  // also move back over.
  FileSource source{stream.fd()};
  std::uint64_t cursor =
      std::uint64_t{header.header_bytes} +
      std::uint64_t{header.manifest_count} * sizeof(ManifestEntry) +
      header.schema_bytes;
  LogSummary summary;
  std::optional<RecordHeader> last;
  while (const std::optional<RecordHeader> record =
             locate(source, cursor, scan_payload_bytes, summary.damage, last)) {
    cursor += sizeof(RecordHeader) + record->stored_bytes;
    ++summary.record_count;
  }
  summary.truncated = summary.damage.tail_bytes != 0;
  return summary;
}

std::string DescribeDamage(const LogDamage& damage) {
  std::string text;
  if (damage.spans != 0) {
    text = std::to_string(damage.spans) + " damaged span(s) skipped (" +
           std::to_string(damage.skipped_bytes) + " bytes, first at byte " +
           std::to_string(damage.first_offset) + ")";
  }
  if (damage.tail_bytes != 0) {
    if (!text.empty()) text += "; ";
    text += "no intact record in the last " +
            std::to_string(damage.tail_bytes) + " bytes (from byte " +
            std::to_string(damage.tail_offset) + ")";
  }
  return text;
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

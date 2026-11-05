#include "talOS/events/log/log_reader.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <system_error>

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

}  // namespace

LogReader::LogReader(std::string_view path) : data_{read_whole_file(path)} {
  // The magic and version come first so an older log -- whose header is
  // shorter -- is reported as one, not as a truncated file.
  constexpr std::size_t kVersionEnd = offsetof(FileHeader, header_bytes);
  if (data_.size() < kVersionEnd) {
    throw std::runtime_error("LogReader: file too short for a header");
  }

  FileHeader header{};
  std::memcpy(&header, data_.data(), std::min(data_.size(), sizeof(header)));

  if (std::memcmp(header.magic, FILE_MAGIC, sizeof(header.magic)) != 0) {
    throw std::runtime_error("LogReader: bad file magic");
  }
  if (header.version < FORMAT_VERSION) {
    throw std::runtime_error(
        "LogReader: format version " + std::to_string(header.version) +
        " predates message schemas in logs (this build reads version " +
        std::to_string(FORMAT_VERSION) +
        "); dump or replay it with the build that recorded it");
  }
  if (header.version != FORMAT_VERSION) {
    throw std::runtime_error("LogReader: unsupported format version " +
                             std::to_string(header.version) + " (expected " +
                             std::to_string(FORMAT_VERSION) + ")");
  }
  if (header.header_bytes < sizeof(FileHeader) ||
      data_.size() < header.header_bytes) {
    throw std::runtime_error("LogReader: file too short for its header");
  }

  const std::size_t manifest_bytes_size =
      static_cast<std::size_t>(header.manifest_count) * sizeof(ManifestEntry);
  const std::size_t manifest_offset = header.header_bytes;
  const std::size_t schema_offset = manifest_offset + manifest_bytes_size;

  if (data_.size() < schema_offset ||
      data_.size() - schema_offset < header.schema_bytes) {
    throw std::runtime_error("LogReader: file too short for its manifest");
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

  if (header.magic != RECORD_MAGIC) {
    throw std::runtime_error("LogReader: record " +
                             std::to_string(record_count_) + ": bad magic");
  }

  const std::size_t total = sizeof(RecordHeader) + header.stored_bytes;
  if (remaining < total) {
    truncated_ = true;
    return false;
  }

  const std::span<const std::byte> stored{
      data_.data() + cursor_ + sizeof(RecordHeader), header.stored_bytes};

  RecordHeader header_for_crc = header;
  header_for_crc.crc32 = 0;
  std::uint32_t crc = crc32(std::span<const std::byte>{
      reinterpret_cast<const std::byte*>(&header_for_crc),
      sizeof(header_for_crc)});
  if (!stored.empty()) {
    crc = crc32(stored, crc);
  }
  if (crc != header.crc32) {
    throw std::runtime_error("LogReader: record " +
                             std::to_string(record_count_) +
                             ": checksum mismatch");
  }
  if (header.stored_bytes > header.payload_bytes) {
    throw std::runtime_error("LogReader: record " +
                             std::to_string(record_count_) +
                             ": stores more bytes than its payload");
  }

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

}  // namespace talos::event::log

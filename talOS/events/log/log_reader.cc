#include "talOS/events/log/log_reader.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
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

  struct stat st{};
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
  if (data_.size() < sizeof(FileHeader)) {
    throw std::runtime_error("LogReader: file too short for a header");
  }

  FileHeader header{};
  std::memcpy(&header, data_.data(), sizeof(header));

  if (std::memcmp(header.magic, FILE_MAGIC, sizeof(header.magic)) != 0) {
    throw std::runtime_error("LogReader: bad file magic");
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

  if (data_.size() < manifest_offset + manifest_bytes_size) {
    throw std::runtime_error("LogReader: file too short for its manifest");
  }

  const std::span<const std::byte> manifest_bytes{
      data_.data() + manifest_offset, manifest_bytes_size};
  if (crc32(manifest_bytes) != header.crc32) {
    throw std::runtime_error("manifest checksum mismatch");
  }

  manifest_.reserve(header.manifest_count);
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
    manifest_.push_back(std::move(reg));
  }

  start_time_ = MonotonicTime::from_nanos(header.start_monotonic_ns);
  start_wall_ns_ = header.start_wall_ns;
  format_version_ = header.version;
  session_id_ = header.session_id;
  process_name_ = read_name(header.process_name);

  records_offset_ = manifest_offset + manifest_bytes_size;
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

  const std::size_t total = sizeof(RecordHeader) + header.payload_bytes;
  if (remaining < total) {
    truncated_ = true;
    return false;
  }

  const std::span<const std::byte> payload{
      data_.data() + cursor_ + sizeof(RecordHeader), header.payload_bytes};

  RecordHeader header_for_crc = header;
  header_for_crc.crc32 = 0;
  std::uint32_t crc = crc32(std::span<const std::byte>{
      reinterpret_cast<const std::byte*>(&header_for_crc),
      sizeof(header_for_crc)});
  if (!payload.empty()) {
    crc = crc32(payload, crc);
  }
  if (crc != header.crc32) {
    throw std::runtime_error("LogReader: record " +
                             std::to_string(record_count_) +
                             ": checksum mismatch");
  }

  out.header = header;
  out.payload = payload;
  cursor_ += total;
  ++record_count_;
  return true;
}

}  // namespace talos::event::log

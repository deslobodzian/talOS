#include "talOS/launcher/wpilog.h"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <ios>
#include <ostream>
#include <span>
#include <string_view>
#include <vector>

namespace talos::wpilog {
namespace {

// Bytes needed for `value`, at least one.
std::size_t Width(std::uint64_t value) {
  std::size_t bytes = 1;
  while (bytes < 8 && (value >> (8U * bytes)) != 0) ++bytes;
  return bytes;
}

void AppendLittleEndian(std::vector<std::uint8_t>& out, std::uint64_t value,
                        std::size_t bytes) {
  for (std::size_t i = 0; i < bytes; ++i) {
    out.push_back(static_cast<std::uint8_t>(value >> (8U * i)));
  }
}

void AppendString(std::vector<std::uint8_t>& out, std::string_view text) {
  AppendLittleEndian(out, text.size(), 4);
  out.insert(out.end(), text.begin(), text.end());
}

constexpr std::uint8_t kStart = 0;
constexpr std::uint8_t kFinish = 1;
constexpr std::uint8_t kSetMetadata = 2;

}  // namespace

void AppendRecord(std::vector<std::uint8_t>& out, std::uint32_t entry,
                  std::uint64_t timestamp_us,
                  std::span<const std::uint8_t> payload) {
  const std::size_t entry_bytes = Width(entry);
  const std::size_t size_bytes = Width(payload.size());
  const std::size_t time_bytes = Width(timestamp_us);
  out.push_back(static_cast<std::uint8_t>(
      (entry_bytes - 1) | ((size_bytes - 1) << 2U) | ((time_bytes - 1) << 4U)));
  AppendLittleEndian(out, entry, entry_bytes);
  AppendLittleEndian(out, payload.size(), size_bytes);
  AppendLittleEndian(out, timestamp_us, time_bytes);
  out.insert(out.end(), payload.begin(), payload.end());
}

void AppendBoolean(std::vector<std::uint8_t>& out, bool value) {
  out.push_back(value ? 1 : 0);
}

void AppendInt64(std::vector<std::uint8_t>& out, std::int64_t value) {
  AppendLittleEndian(out, static_cast<std::uint64_t>(value), 8);
}

void AppendFloat(std::vector<std::uint8_t>& out, float value) {
  AppendLittleEndian(out, std::bit_cast<std::uint32_t>(value), 4);
}

void AppendDouble(std::vector<std::uint8_t>& out, double value) {
  AppendLittleEndian(out, std::bit_cast<std::uint64_t>(value), 8);
}

Writer::Writer(std::ostream& out, std::string_view extra_header) : out_{out} {
  std::vector<std::uint8_t> header{'W', 'P', 'I', 'L', 'O', 'G'};
  AppendLittleEndian(header, kVersion, 2);
  AppendString(header, extra_header);
  out_.write(reinterpret_cast<const char*>(header.data()),
             static_cast<std::streamsize>(header.size()));
}

std::uint32_t Writer::start(std::string_view name, std::string_view type,
                            std::string_view metadata,
                            std::uint64_t timestamp_us) {
  const std::uint32_t entry = next_entry_++;
  control_.clear();
  control_.push_back(kStart);
  AppendLittleEndian(control_, entry, 4);
  AppendString(control_, name);
  AppendString(control_, type);
  AppendString(control_, metadata);
  write(0, timestamp_us, control_);
  return entry;
}

void Writer::finish(std::uint32_t entry, std::uint64_t timestamp_us) {
  control_.clear();
  control_.push_back(kFinish);
  AppendLittleEndian(control_, entry, 4);
  write(0, timestamp_us, control_);
}

void Writer::set_metadata(std::uint32_t entry, std::string_view metadata,
                          std::uint64_t timestamp_us) {
  control_.clear();
  control_.push_back(kSetMetadata);
  AppendLittleEndian(control_, entry, 4);
  AppendString(control_, metadata);
  write(0, timestamp_us, control_);
}

void Writer::append(std::uint32_t entry, std::span<const std::uint8_t> payload,
                    std::uint64_t timestamp_us) {
  write(entry, timestamp_us, payload);
}

void Writer::write(std::uint32_t entry, std::uint64_t timestamp_us,
                   std::span<const std::uint8_t> payload) {
  if (payload.size() > UINT32_MAX) {
    out_.setstate(std::ios::failbit);
    return;
  }
  record_.clear();
  AppendRecord(record_, entry, timestamp_us, payload);
  out_.write(reinterpret_cast<const char*>(record_.data()),
             static_cast<std::streamsize>(record_.size()));
}

}  // namespace talos::wpilog

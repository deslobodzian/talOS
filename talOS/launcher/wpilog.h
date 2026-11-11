#pragma once

// A WPILib DataLog (.wpilog, format version 1.0) writer, for tlog_to_wpilog.
// Self-contained: no WPILib dependency, and every multi-byte value is
// written little-endian byte by byte, never by copying a host struct.
//
//   file      "WPILOG", u16 version 0x0100, u32 extra-header length, the
//             extra header (free text)
//   record    one header byte, then the entry id (1-4 bytes), the payload
//             size (1-4 bytes), the timestamp in microseconds (1-8 bytes),
//             each little-endian in the fewest bytes that hold it, then the
//             payload. The header byte's bits 0-1, 2-3 and 4-6 are those
//             three widths minus one.
//   entry 0   control records: Start (0, u32 entry, u32+name, u32+type,
//             u32+metadata), Finish (1, u32 entry), SetMetadata (2, u32
//             entry, u32+metadata). Strings are length-prefixed UTF-8.
//
// Payloads by type: "boolean" one byte 0/1, "int64" 8 bytes, "float" 4,
// "double" 8; an array type ("double[]", ...) is its elements back to back.

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <span>
#include <string_view>
#include <vector>

namespace talos::wpilog {

inline constexpr std::uint16_t kVersion = 0x0100;

// Appends one record (header byte, entry, size, timestamp, payload) to `out`.
// `payload` is at most UINT32_MAX bytes (Writer refuses a longer one).
void AppendRecord(std::vector<std::uint8_t>& out, std::uint32_t entry,
                  std::uint64_t timestamp_us,
                  std::span<const std::uint8_t> payload);

// Payload encoders: append `value` to `out` as its WPILOG type stores it.
void AppendBoolean(std::vector<std::uint8_t>& out, bool value);
void AppendInt64(std::vector<std::uint8_t>& out, std::int64_t value);
void AppendFloat(std::vector<std::uint8_t>& out, float value);
void AppendDouble(std::vector<std::uint8_t>& out, double value);

// Writes a .wpilog to a stream: the file header on construction, then one
// record per call. Entry ids are handed out from 1 in start() order.
class Writer {
 public:
  Writer(std::ostream& out, std::string_view extra_header);

  // Starts an entry and returns its id.
  std::uint32_t start(std::string_view name, std::string_view type,
                      std::string_view metadata, std::uint64_t timestamp_us);
  void finish(std::uint32_t entry, std::uint64_t timestamp_us);
  void set_metadata(std::uint32_t entry, std::string_view metadata,
                    std::uint64_t timestamp_us);
  // A data record of an entry start() returned.
  void append(std::uint32_t entry, std::span<const std::uint8_t> payload,
              std::uint64_t timestamp_us);

  // False once the stream has refused a write, or a payload was too long.
  bool ok() const { return out_.good(); }

 private:
  void write(std::uint32_t entry, std::uint64_t timestamp_us,
             std::span<const std::uint8_t> payload);

  std::ostream& out_;
  std::uint32_t next_entry_{1};
  std::vector<std::uint8_t> record_;
  std::vector<std::uint8_t> control_;
};

}  // namespace talos::wpilog

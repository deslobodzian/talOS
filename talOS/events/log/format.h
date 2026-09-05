#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <string_view>

#include "talOS/events/manifest.h"

// On-disk layout of a talOS dispatch log.
//
//   [FileHeader][ManifestEntry * manifest_count][Record ...]
//
// where a Record is a RecordHeader followed by payload_bytes of message bytes.
// Everything is fixed-size and little-endian, so the file can be mapped and
// walked without a parser, and a truncated tail costs at most one record.
//
// The format is versioned and never rewritten in place. Readers must reject a
// version they do not know rather than guessing.
namespace talos::event::log {

inline constexpr char FILE_MAGIC[8] = {'T', 'A', 'L', 'O', 'S', 'L', 'O', 'G'};
inline constexpr std::uint32_t FORMAT_VERSION = 1;

// Marks the start of every record so a damaged log can be resynchronised.
inline constexpr std::uint32_t RECORD_MAGIC = 0x54524543;  // "TREC"

#pragma pack(push, 1)

struct FileHeader {
  char magic[8];
  std::uint32_t version;
  std::uint32_t header_bytes;       // sizeof(FileHeader), for forward compat
  std::int64_t start_monotonic_ns;  // loop start on the recording timeline
  std::int64_t start_wall_ns;       // informational only, ignored by replay
  std::uint32_t manifest_count;
  std::uint32_t crc32;  // over the manifest block only
  char process_name[64];
};

struct ManifestEntry {
  std::uint16_t id;
  std::uint16_t kind;  // SourceKind
  std::uint32_t message_bytes;
  std::uint32_t alignment;
  std::uint32_t reserved;
  std::int64_t period_ns;
  std::int64_t offset_ns;
  char name[64];
};

struct RecordHeader {
  std::uint32_t magic;  // RECORD_MAGIC
  std::uint16_t kind;   // EventKind
  std::uint16_t source_id;
  std::uint64_t dispatch_index;
  std::int64_t event_time_ns;
  std::int64_t now_ns;
  std::uint64_t sequence;
  std::uint32_t aux;  // dropped count, FetchOutcome, WriteResult
  std::uint32_t payload_bytes;
  std::uint32_t crc32;  // header with crc32 zeroed, then payload
  std::uint32_t reserved;
};

#pragma pack(pop)

static_assert(sizeof(FileHeader) == 104);
static_assert(sizeof(ManifestEntry) == 96);
static_assert(sizeof(RecordHeader) == 56);

// CRC-32 (IEEE 802.3, reflected) computed with a small runtime table. Logs are
// written on the loop thread, so this stays branch-light and allocation free.
inline std::uint32_t crc32(std::span<const std::byte> bytes,
                           std::uint32_t seed = 0) {
  static const auto table = [] {
    std::array<std::uint32_t, 256> t{};
    for (std::uint32_t i = 0; i < 256; ++i) {
      std::uint32_t c = i;
      for (int k = 0; k < 8; ++k) {
        c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      }
      t[i] = c;
    }
    return t;
  }();

  std::uint32_t crc = ~seed;
  for (std::byte b : bytes) {
    crc = table[(crc ^ static_cast<std::uint8_t>(b)) & 0xFFu] ^ (crc >> 8);
  }
  return ~crc;
}

// Copies a name into a fixed field, truncating at MAX_SOURCE_NAME and always
// leaving it null terminated.
inline void write_name(char (&field)[64], std::string_view name) {
  const std::size_t length = std::min(name.size(), MAX_SOURCE_NAME);
  std::memset(field, 0, sizeof(field));
  std::memcpy(field, name.data(), length);
}

inline std::string read_name(const char (&field)[64]) {
  const std::size_t length = ::strnlen(field, sizeof(field));
  return std::string{field, length};
}

}  // namespace talos::event::log

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
//   [FileHeader][ManifestEntry * manifest_count][schema block][Record ...]
//
// The schema block carries each manifest entry's topic schema
// (talOS/schema/identity.h), in manifest order: its schema_name_bytes of
// name, then its schema_bfbs_bytes of reflection.Schema bytes (none for a
// timer, a byte topic or the hardware packet). So a log decodes to named
// fields without the build that wrote it, and replay compares schema
// identities, not just sizes.
//
// A Record is a RecordHeader followed by stored_bytes of message bytes.
// A payload is stored without its trailing zero bytes, which for fixed-size
// messages is mostly padding; readers zero-fill it back to payload_bytes, so
// what they hand out is byte-identical to what was recorded.
// Everything but the schema block is fixed-size and little-endian, so the
// records can be walked without a parser, and a truncated tail costs at most
// one record. A damaged record costs only itself: LogReader scans past it to
// the next RECORD_MAGIC whose header and checksum validate.
//
// A GAP record (EventKind::GAP) stands where the writer dropped records it
// could not take: a RecordHeader whose source_id is GAP_SOURCE_ID, whose
// dispatch_index is the first dropped record's, event_time_ns the earliest
// time a dropped record names and now_ns the time capture resumed, followed
// by a GapRecord.
//
// The format is versioned and never rewritten in place. Readers must reject a
// version they do not know rather than guessing: a build reads versions
// OLDEST_READABLE_VERSION to FORMAT_VERSION. A record kind an older reader
// would misread needs a new version.
namespace talos::event::log {

inline constexpr char FILE_MAGIC[8] = {'T', 'A', 'L', 'O', 'S', 'L', 'O', 'G'};
// 8: GAP records (EventKind::GAP) mark records the writer dropped and capture
// resumed after. 7: READ records note which watcher value a dispatch used.
// 6: RecordHeader::publish_time_ns preserves producer time separately from
// receipt/dispatch time. Version 5 introduced message schemas.
inline constexpr std::uint32_t FORMAT_VERSION = 8;
// A version-7 log is a version-8 log without a GAP record.
inline constexpr std::uint32_t OLDEST_READABLE_VERSION = 7;
inline constexpr std::uint16_t GAP_SOURCE_ID = 0xFFFF;
enum class GapCause : std::uint16_t { POOL_FULL = 1, RECORD_TOO_LARGE = 2 };
inline constexpr std::uint16_t GAP_SPLITS_DISPATCH = 1u << 0;
inline constexpr std::uint16_t GAP_CLOSED_AT_FLUSH = 1u << 1;

// Marks the start of every record so a damaged log can be resynchronised
// (LogReader); the record's checksum tells a real record from these bytes
// inside a payload.
inline constexpr std::uint32_t RECORD_MAGIC = 0x54524543;  // "TREC"

#pragma pack(push, 1)

struct FileHeader {
  char magic[8];
  std::uint32_t version;
  std::uint32_t header_bytes;  // sizeof(FileHeader), for forward compat
  std::uint64_t session_id;
  std::int64_t start_monotonic_ns;  // the loop's origin, recording timeline
  std::int64_t start_wall_ns;       // informational only, ignored by replay
  std::uint32_t manifest_count;
  std::uint32_t crc32;  // over the manifest block, then the schema block
  char process_name[64];
  std::uint64_t schema_bytes;  // size of the schema block
};

struct ManifestEntry {
  std::uint16_t id;
  std::uint16_t kind;  // SourceKind
  std::uint32_t message_bytes;
  std::uint32_t alignment;
  std::uint32_t armed;
  std::int64_t period_ns;
  std::int64_t offset_ns;
  char name[64];
  std::uint64_t schema_hash;
  std::uint32_t schema_name_bytes;
  std::uint32_t schema_bfbs_bytes;
};

struct RecordHeader {
  std::uint32_t magic;  // RECORD_MAGIC
  std::uint16_t kind;   // EventKind
  std::uint16_t source_id;
  std::uint64_t dispatch_index;
  std::int64_t event_time_ns;
  std::int64_t now_ns;
  std::int64_t publish_time_ns;  // producer time for MESSAGE and SEND
  std::uint64_t sequence;
  std::uint32_t aux;            // dropped count, WriteResult
  std::uint32_t payload_bytes;  // message size as recorded
  std::uint32_t crc32;          // header with crc32 zeroed, then stored bytes
  std::uint32_t stored_bytes;   // payload_bytes minus its trailing zero bytes
};

struct GapRecord {             // a GAP record's payload; 64 B, packed
  std::uint16_t cause;         // GapCause, the first record's
  std::uint16_t flags;         // GAP_SPLITS_DISPATCH | GAP_CLOSED_AT_FLUSH
  std::uint32_t reserved;      // zero
  std::int64_t first_lost_ns;  // now_ns of the first record dropped
  std::int64_t last_lost_ns;   // the latest now_ns a dropped record had
  std::uint64_t last_dispatch_index;  // the last dropped record's
  std::uint64_t records;     // records dropped (a held EXIT is not one)
  std::uint64_t bytes;       // their header and payload bytes, untrimmed
  std::uint64_t dispatches;  // of them, TIMER and MESSAGE records
  std::uint64_t sends;       // of them, SEND records (accepted or not)
};

#pragma pack(pop)

static_assert(sizeof(FileHeader) == 120);
static_assert(sizeof(ManifestEntry) == 112);
static_assert(sizeof(RecordHeader) == 64);
static_assert(sizeof(GapRecord) == 64);
// Every chunk and every segment holds one GAP record.
inline constexpr std::size_t GAP_RECORD_BYTES =
    sizeof(RecordHeader) + sizeof(GapRecord);

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

// Sets header.magic, payload_bytes, stored_bytes (payload less its trailing
// zero bytes) and crc32 (header with crc32 zeroed, then the stored bytes);
// returns the stored bytes. The one record encoder.
inline std::span<const std::byte> SealRecord(
    RecordHeader& header, std::span<const std::byte> payload) noexcept {
  // Fixed-size messages are mostly trailing padding. Only the bytes up to the
  // last non-zero one are stored; the reader restores the rest.
  std::size_t stored = payload.size();
  while (stored > 0 && payload[stored - 1] == std::byte{0}) --stored;
  header.magic = RECORD_MAGIC;
  header.payload_bytes = static_cast<std::uint32_t>(payload.size());
  header.stored_bytes = static_cast<std::uint32_t>(stored);
  header.crc32 = 0;
  payload = payload.first(stored);
  header.crc32 =
      crc32(payload,
            crc32(std::span<const std::byte>{
                reinterpret_cast<const std::byte*>(&header), sizeof(header)}));
  return payload;
}

// Copies a name into a fixed field, truncating at MAX_SOURCE_NAME and always
// leaving it null terminated.
inline void write_name(char (&field)[64], std::string_view name) {
  const std::size_t length = std::min(name.size(), MAX_SOURCE_NAME);
  std::memset(field, 0, sizeof(field));
  std::memcpy(field, name.data(), length);
}

// Where segment `segment` (0 first) of the byte-capped log at `log_path` is
// kept once a newer one has started (LogWriterOptions::max_bytes):
// `<stem>.segment<k>.tlog` beside it, so a session directory's `*.tlog`
// (FindLogFiles) includes it. Each segment is a complete log.
inline std::string LogSegmentPath(std::string_view log_path,
                                  std::uint64_t segment) {
  constexpr std::string_view kExtension = ".tlog";
  std::string_view stem = log_path;
  if (stem.ends_with(kExtension)) stem.remove_suffix(kExtension.size());
  return std::string{stem} + ".segment" + std::to_string(segment) +
         std::string{kExtension};
}

inline std::string read_name(const char (&field)[64]) {
  const std::size_t length = ::strnlen(field, sizeof(field));
  return std::string{field, length};
}

}  // namespace talos::event::log

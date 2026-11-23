#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "talOS/events/context.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/manifest.h"
#include "talOS/events/session_manifest.h"

namespace talos::event::log {
namespace {

class TempLogFile {
 public:
  TempLogFile() {
    path_ = ::testing::TempDir() + "/talos_log_fuzz_test_" +
            std::to_string(::getpid()) + ".tlog";
    fd_ = ::open(path_.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0600);
    if (fd_ < 0) {
      ADD_FAILURE() << "open failed for " << path_;
    }
  }
  ~TempLogFile() {
    if (fd_ >= 0) ::close(fd_);
    ::unlink(path_.c_str());
  }
  TempLogFile(const TempLogFile&) = delete;
  TempLogFile& operator=(const TempLogFile&) = delete;

  const std::string& path() const noexcept { return path_; }

  void Write(std::span<const std::uint8_t> bytes) const {
    ASSERT_GE(fd_, 0);
    ASSERT_EQ(::ftruncate(fd_, 0), 0);
    std::size_t written = 0;
    while (written < bytes.size()) {
      const ssize_t n =
          ::pwrite(fd_, bytes.data() + written, bytes.size() - written,
                   static_cast<off_t>(written));
      ASSERT_GT(n, 0);
      written += static_cast<std::size_t>(n);
    }
  }

 private:
  std::string path_;
  int fd_{-1};
};

void AppendBytes(std::vector<std::uint8_t>& out, const void* ptr,
                 std::size_t size) {
  const auto* p = reinterpret_cast<const std::uint8_t*>(ptr);
  out.insert(out.end(), p, p + size);
}

void AppendRecord(std::vector<std::uint8_t>& out, EventKind kind,
                  std::uint16_t source_id, std::uint64_t dispatch_index,
                  std::int64_t event_time_ns, std::int64_t now_ns,
                  std::span<const std::byte> payload) {
  RecordHeader header{};
  header.kind = static_cast<std::uint16_t>(kind);
  header.source_id = source_id;
  header.dispatch_index = dispatch_index;
  header.event_time_ns = event_time_ns;
  header.now_ns = now_ns;
  header.publish_time_ns = event_time_ns;
  header.sequence = dispatch_index;
  const auto stored = SealRecord(header, payload);
  AppendBytes(out, &header, sizeof(header));
  AppendBytes(out, stored.data(), stored.size());
}

void RepairCrcs(std::vector<std::uint8_t>& bytes) {
  if (bytes.size() >= sizeof(FileHeader)) {
    FileHeader fh{};
    std::memcpy(&fh, bytes.data(), sizeof(fh));
    if (std::memcmp(fh.magic, FILE_MAGIC, sizeof(FILE_MAGIC)) == 0) {
      const std::uint64_t manifest_bytes =
          std::uint64_t{fh.manifest_count} * sizeof(ManifestEntry);
      const std::uint64_t m_off = fh.header_bytes;
      if (m_off >= sizeof(FileHeader) && m_off <= bytes.size() &&
          manifest_bytes <= bytes.size() - m_off) {
        const std::uint64_t s_off = m_off + manifest_bytes;
        if (fh.schema_bytes <= bytes.size() - s_off) {
          const auto* base = reinterpret_cast<const std::byte*>(bytes.data());
          fh.crc32 = crc32(
              {base + s_off, static_cast<std::size_t>(fh.schema_bytes)},
              crc32({base + m_off, static_cast<std::size_t>(manifest_bytes)}));
          std::memcpy(bytes.data(), &fh, sizeof(fh));
        }
      }
    }
  }
  for (std::size_t pos = sizeof(FileHeader);
       pos + sizeof(RecordHeader) <= bytes.size(); ++pos) {
    RecordHeader rh{};
    std::memcpy(&rh, bytes.data() + pos, sizeof(rh));
    if (rh.magic == RECORD_MAGIC &&
        rh.stored_bytes <= bytes.size() - (pos + sizeof(RecordHeader))) {
      rh.crc32 = 0;
      const auto* base = reinterpret_cast<const std::byte*>(bytes.data() + pos);
      const std::uint32_t hcrc = crc32({base, sizeof(RecordHeader)});
      rh.crc32 = crc32({base + sizeof(RecordHeader), rh.stored_bytes}, hcrc);
      std::memcpy(bytes.data() + pos, &rh, sizeof(rh));
      pos += sizeof(RecordHeader) + rh.stored_bytes - 1;
    }
  }
}

std::vector<std::uint8_t> BuildSeedV8Log(bool with_schemas, bool with_damage) {
  std::vector<ManifestEntry> entries(3);
  entries[0].id = 0;
  entries[0].kind = static_cast<std::uint16_t>(SourceKind::TIMER);
  entries[0].period_ns = 1'000'000;
  write_name(entries[0].name, "tick");

  entries[1].id = 1;
  entries[1].kind = static_cast<std::uint16_t>(SourceKind::WATCHER);
  entries[1].message_bytes = 32;
  entries[1].alignment = 8;
  entries[1].schema_hash = 0x123456789abcdef0ULL;
  write_name(entries[1].name, "/a/state");

  entries[2].id = 2;
  entries[2].kind = static_cast<std::uint16_t>(SourceKind::SENDER);
  entries[2].message_bytes = 64;
  entries[2].alignment = 8;
  entries[2].schema_hash = 0xfeedfacecafebeefULL;
  write_name(entries[2].name, "/a/command");

  std::vector<std::byte> schemas;
  if (with_schemas) {
    const std::string_view sname = "test.Sample";
    entries[1].schema_name_bytes = static_cast<std::uint32_t>(sname.size());
    entries[1].schema_bfbs_bytes = 4;
    for (char c : sname) schemas.push_back(static_cast<std::byte>(c));
    for (int i = 0; i < 4; ++i) schemas.push_back(std::byte{0x42});
  }

  FileHeader fh{};
  std::memcpy(fh.magic, FILE_MAGIC, sizeof(FILE_MAGIC));
  fh.version = FORMAT_VERSION;
  fh.header_bytes = sizeof(FileHeader);
  fh.session_id = 42;
  fh.start_monotonic_ns = 1000;
  fh.start_wall_ns = 2000;
  fh.manifest_count = static_cast<std::uint32_t>(entries.size());
  fh.schema_bytes = schemas.size();
  write_name(fh.process_name, "fuzz_node");
  fh.crc32 =
      crc32(schemas, crc32({reinterpret_cast<const std::byte*>(entries.data()),
                            entries.size() * sizeof(ManifestEntry)}));

  std::vector<std::uint8_t> out;
  AppendBytes(out, &fh, sizeof(fh));
  AppendBytes(out, entries.data(), entries.size() * sizeof(ManifestEntry));
  AppendBytes(out, schemas.data(), schemas.size());

  AppendRecord(out, EventKind::TIMER, 0, 1, 1000, 1010, {});
  std::vector<std::byte> msg(32, std::byte{0});
  msg[0] = std::byte{0xaa};
  msg[15] = std::byte{0x55};
  AppendRecord(out, EventKind::MESSAGE, 1, 2, 2000, 2020, msg);

  if (with_damage) {
    out.insert(out.end(), 13, 0xde);
  }

  GapRecord gr{};
  gr.cause = static_cast<std::uint16_t>(GapCause::POOL_FULL);
  gr.flags = GAP_SPLITS_DISPATCH;
  gr.records = 5;
  gr.dispatches = 2;
  gr.sends = 1;
  gr.bytes = 512;
  gr.last_dispatch_index = 4;
  std::vector<std::byte> gp(sizeof(gr));
  std::memcpy(gp.data(), &gr, sizeof(gr));
  AppendRecord(out, EventKind::GAP, GAP_SOURCE_ID, 3, 2500, 3000, gp);

  std::vector<std::byte> snd(64, std::byte{0x11});
  AppendRecord(out, EventKind::SEND, 2, 5, 3000, 3050, snd);
  AppendRecord(out, EventKind::EXIT, 0, 6, 4000, 4050, {});
  return out;
}

void ExerciseLogInput(const TempLogFile& file,
                      std::span<const std::uint8_t> bytes) {
  file.Write(bytes);

  bool lr_ok = false;
  std::size_t lr_count = 0;
  std::size_t lr_gaps = 0;
  LogDamage lr_damage{};
  std::vector<std::uint64_t> lr_offsets;
  try {
    LogReader lr{file.path()};
    LogReader::Record rec;
    while (lr.next(rec)) {
      lr_offsets.push_back(rec.offset);
      if (const auto gap = GapOf(rec.header, rec.payload)) {
        EXPECT_FALSE(DescribeGap(*gap).empty());
      }
    }
    lr_count = lr.record_count();
    lr_gaps = lr.gaps();
    lr_damage = lr.damage();
    (void)DescribeDamage(lr_damage);
    lr_ok = true;
  } catch (const std::runtime_error&) {
    lr_ok = false;
  }

  bool wr_ok = false;
  try {
    WindowedLogReader wr{file.path()};
    LogReader::Record rec;
    std::size_t idx = 0;
    while (wr.next(rec)) {
      if (lr_ok) {
        ASSERT_LT(idx, lr_offsets.size());
        EXPECT_EQ(rec.offset, lr_offsets[idx]);
      }
      ++idx;
    }
    if (lr_ok) {
      EXPECT_EQ(idx, lr_count);
      EXPECT_EQ(wr.record_count(), lr_count);
      EXPECT_EQ(wr.gaps(), lr_gaps);
      EXPECT_EQ(wr.damage().spans, lr_damage.spans);
      EXPECT_EQ(wr.damage().skipped_bytes, lr_damage.skipped_bytes);
      EXPECT_EQ(wr.damage().tail_bytes, lr_damage.tail_bytes);
      if (!lr_offsets.empty()) {
        wr.seek(lr_offsets.back());
        LogReader::Record resumed;
        EXPECT_TRUE(wr.next(resumed));
      }
    }
    wr_ok = true;
  } catch (const std::runtime_error&) {
    wr_ok = false;
  }
  EXPECT_EQ(lr_ok, wr_ok);
}

// Regression: DescribeGap used signed int64_t subtraction `resumed_ns -
// start_ns` on untrusted RecordHeader timestamps, triggering UBSan
// signed-integer-overflow when a GAP record had extreme int64_t values (found
// by libFuzzer).
TEST(LogFuzzTest, DescribeGapExtremeTimestampsDoNotOverflow) {
  constexpr std::int64_t kMin = std::numeric_limits<std::int64_t>::min();
  constexpr std::int64_t kMax = std::numeric_limits<std::int64_t>::max();

  for (const auto& [start_ns, resumed_ns] :
       std::initializer_list<std::pair<std::int64_t, std::int64_t>>{
           {kMin, 0},
           {kMin, kMax},
           {kMax, kMin},
           {0, kMin},
       }) {
    RecordHeader header{};
    header.kind = static_cast<std::uint16_t>(EventKind::GAP);
    header.source_id = GAP_SOURCE_ID;
    header.dispatch_index = 1;
    header.event_time_ns = start_ns;
    header.now_ns = resumed_ns;
    header.payload_bytes = sizeof(GapRecord);
    const auto gap = GapOf(header, {});
    ASSERT_TRUE(gap.has_value());
    EXPECT_FALSE(DescribeGap(*gap).empty());
  }

  // Also end-to-end through a .tlog file read by LogReader and
  // WindowedLogReader.
  TempLogFile file;
  FileHeader fh{};
  std::memcpy(fh.magic, FILE_MAGIC, sizeof(FILE_MAGIC));
  fh.version = FORMAT_VERSION;
  fh.header_bytes = sizeof(FileHeader);
  std::vector<std::uint8_t> bytes;
  AppendBytes(bytes, &fh, sizeof(fh));
  GapRecord gr{};
  gr.cause = static_cast<std::uint16_t>(GapCause::POOL_FULL);
  AppendRecord(bytes, EventKind::GAP, GAP_SOURCE_ID, 1, kMin, 0,
               {reinterpret_cast<const std::byte*>(&gr), sizeof(gr)});
  file.Write(bytes);

  LogReader lr{file.path()};
  LogReader::Record rec;
  ASSERT_TRUE(lr.next(rec));
  const auto gap = GapOf(rec.header, rec.payload);
  ASSERT_TRUE(gap.has_value());
  EXPECT_FALSE(DescribeGap(*gap).empty());
}

// Regression: WindowedLogReader::seek must reject extreme/overflowing offsets
// without unsigned wraparound in FileSource::view.
TEST(LogFuzzTest, WindowedLogReaderSeekExtremeOffsetsDoNotOverflow) {
  TempLogFile file;
  const std::vector<std::uint8_t> bytes = BuildSeedV8Log(true, false);
  file.Write(bytes);

  WindowedLogReader reader{file.path()};
  for (const std::uint64_t bad :
       {std::uint64_t{0}, std::uint64_t{1}, sizeof(FileHeader) - 1,
        std::uint64_t{bytes.size()},
        std::numeric_limits<std::uint64_t>::max() - 10,
        std::numeric_limits<std::uint64_t>::max()}) {
    EXPECT_THROW(reader.seek(bad), std::invalid_argument);
  }
}

// Regression: empty schema block with manifest entries must not pass nullptr to
// pointer arithmetic or std::string::assign.
TEST(LogFuzzTest, EmptySchemaBlockWithManifestEntries) {
  TempLogFile file;
  const std::vector<std::uint8_t> bytes = BuildSeedV8Log(false, false);
  ExerciseLogInput(file, bytes);
}

// Regression: truncated trailing backslash in a JSON string must not advance
// past text_.size() before throwing ManifestError.
TEST(LogFuzzTest, TruncatedManifestStringEscapeThrowsWithoutOob) {
  for (const std::string_view text :
       {std::string_view{"\"\\"}, std::string_view{"{\"a\":\"\\"},
        std::string_view{"{\"nodes\":[{\"log_paths\":[\"\\"}}) {
    EXPECT_THROW(session_manifest::Parse(text),
                 session_manifest::ManifestError);
  }
}

TEST(LogFuzzTest, TlogSeededMutations) {
  TempLogFile file;
  std::vector<std::vector<std::uint8_t>> seeds;
  seeds.push_back(BuildSeedV8Log(true, false));
  seeds.push_back(BuildSeedV8Log(false, true));
  {
    FileHeader fh{};
    std::memcpy(fh.magic, FILE_MAGIC, sizeof(FILE_MAGIC));
    fh.version = OLDEST_READABLE_VERSION;
    fh.header_bytes = sizeof(FileHeader);
    std::vector<std::uint8_t> v7;
    AppendBytes(v7, &fh, sizeof(fh));
    seeds.push_back(std::move(v7));
  }

  std::mt19937 rng(0x74616c6f);
  for (const auto& seed : seeds) {
    // Key header/record boundary truncations.
    for (std::size_t len = 0; len <= std::min<std::size_t>(seed.size(), 256);
         len += 17) {
      ExerciseLogInput(file, std::span{seed}.first(len));
    }
    ExerciseLogInput(file, seed);

    // Deterministic raw + CRC-repaired mutations.
    for (int iter = 0; iter < 150; ++iter) {
      auto mutated = seed;
      const unsigned edits = 1 + (rng() % 6);
      for (unsigned e = 0; e < edits; ++e) {
        const std::size_t pos = rng() % (mutated.size() + 1);
        switch (rng() % 4) {
          case 0:
            if (pos < mutated.size()) {
              mutated[pos] ^= static_cast<std::uint8_t>(1u << (rng() % 8));
            }
            break;
          case 1:
            mutated.insert(mutated.begin() + static_cast<std::ptrdiff_t>(pos),
                           static_cast<std::uint8_t>(rng()));
            break;
          case 2:
            if (pos < mutated.size()) {
              mutated.erase(mutated.begin() + static_cast<std::ptrdiff_t>(pos));
            }
            break;
          case 3:
            if (pos < mutated.size()) {
              mutated[pos] = static_cast<std::uint8_t>(rng());
            }
            break;
          default:
            break;
        }
      }
      if (iter % 2 == 0) {
        RepairCrcs(mutated);
      }
      ExerciseLogInput(file, mutated);
    }
  }
}

TEST(LogFuzzTest, SessionManifestSeededMutations) {
  std::vector<std::string> seeds = {
      R"({"session_id":123,"hardware_protocol_version":7,"config_path":"/a/b.toml","simulation":true,"start_wall_ns":100,"end_wall_ns":200,"extra":[1,-2.5e3,null,false,"x\n\u00e9\uD83D\uDE00"],"nodes":[{"log_gap":true,"log_capped":false,"recorder_failed":false,"log_paths":["a.tlog"],"gap_logs":[],"recorder_failed_logs":["b.tlog"]}]})",
      R"({"nodes":[{"log_gap":false}]})",
  };
  {
    std::string deep;
    for (int i = 0; i < 40; ++i) deep += "{\"a\":";
    deep += "1";
    for (int i = 0; i < 40; ++i) deep += "}";
    seeds.push_back(std::move(deep));
  }

  std::mt19937 rng(0x6d616e69);
  for (const auto& seed : seeds) {
    for (std::size_t len = 0; len <= seed.size(); len += 7) {
      try {
        (void)session_manifest::Parse(std::string_view{seed}.substr(0, len));
      } catch (const session_manifest::ManifestError& error) {
        (void)error;
      }
    }
    for (int iter = 0; iter < 512; ++iter) {
      std::string mutated = seed;
      const unsigned edits = 1 + (rng() % 6);
      for (unsigned e = 0; e < edits; ++e) {
        const std::size_t pos = rng() % (mutated.size() + 1);
        switch (rng() % 4) {
          case 0:
            if (pos < mutated.size()) {
              mutated[pos] =
                  static_cast<char>(static_cast<unsigned char>(mutated[pos]) ^
                                    (1u << (rng() % 8)));
            }
            break;
          case 1:
            mutated.insert(pos, 1, static_cast<char>(rng()));
            break;
          case 2:
            if (pos < mutated.size()) mutated.erase(pos, 1);
            break;
          case 3:
            if (pos < mutated.size()) mutated[pos] = static_cast<char>(rng());
            break;
          default:
            break;
        }
      }
      try {
        (void)session_manifest::Parse(mutated);
      } catch (const session_manifest::ManifestError& error) {
        (void)error;
      }
    }
  }
}

// REQ-OPS-14, REQ-OPS-42: session_manifest::ReadText must bound how much of
// manifest.json it loads (4 MiB, matching Studio's kMaxManifestBytes) and fail
// closed on oversized or non-regular files while keeping empty and valid
// manifests readable.
TEST(LogFuzzTest, SessionManifestReadTextEnforcesSizeBoundAndRegularFiles) {
  namespace fs = std::filesystem;
  constexpr std::size_t kExpectedMaxManifestBytes = std::size_t{4} << 20;
  static_assert(session_manifest::kMaxManifestBytes ==
                kExpectedMaxManifestBytes);
  const fs::path base = fs::path{::testing::TempDir()} /
                        ("talos_manifest_bound_" + std::to_string(::getpid()));
  std::error_code ec;
  fs::remove_all(base, ec);
  ASSERT_TRUE(fs::create_directories(base, ec)) << ec.message();

  // Missing manifest.json -> nullopt.
  EXPECT_FALSE(session_manifest::ReadText(base).has_value());

  // Empty 0-byte manifest.json -> returns "" (so callers parse and fail
  // closed).
  {
    std::ofstream empty{base / session_manifest::kFileName, std::ios::binary};
    ASSERT_TRUE(empty.good());
    empty.close();
    const auto text = session_manifest::ReadText(base);
    ASSERT_TRUE(text.has_value());
    EXPECT_TRUE(text->empty());
  }

  // Exact upper bound (4 MiB) -> accepted in full.
  {
    const std::string payload(kExpectedMaxManifestBytes, ' ');
    std::ofstream out{base / session_manifest::kFileName, std::ios::binary};
    out.write(payload.data(), static_cast<std::streamsize>(payload.size()));
    ASSERT_TRUE(out.good());
    out.close();
    const auto text = session_manifest::ReadText(base);
    ASSERT_TRUE(text.has_value());
    EXPECT_EQ(text->size(), kExpectedMaxManifestBytes);
  }

  // Oversized (4 MiB + 1 byte) -> rejected (nullopt).
  {
    std::ofstream out{base / session_manifest::kFileName,
                      std::ios::binary | std::ios::app};
    out.put('x');
    ASSERT_TRUE(out.good());
    out.close();
    EXPECT_FALSE(session_manifest::ReadText(base).has_value());
  }

  // Directory named manifest.json -> rejected (nullopt), not empty string.
  fs::remove(base / session_manifest::kFileName, ec);
  ASSERT_TRUE(fs::create_directory(base / session_manifest::kFileName, ec));
  EXPECT_FALSE(session_manifest::ReadText(base).has_value());

  // FIFO named manifest.json -> rejected (nullopt) without blocking.
  fs::remove(base / session_manifest::kFileName, ec);
  ASSERT_EQ(::mkfifo((base / session_manifest::kFileName).c_str(), 0600), 0);
  EXPECT_FALSE(session_manifest::ReadText(base).has_value());

  fs::remove_all(base, ec);
}

}  // namespace
}  // namespace talos::event::log

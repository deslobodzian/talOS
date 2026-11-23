#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iterator>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "talOS/events/log/gap_testing.h"
#include "talOS/events/log/log_file_sink.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/process/durable_sync.h"
#include "talOS/process/process.h"

namespace talos::event::log {
namespace {

std::string TempPath(std::string_view suffix) {
  return ::testing::TempDir() + "/talos_log_test_" +
         std::to_string(::getpid()) + "_" + std::string{suffix} + ".tlog";
}

// A periodic sync interval short enough that a test waiting for one finishes
// in milliseconds, and still long against one write and fsync.
constexpr auto kShortSyncInterval = std::chrono::milliseconds{10};

LogWriter::Options ShortSyncOptions() {
  LogWriter::Options options{4096, 2, /*background=*/true};
  options.sync_interval = kShortSyncInterval;
  return options;
}

// Polls `done` until it holds. The deadline only bounds a hang
// (docs/TESTING.md §6.3); no test asserts how long a wait took.
template <typename Done>
bool Eventually(Done done) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds{10};
  while (!done()) {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  return true;
}

Manifest MakeManifest() {
  Manifest manifest;

  Registration timer;
  timer.id = 0;
  timer.kind = SourceKind::TIMER;
  timer.name = "tick";
  timer.message_bytes = 8;
  timer.alignment = 8;
  timer.period_ns = 1'000'000;
  timer.offset_ns = 0;
  manifest.push_back(timer);

  Registration watcher;
  watcher.id = 1;
  watcher.kind = SourceKind::WATCHER;
  // Exactly MAX_SOURCE_NAME characters: must survive round trip untruncated.
  watcher.name = std::string(MAX_SOURCE_NAME, 'q');
  watcher.message_bytes = 128;
  watcher.alignment = 4;
  watcher.period_ns = 0;
  watcher.offset_ns = 0;
  manifest.push_back(watcher);

  Registration sender;
  sender.id = 2;
  sender.kind = SourceKind::SENDER;
  sender.name = "cmd_topic";
  sender.message_bytes = 32;
  sender.alignment = 4;
  sender.period_ns = 0;
  sender.offset_ns = 0;
  manifest.push_back(sender);

  // A struct topic with reflection bytes (any bytes: the log does not parse
  // them) and a byte topic with none.
  manifest[1].schema = {"log_test.Sample", 0x0123456789abcdefULL,
                        std::string{"\x00\x01"
                                    "bfbs\xff",
                                    7}};
  manifest[2].schema = schema::Bytes();

  return manifest;
}

// Where the first record begins in a log of MakeManifest().
std::size_t RecordsOffset() {
  std::size_t offset = sizeof(FileHeader);
  for (const auto& reg : MakeManifest()) {
    offset +=
        sizeof(ManifestEntry) + reg.schema.name.size() + reg.schema.bfbs.size();
  }
  return offset;
}

std::vector<std::byte> MakePayload(std::size_t size, int seed) {
  std::vector<std::byte> payload(size);
  for (std::size_t i = 0; i < size; ++i) {
    payload[i] = static_cast<std::byte>((seed + static_cast<int>(i)) % 256);
  }
  return payload;
}

// What the writer stores of a payload: everything up to its last non-zero byte.
std::size_t StoredSize(const std::vector<std::byte>& payload) {
  std::size_t size = payload.size();
  while (size > 0 && payload[size - 1] == std::byte{0}) --size;
  return size;
}

struct ExpectedRecord {
  std::uint16_t kind{};
  std::uint16_t source_id{};
  std::uint64_t dispatch_index{};
  std::int64_t event_time_ns{};
  std::int64_t now_ns{};
  std::uint64_t sequence{};
  std::uint32_t aux{};
  std::vector<std::byte> payload;
  std::int64_t publish_time_ns{};
};

void ExpectRecordEquals(const ExpectedRecord& expected,
                        const LogReader::Record& actual) {
  EXPECT_EQ(actual.header.kind, expected.kind);
  EXPECT_EQ(actual.header.source_id, expected.source_id);
  EXPECT_EQ(actual.header.dispatch_index, expected.dispatch_index);
  EXPECT_EQ(actual.header.event_time_ns, expected.event_time_ns);
  EXPECT_EQ(actual.header.now_ns, expected.now_ns);
  EXPECT_EQ(actual.header.publish_time_ns, expected.publish_time_ns);
  EXPECT_EQ(actual.header.sequence, expected.sequence);
  EXPECT_EQ(actual.header.aux, expected.aux);
  ASSERT_EQ(actual.payload.size(), expected.payload.size());
  if (!expected.payload.empty()) {
    EXPECT_EQ(0, std::memcmp(actual.payload.data(), expected.payload.data(),
                             expected.payload.size()));
  }
}

// EXIT describes loop completion, not process teardown. Consumers must not
// infer that IPC writer claims or other process resources have been released.
TEST(LogWriter, ExitIsReadableBeforeWriterDestruction) {
  const std::string path = TempPath("live_exit");
  {
    LogWriter writer{path, "live_exit"};
    writer.start(MakeManifest(), MonotonicTime::from_nanos(0));
    Context context{};
    writer.finish(context);
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();

    LogReader reader{path};
    LogReader::Record record;
    ASSERT_TRUE(reader.next(record));
    EXPECT_EQ(record.header.kind, static_cast<std::uint16_t>(EventKind::EXIT));
    EXPECT_FALSE(reader.next(record));
    // Still alive, with shutdown/close work outstanding despite readable EXIT.
    writer.flush();
    EXPECT_FALSE(writer.failed()) << writer.error();
  }
  std::remove(path.c_str());
}

// Session id the round-trip fixture is written with.
constexpr std::uint64_t kFixtureSession = 0x123456789ABCDEF0ULL;

// Writes the round-trip fixture used by several tests: a manifest, ~50
// TIMER/MESSAGE/SEND records with varying payload sizes (including
// zero-length ones), and a final EXIT. Returns the expected records in
// on-disk order so callers can verify the reader against them.
std::vector<ExpectedRecord> WriteFixtureWithOptions(
    const std::string& path, LogWriter::Options options) {
  LogWriter writer{path, "talos_log_test", std::move(options), kFixtureSession};
  const Manifest manifest = MakeManifest();
  const MonotonicTime start_time = MonotonicTime::from_nanos(42);
  writer.start(manifest, start_time);

  std::vector<ExpectedRecord> expected;

  for (int i = 0; i < 50; ++i) {
    const std::uint16_t source_id = static_cast<std::uint16_t>(i % 3);
    const std::size_t payload_size = static_cast<std::size_t>((i * 7) % 37);
    const std::vector<std::byte> payload = MakePayload(payload_size, i);

    Context context;
    context.kind = (i % 2 == 0) ? EventKind::TIMER : EventKind::MESSAGE;
    context.source_id = source_id;
    context.dispatch_index = static_cast<std::uint64_t>(i);
    context.event_time =
        MonotonicTime::from_nanos(static_cast<std::int64_t>(i) * 1000);
    context.now = MonotonicTime::from_nanos(i * 1000 + (i % 5) * 10);
    context.publish_time = context.kind == EventKind::MESSAGE
                               ? MonotonicTime::from_nanos(i * 1000 - 500)
                               : MonotonicTime{};
    context.sequence = static_cast<std::uint64_t>(i);
    context.dropped = static_cast<std::uint64_t>(i % 3);

    writer.dispatch(context, payload);
    expected.push_back({static_cast<std::uint16_t>(context.kind), source_id,
                        context.dispatch_index, context.event_time.nanos(),
                        context.now.nanos(), context.sequence,
                        static_cast<std::uint32_t>(context.dropped), payload,
                        context.publish_time.nanos()});

    if (i % 5 == 2) {
      const std::size_t send_payload_size =
          static_cast<std::size_t>((i * 11) % 41);
      const std::vector<std::byte> send_payload =
          MakePayload(send_payload_size, i + 2);
      const std::uint32_t status = static_cast<std::uint32_t>(i);
      writer.send(context, source_id, send_payload, status,
                  /*sequence=*/3000 + i);
      expected.push_back({static_cast<std::uint16_t>(EventKind::SEND),
                          source_id, context.dispatch_index,
                          context.event_time.nanos(), context.now.nanos(),
                          static_cast<std::uint64_t>(3000 + i), status,
                          send_payload, context.now.nanos()});
    }
  }

  Context exit_context;
  exit_context.kind = EventKind::EXIT;
  exit_context.source_id = 0;
  exit_context.dispatch_index = 50;
  exit_context.event_time = MonotonicTime::from_nanos(50'000);
  exit_context.now = MonotonicTime::from_nanos(50'010);
  exit_context.sequence = 50;
  exit_context.dropped = 0;
  writer.finish(exit_context);
  expected.push_back({static_cast<std::uint16_t>(EventKind::EXIT),
                      exit_context.source_id,
                      exit_context.dispatch_index,
                      exit_context.event_time.nanos(),
                      exit_context.now.nanos(),
                      exit_context.sequence,
                      static_cast<std::uint32_t>(exit_context.dropped),
                      {}});

  writer.flush();
  EXPECT_FALSE(writer.failed()) << writer.error();
  return expected;
}

// The original synchronous fixture: one buffer, written inline.
std::vector<ExpectedRecord> WriteFixture(const std::string& path,
                                         std::size_t buffer_bytes) {
  return WriteFixtureWithOptions(
      path, LogWriter::Options{buffer_bytes, 1, /*background=*/false});
}

TEST(LogRoundTrip, AllFieldsAndManifestMatch) {
  const std::string path = TempPath("roundtrip");
  const std::vector<ExpectedRecord> expected =
      WriteFixture(path, LogWriter::kDefaultBufferBytes);

  LogReader reader{path};
  const Manifest manifest = MakeManifest();
  ASSERT_EQ(reader.manifest().size(), manifest.size());
  for (std::size_t i = 0; i < manifest.size(); ++i) {
    EXPECT_EQ(reader.manifest()[i], manifest[i]);
    EXPECT_EQ(reader.manifest()[i].schema.bfbs, manifest[i].schema.bfbs);
  }
  EXPECT_EQ(reader.process_name(), "talos_log_test");
  EXPECT_EQ(reader.start_time(), MonotonicTime::from_nanos(42));
  EXPECT_GT(reader.start_wall_ns(), 0);

  LogReader::Record record;
  std::size_t index = 0;
  while (reader.next(record)) {
    ASSERT_LT(index, expected.size());
    ExpectRecordEquals(expected[index], record);
    ++index;
  }
  EXPECT_EQ(index, expected.size());
  EXPECT_EQ(reader.record_count(), expected.size());
  EXPECT_FALSE(reader.truncated());
  EXPECT_EQ(reader.format_version(), 8u);
  EXPECT_EQ(reader.session_id(), kFixtureSession);

  ::unlink(path.c_str());
}

TEST(LogRoundTrip, ManifestPreallocatesLargeMessages) {
  const std::string path = TempPath("smallbuffer");
  // 512 bytes is smaller than the manifest block plus several records, and
  // one payload below is chosen to exceed it outright.
  LogWriter writer{path, "small_buffer_writer", 512};
  Manifest manifest = MakeManifest();
  manifest[0].message_bytes = 1000;
  writer.start(manifest, MonotonicTime::from_nanos(7));

  std::vector<ExpectedRecord> expected;
  for (int i = 0; i < 10; ++i) {
    Context context;
    context.kind = EventKind::TIMER;
    context.source_id = 0;
    context.dispatch_index = static_cast<std::uint64_t>(i);
    context.event_time =
        MonotonicTime::from_nanos(static_cast<std::int64_t>(i) * 100);
    context.now = MonotonicTime::from_nanos(i * 100 + 5);
    context.sequence = static_cast<std::uint64_t>(i);
    context.dropped = 0;

    // One record (i == 3) is bigger than the entire 512-byte buffer.
    const std::size_t payload_size = (i == 3) ? 1000 : 20;
    const std::vector<std::byte> payload = MakePayload(payload_size, i);
    writer.dispatch(context, payload);
    expected.push_back({static_cast<std::uint16_t>(context.kind), 0,
                        context.dispatch_index, context.event_time.nanos(),
                        context.now.nanos(), context.sequence, 0, payload});
  }
  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();

  LogReader reader{path};
  LogReader::Record record;
  std::size_t index = 0;
  while (reader.next(record)) {
    ASSERT_LT(index, expected.size());
    ExpectRecordEquals(expected[index], record);
    ++index;
  }
  EXPECT_EQ(index, expected.size());
  EXPECT_FALSE(reader.truncated());

  ::unlink(path.c_str());
}

// Flips one byte and returns the old value, so callers can distinguish "no
// such offset" bugs in the test itself from genuine corruption behaviour.
void FlipByte(const std::string& path, std::size_t offset) {
  std::fstream file{path, std::ios::in | std::ios::out | std::ios::binary};
  ASSERT_TRUE(file.is_open());
  file.seekg(static_cast<std::streamoff>(offset));
  char byte = 0;
  file.read(&byte, 1);
  byte = static_cast<char>(~byte);
  file.seekp(static_cast<std::streamoff>(offset));
  file.write(&byte, 1);
}

std::size_t FileSize(const std::string& path) {
  std::ifstream file{path, std::ios::binary | std::ios::ate};
  return static_cast<std::size_t>(file.tellg());
}

// A fixed-size message is mostly trailing zero padding. The file stores each
// payload only up to its last non-zero byte, and the reader hands back the
// full recorded size, so nothing downstream can tell.
TEST(LogPadding, TrailingZerosAreNotStoredButReadBack) {
  const std::string path = TempPath("padding");
  constexpr std::size_t kBytes = 128;  // the watcher's message size

  std::vector<std::byte> sparse(kBytes);  // an interior zero, then padding
  sparse[0] = std::byte{1};
  sparse[2] = std::byte{3};
  const std::vector<std::byte> empty(kBytes);
  const std::vector<std::byte> dense = MakePayload(kBytes, 1);
  ASSERT_NE(dense.back(), std::byte{0});
  const std::vector<std::vector<std::byte>> payloads{sparse, empty, dense};

  {
    LogWriter writer{path, "padding", LogWriter::kDefaultBufferBytes};
    writer.start(MakeManifest(), MonotonicTime::from_nanos(1));
    for (std::size_t i = 0; i < payloads.size(); ++i) {
      Context context;
      context.kind = EventKind::MESSAGE;
      context.source_id = 1;
      context.dispatch_index = i + 1;
      writer.dispatch(context, payloads[i]);
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }

  EXPECT_EQ(FileSize(path),
            RecordsOffset() + 3 * sizeof(RecordHeader) + 3 + 0 + kBytes);

  LogReader reader{path};
  LogReader::Record record;
  const std::uint32_t stored[] = {3, 0, kBytes};
  for (std::size_t i = 0; i < payloads.size(); ++i) {
    ASSERT_TRUE(reader.next(record)) << "record " << i;
    EXPECT_EQ(record.header.payload_bytes, kBytes);
    EXPECT_EQ(record.header.stored_bytes, stored[i]);
    ASSERT_EQ(record.payload.size(), kBytes);
    EXPECT_TRUE(std::equal(record.payload.begin(), record.payload.end(),
                           payloads[i].begin()))
        << "record " << i;
  }
  EXPECT_FALSE(reader.next(record));
  EXPECT_FALSE(reader.truncated());

  ::unlink(path.c_str());
}

// A header whose checksum is right but which claims to store more bytes than
// its payload holds came from a broken writer; the reader must not guess.
TEST(LogPadding, StoredBytesBeyondThePayloadIsNotARecord) {
  const std::string path = TempPath("overstored");
  {
    LogWriter writer{path, "overstored", LogWriter::kDefaultBufferBytes};
    writer.start(MakeManifest(), MonotonicTime::from_nanos(1));
    Context context;
    context.kind = EventKind::MESSAGE;
    context.source_id = 1;
    context.dispatch_index = 1;
    writer.dispatch(context, MakePayload(8, 1));
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }

  const std::size_t offset = RecordsOffset();
  std::fstream file{path, std::ios::in | std::ios::out | std::ios::binary};
  ASSERT_TRUE(file.is_open());
  RecordHeader header{};
  std::vector<std::byte> stored(8);
  file.seekg(static_cast<std::streamoff>(offset));
  file.read(reinterpret_cast<char*>(&header), sizeof(header));
  file.read(reinterpret_cast<char*>(stored.data()),
            static_cast<std::streamsize>(stored.size()));
  ASSERT_EQ(header.stored_bytes, 8u);

  header.payload_bytes = 4;
  header.crc32 = 0;
  const std::span<const std::byte> header_bytes{
      reinterpret_cast<const std::byte*>(&header), sizeof(header)};
  header.crc32 = crc32(stored, crc32(header_bytes));
  file.seekp(static_cast<std::streamoff>(offset));
  file.write(reinterpret_cast<const char*>(&header), sizeof(header));
  file.close();

  LogReader reader{path};
  LogReader::Record record;
  EXPECT_FALSE(reader.next(record));
  EXPECT_EQ(reader.damage().tail_bytes, sizeof(RecordHeader) + 8);

  ::unlink(path.c_str());
}

// Where record `index` of the round-trip fixture begins.
std::size_t FixtureRecordOffset(const std::vector<ExpectedRecord>& expected,
                                std::size_t index) {
  std::size_t offset = RecordsOffset();
  for (std::size_t i = 0; i < index; ++i) {
    offset += sizeof(RecordHeader) + StoredSize(expected[i].payload);
  }
  return offset;
}

// One damaged record mid-log costs only itself: the reader resynchronises at
// the next intact record and counts the bytes it skipped.
// REQ-CONTRACT-20: a damaged record is skipped and its bytes counted
TEST(LogCorruption, DamagedRecordIsSkippedAndCounted) {
  const std::string path = TempPath("payload_corrupt");
  const std::vector<ExpectedRecord> expected =
      WriteFixture(path, LogWriter::kDefaultBufferBytes);

  std::size_t target = expected.size() / 2;
  while (StoredSize(expected[target].payload) == 0) ++target;
  const std::size_t offset = FixtureRecordOffset(expected, target);
  FlipByte(path, offset + sizeof(RecordHeader));  // its first stored byte

  LogReader reader{path};
  LogReader::Record record;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (i == target) continue;
    SCOPED_TRACE(i);
    ASSERT_TRUE(reader.next(record));
    ExpectRecordEquals(expected[i], record);
  }
  EXPECT_FALSE(reader.next(record));
  EXPECT_FALSE(reader.truncated());
  EXPECT_EQ(reader.damage().spans, 1u);
  EXPECT_EQ(reader.damage().first_offset, offset);
  EXPECT_EQ(reader.damage().skipped_bytes,
            sizeof(RecordHeader) + StoredSize(expected[target].payload));

  ::unlink(path.c_str());
}

// A power loss can leave the file longer than what reached the disk: a
// zero-filled or garbage tail. It ends the log after every intact record.
TEST(LogCorruption, DamagedTailEndsTheLogAfterEveryIntactRecord) {
  const std::string path = TempPath("damaged_tail");
  for (const bool zeros : {true, false}) {
    SCOPED_TRACE(zeros);
    const std::vector<ExpectedRecord> expected =
        WriteFixture(path, LogWriter::kDefaultBufferBytes);
    const std::size_t intact = FileSize(path);
    std::vector<std::byte> tail = MakePayload(4096, 7);
    if (zeros) std::fill(tail.begin(), tail.end(), std::byte{0});
    {
      std::ofstream out{path, std::ios::binary | std::ios::app};
      out.write(reinterpret_cast<const char*>(tail.data()),
                static_cast<std::streamsize>(tail.size()));
    }

    LogReader reader{path};
    LogReader::Record record;
    for (const ExpectedRecord& want : expected) {
      ASSERT_TRUE(reader.next(record));
      ExpectRecordEquals(want, record);
    }
    EXPECT_FALSE(reader.next(record));
    EXPECT_TRUE(reader.truncated());
    EXPECT_EQ(reader.damage().spans, 0u);
    EXPECT_EQ(reader.damage().tail_offset, intact);
    EXPECT_EQ(reader.damage().tail_bytes, tail.size());
  }
  ::unlink(path.c_str());
}

// Record magic inside a payload, with a plausible header after it, is not a
// record unless its checksum says so: a scan that starts inside a damaged
// record must not resume at it.
TEST(LogCorruption, FalseMagicInsideAPayloadIsNotARecord) {
  const std::string path = TempPath("false_magic");
  RecordHeader fake{};
  fake.magic = RECORD_MAGIC;
  fake.kind = static_cast<std::uint16_t>(EventKind::MESSAGE);
  fake.source_id = 1;
  fake.dispatch_index = 99;
  fake.payload_bytes = 8;
  fake.stored_bytes = 8;
  fake.crc32 = 0x12345678;  // not the checksum of this header and its bytes
  std::vector<std::byte> carrier = MakePayload(128, 3);
  std::memcpy(carrier.data() + 16, &fake, sizeof(fake));
  {
    LogWriter writer{path, "false_magic", LogWriter::kDefaultBufferBytes};
    writer.start(MakeManifest(), MonotonicTime::from_nanos(1));
    for (std::uint64_t i = 1; i <= 3; ++i) {
      Context context;
      context.kind = EventKind::MESSAGE;
      context.source_id = 1;
      context.dispatch_index = i;
      writer.dispatch(context, i == 2 ? carrier : MakePayload(128, 1));
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }
  // Damage the carrier's header, so the scan starts inside it.
  const std::size_t carrier_offset =
      RecordsOffset() + sizeof(RecordHeader) + StoredSize(MakePayload(128, 1));
  FlipByte(path, carrier_offset + offsetof(RecordHeader, dispatch_index));

  LogReader reader{path};
  LogReader::Record record;
  ASSERT_TRUE(reader.next(record));
  EXPECT_EQ(record.header.dispatch_index, 1u);
  ASSERT_TRUE(reader.next(record));
  EXPECT_EQ(record.header.dispatch_index, 3u);
  EXPECT_FALSE(reader.next(record));
  EXPECT_EQ(reader.damage().spans, 1u);
  EXPECT_EQ(reader.damage().skipped_bytes,
            sizeof(RecordHeader) + StoredSize(carrier));
  EXPECT_FALSE(reader.truncated());

  ::unlink(path.c_str());
}

// REQ-CONTRACT-20
TEST(LogTruncation, IncompleteTrailingRecordIsNotAnError) {
  const std::string path = TempPath("truncated");
  const std::vector<ExpectedRecord> expected =
      WriteFixture(path, LogWriter::kDefaultBufferBytes);

  const std::size_t full_size = FileSize(path);
  ASSERT_GT(full_size, 20u);
  ASSERT_EQ(0, ::truncate(path.c_str(), static_cast<off_t>(full_size - 20)));

  LogReader reader{path};
  LogReader::Record record;
  std::size_t index = 0;
  while (reader.next(record)) {
    ASSERT_LT(index, expected.size());
    ExpectRecordEquals(expected[index], record);
    ++index;
  }
  EXPECT_LT(index, expected.size());
  EXPECT_TRUE(reader.truncated());

  ::unlink(path.c_str());
}

// A damaged file header, manifest entry or schema block (the schema bytes
// are covered by the manifest checksum, so a flipped name byte cannot decode
// the log as the wrong type) is refused before any record is read.
// REQ-CONTRACT-20
TEST(LogCorruption, DamagedHeaderOrManifestThrowsOnOpen) {
  const std::string path = TempPath("header_corrupt");
  const std::size_t magic = 0;
  const std::size_t manifest_entry = sizeof(FileHeader) + 10;
  const std::size_t schema_block =
      sizeof(FileHeader) + 3 * sizeof(ManifestEntry) + 2;
  for (const std::size_t offset : {magic, manifest_entry, schema_block}) {
    SCOPED_TRACE(offset);
    WriteFixture(path, LogWriter::kDefaultBufferBytes);
    FlipByte(path, offset);
    EXPECT_THROW(LogReader{path}, std::runtime_error);
  }
  ::unlink(path.c_str());
}

// Overwrites the log's format version (the header checksum covers only the
// manifest and schema blocks).
void SetVersion(const std::string& path, std::uint32_t version) {
  std::fstream file{path, std::ios::in | std::ios::out | std::ios::binary};
  ASSERT_TRUE(file.is_open());
  file.seekp(static_cast<std::streamoff>(offsetof(FileHeader, version)));
  file.write(reinterpret_cast<const char*>(&version), sizeof(version));
}

// Version 7 (no GAP records) reads exactly as version 8 does, through both
// readers; legacy and future versions are refused before any record, naming
// the version read and those supported.
// REQ-CONTRACT-20
TEST(LogFormatErrors, ReadsVersionsSevenAndEightAndRejectsOthers) {
  const std::string path = TempPath("versions");
  for (const std::uint32_t version : {7u, 8u}) {
    SCOPED_TRACE(version);
    const std::vector<ExpectedRecord> expected =
        WriteFixture(path, LogWriter::kDefaultBufferBytes);
    SetVersion(path, version);
    LogReader reader{path};
    EXPECT_EQ(reader.format_version(), version);
    LogReader::Record record;
    for (const ExpectedRecord& want : expected) {
      ASSERT_TRUE(reader.next(record));
      ExpectRecordEquals(want, record);
    }
    EXPECT_FALSE(reader.next(record));
    EXPECT_TRUE(reader.damage().clean());
    WindowedLogReader windowed{path};
    std::size_t count = 0;
    while (windowed.next(record)) ++count;
    EXPECT_EQ(count, expected.size());
  }
  for (const std::uint32_t version : {4u, 6u, 9u}) {
    SCOPED_TRACE(version);
    WriteFixture(path, LogWriter::kDefaultBufferBytes);
    SetVersion(path, version);
    if (version == 4) {
      // An empty v4 file is shorter than the current FileHeader.
      ASSERT_EQ(0, ::truncate(path.c_str(), 112));
    }
    for (const bool windowed : {false, true}) {
      SCOPED_TRACE(windowed);
      try {
        if (windowed) {
          WindowedLogReader reader{path};
        } else {
          LogReader reader{path};
        }
        ADD_FAILURE() << "version " << version << " was read";
      } catch (const std::runtime_error& error) {
        const std::string what = error.what();
        EXPECT_NE(what.find("version " + std::to_string(version)),
                  std::string::npos)
            << what;
        EXPECT_NE(what.find("7 to 8"), std::string::npos) << what;
      }
    }
  }
  ::unlink(path.c_str());
}

// Everything one reader returns from a log, and what it counted.
struct ReadBack {
  struct Read {
    RecordHeader header{};
    std::vector<std::byte> payload;
    LogReader::Record view() const { return {header, payload, 0}; }
  };
  std::vector<Read> records;
  std::size_t gaps{0};
  LogDamage damage;
};

ReadBack ReadAll(const std::string& path, bool windowed) {
  ReadBack back;
  const auto take = [&](auto& reader) {
    LogReader::Record record;
    while (reader.next(record)) {
      back.records.push_back(
          {record.header, {record.payload.begin(), record.payload.end()}});
    }
    back.gaps = reader.gaps();
    back.damage = reader.damage();
  };
  if (windowed) {
    WindowedLogReader reader{path};
    take(reader);
  } else {
    LogReader reader{path};
    take(reader);
  }
  return back;
}

// A session merge (log_dump, session_report, Studio's review) holds a
// WindowedLogReader per log at once: readers of more logs than the
// descriptor limit all open and read, interleaved, exactly what each log
// holds, since a reader holds a descriptor only while it reads.
// REQ-OPS-15
TEST(WindowedLogReaderLimits, MoreReadersThanTheDescriptorLimit) {
  const std::string source = TempPath("fd_limit_source");
  const std::vector<ExpectedRecord> expected =
      WriteFixture(source, LogWriter::kDefaultBufferBytes);
  constexpr std::size_t kSpare = 4;
  constexpr std::size_t kLogs = 4 * kSpare;
  std::vector<std::string> paths;
  for (std::size_t i = 0; i < kLogs; ++i) {
    paths.push_back(TempPath("fd_limit_" + std::to_string(i)));
    std::filesystem::copy_file(
        source, paths.back(),
        std::filesystem::copy_options::overwrite_existing);
  }
  // At most kSpare descriptors are free below the lowered limit.
  std::size_t open_now = 0;
  for (const auto& entry : std::filesystem::directory_iterator{"/dev/fd"}) {
    (void)entry;
    ++open_now;
  }
  rlimit saved{};
  ASSERT_EQ(0, ::getrlimit(RLIMIT_NOFILE, &saved));
  rlimit lowered = saved;
  lowered.rlim_cur =
      static_cast<rlim_t>(open_now) + static_cast<rlim_t>(kSpare);
  ASSERT_EQ(0, ::setrlimit(RLIMIT_NOFILE, &lowered));
  std::vector<std::vector<ReadBack::Read>> got(kLogs);
  std::string error;
  try {
    std::vector<WindowedLogReader> readers;
    readers.reserve(kLogs);
    for (const std::string& path : paths) readers.emplace_back(path);
    LogReader::Record record;
    for (bool more = true; more;) {
      more = false;
      for (std::size_t i = 0; i < kLogs; ++i) {
        if (!readers[i].next(record)) continue;
        more = true;
        got[i].push_back(
            {record.header, {record.payload.begin(), record.payload.end()}});
      }
    }
  } catch (const std::exception& e) {
    error = e.what();
  }
  ASSERT_EQ(0, ::setrlimit(RLIMIT_NOFILE, &saved));
  EXPECT_EQ(error, "");
  for (std::size_t i = 0; i < kLogs; ++i) {
    SCOPED_TRACE(i);
    ASSERT_EQ(got[i].size(), expected.size());
    for (std::size_t r = 0; r < expected.size(); ++r) {
      ExpectRecordEquals(expected[r], got[i][r].view());
    }
    ::unlink(paths[i].c_str());
  }
  ::unlink(source.c_str());
}

// Guards WindowedLogReader::start_wall_ns() and start_time() against LogReader:
// callers mapping monotonic log times to wall clock (studio_review) read both
// from WindowedLogReader without re-opening raw FileHeaders.
// REQ-OPS-41
TEST(WindowedLogReaderLimits, StartWallNsMatchesLogReader) {
  const std::string path = TempPath("start_wall_ns");
  WriteFixture(path, LogWriter::kDefaultBufferBytes);
  constexpr std::int64_t kPinnedWallNs = 1'767'225'600'123'456'789LL;
  {
    std::fstream file{path, std::ios::in | std::ios::out | std::ios::binary};
    ASSERT_TRUE(file.is_open());
    file.seekp(offsetof(FileHeader, start_wall_ns));
    file.write(reinterpret_cast<const char*>(&kPinnedWallNs),
               sizeof kPinnedWallNs);
    ASSERT_TRUE(file);
  }
  const LogReader full{path};
  const WindowedLogReader windowed{path};
  EXPECT_EQ(windowed.start_wall_ns(), kPinnedWallNs);
  EXPECT_EQ(windowed.start_wall_ns(), full.start_wall_ns());
  EXPECT_EQ(windowed.start_time().nanos(), full.start_time().nanos());
  ::unlink(path.c_str());
}

// Where the fixture's record of `kind` in dispatch `dispatch` is.
std::size_t FixtureIndex(const std::vector<ExpectedRecord>& expected,
                         std::uint64_t dispatch, EventKind kind) {
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (expected[i].dispatch_index == dispatch &&
        expected[i].kind == static_cast<std::uint16_t>(kind)) {
      return i;
    }
  }
  ADD_FAILURE() << "no " << to_string(kind) << " in dispatch " << dispatch;
  return 0;
}

// The header and payload size of each of records [first, end).
std::uint64_t FixtureBytes(const std::vector<ExpectedRecord>& expected,
                           std::size_t first, std::size_t end) {
  std::uint64_t bytes = 0;
  for (std::size_t i = first; i < end; ++i) {
    bytes += sizeof(RecordHeader) + expected[i].payload.size();
  }
  return bytes;
}

// A GAP record reads back through both readers as a record of kind GAP, in
// place of the records it stands for, counted in gaps() and the summary, its
// payload decoding to the stretch dropped; damage to it, or next to it, and a
// cut through or after it, are handled as for any record.
// REQ-CONTRACT-20
TEST(LogGap, ReadersReturnAGapAndCountIt) {
  const std::string path = TempPath("gap_source");
  const std::string gapped = TempPath("gapped");
  const std::vector<ExpectedRecord> expected =
      WriteFixture(path, LogWriter::kDefaultBufferBytes);
  // Dropped: dispatch 12's SEND (its TIMER was recorded) to dispatch 19.
  const std::size_t first = FixtureIndex(expected, 12, EventKind::SEND);
  const std::size_t end = FixtureIndex(expected, 20, EventKind::TIMER);
  ASSERT_EQ(end - first, 9u);
  const LogGap made = CopyWithGap(path, gapped, first, end);
  const std::size_t gap_offset = FixtureRecordOffset(expected, first);

  for (const bool windowed : {false, true}) {
    SCOPED_TRACE(windowed);
    const ReadBack back = ReadAll(gapped, windowed);
    ASSERT_EQ(back.records.size(), expected.size() - (end - first) + 1);
    for (std::size_t i = 0; i < first; ++i) {
      ExpectRecordEquals(expected[i], back.records[i].view());
    }
    const ReadBack::Read& gap = back.records[first];
    EXPECT_EQ(gap.header.kind, static_cast<std::uint16_t>(EventKind::GAP));
    EXPECT_EQ(gap.header.source_id, GAP_SOURCE_ID);
    EXPECT_EQ(gap.header.dispatch_index, 12u);
    // Dispatch 12's SEND is the earliest: dispatched at 12 us, sent at
    // 12.02 us. Capture resumed with dispatch 20, at 20 us.
    EXPECT_EQ(gap.header.event_time_ns, 12'000);
    EXPECT_EQ(gap.header.now_ns, 20'000);
    EXPECT_EQ(gap.header.publish_time_ns, 0);
    EXPECT_EQ(gap.header.sequence, 0u);
    EXPECT_EQ(gap.header.aux, 0u);
    ASSERT_EQ(gap.payload.size(), sizeof(GapRecord));
    const std::optional<LogGap> decoded = GapOf(gap.header, gap.payload);
    ASSERT_TRUE(decoded.has_value());
    ASSERT_TRUE(decoded->detailed);
    EXPECT_EQ(decoded->first_dispatch_index, 12u);
    EXPECT_EQ(decoded->start_ns, 12'000);
    EXPECT_EQ(decoded->resumed_ns, 20'000);
    const GapRecord& detail = decoded->detail;
    EXPECT_EQ(detail.cause, static_cast<std::uint16_t>(GapCause::POOL_FULL));
    EXPECT_EQ(detail.flags, GAP_SPLITS_DISPATCH);
    EXPECT_EQ(detail.reserved, 0u);
    EXPECT_EQ(detail.first_lost_ns, 12'020);  // the SEND's now
    EXPECT_EQ(detail.last_lost_ns, 19'040);
    EXPECT_EQ(detail.last_dispatch_index, 19u);
    EXPECT_EQ(detail.records, 9u);
    EXPECT_EQ(detail.bytes, FixtureBytes(expected, first, end));
    EXPECT_EQ(detail.dispatches, 7u);  // 13 to 19
    EXPECT_EQ(detail.sends, 2u);       // dispatches 12 and 17
    EXPECT_EQ(0, std::memcmp(&detail, &made.detail, sizeof(GapRecord)));
    // Decoded from the stored bytes alone (trailing zeros trimmed) too.
    const std::optional<LogGap> from_stored = GapOf(
        gap.header,
        std::span<const std::byte>{gap.payload}.first(gap.header.stored_bytes));
    ASSERT_TRUE(from_stored.has_value());
    EXPECT_EQ(0, std::memcmp(&from_stored->detail, &detail, sizeof(detail)));
    for (std::size_t i = end; i < expected.size(); ++i) {
      ExpectRecordEquals(expected[i], back.records[i - end + first + 1].view());
    }
    EXPECT_EQ(back.gaps, 1u);
    EXPECT_TRUE(back.damage.clean());
    EXPECT_FALSE(
        GapOf(back.records[0].header, back.records[0].payload).has_value());
  }
  const std::string sentence = DescribeGap(made);
  EXPECT_NE(sentence.find("9 records"), std::string::npos) << sentence;
  EXPECT_NE(sentence.find("12 to 19"), std::string::npos) << sentence;

  const LogSummary summary = SummarizeLog(gapped);
  EXPECT_EQ(summary.gaps, 1u);
  EXPECT_EQ(summary.record_count, expected.size() - (end - first) + 1);
  {
    LogReader reader{gapped};
    LogReader::Record record;
    while (reader.next(record)) {
    }
    reader.rewind();
    EXPECT_EQ(reader.gaps(), 0u);
  }

  // A damaged GAP is skipped and counted as damage; the record after it
  // still reads.
  const std::size_t gap_bytes =
      sizeof(RecordHeader) +
      ReadAll(gapped, false).records[first].header.stored_bytes;
  FlipByte(gapped, gap_offset + sizeof(RecordHeader));  // its cause
  for (const bool windowed : {false, true}) {
    SCOPED_TRACE(windowed);
    const ReadBack back = ReadAll(gapped, windowed);
    EXPECT_EQ(back.gaps, 0u);
    EXPECT_EQ(back.records.size(), expected.size() - (end - first));
    EXPECT_EQ(back.damage.spans, 1u);
    EXPECT_EQ(back.damage.first_offset, gap_offset);
    EXPECT_EQ(back.damage.skipped_bytes, gap_bytes);
    ASSERT_GT(back.records.size(), first);
    ExpectRecordEquals(expected[end], back.records[first].view());
  }

  // Damage to the record before it: the scan resumes at the GAP.
  CopyWithGap(path, gapped, first, end);
  ASSERT_NE(StoredSize(expected[first - 1].payload), 0u);
  FlipByte(gapped,
           FixtureRecordOffset(expected, first - 1) + sizeof(RecordHeader));
  for (const bool windowed : {false, true}) {
    SCOPED_TRACE(windowed);
    const ReadBack back = ReadAll(gapped, windowed);
    EXPECT_EQ(back.gaps, 1u);
    EXPECT_EQ(back.damage.spans, 1u);
    ASSERT_GT(back.records.size(), first - 1);
    EXPECT_EQ(back.records[first - 1].header.kind,
              static_cast<std::uint16_t>(EventKind::GAP));
  }

  // A cut through the GAP leaves the records before it; a cut after it
  // keeps it.
  for (const std::size_t cut : {gap_offset + 20, gap_offset + gap_bytes + 20}) {
    SCOPED_TRACE(cut);
    CopyWithGap(path, gapped, first, end);
    ASSERT_EQ(0, ::truncate(gapped.c_str(), static_cast<off_t>(cut)));
    const bool kept = cut > gap_offset + gap_bytes;
    for (const bool windowed : {false, true}) {
      SCOPED_TRACE(windowed);
      const ReadBack back = ReadAll(gapped, windowed);
      EXPECT_EQ(back.records.size(), first + (kept ? 1 : 0));
      EXPECT_EQ(back.gaps, kept ? 1u : 0u);
      EXPECT_EQ(back.damage.tail_bytes, 20u);
    }
  }
  ::unlink(path.c_str());
  ::unlink(gapped.c_str());
}

// CopyWithGap marks what the writer would: a gap reaching the end of the log
// is closed at the stop, the EXIT the writer holds follows it and gives it
// its time, and a range whose next record continues a dropped dispatch is
// refused, since capture never resumes there.
TEST(LogGap, CopyWithGapClosesAtTheStopAndKeepsTheExit) {
  const std::string path = TempPath("gap_end_source");
  const std::string gapped = TempPath("gap_end");
  const std::vector<ExpectedRecord> expected =
      WriteFixture(path, LogWriter::kDefaultBufferBytes);
  const std::size_t first = FixtureIndex(expected, 40, EventKind::TIMER);
  CopyWithGap(path, gapped, first, expected.size());

  const ReadBack back = ReadAll(gapped, false);
  ASSERT_EQ(back.records.size(), first + 2);
  const ReadBack::Read& gap = back.records[first];
  ASSERT_EQ(gap.header.kind, static_cast<std::uint16_t>(EventKind::GAP));
  EXPECT_EQ(gap.header.now_ns, expected.back().now_ns);  // the held EXIT's
  const std::optional<LogGap> decoded = GapOf(gap.header, gap.payload);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->detail.flags, GAP_CLOSED_AT_FLUSH);
  EXPECT_EQ(decoded->detail.records, expected.size() - 1 - first);
  EXPECT_EQ(decoded->detail.last_dispatch_index, 49u);
  ExpectRecordEquals(expected.back(), back.records.back().view());

  const std::size_t send = FixtureIndex(expected, 12, EventKind::SEND);
  EXPECT_THROW(CopyWithGap(path, gapped, send - 1, send),
               std::invalid_argument);
  EXPECT_THROW(CopyWithGap(path, gapped, first, first), std::invalid_argument);
  ::unlink(path.c_str());
  ::unlink(gapped.c_str());
}

// Writes must be identical whether they went to disk on the loop thread or
// were handed to the writer thread, and small chunks force many handoffs.
TEST(LogBackgroundWriter, RoundTripsThroughTheWriterThread) {
  const std::string path = TempPath("background_round_trip");
  const std::vector<ExpectedRecord> expected = WriteFixtureWithOptions(
      path, LogWriter::Options{256, 64, /*background=*/true});

  LogReader reader{path};
  LogReader::Record record;

  for (const ExpectedRecord& want : expected) {
    ASSERT_TRUE(reader.next(record));
    ExpectRecordEquals(want, record);
  }

  EXPECT_FALSE(reader.next(record));
  EXPECT_FALSE(reader.truncated());
  EXPECT_EQ(reader.record_count(), expected.size());

  std::remove(path.c_str());
}

TEST(LogBackgroundWriter, IdlePartialChunkReachesFileAndIsNotDuplicated) {
  const auto path = TempPath("periodic_partial");
  {
    LogWriter writer{path, "periodic", ShortSyncOptions()};
    writer.start({}, {});
    for (std::uint64_t i = 1; i <= 2; ++i) {
      Context context{};
      context.dispatch_index = i;
      writer.dispatch(context, MakePayload(16, static_cast<int>(i)));
      // No further producer activity and no flush: the worker must write the
      // committed prefix of a chunk that is nowhere near full.
      struct stat status {};
      const auto expected = static_cast<off_t>(sizeof(FileHeader) +
                                               i * (sizeof(RecordHeader) + 16));
      // The writer thread creates the file, so it may not exist yet.
      EXPECT_TRUE(Eventually([&] {
        return ::stat(path.c_str(), &status) == 0 && status.st_size >= expected;
      }));
      // Every periodic pass since has written nothing again.
      ASSERT_EQ(status.st_size, expected);
      ASSERT_FALSE(writer.failed()) << writer.error();
    }
    // Destruction must write the remaining suffix, not repeat either prefix.
    Context context{};
    context.dispatch_index = 3;
    writer.dispatch(context, MakePayload(16, 3));
  }
  LogReader reader{path};
  LogReader::Record record{};
  for (std::uint64_t i = 1; i <= 3; ++i) {
    ASSERT_TRUE(reader.next(record));
    EXPECT_EQ(record.header.dispatch_index, i);
    const auto expected = MakePayload(16, static_cast<int>(i));
    EXPECT_TRUE(std::equal(record.payload.begin(), record.payload.end(),
                           expected.begin(), expected.end()));
  }
  EXPECT_FALSE(reader.next(record));
  EXPECT_FALSE(reader.truncated());
  ::unlink(path.c_str());
}

// Invalidate only this writer's descriptor once its header is written
// (after start() and, for a background writer, a flush()). No descriptors
// may be opened until the writer is destroyed, since it still owns the
// now-invalid descriptor number.
bool CloseLogDescriptor(const std::string& path) {
  struct stat target {};
  if (::stat(path.c_str(), &target) != 0) return false;
  const auto limit = ::sysconf(_SC_OPEN_MAX);
  for (int fd = 0; fd < limit; ++fd) {
    struct stat candidate {};
    if (::fstat(fd, &candidate) == 0 && candidate.st_dev == target.st_dev &&
        candidate.st_ino == target.st_ino) {
      return ::close(fd) == 0;
    }
  }
  return false;
}

// REQ-RT-11 REQ-OPS-44
TEST(LogDurability, FlushReportsFsyncFailure) {
  const auto path = TempPath("flush_sync_closed");
  for (const bool background : {false, true}) {
    LogWriter writer{path, "sync_failure",
                     LogWriter::Options{4096, 2, background}};
    writer.start({}, {});
    writer.flush();  // A background writer's thread creates the file.
    ASSERT_TRUE(CloseLogDescriptor(path));
    EXPECT_FALSE(writer.failed());
    writer.flush();
    EXPECT_TRUE(writer.failed());
    EXPECT_TRUE(writer.incomplete());  // what the node's exit status says
    writer.flush();
    EXPECT_TRUE(writer.failed());
  }
  // Neither writer buffers records, so EBADF must originate in fsync.
  ::unlink(path.c_str());
}

TEST(LogDurability, IdleBackgroundWriterFsyncsWithoutExplicitFlush) {
  const auto path = TempPath("periodic_sync_closed");
  {
    LogWriter writer{path, "periodic_sync", ShortSyncOptions()};
    writer.start({}, {});
    writer.flush();  // The writer thread creates the file.
    ASSERT_TRUE(CloseLogDescriptor(path));
    EXPECT_TRUE(Eventually([&] { return writer.failed(); }));
  }
  ::unlink(path.c_str());
}

// REQ-OPS-12: flush() durable-syncs the file once; destroying LogWriter with
// no further writes skips a redundant DurableSync, while additional writes
// after flush() or a failed flush() still durable-sync on destruction.
TEST(LogDurability, DestructionSkipsRedundantDurableSyncAfterFlush) {
  const auto path = TempPath("skip_redundant_durable_sync");
  for (const bool background : {false, true}) {
    SCOPED_TRACE(background ? "background" : "synchronous");

    // Case 1: write, flush(), destroy with no further writes -> 1 durable sync.
    {
      std::atomic<int> durable_syncs{0};
      LogWriter::Options options{4096, 2, background};
      options.sync_interval = std::chrono::hours{1};
      options.sink_ops.durable_sync = [&](int fd) {
        durable_syncs.fetch_add(1);
        return process::DurableSync(fd);
      };
      {
        LogWriter writer{path, "no_double_sync", std::move(options)};
        writer.start({}, {});
        Context ctx{};
        ctx.dispatch_index = 1;
        writer.dispatch(ctx, MakePayload(16, 1));
        ctx.kind = EventKind::EXIT;
        ctx.dispatch_index = 2;
        writer.finish(ctx);
        writer.flush();
        ASSERT_FALSE(writer.failed()) << writer.error();
        EXPECT_EQ(durable_syncs.load(), 1);
      }
      EXPECT_EQ(durable_syncs.load(), 1)
          << "destructor ran a redundant DurableSync after flush()";
    }

    // Case 2: write after flush() -> destructor durable-syncs the new bytes.
    {
      std::atomic<int> durable_syncs{0};
      LogWriter::Options options{4096, 2, background};
      options.sync_interval = std::chrono::hours{1};
      options.sink_ops.durable_sync = [&](int fd) {
        durable_syncs.fetch_add(1);
        return process::DurableSync(fd);
      };
      {
        LogWriter writer{path, "dirty_after_flush", std::move(options)};
        writer.start({}, {});
        Context ctx{};
        ctx.dispatch_index = 1;
        writer.dispatch(ctx, MakePayload(16, 1));
        writer.flush();
        ASSERT_EQ(durable_syncs.load(), 1);
        ctx.dispatch_index = 2;
        writer.dispatch(ctx, MakePayload(16, 2));
      }
      EXPECT_EQ(durable_syncs.load(), 2)
          << "destructor skipped DurableSync after post-flush writes";
    }

    // Case 3: failed durable_sync during flush() leaves needs_durable_sync set.
    {
      std::atomic<int> durable_syncs{0};
      LogWriter::Options options{4096, 2, background};
      options.sync_interval = std::chrono::hours{1};
      options.sink_ops.durable_sync = [&](int fd) {
        if (durable_syncs.fetch_add(1) == 0) {
          errno = EIO;
          return -1;
        }
        return process::DurableSync(fd);
      };
      {
        LogWriter writer{path, "failed_flush_sync", std::move(options)};
        writer.start({}, {});
        Context ctx{};
        ctx.dispatch_index = 1;
        writer.dispatch(ctx, MakePayload(16, 1));
        writer.flush();
        EXPECT_TRUE(writer.failed());
        ASSERT_EQ(durable_syncs.load(), 1);
      }
      EXPECT_EQ(durable_syncs.load(), 2)
          << "failed durable sync must not clear needs_durable_sync";
    }
  }
  ::unlink(path.c_str());
}

// REQ-OPS-12: when flush() coincides with a due periodic sync, the background
// writer skips periodic sync(false) so flush() does not stack periodic fsync
// and DurableSync back-to-back.
TEST(LogDurability, FlushSkipsCoincidingPeriodicSync) {
  const auto path = TempPath("flush_skips_periodic_sync");
  std::mutex gate_mutex;
  std::condition_variable gate_cv;
  bool open_released = false;
  bool write_released = false;
  std::atomic<bool> open_entered{false};
  std::atomic<int> write_entries{0};
  std::atomic<int> periodic_syncs{0};
  std::atomic<int> durable_syncs{0};
  int periodic_at_durable = -1;
  std::uint64_t wakeups_at_durable = 999;
  const LogWriter* writer_ptr = nullptr;

  {
    LogWriter::Options options{4096, 2, /*background=*/true};
    options.write_interval = std::chrono::hours{1};
    // Always elapsed so sync_due would fire on every pass unless suppressed.
    options.sync_interval = std::chrono::nanoseconds{0};
    options.sink_ops.open = [&](const char* target, int flags, mode_t mode) {
      open_entered.store(true);
      {
        std::unique_lock lock{gate_mutex};
        gate_cv.wait(lock, [&] { return open_released; });
      }
      return (flags & O_CREAT) != 0 ? ::open(target, flags, mode)
                                    : ::open(target, flags);
    };
    options.sink_ops.periodic_sync = [&](int /*fd*/) {
      periodic_syncs.fetch_add(1);
      return 0;
    };
    options.sink_ops.durable_sync = [&](int fd) {
      durable_syncs.fetch_add(1);
      if (periodic_at_durable < 0) {
        periodic_at_durable = periodic_syncs.load();
        wakeups_at_durable = writer_ptr->wakeups();
      }
      return process::DurableSync(fd);
    };
    options.before_write = [&] {
      write_entries.fetch_add(1);
      std::unique_lock lock{gate_mutex};
      gate_cv.wait(lock, [&] { return write_released; });
    };
    options.before_flush_wait = [&] {
      {
        std::lock_guard lock{gate_mutex};
        write_released = true;
      }
      gate_cv.notify_all();
    };

    LogWriter writer{path, "flush_periodic", std::move(options)};
    writer_ptr = &writer;
    struct ReleaseGates {
      std::mutex& mutex;
      std::condition_variable& cv;
      bool& open_flag;
      bool& write_flag;
      ~ReleaseGates() {
        {
          std::lock_guard lock{mutex};
          open_flag = true;
          write_flag = true;
        }
        cv.notify_all();
      }
    } release_gates{gate_mutex, gate_cv, open_released, write_released};
    writer.start({}, {});
    ASSERT_TRUE(Eventually([&] { return open_entered.load(); }));
    Context ctx{};
    ctx.dispatch_index = 1;
    writer.dispatch(ctx, MakePayload(16, 1));
    {
      std::lock_guard lock{gate_mutex};
      open_released = true;
    }
    gate_cv.notify_all();
    ASSERT_TRUE(Eventually([&] { return write_entries.load() == 1; }));
    ASSERT_EQ(periodic_syncs.load(), 0);

    // flush() sets flushing, releases before_write via before_flush_wait,
    // drains the submitted chunk, and calls DurableSync.
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
    EXPECT_EQ(periodic_at_durable, 0)
        << "writer ran periodic sync right before flush()'s DurableSync";
    EXPECT_EQ(wakeups_at_durable, 0u)
        << "writer polled wake while parked in flush_cv during drain()";
    EXPECT_EQ(durable_syncs.load(), 1);
  }
  EXPECT_EQ(durable_syncs.load(), 1);
  ::unlink(path.c_str());
}

// REQ-OPS-12: when a non-rotating byte cap trips after flush() cleared
// needs_durable_sync_ with zero new bytes written, destruction must still
// create and durable-sync `<path>.capped` without re-syncing the log file;
// LogFileSink::sync() with no open descriptor (fd_ < 0) must not create
// `<path>.capped`.
TEST(LogDurability, DestructionMarksCappedWhenByteCapTripsAfterFlush) {
  const auto path = TempPath("byte_cap_after_flush");
  const auto marker = CappedMarkerPath(path);

  // Unopened sink (fd_ < 0): sync() does not create `<path>.capped`.
  {
    ::unlink(path.c_str());
    ::unlink(marker.c_str());
    LogFileSink sink{path};
    sink.sync(/*durable=*/true, /*capture_stopped=*/true);
    EXPECT_FALSE(std::filesystem::exists(marker));
  }

  for (const bool background : {false, true}) {
    SCOPED_TRACE(background ? "background" : "synchronous");
    ::unlink(path.c_str());
    ::unlink(marker.c_str());

    std::mutex fd_mutex;
    std::map<int, std::string> fd_paths;
    std::atomic<int> log_durable_syncs{0};
    std::atomic<int> marker_durable_syncs{0};

    LogWriter::Options options{4096, 2, background};
    options.sync_interval = std::chrono::hours{1};
    // Fits FileHeader + one 16-byte record exactly (and < 2 * (prefix + GAP),
    // so segment_bytes == 0 and the second record trips BYTE_CAP with 0 bytes
    // written after flush()).
    options.max_bytes = sizeof(FileHeader) + sizeof(RecordHeader) + 16;
    options.sink_ops.open = [&](const char* target, int flags, mode_t mode) {
      const int fd = (flags & O_CREAT) != 0 ? ::open(target, flags, mode)
                                            : ::open(target, flags);
      if (fd >= 0) {
        std::lock_guard lock{fd_mutex};
        fd_paths[fd] = target;
      }
      return fd;
    };
    options.sink_ops.close = [&](int fd) {
      {
        std::lock_guard lock{fd_mutex};
        fd_paths.erase(fd);
      }
      return ::close(fd);
    };
    options.sink_ops.durable_sync = [&](int fd) {
      std::string target;
      {
        std::lock_guard lock{fd_mutex};
        if (const auto it = fd_paths.find(fd); it != fd_paths.end()) {
          target = it->second;
        }
      }
      if (target == path) {
        log_durable_syncs.fetch_add(1);
      } else if (target == marker) {
        marker_durable_syncs.fetch_add(1);
      }
      return process::DurableSync(fd);
    };

    {
      LogWriter writer{path, "cap_after_flush", std::move(options)};
      writer.start({}, {});
      Context ctx{};
      ctx.dispatch_index = 1;
      writer.dispatch(ctx, MakePayload(16, 1));
      writer.flush();
      ASSERT_FALSE(writer.failed()) << writer.error();
      ASSERT_FALSE(writer.capture_stopped());
      ASSERT_FALSE(std::filesystem::exists(marker));
      ASSERT_EQ(log_durable_syncs.load(), 1);
      ASSERT_EQ(marker_durable_syncs.load(), 0);

      // Second record crosses max_bytes -> BYTE_CAP with zero bytes written.
      ctx.dispatch_index = 2;
      writer.dispatch(ctx, MakePayload(16, 2));
      ASSERT_TRUE(writer.capture_stopped());
      ASSERT_TRUE(writer.capped());
    }

    EXPECT_TRUE(std::filesystem::exists(marker))
        << "destructor failed to create `<path>.capped` when BYTE_CAP tripped "
           "after flush()";
    EXPECT_EQ(log_durable_syncs.load(), 1)
        << "destructor redundantly synced the log file after flush()";
    EXPECT_EQ(marker_durable_syncs.load(), 1)
        << "destructor must durable-sync `<path>.capped` once";
  }

  ::unlink(path.c_str());
  ::unlink(marker.c_str());
}

// Segment rotation must leave whole segments across any power cut: durable-sync
// the finished segment, write and durable-sync `.next` before renaming it into
// place, durable-sync `.capped` and the parent directory before unlinking the
// oldest segment once the cap is crossed, and durable-sync the parent directory
// last.
// REQ-OPS-12
TEST(LogDurability, RotateToDurableSyncsSegmentsMarkerAndDirectoryInOrder) {
  const std::string dir = ::testing::TempDir() + "/talos_log_rotate_ops12_" +
                          std::to_string(::getpid());
  std::filesystem::remove_all(dir);
  ASSERT_TRUE(std::filesystem::create_directories(dir));
  const std::string path = dir + "/node.tlog";
  const std::string staged = path + ".next";
  const std::string marker = CappedMarkerPath(path);

  struct Event {
    std::string op;
    std::string detail;
    off_t size{0};
  };
  std::map<int, std::string> fd_paths;
  std::vector<Event> events;

  LogFileSinkOps ops;
  ops.open = [&](const char* target, int flags, mode_t mode) {
    const int fd = (flags & O_CREAT) != 0 ? ::open(target, flags, mode)
                                          : ::open(target, flags);
    if (fd >= 0) fd_paths[fd] = target;
    return fd;
  };
  ops.close = [&](int fd) {
    fd_paths.erase(fd);
    return ::close(fd);
  };
  ops.durable_sync = [&](int fd) {
    struct stat st {};
    const off_t size = ::fstat(fd, &st) == 0 ? st.st_size : -1;
    const auto it = fd_paths.find(fd);
    events.push_back({"durable_sync",
                      it != fd_paths.end() ? it->second : "<unknown>", size});
    return process::DurableSync(fd);
  };
  ops.rename = [&](const char* from, const char* to) {
    events.push_back({"rename", std::string{from} + "->" + to, 0});
    for (auto& [open_fd, mapped] : fd_paths) {
      if (mapped == from) mapped = to;
    }
    return ::rename(from, to);
  };
  ops.unlink = [&](const char* target) {
    events.push_back({"unlink", target, 0});
    return ::unlink(target);
  };

  {
    const std::vector<std::byte> prefix = MakePayload(128, 1);
    const std::vector<std::byte> body0 = MakePayload(32, 2);
    const std::vector<std::byte> body1 = MakePayload(48, 3);

    LogFileSink sink{path, ops};
    sink.open_sync();
    sink.configure_segments(prefix, /*segment_count=*/2);
    sink.write_prefix_sync();
    ASSERT_EQ(sink.write_bytes(body0.data(), body0.size()), body0.size());

    // Segment 0 -> 1: under segment_count (2), so no segment is unlinked yet.
    events.clear();
    sink.rotate_to(1);
    ASSERT_FALSE(sink.failed()) << sink.error();
    EXPECT_FALSE(sink.head_dropped());
    EXPECT_EQ(sink.file_segment(), 1u);
    ASSERT_EQ(events.size(), 5u);
    EXPECT_EQ(events[0].op, "durable_sync");
    EXPECT_EQ(events[0].detail, path);
    EXPECT_EQ(events[0].size, static_cast<off_t>(prefix.size() + body0.size()));
    EXPECT_EQ(events[1].op, "durable_sync");
    EXPECT_EQ(events[1].detail, staged);
    EXPECT_EQ(events[1].size, static_cast<off_t>(prefix.size()));
    EXPECT_EQ(events[2].op, "rename");
    EXPECT_EQ(events[2].detail, path + "->" + LogSegmentPath(path, 0));
    EXPECT_EQ(events[3].op, "rename");
    EXPECT_EQ(events[3].detail, staged + "->" + path);
    EXPECT_EQ(events[4].op, "durable_sync");
    EXPECT_EQ(events[4].detail, dir);

    // Segment 1 -> 2: reaches segment_count (2), so `.capped` and its directory
    // must be durable-synced before renaming and unlinking segment 0, followed
    // by a final directory durable-sync.
    ASSERT_EQ(sink.write_bytes(body1.data(), body1.size()), body1.size());
    events.clear();
    sink.rotate_to(2);
    ASSERT_FALSE(sink.failed()) << sink.error();
    EXPECT_TRUE(sink.head_dropped());
    EXPECT_EQ(sink.file_segment(), 2u);
    ASSERT_EQ(events.size(), 8u);
    EXPECT_EQ(events[0].op, "durable_sync");
    EXPECT_EQ(events[0].detail, path);
    EXPECT_EQ(events[0].size, static_cast<off_t>(prefix.size() + body1.size()));
    EXPECT_EQ(events[1].op, "durable_sync");
    EXPECT_EQ(events[1].detail, staged);
    EXPECT_EQ(events[1].size, static_cast<off_t>(prefix.size()));
    EXPECT_EQ(events[2].op, "durable_sync");
    EXPECT_EQ(events[2].detail, marker);
    EXPECT_EQ(events[2].size, 0);
    EXPECT_EQ(events[3].op, "durable_sync");
    EXPECT_EQ(events[3].detail, dir);
    EXPECT_EQ(events[4].op, "rename");
    EXPECT_EQ(events[4].detail, path + "->" + LogSegmentPath(path, 1));
    EXPECT_EQ(events[5].op, "rename");
    EXPECT_EQ(events[5].detail, staged + "->" + path);
    EXPECT_EQ(events[6].op, "unlink");
    EXPECT_EQ(events[6].detail, LogSegmentPath(path, 0));
    EXPECT_EQ(events[7].op, "durable_sync");
    EXPECT_EQ(events[7].detail, dir);

    // Segment 2 -> 3: `.capped` is already durable on disk; rotation still
    // durable-syncs finished segment, `.next` before rename, unlinks segment 1,
    // and durable-syncs the directory last.
    events.clear();
    sink.rotate_to(3);
    ASSERT_FALSE(sink.failed()) << sink.error();
    EXPECT_EQ(sink.file_segment(), 3u);
    ASSERT_EQ(events.size(), 6u);
    EXPECT_EQ(events[0].op, "durable_sync");
    EXPECT_EQ(events[0].detail, path);
    EXPECT_EQ(events[1].op, "durable_sync");
    EXPECT_EQ(events[1].detail, staged);
    EXPECT_EQ(events[1].size, static_cast<off_t>(prefix.size()));
    EXPECT_EQ(events[2].op, "rename");
    EXPECT_EQ(events[2].detail, path + "->" + LogSegmentPath(path, 2));
    EXPECT_EQ(events[3].op, "rename");
    EXPECT_EQ(events[3].detail, staged + "->" + path);
    EXPECT_EQ(events[4].op, "unlink");
    EXPECT_EQ(events[4].detail, LogSegmentPath(path, 1));
    EXPECT_EQ(events[5].op, "durable_sync");
    EXPECT_EQ(events[5].detail, dir);
  }
  std::filesystem::remove_all(dir);
}

// If creating/syncing `<path>.capped`, syncing its directory, or renaming the
// active segment fails during rotate_to(), rotation aborts before renaming or
// unlinking the oldest segment, keeps head_dropped() false until the marker and
// directory sync succeed, and unlinks the staged `.next` file.
// REQ-OPS-09
// REQ-OPS-14
TEST(LogDurability,
     RotateToAbortsWithoutUnlinkingOrLeakingStagedWhenMarkerOrDirSyncFails) {
  enum class Failure {
    kMarkerOpen,
    kMarkerSync,
    kDirSyncAfterMarker,
    kRenameActive,
  };
  for (const Failure failure :
       {Failure::kMarkerOpen, Failure::kMarkerSync,
        Failure::kDirSyncAfterMarker, Failure::kRenameActive}) {
    SCOPED_TRACE(static_cast<int>(failure));
    const std::string dir = ::testing::TempDir() + "/talos_log_rotate_fail_" +
                            std::to_string(::getpid()) + "_" +
                            std::to_string(static_cast<int>(failure));
    std::filesystem::remove_all(dir);
    ASSERT_TRUE(std::filesystem::create_directories(dir));
    const std::string path = dir + "/node.tlog";
    const std::string staged = path + ".next";
    const std::string marker = CappedMarkerPath(path);
    const std::string seg0 = LogSegmentPath(path, 0);

    std::map<int, std::string> fd_paths;
    bool arm_failure = false;
    int rename_calls = 0;
    int seg0_unlinks = 0;

    LogFileSinkOps ops;
    ops.open = [&](const char* target, int flags, mode_t mode) {
      if (arm_failure && failure == Failure::kMarkerOpen &&
          std::string_view{target} == marker) {
        errno = ENOSPC;
        return -1;
      }
      const int fd = (flags & O_CREAT) != 0 ? ::open(target, flags, mode)
                                            : ::open(target, flags);
      if (fd >= 0) fd_paths[fd] = target;
      return fd;
    };
    ops.close = [&](int fd) {
      fd_paths.erase(fd);
      return ::close(fd);
    };
    ops.durable_sync = [&](int fd) {
      const auto it = fd_paths.find(fd);
      const std::string target = it != fd_paths.end() ? it->second : "";
      if (arm_failure && failure == Failure::kMarkerSync && target == marker) {
        errno = EIO;
        return -1;
      }
      if (arm_failure && failure == Failure::kDirSyncAfterMarker &&
          target == dir) {
        errno = EIO;
        return -1;
      }
      return process::DurableSync(fd);
    };
    ops.rename = [&](const char* from, const char* to) {
      if (arm_failure) {
        ++rename_calls;
        if (failure == Failure::kRenameActive &&
            std::string_view{from} == path) {
          errno = EIO;
          return -1;
        }
      }
      for (auto& [open_fd, mapped] : fd_paths) {
        if (mapped == from) mapped = to;
      }
      return ::rename(from, to);
    };
    ops.unlink = [&](const char* target) {
      if (arm_failure && std::string_view{target} == seg0) {
        ++seg0_unlinks;
      }
      return ::unlink(target);
    };

    {
      const std::vector<std::byte> prefix = MakePayload(128, 1);
      const std::vector<std::byte> body0 = MakePayload(32, 2);
      const std::vector<std::byte> body1 = MakePayload(48, 3);
      LogFileSink sink{path, ops};
      sink.open_sync();
      sink.configure_segments(prefix, /*segment_count=*/2);
      sink.write_prefix_sync();
      ASSERT_EQ(sink.write_bytes(body0.data(), body0.size()), body0.size());
      sink.rotate_to(1);
      ASSERT_FALSE(sink.failed()) << sink.error();
      ASSERT_TRUE(std::filesystem::exists(seg0));
      ASSERT_EQ(sink.write_bytes(body1.data(), body1.size()), body1.size());

      arm_failure = true;
      sink.rotate_to(2);
      EXPECT_TRUE(sink.failed());
      EXPECT_EQ(sink.file_segment(), 1u);
      EXPECT_EQ(seg0_unlinks, 0);
      EXPECT_TRUE(std::filesystem::exists(seg0));
      EXPECT_FALSE(std::filesystem::exists(staged))
          << "staged .next file leaked after rotation failure";
      if (failure != Failure::kRenameActive) {
        EXPECT_FALSE(sink.head_dropped());
        EXPECT_EQ(rename_calls, 0);
      } else {
        EXPECT_TRUE(sink.head_dropped());
      }
    }
    std::filesystem::remove_all(dir);
  }
}

std::vector<char> FileBytes(const std::string& path) {
  std::ifstream in{path, std::ios::binary};
  return {std::istreambuf_iterator<char>{in}, std::istreambuf_iterator<char>{}};
}

// The bytes after the file header of a log written with no manifest: its
// records. The header's wall-clock start differs between two logs.
std::vector<char> RecordBytes(const std::string& path) {
  std::vector<char> bytes = FileBytes(path);
  bytes.erase(bytes.begin(),
              bytes.begin() + static_cast<std::ptrdiff_t>(
                                  std::min(bytes.size(), sizeof(FileHeader))));
  return bytes;
}

// Holds a background writer in LogWriterOptions::before_write, standing for a
// disk that does not return: while closed, each write waits for a permit.
// entries() counts the writes begun, so with write_interval and
// sync_interval too long to come due, entry n + 1 shows n chunks written.
class WriteGate {
 public:
  std::function<void()> hook() {
    return [this] { enter(); };
  }
  void close() {
    const std::lock_guard lock{mutex_};
    open_ = false;
  }
  void open() {
    const std::lock_guard lock{mutex_};
    open_ = true;
    changed_.notify_all();
  }
  void permit() {
    const std::lock_guard lock{mutex_};
    ++permits_;
    changed_.notify_all();
  }
  std::uint64_t entries() const { return entries_.load(); }

 private:
  void enter() {
    entries_.fetch_add(1);
    std::unique_lock lock{mutex_};
    changed_.wait(lock, [this] { return open_ || permits_ > 0; });
    if (!open_) --permits_;
  }

  std::mutex mutex_;
  std::condition_variable changed_;
  bool open_{true};
  int permits_{0};
  std::atomic<std::uint64_t> entries_{0};
};

// Opens the gate when it leaves scope, before the writer declared ahead of it
// is destroyed (which waits for every write), however the test ended.
struct OpenAtExit {
  WriteGate& gate;
  ~OpenAtExit() { gate.open(); }
};

// A background writer of four 4096-byte chunks behind `gate`: 32 records of
// 128 bytes to a chunk, and nothing written but a handed-over chunk.
LogWriter::Options GatedOptions(WriteGate& gate, std::size_t chunks = 4) {
  LogWriter::Options options{4096, chunks, /*background=*/true};
  options.write_interval = std::chrono::hours{24};
  options.sync_interval = std::chrono::hours{24};
  options.before_write = gate.hook();
  return options;
}

constexpr std::size_t kGatedPerChunk = 4096 / (sizeof(RecordHeader) + 64);

// Dispatch `d`'s two records, a TIMER and a SEND of 64 bytes each, at
// 1000 ns per dispatch and 10 ns later.
void GatedTimer(LogWriter& writer, std::uint64_t d) {
  Context context{};
  context.kind = EventKind::TIMER;
  context.dispatch_index = d;
  context.event_time =
      MonotonicTime::from_nanos(static_cast<std::int64_t>(d) * 1000);
  context.now = MonotonicTime::from_nanos(context.event_time.nanos() + 10);
  context.sequence = d;
  const std::vector<std::byte> payload(64, std::byte{0x5a});
  writer.dispatch(context, payload);
}
void GatedSend(LogWriter& writer, std::uint64_t d) {
  Context context{};  // the dispatch's, as a handler's send passes it
  context.dispatch_index = d;
  context.event_time =
      MonotonicTime::from_nanos(static_cast<std::int64_t>(d) * 1000);
  context.now = MonotonicTime::from_nanos(context.event_time.nanos() + 10);
  const std::vector<std::byte> payload(64, std::byte{0xa5});
  writer.send(context, 2, payload, 0, d);
}
void GatedDispatch(LogWriter& writer, std::uint64_t d) {
  GatedTimer(writer, d);
  GatedSend(writer, d);
}
void GatedExit(LogWriter& writer, std::uint64_t d) {
  Context context{};
  context.kind = EventKind::EXIT;
  context.dispatch_index = d;
  context.event_time =
      MonotonicTime::from_nanos(static_cast<std::int64_t>(d) * 1000);
  context.now = MonotonicTime::from_nanos(context.event_time.nanos() + 10);
  writer.finish(context);
}

// The log `GatedDispatch` 1 to `last` and `GatedExit(last + 1)` make with
// every record kept, synchronously.
void WriteGatedReference(const std::string& path, std::uint64_t last) {
  LogWriter writer{path, "gated", LogWriter::Options{4096, 1, false}};
  writer.start({}, {});
  for (std::uint64_t d = 1; d <= last; ++d) GatedDispatch(writer, d);
  GatedExit(writer, last + 1);
  writer.flush();
}

// A disk that stops returning fills the pool and the loop never waits: once
// every chunk is handed over, records are dropped and counted, however many,
// as one gap. With one chunk written capture still does not resume; with two
// (half the pool) it resumes, but only at a record that starts a dispatch:
// the SEND of a dispatch whose TIMER was dropped is dropped too. Between that
// GAP and the next one the loop hands over half the pool again, and the
// second gap, which splits a dispatch, resumes at the next one once the disk
// returns. The log is every record before each gap, its GAP record, and every
// record after, byte for byte what CopyWithGap makes of a log that kept them
// all.
// REQ-RT-11
TEST(LogBackgroundWriter, FullPoolOpensAGapAndHalfAFreePoolResumesIt) {
  const std::string path = TempPath("full_pool_gap");
  const std::string whole = TempPath("full_pool_whole");
  const std::string copied = TempPath("full_pool_copied");
  const std::string copy = TempPath("full_pool_copy");
  WriteGate gate;
  std::uint64_t second_gap = 0;  // the dispatch the second gap splits
  {
    LogWriter writer{path, "gated", GatedOptions(gate)};
    const OpenAtExit open_at_exit{gate};
    writer.start({}, {});
    GatedDispatch(writer, 1);
    // Written: the writer has a write behind it, so from here on it writes
    // only a chunk handed over.
    writer.flush();
    const std::uint64_t base = gate.entries();
    gate.close();
    // Four chunks of 16 dispatches each, handed over and not written.
    for (std::uint64_t d = 2; d <= 65; ++d) GatedDispatch(writer, d);
    EXPECT_FALSE(writer.losing());
    EXPECT_EQ(writer.gaps(), 0u);
    // Dispatch 66 finds every chunk handed over; 500 more follow it.
    for (std::uint64_t d = 66; d <= 565; ++d) GatedDispatch(writer, d);
    EXPECT_TRUE(writer.losing());
    EXPECT_EQ(writer.gaps(), 1u);
    EXPECT_EQ(writer.lost_records(), 2u * 500);

    // One chunk written: three still handed over.
    ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 1; }));
    gate.permit();
    ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 2; }));
    GatedTimer(writer, 566);
    EXPECT_TRUE(writer.losing()) << "resumed with one chunk free";
    // Two written: half the pool free, but dispatch 566 lost its TIMER.
    gate.permit();
    ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 3; }));
    GatedSend(writer, 566);
    EXPECT_TRUE(writer.losing()) << "resumed inside a dropped dispatch";
    GatedTimer(writer, 567);
    EXPECT_FALSE(writer.losing());
    EXPECT_EQ(writer.gaps(), 1u);
    EXPECT_EQ(writer.lost_records(), 2u * 501);

    // Records taken until the pool is full again: two chunks, less the
    // GAP record's room. It fills inside a dispatch, after its TIMER.
    std::size_t taken = 1;  // dispatch 567's TIMER
    std::uint64_t d = 567;
    bool sent = false;
    for (std::size_t k = 0; !writer.losing() && k < 1000; ++k) {
      if (sent) {
        GatedTimer(writer, ++d);
      } else {
        GatedSend(writer, d);
      }
      sent = !sent;
      if (!writer.losing()) ++taken;
    }
    EXPECT_EQ(taken, 2 * kGatedPerChunk - 1);
    ASSERT_TRUE(sent) << "the second gap did not open at a SEND";
    second_gap = d;
    EXPECT_EQ(writer.gaps(), 2u);

    // The disk returns: once every handed-over chunk is written, the next
    // dispatch resumes capture.
    gate.open();
    ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 6; }));
    GatedDispatch(writer, second_gap + 1);
    EXPECT_FALSE(writer.losing());
    GatedExit(writer, second_gap + 2);
    writer.flush();
    EXPECT_EQ(writer.gaps(), 2u);
    EXPECT_EQ(writer.lost_records(), 2u * 501 + 1);
    EXPECT_FALSE(writer.failed()) << writer.error();
  }

  const ReadBack back = ReadAll(path, false);
  EXPECT_EQ(back.gaps, 2u);
  EXPECT_TRUE(back.damage.clean());
  // The first GAP, field by field: dispatches 66 to 566 dropped whole.
  const std::size_t first = std::size_t{2} * 65;
  ASSERT_GT(back.records.size(), first);
  const std::optional<LogGap> gap =
      GapOf(back.records[first].header, back.records[first].payload);
  ASSERT_TRUE(gap.has_value());
  EXPECT_EQ(gap->first_dispatch_index, 66u);
  EXPECT_EQ(gap->start_ns, 66'000);
  EXPECT_EQ(gap->resumed_ns, 567'010);
  EXPECT_EQ(gap->detail.cause, static_cast<std::uint16_t>(GapCause::POOL_FULL));
  EXPECT_EQ(gap->detail.flags, 0u);
  EXPECT_EQ(gap->detail.first_lost_ns, 66'010);
  EXPECT_EQ(gap->detail.last_lost_ns, 566'010);
  EXPECT_EQ(gap->detail.last_dispatch_index, 566u);
  EXPECT_EQ(gap->detail.records, 1002u);
  EXPECT_EQ(gap->detail.bytes, 1002u * (sizeof(RecordHeader) + 64));
  EXPECT_EQ(gap->detail.dispatches, 501u);
  EXPECT_EQ(gap->detail.sends, 501u);

  // Byte for byte what CopyWithGap makes of the whole log: the later gap
  // (the second dispatch's SEND) first, so the earlier indices hold.
  const std::uint64_t last = second_gap + 1;
  WriteGatedReference(whole, last);
  const std::size_t split_send = 2 * (second_gap - 1) + 1;
  const LogGap second = CopyWithGap(whole, copied, split_send, split_send + 1);
  EXPECT_EQ(second.detail.flags, GAP_SPLITS_DISPATCH);
  CopyWithGap(copied, copy, first, std::size_t{2} * 566);
  EXPECT_EQ(RecordBytes(path), RecordBytes(copy));
  for (const auto& file : {path, whole, copied, copy}) ::unlink(file.c_str());
}

// Half the pool is rounded up: a pool of three resumes with two chunks free,
// not one, so between two GAP records the loop still hands over at least
// half the pool.
// REQ-RT-11
TEST(LogBackgroundWriter, AnOddPoolResumesWithHalfOfItRoundedUpFree) {
  const std::string path = TempPath("odd_pool");
  WriteGate gate;
  {
    LogWriter writer{path, "gated", GatedOptions(gate, 3)};
    const OpenAtExit open_at_exit{gate};
    writer.start({}, {});
    GatedDispatch(writer, 1);
    writer.flush();
    const std::uint64_t base = gate.entries();
    gate.close();
    // Three chunks of 16 dispatches handed over; dispatch 50 is dropped.
    for (std::uint64_t d = 2; d <= 50; ++d) GatedDispatch(writer, d);
    ASSERT_TRUE(writer.losing());
    ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 1; }));
    gate.permit();
    ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 2; }));
    GatedDispatch(writer, 51);
    EXPECT_TRUE(writer.losing()) << "resumed with one chunk of three free";
    gate.permit();
    ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 3; }));
    GatedDispatch(writer, 52);
    EXPECT_FALSE(writer.losing());
    EXPECT_EQ(writer.gaps(), 1u);
  }
  ::unlink(path.c_str());
}

// A 2-chunk pool must never emit consecutive GAP records with zero non-GAP
// records between them: start() sizes chunks to fit GAP + the largest manifest
// record, and append() refuses to emit a GAP unless both GAP and the resuming
// record can be taken without exhausting the pool between them (whether due to
// chunk space or a segment rotation boundary).
// REQ-RT-11
TEST(LogBackgroundWriter,
     TwoChunkPoolNeverEmitsConsecutiveGapsWithZeroDataRecords) {
  const auto assert_no_consecutive_gaps = [](const std::string& log_path) {
    bool prev_gap = false;
    std::size_t gap_count = 0;
    for (const std::string& file : LogSegmentFiles(log_path)) {
      LogReader reader{file};
      LogReader::Record record{};
      while (reader.next(record)) {
        const bool is_gap =
            record.header.kind == static_cast<std::uint16_t>(EventKind::GAP);
        EXPECT_FALSE(prev_gap && is_gap)
            << "consecutive GAP records with zero data records between them in "
            << file;
        if (is_gap) ++gap_count;
        prev_gap = is_gap;
      }
    }
    return gap_count;
  };

  {
    SCOPED_TRACE("manifested message with tight chunk_bytes and chunk_count=2");
    const std::string path = TempPath("two_chunk_manifest_gap");
    WriteGate gate;
    constexpr std::size_t kPayload = 200;
    Manifest manifest = MakeManifest();
    manifest[0].message_bytes = kPayload;
    {
      LogWriter::Options options{256, 2, /*background=*/true};
      options.write_interval = std::chrono::hours{24 * 365 * 100};
      options.sync_interval = std::chrono::hours{1};
      options.before_write = gate.hook();
      LogWriter writer{path, "two_chunk_manifest", options};
      const OpenAtExit open_at_exit{gate};
      writer.start(manifest, {});
      Context context{};
      context.dispatch_index = 1;
      writer.dispatch(context, MakePayload(kPayload, 1));
      writer.flush();
      const std::uint64_t base = gate.entries();
      gate.close();

      // Fill both chunks and open a gap while the writer is blocked.
      std::uint64_t d = 2;
      while (!writer.losing() && d < 200) {
        context.dispatch_index = d;
        context.event_time =
            MonotonicTime::from_nanos(static_cast<std::int64_t>(d) * 1000);
        context.now = context.event_time;
        writer.dispatch(context, MakePayload(kPayload, static_cast<int>(d)));
        ++d;
      }
      ASSERT_TRUE(writer.losing());
      ASSERT_EQ(writer.gaps(), 1u);
      ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 1; }));

      // Free exactly 1 of 2 chunks (room_to_resume() is true for pool size 2).
      gate.permit();
      ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 2; }));

      // Dispatching a manifested 200-byte record must fit with GAP in that 1
      // chunk, resuming cleanly without opening a second gap.
      context.dispatch_index = d;
      context.event_time =
          MonotonicTime::from_nanos(static_cast<std::int64_t>(d) * 1000);
      context.now = context.event_time;
      writer.dispatch(context, MakePayload(kPayload, static_cast<int>(d)));
      EXPECT_FALSE(writer.losing());
      EXPECT_EQ(writer.gaps(), 1u);

      gate.open();
      writer.flush();
      EXPECT_EQ(writer.gaps(), 1u);
      EXPECT_FALSE(writer.failed()) << writer.error();
    }
    EXPECT_EQ(assert_no_consecutive_gaps(path), 1u);
    ::unlink(path.c_str());
  }

  {
    SCOPED_TRACE(
        "unmanifested record larger than chunk_bytes - GAP_RECORD_BYTES");
    const std::string path = TempPath("two_chunk_unmanifest_gap");
    WriteGate gate;
    // Empty manifest -> start() sizes chunks to max(256, 256) = 256.
    // A 160-byte payload gives record_total = 224 <= 256, but
    // GAP(128) + 224 = 352 > 256 so GAP and R cannot share a chunk.
    constexpr std::size_t kPayload = 160;
    {
      LogWriter::Options options{256, 2, /*background=*/true};
      options.write_interval = std::chrono::hours{24 * 365 * 100};
      options.sync_interval = std::chrono::hours{1};
      options.before_write = gate.hook();
      LogWriter writer{path, "two_chunk_unmanifest", options};
      const OpenAtExit open_at_exit{gate};
      writer.start({}, {});
      Context context{};
      context.dispatch_index = 1;
      writer.dispatch(context, MakePayload(kPayload, 1));
      writer.flush();
      const std::uint64_t base = gate.entries();
      gate.close();

      std::uint64_t d = 2;
      while (!writer.losing() && d < 200) {
        context.dispatch_index = d;
        context.event_time =
            MonotonicTime::from_nanos(static_cast<std::int64_t>(d) * 1000);
        context.now = context.event_time;
        writer.dispatch(context, MakePayload(kPayload, static_cast<int>(d)));
        ++d;
      }
      ASSERT_TRUE(writer.losing());
      ASSERT_EQ(writer.gaps(), 1u);
      ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 1; }));

      // Free 1 of 2 chunks: GAP + R cannot share one chunk and only 1 chunk is
      // free, so append() must keep the gap open rather than emitting a
      // GAP-only chunk and immediately reopening gap #2.
      gate.permit();
      ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 2; }));
      context.dispatch_index = d++;
      context.event_time =
          MonotonicTime::from_nanos(static_cast<std::int64_t>(d) * 1000);
      context.now = context.event_time;
      writer.dispatch(context, MakePayload(kPayload, static_cast<int>(d)));
      EXPECT_TRUE(writer.losing());
      EXPECT_EQ(writer.gaps(), 1u);

      // Once both chunks are free, GAP takes chunk 0 and R takes chunk 1.
      gate.open();
      ASSERT_TRUE(Eventually([&] {
        context.dispatch_index = d++;
        context.event_time =
            MonotonicTime::from_nanos(static_cast<std::int64_t>(d) * 1000);
        context.now = context.event_time;
        writer.dispatch(context, MakePayload(kPayload, static_cast<int>(d)));
        return !writer.losing();
      }));
      EXPECT_EQ(writer.gaps(), 1u);
      writer.flush();
      EXPECT_EQ(writer.gaps(), 1u);
      EXPECT_FALSE(writer.failed()) << writer.error();
    }
    EXPECT_EQ(assert_no_consecutive_gaps(path), 1u);
    ::unlink(path.c_str());
  }

  {
    SCOPED_TRACE("segment rotation between GAP and resuming record");
    const std::string path = TempPath("two_chunk_segment_gap");
    const std::string marker = CappedMarkerPath(path);
    for (const auto& f : LogSegmentFiles(path)) ::unlink(f.c_str());
    ::unlink(marker.c_str());
    WriteGate gate;
    constexpr std::size_t kPayload = 128;
    constexpr std::size_t kRecord = sizeof(RecordHeader) + kPayload;  // 192
    // Two segments, each holding prefix + 192 bytes: GAP (<=128) fits in a
    // fresh segment, but GAP + R (128 + 192 = 320 > 192) forces rotation!
    const std::uint64_t cap = 2 * (RecordsOffset() + kRecord);
    {
      LogWriter::Options options{4096, 2, /*background=*/true, cap};
      options.write_interval = std::chrono::hours{24 * 365 * 100};
      options.sync_interval = std::chrono::hours{1};
      options.before_write = gate.hook();
      LogWriter writer{path, "two_chunk_seg", options};
      const OpenAtExit open_at_exit{gate};
      writer.start(MakeManifest(), {});
      Context context{};
      context.dispatch_index = 1;
      writer.dispatch(context, MakePayload(kPayload, 1));
      writer.flush();
      const std::uint64_t base = gate.entries();
      gate.close();

      // Each 192-byte record rotates the segment and submits a chunk.
      // 2nd record fills chunk 0, 3rd rotates & fills chunk 1, 4th rotates &
      // hits kPoolFull -> opens gap #1.
      std::uint64_t d = 2;
      while (!writer.losing() && d < 50) {
        context.dispatch_index = d;
        context.event_time =
            MonotonicTime::from_nanos(static_cast<std::int64_t>(d) * 1000);
        context.now = context.event_time;
        writer.dispatch(context, MakePayload(kPayload, static_cast<int>(d)));
        ++d;
      }
      ASSERT_TRUE(writer.losing());
      ASSERT_EQ(writer.gaps(), 1u);
      ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 1; }));

      // Free 1 of 2 chunks: GAP fits in the fresh segment, then R would force
      // rotation to a second chunk that isn't free. Must NOT emit a GAP-only
      // chunk and open gap #2.
      gate.permit();
      ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 2; }));
      context.dispatch_index = d++;
      context.event_time =
          MonotonicTime::from_nanos(static_cast<std::int64_t>(d) * 1000);
      context.now = context.event_time;
      writer.dispatch(context, MakePayload(kPayload, static_cast<int>(d)));
      EXPECT_TRUE(writer.losing());
      EXPECT_EQ(writer.gaps(), 1u);

      gate.open();
      ASSERT_TRUE(Eventually([&] {
        context.dispatch_index = d++;
        context.event_time =
            MonotonicTime::from_nanos(static_cast<std::int64_t>(d) * 1000);
        context.now = context.event_time;
        writer.dispatch(context, MakePayload(kPayload, static_cast<int>(d)));
        return !writer.losing();
      }));
      EXPECT_EQ(writer.gaps(), 1u);
      writer.flush();
      EXPECT_EQ(writer.gaps(), 1u);
      EXPECT_FALSE(writer.failed()) << writer.error();
    }
    EXPECT_LE(assert_no_consecutive_gaps(path), 1u);
    for (const auto& f : LogSegmentFiles(path)) ::unlink(f.c_str());
    ::unlink(marker.c_str());
  }
}

// A gap still open at the stop is closed there: flush() or destruction, once
// the disk returns, writes its GAP record marked GAP_CLOSED_AT_FLUSH. When the
// run called finish() during the gap, the held EXIT follows it and gives it its
// time; when no finish() was called, the GAP takes the last dropped record's
// time and ends the log with no trailing EXIT. A second flush() adds nothing.
// REQ-OPS-44
TEST(LogBackgroundWriter, FlushOrDestructionClosesAnOpenGapAndKeepsTheExit) {
  for (const bool with_exit : {true, false}) {
    for (const bool flush : {true, false}) {
      SCOPED_TRACE(std::string{flush ? "flush" : "destruction"} +
                   (with_exit ? "_with_exit" : "_without_exit"));
      const std::string path = TempPath("closed_gap");
      const std::string whole = TempPath("closed_gap_whole");
      const std::string copy = TempPath("closed_gap_copy");
      WriteGate gate;
      {
        LogWriter writer{path, "gated", GatedOptions(gate)};
        const OpenAtExit open_at_exit{gate};
        writer.start({}, {});
        GatedDispatch(writer, 1);
        writer.flush();
        gate.close();
        for (std::uint64_t d = 2; d <= 70; ++d) GatedDispatch(writer, d);
        if (with_exit) GatedExit(writer, 71);
        EXPECT_TRUE(writer.losing());
        gate.open();
        if (flush) {
          writer.flush();
          EXPECT_FALSE(writer.losing());
          const auto size = std::filesystem::file_size(path);
          writer.flush();
          EXPECT_EQ(std::filesystem::file_size(path), size);
        }
      }
      const ReadBack back = ReadAll(path, false);
      ASSERT_EQ(back.records.size(), 2u * 65 + (with_exit ? 2u : 1u));
      EXPECT_EQ(back.gaps, 1u);
      EXPECT_TRUE(back.damage.clean());
      const ReadBack::Read& read = back.records[std::size_t{2} * 65];
      const std::optional<LogGap> gap = GapOf(read.header, read.payload);
      ASSERT_TRUE(gap.has_value());
      EXPECT_EQ(gap->detail.flags, GAP_CLOSED_AT_FLUSH);
      EXPECT_EQ(gap->detail.records, 2u * 5);
      if (with_exit) {
        EXPECT_EQ(gap->resumed_ns, 71'010);  // the held EXIT's
        EXPECT_EQ(back.records.back().header.kind,
                  static_cast<std::uint16_t>(EventKind::EXIT));

        WriteGatedReference(whole, 70);
        CopyWithGap(whole, copy, std::size_t{2} * 65, std::size_t{2} * 70 + 1);
        EXPECT_EQ(RecordBytes(path), RecordBytes(copy));
      } else {
        EXPECT_EQ(gap->resumed_ns, 70'010);  // last_lost_ns, no held EXIT
        EXPECT_EQ(gap->resumed_ns, gap->detail.last_lost_ns);
        EXPECT_EQ(back.records.back().header.kind,
                  static_cast<std::uint16_t>(EventKind::GAP));
      }
      for (const auto& file : {path, whole, copy}) ::unlink(file.c_str());
    }
  }
}

// The loop thread never waits on its log file: constructing, starting and
// recording into a background writer whose file cannot be opened yet (a FIFO
// with no reader blocks open() the way a create can on a busy disk) all
// return, and once the file opens it holds the header, then every record in
// order.
// REQ-RT-11 REQ-OPS-44
TEST(LogBackgroundWriter, ProducerNeverWaitsForTheLogFile) {
  const auto path = TempPath("unopened_fifo");
  ASSERT_EQ(::mkfifo(path.c_str(), 0600), 0);
  std::optional<LogWriter> writer;
  std::promise<void> recorded;
  std::thread producer{[&] {
    writer.emplace(path, "unopened", LogWriter::Options{4096, 2, true}, 9);
    writer->start(MakeManifest(), MonotonicTime::from_nanos(42));
    for (std::uint64_t i = 1; i <= 3; ++i) {
      Context context{};
      context.dispatch_index = i;
      writer->dispatch(context, MakePayload(16, static_cast<int>(i)));
    }
    recorded.set_value();
  }};
  // Nothing opens the FIFO's read end before this wait ends, so a producer
  // that waits for the file never records; the deadline only bounds that
  // failure (docs/TESTING.md §6.3).
  const bool waited =
      recorded.get_future().wait_for(std::chrono::seconds{10}) !=
      std::future_status::ready;
  // The reader lets the open finish, whichever thread is blocked in it.
  std::vector<std::byte> bytes;
  std::thread consumer{[&] {
    const int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) return;
    std::byte buffer[4096];
    for (;;) {
      const auto n = ::read(fd, buffer, sizeof(buffer));
      if (n <= 0) break;
      bytes.insert(bytes.end(), buffer, buffer + n);
    }
    ::close(fd);
  }};
  producer.join();
  EXPECT_FALSE(waited) << "the producer blocked until the log file opened";
  // No failed() check: Linux's fsync() refuses a FIFO (EINVAL), so the
  // durable flush reports an error there. The bytes below are the invariant.
  writer->flush();
  writer.reset();  // Closes the FIFO: the reader sees its end.
  consumer.join();
  ::unlink(path.c_str());

  const auto copy = TempPath("unopened_copy");
  {
    std::ofstream out{copy, std::ios::binary};
    out.write(reinterpret_cast<const char*>(bytes.data()),
              static_cast<std::streamsize>(bytes.size()));
  }
  LogReader reader{copy};
  EXPECT_EQ(reader.session_id(), 9u);
  EXPECT_EQ(reader.start_time().nanos(), 42);
  EXPECT_EQ(reader.manifest().size(), MakeManifest().size());
  LogReader::Record record{};
  for (std::uint64_t i = 1; i <= 3; ++i) {
    ASSERT_TRUE(reader.next(record));
    EXPECT_EQ(record.header.dispatch_index, i);
  }
  EXPECT_FALSE(reader.next(record));
  EXPECT_FALSE(reader.truncated());
  ::unlink(copy.c_str());
}

// A log file the writer cannot create does not stop the node: the
// constructor returns, records are dropped rather than waited on, the
// failure names the path, and the writer, with no file to sync, still stops
// once its periodic sync is due (a writer spinning on it never returns from
// the destructor).
// REQ-RT-11 REQ-OPS-44
TEST(LogBackgroundWriter, UncreatableLogFailsWithoutThrowing) {
  const std::string path = ::testing::TempDir() + "/talos_log_test_missing_" +
                           std::to_string(::getpid()) + "/node.tlog";
  std::optional<LogWriter> writer;
  const auto created = std::chrono::steady_clock::now();
  ASSERT_NO_THROW(writer.emplace(path, "missing", ShortSyncOptions()));
  writer->start(MakeManifest(), {});
  Context context{};
  context.dispatch_index = 1;
  writer->dispatch(context, MakePayload(16, 1));
  writer->flush();
  EXPECT_TRUE(writer->failed());
  EXPECT_NE(writer->error().find(path), std::string::npos) << writer->error();
  // Not a delay to let something happen: the case needs a sync to be due when
  // the writer stops, and one is due an interval after construction.
  ASSERT_TRUE(Eventually([&] {
    return std::chrono::steady_clock::now() - created > kShortSyncInterval;
  }));
  writer->flush();
  writer.reset();
}

// One record of a gap test: its kind, dispatch and payload size.
struct Step {
  EventKind kind;
  std::uint64_t dispatch;
  std::size_t bytes;
};

// Records step `k` of a sequence: dispatched at 1000 ns per dispatch index,
// made (its now) 10 ns per step later, a MESSAGE published 500 ns before its
// dispatch.
void RecordStep(LogWriter& writer, const Step& step, std::size_t k) {
  const auto at = static_cast<std::int64_t>(step.dispatch) * 1000;
  Context context{};
  context.kind = step.kind;
  context.source_id = 1;
  context.dispatch_index = step.dispatch;
  context.event_time = MonotonicTime::from_nanos(at);
  context.now =
      MonotonicTime::from_nanos(at + 10 * static_cast<std::int64_t>(k));
  if (step.kind == EventKind::MESSAGE) {
    context.publish_time = MonotonicTime::from_nanos(at - 500);
  }
  context.sequence = k;
  const std::vector<std::byte> payload =
      MakePayload(step.bytes, static_cast<int>(k) + 1);
  switch (step.kind) {
    case EventKind::SEND:
      writer.send(context, 2, payload, 0, k);
      break;
    case EventKind::EXIT:
      writer.finish(context);
      break;
    default:
      writer.dispatch(context, payload);
      break;
  }
}

// The record at `offset` of `path` replaced by the GAP `gap` with cause
// `cause`, sealed as the writer seals it.
void RewriteGapCause(const std::string& path, std::uint64_t offset,
                     const LogGap& gap, GapCause cause) {
  RecordHeader header{};
  header.kind = static_cast<std::uint16_t>(EventKind::GAP);
  header.source_id = GAP_SOURCE_ID;
  header.dispatch_index = gap.first_dispatch_index;
  header.event_time_ns = gap.start_ns;
  header.now_ns = gap.resumed_ns;
  GapRecord detail = gap.detail;
  detail.cause = static_cast<std::uint16_t>(cause);
  const std::span<const std::byte> stored = SealRecord(
      header, std::span<const std::byte>{
                  reinterpret_cast<const std::byte*>(&detail), sizeof(detail)});
  std::fstream file{path, std::ios::binary | std::ios::in | std::ios::out};
  file.seekp(static_cast<std::streamoff>(offset));
  file.write(reinterpret_cast<const char*>(&header), sizeof(header));
  file.write(reinterpret_cast<const char*>(stored.data()),
             static_cast<std::streamsize>(stored.size()));
}

// A record larger than the writer's buffers is dropped whole, without
// stopping capture: the synchronous writer opens a gap there, drops the rest
// of that dispatch, and resumes at the next record that starts one (or that
// belongs to none, as every record of the hardware bridge). The GAP record
// it writes before that record names what was dropped, field by field, and
// the log is byte for byte what CopyWithGap makes of one that kept every
// record, cause aside. Rows: the gap starts a dispatch; it splits a dispatch
// whose first records were taken; records of no dispatch.
// REQ-RT-11
TEST(LogGap, OversizedRecordOpensAGapTheNextDispatchCloses) {
  constexpr std::size_t kBuffer = 512;
  constexpr std::size_t kOversized = 1024;
  struct Row {
    const char* name;
    std::vector<Step> steps;
    std::size_t first;  // the oversized record, the first dropped
    std::size_t end;    // the record capture resumes with
    std::uint16_t flags;
    std::uint64_t dispatches;
    std::uint64_t sends;
    std::int64_t start_ns;  // the earliest time a dropped record names
  };
  const Row rows[] = {
      // A dropped MESSAGE names its publication, 500 ns before its drain:
      // the gap starts there, so nothing published inside it is fed past.
      {"starts_a_dispatch",
       {{EventKind::TIMER, 1, 20},
        {EventKind::SEND, 1, 20},
        {EventKind::MESSAGE, 2, kOversized},
        {EventKind::SEND, 2, 20},
        {EventKind::MESSAGE, 3, 20},
        {EventKind::SEND, 3, 20},
        {EventKind::EXIT, 4, 0}},
       2,
       4,
       0,
       1,
       1,
       1500},
      {"splits_a_dispatch",
       {{EventKind::TIMER, 1, 20},
        {EventKind::SEND, 1, 20},
        {EventKind::MESSAGE, 2, 20},
        {EventKind::SEND, 2, kOversized},
        {EventKind::SEND, 2, 20},
        {EventKind::TIMER, 3, 20},
        {EventKind::EXIT, 4, 0}},
       3,
       5,
       GAP_SPLITS_DISPATCH,
       0,
       2,
       2000},
      {"no_dispatch",
       {{EventKind::SEND, 0, 20},
        {EventKind::SEND, 0, kOversized},
        {EventKind::SEND, 0, 20},
        {EventKind::SEND, 0, 20}},
       1,
       2,
       0,
       0,
       1,
       0},
  };
  for (const Row& row : rows) {
    SCOPED_TRACE(row.name);
    const std::string path = TempPath(std::string{"oversized_"} + row.name);
    const std::string whole = TempPath(std::string{"whole_"} + row.name);
    const std::string copy = TempPath(std::string{"copy_"} + row.name);
    {
      LogWriter writer{path, "oversized", kBuffer};
      writer.start({}, {});
      for (std::size_t k = 0; k < row.steps.size(); ++k) {
        RecordStep(writer, row.steps[k], k);
        // Dropped from the oversized record until the one that resumes;
        // the log is incomplete from the first drop on.
        EXPECT_EQ(writer.losing(), k >= row.first && k < row.end)
            << "after step " << k;
        EXPECT_EQ(writer.incomplete(), k >= row.first) << "after step " << k;
      }
      writer.flush();
      EXPECT_EQ(writer.gaps(), 1u);
      EXPECT_EQ(writer.lost_records(), row.end - row.first);
      EXPECT_FALSE(writer.failed()) << writer.error();
      EXPECT_FALSE(writer.capture_stopped());
    }

    const ReadBack back = ReadAll(path, false);
    ASSERT_EQ(back.records.size(),
              row.steps.size() - (row.end - row.first) + 1);
    EXPECT_EQ(back.gaps, 1u);
    for (std::size_t i = 0; i < back.records.size(); ++i) {
      if (i == row.first) continue;  // the GAP, below
      // Before the gap, record i is step i; after it, the steps from `end`.
      const std::size_t step = i < row.first ? i : i - 1 + row.end - row.first;
      EXPECT_EQ(back.records[i].header.kind,
                static_cast<std::uint16_t>(row.steps[step].kind))
          << "record " << i;
      EXPECT_EQ(back.records[i].header.dispatch_index, row.steps[step].dispatch)
          << "record " << i;
    }
    const ReadBack::Read& read = back.records[row.first];
    const std::optional<LogGap> gap = GapOf(read.header, read.payload);
    ASSERT_TRUE(gap.has_value());
    ASSERT_TRUE(gap->detailed);
    const Step& lost = row.steps[row.first];
    const auto now_of = [&](std::size_t k) {
      return static_cast<std::int64_t>(row.steps[k].dispatch) * 1000 +
             10 * static_cast<std::int64_t>(k);
    };
    EXPECT_EQ(gap->first_dispatch_index, lost.dispatch);
    EXPECT_EQ(gap->start_ns, row.start_ns);
    EXPECT_EQ(gap->resumed_ns, now_of(row.end));
    const GapRecord& detail = gap->detail;
    EXPECT_EQ(detail.cause,
              static_cast<std::uint16_t>(GapCause::RECORD_TOO_LARGE));
    EXPECT_EQ(detail.flags, row.flags);
    EXPECT_EQ(detail.first_lost_ns, now_of(row.first));
    EXPECT_EQ(detail.last_lost_ns, now_of(row.end - 1));
    EXPECT_EQ(detail.last_dispatch_index, row.steps[row.end - 1].dispatch);
    EXPECT_EQ(detail.records, row.end - row.first);
    std::uint64_t bytes = 0;
    for (std::size_t k = row.first; k < row.end; ++k) {
      bytes += sizeof(RecordHeader) + row.steps[k].bytes;
    }
    EXPECT_EQ(detail.bytes, bytes);
    EXPECT_EQ(detail.dispatches, row.dispatches);
    EXPECT_EQ(detail.sends, row.sends);

    // The same records, every one kept, then marked by CopyWithGap.
    {
      LogWriter writer{whole, "oversized", 4 * kOversized};
      writer.start({}, {});
      for (std::size_t k = 0; k < row.steps.size(); ++k) {
        RecordStep(writer, row.steps[k], k);
      }
      writer.flush();
      ASSERT_EQ(writer.gaps(), 0u);
    }
    std::uint64_t offset = 0;
    {
      LogReader reader{whole};
      LogReader::Record record;
      for (std::size_t i = 0; i <= row.first; ++i) {
        ASSERT_TRUE(reader.next(record));
      }
      offset = record.offset;
    }
    const LogGap made = CopyWithGap(whole, copy, row.first, row.end);
    RewriteGapCause(copy, offset, made, GapCause::RECORD_TOO_LARGE);
    EXPECT_EQ(RecordBytes(path), RecordBytes(copy));
    ::unlink(path.c_str());
    ::unlink(whole.c_str());
    ::unlink(copy.c_str());
  }
}

// The registry's log-gap flag (Options::gap_flags) is set by the record that
// opens the gap, not by a later sample: a run killed right after that record
// has no GAP record, and the flag is all that says its log lost records. The
// word's other bits are kept. It is set only while the word is still the
// writer's: a row the registry swept and gave another node (a new
// generation) is left alone, so the new owner's run is not marked log_gap.
// REQ-OPS-52
TEST(LogGap, OpeningAGapSetsTheGapFlagAtOnceOnlyInItsOwnRow) {
  constexpr std::uint32_t kOther = 1u << 3;
  constexpr std::uint32_t kGap = 1u << 14;
  constexpr std::uint64_t kClaimed = 5;
  struct Row {
    const char* name;
    std::uint64_t generation_at_gap;
    std::uint32_t flags_after;
  };
  const Row rows[] = {
      {"its own row", kClaimed, kOther | kGap},
      {"claimed again", kClaimed + 1, kOther},
  };
  for (const Row& row : rows) {
    SCOPED_TRACE(row.name);
    std::atomic<std::uint32_t> flags{kOther};
    std::atomic<std::uint64_t> generation{kClaimed};
    const std::string path = TempPath("gap_flag");
    LogWriter::Options options;
    options.chunk_bytes = 512;
    options.chunk_count = 1;
    options.background = false;
    options.gap_flags = {.flags = &flags,
                         .bit = kGap,
                         .owner = &generation,
                         .expected = kClaimed};
    LogWriter writer{path, "gap_flag", options};
    writer.start({}, {});
    RecordStep(writer, {EventKind::TIMER, 1, 20}, 0);
    RecordStep(writer, {EventKind::SEND, 1, 20}, 1);
    EXPECT_EQ(flags.load(), kOther) << "set before any record was dropped";
    generation.store(row.generation_at_gap);
    RecordStep(writer, {EventKind::SEND, 1, 1024}, 2);
    ASSERT_TRUE(writer.losing());
    EXPECT_EQ(flags.load(), row.flags_after);
    writer.flush();
    ::unlink(path.c_str());
  }
}

TEST(LogBackgroundWriter, RepeatedFlushReusesChunksInOrder) {
  const auto path = TempPath("reuse_chunks");
  LogWriter writer{path, "reuse", LogWriter::Options{256, 4, true}};
  writer.start({}, {});
  for (std::uint64_t batch = 0; batch < 20; ++batch) {
    for (std::uint64_t i = 0; i < 8; ++i) {
      Context context{};
      context.dispatch_index = batch * 8 + i + 1;
      writer.dispatch(context, MakePayload(16, static_cast<int>(i)));
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }
  LogReader reader{path};
  LogReader::Record record{};
  for (std::uint64_t i = 1; i <= 160; ++i) {
    ASSERT_TRUE(reader.next(record));
    EXPECT_EQ(record.header.dispatch_index, i);
    EXPECT_EQ(record.payload.size(), 16);
  }
  EXPECT_FALSE(reader.next(record));
  ::unlink(path.c_str());
}

// An intact record from another log -- old disk blocks a power loss exposed
// -- is not where a scan resumes: its dispatch index and time go back.
TEST(LogCorruption, ScanSkipsAnIntactRecordFromAnotherLog) {
  const std::string path = TempPath("foreign_record");
  const std::string other = TempPath("foreign_record_source");
  const std::vector<ExpectedRecord> expected =
      WriteFixture(path, LogWriter::kDefaultBufferBytes);
  WriteFixture(other, LogWriter::kDefaultBufferBytes);

  const auto read_bytes = [](const std::string& file) {
    std::ifstream in{file, std::ios::binary};
    return std::vector<char>{std::istreambuf_iterator<char>{in},
                             std::istreambuf_iterator<char>{}};
  };
  const std::vector<char> log = read_bytes(path);
  const std::vector<char> source = read_bytes(other);
  // The other log's second record (dispatch 1), after garbage, inserted
  // before this log's record `target`, a later dispatch.
  const std::size_t target = expected.size() / 2;
  ASSERT_GT(expected[target].dispatch_index, 1u);
  const std::size_t foreign_offset = FixtureRecordOffset(expected, 1);
  const std::size_t foreign_bytes =
      sizeof(RecordHeader) + StoredSize(expected[1].payload);
  const std::size_t at = FixtureRecordOffset(expected, target);
  const std::vector<char> garbage(16, '\xff');
  {
    std::ofstream out{path, std::ios::binary | std::ios::trunc};
    out.write(log.data(), static_cast<std::streamsize>(at));
    out.write(garbage.data(), static_cast<std::streamsize>(garbage.size()));
    out.write(source.data() + foreign_offset,
              static_cast<std::streamsize>(foreign_bytes));
    out.write(log.data() + at, static_cast<std::streamsize>(log.size() - at));
  }

  LogReader reader{path};
  LogReader::Record record;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE(i);
    ASSERT_TRUE(reader.next(record));
    ExpectRecordEquals(expected[i], record);
  }
  EXPECT_FALSE(reader.next(record));
  EXPECT_EQ(reader.damage().spans, 1u);
  EXPECT_EQ(reader.damage().first_offset, at);
  EXPECT_EQ(reader.damage().skipped_bytes, garbage.size() + foreign_bytes);

  ::unlink(path.c_str());
  ::unlink(other.c_str());
}

// Appends a record whose header checksum is right, as anyone who crafts a
// log (or a writer that went wrong) can compute it.
void AppendRecord(const std::string& path, RecordHeader header,
                  std::span<const std::byte> stored) {
  header.magic = RECORD_MAGIC;
  header.stored_bytes = static_cast<std::uint32_t>(stored.size());
  header.crc32 = 0;
  header.crc32 =
      crc32(stored,
            crc32(std::span<const std::byte>{
                reinterpret_cast<const std::byte*>(&header), sizeof(header)}));
  std::ofstream out{path, std::ios::binary | std::ios::app};
  out.write(reinterpret_cast<const char*>(&header), sizeof(header));
  out.write(reinterpret_cast<const char*>(stored.data()),
            static_cast<std::streamsize>(stored.size()));
}

// A log from a USB stick may be crafted. A record claiming 4 GiB of zero
// padding (stored_bytes 0), with a checksum its author computed, is damage
// whatever its source -- a topic, a timer or an id the manifest does not
// list -- so no reader allocates what it claims.
// REQ-OPS-41
TEST(LogCorruption, RecordClaimingGigabytesIsDamage) {
  const std::string path = TempPath("huge_claim");
  for (const std::uint16_t source_id : {1, 0, 7}) {
    SCOPED_TRACE(source_id);
    const std::vector<ExpectedRecord> expected =
        WriteFixture(path, LogWriter::kDefaultBufferBytes);
    const std::size_t intact = FileSize(path);
    RecordHeader huge{};
    huge.kind = static_cast<std::uint16_t>(EventKind::MESSAGE);
    huge.source_id = source_id;
    huge.dispatch_index = 51;
    huge.now_ns = 60'000;
    huge.payload_bytes = 0xFFFF'FFFFu;
    AppendRecord(path, huge, {});

    LogReader reader{path};
    LogReader::Record record;
    std::size_t count = 0;
    while (reader.next(record)) ++count;
    EXPECT_EQ(count, expected.size());
    EXPECT_EQ(reader.damage().tail_offset, intact);
    EXPECT_EQ(reader.damage().tail_bytes, sizeof(RecordHeader));

    const LogSummary summary = SummarizeLog(path);
    EXPECT_EQ(summary.record_count, expected.size());
    EXPECT_EQ(summary.damage.tail_bytes, sizeof(RecordHeader));
  }
  ::unlink(path.c_str());
}

// A topic's records carry that topic's message: one claiming more than its
// source declares is damage, skipped like any other, and the record after
// it still reads.
// REQ-OPS-41
TEST(LogCorruption, RecordLargerThanItsSourceIsDamage) {
  const std::string path = TempPath("oversized_for_source");
  const std::vector<ExpectedRecord> expected =
      WriteFixture(path, LogWriter::kDefaultBufferBytes);
  const std::size_t bad_offset = FileSize(path);
  RecordHeader header{};
  header.kind = static_cast<std::uint16_t>(EventKind::SEND);
  header.source_id = 2;  // the sender: 32-byte messages
  header.dispatch_index = 51;
  header.now_ns = 60'000;
  header.payload_bytes = 33;
  const std::vector<std::byte> oversized = MakePayload(33, 5);
  AppendRecord(path, header, oversized);
  header.dispatch_index = 52;
  header.payload_bytes = 32;
  AppendRecord(path, header, MakePayload(32, 6));

  for (const bool windowed : {false, true}) {
    SCOPED_TRACE(windowed);
    std::vector<std::uint64_t> dispatches;
    LogDamage damage;
    if (windowed) {
      WindowedLogReader reader{path};
      LogReader::Record record;
      while (reader.next(record)) {
        dispatches.push_back(record.header.dispatch_index);
      }
      damage = reader.damage();
    } else {
      LogReader reader{path};
      LogReader::Record record;
      while (reader.next(record)) {
        dispatches.push_back(record.header.dispatch_index);
      }
      damage = reader.damage();
    }
    ASSERT_EQ(dispatches.size(), expected.size() + 1);
    EXPECT_EQ(dispatches.back(), 52u);
    EXPECT_EQ(damage.spans, 1u);
    EXPECT_EQ(damage.first_offset, bad_offset);
    EXPECT_EQ(damage.skipped_bytes, sizeof(RecordHeader) + oversized.size());
  }
  ::unlink(path.c_str());
}

// SummarizeLog agrees with LogReader read to its end, on a whole log, a
// truncated one, a damaged one, and the corruptions LogReader rejects.
TEST(LogSummary, MatchesTheReaderAndRejectsTheSameCorruption) {
  const std::string path = TempPath("summary");
  const std::vector<ExpectedRecord> expected =
      WriteFixture(path, LogWriter::kDefaultBufferBytes);
  LogSummary summary = SummarizeLog(path);
  EXPECT_EQ(summary.record_count, expected.size());
  EXPECT_FALSE(summary.truncated);

  ASSERT_EQ(0,
            ::truncate(path.c_str(), static_cast<off_t>(FileSize(path) - 20)));
  summary = SummarizeLog(path);
  LogReader reader{path};
  LogReader::Record record;
  while (reader.next(record)) {
  }
  EXPECT_EQ(summary.record_count, reader.record_count());
  EXPECT_TRUE(summary.truncated);
  EXPECT_TRUE(reader.truncated());

  WriteFixture(path, LogWriter::kDefaultBufferBytes);
  FlipByte(path, sizeof(FileHeader) + 10);  // the manifest block
  EXPECT_THROW(SummarizeLog(path), std::runtime_error);

  // A damaged record mid-log, then a zero-filled tail.
  WriteFixture(path, LogWriter::kDefaultBufferBytes);
  FlipByte(path, FixtureRecordOffset(expected, expected.size() / 2) + 8);
  {
    const std::vector<char> zeros(1000);
    std::ofstream out{path, std::ios::binary | std::ios::app};
    out.write(zeros.data(), static_cast<std::streamsize>(zeros.size()));
  }
  summary = SummarizeLog(path);
  LogReader damaged{path};
  while (damaged.next(record)) {
  }
  EXPECT_EQ(summary.record_count, expected.size() - 1);
  EXPECT_EQ(summary.record_count, damaged.record_count());
  EXPECT_TRUE(summary.truncated);
  EXPECT_EQ(summary.damage.spans, 1u);
  EXPECT_EQ(summary.damage.first_offset, damaged.damage().first_offset);
  EXPECT_EQ(summary.damage.skipped_bytes, damaged.damage().skipped_bytes);
  EXPECT_EQ(summary.damage.tail_bytes, 1000u);

  WriteFixture(path, LogWriter::kDefaultBufferBytes);
  FlipByte(path, 0);
  EXPECT_THROW(SummarizeLog(path), std::runtime_error);
  ::unlink(path.c_str());
}

// VisitLog hands over exactly the records LogReader returns, payloads
// zero-filled alike, past a damaged one; a manifest callback returning false
// skips the records, and a record callback returning false stops the walk.
TEST(LogSummary, VisitorSeesWhatTheReaderReturns) {
  const std::string path = TempPath("visit");
  const std::vector<ExpectedRecord> expected =
      WriteFixture(path, LogWriter::kDefaultBufferBytes);
  FlipByte(path, FixtureRecordOffset(expected, expected.size() / 2) + 8);

  std::vector<std::pair<RecordHeader, std::vector<std::byte>>> visited;
  std::size_t entries = 0;
  LogSummary summary =
      VisitLog(path, {.manifest =
                          [&](std::span<const ManifestEntry> manifest) {
                            entries = manifest.size();
                            return true;
                          },
                      .record =
                          [&](const LogReader::Record& record) {
                            visited.emplace_back(
                                record.header,
                                std::vector<std::byte>(record.payload.begin(),
                                                       record.payload.end()));
                            return true;
                          }});
  LogReader reader{path};
  EXPECT_EQ(entries, reader.manifest().size());
  LogReader::Record record;
  std::size_t i = 0;
  while (reader.next(record)) {
    SCOPED_TRACE(i);
    ASSERT_LT(i, visited.size());
    EXPECT_EQ(
        std::memcmp(&visited[i].first, &record.header, sizeof(RecordHeader)),
        0);
    EXPECT_TRUE(std::ranges::equal(visited[i].second, record.payload));
    ++i;
  }
  EXPECT_EQ(i, visited.size());
  EXPECT_EQ(summary.damage.spans, 1u);

  summary = VisitLog(path, {.manifest = [](auto) { return false; },
                            .record = [](const auto&) { return true; }});
  EXPECT_EQ(summary.record_count, 0u);
  summary = VisitLog(
      path, {.record = [](const LogReader::Record&) { return false; }});
  EXPECT_EQ(summary.record_count, 1u);
  ::unlink(path.c_str());
}

// What a walk of one log file hands its visitor: each record's offset,
// header and payload, the manifest's size, and the summary.
struct Walk {
  struct Visited {
    std::uint64_t offset{0};
    RecordHeader header{};
    std::vector<std::byte> payload;
  };
  std::vector<Visited> records;
  std::size_t manifest_entries{0};
  LogSummary summary;
};

Walk WalkLog(const std::string& path, std::uint64_t start_offset) {
  Walk walk;
  walk.summary =
      VisitLog(path,
               {.manifest =
                    [&](std::span<const ManifestEntry> entries) {
                      walk.manifest_entries = entries.size();
                      return true;
                    },
                .record =
                    [&](const LogReader::Record& record) {
                      walk.records.push_back(
                          {record.offset, record.header,
                           std::vector<std::byte>(record.payload.begin(),
                                                  record.payload.end())});
                      return true;
                    }},
               {.start_offset = start_offset});
  return walk;
}

// The session review server (proposed REQ-OPS-41, REQ-OPS-42) indexes a log
// by record offsets and later reads from one instead of from the start.
// Resuming at any offset a reader returned must hand over exactly the
// records, payloads and damage a read from the start does from that record
// on: in every segment of a rotated log, and before, at and after a damaged
// record.
TEST(LogResume, EveryReturnedOffsetResumesWhereAFullReadWas) {
  const std::string path = TempPath("resume");
  // Segments of ten 16-byte records (the cap's arithmetic is
  // ByteCapKeepsTheNewestRecords'); 50 records fill fewer than kLogSegments
  // segments, so none is deleted.
  const std::uint64_t cap =
      kLogSegments * (RecordsOffset() + 10 * (sizeof(RecordHeader) + 16));
  {
    LogWriter writer{path, "resume",
                     LogWriter::Options{256, 64, /*background=*/false, cap}};
    writer.start(MakeManifest(), {});
    for (std::uint64_t i = 1; i <= 50; ++i) {
      Context context{};
      context.kind = EventKind::MESSAGE;
      context.source_id = 1;
      context.dispatch_index = i;
      context.event_time =
          MonotonicTime::from_nanos(static_cast<std::int64_t>(i));
      writer.dispatch(context, MakePayload(16, static_cast<int>(i)));
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }
  const std::vector<std::string> files = LogSegmentFiles(path);
  ASSERT_GE(files.size(), 4u);

  // A record in the middle of a middle segment loses its first stored byte.
  const std::string& damaged_file = files[files.size() / 2];
  const Walk before_damage = WalkLog(damaged_file, 0);
  ASSERT_GE(before_damage.records.size(), 3u);
  const std::uint64_t damaged_offset =
      before_damage.records[before_damage.records.size() / 2].offset;
  FlipByte(damaged_file, damaged_offset + sizeof(RecordHeader));

  for (const std::string& file : files) {
    SCOPED_TRACE(file);
    const Walk full = WalkLog(file, 0);
    ASSERT_FALSE(full.records.empty());
    ASSERT_EQ(full.summary.damage.spans, file == damaged_file ? 1u : 0u);
    // LogReader, holding the whole file, reports the same offsets.
    LogReader whole{file};
    LogReader::Record record;
    for (const Walk::Visited& visited : full.records) {
      ASSERT_TRUE(whole.next(record));
      EXPECT_EQ(record.offset, visited.offset);
    }

    for (std::size_t i = 0; i < full.records.size(); ++i) {
      const std::uint64_t offset = full.records[i].offset;
      SCOPED_TRACE("resumed at record " + std::to_string(i) + ", byte " +
                   std::to_string(offset));
      const Walk resumed = WalkLog(file, offset);
      EXPECT_EQ(resumed.manifest_entries, MakeManifest().size());
      ASSERT_EQ(resumed.records.size(), full.records.size() - i);
      for (std::size_t k = 0; k < resumed.records.size(); ++k) {
        const Walk::Visited& expected = full.records[i + k];
        const Walk::Visited& actual = resumed.records[k];
        EXPECT_EQ(actual.offset, expected.offset) << "record " << i + k;
        EXPECT_EQ(
            std::memcmp(&actual.header, &expected.header, sizeof(RecordHeader)),
            0)
            << "record " << i + k;
        EXPECT_EQ(actual.payload, expected.payload) << "record " << i + k;
      }
      const bool skips_damage = file == damaged_file && offset < damaged_offset;
      EXPECT_EQ(resumed.summary.damage.spans, skips_damage ? 1u : 0u);
      EXPECT_EQ(resumed.summary.damage.skipped_bytes,
                skips_damage ? full.summary.damage.skipped_bytes : 0u);
      EXPECT_EQ(resumed.summary.record_count, resumed.records.size());
    }
  }
  for (const std::string& file : files) ::unlink(file.c_str());
  ::unlink(CappedMarkerPath(path).c_str());
}

// An offset that is not where an intact record starts is refused, naming the
// file, rather than read as a record or scanned past as damage: one byte
// into a record, a damaged record, the end of the file, and an intact record
// inside the schema block, which a reader never returns.
TEST(LogResume, OffsetThatIsNotARecordIsRefused) {
  const std::string path = TempPath("resume_refused");
  // A schema whose reflection bytes happen to hold an intact empty record.
  RecordHeader embedded{};
  embedded.magic = RECORD_MAGIC;
  embedded.crc32 = crc32(std::span<const std::byte>{
      reinterpret_cast<const std::byte*>(&embedded), sizeof(embedded)});
  Manifest manifest = MakeManifest();
  manifest[1].schema.bfbs.assign(reinterpret_cast<const char*>(&embedded),
                                 sizeof(embedded));
  const std::uint64_t embedded_offset =
      sizeof(FileHeader) + manifest.size() * sizeof(ManifestEntry) +
      manifest[0].schema.name.size() + manifest[0].schema.bfbs.size() +
      manifest[1].schema.name.size();
  std::vector<std::uint64_t> offsets;
  {
    LogWriter writer{path, "resume_refused", LogWriter::kDefaultBufferBytes};
    writer.start(manifest, {});
    for (std::uint64_t i = 1; i <= 3; ++i) {
      Context context{};
      context.kind = EventKind::MESSAGE;
      context.source_id = 1;
      context.dispatch_index = i;
      writer.dispatch(context, MakePayload(16, static_cast<int>(i)));
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }
  for (const auto& visited : WalkLog(path, 0).records) {
    offsets.push_back(visited.offset);
  }
  ASSERT_EQ(offsets.size(), 3u);
  FlipByte(path, offsets[2] + sizeof(RecordHeader));

  const struct {
    const char* what;
    std::uint64_t offset;
  } cases[] = {
      {"an intact record in the schema block", embedded_offset},
      {"one byte into a record", offsets[0] + 1},
      {"a damaged record", offsets[2]},
      {"the end of the file", FileSize(path)},
  };
  for (const auto& bad : cases) {
    SCOPED_TRACE(bad.what);
    WindowedLogReader reader{path};
    try {
      reader.seek(bad.offset);
      ADD_FAILURE() << "seek(" << bad.offset << ") was accepted";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string_view{error.what()}.find(path),
                std::string_view::npos)
          << error.what();
    }
  }
  // The records themselves still resume.
  WindowedLogReader reader{path};
  reader.seek(offsets[1]);
  LogReader::Record record;
  ASSERT_TRUE(reader.next(record));
  EXPECT_EQ(record.header.dispatch_index, 2u);
  ::unlink(path.c_str());
}

long PeakRssKiB() {
  rusage usage{};
  ::getrusage(RUSAGE_SELF, &usage);
#ifdef __APPLE__
  return usage.ru_maxrss / 1024;  // macOS reports bytes.
#else
  return usage.ru_maxrss;
#endif
}

// Counting a log streams it: peak memory does not grow by the log's size.
TEST(LogSummary, PeakMemoryDoesNotScaleWithTheLog) {
  const std::string path = TempPath("summary_large");
  constexpr std::size_t kRecords = std::size_t{64} * 1024;
  constexpr std::size_t kPayload = 1000;  // ~68 MB of records
  {
    Manifest manifest = MakeManifest();
    manifest[1].message_bytes = kPayload;
    LogWriter writer{path, "large",
                     LogWriter::Options{1u << 20, 1, /*background=*/false}};
    writer.start(manifest, {});
    const std::vector<std::byte> payload = MakePayload(kPayload, 1);
    Context context{};
    context.source_id = 1;
    for (std::size_t i = 0; i < kRecords; ++i) {
      context.dispatch_index = i;
      writer.dispatch(context, payload);
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }
  ASSERT_GT(FileSize(path), 64u << 20);
  const long before = PeakRssKiB();
  const LogSummary summary = SummarizeLog(path);
  const long grown = PeakRssKiB() - before;
  EXPECT_EQ(summary.record_count, kRecords);
  EXPECT_FALSE(summary.truncated);
  EXPECT_LT(grown, 8 * 1024) << "peak RSS grew by " << grown << " KiB";
  ::unlink(path.c_str());
}

// A capped log keeps the session's tail: past its cap the oldest segment
// goes, every kept file still reads in full, the kept records run without a
// gap to the last one written, and `<path>.capped` marks the missing head.
TEST(LogCapture, ByteCapKeepsTheNewestRecords) {
  for (const bool background : {false, true}) {
    SCOPED_TRACE(background);
    const auto path = TempPath(background ? "tail_bg" : "tail_sync");
    const std::string marker = CappedMarkerPath(path);
    ::unlink(marker.c_str());
    // Four segments, each the prefix and two 16-byte records (the largest
    // source, 128 bytes, sets the segment count).
    const std::uint64_t cap =
        4 * (RecordsOffset() + sizeof(RecordHeader) + 128);
    constexpr std::uint64_t kRecords = 40;
    {
      LogWriter writer{path, "tail",
                       LogWriter::Options{256, 64, background, cap}};
      writer.start(MakeManifest(), {});
      for (std::uint64_t i = 1; i <= kRecords; ++i) {
        Context context{};
        context.dispatch_index = i;
        writer.dispatch(context, MakePayload(16, static_cast<int>(i)));
      }
      writer.flush();
      EXPECT_TRUE(writer.capped());
      EXPECT_FALSE(writer.capture_stopped());
      EXPECT_FALSE(writer.failed()) << writer.error();
    }
    const std::vector<std::string> files = LogSegmentFiles(path);
    ASSERT_EQ(files.size(), 4u);
    EXPECT_EQ(files.back(), path);
    std::uint64_t total = 0;
    std::vector<std::uint64_t> kept;
    for (const auto& file : files) {
      total += FileSize(file);
      LogReader reader{file};
      LogReader::Record record;
      while (reader.next(record)) kept.push_back(record.header.dispatch_index);
      EXPECT_FALSE(reader.truncated()) << file;
    }
    EXPECT_LE(total, cap);
    ASSERT_GE(kept.size(), 6u);
    for (std::size_t i = 0; i < kept.size(); ++i) {
      EXPECT_EQ(kept[i], kRecords - kept.size() + 1 + i);
    }
    EXPECT_TRUE(std::ifstream{marker}.good());
    for (const auto& file : files) ::unlink(file.c_str());
    ::unlink(marker.c_str());
  }
}

// A cap too small for two segments cannot rotate: the writer stops at it,
// leaves a log that still reads in full, is not failed(), and marks the log
// incomplete with `<path>.capped`.
TEST(LogCapture, ByteCapStopsCaptureAndLeavesAValidMarkedLog) {
  for (const bool background : {false, true}) {
    SCOPED_TRACE(background);
    const auto path = TempPath(background ? "capped_bg" : "capped_sync");
    const std::string marker = CappedMarkerPath(path);
    ::unlink(marker.c_str());
    constexpr std::size_t kRecord = sizeof(RecordHeader) + 16;
    const std::uint64_t cap = RecordsOffset() + 3 * kRecord + kRecord / 2;
    {
      LogWriter writer{path, "capped",
                       LogWriter::Options{256, 4, background, cap}};
      writer.start(MakeManifest(), {});
      for (std::uint64_t i = 1; i <= 10; ++i) {
        Context context{};
        context.dispatch_index = i;
        writer.dispatch(context, MakePayload(16, static_cast<int>(i)));
      }
      EXPECT_TRUE(writer.capped());
      EXPECT_TRUE(writer.capture_stopped());
      EXPECT_FALSE(writer.failed()) << writer.error();
    }
    EXPECT_LE(FileSize(path), cap);
    EXPECT_TRUE(std::ifstream{marker}.good());
    LogReader reader{path};
    LogReader::Record record;
    while (reader.next(record)) {
    }
    EXPECT_EQ(reader.record_count(), 3u);
    EXPECT_FALSE(reader.truncated());
    EXPECT_EQ(SummarizeLog(path).record_count, 3u);
    ::unlink(path.c_str());
    ::unlink(marker.c_str());
  }
}

// A gap whose GAP record would cross the cap of a log that cannot rotate
// ends capture there: no gap is open any more (losing() false: nothing will
// resume it), capture has stopped, and the gap is still counted, so
// incomplete() reports the loss its missing GAP record cannot.
// REQ-RT-11
TEST(LogCapture, AGapWhoseResumeCrossesTheCapStopsCapture) {
  for (const bool background : {false, true}) {
    SCOPED_TRACE(background);
    const auto path = TempPath(background ? "gap_cap_bg" : "gap_cap_sync");
    const std::string marker = CappedMarkerPath(path);
    ::unlink(marker.c_str());
    constexpr std::size_t kRecord = sizeof(RecordHeader) + 16;
    // Room for one record and a GAP record's header, not its payload.
    const std::uint64_t cap = RecordsOffset() + kRecord + sizeof(RecordHeader);
    {
      LogWriter writer{path, "gap_cap",
                       LogWriter::Options{256, 4, background, cap}};
      writer.start(MakeManifest(), {});
      Context context{};
      context.dispatch_index = 1;
      writer.dispatch(context, MakePayload(16, 1));
      // Larger than any chunk: dropped, a gap opens.
      context.dispatch_index = 2;
      writer.dispatch(context, MakePayload(1024, 2));
      ASSERT_TRUE(writer.losing());
      ASSERT_FALSE(writer.capture_stopped());
      // A new dispatch resumes capture behind a GAP record the cap refuses.
      context.dispatch_index = 3;
      writer.dispatch(context, MakePayload(16, 3));
      EXPECT_FALSE(writer.losing());
      EXPECT_TRUE(writer.capture_stopped());
      EXPECT_EQ(writer.gaps(), 1u);
      EXPECT_TRUE(writer.incomplete());
      EXPECT_FALSE(writer.failed()) << writer.error();
      writer.flush();
    }
    EXPECT_TRUE(std::ifstream{marker}.good());
    const ReadBack back = ReadAll(path, false);
    ASSERT_EQ(back.records.size(), 1u);
    EXPECT_EQ(back.records[0].header.dispatch_index, 1u);
    EXPECT_EQ(back.gaps, 0u);
    ::unlink(path.c_str());
    ::unlink(marker.c_str());
  }
}

TEST(LogCapture, BudgetTooSmallForHeaderDoesNotGrowOrStopTheNode) {
  for (const bool background : {false, true}) {
    SCOPED_TRACE(background);
    const auto path = TempPath(background ? "no_header_bg" : "no_header_sync");
    {
      LogWriter writer{path, "tiny", LogWriter::Options{256, 1, background, 1}};
      writer.start(MakeManifest(), {});
      writer.dispatch({}, MakePayload(16, 1));
      writer.flush();
      EXPECT_TRUE(writer.capped());
      EXPECT_FALSE(writer.failed()) << writer.error();
    }
    EXPECT_FALSE(std::ifstream{path}.good());
    EXPECT_TRUE(std::ifstream{CappedMarkerPath(path)}.good());
    EXPECT_EQ(FileSize(CappedMarkerPath(path)), 0u);
    ::unlink(CappedMarkerPath(path).c_str());
  }
}

// A capped log written with `options`, one 16-byte record at each of
// `times_ns` (its record time, Context::now). Returns the kept files.
std::vector<std::string> WriteTimedLog(const std::string& path,
                                       const LogWriter::Options& options,
                                       const std::vector<std::int64_t>& times) {
  for (const auto& file : LogSegmentFiles(path)) ::unlink(file.c_str());
  ::unlink(CappedMarkerPath(path).c_str());
  {
    LogWriter writer{path, "timed", options};
    writer.start(MakeManifest(), {});
    for (std::size_t i = 0; i < times.size(); ++i) {
      Context context{};
      context.dispatch_index = i + 1;
      context.now = MonotonicTime::from_nanos(times[i]);
      writer.dispatch(context, MakePayload(16, static_cast<int>(i + 1)));
    }
    writer.flush();
    EXPECT_FALSE(writer.failed()) << writer.error();
    EXPECT_FALSE(writer.capture_stopped());
  }
  return LogSegmentFiles(path);
}

// Seconds from the first kept record to the last.
double KeptSpanSeconds(const std::vector<std::string>& files) {
  std::int64_t first = 0;
  std::int64_t last = 0;
  bool any = false;
  for (const auto& file : files) {
    LogReader reader{file};
    LogReader::Record record;
    while (reader.next(record)) {
      if (!any) first = record.header.now_ns;
      last = record.header.now_ns;
      any = true;
    }
    EXPECT_FALSE(reader.truncated()) << file;
  }
  return static_cast<double>(last - first) / 1e9;
}

std::vector<std::int64_t> EveryPeriod(std::int64_t period_ns,
                                      std::int64_t until_ns) {
  std::vector<std::int64_t> times;
  for (std::int64_t t = 0; t < until_ns; t += period_ns) times.push_back(t);
  return times;
}

// REQ-OPS-32. With a window W, a capped log keeps between 7/8 of W and W of
// its newest record time whatever its rate, while its bytes fit: a log at
// 10 records/s and one at 1000/s, 10.5 s each, under a cap that holds 8 s of
// the fast one, both keep 7 to 8 s of a W = 8 s window. Without the window
// (or uncapped) the slow log keeps all of its 10.5 s, as before; a cap too
// small for W / 8 of the fast log's records still ends its segments on
// bytes, so it keeps less.
TEST(LogCapture, WindowEndsSegmentsOnRecordTime) {
  constexpr std::int64_t kSecond = 1'000'000'000;
  constexpr std::int64_t kRecord = sizeof(RecordHeader) + 16;
  const auto slow = EveryPeriod(kSecond / 10, 10 * kSecond + kSecond / 2);
  const auto fast = EveryPeriod(kSecond / 1000, 10 * kSecond + kSecond / 2);
  // Eight segments, each 1.25 s of the fast log's records.
  const std::uint64_t cap =
      8 * (RecordsOffset() + static_cast<std::uint64_t>(1250 * kRecord));
  // Synchronous: the rule is the producer's, the same for a background
  // writer, which here could run out of chunks with sixty rotations queued.
  LogWriter::Options options{64u << 10, 1, /*background=*/false, cap};
  options.window = std::chrono::seconds{8};
  const auto path = TempPath("window");
  for (const auto* times : {&slow, &fast}) {
    const auto files = WriteTimedLog(path, options, *times);
    EXPECT_EQ(files.size(), kLogSegments);
    // From the first record of the oldest kept segment, seven segments of
    // 1 s at least, to the last record, 8 s at most.
    const double span = KeptSpanSeconds(files);
    EXPECT_GE(span, 7.0) << times->size() << " records";
    EXPECT_LE(span, 8.0) << times->size() << " records";
    EXPECT_TRUE(std::ifstream{CappedMarkerPath(path)}.good());
  }

  // Bytes still bind: half the cap holds under 5 s of the fast log in all.
  LogWriter::Options small = options;
  small.max_bytes = cap / 2;
  std::uint64_t total = 0;
  const auto bound = WriteTimedLog(path, small, fast);
  for (const auto& file : bound) total += FileSize(file);
  EXPECT_LE(total, small.max_bytes);
  EXPECT_LT(KeptSpanSeconds(bound), 5.0);

  // No window, or no cap: the slow log is one file, all 10.5 s of it.
  LogWriter::Options bytes_only = options;
  bytes_only.window = {};
  LogWriter::Options uncapped = options;
  uncapped.max_bytes = 0;
  for (const auto& unrotated : {bytes_only, uncapped}) {
    const auto files = WriteTimedLog(path, unrotated, slow);
    EXPECT_EQ(files.size(), 1u);
    EXPECT_GT(KeptSpanSeconds(files), 10.0);
  }
  for (const auto& file : LogSegmentFiles(path)) ::unlink(file.c_str());
  ::unlink(CappedMarkerPath(path).c_str());
}

// The time rule compares each record with its segment's first, by record
// time only: a clock that does not advance or a record older than the
// segment's first never ends it; the first record a window / n past it does,
// however long the gap.
TEST(LogCapture, WindowIgnoresAClockThatDoesNotAdvance) {
  constexpr std::int64_t kSecond = 1'000'000'000;
  const auto path = TempPath("window_clock");
  LogWriter::Options options{256, 1, /*background=*/false,
                             8 * (RecordsOffset() + 16384)};
  options.window = std::chrono::seconds{8};
  // A frozen clock, then records older than the segment's first, then one a
  // 30 s gap later, which starts the second segment.
  std::vector<std::int64_t> times(100, 5 * kSecond);
  times.insert(times.end(), {4 * kSecond, 0, 5 * kSecond + kSecond / 2,
                             35 * kSecond, 35 * kSecond});
  const auto files = WriteTimedLog(path, options, times);
  ASSERT_EQ(files.size(), 2u);
  LogReader reader{files.back()};
  LogReader::Record record;
  std::vector<std::int64_t> newest;
  while (reader.next(record)) newest.push_back(record.header.now_ns);
  EXPECT_EQ(newest, (std::vector<std::int64_t>{35 * kSecond, 35 * kSecond}));
  for (const auto& file : files) ::unlink(file.c_str());
}

// REQ-OPS-32. Every rotation syncs durably on the writer thread and takes a
// chunk from the pool, so a window too small for that (a 1 ns window, or a
// budget whose window collapsed) once ended a segment at every record, and
// a pool exhausted by rotations drops records. No window
// ends a segment sooner than sync_interval of record time after its first
// record: 10.5 s of records every 1 ms under a 1 ns window keeps eight
// segments that each start at least sync_interval after the one before.
TEST(LogCapture, WindowNeverEndsASegmentSoonerThanTheSyncInterval) {
  constexpr std::int64_t kSecond = 1'000'000'000;
  constexpr std::int64_t kRecord = sizeof(RecordHeader) + 16;
  const auto path = TempPath("window_floor");
  // Room for 2 s of records per segment: only time can end one sooner.
  LogWriter::Options options{
      64u << 10, 1, /*background=*/false,
      8 * (RecordsOffset() + static_cast<std::uint64_t>(2000 * kRecord))};
  options.window = std::chrono::nanoseconds{1};
  options.sync_interval = std::chrono::milliseconds{500};
  const auto files = WriteTimedLog(
      path, options, EveryPeriod(kSecond / 1000, 10 * kSecond + kSecond / 2));
  ASSERT_EQ(files.size(), kLogSegments);
  std::vector<std::int64_t> starts;
  for (const auto& file : files) {
    LogReader reader{file};
    LogReader::Record record;
    ASSERT_TRUE(reader.next(record)) << file;
    starts.push_back(record.header.now_ns);
  }
  for (std::size_t i = 1; i < starts.size(); ++i) {
    EXPECT_EQ(starts[i] - starts[i - 1], kSecond / 2) << files[i];
  }
  for (const auto& file : files) ::unlink(file.c_str());
  ::unlink(CappedMarkerPath(path).c_str());
}

TEST(LogCapture, UncappedWriterLeavesNoMarker) {
  const auto path = TempPath("uncapped");
  WriteFixture(path, LogWriter::kDefaultBufferBytes);
  EXPECT_FALSE(std::ifstream{CappedMarkerPath(path)}.good());
  ::unlink(path.c_str());
}

// write_interval bounds how long a partly filled chunk waits, never a full
// one: a chunk handed over while the writer sleeps until its next timed
// write must wake it. It used to wait out the interval, so a producer that
// filled the pool within one write_interval (a replay at full speed: 1554
// of 5998 States logged on macOS) stopped capture with the pool exhausted.
// Here write_interval is a century, longer than any host has been up (the
// steady clock counts from boot, and a writer that took "never wrote" for
// "wrote at boot" passed this on a workstation and failed on a freshly
// booted CI machine), and the writer has a record waiting for it; every
// chunk handed over must still reach the file, one at a time,
// for more than three times the pool's chunks. The pool holds three: the
// file shows chunk k written before the writer returns it to the pool, so
// with two the producer's next hand-over could find the pool full; with
// three it needs only chunk k - 1, which the writer returned before it
// wrote chunk k.
// REQ-RT-11
TEST(LogBackgroundWriter, FullChunkWakesAWriterWaitingToWrite) {
  const auto path = TempPath("full_chunk_wakes");
  constexpr std::size_t kChunkBytes = 4096;
  constexpr std::size_t kRecordBytes = sizeof(RecordHeader) + 64;
  constexpr std::size_t kPerChunk = kChunkBytes / kRecordBytes;
  constexpr std::size_t kChunks = 10;
  const std::vector<std::byte> payload(64, std::byte{0x5a});
  const auto file_size = [&] {
    struct stat status {};
    return ::stat(path.c_str(), &status) == 0
               ? static_cast<std::size_t>(status.st_size)
               : std::size_t{0};
  };
  std::size_t records = 0;
  {
    LogWriter::Options options{kChunkBytes, 3, /*background=*/true};
    options.write_interval = std::chrono::hours{24 * 365 * 100};
    options.sync_interval = std::chrono::hours{1};
    LogWriter writer{path, "full_chunk", options};
    writer.start({}, {});
    writer.flush();  // The writer thread creates the file.
    const std::size_t base = file_size();
    const auto log_one = [&] {
      Context context{};
      context.dispatch_index = ++records;
      writer.dispatch(context, payload);
    };
    // Found idle and never written before: written at once. The next
    // record waits out the interval.
    log_one();
    ASSERT_TRUE(Eventually([&] { return file_size() >= base + kRecordBytes; }));
    log_one();
    for (std::size_t chunk = 1; chunk <= kChunks; ++chunk) {
      // Fill the active chunk; the record that does not fit hands it over.
      while (records < chunk * kPerChunk + 1) log_one();
      ASSERT_TRUE(Eventually([&] {
        return file_size() >= base + chunk * kPerChunk * kRecordBytes;
      })) << "chunk "
          << chunk << " of " << kChunks
          << " handed over and not written in 10 s (pool of 3)";
      ASSERT_FALSE(writer.failed()) << writer.error();
    }
  }
  LogReader reader{path};
  LogReader::Record record{};
  std::size_t read = 0;
  while (reader.next(record)) {
    ASSERT_EQ(record.header.dispatch_index, ++read);
  }
  EXPECT_EQ(read, records);
  ::unlink(path.c_str());
}

// An owner stopped by destruction alone (no flush()) keeps every record it
// committed, even when the stop lands between the writer's read of the
// hand-off count and its stop check: ~State hands the last chunk over and
// then stops, so the writer must not take the stop for "nothing left".
// REQ-OPS-44
TEST(LogBackgroundWriter, StopAfterTheWritersHandOffCheckKeepsTheLastChunk) {
  const auto path = TempPath("stop_after_check");
  constexpr std::uint64_t kRecords = 5;
  const std::vector<std::byte> payload(16, std::byte{0x5a});
  std::atomic<bool> holding{false};
  {
    LogWriter::Options options{4096, 3, /*background=*/true};
    // Never due: only the stop's hand-off can write these records.
    options.write_interval = std::chrono::hours{24 * 365 * 100};
    options.sync_interval = std::chrono::hours{1};
    // Holds the writer past its hand-off check until the stop has begun.
    options.before_stop_check = [&](const std::atomic<bool>& stopping) {
      holding.store(true);
      while (!stopping.load()) std::this_thread::yield();
    };
    LogWriter writer{path, "stop_after_check", options};
    writer.start({}, {});
    // The writer has found nothing handed over and waits at its stop check;
    // records committed now are in the active chunk, which only the
    // destructor hands over.
    while (!holding.load()) std::this_thread::yield();
    for (std::uint64_t i = 1; i <= kRecords; ++i) {
      Context context{};
      context.dispatch_index = i;
      writer.dispatch(context, payload);
    }
    ASSERT_FALSE(writer.failed()) << writer.error();
  }
  LogReader reader{path};
  LogReader::Record record{};
  std::uint64_t read = 0;
  while (reader.next(record)) {
    ASSERT_EQ(record.header.dispatch_index, ++read);
  }
  EXPECT_EQ(read, kRecords);
  ::unlink(path.c_str());
}

// A node whose records come less often than write_interval (here 50 ms
// apart) finds the writer idle each time: the record's post wakes it
// and it writes at once, one wakeup per record. A burst first (records 2 ms
// apart, as one tick's several records can be) ends the way a stream does:
// a write a timed wait held back, then one empty timed wake. That wake used
// to leave the writer marked as streaming, so every sparse record after it
// cost a second, empty, timed wakeup. Counted with wakeups(), which a write
// blocked on the disk cannot inflate.
// REQ-RT-21
TEST(LogBackgroundWriter, SparseRecordsWakeTheWriterOncePerRecord) {
  const auto path = TempPath("sparse_wakeups");
  {
    LogWriter::Options options{};
    options.sync_interval = std::chrono::hours{1};
    LogWriter writer{path, "sparse", options};
    writer.start({}, {});
    writer.flush();  // The writer thread creates the file.
    std::uint64_t index = 0;
    const auto log_one = [&] {
      Context context{};
      context.dispatch_index = ++index;
      writer.dispatch(context, MakePayload(16, static_cast<int>(index)));
    };
    for (int i = 0; i < 25; ++i) {
      log_one();
      std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    // The burst's last write and its empty timed wake are over by now.
    std::this_thread::sleep_for(std::chrono::milliseconds{30});
    constexpr std::uint64_t kRecords = 10;
    const std::uint64_t start = writer.wakeups();
    for (std::uint64_t i = 0; i < kRecords; ++i) {
      log_one();
      // At least 50 ms apart however loaded the host. Only a writer
      // delayed past 40 ms could find the next record inside its
      // write_interval and add a timed wake.
      std::this_thread::sleep_for(std::chrono::milliseconds{50});
    }
    const std::uint64_t wakeups = writer.wakeups() - start;
    // One per record, plus two for a late burst wake or a delayed writer;
    // the empty timed wake made it 2 per record.
    EXPECT_LE(wakeups, kRecords + 2)
        << kRecords << " records 50 ms apart after a burst, write_interval"
        << " 10 ms";
  }
  ::unlink(path.c_str());
}

// A disk failure while multiple chunks are submitted behind a blocked write
// still advances completed for every queued chunk, so neither flush() nor
// destruction deadlocks in wait_written() and failed() is reported.
// REQ-OPS-44
TEST(LogBackgroundWriter,
     FlushAndDestructionReturnPromptlyWhenFdFailsWithChunksQueued) {
  for (const bool call_flush : {true, false}) {
    SCOPED_TRACE(call_flush ? "flush" : "destruction");
    const std::string path =
        TempPath(call_flush ? "fd_fail_flush" : "fd_fail_dtor");
    WriteGate gate;
    std::optional<LogWriter> writer;
    writer.emplace(path, "gated", GatedOptions(gate));
    const OpenAtExit open_at_exit{gate};
    writer->start({}, {});
    GatedDispatch(*writer, 1);
    writer->flush();
    const std::uint64_t base = gate.entries();

    gate.close();
    // Submit all four chunks and open a gap with a held EXIT behind the gate.
    for (std::uint64_t d = 2; d <= 70; ++d) GatedDispatch(*writer, d);
    GatedExit(*writer, 71);
    ASSERT_TRUE(writer->losing());
    ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 1; }));

    // Invalidate the writer's fd while chunk 0 is paused in before_write() and
    // chunks 1..3 are queued behind it; no new descriptors are opened until
    // writer is destroyed.
    ASSERT_TRUE(CloseLogDescriptor(path));
    gate.open();

    std::promise<bool> flushed;
    std::promise<void> destroyed;
    auto flushed_future = flushed.get_future();
    auto destroyed_future = destroyed.get_future();
    std::thread stopper{[&] {
      bool is_failed = false;
      if (call_flush) {
        writer->flush();
        is_failed = writer->failed();
      } else {
        is_failed = Eventually([&] { return writer->failed(); });
      }
      flushed.set_value(is_failed);
      writer.reset();
      destroyed.set_value();
    }};
    const bool flush_returned =
        flushed_future.wait_for(std::chrono::seconds{10}) ==
        std::future_status::ready;
    const bool dtor_returned =
        flush_returned && destroyed_future.wait_for(std::chrono::seconds{10}) ==
                              std::future_status::ready;
    if (dtor_returned) {
      stopper.join();
    } else {
      stopper.detach();
    }
    EXPECT_TRUE(flush_returned) << "flush() blocked after fd failure";
    EXPECT_TRUE(dtor_returned) << "~LogWriter() blocked after fd failure";
    if (flush_returned) {
      EXPECT_TRUE(flushed_future.get());
    }
    ::unlink(path.c_str());
  }
}

// While small records stream into an active chunk faster than write_interval
// (10 ms), the writer coalesces them and wakes at most once per 10 ms write
// interval rather than once per record.
// REQ-RT-21
TEST(LogBackgroundWriter, StreamingSmallRecordsWakeAtMostOncePerWriteInterval) {
  const auto path = TempPath("stream_wakeups");
  std::uint64_t index = 0;
  {
    LogWriter::Options options{};
    ASSERT_EQ(options.write_interval, std::chrono::milliseconds{10});
    options.sync_interval = std::chrono::hours{1};
    std::atomic<std::uint64_t> writes{0};
    options.before_write = [&] {
      writes.fetch_add(1, std::memory_order_relaxed);
    };
    LogWriter writer{path, "stream", options};
    writer.start({}, {});
    writer.flush();  // The writer thread creates the file.
    const auto log_one = [&] {
      Context context{};
      context.dispatch_index = ++index;
      writer.dispatch(context, MakePayload(16, static_cast<int>(index)));
    };

    // Prime last_write with one record that wakes the idle writer.
    log_one();
    ASSERT_TRUE(Eventually([&] { return writes.load() >= 1; }));

    const std::uint64_t base_writes = writes.load();
    const std::uint64_t base_wakeups = writer.wakeups();
    constexpr std::uint64_t kIntervals = 10;
    constexpr std::uint64_t kRecordsPerPoll = 10;
    std::uint64_t streamed = 0;
    ASSERT_TRUE(Eventually([&] {
      for (std::uint64_t i = 0; i < kRecordsPerPoll; ++i) {
        log_one();
        ++streamed;
      }
      return writes.load() >= base_writes + kIntervals;
    }));
    const std::uint64_t stream_writes = writes.load() - base_writes;
    const std::uint64_t stream_wakeups = writer.wakeups() - base_wakeups;

    EXPECT_GE(stream_writes, kIntervals);
    // At least one 10 ms interval held back multiple polls before writing.
    EXPECT_GT(streamed, kIntervals * kRecordsPerPoll);
    // One wakeup per 10 ms write interval (plus up to 2 for idle->streaming
    // entry or a delayed poll), never once per record.
    EXPECT_LE(stream_wakeups, stream_writes + 2)
        << "streamed=" << streamed << " writes=" << stream_writes;
    EXPECT_LT(stream_wakeups, streamed / 4)
        << "streamed=" << streamed << " writes=" << stream_writes;
    writer.flush();
    EXPECT_FALSE(writer.failed()) << writer.error();
  }
  LogReader reader{path};
  LogReader::Record record{};
  std::uint64_t read = 0;
  while (reader.next(record)) {
    ASSERT_EQ(record.header.dispatch_index, ++read);
  }
  EXPECT_EQ(read, index);
  ::unlink(path.c_str());
}

// A writer whose disk refused a write while a chunk held unwritten records
// accepts nothing more, so it must not keep waking to retry: it used to
// wake every write_interval for ever.
// REQ-RT-21
TEST(LogBackgroundWriter, FailedWriterGoesQuiet) {
  const auto path = TempPath("failed_wakeups");
  {
    LogWriter::Options options{};
    options.sync_interval = std::chrono::hours{1};
    LogWriter writer{path, "failed", options};
    writer.start({}, {});
    writer.flush();  // The writer thread creates the file.
    ASSERT_TRUE(CloseLogDescriptor(path));
    Context context{};
    context.dispatch_index = 1;
    writer.dispatch(context, MakePayload(16, 1));
    ASSERT_TRUE(Eventually([&] { return writer.failed(); }));
    const std::uint64_t start = writer.wakeups();
    std::this_thread::sleep_for(std::chrono::milliseconds{200});
    // None is due for an hour; retrying every write_interval gave about 20.
    EXPECT_LE(writer.wakeups() - start, 1u) << "over 200 ms after the failure";
  }
  ::unlink(path.c_str());
}

// Once a write_interval (10 ms) passes after a streaming burst with no further
// records, a healthy idle writer stops waking until its periodic sync (here an
// hour away): 0 wakeups over ~20 write intervals.
// REQ-RT-21
TEST(LogBackgroundWriter, HealthyIdleWriterProducesZeroWakeupsAfterBurst) {
  const auto path = TempPath("healthy_idle_wakeups");
  {
    LogWriter::Options options{};
    ASSERT_EQ(options.write_interval, std::chrono::milliseconds{10});
    options.sync_interval = std::chrono::hours{1};
    std::atomic<std::uint64_t> writes{0};
    std::atomic<std::uint64_t> stop_checks{0};
    options.before_write = [&] {
      writes.fetch_add(1, std::memory_order_relaxed);
    };
    options.before_stop_check = [&](const std::atomic<bool>&) {
      stop_checks.fetch_add(1, std::memory_order_relaxed);
    };
    LogWriter writer{path, "healthy_idle", options};
    writer.start({}, {});
    writer.flush();  // The writer thread creates the file.

    std::uint64_t index = 0;
    const std::vector<std::byte> payload(16, std::byte{0x5a});
    const auto log_one = [&] {
      Context context{};
      context.dispatch_index = ++index;
      writer.dispatch(context, payload);
    };

    // Prime last_write, then stream enough records to enter streaming mode.
    log_one();
    ASSERT_TRUE(Eventually([&] { return writes.load() >= 1; }));
    const std::uint64_t base_writes = writes.load();
    ASSERT_TRUE(Eventually([&] {
      for (int i = 0; i < 10; ++i) log_one();
      return writes.load() >= base_writes + 3;
    }));

    // Wait for every burst record to reach the file.
    const auto expected_bytes = static_cast<off_t>(
        sizeof(FileHeader) + index * (sizeof(RecordHeader) + 16));
    ASSERT_TRUE(Eventually([&] {
      struct stat status {};
      return ::stat(path.c_str(), &status) == 0 &&
             status.st_size >= expected_bytes;
    }));

    // Wait past the post-burst write_interval and its single empty timed wake
    // (on macOS QOS_CLASS_BACKGROUND, a 10 ms timed wait can coalesce >30 ms):
    // stop_checks goes quiet once the writer enters idle sleep (capped at 400
    // polls so a polling bug still reaches the assertion below quickly).
    std::uint64_t last_checks = stop_checks.load(std::memory_order_relaxed);
    int quiet_polls = 0;
    int settle_polls = 0;
    ASSERT_TRUE(Eventually([&] {
      const std::uint64_t current = stop_checks.load(std::memory_order_relaxed);
      if (current == last_checks) {
        ++quiet_polls;
      } else {
        last_checks = current;
        quiet_polls = 0;
      }
      return quiet_polls >= 150 || ++settle_polls >= 400;
    }));

    // Observe ~20 write intervals (~200 ms) of idle time: 0 wakeups allowed.
    const std::uint64_t idle_start = writer.wakeups();
    int idle_polls = 0;
    ASSERT_TRUE(Eventually([&] { return ++idle_polls >= 200; }));
    EXPECT_EQ(writer.wakeups() - idle_start, 0u)
        << "healthy idle writer woke after post-burst write_interval";
    EXPECT_FALSE(writer.failed()) << writer.error();
  }
  ::unlink(path.c_str());
}

// Once the log file is open (fd >= 0), a stalled disk write in before_write
// past two write intervals (> 20 ms) never blocks the producer's dispatch()
// calls, even after the chunk pool fills and opens a gap.
// REQ-OPS-31
TEST(LogBackgroundWriter, DispatchNeverBlocksWhenWriteStallsPastWriteInterval) {
  const auto path = TempPath("dispatch_nonblocking_write_stall");
  WriteGate gate;
  {
    LogWriter::Options options{4096, 4, /*background=*/true};
    ASSERT_EQ(options.write_interval, std::chrono::milliseconds{10});
    options.sync_interval = std::chrono::hours{1};
    options.before_write = gate.hook();
    LogWriter writer{path, "stall_nonblocking", options};
    const OpenAtExit open_at_exit{gate};
    writer.start({}, {});
    writer.flush();  // Writer thread opened the file (fd >= 0).
    struct stat status {};
    ASSERT_EQ(::stat(path.c_str(), &status), 0);

    const std::uint64_t base = gate.entries();
    gate.close();
    GatedDispatch(writer, 1);
    ASSERT_TRUE(Eventually([&] { return gate.entries() == base + 1; }));

    // Dispatch across > 20 ms (30 x 1 ms polls via Eventually) while the writer
    // stays blocked in before_write, overflowing the pool to open a gap too.
    std::promise<void> done;
    auto future = done.get_future();
    std::thread producer{[&] {
      std::uint64_t d = 2;
      int polls = 0;
      EXPECT_TRUE(Eventually([&] {
        for (int i = 0; i < 5; ++i) GatedDispatch(writer, d++);
        return ++polls >= 30;
      }));
      done.set_value();
    }};

    const bool finished =
        future.wait_for(std::chrono::seconds{10}) == std::future_status::ready;
    EXPECT_TRUE(finished) << "dispatch() blocked while before_write was held";
    EXPECT_EQ(gate.entries(), base + 1)
        << "writer left before_write while gate was closed";
    EXPECT_TRUE(writer.losing());
    EXPECT_EQ(writer.gaps(), 1u);
    EXPECT_FALSE(writer.failed()) << writer.error();

    gate.open();
    if (finished) {
      producer.join();
    } else {
      producer.detach();
    }
    writer.flush();
    EXPECT_FALSE(writer.failed()) << writer.error();
  }
  const ReadBack back = ReadAll(path, false);
  EXPECT_EQ(back.gaps, 1u);
  EXPECT_TRUE(back.damage.clean());
  ::unlink(path.c_str());
}

// The background writer thread configures itself via ConfigureBackgroundThread
// (SCHED_OTHER + nice >= kBackgroundNice on Linux, QOS_CLASS_UTILITY on macOS)
// so it runs below every loop thread, verifiable without realtime privilege.
// REQ-RT-10
TEST(LogBackgroundWriter, WriterThreadRunsAsBackground) {
  const auto path = TempPath("writer_background");
  std::atomic<bool> observed{false};
  std::atomic<bool> is_background{false};
  {
    LogWriter::Options options{};
    options.before_write = [&] {
      is_background.store(process::ThreadIsBackground(),
                          std::memory_order_relaxed);
      observed.store(true, std::memory_order_release);
    };
    LogWriter writer{path, "background_check", options};
    writer.start({}, {});
    Context context{};
    context.dispatch_index = 1;
    writer.dispatch(context, MakePayload(16, 1));
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }
  EXPECT_TRUE(observed.load(std::memory_order_acquire));
  EXPECT_TRUE(is_background.load(std::memory_order_relaxed));
  ::unlink(path.c_str());
}

// REQ-OPS-14, REQ-OPS-15: FindLogFiles and LogSegmentFiles must not throw
// std::filesystem::filesystem_error on unreadable directories, symlink loops,
// or concurrent directory mutations, while preserving sorted discovery on valid
// session directories.
TEST(LogCapture, FindLogFilesAndLogSegmentFilesHandleDirectoryErrors) {
  namespace fs = std::filesystem;
  const fs::path root = fs::path{::testing::TempDir()} /
                        ("talos_find_logs_err_" + std::to_string(::getpid()));
  std::error_code ec;
  fs::remove_all(root, ec);
  ASSERT_TRUE(fs::create_directories(root, ec)) << ec.message();

  // Normal directory with base log, rotated segments, and non-.tlog files.
  const fs::path valid_dir = root / "valid";
  ASSERT_TRUE(fs::create_directories(valid_dir, ec));
  const fs::path base_log = valid_dir / "alpha.tlog";
  const fs::path seg0 = valid_dir / "alpha.segment0.tlog";
  const fs::path seg2 = valid_dir / "alpha.segment2.tlog";
  const fs::path other_log = valid_dir / "beta.tlog";
  const fs::path ignored = valid_dir / "notes.txt";
  for (const fs::path& p : {base_log, seg0, seg2, other_log, ignored}) {
    std::ofstream file{p};
    ASSERT_TRUE(file.good());
  }
  EXPECT_EQ(FindLogFiles(valid_dir.string()),
            (std::vector<std::string>{seg0.string(), seg2.string(),
                                      base_log.string(), other_log.string()}));
  EXPECT_EQ(LogSegmentFiles(base_log.string()),
            (std::vector<std::string>{seg0.string(), seg2.string(),
                                      base_log.string()}));

  // Missing path and symlink loop never throw.
  const fs::path missing = root / "does_not_exist";
  EXPECT_NO_THROW({
    EXPECT_EQ(FindLogFiles(missing.string()),
              (std::vector<std::string>{missing.string()}));
  });
  EXPECT_NO_THROW({
    EXPECT_TRUE(LogSegmentFiles((missing / "alpha.tlog").string()).empty());
  });

  const fs::path loop_a = root / "loop_a";
  const fs::path loop_b = root / "loop_b";
  fs::create_symlink(loop_b, loop_a, ec);
  fs::create_symlink(loop_a, loop_b, ec);
  ASSERT_FALSE(ec) << ec.message();
  EXPECT_NO_THROW({
    EXPECT_EQ(FindLogFiles(loop_a.string()),
              (std::vector<std::string>{loop_a.string()}));
  });
  EXPECT_NO_THROW({
    EXPECT_TRUE(LogSegmentFiles((loop_a / "alpha.tlog").string()).empty());
  });

  // Unreadable directory (mode 0000): is_directory is true, directory_iterator
  // fails with EACCES; neither helper may throw.
  if (::geteuid() != 0) {
    const fs::path unreadable = root / "unreadable";
    ASSERT_TRUE(fs::create_directories(unreadable, ec));
    {
      std::ofstream file{unreadable / "alpha.tlog"};
      ASSERT_TRUE(file.good());
    }
    ASSERT_EQ(::chmod(unreadable.c_str(), 0), 0);
    EXPECT_NO_THROW(
        { EXPECT_TRUE(FindLogFiles(unreadable.string()).empty()); });
    EXPECT_NO_THROW({
      EXPECT_TRUE(
          LogSegmentFiles((unreadable / "alpha.tlog").string()).empty());
    });
    ::chmod(unreadable.c_str(), 0700);
  }

  // Concurrent entry churn during iteration must not throw.
  const fs::path churn_dir = root / "churn";
  ASSERT_TRUE(fs::create_directories(churn_dir, ec));
  std::atomic<bool> stop{false};
  std::thread mutator{[&] {
    for (int i = 0; !stop.load(std::memory_order_relaxed); ++i) {
      const fs::path p =
          churn_dir / ("alpha.segment" + std::to_string(i % 16) + ".tlog");
      if ((i / 16) % 2 == 0) {
        std::ofstream file{p};
        (void)file.good();
      } else {
        std::error_code rm_ec;
        fs::remove(p, rm_ec);
      }
    }
  }};
  for (int i = 0; i < 50; ++i) {
    EXPECT_NO_THROW({ (void)FindLogFiles(churn_dir.string()); });
    EXPECT_NO_THROW(
        { (void)LogSegmentFiles((churn_dir / "alpha.tlog").string()); });
  }
  stop.store(true, std::memory_order_relaxed);
  mutator.join();

  fs::remove_all(root, ec);
}

#if defined(__linux__)
// This process's thread ids (/proc/self/task).
std::set<pid_t> ThreadIds() {
  std::set<pid_t> ids;
  for (const auto& entry :
       std::filesystem::directory_iterator{"/proc/self/task"}) {
    ids.insert(static_cast<pid_t>(std::stol(entry.path().filename().string())));
  }
  return ids;
}

// The one thread in ThreadIds() that is not in `before`, or 0.
pid_t NewThread(const std::set<pid_t>& before) {
  pid_t found = 0;
  for (const pid_t id : ThreadIds()) {
    if (before.contains(id)) continue;
    if (found != 0) return 0;
    found = id;
  }
  return found;
}

// How many times thread `id` has blocked and been woken.
long VoluntarySwitches(pid_t id) {
  std::ifstream status{"/proc/self/task/" + std::to_string(id) + "/status"};
  std::string line;
  constexpr std::string_view kKey = "voluntary_ctxt_switches:";
  while (std::getline(status, line)) {
    if (line.starts_with(kKey)) return std::stol(line.substr(kKey.size()));
  }
  return -1;
}

// Even when a creator thread holds a plain SCHED_FIFO policy (without
// SCHED_RESET_ON_FORK), Linux threads inherit their creator's policy: the log
// writer must still put itself back under SCHED_OTHER rather than compete with
// control loops for the CPU.
TEST(LogBackgroundWriter, WriterLeavesTheRealtimePolicyOfItsCreator) {
  const auto path = TempPath("writer_policy");
  bool realtime = false;
  int writer_policy = -1;
  std::thread creator{[&] {
    sched_param fifo{};
    fifo.sched_priority = ::sched_get_priority_min(SCHED_FIFO);
    realtime =
        ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &fifo) == 0;
    if (!realtime) return;
    const std::set<pid_t> before = ThreadIds();
    LogWriter writer{path, "policy", LogWriter::Options{}};
    writer.start({}, {});
    // The writer sets its policy before it handles anything, so once this
    // record has been written the policy is final.
    Context context{};
    context.dispatch_index = 1;
    writer.dispatch(context, MakePayload(16, 1));
    writer.flush();
    const pid_t id = NewThread(before);
    if (id != 0) writer_policy = ::sched_getscheduler(id);
  }};
  creator.join();
  ::unlink(path.c_str());
  if (!realtime) GTEST_SKIP() << "SCHED_FIFO refused; cannot test inheritance";
  EXPECT_EQ(writer_policy, SCHED_OTHER);
}

// An idle writer sleeps until a hand-off or its periodic sync instead of
// waking every millisecond.
TEST(LogBackgroundWriter, IdleWriterDoesNotPoll) {
  const auto path = TempPath("idle_wakeups");
  {
    const std::set<pid_t> before = ThreadIds();
    LogWriter writer{path, "idle", LogWriter::Options{}};
    writer.start({}, {});
    // Returns once the writer thread has created the file and synced it;
    // all it can do next is go idle.
    writer.flush();
    const pid_t id = NewThread(before);
    ASSERT_NE(id, 0);
    const long start = VoluntarySwitches(id);
    // The window is the subject: how often an idle writer wakes in 500 ms.
    std::this_thread::sleep_for(std::chrono::milliseconds{500});
    const long wakeups = VoluntarySwitches(id) - start;
    ASSERT_GE(start, 0);
    // About 1, for the periodic sync; polling every 1 ms gave about 500.
    EXPECT_LT(wakeups, 50);
  }
  ::unlink(path.c_str());
}
#endif

}  // namespace
}  // namespace talos::event::log

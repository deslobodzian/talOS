#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"

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
  LogWriter writer{path, "talos_log_test", options, kFixtureSession};
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
  EXPECT_EQ(reader.format_version(), 7u);
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

// Legacy and future log versions must be rejected before reading records.
TEST(LogFormatErrors, RejectsUnsupportedVersions) {
  const std::string path = TempPath("unsupported_version");
  for (const std::uint32_t version : {4u, 6u, 8u}) {
    SCOPED_TRACE(version);
    WriteFixture(path, LogWriter::kDefaultBufferBytes);
    {
      std::fstream file{path, std::ios::in | std::ios::out | std::ios::binary};
      ASSERT_TRUE(file.is_open());
      file.seekp(static_cast<std::streamoff>(offsetof(FileHeader, version)));
      file.write(reinterpret_cast<const char*>(&version), sizeof(version));
    }
    if (version == 4) {
      // An empty v4 file is shorter than the current FileHeader.
      ASSERT_EQ(0, ::truncate(path.c_str(), 112));
    }
    EXPECT_THROW((LogReader{path}), std::runtime_error);
  }
  ::unlink(path.c_str());
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

TEST(LogBackgroundWriter, BlockedDiskStopsCaptureWithoutBlockingProducer) {
  const auto path = TempPath("blocked_fifo");
  ASSERT_EQ(::mkfifo(path.c_str(), 0600), 0);
  std::atomic<bool> allow_read{false};
  std::vector<std::byte> bytes;
  std::thread consumer{[&] {
    const int fd = ::open(path.c_str(), O_RDONLY);
    std::byte header[sizeof(FileHeader)];
    std::size_t seen = 0;
    while (seen < sizeof(header)) {
      const auto n = ::read(fd, header + seen, sizeof(header) - seen);
      if (n <= 0) break;
      seen += static_cast<std::size_t>(n);
    }
    bytes.insert(bytes.end(), header, header + seen);
    // The writer cannot complete a 1 MiB chunk until this reader resumes.
    while (!allow_read.load()) std::this_thread::yield();
    std::byte buffer[4096];
    for (;;) {
      const auto n = ::read(fd, buffer, sizeof(buffer));
      if (n <= 0) break;
      bytes.insert(bytes.end(), buffer, buffer + n);
    }
    ::close(fd);
  }};
  constexpr std::size_t payload_size = 1u << 20;
  {
    LogWriter writer{
        path, "blocked",
        LogWriter::Options{payload_size + sizeof(RecordHeader), 1, true}};
    writer.start({}, {});
    const auto payload = MakePayload(payload_size, 7);
    Context context{};
    context.dispatch_index = 1;
    writer.dispatch(context, payload);
    context.dispatch_index = 2;
    writer.dispatch(context, payload);
    EXPECT_TRUE(writer.failed());
    EXPECT_TRUE(writer.capture_stopped());
    writer.finish(context);  // Failure must not leave a misleading clean EXIT.
    allow_read.store(true);
    writer.flush();
    writer.flush();
  }
  consumer.join();
  ASSERT_EQ(bytes.size(),
            sizeof(FileHeader) + sizeof(RecordHeader) + payload_size);
  RecordHeader record{};
  std::memcpy(&record, bytes.data() + sizeof(FileHeader), sizeof(record));
  EXPECT_EQ(record.dispatch_index, 1);
  EXPECT_EQ(record.payload_bytes, payload_size);
  EXPECT_EQ(record.stored_bytes, payload_size);
  ::unlink(path.c_str());
}

// The loop thread never waits on its log file: constructing, starting and
// recording into a background writer whose file cannot be opened yet (a FIFO
// with no reader blocks open() the way a create can on a busy disk) all
// return, and once the file opens it holds the header, then every record in
// order.
TEST(LogBackgroundWriter, ProducerNeverWaitsForTheLogFile) {
  const auto path = TempPath("unopened_fifo");
  ASSERT_EQ(::mkfifo(path.c_str(), 0600), 0);
  std::optional<LogWriter> writer;
  std::atomic<bool> recorded{false};
  std::thread producer{[&] {
    writer.emplace(path, "unopened", LogWriter::Options{4096, 2, true}, 9);
    writer->start(MakeManifest(), MonotonicTime::from_nanos(42));
    for (std::uint64_t i = 1; i <= 3; ++i) {
      Context context{};
      context.dispatch_index = i;
      writer->dispatch(context, MakePayload(16, static_cast<int>(i)));
    }
    recorded.store(true);
  }};
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds{2};
  while (!recorded.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
  const bool waited = !recorded.load();
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

TEST(LogCapture, OversizedRecordStopsCaptureAndPreservesPrefix) {
  const auto path = TempPath("oversized");
  LogWriter writer{path, "oversized", 512};
  writer.start(MakeManifest(), {});
  Context context{};
  context.dispatch_index = 1;
  writer.dispatch(context, MakePayload(20, 1));
  writer.dispatch(context, MakePayload(1024, 2));
  EXPECT_TRUE(writer.failed());
  writer.dispatch(context, {});
  writer.finish(context);
  writer.flush();
  LogReader reader{path};
  LogReader::Record record{};
  ASSERT_TRUE(reader.next(record));
  EXPECT_EQ(record.payload.size(), 20);
  EXPECT_FALSE(reader.next(record));
  ::unlink(path.c_str());
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
// Here write_interval is an hour and the writer has a record waiting for
// it; every chunk handed over must still reach the file, one at a time,
// for five times the pool's chunks.
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
    LogWriter::Options options{kChunkBytes, 2, /*background=*/true};
    options.write_interval = std::chrono::hours{1};
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
    // Found idle, written at once; the next record waits for the hour.
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
          << " handed over and not written in 10 s (pool of 2)";
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

// A node calls process::ConfigureRealtime (SCHED_FIFO) on its loop thread
// before the loop starts the log writer, and Linux threads inherit their
// creator's policy: the writer must put itself back under SCHED_OTHER rather
// than compete with control loops for the CPU.
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
    std::this_thread::sleep_for(std::chrono::milliseconds{50});
    const pid_t id = NewThread(before);
    ASSERT_NE(id, 0);
    const long start = VoluntarySwitches(id);
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

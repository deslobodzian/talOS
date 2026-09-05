#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
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

  Registration fetcher;
  fetcher.id = 1;
  fetcher.kind = SourceKind::FETCHER;
  // Exactly MAX_SOURCE_NAME characters: must survive round trip untruncated.
  fetcher.name = std::string(MAX_SOURCE_NAME, 'q');
  fetcher.message_bytes = 128;
  fetcher.alignment = 4;
  fetcher.period_ns = 0;
  fetcher.offset_ns = 0;
  manifest.push_back(fetcher);

  Registration sender;
  sender.id = 2;
  sender.kind = SourceKind::SENDER;
  sender.name = "cmd_topic";
  sender.message_bytes = 32;
  sender.alignment = 4;
  sender.period_ns = 0;
  sender.offset_ns = 0;
  manifest.push_back(sender);

  return manifest;
}

std::vector<std::byte> MakePayload(std::size_t size, int seed) {
  std::vector<std::byte> payload(size);
  for (std::size_t i = 0; i < size; ++i) {
    payload[i] = static_cast<std::byte>((seed + static_cast<int>(i)) % 256);
  }
  return payload;
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
};

void ExpectRecordEquals(const ExpectedRecord& expected,
                        const LogReader::Record& actual) {
  EXPECT_EQ(actual.header.kind, expected.kind);
  EXPECT_EQ(actual.header.source_id, expected.source_id);
  EXPECT_EQ(actual.header.dispatch_index, expected.dispatch_index);
  EXPECT_EQ(actual.header.event_time_ns, expected.event_time_ns);
  EXPECT_EQ(actual.header.now_ns, expected.now_ns);
  EXPECT_EQ(actual.header.sequence, expected.sequence);
  EXPECT_EQ(actual.header.aux, expected.aux);
  ASSERT_EQ(actual.payload.size(), expected.payload.size());
  EXPECT_EQ(0, std::memcmp(actual.payload.data(), expected.payload.data(),
                           expected.payload.size()));
}

// Writes the round-trip fixture used by several tests: a manifest, ~50
// TIMER/MESSAGE/FETCH/SEND records with varying payload sizes (including
// zero-length ones), and a final EXIT. Returns the expected records in
// on-disk order so callers can verify the reader against them.
std::vector<ExpectedRecord> WriteFixtureWithOptions(
    const std::string& path, LogWriter::Options options) {
  LogWriter writer{path, "talos_log_test", options};
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
    context.event_time = MonotonicTime::from_nanos(i * 1000);
    context.now = MonotonicTime::from_nanos(i * 1000 + (i % 5) * 10);
    context.sequence = static_cast<std::uint64_t>(i);
    context.dropped = static_cast<std::uint64_t>(i % 3);

    writer.dispatch(context, payload);
    expected.push_back({static_cast<std::uint16_t>(context.kind), source_id,
                        context.dispatch_index, context.event_time.nanos(),
                        context.now.nanos(), context.sequence,
                        static_cast<std::uint32_t>(context.dropped), payload});

    if (i % 5 == 0) {
      // A fetch that found nothing: payload must be dropped even though we
      // hand one in, exercising LogWriter's own empty-on-EMPTY guarantee.
      const std::vector<std::byte> ignored = MakePayload(5, i);
      writer.fetch(context, source_id, FetchOutcome::EMPTY, ignored,
                   /*sequence=*/1000 + i, /*dropped=*/0);
      expected.push_back({static_cast<std::uint16_t>(EventKind::FETCH),
                          source_id,
                          context.dispatch_index,
                          context.event_time.nanos(),
                          context.now.nanos(),
                          static_cast<std::uint64_t>(1000 + i),
                          static_cast<std::uint32_t>(FetchOutcome::EMPTY),
                          {}});
    } else if (i % 5 == 1) {
      const std::size_t fetch_payload_size =
          static_cast<std::size_t>((i * 3) % 19);
      const std::vector<std::byte> fetch_payload =
          MakePayload(fetch_payload_size, i + 1);
      writer.fetch(context, source_id, FetchOutcome::VALUE, fetch_payload,
                   /*sequence=*/2000 + i, /*dropped=*/0);
      expected.push_back(
          {static_cast<std::uint16_t>(EventKind::FETCH), source_id,
           context.dispatch_index, context.event_time.nanos(),
           context.now.nanos(), static_cast<std::uint64_t>(2000 + i),
           static_cast<std::uint32_t>(FetchOutcome::VALUE), fetch_payload});
    } else if (i % 5 == 2) {
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
                          send_payload});
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

  ::unlink(path.c_str());
}

TEST(LogRoundTrip, SmallBufferForcesFlushAndGrowth) {
  const std::string path = TempPath("smallbuffer");
  // 512 bytes is smaller than the manifest block plus several records, and
  // one fetch payload below is chosen to exceed it outright.
  LogWriter writer{path, "small_buffer_writer", 512};
  const Manifest manifest = MakeManifest();
  writer.start(manifest, MonotonicTime::from_nanos(7));

  std::vector<ExpectedRecord> expected;
  for (int i = 0; i < 10; ++i) {
    Context context;
    context.kind = EventKind::TIMER;
    context.source_id = 0;
    context.dispatch_index = static_cast<std::uint64_t>(i);
    context.event_time = MonotonicTime::from_nanos(i * 100);
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

TEST(LogCorruption, PayloadCrcMismatchThrows) {
  const std::string path = TempPath("payload_corrupt");
  const std::vector<ExpectedRecord> expected =
      WriteFixture(path, LogWriter::kDefaultBufferBytes);

  // The manifest block ends where the first record begins. Walk forward
  // (skipping the zero-length payloads at the start of the fixture) to find
  // the first record that actually has a payload to corrupt.
  ASSERT_FALSE(expected.empty());
  std::size_t offset = sizeof(FileHeader) + 3 * sizeof(ManifestEntry);
  std::size_t target_index = 0;
  for (; target_index < expected.size(); ++target_index) {
    if (!expected[target_index].payload.empty()) {
      break;
    }
    offset += sizeof(RecordHeader) + expected[target_index].payload.size();
  }
  ASSERT_LT(target_index, expected.size())
      << "fixture has no record with a payload";

  // Corrupt the first byte of that record's payload.
  FlipByte(path, offset + sizeof(RecordHeader));

  LogReader reader{path};
  LogReader::Record record;
  for (std::size_t i = 0; i < target_index; ++i) {
    ASSERT_TRUE(reader.next(record));
  }
  EXPECT_THROW(reader.next(record), std::runtime_error);

  ::unlink(path.c_str());
}

TEST(LogCorruption, ManifestCrcMismatchThrowsOnOpen) {
  const std::string path = TempPath("manifest_corrupt");
  WriteFixture(path, LogWriter::kDefaultBufferBytes);

  // Somewhere inside the manifest block, which starts right after the
  // header.
  FlipByte(path, sizeof(FileHeader) + 10);

  EXPECT_THROW(LogReader{path}, std::runtime_error);

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

TEST(LogFormatErrors, BadMagicThrows) {
  const std::string path = TempPath("bad_magic");
  WriteFixture(path, LogWriter::kDefaultBufferBytes);

  FlipByte(path, 0);

  EXPECT_THROW(LogReader{path}, std::runtime_error);

  ::unlink(path.c_str());
}

TEST(LogFormatErrors, BadVersionThrows) {
  const std::string path = TempPath("bad_version");
  WriteFixture(path, LogWriter::kDefaultBufferBytes);

  FlipByte(path, offsetof(FileHeader, version));

  EXPECT_THROW(LogReader{path}, std::runtime_error);

  ::unlink(path.c_str());
}

// Writes must be identical whether they went to disk on the loop thread or
// were handed to the writer thread, and small chunks force many handoffs.
TEST(LogBackgroundWriter, RoundTripsThroughTheWriterThread) {
  const std::string path = TempPath("background_round_trip");
  const std::vector<ExpectedRecord> expected = WriteFixtureWithOptions(
      path, LogWriter::Options{256, 3, /*background=*/true});

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

// A pool far too small for the write rate must stall the producer rather than
// drop records: an incomplete log cannot be replayed.
TEST(LogBackgroundWriter, StallingNeverLosesRecords) {
  const std::string path = TempPath("background_stall");
  const std::vector<ExpectedRecord> expected = WriteFixtureWithOptions(
      path, LogWriter::Options{128, 2, /*background=*/true});

  LogReader reader{path};
  LogReader::Record record;
  std::size_t seen = 0;

  for (const ExpectedRecord& want : expected) {
    ASSERT_TRUE(reader.next(record)) << "missing record " << seen;
    ExpectRecordEquals(want, record);
    ++seen;
  }

  EXPECT_EQ(seen, expected.size());

  std::remove(path.c_str());
}

// flush() has to wait for the writer thread, not just hand off to it.
TEST(LogBackgroundWriter, FlushWaitsForTheWriterThread) {
  const std::string path = TempPath("background_flush");

  LogWriter writer{path, "talos_log_test",
                   LogWriter::Options{4096, 4, /*background=*/true}};
  writer.start(MakeManifest(), MonotonicTime::from_nanos(7));

  constexpr int records = 40;
  for (int i = 0; i < records; ++i) {
    Context context;
    context.kind = EventKind::TIMER;
    context.source_id = 0;
    context.dispatch_index = static_cast<std::uint64_t>(i + 1);
    context.event_time = MonotonicTime::from_nanos(i * 1000);
    context.now = MonotonicTime::from_nanos(i * 1000);
    context.sequence = static_cast<std::uint64_t>(i);
    writer.dispatch(context, MakePayload(16, i));
  }

  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();

  // Everything must be readable while the writer is still open.
  LogReader reader{path};
  LogReader::Record record;
  int seen = 0;
  while (reader.next(record)) {
    EXPECT_EQ(record.header.dispatch_index,
              static_cast<std::uint64_t>(seen + 1));
    ++seen;
  }

  EXPECT_EQ(seen, records);
  EXPECT_FALSE(reader.truncated());

  std::remove(path.c_str());
}

}  // namespace
}  // namespace talos::event::log

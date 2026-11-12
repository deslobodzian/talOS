#include "talOS/launcher/log_merge.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "talOS/events/context.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/launcher/log_inputs.h"

namespace talos::launcher {
namespace {

constexpr std::uint64_t kSession = 77;

// One record to write: its ordering fields and an id carried as its payload.
struct Row {
  std::int64_t event_ns;
  std::int64_t now_ns;
  std::uint64_t dispatch_index;
  std::uint64_t id;
};

class LogMergeTest : public testing::Test {
 protected:
  void SetUp() override {
    directory_ = testing::TempDir() + "/log_merge_" + std::to_string(getpid());
    std::filesystem::remove_all(directory_);
    std::filesystem::create_directories(directory_);
  }
  void TearDown() override { std::filesystem::remove_all(directory_); }

  // Writes `rows` to `file` as process `process`'s log; each payload is
  // `payload_bytes` long and starts with the row's id.
  void Write(const std::string& file, const std::string& process,
             const std::vector<Row>& rows, std::size_t payload_bytes = 8,
             std::uint64_t session = kSession) {
    event::Registration source;
    source.id = 1;
    source.kind = event::SourceKind::TIMER;
    source.name = "/" + process + "/tick";
    source.message_bytes = static_cast<std::uint32_t>(payload_bytes);
    event::log::LogWriter writer{
        file, process, event::log::LogWriterOptions{1u << 16, 4, false},
        session};
    writer.start({source}, event::MonotonicTime::from_nanos(0));
    std::vector<std::byte> payload(payload_bytes, std::byte{0xAB});
    for (const Row& row : rows) {
      std::memcpy(payload.data(), &row.id, sizeof(row.id));
      event::Context ctx{};
      ctx.kind = event::EventKind::TIMER;
      ctx.source_id = 1;
      ctx.dispatch_index = row.dispatch_index;
      ctx.event_time = event::MonotonicTime::from_nanos(row.event_ns);
      ctx.now = event::MonotonicTime::from_nanos(row.now_ns);
      writer.dispatch(ctx, payload);
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }

  std::string Path(const std::string& name) const {
    return directory_ + "/" + name;
  }

  std::string directory_;
};

std::uint64_t IdOf(const MergedRecord& r) {
  std::uint64_t id = 0;
  std::memcpy(&id, r.payload.data(), sizeof(id));
  return id;
}

// Guards the merged order log_dump and tlog_to_wpilog print and convert in:
// by event time, then now, then dispatch index, then input order (a rotated
// member's older segment first, as LogInputFiles lists it), each record
// named by its own log's process and source, and a cut log's intact prefix
// merged. Swapping now and dispatch index, dropping either, or breaking a
// full tie by anything but input order reorders the ids.
// REQ-OPS-15
TEST_F(LogMergeTest, OrdersByEventNowDispatchThenInputOrder) {
  // Member a, rotated: its older segment, then the log being written.
  Write(event::log::LogSegmentPath(Path("a.tlog"), 0), "a",
        {{1000, 1000, 1, 1}, {3000, 3000, 1, 2}});
  Write(Path("a.tlog"), "a", {{5000, 5000, 3, 3}, {7000, 7000, 4, 4}});
  Write(Path("b.tlog"), "b",
        {{2000, 2000, 1, 11},
         {3000, 2500, 2, 12},    // a's event time, an earlier now
         {5000, 5000, 2, 13},    // a's event time and now, a lower dispatch
         {7000, 7000, 4, 14}});  // a's whole key: input order decides
  // Member c, cut inside its last record by a power loss.
  Write(Path("c.tlog"), "c",
        {{4000, 4000, 1, 21}, {6000, 6000, 2, 22}, {8000, 8000, 3, 23}});
  std::filesystem::resize_file(Path("c.tlog"),
                               std::filesystem::file_size(Path("c.tlog")) - 10);

  std::vector<std::pair<std::string, std::uint64_t>> merged;
  std::string error;
  ASSERT_TRUE(MergeLogs(
      LogInputFiles({directory_}),
      [&](const MergedRecord& r) {
        EXPECT_EQ(r.source_name(), "/" + r.process_name() + "/tick");
        merged.emplace_back(r.process_name(), IdOf(r));
        return true;
      },
      error))
      << error;
  const std::vector<std::pair<std::string, std::uint64_t>> expected{
      {"a", 1},  {"b", 11}, {"b", 12}, {"a", 2}, {"c", 21},
      {"b", 13}, {"a", 3},  {"c", 22}, {"a", 4}, {"b", 14}};
  EXPECT_EQ(merged, expected);
}

// Guards the bound on a session reader's memory: the merge reads each log
// through a window, not whole, so records reach the visitor while most of a
// large log is still unread. Bytes cut from the large log after the first
// visit are never visited; a merge that had read it whole would visit all.
// REQ-OPS-15
TEST_F(LogMergeTest, VisitsBeforeReadingALargeLogWhole) {
  constexpr std::size_t kPayload = 1024;
  constexpr std::uint64_t kRecords = 4096;  // 4.25 MiB, 68 windows
  Write(Path("a.tlog"), "a", {{0, 0, 1, 1}});
  std::vector<Row> rows;
  for (std::uint64_t i = 1; i <= kRecords; ++i) {
    rows.push_back(
        {static_cast<std::int64_t>(i), static_cast<std::int64_t>(i), i, i});
  }
  Write(Path("b.tlog"), "b", rows, kPayload);
  const auto cut = std::filesystem::file_size(Path("b.tlog")) / 4;

  std::uint64_t visited_b = 0;
  std::string error;
  ASSERT_TRUE(MergeLogs(
      {Path("a.tlog"), Path("b.tlog")},
      [&](const MergedRecord& r) {
        if (r.process_name() == "a") {
          std::filesystem::resize_file(Path("b.tlog"), cut);
        } else {
          ++visited_b;
        }
        return true;
      },
      error))
      << error;
  const std::uint64_t record_bytes =
      sizeof(event::log::RecordHeader) + kPayload;
  EXPECT_GT(visited_b, 0u);
  EXPECT_LE(visited_b, cut / record_bytes)
      << "of " << kRecords << " records, cut to " << cut << " bytes";
}

// Guards that logs of different sessions are not merged into one stream.
// REQ-OPS-15
TEST_F(LogMergeTest, RefusesLogsOfAnotherSession) {
  Write(Path("a.tlog"), "a", {{1000, 1000, 1, 1}});
  Write(Path("b.tlog"), "b", {{2000, 2000, 1, 2}}, 8, kSession + 1);
  bool visited = false;
  std::string error;
  EXPECT_FALSE(MergeLogs(
      {Path("a.tlog"), Path("b.tlog")},
      [&](const MergedRecord&) {
        visited = true;
        return true;
      },
      error));
  EXPECT_FALSE(visited);
  EXPECT_NE(error.find(Path("b.tlog")), std::string::npos) << error;
}

}  // namespace
}  // namespace talos::launcher

// What the launcher says when a member's registry row first shows its event
// log capped: a log that rotates keeps recording and deletes its oldest
// records; only one too small to rotate stops.

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/context.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/manifest.h"
#include "talOS/launcher/launcher.h"

namespace talos::launcher {
namespace {

using event::log::LogWriter;

constexpr std::uint32_t kMessageBytes = 16;

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_capped_log_" +
         std::to_string(::getpid()) + "_" + std::string{name} + ".tlog";
}

event::Manifest OneSource() {
  event::Registration source;
  source.id = 0;
  source.kind = event::SourceKind::WATCHER;
  source.name = "sample";
  source.message_bytes = kMessageBytes;
  source.alignment = 4;
  return {source};
}

void RemoveLog(const std::string& path) {
  for (const auto& file : event::log::LogSegmentFiles(path))
    std::filesystem::remove(file);
  std::filesystem::remove(event::log::CappedMarkerPath(path));
}

// The bytes a log of OneSource() takes before its first record.
std::uint64_t PrefixBytes() {
  const std::string path = TempPath("prefix");
  {
    LogWriter writer{path, "prefix",
                     LogWriter::Options{256, 4, /*background=*/false}};
    writer.start(OneSource(), {});
    writer.flush();
  }
  const std::uint64_t bytes = std::filesystem::file_size(path);
  std::filesystem::remove(path);
  return bytes;
}

// REQ-OPS-32
// Each row writes a log under a byte share until its writer reports it
// capped -- the moment the member's registry row gains kFlagLogCapped -- and
// asks whether it stopped. Shares of four segments, and of exactly two (the
// fewest that rotate, which keeps just two files), rotate: they must read as
// recording on. A share below two segments, and one too small for the log's
// header, cannot rotate: those stopped. Each row also holds the answer to
// the writer's own capture_stopped().
TEST(CappedLog, RotatingLogRecordsOnAndOneThatCannotRotateStopped) {
  const std::uint64_t record = sizeof(event::log::RecordHeader) + kMessageBytes;
  const std::uint64_t segment = PrefixBytes() + record;
  struct Row {
    const char* name;
    std::uint64_t max_bytes;
    bool stopped;
  };
  const Row rows[] = {
      {"rotating", 4 * segment, false},
      {"two_segments", 2 * segment, false},
      {"one_segment", segment + record / 2, true},
      {"no_room_for_header", 1, true},
  };
  std::vector<std::byte> payload(kMessageBytes, std::byte{1});
  for (const Row& row : rows) {
    SCOPED_TRACE(row.name);
    const std::string path = TempPath(row.name);
    RemoveLog(path);
    // Synchronous, so each rotation is on disk when dispatch returns.
    LogWriter writer{
        path, "capped",
        LogWriter::Options{256, 4, /*background=*/false, row.max_bytes}};
    writer.start(OneSource(), {});
    for (std::uint64_t i = 1; !writer.capped() && i <= 1000; ++i) {
      event::Context context{};
      context.dispatch_index = i;
      writer.dispatch(context, payload);
    }
    ASSERT_TRUE(writer.capped()) << "max_bytes " << row.max_bytes;
    EXPECT_EQ(CappedLogStopped(path), row.stopped)
        << "max_bytes " << row.max_bytes << ", files "
        << event::log::LogSegmentFiles(path).size();
    EXPECT_EQ(writer.capture_stopped(), row.stopped);
    writer.flush();
    RemoveLog(path);
  }
}

}  // namespace
}  // namespace talos::launcher

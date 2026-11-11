#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"

namespace talos::event::log {
namespace {

std::vector<std::byte> Payload(std::uint64_t index) {
  std::vector<std::byte> payload(16);
  for (std::size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<std::byte>((index + i) % 251 + 1);
  }
  return payload;
}

// The child: logs one record every 5 ms, like a 200 Hz node, until killed.
// The periodic fsync is an hour away and a chunk holds about 3000 of these
// records (15 s of them), so neither a sync nor a full chunk can be what
// puts a record in the file within the parent's 10 s wait.
[[noreturn]] void LogUntilKilled(const std::string& path) {
  LogWriter::Options options{};
  options.sync_interval = std::chrono::hours{1};
  LogWriter writer{path, "killed", options};
  writer.start({}, {});
  for (std::uint64_t i = 1;; ++i) {
    Context context{};
    context.dispatch_index = i;
    writer.dispatch(context, Payload(i));
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
}

// How many intact records `path` holds now, checking that they are records
// 1, 2, ... in order with their payloads; 0 while the file or its header is
// not there yet.
std::uint64_t IntactPrefix(const std::string& path, bool& in_order) {
  in_order = true;
  try {
    LogReader reader{path};
    LogReader::Record record{};
    std::uint64_t count = 0;
    while (reader.next(record)) {
      ++count;
      const auto want = Payload(count);
      if (record.header.dispatch_index != count ||
          !std::equal(record.payload.begin(), record.payload.end(),
                      want.begin(), want.end())) {
        in_order = false;
      }
    }
    if (reader.damage().spans != 0) in_order = false;
    return count;
  } catch (const std::exception&) {
    return 0;
  }
}

// A node killed by a signal (the launcher's stall kill, a segfault, an abort)
// runs no destructor and no flush: its log keeps only what the writer had
// already written to the kernel. The writer used to write a chunk that was
// not full only at its periodic sync, once a second, so a SIGKILLed
// hardware_node lost the last 826 ms of its log on the bench (2026-11-11).
// Here the parent waits, polling the file, until it holds record kRecords of
// a child logging at 200 Hz, with no sync and no full chunk possible, then
// SIGKILLs the child: the file must still hold records 1 to at least
// kRecords, whole and in order (a record cut by the kill is not read as
// one). Before the fix the wait hits its 10 s hang deadline. This proves a
// committed record reaches the kernel without waiting for a sync; how soon
// (write_interval, 10 ms) is a timing figure this test does not assert.
// REQ-OPS-31
TEST(LogKill, KilledWriterKeepsRecordsWrittenBeforeTheKill) {
  constexpr std::uint64_t kRecords = 20;
  const std::string path = ::testing::TempDir() + "/talos_log_kill_" +
                           std::to_string(::getpid()) + ".tlog";

  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) LogUntilKilled(path);

  // The deadline only bounds a hang (docs/TESTING.md §6.3).
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds{10};
  bool in_order = true;
  std::uint64_t seen = 0;
  while ((seen = IntactPrefix(path, in_order)) < kRecords &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  ASSERT_EQ(::kill(child, SIGKILL), 0);
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFSIGNALED(status)) << "child exited with " << status;
  ASSERT_GE(seen, kRecords) << "the log held " << seen << " records after 10 s"
                            << " of logging at 200 Hz with no sync due";

  const std::uint64_t kept = IntactPrefix(path, in_order);
  EXPECT_TRUE(in_order) << "records out of order, damaged or wrong";
  EXPECT_GE(kept, seen) << "the kill lost records already in the file";
  ::unlink(path.c_str());
}

}  // namespace
}  // namespace talos::event::log

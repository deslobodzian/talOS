#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"

namespace talos::event::log {
namespace {

using std::chrono::steady_clock;

// What the child tells the parent after each record is committed.
struct Report {
  std::uint64_t dispatch_index;
  std::int64_t committed_ns;  // steady_clock, the same clock in both.
};
constexpr auto kReportBytes = static_cast<ssize_t>(sizeof(Report));

std::int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             steady_clock::now().time_since_epoch())
      .count();
}

// The child: logs one record per `cadence` like a 200 Hz node, reporting each
// to `report_fd` once it is committed, until it is killed.
[[noreturn]] void LogUntilKilled(const std::string& path, int report_fd,
                                 std::chrono::nanoseconds cadence) {
  LogWriter::Options options{};
  // The periodic fsync must not be what puts the records in the file: it is
  // the power-loss mechanism, not the crash one.
  options.sync_interval = std::chrono::hours{1};
  LogWriter writer{path, "killed", options};
  writer.start({}, {});
  const std::vector<std::byte> payload(16, std::byte{0x5a});
  auto next = steady_clock::now();
  for (std::uint64_t i = 1;; ++i) {
    Context context{};
    context.dispatch_index = i;
    const Report report{i, NowNs()};
    writer.dispatch(context, payload);
    if (::write(report_fd, &report, sizeof(report)) != kReportBytes) {
      ::_exit(2);
    }
    next += cadence;
    std::this_thread::sleep_until(next);
  }
}

// A node killed by a signal (the launcher's stall kill, a segfault, an abort)
// runs no destructor and no flush: what its log keeps is what the writer had
// already handed to the kernel. The writer used to write a chunk that was not
// full only at its periodic sync, once a second, so a SIGKILLed
// hardware_node lost the last 826 ms of its log on the bench (2026-11-11).
// Here a child logs at 200 Hz and is SIGKILLed after 200 ms with no sync due:
// every record committed more than kLossBound before the kill must be in the
// file. kLossBound is the writer's default write_interval (10 ms) plus 40 ms
// for its SCHED_OTHER thread to be scheduled on a loaded host; the defect
// lost every record. Consumer: whoever reads a crashed node's log for why it
// died (no catalogue row states this bound yet; REQ-OPS-12 covers power loss).
TEST(LogKill, KilledWriterKeepsEveryRecordOlderThanTheWriteInterval) {
  constexpr auto kCadence = std::chrono::milliseconds{5};
  constexpr std::uint64_t kReportsBeforeKill = 40;
  constexpr auto kLossBound =
      LogWriterOptions{}.write_interval + std::chrono::milliseconds{40};
  const std::string path = ::testing::TempDir() + "/talos_log_kill_" +
                           std::to_string(::getpid()) + ".tlog";

  int fds[2];
  ASSERT_EQ(::pipe(fds), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::close(fds[0]);
    LogUntilKilled(path, fds[1], kCadence);
  }
  ::close(fds[1]);

  std::vector<Report> reports;
  Report report{};
  while (reports.size() < kReportsBeforeKill &&
         ::read(fds[0], &report, sizeof(report)) == kReportBytes) {
    reports.push_back(report);
  }
  const std::int64_t kill_ns = NowNs();
  ASSERT_EQ(::kill(child, SIGKILL), 0);
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFSIGNALED(status)) << "child exited with " << status;
  // Reports the child made between the parent's last read and the kill.
  while (::read(fds[0], &report, sizeof(report)) == kReportBytes) {
    reports.push_back(report);
  }
  ::close(fds[0]);
  ASSERT_GE(reports.size(), kReportsBeforeKill);

  // The file holds a prefix of the records, in order.
  LogReader reader{path};
  LogReader::Record record{};
  std::uint64_t last = 0;
  while (reader.next(record)) {
    ASSERT_EQ(record.header.dispatch_index, last + 1);
    ++last;
  }
  EXPECT_EQ(reader.damage().spans, 0u);

  const std::int64_t bound_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(kLossBound).count();
  for (const Report& committed : reports) {
    if (committed.dispatch_index <= last) continue;
    EXPECT_GT(committed.committed_ns, kill_ns - bound_ns)
        << "record " << committed.dispatch_index << " was committed "
        << (kill_ns - committed.committed_ns) / 1000
        << " us before the kill and is not in the log (last record " << last
        << " of " << reports.size() << " committed; bound " << bound_ns / 1000
        << " us = write_interval + 40 ms scheduling)";
    break;  // The first missing record is the oldest one lost.
  }
  ::unlink(path.c_str());
}

}  // namespace
}  // namespace talos::event::log

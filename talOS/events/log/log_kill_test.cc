#include <signal.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <new>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"

namespace talos::event::log {
namespace {

constexpr std::size_t kMaxTrackedRecords = 16384;

struct SharedProgress {
  std::atomic<std::uint64_t> committed{0};
  // Steady-clock nanoseconds when record `index` (1-based,
  // index < kMaxTrackedRecords) returned from dispatch(). Written relaxed
  // before `committed.store(index, release)` so any acquire load seeing
  // `committed >= index` reads the matching timestamp.
  std::atomic<std::int64_t> commit_ns[kMaxTrackedRecords]{};
};

struct MappedProgress {
  SharedProgress* ptr{nullptr};

  MappedProgress() {
    void* mem = ::mmap(nullptr, sizeof(SharedProgress), PROT_READ | PROT_WRITE,
                       MAP_SHARED | MAP_ANON, -1, 0);
    if (mem != MAP_FAILED) {
      ptr = new (mem) SharedProgress{};
    }
  }
  ~MappedProgress() {
    if (ptr != nullptr) {
      ptr->~SharedProgress();
      ::munmap(ptr, sizeof(SharedProgress));
    }
  }
  MappedProgress(const MappedProgress&) = delete;
  MappedProgress& operator=(const MappedProgress&) = delete;
};

std::vector<std::byte> Payload(std::uint64_t index) {
  std::vector<std::byte> payload(16);
  for (std::size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<std::byte>((index + i) % 251 + 1);
  }
  return payload;
}

// The child: logs one record every 5 ms, like a 200 Hz node, until killed,
// publishing each committed dispatch_index in shared memory immediately after
// dispatch() returns. The periodic fsync is an hour away and a chunk holds
// about 3000 of these records (15 s of them), so neither a sync nor a full
// chunk can be what puts a record in the file within the parent's wait.
[[noreturn]] void LogUntilKilled(const std::string& path,
                                 SharedProgress* progress) {
  LogWriter::Options options{};
  options.sync_interval = std::chrono::hours{1};
  LogWriter writer{path, "killed", options};
  writer.start({}, {});
  for (std::uint64_t i = 1;; ++i) {
    Context context{};
    context.dispatch_index = i;
    writer.dispatch(context, Payload(i));
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    if (i < kMaxTrackedRecords) {
      progress->commit_ns[i].store(now_ns, std::memory_order_relaxed);
    }
    progress->committed.store(i, std::memory_order_release);
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
// Here the parent verifies the default 10 ms write_interval, waits until the
// child has written kRecords so the file and writer thread are live, lets the
// child keep streaming for 150 ms without pre-waiting for those records to hit
// disk, and SIGKILLs the child mid-stream: every record committed more than
// 100 ms before SIGKILL (10x write_interval, orders of magnitude below the 1 h
// sync and 15 s chunk fill, catching 250 ms–1 s write_interval regressions)
// must survive intact and in order in `.tlog`.
// REQ-OPS-31
TEST(LogKill, KilledWriterKeepsRecordsWrittenBeforeTheKill) {
  constexpr auto kExpectedWriteInterval = std::chrono::milliseconds{10};
  ASSERT_EQ(LogWriter::Options{}.write_interval, kExpectedWriteInterval);
  // REQ-OPS-31: 10 ms write_interval plus writer scheduling delay; 100 ms is
  // generous against scheduling jitter while catching 250 ms–1 s regressions.
  constexpr auto kFlushBound = std::chrono::milliseconds{100};
  constexpr auto kPostSeenWindow = std::chrono::milliseconds{150};

  constexpr std::uint64_t kRecords = 20;
  const std::string path = ::testing::TempDir() + "/talos_log_kill_" +
                           std::to_string(::getpid()) + ".tlog";

  MappedProgress progress;
  ASSERT_NE(progress.ptr, nullptr);

  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) LogUntilKilled(path, progress.ptr);

  // The 10 s deadline only bounds a hang (docs/TESTING.md §6.3).
  std::chrono::steady_clock::time_point deadline{};
  std::chrono::steady_clock::time_point kill_time{};
  bool in_order = true;
  std::uint64_t seen = 0;
  std::uint64_t first_post_seen = 0;
  std::int64_t first_post_seen_ns = 0;
  bool reached_kill_window = false;
  for (bool first = true;; first = false) {
    const auto now = std::chrono::steady_clock::now();
    if (first) deadline = now + std::chrono::seconds{10};
    if (now >= deadline) {
      kill_time = now;
      break;
    }
    if (seen < kRecords) {
      seen = IntactPrefix(path, in_order);
    } else {
      const std::uint64_t committed =
          progress.ptr->committed.load(std::memory_order_acquire);
      if (first_post_seen == 0 && committed > seen &&
          seen + 1 < kMaxTrackedRecords) {
        first_post_seen = seen + 1;
        first_post_seen_ns = progress.ptr->commit_ns[first_post_seen].load(
            std::memory_order_relaxed);
      }
      if (first_post_seen != 0) {
        const auto now_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch())
                .count();
        if (now_ns - first_post_seen_ns >=
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                kPostSeenWindow)
                .count()) {
          kill_time = now;
          reached_kill_window = true;
          break;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  ASSERT_EQ(::kill(child, SIGKILL), 0);
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFSIGNALED(status)) << "child exited with " << status;
  ASSERT_GE(seen, kRecords) << "the log held " << seen << " records after 10 s"
                            << " of logging at 200 Hz with no sync due";
  ASSERT_TRUE(reached_kill_window)
      << "child did not stream past post-seen window within 10 s";

  const std::uint64_t final_committed =
      progress.ptr->committed.load(std::memory_order_acquire);
  const auto kill_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           kill_time.time_since_epoch())
                           .count();
  const auto bound_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(kFlushBound).count();
  const std::uint64_t tracked = std::min(
      final_committed, static_cast<std::uint64_t>(kMaxTrackedRecords - 1));
  std::uint64_t expected_survived = 0;
  for (std::uint64_t i = 1; i <= tracked; ++i) {
    const std::int64_t commit_ns =
        progress.ptr->commit_ns[i].load(std::memory_order_relaxed);
    if (kill_ns - commit_ns > bound_ns) {
      expected_survived = i;
    } else {
      break;
    }
  }
  ASSERT_GE(expected_survived, first_post_seen)
      << "no post-seen record was committed more than 100 ms before SIGKILL";

  const std::uint64_t kept = IntactPrefix(path, in_order);
  EXPECT_TRUE(in_order) << "records out of order, damaged or wrong";
  EXPECT_GE(kept, expected_survived)
      << "SIGKILL lost records committed > 100 ms before the kill "
      << "(kept=" << kept << ", expected_survived=" << expected_survived
      << ", final_committed=" << final_committed << ", seen=" << seen << ")";
  ::unlink(path.c_str());
}

}  // namespace
}  // namespace talos::event::log

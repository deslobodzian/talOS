// RULES.md 3 for the event log's gap path: the loop thread drops records,
// opens and closes a gap and writes its GAP record without allocating. Here,
// not in talOS/events, because AllocationProbe is a talOS/node test library
// and talOS/events may not depend upward.

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "talOS/events/context.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/time.h"
#include "talOS/node/allocation_probe.h"

namespace talos::event::log {
namespace {

// Holds the writer thread in before_write while closed, one write per
// permit; entries() counts the writes begun.
class WriteGate {
 public:
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
  // Waits until `n` writes have begun (a hang is bounded by the test's
  // timeout).
  void wait_for_entries(std::uint64_t n) {
    std::unique_lock lock{mutex_};
    changed_.wait(lock, [this, n] { return entries_.load() >= n; });
  }
  void enter() {
    std::unique_lock lock{mutex_};
    entries_.fetch_add(1);
    changed_.notify_all();
    changed_.wait(lock, [this] { return open_ || permits_ > 0; });
    if (!open_) --permits_;
  }
  std::uint64_t entries() const { return entries_.load(); }

 private:
  std::mutex mutex_;
  std::condition_variable changed_;
  bool open_{true};
  int permits_{0};
  std::atomic<std::uint64_t> entries_{0};
};

struct OpenAtExit {
  WriteGate& gate;
  ~OpenAtExit() { gate.open(); }
};

// Dispatch `d`: a TIMER and a SEND of 64 bytes each.
void Dispatch(LogWriter& writer, std::uint64_t d,
              const std::vector<std::byte>& payload) {
  Context context{};
  context.kind = EventKind::TIMER;
  context.dispatch_index = d;
  context.event_time = MonotonicTime::from_nanos(static_cast<std::int64_t>(d));
  context.now = context.event_time;
  writer.dispatch(context, payload);
  writer.send(context, 1, payload, 0, d);
}

// A disk that stops returning fills the pool; the loop thread then drops
// hundreds of records into one gap, and once half the pool is written
// resumes with a GAP record, allocating nothing in either window.
// REQ-RT-11
TEST(LogWriterAllocation, DroppingAndResumingAllocateNothingOnTheLoopThread) {
  if (const auto why = talos::testing::AllocationProbe::Unsupported();
      !why.empty()) {
    GTEST_SKIP() << why;
  }
  const std::string path = ::testing::TempDir() + "/log_writer_allocation_" +
                           std::to_string(::getpid()) + ".tlog";
  const std::vector<std::byte> payload(64, std::byte{0x5a});
  WriteGate gate;
  LogWriter::Options options{4096, 4, /*background=*/true};
  options.write_interval = std::chrono::hours{24};
  options.sync_interval = std::chrono::hours{24};
  options.before_write = [&gate] { gate.enter(); };
  {
    LogWriter writer{path, "allocation", options};
    const OpenAtExit open_at_exit{gate};
    writer.start({}, {});
    Dispatch(writer, 1, payload);
    writer.flush();  // Only a handed-over chunk is written from here on.
    const std::uint64_t base = gate.entries();
    gate.close();

    talos::testing::AllocationProbe probe;
    probe.start();
    // Four chunks of 16 dispatches handed over, then 200 dropped.
    for (std::uint64_t d = 2; d <= 265; ++d) Dispatch(writer, d, payload);
    const std::uint64_t dropping = probe.stop();
    EXPECT_EQ(dropping, 0u) << probe.sites();
    ASSERT_TRUE(writer.losing());

    gate.permit();
    gate.permit();
    gate.wait_for_entries(base + 3);
    probe.start();
    Dispatch(writer, 266, payload);  // resumes: a GAP record, then 266
    const std::uint64_t resuming = probe.stop();
    EXPECT_EQ(resuming, 0u) << probe.sites();
    EXPECT_FALSE(writer.losing());
    EXPECT_EQ(writer.gaps(), 1u);
    gate.open();
  }
  ::unlink(path.c_str());
}

}  // namespace
}  // namespace talos::event::log

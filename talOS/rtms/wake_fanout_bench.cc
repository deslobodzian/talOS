// What a publication costs its writer by reader count (README.md "Publish
// cost"): the time RTMSQueue::write() takes with 0, 1, 8 and 32 reader
// processes asleep on the topic, and with 32 slots held of which 0 or 8 are
// armed. Each armed reader is its own process sleeping in poll(2) on its own
// WakeReceiver, as a prompt watcher's loop does; before each publication the
// writer waits until every one has read the last message and armed again,
// so every timed write notifies all of them. The "walk" rows arm slots that
// name no receiver: the write takes the waiters word and walks the table
// but sends nothing, which isolates the cost of the table's size from the
// sends.
//
// Run: bazel run -c opt //talOS/rtms:wake_fanout_bench [-- PUBLICATIONS]
// A timing from a loaded or sleeping host is not evidence (DEVELOPMENT.md
// §5): the first line prints the load average.

#include <poll.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <new>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "talOS/rtms/rtms.h"
#include "talOS/rtms/wake.h"
#include "talOS/schema/identity.h"

namespace {

struct Payload {
  std::uint64_t value;
};

const talos::schema::Identity kSchema{"bench.WakeFanout", 0xFA4F0A7ULL};
constexpr std::size_t kSlots = 1024;
constexpr int kWarmup = 200;
// Bounds a hang only: a reader not woken within this gives up.
constexpr int kHangBoundMs = 10'000;

RTMSOptions WriterOptions() {
  return RTMSOptions{.overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
                     .read_mode = ReadMode::SEQUENCE,
                     .publisher = true};
}

RTMSOptions ReaderOptions() {
  return RTMSOptions{.overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
                     .read_mode = ReadMode::SEQUENCE};
}

std::int64_t NowNs() {
  timespec now{};
  ::clock_gettime(CLOCK_MONOTONIC, &now);
  return (static_cast<std::int64_t>(now.tv_sec) * 1'000'000'000) + now.tv_nsec;
}

// A reader process: reads every publication, arms, counts the arm in
// `armed`, and sleeps until woken, until it has read `total` messages.
int ArmedReader(const std::string& path, int total,
                std::atomic<std::uint64_t>* armed) {
  try {
    RTMSQueue queue{path,    sizeof(Payload), alignof(Payload),
                    kSchema, kSlots,          ReaderOptions()};
    const auto reader = queue.register_reader();
    if (!reader) return 1;
    talos::WakeReceiver receiver;
    queue.set_reader_wake(*reader, receiver.id());
    int received = 0;
    Payload payload{};
    while (received < total) {
      MessageInfo info{};
      const ReadResult result = queue.read_next(
          *reader, std::as_writable_bytes(std::span{&payload, 1}), info);
      if (result == ReadResult::OK) {
        ++received;
        continue;
      }
      if (result != ReadResult::EMPTY) return 2;
      if (!queue.arm_wake(*reader)) continue;
      armed->fetch_add(1, std::memory_order_acq_rel);
      pollfd entry{.fd = receiver.fd(), .events = POLLIN, .revents = 0};
      if (::poll(&entry, 1, kHangBoundMs) != 1) return 3;
      receiver.drain();
    }
    return 0;
  } catch (...) {
    return 4;
  }
}

struct Case {
  const char* name;
  std::size_t armed;
  std::size_t held;  // reader slots held in all, armed or not
  bool receivers;    // armed slots name a receiver in a reader process
};

struct Result {
  std::vector<double> write_us;
  bool ok{false};
};

// Waits until `armed` reaches `target`; false after kHangBoundMs.
bool AwaitArms(const std::atomic<std::uint64_t>& armed, std::uint64_t target) {
  const std::int64_t give_up = NowNs() + (kHangBoundMs * 1'000'000LL);
  while (armed.load(std::memory_order_acquire) < target) {
    if (NowNs() > give_up) return false;
    std::this_thread::yield();
  }
  return true;
}

Result Run(const Case& run, int publications, int index) {
  const std::string path = "/bench_fanout_" + std::to_string(::getpid()) + "_" +
                           std::to_string(index);
  const int total = kWarmup + publications;
  const std::size_t processes = run.receivers ? run.armed : 0;
  Result result;

  void* shared =
      ::mmap(nullptr, sizeof(std::atomic<std::uint64_t>),
             PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  if (shared == MAP_FAILED) return result;
  auto* armed = new (shared) std::atomic<std::uint64_t>{0};

  {
    RTMSQueue writer{path,    sizeof(Payload), alignof(Payload),
                     kSchema, kSlots,          WriterOptions()};
    // The slots no reader process holds, in this process: never armed, as
    // an at-drain watcher's, or (walk rows) armed with no receiver.
    RTMSQueue local{path,    sizeof(Payload), alignof(Payload),
                    kSchema, kSlots,          ReaderOptions()};
    std::vector<std::size_t> walk_slots;
    bool ok = true;
    for (std::size_t i = processes; ok && i < run.held; ++i) {
      const auto slot = local.register_reader();
      ok = slot.has_value();
      if (ok && walk_slots.size() < run.armed - processes) {
        walk_slots.push_back(*slot);
      }
    }

    std::vector<pid_t> children;
    for (std::size_t i = 0; ok && i < processes; ++i) {
      const pid_t child = ::fork();
      if (child == 0) ::_exit(ArmedReader(path, total, armed));
      if (child < 0) break;
      children.push_back(child);
    }
    ok = ok && children.size() == processes;

    result.write_us.reserve(static_cast<std::size_t>(publications));
    for (int i = 0; ok && i < total; ++i) {
      // Every reader process has read message i - 1 and armed again.
      ok = AwaitArms(*armed, processes * static_cast<std::uint64_t>(i + 1));
      for (const std::size_t slot : walk_slots) (void)local.arm_wake(slot);
      if (!ok) break;
      const Payload payload{static_cast<std::uint64_t>(i)};
      const std::int64_t start = NowNs();
      const WriteStatus status =
          writer.write(std::as_bytes(std::span{&payload, 1}));
      const std::int64_t end = NowNs();
      ok = status.result == WriteResult::SUCCESS;
      if (ok && i >= kWarmup) {
        result.write_us.push_back(static_cast<double>(end - start) / 1000.0);
      }
    }
    // Each timed write notified every reader process, and no walk slot.
    ok = ok &&
         writer.wakes_sent() == processes * static_cast<std::uint64_t>(total);

    for (const pid_t child : children) {
      int status = 0;
      if (!ok) ::kill(child, SIGKILL);
      if (::waitpid(child, &status, 0) != child || !WIFEXITED(status) ||
          WEXITSTATUS(status) != 0) {
        ok = false;
      }
    }
    result.ok = ok;
  }
  ::munmap(shared, sizeof(std::atomic<std::uint64_t>));
  return result;
}

double Percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const auto at = static_cast<std::size_t>(
      fraction * static_cast<double>(values.size() - 1));
  return values[at];
}

}  // namespace

int main(int argc, char** argv) {
  const int publications = argc > 1 ? std::atoi(argv[1]) : 2000;
  if (publications <= 0) {
    std::fprintf(stderr, "usage: wake_fanout_bench [PUBLICATIONS > 0]\n");
    return 2;
  }

  utsname host{};
  ::uname(&host);
  std::array<double, 3> load{};
  const int loads = ::getloadavg(load.data(), static_cast<int>(load.size()));
  std::printf(
      "wake_fanout_bench: %s %s %s, %ld CPUs online, load %.2f; "
      "MAX_READERS %zu, segment version %llu; %d timed publications per "
      "case after %d warm-up\n\n",
      host.sysname, host.release, host.machine, ::sysconf(_SC_NPROCESSORS_ONLN),
      loads > 0 ? load[0] : -1.0, MAX_READERS,
      static_cast<unsigned long long>(kRtmsVersion), publications, kWarmup);
  std::printf("%-5s %6s %5s %10s %10s %10s %10s %10s\n", "row", "armed", "held",
              "median us", "p90 us", "p99 us", "max us", "us/armed");

  const std::array<Case, 9> cases{{
      {.name = "wake", .armed = 0, .held = 0, .receivers = true},
      {.name = "wake", .armed = 0, .held = 32, .receivers = true},
      {.name = "wake", .armed = 1, .held = 1, .receivers = true},
      {.name = "wake", .armed = 8, .held = 8, .receivers = true},
      {.name = "wake", .armed = 8, .held = 32, .receivers = true},
      {.name = "wake", .armed = 32, .held = 32, .receivers = true},
      {.name = "walk", .armed = 8, .held = 8, .receivers = false},
      {.name = "walk", .armed = 8, .held = 32, .receivers = false},
      {.name = "walk", .armed = 32, .held = 32, .receivers = false},
  }};
  int failures = 0;
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const Case& run = cases[i];
    if (run.held > MAX_READERS) {
      std::printf("%-5s %6zu %5zu   skipped: more than MAX_READERS slots\n",
                  run.name, run.armed, run.held);
      continue;
    }
    const Result result = Run(run, publications, static_cast<int>(i));
    if (!result.ok) {
      std::printf("%-5s %6zu %5zu   FAILED: a reader was not woken or exited\n",
                  run.name, run.armed, run.held);
      ++failures;
      continue;
    }
    const double median = Percentile(result.write_us, 0.5);
    std::printf("%-5s %6zu %5zu %10.2f %10.2f %10.2f %10.2f %10.2f\n", run.name,
                run.armed, run.held, median, Percentile(result.write_us, 0.9),
                Percentile(result.write_us, 0.99),
                Percentile(result.write_us, 1.0),
                run.armed == 0 ? 0.0 : median / static_cast<double>(run.armed));
  }
  return failures == 0 ? 0 : 1;
}

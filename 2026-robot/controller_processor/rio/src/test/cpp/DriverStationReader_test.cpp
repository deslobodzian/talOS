// The RoboRIO loop's Driver Station view without NetComm on the loop thread.
// WPILib's refresh asks FRC_NetCommDaemon for the control word, a round trip
// that took up to 13.5 ms on the bench with no Driver Station attached and
// made the 5 ms loop overrun; DriverStationReader moves it, and the mode
// report, to a helper thread. These tests run that thread for real, with its
// WPILib calls replaced by a Gate the test steers, and wait on the Gate's
// events, never on time (TESTING.md §6.3); the clock is injected.
#include "DriverStationReader.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "driver_station/driver_station.h"

namespace {

namespace ds = talos::driver_station;
using talos::rio::DriverStationReader;
using talos::rio::ProgramMode;

// The default Config's command_timeout_us, the loop's staleness bound.
constexpr uint64_t kTimeoutUs = 100'000;
// An arbitrary steady-clock time.
constexpr uint64_t kT0 = 1'000'000'000;
constexpr uint32_t kTeleopEnabled =
    ds::kDsAttached | ds::kEnabled | ds::kTeleop;
constexpr uint32_t kAutonomousEnabled =
    ds::kDsAttached | ds::kEnabled | ds::kAutonomous;
// A stop bound that never abandons on a slow host.
constexpr auto kNeverAbandon = std::chrono::seconds{10};

// The helper's WPILib calls, for a test to steer and wait on. `wait` returns
// once per packet the test posts (or for good once Stop wakes it); `refresh`
// returns `flags` at once, unless the test holds it, in which case it blocks
// until released: a NetComm call that does not return, until it does.
// Shared (std::shared_ptr) with the calls, so a helper the reader abandoned
// keeps it alive; `destroyed` is told how many modes it saw when the last
// holder lets go. Every wait is on an event; the 10 s bound only fails a test
// that would otherwise hang.
struct Gate {
  std::mutex mutex;
  std::condition_variable changed;
  int posted = 0;
  int taken = 0;
  bool woken = false;
  int waits = 0;
  bool hold = false;
  bool held = false;
  uint32_t flags = 0;
  std::vector<ProgramMode> observed;
  std::shared_ptr<std::promise<std::size_t>> destroyed;

  ~Gate() {
    if (destroyed) destroyed->set_value(observed.size());
  }

  // The helper's side.
  void Wait() {
    std::unique_lock lock{mutex};
    ++waits;
    changed.notify_all();
    changed.wait(lock, [this] { return woken || taken < posted; });
    if (taken < posted) ++taken;
  }
  void Wake() {
    {
      const std::lock_guard lock{mutex};
      woken = true;
    }
    changed.notify_all();
  }
  uint32_t Refresh() {
    std::unique_lock lock{mutex};
    if (hold) {
      held = true;
      changed.notify_all();
      changed.wait(lock, [this] { return !hold; });
      held = false;
    }
    return flags;
  }
  void Observe(ProgramMode mode) {
    const std::lock_guard lock{mutex};
    observed.push_back(mode);
  }

  // The test's side.
  template <typename Predicate>
  bool WaitUntil(Predicate predicate) {
    std::unique_lock lock{mutex};
    return changed.wait_for(lock, std::chrono::seconds{10}, predicate);
  }
  void Post() {
    {
      const std::lock_guard lock{mutex};
      ++posted;
    }
    changed.notify_all();
  }
  // Posts a packet and waits until the helper has refreshed, reported (or
  // not) and is waiting for the next one.
  bool Step() {
    Post();
    return WaitUntil([this] { return waits > posted; });
  }
  void SetFlags(uint32_t value) {
    const std::lock_guard lock{mutex};
    flags = value;
  }
  void Hold() {
    const std::lock_guard lock{mutex};
    hold = true;
  }
  bool WaitHeld() {
    return WaitUntil([this] { return held; });
  }
  void Release() {
    {
      const std::lock_guard lock{mutex};
      hold = false;
    }
    changed.notify_all();
  }
  std::vector<ProgramMode> Observed() {
    const std::lock_guard lock{mutex};
    return observed;
  }
};

DriverStationReader::Calls CallsThrough(const std::shared_ptr<Gate>& gate) {
  DriverStationReader::Calls calls;
  calls.wait = [gate] { gate->Wait(); };
  calls.wake = [gate] { gate->Wake(); };
  calls.refresh = [gate] { return gate->Refresh(); };
  calls.observe = [gate](ProgramMode mode) { gate->Observe(mode); };
  return calls;
}

// The loop's clock, set by the test.
using TestClock = std::shared_ptr<std::atomic<uint64_t>>;

TestClock MakeClock() { return std::make_shared<std::atomic<uint64_t>>(kT0); }

DriverStationReader::Clock ClockThrough(const TestClock& clock) {
  return [clock] { return clock->load(); };
}

// A report from a reader started at priority 0 can only be an abandonment,
// which these tests must not see.
void NoReport(std::string_view line) { ADD_FAILURE() << line; }

// The RoboRIO loop must tick on its own clock whatever NetComm does: with
// the helper inside a refresh that has not returned (the daemon's round
// trip), the loop's Read returns at once with the last refreshed flags. A
// Read that waited for the refresh in flight (a lock held across it) would
// stall the loop, and with it the State per gateway tick, for the length of
// the round trip.
// REQ-RT-13
TEST(DriverStationReader, ALoopReadDoesNotWaitForARefreshInFlight) {
  auto gate = std::make_shared<Gate>();
  const TestClock clock = MakeClock();
  DriverStationReader reader{CallsThrough(gate), ClockThrough(clock), NoReport,
                             kNeverAbandon};
  reader.Start(0);
  gate->SetFlags(kTeleopEnabled);
  ASSERT_TRUE(gate->Step());
  EXPECT_EQ(reader.Read(kT0, kTimeoutUs), kTeleopEnabled);

  gate->Hold();
  gate->SetFlags(0);
  gate->Post();
  ASSERT_TRUE(gate->WaitHeld());
  auto read = std::async(std::launch::async,
                         [&] { return reader.Read(kT0 + 5'000, kTimeoutUs); });
  const bool returned =
      read.wait_for(std::chrono::seconds{10}) == std::future_status::ready;
  gate->Release();
  ASSERT_TRUE(returned) << "the loop's Read waited for the refresh in flight";
  EXPECT_EQ(read.get(), kTeleopEnabled)
      << "the loop's Read did not return the last refreshed flags";
}

// A refresh that does not return (a wedged NetComm daemon) must not leave
// the gateway enabled on the last flags it saw: once the last completed
// refresh is command_timeout_us old, the loop reads no Driver Station (flags
// 0), so the gateway disables. When the refresh returns, the mode comes back
// as a new mode generation, never the one issued before the gap, so no
// target stamped with the old generation can drive (ARCHITECTURE.md
// "Targets"). The loop's sequence here is Robot.cpp's: Read, then the
// ModeTracker observes the flags and stamps the packet.
// REQ-SAFE-10
TEST(DriverStationReader, AViewNotRefreshedWithinTheTimeoutHasNoMode) {
  auto gate = std::make_shared<Gate>();
  const TestClock clock = MakeClock();
  DriverStationReader reader{CallsThrough(gate), ClockThrough(clock), NoReport,
                             kNeverAbandon};
  reader.Start(0);
  ds::ModeTracker tracker;
  struct Tick {
    uint32_t flags;
    uint64_t generation;
  };
  const auto tick = [&](uint64_t now_us) {
    const uint32_t flags = reader.Read(now_us, kTimeoutUs);
    tracker.Observe(flags, now_us);
    ds::DriverStationData packet{};
    packet.sample_time_us = now_us;
    packet.fms.flags = flags;
    tracker.Stamp(packet);
    return Tick{flags, packet.mode_since_us};
  };

  gate->SetFlags(kTeleopEnabled);
  ASSERT_TRUE(gate->Step());
  const Tick enabled = tick(kT0);
  ASSERT_EQ(enabled.flags, kTeleopEnabled);

  gate->Hold();
  gate->Post();
  ASSERT_TRUE(gate->WaitHeld());
  const Tick last_fresh = tick(kT0 + kTimeoutUs - 1);
  EXPECT_EQ(last_fresh.flags, kTeleopEnabled)
      << "a refresh younger than the timeout read as stale";
  EXPECT_EQ(last_fresh.generation, enabled.generation);
  const Tick stale = tick(kT0 + kTimeoutUs);
  EXPECT_EQ(stale.flags, 0u)
      << "a refresh " << kTimeoutUs << " us old still drives the gateway";

  clock->store(kT0 + kTimeoutUs + 5'000);
  gate->Release();
  ASSERT_TRUE(gate->WaitUntil([&] { return gate->waits > gate->posted; }));
  const Tick resumed = tick(kT0 + kTimeoutUs + 5'000);
  EXPECT_EQ(resumed.flags, kTeleopEnabled);
  EXPECT_GT(resumed.generation, enabled.generation)
      << "the mode came back under the generation issued before the gap";
}

// The Driver Station shows the mode robot code reports (TimedRobot reports
// it each iteration). The helper reports it after each refresh only while the
// loop's last Read is at most 20 ms old, so a stalled loop stops the reports,
// as it did when the loop made them; and it reports the mode of the flags it
// just refreshed.
TEST(DriverStationReader, ReportsTheModeOnlyWhileTheLoopTicks) {
  auto gate = std::make_shared<Gate>();
  const TestClock clock = MakeClock();
  DriverStationReader reader{CallsThrough(gate), ClockThrough(clock), NoReport,
                             kNeverAbandon};
  reader.Start(0);
  gate->SetFlags(kAutonomousEnabled);
  ASSERT_TRUE(gate->Step());
  EXPECT_TRUE(gate->Observed().empty()) << "reported before the loop ticked";

  reader.Read(kT0, kTimeoutUs);
  clock->store(kT0 + DriverStationReader::kLoopTickWindowUs);
  ASSERT_TRUE(gate->Step());
  EXPECT_EQ(gate->Observed(), std::vector{ProgramMode::kAutonomous});

  clock->store(kT0 + DriverStationReader::kLoopTickWindowUs + 1);
  ASSERT_TRUE(gate->Step());
  EXPECT_EQ(gate->Observed().size(), 1u)
      << "reported with the loop's last tick 20.001 ms old";

  gate->SetFlags(ds::kDsAttached | ds::kAutonomous);
  reader.Read(kT0 + DriverStationReader::kLoopTickWindowUs + 1, kTimeoutUs);
  ASSERT_TRUE(gate->Step());
  EXPECT_EQ(gate->Observed(),
            (std::vector{ProgramMode::kAutonomous, ProgramMode::kDisabled}));
}

// Stopping the program (EndCompetition, or a test's end) must not hang on a
// NetComm call that never returns: past its bound (here 0, so the test waits
// on nothing) Stop abandons the helper with one console line and returns,
// and so does the destructor. The abandoned helper owns what it still
// touches: once its refresh returns it exits, reporting no mode although the
// loop had ticked, and lets its calls go.
// REQ-RT-13
TEST(DriverStationReader, StopAbandonsARefreshThatDoesNotReturn) {
  auto destroyed = std::make_shared<std::promise<std::size_t>>();
  std::future<std::size_t> released = destroyed->get_future();
  std::weak_ptr<Gate> weak;
  std::vector<std::string> lines;
  {
    auto gate = std::make_shared<Gate>();
    gate->destroyed = destroyed;
    weak = gate;
    const TestClock clock = MakeClock();
    DriverStationReader reader{
        CallsThrough(gate), ClockThrough(clock),
        [&lines](std::string_view line) { lines.emplace_back(line); },
        std::chrono::microseconds{0}};
    reader.Start(0);
    reader.Read(kT0, kTimeoutUs);
    gate->SetFlags(kTeleopEnabled);
    gate->Hold();
    gate->Post();
    ASSERT_TRUE(gate->WaitHeld());
    gate.reset();
    reader.Stop();
    EXPECT_EQ(lines.size(), 1u);
  }
  EXPECT_EQ(lines.size(), 1u) << "the destructor stopped the helper again";
  if (const auto gate = weak.lock()) gate->Release();
  ASSERT_EQ(released.wait_for(std::chrono::seconds{10}),
            std::future_status::ready)
      << "the abandoned helper never exited";
  EXPECT_EQ(released.get(), 0u) << "the abandoned helper reported a mode";
}

}  // namespace

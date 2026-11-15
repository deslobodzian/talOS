// The RoboRIO loop's Driver Station view without NetComm on the loop thread.
// WPILib's refresh asks FRC_NetCommDaemon for the control word, a round trip
// that took up to 13.5 ms on the bench with no Driver Station attached and
// made the 5 ms loop overrun; DriverStationReader moves it, the snapshot of
// what it refreshed and the mode report to a helper thread, and the loop
// reads whole snapshots. These tests run that thread for real, with its
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

// One refresh's Driver Station: `flags`, and stick 0 with two axes and
// `buttons`.
ds::DriverStationData Sample(uint32_t flags, float axis0, float axis1,
                             uint32_t buttons) {
  ds::DriverStationData sample{};
  sample.fms.flags = flags;
  sample.fms.alliance = ds::Alliance::kBlue;
  sample.fms.station = 1;
  auto& stick = sample.joysticks[0];
  stick.connected = true;
  stick.axis_count = 2;
  stick.axes[0] = axis0;
  stick.axes[1] = axis1;
  stick.button_count = 4;
  stick.buttons = buttons;
  return sample;
}

// Two refreshes that any field-by-field mix of them differs from: enabled
// with the stick at (0, 0.9), then disabled with it at (0.9, 0) and a button
// down. Mixed at the gate below (A's fields under B's flags and first axis)
// they read disabled at (0.9, 0.9) with no button.
const ds::DriverStationData kA = Sample(kTeleopEnabled, 0.0f, 0.9f, 0);
const ds::DriverStationData kB = Sample(ds::kDsAttached, 0.9f, 0.0f, 1);
const ds::DriverStationData kNoDriverStation{};

std::string Describe(const ds::DriverStationData& view) {
  const auto& stick = view.joysticks[0];
  return "flags=" + std::to_string(view.fms.flags) +
         " connected=" + std::to_string(stick.connected) + " axes=(" +
         std::to_string(stick.axes[0]) + ", " + std::to_string(stick.axes[1]) +
         ") buttons=" + std::to_string(stick.buttons);
}

// The helper's WPILib calls, for a test to steer and wait on. `wait` returns
// once per packet the test posts (at once in `free_run`, or for good once
// Stop wakes it). `refresh` writes `sample` (in `free_run`, kA and kB in
// turn) one field at a time, as the HAL's cache would be read: the flags and
// stick 0's first axis, then, unless the test holds it there (a NetComm call
// that has not returned, mid-snapshot), the rest. Shared (std::shared_ptr)
// with the calls, so a helper the reader abandoned keeps it alive;
// `destroyed` is told how many modes it saw when the last holder lets go.
// Every wait is on an event; the 10 s bound only fails a test that would
// otherwise hang.
struct Gate {
  std::mutex mutex;
  std::condition_variable changed;
  int posted = 0;
  int taken = 0;
  bool woken = false;
  bool free_run = false;
  int waits = 0;
  std::atomic<int> refreshes{0};
  bool hold = false;
  bool held = false;
  ds::DriverStationData sample{};
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
    changed.wait(lock, [this] { return free_run || woken || taken < posted; });
    if (taken < posted) ++taken;
  }
  void Wake() {
    {
      const std::lock_guard lock{mutex};
      woken = true;
    }
    changed.notify_all();
  }
  void Refresh(ds::DriverStationData& out) {
    std::unique_lock lock{mutex};
    const int count = refreshes.load() + 1;
    const ds::DriverStationData& next =
        free_run ? (count % 2 == 1 ? kA : kB) : sample;
    out.fms.flags = next.fms.flags;
    out.joysticks[0].axes[0] = next.joysticks[0].axes[0];
    if (hold) {
      held = true;
      changed.notify_all();
      changed.wait(lock, [this] { return !hold; });
      held = false;
    }
    out = next;
    refreshes.store(count);
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
  // Waits until the helper has handled every posted packet (refreshed,
  // published, reported or not) and is waiting for the next one.
  bool WaitIdle() {
    return WaitUntil([this] { return waits > posted; });
  }
  bool Step() {
    Post();
    return WaitIdle();
  }
  void SetSample(const ds::DriverStationData& value) {
    const std::lock_guard lock{mutex};
    sample = value;
  }
  void FreeRun() {
    {
      const std::lock_guard lock{mutex};
      free_run = true;
    }
    changed.notify_all();
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
  calls.refresh = [gate](ds::DriverStationData& out) { gate->Refresh(out); };
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

// The packet the loop sends must be one Driver Station refresh, never a mix
// of two: a mixed sample can show every input neutral although no packet
// did (stick 0 moving from (0, 0.9) to (0.9, 0), read half before and half
// after, is (0, 0) or (0.9, 0.9)), and right after a new mode generation
// OperatorArming arms on an all-neutral sample, so the next packet drives
// on a stick being held. And the loop must not wait for a refresh: with the
// helper stopped mid-snapshot (B's flags and first axis written, a NetComm
// call that has not returned), the loop's Read returns at once with the last
// whole snapshot, A; once the helper finishes, B whole.
// REQ-SAFE-13, REQ-RT-13
TEST(DriverStationReader, ALoopReadIsOneWholeRefreshAndNeverWaitsForOne) {
  auto gate = std::make_shared<Gate>();
  const TestClock clock = MakeClock();
  DriverStationReader reader{CallsThrough(gate), ClockThrough(clock), NoReport,
                             kNeverAbandon};
  reader.Start(0);
  gate->SetSample(kA);
  ASSERT_TRUE(gate->Step());
  const ds::DriverStationData first = reader.Read(kT0, kTimeoutUs);
  EXPECT_TRUE(first == kA) << Describe(first);

  gate->SetSample(kB);
  gate->Hold();
  gate->Post();
  ASSERT_TRUE(gate->WaitHeld());
  auto read = std::async(std::launch::async, [&] {
    return ds::DriverStationData{reader.Read(kT0 + 5'000, kTimeoutUs)};
  });
  const bool returned =
      read.wait_for(std::chrono::seconds{10}) == std::future_status::ready;
  gate->Release();
  ASSERT_TRUE(returned) << "the loop's Read waited for the refresh in flight";
  const ds::DriverStationData during = read.get();
  EXPECT_TRUE(during == kA) << "read mid-snapshot: " << Describe(during)
                            << ", expected " << Describe(kA);

  ASSERT_TRUE(gate->WaitIdle());
  const ds::DriverStationData after = reader.Read(kT0 + 5'000, kTimeoutUs);
  EXPECT_TRUE(after == kB) << Describe(after);
}

// The same, across threads with the helper refreshing as fast as it can,
// writing kA and kB in turn field by field: every Read the loop makes is one
// of them whole (or no Driver Station, before the first), its flags and its
// inputs from the same refresh. The publication's memory ordering is what
// this exercises; the test above pins the slot handover.
// REQ-SAFE-13
TEST(DriverStationReader, ReadsDuringConstantRefreshesSeeOnlyWholeRefreshes) {
  constexpr int kRefreshes = 10'000;
  auto gate = std::make_shared<Gate>();
  const TestClock clock = MakeClock();
  DriverStationReader reader{CallsThrough(gate), ClockThrough(clock), NoReport,
                             kNeverAbandon};
  reader.Start(0);
  gate->FreeRun();
  int reads = 0;
  int mixed = 0;
  ds::DriverStationData first_mixed{};
  while (gate->refreshes.load() < kRefreshes) {
    const ds::DriverStationData& view = reader.Read(kT0, kTimeoutUs);
    ++reads;
    if (!(view == kA || view == kB || view == kNoDriverStation)) {
      if (mixed++ == 0) first_mixed = view;
    }
  }
  EXPECT_EQ(mixed, 0) << "of " << reads
                      << " reads; the first: " << Describe(first_mixed);
}

// A refresh that does not return (a wedged NetComm daemon) must not leave
// the gateway enabled on the last snapshot it saw: once the last completed
// refresh is command_timeout_us old, the loop reads no Driver Station (flags
// 0, so the gateway disables; every stick disconnected, no axis or button),
// and when the refresh returns, the mode comes back as a new mode
// generation, never the one issued before the gap, so no target stamped
// with the old generation can drive (ARCHITECTURE.md "Targets"). The loop's
// sequence here is Robot.cpp's: Read, then the ModeTracker observes the
// flags and stamps the packet built from the same view.
// REQ-SAFE-10
TEST(DriverStationReader, AViewNotRefreshedWithinTheTimeoutHasNoDriverStation) {
  auto gate = std::make_shared<Gate>();
  const TestClock clock = MakeClock();
  DriverStationReader reader{CallsThrough(gate), ClockThrough(clock), NoReport,
                             kNeverAbandon};
  reader.Start(0);
  ds::ModeTracker tracker;
  const auto tick = [&](uint64_t now_us) {
    ds::DriverStationData packet = reader.Read(now_us, kTimeoutUs);
    tracker.Observe(packet.fms.flags, now_us);
    packet.sample_time_us = now_us;
    tracker.Stamp(packet);
    return packet;
  };

  gate->SetSample(kA);
  ASSERT_TRUE(gate->Step());
  const ds::DriverStationData enabled = tick(kT0);
  ASSERT_EQ(enabled.fms.flags, kTeleopEnabled);

  gate->Hold();
  gate->Post();
  ASSERT_TRUE(gate->WaitHeld());
  const ds::DriverStationData last_fresh = tick(kT0 + kTimeoutUs - 1);
  EXPECT_EQ(last_fresh.fms.flags, kTeleopEnabled)
      << "a refresh younger than the timeout read as stale";
  EXPECT_EQ(last_fresh.mode_since_us, enabled.mode_since_us);
  ds::DriverStationData stale = tick(kT0 + kTimeoutUs);
  EXPECT_EQ(stale.fms.flags, 0u)
      << "a refresh " << kTimeoutUs << " us old still drives the gateway";
  stale.sample_time_us = 0;
  stale.mode_since_us = 0;
  EXPECT_TRUE(stale == kNoDriverStation)
      << "a stale view carries inputs: " << Describe(stale);

  clock->store(kT0 + kTimeoutUs + 5'000);
  gate->Release();
  ASSERT_TRUE(gate->WaitIdle());
  const ds::DriverStationData resumed = tick(kT0 + kTimeoutUs + 5'000);
  EXPECT_EQ(resumed.fms.flags, kTeleopEnabled);
  EXPECT_GT(resumed.mode_since_us, enabled.mode_since_us)
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
  gate->SetSample(Sample(kAutonomousEnabled, 0.0f, 0.0f, 0));
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

  gate->SetSample(Sample(ds::kDsAttached | ds::kAutonomous, 0.0f, 0.0f, 0));
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
    gate->SetSample(kA);
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

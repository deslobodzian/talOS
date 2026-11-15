#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>

#include "driver_station/driver_station.h"

namespace talos::driver_station {

// The Driver Station packet the gateway sends, from WPILib's cache (no
// NetComm call: every getter it uses copies the HAL's cache under the HAL's
// own mutex), with `flags` (DsFlags) as its flags: the loop passes the flags
// it decided this tick (rio::DriverStationReader::Read), so the packet and
// the loop's ModeTracker never see two different modes in one tick.
DriverStationData SampleDriverStation(uint64_t now_us, uint32_t flags);

// The same, with the flags of the control word WPILib's cache holds now.
DriverStationData SampleDriverStation(uint64_t now_us);

}  // namespace talos::driver_station

namespace talos::rio {

// The mode the program reports to the Driver Station (HAL_ObserveUserProgram*),
// which it displays.
enum class ProgramMode : uint8_t { kDisabled, kAutonomous, kTeleop, kTest };

// The mode to report for Driver Station flags (DsFlags): disabled unless
// kEnabled (WPILib's IsDisabled), else autonomous, test or teleop, in that
// order, as TimedRobot reports them.
constexpr ProgramMode ProgramModeOf(uint32_t flags) {
  if ((flags & driver_station::kEnabled) == 0) return ProgramMode::kDisabled;
  if ((flags & driver_station::kAutonomous) != 0) {
    return ProgramMode::kAutonomous;
  }
  if ((flags & driver_station::kTest) != 0) return ProgramMode::kTest;
  return ProgramMode::kTeleop;
}

// The RoboRIO loop's Driver Station view, read without reaching NetComm.
//
// WPILib's frc::DriverStation::RefreshData (HAL_RefreshDSData) always asks
// NetComm for the control word (FRC_NetworkCommunication_getControlWord).
// With no Driver Station attached that is a synchronous round trip to
// FRC_NetCommDaemon, a SCHED_OTHER process, serialized with every other
// thread's NetComm call; on the bench it took up to 13.5 ms. So the loop
// never refreshes: a helper thread (talos-ds) waits for a new Driver Station
// packet or kWaitTimeoutUs, whichever comes first (`wait`; WPILib's new-data
// event, which NetComm's own thread sets on each packet), refreshes WPILib's
// cache and reads its control word (`refresh`, returning DsFlags), and
// publishes the flags with the refresh's completion time on the loop's clock
// in one atomic word. Then, only while the loop has called Read within
// kLoopTickWindowUs, it reports the program's mode for those flags
// (`observe`, also a NetComm call), so a stalled loop stops the reports as
// it did when the loop made them.
//
// The loop calls Read once per tick: it records the tick and returns the
// last refresh's flags, or 0 (no Driver Station, so no mode and disabled)
// when none has completed within `stale_us` (the gateway's
// command_timeout_us). A refresh that does not return, a wedged daemon,
// therefore disables the gateway within command_timeout_us plus one tick of
// the last refresh that did, and when refreshes resume the mode comes back
// as a new mode generation (driver_station::ModeTracker sees the flags go
// through 0). Read never waits, locks or calls into WPILib.
//
// Stop (also the destructor) wakes the helper and waits for it to exit for
// at most `stop_bound`. A helper still inside a NetComm call by then is
// abandoned: detached, `report` gets one line, and Stop returns. Everything
// an abandoned helper can still touch (the calls, the clock, the published
// word) is shared with it (std::shared_ptr), never the reader itself, and it
// neither publishes nor reports a mode after Stop has begun. Start runs the
// helper once; a stopped reader is not restarted.
class DriverStationReader {
 public:
  // What the helper calls. `wait` returns on a new Driver Station packet or
  // after at most kWaitTimeoutUs; `wake` makes the pending or next `wait`
  // return at once (called by Stop, from another thread); `refresh` updates
  // WPILib's cache and returns its control word as DsFlags; `observe`
  // reports the program's mode. `refresh` and `observe` may block.
  struct Calls {
    std::function<void()> wait;
    std::function<void()> wake;
    std::function<uint32_t()> refresh;
    std::function<void(ProgramMode)> observe;
  };
  // Microseconds on the loop's clock (steady_clock).
  using Clock = std::function<uint64_t()>;
  // One console line; from the helper (its scheduling, at start) or from
  // the thread calling Start or Stop.
  using Report = std::function<void(std::string_view line)>;

  // The longest the helper waits for a packet before refreshing anyway: with
  // no Driver Station attached (none arrive) the view is still refreshed,
  // and its age stays far below any command_timeout_us (at least 40 ms).
  static constexpr uint64_t kWaitTimeoutUs = 20'000;
  // The helper reports the mode only while the loop's last Read is at most
  // this old.
  static constexpr uint64_t kLoopTickWindowUs = 20'000;
  // Stop's bound: a wait plus a refresh as slow as the bench's slowest
  // (13.5 ms), with margin.
  static constexpr std::chrono::microseconds kStopBound{100'000};

  DriverStationReader(Calls calls, Clock clock, Report report,
                      std::chrono::microseconds stop_bound = kStopBound);
  ~DriverStationReader();
  DriverStationReader(const DriverStationReader&) = delete;
  DriverStationReader& operator=(const DriverStationReader&) = delete;

  // WPILib's calls: the HAL's new-data event (HAL_ProvideNewDataEventHandle)
  // waited on with kWaitTimeoutUs, frc::DriverStation::RefreshData then
  // HAL_GetControlWord, and HAL_ObserveUserProgram*.
  static Calls WpilibCalls();

  // Starts the helper, named talos-ds. `realtime_priority` 1-99 puts it on
  // SCHED_FIFO at that priority (SCHED_RESET_ON_FORK, so a thread it starts
  // does not inherit it) on Linux, and it reports what the kernel holds; 0
  // keeps the starter's scheduling. A helper that cannot be created is
  // reported, and Read then returns 0 for good.
  void Start(int realtime_priority);
  void Stop();

  // The loop thread, once per tick at `now_us` (the clock's): see the class
  // comment.
  uint32_t Read(uint64_t now_us, uint64_t stale_us);

 private:
  // No refresh completed yet / no Read yet.
  static constexpr uint64_t kNever = std::numeric_limits<uint64_t>::max();
  // A refresh's flags (every DsFlags bit fits) below its completion time.
  static constexpr unsigned kFlagBits = 8;

  struct Shared {
    Calls calls;
    Clock clock;
    Report report;
    int realtime_priority{0};
    // (completion time << kFlagBits) | flags of the last refresh, or kNever:
    // one word, so the loop never pairs one refresh's flags with another's
    // time.
    std::atomic<uint64_t> refreshed{kNever};
    // The loop's last Read time, or kNever.
    std::atomic<uint64_t> tick_us{kNever};
    std::atomic<bool> stop{false};
    std::mutex mutex;
    std::condition_variable exit;
    bool exited{false};  // under mutex
  };

  // The helper. Touches only `shared`, so it can be abandoned.
  static void Run(const std::shared_ptr<Shared>& shared);

  const std::shared_ptr<Shared> shared_;
  const std::chrono::microseconds stop_bound_;
  std::thread thread_;
};

}  // namespace talos::rio

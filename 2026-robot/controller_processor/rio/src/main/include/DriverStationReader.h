#pragma once

#include <array>
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

// Fills `out` with what WPILib's HAL cache holds now: the control word as
// flags (DsFlags), alliance and station, match information and time, and
// every joystick's axes, buttons, POVs and name. HAL getters only (each a
// copy under the HAL's own mutex, the joysticks in one,
// HAL_GetAllJoystickData): no NetComm call and no WPILib warning. The cache
// changes only on a refresh (HAL_RefreshDSData), so called on the thread that
// refreshes, between two of its refreshes, every field comes from the same one.
// `sample_time_us`, `mode_since_us` and `boot_id` are left 0.
void SnapshotDriverStation(DriverStationData& out);

// SnapshotDriverStation stamped with `now_us`: a desktop test's view of the
// packet after its own refresh.
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
// cache and snapshots all of it (`refresh`), and publishes the snapshot with
// the refresh's completion time on the loop's clock. Then, only while the
// loop has called Read within kLoopTickWindowUs, it reports the program's
// mode for the snapshot's flags (`observe`, also a NetComm call), so a
// stalled loop stops the reports as it did when the loop made them.
//
// The loop calls Read once per tick: it records the tick and returns the
// latest published snapshot, or a neutral one (DriverStationData{}: flags 0,
// so no Driver Station, no mode and disabled; every joystick disconnected,
// no axis, button or POV) when none has completed within `stale_us` (the
// gateway's command_timeout_us). A refresh that does not return, a wedged
// daemon, therefore disables the gateway within command_timeout_us plus one
// tick of the last refresh that did, and when refreshes resume the mode comes
// back as a new mode generation (driver_station::ModeTracker sees the flags go
// through 0). Read never waits, locks or calls into WPILib: one atomic load,
// and one atomic exchange when a new snapshot is there.
//
// Consistency. Each snapshot the loop gets is one refresh's, whole: its flags,
// alliance, station, match fields, match time and every joystick come from the
// same HAL_RefreshDSData, because talos-ds is the program's one refresher
// and it snapshots before its next refresh; and the loop only ever reads a
// snapshot the helper has finished and no longer writes (a triple buffer: the
// helper fills its own slot and swaps it in with one atomic exchange, the loop
// swaps out the newest with another). Within that refresh the joysticks are
// one Driver Station packet's (HAL_GetAllJoystickData), the control word the
// same packet's unless NetComm reports the Driver Station detached (then 0),
// and the match information and joystick names the latest the HAL had
// received by that refresh. Successive Reads return snapshots in the order
// they were published, possibly skipping some, never an older one after a
// newer. The packet the loop sends at a tick is the snapshot it read at that
// tick's start, so the packet's flags are the flags that tick's gateway
// enable and ModeTracker used.
//
// The one-refresher rule: in src/main, frc::DriverStation::RefreshData (and
// with it HAL_RefreshDSData) is called only by WpilibCalls' refresh, and
// nothing calls frc::sim::DriverStationSim::NotifyNewData, which refreshes on
// its caller's thread. The desktop simulation's Driver Station
// (SimDriverStation) hands its data over with
// HALSIM_NotifyDriverStationNewData, which only queues it and sets the
// new-data event talos-ds waits on, as the WPILib simulation GUI and
// Driver Station socket plugins do (SimDriverStation_test,
// ScriptedEventsLeaveTheRefreshToTheReader).
//
// Stop (also the destructor) wakes the helper and waits for it to exit for
// at most `stop_bound`. A helper still inside a NetComm call by then is
// abandoned: detached, `report` gets one line, and Stop returns. Everything
// an abandoned helper can still touch (the calls, the clock, the published
// snapshots) is shared with it (std::shared_ptr), never the reader itself.
// The helper checks the stop flag after each call it makes and then
// publishes nothing more and reports no mode; a publication or report it
// had already decided on when Stop set the flag still completes. Start runs
// the helper once; a stopped reader is not restarted.
class DriverStationReader {
 public:
  // What the helper calls. `wait` returns on a new Driver Station packet or
  // after at most kWaitTimeoutUs; `wake` makes the pending or next `wait`
  // return at once (called by Stop, from another thread); `refresh` updates
  // WPILib's cache and fills `out` with all of it (SnapshotDriverStation);
  // `observe` reports the program's mode. `refresh` and `observe` may block.
  struct Calls {
    std::function<void()> wait;
    std::function<void()> wake;
    std::function<void(driver_station::DriverStationData& out)> refresh;
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
  // SnapshotDriverStation, and HAL_ObserveUserProgram*.
  static Calls WpilibCalls();

  // Starts the helper, named talos-ds. `realtime_priority` 1-99 puts it on
  // SCHED_FIFO at that priority (SCHED_RESET_ON_FORK, so a thread it starts
  // does not inherit it) on Linux, and it reports what the kernel holds; 0
  // keeps the starter's scheduling. A helper that cannot be created is
  // reported, and Read then returns the neutral snapshot for good.
  void Start(int realtime_priority);
  void Stop();

  // The loop thread, once per tick at `now_us` (the clock's): see the class
  // comment. The reference stays valid, and unchanged, until the next Read.
  const driver_station::DriverStationData& Read(uint64_t now_us,
                                                uint64_t stale_us);

 private:
  // No Read yet.
  static constexpr uint64_t kNever = std::numeric_limits<uint64_t>::max();
  // The middle slot's index, and the bit that says the helper has put a
  // snapshot there the loop has not taken.
  static constexpr uint8_t kIndexMask = 0x3;
  static constexpr uint8_t kFresh = 0x4;

  struct Snapshot {
    bool refreshed{false};
    uint64_t refreshed_us{0};
    driver_station::DriverStationData data{};
  };

  struct Shared {
    Calls calls;
    Clock clock;
    Report report;
    int realtime_priority{0};
    // The triple buffer. The helper owns one slot (it starts with 0), the
    // loop one (front_, 2), and `middle` names the third, with kFresh while
    // it holds a snapshot the loop has not taken. Each side writes or reads
    // only the slot it owns; the exchanges on `middle` hand slots over.
    std::array<Snapshot, 3> slots{};
    std::atomic<uint8_t> middle{1};
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
  // The loop's slot.
  uint8_t front_{2};
  std::thread thread_;
};

}  // namespace talos::rio

#include "Robot.h"

#include <frc/DriverStation.h>
#include <hal/DriverStation.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <optional>
#include <random>
#include <string_view>
#include <thread>

#include "ConfigStore.h"
#include "DriverStationReader.h"
#include "LoopSchedule.h"
#include "PhoenixBackend.h"
#include "RioBuild.h"
#include "SocketRetry.h"

#ifdef __linux__
#include <time.h>
#endif

#ifdef __FRC_ROBORIO__
#include <sched.h>
#include <unistd.h>
#else
#include <cstdlib>

#include "SimDriverStation.h"
#endif

namespace {

uint64_t SteadyUs(std::chrono::steady_clock::time_point time) {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             time.time_since_epoch())
      .count();
}

// The gateway's clock and the loop's: microseconds of steady_clock.
uint64_t NowUs() { return SteadyUs(std::chrono::steady_clock::now()); }

#ifdef __FRC_ROBORIO__
constexpr const char* kConfigPath = "/home/lvuser/talos_config.bin";
#else
constexpr const char* kConfigPath = "talos_config.bin";
#endif

// RioLog kMessage indices (rate-limit keys) of this file's reports; index 0 is
// PhoenixBackend's configure failure.
constexpr uint16_t kSocketMessage = 1;
constexpr uint16_t kPersistMessage = 2;
constexpr uint16_t kSocketOpenMessage = 3;

void Report(talos::rio::RioLog& log, uint16_t index, std::string_view text) {
  talos::rio::LogEvent event;
  event.index = index;
  event.SetText(text);
  log.Post(event);
}

// Reports the mode this program is running in, which the Driver Station
// displays: the one piece of TimedRobot's job that has to be done. Whether a
// lapse in these reports disables the robot is not established (docs/BENCH.md,
// loop stall check); a stalled loop's outputs are LoopWatchdog's.
void ObserveMode() {
  if (frc::DriverStation::IsDisabled()) {
    HAL_ObserveUserProgramDisabled();
  } else if (frc::DriverStation::IsAutonomous()) {
    HAL_ObserveUserProgramAutonomous();
  } else if (frc::DriverStation::IsTest()) {
    HAL_ObserveUserProgramTest();
  } else {
    HAL_ObserveUserProgramTeleop();
  }
}

uint32_t DriverStationModeFlags() {
  namespace ds = talos::driver_station;
  uint32_t flags = 0;
  if (frc::DriverStation::IsDSAttached()) flags |= ds::kDsAttached;
  if (frc::DriverStation::IsEnabled()) flags |= ds::kEnabled;
  if (frc::DriverStation::IsAutonomous()) flags |= ds::kAutonomous;
  if (frc::DriverStation::IsTeleop()) flags |= ds::kTeleop;
  if (frc::DriverStation::IsTest()) flags |= ds::kTest;
  if (frc::DriverStation::IsEStopped()) flags |= ds::kEStop;
  return flags;
}

// The loop watchdog's SCHED_FIFO priority: above the loop thread's, so it
// preempts a loop that never sleeps, and meant to stay below the kernel's
// threaded interrupt handlers (SCHED_FIFO 50 by default on PREEMPT_RT), which
// may carry the CAN frames its NeutralOut needs; the RoboRIO's actual thread
// priorities are unrecorded (docs/BENCH.md, loop stall check). Desktop
// simulation keeps the default scheduling.
#ifdef __FRC_ROBORIO__
constexpr int kWatchdogPriority = 30;
#else
constexpr int kWatchdogPriority = 0;
#endif

#ifdef __FRC_ROBORIO__
// The loop thread's SCHED_FIFO priority, so ordinary processes (an ssh
// session, top) no longer delay its ticks. Strictly below the watchdog, which
// must still preempt it when it spins.
constexpr int kLoopPriority = 20;
static_assert(kLoopPriority > 0 && kLoopPriority < kWatchdogPriority);

// Present at program start: the loop keeps the default scheduling (a bench
// comparison, or a field fallback that needs no rebuild).
constexpr const char* kLoopDefaultSchedulingFile =
    "/home/lvuser/talos_loop_default_sched";

const char* PolicyName(int policy) {
  switch (policy) {
    case SCHED_OTHER:
      return "SCHED_OTHER";
    case SCHED_FIFO:
      return "SCHED_FIFO";
    case SCHED_RR:
      return "SCHED_RR";
    default:
      return "other policy";
  }
}

// Puts the calling thread, the loop, on SCHED_FIFO kLoopPriority, unless
// kLoopDefaultSchedulingFile exists, and prints its policy and priority, with
// the reason when it stays at the default scheduling. A refusal leaves it
// there; nothing else changes.
//
// SCHED_RESET_ON_FORK: a thread the loop creates later (a vendor library's
// lazily started thread) starts at the default scheduling instead of
// inheriting SCHED_FIFO, where a busy one would starve everything below it.
// Otherwise it is the request frc::SetCurrentThreadPriority makes (with which
// LoopWatchdog::Run raises the watchdog), made directly to pass that flag and
// to print the refusal's errno.
void ScheduleLoopThread() {
  char reason[160] = "";
  if (access(kLoopDefaultSchedulingFile, F_OK) == 0) {
    std::snprintf(reason, sizeof(reason), " (%s present)",
                  kLoopDefaultSchedulingFile);
  } else {
    sched_param param{};
    param.sched_priority = kLoopPriority;
    if (sched_setscheduler(0, SCHED_FIFO | SCHED_RESET_ON_FORK, &param) != 0) {
      std::snprintf(reason, sizeof(reason),
                    " (SCHED_FIFO %d refused, errno %d)", kLoopPriority, errno);
    }
  }
  // What the kernel holds now, whatever was asked.
  const int policy = sched_getscheduler(0);
  sched_param actual{};
  sched_getparam(0, &actual);
  std::printf(
      "%shardware gateway loop %s %d, watchdog asks SCHED_FIFO %d%s\n",
      talos::rio::ConsoleTime(NowUs()).c_str(),
      policy < 0 ? "unknown policy" : PolicyName(policy & ~SCHED_RESET_ON_FORK),
      actual.sched_priority, kWatchdogPriority, reason);
}
#endif

// Sleeps until `wake_us` on NowUs' clock.
void SleepUntilUs(uint64_t wake_us) {
#if defined(__linux__) && defined(_GLIBCXX_USE_CLOCK_MONOTONIC)
  // libstdc++'s steady_clock, NowUs' clock, reads CLOCK_MONOTONIC. An absolute
  // sleep on it wakes at the deadline however long the thread was preempted
  // before sleeping; std::this_thread::sleep_until sleeps a relative time
  // computed before that preemption.
  timespec wake{};
  wake.tv_sec = static_cast<time_t>(wake_us / 1'000'000);
  wake.tv_nsec = static_cast<long>((wake_us % 1'000'000) * 1'000);
  int result = 0;
  do {
    result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, nullptr);
  } while (result == EINTR);
  if (result == 0) return;
  // Not reachable with a valid time; never let the loop spin.
#elif defined(__FRC_ROBORIO__)
#error "the RoboRIO loop needs libstdc++'s steady_clock on CLOCK_MONOTONIC"
#endif
  std::this_thread::sleep_until(std::chrono::steady_clock::time_point{
      std::chrono::microseconds{wake_us}});
}

class RioWatchdogOutputs final : public talos::rio::WatchdogOutputs {
 public:
  explicit RioWatchdogOutputs(PhoenixBackend& backend) : backend_{backend} {}
  bool Neutralize() override { return backend_.WatchdogNeutral(); }
  void Report(std::string_view text) override {
    char details[256];
    std::snprintf(details, sizeof(details), "%.*s",
                  static_cast<int>(text.size()), text.data());
    // printMsg also copies it to the console (the RoboRIO's netconsole log).
    HAL_SendError(1, 1, 0, details, "talos loop watchdog", "", 1);
  }

 private:
  PhoenixBackend& backend_;
};

}  // namespace

void Robot::StartCompetition() {
  log_ = std::make_unique<talos::rio::RioLog>();
  log_->Start();
  auto phoenix = std::make_unique<PhoenixBackend>(IsSimulation(), log_.get());
  watchdog_outputs_ = std::make_unique<RioWatchdogOutputs>(*phoenix);
  watchdog_ = std::make_unique<talos::rio::LoopWatchdog>(*watchdog_outputs_);
  backend_ = std::move(phoenix);

  // A fresh boot id every start, so a command built against a previous boot's
  // state can never be accepted after a code restart.
  std::random_device random;
  const uint64_t boot_id = (uint64_t{random()} << 32) | random() | 1;

  gateway_ = std::make_unique<talos::hardware::Gateway>(*backend_, boot_id);
  gateway_->SetClock(NowUs);
  // Every State names this build, so a session record shows which build
  // drove the robot (RioBuild.h, generated by build.gradle).
  gateway_->SetBuild({
      .revision = talos::rio::kBuildRevision,
      .program = IsSimulation()
                     ? talos::hardware::GatewayProgram::kRoboRioSimulation
                     : talos::hardware::GatewayProgram::kRoboRio,
      .dirty = talos::rio::kBuildDirty,
  });
  // The main processor this program sends State to and, its socket being
  // connected, the only host it accepts frames from: fixed at build time
  // (-Pcompanion, build.gradle), never learned from a datagram.
  const char* const companion_address =
      IsSimulation() ? "127.0.0.1" : talos::rio::kCompanionAddress;
  // Prefixed, like every RioLog line, with the gateway's clock: on the
  // RoboRIO, seconds since its kernel booted.
  std::printf("%shardware gateway build %s%s companion %s:5803\n",
              talos::rio::ConsoleTime(NowUs()).c_str(),
              talos::rio::kBuildCommit,
              talos::rio::kBuildDirty ? " (dirty)" : "", companion_address);
  // The persisted configuration configures devices on the backend's worker,
  // polled by the loop below: a reboot with absent devices (each one a CAN
  // timeout) must not hold back robot code start or this loop's reports of
  // its mode. Until it completes the gateway is unconfigured and drives
  // nothing.
  const auto restoring = talos::rio::BeginRestore(*gateway_, kConfigPath);
  const uint64_t restoring_id =
      restoring ? talos::hardware::ConfigurationId(*restoring) : 0;
  // An acknowledged configuration not yet on disk. The save syncs storage on
  // this loop thread, so it waits for a disabled tick: an acknowledgement can
  // come while enabled (a configure finishing after enable, or an identical
  // push after a failed save), and nothing may stall the loop while devices
  // hold their last demand.
  std::optional<talos::hardware::Config> unsaved;
  uint64_t saved_id = restoring_id;
  endpoint_ = std::make_unique<talos::hardware::Endpoint>(*gateway_);
  companion_sink_ = talos::rio::MakeDashboardSink();
  companion_ =
      std::make_unique<talos::rio::CompanionReporter>(*companion_sink_);
  companion_->Start();
  endpoint_->OnConfigured(
      [&unsaved, &saved_id](const talos::hardware::Config& config) {
        if (talos::hardware::ConfigurationId(config) != saved_id)
          unsaved = config;
      });

  // Opened from the loop, retried every SocketRetry::kPeriodUs until the
  // network has a route to the companion (after a RoboRIO boot, the program
  // can start before it does).
  talos::rio::SocketRetry socket_retry;

  // The configuration being restored, if any; the push still follows.
  std::printf(
      "%shardware gateway config=%llu commissioned=%d (waiting for config "
      "push)\n",
      talos::rio::ConsoleTime(NowUs()).c_str(),
      static_cast<unsigned long long>(restoring_id),
      restoring ? restoring->commissioned : 0);

  // Tells the Driver Station that robot code is up. Nothing before this waits
  // on CAN: device configuration runs on the backend's worker.
  HAL_ObserveUserProgramStarting();

  // Disarmed until a tick ends with a command in force.
  watchdog_->Start(NowUs, kWatchdogPriority);

#ifdef __FRC_ROBORIO__
  // After every other thread of this program has started, at the default
  // scheduling: none inherits the loop's.
  ScheduleLoopThread();
#endif

  uint64_t period_us = 5000;
  const auto loop_start = std::chrono::steady_clock::now();
  // The next tick's deadline, on NowUs' clock.
  uint64_t next_us = SteadyUs(loop_start);

#ifndef __FRC_ROBORIO__
  talos::rio::SimDriverStation simulated_driver_station{
      talos::rio::SimDriverStation::SettingsFromEnvironment(), loop_start};

  // A bounded simulation run, so an automated test does not have to race a
  // kill signal to get a clean summary out.
  std::optional<std::chrono::steady_clock::time_point> stop_at;
  if (const char* ms = std::getenv("TALOS_SIM_RUN_MS")) {
    stop_at = loop_start + std::chrono::milliseconds{std::atoi(ms)};
  }
  uint64_t active_ticks = 0;
#endif

  std::array<uint8_t, talos::protocol::kMaxPayloadSize> ds_buffer{};
  uint64_t last_ds_sample_us = 0;
  // The mode (ModeKey) of the last packet sent.
  uint32_t last_ds_mode = 0;
  // Observe every hardware tick, including transitions between DS packets.
  // Real and desktop Driver Stations are both read back through WPILib.
  talos::driver_station::ModeTracker ds_mode;

  // Tick starts, so the logger can report a loop woken late (starved of the
  // CPU) even when each tick's own work is short.
  std::optional<uint64_t> previous_tick_start_us;
  while (running_.load(std::memory_order_relaxed)) {
    const uint64_t tick_start_us = NowUs();
    const uint64_t tick_interval_us =
        previous_tick_start_us ? tick_start_us - *previous_tick_start_us : 0;
    previous_tick_start_us = tick_start_us;
    if (gateway_->configured()) {
      period_us = gateway_->config().period_us;
    }
    next_us += period_us;

#ifndef __FRC_ROBORIO__
    simulated_driver_station.Update(std::chrono::steady_clock::now(),
                                    gateway_->configured());
    if (stop_at && std::chrono::steady_clock::now() >= *stop_at) {
      break;
    }
    active_ticks +=
        (gateway_->snapshot().flags & talos::hardware::kCommandActive) != 0;
#endif

    frc::DriverStation::RefreshData();
    ObserveMode();

    const uint64_t now_us = NowUs();
    const uint32_t mode_flags = DriverStationModeFlags();
    ds_mode.Observe(mode_flags, now_us);
    auto open_status = talos::protocol::UdpStatus::kOk;
    int open_errno = 0;
    const auto socket_event = socket_retry.Poll(now_us, [&] {
      // The failing call's errno (Open's close of the half-made socket
      // leaves it): ENETUNREACH while the network has no route yet.
      errno = 0;
      open_status = endpoint_->Open(IsSimulation() ? "127.0.0.1" : "0.0.0.0",
                                    5802, companion_address, 5803);
      open_errno = errno;
      return open_status == talos::protocol::UdpStatus::kOk;
    });
    if (socket_event == talos::rio::SocketRetry::Event::kFailed) {
      // RioLog coalesces these to one line a second with their count. The
      // errno is printed as a number (101 is ENETUNREACH): strerror may take
      // a locale lock on this thread.
      char text[128];
      std::snprintf(text, sizeof(text),
                    "hardware gateway socket failed: %s (errno %d)",
                    talos::protocol::UdpStatusName(open_status), open_errno);
      Report(*log_, kSocketMessage, text);
    } else if (socket_event == talos::rio::SocketRetry::Event::kOpened &&
               socket_retry.failures() > 0) {
      char text[128];
      std::snprintf(
          text, sizeof(text),
          "hardware gateway socket open after %u failed attempts, %.3f s "
          "after the first",
          static_cast<unsigned>(socket_retry.failures()),
          static_cast<double>(now_us - socket_retry.first_failure_us()) / 1e6);
      Report(*log_, kSocketOpenMessage, text);
    }
    if (socket_retry.open()) {
      // Endpoint handles clock probes even disabled/uncommissioned, echoing
      // only on a subsequent tick so its sample stamp follows probe receipt.
      endpoint_->Tick(now_us,
                      (mode_flags & talos::driver_station::kEnabled) != 0 &&
                          (mode_flags & talos::driver_station::kEStop) == 0);

      // The Driver Station only produces new data at 50 Hz, so sampling it on
      // every hardware tick would put three duplicate packets on the link, and
      // in the dispatch log downstream, for every real one. A new mode goes
      // out on the tick that observes it: an enabled switch (autonomous to
      // teleop) leaves the State unchanged, so until its generation arrives
      // the old mode's target lanes still drive.
      const uint32_t mode = talos::driver_station::ModeKey(mode_flags);
      const bool sample_due =
          now_us - last_ds_sample_us >= talos::driver_station::kSamplePeriodUs;
      if (mode != last_ds_mode || sample_due) {
        last_ds_sample_us = now_us;
        last_ds_mode = mode;
        auto ds = talos::driver_station::SampleDriverStation(now_us);
        ds_mode.Stamp(ds);
        // The same boot id as this program's State: a consumer orders
        // packets within one boot and starts over on a new one.
        ds.boot_id = boot_id;
        // Encode() returns 0 if the payload will not fit; send nothing rather
        // than an empty frame the receiver would forward as a decode failure.
        if (const auto ds_size = talos::driver_station::Encode(ds, ds_buffer)) {
          endpoint_->SendFrame(talos::protocol::FrameType::kDriverStation,
                               now_us, ds_buffer.data(), ds_size);
        }
      }
    } else {
      gateway_->Tick(now_us, false);
    }

    // Arms the watchdog only while a command is in force (a tick without one
    // neutralized every output itself), with the time by which this loop's
    // next tick would have invalidated it and beaten again: the gateway's
    // expiry, plus one period for the tick that sees it and one more for that
    // tick's own work before its beat, so an on-time loop is never acted on.
    const uint64_t expiry_us = gateway_->command_expiry_us();
    watchdog_->Beat(now_us, expiry_us == 0 ? 0 : expiry_us + 2 * period_us);

    // What the endpoint last heard, for the drive team's link report; atomic
    // stores only, the reporter thread does the publishing.
    {
      const auto& contact = endpoint_->peer_contact();
      talos::rio::CompanionContact observed;
      observed.frame_us = contact.frame_us;
      observed.command_us = contact.command_us;
      observed.version_mismatch_us = contact.version_mismatch_us;
      observed.mismatched_version = contact.mismatched_version;
      observed.configured = gateway_->configured();
      observed.commissioned = gateway_->config().commissioned;
      observed.config_id = gateway_->snapshot().config_id;
      companion_->Observe(observed);
    }

    if (unsaved && !(gateway_->snapshot().flags & talos::hardware::kEnabled)) {
      if (talos::rio::SaveConfigFile(kConfigPath, *unsaved))
        saved_id = talos::hardware::ConfigurationId(*unsaved);
      else
        Report(*log_, kPersistMessage, "cannot persist hardware configuration");
      unsaved.reset();
    }

    const uint64_t end_us = NowUs();
    // Start-to-start interval and wake-to-sleep work against the period,
    // which the logger thread reports once a second as late ticks and
    // overruns.
    log_->RecordTick(tick_interval_us, end_us - tick_start_us, period_us);
    // On time, the loop keeps its phase; past the deadline it drops the
    // missed ticks instead of catching up in a burst, yields the CPU for
    // kOverrunSleepUs and counts the next period from there, so the phase
    // moves (LoopSchedule.h).
    next_us = talos::rio::NextWakeUs(next_us, end_us);
    SleepUntilUs(next_us);
  }

  // The loop no longer beats; the shutdown below neutralizes on its own.
  watchdog_->Stop();

  // A configure still running on the backend's worker (the boot restore or a
  // push) owns the devices; neutralizing them now would race it. Poll it to
  // completion disabled, then neutralize what it built.
  while (gateway_->configuring()) {
    gateway_->Tick(NowUs(), false);
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
  backend_->Neutral();

#ifndef __FRC_ROBORIO__
  // The same summary line the standalone sim_gateway prints, so the drivetrain
  // integration test can make the identical assertions about either gateway.
  const auto final_state = gateway_->snapshot();
  std::printf(
      "sim_robot: active_ticks=%llu first_wheel_rotations=%.6f "
      "first_wheel_rps=%.6f command_active=%d\n",
      static_cast<unsigned long long>(active_ticks),
      final_state.motors[0].position_rot, final_state.motors[0].velocity_rps,
      (final_state.flags & talos::hardware::kCommandActive) != 0);
  std::fflush(stdout);
#endif

  // WPILib keeps the robot in a function-local static, so anything still held
  // here would be destroyed during static destruction at exit, racing the
  // Phoenix and HAL statics it depends on ("mutex lock failed" aborts). Release
  // the hardware while those libraries are still alive.
  companion_.reset();
  companion_sink_.reset();
  watchdog_.reset();
  watchdog_outputs_.reset();
  endpoint_.reset();
  gateway_.reset();
  backend_.reset();
  log_.reset();
}

void Robot::EndCompetition() {
  running_.store(false, std::memory_order_relaxed);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

#include "DriverStationReader.h"

#include <frc/DriverStation.h>
#include <hal/DriverStation.h>
#include <wpi/Synchronization.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <string_view>
#include <system_error>
#include <utility>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#elif defined(__APPLE__)
#include <pthread.h>
#endif

namespace talos::driver_station {

// The wire enum is cast straight from WPILib's below, so a reordering on
// either side has to fail the build rather than mislabel a match.
static_assert(static_cast<int>(MatchType::kNone) ==
              static_cast<int>(frc::DriverStation::MatchType::kNone));
static_assert(static_cast<int>(MatchType::kPractice) ==
              static_cast<int>(frc::DriverStation::MatchType::kPractice));
static_assert(static_cast<int>(MatchType::kQualification) ==
              static_cast<int>(frc::DriverStation::MatchType::kQualification));
static_assert(static_cast<int>(MatchType::kElimination) ==
              static_cast<int>(frc::DriverStation::MatchType::kElimination));

namespace {

template <std::size_t N>
void CopyString(std::string_view src, char (&dst)[N]) {
  static_assert(N > 0);
  const std::size_t count = std::min(src.size(), N - 1);
  std::copy_n(src.data(), count, dst);
  dst[count] = '\0';
}

// A control word as DsFlags, as WPILib's IsDSAttached, IsFMSAttached,
// IsEnabled (attached and enabled), IsAutonomous, IsTeleop (neither
// autonomous nor test), IsTest and IsEStopped read it: one copy of the
// cache, so the flags all come from one refresh.
uint32_t FlagsOf(const HAL_ControlWord& word) {
  uint32_t flags = 0;
  if (word.dsAttached) flags |= kDsAttached;
  if (word.fmsAttached) flags |= kFmsAttached;
  if (word.enabled && word.dsAttached) flags |= kEnabled;
  if (word.autonomous) flags |= kAutonomous;
  if (!(word.autonomous || word.test)) flags |= kTeleop;
  if (word.test) flags |= kTest;
  if (word.eStop) flags |= kEStop;
  return flags;
}

uint32_t CachedFlags() {
  HAL_ControlWord word{};
  HAL_GetControlWord(&word);
  return FlagsOf(word);
}

}  // namespace

DriverStationData SampleDriverStation(uint64_t now_us) {
  return SampleDriverStation(now_us, CachedFlags());
}

DriverStationData SampleDriverStation(uint64_t now_us, uint32_t flags) {
  DriverStationData ds{};
  ds.sample_time_us = now_us;
  ds.fms.flags = flags;

  // Alliance & Location
  const auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value()) {
    ds.fms.alliance = (*alliance == frc::DriverStation::Alliance::kRed)
                          ? Alliance::kRed
                          : Alliance::kBlue;
  }
  const auto location = frc::DriverStation::GetLocation();
  if (location.has_value()) {
    ds.fms.station = static_cast<uint8_t>(*location);
  }

  // Match info
  ds.fms.match_type =
      static_cast<MatchType>(frc::DriverStation::GetMatchType());
  ds.fms.match_number =
      static_cast<uint16_t>(frc::DriverStation::GetMatchNumber());
  ds.fms.replay_number =
      static_cast<uint8_t>(frc::DriverStation::GetReplayNumber());
  ds.fms.match_time_s =
      static_cast<float>(frc::DriverStation::GetMatchTime().value());
  CopyString(frc::DriverStation::GetGameSpecificMessage(),
             ds.fms.game_specific_message);
  CopyString(frc::DriverStation::GetEventName(), ds.fms.event_name);

  // Joysticks
  for (std::size_t i = 0; i < kMaxJoysticks; ++i) {
    auto& j = ds.joysticks[i];
    j.connected = frc::DriverStation::IsJoystickConnected(static_cast<int>(i));
    if (!j.connected) continue;

    j.axis_count = static_cast<uint8_t>(std::min<int>(
        frc::DriverStation::GetStickAxisCount(static_cast<int>(i)),
        static_cast<int>(kMaxAxes)));
    for (std::size_t a = 0; a < j.axis_count; ++a) {
      j.axes[a] = static_cast<float>(frc::DriverStation::GetStickAxis(
          static_cast<int>(i), static_cast<int>(a)));
    }

    j.button_count = static_cast<uint8_t>(std::min<int>(
        frc::DriverStation::GetStickButtonCount(static_cast<int>(i)), 32));
    j.buttons = static_cast<uint32_t>(
        frc::DriverStation::GetStickButtons(static_cast<int>(i)));

    j.pov_count = static_cast<uint8_t>(
        std::min<int>(frc::DriverStation::GetStickPOVCount(static_cast<int>(i)),
                      static_cast<int>(kMaxPovs)));
    for (std::size_t p = 0; p < j.pov_count; ++p) {
      j.povs[p] = static_cast<int16_t>(frc::DriverStation::GetStickPOV(
          static_cast<int>(i), static_cast<int>(p)));
    }

    CopyString(frc::DriverStation::GetJoystickName(static_cast<int>(i)),
               j.name);
  }

  return ds;
}

}  // namespace talos::driver_station

namespace talos::rio {

namespace ds = driver_station;

static_assert((ds::kDsAttached | ds::kFmsAttached | ds::kEnabled |
               ds::kAutonomous | ds::kTeleop | ds::kTest | ds::kEStop) <
                  (1u << 8),
              "every DsFlags bit must fit below a refresh's time");

DriverStationReader::DriverStationReader(Calls calls, Clock clock,
                                         Report report,
                                         std::chrono::microseconds stop_bound)
    : shared_{std::make_shared<Shared>()}, stop_bound_{stop_bound} {
  shared_->calls = std::move(calls);
  shared_->clock = std::move(clock);
  shared_->report = std::move(report);
}

DriverStationReader::~DriverStationReader() { Stop(); }

DriverStationReader::Calls DriverStationReader::WpilibCalls() {
  // WPILib's new-data event, which the HAL sets from NetComm's own thread on
  // each Driver Station packet (its udpOccur), without a refresh of ours;
  // frc::DriverStation's refreshed-data event would fire only after one.
  // Registered for as long as the helper holds these calls, an abandoned
  // one included.
  struct NewData {
    wpi::Event event;  // auto-reset
    NewData() { HAL_ProvideNewDataEventHandle(event.GetHandle()); }
    ~NewData() { HAL_RemoveNewDataEventHandle(event.GetHandle()); }
    NewData(const NewData&) = delete;
    NewData& operator=(const NewData&) = delete;
  };
  auto new_data = std::make_shared<NewData>();
  Calls calls;
  calls.wait = [new_data] {
    bool timed_out = false;
    wpi::WaitForObject(new_data->event.GetHandle(),
                       static_cast<double>(kWaitTimeoutUs) / 1e6, &timed_out);
  };
  calls.wake = [new_data] { new_data->event.Set(); };
  calls.refresh = [] {
    frc::DriverStation::RefreshData();
    return ds::CachedFlags();
  };
  calls.observe = [](ProgramMode mode) {
    switch (mode) {
      case ProgramMode::kDisabled:
        HAL_ObserveUserProgramDisabled();
        return;
      case ProgramMode::kAutonomous:
        HAL_ObserveUserProgramAutonomous();
        return;
      case ProgramMode::kTeleop:
        HAL_ObserveUserProgramTeleop();
        return;
      case ProgramMode::kTest:
        HAL_ObserveUserProgramTest();
        return;
    }
  };
  return calls;
}

void DriverStationReader::Start(int realtime_priority) {
  if (thread_.joinable() || shared_->stop.load(std::memory_order_acquire)) {
    return;
  }
  shared_->realtime_priority = realtime_priority;
  try {
    thread_ = std::thread{[shared = shared_] { Run(shared); }};
  } catch (const std::system_error& error) {
    if (shared_->report) {
      char line[160];
      std::snprintf(line, sizeof line,
                    "driver station reader not started (%s): the Driver "
                    "Station reads as no mode, so the gateway stays disabled",
                    error.what());
      shared_->report(line);
    }
  }
}

void DriverStationReader::Stop() {
  if (!thread_.joinable()) return;
  shared_->stop.store(true, std::memory_order_release);
  if (shared_->calls.wake) shared_->calls.wake();
  bool exited = false;
  {
    std::unique_lock lock{shared_->mutex};
    exited = shared_->exit.wait_for(lock, stop_bound_,
                                    [&] { return shared_->exited; });
  }
  if (exited) {
    thread_.join();
    return;
  }
  // Inside a NetComm call that has not returned: it owns what it touches.
  thread_.detach();
  if (shared_->report) {
    char line[160];
    std::snprintf(
        line, sizeof line,
        "driver station reader did not stop within %llu ms (a Driver Station "
        "call that did not return); it was abandoned",
        static_cast<unsigned long long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(stop_bound_)
                .count()));
    shared_->report(line);
  }
}

uint32_t DriverStationReader::Read(uint64_t now_us, uint64_t stale_us) {
  shared_->tick_us.store(now_us, std::memory_order_release);
  const uint64_t refreshed = shared_->refreshed.load(std::memory_order_acquire);
  if (refreshed == kNever) return 0;
  const uint64_t at_us = refreshed >> kFlagBits;
  // A refresh completed after this tick's clock reading is current; one
  // command_timeout_us old is stale, as the gateway's own command expiry is.
  if (at_us < now_us && now_us - at_us >= stale_us) return 0;
  return static_cast<uint32_t>(refreshed & ((uint64_t{1} << kFlagBits) - 1));
}

void DriverStationReader::Run(const std::shared_ptr<Shared>& shared) {
#ifdef __linux__
  // So a thread listing (ps -L) on the RoboRIO tells it from the loop.
  pthread_setname_np(pthread_self(), "talos-ds");
  if (shared->realtime_priority > 0) {
    sched_param param{};
    param.sched_priority = shared->realtime_priority;
    const int refused =
        sched_setscheduler(0, SCHED_FIFO | SCHED_RESET_ON_FORK, &param) != 0
            ? errno
            : 0;
    // What the kernel holds, whatever was asked.
    const int policy = sched_getscheduler(0) & ~SCHED_RESET_ON_FORK;
    sched_param held{};
    sched_getparam(0, &held);
    if (shared->report) {
      char line[160];
      if (refused != 0) {
        std::snprintf(line, sizeof line,
                      "driver station reader %s %d (SCHED_FIFO %d refused, "
                      "errno %d)",
                      policy == SCHED_FIFO ? "SCHED_FIFO" : "not realtime",
                      held.sched_priority, shared->realtime_priority, refused);
      } else {
        std::snprintf(line, sizeof line, "driver station reader %s %d",
                      policy == SCHED_FIFO ? "SCHED_FIFO" : "not realtime",
                      held.sched_priority);
      }
      shared->report(line);
    }
  }
#elif defined(__APPLE__)
  pthread_setname_np("talos-ds");
#endif
  const auto stopping = [&shared] {
    return shared->stop.load(std::memory_order_acquire);
  };
  while (!stopping()) {
    shared->calls.wait();
    if (stopping()) break;
    const uint32_t flags = shared->calls.refresh();
    // A time that would not fit below the flags is held at the largest that
    // does (2^56 us, two thousand years of uptime).
    const uint64_t done_us =
        std::min(shared->clock(), (kNever >> kFlagBits) - 1);
    if (stopping()) break;
    shared->refreshed.store((done_us << kFlagBits) | flags,
                            std::memory_order_release);
    const uint64_t tick_us = shared->tick_us.load(std::memory_order_acquire);
    const bool loop_ticking =
        tick_us != kNever &&
        (tick_us >= done_us || done_us - tick_us <= kLoopTickWindowUs);
    if (loop_ticking) shared->calls.observe(ProgramModeOf(flags));
  }
  {
    const std::lock_guard lock{shared->mutex};
    shared->exited = true;
  }
  shared->exit.notify_all();
}

}  // namespace talos::rio

#include "DriverStationReader.h"

#include <frc/DriverStation.h>
#include <hal/DriverStation.h>
#include <wpi/Synchronization.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
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

// The wire enum is cast straight from the HAL's below, so a reordering on
// either side has to fail the build rather than mislabel a match.
static_assert(static_cast<int>(MatchType::kNone) == HAL_kMatchType_none);
static_assert(static_cast<int>(MatchType::kPractice) ==
              HAL_kMatchType_practice);
static_assert(static_cast<int>(MatchType::kQualification) ==
              HAL_kMatchType_qualification);
static_assert(static_cast<int>(MatchType::kElimination) ==
              HAL_kMatchType_elimination);
static_assert(kMaxJoysticks == HAL_kMaxJoysticks);
static_assert(kMaxAxes <= HAL_kMaxJoystickAxes);
static_assert(kMaxPovs <= HAL_kMaxJoystickPOVs);

namespace {

template <std::size_t N>
void CopyString(std::string_view src, char (&dst)[N]) {
  static_assert(N > 0);
  const std::size_t count = std::min(src.size(), N - 1);
  std::copy_n(src.data(), count, dst);
  dst[count] = '\0';
}

// A HAL string of at most N bytes, ending at its first NUL.
template <std::size_t N>
std::string_view Terminated(const char (&src)[N]) {
  return {src, strnlen(src, N)};
}

// A control word as DsFlags, as WPILib's IsDSAttached, IsFMSAttached,
// IsEnabled (attached and enabled), IsAutonomous, IsTeleop (neither
// autonomous nor test), IsTest and IsEStopped read it.
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

}  // namespace

// Each field as frc::DriverStation's getter of it reads the HAL (GetAlliance,
// GetLocation, GetMatchType ... GetMatchTime, IsJoystickConnected,
// GetStickAxis, GetStickButtons, GetStickPOV, GetJoystickName), without their
// range warnings, which are HAL_SendError calls into NetComm.
void SnapshotDriverStation(DriverStationData& out) {
  out = DriverStationData{};

  HAL_ControlWord word{};
  HAL_GetControlWord(&word);
  out.fms.flags = FlagsOf(word);

  int32_t status = 0;
  switch (HAL_GetAllianceStation(&status)) {
    case HAL_AllianceStationID_kRed1:
      out.fms.alliance = Alliance::kRed;
      out.fms.station = 1;
      break;
    case HAL_AllianceStationID_kRed2:
      out.fms.alliance = Alliance::kRed;
      out.fms.station = 2;
      break;
    case HAL_AllianceStationID_kRed3:
      out.fms.alliance = Alliance::kRed;
      out.fms.station = 3;
      break;
    case HAL_AllianceStationID_kBlue1:
      out.fms.alliance = Alliance::kBlue;
      out.fms.station = 1;
      break;
    case HAL_AllianceStationID_kBlue2:
      out.fms.alliance = Alliance::kBlue;
      out.fms.station = 2;
      break;
    case HAL_AllianceStationID_kBlue3:
      out.fms.alliance = Alliance::kBlue;
      out.fms.station = 3;
      break;
    default:
      break;
  }

  HAL_MatchInfo info{};
  HAL_GetMatchInfo(&info);
  out.fms.match_type = static_cast<MatchType>(info.matchType);
  out.fms.match_number = info.matchNumber;
  out.fms.replay_number = info.replayNumber;
  status = 0;
  out.fms.match_time_s = static_cast<float>(HAL_GetMatchTime(&status));
  CopyString(
      std::string_view{reinterpret_cast<const char*>(info.gameSpecificMessage),
                       std::min<std::size_t>(info.gameSpecificMessageSize,
                                             sizeof info.gameSpecificMessage)},
      out.fms.game_specific_message);
  CopyString(Terminated(info.eventName), out.fms.event_name);

  // One copy of every joystick, under one HAL lock.
  std::array<HAL_JoystickAxes, HAL_kMaxJoysticks> axes{};
  std::array<HAL_JoystickPOVs, HAL_kMaxJoysticks> povs{};
  std::array<HAL_JoystickButtons, HAL_kMaxJoysticks> buttons{};
  HAL_GetAllJoystickData(axes.data(), povs.data(), buttons.data());
  for (std::size_t i = 0; i < kMaxJoysticks; ++i) {
    auto& j = out.joysticks[i];
    j.connected =
        axes[i].count > 0 || buttons[i].count > 0 || povs[i].count > 0;
    if (!j.connected) continue;
    j.axis_count = static_cast<uint8_t>(
        std::clamp<int>(axes[i].count, 0, static_cast<int>(kMaxAxes)));
    for (std::size_t a = 0; a < j.axis_count; ++a) {
      j.axes[a] = axes[i].axes[a];
    }
    j.button_count = static_cast<uint8_t>(
        std::min<int>(buttons[i].count, static_cast<int>(kMaxButtons)));
    j.buttons = buttons[i].buttons;
    j.pov_count = static_cast<uint8_t>(
        std::clamp<int>(povs[i].count, 0, static_cast<int>(kMaxPovs)));
    for (std::size_t p = 0; p < j.pov_count; ++p) {
      j.povs[p] = povs[i].povs[p];
    }
    HAL_JoystickDescriptor descriptor{};
    HAL_GetJoystickDescriptor(static_cast<int32_t>(i), &descriptor);
    CopyString(Terminated(descriptor.name), j.name);
  }
}

DriverStationData SampleDriverStation(uint64_t now_us) {
  DriverStationData ds;
  SnapshotDriverStation(ds);
  ds.sample_time_us = now_us;
  return ds;
}

}  // namespace talos::driver_station

namespace talos::rio {

namespace ds = driver_station;

namespace {

// What the loop reads with no refresh within its bound: no Driver Station.
const ds::DriverStationData kNoDriverStation{};

}  // namespace

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
  calls.refresh = [](ds::DriverStationData& out) {
    frc::DriverStation::RefreshData();
    ds::SnapshotDriverStation(out);
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

const ds::DriverStationData& DriverStationReader::Read(uint64_t now_us,
                                                       uint64_t stale_us) {
  shared_->tick_us.store(now_us, std::memory_order_release);
  // Takes the newest snapshot, if the helper has published one since the
  // last Read, handing the slot read until now back to the helper.
  if ((shared_->middle.load(std::memory_order_acquire) & kFresh) != 0) {
    front_ = shared_->middle.exchange(front_, std::memory_order_acq_rel) &
             kIndexMask;
  }
  const Snapshot& snapshot = shared_->slots[front_];
  if (!snapshot.refreshed) return kNoDriverStation;
  // A refresh completed after this tick's clock reading is current; one
  // command_timeout_us old is stale, as the gateway's own command expiry is.
  if (snapshot.refreshed_us < now_us &&
      now_us - snapshot.refreshed_us >= stale_us) {
    return kNoDriverStation;
  }
  return snapshot.data;
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
  // The helper's slot: the loop never reads it until it is swapped out.
  uint8_t back = 0;
  while (!stopping()) {
    shared->calls.wait();
    if (stopping()) break;
    Snapshot& slot = shared->slots[back];
    shared->calls.refresh(slot.data);
    const uint64_t done_us = shared->clock();
    const uint32_t flags = slot.data.fms.flags;
    slot.refreshed_us = done_us;
    slot.refreshed = true;
    if (stopping()) break;
    // Publishes the slot whole and takes back the one the loop last
    // returned, or the unread one this replaces. The slot is the loop's to
    // read from here on.
    back = shared->middle.exchange(static_cast<uint8_t>(back | kFresh),
                                   std::memory_order_acq_rel) &
           kIndexMask;
    const uint64_t tick_us = shared->tick_us.load(std::memory_order_acquire);
    const bool loop_ticking =
        tick_us != kNever &&
        (tick_us >= done_us || done_us - tick_us <= kLoopTickWindowUs);
    if (stopping()) break;
    if (loop_ticking) shared->calls.observe(ProgramModeOf(flags));
  }
  {
    const std::lock_guard lock{shared->mutex};
    shared->exited = true;
  }
  shared->exit.notify_all();
}

}  // namespace talos::rio

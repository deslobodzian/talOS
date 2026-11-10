#include "Robot.h"

#include <frc/DriverStation.h>
#include <hal/DriverStation.h>

#include <array>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "DriverStationReader.h"
#include "PhoenixBackend.h"

#ifndef __FRC_ROBORIO__
#include <frc/simulation/DriverStationSim.h>

#include <cstdlib>
#include <optional>
#include <string_view>
#endif

namespace {

uint64_t NowUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// Set to the companion's static address before deploying to the robot.
constexpr const char* kCompanionAddress = "10.56.87.11";
#ifdef __FRC_ROBORIO__
constexpr const char* kConfigPath = "/home/lvuser/talos_config.bin";
#else
constexpr const char* kConfigPath = "talos_config.bin";
#endif

// The Driver Station's watchdog: it must hear the current mode every loop, or
// it decides robot code has hung and disables the robot. This is the one piece
// of TimedRobot's job that genuinely has to be done.
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

#ifndef __FRC_ROBORIO__
// An automated stand-in for the Driver Station, for tests that run this program
// headless. It is off unless TALOS_SIM_DS is set, so an interactive
// `./gradlew simulateNative` behaves like any other WPILib robot: it starts
// disabled and you enable it and pick Teleoperated in the simulation GUI.
//
//   TALOS_SIM_DS=1                        enable in teleop at startup
//   TALOS_SIM_DS=0                        attach a Driver Station, stay
//   disabled TALOS_SIM_DS=auto                     stay disabled until the
//   gateway is
//                                         configured, then enable in teleop
//   TALOS_SIM_DS_DISABLE_AFTER_MS=2000    ... then disable two seconds in
//   TALOS_SIM_RUN_MS=10000                stop after ten seconds and report
//
// This exists only in the desktop build. On the RoboRIO the enable state comes
// from the real Driver Station and nothing here can forge it.
class SimDriverStation {
 public:
  explicit SimDriverStation(const talos::hardware::Gateway& gateway)
      : gateway_(gateway) {
    const char* requested = std::getenv("TALOS_SIM_DS");
    if (requested == nullptr) {
      return;  // Leave the Driver Station to the GUI or to a real one.
    }

    active_ = true;
    const std::string_view mode{requested};
    if (mode == "auto") {
      // Without a persisted configuration the first push still requires
      // disabled operation. A restored configuration can start enabled.
      enabled_ = false;
      enable_when_configured_ = true;
    } else {
      enabled_ = mode != "0";
    }

    if (const char* after = std::getenv("TALOS_SIM_DS_DISABLE_AFTER_MS")) {
      disable_after_ = std::chrono::milliseconds{std::atoi(after)};
    }

    // Teleoperated: neither autonomous nor test. The gateway does not care
    // which mode it is, only whether the robot is enabled, but the Driver
    // Station watchdog and any future mode-dependent code do.
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetJoystickAxisCount(0, 4);
    frc::sim::DriverStationSim::SetJoystickButtonCount(0, 10);
    frc::sim::DriverStationSim::SetJoystickPOVCount(0, 1);
    Publish();
  }

  void Update() {
    if (!active_) {
      return;
    }
    if (enable_when_configured_ && !enabled_ && gateway_.configured()) {
      std::printf("simulated Driver Station: enabling (gateway configured)\n");
      enabled_ = true;
      Publish();
    }
    if (!enabled_ || !disable_after_) {
      return;
    }
    if (std::chrono::steady_clock::now() - started_ < *disable_after_) {
      return;
    }
    std::printf("simulated Driver Station: disabling\n");
    enabled_ = false;
    Publish();
  }

 private:
  void Publish() const {
    frc::sim::DriverStationSim::SetEnabled(enabled_);
    frc::sim::DriverStationSim::NotifyNewData();
  }

  const talos::hardware::Gateway& gateway_;
  bool active_{false};
  bool enabled_{false};
  bool enable_when_configured_{false};
  std::optional<std::chrono::milliseconds> disable_after_;
  std::chrono::steady_clock::time_point started_{
      std::chrono::steady_clock::now()};
};
#endif

}  // namespace

void Robot::StartCompetition() {
  backend_ = std::make_unique<PhoenixBackend>(IsSimulation());

  // A fresh boot id every start, so a command built against a previous boot's
  // state can never be accepted after a code restart.
  std::random_device random;
  const uint64_t boot_id = (uint64_t{random()} << 32) | random() | 1;

  gateway_ = std::make_unique<talos::hardware::Gateway>(*backend_, boot_id);
  std::ifstream saved{kConfigPath, std::ios::binary};
  if (saved) {
    const std::vector<uint8_t> bytes{std::istreambuf_iterator<char>{saved}, {}};
    talos::hardware::Config restored;
    if (talos::hardware::LoadPersistedConfig(bytes, restored))
      gateway_->Reconfigure(restored);
  }
  endpoint_ = std::make_unique<talos::hardware::Endpoint>(*gateway_);
  endpoint_->OnConfigured([last_id = gateway_->snapshot().config_id](
                              const talos::hardware::Config& config) mutable {
    const auto id = talos::hardware::ConfigurationId(config);
    if (id == last_id) return;
    const auto bytes = talos::hardware::PersistedConfig(config);
    const std::string temporary = std::string{kConfigPath} + ".tmp";
    std::ofstream file{temporary, std::ios::binary | std::ios::trunc};
    file.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    file.close();
    if (file && std::rename(temporary.c_str(), kConfigPath) == 0)
      last_id = id;
    else
      std::fprintf(stderr, "cannot persist hardware configuration\n");
  });

  bool socket_open = false;
  uint64_t next_socket_retry_us = 0;

  std::printf(
      "hardware gateway config=%llu commissioned=%d (waiting for config "
      "push)\n",
      static_cast<unsigned long long>(gateway_->snapshot().config_id),
      gateway_->configured() ? gateway_->config().commissioned : 0);

  // Tells the Driver Station that robot code is up. Everything that can block
  // for a long time, in particular configuring devices over CAN, has happened
  // by now.
  HAL_ObserveUserProgramStarting();

  auto period = std::chrono::microseconds{5000};
  auto next = std::chrono::steady_clock::now();

#ifndef __FRC_ROBORIO__
  SimDriverStation simulated_driver_station{*gateway_};

  // A bounded simulation run, so an automated test does not have to race a
  // kill signal to get a clean summary out.
  std::optional<std::chrono::steady_clock::time_point> stop_at;
  if (const char* ms = std::getenv("TALOS_SIM_RUN_MS")) {
    stop_at = next + std::chrono::milliseconds{std::atoi(ms)};
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

  while (running_.load(std::memory_order_relaxed)) {
    if (gateway_->configured()) {
      period = std::chrono::microseconds{gateway_->config().period_us};
    }
    next += period;

#ifndef __FRC_ROBORIO__
    simulated_driver_station.Update();
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
    if (!socket_open && now_us >= next_socket_retry_us) {
      const auto status = endpoint_->Open(
          IsSimulation() ? "127.0.0.1" : "0.0.0.0", 5802,
          IsSimulation() ? "127.0.0.1" : kCompanionAddress, 5803);
      socket_open = status == talos::protocol::UdpStatus::kOk;
      next_socket_retry_us = now_us + 1000000;
      if (!socket_open)
        std::printf("hardware gateway socket failed: %s\n",
                    talos::protocol::UdpStatusName(status));
    }
    if (socket_open) {
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

    // An overrun must not turn into a burst of catch-up iterations that starve
    // the Driver Station refresh; give up the missed cycles and stay on phase.
    const auto now = std::chrono::steady_clock::now();
    if (next < now) {
      next = now;
    }
    std::this_thread::sleep_until(next);
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
  endpoint_.reset();
  gateway_.reset();
  backend_.reset();
}

void Robot::EndCompetition() {
  running_.store(false, std::memory_order_relaxed);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

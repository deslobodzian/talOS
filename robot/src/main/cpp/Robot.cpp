#include "Robot.h"

#include <frc/DriverStation.h>
#include <hal/DriverStation.h>

#include <chrono>
#include <cstdio>
#include <random>
#include <thread>

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

#ifndef __FRC_ROBORIO__
// An automated stand-in for the Driver Station, for tests that run this program
// headless. It is off unless TALOS_SIM_DS is set, so an interactive
// `./gradlew simulateNative` behaves like any other WPILib robot: it starts
// disabled and you enable it and pick Teleoperated in the simulation GUI.
//
//   TALOS_SIM_DS=1                        enable in teleop at startup
//   TALOS_SIM_DS=0                        attach a Driver Station, stay disabled
//   TALOS_SIM_DS_DISABLE_AFTER_MS=2000    ... then disable two seconds in
//   TALOS_SIM_RUN_MS=10000                stop after ten seconds and report
//
// This exists only in the desktop build. On the RoboRIO the enable state comes
// from the real Driver Station and nothing here can forge it.
class SimDriverStation {
 public:
  SimDriverStation() {
    const char* requested = std::getenv("TALOS_SIM_DS");
    if (requested == nullptr) {
      return;  // Leave the Driver Station to the GUI or to a real one.
    }

    active_ = true;
    enabled_ = std::string_view{requested} != "0";

    if (const char* after = std::getenv("TALOS_SIM_DS_DISABLE_AFTER_MS")) {
      disable_after_ = std::chrono::milliseconds{std::atoi(after)};
    }

    // Teleoperated: neither autonomous nor test. The gateway does not care
    // which mode it is, only whether the robot is enabled, but the Driver
    // Station watchdog and any future mode-dependent code do.
    frc::sim::DriverStationSim::SetAutonomous(false);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::SetDsAttached(true);
    Publish();
  }

  void Update() {
    if (!active_ || !enabled_ || !disable_after_) {
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

  bool active_{false};
  bool enabled_{false};
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
  endpoint_ = std::make_unique<talos::hardware::Endpoint>(*gateway_);

  const auto status =
      endpoint_->Open(IsSimulation() ? "127.0.0.1" : "0.0.0.0", 5802,
                      IsSimulation() ? "127.0.0.1" : kCompanionAddress, 5803);
  if (status != talos::protocol::UdpStatus::kOk) {
    std::printf("hardware gateway socket failed: %s\n",
                talos::protocol::UdpStatusName(status));
    endpoint_.reset();
  }

  std::printf("hardware gateway config=%llu commissioned=%d (waiting for config push)\n",
              static_cast<unsigned long long>(gateway_->snapshot().config_id),
              gateway_->configured() ? gateway_->config().commissioned : 0);

  // Tells the Driver Station that robot code is up. Everything that can block
  // for a long time, in particular configuring devices over CAN, has happened
  // by now.
  HAL_ObserveUserProgramStarting();

  auto period = std::chrono::microseconds{5000};
  auto next = std::chrono::steady_clock::now();

#ifndef __FRC_ROBORIO__
  SimDriverStation simulated_driver_station;

  // A bounded simulation run, so an automated test does not have to race a
  // kill signal to get a clean summary out.
  std::optional<std::chrono::steady_clock::time_point> stop_at;
  if (const char* ms = std::getenv("TALOS_SIM_RUN_MS")) {
    stop_at = next + std::chrono::milliseconds{std::atoi(ms)};
  }
  uint64_t active_ticks = 0;
#endif

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

    if (endpoint_) {
      endpoint_->Tick(NowUs(), frc::DriverStation::IsEnabled() &&
                                   !frc::DriverStation::IsEStopped());
    } else {
      backend_->Neutral();
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
}

void Robot::EndCompetition() {
  running_.store(false, std::memory_order_relaxed);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

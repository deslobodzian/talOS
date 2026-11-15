// The whole teleop path, with the same nodes the launcher starts and
// nothing stubbed: a joystick packet lands on /hw/state/driver_station (sent
// by SimHardware, as the bridge forwards it), the driver writes
// it to /drivetrain/target/teleop, the drivetrain's Target selects that lane
// because the mode says teleop, and resolves it into per-module requests.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#include "driver/driver.h"
#include "drivetrain/drivetrain.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/testing.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/messages.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/node/testing.h"

namespace integration {
namespace {

using namespace std::chrono_literals;
using talos::testing::Sim;
using talos::testing::SimHardware;
namespace ds = talos::driver_station;

ds::JoystickData Stick(float axis0, float axis1, float axis2) {
  ds::JoystickData stick{};
  stick.connected = true;
  stick.axis_count = 4;
  stick.button_count = 12;
  stick.pov_count = 1;
  stick.axes[0] = axis0;
  stick.axes[1] = axis1;
  stick.axes[2] = axis2;
  return stick;
}

// The packet the bridge forwards on /hw/state/driver_station.
ds::DriverStationData DsState(std::uint32_t flags,
                              const ds::JoystickData& stick) {
  ds::DriverStationData data{};
  data.fms.flags = flags;
  data.joysticks[0] = stick;
  return data;
}

constexpr std::uint32_t kTeleopEnabled =
    ds::kEnabled | ds::kTeleop | ds::kDsAttached;

std::string RobotConfig() {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv("TALOS_TEST_CONFIG");
  if (!runfiles || location == nullptr) {
    throw std::runtime_error("robot config unavailable: " + error);
  }
  return runfiles->Rlocation(location);
}

class TeleopChain : public ::testing::Test {
 protected:
  void Push(const ds::DriverStationData& data) {
    hw.set_driver_station(data);
    sim.step(20ms);
  }
  static ds::DriverStationData ForwardStick() {
    return DsState(kTeleopEnabled, Stick(0, -1.0f, 0));  // full forward
  }
  void ExpectModulesNeutral() const {
    for (const auto& module : geometry.modules) {
      EXPECT_EQ(hw.motor_request("drivetrain", module.drive).mode,
                talos::hardware::Mode::kNeutral)
          << module.name;
    }
  }

  Sim sim{RobotConfig()};
  driver::Driver& driver_node = sim.add<driver::Driver>("driver");
  drivetrain::Drivetrain& drivetrain_node =
      sim.add<drivetrain::Drivetrain>("drivetrain");
  SimHardware hw{sim};
  const drivetrain::SwerveGeometry geometry = drivetrain::BuildSwerveGeometry(
      talos::config::ParseRobotConfig(RobotConfig()), "drivetrain");

  void SetUp() override {
    hw.enable();
    // ClockOffset requires 8 hardware samples (40 ms at 5 ms) before
    // sample_time() and the drivetrain's gyro heading become valid.
    Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
    Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
  }
};

TEST_F(TeleopChain, JoystickDeflectionReachesTheSwerveModulesAsVelocity) {
  // 4 m/s over a 0.0508 m wheel (12.5 rps), below the robot's 14.5 rps drive
  // limit, so the request is not desaturated.
  const double expected_rps = 4.0 / (2 * std::numbers::pi * 0.0508);
  Push(ForwardStick());
  // drivetrain.toml's chassis slew limit ramps the step instead of applying
  // it in one tick.
  const double first =
      hw.motor_request("drivetrain", geometry.modules[0].drive).demand;
  EXPECT_GT(first, 0.0);
  EXPECT_LT(first, expected_rps);
  for (int i = 0; i < 50; ++i) Push(ForwardStick());  // 1 s, past the ramp
  for (const auto& module : geometry.modules) {
    const auto drive = hw.motor_request("drivetrain", module.drive);
    const auto steer = hw.motor_request("drivetrain", module.steer);
    EXPECT_EQ(drive.mode, talos::hardware::Mode::kVelocity) << module.name;
    EXPECT_NEAR(drive.demand, expected_rps, 1e-9) << module.name;
    EXPECT_EQ(steer.mode, talos::hardware::Mode::kPosition) << module.name;
    EXPECT_NEAR(steer.demand, 0.0, 1e-9) << module.name;
  }
}

TEST_F(TeleopChain, ReleasingTheStickDeceleratesTheModulesToZero) {
  for (int i = 0; i < 51; ++i) Push(ForwardStick());
  const auto& drive = geometry.modules[0].drive;
  const double cruising = hw.motor_request("drivetrain", drive).demand;
  ASSERT_GT(cruising, 0.0);
  // A centred stick is a live zero target: the modules decelerate under the
  // slew limit, still under velocity control, not neutralized by staleness.
  double previous = cruising;
  for (int i = 0; i < 50; ++i) {
    Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
    const auto request = hw.motor_request("drivetrain", drive);
    ASSERT_EQ(request.mode, talos::hardware::Mode::kVelocity);
    EXPECT_LE(request.demand, previous);
    previous = request.demand;
  }
  for (const auto& module : geometry.modules) {
    EXPECT_NEAR(hw.motor_request("drivetrain", module.drive).demand, 0.0, 1e-9)
        << module.name;
  }
}

// A stick held through a disable drives no module after re-enable.
// REQ-SAFE-14
TEST_F(TeleopChain, DisablingOrLosingTheDriverStationStopsTheModules) {
  Push(ForwardStick());
  ASSERT_EQ(hw.motor_request("drivetrain", geometry.modules[0].drive).mode,
            talos::hardware::Mode::kVelocity);
  Push(DsState(ds::kDsAttached, Stick(0, -1.0f, 0)));
  ExpectModulesNeutral();
  Push(ForwardStick());
  ExpectModulesNeutral();  // Held sticks cannot resume after disable.
  Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));

  Push(ForwardStick());
  ASSERT_EQ(hw.motor_request("drivetrain", geometry.modules[0].drive).mode,
            talos::hardware::Mode::kVelocity);
  hw.stop_driver_station();
  sim.step(200ms);
  ExpectModulesNeutral();
}

// RULES.md 3: once the stick is held, each packet's evaluation and lane
// write, the driver's update(), and the drivetrain's target read and module
// requests allocate nothing. SimHardware forwards the held packet every
// period through the window.
// REQ-RT-06
TEST_F(TeleopChain, UpdateDoesNotAllocate) {
  if (const auto why = talos::testing::AllocationProbe::Unsupported();
      !why.empty()) {
    GTEST_SKIP() << why;
  }
  for (int i = 0; i < 10; ++i) Push(ForwardStick());
  talos::testing::AllocationProbe probe;
  probe.start();
  sim.step(100ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(hw.motor_request("drivetrain", geometry.modules[0].drive).mode,
            talos::hardware::Mode::kVelocity);
}

TEST_F(TeleopChain, AutonomousIgnoresTheSticks) {
  Push(DsState(ds::kEnabled | ds::kAutonomous | ds::kDsAttached,
               Stick(0, -1.0f, 0)));
  sim.step(40ms);
  ExpectModulesNeutral();
}

}  // namespace
}  // namespace integration

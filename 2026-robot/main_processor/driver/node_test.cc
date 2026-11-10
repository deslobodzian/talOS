#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "driver/driver.h"
#include "drivetrain/drivetrain.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/devices/testing.h"
#include "talOS/node/testing.h"

namespace driver {
namespace {

using namespace std::chrono_literals;
using talos::testing::LatestTarget;
using talos::testing::Sim;
using talos::testing::SimHardware;
namespace ds = talos::driver_station;

constexpr double kMaxLinear = 4.0;  // the compiled defaults
constexpr double kMaxAngular = 6.0;
constexpr double kQuarterTurnRad = std::numbers::pi / 2;

ds::JoystickData Stick(float axis0, float axis1, float axis2,
                       std::uint32_t buttons = 0, bool connected = true) {
  ds::JoystickData stick{};
  stick.connected = connected;
  stick.axis_count = 4;
  stick.button_count = 12;
  stick.pov_count = 1;
  stick.buttons = buttons;
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

// One driver node whose [subsystems.driver] table holds `keys` and nothing
// else, so every other setting is the compiled default.
class Harness {
 public:
  explicit Harness(std::string_view keys = "", bool prime = true,
                   int period_us = 20000)
      : sim_{Sim::FromToml(std::string{R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.mechanism.motors.unused]
can_id = 1

[subsystems.driver]
)"} + "period_us = " + std::to_string(period_us) +
                           "\n" + std::string{keys})} {
    if (prime) Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
  }

  // SimHardware forwards this packet every robot period from now on.
  void Push(const ds::DriverStationData& state) {
    hw_->set_driver_station(state);
    sim_.step(10ms);
  }
  // The bridge goes silent: no more Driver Station packets.
  void Silence() { hw_.reset(); }
  void Reconnect() { hw_ = std::make_unique<SimHardware>(sim_); }
  // `omega_radps` is the rotation the drivetrain's modules measure.
  void PublishHeading(double yaw_rad, bool yaw_valid = true,
                      double omega_radps = 0) {
    const drivetrain::DrivetrainState state{
        0, true, true, yaw_valid, 0, 0, omega_radps, yaw_rad, {}, {}, {}, {}};
    sim_.publish(drivetrain::topics::kState, state);
    sim_.step(10ms);
  }
  void Advance(talos::Duration d) { sim_.step(d); }
  // Envelopes written on /drivetrain/target/teleop so far.
  std::uint64_t chassis_count() const {
    return talos::detail::SimAccess::message_count(
        sim_, "/drivetrain/target/teleop",
        sizeof(talos::TargetEnvelope<drivetrain::ChassisTarget>));
  }
  // The newest envelope on drivetrain teleop lane (nullopt: never written).
  std::optional<talos::TargetEnvelope<drivetrain::ChassisTarget>> chassis_lane()
      const {
    return LatestTarget<drivetrain::ChassisTarget>(sim_, "drivetrain",
                                                   "teleop");
  }
  // The message of an active lane; fails the test when it is released.
  drivetrain::ChassisTarget chassis() const { return Active(chassis_lane()); }
  bool chassis_active() const { return IsActive(chassis_lane()); }

 private:
  template <class M>
  static bool IsActive(const std::optional<talos::TargetEnvelope<M>>& e) {
    EXPECT_TRUE(e.has_value());
    return e.has_value() && e->active != 0;
  }
  template <class M>
  static M Active(const std::optional<talos::TargetEnvelope<M>>& e) {
    EXPECT_TRUE(IsActive(e));
    return e.has_value() ? e->message : M{};
  }

  Sim sim_;
  Driver& node_ = sim_.add<Driver>("driver");
  std::unique_ptr<SimHardware> hw_ = std::make_unique<SimHardware>(sim_);
};

constexpr const char* kNoDeadbandRobotRelative =
    "deadband_frac = 0.0\nfield_oriented = false\n";
constexpr const char* kNoDeadbandFieldOriented =
    "deadband_frac = 0.0\nfield_oriented = true\n";

TEST(Driver, MapsStickAxesToChassisRequests) {
  Harness harness{kNoDeadbandRobotRelative};
  // Forward is negative axis 1, left is negative axis 0, CCW is negative 2.
  harness.Push(DsState(kTeleopEnabled, Stick(-0.5f, -0.75f, -0.25f, 1)));
  const auto target = harness.chassis();
  EXPECT_NEAR(target.vx_mps(), 0.75 * kMaxLinear, 1e-9);
  EXPECT_NEAR(target.vy_mps(), 0.5 * kMaxLinear, 1e-9);
  EXPECT_NEAR(target.omega_radps(), 0.25 * kMaxAngular, 1e-9);
}

TEST(Driver, DeadbandIsZeroInsideAndContinuousAtTheThreshold) {
  Harness harness{"deadband_frac = 0.05\nfield_oriented = false\n"};
  harness.Push(DsState(kTeleopEnabled, Stick(0, -0.04f, 0)));
  EXPECT_NEAR(harness.chassis().vx_mps(), 0.0, 1e-12);
  // Just outside: the output ramps from zero rather than stepping to full.
  harness.Push(DsState(kTeleopEnabled, Stick(0, -0.10f, 0)));
  EXPECT_NEAR(harness.chassis().vx_mps(), (0.10 - 0.05) / 0.95 * kMaxLinear,
              1e-6);
}

TEST(Driver, CircularDeadbandCapsDiagonalAndPreservesDirection) {
  Harness harness{kNoDeadbandRobotRelative};
  harness.Push(DsState(kTeleopEnabled, Stick(-1, -1, 0)));
  EXPECT_NEAR(harness.chassis().vx_mps(), kMaxLinear / std::sqrt(2), 1e-9);
  EXPECT_NEAR(harness.chassis().vy_mps(), kMaxLinear / std::sqrt(2), 1e-9);
  Harness deadband{"deadband_frac = 0.05\nfield_oriented = false\n"};
  deadband.Push(DsState(kTeleopEnabled, Stick(-0.04f, -0.04f, 0)));
  EXPECT_GT(deadband.chassis().vx_mps(), 0);
  EXPECT_NEAR(deadband.chassis().vx_mps(), deadband.chassis().vy_mps(), 1e-9);
}

TEST(Driver, BootForwardIsAwayFromEitherAllianceDriver) {
  for (const auto alliance : {ds::Alliance::kBlue, ds::Alliance::kRed}) {
    Harness harness{kNoDeadbandFieldOriented};
    harness.PublishHeading(0);  // boot facing away from this driver
    auto packet = DsState(kTeleopEnabled, Stick(0, -1, -0.5f));
    packet.fms.alliance = alliance;
    harness.Push(packet);
    EXPECT_NEAR(harness.chassis().vx_mps(), kMaxLinear, 1e-9);
    EXPECT_NEAR(harness.chassis().vy_mps(), 0, 1e-9);
    EXPECT_NEAR(harness.chassis().omega_radps(), 3, 1e-9);
  }
}

TEST(Driver, RezeroForwardKeepsLatchedDirectionOnBothAlliances) {
  for (const auto alliance : {ds::Alliance::kBlue, ds::Alliance::kRed}) {
    for (const double yaw : {-2.3, 0.7, std::numbers::pi}) {
      Harness harness{kNoDeadbandFieldOriented};
      harness.PublishHeading(yaw);
      auto packet = DsState(kTeleopEnabled, Stick(0, -1, 0, 1));
      packet.fms.alliance = alliance;
      harness.Push(packet);
      EXPECT_NEAR(harness.chassis().vx_mps(), kMaxLinear, 1e-9);
      EXPECT_NEAR(harness.chassis().vy_mps(), 0, 1e-9);
      harness.PublishHeading(yaw + kQuarterTurnRad);
      packet.joysticks[0] = Stick(0, -1, 0);
      harness.Push(packet);
      EXPECT_NEAR(harness.chassis().vx_mps(), 0, 1e-9);
      EXPECT_NEAR(harness.chassis().vy_mps(), -kMaxLinear, 1e-9);
    }
  }
}

TEST(Driver, RezeroLatchesOnlyOnRisingEdge) {
  Harness harness{kNoDeadbandFieldOriented};
  harness.PublishHeading(kQuarterTurnRad);
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1, 0, 1)));
  EXPECT_NEAR(harness.chassis().vx_mps(), kMaxLinear, 1e-9);
  harness.PublishHeading(std::numbers::pi);
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1, 0, 1)));
  EXPECT_NEAR(harness.chassis().vy_mps(), -kMaxLinear, 1e-9);
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1, 0)));
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1, 0, 1)));
  EXPECT_NEAR(harness.chassis().vx_mps(), kMaxLinear, 1e-9);
}

TEST(Driver, HeadingHoldCorrectsWrappedErrorAndResetsAfterManualTurn) {
  Harness harness{"deadband_frac = 0.0\nheading_hold_radps = 4.0\n"};
  harness.PublishHeading(std::numbers::pi - 0.1);
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
  harness.PublishHeading(-std::numbers::pi + 0.1);
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
  EXPECT_NEAR(harness.chassis().omega_radps(), -0.8, 1e-9);
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, -0.5f)));
  EXPECT_NEAR(harness.chassis().omega_radps(), 3, 1e-9);
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
  EXPECT_NEAR(harness.chassis().omega_radps(), 0, 1e-9);
}

TEST(Driver, HeadingHoldLatchesWhereAReleasedTurnStops) {
  Harness harness{"deadband_frac = 0.0\nheading_hold_radps = 4.0\n"};
  harness.PublishHeading(0);
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, -1)));
  harness.PublishHeading(0.5, true, kMaxAngular);
  // Released at 0.5 rad while the drivetrain's slew limit still turns the
  // robot: the coast is not an error to drive back.
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
  harness.PublishHeading(1.2, true, 3);
  EXPECT_NEAR(harness.chassis().omega_radps(), 0, 1e-9);
  // Latched where the turn stopped; the hold's own correction, measured as
  // rotation, does not move it.
  harness.PublishHeading(1.5, true, 0.05);
  harness.PublishHeading(1.4, true, 0.4);
  EXPECT_NEAR(harness.chassis().omega_radps(), 4 * 0.1, 1e-9);
  // A stale drivetrain state commands no correction, and one fresh again
  // mid-turn restarts the hold where that turn stops.
  harness.Advance(150ms);
  EXPECT_NEAR(harness.chassis().omega_radps(), 0, 1e-9);
  harness.PublishHeading(2.0, true, 2);
  EXPECT_NEAR(harness.chassis().omega_radps(), 0, 1e-9);
  harness.PublishHeading(2.3, true, 0);
  harness.PublishHeading(2.2, true, 0);
  EXPECT_NEAR(harness.chassis().omega_radps(), 4 * 0.1, 1e-9);
}

TEST(Driver, HeadingWithinTimeoutStillRotatesTheFieldRelativeRequest) {
  Harness harness{kNoDeadbandFieldOriented};
  harness.PublishHeading(kQuarterTurnRad);
  harness.Advance(20ms);  // one dropped packet is not a stale publisher
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));
  EXPECT_NEAR(harness.chassis().vx_mps(), 0.0, 1e-9);
  EXPECT_NEAR(harness.chassis().vy_mps(), -kMaxLinear, 1e-9);
}

TEST(Driver, WithoutFreshFiniteHeadingRearmsRobotRelativeAtNeutral) {
  for (const double yaw :
       {kQuarterTurnRad, std::numeric_limits<double>::quiet_NaN()}) {
    Harness harness{kNoDeadbandFieldOriented};
    harness.PublishHeading(kQuarterTurnRad);
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1, -0.5f)));
    // Invalid observations cannot refresh the last heading.
    harness.PublishHeading(yaw, std::isnan(yaw));
    for (int i = 0; i < 20; ++i) harness.PublishHeading(yaw, std::isnan(yaw));
    harness.Advance(150ms);
    EXPECT_DOUBLE_EQ(harness.chassis().vx_mps(), 0);
    EXPECT_DOUBLE_EQ(harness.chassis().vy_mps(), 0);
    EXPECT_DOUBLE_EQ(harness.chassis().omega_radps(), 3);
    // Translation neutral is sufficient; rotation need not be centred.
    harness.Push(DsState(kTeleopEnabled, Stick(0, 0, -0.5f)));
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1, -0.5f)));
    EXPECT_DOUBLE_EQ(harness.chassis().vx_mps(), kMaxLinear);
    EXPECT_DOUBLE_EQ(harness.chassis().vy_mps(), 0);
    EXPECT_DOUBLE_EQ(harness.chassis().omega_radps(), 3);
    harness.PublishHeading(kQuarterTurnRad);
    // A fresh heading must not change the frame under a held stick.
    EXPECT_DOUBLE_EQ(harness.chassis().vx_mps(), kMaxLinear);
    EXPECT_DOUBLE_EQ(harness.chassis().vy_mps(), 0);
    harness.Push(DsState(kTeleopEnabled, Stick(0, 0, -0.5f)));
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1, -0.5f)));
    EXPECT_NEAR(harness.chassis().vx_mps(), 0, 1e-9);
    EXPECT_NEAR(harness.chassis().vy_mps(), -kMaxLinear, 1e-9);
  }
}

TEST(Driver, MissingHeadingAtStartupAllowsRobotRelativeAfterNeutral) {
  for (const bool valid : {false, true}) {
    Harness harness{kNoDeadbandFieldOriented, false};
    harness.PublishHeading(std::numeric_limits<double>::quiet_NaN(), valid);
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1, -0.5f)));
    EXPECT_EQ(harness.chassis_count(), 0u);
    harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1, -0.5f)));
    EXPECT_DOUBLE_EQ(harness.chassis().vx_mps(), kMaxLinear);
    EXPECT_DOUBLE_EQ(harness.chassis().omega_radps(), 3);
  }
}

TEST(Driver, FreshHeadingRequiresNeutralTranslationBeforeResuming) {
  Harness harness{kNoDeadbandFieldOriented};
  harness.PublishHeading(kQuarterTurnRad);
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, -0.5f)));
  EXPECT_NEAR(harness.chassis().vy_mps(), -kMaxLinear, 1e-9);
  harness.Advance(150ms);
  EXPECT_DOUBLE_EQ(harness.chassis().vx_mps(), 0.0);
  EXPECT_DOUBLE_EQ(harness.chassis().vy_mps(), 0.0);
  EXPECT_DOUBLE_EQ(harness.chassis().omega_radps(), 3.0);
  harness.PublishHeading(kQuarterTurnRad);
  EXPECT_DOUBLE_EQ(harness.chassis().vx_mps(), 0.0);
  EXPECT_DOUBLE_EQ(harness.chassis().vy_mps(), 0.0);
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, -0.5f)));
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, -0.5f)));
  EXPECT_NEAR(harness.chassis().vy_mps(), -kMaxLinear, 1e-9);
}

TEST(Driver, FallbackStillReleasesAndRequiresNeutralAfterDisableOrDsLoss) {
  for (const bool stale : {false, true}) {
    Harness harness{kNoDeadbandFieldOriented};
    // No heading: neutral startup arms robot-relative fallback.
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1, -0.5f)));
    ASSERT_TRUE(harness.chassis_active());
    if (stale) {
      harness.Silence();
      harness.Advance(150ms);
      harness.Reconnect();
    } else {
      harness.Push(DsState(ds::kDsAttached, Stick(0, -1, -0.5f)));
    }
    EXPECT_FALSE(harness.chassis_active());
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1, -0.5f)));
    EXPECT_FALSE(harness.chassis_active());
    harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1, -0.5f)));
    EXPECT_DOUBLE_EQ(harness.chassis().vx_mps(), kMaxLinear);
    EXPECT_DOUBLE_EQ(harness.chassis().omega_radps(), 3);
  }
}

TEST(Driver, RequestsNothingWhenDisabledAutonomousOrDisconnected) {
  const ds::JoystickData deflected = Stick(0, -0.9f, 0);
  const std::pair<std::uint32_t, ds::JoystickData> cases[]{
      {ds::kDsAttached, deflected},
      {ds::kEnabled | ds::kTeleop, deflected},
      {ds::kEnabled | ds::kAutonomous | ds::kDsAttached, deflected},
      {kTeleopEnabled, Stick(0, -0.9f, 0, 0, /*connected=*/false)},
  };
  for (const auto& [flags, stick] : cases) {
    Harness harness{kNoDeadbandRobotRelative, false};
    harness.Push(DsState(flags, stick));
    EXPECT_EQ(harness.chassis_count(), 0u);
  }
}

TEST(Driver, ReleasesLaneOnceOnTheDisableEdge) {
  Harness harness{kNoDeadbandRobotRelative};
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));
  ASSERT_TRUE(harness.chassis_active());

  // Disabled with the stick still deflected: lane is released rather
  // than left to time the last request out.
  harness.Push(DsState(ds::kDsAttached, Stick(0, -1.0f, 0)));
  EXPECT_FALSE(harness.chassis_active());
  const std::uint64_t after_release = harness.chassis_count();

  // Staying disabled is not another edge.
  harness.Push(DsState(ds::kDsAttached, Stick(0, -1.0f, 0)));
  EXPECT_EQ(harness.chassis_count(), after_release);
}

TEST(Driver, ASilentDriverStationReleasesLaneOnce) {
  Harness harness{kNoDeadbandRobotRelative};
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1.0f, 0)));
  ASSERT_TRUE(harness.chassis_active());

  // No packet arrives to say "disabled"; the last one ages past the command
  // timeout and update() releases the lane once.
  harness.Silence();
  harness.Advance(300ms);
  EXPECT_FALSE(harness.chassis_active());
  const std::uint64_t after_release = harness.chassis_count();
  harness.Advance(100ms);
  EXPECT_EQ(harness.chassis_count(), after_release);
}

TEST(Driver, StartupHeldAxesOrRezeroCannotArmProducer) {
  for (const auto& held :
       {Stick(0, -1, 0), Stick(0, 0, -1), Stick(0, 0, 0, 1)}) {
    Harness harness{kNoDeadbandRobotRelative, false};
    harness.Push(DsState(kTeleopEnabled, held));
    EXPECT_EQ(harness.chassis_count(), 0u);
    harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1, 0)));
    EXPECT_DOUBLE_EQ(harness.chassis().vx_mps(), kMaxLinear);
  }
}

TEST(Driver, FreshHeadingCannotArmAStartupHeldFieldRelativeStick) {
  Harness harness{kNoDeadbandFieldOriented, false};
  const auto held = DsState(kTeleopEnabled, Stick(0, -1, -0.5f));
  harness.Push(held);
  EXPECT_EQ(harness.chassis_count(), 0u);
  harness.PublishHeading(kQuarterTurnRad);
  EXPECT_EQ(harness.chassis_count(), 0u);
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
  harness.Push(held);
  EXPECT_NEAR(harness.chassis().vx_mps(), 0.0, 1e-9);
  EXPECT_NEAR(harness.chassis().vy_mps(), -kMaxLinear, 1e-9);
  EXPECT_DOUBLE_EQ(harness.chassis().omega_radps(), 3.0);
}

TEST(Driver, HeldStickDoesNotResumeAfterDsLossOrDisable) {
  Harness harness{kNoDeadbandRobotRelative};
  for (const bool stale : {false, true}) {
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1, 0)));
    ASSERT_TRUE(harness.chassis_active());
    if (stale) {
      harness.Silence();
      harness.Advance(150ms);
      harness.Reconnect();
    } else {
      harness.Push(DsState(ds::kDsAttached, Stick(0, -1, 0)));
    }
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1, 0)));
    EXPECT_FALSE(harness.chassis_active());
    harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
    harness.Push(DsState(kTeleopEnabled, Stick(0, -1, 0)));
    EXPECT_DOUBLE_EQ(harness.chassis().vx_mps(), kMaxLinear);
  }
}

TEST(Driver, OwnEvaluationGapRequiresNeutralWithoutObservingStaleDs) {
  // No periodic update runs in this gap. The next evaluation receives a
  // fresh DS packet, just as a suspended process would on resumption.
  Harness harness{kNoDeadbandRobotRelative, true, 200000};
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1, 0)));
  ASSERT_TRUE(harness.chassis_active());
  harness.Silence();
  harness.Advance(150ms);
  harness.Reconnect();
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1, 0)));
  EXPECT_FALSE(harness.chassis_active());
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0)));
  harness.Push(DsState(kTeleopEnabled, Stick(0, -1, 0)));
  EXPECT_DOUBLE_EQ(harness.chassis().vx_mps(), kMaxLinear);
}

// --- the teleop chain on the real robot config ---------------------------

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

// The whole teleop path, with the same nodes the launcher starts and
// nothing stubbed: a joystick packet lands on /hw/state/driver_station (sent
// by SimHardware, as the bridge forwards it), the driver writes
// it to /drivetrain/target/teleop, the drivetrain's Target selects that lane
// because the mode says teleop, and resolves it into per-module requests.
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
  Driver& driver_node = sim.add<Driver>("driver");
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
  // 4 m/s over a 0.0508 m wheel, below the 20 rps drive ceiling, so the
  // request is not desaturated.
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

TEST_F(TeleopChain, AutonomousIgnoresTheSticks) {
  Push(DsState(ds::kEnabled | ds::kAutonomous | ds::kDsAttached,
               Stick(0, -1.0f, 0)));
  sim.step(40ms);
  ExpectModulesNeutral();
}

}  // namespace
}  // namespace driver

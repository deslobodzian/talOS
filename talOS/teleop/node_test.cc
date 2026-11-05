#include "talOS/node/node.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "control/control_generated.h"
#include "schema_test/shapes_generated.h"
#include "talOS/devices/target.h"
#include "talOS/devices/testing.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/packet.h"
#include "talOS/node/testing.h"
#include "talOS/teleop/deadband.h"
#include "talOS/teleop/node.h"

namespace talos::teleop {
namespace {

using namespace std::chrono_literals;
using talos::testing::Sim;
using talos::testing::SimHardware;
namespace ds = talos::driver_station;
template <class M>
struct ConsumerNode {
  explicit ConsumerNode(talos::Node& node)
      : node_{node}, target_{node}, ds_{node} {}
  void update() {
    const M* t = target_.get();
    seen = t != nullptr ? std::optional<M>{*t} : std::nullopt;
    source = std::string{target_.source()};
  }
  talos::Node& node_;
  talos::Target<M> target_;
  talos::DriverStation ds_;
  std::optional<M> seen;
  std::string source;
};

ds::JoystickData MakeStick(float axis0 = 0.0f, float axis1 = 0.0f,
                           float axis2 = 0.0f, std::uint32_t buttons = 0,
                           bool connected = true) {
  ds::JoystickData stick{};
  stick.connected = connected;
  stick.axis_count = 4;
  stick.button_count = 12;
  stick.buttons = buttons;
  stick.axes[0] = axis0;
  stick.axes[1] = axis1;
  stick.axes[2] = axis2;
  return stick;
}

ds::DriverStationData DsPacket(std::uint32_t flags,
                               const ds::JoystickData& stick,
                               std::size_t port = 0) {
  ds::DriverStationData data{};
  data.fms.flags = flags;
  data.joysticks[port] = stick;
  return data;
}

constexpr std::uint32_t kTeleopEnabled =
    ds::kEnabled | ds::kTeleop | ds::kDsAttached;
Sim MakeSim(std::string_view toml) {
  constexpr const char* kDummyMotor = R"(
[subsystems.dummy.motors.unused]
can_id = 1
)";
  return Sim::FromToml(std::string{kDummyMotor} + std::string{toml});
}
inline void Push(SimHardware& hw, Sim& sim, const ds::DriverStationData& data,
                 talos::Duration d = 60ms) {
  hw.set_driver_station(data);
  sim.step(d);
}

constexpr const char* kRollerToml = R"(
[robot]
period_us = 20000
command_timeout_us = 100000

[subsystems.roller]
period_us = 20000

[[subsystems.roller.teleop.bindings]]
button = 1
while_held = { velocity_rps = 60.0 }

[subsystems.teleop]
period_us = 20000
)";

TEST(TeleopNode, ButtonDrivesLaneWithEncodedValue) {
  Sim sim = MakeSim(kRollerToml);
  auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();

  // Initially not pressed
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 0)));
  EXPECT_FALSE(roller.seen.has_value());

  // Press button 1 (1-based, bit 0 in buttons mask: 1u << 0 = 1)
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 1)));
  if (!roller.seen.has_value()) {
    FAIL() << "Expected teleop target";
  }
  const auto& pressed = roller.seen.value();
  EXPECT_DOUBLE_EQ(pressed.velocity_rps(), 60.0);
  EXPECT_EQ(roller.source, "teleop");

  // Release button 1
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 0)));
  EXPECT_FALSE(roller.seen.has_value());
}

constexpr const char* kRichToml = R"(
[robot]
period_us = 20000
command_timeout_us = 100000

[subsystems.mechanism]
period_us = 20000

[[subsystems.mechanism.teleop.bindings]]
button = 2
while_held = { flag = true, position_m = 3.5, level = "TOP", "inner/mode" = "HIGH", "inner/offset_m" = 1.25 }

[subsystems.teleop]
period_us = 20000
)";

TEST(TeleopNode, RichStructFieldEncoding) {
  Sim sim = MakeSim(kRichToml);
  auto& mech = sim.add<ConsumerNode<schema_test::Mixed>>("mechanism");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();

  // Button 2 (bit 1: mask 2)
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 2)));
  if (!mech.seen.has_value()) {
    FAIL() << "Expected teleop target";
  }
  const auto& seen = mech.seen.value();
  EXPECT_TRUE(seen.flag());
  EXPECT_DOUBLE_EQ(seen.position_m(), 3.5);
  EXPECT_EQ(seen.level(), schema_test::Level::TOP);
  EXPECT_EQ(seen.inner().mode(), schema_test::Mode::HIGH);
  EXPECT_FLOAT_EQ(seen.inner().offset_m(), 1.25f);
}

constexpr const char* kPrecedenceToml = R"(
[robot]
period_us = 20000
command_timeout_us = 100000

[subsystems.roller]
period_us = 20000

[[subsystems.roller.teleop.bindings]]
button = 1
while_held = { velocity_rps = 10.0 }

[[subsystems.roller.teleop.bindings]]
button = 2
while_held = { velocity_rps = 20.0 }

[subsystems.teleop]
period_us = 20000
)";

TEST(TeleopNode, PrecedenceBetweenTwoBindingsOnOneField) {
  Sim sim = MakeSim(kPrecedenceToml);
  auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();

  // Both button 1 and button 2 held (mask 3: bits 0 and 1)
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 3)));
  if (!roller.seen.has_value()) {
    FAIL() << "Expected teleop target";
  }
  const auto& both_pressed = roller.seen.value();
  EXPECT_DOUBLE_EQ(both_pressed.velocity_rps(),
                   10.0);  // First binding in list wins

  // Only button 2 held (mask 2)
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 2)));
  if (!roller.seen.has_value()) {
    FAIL() << "Expected teleop target";
  }
  const auto& second_pressed = roller.seen.value();
  EXPECT_DOUBLE_EQ(second_pressed.velocity_rps(), 20.0);

  // Neither held
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 0)));
  EXPECT_FALSE(roller.seen.has_value());
}

constexpr const char* kAxisToml = R"(
[robot]
period_us = 20000
command_timeout_us = 100000

[subsystems.arm]
period_us = 20000

[[subsystems.arm.teleop.bindings]]
axis = 0
field = "velocity_rps"
scale = 10.0
deadband_frac = 0.125

[subsystems.teleop]
period_us = 20000
)";

TEST(TeleopNode, AxisDeadbandEdgeAndContinuousScaling) {
  Sim sim = MakeSim(kAxisToml);
  auto& arm = sim.add<ConsumerNode<control::VelocityTarget>>("arm");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();

  // Inside deadband (|axis| = 0.0625 <= 0.125) -> none live -> released
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0.0625f)));
  EXPECT_FALSE(arm.seen.has_value());

  // At deadband edge (|axis| = 0.125 <= 0.125) -> none live -> released
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0.125f)));
  EXPECT_FALSE(arm.seen.has_value());

  // Just above deadband (axis = 0.2125):
  // rescaled = (0.2125 - 0.125) / (1.0 - 0.125) = 0.0875 / 0.875 = 0.1
  // value = scale * 0.1 = 1.0
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0.2125f)));
  if (!arm.seen.has_value()) {
    FAIL() << "Expected teleop target";
  }
  const auto& above_deadband = arm.seen.value();
  EXPECT_NEAR(above_deadband.velocity_rps(), 1.0, 1e-6);

  // Full scale (axis = 1.0): rescaled = 1.0 -> 10.0
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1.0f)));
  if (!arm.seen.has_value()) {
    FAIL() << "Expected teleop target";
  }
  const auto& full_scale = arm.seen.value();
  EXPECT_NEAR(full_scale.velocity_rps(), 10.0, 1e-6);

  // Negative axis: axis = -0.5625:
  // rescaled = copysign((0.5625 - 0.125) / 0.875, -0.5625) = -0.4375 / 0.875 =
  // -0.5 value = 10.0 * -0.5 = -5.0
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(-0.5625f)));
  if (!arm.seen.has_value()) {
    FAIL() << "Expected teleop target";
  }
  const auto& negative_axis = arm.seen.value();
  EXPECT_NEAR(negative_axis.velocity_rps(), -5.0, 1e-6);
}

TEST(TeleopNode, DisabledRobotReleasesOnceAndSendsNoFurtherMessages) {
  Sim sim = MakeSim(kRollerToml);
  auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();

  // Enabled, button 1 pressed
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 1)));
  ASSERT_TRUE(roller.seen.has_value());

  const auto lane_count = [&] {
    return talos::detail::SimAccess::message_count(
        sim, "/roller/target/teleop",
        sizeof(talos::TargetEnvelope<control::VelocityTarget>));
  };

  // Disabled
  const std::uint32_t kTeleopDisabled = ds::kTeleop | ds::kDsAttached;
  hw.set_driver_station(DsPacket(kTeleopDisabled, MakeStick(0, 0, 0, 1)));
  sim.step(40ms);
  EXPECT_FALSE(roller.seen.has_value());
  const std::uint64_t count_after_release = lane_count();

  // Next ticks while disabled: no further sends
  sim.step(40ms);
  sim.step(40ms);
  EXPECT_EQ(lane_count(), count_after_release);
}

TEST(TeleopNode, NonTeleopModeReleasesTeleopLane) {
  Sim sim = MakeSim(kRollerToml);
  auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();

  // Autonomous enabled
  const std::uint32_t kAutoEnabled =
      ds::kEnabled | ds::kAutonomous | ds::kDsAttached;
  Push(hw, sim, DsPacket(kAutoEnabled, MakeStick(0, 0, 0, 1)));
  EXPECT_FALSE(roller.seen.has_value());
}

TEST(TeleopNode, StaleDriverStationReleasesLane) {
  Sim sim = MakeSim(kRollerToml);
  auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();

  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 1)));
  ASSERT_TRUE(roller.seen.has_value());
  // Stop driver station packets (stale after command_timeout_us = 100ms)
  hw.stop_driver_station();
  sim.step(150ms);
  EXPECT_FALSE(roller.seen.has_value());
}

TEST(TeleopNode, StickDisconnectedReleasesLaneOnce) {
  Sim sim = MakeSim(kRollerToml);
  auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();

  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 1)));
  ASSERT_TRUE(roller.seen.has_value());
  const auto lane_count = [&] {
    return talos::detail::SimAccess::message_count(
        sim, "/roller/target/teleop",
        sizeof(talos::TargetEnvelope<control::VelocityTarget>));
  };

  // Stick disconnected
  hw.set_driver_station(
      DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 1, /*connected=*/false)));
  sim.step(60ms);
  EXPECT_FALSE(roller.seen.has_value());
  const std::uint64_t count_after_release = lane_count();

  // Subsequent steps with disconnected stick send nothing
  sim.step(40ms);
  sim.step(40ms);
  EXPECT_EQ(lane_count(), count_after_release);
}

// --- Construction error tests ---

TEST(TeleopNode, ConstructionErrorUnknownFieldInButton) {
  constexpr const char* kToml = R"(
[robot]
period_us = 20000

[subsystems.shooter]
period_us = 20000

[[subsystems.shooter.teleop.bindings]]
button = 1
while_held = { foo_rps = 60.0 }

[subsystems.teleop]
period_us = 20000
)";
  Sim sim = MakeSim(kToml);
  sim.add<ConsumerNode<control::VelocityTarget>>("shooter");
  try {
    sim.add<Teleop>("teleop");
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    const std::string msg = e.what();
    EXPECT_NE(
        msg.find("[subsystems.shooter] teleop.bindings[0].while_held.foo_rps: "
                 "no field 'foo_rps' in control.VelocityTarget"),
        std::string::npos)
        << msg;
  }
}

TEST(TeleopNode, ConstructionErrorUnknownFieldInAxis) {
  constexpr const char* kToml = R"(
[robot]
period_us = 20000

[subsystems.shooter]
period_us = 20000

[[subsystems.shooter.teleop.bindings]]
axis = 0
field = "bad_field"
scale = 1.0

[subsystems.teleop]
period_us = 20000
)";
  Sim sim = MakeSim(kToml);
  sim.add<ConsumerNode<control::VelocityTarget>>("shooter");
  try {
    sim.add<Teleop>("teleop");
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("no field 'bad_field' in control.VelocityTarget"),
              std::string::npos)
        << msg;
  }
}

TEST(TeleopNode, ConstructionErrorWrongTypeInButton) {
  constexpr const char* kToml = R"(
[robot]
period_us = 20000

[subsystems.shooter]
period_us = 20000

[[subsystems.shooter.teleop.bindings]]
button = 1
while_held = { velocity_rps = "fast" }

[subsystems.teleop]
period_us = 20000
)";
  Sim sim = MakeSim(kToml);
  sim.add<ConsumerNode<control::VelocityTarget>>("shooter");
  try {
    sim.add<Teleop>("teleop");
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("is numeric but value is not a number"),
              std::string::npos)
        << msg;
  }
}
TEST(TeleopNode, ConstructionErrorNumericDoesNotFitIntegerField) {
  constexpr const char* kTomlOverflow = R"(
[robot]
period_us = 20000

[subsystems.mechanism]
period_us = 20000

[[subsystems.mechanism.teleop.bindings]]
button = 1
while_held = { sample_count = 5000000000 }

[subsystems.teleop]
period_us = 20000
)";
  Sim sim1 = MakeSim(kTomlOverflow);
  sim1.add<ConsumerNode<schema_test::Mixed>>("mechanism");
  try {
    sim1.add<Teleop>("teleop");
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("does not fit integer field"), std::string::npos) << msg;
  }

  constexpr const char* kTomlFraction = R"(
[robot]
period_us = 20000

[subsystems.mechanism]
period_us = 20000

[[subsystems.mechanism.teleop.bindings]]
button = 1
while_held = { sample_count = 1.5 }

[subsystems.teleop]
period_us = 20000
)";
  Sim sim2 = MakeSim(kTomlFraction);
  sim2.add<ConsumerNode<schema_test::Mixed>>("mechanism");
  try {
    sim2.add<Teleop>("teleop");
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("fractional value for integer field"), std::string::npos)
        << msg;
  }
}

TEST(TeleopNode, ConstructionErrorAxisBoundToNonNumeric) {
  constexpr const char* kToml = R"(
[robot]
period_us = 20000

[subsystems.mech]
period_us = 20000

[[subsystems.mech.teleop.bindings]]
axis = 0
field = "flag"
scale = 1.0

[subsystems.teleop]
period_us = 20000
)";
  Sim sim = MakeSim(kToml);
  sim.add<ConsumerNode<schema_test::Mixed>>("mech");
  try {
    sim.add<Teleop>("teleop");
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("must be a numeric scalar"), std::string::npos) << msg;
  }
}

TEST(TeleopNode, ConstructionErrorMissingReaderInGraph) {
  constexpr const char* kToml = R"(
[robot]
period_us = 20000

[subsystems.ghost]
period_us = 20000

[[subsystems.ghost.teleop.bindings]]
button = 1
while_held = { position_rot = 1.0 }

[subsystems.teleop]
period_us = 20000
)";
  Sim sim = MakeSim(kToml);
  // Do NOT add consumer node for "ghost"
  try {
    sim.add<Teleop>("teleop");
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("[subsystems.ghost] teleop: no reader of "
                       "'/ghost/target/teleop' in the graph"),
              std::string::npos)
        << msg;
  }
}

TEST(TeleopNode, ConstructionErrorMissingTeleopInTargetPolicy) {
  constexpr const char* kToml = R"(
[robot]
period_us = 20000

[subsystems.roller]
period_us = 20000

[subsystems.roller.targets]
teleop = ["auto"]

[[subsystems.roller.teleop.bindings]]
button = 1
while_held = { velocity_rps = 10.0 }

[subsystems.teleop]
period_us = 20000
)";
  Sim sim = MakeSim(kToml);
  sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  try {
    sim.add<Teleop>("teleop");
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("[subsystems.roller] teleop: 'teleop' missing from the "
                       "subsystem's teleop-mode target policy"),
              std::string::npos)
        << msg;
  }
}

}  // namespace
}  // namespace talos::teleop

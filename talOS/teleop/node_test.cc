#include "talOS/node/node.h"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

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

[subsystems.roller.teleop]
defaults = { feedforward_v = 0.0 }

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
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));

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

TEST(TeleopNode, PovAndToggleEdgesResetOnSafetyTransitions) {
  for (const std::string trigger : {"button = 1", "pov = 90\npov_index = 1"}) {
    for (const std::string action : {"while_held", "on_toggle"}) {
      SCOPED_TRACE(trigger + action);
      std::string config =
          "[robot]\nperiod_us = 20000\ncommand_timeout_us = 100000\n"
          "[subsystems.roller]\n[subsystems.roller.teleop]\n"
          "defaults = { feedforward_v = 0.0 }\n"
          "[[subsystems.roller.teleop.bindings]]\n";
      config.append(trigger);
      config.append("\n");
      config.append(action);
      config.append(" = { velocity_rps = 60.0 }\n[subsystems.teleop]\n");
      Sim sim = MakeSim(config);
      auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
      sim.add<Teleop>("teleop");
      SimHardware hw{sim};
      hw.enable();
      Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
      auto packet = [&](bool pressed, std::uint32_t flags = kTeleopEnabled,
                        bool connected = true) {
        auto stick = MakeStick(0, 0, 0, pressed ? 1 : 0, connected);
        stick.pov_count = 2;
        stick.povs[0] = -1;
        stick.povs[1] = pressed ? 90 : -1;
        Push(hw, sim, DsPacket(flags, stick));
      };
      packet(false);
      EXPECT_FALSE(roller.seen);
      packet(true);
      if (!roller.seen.has_value()) {
        FAIL() << "Expected teleop target";
      }
      const auto& pressed = roller.seen.value();
      EXPECT_DOUBLE_EQ(pressed.velocity_rps(), 60.0);
      packet(true);  // Holding never generates another rising edge.
      EXPECT_TRUE(roller.seen);
      packet(false);
      EXPECT_EQ(roller.seen.has_value(), action == "on_toggle");
      packet(true);
      EXPECT_EQ(roller.seen.has_value(), action == "while_held");
      packet(false);
      packet(true);  // Latch back on.
      EXPECT_TRUE(roller.seen);
      for (const auto flags :
           {ds::kDsAttached | ds::kTeleop,
            ds::kDsAttached | ds::kEnabled | ds::kAutonomous}) {
        packet(false, flags);
        EXPECT_FALSE(roller.seen);
        packet(false);
        EXPECT_FALSE(roller.seen);  // No stale toggle on re-enable.
        packet(true);
        EXPECT_TRUE(roller.seen);
      }
      packet(false, kTeleopEnabled, false);
      EXPECT_FALSE(roller.seen);
      packet(false);
      EXPECT_FALSE(roller.seen);
      packet(true);
      EXPECT_TRUE(roller.seen);
      hw.stop_driver_station();
      sim.step(300ms);
      EXPECT_FALSE(roller.seen);
      packet(false);
      EXPECT_FALSE(roller.seen);
    }
  }
}

constexpr const char* kRichToml = R"(
[robot]
period_us = 20000
command_timeout_us = 100000

[subsystems.mechanism]
period_us = 20000

[subsystems.mechanism.teleop]
defaults = { sample_count = 0, speed_frac = 0.0 }

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
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));

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

[subsystems.roller.teleop]
defaults = { feedforward_v = 0.0 }

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
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));

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

[subsystems.arm.teleop]
defaults = { feedforward_v = 0.0 }

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
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));

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

TEST(TeleopNode, OverflowingAxisOutputReleasesEntireLaneAndRecovers) {
  for (const std::string field : {"inner/offset_m", "position_m"}) {
    SCOPED_TRACE(field);
    const std::string scale = field == "position_m" ? "1e308" : "1e40";
    const std::string config =
        "[robot]\nperiod_us = 20000\ncommand_timeout_us = 100000\n"
        "[subsystems.mechanism]\n"
        "[subsystems.mechanism.teleop]\n"
        "defaults = { flag = false, position_m = 0.0, level = 'NONE', "
        "sample_count = 0, speed_frac = 0.0, "
        "'inner/mode' = 'OFF', 'inner/offset_m' = 0.0 }\n"
        "[[subsystems.mechanism.teleop.bindings]]\n"
        "button = 1\nwhile_held = { flag = true }\n"
        "[[subsystems.mechanism.teleop.bindings]]\n"
        "axis = 0\nfield = \"" +
        field + "\"\nscale = " + scale + "\n[subsystems.teleop]\n";
    Sim sim = MakeSim(config);
    auto& mechanism = sim.add<ConsumerNode<schema_test::Mixed>>("mechanism");
    sim.add<Teleop>("teleop");
    SimHardware hw{sim};
    hw.enable();
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));

    auto push = [&](float axis, std::uint32_t flags = kTeleopEnabled) {
      Push(hw, sim, DsPacket(flags, MakeStick(axis, 0, 0, 1)));
    };
    push(0.001f);
    ASSERT_TRUE(mechanism.seen);
    EXPECT_TRUE(mechanism.seen->flag());
    EXPECT_TRUE(std::isfinite(mechanism.seen->position_m()));
    EXPECT_TRUE(std::isfinite(mechanism.seen->inner().offset_m()));
    EXPECT_GT(field == "position_m" ? mechanism.seen->position_m()
                                    : mechanism.seen->inner().offset_m(),
              0.0);

    // Float narrowing and double multiplication overflow both fail closed,
    // even though the button binding is still live.
    push(4.0f);
    EXPECT_FALSE(mechanism.seen);
    push(0.001f);
    EXPECT_TRUE(mechanism.seen);
    push(0.001f, ds::kTeleop | ds::kDsAttached);
    EXPECT_FALSE(mechanism.seen);
    push(0.001f);
    EXPECT_FALSE(mechanism.seen);
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
    push(0.001f);
    EXPECT_TRUE(mechanism.seen);
    hw.stop_driver_station();
    sim.step(150ms);
    EXPECT_FALSE(mechanism.seen);
  }
}

TEST(TeleopNode, DisabledRobotReleasesOnceAndSendsNoFurtherMessages) {
  Sim sim = MakeSim(kRollerToml);
  auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));

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
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));

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
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));

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

  // Reconnecting with the button already held is not fresh input.
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 1)));
  EXPECT_FALSE(roller.seen.has_value());
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 1)));
  EXPECT_TRUE(roller.seen.has_value());
}

TEST(TeleopNode, AbsentSpareControllerDoesNotBlockRearm) {
  std::string config{kRollerToml};
  const std::string marker{"[subsystems.teleop]"};
  config.insert(config.find(marker),
                "[[subsystems.roller.teleop.bindings]]\nport = 1\nbutton = 2\n"
                "while_held = { velocity_rps = 5.0 }\n\n");
  Sim sim = MakeSim(config);
  auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 1)));
  ASSERT_TRUE(roller.seen.has_value());
  EXPECT_DOUBLE_EQ(roller.seen->velocity_rps(), 60.0);
}

TEST(TeleopNode, StartupHeldTriggerDoesNotCommandOrSynthesizeToggle) {
  for (const std::string action : {"while_held", "on_toggle"}) {
    std::string config{kRollerToml};
    config.replace(config.find("while_held"), std::string{"while_held"}.size(),
                   action);
    Sim sim = MakeSim(config);
    auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
    sim.add<Teleop>("teleop");
    SimHardware hw{sim};
    hw.enable();
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 1)));
    EXPECT_FALSE(roller.seen);
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
    EXPECT_FALSE(roller.seen);
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 1)));
    ASSERT_TRUE(roller.seen);
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
    EXPECT_EQ(roller.seen.has_value(), action == "on_toggle");
  }
}

TEST(TeleopNode, BoundPovMustBeReleasedBeforeArmingEvenFromAnotherDirection) {
  for (const std::string action : {"while_held", "on_toggle"}) {
    std::string config{kRollerToml};
    config.replace(config.find("button = 1"), std::string{"button = 1"}.size(),
                   "pov = 90");
    config.replace(config.find("while_held"), std::string{"while_held"}.size(),
                   action);
    Sim sim = MakeSim(config);
    auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
    sim.add<Teleop>("teleop");
    SimHardware hw{sim};
    hw.enable();
    const auto packet = [&](int angle) {
      auto stick = MakeStick();
      stick.pov_count = 1;
      stick.povs[0] = angle;
      Push(hw, sim, DsPacket(kTeleopEnabled, stick));
    };
    packet(0);  // A different direction is still not a released POV.
    EXPECT_FALSE(roller.seen);
    packet(90);
    EXPECT_FALSE(roller.seen);
    packet(-1);
    EXPECT_FALSE(roller.seen);
    packet(90);
    ASSERT_TRUE(roller.seen);
    EXPECT_DOUBLE_EQ(roller.seen->velocity_rps(), 60.0);
  }
}

TEST(TeleopNode, HeldAxisMustReturnToNeutralAfterDsLossOrDisable) {
  Sim sim = MakeSim(kAxisToml);
  auto& arm = sim.add<ConsumerNode<control::VelocityTarget>>("arm");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1)));
  EXPECT_FALSE(arm.seen);
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1)));
  ASSERT_TRUE(arm.seen);
  for (const bool stale : {false, true}) {
    if (stale) {
      hw.stop_driver_station();
      sim.step(150ms);
    } else {
      Push(hw, sim, DsPacket(ds::kTeleop | ds::kDsAttached, MakeStick(1)));
    }
    EXPECT_FALSE(arm.seen);
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1)));
    EXPECT_FALSE(arm.seen);
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
    EXPECT_FALSE(arm.seen);
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1)));
    ASSERT_TRUE(arm.seen);
  }
}

struct PausableTeleop {
  explicit PausableTeleop(talos::Node& node) : teleop{node} {}
  void update() {
    if (!paused) teleop.update();
  }
  Teleop teleop;
  bool paused{false};
};

TEST(TeleopNode, OwnEvaluationGapRequiresNeutralEvenWithFreshDs) {
  Sim sim = MakeSim(kAxisToml);
  auto& arm = sim.add<ConsumerNode<control::VelocityTarget>>("arm");
  auto& teleop = sim.add<PausableTeleop>("teleop");
  SimHardware hw{sim};
  hw.enable();
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1)));
  ASSERT_TRUE(arm.seen);
  teleop.paused = true;
  sim.step(150ms);
  teleop.paused = false;
  sim.step(60ms);
  EXPECT_FALSE(arm.seen);
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1)));
  ASSERT_TRUE(arm.seen);
}

// A disable shorter than one teleop period (a radio blip) is drained together
// with the re-enable, so no update() samples it; the new mode generation alone
// must disarm a held stick until it returns to neutral.
TEST(TeleopNode, UnsampledDisableRequiresNeutralInTheNewGeneration) {
  Sim sim = MakeSim(kAxisToml);
  auto& arm = sim.add<ConsumerNode<control::VelocityTarget>>("arm");
  auto& teleop = sim.add<PausableTeleop>("teleop");
  SimHardware hw{sim};
  hw.enable();
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1)));
  ASSERT_TRUE(arm.seen);
  // Two skipped evaluations stay well inside command_timeout_us, so only the
  // generation can disarm.
  teleop.paused = true;
  Push(hw, sim, DsPacket(ds::kTeleop | ds::kDsAttached, MakeStick(1)), 20ms);
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1)), 20ms);
  teleop.paused = false;
  sim.step(60ms);
  EXPECT_FALSE(arm.seen);
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1)));
  ASSERT_TRUE(arm.seen);
}

constexpr const char* kPartialToml = R"(
[robot]
period_us = 20000
[subsystems.mechanism.teleop]
defaults = { flag = false, position_m = 2.0, level = "BOTTOM", sample_count = 7, speed_frac = 0.5, "inner/mode" = "LOW", "inner/offset_m" = 1.25 }
[[subsystems.mechanism.teleop.bindings]]
button = 1
while_held = { position_m = 3.5 }
[subsystems.teleop]
)";

TEST(TeleopNode, PartialBindingPublishesExplicitDefaultsIncludingNestedFields) {
  Sim sim = MakeSim(kPartialToml);
  auto& mechanism = sim.add<ConsumerNode<schema_test::Mixed>>("mechanism");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
  EXPECT_FALSE(mechanism.seen);
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 1)));
  ASSERT_TRUE(mechanism.seen);
  EXPECT_DOUBLE_EQ(mechanism.seen->position_m(), 3.5);
  EXPECT_FALSE(mechanism.seen->flag());
  EXPECT_EQ(mechanism.seen->level(), schema_test::Level::BOTTOM);
  EXPECT_EQ(mechanism.seen->sample_count(), 7u);
  EXPECT_FLOAT_EQ(mechanism.seen->speed_frac(), 0.5f);
  EXPECT_EQ(mechanism.seen->inner().mode(), schema_test::Mode::LOW);
  EXPECT_FLOAT_EQ(mechanism.seen->inner().offset_m(), 1.25f);
}

TEST(TeleopNode, MissingDefaultNamesLaneAndUnassignedField) {
  std::string config{kPartialToml};
  const std::string assignment{", \"inner/offset_m\" = 1.25"};
  config.erase(config.find(assignment), assignment.size());
  Sim sim = MakeSim(config);
  sim.add<ConsumerNode<schema_test::Mixed>>("mechanism");
  EXPECT_THROW(sim.add<Teleop>("teleop"), std::invalid_argument);
}

TEST(TeleopNode, IdleAxisCannotCoverAFieldNeededByAnotherLiveBinding) {
  std::string config{kRollerToml};
  config.insert(config.find("[subsystems.teleop]"),
                "[[subsystems.roller.teleop.bindings]]\naxis = 0\n"
                "field = 'feedforward_v'\nscale = 2.0\n\n");
  Sim sim = MakeSim(config);
  sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  EXPECT_THROW(sim.add<Teleop>("teleop"), std::invalid_argument);
}

TEST(TeleopNode, DefaultsCoverEachIndependentLiveBindingCombination) {
  std::string config{kRollerToml};
  const std::string defaults{"defaults = { feedforward_v = 0.0 }"};
  config.replace(config.find(defaults), defaults.size(),
                 "defaults = { velocity_rps = 0.0, feedforward_v = 1.5 }");
  config.insert(config.find("[subsystems.teleop]"),
                "[[subsystems.roller.teleop.bindings]]\naxis = 0\n"
                "field = 'feedforward_v'\nscale = 2.0\n\n");
  Sim sim = MakeSim(config);
  auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 1)));
  ASSERT_TRUE(roller.seen);
  EXPECT_DOUBLE_EQ(roller.seen->velocity_rps(), 60.0);
  EXPECT_DOUBLE_EQ(roller.seen->feedforward_v(), 1.5);
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1)));
  ASSERT_TRUE(roller.seen);
  EXPECT_DOUBLE_EQ(roller.seen->velocity_rps(), 0.0);
  EXPECT_DOUBLE_EQ(roller.seen->feedforward_v(), 2.0);
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1, 0, 0, 1)));
  ASSERT_TRUE(roller.seen);
  EXPECT_DOUBLE_EQ(roller.seen->velocity_rps(), 60.0);
  EXPECT_DOUBLE_EQ(roller.seen->feedforward_v(), 2.0);
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
  EXPECT_FALSE(roller.seen);
}

TEST(TeleopNode, DefaultsShareAssignmentSchemaValidation) {
  constexpr std::pair<std::string_view, std::string_view> kInvalidDefaults[]{
      {"position_m = 2.0", "position_m = 'fast'"},
      {"position_m = 2.0", "position_m = nan"},
      {"position_m = 2.0", "position_m = inf"},
      {"speed_frac = 0.5", "speed_frac = 1e40"},
      {"sample_count = 7", "sample_count = -1"},
      {"flag = false", "flag = 1"},
      {"level = \"BOTTOM\"", "level = 'UNKNOWN'"},
      {"\"inner/mode\" = \"LOW\"", "\"inner/mode\" = 0"},
  };
  for (const auto& [assignment, replacement] : kInvalidDefaults) {
    SCOPED_TRACE(replacement);
    std::string config{kPartialToml};
    config.replace(config.find(assignment), assignment.size(), replacement);
    Sim sim = MakeSim(config);
    sim.add<ConsumerNode<schema_test::Mixed>>("mechanism");
    EXPECT_THROW(sim.add<Teleop>("teleop"), std::invalid_argument);
  }
}

// --- Construction error tests ---

TEST(TeleopNode, ConstructionRejectsInvalidFloatingPointButtonAssignments) {
  for (const std::string action : {"while_held", "on_toggle"}) {
    for (const std::string field : {"position_m", "inner/offset_m"}) {
      for (const std::string value : {"nan", "inf", "-inf", "1e40", "-1e40"}) {
        if (field == "position_m" && (value == "1e40" || value == "-1e40")) {
          continue;  // These values fit a double.
        }
        SCOPED_TRACE(action + "." + field + " = " + value);
        const std::string config =
            "[robot]\nperiod_us = 20000\n[subsystems.mechanism]\n"
            "[[subsystems.mechanism.teleop.bindings]]\nbutton = 1\n" +
            action + " = { \"" + field + "\" = " + value +
            " }\n[subsystems.teleop]\n";
        Sim sim = MakeSim(config);
        sim.add<ConsumerNode<schema_test::Mixed>>("mechanism");
        EXPECT_THROW(sim.add<Teleop>("teleop"), std::invalid_argument);
      }
    }
  }
}

TEST(TeleopNode, FloatingPointEncodingAcceptsFiniteLimitsWithoutOverflow) {
  using reflection::BaseType;
  std::array<std::byte, 8> bytes{};
  const double float_max = std::numeric_limits<float>::max();
  for (const double value : {float_max, -float_max}) {
    EXPECT_NO_THROW(
        detail::EncodeNumericBytes(bytes.data(), BaseType::Float, value));
    float decoded{};
    std::memcpy(&decoded, bytes.data(), sizeof(decoded));
    EXPECT_TRUE(std::isfinite(decoded));
    EXPECT_DOUBLE_EQ(decoded, value);
  }
  EXPECT_THROW(
      detail::EncodeNumericBytes(
          bytes.data(), BaseType::Float,
          std::nextafter(float_max, std::numeric_limits<double>::infinity())),
      std::invalid_argument);
  EXPECT_NO_THROW(detail::EncodeNumericBytes(
      bytes.data(), BaseType::Double, std::numeric_limits<double>::max()));
}

TEST(TeleopNode, ConstructionRejectsNonFiniteAxisParameters) {
  for (const std::string parameter : {"scale", "deadband_frac"}) {
    for (const std::string value : {"nan", "inf", "-inf"}) {
      SCOPED_TRACE(parameter + " = " + value);
      std::string config{kAxisToml};
      const std::string old_value =
          parameter == "scale" ? "scale = 10.0" : "deadband_frac = 0.125";
      config.replace(config.find(old_value), old_value.size(),
                     parameter + " = " + value);
      // Some parameter parsers reject these even before node construction.
      EXPECT_THROW(
          {
            Sim sim = MakeSim(config);
            sim.add<ConsumerNode<control::VelocityTarget>>("arm");
            sim.add<Teleop>("teleop");
          },
          std::invalid_argument);
    }
  }
}

TEST(TeleopNode, ConstructionRejectsAxisBoundToInteger) {
  const std::string config = R"(
[robot]
period_us = 20000
[subsystems.mechanism]
period_us = 20000
[[subsystems.mechanism.teleop.bindings]]
axis = 0
field = "sample_count"
scale = 1.0
[subsystems.teleop]
period_us = 20000
)";
  Sim sim = MakeSim(config);
  sim.add<ConsumerNode<schema_test::Mixed>>("mechanism");
  EXPECT_THROW(sim.add<Teleop>("teleop"), std::invalid_argument);
}

TEST(TeleopNode, NumericIntegerEncodingChecksBoundsBeforeConversion) {
  using reflection::BaseType;
  std::array<std::byte, 8> bytes{};
  for (const double value :
       {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), std::ldexp(1.0, 63),
        std::nextafter(-std::ldexp(1.0, 63),
                       -std::numeric_limits<double>::infinity())}) {
    EXPECT_THROW(
        detail::EncodeNumericBytes(bytes.data(), BaseType::Long, value),
        std::invalid_argument);
  }
  EXPECT_NO_THROW(detail::EncodeNumericBytes(bytes.data(), BaseType::Long,
                                             -std::ldexp(1.0, 63)));
  std::int64_t signed_value{};
  std::memcpy(&signed_value, bytes.data(), sizeof(signed_value));
  EXPECT_EQ(signed_value, std::numeric_limits<std::int64_t>::min());
  const double largest_signed = std::nextafter(std::ldexp(1.0, 63), 0.0);
  EXPECT_NO_THROW(
      detail::EncodeNumericBytes(bytes.data(), BaseType::Long, largest_signed));
  EXPECT_THROW(detail::EncodeNumericBytes(bytes.data(), BaseType::ULong,
                                          std::ldexp(1.0, 64)),
               std::invalid_argument);
  EXPECT_THROW(detail::EncodeNumericBytes(bytes.data(), BaseType::ULong, -1.0),
               std::invalid_argument);
  const double largest_unsigned = std::nextafter(std::ldexp(1.0, 64), 0.0);
  EXPECT_NO_THROW(detail::EncodeNumericBytes(bytes.data(), BaseType::ULong,
                                             largest_unsigned));
  std::uint64_t unsigned_value{};
  std::memcpy(&unsigned_value, bytes.data(), sizeof(unsigned_value));
  EXPECT_EQ(unsigned_value, static_cast<std::uint64_t>(largest_unsigned));
  EXPECT_THROW(detail::EncodeNumericBytes(bytes.data(), BaseType::Byte, 128.0),
               std::invalid_argument);
  EXPECT_THROW(detail::EncodeNumericBytes(bytes.data(), BaseType::UInt, -1.0),
               std::invalid_argument);
}

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
  EXPECT_THROW(sim.add<Teleop>("teleop"), std::invalid_argument);
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
  EXPECT_THROW(sim.add<Teleop>("teleop"), std::invalid_argument);
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
  EXPECT_THROW(sim.add<Teleop>("teleop"), std::invalid_argument);
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
  EXPECT_THROW(sim1.add<Teleop>("teleop"), std::invalid_argument);

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
  EXPECT_THROW(sim2.add<Teleop>("teleop"), std::invalid_argument);
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
  EXPECT_THROW(sim.add<Teleop>("teleop"), std::invalid_argument);
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
  EXPECT_THROW(sim.add<Teleop>("teleop"), std::invalid_argument);
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
  EXPECT_THROW(sim.add<Teleop>("teleop"), std::invalid_argument);
}

}  // namespace
}  // namespace talos::teleop

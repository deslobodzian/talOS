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
#include "talOS/node/allocation_probe.h"
#include "talOS/node/testing.h"
#include "talOS/schema/field_path.h"
#include "talOS/teleop/node.h"
#include "teleop/status_generated.h"
#include "teleop/topics.h"

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

// The node test matrix for a producer, docs/TESTING.md §7: the case names are
// the matrix's, which //tools:check_node_matrix looks for. The producer cases
// assert the lane's own envelope (active, released) besides what a consumer's
// Target accepts, since a consumer rejects a disabled or stale lane whatever
// the producer sends.

// The roller's teleop lane as the producer last wrote it.
std::optional<talos::TargetEnvelope<control::VelocityTarget>> RollerLane(
    const Sim& sim) {
  return talos::testing::LatestTarget<control::VelocityTarget>(sim, "roller",
                                                               "teleop");
}

// REQ-CONTRACT-24
TEST(TeleopNode, SendsAtALiveInput) {
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
  ASSERT_TRUE(RollerLane(sim).has_value());
  EXPECT_EQ(RollerLane(sim)->active, 1);
  EXPECT_EQ(RollerLane(sim)->mode_since_us(),
            talos::testing::DriverStationGeneration(sim));

  // Release button 1
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0, 0, 0, 0)));
  EXPECT_FALSE(roller.seen.has_value());
  EXPECT_EQ(RollerLane(sim)->active, 0);
}

// REQ-CONTRACT-24
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

// REQ-RT-06: with a held axis, each tick's binding evaluation, field
// encoding and lane write allocate nothing (nor does the consumer's Target).
TEST(TeleopNode, UpdateDoesNotAllocate) {
  if (const auto why = talos::testing::AllocationProbe::Unsupported();
      !why.empty()) {
    GTEST_SKIP() << why;
  }
  Sim sim = MakeSim(kAxisToml);
  auto& arm = sim.add<ConsumerNode<control::VelocityTarget>>("arm");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(-0.5625f)));
  ASSERT_TRUE(arm.seen.has_value());
  talos::testing::AllocationProbe probe;
  probe.start();
  sim.step(100ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  ASSERT_TRUE(arm.seen.has_value());
  EXPECT_NEAR(arm.seen->velocity_rps(), -5.0, 1e-6);
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
    // Just past the default 0.05 deadband: a small output that fits.
    constexpr float kSmall = 0.06f;
    push(kSmall);
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
    push(kSmall);
    EXPECT_TRUE(mechanism.seen);
    push(kSmall, ds::kTeleop | ds::kDsAttached);
    EXPECT_FALSE(mechanism.seen);
    push(kSmall);
    EXPECT_FALSE(mechanism.seen);
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
    push(kSmall);
    EXPECT_TRUE(mechanism.seen);
    hw.stop_driver_station();
    sim.step(150ms);
    EXPECT_FALSE(mechanism.seen);
  }
}

// REQ-SAFE-13: disabled with the button still held, the lane is released
// once and nothing more is written.
TEST(TeleopNode, DisabledRobotReleasesItsLanes) {
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
  EXPECT_EQ(RollerLane(sim)->active, 0);
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

// REQ-SAFE-13
TEST(TeleopNode, DriverStationLossReleasesItsLanes) {
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
  EXPECT_EQ(RollerLane(sim)->active, 0);
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

// REQ-SAFE-31: started with a trigger held, teleop sends no target until it
// is released
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
      stick.povs[0] = static_cast<std::int16_t>(angle);
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

// REQ-SAFE-14
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

// kAxisToml with its axis binding's deadband_frac set to `frac`, or left out
// when `frac` is empty.
std::string AxisTomlWithDeadband(std::string_view frac) {
  std::string config{kAxisToml};
  const std::string old_line{"deadband_frac = 0.125\n"};
  config.replace(config.find(old_line), old_line.size(),
                 frac.empty() ? std::string{}
                              : "deadband_frac = " + std::string{frac} + "\n");
  return config;
}

// REQ-CONTRACT-24, REQ-SAFE-14: after a disable with the axis held, the lane
// rearms only once the axis returns inside its deadband_frac (0.05 when the
// binding states none; inclusive), and an axis resting inside the band that
// armed it commands nothing on the armed ticks that follow, since the
// arming-neutral band is the output dead zone. The arming evaluation itself
// never sends (talos::OperatorArming arms for the evaluations after it), so
// the judged window must hold the first armed ticks and the consumer's view
// of them.
TEST(TeleopNode, HeldAxisRearmsInsideItsDeadbandAndSendsNothingThere) {
  struct Row {
    const char* deadband;
    float rest;
    bool arms;
  };
  constexpr std::uint32_t kTeleopDisabled = ds::kTeleop | ds::kDsAttached;
  // The window the resting stick is held and judged for, in 20 ms periods
  // after the rest reading is set at R. Teleop runs after the consumer and
  // before SimHardware at each instant, so: R+20 teleop still reads the held
  // stick, then the rest reading is published; R+40 the arming evaluation
  // reads it and arms, sending nothing; R+60 the first armed tick, R+80 the
  // second; the consumer sees them at R+80 and R+100. The window runs to
  // R+200, so it holds all of that whatever the same-instant order.
  constexpr int kWindowPeriods = 10;
  // The upper edge: Driver Station axes are floats, compared as doubles.
  // 0.0625f is exact, so it pins the inclusive edge; 0.05f is
  // 0.0500000007450580596923828125, about 7.5e-10 above the double 0.05, so
  // a reading of exactly 0.05f lies outside a 0.05 deadband and never arms.
  for (const Row& row :
       {Row{"", 0.04f, true}, Row{"", 0.06f, false}, Row{"0.05", 0.0499f, true},
        Row{"0.05", 0.05f, false}, Row{"0.05", 0.0501f, false},
        Row{"0.0625", 0.0625f, true}, Row{"0.2", 0.15f, true},
        Row{"0.2", 0.25f, false}}) {
    SCOPED_TRACE(std::string{"deadband_frac '"} + row.deadband +
                 "', axis returns to " + std::to_string(row.rest));
    Sim sim = MakeSim(AxisTomlWithDeadband(row.deadband));
    auto& arm = sim.add<ConsumerNode<control::VelocityTarget>>("arm");
    sim.add<Teleop>("teleop");
    SimHardware hw{sim};
    hw.enable();
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0.5f)));
    ASSERT_TRUE(arm.seen);
    Push(hw, sim, DsPacket(kTeleopDisabled, MakeStick(0.5f)));
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0.5f)));
    EXPECT_FALSE(arm.seen) << "held at 0.5 through a disable";
    hw.set_driver_station(DsPacket(kTeleopEnabled, MakeStick(row.rest)));
    // The first period of the window (0: none) in which the lane went
    // active, and in which the consumer got a target, with what each carried.
    int lane_active_period = 0;
    double lane_rps = 0.0;
    int consumer_period = 0;
    double consumer_rps = 0.0;
    for (int period = 1; period <= kWindowPeriods; ++period) {
      sim.step(20ms);
      const auto lane = talos::testing::LatestTarget<control::VelocityTarget>(
          sim, "arm", "teleop");
      if (lane_active_period == 0 && lane && lane->active != 0) {
        lane_active_period = period;
        lane_rps = lane->message.velocity_rps();
      }
      if (consumer_period == 0 && arm.seen) {
        consumer_period = period;
        consumer_rps = arm.seen->velocity_rps();
      }
    }
    EXPECT_EQ(lane_active_period, 0)
        << "resting at " << row.rest << ", the lane went active at R+"
        << 20 * lane_active_period << " ms with " << lane_rps << " rps";
    EXPECT_EQ(consumer_period, 0)
        << "resting at " << row.rest << ", the consumer got " << consumer_rps
        << " rps at R+" << 20 * consumer_period << " ms";
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0.5f)));
    EXPECT_EQ(arm.seen.has_value(), row.arms);
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

// REQ-SAFE-14
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

// REQ-SAFE-14: gateway reboot (boot_id change), Driver Station dropout, and
// producer evaluation stall each disarm teleop and keep every lane inactive
// until the held axis, button (while_held / on_toggle), or POV input returns
// to neutral.
TEST(TeleopNode, HeldInputsWaitForNeutralAcrossRebootDsDropoutAndStall) {
  struct InputSpec {
    const char* name;
    const char* binding_toml;
    bool is_toggle;
    bool is_pov;
    ds::JoystickData held;
    ds::JoystickData alt_non_neutral;
  };
  const auto neutral_stick = [] {
    auto s = MakeStick();
    s.pov_count = 2;
    s.povs[0] = -1;
    s.povs[1] = -1;
    return s;
  };
  auto held_axis = neutral_stick();
  held_axis.axes[0] = 1.0f;
  auto held_button = neutral_stick();
  held_button.buttons = 1u;
  auto held_pov = neutral_stick();
  held_pov.povs[1] = 90;
  auto alt_pov = neutral_stick();
  alt_pov.povs[1] = 180;

  const InputSpec inputs[] = {
      {"axis",
       "axis = 0\nfield = \"velocity_rps\"\nscale = 10.0\n"
       "deadband_frac = 0.125\n",
       false, false, held_axis, neutral_stick()},
      {"button while_held",
       "button = 1\nwhile_held = { velocity_rps = 60.0 }\n", false, false,
       held_button, neutral_stick()},
      {"button on_toggle", "button = 1\non_toggle = { velocity_rps = 60.0 }\n",
       true, false, held_button, neutral_stick()},
      {"pov while_held",
       "pov = 90\npov_index = 1\nwhile_held = { velocity_rps = 60.0 }\n", false,
       true, held_pov, alt_pov},
      {"pov on_toggle",
       "pov = 90\npov_index = 1\non_toggle = { velocity_rps = 60.0 }\n", true,
       true, held_pov, alt_pov},
  };
  enum class Transition { kGatewayReboot, kDsDropout, kProducerStall };
  const std::pair<const char*, Transition> transitions[] = {
      {"gateway reboot", Transition::kGatewayReboot},
      {"ds dropout", Transition::kDsDropout},
      {"producer stall", Transition::kProducerStall},
  };

  for (const InputSpec& input : inputs) {
    for (const auto& [transition_name, transition] : transitions) {
      SCOPED_TRACE(std::string{input.name} + " across " + transition_name);
      const std::string config =
          "[robot]\nperiod_us = 20000\ncommand_timeout_us = 100000\n"
          "[subsystems.roller]\nperiod_us = 20000\n"
          "[subsystems.roller.teleop]\n"
          "defaults = { feedforward_v = 0.0 }\n"
          "[[subsystems.roller.teleop.bindings]]\n" +
          std::string{input.binding_toml} +
          "[subsystems.teleop]\nperiod_us = 20000\n";
      Sim sim = MakeSim(config);
      auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
      auto& teleop = sim.add<PausableTeleop>("teleop");
      SimHardware hw{sim};
      hw.enable();

      const auto latest_status = [&sim] {
        return sim.latest<::teleop::TeleopStatus>(::teleop::topics::kStatus);
      };

      // Arm at neutral, then hold the input and confirm the lane is active.
      Push(hw, sim, DsPacket(kTeleopEnabled, neutral_stick()));
      Push(hw, sim, DsPacket(kTeleopEnabled, input.held));
      ASSERT_TRUE(roller.seen.has_value());
      ASSERT_TRUE(RollerLane(sim).has_value());
      EXPECT_EQ(RollerLane(sim)->active, 1);
      ASSERT_TRUE(latest_status().has_value());
      EXPECT_TRUE(latest_status()->armed());

      switch (transition) {
        case Transition::kGatewayReboot:
          hw.reboot();
          sim.step(60ms);
          break;
        case Transition::kDsDropout:
          hw.stop_driver_station();
          sim.step(150ms);
          EXPECT_FALSE(roller.seen.has_value());
          ASSERT_TRUE(RollerLane(sim).has_value());
          EXPECT_EQ(RollerLane(sim)->active, 0);
          Push(hw, sim, DsPacket(kTeleopEnabled, input.held));
          break;
        case Transition::kProducerStall:
          teleop.paused = true;
          sim.step(150ms);
          teleop.paused = false;
          sim.step(60ms);
          break;
      }

      // Still held: teleop must stay disarmed and keep the target lane
      // inactive.
      EXPECT_FALSE(roller.seen.has_value());
      ASSERT_TRUE(RollerLane(sim).has_value());
      EXPECT_EQ(RollerLane(sim)->active, 0);
      ASSERT_TRUE(latest_status().has_value());
      EXPECT_FALSE(latest_status()->armed());
      EXPECT_TRUE(latest_status()->alert_warning_disarmed());

      // Continuing to hold the input never rearms.
      Push(hw, sim, DsPacket(kTeleopEnabled, input.held));
      EXPECT_FALSE(roller.seen.has_value());
      EXPECT_EQ(RollerLane(sim)->active, 0);
      EXPECT_FALSE(latest_status()->armed());

      // For POVs, moving to a different non-neutral angle does not rearm.
      if (input.is_pov) {
        Push(hw, sim, DsPacket(kTeleopEnabled, input.alt_non_neutral));
        EXPECT_FALSE(roller.seen.has_value());
        EXPECT_EQ(RollerLane(sim)->active, 0);
        EXPECT_FALSE(latest_status()->armed());
        Push(hw, sim, DsPacket(kTeleopEnabled, input.held));
        EXPECT_FALSE(roller.seen.has_value());
        EXPECT_EQ(RollerLane(sim)->active, 0);
        EXPECT_FALSE(latest_status()->armed());
      }

      // Returning to neutral rearms without commanding or synthesizing a
      // toggle.
      Push(hw, sim, DsPacket(kTeleopEnabled, neutral_stick()));
      EXPECT_FALSE(roller.seen.has_value());
      EXPECT_EQ(RollerLane(sim)->active, 0);
      ASSERT_TRUE(latest_status().has_value());
      EXPECT_TRUE(latest_status()->armed());
      EXPECT_FALSE(latest_status()->alert_warning_disarmed());

      // Fresh input after neutral drives the lane again.
      Push(hw, sim, DsPacket(kTeleopEnabled, input.held));
      ASSERT_TRUE(roller.seen.has_value());
      EXPECT_EQ(RollerLane(sim)->active, 1);

      // For on_toggle, releasing after the fresh press keeps it latched.
      if (input.is_toggle) {
        Push(hw, sim, DsPacket(kTeleopEnabled, neutral_stick()));
        EXPECT_TRUE(roller.seen.has_value());
        EXPECT_EQ(RollerLane(sim)->active, 1);
      }
    }
  }
}

// REQ-SAFE-14: an on_toggle binding latched on and then released must clear its
// latch and release the target lane across gateway reboot, DS dropout, and
// producer stall, staying inactive until freshly pressed again.
TEST(TeleopNode, LatchedToggleClearsAcrossRebootDsDropoutAndStallWhenReleased) {
  const auto neutral_stick = [] {
    auto s = MakeStick();
    s.pov_count = 2;
    s.povs[0] = -1;
    s.povs[1] = -1;
    return s;
  };
  auto pressed_button = neutral_stick();
  pressed_button.buttons = 1u;
  auto pressed_pov = neutral_stick();
  pressed_pov.povs[1] = 90;

  struct TriggerSpec {
    const char* name;
    const char* binding_toml;
    ds::JoystickData pressed;
  };
  const TriggerSpec triggers[] = {
      {"button", "button = 1\non_toggle = { velocity_rps = 60.0 }\n",
       pressed_button},
      {"pov", "pov = 90\npov_index = 1\non_toggle = { velocity_rps = 60.0 }\n",
       pressed_pov},
  };
  enum class Transition { kGatewayReboot, kDsDropout, kProducerStall };
  const std::pair<const char*, Transition> transitions[] = {
      {"gateway reboot", Transition::kGatewayReboot},
      {"ds dropout", Transition::kDsDropout},
      {"producer stall", Transition::kProducerStall},
  };

  for (const TriggerSpec& trigger : triggers) {
    for (const auto& [transition_name, transition] : transitions) {
      SCOPED_TRACE(std::string{trigger.name} + " across " + transition_name);
      const std::string config =
          "[robot]\nperiod_us = 20000\ncommand_timeout_us = 100000\n"
          "[subsystems.roller]\nperiod_us = 20000\n"
          "[subsystems.roller.teleop]\n"
          "defaults = { feedforward_v = 0.0 }\n"
          "[[subsystems.roller.teleop.bindings]]\n" +
          std::string{trigger.binding_toml} +
          "[subsystems.teleop]\nperiod_us = 20000\n";
      Sim sim = MakeSim(config);
      auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
      auto& teleop = sim.add<PausableTeleop>("teleop");
      SimHardware hw{sim};
      hw.enable();

      // Arm, press to latch on, then release trigger while staying latched.
      Push(hw, sim, DsPacket(kTeleopEnabled, neutral_stick()));
      Push(hw, sim, DsPacket(kTeleopEnabled, trigger.pressed));
      Push(hw, sim, DsPacket(kTeleopEnabled, neutral_stick()));
      ASSERT_TRUE(roller.seen.has_value());
      ASSERT_TRUE(RollerLane(sim).has_value());
      EXPECT_EQ(RollerLane(sim)->active, 1);

      switch (transition) {
        case Transition::kGatewayReboot:
          hw.reboot();
          sim.step(60ms);
          break;
        case Transition::kDsDropout:
          hw.stop_driver_station();
          sim.step(150ms);
          Push(hw, sim, DsPacket(kTeleopEnabled, neutral_stick()));
          break;
        case Transition::kProducerStall:
          teleop.paused = true;
          sim.step(150ms);
          teleop.paused = false;
          sim.step(60ms);
          break;
      }

      // Latch must be cleared and the lane inactive until pressed again.
      EXPECT_FALSE(roller.seen.has_value());
      ASSERT_TRUE(RollerLane(sim).has_value());
      EXPECT_EQ(RollerLane(sim)->active, 0);

      Push(hw, sim, DsPacket(kTeleopEnabled, trigger.pressed));
      ASSERT_TRUE(roller.seen.has_value());
      EXPECT_EQ(RollerLane(sim)->active, 1);
    }
  }
}

// REQ-SAFE-13: a disable shorter than one teleop period (a radio blip) is
// drained together with the re-enable, so no update() samples it; the new
// mode generation alone must disarm a held stick until it returns to neutral.
TEST(TeleopNode, NewGenerationDisarms) {
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
  const auto lane = talos::testing::LatestTarget<control::VelocityTarget>(
      sim, "arm", "teleop");
  ASSERT_TRUE(lane.has_value());
  EXPECT_EQ(lane->active, 0);
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
    EXPECT_THROW(
        {
          Sim sim = MakeSim(config);
          sim.add<ConsumerNode<schema_test::Mixed>>("mechanism");
          sim.add<Teleop>("teleop");
        },
        std::invalid_argument);
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
        EXPECT_THROW(
            {
              Sim sim = MakeSim(config);
              sim.add<ConsumerNode<schema_test::Mixed>>("mechanism");
              sim.add<Teleop>("teleop");
            },
            std::invalid_argument);
      }
    }
  }
}

TEST(TeleopNode, FloatingPointEncodingAcceptsFiniteLimitsWithoutOverflow) {
  using reflection::BaseType;
  std::array<std::byte, 8> bytes{};
  const double float_max = std::numeric_limits<float>::max();
  for (const double value : {float_max, -float_max}) {
    EXPECT_NO_THROW(schema::EncodeNumber(bytes.data(), BaseType::Float, value));
    float decoded{};
    std::memcpy(&decoded, bytes.data(), sizeof(decoded));
    EXPECT_TRUE(std::isfinite(decoded));
    EXPECT_DOUBLE_EQ(decoded, value);
  }
  EXPECT_THROW(
      schema::EncodeNumber(
          bytes.data(), BaseType::Float,
          std::nextafter(float_max, std::numeric_limits<double>::infinity())),
      std::invalid_argument);
  EXPECT_NO_THROW(schema::EncodeNumber(bytes.data(), BaseType::Double,
                                       std::numeric_limits<double>::max()));
}

// REQ-CONTRACT-24: non-finite axis scales are refused during config load (and
// by Teleop construction if passed programmatically).
TEST(TeleopNode, ConstructionRejectsNonFiniteAxisScale) {
  for (const std::string value : {"nan", "inf", "-inf"}) {
    SCOPED_TRACE("scale = " + value);
    std::string config{kAxisToml};
    const std::string old_value = "scale = 10.0";
    config.replace(config.find(old_value), old_value.size(),
                   "scale = " + value);
    EXPECT_THROW(
        {
          Sim sim = MakeSim(config);
          sim.add<ConsumerNode<control::VelocityTarget>>("arm");
          sim.add<Teleop>("teleop");
        },
        std::invalid_argument);
  }
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
    EXPECT_THROW(schema::EncodeNumber(bytes.data(), BaseType::Long, value),
                 std::invalid_argument);
  }
  EXPECT_NO_THROW(
      schema::EncodeNumber(bytes.data(), BaseType::Long, -std::ldexp(1.0, 63)));
  std::int64_t signed_value{};
  std::memcpy(&signed_value, bytes.data(), sizeof(signed_value));
  EXPECT_EQ(signed_value, std::numeric_limits<std::int64_t>::min());
  const double largest_signed = std::nextafter(std::ldexp(1.0, 63), 0.0);
  EXPECT_NO_THROW(
      schema::EncodeNumber(bytes.data(), BaseType::Long, largest_signed));
  EXPECT_THROW(
      schema::EncodeNumber(bytes.data(), BaseType::ULong, std::ldexp(1.0, 64)),
      std::invalid_argument);
  EXPECT_THROW(schema::EncodeNumber(bytes.data(), BaseType::ULong, -1.0),
               std::invalid_argument);
  const double largest_unsigned = std::nextafter(std::ldexp(1.0, 64), 0.0);
  EXPECT_NO_THROW(
      schema::EncodeNumber(bytes.data(), BaseType::ULong, largest_unsigned));
  std::uint64_t unsigned_value{};
  std::memcpy(&unsigned_value, bytes.data(), sizeof(unsigned_value));
  EXPECT_EQ(unsigned_value, static_cast<std::uint64_t>(largest_unsigned));
  EXPECT_THROW(schema::EncodeNumber(bytes.data(), BaseType::Byte, 128.0),
               std::invalid_argument);
  EXPECT_THROW(schema::EncodeNumber(bytes.data(), BaseType::UInt, -1.0),
               std::invalid_argument);
}

// REQ-CONTRACT-24: a malformed binding refuses construction, naming the
// field, subsystem or lane at fault. Each row is one subsystem `sub` read
// through a Target of `message` ("velocity" or "mixed"), with `binding` as
// its [[subsystems.<sub>.teleop.bindings]] entry, plus `extra` TOML.
TEST(TeleopNode, BadParameterFailsConstruction) {
  struct Row {
    const char* what;
    const char* sub;
    const char* message;  // the consumer's target type, or "" for none
    const char* binding;
    const char* extra;
    const char* key;  // an identifier the error must name
  };
  const Row rows[] = {
      {"unknown field in a button", "shooter", "velocity",
       "button = 1\nwhile_held = { foo_rps = 60.0 }", "", "foo_rps"},
      {"unknown field in an axis", "shooter", "velocity",
       "axis = 0\nfield = \"bad_field\"\nscale = 1.0", "", "bad_field"},
      {"wrong type in a button", "shooter", "velocity",
       "button = 1\nwhile_held = { velocity_rps = \"fast\" }", "",
       "velocity_rps"},
      {"integer overflow", "mechanism", "mixed",
       "button = 1\nwhile_held = { sample_count = 5000000000 }", "",
       "sample_count"},
      {"fraction for an integer", "mechanism", "mixed",
       "button = 1\nwhile_held = { sample_count = 1.5 }", "", "sample_count"},
      {"axis bound to a bool", "mech", "mixed",
       "axis = 0\nfield = \"flag\"\nscale = 1.0", "", "flag"},
      {"axis bound to an integer", "mechanism", "mixed",
       "axis = 0\nfield = \"sample_count\"\nscale = 1.0", "", "sample_count"},
      {"no reader of the lane", "ghost", "",
       "button = 1\nwhile_held = { position_rot = 1.0 }", "", "ghost"},
      {"teleop missing from the target policy", "roller", "velocity",
       "button = 1\nwhile_held = { velocity_rps = 10.0 }",
       "[subsystems.roller.targets]\nteleop = [\"auto\"]\n", "roller"},
  };
  for (const Row& row : rows) {
    SCOPED_TRACE(row.what);
    const std::string sub{row.sub};
    Sim sim = MakeSim("[robot]\nperiod_us = 20000\n[subsystems." + sub +
                      "]\nperiod_us = 20000\n" + row.extra + "[[subsystems." +
                      sub + ".teleop.bindings]]\n" + row.binding +
                      "\n[subsystems.teleop]\nperiod_us = 20000\n");
    if (std::string_view{row.message} == "velocity") {
      sim.add<ConsumerNode<control::VelocityTarget>>(sub);
    } else if (std::string_view{row.message} == "mixed") {
      sim.add<ConsumerNode<schema_test::Mixed>>(sub);
    }
    try {
      sim.add<Teleop>("teleop");
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string{error.what()}.find(row.key), std::string::npos)
          << error.what();
    }
  }
}

// REQ-OPS-55: /teleop/status reports armed state, alert_warning_disarmed
// (only while enabled in teleop and disarmed; never while disabled, e-stopped,
// or in autonomous), and the exact blocking axis/button/POV until inputs return
// to neutral.
TEST(TeleopNode, PublishesStatusAndDisarmedAlertForHeldAxisButtonAndPov) {
  constexpr const char* kMultiInputToml = R"(
[robot]
period_us = 20000
command_timeout_us = 100000

[subsystems.roller]
period_us = 20000

[subsystems.roller.teleop]
defaults = { velocity_rps = 0.0, feedforward_v = 0.0 }

[[subsystems.roller.teleop.bindings]]
port = 0
axis = 1
field = "velocity_rps"
scale = 10.0
deadband_frac = 0.1

[[subsystems.roller.teleop.bindings]]
port = 1
button = 3
while_held = { velocity_rps = 20.0 }

[[subsystems.roller.teleop.bindings]]
port = 0
pov = 90
pov_index = 1
while_held = { velocity_rps = 30.0 }

[subsystems.teleop]
period_us = 20000
)";
  Sim sim = MakeSim(kMultiInputToml);
  sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();

  const auto latest_status = [&sim] {
    return sim.latest<::teleop::TeleopStatus>(::teleop::topics::kStatus);
  };
  const auto push_sticks = [&](std::uint32_t flags,
                               const ds::JoystickData& stick0,
                               const ds::JoystickData& stick1) {
    ds::DriverStationData data{};
    data.fms.flags = flags;
    data.joysticks[0] = stick0;
    data.joysticks[1] = stick1;
    Push(hw, sim, data);
  };
  const auto neutral_stick = [] {
    auto s = MakeStick();
    s.pov_count = 2;
    s.povs[0] = -1;
    s.povs[1] = -1;
    return s;
  };

  // Disabled with an axis held: disarmed, but alert_warning_disarmed is FALSE.
  constexpr std::uint32_t kTeleopDisabled = ds::kTeleop | ds::kDsAttached;
  auto held_axis = neutral_stick();
  held_axis.axes[1] = 0.8f;
  push_sticks(kTeleopDisabled, held_axis, neutral_stick());
  auto st = latest_status();
  ASSERT_TRUE(st.has_value());
  EXPECT_FALSE(st->armed());
  EXPECT_FALSE(st->alert_warning_disarmed());

  // Autonomous enabled: teleop is inactive, alert_warning_disarmed is FALSE.
  constexpr std::uint32_t kAutoEnabled =
      ds::kEnabled | ds::kAutonomous | ds::kDsAttached;
  push_sticks(kAutoEnabled, neutral_stick(), neutral_stick());
  st = latest_status();
  ASSERT_TRUE(st.has_value());
  EXPECT_FALSE(st->armed());
  EXPECT_FALSE(st->alert_warning_disarmed());

  // Enable teleop with axis 1 on port 0 held -> disarmed alert + exact axis.
  push_sticks(kTeleopEnabled, held_axis, neutral_stick());
  st = latest_status();
  ASSERT_TRUE(st.has_value());
  EXPECT_FALSE(st->armed());
  EXPECT_TRUE(st->alert_warning_disarmed());
  EXPECT_EQ(st->blocking_kind(), ::teleop::BlockingInputKind::AXIS);
  EXPECT_EQ(st->blocking_joystick_index(), 0);
  EXPECT_EQ(st->blocking_input_index(), 1);

  // Release -> armed, alert clears, blocker resets to NONE/-1/-1.
  push_sticks(kTeleopEnabled, neutral_stick(), neutral_stick());
  st = latest_status();
  ASSERT_TRUE(st.has_value());
  EXPECT_TRUE(st->armed());
  EXPECT_FALSE(st->alert_warning_disarmed());
  EXPECT_EQ(st->blocking_kind(), ::teleop::BlockingInputKind::NONE);
  EXPECT_EQ(st->blocking_joystick_index(), -1);
  EXPECT_EQ(st->blocking_input_index(), -1);

  // E-stopped teleop with an axis held: disarms and keeps
  // alert_warning_disarmed FALSE.
  push_sticks(kTeleopEnabled | ds::kEStop, held_axis, neutral_stick());
  st = latest_status();
  ASSERT_TRUE(st.has_value());
  EXPECT_FALSE(st->armed());
  EXPECT_FALSE(st->alert_warning_disarmed());

  // Disable then re-enable with button 3 on port 1 held -> BUTTON / 1 / 3.
  auto held_btn = neutral_stick();
  held_btn.buttons = 1u << 2;  // 1-based button 3
  push_sticks(kTeleopDisabled, neutral_stick(), held_btn);
  EXPECT_FALSE(latest_status()->alert_warning_disarmed());
  push_sticks(kTeleopEnabled, neutral_stick(), held_btn);
  st = latest_status();
  ASSERT_TRUE(st.has_value());
  EXPECT_FALSE(st->armed());
  EXPECT_TRUE(st->alert_warning_disarmed());
  EXPECT_EQ(st->blocking_kind(), ::teleop::BlockingInputKind::BUTTON);
  EXPECT_EQ(st->blocking_joystick_index(), 1);
  EXPECT_EQ(st->blocking_input_index(), 3);

  // Release -> armed again.
  push_sticks(kTeleopEnabled, neutral_stick(), neutral_stick());
  st = latest_status();
  ASSERT_TRUE(st.has_value());
  EXPECT_TRUE(st->armed());
  EXPECT_FALSE(st->alert_warning_disarmed());
  EXPECT_EQ(st->blocking_kind(), ::teleop::BlockingInputKind::NONE);
  EXPECT_EQ(st->blocking_joystick_index(), -1);
  EXPECT_EQ(st->blocking_input_index(), -1);

  // Disable then re-enable with POV 1 on port 0 deflected in another direction.
  auto held_pov = neutral_stick();
  held_pov.povs[1] = 180;
  push_sticks(kTeleopDisabled, held_pov, neutral_stick());
  push_sticks(kTeleopEnabled, held_pov, neutral_stick());
  st = latest_status();
  ASSERT_TRUE(st.has_value());
  EXPECT_FALSE(st->armed());
  EXPECT_TRUE(st->alert_warning_disarmed());
  EXPECT_EQ(st->blocking_kind(), ::teleop::BlockingInputKind::POV);
  EXPECT_EQ(st->blocking_joystick_index(), 0);
  EXPECT_EQ(st->blocking_input_index(), 1);

  // Release POV -> armed, alert clears, blocker resets.
  push_sticks(kTeleopEnabled, neutral_stick(), neutral_stick());
  st = latest_status();
  ASSERT_TRUE(st.has_value());
  EXPECT_TRUE(st->armed());
  EXPECT_FALSE(st->alert_warning_disarmed());
  EXPECT_EQ(st->blocking_kind(), ::teleop::BlockingInputKind::NONE);
  EXPECT_EQ(st->blocking_joystick_index(), -1);
  EXPECT_EQ(st->blocking_input_index(), -1);
}

// REQ-OPS-55, REQ-RT-06: while disarmed with a held axis, button, or POV,
// Teleop::update() (blocking-input scan, arming evaluation, and /teleop/status
// publication) allocates nothing.
TEST(TeleopNode, DisarmedStatusUpdateDoesNotAllocate) {
  if (const auto why = talos::testing::AllocationProbe::Unsupported();
      !why.empty()) {
    GTEST_SKIP() << why;
  }
  constexpr const char* kMultiInputToml = R"(
[robot]
period_us = 20000
command_timeout_us = 100000

[subsystems.roller]
period_us = 20000

[subsystems.roller.teleop]
defaults = { velocity_rps = 0.0, feedforward_v = 0.0 }

[[subsystems.roller.teleop.bindings]]
port = 0
axis = 1
field = "velocity_rps"
scale = 10.0
deadband_frac = 0.1

[[subsystems.roller.teleop.bindings]]
port = 1
button = 3
while_held = { velocity_rps = 20.0 }

[[subsystems.roller.teleop.bindings]]
port = 0
pov = 90
pov_index = 1
while_held = { velocity_rps = 30.0 }

[subsystems.teleop]
period_us = 20000
)";
  Sim sim = MakeSim(kMultiInputToml);
  sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();

  const auto neutral_stick = [] {
    auto s = MakeStick();
    s.pov_count = 2;
    s.povs[0] = -1;
    s.povs[1] = -1;
    return s;
  };
  auto held_axis = neutral_stick();
  held_axis.axes[1] = 0.8f;
  auto held_btn = neutral_stick();
  held_btn.buttons = 1u << 2;
  auto held_pov = neutral_stick();
  held_pov.povs[1] = 180;

  struct Case {
    const char* label;
    ds::JoystickData stick0;
    ds::JoystickData stick1;
    ::teleop::BlockingInputKind kind;
  };
  constexpr std::uint32_t kTeleopDisabled = ds::kTeleop | ds::kDsAttached;
  for (const Case& c : {Case{"held axis", held_axis, neutral_stick(),
                             ::teleop::BlockingInputKind::AXIS},
                        Case{"held button", neutral_stick(), held_btn,
                             ::teleop::BlockingInputKind::BUTTON},
                        Case{"held pov", held_pov, neutral_stick(),
                             ::teleop::BlockingInputKind::POV}}) {
    SCOPED_TRACE(c.label);
    ds::DriverStationData data{};
    data.joysticks[0] = c.stick0;
    data.joysticks[1] = c.stick1;
    data.fms.flags = kTeleopDisabled;
    Push(hw, sim, data);
    data.fms.flags = kTeleopEnabled;
    Push(hw, sim, data);

    talos::testing::AllocationProbe probe;
    probe.start();
    sim.step(100ms);
    EXPECT_EQ(probe.stop(), 0u) << probe.sites();

    const auto st =
        sim.latest<::teleop::TeleopStatus>(::teleop::topics::kStatus);
    ASSERT_TRUE(st.has_value());
    EXPECT_FALSE(st->armed());
    EXPECT_TRUE(st->alert_warning_disarmed());
    EXPECT_EQ(st->blocking_kind(), c.kind);
  }
}

// REQ-SAFE-14: when controller port 0 is armed and driving subsystem A, a
// second bound controller (port 1) connecting or reconnecting with a held
// input disarms shared arming_ and releases subsystem A's lane until every
// connected bound input returns to neutral.
TEST(TeleopNode, MultiControllerConnectAndReconnectWithHeldInputDisarmsAll) {
  constexpr const char* kTwoStickToml = R"(
[robot]
period_us = 20000
command_timeout_us = 100000

[subsystems.roller]
period_us = 20000

[subsystems.roller.teleop]
defaults = { feedforward_v = 0.0 }

[[subsystems.roller.teleop.bindings]]
port = 0
button = 1
while_held = { velocity_rps = 60.0 }

[subsystems.arm]
period_us = 20000

[subsystems.arm.teleop]
defaults = { feedforward_v = 0.0 }

[[subsystems.arm.teleop.bindings]]
port = 1
axis = 0
field = "velocity_rps"
scale = 10.0
deadband_frac = 0.1

[subsystems.teleop]
period_us = 20000
)";
  Sim sim = MakeSim(kTwoStickToml);
  auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  auto& arm = sim.add<ConsumerNode<control::VelocityTarget>>("arm");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();

  const auto push_sticks = [&](const ds::JoystickData& stick0,
                               const ds::JoystickData& stick1) {
    ds::DriverStationData data{};
    data.fms.flags = kTeleopEnabled;
    data.joysticks[0] = stick0;
    data.joysticks[1] = stick1;
    Push(hw, sim, data);
  };
  const auto latest_status = [&sim] {
    return sim.latest<::teleop::TeleopStatus>(::teleop::topics::kStatus);
  };
  const ds::JoystickData disconnected = MakeStick(0, 0, 0, 0, false);

  // Arm with only port 0 connected, then drive roller from port 0.
  push_sticks(MakeStick(), disconnected);
  push_sticks(MakeStick(0, 0, 0, 1), disconnected);
  ASSERT_TRUE(roller.seen.has_value());
  EXPECT_DOUBLE_EQ(roller.seen->velocity_rps(), 60.0);
  ASSERT_TRUE(RollerLane(sim).has_value());
  EXPECT_EQ(RollerLane(sim)->active, 1);
  ASSERT_TRUE(latest_status().has_value());
  EXPECT_TRUE(latest_status()->armed());

  // Port 1 connects with a held axis: shared arming_ disarms and roller's lane
  // is released even though port 0 stayed connected.
  push_sticks(MakeStick(0, 0, 0, 1), MakeStick(0.8f));
  EXPECT_FALSE(roller.seen.has_value());
  EXPECT_FALSE(arm.seen.has_value());
  EXPECT_EQ(RollerLane(sim)->active, 0);
  ASSERT_TRUE(latest_status().has_value());
  EXPECT_FALSE(latest_status()->armed());
  EXPECT_TRUE(latest_status()->alert_warning_disarmed());

  // Releasing port 0 alone is not enough while port 1 stays deflected.
  push_sticks(MakeStick(), MakeStick(0.8f));
  push_sticks(MakeStick(0, 0, 0, 1), MakeStick(0.8f));
  EXPECT_FALSE(roller.seen.has_value());
  EXPECT_FALSE(arm.seen.has_value());

  // Both controllers neutral -> rearms; fresh inputs drive both lanes.
  push_sticks(MakeStick(), MakeStick());
  ASSERT_TRUE(latest_status().has_value());
  EXPECT_TRUE(latest_status()->armed());
  push_sticks(MakeStick(0, 0, 0, 1), MakeStick(1.0f));
  ASSERT_TRUE(roller.seen.has_value());
  ASSERT_TRUE(arm.seen.has_value());
  EXPECT_DOUBLE_EQ(roller.seen->velocity_rps(), 60.0);
  EXPECT_NEAR(arm.seen->velocity_rps(), 10.0, 1e-6);

  // Port 1 disconnects, then reconnects with held axis while port 0 is driving
  // roller: reconnect disarms shared arming_ and releases roller's lane again.
  push_sticks(MakeStick(0, 0, 0, 1), disconnected);
  ASSERT_TRUE(roller.seen.has_value());
  EXPECT_EQ(RollerLane(sim)->active, 1);

  push_sticks(MakeStick(0, 0, 0, 1), MakeStick(0.8f));
  EXPECT_FALSE(roller.seen.has_value());
  EXPECT_FALSE(arm.seen.has_value());
  EXPECT_EQ(RollerLane(sim)->active, 0);
  ASSERT_TRUE(latest_status().has_value());
  EXPECT_FALSE(latest_status()->armed());
  EXPECT_TRUE(latest_status()->alert_warning_disarmed());
}

// REQ-CONTRACT-24: when a button binding and an axis binding target the same
// field, the earlier live binding in config order sets field_set and wins; an
// inactive earlier binding (e.g. axis inside deadband) yields to the later one.
TEST(TeleopNode, CombinedButtonAndAxisOnSameFieldRespectsBindingOrder) {
  const std::string button_binding =
      "[[subsystems.roller.teleop.bindings]]\n"
      "button = 1\n"
      "while_held = { velocity_rps = 60.0 }\n";
  const std::string axis_binding =
      "[[subsystems.roller.teleop.bindings]]\n"
      "axis = 0\n"
      "field = \"velocity_rps\"\n"
      "scale = 10.0\n"
      "deadband_frac = 0.125\n";

  for (const bool button_first : {true, false}) {
    SCOPED_TRACE(button_first ? "button_first" : "axis_first");
    const std::string config =
        "[robot]\nperiod_us = 20000\ncommand_timeout_us = 100000\n"
        "[subsystems.roller]\nperiod_us = 20000\n"
        "[subsystems.roller.teleop]\n"
        "defaults = { velocity_rps = 0.0, feedforward_v = 0.0 }\n" +
        (button_first ? button_binding + axis_binding
                      : axis_binding + button_binding) +
        "[subsystems.teleop]\nperiod_us = 20000\n";
    Sim sim = MakeSim(config);
    auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
    sim.add<Teleop>("teleop");
    SimHardware hw{sim};
    hw.enable();
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));

    // Button only (axis inside deadband): button writes even when axis is
    // listed first.
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0.05f, 0, 0, 1)));
    ASSERT_TRUE(roller.seen.has_value());
    EXPECT_DOUBLE_EQ(roller.seen->velocity_rps(), 60.0);

    // Axis only: axis writes even when button is listed first.
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1.0f, 0, 0, 0)));
    ASSERT_TRUE(roller.seen.has_value());
    EXPECT_NEAR(roller.seen->velocity_rps(), 10.0, 1e-6);

    // Both live simultaneously: first binding in config order wins.
    Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1.0f, 0, 0, 1)));
    ASSERT_TRUE(roller.seen.has_value());
    EXPECT_NEAR(roller.seen->velocity_rps(), button_first ? 60.0 : 10.0, 1e-6);

    // Release the winning input while keeping the other live: fallback takes
    // over cleanly.
    if (button_first) {
      Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(1.0f, 0, 0, 0)));
      ASSERT_TRUE(roller.seen.has_value());
      EXPECT_NEAR(roller.seen->velocity_rps(), 10.0, 1e-6);
    } else {
      Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(0.0f, 0, 0, 1)));
      ASSERT_TRUE(roller.seen.has_value());
      EXPECT_DOUBLE_EQ(roller.seen->velocity_rps(), 60.0);
    }
  }
}

// REQ-SAFE-14, REQ-CONTRACT-24: an invalid (overflowing) axis on one subsystem
// releases only that subsystem's lane and leaves another subsystem's live lane
// unaffected (including a subsystem evaluated later in map order).
TEST(TeleopNode, InvalidAxisOnOneSubsystemLeavesOtherSubsystemUnaffected) {
  constexpr const char* kTwoSubToml = R"(
[robot]
period_us = 20000
command_timeout_us = 100000

[subsystems.mechanism]
period_us = 20000

[subsystems.mechanism.teleop]
defaults = { flag = false, position_m = 0.0, level = 'NONE', sample_count = 0, speed_frac = 0.0, 'inner/mode' = 'OFF', 'inner/offset_m' = 0.0 }

[[subsystems.mechanism.teleop.bindings]]
axis = 0
field = "inner/offset_m"
scale = 1e40

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
  Sim sim = MakeSim(kTwoSubToml);
  auto& mechanism = sim.add<ConsumerNode<schema_test::Mixed>>("mechanism");
  auto& roller = sim.add<ConsumerNode<control::VelocityTarget>>("roller");
  sim.add<Teleop>("teleop");
  SimHardware hw{sim};
  hw.enable();
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick()));

  // Both subsystems live with finite values.
  constexpr float kSmall = 0.06f;
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(kSmall, 0, 0, 1)));
  ASSERT_TRUE(mechanism.seen.has_value());
  ASSERT_TRUE(roller.seen.has_value());
  EXPECT_DOUBLE_EQ(roller.seen->velocity_rps(), 60.0);

  // Overflow mechanism's float field (invalid_axis = true): mechanism's lane
  // releases, while roller (evaluated after mechanism) stays live untouched.
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(4.0f, 0, 0, 1)));
  EXPECT_FALSE(mechanism.seen.has_value());
  const auto mech_lane = talos::testing::LatestTarget<schema_test::Mixed>(
      sim, "mechanism", "teleop");
  ASSERT_TRUE(mech_lane.has_value());
  EXPECT_EQ(mech_lane->active, 0);
  ASSERT_TRUE(roller.seen.has_value());
  EXPECT_DOUBLE_EQ(roller.seen->velocity_rps(), 60.0);
  ASSERT_TRUE(RollerLane(sim).has_value());
  EXPECT_EQ(RollerLane(sim)->active, 1);

  // Returning mechanism's axis to a finite output recovers mechanism while
  // roller stays live.
  Push(hw, sim, DsPacket(kTeleopEnabled, MakeStick(kSmall, 0, 0, 1)));
  ASSERT_TRUE(mechanism.seen.has_value());
  ASSERT_TRUE(roller.seen.has_value());
  EXPECT_DOUBLE_EQ(roller.seen->velocity_rps(), 60.0);
}

}  // namespace
}  // namespace talos::teleop

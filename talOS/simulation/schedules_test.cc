#include "talOS/simulation/schedules.h"

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <vector>

namespace talos::simulation {
namespace {

TEST(Schedules, RejectsMalformedFaultsRatherThanSilentlySkippingThem) {
  for (const auto* spec :
       {"", "1:1:reset:drive", "-1:2:reset:drive", "nan:2:reset:drive",
        "0:inf:reset:drive", "0:1:unknown:drive", "0:1:udp_blackout:drive",
        "0:1:supply_voltage:all:17", "0:1:supply_voltage:drive:6",
        "0:1:supply_voltage:all", "0:1:freeze:drive:4",
        "0:2:freeze:drive|1:3:freeze:drive", "0:1:forward_limit:drive:4",
        "0:2:reverse_limit:drive|1:3:reverse_limit:drive"}) {
    EXPECT_THROW(ParseFaults(spec), std::invalid_argument) << spec;
  }
}

TEST(Schedules, FaultIntervalsUseExistingHooksAndRestoreOnTheirEnd) {
  talos::hardware::Config config;
  config.motors.push_back({.id = 1, .name = "drive", .can_id = 1});
  config.sensors.push_back({.id = 2, .name = "encoder", .can_id = 2});
  talos::hardware::sim::PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(config));
  auto events = ParseFaults(
      "1:2:motor_disconnect:drive|1:2:sensor_disconnect:encoder|"
      "1:2:freeze:drive|1:2:supply_voltage:all:6|"
      "1:2:udp_blackout:bridge|3:4:reset:drive");
  ResolveFaults(events, config);
  EXPECT_FALSE(ApplyFaults(events, 0, backend));
  EXPECT_TRUE(ApplyFaults(events, 1, backend));
  EXPECT_DOUBLE_EQ(backend.supply_voltage(), 6);
  talos::hardware::State state;
  ASSERT_FALSE(backend.Read(state));  // Every configured device disconnected.
  EXPECT_FALSE(state.motors[0].valid);
  EXPECT_FALSE(state.sensors[0].valid);
  EXPECT_FALSE(ApplyFaults(events, 2, backend));
  EXPECT_DOUBLE_EQ(backend.supply_voltage(),
                   talos::hardware::kSimRestingSupplyV);
  ASSERT_TRUE(backend.Read(state));
  EXPECT_TRUE(state.motors[0].valid);
  EXPECT_TRUE(state.sensors[0].valid);
  EXPECT_FALSE(ApplyFaults(events, 3, backend));
  std::vector<uint16_t> resets;
  ASSERT_TRUE(backend.PollDeviceResets(resets));
  ASSERT_EQ(resets.size(), 1u);
  EXPECT_EQ(resets[0], 1);
  EXPECT_FALSE(ApplyFaults(events, 3.5, backend));
  EXPECT_FALSE(backend.PollDeviceResets(resets));
  auto unknown = ParseFaults("0:1:motor_disconnect:missing");
  EXPECT_THROW(ResolveFaults(unknown, config), std::invalid_argument);
}

TEST(Schedules, FrozenFeedbackHoldsThenReleasesTheSample) {
  talos::hardware::Config config;
  config.motors.push_back({.id = 1, .name = "drive", .can_id = 1});
  talos::hardware::sim::PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(config));
  talos::hardware::State state;
  ASSERT_TRUE(backend.Read(state));
  const auto held = state.motors[0];
  const std::array<talos::hardware::MotorRequest, 1> request{{
      {1, talos::hardware::Mode::kVoltage, 0, 6, 0},
  }};
  ASSERT_TRUE(backend.Apply(request));
  auto events = ParseFaults("1:2:freeze:drive");
  ResolveFaults(events, config);
  ApplyFaults(events, 1, backend);
  backend.Step(0.1);
  ASSERT_TRUE(backend.Read(state));
  EXPECT_EQ(state.motors[0], held);
  ApplyFaults(events, 2, backend);
  ASSERT_TRUE(backend.Read(state));
  EXPECT_GT(state.motors[0].velocity_rps, 0);
}

// A pressed switch is what the TalonFX sees: output toward it is held neutral
// and the motor reports the asserted limit, for the interval only.
TEST(Schedules, LimitSwitchIntervalsPressTheMotorsSwitch) {
  talos::hardware::Config config;
  config.motors.push_back({.id = 1, .name = "drive", .can_id = 1});
  config.sensors.push_back({.id = 2, .name = "encoder", .can_id = 2});
  talos::hardware::sim::PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(config));
  auto sensor = ParseFaults("0:1:forward_limit:encoder");
  EXPECT_THROW(ResolveFaults(sensor, config), std::invalid_argument);
  auto events = ParseFaults("1:2:forward_limit:drive|2:3:reverse_limit:drive");
  ResolveFaults(events, config);
  const std::array<talos::hardware::MotorRequest, 1> request{{
      {1, talos::hardware::Mode::kVoltage, 0, 6, 0},
  }};
  ASSERT_TRUE(backend.Apply(request));
  talos::hardware::State state;
  const auto flags_at = [&](double elapsed_s) {
    ApplyFaults(events, elapsed_s, backend);
    backend.Step(0.01);
    EXPECT_TRUE(backend.Read(state));
    return state.motors[0].fault_flags;
  };
  EXPECT_EQ(flags_at(0), 0u);
  EXPECT_GT(backend.applied_voltage(0), 0);
  EXPECT_EQ(flags_at(1), talos::hardware::kMotorForwardHardLimitFlag);
  EXPECT_DOUBLE_EQ(backend.applied_voltage(0), 0);
  EXPECT_EQ(flags_at(2), talos::hardware::kMotorReverseHardLimitFlag);
  EXPECT_GT(backend.applied_voltage(0), 0);
  EXPECT_EQ(flags_at(3), 0u);
}

// A gateway that joins its schedule partway must not replay what already
// ended (a spurious device reset), yet still applies an interval in progress.
TEST(Schedules, JoiningPartwaySkipsEndedFaultsOnly) {
  talos::hardware::Config config;
  config.motors.push_back({.id = 1, .name = "drive", .can_id = 1});
  talos::hardware::sim::PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(config));
  auto events = ParseFaults("1:2:reset:drive|2:4:motor_disconnect:drive");
  ResolveFaults(events, config);
  SkipPastFaults(events, 3);
  ApplyFaults(events, 3, backend);
  std::vector<uint16_t> resets;
  EXPECT_FALSE(backend.PollDeviceResets(resets));
  talos::hardware::State state;
  ASSERT_FALSE(backend.Read(state));
  EXPECT_FALSE(state.motors[0].valid);
}

TEST(Schedules, ScheduleStartIsANonNegativeIntegerTime) {
  EXPECT_EQ(ParseScheduleStart("0"), 0);
  EXPECT_EQ(ParseScheduleStart("123456789012\n"), 123456789012);
  for (const auto* text : {"", "\n", "-5", "+5", "1.5", "12abc", " 1", "abc",
                           "1234567890123456789"}) {
    EXPECT_THROW(ParseScheduleStart(text), std::invalid_argument) << text;
  }
}

TEST(Schedules, AdjacentFaultsRestoreBeforeActivatingInAnyInputOrder) {
  talos::hardware::Config config;
  config.motors.push_back({.id = 1, .name = "drive", .can_id = 1});
  talos::hardware::sim::PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(config));
  auto events = ParseFaults(
      "2:3:motor_disconnect:drive|1:2:motor_disconnect:drive|"
      "2:3:supply_voltage:all:7|1:2:supply_voltage:all:6");
  ResolveFaults(events, config);
  ApplyFaults(events, 1, backend);
  EXPECT_DOUBLE_EQ(backend.supply_voltage(), 6);
  ApplyFaults(events, 2, backend);
  talos::hardware::State state;
  ASSERT_FALSE(backend.Read(state));
  EXPECT_FALSE(state.motors[0].valid);
  EXPECT_DOUBLE_EQ(backend.supply_voltage(), 7);
  ApplyFaults(events, 3, backend);
  ASSERT_TRUE(backend.Read(state));
  EXPECT_TRUE(state.motors[0].valid);
  EXPECT_DOUBLE_EQ(backend.supply_voltage(),
                   talos::hardware::kSimRestingSupplyV);
}

// REQ-DEV-05: non-finite elapsed_s must fail closed rather than firing every
// future reset fault, overflowed numbers map to invalid_argument, and an
// ambiguous motor/sensor fault target is refused.
TEST(Schedules, RejectsNonFiniteElapsedOverflowAndAmbiguousTargets) {
  EXPECT_THROW(ParseFaults("0:1e400:reset:drive"), std::invalid_argument);
  EXPECT_THROW(ParseFaults(" 0:1:reset:drive"), std::invalid_argument);

  talos::hardware::Config ambiguous;
  ambiguous.motors.push_back({.id = 1, .name = "shared", .can_id = 1});
  ambiguous.sensors.push_back({.id = 2, .name = "shared", .can_id = 2});
  auto reset_shared = ParseFaults("1:2:reset:shared");
  EXPECT_THROW(ResolveFaults(reset_shared, ambiguous), std::invalid_argument);

  talos::hardware::Config config;
  config.motors.push_back({.id = 1, .name = "drive", .can_id = 1});
  talos::hardware::sim::PhysicsSimBackend backend;
  ASSERT_TRUE(backend.Configure(config));
  auto events = ParseFaults("1:2:reset:drive");
  ResolveFaults(events, config);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(SkipPastFaults(events, nan), std::invalid_argument);
  EXPECT_THROW(SkipPastFaults(events, inf), std::invalid_argument);
  EXPECT_THROW(ApplyFaults(events, nan, backend), std::invalid_argument);
  EXPECT_THROW(ApplyFaults(events, inf, backend), std::invalid_argument);
  std::vector<uint16_t> resets;
  EXPECT_FALSE(backend.PollDeviceResets(resets));
  EXPECT_FALSE(events[0].finished);
}

// REQ-DEV-05, REQ-DEV-10: a supply_voltage fault overrides the battery model
// during its interval and restores the configured battery model (not a
// hardcoded 12.8 V resting bus) via ClearSupplyVoltageOverride() at its end.
TEST(Schedules, SupplyVoltageFaultRestoresConfiguredBatteryModel) {
  talos::hardware::Config config;
  talos::hardware::MotorConfig drive{.id = 1, .name = "drive", .can_id = 1};
  drive.stator_limit_a = 100.0;
  drive.supply_limit_a = 60.0;
  config.motors.push_back(drive);
  talos::hardware::sim::PhysicsSimBackend backend;
  backend.SetBatteryModel({.nominal_voltage_v = 12.6,
                           .internal_resistance_ohm = 0.05,
                           .base_current_a = 4.0,
                           .brownout_voltage_v = 6.75});
  ASSERT_TRUE(backend.Configure(config));
  constexpr double kRestingV = 12.6 - 0.05 * 4.0;
  EXPECT_NEAR(backend.supply_voltage(), kRestingV, 1e-9);

  const std::array<talos::hardware::MotorRequest, 1> request{{
      {1, talos::hardware::Mode::kVoltage, 0, 10.0, 0.0},
  }};
  ASSERT_TRUE(backend.Apply(request));
  auto events = ParseFaults("1:2:supply_voltage:all:7");
  ResolveFaults(events, config);

  // Before the fault: battery model sags below its resting 12.4 V under load.
  EXPECT_FALSE(ApplyFaults(events, 0.0, backend));
  backend.Step(0.005);
  EXPECT_LT(backend.supply_voltage(), kRestingV - 0.1);

  // During the fault: explicit override pins the bus to 7.0 V despite load.
  EXPECT_FALSE(ApplyFaults(events, 1.0, backend));
  backend.Step(0.005);
  EXPECT_DOUBLE_EQ(backend.supply_voltage(), 7.0);

  // At interval end: ClearSupplyVoltageOverride() restores the battery model's
  // resting voltage (12.4 V, not 12.8 V) and resumes dynamic load sag.
  EXPECT_FALSE(ApplyFaults(events, 2.0, backend));
  EXPECT_NEAR(backend.supply_voltage(), kRestingV, 1e-9);
  backend.Step(0.005);
  EXPECT_LT(backend.supply_voltage(), kRestingV - 0.1);

  talos::hardware::State state;
  state.sample_time_us = 10000;
  ASSERT_TRUE(backend.Read(state));
  EXPECT_TRUE(state.health.valid);
  EXPECT_LT(state.health.battery_v, kRestingV - 0.1);

  // Unloaded after Neutral(): returns to configured resting 12.4 V, not 12.8 V.
  backend.Neutral();
  EXPECT_NEAR(backend.supply_voltage(), kRestingV, 1e-9);
}

}  // namespace
}  // namespace talos::simulation

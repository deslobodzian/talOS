#include "schedules.h"

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace robot::simulation {
namespace {

TEST(Schedules, RejectsMalformedFaultsRatherThanSilentlySkippingThem) {
  for (const auto* spec :
       {"", "1:1:reset:drive", "-1:2:reset:drive", "nan:2:reset:drive",
        "0:inf:reset:drive", "0:1:unknown:drive", "0:1:udp_blackout:drive",
        "0:1:supply_voltage:all:17", "0:1:supply_voltage:drive:6",
        "0:1:supply_voltage:all", "0:1:freeze:drive:4",
        "0:2:freeze:drive|1:3:freeze:drive"}) {
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
  EXPECT_DOUBLE_EQ(backend.supply_voltage(), 12);
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
  EXPECT_DOUBLE_EQ(backend.supply_voltage(), 12);
}

}  // namespace
}  // namespace robot::simulation

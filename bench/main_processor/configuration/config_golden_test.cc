#include <gtest/gtest.h>

#include "talOS/configuration/config_parser.h"

TEST(BenchConfig, OneConservativeUncommissionedMotor) {
  for (const auto* path :
       {"bench/main_processor/configuration/robot.toml",
        "bench/main_processor/configuration/robot_bench.toml"}) {
    const auto config = talos::config::ParseRobotConfig(path);
    EXPECT_FALSE(config.hardware.commissioned);
    EXPECT_TRUE(config.require_target_coverage);
    EXPECT_EQ(config.controller_port, 5802);
    EXPECT_EQ(config.local_port, 5803);
    ASSERT_EQ(config.hardware.motors.size(), 1u);
    const auto& motor = config.hardware.motors.front();
    EXPECT_EQ(motor.name, "kraken");
    EXPECT_EQ(motor.can_id, 1);  // Placeholder, not a measured physical id.
    EXPECT_DOUBLE_EQ(motor.max_voltage, 4.0);
    EXPECT_DOUBLE_EQ(motor.supply_limit_a, 30.0);
    EXPECT_DOUBLE_EQ(motor.stator_limit_a, 40.0);
    EXPECT_DOUBLE_EQ(motor.max_velocity_rps, 30.0);
  }
}

TEST(BenchConfig, DeploymentOverridesOnlyControllerAddress) {
  const auto sim = talos::config::ParseRobotConfig(
      "bench/main_processor/configuration/robot.toml");
  const auto bench = talos::config::ParseRobotConfig(
      "bench/main_processor/configuration/robot_bench.toml");
  EXPECT_EQ(sim.controller_address, "127.0.0.1");
  EXPECT_EQ(bench.controller_address, "10.56.87.2");
  EXPECT_EQ(sim.hardware.motors.size(), bench.hardware.motors.size());
  EXPECT_EQ(sim.subsystems.size(), 3u);
  EXPECT_EQ(bench.subsystems.size(), 3u);
}

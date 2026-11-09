#include <gtest/gtest.h>

#include "talOS/configuration/config_parser.h"

TEST(BenchConfig, OneConservativeUncommissionedMotor) {
  for (const auto* path :
       {"bench/main_processor/configuration/robot.toml",
        "bench/main_processor/configuration/robot_bench.toml"}) {
    SCOPED_TRACE(path);
    const auto config = talos::config::ParseRobotConfig(path);
    EXPECT_FALSE(config.hardware.commissioned);
    EXPECT_TRUE(config.require_target_coverage);
    EXPECT_EQ(config.hardware.command_timeout_us, 100000u);
    ASSERT_EQ(config.hardware.motors.size(), 1u);
    const auto& motor = config.hardware.motors.front();
    EXPECT_DOUBLE_EQ(motor.max_voltage, 4.0);
    EXPECT_DOUBLE_EQ(motor.supply_limit_a, 30.0);
    EXPECT_DOUBLE_EQ(motor.stator_limit_a, 40.0);
    EXPECT_DOUBLE_EQ(motor.max_velocity_rps, 30.0);
  }
}

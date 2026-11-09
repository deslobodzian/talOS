#include <gtest/gtest.h>

#include <cstddef>
#include <string>

#include "talOS/configuration/config_parser.h"

namespace talos::config {
namespace {

TEST(ConfigTest, EveryDeploymentRemainsUncommissionedWithBoundedCommands) {
  for (const char* path :
       {"2026-robot/main_processor/configuration/robot.toml",
        "2026-robot/main_processor/configuration/robot_field.toml",
        "2026-robot/main_processor/configuration/robot_physics_sim.toml"}) {
    SCOPED_TRACE(path);
    const auto config = ParseRobotConfig(path);
    EXPECT_FALSE(config.hardware.commissioned);
    EXPECT_TRUE(config.require_target_coverage);
    EXPECT_EQ(config.hardware.command_timeout_us, 100000u);
    for (const char* name : {"drivetrain", "shooter", "intake"}) {
      EXPECT_TRUE(config.TargetsFor(name).test.empty()) << name;
    }
  }
}

// The estimator and drivetrain independently consume these tables. Different
// geometry would make odometry disagree with the commanded wheel motion.
TEST(ConfigTest, PoseEstimatorGeometryMatchesDrivetrain) {
  const auto config =
      ParseRobotConfig("2026-robot/main_processor/configuration/robot.toml");
  const toml::table& root = config.toml_data;
  for (const char* key : {"field_length_m", "field_width_m"}) {
    const auto estimator =
        root.at_path(std::string{"subsystems.pose_estimator."} + key)
            .value<double>();
    ASSERT_TRUE(estimator.has_value()) << key;
    EXPECT_EQ(estimator,
              root.at_path(std::string{"subsystems.autonomous."} + key)
                  .value<double>());
  }
  const auto* modules =
      root.at_path("subsystems.drivetrain.geometry.modules").as_array();
  const auto* xs =
      root.at_path("subsystems.pose_estimator.module_x_m").as_array();
  const auto* ys =
      root.at_path("subsystems.pose_estimator.module_y_m").as_array();
  ASSERT_NE(modules, nullptr);
  ASSERT_NE(xs, nullptr);
  ASSERT_NE(ys, nullptr);
  ASSERT_EQ(modules->size(), 4u);
  ASSERT_EQ(xs->size(), modules->size());
  ASSERT_EQ(ys->size(), modules->size());
  for (std::size_t i = 0; i < modules->size(); ++i) {
    SCOPED_TRACE(i);
    const auto* module = (*modules)[i].as_table();
    ASSERT_NE(module, nullptr);
    const auto x = (*module)["x_m"].value<double>();
    const auto y = (*module)["y_m"].value<double>();
    ASSERT_TRUE(x.has_value());
    ASSERT_TRUE(y.has_value());
    EXPECT_EQ((*xs)[i].value<double>(), x);
    EXPECT_EQ((*ys)[i].value<double>(), y);
  }
}

}  // namespace
}  // namespace talos::config

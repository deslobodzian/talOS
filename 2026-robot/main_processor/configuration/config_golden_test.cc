#include <gtest/gtest.h>

#include <cstddef>
#include <string>

#include "talOS/configuration/config_parser.h"
#include "talOS/vision/field_sim/field_layout_loader.h"
#include "vision/data/field_layout_2026.h"

namespace talos::config {
namespace {

constexpr const char* kDeployments[] = {
    "2026-robot/main_processor/configuration/robot.toml",
    "2026-robot/main_processor/configuration/robot_field.toml",
    "2026-robot/main_processor/configuration/robot_physics_sim.toml"};

TEST(ConfigTest, EveryDeploymentRemainsUncommissionedWithBoundedCommands) {
  for (const char* path : kDeployments) {
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

// The driver reads its re-zero button from joystick 0 (driver.h). A teleop
// binding on the same button would re-zero driver-forward, mid-drive, every
// time the driver uses that mechanism.
TEST(ConfigTest, DriverRezeroButtonHasNoTeleopBinding) {
  constexpr int kDriverJoystick = 0;
  for (const char* path : kDeployments) {
    SCOPED_TRACE(path);
    const auto config = ParseRobotConfig(path);
    const auto rezero =
        config.toml_data.at_path("subsystems.driver.rezero_button_id")
            .value<int>();
    ASSERT_TRUE(rezero.has_value());
    for (const auto& [subsystem, bindings] : config.teleop) {
      for (const auto& binding : bindings) {
        EXPECT_FALSE(binding.port == kDriverJoystick &&
                     binding.button == rezero)
            << subsystem << " binds the driver's re-zero button " << *rezero;
      }
    }
  }
}

// The estimator and drivetrain independently consume these tables. Different
// geometry would make odometry disagree with the commanded wheel motion.
TEST(ConfigTest, PoseEstimatorGeometryMatchesDrivetrain) {
  const auto config =
      ParseRobotConfig("2026-robot/main_processor/configuration/robot.toml");
  const toml::table& root = config.toml_data;
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

// The field is the one the cameras' tag layout describes: autonomous mirrors
// Red waypoints across it and the estimator rejects observations outside it.
TEST(ConfigTest, FieldDimensionsMatchTheTagLayout) {
  talos::vision::field_sim::FieldLayoutData layout;
  std::string error;
  ASSERT_TRUE(talos::vision::field_sim::LoadFieldLayoutFromJson(
      ::vision::kFieldLayoutJson, /*tag_size_m=*/0.1651, &layout, &error))
      << error;
  for (const char* path : kDeployments) {
    SCOPED_TRACE(path);
    const auto config = ParseRobotConfig(path);
    for (const char* subsystem : {"autonomous", "pose_estimator"}) {
      SCOPED_TRACE(subsystem);
      const toml::table* table = config.GetSubsystemTable(subsystem);
      ASSERT_NE(table, nullptr);
      const auto length = (*table)["field_length_m"].value<double>();
      const auto width = (*table)["field_width_m"].value<double>();
      ASSERT_TRUE(length.has_value());
      ASSERT_TRUE(width.has_value());
      EXPECT_DOUBLE_EQ(*length, layout.dims.length_m);
      EXPECT_DOUBLE_EQ(*width, layout.dims.width_m);
    }
  }
}

}  // namespace
}  // namespace talos::config

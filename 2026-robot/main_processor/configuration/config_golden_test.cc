#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "drivetrain/geometry.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/telemetry/dashboard_layout.h"
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

// Every deployment serves the drive team a dashboard whose layout the page
// can draw within its feed budget (talOS/telemetry/README.md "Dashboard
// layout"); that each channel exists is the telemetry node's check.
// REQ-OPS-39
TEST(ConfigTest, EveryDeploymentNamesAValidDashboardLayout) {
  for (const char* path : kDeployments) {
    SCOPED_TRACE(path);
    const auto config = ParseRobotConfig(path);
    ASSERT_FALSE(config.studio.dashboard.empty());
    try {
      talos::telemetry::ParseDashboardLayout(config.studio.dashboard);
    } catch (const talos::telemetry::DashboardLayoutError& error) {
      ADD_FAILURE() << error.what();
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

// The drivetrain holds a wheel's azimuth at zero drive below a hold speed
// (swerve.h). Autonomous ends a drive phase only inside its position and yaw
// tolerances, so its feedback just outside them must ask every wheel for more
// than that, with margin; otherwise a residual error the drivetrain will not
// act on stalls the routine until its drive timeout aborts it.
TEST(ConfigTest, AutonomousToleranceFeedbackClearsTheSwerveHoldSpeed) {
  constexpr double kMargin = 1.5;
  for (const char* path : kDeployments) {
    SCOPED_TRACE(path);
    const auto config = ParseRobotConfig(path);
    const toml::table* autonomous = config.GetSubsystemTable("autonomous");
    ASSERT_NE(autonomous, nullptr);
    // The same defaults autonomous.h reads.
    const double position_tolerance_m =
        (*autonomous)["position_tolerance_m"].value_or(0.10);
    const double yaw_tolerance_rad =
        (*autonomous)["yaw_tolerance_rad"].value_or(0.05);
    const double kp_xy = (*autonomous)["xy_feedback_per_m_mps"].value_or(1.0);
    const double kp_yaw =
        (*autonomous)["yaw_feedback_per_rad_radps"].value_or(1.0);
    const auto geometry = drivetrain::BuildSwerveGeometry(config, "drivetrain");
    for (const auto& module : geometry.modules) {
      SCOPED_TRACE(module.name);
      const double hold_mps = drivetrain::SwerveKinematics::HoldSpeed(module);
      EXPECT_GT(kp_xy * position_tolerance_m, kMargin * hold_mps);
      EXPECT_GT(kp_yaw * yaw_tolerance_rad * std::hypot(module.x_m, module.y_m),
                kMargin * hold_mps);
    }
  }
}

}  // namespace
}  // namespace talos::config

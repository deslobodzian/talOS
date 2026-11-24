// REQ-DEV-37: a model that owns the chassis (its root body has a free joint)
// replaces the analytic chassis, so MujocoGatewayExtension::Configure
// refuses each analytic chassis key and odometry-error flag, naming it; a
// welded-root model keeps the analytic chassis and accepts them. The robot
// starts at the start pose the program resolves.

#include "talOS/simulation/mujoco/gateway_extension.h"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/sim_physics.h"
#include "talOS/simulation/gateway_main.h"
#include "talOS/simulation/mujoco/plant.h"

namespace talos::simulation::mujoco {
namespace {

using rules_cc::cc::runfiles::Runfiles;

struct Row {
  const char* what;
  const char* model_env;  // ROVER_XML (free root) or ARM_XML (welded)
  const char* motor;      // the model's actuator
  std::string toml;       // added before [simulation.mujoco]
  hardware::sim::PhysicsOptions options;
  const char* refused;  // the key or flag named; nullptr: accepted
};

// Writes the row's config beside a copy of its testdata model and parses it.
config::RobotConfig RowConfig(const Row& row, int index) {
  std::string error;
  const std::unique_ptr<Runfiles> runfiles(
      Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY, &error));
  const char* model = std::getenv(row.model_env);
  if (runfiles == nullptr || model == nullptr) {
    throw std::runtime_error("no runfiles or no " + std::string(row.model_env) +
                             ": " + error);
  }
  const std::filesystem::path dir =
      std::filesystem::path(::testing::TempDir()) /
      ("extension_" + std::to_string(index));
  std::filesystem::create_directories(dir);
  std::filesystem::copy_file(runfiles->Rlocation(model), dir / "model.xml",
                             std::filesystem::copy_options::overwrite_existing);
  std::ofstream(dir / "robot.toml")
      << "[robot]\nname = \"extension_test\"\nperiod_us = 5000\n"
         "command_timeout_us = 100000\n\n"
      << "[subsystems.s.motors." << row.motor << "]\ncan_id = 1\n"
      << "[subsystems.s.motors.d]\ncan_id = 2\n"
      << "[subsystems.s.motors.t]\ncan_id = 3\n\n"
      << row.toml << "\n[simulation.mujoco]\nrobot_model = \"model.xml\"\n";
  return config::ParseRobotConfig((dir / "robot.toml").string());
}

// REQ-DEV-37
TEST(MujocoGatewayExtension, ChassisOwnerRefusesEachAnalyticChassisSetting) {
  constexpr hardware::sim::PhysicsOptions kNone{};
  const Row rows[] = {
      {"wheel radius", "ROVER_XML", "roller",
       "[simulation]\nwheel_radius_m = 0.06\n", kNone,
       "[simulation] wheel_radius_m"},
      {"swerve modules", "ROVER_XML", "roller",
       "[[simulation.swerve_modules]]\ndrive = \"d\"\nsteer = \"t\"\n"
       "x_m = 0.3\ny_m = 0.3\n",
       kNone, "[[simulation.swerve_modules]]"},
      {"chassis dynamics", "ROVER_XML", "roller",
       "[simulation.chassis]\nmass_kg = 50.0\n", kNone, "[simulation.chassis]"},
      {"wheel radius error",
       "ROVER_XML",
       "roller",
       "",
       {.wheel_radius_error_frac = 0.1},
       "--wheel-radius-error"},
      {"wheel slip",
       "ROVER_XML",
       "roller",
       "",
       {.wheel_slip_gain = 0.2},
       "--wheel-slip-gain"},
      {"welded root keeps the analytic chassis",
       "ARM_XML",
       "arm",
       "[simulation]\nwheel_radius_m = 0.06\n"
       "[simulation.chassis]\nmass_kg = 50.0\n",
       {.wheel_radius_error_frac = 0.1, .wheel_slip_gain = 0.2},
       nullptr},
  };
  InstallMujocoProcessHandlers();
  int index = 0;
  for (const Row& row : rows) {
    SCOPED_TRACE(row.what);
    const config::RobotConfig config = RowConfig(row, index++);
    std::ostringstream log;
    MujocoGatewayExtension extension(
        {.field = [](std::string_view) -> std::string { return {}; },
         .wall_clock_ns = [] { return int64_t{0}; },
         .log = &log,
         .summary = &log});
    hardware::sim::PhysicsSimBackend backend(row.options);
    if (row.refused == nullptr) {
      EXPECT_NO_THROW(extension.Configure(config, row.options, backend));
      continue;
    }
    try {
      extension.Configure(config, row.options, backend);
      ADD_FAILURE() << "accepted; expected a refusal naming " << row.refused;
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string_view(e.what()).find(row.refused),
                std::string_view::npos)
          << e.what();
    }
  }
}

// REQ-DEV-37: the robot's root body starts where the gateway's start pose
// says: --start-pose when given, else the config's [simulation.start_pose],
// else 2, 2, 0, as the program applies it (ApplyStartPose) before Configure.
TEST(MujocoGatewayExtension, RobotStartsAtTheFlagElseTheConfigsPose) {
  struct Case {
    const char* what;
    std::string toml;
    std::optional<config::SimulationStartPose> flag;
    std::string_view source;
    config::SimulationStartPose at;
  };
  const std::string pose =
      "[simulation.start_pose]\nx_m = 1.907\ny_m = 5.302\nyaw_rad = 2.705\n";
  const Case cases[] = {
      {"default", "", std::nullopt, "default", {2.0, 2.0, 0.0}},
      {"config", pose, std::nullopt, "config", {1.907, 5.302, 2.705}},
      {"flag over config",
       pose,
       config::SimulationStartPose{3.0, 4.0, -1.0},
       "--start-pose",
       {3.0, 4.0, -1.0}},
  };
  InstallMujocoProcessHandlers();
  int index = 100;
  for (const Case& c : cases) {
    SCOPED_TRACE(c.what);
    const config::RobotConfig config = RowConfig(
        {c.what, "ROVER_XML", "roller", c.toml, {}, nullptr}, index++);
    hardware::sim::PhysicsOptions options;
    EXPECT_EQ(ApplyStartPose(config, c.flag, options), c.source);
    std::ostringstream log;
    MujocoGatewayExtension extension(
        {.field = [](std::string_view) -> std::string { return {}; },
         .wall_clock_ns = [] { return int64_t{0}; },
         .log = &log,
         .summary = &log});
    hardware::sim::PhysicsSimBackend backend(options);
    extension.Configure(config, options, backend);
    ASSERT_TRUE(extension.plant()->chassis().has_value());
    const hardware::sim::PlantChassis chassis =
        extension.plant()->chassis().value_or(hardware::sim::PlantChassis{});
    EXPECT_NEAR(chassis.x_m, c.at.x_m, 1e-12);
    EXPECT_NEAR(chassis.y_m, c.at.y_m, 1e-12);
    EXPECT_NEAR(chassis.yaw_rad, c.at.yaw_rad, 1e-12);
  }
}

// REQ-DEV-40: the match input a tick shows the rules is the Driver Station's
// mode (its bits only while one is attached), the gateway's own enable
// (enabled and not e-stopped), the FMS alliance and the schedule clock.
TEST(MujocoGatewayExtension, MatchInputIsTheTicksDriverStation) {
  namespace ds = driver_station;
  using Mode = MatchInput::Mode;
  struct Case {
    const char* what;
    uint32_t flags;
    ds::Alliance alliance;
    std::optional<uint64_t> schedule_us;
    MatchInput expected;
  };
  constexpr uint32_t kOn = ds::kDsAttached | ds::kEnabled;
  const Case cases[] = {
      {"disabled teleop",
       ds::kDsAttached | ds::kTeleop,
       ds::Alliance::kBlue,
       0,
       {Mode::kTeleop, false, false, true, 0}},
      {"autonomous",
       kOn | ds::kAutonomous,
       ds::Alliance::kBlue,
       1500,
       {Mode::kAutonomous, true, false, true, 1500}},
      {"teleop",
       kOn | ds::kTeleop,
       ds::Alliance::kBlue,
       20'000'000,
       {Mode::kTeleop, true, false, true, 20'000'000}},
      {"e-stopped",
       kOn | ds::kTeleop | ds::kEStop,
       ds::Alliance::kBlue,
       7,
       {Mode::kTeleop, false, false, true, 7}},
      {"test",
       kOn | ds::kTest,
       ds::Alliance::kBlue,
       7,
       {Mode::kTest, true, false, true, 7}},
      {"red",
       kOn | ds::kTeleop,
       ds::Alliance::kRed,
       7,
       {Mode::kTeleop, true, true, true, 7}},
      {"no Driver Station: no mode",
       ds::kEnabled | ds::kTeleop,
       ds::Alliance::kBlue,
       7,
       {Mode::kNone, true, false, true, 7}},
      {"test and autonomous: test",
       kOn | ds::kTest | ds::kAutonomous,
       ds::Alliance::kBlue,
       7,
       {Mode::kTest, true, false, true, 7}},
      {"autonomous and teleop: autonomous",
       kOn | ds::kAutonomous | ds::kTeleop,
       ds::Alliance::kBlue,
       7,
       {Mode::kAutonomous, true, false, true, 7}},
      {"before the schedule",
       kOn | ds::kAutonomous,
       ds::Alliance::kRed,
       std::nullopt,
       {Mode::kAutonomous, true, true, false, 0}},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.what);
    ds::DriverStationData station;
    station.fms.flags = c.flags;
    station.fms.alliance = c.alliance;
    const MatchInput input = MatchInputOf({.now_us = 1,
                                           .schedule_us = c.schedule_us,
                                           .driver_station = &station});
    EXPECT_EQ(input.mode, c.expected.mode);
    EXPECT_EQ(input.enabled, c.expected.enabled);
    EXPECT_EQ(input.red_alliance, c.expected.red_alliance);
    EXPECT_EQ(input.scheduled, c.expected.scheduled);
    EXPECT_EQ(input.schedule_us, c.expected.schedule_us);
  }
}

}  // namespace
}  // namespace talos::simulation::mujoco

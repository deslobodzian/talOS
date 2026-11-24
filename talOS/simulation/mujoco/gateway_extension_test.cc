// REQ-DEV-37: a model that owns the chassis (its root body has a free joint)
// replaces the analytic chassis, so MujocoGatewayExtension::Configure
// refuses each analytic chassis key and odometry-error flag, naming it; a
// welded-root model keeps the analytic chassis and accepts them.

#include "talOS/simulation/mujoco/gateway_extension.h"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/hardware/sim_physics.h"
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

}  // namespace
}  // namespace talos::simulation::mujoco

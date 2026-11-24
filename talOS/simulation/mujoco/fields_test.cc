// REQ-DEV-37: every field name config load accepts
// (config::kSimulationMujocoFields) is a model mujoco_gateway finds in its
// runfiles and loads (the compiled world holds that field's FUEL), and the
// gateway knows no field config load would refuse.

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <string>
#include <string_view>

#include "talOS/configuration/config_parser.h"
#include "talOS/simulation/mujoco/gateway_extension.h"
#include "talOS/simulation/mujoco/plant.h"

namespace talos::simulation::mujoco {
namespace {

// The FUEL bodies (`fuel_<n>`) each field stages; a field added to the
// config's list adds its count here.
const std::map<std::string_view, int> kFuelBodies = {{"rebuilt_2026", 504}};

int FuelBodies(const MujocoPlant& plant) {
  const mjModel* m = plant.model();
  int count = 0;
  for (int b = 0; b < m->nbody; ++b) {
    const char* name = mj_id2name(m, mjOBJ_BODY, b);
    count += name != nullptr && std::string_view(name).starts_with("fuel_");
  }
  return count;
}

bool Contains(auto&& names, std::string_view name) {
  return std::find(std::begin(names), std::end(names), name) != std::end(names);
}

// REQ-DEV-37
TEST(MujocoGatewayFields, EveryConfigFieldLoadsAndNoOtherExists) {
  const std::string robot = ::testing::TempDir() + "/fields_robot.xml";
  std::ofstream(robot) << R"(<mujoco><worldbody>
  <body name="probe_base" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
  </body>
</worldbody></mujoco>)";
  InstallMujocoProcessHandlers();
  const FieldResolver resolve = RunfilesFieldResolver("");
  for (const std::string_view field : config::kSimulationMujocoFields) {
    SCOPED_TRACE(std::string(field));
    EXPECT_TRUE(Contains(GatewayFields(), field));
    ASSERT_TRUE(kFuelBodies.contains(field));
    const auto plant =
        MakePlant({.field_xml = resolve(field), .robot_xml = robot}, {});
    EXPECT_EQ(FuelBodies(*plant), kFuelBodies.at(field));
  }
  for (const std::string_view field : GatewayFields()) {
    EXPECT_TRUE(Contains(config::kSimulationMujocoFields, field)) << field;
  }
}

}  // namespace
}  // namespace talos::simulation::mujoco

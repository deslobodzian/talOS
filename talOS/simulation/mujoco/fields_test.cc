// REQ-DEV-37: every field name config load accepts
// (config::kSimulationMujocoFields) is a model the MuJoCo gateway finds in its
// runfiles and loads (the compiled world holds that field's FUEL), a field
// with a game attaches its rules to that world, and the gateway knows no
// field config load would refuse.

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <span>
#include <string>
#include <string_view>

#include "sim/sim_generated.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/simulation/mujoco/game.h"
#include "talOS/simulation/mujoco/gateway_extension.h"
#include "talOS/simulation/mujoco/plant.h"

namespace talos::simulation::mujoco {
namespace {

// The FUEL bodies (`fuel_<n>`) each field stages; a field added to the
// config's list adds its count here.
const std::map<std::string_view, int> kFuelBodies = {{"rebuilt_2026", 504}};
// The fields that carry a game.
const std::set<std::string_view> kGames = {"rebuilt_2026"};

int FuelBodies(const MujocoPlant& plant) {
  const mjModel* m = plant.model();
  int count = 0;
  for (int b = 0; b < m->nbody; ++b) {
    const char* name = mj_id2name(m, mjOBJ_BODY, b);
    count += name != nullptr && std::string_view(name).starts_with("fuel_");
  }
  return count;
}

bool Contains(std::span<const FieldEntry> fields, std::string_view name) {
  return std::any_of(fields.begin(), fields.end(),
                     [name](const FieldEntry& f) { return f.name == name; });
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
  for (const FieldEntry& field : GatewayFields()) {
    SCOPED_TRACE(std::string(field.name));
    EXPECT_NE(std::find(std::begin(config::kSimulationMujocoFields),
                        std::end(config::kSimulationMujocoFields), field.name),
              std::end(config::kSimulationMujocoFields));
    EXPECT_EQ(field.make_rules != nullptr, kGames.contains(field.name));
    if (field.make_rules == nullptr) continue;
    const auto plant =
        MakePlant({.field_xml = resolve(field.name), .robot_xml = robot}, {});
    const std::unique_ptr<GameRules> rules = field.make_rules(GameOptions{});
    plant->AddRules(*rules);  // throws on a contract the model lacks
    ::sim::MatchState match;
    rules->Report(match);
    EXPECT_GT(match.pieces_in_play_count(), 0);
  }
}

}  // namespace
}  // namespace talos::simulation::mujoco

// MuJoCo's sleeping on the 2026 field through MujocoPlant (REQ-DEV-37), as
// [simulation.mujoco] sleeping = true asks for it: resting FUEL fall asleep,
// no robot tree does (a free root's, or a welded root's passive child), and
// a sleeping FUEL the robot strikes wakes and moves. Asleep is MuJoCo's own
// state (mjData::tree_asleep), never a wall-clock time.

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/simulation/mujoco/plant.h"

namespace talos::simulation::mujoco {
namespace {

std::string FieldXml() {
  std::string error;
  const std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* path = std::getenv("FIELD_XML");
  if (!runfiles || path == nullptr) {
    throw std::runtime_error("runfiles: " + error + ", FIELD_XML unset");
  }
  return runfiles->Rlocation(path);
}

// A free-root box robot with the field's robot bits and no actuator, so
// MuJoCo alone would let it sleep at rest; 30 kg, 0.6 m square.
constexpr const char* kRobot = R"(<mujoco><compiler angle="radian"/>
<default><geom contype="4" conaffinity="21"/></default>
<worldbody>
  <body name="box" pos="0 0 0.06">
    <freejoint name="box_root" align="false"/>
    <geom name="box_shell" type="box" size="0.3 0.3 0.05" mass="30"/>
  </body>
</worldbody></mujoco>)";

// The 2026 field's reserve row (fields/rebuilt_2026/README.md): FUEL 456 to
// 503 on the carpet outside the field at y = -2.0 m, from x = 1.0 m, 0.2 m
// apart.
constexpr int kFirstReserve = 456;
constexpr int kFuel = 504;
constexpr int kStruck = 466;  // at x = 3.0 m

int FuelBody(const mjModel* m, int k) {
  std::array<char, 16> name{};
  std::snprintf(name.data(), name.size(), "fuel_%03d", k);
  const int body = mj_name2id(m, mjOBJ_BODY, name.data());
  if (body < 0)
    throw std::logic_error("no FUEL body " + std::string(name.data()));
  return body;
}

bool Asleep(const mjModel* m, const mjData* d, int body) {
  return d->tree_asleep[m->body_treeid[body]] >= 0;
}

// REQ-DEV-37: with sleeping on, after 2 s at rest every FUEL sleeps and the
// robot does not; the robot driven at 2 m/s into a sleeping FUEL wakes it
// and moves it.
TEST(MujocoSleeping, RestingFuelSleepsTheRobotNeverAndAStruckFuelWakes) {
  InstallMujocoProcessHandlers();
  PlantModel model;
  model.field_xml = FieldXml();
  model.robot_xml = ::testing::TempDir() + "/sleeping_robot.xml";
  std::ofstream(model.robot_xml) << kRobot;
  // 0.6 m short of the struck FUEL's centre, in -y, facing nothing else.
  model.start_x_m = 3.0;
  model.start_y_m = -1.4;
  model.sleeping = true;
  auto plant = MakePlant(model, {});
  const mjModel* m = plant->model();
  // The test drives the robot by its root velocity between steps.
  auto* d = const_cast<mjData*>(plant->data());
  const int robot_tree = m->body_treeid[plant->robot_root_body()];
  ASSERT_GE(robot_tree, 0);

  for (int step = 0; step < 400; ++step) {
    ASSERT_TRUE(plant->Step(1, {})) << plant->failure();
  }
  EXPECT_LT(d->tree_asleep[robot_tree], 0) << "the robot fell asleep";
  int awake = 0;
  for (int k = 0; k < kFuel; ++k) awake += !Asleep(m, d, FuelBody(m, k));
  EXPECT_EQ(awake, 0) << "FUEL still awake after 2 s at rest";

  const int struck = FuelBody(m, kStruck);
  ASSERT_TRUE(Asleep(m, d, struck));
  const mjtNum* position = d->xpos + 3 * static_cast<std::ptrdiff_t>(struck);
  const std::array<double, 2> before = {position[0], position[1]};
  const int dof = m->jnt_dofadr[m->body_jntadr[plant->robot_root_body()]];
  // 0.4 s at 2 m/s in -y: the box's front edge travels 0.8 m, past the
  // FUEL's centre 0.3 m beyond it.
  for (int step = 0; step < 80; ++step) {
    d->qvel[dof + 0] = 0.0;
    d->qvel[dof + 1] = -2.0;
    ASSERT_TRUE(plant->Step(1, {})) << plant->failure();
  }
  const double moved =
      std::hypot(position[0] - before[0], position[1] - before[1]);
  EXPECT_GT(moved, 0.1) << "the struck FUEL stayed where it slept";
  // Waking is local: the row's far end (x = 1.0 m), untouched, still sleeps.
  EXPECT_TRUE(Asleep(m, d, FuelBody(m, kFirstReserve)));
}

// A welded robot (its root has no joint, so it is no tree) with a passive
// flap hanging at rest from a hinge: the flap is a tree of its own, which
// MuJoCo alone would let sleep.
constexpr const char* kWeldedRobot = R"(<mujoco><compiler angle="radian"/>
<default><geom contype="4" conaffinity="21"/></default>
<worldbody>
  <body name="frame" pos="0 0 0.5">
    <geom name="frame_post" type="box" size="0.05 0.05 0.45" pos="0 0 -0.05"/>
    <body name="flap" pos="0.2 0 0">
      <joint name="flap_hinge" type="hinge" axis="0 1 0" damping="0.01"/>
      <geom name="flap_plate" type="box" size="0.01 0.1 0.1" pos="0 0 -0.1"
            mass="0.5" contype="0" conaffinity="0"/>
    </body>
  </body>
</worldbody></mujoco>)";

// REQ-DEV-37: no robot tree sleeps, not only the root's: with a welded root,
// a passive flap at rest stays awake for 2 s while a FUEL beside it sleeps.
TEST(MujocoSleeping, AWeldedRobotsPassiveTreeNeverSleeps) {
  InstallMujocoProcessHandlers();
  PlantModel model;
  model.field_xml = FieldXml();
  model.robot_xml = ::testing::TempDir() + "/sleeping_welded_robot.xml";
  std::ofstream(model.robot_xml) << kWeldedRobot;
  model.start_x_m = 3.0;  // 0.6 m from the reserve row's FUEL at x = 3.0 m
  model.start_y_m = -1.4;
  model.sleeping = true;
  auto plant = MakePlant(model, {});
  const mjModel* m = plant->model();
  const mjData* d = plant->data();
  ASSERT_LT(m->body_treeid[plant->robot_root_body()], 0) << "root not welded";
  const int flap = mj_name2id(m, mjOBJ_BODY, "flap");
  ASSERT_GE(flap, 0);
  ASSERT_GE(m->body_treeid[flap], 0);

  for (int step = 0; step < 400; ++step) {
    ASSERT_TRUE(plant->Step(1, {})) << plant->failure();
  }
  EXPECT_FALSE(Asleep(m, d, flap)) << "the robot's passive flap fell asleep";
  EXPECT_TRUE(Asleep(m, d, FuelBody(m, kStruck)));
}

}  // namespace
}  // namespace talos::simulation::mujoco

// The scripted shooter (REQ-DEV-39) on MujocoPlant, on inline MJCF: where
// and how fast a launch leaves, what it takes from the flywheel, which
// bodies it launches and when, how far apart in flight and how long a body
// waits, the contract's refusals, and that it is repeatable and allocates
// nothing.

#include "talOS/simulation/mujoco/scripted_shooter.h"

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/hardware/sim_plant.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/simulation/mujoco/plant.h"
#include "talOS/simulation/mujoco/world_rules.h"

namespace talos::simulation::mujoco {
namespace {

using hardware::ConfigError;
using hardware::sim::PlantDrive;
using hardware::sim::PlantMotor;
using Vec3 = std::array<double, 3>;

constexpr double kH = 0.005;  // the option table's step
// The contract's numbers in the robot below.
constexpr double kEfficiency = 0.5;
constexpr double kContactRadius = 0.0508;
constexpr double kExitPitch = 0.7;  // the exit's +x, pitched up from +x
// The flywheel: hinge about the root's z through its origin. Its inertia is
// the wheel's about the axis plus the joint's and the actuator's armature
// (gear 1).
struct Wheel {
  double inertia;
  double joint_armature;
  double actuator_armature;
  double total() const { return inertia + joint_armature + actuator_armature; }
};
constexpr Wheel kWheel{0.0025, 0.0005, 0.0003};
// A wheel as light as m·r_c² of the ball below: it slows a lot per launch.
constexpr Wheel kLightWheel{0.0003, 0.0, 0.0};
// A ball of radius 0.075 m, 0.2 kg, inertia 0.6·m·r² (beta 0.6, not a solid
// sphere's 0.4).
constexpr double kBallRadius = 0.075;
constexpr double kBallMass = 0.2;
constexpr double kBeta = 0.6;

// A Kraken X60's winding with no no-load drag, so a flywheel nothing drives
// keeps its speed exactly.
PlantMotor Motor(std::string_view name) {
  return PlantMotor{name, 1.0, 0.01937, 0.01899, 0.0328, 0.0};
}

std::unique_ptr<MujocoPlant> Make(const std::string& name,
                                  const std::string& robot_xml,
                                  const std::string& field_xml,
                                  bool sleeping = false) {
  InstallMujocoProcessHandlers();
  PlantModel model;
  model.sleeping = sleeping;
  model.robot_xml = ::testing::TempDir() + "/" + name + "_robot.xml";
  model.field_xml = ::testing::TempDir() + "/" + name + "_field.xml";
  std::ofstream(model.robot_xml) << robot_xml;
  std::ofstream(model.field_xml) << field_xml;
  const std::vector<PlantMotor> motors = {Motor("flywheel")};
  return MakePlant(model, motors);
}

// A robot whose root, at height 0.5, holds the flywheel on its z axis
// through the root's origin, so the robot's centre of mass stays there and a
// free root's twist is constant; the inlet is a 0.2 m box at `inlet_pos`, the
// exit at (0.3, 0, 0.2) pitched up by `exit_pitch`.
std::string Robot(bool free_root, std::string_view inlet_pos = "0 0 0.3",
                  const Wheel& wheel = kWheel, double exit_pitch = kExitPitch) {
  return std::string(R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="frame" pos="0 0 0.5">)") +
         (free_root ? R"(<freejoint name="root"/>)" : "") +
         R"(<inertial pos="0 0 0" mass="10" diaginertia="1 1 2"/>
    <site name="flywheel_inlet" type="box" size="0.1 0.1 0.1" pos=")" +
         std::string(inlet_pos) + R"("/>
    <site name="flywheel_exit" pos="0.3 0 0.2" euler="0 )" +
         std::to_string(-exit_pitch) + R"( 0"/>
    <body name="wheel"><joint name="flywheel_joint" axis="0 0 1" armature=")" +
         std::to_string(wheel.joint_armature) + R"("/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 )" +
         std::to_string(wheel.inertia) + R"("/></body>
  </body>
</worldbody>
<custom><numeric name="flywheel_shooter" data="0.5 0.0508"/></custom>
<actuator><motor name="flywheel" joint="flywheel_joint" armature=")" +
         std::to_string(wheel.actuator_armature) + R"("/></actuator>
</mujoco>)";
}

// A ball that touches nothing unless `collides` (then the field's fuel bits).
std::string Ball(std::string_view name, const Vec3& at, bool collides = false) {
  return std::string(R"(<body name=")") + std::string(name) + R"(" pos=")" +
         std::to_string(at[0]) + " " + std::to_string(at[1]) + " " +
         std::to_string(at[2]) + R"("><freejoint/>
    <inertial pos="0 0 0" mass="0.2" diaginertia="0.000675 0.000675 0.000675"/>
    <geom type="sphere" size="0.075" )" +
         (collides ? R"(contype="2" conaffinity="15")"
                   : R"(contype="0" conaffinity="0")") +
         "/></body>";
}

std::string Field(const std::string& bodies, bool gravity = false) {
  return std::string("<mujoco>") +
         (gravity ? "" : R"(<option gravity="0 0 0"/>)") + R"(<worldbody>
  <geom name="carpet_robot" type="plane" size="5 5 0.1" contype="16" conaffinity="0"/>
  <geom name="carpet_fuel" type="plane" size="5 5 0.1" contype="8" conaffinity="0"/>
)" + bodies +
         "</worldbody></mujoco>";
}

// The world as a test sets it up before the first Step (the plant's own
// accessors are const: outside Step nothing else writes it).
mjData* Writable(MujocoPlant& plant) {
  return const_cast<mjData*>(plant.data());
}

int Dof(const mjModel* m, const char* joint) {
  return m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, joint)];
}

// A body's free-joint position and world-frame linear and angular velocity.
struct BodyState {
  Vec3 position;
  Vec3 velocity;
  Vec3 spin;
};
BodyState StateOf(const MujocoPlant& plant, const char* body) {
  const mjModel* m = plant.model();
  const mjData* d = plant.data();
  const int joint = m->body_jntadr[mj_name2id(m, mjOBJ_BODY, body)];
  const mjtNum* q = d->qpos + m->jnt_qposadr[joint];
  const mjtNum* v = d->qvel + m->jnt_dofadr[joint];
  BodyState state{{q[0], q[1], q[2]}, {v[0], v[1], v[2]}, {}};
  mju_rotVecQuat(state.spin.data(), v + 3, q + 3);  // local to world
  return state;
}

Vec3 RotateZ(double yaw, const Vec3& v) {
  return {std::cos(yaw) * v[0] - std::sin(yaw) * v[1],
          std::sin(yaw) * v[0] + std::cos(yaw) * v[1], v[2]};
}

void ExpectNear(const Vec3& actual, const Vec3& expected, const char* what) {
  for (int k = 0; k < 3; ++k) {
    EXPECT_NEAR(actual[k], expected[k], 1e-9) << what << "[" << k << "]";
  }
}

ScriptedShooters& Attach(MujocoPlant& plant,
                         std::unique_ptr<WorldRules>& rules) {
  rules = MakeScriptedShooters(*plant.model());
  if (!rules) throw std::logic_error("the model carries no scripted shooter");
  plant.AddRules(*rules);
  return static_cast<ScriptedShooters&>(*rules);
}

// REQ-DEV-39: a body in the inlet leaves from the exit at the exit's own
// velocity plus, along the exit's +x, efficiency × contact radius × the speed
// the launch leaves the flywheel with, rolling about its +y; the flywheel
// loses exactly the launch's angular momentum, (1 + beta)/2·m·v·r_c, over its
// inertia with armature, and the launch adds no energy: on a robot at rest,
// on one moving at (1, 2) m/s and turning at 3 rad/s, whose exit has moved
// and turned by the end of the step, and with a light flywheel.
// REQ-DEV-37: the same with MuJoCo's sleeping on, the body asleep in the
// inlet when it is launched; the launch wakes it, so it flies on.
class LaunchesFromTheExit : public ::testing::TestWithParam<bool> {};

TEST_P(LaunchesFromTheExit, AndDebitsTheFlywheel) {
  const bool sleeping = GetParam();
  struct Case {
    const char* what;
    Vec3 velocity;
    double yaw_rate;
    Wheel wheel;
  };
  for (const Case& row :
       {Case{"at rest", {0, 0, 0}, 0.0, kWheel},
        Case{"moving and turning", {1.0, 2.0, 0.0}, 3.0, kWheel},
        Case{"light flywheel", {0, 0, 0}, 0.0, kLightWheel}}) {
    SCOPED_TRACE(row.what);
    auto plant = Make("launch", Robot(true, "0 0 0.3", row.wheel),
                      Field(Ball("ball", {0.0, 0.0, 0.8})), sleeping);
    std::unique_ptr<WorldRules> rules;
    ScriptedShooters& shooters = Attach(*plant, rules);
    const mjModel* m = plant->model();
    mjData* d = Writable(*plant);
    const int ball_tree = m->body_treeid[mj_name2id(m, mjOBJ_BODY, "ball")];
    if (sleeping) {
      // Robot and ball at rest, the flywheel stopped (nothing launches): the
      // ball falls asleep; nothing moves the robot's root from the origin.
      for (int step = 0; step < 20; ++step) ASSERT_TRUE(plant->Step(1, {}));
      ASSERT_GE(d->tree_asleep[ball_tree], 0) << "the ball never slept";
      ASSERT_EQ(shooters.shots(), 0u);
    }
    constexpr double kSpeed = 300.0;  // rad/s, before the launch
    const int root = Dof(m, "root");
    for (int k = 0; k < 3; ++k) d->qvel[root + k] = row.velocity[k];
    d->qvel[root + 5] = row.yaw_rate;  // local z is world z
    d->qvel[Dof(m, "flywheel_joint")] = kSpeed;

    ASSERT_TRUE(plant->Step(1, {}));
    ASSERT_EQ(shooters.shots(), 1u);
    // The exit at the end of the step.
    const double yaw = row.yaw_rate * kH;
    const Vec3 offset = RotateZ(yaw, {0.3, 0.0, 0.2});
    const Vec3 root_at = {row.velocity[0] * kH, row.velocity[1] * kH, 0.5};
    const Vec3 axis =
        RotateZ(yaw, {std::cos(kExitPitch), 0.0, std::sin(kExitPitch)});
    const Vec3 spin_axis = RotateZ(yaw, {0.0, 1.0, 0.0});
    const Vec3 exit_velocity = {row.velocity[0] - row.yaw_rate * offset[1],
                                row.velocity[1] + row.yaw_rate * offset[0],
                                0.0};
    // v = eta r_c (w - k v), k = (1 + beta)/2 m r_c / J.
    const double inertia = row.wheel.total();
    const double loss_per_mps =
        (1.0 + kBeta) / 2.0 * kBallMass * kContactRadius / inertia;
    const double launch = kEfficiency * kContactRadius * kSpeed /
                          (1.0 + kEfficiency * kContactRadius * loss_per_mps);

    const BodyState ball = StateOf(*plant, "ball");
    Vec3 position{};
    Vec3 velocity{};
    Vec3 spin{};
    for (int k = 0; k < 3; ++k) {
      position[k] = root_at[k] + offset[k];
      velocity[k] = exit_velocity[k] + launch * axis[k];
      spin[k] =
          (k == 2 ? row.yaw_rate : 0.0) + launch / kBallRadius * spin_axis[k];
    }
    ExpectNear(ball.position, position, "position");
    ExpectNear(ball.velocity, velocity, "velocity");
    ExpectNear(ball.spin, spin, "spin");

    const double after = plant->data()->qvel[Dof(m, "flywheel_joint")];
    EXPECT_NEAR(after, kSpeed - loss_per_mps * launch, 1e-9);
    EXPECT_NEAR(launch, kEfficiency * kContactRadius * after, 1e-9)
        << "the body did not leave at the speed the flywheel is left with";

    // Measured relative to the exit: the momentum the launch carries is what
    // the flywheel loses, and the body gains no more energy than it loses.
    Vec3 relative{};
    Vec3 rolling{};
    for (int k = 0; k < 3; ++k) {
      relative[k] = ball.velocity[k] - exit_velocity[k];
      rolling[k] = ball.spin[k] - (k == 2 ? row.yaw_rate : 0.0);
    }
    const auto norm = [](const Vec3& v) {
      return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    };
    const double speed = norm(relative);
    EXPECT_NEAR(inertia * (kSpeed - after),
                (1.0 + kBeta) / 2.0 * kBallMass * speed * kContactRadius, 1e-9)
        << "the flywheel did not lose the launch's angular momentum";
    const double gained = 0.5 * kBallMass * speed * speed +
                          0.5 * kBeta * kBallMass * kBallRadius * kBallRadius *
                              norm(rolling) * norm(rolling);
    EXPECT_LE(gained, 0.5 * inertia * (kSpeed * kSpeed - after * after));

    // Awake, it flies on: one more step moves it by its velocity times the
    // step (no gravity, no contact).
    EXPECT_LT(d->tree_asleep[ball_tree], 0) << "the launch left it asleep";
    ASSERT_TRUE(plant->Step(1, {}));
    const BodyState next = StateOf(*plant, "ball");
    for (int k = 0; k < 3; ++k) {
      position[k] += kH * ball.velocity[k];
    }
    ExpectNear(next.position, position, "position a step later");
  }
  if (sleeping) return;
  // The light flywheel by hand: v = 4.5138 m/s, the wheel left at 177.71
  // rad/s (the pre-launch speed would give 7.62 m/s).
  {
    auto plant = Make("light", Robot(false, "0 0 0.3", kLightWheel),
                      Field(Ball("ball", {0.0, 0.0, 0.8})));
    std::unique_ptr<WorldRules> rules;
    Attach(*plant, rules);
    const int wheel = Dof(plant->model(), "flywheel_joint");
    Writable(*plant)->qvel[wheel] = 300.0;
    ASSERT_TRUE(plant->Step(1, {}));
    const BodyState ball = StateOf(*plant, "ball");
    EXPECT_NEAR(
        std::hypot(ball.velocity[0], ball.velocity[1], ball.velocity[2]),
        4.51376, 1e-5);
    EXPECT_NEAR(plant->data()->qvel[wheel], 177.70715, 1e-5);
  }
}

INSTANTIATE_TEST_SUITE_P(ScriptedShooter, LaunchesFromTheExit,
                         ::testing::Bool(),
                         [](const ::testing::TestParamInfo<bool>& info) {
                           return info.param ? std::string("SleepingOn")
                                             : std::string("SleepingOff");
                         });

// REQ-DEV-39: the robot's own bodies are never launched: a free root whose
// origin sits inside its own inlet stays where it is.
TEST(ScriptedShooter, NeverLaunchesTheRobot) {
  auto plant = Make("robot_body", Robot(true, "0 0 0"), Field(""));
  std::unique_ptr<WorldRules> rules;
  ScriptedShooters& shooters = Attach(*plant, rules);
  Writable(*plant)->qvel[Dof(plant->model(), "flywheel_joint")] = 300.0;
  ASSERT_TRUE(plant->Step(10, {}));
  EXPECT_EQ(shooters.shots(), 0u);
  ExpectNear(StateOf(*plant, "frame").position, {0.0, 0.0, 0.5}, "root");
}

// A flywheel at rest or turning backward launches nothing; turning forward
// it launches one body at a time, the next only once the last is clear of the
// exit by their two radii (here, without gravity, by two radii and the
// spacing's 1 cm), and each body once.
TEST(ScriptedShooter, LaunchesOneBodyAtATimeOnlyWhenTurningForward) {
  auto plant = Make("one_at_a_time", Robot(false),
                    Field(Ball("first", {0.0, 0.0, 0.78}) +
                          Ball("second", {0.0, 0.0, 0.82})));
  std::unique_ptr<WorldRules> rules;
  ScriptedShooters& shooters = Attach(*plant, rules);
  const int wheel = Dof(plant->model(), "flywheel_joint");
  mjData* d = Writable(*plant);
  for (const double speed : {0.0, -50.0}) {
    d->qvel[wheel] = speed;
    ASSERT_TRUE(plant->Step(1, {}));
    EXPECT_EQ(shooters.shots(), 0u) << "at " << speed << " rad/s";
  }
  ExpectNear(StateOf(*plant, "second").position, {0.0, 0.0, 0.82}, "second");

  d->qvel[wheel] = 300.0;
  ASSERT_TRUE(plant->Step(1, {}));
  ASSERT_EQ(shooters.shots(), 1u);
  ExpectNear(StateOf(*plant, "second").position, {0.0, 0.0, 0.82},
             "second, while the first is at the exit");
  // The first leaves at 7.17 m/s, 0.0359 m a step: 0.179 m, past the two
  // radii and 1 cm, after five steps (0.143 m after four).
  for (int step = 1; step <= 4; ++step) {
    ASSERT_TRUE(plant->Step(1, {}));
    EXPECT_EQ(shooters.shots(), 1u) << "step " << step;
  }
  ASSERT_TRUE(plant->Step(1, {}));
  EXPECT_EQ(shooters.shots(), 2u);
  ASSERT_TRUE(plant->Step(40, {}));
  EXPECT_EQ(shooters.shots(), 2u);
}

// REQ-DEV-39: two bodies in the inlet together leave far enough apart not
// to meet in flight. On one arc the two are closest near the apex, where the
// first moves at only v·cos(theta): at 60 degrees, launched as soon as the
// first is its two radii from the exit, the second comes within 0.08 m of it
// there. With gravity, a 60 degree exit and a flywheel heavy enough that both
// leave at one speed, their centres stay more than two radii plus 1 cm apart
// from the second's launch until the first has fallen back to the exit's
// height, and the second waits no more than the spacing needs.
TEST(ScriptedShooter, SecondLaunchDoesNotMeetTheFirstInFlight) {
  constexpr Wheel kHeavyWheel{0.0025, 1.0, 0.0};  // armature 1 kg·m²
  constexpr double kPitch = std::numbers::pi / 3.0;
  constexpr double kClearance = 2.0 * kBallRadius + 0.01;
  auto plant = Make(
      "in_flight", Robot(false, "0 0 0.3", kHeavyWheel, kPitch),
      Field(Ball("first", {0.0, 0.0, 0.78}) + Ball("second", {0.0, 0.0, 0.86}),
            true));
  std::unique_ptr<WorldRules> rules;
  ScriptedShooters& shooters = Attach(*plant, rules);
  Writable(*plant)->qvel[Dof(plant->model(), "flywheel_joint")] = 300.0;
  constexpr double kExitZ = 0.7;  // the exit's height, root at 0.5
  int second_at = 0;
  double closest = 1e9;
  for (int step = 1; step <= 400; ++step) {
    ASSERT_TRUE(plant->Step(1, {}));
    if (step == 1) {
      ASSERT_EQ(shooters.shots(), 1u);
    }
    if (second_at == 0 && shooters.shots() == 2) second_at = step;
    if (second_at == 0) continue;
    const BodyState first = StateOf(*plant, "first");
    const BodyState second = StateOf(*plant, "second");
    if (first.position[2] < kExitZ && first.velocity[2] < 0.0) break;
    closest =
        std::min(closest, std::hypot(first.position[0] - second.position[0],
                                     first.position[1] - second.position[1],
                                     first.position[2] - second.position[2]));
  }
  ASSERT_GT(second_at, 0) << "the second body was never launched";
  EXPECT_GT(closest, kClearance) << "the second launch, at step " << second_at
                                 << ", meets the first in flight";
  // Both leave at 7.6 m/s, 3.8 m/s across the ground: the spacing needs
  // 0.16 / 3.8 = 42 ms, nine steps after the first's.
  EXPECT_LE(second_at, 10) << "the second waits longer than the spacing needs";
}

// REQ-DEV-39: the spacing never stalls a shooter: a body waits at most
// 0.5 s after the shooter's last launch, even while that body stands still
// in its path.
TEST(ScriptedShooter, WaitsAtMostHalfASecondForTheLastLaunch) {
  auto plant = Make("stalled", Robot(false),
                    Field(Ball("first", {0.0, 0.0, 0.78}) +
                          Ball("second", {0.0, 0.0, 0.82})));
  std::unique_ptr<WorldRules> rules;
  ScriptedShooters& shooters = Attach(*plant, rules);
  const mjModel* m = plant->model();
  mjData* d = Writable(*plant);
  d->qvel[Dof(m, "flywheel_joint")] = 300.0;
  ASSERT_TRUE(plant->Step(1, {}));
  ASSERT_EQ(shooters.shots(), 1u);
  // The first stops 0.2 m along the exit's axis from the exit (0.3, 0, 0.7):
  // clear of the exit by more than two radii, in the second's path.
  const int first = m->body_jntadr[mj_name2id(m, mjOBJ_BODY, "first")];
  mjtNum* at = d->qpos + m->jnt_qposadr[first];
  at[0] = 0.3 + 0.2 * std::cos(kExitPitch);
  at[2] = 0.7 + 0.2 * std::sin(kExitPitch);
  for (int k = 0; k < 6; ++k) d->qvel[m->jnt_dofadr[first] + k] = 0.0;
  ASSERT_TRUE(plant->Step(90, {}));  // 0.45 s after the first's launch
  EXPECT_EQ(shooters.shots(), 1u) << "launched into the first";
  ASSERT_TRUE(plant->Step(11, {}));  // 0.505 s
  EXPECT_EQ(shooters.shots(), 2u) << "still waiting 0.5 s after the first";
}

// REQ-DEV-39: the spacing looks at the last launch only until it falls back
// to the exit's height: a leader parked below the exit, on the straight line
// of a launch pitched 0.3 rad down and 0.3 m from the exit (clear of it by
// more than two radii), does not delay the next launch, which leaves on the
// first step the exit is clear.
TEST(ScriptedShooter, ALeaderBelowTheExitDoesNotDelayTheNextLaunch) {
  constexpr double kPitch = -0.3;
  auto plant = Make(
      "below", Robot(false, "0 0 0.3", kWheel, kPitch),
      Field(Ball("first", {0.0, 0.0, 0.78}) + Ball("second", {0.0, 0.0, 0.86}),
            true));
  std::unique_ptr<WorldRules> rules;
  ScriptedShooters& shooters = Attach(*plant, rules);
  const mjModel* m = plant->model();
  mjData* d = Writable(*plant);
  d->qvel[Dof(m, "flywheel_joint")] = 300.0;
  ASSERT_TRUE(plant->Step(1, {}));
  ASSERT_EQ(shooters.shots(),
            1u);  // the second waits: the first is at the exit
  const int first = m->body_jntadr[mj_name2id(m, mjOBJ_BODY, "first")];
  mjtNum* at = d->qpos + m->jnt_qposadr[first];
  at[0] = 0.3 + 0.3 * std::cos(kPitch);
  at[2] = 0.7 + 0.3 * std::sin(kPitch);
  for (int k = 0; k < 6; ++k) d->qvel[m->jnt_dofadr[first] + k] = 0.0;
  ASSERT_TRUE(plant->Step(1, {}));
  EXPECT_EQ(shooters.shots(), 2u)
      << "waited for a leader already below the exit's height";
}

// REQ-DEV-39: a contract with a part missing or malformed is refused,
// naming the element.
TEST(ScriptedShooter, RefusesAPartialContractNamingTheElement) {
  const auto robot = [](std::string_view sites, std::string_view numeric,
                        std::string_view actuator) {
    return std::string(R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="frame" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="10" diaginertia="1 1 2"/>)") +
           std::string(sites) + R"(
    <body name="wheel"><joint name="flywheel_joint" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.0025"/></body>
    <body name="carriage"><joint name="carriage_slide" type="slide" axis="1 0 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body>
  </body>
</worldbody>
<tendon><fixed name="belt"><joint joint="flywheel_joint" coef="1"/></fixed></tendon>
<custom>)" +
           std::string(numeric) + "</custom><actuator>" +
           std::string(actuator) + "</actuator></mujoco>";
  };
  const std::string inlet =
      R"(<site name="flywheel_inlet" type="box" size="0.1 0.1 0.1" pos="0 0 0.3"/>)";
  const std::string exit = R"(<site name="flywheel_exit" pos="0.3 0 0.2"/>)";
  const std::string numeric =
      R"(<numeric name="flywheel_shooter" data="0.5 0.0508"/>)";
  const std::string motor =
      R"(<motor name="flywheel" joint="flywheel_joint"/>)";
  struct Case {
    const char* what;
    std::string xml;
    const char* names;
    std::string field;  // the field's worldbody
  };
  const std::vector<Case> cases = {
      {"inlet without exit", robot(inlet, numeric, motor), "'flywheel_exit'"},
      {"no numeric", robot(inlet + exit, "", motor), "'flywheel_shooter'"},
      {"numeric with one value",
       robot(inlet + exit, R"(<numeric name="flywheel_shooter" data="0.5"/>)",
             motor),
       "'flywheel_shooter' holds 1"},
      {"flywheel motor that is no actuator",
       robot(R"(<site name="ghost_inlet" type="box" size="0.1 0.1 0.1"/>
             <site name="ghost_exit" pos="0.3 0 0"/>)",
             R"(<numeric name="ghost_shooter" data="0.5 0.0508"/>)", motor),
       "'ghost'"},
      {"inlet not a box",
       robot(R"(<site name="flywheel_inlet" size="0.1" pos="0 0 0.3"/>)" + exit,
             numeric, motor),
       "'flywheel_inlet' is not a box"},
      {"slide flywheel",
       robot(inlet + exit, numeric,
             R"(<motor name="flywheel" joint="carriage_slide"/>)"),
       "'carriage_slide'"},
      {"tendon flywheel",
       robot(inlet + exit, numeric,
             R"(<motor name="flywheel" tendon="belt"/>)"),
       "drives a tendon"},
      {"efficiency above 1",
       robot(inlet + exit,
             R"(<numeric name="flywheel_shooter" data="1.5 0.0508"/>)", motor),
       "efficiency"},
      {"contact radius zero",
       robot(inlet + exit, R"(<numeric name="flywheel_shooter" data="0.5 0"/>)",
             motor),
       "contact radius"},
      {"negative gear",
       robot(inlet + exit, numeric,
             R"(<motor name="flywheel" joint="flywheel_joint" gear="-1"/>)"),
       "gear"},
      {"exit on the field, not the robot", robot(inlet, numeric, motor),
       "'flywheel_exit' is not on the robot",
       R"(<site name="flywheel_exit" pos="1 0 0.5"/>)"},
  };
  for (const Case& row : cases) {
    SCOPED_TRACE(row.what);
    auto plant = Make("partial", row.xml, Field(row.field));
    try {
      MakeScriptedShooters(*plant->model());
      ADD_FAILURE() << "not refused";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string_view(error.what()).find(row.names),
                std::string_view::npos)
          << error.what();
    }
  }
  // An exit inside its own inlet would launch every launched body again.
  auto plant = Make("exit_inside", Robot(true, "0.3 0 0.2"), Field(""));
  std::unique_ptr<WorldRules> rules = MakeScriptedShooters(*plant->model());
  ASSERT_NE(rules, nullptr);
  try {
    plant->AddRules(*rules);
    ADD_FAILURE() << "not refused";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string_view(error.what()).find("'flywheel_exit'"),
              std::string_view::npos)
        << error.what();
  }
}

// Five balls falling on contacts into the inlet of a welded robot whose
// flywheel is driven at 12 V: the plant after `ticks` Steps of two steps.
struct Scenario {
  std::unique_ptr<MujocoPlant> plant;
  std::unique_ptr<WorldRules> rules;
  ScriptedShooters* shooters{};
};
Scenario Barrage(const std::string& name) {
  std::string balls;
  for (int i = 0; i < 5; ++i) {
    balls += Ball("ball_" + std::to_string(i), {0.0, 0.0, 1.0 + 0.2 * i}, true);
  }
  Scenario scenario;
  scenario.plant = Make(name, Robot(false), Field(balls, true));
  scenario.shooters = &Attach(*scenario.plant, scenario.rules);
  const std::vector<PlantMotor> motors = {Motor("flywheel")};
  std::array<bool, 1> owned{};
  ConfigError error;
  if (!scenario.plant->Bind(motors, {}, owned, error)) {
    throw std::logic_error(FormatConfigError(error));
  }
  return scenario;
}
constexpr PlantDrive kSpinUp{.volts = 12.0, .back_emf = true};

// REQ-DEV-39: the same model and commands give the same launches: two runs
// with several launches end with the same positions and velocities, bit for
// bit.
TEST(ScriptedShooter, RunsAreBitIdentical) {
  Scenario a = Barrage("identical_a");
  Scenario b = Barrage("identical_b");
  for (int tick = 0; tick < 150; ++tick) {
    ASSERT_TRUE(a.plant->Step(2, std::span(&kSpinUp, 1))) << a.plant->failure();
    ASSERT_TRUE(b.plant->Step(2, std::span(&kSpinUp, 1))) << b.plant->failure();
  }
  ASSERT_GE(a.shooters->shots(), 3u);
  EXPECT_EQ(a.shooters->shots(), b.shooters->shots());
  const mjModel* m = a.plant->model();
  const std::size_t qpos = sizeof(mjtNum) * static_cast<std::size_t>(m->nq);
  const std::size_t qvel = sizeof(mjtNum) * static_cast<std::size_t>(m->nv);
  EXPECT_EQ(std::memcmp(a.plant->data()->qpos, b.plant->data()->qpos, qpos), 0);
  EXPECT_EQ(std::memcmp(a.plant->data()->qvel, b.plant->data()->qvel, qvel), 0);
}

// Step with the shooter attached runs inside the gateway's tick: it
// allocates nothing, launches included.
TEST(ScriptedShooter, StepAllocatesNothing) {
  if (const auto why = testing::AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  Scenario scenario = Barrage("allocation");
  ASSERT_TRUE(scenario.plant->Step(2, std::span(&kSpinUp, 1)));
  testing::AllocationProbe probe;
  probe.start();
  for (int tick = 0; tick < 150; ++tick) {
    scenario.plant->Step(2, std::span(&kSpinUp, 1));
  }
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_GE(scenario.shooters->shots(), 3u);
  EXPECT_TRUE(scenario.plant->failure().empty()) << scenario.plant->failure();
}

}  // namespace
}  // namespace talos::simulation::mujoco

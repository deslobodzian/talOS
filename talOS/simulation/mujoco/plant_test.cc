// MujocoPlant and MakePlant (REQ-DEV-37), on inline MJCF: binding by name,
// the torque law MuJoCo integrates per step, rotor frames, the chassis,
// world inputs, failure, and how MakePlant assembles robot and field.

#include "talOS/simulation/mujoco/plant.h"

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/hardware/sim_plant.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/simulation/mujoco/world_rules.h"

namespace talos::simulation::mujoco {
namespace {

using hardware::ConfigError;
using hardware::sim::PlantDrive;
using hardware::sim::PlantInput;
using hardware::sim::PlantMotor;

constexpr double kPi = std::numbers::pi;
constexpr double kH = 0.005;  // the option table's step
// A Kraken X60's winding (the design note's numbers) and a nonzero no-load
// drag, so the compiled damping G²·b_m is part of every closed form.
constexpr double kR = 0.0328;
constexpr double kKt = 0.01937;
constexpr double kKe = 0.01899;
constexpr double kBm = 2e-4;

PlantMotor Motor(std::string_view name, double gear = 1.0) {
  return PlantMotor{name, gear, kKt, kKe, kR, kBm};
}

// The `index`th group of `stride` entries of a MuJoCo array.
template <typename T>
const T* At(const T* array, int stride, int index) {
  return array + static_cast<std::ptrdiff_t>(stride) * index;
}

std::string Write(const std::string& name, const std::string& xml) {
  const std::string path = ::testing::TempDir() + "/" + name;
  std::ofstream(path) << xml;
  return path;
}

std::unique_ptr<MujocoPlant> Make(const std::string& name,
                                  const std::string& robot_xml,
                                  std::span<const PlantMotor> motors,
                                  PlantModel model = {},
                                  const std::string& field_xml = "") {
  InstallMujocoProcessHandlers();
  model.robot_xml = Write(name + "_robot.xml", robot_xml);
  if (!field_xml.empty()) {
    model.field_xml = Write(name + "_field.xml", field_xml);
  }
  return MakePlant(model, motors);
}

// Binds `motors` and `inputs`; returns which motors the plant owns.
std::vector<bool> Bind(MujocoPlant& plant, std::span<const PlantMotor> motors,
                       std::span<const PlantInput> inputs = {}) {
  std::unique_ptr<bool[]> owned(new bool[motors.size() + 1]());
  ConfigError error;
  EXPECT_TRUE(plant.Bind(motors, inputs,
                         std::span<bool>(owned.get(), motors.size()), error))
      << FormatConfigError(error);
  return std::vector<bool>(owned.get(), owned.get() + motors.size());
}

// A robot whose welded root holds one hinge `joint` about z, of inertia
// `inertia` about it, driven by <motor name="arm">.
std::string HingeRobot(double inertia, std::string_view joint_extra = "") {
  return R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="base" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <body name="arm">
      <joint name="arm_joint" type="hinge" axis="0 0 1" )" +
         std::string(joint_extra) + R"(/>
      <inertial pos="0 0 0" mass="1" diaginertia=")" +
         std::to_string(inertia) + " " + std::to_string(inertia) + " " +
         std::to_string(inertia) + R"("/>
    </body>
  </body>
</worldbody>
<actuator><motor name="arm" joint="arm_joint"/></actuator></mujoco>)";
}

// REQ-DEV-37: a motor is bound by the actuator of its name, whatever the
// order of either list; one no actuator names stays unbound.
TEST(MujocoPlant, BindsEachMotorToTheActuatorOfItsName) {
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="base">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <body name="link_b"><joint name="joint_b" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body>
    <body name="link_a"><joint name="joint_a" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body>
  </body>
</worldbody>
<actuator>
  <motor name="b" joint="joint_b"/>
  <motor name="a" joint="joint_a"/>
</actuator></mujoco>)";
  const std::vector<PlantMotor> motors = {Motor("a"), Motor("x"), Motor("b")};
  auto plant = Make("binds", robot, motors);
  EXPECT_EQ(Bind(*plant, motors), (std::vector<bool>{true, false, true}));

  // Drive only motor 2, "b": joint_b turns, joint_a does not.
  std::vector<PlantDrive> drives(3);
  drives[2].volts = 6.0;
  ASSERT_TRUE(plant->Step(4, drives));
  const mjModel* m = plant->model();
  const mjData* d = plant->data();
  const int joint_a = mj_name2id(m, mjOBJ_JOINT, "joint_a");
  const int joint_b = mj_name2id(m, mjOBJ_JOINT, "joint_b");
  EXPECT_GT(d->qvel[m->jnt_dofadr[joint_b]], 0.0);
  EXPECT_EQ(d->qvel[m->jnt_dofadr[joint_a]], 0.0);
  EXPECT_GT(plant->rotor(2).velocity_rps, 0.0);
  EXPECT_EQ(plant->rotor(0).velocity_rps, 0.0);
  EXPECT_EQ(plant->rotor(1).velocity_rps, 0.0);
}

// REQ-DEV-37: one mj_step is the implicit update of PlantDrive's torque law
// through the gear: rotor torque k_t/R·(V − k_e·ω_r) − b_m·ω_r at ω_r = G·v,
// so the mechanism sees G·k_t/R·V − G²·(k_t·k_e/R + b_m)·v, its velocity
// term integrated implicitly. G = 5 tells G from G².
TEST(MujocoPlant, OneStepIsTheImplicitTorqueLawThroughTheGear) {
  constexpr double kJ = 0.002;
  for (const double gear : {1.0, 5.0}) {
    SCOPED_TRACE("G = " + std::to_string(gear));
    const std::vector<PlantMotor> motors = {Motor("arm", gear)};
    auto plant = Make("implicit", HingeRobot(kJ), motors);
    Bind(*plant, motors);
    const PlantDrive drive{.volts = 12.0, .back_emf = true};
    const double gain = gear * kKt / kR;
    const double velocity_term = gear * gear * (kKt * kKe / kR + kBm);
    double v = 0.0;
    double q = 0.0;
    for (int step = 1; step <= 2; ++step) {
      ASSERT_TRUE(plant->Step(1, std::span(&drive, 1)));
      // qfrc = gain·V − G²·k_t·k_e/R·v − G²·b_m·v; qH = J + h·velocity_term.
      v += kH * (gain * drive.volts - velocity_term * v) /
           (kJ + kH * velocity_term);
      q += kH * v;
      const double expected_rps = gear * v / (2.0 * kPi);
      EXPECT_NEAR(plant->rotor(0).velocity_rps, expected_rps,
                  1e-12 * expected_rps)
          << "step " << step;
      EXPECT_NEAR(plant->rotor(0).position_rot, gear * q / (2.0 * kPi),
                  1e-12 * gear * q)
          << "step " << step;
    }
  }
}

// The torque limits clamp at every mj_step, at that step's speed, on both
// sides. Each row's Steps start clamped (constant torque, only the drag
// implicit) and, as back EMF moves, leave the clamp mid-Step for the implicit
// law; the last row is the 0 A torque-current drive {0 V, back EMF, [0, +∞]}
// on a rotor already turning forward, whose lower bound holds it at zero
// torque.
TEST(MujocoPlant, TorqueLimitsClampAtEveryStep) {
  constexpr double kJ = 0.004;
  constexpr double kGear = 3.0;
  constexpr double kInf = std::numeric_limits<double>::infinity();
  struct Phase {
    PlantDrive drive;
    int steps;
  };
  struct Case {
    const char* what;
    std::vector<Phase> phases;
  };
  // Stall torque at 12 V is 7.1 N·m at the rotor.
  const std::vector<Case> cases = {
      {"upper bound", {{{12.0, true, -kInf, 3.0}, 20}}},
      {"lower bound", {{{-12.0, true, -3.0, kInf}, 20}}},
      {"0 A on a forward rotor",
       {{{12.0, true, -kInf, kInf}, 10}, {{0.0, true, 0.0, kInf}, 10}}},
  };
  const double gain = kGear * kKt / kR;
  const double back_emf = kGear * kGear * kKt * kKe / kR;
  const double drag = kGear * kGear * kBm;
  for (const Case& row : cases) {
    SCOPED_TRACE(row.what);
    const std::vector<PlantMotor> motors = {Motor("arm", kGear)};
    auto plant = Make("clamp", HingeRobot(kJ), motors);
    Bind(*plant, motors);
    double v = 0.0;
    int clamped = 0;
    int free = 0;
    for (const Phase& phase : row.phases) {
      ASSERT_TRUE(plant->Step(phase.steps, std::span(&phase.drive, 1)));
      const double low = kGear * phase.drive.min_torque_nm;
      const double high = kGear * phase.drive.max_torque_nm;
      for (int step = 0; step < phase.steps; ++step) {
        const double force = gain * phase.drive.volts - back_emf * v;
        if (force <= low || force >= high) {
          ++clamped;
          const double held = force <= low ? low : high;
          v += kH * (held - drag * v) / (kJ + kH * drag);
        } else {
          ++free;
          v += kH * (force - drag * v) / (kJ + kH * (back_emf + drag));
        }
      }
    }
    ASSERT_GT(clamped, 0);
    ASSERT_GT(free, 0) << "the clamp never released; the test cannot tell a "
                          "per-Step clamp from a per-step one";
    const double expected_rps = kGear * v / (2.0 * kPi);
    EXPECT_NEAR(plant->rotor(0).velocity_rps, expected_rps,
                1e-12 * std::fabs(expected_rps))
        << clamped << " steps clamped, " << free << " free";
  }
}

// REQ-DEV-37: a rotor reads zero at the model's reference configuration and
// counts G·(length − length0)/2π: a hinge with a nonzero `ref`, a slide whose
// gear maps metres to drum radians, and a tendon with a rest length.
TEST(MujocoPlant, RotorCountsFromTheActuatorsReferenceLength) {
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="base" pos="0 0 1">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <site name="anchor" pos="0 0 0"/>
    <body name="arm"><joint name="hinge" axis="0 0 1" ref="0.3"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body>
    <body name="carriage"><joint name="slide" type="slide" axis="1 0 0" ref="0.1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body>
    <body name="sled" pos="0.62 0 0">
      <joint name="rope_joint" type="slide" axis="1 0 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="hook" pos="0 0 0"/>
    </body>
  </body>
</worldbody>
<tendon><spatial name="rope"><site site="anchor"/><site site="hook"/></spatial></tendon>
<actuator>
  <motor name="arm" joint="hinge"/>
  <motor name="lift" joint="slide" gear="20"/>
  <motor name="winch" tendon="rope"/>
</actuator></mujoco>)";
  const std::vector<PlantMotor> motors = {Motor("arm", 2.0), Motor("lift", 3.0),
                                          Motor("winch", 4.0)};
  auto plant = Make("frames", robot, motors);
  Bind(*plant, motors);
  for (std::size_t i = 0; i < motors.size(); ++i) {
    EXPECT_EQ(plant->rotor(i).position_rot, 0.0) << motors[i].name;
  }

  const std::vector<PlantDrive> drives(3, PlantDrive{.volts = 3.0});
  ASSERT_TRUE(plant->Step(10, drives));
  const mjModel* m = plant->model();
  const mjData* d = plant->data();
  const auto qpos = [&](const char* joint) {
    return d->qpos[m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, joint)]];
  };
  // Lengths in mechanism radians: the hinge's angle, the slide's metres times
  // its gear, the rope's metres.
  const double hinge = qpos("hinge") - 0.3;
  const double slide = 20.0 * (qpos("slide") - 0.1);
  const double rope = d->ten_length[mj_name2id(m, mjOBJ_TENDON, "rope")] - 0.62;
  ASSERT_GT(std::fabs(hinge), 1e-3);
  ASSERT_GT(std::fabs(slide), 1e-3);
  ASSERT_GT(std::fabs(rope), 1e-4);
  EXPECT_NEAR(plant->rotor(0).position_rot, 2.0 * hinge / (2.0 * kPi), 1e-12);
  EXPECT_NEAR(plant->rotor(1).position_rot, 3.0 * slide / (2.0 * kPi), 1e-12);
  EXPECT_NEAR(plant->rotor(2).position_rot, 4.0 * rope / (2.0 * kPi), 1e-12);
}

// A free root's chassis heading is continuous: a root spun past ±π by the
// reaction of a motor reads a yaw that keeps going, at the rate it reports.
TEST(MujocoPlant, ChassisYawIsContinuousAcrossPi) {
  const std::string field = R"(<mujoco><option gravity="0 0 0"/><worldbody>
  <geom name="carpet_robot" type="plane" size="5 5 0.1" contype="16" conaffinity="0"/>
</worldbody></mujoco>)";
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="chassis" pos="0 0 1"><freejoint name="root"/>
    <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
    <body name="flywheel"><joint name="spin_joint" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.02"/></body>
  </body>
</worldbody>
<actuator><motor name="spin" joint="spin_joint"/></actuator></mujoco>)";
  const std::vector<PlantMotor> motors = {Motor("spin")};
  auto plant = Make("yaw", robot, motors, {}, field);
  Bind(*plant, motors);
  // A constant 0.5 N·m on the flywheel turns the chassis the other way.
  const PlantDrive drive{
      .volts = 12.0, .min_torque_nm = 0.5, .max_torque_nm = 0.5};
  ASSERT_TRUE(plant->chassis().has_value());
  double yaw = plant->chassis().value_or(hardware::sim::PlantChassis{}).yaw_rad;
  for (int step = 1; step <= 160; ++step) {
    ASSERT_TRUE(plant->Step(1, std::span(&drive, 1)));
    ASSERT_TRUE(plant->chassis().has_value());
    const hardware::sim::PlantChassis chassis =
        plant->chassis().value_or(hardware::sim::PlantChassis{});
    EXPECT_NEAR(chassis.yaw_rad - yaw, kH * chassis.omega_radps, 1e-9)
        << "step " << step;
    yaw = chassis.yaw_rad;
  }
  EXPECT_LT(yaw, -3.0 * kPi);
  const mjtNum* q = At(plant->data()->xquat, 4,
                       mj_name2id(plant->model(), mjOBJ_BODY, "chassis"));
  const double raw = std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                                1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
  EXPECT_NEAR(std::remainder(yaw - raw, 2.0 * kPi), 0.0, 1e-9);
}

// A root without a free joint is welded at the start pose and reports no
// chassis (the backend keeps its analytic one).
TEST(MujocoPlant, WeldedRootHasNoChassis) {
  const std::vector<PlantMotor> motors = {Motor("arm")};
  PlantModel pose;
  pose.start_x_m = 2.0;
  pose.start_y_m = 1.0;
  auto plant = Make("welded", HingeRobot(0.01), motors, pose);
  Bind(*plant, motors);
  const std::vector<PlantDrive> drives(1);
  ASSERT_TRUE(plant->Step(40, drives));
  EXPECT_FALSE(plant->chassis().has_value());
  const mjtNum* base = At(plant->data()->xpos, 3,
                          mj_name2id(plant->model(), mjOBJ_BODY, "base"));
  EXPECT_NEAR(base[0], 2.0, 1e-12);
  EXPECT_NEAR(base[1], 1.0, 1e-12);
  EXPECT_NEAR(base[2], 0.5, 1e-12);
}

// An unstable step fails the plant for good, naming the warning and the
// joint, with the world left as it was (MuJoCo's autoreset is off) and
// nothing written to MUJOCO_LOG.TXT.
TEST(MujocoPlant, InstabilityFailsForGoodWithoutReset) {
  std::filesystem::remove("MUJOCO_LOG.TXT");
  const std::vector<PlantMotor> motors = {Motor("arm")};
  auto plant = Make("unstable", HingeRobot(1e-3), motors);
  Bind(*plant, motors);
  // 1e9 V is below MuJoCo's 1e10 bad-control bound; its acceleration is not.
  const PlantDrive drive{.volts = 1e9};
  EXPECT_FALSE(plant->Step(3, std::span(&drive, 1)));
  EXPECT_NE(plant->failure().find("BADQACC"), std::string_view::npos)
      << plant->failure();
  EXPECT_NE(plant->failure().find("arm_joint"), std::string_view::npos)
      << plant->failure();
  const mjData* d = plant->data();
  EXPECT_EQ(d->time, kH) << "stepped past the failure, or reset";
  EXPECT_NE(d->qvel[0], 0.0) << "the world was reset";
  EXPECT_FALSE(plant->Step(1, std::span(&drive, 1)));
  EXPECT_EQ(d->time, kH);
  EXPECT_FALSE(std::filesystem::exists("MUJOCO_LOG.TXT"));
}

// A rangefinder input is closed while it sees a surface nearer than its
// cutoff (−1, no surface, is open); a touch input while its zone has contact
// force.
TEST(MujocoPlant, WorldInputsFollowTheirSensors) {
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="base" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <body name="probe">
      <joint name="drop" type="slide" axis="0 0 1" range="-0.46 0"/>
      <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>
      <geom name="probe_ball" type="sphere" size="0.06" contype="4" conaffinity="21"/>
      <site name="down" zaxis="0 0 -1"/>
      <site name="up" zaxis="0 0 1"/>
      <site name="zone" type="sphere" size="0.08"/>
    </body>
  </body>
</worldbody>
<sensor>
  <rangefinder name="beam" site="down" cutoff="0.1"/>
  <rangefinder name="sky" site="up" cutoff="0.1"/>
  <touch name="bumper" site="zone"/>
</sensor></mujoco>)";
  auto plant = Make("inputs", robot, {});
  const std::vector<PlantInput> inputs = {
      {"intake_beam", "beam"}, {"bumper", "bumper"}, {"sky", "sky"}};
  Bind(*plant, {}, inputs);
  // 0.5 m above the carpet, in the air.
  EXPECT_FALSE(plant->input_closed(0));
  EXPECT_FALSE(plant->input_closed(1));
  EXPECT_FALSE(plant->input_closed(2));
  // Fallen onto the carpet: 0.06 m to it, resting on it.
  ASSERT_TRUE(plant->Step(200, {}));
  EXPECT_TRUE(plant->input_closed(0));
  EXPECT_TRUE(plant->input_closed(1));
  EXPECT_FALSE(plant->input_closed(2));
}

// A rotor above Gateway::kMaxRotorRps counts one excursion, however long it
// stays there, and does not fail the plant (Q9).
TEST(MujocoPlant, OverspeedCountsExcursionsAndIsNotFatal) {
  const std::vector<PlantMotor> motors = {Motor("arm")};
  auto plant = Make("overspeed", HingeRobot(1e-3), motors);
  Bind(*plant, motors);
  const PlantDrive drive{.volts = 12.0};  // no back EMF: it keeps speeding up
  for (int step = 0; step < 80; ++step) {
    ASSERT_TRUE(plant->Step(1, std::span(&drive, 1)));
  }
  ASSERT_GT(plant->rotor(0).velocity_rps, 300.0);
  EXPECT_EQ(plant->overspeed_excursions(), 1u);
}

// Step runs inside the gateway's tick: it allocates nothing.
TEST(MujocoPlant, StepDoesNotAllocate) {
  if (const auto why = testing::AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="chassis" pos="0 0 0.2"><freejoint/>
    <geom type="box" size="0.3 0.3 0.05" contype="4" conaffinity="21" mass="20"/>
    <body name="arm"><joint name="arm_joint" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0.1 0.3 0 0.1" size="0.02" contype="0" conaffinity="0" mass="1"/></body>
  </body>
</worldbody>
<actuator><motor name="arm" joint="arm_joint"/></actuator></mujoco>)";
  const std::vector<PlantMotor> motors = {Motor("arm", 5.0)};
  auto plant = Make("allocation", robot, motors);
  Bind(*plant, motors);
  const PlantDrive drive{.volts = 4.0, .back_emf = true};
  ASSERT_TRUE(plant->Step(2, std::span(&drive, 1)));
  testing::AllocationProbe probe;
  probe.start();
  for (int tick = 0; tick < 20; ++tick) {
    plant->Step(2, std::span(&drive, 1));
  }
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_TRUE(plant->failure().empty()) << plant->failure();
}

// A world rule that logs each AfterStep and, at step `fail_at` (0: never),
// returns false or, with `throws`, throws.
struct RuleCall {
  int rule{};
  uint64_t step{};
  MatchInput::Mode mode{};
  uint64_t schedule_us{};
  bool operator==(const RuleCall&) const = default;
};

class LoggingRule final : public WorldRules {
 public:
  LoggingRule(int id, std::vector<RuleCall>& log, uint64_t fail_at = 0,
              bool throws = false)
      : id_(id), log_(log), fail_at_(fail_at), throws_(throws) {}
  void Attach(const mjModel& /*model*/, mjData& /*data*/) override {
    ++attached;
  }
  bool AfterStep(mjModel& /*model*/, mjData& /*data*/, uint64_t step,
                 const MatchInput& input) override {
    log_.push_back({id_, step, input.mode, input.schedule_us});
    if (step == fail_at_ && throws_) {
      throw std::runtime_error("the hub ran dry");
    }
    return step != fail_at_;
  }
  std::string_view failure() const override { return "the hub ran dry"; }
  int attached{0};

 private:
  int id_;
  std::vector<RuleCall>& log_;
  uint64_t fail_at_;
  bool throws_;
};

// The rules seam: rules attach once when added (a rule added twice is
// refused) and run after every MuJoCo step, in the order added, numbered
// from 1, with the match input last set.
TEST(MujocoPlant, RulesRunInOrderAfterEveryStepWithTheMatchInput) {
  const std::vector<PlantMotor> motors = {Motor("arm")};
  auto plant = Make("rules", HingeRobot(0.01), motors);
  std::vector<RuleCall> log;
  LoggingRule first(0, log);
  LoggingRule second(1, log);
  plant->AddRules(first);
  plant->AddRules(second);
  EXPECT_THROW(plant->AddRules(first), std::invalid_argument);
  EXPECT_EQ(first.attached, 1);
  EXPECT_EQ(second.attached, 1);

  using Mode = MatchInput::Mode;
  plant->SetMatchInput({.mode = Mode::kTeleop, .schedule_us = 7000});
  ASSERT_TRUE(plant->Step(2, {}));
  plant->SetMatchInput({.mode = Mode::kAutonomous, .schedule_us = 12000});
  ASSERT_TRUE(plant->Step(1, {}));
  EXPECT_EQ(log, (std::vector<RuleCall>{{0, 1, Mode::kTeleop, 7000},
                                        {1, 1, Mode::kTeleop, 7000},
                                        {0, 2, Mode::kTeleop, 7000},
                                        {1, 2, Mode::kTeleop, 7000},
                                        {0, 3, Mode::kAutonomous, 12000},
                                        {1, 3, Mode::kAutonomous, 12000}}));
  LoggingRule late(2, log);
  EXPECT_THROW(plant->AddRules(late), std::logic_error);
}

// The rules seam fails closed: a rule that returns false, or throws, fails
// the plant for good with its text; no rule runs after it, in that step or
// in a later Step.
TEST(MujocoPlant, AFailingRuleFailsThePlantForGood) {
  for (const bool throws : {false, true}) {
    SCOPED_TRACE(throws ? "throws" : "returns false");
    const std::vector<PlantMotor> motors = {Motor("arm")};
    auto plant = Make("failing_rule", HingeRobot(0.01), motors);
    std::vector<RuleCall> log;
    LoggingRule failing(0, log, 2, throws);
    LoggingRule after(1, log);
    plant->AddRules(failing);
    plant->AddRules(after);
    EXPECT_FALSE(plant->Step(3, {}));
    EXPECT_NE(plant->failure().find("the hub ran dry"), std::string_view::npos)
        << plant->failure();
    const std::vector<RuleCall> expected = {{0, 1}, {1, 1}, {0, 2}};
    EXPECT_EQ(log, expected);
    EXPECT_FALSE(plant->Step(1, {}));
    EXPECT_EQ(log, expected);
    EXPECT_EQ(plant->data()->time, 2 * kH);
  }
}

// A rule whose Attach leaves the world unstable is refused by AddRules,
// naming the warning and the joint, not blamed on the first Step; the plant
// is failed and takes no further rule.
TEST(MujocoPlant, AnUnstableAttachIsRefusedWhenAdded) {
  class BadPlacement final : public WorldRules {
   public:
    void Attach(const mjModel& /*model*/, mjData& data) override {
      data.qpos[0] = std::numeric_limits<double>::quiet_NaN();
    }
    bool AfterStep(mjModel& /*model*/, mjData& /*data*/, uint64_t /*step*/,
                   const MatchInput& /*input*/) override {
      return true;
    }
    std::string_view failure() const override { return {}; }
  };
  const std::vector<PlantMotor> motors = {Motor("arm")};
  auto plant = Make("bad_attach", HingeRobot(0.01), motors);
  BadPlacement bad;
  try {
    plant->AddRules(bad);
    ADD_FAILURE() << "not refused";
  } catch (const std::invalid_argument& error) {
    const std::string_view message = error.what();
    EXPECT_NE(message.find("Attach"), std::string_view::npos) << message;
    EXPECT_NE(message.find("BADQPOS"), std::string_view::npos) << message;
    EXPECT_NE(message.find("arm_joint"), std::string_view::npos) << message;
  }
  EXPECT_FALSE(plant->failure().empty());
  EXPECT_FALSE(plant->Step(1, {}));
  std::vector<RuleCall> log;
  LoggingRule later(0, log);
  EXPECT_THROW(plant->AddRules(later), std::logic_error);
}

// A model MakePlant cannot bind or place is refused before it runs, naming
// the element and the file.
TEST(MakePlant, RefusesAModelItCannotBindOrPlace) {
  const std::string field = R"(<mujoco><worldbody>
  <geom name="carpet_robot" type="plane" size="5 5 0.1" contype="16" conaffinity="0"/>
  <site name="blue_hub_score" pos="1 1 1"/>
</worldbody></mujoco>)";
  const auto robot = [](std::string_view body, std::string_view actuator,
                        std::string_view world = "",
                        std::string_view joint = "",
                        std::string_view sections = "") {
    return std::string(R"(<mujoco><compiler angle="radian"/><worldbody>)") +
           std::string(world) + R"(
  <body name="base" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <site name="anchor"/>
    <body name="arm"><joint name="arm_joint" axis="0 0 1" )" +
           std::string(joint) + R"(/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="hook" pos="0.1 0 0"/>)" +
           std::string(body) + R"(</body>
  </body>
</worldbody>)" +
           std::string(sections) + "<actuator>" + std::string(actuator) +
           "</actuator></mujoco>";
  };
  const std::string motor = R"(<motor name="arm" joint="arm_joint"/>)";
  struct Case {
    const char* what;
    std::string xml;
    const char* names;
  };
  const std::vector<Case> cases = {
      {"position actuator",
       robot("", R"(<position name="arm" joint="arm_joint" kp="10"/>)"), "arm"},
      {"ctrlrange",
       robot("", R"(<motor name="arm" joint="arm_joint" ctrlrange="-1 1"/>)"),
       "arm"},
      {"forcerange",
       robot("", R"(<motor name="arm" joint="arm_joint" forcerange="-1 1"/>)"),
       "arm"},
      {"delay",
       robot(
           "",
           R"(<motor name="arm" joint="arm_joint" delay="0.01" nsample="4"/>)"),
       "arm"},
      {"actuator naming no motor",
       robot("", R"(<motor name="ghost" joint="arm_joint"/>)" + motor),
       "ghost"},
      {"default collision bits",
       robot(R"(<geom name="plate" type="box" size="0.1 0.1 0.01"/>)", motor),
       "plate"},
      {"geom in the robot's worldbody",
       robot(
           "", motor,
           R"(<geom name="own_floor" type="plane" size="1 1 0.1" contype="4" conaffinity="21"/>)"),
       "own_floor"},
      {"two root bodies",
       robot(
           "", motor,
           R"(<body name="second_root"><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>)"),
       "second_root"},
      {"name of a field element of another type",
       robot(
           R"(<geom name="blue_hub_score" type="sphere" size="0.05" contype="4" conaffinity="21"/>)",
           motor),
       "blue_hub_score"},
      // mjs_attach would drop each of these silently.
      {"actuator on a missing joint",
       robot("", R"(<motor name="arm" joint="arm_jiont"/>)"), "arm_jiont"},
      {"actuator on a missing tendon",
       robot("", R"(<motor name="arm" tendon="no_rope"/>)"), "no_rope"},
      {"sensor on a missing site",
       robot("", motor, "", "",
             R"(<sensor><touch name="bump" site="no_site"/></sensor>)"),
       "no_site"},
      // What would change the force behind the motor's law.
      {"joint actuatorfrcrange",
       robot("", motor, "", R"(actuatorfrcrange="-1 1")"), "arm_joint"},
      {"joint actuatorgravcomp",
       robot("", motor, "", R"(actuatorgravcomp="true")"), "arm_joint"},
      {"tendon actuatorfrcrange",
       robot("", R"(<motor name="arm" tendon="rope"/>)", "", "",
             R"(<tendon><spatial name="rope" actuatorfrcrange="-1 1">)"
             R"(<site site="anchor"/><site site="hook"/></spatial></tendon>)"),
       "rope"},
      // A robot model is self-contained: it names no field element.
      {"robot pair naming a field geom",
       robot(
           R"(<geom name="wheel" type="sphere" size="0.05" contype="4" conaffinity="21"/>)",
           motor, "", "",
           R"(<contact><pair geom1="wheel" geom2="carpet_robot"/></contact>)"),
       "carpet_robot"},
      {"free root MuJoCo would align",
       std::string(R"(<mujoco><worldbody><body name="lone" pos="0 0 1">
         <freejoint name="lone_root" align="true"/>
         <geom type="box" pos="0.1 0 0" size="0.1 0.1 0.1" contype="4" conaffinity="21"/>
       </body></worldbody></mujoco>)"),
       "lone_root"},
  };
  const std::vector<PlantMotor> motors = {Motor("arm")};
  for (const Case& row : cases) {
    SCOPED_TRACE(row.what);
    try {
      Make("refused", row.xml, motors, {}, field);
      ADD_FAILURE() << "not refused";
    } catch (const std::invalid_argument& error) {
      const std::string message = error.what();
      EXPECT_NE(message.find(row.names), std::string::npos) << message;
      EXPECT_NE(message.find("refused_robot.xml"), std::string::npos)
          << message;
    }
  }
}

// Bind refuses, with the device and the reason in its ConfigError, a pushed
// config that changes a bound motor's gear or drops its motor, and a world
// input whose sensor is missing or cannot be read as a switch.
TEST(MujocoPlant, BindRefusesWhatTheCompiledModelCannotServe) {
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="base" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <body name="arm"><joint name="arm_joint" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="eye"/></body>
  </body>
</worldbody>
<actuator><motor name="arm" joint="arm_joint"/></actuator>
<sensor>
  <rangefinder name="no_cutoff" site="eye"/>
  <jointpos name="angle" joint="arm_joint"/>
</sensor></mujoco>)";
  const std::vector<PlantMotor> compiled = {Motor("arm", 5.0)};
  auto plant = Make("bind_refused", robot, compiled);
  struct Case {
    const char* what;
    std::vector<PlantMotor> motors;
    std::vector<PlantInput> inputs;
    const char* device;
  };
  const std::vector<Case> cases = {
      {"changed gear", {Motor("arm", 4.0)}, {}, "arm"},
      {"motor dropped", {Motor("other", 5.0)}, {}, "arm"},
      {"missing sensor", compiled, {{"intake_beam", "nowhere"}}, "intake_beam"},
      {"rangefinder without cutoff", compiled, {{"beam", "no_cutoff"}}, "beam"},
      {"unsupported sensor type", compiled, {{"limit", "angle"}}, "limit"},
  };
  for (const Case& row : cases) {
    SCOPED_TRACE(row.what);
    std::unique_ptr<bool[]> flags(new bool[row.motors.size() + 1]());
    ConfigError error;
    EXPECT_FALSE(plant->Bind(row.motors, row.inputs,
                             std::span<bool>(flags.get(), row.motors.size()),
                             error));
    EXPECT_EQ(error.device_name, row.device);
    EXPECT_FALSE(error.operation.empty());
    EXPECT_FALSE(error.message.empty());
  }
}

// The robot lands at the start pose with its authored pose composed after it
// and its authored height kept. Its free joint asks for alignment, which
// MuJoCo does not apply to a root with a child body, so it is accepted and
// the chassis is the root's own frame, not its centre of mass.
TEST(MakePlant, StartPoseComposesWithTheAuthoredPose) {
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="chassis" pos="0.5 0 0.2" euler="0 0 1.5707963267948966">
    <freejoint align="true"/>
    <geom type="box" pos="0.1 0.05 0" size="0.3 0.3 0.05" contype="4" conaffinity="21" mass="20"/>
    <body name="intake" pos="0.3 0 0">
      <geom type="box" size="0.05 0.2 0.05" contype="4" conaffinity="21" mass="2"/>
    </body>
  </body>
</worldbody></mujoco>)";
  PlantModel pose;
  pose.start_x_m = 1.0;
  pose.start_y_m = 2.0;
  pose.start_yaw_rad = kPi / 2.0;
  auto plant = Make("pose", robot, {}, pose);
  ASSERT_TRUE(plant->chassis().has_value());
  const hardware::sim::PlantChassis chassis =
      plant->chassis().value_or(hardware::sim::PlantChassis{});
  EXPECT_NEAR(chassis.x_m, 1.0, 1e-12);
  EXPECT_NEAR(chassis.y_m, 2.5, 1e-12);
  EXPECT_NEAR(chassis.yaw_rad, kPi, 1e-12);
  const int body = mj_name2id(plant->model(), mjOBJ_BODY, "chassis");
  EXPECT_NEAR(At(plant->data()->xpos, 3, body)[2], 0.2, 1e-12);
}

// Attached with no prefix: the robot's actuators and sensors keep the motor
// and sim_sensor names, and the field's elements keep theirs.
TEST(MakePlant, AttachKeepsEveryName) {
  const std::string field = R"(<mujoco><worldbody>
  <geom name="carpet_robot" type="plane" size="5 5 0.1" contype="16" conaffinity="0"/>
  <site name="blue_hub_score" pos="1 1 1"/>
  <body name="fuel_000" pos="2 2 0.1"><freejoint name="fuel_000_joint"/>
    <geom name="fuel_000_geom" type="sphere" size="0.075" contype="2" conaffinity="15"/></body>
</worldbody></mujoco>)";
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="chassis" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <body name="arm"><joint name="arm_joint" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="eye"/></body>
  </body>
</worldbody>
<actuator><motor name="arm" joint="arm_joint"/></actuator>
<sensor><touch name="intake_beam" site="eye"/></sensor></mujoco>)";
  const std::vector<PlantMotor> motors = {Motor("arm")};
  auto plant = Make("names", robot, motors, {}, field);
  const mjModel* m = plant->model();
  EXPECT_GE(mj_name2id(m, mjOBJ_ACTUATOR, "arm"), 0);
  EXPECT_GE(mj_name2id(m, mjOBJ_SENSOR, "intake_beam"), 0);
  EXPECT_GE(mj_name2id(m, mjOBJ_BODY, "chassis"), 0);
  EXPECT_GE(mj_name2id(m, mjOBJ_SITE, "blue_hub_score"), 0);
  EXPECT_GE(mj_name2id(m, mjOBJ_BODY, "fuel_000"), 0);
  EXPECT_GE(mj_name2id(m, mjOBJ_JOINT, "fuel_000_joint"), 0);
  EXPECT_GE(mj_name2id(m, mjOBJ_GEOM, "fuel_000_geom"), 0);
}

// A robot whose angles are radians, attached to a field in MJCF's default
// degrees, keeps its own unit, and the field keeps its.
TEST(MakePlant, RadianRobotInADegreeFieldKeepsItsAngles) {
  const std::string field = R"(<mujoco><worldbody>
  <geom name="carpet_robot" type="plane" size="5 5 0.1" contype="16" conaffinity="0"/>
  <geom name="ramp" type="box" size="1 1 0.1" euler="0 0 90" pos="3 3 0" contype="1" conaffinity="0"/>
</worldbody></mujoco>)";
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="chassis" pos="0 0 0.5" euler="0 0 0.5">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <body name="arm"><joint name="arm_joint" axis="0 0 1" range="0 1.5" limited="true"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body>
  </body>
</worldbody></mujoco>)";
  auto plant = Make("units", robot, {}, {}, field);
  const mjModel* m = plant->model();
  const int joint = mj_name2id(m, mjOBJ_JOINT, "arm_joint");
  EXPECT_NEAR(At(m->jnt_range, 2, joint)[1], 1.5, 1e-12);
  const mjtNum* body =
      At(m->body_quat, 4, mj_name2id(m, mjOBJ_BODY, "chassis"));
  EXPECT_NEAR(2.0 * std::atan2(body[3], body[0]), 0.5, 1e-12);
  const mjtNum* ramp = At(m->geom_quat, 4, mj_name2id(m, mjOBJ_GEOM, "ramp"));
  EXPECT_NEAR(2.0 * std::atan2(ramp[3], ramp[0]), kPi / 2.0, 1e-12);
}

// A robot default class named like a field class (and both models' main
// class) applies to the robot's geoms only, and the field's to its own.
TEST(MakePlant, SameNamedDefaultClassesStayApart) {
  const std::string field = R"(<mujoco>
<default>
  <geom friction="0.6"/>
  <default class="structure"><geom contype="1" conaffinity="0" friction="0.4"/></default>
</default>
<worldbody>
  <geom name="carpet_robot" type="plane" size="5 5 0.1" contype="16" conaffinity="0"/>
  <geom name="wall" class="structure" type="box" size="1 0.1 0.5" pos="3 0 0.5"/>
  <geom name="plain_field" type="box" size="0.1 0.1 0.1" pos="4 0 0.1" contype="1" conaffinity="0"/>
</worldbody></mujoco>)";
  const std::string robot = R"(<mujoco><compiler angle="radian"/>
<default>
  <geom contype="4" conaffinity="21" friction="0.9"/>
  <default class="structure"><geom contype="0" conaffinity="0" friction="0.3"/></default>
</default>
<worldbody>
  <body name="chassis" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <geom name="bumper" class="structure" type="box" size="0.3 0.05 0.05"/>
    <geom name="plain_robot" type="box" size="0.3 0.3 0.05"/>
  </body>
</worldbody></mujoco>)";
  auto plant = Make("classes", robot, {}, {}, field);
  const mjModel* m = plant->model();
  struct Expected {
    const char* geom;
    int contype;
    int conaffinity;
    double friction;
  };
  for (const Expected& row :
       {Expected{"wall", 1, 0, 0.4}, Expected{"plain_field", 1, 0, 0.6},
        Expected{"bumper", 0, 0, 0.3}, Expected{"plain_robot", 4, 21, 0.9}}) {
    SCOPED_TRACE(row.geom);
    const int geom = mj_name2id(m, mjOBJ_GEOM, row.geom);
    ASSERT_GE(geom, 0);
    EXPECT_EQ(m->geom_contype[geom], row.contype);
    EXPECT_EQ(m->geom_conaffinity[geom], row.conaffinity);
    EXPECT_EQ(At(m->geom_friction, 3, geom)[0], row.friction);
  }
}

// The option table wins over the field's and the robot's <option> and
// <size>, and step_us comes from it.
TEST(MakePlant, OptionTableWinsOverBothModels) {
  const std::string field = R"(<mujoco>
<option timestep="0.002" cone="pyramidal" iterations="5"/>
<worldbody>
  <geom name="carpet_robot" type="plane" size="5 5 0.1" contype="16" conaffinity="0"/>
</worldbody></mujoco>)";
  const std::string robot = R"(<mujoco><compiler angle="radian"/>
<option timestep="0.001" iterations="50" integrator="RK4" solver="Newton" tolerance="1e-3">
  <flag sleep="enable" autoreset="enable"/>
</option>
<size memory="1M"/>
<worldbody>
  <body name="chassis" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
  </body>
</worldbody></mujoco>)";
  auto plant = Make("options", robot, {}, {}, field);
  const mjOption& option = plant->model()->opt;
  EXPECT_EQ(plant->step_us(), 5000u);
  EXPECT_EQ(option.timestep, 0.005);
  EXPECT_EQ(option.solver, mjSOL_CG);
  EXPECT_EQ(option.iterations, 20);
  EXPECT_EQ(option.tolerance, 1e-8);
  EXPECT_EQ(option.cone, mjCONE_ELLIPTIC);
  EXPECT_EQ(option.integrator, mjINT_IMPLICITFAST);
  EXPECT_EQ(option.enableflags & mjENBL_SLEEP, 0);
  EXPECT_NE(option.disableflags & mjDSBL_AUTORESET, 0);
  EXPECT_GE(plant->model()->narena, mjtSize{64} << 20);
}

// With no field the world is the field's two carpets and follows its
// collision contract: a robot geom (4, 21) rests on carpet_robot (no rolling
// resistance), a fuel geom (2, 15) on carpet_fuel (rolling resistance slows
// it), and a visual geom (0, 0) falls through.
TEST(MakePlant, FlatCarpetFollowsTheCollisionContract) {
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="ball" pos="0 0 0.3"><freejoint/>
    <geom name="ball_geom" type="sphere" size="0.05" contype="4" conaffinity="21" mass="1"/>
  </body>
</worldbody></mujoco>)";
  auto plant = Make("carpet", robot, {});
  struct Case {
    const char* what;
    int contype;
    int conaffinity;
    const char* carpet;  // the only geom it touches; null: falls through
  };
  for (const Case& row :
       {Case{"robot", 4, 21, "carpet_robot"},
        Case{"fuel", 2, 15, "carpet_fuel"}, Case{"visual", 0, 0, nullptr}}) {
    SCOPED_TRACE(row.what);
    const std::unique_ptr<mjModel, MujocoPlant::ModelDeleter> m(
        mj_copyModel(nullptr, plant->model()));
    const int ball = mj_name2id(m.get(), mjOBJ_GEOM, "ball_geom");
    m->geom_contype[ball] = row.contype;
    m->geom_conaffinity[ball] = row.conaffinity;
    const std::unique_ptr<mjData, MujocoPlant::DataDeleter> d(
        mj_makeData(m.get()));
    d->qvel[0] = 1.0;  // rolling along x
    for (int step = 0; step < 200; ++step) mj_step(m.get(), d.get());
    const mjtNum* position =
        At(d->xpos, 3, mj_name2id(m.get(), mjOBJ_BODY, "ball"));
    if (row.carpet == nullptr) {
      EXPECT_LT(position[2], -0.1);
      continue;
    }
    EXPECT_NEAR(position[2], 0.05, 2e-3);
    const int carpet = mj_name2id(m.get(), mjOBJ_GEOM, row.carpet);
    ASSERT_GT(d->ncon, 0);
    for (int c = 0; c < d->ncon; ++c) {
      const int g0 = d->contact[c].geom[0];
      const int g1 = d->contact[c].geom[1];
      EXPECT_TRUE((g0 == ball && g1 == carpet) || (g0 == carpet && g1 == ball))
          << "contact " << c << ": geoms " << g0 << ", " << g1;
    }
    const double speed = d->qvel[0];
    if (row.contype == 2) {
      EXPECT_LT(speed, 0.4) << "rolling resistance did not slow it";
    } else {
      EXPECT_GT(speed, 0.6);
    }
  }
}

// REQ-DEV-37: the binding reports each bound actuator's inertia in its own
// coordinate, 1/(moment·M⁻¹·moment) with armature, and J_rotor = J_act/G²,
// light below h·b_em/2: a hinge of 0.002 kg·m² plus 0.001 armature at G 5,
// a 0.5 kg slide geared 20 to drum radians at G 10, which is light, and a
// tendon across its sled's travel, with no moment at the reference pose,
// whose inertia is unknown (not zero, and not light).
TEST(MujocoPlant, BindingReportsReflectedInertia) {
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="base" pos="0 0 1">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <site name="anchor" pos="0 0.5 0"/>
    <body name="sled"><joint name="sled_slide" type="slide" axis="1 0 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.001 0.001 0.001"/>
      <site name="hook"/></body>
    <body name="arm"><joint name="hinge" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.002"/></body>
    <body name="carriage"><joint name="slide" type="slide" axis="1 0 0"/>
      <inertial pos="0 0 0" mass="0.5" diaginertia="0.001 0.001 0.001"/></body>
  </body>
</worldbody>
<actuator>
  <motor name="arm" joint="hinge" armature="0.001"/>
  <motor name="lift" joint="slide" gear="20"/>
  <motor name="winch" tendon="rope"/>
</actuator>
<tendon><spatial name="rope"><site site="anchor"/><site site="hook"/></spatial></tendon>
</mujoco>)";
  const std::vector<PlantMotor> motors = {
      Motor("arm", 5.0), Motor("lift", 10.0), Motor("winch", 2.0)};
  auto plant = Make("inertia", robot, motors);
  Bind(*plant, motors);
  const std::span<const MujocoPlant::Binding> bindings = plant->bindings();
  ASSERT_EQ(bindings.size(), 3u);
  struct Expected {
    double inertia;
    double rotor;
    bool light;
  };
  // h·b_em/2 = 0.005·k_t·k_e/R/2 = 2.8e-5 kg·m².
  constexpr double kUnknown = std::numeric_limits<double>::quiet_NaN();
  const Expected expected[] = {{0.003, 0.003 / 25.0, false},
                               {0.5 / 400.0, 0.5 / 400.0 / 100.0, true},
                               {kUnknown, kUnknown, false}};
  for (std::size_t i = 0; i < bindings.size(); ++i) {
    SCOPED_TRACE(motors[bindings[i].motor].name);
    ASSERT_EQ(bindings[i].motor, i);
    EXPECT_EQ(bindings[i].light, expected[i].light);
    if (std::isnan(expected[i].inertia)) {
      EXPECT_TRUE(std::isnan(bindings[i].inertia_kg_m2))
          << bindings[i].inertia_kg_m2;
      EXPECT_TRUE(std::isnan(bindings[i].rotor_inertia_kg_m2))
          << bindings[i].rotor_inertia_kg_m2;
      continue;
    }
    EXPECT_NEAR(bindings[i].inertia_kg_m2, expected[i].inertia,
                1e-12 * expected[i].inertia);
    EXPECT_NEAR(bindings[i].rotor_inertia_kg_m2, expected[i].rotor,
                1e-12 * expected[i].rotor);
  }
}

// A bad position names the joint that holds it: MuJoCo reports a qpos
// address, which a free joint ahead of it (7 positions, 6 dofs) sets apart
// from a dof index.
TEST(MujocoPlant, BadPositionNamesItsJoint) {
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="chassis" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <body name="arm"><joint name="first_hinge" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body>
    <body name="carriage"><joint name="bad_slide" type="slide" axis="1 0 0" ref="2e10"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body>
  </body>
</worldbody></mujoco>)";
  auto plant = Make("bad_position", robot, {});
  Bind(*plant, {});
  EXPECT_FALSE(plant->Step(1, {}));
  EXPECT_NE(plant->failure().find("BADQPOS"), std::string_view::npos)
      << plant->failure();
  EXPECT_NE(plant->failure().find("bad_slide"), std::string_view::npos)
      << plant->failure();
}

// The process handlers drop the warning every MakePlant raises by design and
// still print MuJoCo's other warnings (here, the robot's ignored <option>).
TEST(MakePlant, ProcessHandlersDropOnlyTheEmptyPrefixWarning) {
  const std::string robot = R"(<mujoco><compiler angle="radian"/>
<option timestep="0.001"/>
<worldbody><body name="chassis" pos="0 0 0.5">
  <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
</body></worldbody></mujoco>)";
  ::testing::internal::CaptureStderr();
  Make("handlers", robot, {});
  const std::string printed = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(printed.find("empty prefix"), std::string::npos) << printed;
  EXPECT_NE(printed.find("timestep"), std::string::npos) << printed;
}

// An engine error (MuJoCo's handler must not return) exits 1 with a line
// naming the MuJoCo plant and MuJoCo's message.
TEST(MujocoProcessHandlersDeathTest, EngineErrorExitsOneNamingThePlant) {
  EXPECT_EXIT(
      {
        InstallMujocoProcessHandlers();
        mju_error("probe arena overflow");
      },
      ::testing::ExitedWithCode(1), "mujoco plant.*probe arena overflow");
}

// REQ-DEV-37: MakePlant refuses non-finite start poses and invalid or
// duplicate motor parameters before compiling.
TEST(MakePlant, RefusesNonFiniteStartPoseAndInvalidMotorParameters) {
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const std::vector<PlantMotor> valid_motors = {Motor("arm")};
  for (const PlantModel& pose :
       {PlantModel{.start_x_m = kNaN}, PlantModel{.start_y_m = kInf},
        PlantModel{.start_yaw_rad = -kInf}}) {
    EXPECT_THROW(Make("bad_pose", HingeRobot(0.01), valid_motors, pose),
                 std::invalid_argument);
  }
  const std::vector<std::vector<PlantMotor>> bad_motors = {
      {Motor("arm", 0.0)},
      {Motor("arm", -2.0)},
      {Motor("arm", kNaN)},
      {Motor("arm", kInf)},
      {PlantMotor{"arm", 1.0, kKt, kKe, kR, -1e-4}},
      {PlantMotor{"arm", 1.0, kKt, kKe, kR, kNaN}},
      {PlantMotor{"arm", 1.0, kKt, kKe, kR, kInf}},
      {Motor("arm", 1.0), Motor("arm", 1.0)},
  };
  for (const auto& motors : bad_motors) {
    EXPECT_THROW(Make("bad_motor", HingeRobot(0.01), motors),
                 std::invalid_argument);
  }
}

// REQ-DEV-37: Bind refuses non-finite/non-positive resistance, negative or
// non-finite k_t/k_e, duplicate motor names, and a failed plant.
TEST(MujocoPlant, BindRefusesInvalidWindingConstantsDuplicatesAndFailedPlant) {
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const std::vector<PlantMotor> compiled = {Motor("arm", 5.0)};
  auto plant = Make("bind_winding", HingeRobot(0.01), compiled);
  const std::vector<std::vector<PlantMotor>> cases = {
      {PlantMotor{"arm", 5.0, kKt, kKe, 0.0, kBm}},
      {PlantMotor{"arm", 5.0, kKt, kKe, -0.1, kBm}},
      {PlantMotor{"arm", 5.0, kKt, kKe, kNaN, kBm}},
      {PlantMotor{"arm", 5.0, kKt, kKe, kInf, kBm}},
      {PlantMotor{"arm", 5.0, -0.01, kKe, kR, kBm}},
      {PlantMotor{"arm", 5.0, kNaN, kKe, kR, kBm}},
      {PlantMotor{"arm", 5.0, kInf, kKe, kR, kBm}},
      {PlantMotor{"arm", 5.0, kKt, -0.01, kR, kBm}},
      {PlantMotor{"arm", 5.0, kKt, kNaN, kR, kBm}},
      {PlantMotor{"arm", 5.0, kKt, kInf, kR, kBm}},
      {Motor("arm", 5.0), Motor("arm", 5.0)},
  };
  for (const auto& motors : cases) {
    std::unique_ptr<bool[]> flags(new bool[motors.size() + 1]());
    ConfigError error;
    EXPECT_FALSE(plant->Bind(
        motors, {}, std::span<bool>(flags.get(), motors.size()), error));
    EXPECT_EQ(error.device_name, "arm");
    EXPECT_FALSE(error.message.empty());
  }
  // Fail the plant, then verify Bind refuses it.
  EXPECT_FALSE(plant->Step(-1, {}));
  bool owned = false;
  ConfigError error;
  EXPECT_FALSE(plant->Bind(compiled, {}, std::span(&owned, 1), error));
  EXPECT_FALSE(error.message.empty());
}

// REQ-DEV-37: Step refuses non-finite voltages, NaN torque limits, inverted
// torque bounds, and degenerate infinite bounds without mutating ctrl, even
// when steps == 0.
TEST(MujocoPlant, StepRefusesNonFiniteAndInvertedDrivesWithoutMutatingState) {
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  struct Case {
    const char* what;
    int steps;
    PlantDrive drive;
  };
  const std::vector<Case> cases = {
      {"NaN volts (0 steps)", 0, {.volts = kNaN}},
      {"NaN volts (1 step)", 1, {.volts = kNaN}},
      {"+Inf volts", 1, {.volts = kInf}},
      {"-Inf volts", 1, {.volts = -kInf}},
      {"NaN min torque", 1, {.volts = 1.0, .min_torque_nm = kNaN}},
      {"NaN max torque", 1, {.volts = 1.0, .max_torque_nm = kNaN}},
      {"inverted bounds",
       1,
       {.volts = 1.0, .min_torque_nm = 2.0, .max_torque_nm = -2.0}},
      {"min +Inf", 1, {.volts = 1.0, .min_torque_nm = kInf}},
      {"max -Inf", 1, {.volts = 1.0, .max_torque_nm = -kInf}},
  };
  const std::vector<PlantMotor> motors = {Motor("arm")};
  const PlantDrive valid{.volts = 3.0};
  for (const Case& row : cases) {
    SCOPED_TRACE(row.what);
    auto plant = Make("bad_drive", HingeRobot(0.01), motors);
    Bind(*plant, motors);
    ASSERT_TRUE(plant->Step(1, std::span(&valid, 1)));
    ASSERT_EQ(plant->data()->ctrl[0], 3.0);
    EXPECT_FALSE(plant->Step(row.steps, std::span(&row.drive, 1)));
    EXPECT_FALSE(plant->failure().empty());
    EXPECT_EQ(plant->data()->ctrl[0], 3.0);
    EXPECT_FALSE(plant->Step(1, std::span(&valid, 1)));
  }
}

// REQ-DEV-37: a WorldRules::AfterStep that writes NaN or Inf into qpos or qvel
// fails that Step immediately rather than leaking non-finite rotor state.
TEST(MujocoPlant, RuleWritingNonFiniteStateFailsThatStep) {
  class CorruptingRule final : public WorldRules {
   public:
    explicit CorruptingRule(bool corrupt_vel) : corrupt_vel_(corrupt_vel) {}
    void Attach(const mjModel& /*model*/, mjData& /*data*/) override {}
    bool AfterStep(mjModel& /*model*/, mjData& data, uint64_t /*step*/,
                   const MatchInput& /*input*/) override {
      if (corrupt_vel_) {
        data.qvel[0] = std::numeric_limits<double>::infinity();
      } else {
        data.qpos[0] = std::numeric_limits<double>::quiet_NaN();
      }
      return true;
    }
    std::string_view failure() const override { return {}; }

   private:
    bool corrupt_vel_;
  };
  const std::vector<PlantMotor> motors = {Motor("arm")};
  const PlantDrive drive{.volts = 0.0};
  for (const bool corrupt_vel : {false, true}) {
    SCOPED_TRACE(corrupt_vel ? "qvel" : "qpos");
    auto plant = Make("corrupt_rule", HingeRobot(0.01), motors);
    Bind(*plant, motors);
    CorruptingRule rule(corrupt_vel);
    plant->AddRules(rule);
    EXPECT_FALSE(plant->Step(1, std::span(&drive, 1)));
    EXPECT_NE(plant->failure().find(corrupt_vel ? "BADQVEL" : "BADQPOS"),
              std::string_view::npos)
        << plant->failure();
    EXPECT_NE(plant->failure().find("arm_joint"), std::string_view::npos)
        << plant->failure();
  }
}

// REQ-DEV-37: reentrant calls into MujocoPlant from WorldRules hooks fail
// closed without corrupting rule iteration or state.
TEST(MujocoPlant, ReentrantRuleHookFailsClosed) {
  class DummyRule final : public WorldRules {
   public:
    void Attach(const mjModel& /*model*/, mjData& /*data*/) override {}
    bool AfterStep(mjModel& /*model*/, mjData& /*data*/, uint64_t /*step*/,
                   const MatchInput& /*input*/) override {
      return true;
    }
    std::string_view failure() const override { return {}; }
  };
  class ReentrantRule final : public WorldRules {
   public:
    enum class Kind { kStep, kMatchInput, kAddInObserve };
    ReentrantRule(MujocoPlant& plant, Kind kind, WorldRules& extra)
        : plant_(plant), kind_(kind), extra_(extra) {}
    void Attach(const mjModel& /*model*/, mjData& /*data*/) override {}
    bool ObserveMatch(const MatchInput& /*input*/) override {
      if (kind_ == Kind::kAddInObserve) plant_.AddRules(extra_);
      return true;
    }
    bool AfterStep(mjModel& /*model*/, mjData& /*data*/, uint64_t /*step*/,
                   const MatchInput& input) override {
      if (kind_ == Kind::kStep) {
        const PlantDrive drive{};
        EXPECT_FALSE(plant_.Step(1, std::span(&drive, 1)));
      } else if (kind_ == Kind::kMatchInput) {
        plant_.SetMatchInput(input);
      }
      return true;
    }
    std::string_view failure() const override { return {}; }

   private:
    MujocoPlant& plant_;
    Kind kind_;
    WorldRules& extra_;
  };
  const std::vector<PlantMotor> motors = {Motor("arm")};
  const PlantDrive drive{};
  for (const auto kind :
       {ReentrantRule::Kind::kStep, ReentrantRule::Kind::kMatchInput,
        ReentrantRule::Kind::kAddInObserve}) {
    auto plant = Make("reentrant", HingeRobot(0.01), motors);
    Bind(*plant, motors);
    DummyRule extra;
    ReentrantRule rule(*plant, kind, extra);
    plant->AddRules(rule);
    if (kind == ReentrantRule::Kind::kAddInObserve) {
      plant->SetMatchInput({});
      EXPECT_FALSE(plant->Step(1, std::span(&drive, 1)));
    } else {
      EXPECT_FALSE(plant->Step(1, std::span(&drive, 1)));
    }
    EXPECT_NE(plant->failure().find("reentrant"), std::string_view::npos)
        << plant->failure();
  }
}

// REQ-DEV-37: non-finite sensor data (NaN or +Inf) never falsely closes a
// touch input.
TEST(MujocoPlant, TouchInputIgnoresNonFiniteSensorData) {
  class InjectSensor final : public WorldRules {
   public:
    explicit InjectSensor(double value) : value_(value) {}
    void Attach(const mjModel& /*model*/, mjData& /*data*/) override {}
    bool AfterStep(mjModel& /*model*/, mjData& data, uint64_t /*step*/,
                   const MatchInput& /*input*/) override {
      data.sensordata[0] = value_;
      return true;
    }
    std::string_view failure() const override { return {}; }

   private:
    double value_;
  };
  const std::string robot = R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="base" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <site name="zone" type="sphere" size="0.08"/>
  </body>
</worldbody>
<sensor><touch name="bumper" site="zone"/></sensor></mujoco>)";
  for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
    auto plant = Make("touch_nonfinite", robot, {});
    const std::vector<PlantInput> inputs = {{"bumper", "bumper"}};
    Bind(*plant, {}, inputs);
    InjectSensor rule(bad);
    plant->AddRules(rule);
    ASSERT_TRUE(plant->Step(1, {}));
    EXPECT_FALSE(plant->input_closed(0));
  }
}

}  // namespace
}  // namespace talos::simulation::mujoco

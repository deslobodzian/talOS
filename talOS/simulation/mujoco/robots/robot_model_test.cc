// The generated example models (REQ-DEV-41) against the plant's rules and
// the conventions README.md states: each assembles on the flat carpet and the
// 2026 field, binds every motor its parameter file names and carries a
// scripted-shooter contract MakeScriptedShooters accepts; each exit and each
// preload site clears the robot's geoms; each beam break closes on a piece in
// its path; at rest on the carpet it stays still, warns nothing, reads its
// beam breaks open, and touches the carpet with its wheels alone; and the
// hopper holds its preloads behind a stopped feeder and feeds every one of
// them to the inlet through a turning one.

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <numbers>
#include <ostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/hardware/config_error.h"
#include "talOS/hardware/sim_plant.h"
#include "talOS/simulation/mujoco/plant.h"
#include "talOS/simulation/mujoco/scripted_shooter.h"
#include "talOS/simulation/mujoco/world_rules.h"

namespace talos::simulation::mujoco {
namespace {

using hardware::ConfigError;
using hardware::sim::PlantDrive;
using hardware::sim::PlantInput;
using hardware::sim::PlantMotor;

// A Kraken X60's winding, as plant_test.cc gives it.
constexpr double kR = 0.0328;
constexpr double kKt = 0.01937;
constexpr double kKe = 0.01899;
constexpr double kBm = 2e-4;
constexpr double kPieceRadiusM = 0.07506;  // the parameter files' game piece
constexpr double kExitCheckM = 0.3;
constexpr double kStepS = 0.005;

// A [[beam_breaks]] entry of a parameter file.
struct Beam {
  std::string_view name;
  std::array<double, 3> position_m;
  std::array<double, 3> direction;  // unit
  double cutoff_m;
};

struct Robot {
  const char* name;
  const char* xml_variable;
  // The motors and beam breaks of its parameter file, as a config names them.
  std::vector<std::pair<std::string_view, double>> motors;  // name, gear
  std::vector<Beam> beams;
  bool free_root;
  std::size_t preloads;
  std::size_t feeder;    // index in motors of the hopper's gate
  std::size_t flywheel;  // and of the flywheel
};

// gtest prints a parameter by this, not as its bytes.
void PrintTo(const Robot& robot, std::ostream* os) { *os << robot.name; }

const std::vector<Robot>& Robots() {
  static const auto* const robots = new std::vector<Robot>{
      {"swerve_robot",
       "SWERVE_ROBOT_XML",
       {{"front_left_drive", 6.75},
        {"front_left_steer", 12.8},
        {"front_right_drive", 6.75},
        {"front_right_steer", 12.8},
        {"back_left_drive", 6.75},
        {"back_left_steer", 12.8},
        {"back_right_drive", 6.75},
        {"back_right_steer", 12.8},
        {"roller", 1.0},
        {"feeder", 1.0},
        {"flywheel", 1.0}},
       {{"intake_beam", {0.41925, -0.28, 0.29556}, {0, 1, 0}, 0.54},
        {"beam_break", {-0.21776, -0.27, 0.15}, {0, 1, 0}, 0.54}},
       true,
       8,
       9,
       10},
      {"shooter_rig",
       "SHOOTER_RIG_XML",
       {{"roller", 1.0}, {"flywheel", 1.0}},
       {{"intake_beam", {-0.06, -0.27, 0.195}, {0, 1, 0}, 0.54},
        {"beam_break", {-0.21776, -0.27, 0.18}, {0, 1, 0}, 0.54}},
       false,
       8,
       0,
       1},
  };
  return *robots;
}

std::string Runfile(const char* variable) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* path = std::getenv(variable);
  if (!runfiles || path == nullptr) {
    throw std::runtime_error("runfiles: " + error + ", " + variable + " unset");
  }
  return runfiles->Rlocation(path);
}

std::vector<PlantMotor> Motors(const Robot& robot) {
  std::vector<PlantMotor> motors;
  motors.reserve(robot.motors.size());
  for (const auto& [name, gear] : robot.motors) {
    motors.push_back(PlantMotor{name, gear, kKt, kKe, kR, kBm});
  }
  return motors;
}

std::vector<PlantInput> Beams(const Robot& robot) {
  std::vector<PlantInput> inputs;
  inputs.reserve(robot.beams.size());
  for (const Beam& beam : robot.beams) {
    inputs.push_back(PlantInput{beam.name, beam.name});
  }
  return inputs;
}

std::unique_ptr<MujocoPlant> Make(const Robot& robot,
                                  const std::string& field_xml,
                                  std::span<const PlantMotor> motors,
                                  double start_x_m = 0.0,
                                  double start_y_m = 0.0) {
  InstallMujocoProcessHandlers();
  PlantModel model;
  model.field_xml = field_xml;
  model.robot_xml = Runfile(robot.xml_variable);
  model.start_x_m = start_x_m;
  model.start_y_m = start_y_m;
  return MakePlant(model, motors);
}

// A world of the field's two carpets and `bodies`, written for MakePlant.
std::string TestField(const std::string& name, const std::string& bodies) {
  const std::string path = ::testing::TempDir() + "/" + name + "_field.xml";
  std::ofstream(path) << R"(<mujoco><worldbody>
  <geom name="carpet_robot" type="plane" size="20 20 0.1" contype="16"
        conaffinity="0" condim="3" friction="0.8 0.005 0.0001"/>
  <geom name="carpet_fuel" type="plane" size="20 20 0.1" contype="8"
        conaffinity="0" condim="6" friction="0.8 0.001 0.01"/>
)" << bodies << "</worldbody></mujoco>";
  return path;
}

std::string Xyz(const mjtNum* v) {
  return std::to_string(v[0]) + " " + std::to_string(v[1]) + " " +
         std::to_string(v[2]);
}

using ModelPtr = std::unique_ptr<mjModel, void (*)(mjModel*)>;
using DataPtr = std::unique_ptr<mjData, void (*)(mjData*)>;

// The robot model alone at its authored pose, which is where the plant puts
// it with the start pose at the origin.
struct Alone {
  ModelPtr model{nullptr, mj_deleteModel};
  DataPtr data{nullptr, mj_deleteData};
};
Alone CompileAlone(const Robot& robot) {
  std::array<char, 1024> error{};
  Alone alone;
  alone.model.reset(mj_loadXML(Runfile(robot.xml_variable).c_str(), nullptr,
                               error.data(), static_cast<int>(error.size())));
  if (!alone.model) throw std::runtime_error(error.data());
  alone.data.reset(mj_makeData(alone.model.get()));
  mj_kinematics(alone.model.get(), alone.data.get());
  return alone;
}

// Binds the robot's motors and beams; every motor must be the model's.
void BindAll(MujocoPlant& plant, const Robot& robot,
             std::span<const PlantMotor> motors) {
  const std::vector<PlantInput> inputs = Beams(robot);
  std::unique_ptr<bool[]> owned(new bool[motors.size()]());
  ConfigError error;
  ASSERT_TRUE(plant.Bind(motors, inputs,
                         std::span<bool>(owned.get(), motors.size()), error))
      << FormatConfigError(error);
  for (std::size_t i = 0; i < motors.size(); ++i) {
    EXPECT_TRUE(owned[i]) << motors[i].name << " has no actuator";
  }
}

// The `index`th group of `stride` entries of a MuJoCo array.
template <typename T>
T* At(T* array, int stride, int index) {
  return array + static_cast<std::ptrdiff_t>(stride) * index;
}

bool IsRobotGeom(const mjModel* m, int geom, int root) {
  return m->body_rootid[m->geom_bodyid[geom]] == root;
}

class RobotModelTest : public ::testing::TestWithParam<Robot> {};

// REQ-DEV-41: one model assembles on the flat carpet and on the 2026 field
// (self-contained, the collision contract, names clear of the field's), and
// every motor its parameter file names binds an actuator whose rotor inertia
// is the model's armature, not the light free-flight value.
TEST_P(RobotModelTest, AssemblesOnEveryWorldAndBindsEveryMotor) {
  const Robot& robot = GetParam();
  const std::vector<PlantMotor> motors = Motors(robot);
  for (const std::string& field : {std::string(), Runfile("FIELD_XML")}) {
    SCOPED_TRACE(field.empty() ? "flat carpet" : field);
    std::unique_ptr<WorldRules> shooters;  // outlives the plant it is in
    // On the field: in the blue half, clear of every element.
    std::unique_ptr<MujocoPlant> plant =
        Make(robot, field, motors, field.empty() ? 0.0 : 2.5,
             field.empty() ? 0.0 : 2.0);
    BindAll(*plant, robot, motors);
    // The contract as the scripted shooter reads it (REQ-DEV-39): it throws
    // naming a missing or malformed element, and Attach refuses an exit
    // inside its inlet.
    shooters = MakeScriptedShooters(*plant->model());
    ASSERT_NE(shooters, nullptr);
    plant->AddRules(*shooters);
    ASSERT_EQ(plant->bindings().size(), motors.size());
    for (const MujocoPlant::Binding& binding : plant->bindings()) {
      EXPECT_TRUE(std::isfinite(binding.rotor_inertia_kg_m2))
          << motors[binding.motor].name;
      EXPECT_FALSE(binding.light) << motors[binding.motor].name;
    }
    EXPECT_EQ(plant->chassis().has_value(), robot.free_root);
  }
}

// REQ-DEV-41: the scripted-shooter trio is whole, and a piece launched from
// the exit clears every robot geom by its radius for 0.3 m along the launch
// axis; each preload site holds a piece clear of every robot geom. Distances
// are MuJoCo's, from a probe sphere of the piece's radius added to the robot
// alone.
TEST_P(RobotModelTest, ExitsAndPreloadsClearTheRobot) {
  const Robot& robot = GetParam();
  std::array<char, 1024> error{};
  std::unique_ptr<mjSpec, void (*)(mjSpec*)> spec(
      mj_parseXML(Runfile(robot.xml_variable).c_str(), nullptr, error.data(),
                  static_cast<int>(error.size())),
      mj_deleteSpec);
  ASSERT_NE(spec, nullptr) << error.data();
  mjsBody* probe_body = mjs_addBody(mjs_findBody(spec.get(), "world"), nullptr);
  probe_body->mocap = 1;
  mjsGeom* probe = mjs_addGeom(probe_body, nullptr);
  probe->type = mjGEOM_SPHERE;
  probe->size[0] = kPieceRadiusM;
  probe->contype = 0;
  probe->conaffinity = 0;
  mjs_setName(probe->element, "probe");
  std::unique_ptr<mjModel, void (*)(mjModel*)> model(
      mj_compile(spec.get(), nullptr), mj_deleteModel);
  ASSERT_NE(model, nullptr) << mjs_getError(spec.get());
  const mjModel* m = model.get();
  std::unique_ptr<mjData, void (*)(mjData*)> data(mj_makeData(m),
                                                  mj_deleteData);
  mjData* d = data.get();
  mj_kinematics(m, d);
  const int probe_id = mj_name2id(m, mjOBJ_GEOM, "probe");
  const int root = m->body_rootid[mj_name2id(m, mjOBJ_BODY, "chassis")];

  // The nearest robot collision geom to a piece centred at `point`.
  auto clearance = [&](const mjtNum* point, std::string& nearest) {
    mju_copy3(d->mocap_pos, point);
    mj_kinematics(m, d);
    mjtNum least = 1.0;
    for (int g = 0; g < m->ngeom; ++g) {
      if (!IsRobotGeom(m, g, root) || m->geom_contype[g] == 0) continue;
      const mjtNum gap = mj_geomDistance(m, d, probe_id, g, 1.0, nullptr);
      if (gap < least) {
        least = gap;
        nearest = mj_id2name(m, mjOBJ_GEOM, g);
      }
    }
    return least;
  };

  int shooters = 0;
  for (int n = 0; n < m->nnumeric; ++n) {
    const std::string numeric = mj_id2name(m, mjOBJ_NUMERIC, n);
    constexpr std::string_view kSuffix = "_shooter";
    if (!numeric.ends_with(kSuffix)) continue;
    const std::string motor =
        numeric.substr(0, numeric.size() - kSuffix.size());
    SCOPED_TRACE(motor);
    ++shooters;
    EXPECT_GE(mj_name2id(m, mjOBJ_ACTUATOR, motor.c_str()), 0);
    EXPECT_EQ(m->numeric_size[n], 2);
    const int inlet = mj_name2id(m, mjOBJ_SITE, (motor + "_inlet").c_str());
    ASSERT_GE(inlet, 0);
    EXPECT_EQ(m->site_type[inlet], mjGEOM_BOX);
    const int exit = mj_name2id(m, mjOBJ_SITE, (motor + "_exit").c_str());
    ASSERT_GE(exit, 0);
    mj_kinematics(m, d);
    std::array<mjtNum, 3> origin{};
    mju_copy3(origin.data(), At(d->site_xpos, 3, exit));
    const mjtNum* frame = At(d->site_xmat, 9, exit);
    const std::array<mjtNum, 3> axis = {frame[0], frame[3], frame[6]};
    for (int step = 0; step <= 60; ++step) {
      const mjtNum s = kExitCheckM * step / 60;
      std::array<mjtNum, 3> point{};
      for (int k = 0; k < 3; ++k) point[k] = origin[k] + s * axis[k];
      std::string nearest;
      EXPECT_GE(clearance(point.data(), nearest), -1e-9)
          << nearest << " is within a piece's radius " << s
          << " m along the launch axis";
    }
  }
  EXPECT_EQ(shooters, 1);

  std::size_t preloads = 0;
  for (int site = 0; site < m->nsite; ++site) {
    const std::string name = mj_id2name(m, mjOBJ_SITE, site);
    if (!name.starts_with("preload_")) continue;
    ++preloads;
    std::string nearest;
    EXPECT_GE(clearance(At(d->site_xpos, 3, site), nearest), -1e-9)
        << name << " overlaps " << nearest;
  }
  EXPECT_EQ(preloads, robot.preloads);
}

// REQ-DEV-41: each beam break looks where its parameter file points it: a
// small sphere at the middle of the beam the file gives (position, direction
// and cutoff, not the model's site) closes the plant's digital input of its
// name. RestsStillOnTheCarpetOnItsWheels holds it open without one.
TEST_P(RobotModelTest, BeamBreaksCloseOnAPieceInTheirPath) {
  const Robot& robot = GetParam();
  const std::vector<PlantMotor> motors = Motors(robot);
  // The plant puts the root's origin at the start pose; a free root's own
  // authored height lifts it, and its beams, 2 mm.
  const double lift_m = robot.free_root ? 0.002 : 0.0;
  for (std::size_t i = 0; i < robot.beams.size(); ++i) {
    const Beam& beam = robot.beams[i];
    SCOPED_TRACE(std::string(beam.name));
    std::array<mjtNum, 3> middle{};
    for (int k = 0; k < 3; ++k) {
      middle[k] = beam.position_m[k] + beam.cutoff_m / 2 * beam.direction[k];
    }
    middle[2] += lift_m;
    std::unique_ptr<MujocoPlant> plant = Make(
        robot,
        TestField(std::string(robot.name) + "_" + std::string(beam.name),
                  "<body name=\"beam_probe\" pos=\"" + Xyz(middle.data()) +
                      "\"><geom type=\"sphere\" size=\"0.02\" contype=\"0\" "
                      "conaffinity=\"0\"/></body>"),
        motors);
    BindAll(*plant, robot, motors);
    const std::vector<PlantDrive> drives(motors.size());
    ASSERT_TRUE(plant->Step(1, drives)) << plant->failure();
    EXPECT_TRUE(plant->input_closed(i));
  }
}

// The FUEL pieces of a test field, one at each of the robot's preload sites.
std::string PreloadPieces(const Robot& robot) {
  const Alone alone = CompileAlone(robot);
  std::string pieces;
  for (std::size_t k = 0; k < robot.preloads; ++k) {
    const int site = mj_name2id(alone.model.get(), mjOBJ_SITE,
                                ("preload_" + std::to_string(k)).c_str());
    if (site < 0) throw std::runtime_error("no preload_" + std::to_string(k));
    std::array<mjtNum, 3> centre{};
    mju_copy3(centre.data(), At(alone.data->site_xpos, 3, site));
    pieces += "<body name=\"piece_" + std::to_string(k) + "\" pos=\"" +
              Xyz(centre.data()) +
              "\"><freejoint/><geom type=\"sphere\" size=\"0.07506\" "
              "mass=\"0.215\" contype=\"2\" conaffinity=\"15\" "
              "friction=\"0.7 0.005 0.0001\" solref=\"0.02 1\" "
              "solimp=\"0.9 0.95 0.001\"/></body>\n";
  }
  return pieces;
}

// REQ-DEV-41, REQ-DEV-39: from preloads to launches on a generated model,
// with the real scripted shooter (MakeScriptedShooters, AddRules). A piece
// of the field's FUEL at every preload site; the flywheel at kFlywheelVolts
// from the start (about 34 rps); 2 s to settle and kHoldS more with the
// feeder's motor neutral, in which nothing launches; then the feeder at each
// of kFeedVolts, in which all eight launch within kFeedS. Each launched piece
// leaves along the exit's x axis and touches no robot geom before it first
// meets the carpet. Measured: the last launch comes 0.77 to 0.95 s after
// the feeder starts, so kFeedS leaves a margin of about 3 times.
TEST_P(RobotModelTest, HopperHoldsThenFeedsEveryPreloadToALaunch) {
  constexpr std::array<double, 3> kFeedVolts = {2.5, 4.0, 6.0};
  constexpr double kFlywheelVolts = 4.3;
  constexpr double kHoldS = 5.0;
  constexpr double kFeedS = 3.0;
  constexpr double kFlightS = 1.0;  // how long a launched piece is watched
  const Robot& robot = GetParam();
  const std::vector<PlantMotor> motors = Motors(robot);
  const std::string field =
      TestField(std::string(robot.name) + "_feed", PreloadPieces(robot));
  const double stator_nm = 80.0 * kKt;  // the config's stator limit, 80 A
  for (const double volts : kFeedVolts) {
    SCOPED_TRACE(std::to_string(volts) + " V");
    std::unique_ptr<WorldRules> rules;  // outlives the plant it is in
    std::unique_ptr<MujocoPlant> plant = Make(robot, field, motors);
    BindAll(*plant, robot, motors);
    const mjModel* m = plant->model();
    const mjData* d = plant->data();
    rules = MakeScriptedShooters(*m);
    ASSERT_NE(rules, nullptr);
    plant->AddRules(*rules);
    const auto& shooters = static_cast<const ScriptedShooters&>(*rules);
    const int exit = mj_name2id(m, mjOBJ_SITE, "flywheel_exit");
    const int root = m->body_rootid[mj_name2id(m, mjOBJ_BODY, "chassis")];
    std::vector<int> bodies;
    for (std::size_t k = 0; k < robot.preloads; ++k) {
      bodies.push_back(
          mj_name2id(m, mjOBJ_BODY, ("piece_" + std::to_string(k)).c_str()));
    }

    std::vector<PlantDrive> drives(motors.size());
    drives[robot.flywheel] =
        PlantDrive{kFlywheelVolts, true, -stator_nm, stator_nm};
    ASSERT_TRUE(plant->Step(static_cast<int>((2.0 + kHoldS) / kStepS), drives))
        << plant->failure();
    EXPECT_EQ(shooters.shots(), 0U) << "launched with the feeder stopped";

    drives[robot.feeder] = PlantDrive{volts, true, -stator_nm, stator_nm};
    std::vector<double> launched_s(bodies.size(), -1.0);
    std::vector<bool> flying(bodies.size(), false);
    double last_s = 0.0;
    const int steps = static_cast<int>((kFeedS + kFlightS) / kStepS);
    for (int step = 1; step <= steps; ++step) {
      const uint64_t before = shooters.shots();
      ASSERT_TRUE(plant->Step(1, drives)) << plant->failure();
      const double t = step * kStepS;
      // Contacts come from inside the step, before the rule moved a piece
      // to the exit: a piece launched in this step is watched from the next.
      for (int c = 0; c < d->ncon; ++c) {
        for (std::size_t k = 0; k < bodies.size(); ++k) {
          if (!flying[k]) continue;
          const int g1 = d->contact[c].geom[0];
          const int g2 = d->contact[c].geom[1];
          const int own = m->geom_bodyid[g1] == bodies[k]   ? g1
                          : m->geom_bodyid[g2] == bodies[k] ? g2
                                                            : -1;
          if (own < 0) continue;
          const int other = own == g1 ? g2 : g1;
          if (IsRobotGeom(m, other, root)) {
            ADD_FAILURE() << "launched piece " << k << " touches "
                          << mj_id2name(m, mjOBJ_GEOM, other) << " at " << t
                          << " s";
            flying[k] = false;
          } else if (m->geom_bodyid[other] == 0) {
            flying[k] = false;  // on the carpet
          }
        }
      }
      if (shooters.shots() > before) {
        // The launched piece sits at the exit, moving along its x axis.
        for (std::size_t k = 0; k < bodies.size(); ++k) {
          const mjtNum* centre = At(d->xpos, 3, bodies[k]);
          if (launched_s[k] >= 0.0 ||
              mju_dist3(centre, At(d->site_xpos, 3, exit)) > 1e-6) {
            continue;
          }
          launched_s[k] = t;
          last_s = t;
          flying[k] = true;
          const mjtNum* frame = At(d->site_xmat, 9, exit);
          const std::array<mjtNum, 3> axis = {frame[0], frame[3], frame[6]};
          const mjtNum* v =
              At(d->qvel, 1, m->jnt_dofadr[m->body_jntadr[bodies[k]]]);
          EXPECT_GT(mju_dot3(v, axis.data()) / mju_norm3(v), 0.999)
              << "piece " << k << " leaves off the exit's axis";
        }
      }
      for (std::size_t k = 0; k < bodies.size(); ++k) {
        if (flying[k] && t - launched_s[k] > kFlightS) flying[k] = false;
      }
    }
    std::size_t launched = 0;
    for (const double s : launched_s) launched += s >= 0.0 ? 1 : 0;
    EXPECT_EQ(shooters.shots(), robot.preloads);
    EXPECT_EQ(launched, robot.preloads);
    EXPECT_LE(last_s, kFeedS);
    std::cout << robot.name << " at " << volts << " V: last launch at "
              << last_s << " s of " << kFeedS << " s, flywheel "
              << d->qvel[m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT,
                                                  "flywheel_joint")]] /
                     (2 * std::numbers::pi)
              << " rps\n";
  }
}

// The pieces of `robot` (named piece_N) outside the swerve example's hopper:
// between its floor's front edge and the feeder, between its walls, below
// their top, in the chassis frame.
std::string OutsideTheHopper(const mjModel* m, const mjData* d,
                             const Robot& robot) {
  const int chassis = mj_name2id(m, mjOBJ_BODY, "chassis");
  std::string outside;
  for (std::size_t k = 0; k < robot.preloads; ++k) {
    const int body =
        mj_name2id(m, mjOBJ_BODY, ("piece_" + std::to_string(k)).c_str());
    std::array<mjtNum, 3> offset{};
    mju_sub3(offset.data(), At(d->xpos, 3, body), At(d->xpos, 3, chassis));
    std::array<mjtNum, 3> local{};
    mju_mulMatTVec3(local.data(), At(d->xmat, 9, chassis), offset.data());
    if (!(local[0] > -0.12 && local[0] < 0.33 && std::abs(local[1]) < 0.28 &&
          local[2] > 0.0 && local[2] < 0.52)) {
      outside += " piece " + std::to_string(k) + " at " + Xyz(local.data());
    }
  }
  return outside;
}

// REQ-DEV-41: the swerve example keeps its preloads through a stop at up to
// 8 m/s^2. Each drive motor's torque is limited to a quarter of the robot's
// weight times kDecelerationMPerS2 at the wheel (rotor torque = that / G),
// so its deceleration is bounded by that limit, not by traction: driven
// forward at 12 V to kSpeedMPerS, then reverse at 12 V until it stops, then
// 1 s on the brake, all eight pieces are still in the hopper. Measured:
// mean deceleration 6.9 m/s^2; over 25 ms it peaks at 9.8 m/s^2 as the
// torque reverses.
TEST(RobotModelSwerveTest,
     StopWithDriveTorqueCappedFor8MPerS2KeepsThePreloads) {
  constexpr double kSpeedMPerS = 4.0;
  constexpr double kDecelerationMPerS2 = 8.0;
  constexpr double kRobotMassKg = 60.0 + 8 * 0.215;
  constexpr double kWheelRadiusM = 0.0508;
  constexpr double kDriveGear = 6.75;
  const Robot& robot = Robots()[0];
  ASSERT_STREQ(robot.name, "swerve_robot");
  const std::vector<PlantMotor> motors = Motors(robot);
  std::unique_ptr<MujocoPlant> plant =
      Make(robot, TestField("brake", PreloadPieces(robot)), motors);
  BindAll(*plant, robot, motors);
  const mjModel* m = plant->model();
  const mjData* d = plant->data();
  const int chassis = mj_name2id(m, mjOBJ_BODY, "chassis");
  std::vector<PlantDrive> drives(motors.size(), PlantDrive{0.0, true});
  ASSERT_TRUE(plant->Step(400, drives)) << plant->failure();
  const double rotor_nm =
      kRobotMassKg * kDecelerationMPerS2 / 4 * kWheelRadiusM / kDriveGear;
  auto drive = [&](double volts) {
    for (std::size_t i = 0; i < motors.size(); ++i) {
      if (std::string_view(motors[i].name).ends_with("_drive")) {
        drives[i] = PlantDrive{volts, true, -rotor_nm, rotor_nm};
      }
    }
  };
  auto speed = [&] {
    std::array<mjtNum, 6> velocity{};
    mj_objectVelocity(m, d, mjOBJ_BODY, chassis, velocity.data(), 0);
    return velocity[3];
  };
  drive(12.0);
  for (int step = 0; step < 1000 && speed() < kSpeedMPerS; ++step) {
    ASSERT_TRUE(plant->Step(1, drives)) << plant->failure();
  }
  ASSERT_GE(speed(), kSpeedMPerS);
  const double from = speed();
  drive(-12.0);
  std::vector<double> speeds = {from};
  while (speeds.size() < 1000 && speed() > 0.0) {
    ASSERT_TRUE(plant->Step(1, drives)) << plant->failure();
    speeds.push_back(speed());
  }
  double peak = 0.0;
  for (std::size_t i = 5; i < speeds.size(); ++i) {
    peak = std::max(peak, (speeds[i - 5] - speeds[i]) / (5 * kStepS));
  }
  drive(0.0);
  ASSERT_TRUE(plant->Step(200, drives)) << plant->failure();
  std::cout << "stopped from " << from << " m/s: mean "
            << from / (static_cast<double>(speeds.size() - 1) * kStepS)
            << " m/s^2, peak over "
            << "25 ms " << peak << " m/s^2\n";
  EXPECT_EQ(OutsideTheHopper(m, d, robot), "");
}

// REQ-DEV-41: tilted 15 degrees nose down (gravity turned, the chassis held
// still), the swerve example keeps its preloads for 3 s.
TEST(RobotModelSwerveTest, NoseDownTiltKeepsThePreloads) {
  const Robot& robot = Robots()[0];
  ASSERT_STREQ(robot.name, "swerve_robot");
  const std::vector<PlantMotor> motors = Motors(robot);
  const double tilt = 15.0 * std::numbers::pi / 180.0;
  std::string field = TestField("tilt", PreloadPieces(robot));
  {
    std::ifstream in(field);
    std::string text((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
    text.replace(text.find("<worldbody>"), 0,
                 "<option gravity=\"" + std::to_string(9.81 * std::sin(tilt)) +
                     " 0 " + std::to_string(-9.81 * std::cos(tilt)) + "\"/>");
    std::ofstream(field) << text;
  }
  std::unique_ptr<MujocoPlant> plant = Make(robot, field, motors);
  BindAll(*plant, robot, motors);
  const mjModel* m = plant->model();
  // The world as the test holds it between Steps (the plant's accessors are
  // const): the chassis's free joint stays still.
  mjData* d = const_cast<mjData*>(plant->data());
  const int dof =
      m->jnt_dofadr[m->body_jntadr[mj_name2id(m, mjOBJ_BODY, "chassis")]];
  const std::vector<PlantDrive> drives(motors.size(), PlantDrive{0.0, true});
  for (int step = 0; step < 600; ++step) {
    ASSERT_TRUE(plant->Step(1, drives)) << plant->failure();
    mju_zero(At(d->qvel, 1, dof), 6);
  }
  EXPECT_EQ(OutsideTheHopper(m, d, robot), "");
}

// REQ-DEV-41: the swerve example passes under a TRENCH of the 2026 field.
// Placed 1.4 m before the outpost-side TRENCH's arm, centred in its opening,
// it drives straight at kDriveVolts and within kDriveS its back bumper is
// beyond the arm, no robot geom having touched a TRENCH geom.
TEST(RobotModelFieldTest, SwerveRobotPassesUnderATrench) {
  constexpr double kDriveVolts = 6.0;
  constexpr double kDriveS = 3.0;
  constexpr double kArmFarXM = 4.6256 + 0.0762;  // hub x + 3 in
  constexpr double kBackBumperM = 0.34925 + 0.085;
  const Robot& robot = Robots()[0];
  ASSERT_STREQ(robot.name, "swerve_robot");
  const std::vector<PlantMotor> motors = Motors(robot);
  std::unique_ptr<MujocoPlant> plant =
      Make(robot, Runfile("FIELD_XML"), motors, 3.2, 0.64);
  BindAll(*plant, robot, motors);
  const mjModel* m = plant->model();
  const mjData* d = plant->data();
  const int chassis = mj_name2id(m, mjOBJ_BODY, "chassis");
  const int root = m->body_rootid[chassis];
  std::vector<PlantDrive> drives(motors.size(), PlantDrive{0.0, true});
  for (std::size_t i = 0; i < motors.size(); ++i) {
    if (std::string_view(motors[i].name).ends_with("_drive")) {
      drives[i].volts = kDriveVolts;
    }
  }
  double t = 0.0;
  for (; t < kDriveS && At(d->xpos, 3, chassis)[0] - kBackBumperM <= kArmFarXM;
       t += kStepS) {
    ASSERT_TRUE(plant->Step(1, drives)) << plant->failure();
    for (int c = 0; c < d->ncon; ++c) {
      const int g1 = d->contact[c].geom[0];
      const int g2 = d->contact[c].geom[1];
      for (const auto& [own, other] : {std::pair{g1, g2}, std::pair{g2, g1}}) {
        const char* name = mj_id2name(m, mjOBJ_GEOM, other);
        if (IsRobotGeom(m, own, root) && name != nullptr &&
            std::string_view(name).starts_with("blue_trench_")) {
          ADD_FAILURE() << mj_id2name(m, mjOBJ_GEOM, own) << " touches " << name
                        << " at " << t << " s";
        }
      }
    }
  }
  EXPECT_GT(At(d->xpos, 3, chassis)[0] - kBackBumperM, kArmFarXM)
      << "after " << t << " s";
  std::cout << "under the TRENCH in " << t << " s\n";
}

// REQ-DEV-41: on the flat carpet with no drive, 2 s from its authored pose
// the chassis is still (below 1e-3 m/s and 1e-3 rad/s, within 1 mm of where
// it started in x and y), MuJoCo warned nothing, its beam breaks read open,
// and only its wheels touch anything, each of them the robot carpet. Every
// mechanism joint stays below kResidualRadPerS: on a free chassis the CG
// solver leaves the light belted intake rollers a residual that does not
// decay, changing sign and disagreeing with their driver's (measured
// 3.5e-4 rad/s at most with no contact but the wheels'; a welded rig's
// joints are still to 1e-12), and the bound keeps a contact pushing a part
// from hiding in it.
TEST_P(RobotModelTest, RestsStillOnTheCarpetOnItsWheels) {
  const Robot& robot = GetParam();
  const std::vector<PlantMotor> motors = Motors(robot);
  std::unique_ptr<MujocoPlant> plant = Make(robot, "", motors);
  BindAll(*plant, robot, motors);
  const std::vector<PlantDrive> drives(motors.size());
  ASSERT_TRUE(plant->Step(400, drives)) << plant->failure();

  const mjModel* m = plant->model();
  const mjData* d = plant->data();
  for (int w = 0; w < mjNWARNING; ++w) {
    EXPECT_EQ(d->warning[w].number, 0) << "MuJoCo warning " << w;
  }
  const int chassis = mj_name2id(m, mjOBJ_BODY, "chassis");
  std::array<mjtNum, 6> velocity{};  // angular, then linear; world frame
  mj_objectVelocity(m, d, mjOBJ_BODY, chassis, velocity.data(), 0);
  EXPECT_LT(mju_norm3(velocity.data() + 3), 1e-3) << "m/s";
  EXPECT_LT(mju_norm3(velocity.data()), 1e-3) << "rad/s";
  const mjtNum* position = At(d->xpos, 3, chassis);
  EXPECT_LT(std::hypot(position[0], position[1]), 1e-3);
  constexpr double kResidualRadPerS = 5e-3;
  double residual = 0.0;
  for (int dof = robot.free_root ? 6 : 0; dof < m->nv; ++dof) {
    residual = std::max(residual, std::abs(d->qvel[dof]));
  }
  EXPECT_LT(residual, kResidualRadPerS);
  std::cout << robot.name << ": mechanism residual " << residual << " rad/s\n";
  for (std::size_t i = 0; i < robot.beams.size(); ++i) {
    EXPECT_FALSE(plant->input_closed(i))
        << robot.beams[i].name << " reads closed";
  }

  const int root = m->body_rootid[chassis];
  const int carpet = mj_name2id(m, mjOBJ_GEOM, "carpet_robot");
  std::vector<int> wheel_contacts(m->ngeom, 0);
  for (int c = 0; c < d->ncon; ++c) {
    const int g1 = d->contact[c].geom[0];
    const int g2 = d->contact[c].geom[1];
    if (!IsRobotGeom(m, g1, root) && !IsRobotGeom(m, g2, root)) continue;
    const int own = IsRobotGeom(m, g1, root) ? g1 : g2;
    const int other = own == g1 ? g2 : g1;
    const std::string name = mj_id2name(m, mjOBJ_GEOM, own);
    EXPECT_TRUE(name.ends_with("_wheel_geom") && other == carpet)
        << name << " touches "
        << (mj_id2name(m, mjOBJ_GEOM, other) ? mj_id2name(m, mjOBJ_GEOM, other)
                                             : "?");
    ++wheel_contacts[own];
  }
  int wheels = 0;
  for (int g = 0; g < m->ngeom; ++g) {
    const char* name = mj_id2name(m, mjOBJ_GEOM, g);
    if (name == nullptr || !std::string_view(name).ends_with("_wheel_geom")) {
      continue;
    }
    ++wheels;
    EXPECT_GT(wheel_contacts[g], 0) << name << " is off the carpet";
  }
  EXPECT_EQ(wheels, robot.free_root ? 4 : 0);
}

INSTANTIATE_TEST_SUITE_P(Examples, RobotModelTest,
                         ::testing::ValuesIn(Robots()),
                         [](const ::testing::TestParamInfo<Robot>& info) {
                           return std::string(info.param.name);
                         });

}  // namespace
}  // namespace talos::simulation::mujoco

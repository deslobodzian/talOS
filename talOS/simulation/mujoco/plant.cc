#include "talOS/simulation/mujoco/plant.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "talOS/hardware/gateway.h"

namespace talos::simulation::mujoco {

namespace {

using hardware::sim::PlantMotor;

constexpr double kTwoPi = 2.0 * std::numbers::pi;

// Section E's option table: the only solver settings a MujocoPlant steps
// with, whatever the field or robot MJCF says.
constexpr double kTimestepS = 0.005;
constexpr int kIterations = 20;
constexpr double kTolerance = 1e-8;
constexpr mjtSize kArenaBytes = mjtSize{64} << 20;

// The field's robot-geom collision bits (fields/rebuilt_2026/README.md,
// "Collision contract"), and a visual geom's.
constexpr int kRobotContype = 4;
constexpr int kRobotConaffinity = 21;

// What mjs_attach warns on every attach with an empty prefix (3.17.0
// user_api.cc), which MakePlant does by design.
constexpr std::string_view kEmptyPrefixWarning =
    "Attaching a spec with an empty prefix";

// The warnings after which the world is not trusted (D). mjWARN_INERTIA is
// left to MuJoCo, which prints its first occurrence.
constexpr std::array<int, 6> kFatalWarnings = {
    mjWARN_CONTACTFULL, mjWARN_CNSTRFULL, mjWARN_BADQPOS,
    mjWARN_BADQVEL,     mjWARN_BADQACC,   mjWARN_BADCTRL,
};

struct SpecDeleter {
  void operator()(mjSpec* spec) const { mj_deleteSpec(spec); }
};
using SpecPtr = std::unique_ptr<mjSpec, SpecDeleter>;

// The `index`th group of `stride` entries of a MuJoCo array.
template <typename T>
T* At(T* array, int stride, int index) {
  return array + static_cast<std::ptrdiff_t>(stride) * index;
}

// Heading of a body quaternion (w, x, y, z): its ZYX yaw.
double Yaw(const mjtNum* q) {
  return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                    1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

[[noreturn]] void Refuse(const std::string& message) {
  throw std::invalid_argument(message);
}

std::string NameOf(mjsElement* element) {
  const char* name = mjs_getString(mjs_getName(element));
  return name == nullptr ? std::string() : std::string(name);
}

std::string TypeName(mjtObj type) {
  if (type == mjOBJ_FRAME) return "frame";
  const char* name = mju_type2Str(type);
  return name == nullptr ? std::string("element") : std::string(name);
}

// "geom 'wheel'", or "unnamed geom in body 'chassis'".
std::string Describe(mjsElement* element, mjtObj type) {
  const std::string name = NameOf(element);
  if (!name.empty()) return TypeName(type) + " '" + name + "'";
  std::string out = "unnamed " + TypeName(type);
  if (mjsBody* parent = mjs_getParent(element); parent != nullptr) {
    out += " in body '" + NameOf(parent->element) + "'";
  }
  return out;
}

SpecPtr Parse(const std::string& path, std::string_view role) {
  std::array<char, 1024> error{};
  SpecPtr spec(mj_parseXML(path.c_str(), nullptr, error.data(),
                           static_cast<int>(error.size())));
  if (!spec) {
    Refuse(std::string(role) + " model '" + path + "': " + error.data());
  }
  return spec;
}

// The field's two coincident carpet planes, with its values and collision
// bits (fields/rebuilt_2026/generate_field.py), and nothing else.
SpecPtr FlatCarpet() {
  SpecPtr spec(mj_makeSpec());
  if (!spec) Refuse("flat carpet: mj_makeSpec failed");
  mjsBody* world = mjs_findBody(spec.get(), "world");
  // The field's centre, 651.22 x 317.69 in; a plane collides over its whole
  // extent, so its position and size are only rendering.
  constexpr double kCentreX = 651.22 * 0.0254 / 2.0;
  constexpr double kCentreY = 317.69 * 0.0254 / 2.0;
  struct Carpet {
    const char* name;
    int contype;
    int condim;
    std::array<double, 3> friction;
    int group;
  };
  constexpr std::array<Carpet, 2> kCarpets = {{
      {"carpet_fuel", 8, 6, {0.8, 0.001, 0.01}, 0},
      {"carpet_robot", 16, 3, {0.8, 0.005, 0.0001}, 3},
  }};
  for (const Carpet& carpet : kCarpets) {
    mjsGeom* geom = mjs_addGeom(world, nullptr);
    geom->type = mjGEOM_PLANE;
    geom->pos[0] = kCentreX;
    geom->pos[1] = kCentreY;
    geom->pos[2] = 0.0;
    geom->size[0] = 11.0;
    geom->size[1] = 7.0;
    geom->size[2] = 0.1;
    geom->contype = carpet.contype;
    geom->conaffinity = 0;
    geom->condim = carpet.condim;
    std::copy(carpet.friction.begin(), carpet.friction.end(), geom->friction);
    geom->group = carpet.group;
    mjs_setName(geom->element, carpet.name);
  }
  return spec;
}

// Writes section E's table into the assembled spec, after attach (which has
// already dropped the robot's <option> and <size>) and before compile, so
// the field's solver settings give way too; its other options stay.
void ApplyOptionTable(mjSpec* spec) {
  mjOption& option = spec->option;
  option.timestep = kTimestepS;
  option.solver = mjSOL_CG;
  option.iterations = kIterations;
  option.tolerance = kTolerance;
  option.cone = mjCONE_ELLIPTIC;
  option.integrator = mjINT_IMPLICITFAST;
  option.enableflags &= ~static_cast<int>(mjENBL_SLEEP);
  option.disableflags |= static_cast<int>(mjDSBL_AUTORESET);
  spec->memory = kArenaBytes;
}

const PlantMotor* FindMotor(std::span<const PlantMotor> motors,
                            std::string_view name) {
  for (const PlantMotor& motor : motors) {
    if (motor.name == name) return &motor;
  }
  return nullptr;
}

bool IsLimited(mjtLimited limited, const double range[2]) {
  return limited == mjLIMITED_TRUE || range[0] != 0.0 || range[1] != 0.0;
}

// Every actuator of `spec` names a motor and is a plain <motor> on a hinge or
// slide joint or a tendon (A, H.5).
void CheckActuators(mjSpec* spec, const std::string& file,
                    std::span<const PlantMotor> motors) {
  for (mjsElement* element = mjs_firstElement(spec, mjOBJ_ACTUATOR);
       element != nullptr; element = mjs_nextElement(spec, element)) {
    mjsActuator* actuator = mjs_asActuator(element);
    const std::string where =
        file + ": " + Describe(element, mjOBJ_ACTUATOR) + ": ";
    const std::string name = NameOf(element);
    if (FindMotor(motors, name) == nullptr) {
      Refuse(where +
             "names no configured motor; every actuator binds the motor of "
             "its name");
    }
    if (actuator->type != mjACTUATOR_MOTOR) {
      Refuse(where + "is not a <motor>; a bound actuator is a plain <motor>");
    }
    if (actuator->gaintype != mjGAIN_FIXED || actuator->gainprm[0] != 1.0 ||
        actuator->biastype != mjBIAS_NONE || actuator->dyntype != mjDYN_NONE) {
      Refuse(where +
             "has a gain, bias or dynamics of its own; a bound actuator is a "
             "plain <motor>");
    }
    if (IsLimited(actuator->ctrllimited, actuator->ctrlrange)) {
      Refuse(where + "has a ctrlrange; the backend limits the voltage");
    }
    if (IsLimited(actuator->forcelimited, actuator->forcerange)) {
      Refuse(where +
             "has a forcerange; the plant writes the motor's torque limits");
    }
    if (actuator->delay != 0.0 || actuator->nsample != 0) {
      Refuse(where + "has a delay or nsample; a bound actuator acts at once");
    }
    const char* target_name = mjs_getString(actuator->target);
    const std::string target = target_name == nullptr ? "" : target_name;
    // What changes the force after the plant's law (an actuator force range
    // on the joint or tendon, which would clamp behind the backend's reported
    // current, or gravity compensation routed through actuators) is refused.
    const std::string behind =
        "; the force a bound actuator applies is the motor's law alone";
    if (actuator->trntype == mjTRN_JOINT) {
      mjsElement* element = mjs_findElement(spec, mjOBJ_JOINT, target.c_str());
      if (const mjsJoint* joint = element ? mjs_asJoint(element) : nullptr) {
        if (joint->type != mjJNT_HINGE && joint->type != mjJNT_SLIDE) {
          Refuse(where + "drives joint '" + target +
                 "', which is neither a hinge nor a slide");
        }
        if (IsLimited(joint->actfrclimited, joint->actfrcrange)) {
          Refuse(where + "drives joint '" + target +
                 "', which has an actuatorfrcrange" + behind);
        }
        if (joint->actgravcomp) {
          Refuse(where + "drives joint '" + target +
                 "', which has actuatorgravcomp" + behind);
        }
      }
    } else if (actuator->trntype == mjTRN_TENDON) {
      mjsElement* element = mjs_findElement(spec, mjOBJ_TENDON, target.c_str());
      if (const mjsTendon* tendon = element ? mjs_asTendon(element) : nullptr;
          tendon != nullptr &&
          IsLimited(tendon->actfrclimited, tendon->actfrcrange)) {
        Refuse(where + "drives tendon '" + target +
               "', which has an actuatorfrcrange" + behind);
      }
    } else {
      Refuse(where +
             "transmission is not a joint or a tendon; a bound actuator "
             "drives a hinge, a slide or a tendon");
    }
  }
}

// H.2: one root body, nothing else of the robot's own in its worldbody, and
// every geom on the field's robot bits or visual. Returns the root.
mjsBody* CheckRobot(mjSpec* robot, const std::string& file) {
  mjsBody* world = mjs_findBody(robot, "world");
  if (world == nullptr) Refuse(file + ": no worldbody");
  for (const mjtObj type : {mjOBJ_GEOM, mjOBJ_SITE, mjOBJ_JOINT, mjOBJ_CAMERA,
                            mjOBJ_LIGHT, mjOBJ_FRAME}) {
    if (mjsElement* own = mjs_firstChild(world, type, 0); own != nullptr) {
      Refuse(file + ": " + Describe(own, type) +
             " is in the worldbody; a robot model's worldbody holds only its "
             "root body (the field is the world)");
    }
  }
  mjsBody* root = nullptr;
  for (mjsElement* body = mjs_firstChild(world, mjOBJ_BODY, 0); body != nullptr;
       body = mjs_nextChild(world, body, 0)) {
    if (root != nullptr) {
      Refuse(file + ": the worldbody holds " +
             Describe(root->element, mjOBJ_BODY) + " and " +
             Describe(body, mjOBJ_BODY) +
             "; a robot model has exactly one root body");
    }
    root = mjs_asBody(body);
  }
  if (root == nullptr) Refuse(file + ": the worldbody holds no body");

  for (mjsElement* element = mjs_firstElement(robot, mjOBJ_GEOM);
       element != nullptr; element = mjs_nextElement(robot, element)) {
    const mjsGeom* geom = mjs_asGeom(element);
    const bool robot_bits = geom->contype == kRobotContype &&
                            geom->conaffinity == kRobotConaffinity;
    const bool visual = geom->contype == 0 && geom->conaffinity == 0;
    if (!robot_bits && !visual) {
      Refuse(
          file + ": " + Describe(element, mjOBJ_GEOM) + " has contype " +
          std::to_string(geom->contype) + " and conaffinity " +
          std::to_string(geom->conaffinity) +
          "; a robot geom collides as (4, 21) or is visual (0, 0), per "
          "the field's collision contract (fix: <default><geom contype=\"4\" "
          "conaffinity=\"21\"/></default>)");
    }
  }

  // MuJoCo 3.17 aligns a free joint with its body's inertial frame only on a
  // body with that one joint and no child body (user_objects.cc,
  // mjCBody::Compile). Aligned, the root's frame would move to its centre of
  // mass and turn to its principal axes, and the chassis reported from it.
  mjsElement* joint = mjs_firstChild(root, mjOBJ_JOINT, 0);
  const bool single_free = joint != nullptr &&
                           mjs_nextChild(root, joint, 0) == nullptr &&
                           mjs_asJoint(joint)->type == mjJNT_FREE &&
                           mjs_firstChild(root, mjOBJ_BODY, 0) == nullptr;
  if (single_free) {
    const mjtAlignFree align = mjs_asJoint(joint)->align;
    if (align == mjALIGNFREE_TRUE ||
        (align == mjALIGNFREE_AUTO && robot->compiler.alignfree)) {
      Refuse(file + ": " + Describe(joint, mjOBJ_JOINT) +
             " would be aligned with the root's centre of mass (align / "
             "alignfree), which moves and turns the reported chassis frame "
             "away from the model's origin; set align=\"false\"");
    }
  }
  return root;
}

constexpr std::array<mjtObj, 24> kNamedTypes = {
    mjOBJ_BODY,     mjOBJ_JOINT,   mjOBJ_GEOM,     mjOBJ_SITE,   mjOBJ_CAMERA,
    mjOBJ_LIGHT,    mjOBJ_FRAME,   mjOBJ_FLEX,     mjOBJ_MESH,   mjOBJ_SKIN,
    mjOBJ_HFIELD,   mjOBJ_TEXTURE, mjOBJ_MATERIAL, mjOBJ_PAIR,   mjOBJ_EXCLUDE,
    mjOBJ_EQUALITY, mjOBJ_TENDON,  mjOBJ_ACTUATOR, mjOBJ_SENSOR, mjOBJ_NUMERIC,
    mjOBJ_TEXT,     mjOBJ_TUPLE,   mjOBJ_KEY,      mjOBJ_PLUGIN,
};

// H.3: attached with no prefix, a robot name equal to a field name of any
// type is refused here, naming both (MuJoCo refuses only one of the same
// type, at compile).
void CheckNames(mjSpec* field, const std::string& field_file, mjSpec* robot,
                const std::string& robot_file) {
  std::unordered_map<std::string, mjtObj> field_names;
  for (const mjtObj type : kNamedTypes) {
    for (mjsElement* element = mjs_firstElement(field, type);
         element != nullptr; element = mjs_nextElement(field, element)) {
      std::string name = NameOf(element);
      if (!name.empty() && name != "world") {
        field_names.emplace(std::move(name), type);
      }
    }
  }
  for (const mjtObj type : kNamedTypes) {
    for (mjsElement* element = mjs_firstElement(robot, type);
         element != nullptr; element = mjs_nextElement(robot, element)) {
      const std::string name = NameOf(element);
      if (name.empty() || name == "world") continue;
      if (const auto it = field_names.find(name); it != field_names.end()) {
        Refuse(robot_file + ": " + TypeName(type) + " '" + name +
               "' has the name of " + field_file + "'s " +
               TypeName(it->second) +
               "; robot and field names are not prefixed, so they must "
               "differ");
      }
    }
  }
}

std::string JointOrTendon(const mjModel* m, int actuator) {
  const int id = *At(m->actuator_trnid, 2, actuator);
  const bool tendon = m->actuator_trntype[actuator] == mjTRN_TENDON;
  const char* name = mj_id2name(m, tendon ? mjOBJ_TENDON : mjOBJ_JOINT, id);
  return std::string(tendon ? "tendon '" : "joint '") +
         (name == nullptr ? "" : name) + "'";
}

// Elements of `type` in `spec`, the world body included for mjOBJ_BODY.
int CountElements(mjSpec* spec, mjtObj type) {
  int count = 0;
  for (mjsElement* element = mjs_firstElement(spec, type); element != nullptr;
       element = mjs_nextElement(spec, element)) {
    ++count;
  }
  return count;
}

// The joint whose qpos block holds `address` (joints are numbered in qpos
// order), or -1.
int JointOfQpos(const mjModel* m, int address) {
  int joint = -1;
  for (int j = 0; j < m->njnt && m->jnt_qposadr[j] <= address; ++j) {
    joint = j;
  }
  return joint;
}

// MuJoCo's log handler before InstallMujocoProcessHandlers replaced it.
mjfLogHandler previous_log_handler = nullptr;

// Errors end the process naming the plant (MuJoCo requires an error handler
// not to return); the warning every MakePlant raises by design is dropped;
// everything else goes to MuJoCo's own handler, set to the console only.
void GatewayLogHandler(const mjLogMessage* message) {
  if (message->level == mjLOG_ERROR) {
    std::fprintf(stderr, "mujoco plant: MuJoCo error%s%s: %s\n",
                 message->func != nullptr ? " in " : "",
                 message->func != nullptr ? message->func : "",
                 message->subject);
    std::fflush(stderr);
    std::exit(1);
  }
  if (message->level == mjLOG_WARNING &&
      std::string_view(message->subject).find(kEmptyPrefixWarning) !=
          std::string_view::npos) {
    return;
  }
  previous_log_handler(message);
}

const char* WarningName(int warning) {
  switch (warning) {
    case mjWARN_INERTIA:
      return "mjWARN_INERTIA";
    case mjWARN_CONTACTFULL:
      return "mjWARN_CONTACTFULL";
    case mjWARN_CNSTRFULL:
      return "mjWARN_CNSTRFULL";
    case mjWARN_BADQPOS:
      return "mjWARN_BADQPOS";
    case mjWARN_BADQVEL:
      return "mjWARN_BADQVEL";
    case mjWARN_BADQACC:
      return "mjWARN_BADQACC";
    case mjWARN_BADCTRL:
      return "mjWARN_BADCTRL";
    default:
      return "mjWARN_?";
  }
}

}  // namespace

static_assert(mjNWARNING == 7,
              "MujocoPlant::warnings_ holds one count per mjtWarning");

void InstallMujocoProcessHandlers() {
  mjLogConfig config = mju_getLogConfig();
  config.logto_console = 1;
  config.logto_file = 0;
  mju_setLogConfig(config);
  const mjfLogHandler previous = mju_setLogHandler(GatewayLogHandler);
  if (previous != GatewayLogHandler) previous_log_handler = previous;
}

void MujocoPlant::ModelDeleter::operator()(mjModel_* model) const {
  mj_deleteModel(model);
}
void MujocoPlant::DataDeleter::operator()(mjData_* data) const {
  mj_deleteData(data);
}

MujocoPlant::~MujocoPlant() = default;

std::unique_ptr<MujocoPlant> MakePlant(const PlantModel& model,
                                       std::span<const PlantMotor> motors) {
  const std::string& robot_file = model.robot_xml;
  const std::string field_file = model.field_xml.empty()
                                     ? std::string("the flat carpet")
                                     : model.field_xml;
  SpecPtr world =
      model.field_xml.empty() ? FlatCarpet() : Parse(model.field_xml, "field");
  SpecPtr robot = Parse(robot_file, "robot");

  mjsBody* robot_root = CheckRobot(robot.get(), robot_file);
  CheckActuators(world.get(), field_file, motors);
  CheckActuators(robot.get(), robot_file, motors);
  CheckNames(world.get(), field_file, robot.get(), robot_file);

  // A robot model is self-contained: it compiles on its own and names no
  // field element, so one model runs on every field and on the flat carpet.
  // Compiling it alone also catches what mjs_attach would drop without an
  // error, a robot element whose reference does not resolve (a misspelt
  // joint, tendon or site), with MuJoCo naming the reference; below, every
  // element it holds must arrive in the world. (A second parse: mj_copySpec
  // drops such an element too.) Its compile warnings print here; some (a
  // deprecated <compiler> or <statistic> attribute) only here, since attach
  // drops those settings, so they are not silenced.
  {
    SpecPtr alone = Parse(robot_file, "robot");
    const std::unique_ptr<mjModel, MujocoPlant::ModelDeleter> compiled(
        mj_compile(alone.get(), nullptr));
    if (!compiled) {
      Refuse(robot_file +
             ": a robot model is self-contained (it compiles on its own and "
             "names no field element, so it runs on every field and on the "
             "flat carpet), and this one does not compile alone: " +
             mjs_getError(alone.get()));
    }
  }
  std::array<int, kNamedTypes.size()> expected{};
  for (std::size_t t = 0; t < kNamedTypes.size(); ++t) {
    expected[t] = CountElements(world.get(), kNamedTypes[t]) +
                  CountElements(robot.get(), kNamedTypes[t]);
  }

  // H.4: the root body on a frame at the start pose; its own pos and quat
  // compose after the frame. A deep copy, so the robot spec can go.
  mjs_setDeepCopy(world.get(), 1);
  mjsFrame* frame = mjs_addFrame(mjs_findBody(world.get(), "world"), nullptr);
  frame->pos[0] = model.start_x_m;
  frame->pos[1] = model.start_y_m;
  frame->pos[2] = 0.0;
  frame->quat[0] = std::cos(model.start_yaw_rad / 2.0);
  frame->quat[1] = 0.0;
  frame->quat[2] = 0.0;
  frame->quat[3] = std::sin(model.start_yaw_rad / 2.0);
  mjsElement* attached =
      mjs_attach(frame->element, robot_root->element, "", "");
  if (attached == nullptr) {
    Refuse(robot_file + ": attaching to " + field_file + ": " +
           mjs_getError(world.get()));
  }
  for (std::size_t t = 0; t < kNamedTypes.size(); ++t) {
    // The robot's world body stays behind; the start-pose frame is new.
    const int arrived = CountElements(world.get(), kNamedTypes[t]) +
                        (kNamedTypes[t] == mjOBJ_BODY ? 1 : 0) -
                        (kNamedTypes[t] == mjOBJ_FRAME ? 1 : 0);
    if (arrived != expected[t]) {
      Refuse(robot_file + ": the attached model has " +
             std::to_string(expected[t] - arrived) + " fewer " +
             TypeName(kNamedTypes[t]) +
             " element(s) than the robot file: MuJoCo dropped them when "
             "attaching");
    }
  }
  robot.reset();

  // H.5 and A: every actuator (all bind a motor) becomes gain·ctrl +
  // bias₂·velocity, force-limited, with the motor's no-load drag G²·b_m as
  // its own damping. The gain, bias and limits are written per Step; until
  // the first, they are zero.
  std::unique_ptr<MujocoPlant> plant(new MujocoPlant());
  for (mjsElement* element = mjs_firstElement(world.get(), mjOBJ_ACTUATOR);
       element != nullptr; element = mjs_nextElement(world.get(), element)) {
    mjsActuator* actuator = mjs_asActuator(element);
    const PlantMotor* motor = FindMotor(motors, NameOf(element));
    const double gear = motor->gear;
    actuator->gaintype = mjGAIN_FIXED;
    actuator->biastype = mjBIAS_AFFINE;
    actuator->dyntype = mjDYN_NONE;
    std::fill(std::begin(actuator->gainprm), std::end(actuator->gainprm), 0.0);
    std::fill(std::begin(actuator->biasprm), std::end(actuator->biasprm), 0.0);
    actuator->ctrllimited = mjLIMITED_FALSE;
    actuator->forcelimited = mjLIMITED_TRUE;
    actuator->forcerange[0] = -1.0;
    actuator->forcerange[1] = 1.0;
    actuator->damping[0] += gear * gear * motor->no_load_damping_nm_per_radps;
    MujocoPlant::Actuator entry;
    entry.motor = NameOf(element);
    entry.gear = gear;
    entry.no_load_damping_nm_per_radps = motor->no_load_damping_nm_per_radps;
    plant->actuators_.push_back(std::move(entry));
  }

  ApplyOptionTable(world.get());
  plant->model_.reset(mj_compile(world.get(), nullptr));
  if (!plant->model_) {
    Refuse("compiling " + robot_file + " on " + field_file + ": " +
           mjs_getError(world.get()));
  }
  const mjModel* m = plant->model_.get();
  plant->root_body_ = mjs_getId(attached);
  world.reset();

  plant->data_.reset(mj_makeData(m));
  if (!plant->data_) {
    Refuse("compiling " + robot_file + ": mj_makeData failed (arena of " +
           std::to_string(kArenaBytes) + " bytes)");
  }
  plant->step_us_ = static_cast<uint32_t>(std::llround(m->opt.timestep * 1e6));

  for (MujocoPlant::Actuator& actuator : plant->actuators_) {
    actuator.id = mj_name2id(m, mjOBJ_ACTUATOR, actuator.motor.c_str());
    actuator.ctrl = m->actuator_ctrladr[actuator.id];
    actuator.output = m->actuator_outadr[actuator.id];
    actuator.target = JointOrTendon(m, actuator.id);
  }
  const int root = plant->root_body_;
  plant->root_free_ = m->body_jntnum[root] > 0 &&
                      m->jnt_type[m->body_jntadr[root]] == mjJNT_FREE;

  mjData* d = plant->data_.get();
  mj_forward(m, d);
  // Each actuator's inertia in its own coordinate at the reference pose,
  // 1/(moment·M⁻¹·moment) (as MuJoCo's dampratio uses; actuator_acc0 is
  // |M⁻¹·moment|, not an inertia): mechanism kg·m², armature included.
  const int nv = static_cast<int>(m->nv);
  std::vector<mjtNum> moment(static_cast<std::size_t>(nv));
  std::vector<mjtNum> solved(moment.size());
  for (MujocoPlant::Actuator& actuator : plant->actuators_) {
    const int out = actuator.output;
    mju_sparse2dense(moment.data(), d->actuator_moment, 1, nv,
                     d->moment_rownnz + out, d->moment_rowadr + out,
                     d->moment_colind);
    mj_solveM(m, d, solved.data(), moment.data(), 1);
    const double weight = mju_dot(moment.data(), solved.data(), nv);
    // No moment at the reference pose (a tendon across its sled's travel,
    // say): the inertia there is unknown, not zero.
    actuator.inertia_kg_m2 =
        weight > 0.0 ? 1.0 / weight : std::numeric_limits<double>::quiet_NaN();
  }
  for (std::size_t w = 0; w < plant->warnings_.size(); ++w) {
    plant->warnings_[w] = d->warning[w].number;
  }
  // The heading starts at the start pose's turn, not at atan2's branch.
  plant->raw_yaw_rad_ = Yaw(At(d->xquat, 4, root));
  plant->chassis_.yaw_rad =
      model.start_yaw_rad +
      std::remainder(plant->raw_yaw_rad_ - model.start_yaw_rad, kTwoPi);
  plant->Refresh();
  return plant;
}

bool MujocoPlant::Bind(std::span<const hardware::sim::PlantMotor> motors,
                       std::span<const hardware::sim::PlantInput> inputs,
                       std::span<bool> owned, hardware::ConfigError& error) {
  const mjModel* m = model_.get();
  auto refuse = [&error](std::string device, std::string operation,
                         std::string message) {
    error = hardware::ConfigError{};
    error.device_name = std::move(device);
    error.operation = std::move(operation);
    error.message = std::move(message);
    return false;
  };
  if (owned.size() != motors.size()) {
    return refuse("", "bind_mujoco_plant",
                  "owned has " + std::to_string(owned.size()) +
                      " entries for " + std::to_string(motors.size()) +
                      " motors");
  }

  std::vector<Bound> bound;
  for (std::size_t a = 0; a < actuators_.size(); ++a) {
    const Actuator& actuator = actuators_[a];
    std::size_t index = motors.size();
    for (std::size_t i = 0; i < motors.size(); ++i) {
      if (motors[i].name == actuator.motor) index = i;
    }
    if (index == motors.size()) {
      return refuse(actuator.motor, "bind_mujoco_actuator",
                    "actuator '" + actuator.motor +
                        "' of the model names no motor of this config");
    }
    const PlantMotor& motor = motors[index];
    // Exact: the wire carries the ratios as doubles, and both ends compute
    // the gear and the motor type's damping the same way.
    if (motor.gear != actuator.gear ||
        motor.no_load_damping_nm_per_radps !=
            actuator.no_load_damping_nm_per_radps) {
      return refuse(actuator.motor, "bind_mujoco_actuator",
                    "gear or motor type differs from the config the model "
                    "was compiled with (its damping G^2*b_m is compiled in); "
                    "restart the gateway with this config");
    }
    if (!(motor.resistance_ohm > 0.0)) {
      return refuse(actuator.motor, "bind_mujoco_actuator",
                    "motor has no winding resistance");
    }
    Bound entry;
    entry.motor = index;
    entry.actuator = &actuator;
    entry.gear = motor.gear;
    entry.torque_constant_nm_per_a = motor.torque_constant_nm_per_a;
    entry.back_emf_v_per_radps = motor.back_emf_v_per_radps;
    entry.resistance_ohm = motor.resistance_ohm;
    bound.push_back(entry);
  }

  std::vector<Input> bound_inputs;
  for (const hardware::sim::PlantInput& input : inputs) {
    const std::string sensor(input.sensor);
    const std::string device(input.input);
    const int id = mj_name2id(m, mjOBJ_SENSOR, sensor.c_str());
    if (id < 0) {
      return refuse(device, "bind_world_input",
                    "the model has no sensor '" + sensor + "'");
    }
    Input entry;
    entry.address = m->sensor_adr[id];
    if (m->sensor_type[id] == mjSENS_RANGEFINDER) {
      entry.rangefinder = true;
      entry.cutoff = m->sensor_cutoff[id];
      if (m->sensor_objtype[id] != mjOBJ_SITE || m->sensor_dim[id] != 1) {
        return refuse(device, "bind_world_input",
                      "rangefinder '" + sensor +
                          "' is not one ray from a site measuring dist");
      }
      if (!(entry.cutoff > 0.0)) {
        return refuse(device, "bind_world_input",
                      "rangefinder '" + sensor +
                          "' has no cutoff; it reads closed while it sees a "
                          "surface nearer than its cutoff");
      }
    } else if (m->sensor_type[id] != mjSENS_TOUCH) {
      return refuse(device, "bind_world_input",
                    "sensor '" + sensor +
                        "' is neither a rangefinder nor a touch sensor");
    }
    bound_inputs.push_back(entry);
  }

  // The table: one line per motor, then the light-inertia warnings (Q4).
  std::vector<Binding> bindings;
  std::ostringstream table;
  table << "mujoco binding (step " << step_us_ << " us)\n";
  std::ostringstream warnings;
  const double h = m->opt.timestep;
  for (std::size_t i = 0; i < motors.size(); ++i) {
    const auto it =
        std::find_if(bound.begin(), bound.end(),
                     [i](const Bound& entry) { return entry.motor == i; });
    table << "  " << motors[i].name;
    if (it == bound.end()) {
      table << ": unbound, analytic\n";
      continue;
    }
    const Actuator& actuator = *it->actuator;
    const double j_act = actuator.inertia_kg_m2;
    const double j_rotor = j_act / (it->gear * it->gear);
    table << ": actuator '" << actuator.motor << "' on " << actuator.target
          << ", G " << it->gear;
    if (std::isnan(j_act)) {
      table << ", J_act and J_rotor unknown at the reference pose (no "
               "moment there)\n";
    } else {
      table << ", J_act " << j_act << " kg*m^2, J_rotor " << j_rotor
            << " kg*m^2\n";
    }
    const double b_em = it->torque_constant_nm_per_a *
                        it->back_emf_v_per_radps / it->resistance_ohm;
    const double light = h * b_em / 2.0;
    bindings.push_back(
        Binding{.motor = i,
                .inertia_kg_m2 = j_act,
                .rotor_inertia_kg_m2 = j_rotor,
                .light = !std::isnan(j_rotor) && j_rotor < light});
    if (bindings.back().light) {
      warnings << "  warning: " << motors[i].name << "'s rotor-side inertia "
               << j_rotor << " kg*m^2 is below h*b_em/2 = " << light
               << " kg*m^2: its transients are wrong; add the rotor's "
                  "armature to its actuator\n";
    }
  }

  // Commit.
  std::fill(owned.begin(), owned.end(), false);
  for (const Bound& entry : bound) owned[entry.motor] = true;
  bound_ = std::move(bound);
  inputs_ = std::move(bound_inputs);
  rotors_.assign(motors.size(), hardware::sim::PlantRotor{});
  overspeed_.assign(bound_.size(), false);
  binding_table_ = table.str() + warnings.str();
  bindings_ = std::move(bindings);
  Refresh();
  for (std::size_t b = 0; b < bound_.size(); ++b) {
    overspeed_[b] = std::fabs(rotors_[bound_[b].motor].velocity_rps) >
                    hardware::Gateway::kMaxRotorRps;
  }
  return true;
}

bool MujocoPlant::Step(int steps,
                       std::span<const hardware::sim::PlantDrive> drives) {
  if (failed_) return false;
  mjModel* m = model_.get();
  mjData* d = data_.get();
  if (steps < 0 || drives.size() < rotors_.size()) {
    std::snprintf(failure_.data(), failure_.size(),
                  "mujoco plant: Step(%d) with %zu drives for %zu motors",
                  steps, drives.size(), rotors_.size());
    failed_ = true;
    return false;
  }

  // A: the torque law of PlantDrive, in mechanism units (actuator length is
  // mechanism radians, so rotor speed is G times its velocity):
  //   force = G·k_t/R·volts − G²·k_t·k_e/R·v, clamped to G·[min, max],
  // with the passive G²·b_m·v compiled into the actuator's damping.
  for (const Bound& entry : bound_) {
    const hardware::sim::PlantDrive& drive = drives[entry.motor];
    const Actuator& actuator = *entry.actuator;
    const double g = entry.gear;
    const double kt_over_r =
        entry.torque_constant_nm_per_a / entry.resistance_ohm;
    d->ctrl[actuator.ctrl] = drive.volts;
    mjtNum* gain = At(m->actuator_gainprm, mjNGAIN, actuator.id);
    mjtNum* bias = At(m->actuator_biasprm, mjNBIAS, actuator.id);
    gain[0] = g * kt_over_r;
    bias[0] = 0.0;
    bias[1] = 0.0;
    bias[2] =
        drive.back_emf ? -g * g * kt_over_r * entry.back_emf_v_per_radps : 0.0;
    mjtNum* range = At(m->actuator_forcerange, 2, actuator.id);
    range[0] = g * drive.min_torque_nm;
    range[1] = g * drive.max_torque_nm;
  }

  for (int step = 0; step < steps; ++step) {
    mj_step(m, d);
    ++steps_;
    for (const int warning : kFatalWarnings) {
      if (d->warning[warning].number != warnings_[warning]) {
        Fail(warning, d->warning[warning].lastinfo);
        return false;
      }
    }
    if (rules_.empty()) continue;
    // mj_step leaves the kinematics at the step's start; the rules read them
    // at its end (world_rules.h). Positions and velocities, no more: not
    // mj_forward, whose collision and constraint stages cost a step's worth.
    mj_kinematics(m, d);
    mj_comPos(m, d);
    mj_comVel(m, d);
    // A rule must not throw; one that does fails the plant like one that
    // returns false (no cost while none throws).
    try {
      for (WorldRules* rules : rules_) {
        if (!rules->AfterStep(*m, *d, steps_, match_input_)) {
          Fail(rules->failure());
          return false;
        }
      }
    } catch (const std::exception& error) {
      Fail(error.what());
      return false;
    } catch (...) {
      Fail("a world rule threw something that is not a std::exception");
      return false;
    }
  }
  Refresh();
  for (std::size_t b = 0; b < bound_.size(); ++b) {
    const bool over = std::fabs(rotors_[bound_[b].motor].velocity_rps) >
                      hardware::Gateway::kMaxRotorRps;
    if (over && !overspeed_[b]) ++overspeed_excursions_;
    overspeed_[b] = over;
  }
  return true;
}

void MujocoPlant::Fail(int warning, int info) {
  const mjModel* m = model_.get();
  const mjData* d = data_.get();
  const char* element = nullptr;
  const char* kind = "";
  if (warning == mjWARN_BADQPOS) {
    // Its info is a qpos address, which a free joint (7 of them for 6 dofs)
    // sets apart from a dof.
    if (const int joint = JointOfQpos(m, info); joint >= 0) {
      element = mj_id2name(m, mjOBJ_JOINT, joint);
      kind = "joint";
    }
  } else if (warning == mjWARN_BADQVEL || warning == mjWARN_BADQACC) {
    if (info >= 0 && info < m->nv) {
      element = mj_id2name(m, mjOBJ_JOINT, m->dof_jntid[info]);
      kind = "joint";
    }
  } else if (warning == mjWARN_BADCTRL) {
    if (info >= 0 && info < m->nu) {
      element = mj_id2name(m, mjOBJ_ACTUATOR, info);
      kind = "actuator";
    }
  }
  const std::size_t text_info = info < 0 ? 0 : static_cast<std::size_t>(info);
  if (element != nullptr) {
    std::snprintf(failure_.data(), failure_.size(),
                  "mujoco %s at plant time %.4f s (%s '%s'): %s",
                  WarningName(warning), d->time, kind, element,
                  mju_warningText(warning, text_info));
  } else {
    std::snprintf(failure_.data(), failure_.size(),
                  "mujoco %s at plant time %.4f s: %s", WarningName(warning),
                  d->time, mju_warningText(warning, text_info));
  }
  failed_ = true;
}

void MujocoPlant::Fail(std::string_view text) {
  std::snprintf(failure_.data(), failure_.size(),
                "mujoco world rule at plant time %.4f s: %.*s", data_->time,
                static_cast<int>(text.size()), text.data());
  failed_ = true;
}

void MujocoPlant::AddRules(WorldRules& rules) {
  if (failed_) {
    throw std::logic_error("MujocoPlant::AddRules on a failed plant: " +
                           std::string(failure()));
  }
  if (steps_ != 0) {
    throw std::logic_error(
        "MujocoPlant::AddRules after the first Step; rules attach before the "
        "world moves");
  }
  if (std::find(rules_.begin(), rules_.end(), &rules) != rules_.end()) {
    throw std::invalid_argument(
        "MujocoPlant::AddRules: this rule is already added; a rule runs once "
        "after each step");
  }
  mjModel* m = model_.get();
  mjData* d = data_.get();
  rules.Attach(*m, *d);
  // What Attach placed is the world every reader and the next Attach see,
  // checked as mj_step checks it, so a bad placement is refused here, naming
  // the warning, not blamed on the first Step.
  mj_forward(m, d);
  mj_checkPos(m, d);
  mj_checkVel(m, d);
  mj_checkAcc(m, d);
  for (const int warning : kFatalWarnings) {
    if (d->warning[warning].number != warnings_[warning]) {
      Fail(warning, d->warning[warning].lastinfo);
      throw std::invalid_argument(
          "a world rule's Attach left the world unstable: " +
          std::string(failure()));
    }
  }
  Refresh();
  rules_.push_back(&rules);
}

void MujocoPlant::SetMatchInput(const MatchInput& input) {
  match_input_ = input;
}

void MujocoPlant::Refresh() {
  const mjModel* m = model_.get();
  mjData* d = data_.get();
  // mj_step leaves the derived quantities at the start of its last step;
  // these recompute what the rotors and the chassis read from the new qpos
  // and qvel. The next mj_step recomputes them all before it uses them.
  mj_kinematics(m, d);
  mj_comPos(m, d);
  mj_tendon(m, d);
  mj_transmission(m, d);

  for (const Bound& entry : bound_) {
    const int out = entry.actuator->output;
    double velocity = 0.0;
    const int start = d->moment_rowadr[out];
    for (int k = 0; k < d->moment_rownnz[out]; ++k) {
      velocity +=
          d->actuator_moment[start + k] * d->qvel[d->moment_colind[start + k]];
    }
    hardware::sim::PlantRotor& rotor = rotors_[entry.motor];
    rotor.position_rot = entry.gear *
                         (d->actuator_length[out] - m->actuator_length0[out]) /
                         kTwoPi;
    rotor.velocity_rps = entry.gear * velocity / kTwoPi;
  }

  if (root_free_) {
    const int root = root_body_;
    mj_comVel(m, d);
    std::array<mjtNum, 6> velocity{};
    mj_objectVelocity(m, d, mjOBJ_XBODY, root, velocity.data(), 0);
    const double raw = Yaw(At(d->xquat, 4, root));
    chassis_.yaw_rad += std::remainder(raw - raw_yaw_rad_, kTwoPi);
    raw_yaw_rad_ = raw;
    const mjtNum* position = At(d->xpos, 3, root);
    chassis_.x_m = position[0];
    chassis_.y_m = position[1];
    chassis_.vx_mps = velocity[3];
    chassis_.vy_mps = velocity[4];
    chassis_.omega_radps = velocity[2];
  }
}

hardware::sim::PlantRotor MujocoPlant::rotor(std::size_t motor) const {
  return motor < rotors_.size() ? rotors_[motor] : hardware::sim::PlantRotor{};
}

std::optional<hardware::sim::PlantChassis> MujocoPlant::chassis() const {
  if (!root_free_) return std::nullopt;
  return chassis_;
}

bool MujocoPlant::input_closed(std::size_t input) const {
  if (input >= inputs_.size()) return false;
  const Input& entry = inputs_[input];
  const double value = data_->sensordata[entry.address];
  return entry.rangefinder ? value >= 0.0 && value < entry.cutoff : value > 0.0;
}

std::string_view MujocoPlant::failure() const {
  return failed_ ? std::string_view(failure_.data()) : std::string_view();
}

}  // namespace talos::simulation::mujoco

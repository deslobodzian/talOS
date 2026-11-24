#include "talOS/simulation/mujoco/scripted_shooter.h"

#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace talos::simulation::mujoco {

namespace {

constexpr std::string_view kInlet = "_inlet";
constexpr std::string_view kExit = "_exit";
constexpr std::string_view kNumeric = "_shooter";
// Launch spacing (README.md "Scripted shooter"): the clearance beyond two
// radii a launch keeps from the shooter's previous body in flight, and the
// longest a launch waits after that body's.
constexpr double kSpacingMarginM = 0.01;
constexpr double kMaxSpacingWaitS = 0.5;

// The `index`th group of `stride` entries of a MuJoCo array.
template <typename T>
T* At(T* array, int stride, int index) {
  return array + static_cast<std::ptrdiff_t>(stride) * index;
}

[[noreturn]] void Refuse(const std::string& message) {
  throw std::invalid_argument(message);
}

std::string NameOf(const mjModel& m, mjtObj type, int id) {
  const char* name = mj_id2name(&m, type, id);
  return name == nullptr ? std::string() : std::string(name);
}

bool EndsWith(std::string_view text, std::string_view suffix) {
  return text.size() > suffix.size() &&
         text.substr(text.size() - suffix.size()) == suffix;
}

// Whether `point` lies in the box site `site` (half-sizes in site_size),
// boundary included. xmat is row-major, local to world.
bool InsideBox(const mjModel& m, const mjData& d, int site,
               const mjtNum* point) {
  const mjtNum* origin = At(d.site_xpos, 3, site);
  const mjtNum* rotation = At(d.site_xmat, 9, site);
  const mjtNum* half = At(m.site_size, 3, site);
  const std::array<mjtNum, 3> delta = {
      point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]};
  for (int k = 0; k < 3; ++k) {
    const mjtNum local = rotation[k] * delta[0] + rotation[3 + k] * delta[1] +
                         rotation[6 + k] * delta[2];
    if (std::fabs(local) > half[k]) return false;
  }
  return true;
}

// The contract `motor` carries; inlet -1 when it carries none of it. Refuses
// a partial or malformed one.
ScriptedShooters::Shooter FindShooter(const mjModel& m, int actuator,
                                      const std::string& motor) {
  ScriptedShooters::Shooter shooter;
  const std::string inlet_name = motor + std::string(kInlet);
  const std::string exit_name = motor + std::string(kExit);
  const std::string numeric_name = motor + std::string(kNumeric);
  const int inlet = mj_name2id(&m, mjOBJ_SITE, inlet_name.c_str());
  const int exit = mj_name2id(&m, mjOBJ_SITE, exit_name.c_str());
  const int numeric = mj_name2id(&m, mjOBJ_NUMERIC, numeric_name.c_str());
  if (inlet < 0 && exit < 0 && numeric < 0) return shooter;

  const std::string where = "scripted shooter '" + motor + "': ";
  const std::string contract =
      "; a flywheel motor's scripted shooter is box site '" + inlet_name +
      "', site '" + exit_name + "' and numeric '" + numeric_name +
      "' (efficiency contact_radius_m)";
  std::string missing;
  const auto add = [&missing](const std::string& element) {
    missing += (missing.empty() ? "" : ", ") + element;
  };
  if (inlet < 0) add("site '" + inlet_name + "'");
  if (exit < 0) add("site '" + exit_name + "'");
  if (numeric < 0) add("numeric '" + numeric_name + "'");
  if (!missing.empty()) Refuse(where + "no " + missing + contract);

  if (m.numeric_size[numeric] != 2) {
    Refuse(where + "numeric '" + numeric_name + "' holds " +
           std::to_string(m.numeric_size[numeric]) +
           " value(s); it holds two, efficiency and contact_radius_m");
  }
  const mjtNum* values = m.numeric_data + m.numeric_adr[numeric];
  if (!(values[0] > 0.0 && values[0] <= 1.0)) {
    Refuse(where + "numeric '" + numeric_name + "' has efficiency " +
           std::to_string(values[0]) +
           "; it is the launched body's speed over the flywheel's surface "
           "speed, in (0, 1]");
  }
  if (!(values[1] > 0.0 && std::isfinite(values[1]))) {
    Refuse(where + "numeric '" + numeric_name + "' has contact radius " +
           std::to_string(values[1]) + " m; it is positive");
  }
  if (m.site_type[inlet] != mjGEOM_BOX) {
    Refuse(where + "site '" + inlet_name +
           "' is not a box; the inlet is a box site (type=\"box\")");
  }
  if (m.actuator_trntype[actuator] != mjTRN_JOINT) {
    Refuse(where + "actuator '" + motor +
           "' drives a tendon; a scripted shooter's flywheel is a hinge joint");
  }
  if (!(*At(m.actuator_gear, 6, actuator) > 0.0)) {
    Refuse(where + "actuator '" + motor + "' has gear " +
           std::to_string(*At(m.actuator_gear, 6, actuator)) +
           "; a scripted shooter's flywheel turns forward with its motor, so "
           "its gear is positive");
  }
  const int joint = *At(m.actuator_trnid, 2, actuator);
  if (m.jnt_type[joint] != mjJNT_HINGE) {
    Refuse(where + "actuator '" + motor + "' drives joint '" +
           NameOf(m, mjOBJ_JOINT, joint) +
           "', which is not a hinge; a scripted shooter's flywheel is a hinge "
           "joint");
  }
  const int root = m.body_rootid[m.jnt_bodyid[joint]];
  for (const int site : {inlet, exit}) {
    if (m.body_rootid[m.site_bodyid[site]] != root) {
      Refuse(where + "site '" + NameOf(m, mjOBJ_SITE, site) +
             "' is not on the robot that holds flywheel joint '" +
             NameOf(m, mjOBJ_JOINT, joint) + "'");
    }
  }
  shooter.motor = motor;
  shooter.inlet = inlet;
  shooter.exit = exit;
  shooter.dof = m.jnt_dofadr[joint];
  shooter.robot_root = root;
  shooter.efficiency = values[0];
  shooter.contact_radius_m = values[1];
  shooter.inertia_kg_m2 = m.dof_M0[shooter.dof];
  return shooter;
}

// Whether a body launched from `exit` at `velocity` would come within
// `clearance` of `leader`, launched earlier, before the leader falls back to
// the exit's height. Both move under gravity alone, which cancels in their
// difference: the gap is linear in time, its closest approach closed-form.
bool MeetsInFlight(const mjModel& m, const mjData& d, int leader_qpos,
                   int leader_dof, const mjtNum* exit,
                   const std::array<mjtNum, 3>& velocity, double clearance) {
  const mjtNum* at = d.qpos + leader_qpos;
  const mjtNum* moving = d.qvel + leader_dof;  // world frame
  std::array<mjtNum, 3> gap{};
  std::array<mjtNum, 3> closing{};
  for (int k = 0; k < 3; ++k) {
    gap[k] = exit[k] - at[k];
    closing[k] = velocity[k] - moving[k];
  }
  // How long the leader stays above the exit's height; without gravity it
  // never falls back.
  double horizon_s = mjMAXVAL;
  const double g = mju_norm3(m.opt.gravity);
  if (g > 0.0) {
    double height = 0.0;  // the leader's, over the exit, and its rise rate
    double rising = 0.0;
    for (int k = 0; k < 3; ++k) {
      height += gap[k] * m.opt.gravity[k] / g;
      rising -= moving[k] * m.opt.gravity[k] / g;
    }
    const double discriminant = rising * rising + 2.0 * g * height;
    if (discriminant < 0.0) return false;
    horizon_s = (rising + std::sqrt(discriminant)) / g;
    if (!(horizon_s > 0.0)) return false;
  }
  const double rate = mju_dot3(closing.data(), closing.data());
  double when_s = 0.0;
  if (rate > 0.0) {
    when_s = -mju_dot3(gap.data(), closing.data()) / rate;
    when_s = when_s < 0.0 ? 0.0 : (when_s > horizon_s ? horizon_s : when_s);
  }
  std::array<mjtNum, 3> nearest{};
  for (int k = 0; k < 3; ++k) nearest[k] = gap[k] + closing[k] * when_s;
  return mju_dot3(nearest.data(), nearest.data()) < clearance * clearance;
}

}  // namespace

ScriptedShooters::ScriptedShooters(std::vector<Shooter> shooters)
    : shooters_(std::move(shooters)), last_(shooters_.size()) {}

void ScriptedShooters::Attach(const mjModel& model, mjData& data) {
  bodies_.clear();
  for (int b = 1; b < model.nbody; ++b) {
    if (model.body_jntnum[b] < 1 ||
        model.jnt_type[model.body_jntadr[b]] != mjJNT_FREE) {
      continue;
    }
    const int joint = model.body_jntadr[b];
    Body body;
    body.body = b;
    body.qpos = model.jnt_qposadr[joint];
    body.dof = model.jnt_dofadr[joint];
    body.root = model.body_rootid[b];
    body.mass_kg = model.body_mass[b];
    body.radius_m = model.body_geomnum[b] > 0
                        ? model.geom_rbound[model.body_geomadr[b]]
                        : 0.0;
    const mjtNum* inertia = At(model.body_inertia, 3, b);
    const double mean = (inertia[0] + inertia[1] + inertia[2]) / 3.0;
    body.beta = body.mass_kg > 0.0 && body.radius_m > 0.0
                    ? mean / (body.mass_kg * body.radius_m * body.radius_m)
                    : 0.0;
    bodies_.push_back(body);
  }
  launched_step_.assign(bodies_.size(), 0);
  last_.assign(shooters_.size(), Last{});
  for (const Shooter& shooter : shooters_) {
    if (InsideBox(model, data, shooter.inlet,
                  At(data.site_xpos, 3, shooter.exit))) {
      Refuse("scripted shooter '" + shooter.motor + "': site '" +
             shooter.motor + std::string(kExit) + "' lies inside site '" +
             shooter.motor + std::string(kInlet) +
             "', so every launch would trigger again; place the exit outside "
             "the inlet");
    }
  }
}

bool ScriptedShooters::AfterStep(mjModel& model, mjData& data, uint64_t step,
                                 const MatchInput& /*input*/) {
  for (std::size_t s = 0; s < shooters_.size(); ++s) {
    const Shooter& shooter = shooters_[s];
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
      const Body& body = bodies_[i];
      // A body launched this step (by any shooter) waits for the next.
      if (body.root == shooter.robot_root || launched_step_[i] == step) {
        continue;
      }
      if (!InsideBox(model, data, shooter.inlet, data.qpos + body.qpos)) {
        continue;
      }
      const double since_s =
          last_[s].body < 0
              ? kMaxSpacingWaitS
              : static_cast<double>(step - last_[s].step) * model.opt.timestep;
      if (Launch(model, data, shooter, body,
                 since_s < kMaxSpacingWaitS ? last_[s].body : -1)) {
        launched_step_[i] = step;
        last_[s] = Last{static_cast<int>(i), step};
        ++shots_;
      }
    }
  }
  return true;
}

bool ScriptedShooters::Launch(const mjModel& model, mjData& data,
                              const Shooter& shooter, const Body& body,
                              int leader) {
  // A flywheel at rest or turning backward launches nothing: the body stays
  // where the contacts hold it, and goes when the flywheel turns forward.
  // Forward is the joint's positive direction, the motor's too: an actuator
  // with a gear that is not positive is refused (MakeScriptedShooters).
  const double speed_radps = data.qvel[shooter.dof];
  if (!(speed_radps > 0.0)) return false;
  // One body at a time: it waits while another's centre is nearer the exit
  // than their two radii (the launched one has not cleared it yet).
  const mjtNum* exit = At(data.site_xpos, 3, shooter.exit);
  for (const Body& other : bodies_) {
    if (other.body == body.body || other.root == shooter.robot_root) continue;
    const mjtNum* centre = data.qpos + other.qpos;
    const double clear = body.radius_m + other.radius_m;
    const double dx = centre[0] - exit[0];
    const double dy = centre[1] - exit[1];
    const double dz = centre[2] - exit[2];
    if (dx * dx + dy * dy + dz * dz < clear * clear) return false;
  }

  // The body rolls between the flywheel and a fixed hood: the flywheel's
  // share of the impulse is (1 + beta)/2 m v, at the contact radius, so the
  // flywheel loses k v, k = (1 + beta)/2 m r_c / J. The body leaves at the
  // speed the flywheel is left with, v = eta r_c (w - k v):
  //   v = eta r_c w / (1 + eta r_c k).
  // The momentum the launch carries is what the flywheel loses, the flywheel
  // never reverses, and the body's energy, linear and rolling, is at most
  // eta times what the flywheel loses.
  const double eta_r = shooter.efficiency * shooter.contact_radius_m;
  const double k = (1.0 + body.beta) / 2.0 * body.mass_kg *
                   shooter.contact_radius_m / shooter.inertia_kg_m2;
  const double launch_mps = eta_r * speed_radps / (1.0 + eta_r * k);
  const mjtNum* frame = At(data.site_xmat, 9, shooter.exit);
  const std::array<mjtNum, 3> x_axis = {frame[0], frame[3], frame[6]};
  const std::array<mjtNum, 3> y_axis = {frame[1], frame[4], frame[7]};
  // The exit's own velocity, world frame: [angular, linear] at the site.
  std::array<mjtNum, 6> exit_velocity{};
  mj_objectVelocity(&model, &data, mjOBJ_SITE, shooter.exit,
                    exit_velocity.data(), 0);

  std::array<mjtNum, 3> leaves{};
  for (int k = 0; k < 3; ++k) {
    leaves[k] = exit_velocity[3 + k] + launch_mps * x_axis[k];
  }
  // Spaced from the shooter's previous launch: it waits while the two would
  // meet in flight (on one arc, near the apex, where the first moves at only
  // v·cos(theta)).
  if (leader >= 0) {
    const Body& first = bodies_[static_cast<std::size_t>(leader)];
    if (MeetsInFlight(model, data, first.qpos, first.dof, exit, leaves,
                      first.radius_m + body.radius_m + kSpacingMarginM)) {
      return false;
    }
  }

  mjtNum* position = data.qpos + body.qpos;
  mjtNum* velocity = data.qvel + body.dof;
  for (int k = 0; k < 3; ++k) {
    position[k] = exit[k];
    velocity[k] = leaves[k];
  }
  // Rolling spin about the exit's y axis on top of the exit's own turn; a
  // free joint's angular velocity is in the body's frame.
  const double spin_radps =
      body.radius_m > 0.0 ? launch_mps / body.radius_m : 0.0;
  std::array<mjtNum, 3> spin_world{};
  for (int k = 0; k < 3; ++k) {
    spin_world[k] = exit_velocity[k] + spin_radps * y_axis[k];
  }
  std::array<mjtNum, 4> inverse{};
  mju_negQuat(inverse.data(), position + 3);
  mju_rotVecQuat(velocity + 3, spin_world.data(), inverse.data());

  data.qvel[shooter.dof] = speed_radps - k * launch_mps;
  return true;
}

std::unique_ptr<WorldRules> MakeScriptedShooters(const mjModel& model) {
  std::vector<ScriptedShooters::Shooter> shooters;
  for (int a = 0; a < model.nu; ++a) {
    const std::string motor = NameOf(model, mjOBJ_ACTUATOR, a);
    if (motor.empty()) continue;
    ScriptedShooters::Shooter shooter = FindShooter(model, a, motor);
    if (shooter.inlet >= 0) shooters.push_back(std::move(shooter));
  }
  // An inlet or a numeric whose flywheel motor is no actuator.
  const auto orphan = [&model](mjtObj type, int count, std::string_view suffix,
                               std::string_view what) {
    for (int id = 0; id < count; ++id) {
      const std::string name = NameOf(model, type, id);
      if (!EndsWith(name, suffix)) continue;
      const std::string motor = name.substr(0, name.size() - suffix.size());
      if (mj_name2id(&model, mjOBJ_ACTUATOR, motor.c_str()) < 0) {
        Refuse("scripted shooter '" + motor + "': " + std::string(what) + " '" +
               name + "' names flywheel motor '" + motor +
               "', which is no actuator; the contract's motor is a <motor> "
               "actuator of that name");
      }
    }
  };
  orphan(mjOBJ_SITE, static_cast<int>(model.nsite), kInlet, "site");
  orphan(mjOBJ_NUMERIC, static_cast<int>(model.nnumeric), kNumeric, "numeric");
  if (shooters.empty()) return nullptr;
  return std::make_unique<ScriptedShooters>(std::move(shooters));
}

}  // namespace talos::simulation::mujoco

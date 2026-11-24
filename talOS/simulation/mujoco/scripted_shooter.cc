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

bool IsFinite(const mjtNum* values, int count) {
  for (int k = 0; k < count; ++k) {
    if (!std::isfinite(values[k])) return false;
  }
  return true;
}

// Whether `point` lies in the box site `site` (half-sizes in site_size),
// boundary included. xmat is row-major, local to world. Returns false when
// any coordinate or site parameter is non-finite.
bool InsideBox(const mjModel& m, const mjData& d, int site,
               const mjtNum* point) {
  const mjtNum* origin = At(d.site_xpos, 3, site);
  const mjtNum* rotation = At(d.site_xmat, 9, site);
  const mjtNum* half = At(m.site_size, 3, site);
  if (!IsFinite(point, 3) || !IsFinite(origin, 3) || !IsFinite(rotation, 9) ||
      !IsFinite(half, 3)) {
    return false;
  }
  const std::array<mjtNum, 3> delta = {
      point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]};
  for (int k = 0; k < 3; ++k) {
    const mjtNum local = rotation[k] * delta[0] + rotation[3 + k] * delta[1] +
                         rotation[6 + k] * delta[2];
    if (!(std::fabs(local) <= half[k])) return false;
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
    Refuse(where + "actuator '" + motor + "'" +
           (m.actuator_trntype[actuator] == mjTRN_TENDON
                ? " drives a tendon"
                : " does not drive a joint") +
           "; a scripted shooter's flywheel is a hinge joint");
  }
  const double gear = *At(m.actuator_gear, 6, actuator);
  if (!(gear > 0.0 && std::isfinite(gear))) {
    Refuse(where + "actuator '" + motor + "' has gear " + std::to_string(gear) +
           "; a scripted shooter's flywheel turns forward with its motor, so "
           "its gear is positive and finite");
  }
  const int joint = *At(m.actuator_trnid, 2, actuator);
  if (joint < 0 || joint >= m.njnt || m.jnt_type[joint] != mjJNT_HINGE) {
    Refuse(where + "actuator '" + motor + "' drives joint '" +
           (joint >= 0 && joint < m.njnt ? NameOf(m, mjOBJ_JOINT, joint)
                                         : std::to_string(joint)) +
           "', which is not a hinge; a scripted shooter's flywheel is a hinge "
           "joint");
  }
  const int dof = m.jnt_dofadr[joint];
  if (dof < 0 || dof >= m.nv) {
    Refuse(where + "flywheel joint '" + NameOf(m, mjOBJ_JOINT, joint) +
           "' has invalid dof address " + std::to_string(dof));
  }
  const double inertia = m.dof_M0[dof];
  if (!(inertia > 0.0 && std::isfinite(inertia))) {
    Refuse(where + "flywheel joint '" + NameOf(m, mjOBJ_JOINT, joint) +
           "' has inertia " + std::to_string(inertia) +
           " kg*m^2; it is positive and finite");
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
  shooter.dof = dof;
  shooter.robot_root = root;
  shooter.efficiency = values[0];
  shooter.contact_radius_m = values[1];
  shooter.inertia_kg_m2 = inertia;
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
  if (!IsFinite(at, 3) || !IsFinite(moving, 3) || !IsFinite(exit, 3) ||
      !IsFinite(velocity.data(), 3) ||
      !(clearance > 0.0 && std::isfinite(clearance))) {
    return false;
  }
  std::array<mjtNum, 3> gap{};
  std::array<mjtNum, 3> closing{};
  for (int k = 0; k < 3; ++k) {
    gap[k] = exit[k] - at[k];
    closing[k] = velocity[k] - moving[k];
  }
  if (!IsFinite(gap.data(), 3) || !IsFinite(closing.data(), 3)) return false;
  // How long the leader stays above the exit's height; without gravity it
  // never falls back.
  double horizon_s = mjMAXVAL;
  if (IsFinite(m.opt.gravity, 3)) {
    const double g = mju_norm3(m.opt.gravity);
    if (g > 0.0 && std::isfinite(g)) {
      double height = 0.0;  // the leader's, over the exit, and its rise rate
      double rising = 0.0;
      for (int k = 0; k < 3; ++k) {
        height += gap[k] * m.opt.gravity[k] / g;
        rising -= moving[k] * m.opt.gravity[k] / g;
      }
      const double discriminant = rising * rising + 2.0 * g * height;
      if (!(discriminant >= 0.0)) return false;
      horizon_s = (rising + std::sqrt(discriminant)) / g;
      if (!(horizon_s > 0.0)) return false;
      if (!std::isfinite(horizon_s)) horizon_s = mjMAXVAL;
    }
  }
  const double rate = mju_dot3(closing.data(), closing.data());
  double when_s = 0.0;
  if (rate > 0.0 && std::isfinite(rate)) {
    const double proj = -mju_dot3(gap.data(), closing.data());
    if (std::isfinite(proj)) {
      when_s = proj / rate;
      if (!std::isfinite(when_s) || when_s < 0.0) {
        when_s = 0.0;
      } else if (when_s > horizon_s) {
        when_s = horizon_s;
      }
    }
  }
  std::array<mjtNum, 3> nearest{};
  for (int k = 0; k < 3; ++k) nearest[k] = gap[k] + closing[k] * when_s;
  const double dist2 = mju_dot3(nearest.data(), nearest.data());
  return std::isfinite(dist2) && dist2 < clearance * clearance;
}

}  // namespace

ScriptedShooters::ScriptedShooters(std::vector<Shooter> shooters)
    : shooters_(std::move(shooters)), last_(shooters_.size()) {}

void ScriptedShooters::Attach(const mjModel& model, mjData& data) {
  for (const Shooter& shooter : shooters_) {
    const std::string where = "scripted shooter '" + shooter.motor + "': ";
    if (shooter.inlet < 0 || shooter.inlet >= model.nsite) {
      Refuse(where + "inlet site id " + std::to_string(shooter.inlet) +
             " is out of range for model with " + std::to_string(model.nsite) +
             " sites");
    }
    if (model.site_type[shooter.inlet] != mjGEOM_BOX) {
      Refuse(where + "inlet site '" + NameOf(model, mjOBJ_SITE, shooter.inlet) +
             "' is not a box");
    }
    if (shooter.exit < 0 || shooter.exit >= model.nsite) {
      Refuse(where + "exit site id " + std::to_string(shooter.exit) +
             " is out of range for model with " + std::to_string(model.nsite) +
             " sites");
    }
    if (shooter.dof < 0 || shooter.dof >= model.nv) {
      Refuse(where + "flywheel dof " + std::to_string(shooter.dof) +
             " is out of range for model with " + std::to_string(model.nv) +
             " dofs");
    }
    if (shooter.robot_root < 0 || shooter.robot_root >= model.nbody) {
      Refuse(where + "robot root body " + std::to_string(shooter.robot_root) +
             " is out of range for model with " + std::to_string(model.nbody) +
             " bodies");
    }
    if (!(shooter.efficiency > 0.0 && shooter.efficiency <= 1.0)) {
      Refuse(where + "efficiency " + std::to_string(shooter.efficiency) +
             " must be in (0, 1]");
    }
    if (!(shooter.contact_radius_m > 0.0 &&
          std::isfinite(shooter.contact_radius_m))) {
      Refuse(where + "contact radius " +
             std::to_string(shooter.contact_radius_m) +
             " m must be positive and finite");
    }
    if (!(shooter.inertia_kg_m2 > 0.0 &&
          std::isfinite(shooter.inertia_kg_m2))) {
      Refuse(where + "inertia " + std::to_string(shooter.inertia_kg_m2) +
             " kg*m^2 must be positive and finite");
    }
    const mjtNum* exit_pos = At(data.site_xpos, 3, shooter.exit);
    if (!IsFinite(At(data.site_xpos, 3, shooter.inlet), 3) ||
        !IsFinite(exit_pos, 3)) {
      Refuse(where + "inlet or exit site pose is non-finite");
    }
    if (InsideBox(model, data, shooter.inlet, exit_pos)) {
      Refuse(where + "site '" + shooter.motor + std::string(kExit) +
             "' lies inside site '" + shooter.motor + std::string(kInlet) +
             "', so every launch would trigger again; place the exit outside "
             "the inlet");
    }
  }

  std::vector<Body> bodies;
  for (int b = 1; b < model.nbody; ++b) {
    if (model.body_jntnum[b] < 1 ||
        model.jnt_type[model.body_jntadr[b]] != mjJNT_FREE) {
      continue;
    }
    const int joint = model.body_jntadr[b];
    const int qpos = model.jnt_qposadr[joint];
    const int dof = model.jnt_dofadr[joint];
    if (qpos < 0 || static_cast<std::int64_t>(qpos) + 7 > model.nq || dof < 0 ||
        static_cast<std::int64_t>(dof) + 6 > model.nv) {
      continue;
    }
    Body body;
    body.body = b;
    body.qpos = qpos;
    body.dof = dof;
    body.root = model.body_rootid[b];
    const double mass = model.body_mass[b];
    body.mass_kg = (mass > 0.0 && std::isfinite(mass)) ? mass : 0.0;
    double radius = 0.0;
    if (model.body_geomnum[b] > 0) {
      const int geom = model.body_geomadr[b];
      if (geom >= 0 && geom < model.ngeom) {
        const double r = model.geom_rbound[geom];
        if (r > 0.0 && std::isfinite(r)) radius = r;
      }
    }
    body.radius_m = radius;
    const mjtNum* inertia = At(model.body_inertia, 3, b);
    const double mean = (inertia[0] + inertia[1] + inertia[2]) / 3.0;
    const double mr2 = body.mass_kg * body.radius_m * body.radius_m;
    const double beta =
        (std::isfinite(mean) && mean > 0.0 && std::isfinite(mr2) && mr2 > 0.0)
            ? mean / mr2
            : 0.0;
    body.beta = std::isfinite(beta) ? beta : 0.0;
    bodies.push_back(body);
  }
  bodies_ = std::move(bodies);
  launched_.assign(bodies_.size(), LaunchMark{});
  last_.assign(shooters_.size(), Last{});
  shots_ = 0;
}

bool ScriptedShooters::AfterStep(mjModel& model, mjData& data, uint64_t step,
                                 const MatchInput& /*input*/) {
  const bool valid_dt =
      model.opt.timestep > 0.0 && std::isfinite(model.opt.timestep);
  for (std::size_t s = 0; s < shooters_.size(); ++s) {
    const Shooter& shooter = shooters_[s];
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
      const Body& body = bodies_[i];
      // A body launched this step (by any shooter) waits for the next.
      if (body.root == shooter.robot_root ||
          (launched_[i].launched && launched_[i].step == step)) {
        continue;
      }
      if (!InsideBox(model, data, shooter.inlet, data.qpos + body.qpos)) {
        continue;
      }
      int leader = -1;
      if (valid_dt && last_[s].body >= 0 &&
          last_[s].body != static_cast<int>(i) && step >= last_[s].step) {
        const double since_s =
            static_cast<double>(step - last_[s].step) * model.opt.timestep;
        if (since_s < kMaxSpacingWaitS) leader = last_[s].body;
      }
      if (Launch(model, data, shooter, body, leader)) {
        launched_[i] = LaunchMark{true, step};
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
  if (!(speed_radps > 0.0 && std::isfinite(speed_radps))) return false;
  const mjtNum* exit = At(data.site_xpos, 3, shooter.exit);
  const mjtNum* frame = At(data.site_xmat, 9, shooter.exit);
  const mjtNum* body_pos = data.qpos + body.qpos;
  if (!IsFinite(exit, 3) || !IsFinite(frame, 9) || !IsFinite(body_pos + 3, 4)) {
    return false;
  }
  // One body at a time: it waits while another's centre is nearer the exit
  // than their two radii (the launched one has not cleared it yet).
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
  //   v = eta r_c w / (1 + eta r_c k),  w' = w / (1 + eta r_c k).
  // Writing w' = w / (1 + eta r_c k) avoids inf * 0 = NaN if k overflows and
  // guarantees w' >= 0. The momentum the launch carries is what the flywheel
  // loses, the flywheel never reverses, and the body's energy, linear and
  // rolling, is at most eta times what the flywheel loses.
  const double eta_r = shooter.efficiency * shooter.contact_radius_m;
  const double k = (1.0 + body.beta) / 2.0 * body.mass_kg *
                   shooter.contact_radius_m / shooter.inertia_kg_m2;
  const double denom = 1.0 + eta_r * k;
  if (!(denom >= 1.0)) return false;
  const double launch_mps = eta_r * speed_radps / denom;
  const double speed_after = speed_radps / denom;
  if (!std::isfinite(launch_mps) || !std::isfinite(speed_after)) return false;

  const std::array<mjtNum, 3> x_axis = {frame[0], frame[3], frame[6]};
  const std::array<mjtNum, 3> y_axis = {frame[1], frame[4], frame[7]};
  // The exit's own velocity, world frame: [angular, linear] at the site.
  std::array<mjtNum, 6> exit_velocity{};
  mj_objectVelocity(&model, &data, mjOBJ_SITE, shooter.exit,
                    exit_velocity.data(), 0);
  if (!IsFinite(exit_velocity.data(), 6)) return false;

  std::array<mjtNum, 3> leaves{};
  for (int axis = 0; axis < 3; ++axis) {
    leaves[axis] = exit_velocity[3 + axis] + launch_mps * x_axis[axis];
  }
  // Rolling spin about the exit's y axis on top of the exit's own turn; a
  // free joint's angular velocity is in the body's frame.
  const double spin_radps =
      body.radius_m > 0.0 ? launch_mps / body.radius_m : 0.0;
  if (!std::isfinite(spin_radps)) return false;
  std::array<mjtNum, 3> spin_world{};
  for (int axis = 0; axis < 3; ++axis) {
    spin_world[axis] = exit_velocity[axis] + spin_radps * y_axis[axis];
  }
  if (!IsFinite(leaves.data(), 3) || !IsFinite(spin_world.data(), 3)) {
    return false;
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
  for (int axis = 0; axis < 3; ++axis) {
    position[axis] = exit[axis];
    velocity[axis] = leaves[axis];
  }
  std::array<mjtNum, 4> inverse{};
  mju_negQuat(inverse.data(), position + 3);
  mju_rotVecQuat(velocity + 3, spin_world.data(), inverse.data());

  data.qvel[shooter.dof] = speed_after;
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

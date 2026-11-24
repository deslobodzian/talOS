#pragma once

// WorldRules: what changes the MuJoCo world after a step besides physics (a
// robot's scripted shot, a game's referee), run by MujocoPlant::Step after
// every MuJoCo step (MujocoPlant::AddRules). A rule is a pure function of the
// model, the data, its own fixed-size state, the step count and the match
// input, so the plant's bit identity (REQ-DEV-37) covers what it does.

#include <cstdint>
#include <string_view>

// MuJoCo's model and data (mujoco.h typedefs them as mjModel and mjData).
struct mjModel_;
struct mjData_;

namespace talos::simulation::mujoco {

// The match as the gateway's Driver Station and schedule show it this tick.
struct MatchInput {
  enum class Mode : uint8_t { kNone, kAutonomous, kTeleop, kTest };
  Mode mode{Mode::kNone};
  bool enabled{false};       // the gateway's enable (enabled, not e-stopped)
  bool red_alliance{false};  // fms.alliance
  bool scheduled{false};     // false until the schedules' time zero is reached
  uint64_t schedule_us{0};   // since that time zero
};

// Runs inside MujocoPlant::Step after every MuJoCo step that raised no fatal
// warning. May write qpos/qvel of free joints, qvel of joints it names, and
// geom_contype/conaffinity of geoms its README names; nothing else. Reads the
// step's kinematics. No clock, no allocation after Attach, no thread.
//
// What is fresh when AfterStep runs: qpos, qvel and time after the step, and
// the plant has brought the position and velocity kinematics up to them
// (mj_kinematics, mj_comPos, mj_comVel): body, geom and site poses (xpos,
// xquat, xmat, geom_xpos, site_xpos, site_xmat), subtree_com, cdof and cvel,
// so mj_objectVelocity of a body or site is the velocity at the end of the
// step. Everything else (contacts, sensors, forces, the mass matrix, tendon
// lengths) is as of the step's start. Every rule of a step sees the same
// kinematics: what a rule writes to qpos or qvel is not reflected in them
// (read the written qpos or qvel itself), and the plant recomputes what it
// reports after the step's last rule, as the next step does before it uses
// anything.
class WorldRules {
 public:
  virtual ~WorldRules() = default;
  // Once, after compile and before the first step; may place bodies.
  // Throws std::invalid_argument naming a missing site, numeric or body.
  // The data is mj_forward's at the plant's current qpos and qvel.
  virtual void Attach(const mjModel_& model, mjData_& data) = 0;
  // Each time the plant is given a match input (MujocoPlant::SetMatchInput),
  // before the steps that use it and whether or not any follow: what a rule
  // reports that depends on the input alone (a match clock, its period)
  // follows it here, so it is current on a tick whose plant time rounds to
  // no step. It touches no model or data, and the steps then see the same
  // input in AfterStep. No clock, no allocation. False, or a throw: the
  // plant fails with failure().
  virtual bool ObserveMatch(const MatchInput& /*input*/) { return true; }
  // `step` counts plant steps from 1. False: the plant fails with failure().
  virtual bool AfterStep(mjModel_& model, mjData_& data, uint64_t step,
                         const MatchInput& input) = 0;
  virtual std::string_view failure() const = 0;
};

}  // namespace talos::simulation::mujoco

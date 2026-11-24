#pragma once

// The scripted shooter (REQ-DEV-39): a robot model's flywheel launches the
// free bodies fed to it by rule, not by contact, which at 5 ms is not
// predictive. Robot-generic: it runs on every field and on the flat carpet.
//
// The contract a robot model carries, per flywheel motor `<motor>` (the name
// of a <motor> actuator, which drives a hinge):
//   - box site `<motor>_inlet`: the trigger volume;
//   - site `<motor>_exit`: where a launched body's centre reappears, its +x
//     axis the launch direction, its +y axis the rolling spin's;
//   - numeric `<motor>_shooter`, data "efficiency contact_radius_m".
// A body leaves the exit at the exit's own velocity plus, along its +x,
// v = eta r_c w / (1 + eta r_c k), efficiency × contact radius × the speed
// the launch leaves the flywheel with, where w is the flywheel's speed before
// and k = (1 + beta)/2 m r_c / J; the flywheel loses k v. README.md
// "Scripted shooter" says the rest of what a launch does and when a body
// waits: among others, while it would meet the shooter's previous launch in
// flight, for at most 0.5 s after that launch.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/simulation/mujoco/world_rules.h"

namespace talos::simulation::mujoco {

class ScriptedShooters final : public WorldRules {
 public:
  // One flywheel's contract, as MakeScriptedShooters found it.
  struct Shooter {
    std::string motor;
    int inlet{-1};  // site ids
    int exit{-1};
    int dof{-1};         // the flywheel hinge's dof
    int robot_root{-1};  // the root body of the flywheel's tree
    double efficiency{};
    double contact_radius_m{};
    // The flywheel dof's inertia at the reference pose, armature included
    // (MuJoCo's dof_M0: its subtree about the axis, the joint's armature and
    // its actuator's armature times gear squared).
    double inertia_kg_m2{};
  };

  explicit ScriptedShooters(std::vector<Shooter> shooters);

  // Sizes the per-body state; refuses an exit inside its own inlet at the
  // current pose (every launch would launch again).
  void Attach(const mjModel_& model, mjData_& data) override;
  bool AfterStep(mjModel_& model, mjData_& data, uint64_t step,
                 const MatchInput& input) override;
  // Never fails.
  std::string_view failure() const override { return {}; }

  std::span<const Shooter> shooters() const { return shooters_; }
  // Launches so far, every shooter's.
  uint64_t shots() const { return shots_; }

 private:
  // A free body the shooters may launch, at Attach.
  struct Body {
    int body{-1};
    int qpos{-1};  // its free joint's qpos and dof addresses
    int dof{-1};
    int root{-1};  // body_rootid: itself
    double mass_kg{};
    double radius_m{};  // its first geom's bounding radius; 0 with no geom
    double beta{};      // I/(m r^2), I the mean principal inertia
  };

  // The last body a shooter launched (its bodies_ index; -1: none yet) and
  // the step it launched in.
  struct Last {
    int body{-1};
    uint64_t step{};
  };

  // Launches `body` from `shooter` unless the flywheel is not turning forward,
  // another body is still at the exit, or it would meet `leader` (a bodies_
  // index; -1: none to space from) in flight. True when it launched.
  bool Launch(const mjModel_& model, mjData_& data, const Shooter& shooter,
              const Body& body, int leader);

  std::vector<Shooter> shooters_;
  std::vector<Body> bodies_;
  std::vector<uint64_t> launched_step_;  // by bodies_ entry; 0: never
  std::vector<Last> last_;               // by shooters_ entry
  uint64_t shots_{};
};

// The scripted shooters of `model` in actuator order, or null when no
// actuator carries any part of the contract. Throws std::invalid_argument,
// naming the motor and the element, for a partial or malformed contract: a
// missing site or numeric, a numeric without two values, an efficiency
// outside (0, 1] or a contact radius that is not positive, an inlet that is
// not a box, a flywheel that is not a hinge (a slide or a tendon) or whose
// actuator's gear is not positive, a site off
// the flywheel's robot tree, and a `*_inlet` site or `*_shooter` numeric
// whose motor no actuator is.
std::unique_ptr<WorldRules> MakeScriptedShooters(const mjModel_& model);

}  // namespace talos::simulation::mujoco

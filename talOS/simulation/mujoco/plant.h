#pragma once

// MujocoPlant: a hardware::sim::ExternalPlant whose world is a MuJoCo model,
// the robot MJCF attached to a field MJCF (or to a flat carpet), stepped at
// the option table's 5 ms (REQ-DEV-37). PhysicsSimBackend stays the only
// controller and electrical model: each Step writes, per bound motor, the
// winding's torque law into its actuator, and MuJoCo applies it at every one
// of its steps at that step's rotor speed.
//
// Its Bazel visibility keeps it inside this package tree, so MuJoCo stays
// behind it; tools/check_layers.py --source-only refuses the @mujoco label
// outside the tree.

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/hardware/sim_plant.h"
#include "talOS/simulation/mujoco/world_rules.h"

// MuJoCo's model and data (mujoco.h typedefs them as mjModel and mjData).
struct mjModel_;
struct mjData_;

namespace talos::simulation::mujoco {

// MuJoCo's log handling is process-wide (3.17.0 engine_util_errmem.c): by
// default it appends every warning to MUJOCO_LOG.TXT in the working directory
// and an engine error (an arena overflow inside mj_step, say) exits the
// process without saying which program. A program that steps a MujocoPlant
// calls this once, before MakePlant: MuJoCo's log goes to stderr only; an
// engine error prints a line naming the MuJoCo plant and MuJoCo's message and
// exits 1; and the warning every MakePlant raises by design ("Attaching a spec
// with an empty prefix") is dropped, while every other warning still prints.
void InstallMujocoProcessHandlers();

struct PlantModel {
  std::string field_xml;  // field MJCF path; empty: flat carpet built in code
  std::string robot_xml;  // robot MJCF path
  double start_x_m{}, start_y_m{}, start_yaw_rad{};  // z from the model
  // MuJoCo's sleeping (mjENBL_SLEEP, MuJoCo's default tolerance): a tree
  // whose every velocity stays under it for 10 steps sleeps, and is not
  // integrated or collided with other sleepers until a contact with an awake
  // tree, an applied force or a written qpos or qvel wakes it. Never a
  // robot tree: MakePlant sets each to mjSLEEP_NEVER.
  bool sleeping{false};
};

class MujocoPlant final : public hardware::sim::ExternalPlant {
 public:
  struct ModelDeleter {
    void operator()(mjModel_* model) const;
  };
  struct DataDeleter {
    void operator()(mjData_* data) const;
  };

  ~MujocoPlant() override;
  MujocoPlant(const MujocoPlant&) = delete;
  MujocoPlant& operator=(const MujocoPlant&) = delete;
  MujocoPlant(MujocoPlant&&) = delete;
  MujocoPlant& operator=(MujocoPlant&&) = delete;

  uint32_t step_us() const override { return step_us_; }
  bool Bind(std::span<const hardware::sim::PlantMotor> motors,
            std::span<const hardware::sim::PlantInput> inputs,
            std::span<bool> owned, hardware::ConfigError& error) override;
  bool Step(int steps,
            std::span<const hardware::sim::PlantDrive> drives) override;
  hardware::sim::PlantRotor rotor(std::size_t motor) const override;
  std::optional<hardware::sim::PlantChassis> chassis() const override;
  // Reads the world as of the start of the last MuJoCo step (MuJoCo computes
  // sensors before it integrates), one step behind rotor() and chassis().
  bool input_closed(std::size_t input) const override;
  std::string_view failure() const override;

  // A bound motor of the last successful Bind, with its inertias at the
  // model's reference pose: J_act = 1/(moment·M⁻¹·moment) in the actuator's
  // coordinate (mechanism radians), armature included, and J_rotor =
  // J_act/G². J_act is the free-flight value there, with no contacts, so a
  // free chassis's own mobility lowers it for a mechanism mounted on it. Both
  // are NaN, and `light` false, when the actuator has no moment at the
  // reference pose. `light` when J_rotor < h·b_em/2: its transients are wrong
  // until the actuator's `armature` carries G²·J_rotor of the real rotor.
  struct Binding {
    std::size_t motor{};
    double inertia_kg_m2{};
    double rotor_inertia_kg_m2{};
    bool light{false};
  };
  std::span<const Binding> bindings() const { return bindings_; }
  // The same as text, one line per motor (its actuator, joint or tendon, G,
  // J_act and J_rotor) and a warning line for each light one. For the gateway
  // to print.
  const std::string& binding_table() const { return binding_table_; }
  // How many times a bound rotor's speed rose above Gateway::kMaxRotorRps
  // (one per excursion, not per Step): logged by the caller, not fatal (Q9).
  uint64_t overspeed_excursions() const { return overspeed_excursions_; }
  // The compiled world and its state, for a truth publisher; the plant owns
  // both and changes them only in Bind, AddRules (a rule's Attach) and Step
  // (Step rewrites the bound actuators' gain, bias and force range, and its
  // rules write what world_rules.h allows).
  const mjModel_* model() const { return model_.get(); }
  const mjData_* data() const { return data_.get(); }
  // The robot model's root body in model(): every robot body is in its
  // subtree (body_rootid), and no other body is.
  int robot_root_body() const { return root_body_; }

  // World rules (world_rules.h). Before the first Step; not owned; run in the
  // order added. Calls Attach, then mj_forward and mj_step's position,
  // velocity and acceleration checks, so the next rule's Attach and every
  // reader see what it placed; a fatal warning there fails the plant and
  // throws std::invalid_argument naming it. Throws std::invalid_argument for
  // a rule already added, and std::logic_error after the first Step or on a
  // failed plant. While rules are attached, each MuJoCo step that passes the
  // fatal-warning check is followed by mj_kinematics, mj_comPos and
  // mj_comVel, then every rule's AfterStep. AfterStep must not throw: a rule
  // that returns false, or throws anyway, fails the plant for good with its
  // failure() text (or the exception's), and no rule runs again. Such a rule
  // may have written part of what it meant to (some bodies placed, others
  // not); the failure is permanent, so nothing reads that world as valid.
  void AddRules(WorldRules& rules);
  // The match input every rule sees in every Step until the next call. Each
  // rule first observes it here (WorldRules::ObserveMatch), in the order
  // added, whether or not a step follows; a rule that returns false or
  // throws fails the plant for good, as in AfterStep. On a failed plant it
  // only holds the input.
  void SetMatchInput(const MatchInput& input);

 private:
  friend std::unique_ptr<MujocoPlant> MakePlant(
      const PlantModel& model,
      std::span<const hardware::sim::PlantMotor> motors);

  // One model actuator, all of which name a motor (MakePlant refuses one
  // that does not).
  struct Actuator {
    std::string motor;  // its name, the actuator's
    int id{-1};
    int ctrl{-1};        // d->ctrl index
    int output{-1};      // actuator_length / moment row
    std::string target;  // joint or tendon name, for the binding table
    // From MakePlant's motors: compiled into the actuator's damping, G²·b_m.
    double gear{1.0};
    double no_load_damping_nm_per_radps{};
    double inertia_kg_m2{};  // J_act, from MakePlant
  };
  // A motor of the last Bind that an actuator binds.
  struct Bound {
    std::size_t motor{};
    const Actuator* actuator{};  // in actuators_, fixed after MakePlant
    double gear{1.0};
    double torque_constant_nm_per_a{};
    double back_emf_v_per_radps{};
    double resistance_ohm{};
  };
  struct Input {
    int address{-1};  // d->sensordata index
    bool rangefinder{false};
    double cutoff{};
  };

  MujocoPlant() = default;
  // Brings positions, tendons, transmissions and (for a free root) body
  // velocities up to the current qpos and qvel, then the rotors and chassis.
  void Refresh();
  void Fail(int warning, int info);
  // Fails the plant for good with `text`, at the current plant time.
  void Fail(std::string_view text);

  std::unique_ptr<mjModel_, ModelDeleter> model_;
  std::unique_ptr<mjData_, DataDeleter> data_;
  uint32_t step_us_{};
  std::vector<Actuator> actuators_;
  std::vector<Bound> bound_;
  std::vector<hardware::sim::PlantRotor> rotors_;  // by motor of the last Bind
  std::vector<bool> overspeed_;                    // by entry of bound_
  std::vector<Input> inputs_;
  std::vector<Binding> bindings_;
  std::string binding_table_;
  uint64_t overspeed_excursions_{};
  int root_body_{-1};
  bool root_free_{false};
  hardware::sim::PlantChassis chassis_{};
  double raw_yaw_rad_{};
  std::array<int, 7> warnings_{};  // d->warning[].number per mjtWarning
  std::vector<WorldRules*> rules_;
  MatchInput match_input_{};
  uint64_t steps_{};  // MuJoCo steps taken, the rules' step count
  bool in_hook_{false};
  bool failed_{false};
  std::array<char, 768> failure_{};
};

// Assembles (H), compiles and makes the data. A robot model is
// self-contained: it compiles on its own and names no field element, so one
// model runs on every field and on the flat carpet. The robot's whole
// <option> and <size> are dropped when it is attached (the field's win:
// MuJoCo warns for each field the robot authored, and drops its flags
// silently); the option table then replaces the field's solver settings
// (sleeping included: off unless `model.sleeping`), and the field's other
// options (gravity, say) stay. `motors` are the local
// config's, for the compile-time damping G²·b_m; Bind later refuses a pushed
// config that changes a bound motor's gear or motor type. Throws
// std::invalid_argument naming the file, element or rule.
std::unique_ptr<MujocoPlant> MakePlant(
    const PlantModel& model, std::span<const hardware::sim::PlantMotor> motors);

}  // namespace talos::simulation::mujoco

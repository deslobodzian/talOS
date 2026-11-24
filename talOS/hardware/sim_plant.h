#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string_view>

#include "config_error.h"

namespace talos::hardware::sim {

// A plant outside PhysicsSimBackend that integrates the motion of the motors
// it binds (and, when its model has one, the chassis), while the backend
// stays the only controller and electrical model. The backend makes every
// call on one thread.
//
// Rotor: physical frame (the transmission's positive direction), rotations,
// rps; torque N·m at the rotor; chassis in WorldState's field frame.
// The string_views in PlantMotor and PlantInput are valid only during the
// Bind that receives them: a plant copies what it keeps.
struct PlantMotor {
  std::string_view name;                  // MotorConfig::name
  double gear{1.0};                       // rotor rot per mechanism rot
  double torque_constant_nm_per_a{};      // k_t
  double back_emf_v_per_radps{};          // k_e
  double resistance_ohm{};                // R
  double no_load_damping_nm_per_radps{};  // b_m
};
struct PlantInput {
  std::string_view input;   // DigitalInputConfig::name
  std::string_view sensor;  // the plant's sensor of that name
};
// Torque at every step of the next Step, at that step's rotor speed w:
//   clamp(k_t/R * (volts - (back_emf ? k_e*w : 0)), min, max) - b_m*w
struct PlantDrive {
  double volts{0.0};
  bool back_emf{false};
  double min_torque_nm{-std::numeric_limits<double>::infinity()};
  double max_torque_nm{std::numeric_limits<double>::infinity()};
};
struct PlantRotor {
  double position_rot{};
  double velocity_rps{};
};
struct PlantChassis {
  double x_m{}, y_m{}, yaw_rad{};            // yaw: heading, continuous
  double vx_mps{}, vy_mps{}, omega_radps{};  // field frame, about field z
};

class ExternalPlant {
 public:
  virtual ~ExternalPlant() = default;
  // Even, > 0, and constant for the plant's life.
  virtual uint32_t step_us() const = 0;
  // From Backend::Configure, motors in config order; `owned` arrives all
  // false, and the plant sets owned[i] for each motor it integrates; every
  // input must bind. False refuses the configuration with `error`, and the
  // plant keeps nothing of that call. True may still be followed by the
  // backend refusing the configuration; the plant is then not stepped until
  // its next Bind.
  virtual bool Bind(std::span<const PlantMotor> motors,
                    std::span<const PlantInput> inputs, std::span<bool> owned,
                    ConfigError& error) = 0;
  // `steps` >= 0; drives indexed by motor, unowned entries ignored. Fewer
  // drives than the last Bind's motors fails the plant. False: failed for
  // good (failure()); the backend then makes no further call but failure().
  virtual bool Step(int steps, std::span<const PlantDrive> drives) = 0;
  // Called only for a motor the last Bind owned.
  virtual PlantRotor rotor(std::size_t motor) const = 0;
  // Whether it has a value is constant for the plant's life.
  virtual std::optional<PlantChassis> chassis() const = 0;
  // `input` indexes the last Bind's `inputs`; the reading may describe the
  // world one plant step earlier.
  virtual bool input_closed(std::size_t input) const = 0;
  virtual std::string_view failure() const = 0;
};

}  // namespace talos::hardware::sim

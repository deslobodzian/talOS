#pragma once

// Physics-backed simulation backend for software tests.
//
// Two backends, two jobs:
//   SimBackend (sim_backend.h) ....... ideal actuators for fast logic tests.
//     Velocity/position demands apply instantly; zero gains still move.
//   PhysicsSimBackend (this file) .... believability for PID, gearing,
//     direction, and limit checks. Demands pass through an emulated
//     controller voltage first; zero gains yield ~0 V and a stalled rotor,
//     and nothing teleports.
//
// The motor/mechanism model below is the one reusable model: motor
// type, gear ratio, inertia, friction, and load (roller/flywheel first).
// RoboRIO desktop simulation (PhoenixBackend) instead uses WPILib's
// LinearSystemId::DCMotorSystem plant and chassis kinematics for Pigeon2 yaw;
// the GradleRIO build excludes sim_* headers (see build.gradle).
//
// Physics is driven by controller voltage output. Closed-loop modes
// emulate a TalonFX-style P+FF loop producing volts from the configured gain
// slot; the volts then drive the rotor, whose position/velocity are written
// back as mechanism units (rotor / gear ratio), per CTRE sim guidance that
// sim state is fed from the device's voltage output, not from the demand. A
// torque-current demand becomes the volts that pass that current through the
// winding at the rotor's speed (TorqueCurrentVolts).
// Rotor state is physical (counterclockwise positive, the sim's mounting). A
// TalonFX's `inverted` flips its output and its own position and velocity
// together, as Phoenix 6's MotorOutput.Inverted does, so its closed loops run
// in the configured positive direction; CANcoders see the physical rotor with
// their own `inverted` and `offset_rot`, and the chassis applies each drive
// motor's configured direction and each steer motor's feedback frame
// (including remote CANcoder inversion and `offset_rot`), matching
// PhoenixBackend. A follower ignores its own `inverted`, as under Phoenix 6's
// Follower request: it drives in its master's configured direction, reversed
// by `oppose_master`, and reports in its master's frame, as PhoenixBackend
// reports it. A motor whose feedback is a remote CANcoder reports, and closes
// its loops on, that CANcoder's reading in the CANcoder's own direction, so a
// motor/CANcoder direction mismatch runs away here as on a robot.
//
// Sensors cross the hardware boundary. CANcoder samples mirror the linked
// sensor shaft (rotor / rotor_to_sensor_ratio = mechanism *
// sensor_to_mechanism_ratio), the position wrapped to [-0.5, 0.5) rotations as
// the RoboRIO reports it (CANcoderAbsoluteRot). Swerve ground motion determines
// the true field pose; the Pigeon reports true yaw plus optional drift, unless
// explicitly overridden.
//
// Stopping is consistent. Commanded neutral (Mode::kNeutral, and
// Neutral() on disable/timeout/failure) removes drive and lets the rotor
// decay per the motor's brake flag: brake stops quickly, coast rolls on. A
// zero-velocity demand is instead an active standstill hold (when gains are
// tuned); with zero gains it honestly cannot hold and coasts.
//
// Controlled inputs/faults. Beam breaks and RoboRIO limit-switch inputs via
// SetDigitalInput, a home switch a motor's position closes via
// SetHomeSwitches, physical end-of-travel hard stops via SetHardStops,
// switches on a TalonFX's own limit inputs via
// Set{Forward,Reverse}LimitPressed (enforced per MotorConfig's
// forward_limit_switch and reverse_limit_switch), soft limits from
// MotorConfig enforced in physics, frozen feedback, per-device disconnect,
// supply-voltage change, controller restart.
//
// Explicit sim clock, deterministic stepping. Read() derives dt from
// state.sample_time_us deltas, so Gateway ticks with explicit timestamps are
// exactly reproducible; Step() advances the models without a Gateway. No
// wall-clock or randomness is consulted anywhere here. Scheduling and UDP
// timing (period_us pacing, link latency) stay endpoint concerns and are
// tested separately.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "feedback.h"
#include "gateway.h"
#include "sim_backend.h"

namespace talos::hardware::sim {

// Load reflected onto the rotor shaft. Roller/flywheel first; arm/elevator
// gravity loads are future work.
enum class LoadKind : uint8_t { kRoller = 0, kFlywheel };

// One reusable motor/mechanism model.
struct MotorModel {
  // Rotor free speed at 12 V. Kraken X60 default; matches the PhoenixBackend
  // desktop model so both sims agree on unloaded speed.
  double free_speed_rps_at_12v{100.0};
  // Stall current at 12 V, which sets the winding resistance,
  // R = 12 V / stall current.
  double stall_current_a_at_12v{366.0};
  // Free (no-load) stator current at 12 V, which sets the back-EMF constant,
  // k_e = (12 V - free current * R) / free speed.
  double free_current_a_at_12v{2.0};
  // Stall torque at 12 V, which sets the torque constant,
  // k_t = stall_torque_nm_at_12v / stall_current_a_at_12v.
  double stall_torque_nm_at_12v{7.09};
  // First-order lag at nominal_inertia_kg_m2.
  double time_constant_s{0.02};
  double nominal_inertia_kg_m2{0.002};
  // Extra viscous damping in V per rotor rps, beyond the back-EMF slope.
  double viscous_friction_vs_per_rps{0.0};
  // Deadband: a standing-still rotor will not break away while |volts| is
  // below this. Kept at 0 by default so small signals stay observable.
  double static_friction_v{0.0};

  double resistance_ohm() const { return 12.0 / stall_current_a_at_12v; }
  double back_emf_v_per_rps() const {
    return (12.0 - free_current_a_at_12v * resistance_ohm()) /
           free_speed_rps_at_12v;
  }
  double back_emf_v_per_radps() const {
    constexpr double kTau = 6.28318530717958647692;
    return back_emf_v_per_rps() / kTau;
  }
  double torque_constant_nm_per_a() const {
    return stall_torque_nm_at_12v / stall_current_a_at_12v;
  }
  // No-load internal viscous damping on the rotor shaft (N*m / (rad/s)),
  // chosen so 12 V unloaded steady state matches free_speed_rps_at_12v and
  // free_current_a_at_12v exactly.
  double no_load_damping_nm_per_radps() const {
    constexpr double kTau = 6.28318530717958647692;
    return torque_constant_nm_per_a() * free_current_a_at_12v /
           (kTau * free_speed_rps_at_12v);
  }
};

// The published 12 V free speed, stall current, free current, and stall torque
// of each motor type (CTRE manufacturer specifications / ReCalc, reca.lc):
//   Kraken X60: 6000 rpm, 366 A stall, 2.0 A free, 7.09 N*m stall
//   Kraken X44: 7530 rpm, 275 A stall, 2.0 A free, 4.05 N*m stall
//   Falcon 500: 6380 rpm, 257 A stall, 1.5 A free, 4.69 N*m stall
inline MotorModel MotorModelFor(MotorType type) {
  MotorModel model;
  switch (type) {
    case MotorType::kKrakenX60:
      model.free_speed_rps_at_12v = 6000.0 / 60.0;
      model.stall_current_a_at_12v = 366.0;
      model.free_current_a_at_12v = 2.0;
      model.stall_torque_nm_at_12v = 7.09;
      break;
    case MotorType::kKrakenX44:
      model.free_speed_rps_at_12v = 7530.0 / 60.0;
      model.stall_current_a_at_12v = 275.0;
      model.free_current_a_at_12v = 2.0;
      model.stall_torque_nm_at_12v = 4.05;
      break;
    case MotorType::kFalcon500:
      model.free_speed_rps_at_12v = 6380.0 / 60.0;
      model.stall_current_a_at_12v = 257.0;
      model.free_current_a_at_12v = 1.5;
      model.stall_torque_nm_at_12v = 4.69;
      break;
  }
  return model;
}

struct MechanismModel {
  // Rotor revolutions per mechanism revolution, normally
  // rotor_to_sensor_ratio * sensor_to_mechanism_ratio from MotorConfig.
  double gear_ratio{1.0};
  // Reflected load inertia at the mechanism shaft.
  double inertia_kg_m2{0.005};
  LoadKind load{LoadKind::kRoller};
  // From MotorConfig::brake: brake stops quickly, coast rolls on.
  bool brake{true};
};

// Explicit sim clock. Tests advance this instead of sleeping.
struct SimClock {
  uint64_t now_us{0};
  void Advance(uint64_t dt_us) { now_us += dt_us; }
};

struct AxisState {
  double rotor_position_rot{0.0};
  double rotor_velocity_rps{0.0};
};

struct PhysicsOptions {
  static constexpr double kDefaultWheelRadiusM = 0.0508;
  double start_x_m{0.0};
  double start_y_m{0.0};
  double start_yaw_rad{0.0};
  // Nominal wheel radius in meters. Non-positive or non-finite values fall
  // back to kDefaultWheelRadiusM.
  double wheel_radius_m{kDefaultWheelRadiusM};
  double wheel_radius_error_frac{0.0};
  // Deterministic traction simplification: both acceleration and braking
  // only reduce ground travel; wheel odometry over-reports either way.
  double wheel_slip_gain{0.0};
  double gyro_drift_radps{0.0};
};

// Blue-alliance field frame: x downfield, y left, yaw CCW+. Velocity is in
// that same field frame, not the rotating robot frame.
struct WorldState {
  double x_m{};
  double y_m{};
  double yaw_rad{};
  double vx_mps{};
  double vy_mps{};
  double omega_radps{};
};

// Single deterministic integration step (rotor side). Returns the new state.
// `powered` is false for commanded neutral: the rotor decays per brake/coast
// instead of tracking a voltage.
inline AxisState StepAxis(const MotorModel& motor, const MechanismModel& mech,
                          const AxisState& state, double volts, bool powered,
                          double supply_v, double dt_s) {
  AxisState out = state;
  if (dt_s <= 0.0) return out;
  const double supply = std::max(0.0, supply_v);
  const double effective_volts = std::clamp(volts, -supply, supply);
  const double inertia_scale =
      mech.inertia_kg_m2 > 0.0
          ? mech.inertia_kg_m2 / motor.nominal_inertia_kg_m2
          : 1.0;
  double tau = motor.time_constant_s * std::max(0.1, inertia_scale);
  double target = 0.0;
  if (powered) {
    const double bemf_slope = motor.free_speed_rps_at_12v > 0.0
                                  ? 12.0 / motor.free_speed_rps_at_12v
                                  : 0.0;
    const double damping_v_per_rps =
        bemf_slope + motor.viscous_friction_vs_per_rps;
    target =
        damping_v_per_rps > 0.0 ? effective_volts / damping_v_per_rps : 0.0;
    if (std::abs(out.rotor_velocity_rps) < 1e-9 &&
        std::abs(effective_volts) < motor.static_friction_v)
      target = 0.0;
  } else {
    // Unpowered decay: brake shorts the motor (fast stop), coast lets the
    // load roll. Flywheels carry more energy, so they coast longer.
    tau = mech.brake
              ? std::min(tau, 0.02)
              : std::max(tau, mech.load == LoadKind::kFlywheel ? 1.5 : 0.6);
  }
  const double alpha = std::min(1.0, dt_s / std::max(tau, 1e-4));
  out.rotor_velocity_rps += (target - out.rotor_velocity_rps) * alpha;
  if (!powered && std::abs(out.rotor_velocity_rps) < 0.01) {
    // Standstill snap: friction holds a nearly-stopped unpowered rotor rather
    // than creeping forever. Powered holds never snap; the loop drives them.
    out.rotor_velocity_rps = 0.0;
  }
  out.rotor_position_rot += out.rotor_velocity_rps * dt_s;
  return out;
}

inline double GravityVoltage(const Gains& gains, double mech_pos) {
  constexpr double kTau = 6.28318530717958647692;
  return gains.g * (gains.gravity_type == GravityType::kArmCosine
                        ? std::cos(mech_pos * kTau)
                        : 1.0);
}

// Emulated controller voltage. Closed-loop modes run a P+FF loop on
// the configured gain slot and clamp to the motor's voltage ceiling, so
// missing gains honestly produce ~0 V. Returns false for commanded neutral
// (unpowered); otherwise writes volts. Torque current needs the rotor's
// speed and motor type, so PhysicsSimBackend computes it
// (TorqueCurrentVolts); here it returns false.
inline bool DriveVoltage(const MotorRequest& req, const MotorConfig& cfg,
                         double mech_pos, double mech_vel, double supply_v,
                         double& volts_out) {
  const double supply = std::max(0.0, supply_v);
  const double ceiling =
      std::min(cfg.max_voltage > 0.0 ? cfg.max_voltage : 12.0, supply);
  const auto clamp = [](double v, double lim) {
    return std::clamp(v, -lim, lim);
  };
  switch (req.mode) {
    case Mode::kNeutral:
      return false;
    case Mode::kDutyCycle:
      volts_out = clamp(req.demand * supply, ceiling);
      return true;
    case Mode::kVoltage:
      volts_out = clamp(req.demand, ceiling);
      return true;
    case Mode::kVelocity: {
      const auto& gains = cfg.slots[req.slot < 3 ? req.slot : 0];
      const double error = req.demand - mech_vel;
      volts_out =
          clamp(req.feedforward_v + gains.p * error +
                    gains.s * (req.demand >= 0.0 ? 1.0 : -1.0) *
                        (std::abs(req.demand) > 1e-9 ? 1.0 : 0.0) +
                    gains.v * req.demand + GravityVoltage(gains, mech_pos),
                ceiling);
      return true;
    }
    case Mode::kPosition:
    case Mode::kMotionMagic: {
      // Without positive cruise/acceleration limits, MotionMagic falls back
      // to position control. Profiled requests are handled by the backend.
      const auto& gains = cfg.slots[req.slot < 3 ? req.slot : 0];
      const double error = cfg.continuous_wrap
                               ? std::remainder(req.demand - mech_pos, 1.0)
                               : (req.demand - mech_pos);
      volts_out =
          clamp(req.feedforward_v + gains.p * error - gains.d * mech_vel +
                    GravityVoltage(gains, mech_pos),
                ceiling);
      return true;
    }
    case Mode::kTorqueCurrent:
      return false;
  }
  return false;
}

// A home switch the mechanism closes: the RoboRIO digital input named
// `input` is closed while the motor named `motor` is at `closed_at_rot` or
// beyond it, below it (`closed_above` false) or above it (true). The
// position is the one the motor reports, in mechanism rotations of its own
// hardware frame (before any node's rezero), taken from the physical axis
// whether or not the motor's feedback is valid. The boundary is exact and
// has no hysteresis. A closed switch reads `closed_reads` (false: normally
// open to ground behind the RoboRIO's pull-up, as PositionMechanism's
// limit_switch_closed_reads defaults), an open one the opposite. The switch
// only reports: it stops nothing.
struct HomeSwitch {
  std::string input;
  std::string motor;
  double closed_at_rot{};
  bool closed_above{false};
  bool closed_reads{false};
};

// One swerve module's drive/steer motor names and robot-frame coordinates
// (+x forward, +y left, meters). Bound by motor name so it survives every
// later Configure(), including a config pushed by the bridge.
struct SwerveModuleSpec {
  std::string drive;
  std::string steer;
  double x_m{};
  double y_m{};
};

// Mechanical end-of-travel hard stops for a named master motor, in mechanism
// rotations of its own hardware frame (before any node's rezero), matching
// HomeSwitch. Enforced even when MotorConfig::soft_limits is false (as
// required for rezero-frame mechanisms): clamps position into [min_rot,
// max_rot] and zeroes velocity when driven into a stop (for the master and
// every follower in its chain), so back-EMF drops to 0 and stator current rises
// while motion away from the stop stays free. Bound by motor name so it
// survives every later Configure().
struct HardStop {
  std::string motor;
  std::optional<double> min_rot;
  std::optional<double> max_rot;
};

// Battery internal-resistance sag and automatic brownout model.
// Default values (internal_resistance_ohm == 0, nominal_voltage_v == 12.8)
// preserve bit-for-bit legacy resting supply behavior.
struct BatteryModel {
  double nominal_voltage_v{12.8};
  double internal_resistance_ohm{0.0};
  double base_current_a{0.0};
  double brownout_voltage_v{6.75};
};

// Coupled 2D rigid-body chassis mass and wheel traction model for swerve.
// Survives every Configure().
struct ChassisDynamics {
  double mass_kg{
      0.0};  // > 0 activates dynamic chassis; <= 0 keeps legacy kinematics
  double moment_of_inertia_kg_m2{
      0.0};  // <= 0 defaults to mass_kg * mean(x_i^2 + y_i^2)
  double traction_coefficient{
      0.0};  // <= 0 means infinite traction (mass-only limit)
};

// Mechanism inertia and friction parameters for rigid-body rotor ODE on a
// master motor (matched by CAN id) and its follower chain. Survives every
// Configure().
struct MechanismSpec {
  std::uint8_t motor_can_id{0};
  double inertia_kg_m2{
      0.0};  // > 0 activates rigid-body ODE on this master motor
  LoadKind load{LoadKind::kRoller};
  double viscous_damping_nm_per_radps{0.0};
  double static_friction_nm{0.0};
};

// Physics-backed Backend. See the header comment for the item-by-item map.
class PhysicsSimBackend final : public SimBackendBase {
 public:
  explicit PhysicsSimBackend(PhysicsOptions options = {}) : options_{options} {
    if (!std::isfinite(options_.wheel_radius_m) ||
        options_.wheel_radius_m <= 0.0) {
      options_.wheel_radius_m = PhysicsOptions::kDefaultWheelRadiusM;
    }
  }

  const WorldState& truth() const { return truth_; }

  // The motor types of the robot config sim_gateway parsed itself. A config
  // the bridge pushes carries none (MotorConfig::motor_type is off the wire),
  // so every later Configure gives each motor the type of the local motor
  // with the same name (names are unique in a valid config); a motor the
  // local config does not name keeps the type it was configured with, a
  // Kraken X60 off the wire.
  void SetLocalMotorTypes(const Config& local) {
    local_motor_types_.clear();
    for (const auto& m : local.motors) {
      local_motor_types_[m.name.str()] = m.motor_type.value;
    }
  }

  // The home switches (HomeSwitch), replacing any set before. They survive
  // every later Configure, a config the bridge pushes included, matched by
  // the input's and the motor's names; a switch whose input or motor the
  // config does not name is inactive, and that input reads as
  // SetDigitalInput left it. A bound input ignores SetDigitalInput.
  void SetHomeSwitches(std::vector<HomeSwitch> switches) {
    home_switches_ = std::move(switches);
    BindHomeSwitches();
  }

  // Configured swerve modules (SwerveModuleSpec), replacing any set before.
  // When non-empty, Configure() binds only these modules by motor name instead
  // of inferring ±0.30 m coordinates from motor names, and they survive every
  // later Configure, a config the bridge pushes included. Passing an empty
  // vector restores name inference.
  void SetSwerveModules(std::vector<SwerveModuleSpec> modules) {
    configured_modules_ = std::move(modules);
    ConfigureSwerve();
  }

  // Mechanical end-of-travel hard stops (HardStop), replacing any set before.
  // Matched by motor name so they survive every later Configure, including a
  // config pushed by the bridge; an entry whose motor is not configured or
  // whose bounds are invalid is ignored.
  void SetHardStops(std::vector<HardStop> stops) {
    hard_stops_ = std::move(stops);
    BindHardStops();
  }

  // Battery sag and brownout model. Survives every later Configure().
  // Inactive while an explicit SetSupplyVoltage / SetPowerHealth override is
  // set; ClearSupplyVoltageOverride() resumes it.
  void SetBatteryModel(const BatteryModel& model) {
    battery_model_.nominal_voltage_v =
        std::isfinite(model.nominal_voltage_v)
            ? std::clamp(model.nominal_voltage_v, 0.0, 16.0)
            : kSimRestingSupplyV;
    battery_model_.internal_resistance_ohm =
        std::isfinite(model.internal_resistance_ohm) &&
                model.internal_resistance_ohm > 0.0
            ? model.internal_resistance_ohm
            : 0.0;
    battery_model_.base_current_a =
        std::isfinite(model.base_current_a) && model.base_current_a > 0.0
            ? model.base_current_a
            : 0.0;
    battery_model_.brownout_voltage_v =
        std::isfinite(model.brownout_voltage_v) &&
                model.brownout_voltage_v >= 0.0
            ? model.brownout_voltage_v
            : 0.0;
    if (!supply_voltage_override_) {
      ApplyRestingBattery();
    }
  }

  // Clears any explicit SetSupplyVoltage override and resumes the configured
  // BatteryModel.
  void ClearSupplyVoltageOverride() {
    supply_voltage_override_.reset();
    ApplyRestingBattery();
  }

  // Coupled 2D swerve chassis mass, rotational inertia, and tire traction
  // model. Survives every later Configure().
  void SetChassisDynamics(const ChassisDynamics& dynamics) {
    chassis_dynamics_.mass_kg =
        std::isfinite(dynamics.mass_kg) && dynamics.mass_kg > 0.0
            ? dynamics.mass_kg
            : 0.0;
    chassis_dynamics_.moment_of_inertia_kg_m2 =
        std::isfinite(dynamics.moment_of_inertia_kg_m2) &&
                dynamics.moment_of_inertia_kg_m2 > 0.0
            ? dynamics.moment_of_inertia_kg_m2
            : 0.0;
    chassis_dynamics_.traction_coefficient =
        std::isfinite(dynamics.traction_coefficient) &&
                dynamics.traction_coefficient > 0.0
            ? dynamics.traction_coefficient
            : 0.0;
  }

  // Rigid-body mechanism specs (matched by master motor CAN id), replacing any
  // set before. Survives every later Configure().
  void SetMechanisms(std::vector<MechanismSpec> specs) {
    mechanisms_ = std::move(specs);
    BindMechanisms();
  }

  bool Configure(const Config& config) override {
    config_ = config;
    can_bus_count_ = CanBuses(config_).count;
    for (auto& m : config_.motors) {
      if (const auto it = local_motor_types_.find(m.name.str());
          it != local_motor_types_.end()) {
        m.motor_type.value = it->second;
      }
    }
    BindHomeSwitches();
    BindHardStops();
    BindMechanisms();
    axes_.fill(AxisState{});
    profiles_.fill(Profile{});
    gyro_override_ = false;
    modules_.clear();
    commands_.fill(MotorRequest{});
    applied_volts_.fill(0.0);
    powered_.fill(false);
    frozen_.fill(false);
    held_.fill(MotorSample{});
    held_at_us_.fill(0);
    motor_connected_.fill(true);
    sensor_connected_.fill(true);
    sensor_read_at_us_.fill(0);
    motor_to_sensor_.fill(-1);
    forward_pressed_.fill(false);
    reverse_pressed_.fill(false);
    previous_time_.reset();
    digital_inputs_.clear();
    digital_outputs_.clear();
    analog_inputs_.clear();
    encoders_.clear();
    pwm_outputs_.clear();
    pending_resets_.clear();
    restoring_.fill(false);
    supply_voltage_override_.reset();
    explicit_brownout_ = false;
    ApplyRestingBattery();
    yaw_rot_ = 0.0;
    yaw_rate_rps_ = 0.0;
    truth_ = {options_.start_x_m, options_.start_y_m, options_.start_yaw_rad};
    chassis_vx_body_mps_ = 0.0;
    chassis_vy_body_mps_ = 0.0;
    chassis_omega_radps_ = 0.0;
    gyro_zero_yaw_rad_ = truth_.yaw_rad;
    elapsed_s_ = 0.0;
    for (std::size_t i = 0; i < config.motors.size(); ++i) {
      const auto& m = config.motors[i];
      held_[i].id = m.id;
      held_[i].valid = true;
      if (m.feedback == Feedback::kRemoteCANcoder) {
        for (std::size_t s = 0; s < config.sensors.size(); ++s) {
          if (config.sensors[s].id == m.feedback_sensor_id) {
            motor_to_sensor_[i] = static_cast<int>(s);
            break;
          }
        }
      }
    }
    ConfigureSwerve();
    return true;
  }

  bool Read(State& state) override {
    ReadSimulatedCanBus(state.health, can_bus_count_);
    double dt = 0.0;
    if (previous_time_) {
      const int64_t delta =
          static_cast<int64_t>(state.sample_time_us - *previous_time_);
      dt = delta > 0 ? static_cast<double>(delta) * 1e-6 : 0.0;
      if (dt > 0.05) dt = 0.05;
    }
    previous_time_ = state.sample_time_us;
    StepModels(dt);
    ReadPower(state.health, {true, supply_v_, brownout_});
    bool healthy = config_.motors.empty() && config_.sensors.empty();
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      const auto& m = config_.motors[i];
      auto& out = state.motors[i];
      out.id = m.id;
      out.position_age_us = 0;
      out.velocity_age_us = 0;
      if (!motor_connected_[i]) {
        out = HeldSample(i, state.sample_time_us);
        MarkInvalid(out);
        continue;
      }
      if (restoring_[i]) {
        MarkInvalid(out);
        out.temperature_c = 25.0;
        out.fault_flags = 0;
        continue;
      }
      healthy = true;
      if (frozen_[i]) {
        // Stale, as a Phoenix signal that stops updating, but still valid.
        out = HeldSample(i, state.sample_time_us);
        continue;
      }
      const double gear = Gear(i);
      const double mech_pos = MechPosition(i);
      const double mech_vel =
          axes_[i].rotor_velocity_rps * FeedbackDirection(i) / gear;
      out.valid = true;
      out.position_rot = mech_pos;
      out.velocity_rps = mech_vel;
      out.voltage = powered_[i] ? applied_volts_[i] : 0.0;
      out.temperature_c = 25.0;
      out.fault_flags = HardLimitFlags(i);
      if (EnforceSoftLimits(m, out, i) && powered_[i]) {
        applied_volts_[i] = LimitWindingVoltage(i, m, applied_volts_[i]);
        out.voltage = applied_volts_[i];
      }
      // After any limit stops the rotor: a motor held at it is stalled.
      out.stator_current_a = StatorCurrent(i, m);
      out.supply_current_a = SupplyCurrent(i, m);
      held_[i] = out;
      held_at_us_[i] = state.sample_time_us;
    }
    for (std::size_t i = 0; i < config_.sensors.size(); ++i) {
      const auto& s = config_.sensors[i];
      auto& out = state.sensors[i];
      out.id = s.id;
      out.position_age_us = 0;
      out.velocity_age_us = 0;
      if (!sensor_connected_[i]) {
        // Its frames stopped: its ages grow from its last sample.
        const uint32_t age_us =
            AgeUs(state.sample_time_us, sensor_read_at_us_[i]);
        out.position_age_us = age_us;
        out.velocity_age_us = age_us;
        MarkInvalid(out);
        continue;
      }
      sensor_read_at_us_[i] = state.sample_time_us;
      healthy = true;
      if (s.kind == SensorKind::kPigeon2) {
        // Pigeon yaw comes from simulated chassis motion, never
        // from a motor demand directly.
        out.valid = true;
        const double sign = s.inverted ? -1.0 : 1.0;
        out.position_rot = (yaw_rot_ + s.offset_rot) * sign;
        out.velocity_rps = yaw_rate_rps_ * sign;
      } else {
        // CANcoder sits on the sensor shaft (rotor / rotor_to_sensor_ratio =
        // mechanism * sensor_to_mechanism_ratio); its own health is independent
        // of whether the linked motor is connected or frozen.
        out.valid = true;
        double pos_rot = 0.0;
        double vel_rps = 0.0;
        for (std::size_t j = 0; j < config_.motors.size(); ++j) {
          if (motor_to_sensor_[j] == static_cast<int>(i)) {
            const double rotor_to_sensor = RotorToSensor(j);
            pos_rot = axes_[j].rotor_position_rot / rotor_to_sensor;
            vel_rps = axes_[j].rotor_velocity_rps / rotor_to_sensor;
            break;
          }
        }
        if (s.inverted) {
          pos_rot = -pos_rot;
          vel_rps = -vel_rps;
        }
        out.position_rot = CANcoderAbsoluteRot(pos_rot + s.offset_rot);
        out.velocity_rps = vel_rps;
      }
    }
    for (std::size_t i = 0; i < config_.digital_inputs.size(); ++i) {
      const auto id = config_.digital_inputs[i].id;
      const auto it = digital_inputs_.find(id);
      state.digital_inputs[i] = {
          id, true, it != digital_inputs_.end() ? it->second : false};
    }
    // After the motors, so a soft limit has already stopped each axis.
    for (const BoundHomeSwitch& s : bound_home_switches_) {
      const double position_rot = MechPosition(s.motor);
      const bool closed = s.closed_above ? position_rot >= s.closed_at_rot
                                         : position_rot <= s.closed_at_rot;
      state.digital_inputs[s.input].value =
          closed ? s.closed_reads : !s.closed_reads;
    }
    for (std::size_t i = 0; i < config_.digital_outputs.size(); ++i) {
      const auto id = config_.digital_outputs[i].id;
      const auto it = digital_outputs_.find(id);
      state.digital_outputs[i] = {
          id, true, it != digital_outputs_.end() ? it->second : false};
    }
    for (std::size_t i = 0; i < config_.analog_inputs.size(); ++i) {
      const auto id = config_.analog_inputs[i].id;
      const auto it = analog_inputs_.find(id);
      state.analog_inputs[i] = {
          id, true, it != analog_inputs_.end() ? it->second : 0.0, 0};
    }
    for (std::size_t i = 0; i < config_.encoders.size(); ++i) {
      const auto id = config_.encoders[i].id;
      const auto it = encoders_.find(id);
      const double pos = it != encoders_.end() ? it->second.position_rot : 0.0;
      const double vel = it != encoders_.end() ? it->second.velocity_rps : 0.0;
      state.encoders[i] = {id, true, pos, vel, 0};
    }
    for (std::size_t i = 0; i < config_.pwm_outputs.size(); ++i) {
      state.pwm_outputs[i] = PwmSampleOf(GetPwm(config_.pwm_outputs[i].id));
    }
    return healthy;
  }

  // Safe-state policy mirrors SimBackend: every output type turns off on
  // disable, command timeout, and failure (motors to kNeutral, PWM off,
  // digital outputs de-energized). Neutral removes drive but does NOT zero
  // the rotors: they decay per brake/coast. Use RestartController
  // to zero motion state.
  void Neutral() override {
    commands_ = {};
    profiles_.fill(Profile{});
    applied_volts_.fill(0.0);
    powered_.fill(false);
    digital_outputs_.clear();
    pwm_outputs_.clear();
    if (!supply_voltage_override_) {
      ApplyRestingBattery();
    }
  }

  // Deterministic stepping without a Gateway: advances the models
  // by dt_s under the currently applied commands.
  void Step(double dt_s) { StepModels(dt_s); }

  // Controlled inputs/faults.
  void SetDigitalInput(uint16_t id, bool value) { digital_inputs_[id] = value; }
  // The mechanism pressing the switch on a motor's forward or reverse TalonFX
  // limit input. It models a correctly wired switch: an enabled one of either
  // type is asserted while pressed, and the motor's output toward it goes
  // neutral as on the TalonFX. A follower takes its master's output; its own
  // switches are not modelled. While such a switch is asserted the motor
  // reports its ForwardHardLimit or ReverseHardLimit fault bit
  // (kMotorForwardHardLimitFlag, kMotorReverseHardLimitFlag), as PhoenixBackend
  // reports the TalonFX's FaultField.
  void SetForwardLimitPressed(uint16_t motor_id, bool pressed) {
    if (const int i = MotorIndex(motor_id); i >= 0)
      forward_pressed_[i] = pressed;
  }
  void SetReverseLimitPressed(uint16_t motor_id, bool pressed) {
    if (const int i = MotorIndex(motor_id); i >= 0)
      reverse_pressed_[i] = pressed;
  }
  void SetAnalogInput(uint16_t id, double voltage) {
    analog_inputs_[id] = voltage;
  }
  void SetEncoder(uint16_t id, double position_rot, double velocity_rps) {
    encoders_[id] = {position_rot, velocity_rps};
  }
  // Restore only reset motors, preserving positions and unrelated motion.
  // The gateway neutralizes dependent motors before starting restoration.
  // The restored chain reads invalid until a PollRestore that succeeds
  // (RestoredWith), as on the robot.
  void BeginRestoreAfterReset(std::span<const uint16_t> ids) override {
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      if (RestoredWith(config_.motors, i, ids)) restoring_[i] = true;
    }
    for (auto id : ids) {
      const int i = MotorIndex(id);
      if (i >= 0) {
        axes_[i].rotor_velocity_rps = 0.0;
        profiles_[i] = Profile{};
        commands_[i] = MotorRequest{};
        applied_volts_[i] = 0.0;
        powered_[i] = false;
        frozen_[i] = false;
        motor_connected_[i] = true;
        held_[i] = MotorSample{};
        held_[i].id = id;
        held_[i].valid = true;
      }
      for (std::size_t j = 0; j < config_.sensors.size(); ++j)
        if (config_.sensors[j].id == id) sensor_connected_[j] = true;
    }
  }
  void FreezeFeedback(uint16_t motor_id) {
    if (const int i = MotorIndex(motor_id); i >= 0) frozen_[i] = true;
  }
  void UnfreezeFeedback(uint16_t motor_id) {
    if (const int i = MotorIndex(motor_id); i >= 0) frozen_[i] = false;
  }
  void SetMotorConnected(uint16_t motor_id, bool connected) {
    if (const int i = MotorIndex(motor_id); i >= 0)
      motor_connected_[i] = connected;
  }
  void SetSensorConnected(uint16_t sensor_id, bool connected) {
    for (std::size_t i = 0; i < config_.sensors.size(); ++i) {
      if (config_.sensors[i].id == sensor_id) sensor_connected_[i] = connected;
    }
  }
  void SetSupplyVoltage(double volts) {
    const double clamped = std::clamp(volts, 0.0, 16.0);
    supply_voltage_override_ = clamped;
    supply_v_ = clamped;
    brownout_ = explicit_brownout_;
  }
  double supply_voltage() const { return supply_v_; }
  void SetPowerHealth(double battery_v, bool brownout) {
    explicit_brownout_ = brownout;
    SetSupplyVoltage(battery_v);
  }
  // Zeroes rotor states, latched commands, reported gyro, and fault flags.
  // The physical field pose survives a controller restart; subsequent gyro
  // samples measure yaw relative to this restart. Drift restarts at zero.
  // Config/output states and the gateway-owned boot id are unchanged.
  void RestartController() {
    axes_.fill(AxisState{});
    profiles_.fill(Profile{});
    gyro_override_ = false;
    commands_.fill(MotorRequest{});
    applied_volts_.fill(0.0);
    powered_.fill(false);
    frozen_.fill(false);
    restoring_.fill(false);
    motor_connected_.fill(true);
    sensor_connected_.fill(true);
    explicit_brownout_ = false;
    if (supply_voltage_override_) {
      brownout_ = false;
    } else {
      ApplyRestingBattery();
    }
    yaw_rot_ = 0.0;
    yaw_rate_rps_ = 0.0;
    gyro_zero_yaw_rad_ = truth_.yaw_rad;
    elapsed_s_ = 0.0;
    truth_.vx_mps = truth_.vy_mps = truth_.omega_radps = 0.0;
    chassis_vx_body_mps_ = 0.0;
    chassis_vy_body_mps_ = 0.0;
    chassis_omega_radps_ = 0.0;
    for (auto& module : modules_) module.previous_surface_speed_mps = 0.0;
  }
  // Explicit reported-gyro overrides, in rotations/s. These do not teleport
  // the physical robot or change the module-derived ground motion.
  void SetChassisYawRate(double yaw_rate_rps) {
    gyro_override_ = true;
    yaw_rate_rps_ = yaw_rate_rps;
  }
  void SetGyro(double yaw_rot, double yaw_rate_rps) {
    SetChassisYawRate(yaw_rate_rps);
    yaw_rot_ = yaw_rot;
  }
  // Override inferred placement, or register arbitrarily named module pairs.
  // Wheel travel uses options_.wheel_radius_m (default 0.0508 m) plus the
  // configured error.
  void SetSwerveModuleGeometry(uint16_t drive_id, uint16_t steer_id, double x_m,
                               double y_m) {
    const int drive = MotorIndex(drive_id);
    const int steer = MotorIndex(steer_id);
    if (drive < 0 || steer < 0 || drive == steer || !std::isfinite(x_m) ||
        !std::isfinite(y_m) || (x_m == 0.0 && y_m == 0.0))
      return;
    for (auto& module : modules_) {
      if (module.drive == static_cast<std::size_t>(drive)) {
        module = {static_cast<std::size_t>(drive),
                  static_cast<std::size_t>(steer), x_m, y_m};
        return;
      }
    }
    modules_.push_back({static_cast<std::size_t>(drive),
                        static_cast<std::size_t>(steer), x_m, y_m});
  }
  double chassis_yaw_rot() const { return yaw_rot_; }
  double applied_voltage(std::size_t motor_index) const {
    return motor_index < applied_volts_.size() ? applied_volts_[motor_index]
                                               : 0.0;
  }

 private:
  static constexpr double kTau = 6.28318530717958647692;

  struct Profile {
    double profile_pos_rot{};
    double profile_vel_rps{};
    bool active{};
  };
  struct Module {
    std::size_t drive;
    std::size_t steer;
    double x_m;
    double y_m;
    double previous_surface_speed_mps{};
  };

  void ConfigureSwerve() {
    modules_.clear();
    if (!configured_modules_.empty()) {
      for (const auto& spec : configured_modules_) {
        if (!std::isfinite(spec.x_m) || !std::isfinite(spec.y_m) ||
            (spec.x_m == 0.0 && spec.y_m == 0.0)) {
          continue;
        }
        std::optional<std::size_t> drive;
        std::optional<std::size_t> steer;
        for (std::size_t i = 0; i < config_.motors.size(); ++i) {
          if (config_.motors[i].name == spec.drive) drive = i;
          if (config_.motors[i].name == spec.steer) steer = i;
        }
        if (!drive || !steer) continue;
        const std::size_t drive_idx = *drive;
        const std::size_t steer_idx = *steer;
        if (drive_idx == steer_idx) continue;
        bool duplicate = false;
        for (const auto& existing : modules_) {
          if (existing.drive == drive_idx || existing.steer == steer_idx ||
              existing.drive == steer_idx || existing.steer == drive_idx) {
            duplicate = true;
            break;
          }
        }
        if (!duplicate) {
          modules_.push_back({drive_idx, steer_idx, spec.x_m, spec.y_m});
        }
      }
      return;
    }
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      const auto& name = config_.motors[i].name;
      if (!name.ends_with("_drive")) continue;
      const auto prefix = name.substr(0, name.size() - 6);
      for (std::size_t j = 0; j < config_.motors.size(); ++j) {
        if (config_.motors[j].name != prefix + "_steer") continue;
        // Conventional module names locate the wheel in the robot frame:
        // +x forward, +y left. Unknown locations cannot determine rotation.
        const double x = prefix.find("front") != std::string::npos ? 0.30
                         : prefix.find("back") != std::string::npos ||
                                 prefix.find("rear") != std::string::npos
                             ? -0.30
                             : 0.0;
        const double y = prefix.find("left") != std::string::npos    ? 0.30
                         : prefix.find("right") != std::string::npos ? -0.30
                                                                     : 0.0;
        if (x != 0.0 || y != 0.0) modules_.push_back({i, j, x, y});
      }
    }
  }

  // Exact acceleration/cruise/deceleration segments, with a stationary goal.
  // Replanning from the current setpoint handles demand changes, goal
  // shortening inside the stopping distance, and initial speeds above cruise
  // without teleporting or exceeding acceleration_rps2.
  double AdvanceProfile(Profile& p, const MotorConfig& cfg, double goal,
                        double dt) {
    if (dt <= 0.0) return 0.0;
    const double accel = cfg.acceleration_rps2;
    const double cruise = cfg.cruise_velocity_rps;
    const double previous_vel = p.profile_vel_rps;
    const double stop_pos =
        p.profile_pos_rot +
        p.profile_vel_rps * std::abs(p.profile_vel_rps) / (2.0 * accel);
    const double stop_error = goal - stop_pos;
    const double direction =
        (stop_error > 0.0 || (stop_error == 0.0 && p.profile_vel_rps < 0.0))
            ? 1.0
            : -1.0;
    double pos = p.profile_pos_rot * direction;
    double vel = p.profile_vel_rps * direction;
    const double target = goal * direction;
    double rem_dt = dt;

    // Moving away from target (including overshoot braking): brake at +accel
    // until velocity reaches 0.
    if (vel < 0.0) {
      const double t_stop = -vel / accel;
      if (rem_dt <= t_stop) {
        pos += vel * rem_dt + 0.5 * accel * rem_dt * rem_dt;
        vel += accel * rem_dt;
        p.profile_pos_rot = pos * direction;
        p.profile_vel_rps = vel * direction;
        return (p.profile_vel_rps - previous_vel) / dt;
      }
      pos += vel * t_stop + 0.5 * accel * t_stop * t_stop;
      vel = 0.0;
      rem_dt -= t_stop;
    }

    // Moving toward target faster than cruise: decelerate at -accel down to
    // cruise. By choice of direction, target - pos >= vel^2 / (2 * accel), so
    // decelerating to cruise never overshoots target.
    if (vel > cruise) {
      const double t_slow = (vel - cruise) / accel;
      if (rem_dt <= t_slow) {
        pos += vel * rem_dt - 0.5 * accel * rem_dt * rem_dt;
        vel -= accel * rem_dt;
        p.profile_pos_rot = pos * direction;
        p.profile_vel_rps = vel * direction;
        return (p.profile_vel_rps - previous_vel) / dt;
      }
      pos += 0.5 * (vel + cruise) * t_slow;
      vel = cruise;
      rem_dt -= t_slow;
    }

    // Standard trapezoid with 0 <= vel <= cruise and
    // rem_dist >= vel^2 / (2 * accel).
    const double rem_dist = std::max(0.0, target - pos);
    const double v_peak_tri =
        std::sqrt(std::max(0.0, rem_dist * accel + 0.5 * vel * vel));
    const double v_peak = std::clamp(v_peak_tri, vel, cruise);
    const double t_accel = (v_peak - vel) / accel;
    const double d_accel = 0.5 * (vel + v_peak) * t_accel;
    const double t_decel = v_peak / accel;
    const double d_decel = 0.5 * v_peak * t_decel;
    const double d_cruise = std::max(0.0, rem_dist - d_accel - d_decel);
    const double t_cruise = d_cruise > 0.0 ? d_cruise / cruise : 0.0;
    const double end_cruise = t_accel + t_cruise;
    const double end_decel = end_cruise + t_decel;

    double next_pos = target;
    double next_vel = 0.0;
    if (rem_dt < t_accel) {
      next_vel = vel + rem_dt * accel;
      next_pos = pos + (vel + 0.5 * rem_dt * accel) * rem_dt;
    } else if (rem_dt < end_cruise) {
      next_vel = cruise;
      next_pos = pos + d_accel + cruise * (rem_dt - t_accel);
    } else if (rem_dt < end_decel) {
      const double remaining = end_decel - rem_dt;
      next_vel = remaining * accel;
      next_pos = target - 0.5 * remaining * next_vel;
    }
    p.profile_pos_rot = next_pos * direction;
    p.profile_vel_rps = next_vel * direction;
    return (p.profile_vel_rps - previous_vel) / dt;
  }
  bool brownout_{};
  struct ManualEncoder {
    double position_rot{};
    double velocity_rps{};
  };

  // Phoenix 6 output direction of motor i in the physical rotor frame. A
  // follower's own Inverted does not apply: the Follower request takes the
  // master's configured direction, reversed when opposed.
  double OutputDirection(std::size_t i) const {
    double sign = 1.0;
    for (std::size_t hop = 0; hop <= config_.motors.size(); ++hop) {
      const auto& m = config_.motors[i];
      const int master = m.master_id != 0 ? MotorIndex(m.master_id) : -1;
      if (master < 0) return m.inverted ? -sign : sign;
      if (m.oppose_master) sign = -sign;
      i = static_cast<std::size_t>(master);
    }
    return sign;  // A follower cycle is never powered.
  }

  // The frame motor i's voltage and rotor feedback are reported in, in the
  // physical rotor frame: its configured direction, or for a follower its
  // master's, matching PhoenixBackend, which undoes the opposition Phoenix
  // applies to an opposed follower's signals.
  double Direction(std::size_t i) const {
    const auto& m = config_.motors[i];
    return m.master_id != 0 && m.oppose_master ? -OutputDirection(i)
                                               : OutputDirection(i);
  }

  // The direction of motor i's position and velocity feedback in the physical
  // rotor frame: a remote CANcoder's own direction, else Direction(i).
  double FeedbackDirection(std::size_t i) const {
    const int sensor = motor_to_sensor_[i];
    if (sensor < 0) return Direction(i);
    return config_.sensors[sensor].inverted ? -1.0 : 1.0;
  }

  // Offset of motor i's position feedback in mechanism rotations: a remote
  // CANcoder's magnet offset divided by sensor_to_mechanism_ratio (matching
  // Phoenix 6, which divides the remote CANcoder reading once), else 0.
  double FeedbackOffset(std::size_t i) const {
    const int sensor = motor_to_sensor_[i];
    if (sensor < 0) return 0.0;
    return config_.sensors[sensor].offset_rot / SensorToMechanism(i);
  }

  double MechPosition(std::size_t i) const {
    return axes_[i].rotor_position_rot * FeedbackDirection(i) / Gear(i) +
           FeedbackOffset(i);
  }

  double RotorToSensor(std::size_t i) const {
    const double ratio = config_.motors[i].rotor_to_sensor_ratio;
    return std::isfinite(ratio) && ratio > 0.0 ? ratio : 1.0;
  }

  double SensorToMechanism(std::size_t i) const {
    const double ratio = config_.motors[i].sensor_to_mechanism_ratio;
    return std::isfinite(ratio) && ratio > 0.0 ? ratio : 1.0;
  }

  double Gear(std::size_t i) const {
    const double gear = RotorToSensor(i) * SensorToMechanism(i);
    return std::isfinite(gear) && gear > 0.0 ? gear : 1.0;
  }

  int MotorIndex(uint16_t id) const {
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      if (config_.motors[i].id == id) return static_cast<int>(i);
    }
    return -1;
  }

  // Resolves home_switches_ by name against config_.
  void BindHomeSwitches() {
    bound_home_switches_.clear();
    bound_home_switches_.reserve(home_switches_.size());
    for (const HomeSwitch& s : home_switches_) {
      std::optional<std::size_t> input;
      std::optional<std::size_t> motor;
      for (std::size_t i = 0; i < config_.digital_inputs.size(); ++i) {
        if (config_.digital_inputs[i].name == s.input) input = i;
      }
      for (std::size_t i = 0; i < config_.motors.size(); ++i) {
        if (config_.motors[i].name == s.motor) motor = i;
      }
      if (input && motor) {
        bound_home_switches_.push_back(
            {*input, *motor, s.closed_at_rot, s.closed_above, s.closed_reads});
      }
    }
  }

  // Resolves motor i to its root master along any follower chain, returning
  // the root index and cumulative oppose_master sign (+1 aligned, -1 opposed).
  // Returns {i, +1.0} for an independent motor or on a missing master / cycle.
  std::pair<std::size_t, double> ResolveRootMaster(std::size_t i) const {
    std::size_t cur = i;
    double oppose_sign = 1.0;
    for (std::size_t hop = 0; hop < config_.motors.size(); ++hop) {
      const auto& m = config_.motors[cur];
      if (m.master_id == 0) return {cur, oppose_sign};
      const int master = MotorIndex(m.master_id);
      if (master < 0) return {i, 1.0};
      if (m.oppose_master) oppose_sign = -oppose_sign;
      cur = static_cast<std::size_t>(master);
    }
    return {i, 1.0};
  }

  // Resolves hard_stops_ by motor name against config_. Follower motors are
  // ignored here (config load refuses them; EnforceHardStop applies the root
  // master's stop to every follower in its chain).
  void BindHardStops() {
    bound_hard_stops_.fill(BoundHardStop{});
    for (const HardStop& s : hard_stops_) {
      if (!s.min_rot && !s.max_rot) continue;
      if (s.min_rot && !std::isfinite(*s.min_rot)) continue;
      if (s.max_rot && !std::isfinite(*s.max_rot)) continue;
      if (s.min_rot && s.max_rot && !(*s.min_rot < *s.max_rot)) continue;
      for (std::size_t i = 0; i < config_.motors.size(); ++i) {
        if (config_.motors[i].name == s.motor) {
          if (config_.motors[i].master_id == 0) {
            bound_hard_stops_[i] = {s.min_rot, s.max_rot};
          }
          break;
        }
      }
    }
  }

  // Enforces physical end-of-travel hard stops on axis i regardless of
  // MotorConfig::soft_limits. A follower shares its root master's stop in the
  // master's mechanism frame (flipped by cumulative oppose_master). Returns
  // true when a stop zeroed velocity.
  bool EnforceHardStop(std::size_t i) {
    const auto [root, oppose_sign] = ResolveRootMaster(i);
    const BoundHardStop& stop = bound_hard_stops_[root];
    if (!stop.min_rot && !stop.max_rot) return false;
    const double gear = Gear(root);
    const double dir = FeedbackDirection(root) * oppose_sign;
    const double offset = FeedbackOffset(root);
    double mech_pos = axes_[i].rotor_position_rot * dir / gear + offset;
    double mech_vel = axes_[i].rotor_velocity_rps * dir / gear;
    bool stopped = false;
    if (stop.min_rot && mech_pos <= *stop.min_rot) {
      mech_pos = *stop.min_rot;
      if (mech_vel < 0.0) {
        mech_vel = 0.0;
        stopped = true;
      }
    }
    if (stop.max_rot && mech_pos >= *stop.max_rot) {
      mech_pos = *stop.max_rot;
      if (mech_vel > 0.0) {
        mech_vel = 0.0;
        stopped = true;
      }
    }
    axes_[i].rotor_position_rot = (mech_pos - offset) * dir * gear;
    axes_[i].rotor_velocity_rps = mech_vel * dir * gear;
    return stopped;
  }

  bool EnforceSoftLimits(const MotorConfig& m, MotorSample& out,
                         std::size_t i) {
    if (!m.soft_limits) return false;
    const double lo = std::min(m.reverse_limit_rot, m.forward_limit_rot);
    const double hi = std::max(m.reverse_limit_rot, m.forward_limit_rot);
    const double gear = Gear(i);
    bool stopped = false;
    if (out.position_rot < lo || out.position_rot > hi) {
      const double clamped = std::clamp(out.position_rot, lo, hi);
      axes_[i].rotor_position_rot =
          (clamped - FeedbackOffset(i)) * FeedbackDirection(i) * gear;
      out.position_rot = clamped;
      if ((out.position_rot <= lo && out.velocity_rps < 0) ||
          (out.position_rot >= hi && out.velocity_rps > 0)) {
        axes_[i].rotor_velocity_rps = 0.0;
        out.velocity_rps = 0.0;
        stopped = true;
      }
    }
    return stopped;
  }

  // Output toward an asserted, enabled limit switch is held neutral; volts
  // are in the motor's configured positive (forward) direction.
  bool LimitSwitchBlocks(std::size_t i, double volts) const {
    const auto& m = config_.motors[i];
    return (volts > 0.0 && forward_pressed_[i] &&
            m.forward_limit_switch != LimitSwitch::kDisabled) ||
           (volts < 0.0 && reverse_pressed_[i] &&
            m.reverse_limit_switch != LimitSwitch::kDisabled);
  }

  // The age of a signal last read at `read_at_us`, as PhoenixBackend's Age
  // reports it, below its UINT32_MAX "unknown".
  static uint32_t AgeUs(uint64_t now_us, uint64_t read_at_us) {
    return static_cast<uint32_t>(
        std::min<uint64_t>(now_us - std::min(now_us, read_at_us),
                           std::numeric_limits<uint32_t>::max() - 1));
  }

  // Motor i's last connected sample as PhoenixBackend reads a device whose
  // frames stopped then: its ages grow from it, its FaultField's limit and
  // limiter bits clear at kFaultFieldMaxAgeUs (its fault bits stay; none is
  // modelled here) and its temperature reads 0 at kTemperatureMaxAgeUs.
  MotorSample HeldSample(std::size_t i, uint64_t now_us) const {
    MotorSample out = held_[i];
    const uint32_t age_us = AgeUs(now_us, held_at_us_[i]);
    out.position_age_us = age_us;
    out.velocity_age_us = age_us;
    if (age_us >= kFaultFieldMaxAgeUs) out.fault_flags &= ~kMotorNotFaultFlags;
    if (age_us >= kTemperatureMaxAgeUs) out.temperature_c = 0;
    return out;
  }

  // The FaultField bits of motor i's asserted, enabled limit switches; a
  // follower's own switches are not modelled.
  uint32_t HardLimitFlags(std::size_t i) const {
    const auto& m = config_.motors[i];
    if (m.master_id != 0) return 0;
    uint32_t flags = 0;
    if (forward_pressed_[i] && m.forward_limit_switch != LimitSwitch::kDisabled)
      flags |= kMotorForwardHardLimitFlag;
    if (reverse_pressed_[i] && m.reverse_limit_switch != LimitSwitch::kDisabled)
      flags |= kMotorReverseHardLimitFlag;
    return flags;
  }

  // Resolves mechanisms_ by master motor CAN id against config_. Follower
  // motors and non-positive / non-finite inertias are ignored.
  void BindMechanisms() {
    bound_mechanisms_.fill(BoundMechanism{});
    for (const MechanismSpec& spec : mechanisms_) {
      if (!std::isfinite(spec.inertia_kg_m2) || spec.inertia_kg_m2 <= 0.0) {
        continue;
      }
      const double viscous = std::isfinite(spec.viscous_damping_nm_per_radps) &&
                                     spec.viscous_damping_nm_per_radps > 0.0
                                 ? spec.viscous_damping_nm_per_radps
                                 : 0.0;
      const double static_fric = std::isfinite(spec.static_friction_nm) &&
                                         spec.static_friction_nm > 0.0
                                     ? spec.static_friction_nm
                                     : 0.0;
      for (std::size_t i = 0; i < config_.motors.size(); ++i) {
        if (config_.motors[i].can_id == static_cast<int>(spec.motor_can_id) &&
            config_.motors[i].master_id == 0) {
          bound_mechanisms_[i] = {true, spec.inertia_kg_m2, spec.load, viscous,
                                  static_fric};
          break;
        }
      }
    }
  }

  void ApplyRestingBattery() {
    const double v_oc = battery_model_.nominal_voltage_v;
    const double r_int = battery_model_.internal_resistance_ohm;
    const double base_a = battery_model_.base_current_a;
    const bool sag_active = r_int > 0.0;
    supply_v_ = std::clamp(v_oc - r_int * base_a, 0.0, 16.0);
    brownout_ = explicit_brownout_ ||
                (sag_active && supply_v_ < battery_model_.brownout_voltage_v);
  }

  bool DynamicChassisActive() const {
    return chassis_dynamics_.mass_kg > 0.0 && !modules_.empty();
  }

  // Effective winding voltage in motor i's configured direction at an explicit
  // bus voltage and physical rotor velocity.
  double LimitWindingVoltageAt(std::size_t i, const MotorConfig& m,
                               double volts, double supply_v,
                               double rotor_velocity_rps) const {
    const MotorModel motor = MotorModelFor(m.motor_type.value);
    const double dir = Direction(i);
    const double supply = std::max(0.0, supply_v);
    const double ceiling =
        std::min(m.max_voltage > 0.0 ? m.max_voltage : 12.0, supply);
    double rotor_volts = std::clamp(dir * volts, -ceiling, ceiling);
    const double back_emf_v = motor.back_emf_v_per_rps() * rotor_velocity_rps;
    const double resistance_ohm = motor.resistance_ohm();
    if (m.stator_limit_a > 0.0) {
      const double max_drop_v = m.stator_limit_a * resistance_ohm;
      rotor_volts = std::clamp(rotor_volts, back_emf_v - max_drop_v,
                               back_emf_v + max_drop_v);
      rotor_volts = std::clamp(rotor_volts, -ceiling, ceiling);
    }
    if (m.supply_limit_a > 0.0 && supply > 0.0) {
      const double max_power_drop = m.supply_limit_a * supply * resistance_ohm;
      const double draw_drop = rotor_volts * (rotor_volts - back_emf_v);
      if (draw_drop > max_power_drop) {
        const double root =
            std::sqrt(back_emf_v * back_emf_v + 4.0 * max_power_drop);
        rotor_volts = rotor_volts > back_emf_v ? 0.5 * (back_emf_v + root)
                                               : 0.5 * (back_emf_v - root);
        rotor_volts = std::clamp(rotor_volts, -ceiling, ceiling);
      }
    }
    return dir * rotor_volts;
  }

  // Effective winding voltage in motor i's configured direction after
  // enforcing the voltage ceiling, stator_limit_a (|V_rotor - back_emf| / R <=
  // stator_limit_a), and supply_limit_a (power drawn from supply_v_ capped to
  // supply_limit_a * supply_v_). Fed into StepAxis and recorded in
  // applied_volts_ so rotor dynamics, voltage, stator current, and supply
  // current stay mutually consistent.
  double LimitWindingVoltage(std::size_t i, const MotorConfig& m,
                             double volts) const {
    return LimitWindingVoltageAt(i, m, volts, supply_v_,
                                 axes_[i].rotor_velocity_rps);
  }

  // Signed stator current in motor i's physical rotor frame at explicit
  // applied voltage and rotor velocity.
  double SignedStatorCurrentAt(std::size_t i, const MotorConfig& m,
                               double applied_v, bool powered,
                               double rotor_velocity_rps) const {
    if (!powered) return 0.0;
    const MotorModel motor = MotorModelFor(m.motor_type.value);
    const double back_emf_v = motor.back_emf_v_per_rps() * rotor_velocity_rps;
    const double rotor_volts = Direction(i) * applied_v;
    return (rotor_volts - back_emf_v) / motor.resistance_ohm();
  }

  // Signed stator current in motor i's physical rotor frame:
  // (rotor_volts - back_emf) / R, with R = 12 V / stall current and
  // k_e = (12 V - free current * R) / free speed; 0 while unpowered.
  double SignedStatorCurrent(std::size_t i, const MotorConfig& m) const {
    return SignedStatorCurrentAt(i, m, applied_volts_[i], powered_[i],
                                 axes_[i].rotor_velocity_rps);
  }

  // Stator current magnitude from the actual applied_volts_ and back-EMF:
  // |rotor_volts - back_emf| / R. LimitWindingVoltage already enforces
  // stator_limit_a in dynamics when the voltage ceiling allows; no second
  // clamp is applied here, so overspeed or brownout current exceeding
  // stator_limit_a stays visible rather than hidden. At 12 V stall it equals
  // stall current; at 12 V free speed it equals free current; 0 while
  // unpowered.
  double StatorCurrent(std::size_t i, const MotorConfig& m) const {
    return std::abs(SignedStatorCurrent(i, m));
  }

  double SupplyCurrentAt(std::size_t i, const MotorConfig& m, double applied_v,
                         bool powered, double supply_v,
                         double rotor_velocity_rps) const {
    if (!powered || supply_v <= 0.0) return 0.0;
    const double rotor_volts = Direction(i) * applied_v;
    const double current_a =
        rotor_volts *
        SignedStatorCurrentAt(i, m, applied_v, powered, rotor_velocity_rps) /
        supply_v;
    return m.supply_limit_a > 0.0 ? std::min(m.supply_limit_a, current_a)
                                  : current_a;
  }

  // Signed power-conservation supply current:
  // (rotor_volts * signed_stator_current) / supply_v_ (positive = battery
  // draw, negative = regenerative braking). Positive draw is clamped to
  // supply_limit_a (matching LimitWindingVoltage, which limits draw and not
  // regen), while negative regen stays signed, matching Phoenix 6. 0 when
  // unpowered or supply_v_ <= 0.
  double SupplyCurrent(std::size_t i, const MotorConfig& m) const {
    return SupplyCurrentAt(i, m, applied_volts_[i], powered_[i], supply_v_,
                           axes_[i].rotor_velocity_rps);
  }

  double TorqueCurrentVoltsAt(std::size_t i, const MotorConfig& m,
                              double amperes, double supply_v,
                              double rotor_velocity_rps) const {
    const MotorModel motor = MotorModelFor(m.motor_type.value);
    const double peak_a =
        std::max(0.0, std::min(m.max_torque_current_a, m.stator_limit_a));
    const double current_a = std::clamp(amperes, -peak_a, peak_a);
    const double back_emf_v = motor.back_emf_v_per_rps() * rotor_velocity_rps;
    const double supply = std::max(0.0, supply_v);
    const double ceiling =
        std::min(m.max_voltage > 0.0 ? m.max_voltage : 12.0, supply);
    return std::clamp(
        current_a * motor.resistance_ohm() + Direction(i) * back_emf_v,
        -ceiling, ceiling);
  }

  // The volts TorqueCurrentFOC applies for a demand of `amperes`: the volts
  // that pass that stator current through the winding at the rotor's speed,
  // V = I R + back-EMF (same k_e and R as StatorCurrent), in the motor's
  // output direction. The current is clamped to the peak torque current
  // (max_torque_current_a) and the stator limit, the volts to max_voltage
  // and the supply, as the RoboRIO's duty cap of max_voltage over the
  // battery does. FOC itself is not modelled.
  double TorqueCurrentVolts(std::size_t i, const MotorConfig& m,
                            double amperes) const {
    return TorqueCurrentVoltsAt(i, m, amperes, supply_v_,
                                axes_[i].rotor_velocity_rps);
  }

  bool FeedbackValid(std::size_t i) const {
    if (!motor_connected_[i]) return false;
    if (config_.motors[i].feedback != Feedback::kRemoteCANcoder) return true;
    const int sensor = motor_to_sensor_[i];
    return sensor >= 0 && sensor_connected_[sensor];
  }

  // Resolves master and follower command voltages and current-limited winding
  // voltages at an explicit candidate bus voltage, without mutating backend
  // state. Returns the signed sum of motor supply currents (amperes).
  double ResolveCommandsAtSupply(
      double supply_v, const std::array<double, kMaxMotors>& profile_accel,
      const std::array<bool, kMaxMotors>& profiled,
      std::array<bool, kMaxMotors>& powered_out,
      std::array<double, kMaxMotors>& raw_volts_out,
      std::array<double, kMaxMotors>& applied_volts_out) const {
    powered_out.fill(false);
    raw_volts_out.fill(0.0);
    applied_volts_out.fill(0.0);
    std::array<bool, kMaxMotors> resolved{};
    const double supply = std::max(0.0, supply_v);
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      const auto& m = config_.motors[i];
      if (m.master_id != 0) continue;
      resolved[i] = true;
      double volts = 0.0;
      const double gear = Gear(i);
      const double mech_pos = MechPosition(i);
      const double mech_vel =
          axes_[i].rotor_velocity_rps * FeedbackDirection(i) / gear;
      const auto& req = commands_[i];
      bool want_power = false;
      if (profiled[i]) {
        const auto& profile = profiles_[i];
        const auto& gains = m.slots[req.slot < 3 ? req.slot : 0];
        const double velocity = profile.profile_vel_rps;
        const double sign = velocity > 0.0 ? 1.0 : velocity < 0.0 ? -1.0 : 0.0;
        const double ceiling =
            std::min(m.max_voltage > 0.0 ? m.max_voltage : 12.0, supply);
        const double pos_error =
            m.continuous_wrap
                ? std::remainder(profile.profile_pos_rot - mech_pos, 1.0)
                : (profile.profile_pos_rot - mech_pos);
        volts =
            std::clamp(req.feedforward_v + gains.p * pos_error +
                           gains.v * velocity + gains.a * profile_accel[i] +
                           gains.s * sign + gains.d * (velocity - mech_vel) +
                           GravityVoltage(gains, mech_pos),
                       -ceiling, ceiling);
        want_power = FeedbackValid(i);
      } else if (req.mode == Mode::kTorqueCurrent) {
        // Powered at any demand: 0 A is zero torque, not neutral.
        volts = TorqueCurrentVoltsAt(i, m, req.demand, supply,
                                     axes_[i].rotor_velocity_rps);
        want_power = FeedbackValid(i);
      } else {
        want_power = FeedbackValid(i) &&
                     DriveVoltage(req, m, mech_pos, mech_vel, supply, volts);
      }
      want_power = want_power && !LimitSwitchBlocks(i, volts);
      powered_out[i] = want_power;
      raw_volts_out[i] = want_power ? volts : 0.0;
      applied_volts_out[i] =
          want_power ? LimitWindingVoltageAt(i, m, volts, supply,
                                             axes_[i].rotor_velocity_rps)
                     : 0.0;
    }
    // Resolve follower chains before integrating any axis: configuration order
    // cannot introduce a one-step delay. Missing masters/cycles stay neutral.
    for (std::size_t pass = 0; pass < config_.motors.size(); ++pass) {
      bool progress = false;
      for (std::size_t i = 0; i < config_.motors.size(); ++i) {
        if (resolved[i]) continue;
        const auto& m = config_.motors[i];
        const int master = MotorIndex(m.master_id);
        if (master < 0 || !resolved[master]) continue;
        resolved[i] = true;
        progress = true;
        powered_out[i] = commands_[i].mode != Mode::kNeutral &&
                         FeedbackValid(i) && powered_out[master];
        const double follower_raw =
            applied_volts_out[master] * (m.oppose_master ? -1.0 : 1.0);
        raw_volts_out[i] = powered_out[i] ? follower_raw : 0.0;
        applied_volts_out[i] =
            powered_out[i] ? LimitWindingVoltageAt(i, m, follower_raw, supply,
                                                   axes_[i].rotor_velocity_rps)
                           : 0.0;
      }
      if (!progress) break;
    }
    double total_supply_a = 0.0;
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      total_supply_a +=
          SupplyCurrentAt(i, config_.motors[i], applied_volts_out[i],
                          powered_out[i], supply, axes_[i].rotor_velocity_rps);
    }
    return total_supply_a;
  }

  // Exact analytical solution of J * d(omega)/dt + B * omega = T over step h.
  // Unconditionally stable for all J > 0, B >= 0, h >= 0.
  static std::pair<double, double> ExponentialStep(double omega, double torque,
                                                   double damping,
                                                   double inertia, double h) {
    if (h <= 0.0) return {omega, 0.0};
    const double rate = damping / inertia;
    const double x = rate * h;
    if (x > 40.0 && damping > 0.0) {
      const double omega_inf = torque / damping;
      const double dtheta =
          omega_inf * h + (omega - omega_inf) * (inertia / damping);
      return {omega_inf, dtheta};
    }
    const double phi1 = x < 1e-5 ? 1.0 - x * (0.5 - x * (1.0 / 6.0 - x / 24.0))
                                 : -std::expm1(-x) / x;
    const double phi2 =
        x < 1e-4 ? 0.5 - x * (1.0 / 6.0 - x * (1.0 / 24.0 - x / 120.0))
                 : (1.0 - phi1) / x;
    const double accel0 = (torque - damping * omega) / inertia;
    const double omega_next = omega + accel0 * h * phi1;
    const double dtheta = omega * h + accel0 * h * h * phi2;
    return {omega_next, dtheta};
  }

  // Time in (0, rem_h) when J * d(omega)/dt + B * omega = T crosses
  // omega_target, given omega_target lies strictly between omega and
  // omega_cand.
  static double TimeToCrossOmega(double omega, double omega_target,
                                 double torque, double damping, double inertia,
                                 double rem_h) {
    const double rate = damping / inertia;
    if (rate * rem_h < 1e-5) {
      const double accel0 = (torque - damping * omega) / inertia;
      if (std::abs(accel0) > 1e-18) {
        return std::clamp((omega_target - omega) / accel0, 0.0, rem_h);
      }
      return rem_h;
    }
    const double omega_inf = torque / damping;
    const double denom = omega - omega_inf;
    if (std::abs(denom) <= 1e-18) return rem_h;
    const double ratio = (omega_target - omega_inf) / denom;
    if (ratio <= 0.0 || ratio >= 1.0) return rem_h;
    return std::clamp(-std::log(ratio) / rate, 0.0, rem_h);
  }

  // Integrates one rigid-body mechanism shaft (root master + all followers on
  // its chain) over dt_s using fixed sub-steps and unconditionally stable
  // exponential stepping with exact current-limit exit and static-friction
  // zero-crossing splits.
  void StepRigidBodyShaft(std::size_t root,
                          const std::array<double, kMaxMotors>& raw_volts,
                          double dt_s, std::array<bool, kMaxMotors>& stepped) {
    constexpr int kSubSteps = 10;
    const BoundMechanism& spec = bound_mechanisms_[root];
    const double gear = Gear(root);
    const double j_rotor = spec.inertia_kg_m2 / (gear * gear);
    const double b_ext = spec.viscous_damping_nm_per_radps / (gear * gear);
    const double tau_static = spec.static_friction_nm / gear;

    // Gather root + all followers in its chain without heap allocation.
    std::array<std::pair<std::size_t, double>, kMaxMotors> members{};
    std::size_t member_count = 0;
    members[member_count++] = {root, 1.0};
    stepped[root] = true;
    for (std::size_t k = 0; k < config_.motors.size(); ++k) {
      if (k == root || config_.motors[k].master_id == 0) continue;
      const auto [resolved_root, oppose_sign] = ResolveRootMaster(k);
      if (resolved_root == root) {
        members[member_count++] = {k, oppose_sign};
        stepped[k] = true;
      }
    }

    if (dt_s <= 0.0) {
      const bool stopped = EnforceHardStop(root);
      for (std::size_t m_idx = 1; m_idx < member_count; ++m_idx) {
        const auto [k, s_k] = members[m_idx];
        axes_[k].rotor_position_rot = s_k * axes_[root].rotor_position_rot;
        axes_[k].rotor_velocity_rps = s_k * axes_[root].rotor_velocity_rps;
      }
      if (stopped && powered_[root]) {
        const auto& m_root = config_.motors[root];
        if (commands_[root].mode == Mode::kTorqueCurrent) {
          applied_volts_[root] = LimitWindingVoltage(
              root, m_root,
              TorqueCurrentVolts(root, m_root, commands_[root].demand));
        } else {
          applied_volts_[root] =
              LimitWindingVoltage(root, m_root, raw_volts[root]);
        }
        for (std::size_t m_idx = 1; m_idx < member_count; ++m_idx) {
          const auto [k, s_k] = members[m_idx];
          if (powered_[k]) {
            applied_volts_[k] = LimitWindingVoltage(k, config_.motors[k],
                                                    applied_volts_[root] * s_k);
          }
        }
      }
      return;
    }

    const double h = dt_s / static_cast<double>(kSubSteps);
    const double supply = std::max(0.0, supply_v_);

    for (int sub = 0; sub < kSubSteps; ++sub) {
      double omega = axes_[root].rotor_velocity_rps * kTau;
      double dtheta_rot = 0.0;
      double rem_h = h;

      for (int seg = 0; seg < 4 && rem_h > 1e-12; ++seg) {
        double total_torque = 0.0;
        double total_damping = b_ext;
        double root_v_lim_cfg = 0.0;

        // Collect candidate current-limit exit speeds (rotor rad/s).
        std::array<double, kMaxMotors * 3> exit_omegas{};
        std::size_t exit_count = 0;
        bool any_current_limited = false;
        double max_ceil_omega = std::numeric_limits<double>::infinity();
        double min_ceil_omega = -std::numeric_limits<double>::infinity();

        for (std::size_t idx = 0; idx < member_count; ++idx) {
          const auto [k, s_k] = members[idx];
          const auto& mk = config_.motors[k];
          const MotorModel motor = MotorModelFor(mk.motor_type.value);
          const double r_ohm = motor.resistance_ohm();
          const double ke_rad = motor.back_emf_v_per_radps();
          const double kt = motor.torque_constant_nm_per_a();
          const double b_m = motor.no_load_damping_nm_per_radps();
          const double b_em = (kt * ke_rad) / r_ohm;
          const double omega_k = s_k * omega;
          const double rps_k = omega_k / kTau;

          total_damping += b_m;

          if (!powered_[k]) {
            if (mk.brake) {
              // Shorted-winding back-EMF braking (V = 0).
              total_damping += b_em;
            }
            // Coast neutral: I = 0, decays via b_m + b_ext + static_friction.
            continue;
          }

          const double ceiling =
              std::min(mk.max_voltage > 0.0 ? mk.max_voltage : 12.0, supply);
          const double dir_k = Direction(k);
          const bool is_tc =
              (k == root && commands_[k].mode == Mode::kTorqueCurrent);
          double cmd_v_cfg = 0.0;
          if (k == root) {
            cmd_v_cfg = is_tc ? TorqueCurrentVoltsAt(k, mk, commands_[k].demand,
                                                     supply, rps_k)
                              : raw_volts[k];
          } else {
            cmd_v_cfg = root_v_lim_cfg * s_k;
          }
          const double v_lim_cfg =
              LimitWindingVoltageAt(k, mk, cmd_v_cfg, supply, rps_k);
          if (k == root) root_v_lim_cfg = v_lim_cfg;

          const double v_lim_rotor = dir_k * v_lim_cfg;
          const double v_bemf = ke_rad * omega_k;
          double v_ceil_rotor =
              std::clamp(dir_k * cmd_v_cfg, -ceiling, ceiling);
          bool current_limited = false;
          if (is_tc) {
            const double peak_a = std::max(
                0.0, std::min(mk.max_torque_current_a, mk.stator_limit_a));
            const double i_req =
                std::clamp(commands_[k].demand, -peak_a, peak_a);
            if (std::abs(i_req) > 1e-12) {
              v_ceil_rotor = dir_k * (i_req > 0.0 ? ceiling : -ceiling);
              current_limited = std::abs(v_lim_rotor - v_ceil_rotor) > 1e-9;
            } else {
              // 0 A torque-current demand: zero electromagnetic torque unless
              // overspeeding past the voltage ceiling.
              if (std::abs(v_bemf) <= ceiling + 1e-9) {
                continue;
              }
              v_ceil_rotor = std::clamp(v_bemf, -ceiling, ceiling);
              current_limited = false;
            }
          } else {
            current_limited = std::abs(v_lim_rotor - v_ceil_rotor) > 1e-9;
          }

          if (!current_limited) {
            total_torque += s_k * (kt / r_ohm) * v_lim_rotor;
            total_damping += b_em;
          } else {
            any_current_limited = true;
            const double i_stator = (v_lim_rotor - v_bemf) / r_ohm;
            total_torque += s_k * kt * i_stator;

            if (ke_rad > 1e-12) {
              // Stator/TC ceiling exit: where back-EMF + I*R hits v_ceil_rotor.
              const double w_exit_stat =
                  s_k * (v_ceil_rotor - i_stator * r_ohm) / ke_rad;
              exit_omegas[exit_count++] = w_exit_stat;
              // Supply-limit exit: where v_ceil_rotor * (v_ceil_rotor - bemf)
              // equals supply_limit_a * supply * R.
              if (mk.supply_limit_a > 0.0 && supply > 0.0 &&
                  std::abs(v_ceil_rotor) > 1e-9) {
                const double p_max_r = mk.supply_limit_a * supply * r_ohm;
                const double v_bemf_sup = v_ceil_rotor - p_max_r / v_ceil_rotor;
                exit_omegas[exit_count++] = s_k * v_bemf_sup / ke_rad;
                // Stator-to-supply transition: where |v_lim * i_stator| = Pmax.
                if (std::abs(i_stator) > 1e-9) {
                  const double v_lim_s2s = p_max_r / (i_stator * r_ohm);
                  const double v_bemf_s2s = v_lim_s2s - i_stator * r_ohm;
                  exit_omegas[exit_count++] = s_k * v_bemf_s2s / ke_rad;
                }
              }
              const double w_ceil = s_k * v_ceil_rotor / ke_rad;
              if (s_k * i_stator > 0.0) {
                max_ceil_omega = std::min(max_ceil_omega, w_ceil);
              } else if (s_k * i_stator < 0.0) {
                min_ceil_omega = std::max(min_ceil_omega, w_ceil);
              }
            }
          }
        }

        // Static friction (Coulomb + stiction at zero velocity).
        bool stuck_at_zero = false;
        if (tau_static > 0.0) {
          if (std::abs(omega) < 1e-9) {
            if (std::abs(total_torque) <= tau_static) {
              omega = 0.0;
              stuck_at_zero = true;
            } else {
              total_torque -= std::copysign(tau_static, total_torque);
            }
          } else {
            total_torque -= std::copysign(tau_static, omega);
          }
        }
        if (stuck_at_zero) break;

        const auto [omega_cand, dtheta_cand] =
            ExponentialStep(omega, total_torque, total_damping, j_rotor, rem_h);

        // Find the earliest regime event inside (0, rem_h).
        double event_h = rem_h;
        bool is_zero_cross = false;
        double nudge = 0.0;

        if (tau_static > 0.0 && std::abs(omega) >= 1e-9 &&
            omega * omega_cand <= 0.0) {
          const double hz = TimeToCrossOmega(omega, 0.0, total_torque,
                                             total_damping, j_rotor, rem_h);
          if (hz < event_h) {
            event_h = hz;
            is_zero_cross = true;
            nudge = 0.0;
          }
        }

        if (any_current_limited) {
          const double delta_w = omega_cand - omega;
          for (std::size_t e = 0; e < exit_count; ++e) {
            const double w_e = exit_omegas[e];
            const bool between =
                (delta_w > 0.0 && w_e > omega + 1e-7 && w_e < omega_cand) ||
                (delta_w < 0.0 && w_e < omega - 1e-7 && w_e > omega_cand);
            if (!between) continue;
            const double he = TimeToCrossOmega(omega, w_e, total_torque,
                                               total_damping, j_rotor, rem_h);
            if (he < event_h) {
              event_h = he;
              is_zero_cross = false;
              nudge = std::copysign(1e-6, delta_w);
            }
          }
        }

        if (event_h < rem_h && seg < 3) {
          const auto [w_step, dth_step] = ExponentialStep(
              omega, total_torque, total_damping, j_rotor, event_h);
          dtheta_rot += dth_step / kTau;
          omega = is_zero_cross ? 0.0 : (w_step + nudge);
          rem_h -= event_h;
        } else {
          double w_final = omega_cand;
          if (any_current_limited) {
            if (omega <= max_ceil_omega && w_final > max_ceil_omega) {
              w_final = max_ceil_omega;
            }
            if (omega >= min_ceil_omega && w_final < min_ceil_omega) {
              w_final = min_ceil_omega;
            }
          }
          dtheta_rot += dtheta_cand / kTau;
          omega = w_final;
          rem_h = 0.0;
        }
      }

      if (!powered_[root] && std::abs(omega) < 1e-9) {
        omega = 0.0;
      }
      axes_[root].rotor_position_rot += dtheta_rot;
      axes_[root].rotor_velocity_rps = omega / kTau;
      EnforceHardStop(root);
      for (std::size_t idx = 1; idx < member_count; ++idx) {
        const auto [k, s_k] = members[idx];
        axes_[k].rotor_position_rot = s_k * axes_[root].rotor_position_rot;
        axes_[k].rotor_velocity_rps = s_k * axes_[root].rotor_velocity_rps;
      }
    }

    // Re-evaluate final applied_volts_ at the post-step rotor velocity so
    // reported voltage, stator current, and supply current are mutually
    // consistent.
    if (powered_[root]) {
      const auto& m_root = config_.motors[root];
      if (commands_[root].mode == Mode::kTorqueCurrent) {
        applied_volts_[root] = LimitWindingVoltage(
            root, m_root,
            TorqueCurrentVolts(root, m_root, commands_[root].demand));
      } else {
        applied_volts_[root] =
            LimitWindingVoltage(root, m_root, raw_volts[root]);
      }
    }
    for (std::size_t idx = 1; idx < member_count; ++idx) {
      const auto [k, s_k] = members[idx];
      if (powered_[k]) {
        applied_volts_[k] = LimitWindingVoltage(k, config_.motors[k],
                                                applied_volts_[root] * s_k);
      }
    }
  }

  void StepModels(double dt_s) {
    if (dt_s < 0.0) return;

    // Advance Motion Magic profiles once per tick before battery sag iteration
    // (profile kinematics are independent of supply voltage).
    std::array<double, kMaxMotors> profile_accel{};
    std::array<bool, kMaxMotors> profiled{};
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      const auto& m = config_.motors[i];
      if (m.master_id != 0) continue;
      const auto& req = commands_[i];
      if (req.mode == Mode::kMotionMagic && m.cruise_velocity_rps > 0.0 &&
          m.acceleration_rps2 > 0.0) {
        const double gear = Gear(i);
        const double mech_pos = MechPosition(i);
        const double mech_vel =
            axes_[i].rotor_velocity_rps * FeedbackDirection(i) / gear;
        auto& profile = profiles_[i];
        if (!profile.active) profile = {mech_pos, mech_vel, true};
        const double goal =
            m.continuous_wrap
                ? profile.profile_pos_rot +
                      std::remainder(req.demand - profile.profile_pos_rot, 1.0)
                : req.demand;
        profile_accel[i] = AdvanceProfile(profile, m, goal, dt_s);
        profiled[i] = true;
      } else {
        profiles_[i].active = false;
      }
    }

    // Solve battery sag & auto-brownout when no explicit supply override is
    // active. Bisection on [0, V_oc] converges deterministically in 28 steps
    // with zero oscillation even when equivalent load resistance < R_int.
    std::array<double, kMaxMotors> raw_volts{};
    if (!supply_voltage_override_) {
      const double v_oc = battery_model_.nominal_voltage_v;
      const double r_int = battery_model_.internal_resistance_ohm;
      const double base_a = battery_model_.base_current_a;
      if (r_int > 0.0) {
        std::array<bool, kMaxMotors> tmp_powered{};
        std::array<double, kMaxMotors> tmp_raw{};
        std::array<double, kMaxMotors> tmp_applied{};
        double v_solved = 0.0;
        if (r_int * base_a < v_oc) {
          const double motor_a_at_voc = ResolveCommandsAtSupply(
              v_oc, profile_accel, profiled, tmp_powered, tmp_raw, tmp_applied);
          if (base_a + motor_a_at_voc <= 0.0) {
            v_solved = v_oc;
          } else {
            double lo = 0.0;
            double hi = v_oc;
            for (int iter = 0; iter < 28; ++iter) {
              const double mid = 0.5 * (lo + hi);
              const double motor_a =
                  ResolveCommandsAtSupply(mid, profile_accel, profiled,
                                          tmp_powered, tmp_raw, tmp_applied);
              const double i_total = std::max(0.0, base_a + motor_a);
              const double residual = mid + r_int * i_total - v_oc;
              if (residual < 0.0) {
                lo = mid;
              } else {
                hi = mid;
              }
            }
            v_solved = 0.5 * (lo + hi);
          }
        }
        supply_v_ = std::clamp(v_solved, 0.0, 16.0);
        brownout_ = explicit_brownout_ ||
                    (supply_v_ < battery_model_.brownout_voltage_v);
      } else {
        supply_v_ = v_oc;
        brownout_ = explicit_brownout_;
      }
    }
    ResolveCommandsAtSupply(supply_v_, profile_accel, profiled, powered_,
                            raw_volts, applied_volts_);

    std::array<bool, kMaxMotors> stepped{};
    const bool dynamic_chassis = DynamicChassisActive();
    if (dynamic_chassis) {
      // Dynamic swerve drive motors are integrated together with chassis mass
      // and wheel traction in StepChassisDynamic().
      for (const auto& module : modules_) {
        stepped[module.drive] = true;
      }
    }

    // Step rigid-body mechanisms (master + followers on shared shaft).
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      if (stepped[i] || config_.motors[i].master_id != 0) continue;
      if (bound_mechanisms_[i].active) {
        StepRigidBodyShaft(i, raw_volts, dt_s, stepped);
      }
    }

    // Legacy StepAxis for all motors without a MechanismSpec (and not driven
    // by dynamic chassis).
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      if (stepped[i]) continue;
      const auto& m = config_.motors[i];
      const MotorModel motor = MotorModelFor(m.motor_type.value);
      MechanismModel mech;
      mech.gear_ratio = Gear(i);
      mech.brake = m.brake;
      const double volts = applied_volts_[i];
      // A follower's applied volts are its master's, reversed if opposed, so
      // this is OutputDirection(i) times the master's volts.
      const double rotor_volts = Direction(i) * volts;
      axes_[i] = StepAxis(motor, mech, axes_[i], rotor_volts, powered_[i],
                          supply_v_, dt_s);
      const bool stopped = EnforceHardStop(i);
      if (powered_[i]) {
        if (commands_[i].mode == Mode::kTorqueCurrent && m.master_id == 0) {
          applied_volts_[i] = LimitWindingVoltage(
              i, m, TorqueCurrentVolts(i, m, commands_[i].demand));
        } else if (stopped) {
          applied_volts_[i] = LimitWindingVoltage(i, m, applied_volts_[i]);
        }
      }
    }

    if (dynamic_chassis) {
      StepChassisDynamic(raw_volts, dt_s);
    } else {
      StepChassis(dt_s);
    }
    elapsed_s_ += dt_s;
    if (gyro_override_) {
      yaw_rot_ += yaw_rate_rps_ * dt_s;
    } else {
      yaw_rot_ = (truth_.yaw_rad - gyro_zero_yaw_rad_ +
                  options_.gyro_drift_radps * elapsed_s_) /
                 kTau;
      yaw_rate_rps_ = (truth_.omega_radps + options_.gyro_drift_radps) / kTau;
    }
  }

  // Coupled 2D rigid-body swerve chassis + wheel friction-circle dynamics.
  void StepChassisDynamic(const std::array<double, kMaxMotors>& raw_volts,
                          double dt_s) {
    if (dt_s <= 0.0 || modules_.empty()) return;
    constexpr double kGravityMps2 = 9.80665;
    constexpr double kDefaultWheelInertiaKgM2 = 0.004;
    constexpr int kSubSteps = 10;

    const std::size_t n_mod = modules_.size();
    const double n_mod_d = static_cast<double>(n_mod);
    const double mass = chassis_dynamics_.mass_kg;
    const double r_true =
        std::max(1e-6, options_.wheel_radius_m *
                           (1.0 + options_.wheel_radius_error_frac));

    double moi = chassis_dynamics_.moment_of_inertia_kg_m2;
    if (moi <= 0.0) {
      double mean_r2 = 0.0;
      for (const auto& mod : modules_) {
        mean_r2 += mod.x_m * mod.x_m + mod.y_m * mod.y_m;
      }
      mean_r2 /= n_mod_d;
      moi = std::max(1e-6, mass * mean_r2);
    }

    const bool has_traction_limit =
        chassis_dynamics_.traction_coefficient > 0.0;
    const double f_max = has_traction_limit
                             ? chassis_dynamics_.traction_coefficient * mass *
                                   kGravityMps2 / n_mod_d
                             : std::numeric_limits<double>::infinity();

    const double h = dt_s / static_cast<double>(kSubSteps);
    const double supply = std::max(0.0, supply_v_);

    double vx = chassis_vx_body_mps_;
    double vy = chassis_vy_body_mps_;
    double omega = chassis_omega_radps_;

    for (int sub = 0; sub < kSubSteps; ++sub) {
      double fx_total = 0.0;
      double fy_total = 0.0;
      double tau_z_total = 0.0;
      double d_xx = 0.0;
      double d_yy = 0.0;
      double d_ww = 0.0;

      // Per-module sub-step workspace (fixed stack arrays, zero heap alloc).
      std::array<bool, kMaxMotors> slipping{};
      std::array<bool, kMaxMotors> curr_limited{};
      std::array<double, kMaxMotors> tau_wheel_arr{};
      std::array<double, kMaxMotors> f_long_arr{};
      std::array<double, kMaxMotors> j_wheel_arr{};
      std::array<double, kMaxMotors> b_wheel_em_arr{};
      std::array<double, kMaxMotors> v_long_old_arr{};
      std::array<double, kMaxMotors> v_wheel_old_arr{};
      std::array<double, kMaxMotors> w_ceil_wheel_arr{};
      std::array<double, kMaxMotors> ux_arr{};
      std::array<double, kMaxMotors> uy_arr{};

      for (std::size_t idx = 0; idx < n_mod; ++idx) {
        const auto& mod = modules_[idx];
        const std::size_t d = mod.drive;
        const auto& md = config_.motors[d];
        const MotorModel motor = MotorModelFor(md.motor_type.value);
        const double r_ohm = motor.resistance_ohm();
        const double ke_rad = motor.back_emf_v_per_radps();
        const double kt = motor.torque_constant_nm_per_a();
        const double b_m = motor.no_load_damping_nm_per_radps();
        const double b_em = (kt * ke_rad) / r_ohm;
        const double dir_d = Direction(d);
        const double gear_d = Gear(d);
        const double j_wheel = bound_mechanisms_[d].active
                                   ? bound_mechanisms_[d].inertia_kg_m2
                                   : kDefaultWheelInertiaKgM2;
        j_wheel_arr[idx] = j_wheel;

        const double angle = MechPosition(mod.steer) * kTau;
        const double ux = std::cos(angle);
        const double uy = std::sin(angle);
        const double vx_lat_dir = -uy;
        const double vy_lat_dir = ux;
        ux_arr[idx] = ux;
        uy_arr[idx] = uy;

        const double v_patch_x = vx - omega * mod.y_m;
        const double v_patch_y = vy + omega * mod.x_m;
        const double v_long = v_patch_x * ux + v_patch_y * uy;
        const double v_lat = v_patch_x * vx_lat_dir + v_patch_y * vy_lat_dir;
        v_long_old_arr[idx] = v_long;

        const double rps_d = axes_[d].rotor_velocity_rps;
        const double omega_r = rps_d * kTau;
        const double omega_wheel = dir_d * omega_r / gear_d;
        const double v_wheel = omega_wheel * r_true;
        v_wheel_old_arr[idx] = v_wheel;

        double tau_rotor = 0.0;
        bool is_cl = false;
        double wheel_em_damp = 0.0;
        double w_ceil_wheel = std::numeric_limits<double>::infinity();

        if (powered_[d]) {
          const double ceiling =
              std::min(md.max_voltage > 0.0 ? md.max_voltage : 12.0, supply);
          const bool is_tc = (commands_[d].mode == Mode::kTorqueCurrent);
          const double cmd_v =
              is_tc ? TorqueCurrentVoltsAt(d, md, commands_[d].demand, supply,
                                           rps_d)
                    : raw_volts[d];
          const double v_lim_cfg =
              LimitWindingVoltageAt(d, md, cmd_v, supply, rps_d);
          const double v_lim_rotor = dir_d * v_lim_cfg;
          const double v_bemf = ke_rad * omega_r;
          const double i_stator = (v_lim_rotor - v_bemf) / r_ohm;
          tau_rotor = kt * i_stator - b_m * omega_r;

          double v_ceil_rotor = std::clamp(dir_d * cmd_v, -ceiling, ceiling);
          if (is_tc) {
            const double peak_a = std::max(
                0.0, std::min(md.max_torque_current_a, md.stator_limit_a));
            const double i_req =
                std::clamp(commands_[d].demand, -peak_a, peak_a);
            v_ceil_rotor = dir_d * (i_req > 0.0   ? ceiling
                                    : i_req < 0.0 ? -ceiling
                                                  : 0.0);
          }
          is_cl = std::abs(v_lim_rotor - v_ceil_rotor) > 1e-9;
          wheel_em_damp = gear_d * gear_d * (is_cl ? b_m : (b_em + b_m));
          if (ke_rad > 1e-12) {
            w_ceil_wheel = std::abs(v_ceil_rotor / ke_rad) / gear_d;
          }
        } else {
          if (md.brake) {
            tau_rotor = -(b_em + b_m) * omega_r;
            wheel_em_damp = gear_d * gear_d * (b_em + b_m);
          } else {
            tau_rotor = -b_m * omega_r;
            wheel_em_damp = gear_d * gear_d * b_m;
          }
        }

        double tau_wheel = dir_d * gear_d * tau_rotor;
        if (bound_mechanisms_[d].active) {
          tau_wheel -=
              bound_mechanisms_[d].viscous_damping_nm_per_radps * omega_wheel;
          wheel_em_damp += bound_mechanisms_[d].viscous_damping_nm_per_radps;
          const double sf = bound_mechanisms_[d].static_friction_nm;
          if (sf > 0.0) {
            if (std::abs(omega_wheel) < 1e-9) {
              if (std::abs(tau_wheel) <= sf) {
                tau_wheel = 0.0;
              } else {
                tau_wheel -= std::copysign(sf, tau_wheel);
              }
            } else {
              tau_wheel -= std::copysign(sf, omega_wheel);
            }
          }
        }

        curr_limited[idx] = is_cl;
        tau_wheel_arr[idx] = tau_wheel;
        b_wheel_em_arr[idx] = wheel_em_damp;
        w_ceil_wheel_arr[idx] = w_ceil_wheel;

        // Longitudinal force: wheel drive torque plus any slip reconciliation
        // impulse when the wheel is currently spinning relative to ground.
        const double inv_m_eff_long =
            (r_true * r_true) / j_wheel + n_mod_d / mass;
        const double f_sync = (v_wheel - v_long) / (inv_m_eff_long * h);
        const double f_long_des = tau_wheel / r_true + f_sync;

        // Lateral scrub resistance opposing perpendicular patch velocity.
        const double lat_arm = mod.x_m * vy_lat_dir - mod.y_m * vx_lat_dir;
        const double inv_m_lat =
            n_mod_d * (1.0 / mass + (lat_arm * lat_arm) / moi);
        const double f_lat_des = -v_lat / (inv_m_lat * h);

        double f_long = f_long_des;
        double f_lat = f_lat_des;
        bool is_slip = false;
        if (has_traction_limit) {
          const double f_mag = std::hypot(f_long_des, f_lat_des);
          if (f_mag > f_max) {
            const double scale = f_max / f_mag;
            f_long = f_long_des * scale;
            f_lat = f_lat_des * scale;
            is_slip = true;
          }
        }
        slipping[idx] = is_slip;
        f_long_arr[idx] = f_long;

        const double fx_i = f_long * ux + f_lat * vx_lat_dir;
        const double fy_i = f_long * uy + f_lat * vy_lat_dir;
        fx_total += fx_i;
        fy_total += fy_i;
        tau_z_total += mod.x_m * fy_i - mod.y_m * fx_i;

        // Implicit linear back-EMF damping for non-slipping, voltage-clamped
        // wheels keeps integration unconditionally stable even at small mass.
        if (!is_slip && !is_cl) {
          const double c_long = wheel_em_damp / (r_true * r_true);
          const double long_arm = mod.x_m * uy - mod.y_m * ux;
          d_xx += c_long * ux * ux;
          d_yy += c_long * uy * uy;
          d_ww += c_long * long_arm * long_arm;
        }
      }

      const double vx_old = vx;
      const double vy_old = vy;
      const double omega_old = omega;

      const double vx_step = vx + (fx_total * h) / (mass + d_xx * h);
      const double vy_step = vy + (fy_total * h) / (mass + d_yy * h);
      const double omega_new = omega + (tau_z_total * h) / (moi + d_ww * h);

      // Exact constant-twist SE(2) pose step using midpoint twist over h.
      const double vx_avg = 0.5 * (vx_old + vx_step);
      const double vy_avg = 0.5 * (vy_old + vy_step);
      const double omega_avg = 0.5 * (omega_old + omega_new);
      const double d_yaw = omega_avg * h;
      const double sinc = std::abs(d_yaw) < 1e-6 ? 1.0 - d_yaw * d_yaw / 6.0
                                                 : std::sin(d_yaw) / d_yaw;
      const double cosc = std::abs(d_yaw) < 1e-6
                              ? d_yaw / 2.0 - d_yaw * d_yaw * d_yaw / 24.0
                              : (1.0 - std::cos(d_yaw)) / d_yaw;
      const double dx = h * (sinc * vx_avg - cosc * vy_avg);
      const double dy = h * (cosc * vx_avg + sinc * vy_avg);
      const double c_yaw = std::cos(truth_.yaw_rad);
      const double s_yaw = std::sin(truth_.yaw_rad);
      truth_.x_m += c_yaw * dx - s_yaw * dy;
      truth_.y_m += s_yaw * dx + c_yaw * dy;
      truth_.yaw_rad += d_yaw;

      // Rotate body velocity by -d_yaw so inertial field momentum is preserved
      // as the robot body frame rotates.
      const double cd = std::cos(d_yaw);
      const double sd = std::sin(d_yaw);
      vx = cd * vx_step + sd * vy_step;
      vy = -sd * vx_step + cd * vy_step;
      omega = omega_new;

      // Update each drive motor's wheel speed and integrate encoder position.
      for (std::size_t idx = 0; idx < n_mod; ++idx) {
        const auto& mod = modules_[idx];
        const std::size_t d = mod.drive;
        const double dir_d = Direction(d);
        const double gear_d = Gear(d);
        const double old_rps = axes_[d].rotor_velocity_rps;
        const double v_patch_x = vx - omega * mod.y_m;
        const double v_patch_y = vy + omega * mod.x_m;
        const double v_long_new =
            v_patch_x * ux_arr[idx] + v_patch_y * uy_arr[idx];

        double v_wheel_new = v_long_new;
        if (slipping[idx]) {
          // Excess torque spins the wheel independently of ground speed.
          const double omega_w_old = v_wheel_old_arr[idx] / r_true;
          const double tau_excess =
              tau_wheel_arr[idx] - f_long_arr[idx] * r_true;
          const double damp = curr_limited[idx] ? 0.0 : b_wheel_em_arr[idx];
          double omega_w_new =
              omega_w_old + (tau_excess * h) / (j_wheel_arr[idx] + damp * h);
          const double w_ceil = w_ceil_wheel_arr[idx];
          if (std::isfinite(w_ceil)) {
            if (omega_w_old <= w_ceil && omega_w_new > w_ceil) {
              omega_w_new = w_ceil;
            } else if (omega_w_old >= -w_ceil && omega_w_new < -w_ceil) {
              omega_w_new = -w_ceil;
            }
          }
          v_wheel_new = omega_w_new * r_true;
          // Clean re-adhesion if the slipping wheel crossed ground speed.
          const double slip_old = v_wheel_old_arr[idx] - v_long_old_arr[idx];
          const double slip_new = v_wheel_new - v_long_new;
          if (slip_old * slip_new < 0.0) {
            v_wheel_new = v_long_new;
          }
        }

        const double new_rps = (v_wheel_new / r_true) * gear_d / (dir_d * kTau);
        axes_[d].rotor_velocity_rps = new_rps;
        axes_[d].rotor_position_rot += 0.5 * (old_rps + new_rps) * h;
      }
    }

    chassis_vx_body_mps_ = vx;
    chassis_vy_body_mps_ = vy;
    chassis_omega_radps_ = omega;
    const double c_end = std::cos(truth_.yaw_rad);
    const double s_end = std::sin(truth_.yaw_rad);
    truth_.vx_mps = c_end * vx - s_end * vy;
    truth_.vy_mps = s_end * vx + c_end * vy;
    truth_.omega_radps = omega;

    // Update final applied_volts_ on each dynamic drive motor for consistent
    // voltage/current reporting.
    for (auto& mod : modules_) {
      const std::size_t d = mod.drive;
      mod.previous_surface_speed_mps =
          axes_[d].rotor_velocity_rps * Direction(d) / Gear(d) * kTau * r_true;
      if (powered_[d]) {
        const auto& md = config_.motors[d];
        if (commands_[d].mode == Mode::kTorqueCurrent && md.master_id == 0) {
          applied_volts_[d] = LimitWindingVoltage(
              d, md, TorqueCurrentVolts(d, md, commands_[d].demand));
        } else {
          applied_volts_[d] = LimitWindingVoltage(d, md, raw_volts[d]);
        }
      }
    }
  }

  void StepChassis(double dt_s) {
    if (dt_s <= 0.0) return;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_vx = 0.0;
    double sum_vy = 0.0;
    double moment = 0.0;
    double lever_squared = 0.0;
    for (auto& module : modules_) {
      const double surface_speed =
          axes_[module.drive].rotor_velocity_rps * Direction(module.drive) /
          Gear(module.drive) * kTau * options_.wheel_radius_m *
          (1.0 + options_.wheel_radius_error_frac);
      const double acceleration =
          (surface_speed - module.previous_surface_speed_mps) / dt_s;
      module.previous_surface_speed_mps = surface_speed;
      const double slip = std::min(
          1.0, options_.wheel_slip_gain * std::abs(acceleration) / 9.80665);
      const double speed = surface_speed * (1.0 - slip);
      const double angle = MechPosition(module.steer) * kTau;
      const double vx = speed * std::cos(angle);
      const double vy = speed * std::sin(angle);
      sum_x += module.x_m;
      sum_y += module.y_m;
      sum_vx += vx;
      sum_vy += vy;
      moment += module.x_m * vy - module.y_m * vx;
      lever_squared += module.x_m * module.x_m + module.y_m * module.y_m;
    }
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
    if (!modules_.empty()) {
      // Eliminate translation from the rigid-body least-squares equations
      // vx - y*omega = wheel_vx, vy + x*omega = wheel_vy. Centering is
      // essential for module layouts whose centroid is not the robot origin.
      const double n = static_cast<double>(modules_.size());
      const double centered_lever =
          lever_squared - (sum_x * sum_x + sum_y * sum_y) / n;
      if (centered_lever > 1e-12) {
        omega =
            (moment - (sum_x * sum_vy - sum_y * sum_vx) / n) / centered_lever;
      }
      vx = (sum_vx + sum_y * omega) / n;
      vy = (sum_vy - sum_x * omega) / n;
    }
    // Exact constant-twist integration over this step avoids an arc's
    // translation depending on the gateway tick size.
    const double angle = omega * dt_s;
    const double sinc = std::abs(angle) < 1e-6 ? 1.0 - angle * angle / 6.0
                                               : std::sin(angle) / angle;
    const double cosc = std::abs(angle) < 1e-6
                            ? angle / 2.0 - angle * angle * angle / 24.0
                            : (1.0 - std::cos(angle)) / angle;
    const double dx = dt_s * (sinc * vx - cosc * vy);
    const double dy = dt_s * (cosc * vx + sinc * vy);
    const double c = std::cos(truth_.yaw_rad);
    const double s = std::sin(truth_.yaw_rad);
    truth_.x_m += c * dx - s * dy;
    truth_.y_m += s * dx + c * dy;
    truth_.yaw_rad += angle;
    truth_.vx_mps =
        std::cos(truth_.yaw_rad) * vx - std::sin(truth_.yaw_rad) * vy;
    truth_.vy_mps =
        std::sin(truth_.yaw_rad) * vx + std::cos(truth_.yaw_rad) * vy;
    truth_.omega_radps = omega;
  }

  Config config_;
  std::size_t can_bus_count_{};
  PhysicsOptions options_;
  WorldState truth_;
  double gyro_zero_yaw_rad_{};
  double elapsed_s_{};
  std::optional<uint64_t> previous_time_;
  std::array<AxisState, kMaxMotors> axes_{};
  std::array<Profile, kMaxMotors> profiles_{};
  std::vector<Module> modules_;
  bool gyro_override_{false};
  std::array<double, kMaxMotors> applied_volts_{};
  std::array<bool, kMaxMotors> powered_{};
  std::array<bool, kMaxMotors> frozen_{};
  std::array<MotorSample, kMaxMotors> held_{};
  std::array<uint64_t, kMaxMotors> held_at_us_{};
  std::array<bool, kMaxMotors> motor_connected_{};
  std::array<bool, kMaxMotors> forward_pressed_{};
  std::array<bool, kMaxMotors> reverse_pressed_{};
  std::array<bool, kMaxSensors> sensor_connected_{};
  // When each sensor last read connected (sample time, µs).
  std::array<uint64_t, kMaxSensors> sensor_read_at_us_{};
  // Filled with -1 by Configure; valid only after Configure.
  std::array<int, kMaxMotors> motor_to_sensor_;
  std::unordered_map<uint16_t, bool> digital_inputs_{};
  std::unordered_map<uint16_t, double> analog_inputs_{};
  std::unordered_map<uint16_t, ManualEncoder> encoders_{};
  // By motor name; SetLocalMotorTypes.
  std::unordered_map<std::string, MotorType> local_motor_types_{};
  // SetHomeSwitches, and those of them config_ names, by index.
  struct BoundHomeSwitch {
    std::size_t input;
    std::size_t motor;
    double closed_at_rot;
    bool closed_above;
    bool closed_reads;
  };
  std::vector<HomeSwitch> home_switches_;
  std::vector<BoundHomeSwitch> bound_home_switches_;
  struct BoundHardStop {
    std::optional<double> min_rot;
    std::optional<double> max_rot;
  };
  std::vector<HardStop> hard_stops_;
  std::array<BoundHardStop, kMaxMotors> bound_hard_stops_{};
  struct BoundMechanism {
    bool active{false};
    double inertia_kg_m2{0.0};
    LoadKind load{LoadKind::kRoller};
    double viscous_damping_nm_per_radps{0.0};
    double static_friction_nm{0.0};
  };
  std::vector<MechanismSpec> mechanisms_;
  std::array<BoundMechanism, kMaxMotors> bound_mechanisms_{};
  BatteryModel battery_model_{};
  std::optional<double> supply_voltage_override_;
  bool explicit_brownout_{false};
  ChassisDynamics chassis_dynamics_{};
  double chassis_vx_body_mps_{0.0};
  double chassis_vy_body_mps_{0.0};
  double chassis_omega_radps_{0.0};
  std::vector<SwerveModuleSpec> configured_modules_;
  double supply_v_{kSimRestingSupplyV};
  double yaw_rot_{0.0};
  double yaw_rate_rps_{0.0};
};

}  // namespace talos::hardware::sim

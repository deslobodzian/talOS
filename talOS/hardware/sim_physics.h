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
// type, gear ratio, inertia, friction, and load (roller/flywheel first). The
// RoboRIO PhoenixBackend keeps its own first-order Kraken model (same 100 rps
// at 12 V family); the GradleRIO build excludes sim_* headers (see
// build.gradle).
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
// in the configured positive direction; CANcoders and the chassis see the
// physical rotor. A follower ignores its own `inverted`, as under Phoenix 6's
// Follower request: it drives in its master's configured direction, reversed
// by `oppose_master`, and reports in its master's frame, as PhoenixBackend
// reports it. A motor whose feedback is a remote CANcoder reports, and closes
// its loops on, that CANcoder's reading in the CANcoder's own direction, so a
// motor/CANcoder direction mismatch runs away here as on a robot.
//
// Sensors cross the hardware boundary. CANcoder samples mirror the linked
// mechanism, the position wrapped to [-0.5, 0.5) rotations as the RoboRIO
// reports it (CANcoderAbsoluteRot). Swerve ground motion determines the true
// field pose; the Pigeon reports true yaw plus optional drift, unless
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
};

// The published 12 V free speed, stall current and free current of each motor
// type (CTRE manufacturer specifications / ReCalc, reca.lc):
//   Kraken X60: 6000 rpm, 366 A stall, 2.0 A free
//   Kraken X44: 7530 rpm, 275 A stall, 2.0 A free
//   Falcon 500: 6380 rpm, 257 A stall, 1.5 A free
inline MotorModel MotorModelFor(MotorType type) {
  MotorModel model;
  switch (type) {
    case MotorType::kKrakenX60:
      model.free_speed_rps_at_12v = 6000.0 / 60.0;
      model.stall_current_a_at_12v = 366.0;
      model.free_current_a_at_12v = 2.0;
      break;
    case MotorType::kKrakenX44:
      model.free_speed_rps_at_12v = 7530.0 / 60.0;
      model.stall_current_a_at_12v = 275.0;
      model.free_current_a_at_12v = 2.0;
      break;
    case MotorType::kFalcon500:
      model.free_speed_rps_at_12v = 6380.0 / 60.0;
      model.stall_current_a_at_12v = 257.0;
      model.free_current_a_at_12v = 1.5;
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
  const double supply = supply_v > 0.0 ? supply_v : 12.0;
  const double free_rotor = motor.free_speed_rps_at_12v * (supply / 12.0);
  const double inertia_scale =
      mech.inertia_kg_m2 > 0.0
          ? mech.inertia_kg_m2 / motor.nominal_inertia_kg_m2
          : 1.0;
  double tau = motor.time_constant_s * std::max(0.1, inertia_scale);
  double target = 0.0;
  if (powered) {
    const double bemf_slope = free_rotor > 0.0 ? supply / free_rotor : 0.0;
    const double damping_v_per_rps =
        bemf_slope + motor.viscous_friction_vs_per_rps;
    target = damping_v_per_rps > 0.0 ? volts / damping_v_per_rps
                                     : (volts / supply) * free_rotor;
    if (std::abs(out.rotor_velocity_rps) < 1e-9 &&
        std::abs(volts) < motor.static_friction_v)
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
  const double ceiling =
      std::min(cfg.max_voltage > 0.0 ? cfg.max_voltage : 12.0,
               supply_v > 0.0 ? supply_v : 12.0);
  const auto clamp = [](double v, double lim) {
    return std::clamp(v, -lim, lim);
  };
  switch (req.mode) {
    case Mode::kNeutral:
      return false;
    case Mode::kDutyCycle:
      volts_out = clamp(req.demand * supply_v, ceiling);
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
      const double error = req.demand - mech_pos;
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

// Physics-backed Backend. See the header comment for the item-by-item map.
class PhysicsSimBackend final : public Backend {
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
      local_motor_types_[m.name] = m.motor_type.value;
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

  bool Configure(const Config& config) override {
    config_ = config;
    can_bus_count_ = CanBuses(config_).count;
    for (auto& m : config_.motors) {
      if (const auto it = local_motor_types_.find(m.name);
          it != local_motor_types_.end()) {
        m.motor_type.value = it->second;
      }
    }
    BindHomeSwitches();
    BindHardStops();
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
    supply_v_ = kSimRestingSupplyV;
    yaw_rot_ = 0.0;
    yaw_rate_rps_ = 0.0;
    truth_ = {options_.start_x_m, options_.start_y_m, options_.start_yaw_rad};
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
    ReadPower(state.health, {true, supply_v_, brownout_});
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
      const double mech_pos =
          axes_[i].rotor_position_rot * FeedbackDirection(i) / gear;
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
        // CANcoder mirrors its linked mechanism through the boundary.
        out.valid = true;
        out.position_rot = 0;
        out.velocity_rps = 0;
        for (std::size_t j = 0; j < config_.motors.size(); ++j) {
          if (motor_to_sensor_[j] == static_cast<int>(i) &&
              motor_connected_[j] && !frozen_[j]) {
            const double gear = Gear(j);
            out.position_rot = axes_[j].rotor_position_rot / gear;
            out.velocity_rps = axes_[j].rotor_velocity_rps / gear;
            break;
          }
        }
        if (s.inverted) {
          out.position_rot = -out.position_rot;
          out.velocity_rps = -out.velocity_rps;
        }
        out.position_rot = CANcoderAbsoluteRot(out.position_rot);
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
      const double position_rot = axes_[s.motor].rotor_position_rot *
                                  FeedbackDirection(s.motor) / Gear(s.motor);
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

  bool Apply(std::span<const MotorRequest> commands) override {
    std::copy(commands.begin(), commands.end(), commands_.begin());
    return true;
  }
  std::span<const uint16_t> ApplyFailedDevices() const override { return {}; }

  bool ApplyOutputs(std::span<const DigitalOutputRequest> digital_outputs,
                    std::span<const PwmRequest> pwm_outputs) override {
    for (const auto& d : digital_outputs) digital_outputs_[d.id] = d.value;
    for (const auto& p : pwm_outputs) pwm_outputs_[p.id] = p;
    return true;
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
  void SimulateDeviceReset(uint16_t id) { pending_resets_.push_back(id); }
  bool PollDeviceResets(std::vector<uint16_t>& reset_ids) override {
    reset_ids = pending_resets_;
    pending_resets_.clear();
    return !reset_ids.empty();
  }
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
  RestoreStatus PollRestore() override {
    if (restore_fails_) return RestoreStatus::kFailed;
    restoring_.fill(false);
    return RestoreStatus::kDone;
  }
  void SetRestoreFails(bool fails) { restore_fails_ = fails; }
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
    supply_v_ = std::clamp(volts, 0.0, 16.0);
  }
  double supply_voltage() const { return supply_v_; }
  void SetPowerHealth(double battery_v, bool brownout) {
    SetSupplyVoltage(battery_v);
    brownout_ = brownout;
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
    yaw_rot_ = 0.0;
    yaw_rate_rps_ = 0.0;
    gyro_zero_yaw_rad_ = truth_.yaw_rad;
    elapsed_s_ = 0.0;
    truth_.vx_mps = truth_.vy_mps = truth_.omega_radps = 0.0;
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
  bool GetDigitalOutput(uint16_t id) const {
    const auto it = digital_outputs_.find(id);
    return it != digital_outputs_.end() ? it->second : false;
  }
  // As SimBackend::GetPwm: off ({.id = id}) when none since Neutral().
  PwmRequest GetPwm(uint16_t id) const {
    const auto it = pwm_outputs_.find(id);
    return it != pwm_outputs_.end() ? it->second : PwmRequest{.id = id};
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
  // Replanning from the current setpoint also handles demand changes.
  double AdvanceProfile(Profile& p, const MotorConfig& cfg, double goal,
                        double dt) {
    if (dt <= 0.0) return 0.0;
    const double direction = p.profile_pos_rot > goal ? -1.0 : 1.0;
    const double pos = p.profile_pos_rot * direction;
    const double vel = p.profile_vel_rps * direction;
    const double accel = cfg.acceleration_rps2;
    const double cruise = cfg.cruise_velocity_rps;
    const double cutoff_time = vel / accel;
    const double distance =
        std::max(0.0, goal * direction - pos + vel * vel / (2.0 * accel));
    const double accel_time =
        std::min(cruise / accel, std::sqrt(distance / accel));
    const double full_speed_distance =
        std::max(0.0, distance - accel_time * accel_time * accel);
    const double end_accel = accel_time - cutoff_time;
    const double end_cruise = end_accel + full_speed_distance / cruise;
    const double end_decel = end_cruise + accel_time;
    double next_pos = goal * direction;
    double next_vel = 0.0;
    if (dt < end_accel) {
      next_vel = vel + dt * accel;
      next_pos = pos + (vel + dt * accel / 2.0) * dt;
    } else if (dt < end_cruise) {
      next_vel = cruise;
      next_pos = pos + (vel + end_accel * accel / 2.0) * end_accel +
                 cruise * (dt - end_accel);
    } else if (dt < end_decel) {
      const double remaining = end_decel - dt;
      next_vel = remaining * accel;
      next_pos = goal * direction - remaining * next_vel / 2.0;
    }
    const double previous_vel = p.profile_vel_rps;
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

  double Gear(std::size_t i) const {
    const double gear = config_.motors[i].rotor_to_sensor_ratio *
                        config_.motors[i].sensor_to_mechanism_ratio;
    return gear > 0.0 ? gear : 1.0;
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
    double mech_pos = axes_[i].rotor_position_rot * dir / gear;
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
    axes_[i].rotor_position_rot = mech_pos * dir * gear;
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
      axes_[i].rotor_position_rot = clamped * FeedbackDirection(i) * gear;
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

  // Effective winding voltage in motor i's configured direction after
  // enforcing the voltage ceiling, stator_limit_a (|V_rotor - back_emf| / R <=
  // stator_limit_a), and supply_limit_a (power drawn from supply_v_ capped to
  // supply_limit_a * supply_v_). Fed into StepAxis and recorded in
  // applied_volts_ so rotor dynamics, voltage, stator current, and supply
  // current stay mutually consistent.
  double LimitWindingVoltage(std::size_t i, const MotorConfig& m,
                             double volts) const {
    const MotorModel motor = MotorModelFor(m.motor_type.value);
    const double dir = Direction(i);
    const double ceiling = std::min(m.max_voltage > 0.0 ? m.max_voltage : 12.0,
                                    supply_v_ > 0.0 ? supply_v_ : 12.0);
    double rotor_volts = std::clamp(dir * volts, -ceiling, ceiling);
    const double back_emf_v =
        motor.back_emf_v_per_rps() * axes_[i].rotor_velocity_rps;
    const double resistance_ohm = motor.resistance_ohm();
    if (m.stator_limit_a > 0.0) {
      const double max_drop_v = m.stator_limit_a * resistance_ohm;
      rotor_volts = std::clamp(rotor_volts, back_emf_v - max_drop_v,
                               back_emf_v + max_drop_v);
      rotor_volts = std::clamp(rotor_volts, -ceiling, ceiling);
    }
    if (m.supply_limit_a > 0.0 && supply_v_ > 0.0) {
      const double max_power_drop =
          m.supply_limit_a * supply_v_ * resistance_ohm;
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

  // Signed stator current in motor i's physical rotor frame:
  // (rotor_volts - back_emf) / R, with R = 12 V / stall current and
  // k_e = (12 V - free current * R) / free speed; 0 while unpowered.
  double SignedStatorCurrent(std::size_t i, const MotorConfig& m) const {
    if (!powered_[i]) return 0.0;
    const MotorModel motor = MotorModelFor(m.motor_type.value);
    const double back_emf_v =
        motor.back_emf_v_per_rps() * axes_[i].rotor_velocity_rps;
    const double rotor_volts = Direction(i) * applied_volts_[i];
    return (rotor_volts - back_emf_v) / motor.resistance_ohm();
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

  // Signed power-conservation supply current:
  // (rotor_volts * signed_stator_current) / supply_v_ (positive = battery
  // draw, negative = regenerative braking). Positive draw is clamped to
  // supply_limit_a (matching LimitWindingVoltage, which limits draw and not
  // regen), while negative regen stays signed, matching Phoenix 6. 0 when
  // unpowered or supply_v_ <= 0.
  double SupplyCurrent(std::size_t i, const MotorConfig& m) const {
    if (!powered_[i] || supply_v_ <= 0.0) return 0.0;
    const double rotor_volts = Direction(i) * applied_volts_[i];
    const double current_a =
        rotor_volts * SignedStatorCurrent(i, m) / supply_v_;
    return m.supply_limit_a > 0.0 ? std::min(m.supply_limit_a, current_a)
                                  : current_a;
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
    const MotorModel motor = MotorModelFor(m.motor_type.value);
    const double peak_a =
        std::max(0.0, std::min(m.max_torque_current_a, m.stator_limit_a));
    const double current_a = std::clamp(amperes, -peak_a, peak_a);
    const double back_emf_v =
        motor.back_emf_v_per_rps() * axes_[i].rotor_velocity_rps;
    const double ceiling =
        std::min(m.max_voltage > 0.0 ? m.max_voltage : 12.0, supply_v_);
    return std::clamp(
        current_a * motor.resistance_ohm() + Direction(i) * back_emf_v,
        -ceiling, ceiling);
  }

  bool FeedbackValid(std::size_t i) const {
    if (!motor_connected_[i]) return false;
    if (config_.motors[i].feedback != Feedback::kRemoteCANcoder) return true;
    const int sensor = motor_to_sensor_[i];
    return sensor >= 0 && sensor_connected_[sensor];
  }

  void StepModels(double dt_s) {
    if (dt_s < 0.0) return;
    applied_volts_.fill(0.0);
    powered_.fill(false);
    std::array<bool, kMaxMotors> resolved{};
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      const auto& m = config_.motors[i];
      if (m.master_id != 0) continue;
      resolved[i] = true;
      MechanismModel mech;
      mech.gear_ratio = Gear(i);
      mech.brake = m.brake;
      // Heavier current limits imply a beefier mechanism; flywheels coast.
      mech.load = LoadKind::kRoller;
      double volts = 0.0;
      const double gear = mech.gear_ratio;
      const double mech_pos =
          axes_[i].rotor_position_rot * FeedbackDirection(i) / gear;
      const double mech_vel =
          axes_[i].rotor_velocity_rps * FeedbackDirection(i) / gear;
      auto& profile = profiles_[i];
      const auto& req = commands_[i];
      bool want_power;
      if (req.mode == Mode::kMotionMagic && m.cruise_velocity_rps > 0.0 &&
          m.acceleration_rps2 > 0.0) {
        if (!profile.active) profile = {mech_pos, mech_vel, true};
        const double accel = AdvanceProfile(profile, m, req.demand, dt_s);
        const auto& gains = m.slots[req.slot < 3 ? req.slot : 0];
        const double velocity = profile.profile_vel_rps;
        const double sign = velocity > 0.0 ? 1.0 : velocity < 0.0 ? -1.0 : 0.0;
        const double ceiling =
            std::min(m.max_voltage > 0.0 ? m.max_voltage : 12.0, supply_v_);
        volts = std::clamp(
            req.feedforward_v + gains.p * (profile.profile_pos_rot - mech_pos) +
                gains.v * velocity + gains.a * accel + gains.s * sign +
                gains.d * (velocity - mech_vel) +
                GravityVoltage(gains, mech_pos),
            -ceiling, ceiling);
        want_power = FeedbackValid(i);
      } else if (req.mode == Mode::kTorqueCurrent) {
        // Powered at any demand: 0 A is zero torque, not neutral.
        profile.active = false;
        volts = TorqueCurrentVolts(i, m, req.demand);
        want_power = FeedbackValid(i);
      } else {
        profile.active = false;
        want_power = FeedbackValid(i) &&
                     DriveVoltage(req, m, mech_pos, mech_vel, supply_v_, volts);
      }
      want_power = want_power && !LimitSwitchBlocks(i, volts);
      applied_volts_[i] = want_power ? LimitWindingVoltage(i, m, volts) : 0.0;
      powered_[i] = want_power;
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
        powered_[i] = commands_[i].mode != Mode::kNeutral && FeedbackValid(i) &&
                      powered_[master];
        applied_volts_[i] =
            powered_[i]
                ? LimitWindingVoltage(
                      i, m,
                      applied_volts_[master] * (m.oppose_master ? -1.0 : 1.0))
                : 0.0;
      }
      if (!progress) break;
    }
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
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
    StepChassis(dt_s);
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
          axes_[module.drive].rotor_velocity_rps / Gear(module.drive) * kTau *
          options_.wheel_radius_m * (1.0 + options_.wheel_radius_error_frac);
      const double acceleration =
          (surface_speed - module.previous_surface_speed_mps) / dt_s;
      module.previous_surface_speed_mps = surface_speed;
      const double slip = std::min(
          1.0, options_.wheel_slip_gain * std::abs(acceleration) / 9.80665);
      const double speed = surface_speed * (1.0 - slip);
      const double angle =
          axes_[module.steer].rotor_position_rot / Gear(module.steer) * kTau;
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
  std::array<MotorRequest, kMaxMotors> commands_{};
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
  std::unordered_map<uint16_t, bool> digital_outputs_{};
  std::unordered_map<uint16_t, double> analog_inputs_{};
  std::unordered_map<uint16_t, ManualEncoder> encoders_{};
  std::unordered_map<uint16_t, PwmRequest> pwm_outputs_{};
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
  std::vector<SwerveModuleSpec> configured_modules_;
  double supply_v_{kSimRestingSupplyV};
  double yaw_rot_{0.0};
  double yaw_rate_rps_{0.0};
  std::vector<uint16_t> pending_resets_;
  bool restore_fails_{false};
  std::array<bool, kMaxMotors> restoring_{};
};

}  // namespace talos::hardware::sim

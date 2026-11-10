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
// sim state is fed from the device's voltage output, not from the demand.
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
// mechanism. Swerve ground motion determines the true field pose; the Pigeon
// reports true yaw plus optional drift, unless explicitly overridden.
//
// Stopping is consistent. Commanded neutral (Mode::kNeutral, and
// Neutral() on disable/timeout/failure) removes drive and lets the rotor
// decay per the motor's brake flag: brake stops quickly, coast rolls on. A
// zero-velocity demand is instead an active standstill hold (when gains are
// tuned); with zero gains it honestly cannot hold and coasts.
//
// Controlled inputs/faults. Beam breaks and RoboRIO limit-switch inputs via
// SetDigitalInput, switches on a TalonFX's own limit inputs via
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
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "gateway.h"

namespace talos::hardware::sim {

// One reusable motor/mechanism model.
enum class MotorType : uint8_t {
  kKrakenX60 = 0,
  kKrakenX44,
  kFalcon500,
  kGeneric
};

// Load reflected onto the rotor shaft. Roller/flywheel first; arm/elevator
// gravity loads are future work.
enum class LoadKind : uint8_t { kRoller = 0, kFlywheel };

struct MotorModel {
  MotorType type{MotorType::kKrakenX60};
  // Rotor free speed at 12 V. Kraken X60 default; matches the PhoenixBackend
  // desktop model so both sims agree on unloaded speed.
  double free_speed_rps_at_12v{100.0};
  // First-order lag at nominal_inertia_kg_m2.
  double time_constant_s{0.02};
  double nominal_inertia_kg_m2{0.002};
  // Extra viscous damping in V per rotor rps, beyond the back-EMF slope.
  double viscous_friction_vs_per_rps{0.0};
  // Deadband: a standing-still rotor will not break away while |volts| is
  // below this. Kept at 0 by default so small signals stay observable.
  double static_friction_v{0.0};
};

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
  double start_x_m{0.0};
  double start_y_m{0.0};
  double start_yaw_rad{0.0};
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
// (unpowered); otherwise writes volts.
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
  }
  return false;
}

// Physics-backed Backend. See the header comment for the item-by-item map.
class PhysicsSimBackend final : public Backend {
 public:
  explicit PhysicsSimBackend(PhysicsOptions options = {}) : options_{options} {}

  const WorldState& truth() const { return truth_; }

  bool Configure(const Config& config) override {
    config_ = config;
    axes_.fill(AxisState{});
    profiles_.fill(Profile{});
    gyro_override_ = false;
    modules_.clear();
    commands_.fill(MotorRequest{});
    applied_volts_.fill(0.0);
    powered_.fill(false);
    frozen_.fill(false);
    held_.fill(MotorSample{});
    motor_connected_.fill(true);
    sensor_connected_.fill(true);
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
    supply_v_ = 12.0;
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
    state.health = {true, supply_v_, brownout_, 0.0};
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
        out.valid = false;
        out.position_rot = 0;
        out.velocity_rps = 0;
        out.voltage = 0;
        out.stator_current_a = 0;
        out.temperature_c = 0;
        out.supply_current_a = 0;
        out.fault_flags = 0;
        continue;
      }
      healthy = true;
      if (frozen_[i]) {
        out = held_[i];
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
      // Placeholder current proportional to effort; not a winding model.
      out.stator_current_a = std::abs(out.voltage) / 12.0 * m.stator_limit_a;
      // Placeholder supply current proportional to effort and constant
      // ambient temperature; not electrical or thermal models.
      out.supply_current_a = std::abs(out.voltage) / 12.0 * m.supply_limit_a;
      out.temperature_c = 25.0;
      out.fault_flags = HardLimitFlags(i);
      EnforceSoftLimits(m, out, i);
      held_[i] = out;
    }
    for (std::size_t i = 0; i < config_.sensors.size(); ++i) {
      const auto& s = config_.sensors[i];
      auto& out = state.sensors[i];
      out.id = s.id;
      out.position_age_us = 0;
      out.velocity_age_us = 0;
      if (!sensor_connected_[i]) {
        out.valid = false;
        out.position_rot = 0;
        out.velocity_rps = 0;
        continue;
      }
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
      }
    }
    for (std::size_t i = 0; i < config_.digital_inputs.size(); ++i) {
      const auto id = config_.digital_inputs[i].id;
      const auto it = digital_inputs_.find(id);
      state.digital_inputs[i] = {
          id, true, it != digital_inputs_.end() ? it->second : false};
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
      const auto id = config_.pwm_outputs[i].id;
      const auto it = pwm_outputs_.find(id);
      state.pwm_outputs[i] = {id, true,
                              it != pwm_outputs_.end() ? it->second : 0.0};
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
    for (const auto& p : pwm_outputs) pwm_outputs_[p.id] = p.output;
    return true;
  }

  // Safe-state policy mirrors SimBackend: every output type turns off on
  // disable, command timeout, and failure (motors to kNeutral, PWM to 0,
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
  void SimulateDeviceReset(uint16_t id) { pending_resets_.push_back(id); }
  bool PollDeviceResets(std::vector<uint16_t>& reset_ids) override {
    reset_ids = pending_resets_;
    pending_resets_.clear();
    return !reset_ids.empty();
  }
  void BeginRestoreAfterReset(std::span<const uint16_t> ids) override {
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
    return restore_fails_ ? RestoreStatus::kFailed : RestoreStatus::kDone;
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
  // Wheel travel uses nominal radius 0.0508 m plus the configured error.
  void SetSwerveModuleGeometry(uint16_t drive_id, uint16_t steer_id, double x_m,
                               double y_m) {
    const int drive = MotorIndex(drive_id);
    const int steer = MotorIndex(steer_id);
    if (drive < 0 || steer < 0 || !std::isfinite(x_m) || !std::isfinite(y_m) ||
        (x_m == 0.0 && y_m == 0.0))
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
  double GetPwm(uint16_t id) const {
    const auto it = pwm_outputs_.find(id);
    return it != pwm_outputs_.end() ? it->second : 0.0;
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

  void EnforceSoftLimits(const MotorConfig& m, MotorSample& out,
                         std::size_t i) {
    if (!m.soft_limits) return;
    const double lo = std::min(m.reverse_limit_rot, m.forward_limit_rot);
    const double hi = std::max(m.reverse_limit_rot, m.forward_limit_rot);
    const double gear = Gear(i);
    if (out.position_rot < lo || out.position_rot > hi) {
      const double clamped = std::clamp(out.position_rot, lo, hi);
      axes_[i].rotor_position_rot = clamped * FeedbackDirection(i) * gear;
      out.position_rot = clamped;
      if ((out.position_rot <= lo && out.velocity_rps < 0) ||
          (out.position_rot >= hi && out.velocity_rps > 0)) {
        axes_[i].rotor_velocity_rps = 0.0;
        out.velocity_rps = 0.0;
      }
    }
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
      } else {
        profile.active = false;
        want_power = FeedbackValid(i) &&
                     DriveVoltage(req, m, mech_pos, mech_vel, supply_v_, volts);
      }
      want_power = want_power && !LimitSwitchBlocks(i, volts);
      applied_volts_[i] = want_power ? volts : 0.0;
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
        applied_volts_[i] = powered_[i] ? applied_volts_[master] *
                                              (m.oppose_master ? -1.0 : 1.0)
                                        : 0.0;
      }
      if (!progress) break;
    }
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      const auto& m = config_.motors[i];
      MotorModel motor;
      MechanismModel mech;
      mech.gear_ratio = Gear(i);
      mech.brake = m.brake;
      const double volts = applied_volts_[i];
      // A follower's applied volts are its master's, reversed if opposed, so
      // this is OutputDirection(i) times the master's volts.
      const double rotor_volts = Direction(i) * volts;
      axes_[i] = StepAxis(motor, mech, axes_[i], rotor_volts, powered_[i],
                          supply_v_, dt_s);
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
      const double surface_speed = axes_[module.drive].rotor_velocity_rps /
                                   Gear(module.drive) * kTau * 0.0508 *
                                   (1.0 + options_.wheel_radius_error_frac);
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
  std::array<bool, kMaxMotors> motor_connected_{};
  std::array<bool, kMaxMotors> forward_pressed_{};
  std::array<bool, kMaxMotors> reverse_pressed_{};
  std::array<bool, kMaxSensors> sensor_connected_{};
  // Filled with -1 by Configure; valid only after Configure.
  std::array<int, kMaxMotors> motor_to_sensor_;
  std::unordered_map<uint16_t, bool> digital_inputs_{};
  std::unordered_map<uint16_t, bool> digital_outputs_{};
  std::unordered_map<uint16_t, double> analog_inputs_{};
  std::unordered_map<uint16_t, ManualEncoder> encoders_{};
  std::unordered_map<uint16_t, double> pwm_outputs_{};
  double supply_v_{12.0};
  double yaw_rot_{0.0};
  double yaw_rate_rps_{0.0};
  std::vector<uint16_t> pending_resets_;
  bool restore_fails_{false};
};

}  // namespace talos::hardware::sim

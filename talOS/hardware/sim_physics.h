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
//
// Sensors cross the hardware boundary. CANcoder samples mirror the linked
// mechanism. Pigeon yaw integrates swerve module motion unless explicitly
// overridden with SetGyro or SetChassisYawRate.
//
// Stopping is consistent. Commanded neutral (Mode::kNeutral, and
// Neutral() on disable/timeout/failure) removes drive and lets the rotor
// decay per the motor's brake flag: brake stops quickly, coast rolls on. A
// zero-velocity demand is instead an active standstill hold (when gains are
// tuned); with zero gains it honestly cannot hold and coasts.
//
// Controlled inputs/faults. Beam breaks and limit-switch inputs via
// SetDigitalInput, soft limits from MotorConfig enforced in physics, frozen
// feedback, per-device disconnect, supply-voltage change, controller restart.
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
      volts_out = clamp(req.feedforward_v + gains.p * error +
                            gains.s * (req.demand >= 0.0 ? 1.0 : -1.0) *
                                (std::abs(req.demand) > 1e-9 ? 1.0 : 0.0) +
                            gains.v * req.demand,
                        ceiling);
      return true;
    }
    case Mode::kPosition:
    case Mode::kMotionMagic: {
      // Without positive cruise/acceleration limits, MotionMagic falls back
      // to position control. Profiled requests are handled by the backend.
      const auto& gains = cfg.slots[req.slot < 3 ? req.slot : 0];
      const double error = req.demand - mech_pos;
      volts_out = clamp(
          req.feedforward_v + gains.p * error - gains.d * mech_vel, ceiling);
      return true;
    }
  }
  return false;
}

// Physics-backed Backend. See the header comment for the item-by-item map.
class PhysicsSimBackend final : public Backend {
 public:
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
        continue;
      }
      healthy = true;
      if (frozen_[i]) {
        out = held_[i];
        continue;
      }
      const double gear = Gear(i);
      const double mech_pos = axes_[i].rotor_position_rot / gear;
      const double mech_vel = axes_[i].rotor_velocity_rps / gear;
      out.valid = true;
      out.position_rot = mech_pos;
      out.velocity_rps = mech_vel;
      out.voltage = powered_[i] ? applied_volts_[i] : 0.0;
      // Placeholder current proportional to effort; not a winding model.
      out.stator_current_a = std::abs(out.voltage) / 12.0 * m.stator_limit_a;
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
        out.position_rot = yaw_rot_;
        out.velocity_rps = yaw_rate_rps_;
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
  void SetAnalogInput(uint16_t id, double voltage) {
    analog_inputs_[id] = voltage;
  }
  void SetEncoder(uint16_t id, double position_rot, double velocity_rps) {
    encoders_[id] = {position_rot, velocity_rps};
  }
  // Fault injection for reset handling. The next gateway Tick
  // reports these logical IDs as reset; BeginRestoreAfterReset re-applies the
  // motor-to-sensor mapping and zeroes rotor states, mirroring a device that
  // lost its configuration and position across the reset. The gateway drops
  // obsolete demand separately via Neutral/epoch bump.
  void SimulateDeviceReset(uint16_t id) { pending_resets_.push_back(id); }
  bool PollDeviceResets(std::vector<uint16_t>& reset_ids) override {
    reset_ids = pending_resets_;
    pending_resets_.clear();
    return !reset_ids.empty();
  }
  void BeginRestoreAfterReset() override {
    axes_.fill(AxisState{});
    profiles_.fill(Profile{});
    commands_.fill(MotorRequest{});
    applied_volts_.fill(0.0);
    powered_.fill(false);
    frozen_.fill(false);
    motor_connected_.fill(true);
    sensor_connected_.fill(true);
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      held_[i] = MotorSample{};
      held_[i].id = config_.motors[i].id;
      held_[i].valid = true;
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
  // Zeroes rotor states, latched commands, yaw, and fault flags. Config and
  // output states are kept; the gateway boot id is gateway-owned and
  // unaffected (a full reboot epoch bump goes through Reconfigure/Invalidate).
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
  }
  // Chassis motion driving the Pigeon, in mechanism rotations/s.
  void SetChassisYawRate(double yaw_rate_rps) {
    gyro_override_ = true;
    yaw_rate_rps_ = yaw_rate_rps;
  }
  void SetGyro(double yaw_rot, double yaw_rate_rps) {
    SetChassisYawRate(yaw_rate_rps);
    yaw_rot_ = yaw_rot;
  }
  // Override inferred placement, or register arbitrarily named module pairs.
  // Wheel travel uses the standard 0.0508 m wheel radius.
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
  };

  void ConfigureSwerve() {
    const bool has_gyro = std::any_of(
        config_.sensors.begin(), config_.sensors.end(),
        [](const auto& s) { return s.kind == SensorKind::kPigeon2; });
    if (!has_gyro) return;
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
      axes_[i].rotor_position_rot = clamped * gear;
      out.position_rot = clamped;
      if ((out.position_rot <= lo && out.velocity_rps < 0) ||
          (out.position_rot >= hi && out.velocity_rps > 0)) {
        axes_[i].rotor_velocity_rps = 0.0;
        out.velocity_rps = 0.0;
      }
    }
  }

  void StepModels(double dt_s) {
    if (dt_s < 0.0) return;
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      const auto& m = config_.motors[i];
      MotorModel motor;
      MechanismModel mech;
      mech.gear_ratio = Gear(i);
      mech.brake = m.brake;
      // Heavier current limits imply a beefier mechanism; flywheels coast.
      mech.load = LoadKind::kRoller;
      double volts = 0.0;
      const double gear = mech.gear_ratio;
      const double mech_pos = axes_[i].rotor_position_rot / gear;
      const double mech_vel = axes_[i].rotor_velocity_rps / gear;
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
        volts = std::clamp(req.feedforward_v +
                               gains.p * (profile.profile_pos_rot - mech_pos) +
                               gains.v * velocity + gains.a * accel +
                               gains.s * sign + gains.d * (velocity - mech_vel),
                           -ceiling, ceiling);
        want_power = motor_connected_[i];
      } else {
        profile.active = false;
        want_power = motor_connected_[i] &&
                     DriveVoltage(req, m, mech_pos, mech_vel, supply_v_, volts);
      }
      double rotor_volts = m.inverted ? -volts : volts;
      axes_[i] = StepAxis(motor, mech, axes_[i], rotor_volts, want_power,
                          supply_v_, dt_s);
      applied_volts_[i] = want_power ? volts : 0.0;
      powered_[i] = want_power;
    }
    if (!gyro_override_ && !modules_.empty()) {
      constexpr double kTau = 6.28318530717958647692;
      double angular_velocity = 0.0;
      double lever_squared = 0.0;
      for (const auto& module : modules_) {
        const double speed = axes_[module.drive].rotor_velocity_rps /
                             Gear(module.drive) * kTau * 0.0508;
        const double angle =
            axes_[module.steer].rotor_position_rot / Gear(module.steer) * kTau;
        angular_velocity += speed * (module.x_m * std::sin(angle) -
                                     module.y_m * std::cos(angle));
        lever_squared += module.x_m * module.x_m + module.y_m * module.y_m;
      }
      yaw_rate_rps_ = angular_velocity / lever_squared / kTau;
    }
    yaw_rot_ += yaw_rate_rps_ * dt_s;
  }

  Config config_;
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

#include "PhoenixBackend.h"

#include <frc/RobotController.h>

#include <algorithm>
#include <cmath>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

namespace ph = ctre::phoenix6;
namespace hw = talos::hardware;
using namespace units::literals;

namespace {
template <typename Slot>
void SetGains(Slot& slot, const hw::Gains& gains) {
  slot.kP = gains.p;
  slot.kI = gains.i;
  slot.kD = gains.d;
  slot.kS = gains.s;
  slot.kV = gains.v;
  slot.kA = gains.a;
  slot.kG = gains.g;
  slot.GravityType = gains.gravity_type == hw::GravityType::kArmCosine
                         ? ph::signals::GravityTypeValue::Arm_Cosine
                         : ph::signals::GravityTypeValue::Elevator_Static;
}
bool Fresh(const ph::BaseStatusSignal& signal,
           units::second_t max_age = 50_ms) {
  return signal.GetStatus().IsOK() &&
         signal.GetTimestamp().GetLatency() < max_age;
}
template <typename Position, typename Velocity>
bool ValidMotionFeedback(const Position& position, const Velocity& velocity) {
  return Fresh(position) && Fresh(velocity) &&
         std::isfinite(position.GetValue().value()) &&
         std::isfinite(velocity.GetValue().value());
}
uint32_t Age(const ph::BaseStatusSignal& signal) {
  const double age = signal.GetTimestamp().GetLatency().value() * 1e6;
  if (!std::isfinite(age) || age >= std::numeric_limits<uint32_t>::max())
    return std::numeric_limits<uint32_t>::max();
  return static_cast<uint32_t>(std::max(0.0, std::ceil(age)));
}

void LogFeedbackChange(const char* kind, const std::string& name, uint16_t id,
                       const std::string& bus, int can_id, bool valid,
                       const ph::BaseStatusSignal& p,
                       const ph::BaseStatusSignal& v) {
  std::cerr << "[phoenix] " << kind << " " << name << " (id=" << id
            << ", bus=" << bus << ", can_id=" << can_id << ") feedback "
            << (valid ? "RECOVERED" : "INVALID/STALE")
            << " (position_status=" << p.GetStatus().GetName()
            << ", velocity_status=" << v.GetStatus().GetName()
            << ", position_age_us=" << Age(p) << ", velocity_age_us=" << Age(v)
            << ")\n";
}

// A config apply or a signal-rate change can miss its reply window while a
// device is still booting, or in simulation while a re-created sim device comes
// up. CTRE's guidance is to retry; five tries bound each call to about half a
// second.
template <typename Call>
ctre::phoenix::StatusCode Retry(Call call) {
  ctre::phoenix::StatusCode status = call();
  for (int attempt = 1; attempt < 5 && !status.IsOK(); ++attempt) {
    status = call();
  }
  return status;
}

// Desktop-sim neutral decay time constants (rotor velocity, first order).
// Neutral removes drive but keeps momentum: brake shorts the windings and
// stops the rotor quickly, coast lets the load roll on. Matches the unpowered
// branch of talos::hardware::sim::StepAxis (talOS/hardware/sim_physics.h), so
// disable-time behaviour agrees between the two simulators.
constexpr double kSimNeutralBrakeTauS = 0.05;
constexpr double kSimNeutralCoastTauS = 1.0;
// Friction holds a nearly-stopped unpowered rotor instead of creeping forever
// (same standstill snap as StepAxis).
constexpr double kSimNeutralStandstillRps = 0.01;
}  // namespace

PhoenixBackend::PhoenixBackend(bool is_simulation)
    : is_simulation_{is_simulation} {}

std::string PhoenixBackend::DecodeTalonFaultBits(uint32_t fault_flags) {
  if (fault_flags == 0) return "none";
  // Phoenix 6 TalonFX FaultField, SPN 0x34c / CAN frame 0xad.
  static constexpr std::array<const char*, 32> names{
      "Hardware (hardware fault occurred)",
      "ProcTemp (processor temperature exceeded limit)",
      "DeviceTemp (device temperature exceeded limit)",
      "Undervoltage (supply voltage near brownout)",
      "BootDuringEnable (booted while enabled)",
      "UnlicensedFeatureInUse (unlicensed feature in use)",
      "StaticBrakeDisabled (static brake momentarily disabled)",
      "UsingFusedCANcoderWhileUnlicensed (fused CANcoder requires Pro)",
      "SupplyCurrLimit (supply current limit active)",
      "StatorCurrLimit (stator current limit active)",
      "FusedSensorOutOfSync (remote fused sensor out of sync)",
      "RemoteSensorDataInvalid (remote sensor data invalid)",
      "MissingHardLimitRemote (remote hard limit device missing)",
      "MissingSoftLimitRemote (remote soft limit device missing)",
      "ForwardSoftLimit (forward soft limit asserted)",
      "ReverseSoftLimit (reverse soft limit asserted)",
      "ForwardHardLimit (forward hard limit asserted)",
      "ReverseHardLimit (reverse hard limit asserted)",
      "OverSupplyV (supply voltage exceeded rating)",
      "UnstableSupplyV (supply voltage unstable)",
      "RemoteSensorPosOverflow (remote sensor position overflowed)",
      "MissingDifferentialFX (differential TalonFX missing)",
      "RemoteSensorReset (remote sensor reset)",
      "BridgeBrownout (bridge disabled due to supply voltage)",
      nullptr,
      "RotorFault1 (rotor fault 1)",
      "RotorFault2 (rotor fault 2)",
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      nullptr};
  std::string faults;
  for (unsigned bit = 0; bit < names.size(); ++bit) {
    if ((fault_flags & (uint32_t{1} << bit)) == 0) continue;
    if (!faults.empty()) faults += ", ";
    if (names[bit]) {
      faults += names[bit];
    } else {
      faults += "UnknownBit" + std::to_string(bit);
    }
  }
  return faults;
}

std::optional<hw::ConfigError> PhoenixBackend::last_config_error() const {
  return last_error_;
}

void PhoenixBackend::BeginConfigure(const hw::Config& config) {
  configure_ = std::async(std::launch::async,
                          [this, config] { return Configure(config); });
}

PhoenixBackend::ConfigureStatus PhoenixBackend::PollConfigure() {
  if (!configure_.valid()) return ConfigureStatus::kFailed;
  if (configure_.wait_for(std::chrono::seconds{0}) != std::future_status::ready)
    return ConfigureStatus::kPending;
  return configure_.get() ? ConfigureStatus::kDone : ConfigureStatus::kFailed;
}

bool PhoenixBackend::Configure(const hw::Config& c) {
  hw::Validate(c);
  restore_operations_.clear();
  unavailable_.clear();
  signal_ids_.clear();
  last_error_.reset();
  last_fault_flags_.fill(0);
  last_motor_valid_.fill(true);
  last_sensor_valid_.fill(true);
  last_motor_fault_mask_ = 0;
  neutral_send_failing_.fill(false);
  apply_failing_.fill(false);
  apply_failed_.clear();
  apply_failed_.reserve(c.motors.size());
  last_brownout_ = false;
  // The first failed device apply wins the precise error: device
  // name, bus, CAN ID, operation, and the raw vendor status value.
  const auto note_failure = [this](const std::string& name,
                                   const std::string& bus, int can_id,
                                   const std::string& operation,
                                   const ctre::phoenix::StatusCode& code) {
    if (code.IsOK()) return true;
    uint16_t id = 0;
    for (const auto& m : config_.motors)
      if (m.name == name) id = m.id;
    for (const auto& s : config_.sensors)
      if (s.name == name) id = s.id;
    if (id != 0 && std::find(unavailable_.begin(), unavailable_.end(), id) ==
                       unavailable_.end())
      unavailable_.push_back(id);
    if (last_error_.has_value()) return false;
    hw::ConfigError err;
    err.device_name = name;
    err.bus = bus;
    err.can_id = can_id;
    err.operation = operation;
    err.vendor_status = static_cast<int>(code);
    err.message = code.GetName();
    last_error_ = err;
    return false;
  };
  config_ = c;
  signals_.clear();
  motors_.clear();
  sensors_.clear();
  digital_inputs_.clear();
  digital_outputs_.clear();
  analog_inputs_.clear();
  rio_encoders_.clear();
  pwm_outputs_.clear();

  sim_rotor_velocity_.fill(0.0);
  sim_rotor_position_.fill(0.0);
  motor_to_sensor_map_.fill(-1);
  sim_neutral_.fill(true);
  sim_yaw_deg_ = 0.0;
  last_sim_time_ = std::nullopt;

  for (const auto& d : c.digital_inputs) {
    digital_inputs_.push_back(std::make_unique<frc::DigitalInput>(d.dio));
  }
  for (const auto& d : c.digital_outputs) {
    digital_outputs_.push_back(std::make_unique<frc::DigitalOutput>(d.dio));
  }
  for (const auto& a : c.analog_inputs) {
    analog_inputs_.push_back(std::make_unique<frc::AnalogInput>(a.channel));
  }
  for (const auto& e : c.encoders) {
    RioEncoder dev;
    dev.config = e;
    if (e.kind == hw::EncoderKind::kDutyCycle) {
      // Single-channel ctor reports fullRange 1, so Get() is raw rotations.
      dev.duty_cycle = std::make_unique<frc::DutyCycleEncoder>(e.dio_a);
    } else {
      dev.quadrature = std::make_unique<frc::Encoder>(e.dio_a, e.dio_b);
      // Default k4X decoding counts every edge, and counts_per_rev is edges
      // per rev, so this reports rotations (Validate guarantees cpr > 0).
      dev.quadrature->SetDistancePerPulse(1.0 / e.counts_per_rev);
    }
    rio_encoders_.push_back(std::move(dev));
  }
  for (const auto& p : c.pwm_outputs) {
    pwm_outputs_.push_back(std::make_unique<frc::PWM>(p.channel));
  }

  bool ok = true;
  for (const auto& s : c.sensors) {
    Sensor device;
    if (s.kind == hw::SensorKind::kCANcoder) {
      device.encoder =
          std::make_unique<ph::hardware::CANcoder>(s.can_id, ph::CANBus{s.bus});
      ph::configs::CANcoderConfiguration cfg;
      cfg.MagnetSensor.MagnetOffset = units::turn_t{s.offset_rot};
      cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;
      cfg.MagnetSensor.SensorDirection =
          s.inverted
              ? ph::signals::SensorDirectionValue::Clockwise_Positive
              : ph::signals::SensorDirectionValue::CounterClockwise_Positive;
      ok &= note_failure(
          s.name, s.bus, s.can_id, "apply_cancoder_config",
          Retry([&] { return device.encoder->GetConfigurator().Apply(cfg); }));
      restore_operations_.push_back(
          {s.id, [ptr = device.encoder.get(), cfg] {
             return Retry([&] { return ptr->GetConfigurator().Apply(cfg); })
                 .IsOK();
           }});
      signals_.push_back(&device.encoder->GetAbsolutePosition(false));
      signals_.push_back(&device.encoder->GetVelocity(false));
    } else {
      device.imu =
          std::make_unique<ph::hardware::Pigeon2>(s.can_id, ph::CANBus{s.bus});
      ph::configs::Pigeon2Configuration cfg;
      cfg.MountPose.MountPoseYaw = units::degree_t{s.offset_rot * 360.0};
      ok &= note_failure(
          s.name, s.bus, s.can_id, "apply_pigeon2_config",
          Retry([&] { return device.imu->GetConfigurator().Apply(cfg); }));
      restore_operations_.push_back(
          {s.id, [ptr = device.imu.get(), cfg] {
             return Retry([&] { return ptr->GetConfigurator().Apply(cfg); })
                 .IsOK();
           }});
      signals_.push_back(&device.imu->GetYaw(false));
      signals_.push_back(&device.imu->GetAngularVelocityZWorld(false));
    }
    signal_ids_.push_back(s.id);
    signal_ids_.push_back(s.id);
    sensors_.push_back(std::move(device));
  }
  for (const auto& m : c.motors) {
    auto motor =
        std::make_unique<ph::hardware::TalonFX>(m.can_id, ph::CANBus{m.bus});
    ph::configs::TalonFXConfiguration cfg;
    cfg.MotorOutput.Inverted =
        m.inverted ? ph::signals::InvertedValue::Clockwise_Positive
                   : ph::signals::InvertedValue::CounterClockwise_Positive;
    cfg.MotorOutput.NeutralMode = m.brake
                                      ? ph::signals::NeutralModeValue::Brake
                                      : ph::signals::NeutralModeValue::Coast;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = units::ampere_t{m.supply_limit_a};
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = units::ampere_t{m.stator_limit_a};
    cfg.Voltage.PeakForwardVoltage = units::volt_t{m.max_voltage};
    cfg.Voltage.PeakReverseVoltage = units::volt_t{-m.max_voltage};
    cfg.Feedback.RotorToSensorRatio = m.rotor_to_sensor_ratio;
    cfg.Feedback.SensorToMechanismRatio = m.sensor_to_mechanism_ratio;
    if (m.feedback == hw::Feedback::kRemoteCANcoder) {
      cfg.Feedback.FeedbackSensorSource =
          ph::signals::FeedbackSensorSourceValue::RemoteCANcoder;
      for (const auto& s : c.sensors)
        if (s.id == m.feedback_sensor_id)
          cfg.Feedback.FeedbackRemoteSensorID = s.can_id;
    }
    cfg.ClosedLoopGeneral.ContinuousWrap = m.continuous_wrap;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = m.soft_limits;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = m.soft_limits;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        units::turn_t{m.forward_limit_rot};
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        units::turn_t{m.reverse_limit_rot};
    cfg.MotionMagic.MotionMagicCruiseVelocity =
        units::turns_per_second_t{m.cruise_velocity_rps};
    cfg.MotionMagic.MotionMagicAcceleration =
        units::turns_per_second_squared_t{m.acceleration_rps2};
    cfg.MotionMagic.MotionMagicJerk =
        decltype(cfg.MotionMagic.MotionMagicJerk){m.jerk_rps3};
    SetGains(cfg.Slot0, m.slots[0]);
    SetGains(cfg.Slot1, m.slots[1]);
    SetGains(cfg.Slot2, m.slots[2]);
    ok &= note_failure(
        m.name, m.bus, m.can_id, "apply_talonfx_config",
        Retry([&] { return motor->GetConfigurator().Apply(cfg); }));
    restore_operations_.push_back(
        {m.id, [ptr = motor.get(), cfg] {
           return Retry([&] { return ptr->GetConfigurator().Apply(cfg); })
               .IsOK();
         }});
    signals_.push_back(&motor->GetPosition(false));
    signals_.push_back(&motor->GetVelocity(false));
    signals_.push_back(&motor->GetMotorVoltage(false));
    signals_.push_back(&motor->GetStatorCurrent(false));
    signals_.push_back(&motor->GetFaultField(false));
    signals_.push_back(&motor->GetDeviceTemp(false));
    signals_.push_back(&motor->GetSupplyCurrent(false));
    for (int i = 0; i < 7; ++i) signal_ids_.push_back(m.id);
    motors_.push_back(std::move(motor));
  }

  if (is_simulation_) {
    for (auto& motor : motors_) {
      auto& sim = motor->GetSimState();
      sim.SetSupplyVoltage(12_V);
      sim.SetRawRotorPosition(0_tr);
      sim.SetRotorVelocity(0_tps);
    }
    for (auto& device : sensors_) {
      if (device.encoder) {
        auto& sim = device.encoder->GetSimState();
        sim.SetSupplyVoltage(12_V);
        sim.SetRawPosition(0_tr);
        sim.SetVelocity(0_tps);
      } else if (device.imu) {
        auto& sim = device.imu->GetSimState();
        sim.SetSupplyVoltage(12_V);
        sim.SetRawYaw(0_deg);
      }
    }
    for (std::size_t m_idx = 0;
         m_idx < c.motors.size() && m_idx < motor_to_sensor_map_.size();
         ++m_idx) {
      const auto& m = c.motors[m_idx];
      if (m.feedback == hw::Feedback::kRemoteCANcoder) {
        for (std::size_t s_idx = 0; s_idx < c.sensors.size(); ++s_idx) {
          if (c.sensors[s_idx].id == m.feedback_sensor_id) {
            motor_to_sensor_map_[m_idx] = static_cast<int>(s_idx);
            break;
          }
        }
      }
    }
  }

  for (std::size_t i = 0; i < signals_.size(); ++i) {
    auto* signal = signals_[i];
    const uint16_t id = signal_ids_[i];
    double hz = c.status_hz;
    std::string name, bus;
    int can_id = -1;
    for (const auto& m : c.motors) {
      if (m.id != id) continue;
      name = m.name;
      bus = m.bus;
      can_id = m.can_id;
      // Diagnostics request lower rates, but Phoenix applies the highest
      // requested rate to every signal sharing a status frame. In SDK 26.3,
      // StatorCurrent, SupplyCurrent and DeviceTemp are SupplyAndTemp SPNs.
      auto& motor = *motors_[&m - c.motors.data()];
      if (signal == &motor.GetMotorVoltage(false) ||
          signal == &motor.GetStatorCurrent(false) ||
          signal == &motor.GetSupplyCurrent(false) ||
          signal == &motor.GetFaultField(false))
        hz = std::min(hz, 50.0);
      if (signal == &motor.GetDeviceTemp(false)) hz = std::min(hz, 4.0);
    }
    for (const auto& s : c.sensors) {
      if (s.id != id) continue;
      name = s.name;
      bus = s.bus;
      can_id = s.can_id;
    }
    const auto operation = [signal, hz] {
      return Retry(
          [&] { return signal->SetUpdateFrequency(units::hertz_t{hz}); });
    };
    ok &= note_failure(name, bus, can_id, "set_signal_frequency", operation());
    restore_operations_.push_back(
        {id, [operation] { return operation().IsOK(); }});
  }
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const auto& m = c.motors[i];
    const auto operation = [ptr = motors_[i].get()] {
      return Retry([&] { return ptr->OptimizeBusUtilization(); });
    };
    ok &= note_failure(m.name, m.bus, m.can_id, "optimize_bus_utilization",
                       operation());
    restore_operations_.push_back(
        {m.id, [operation] { return operation().IsOK(); }});
  }
  for (std::size_t i = 0; i < sensors_.size(); ++i) {
    const auto& s = c.sensors[i];
    const auto operation = [encoder = sensors_[i].encoder.get(),
                            imu = sensors_[i].imu.get()] {
      return Retry([&] {
        return encoder ? encoder->OptimizeBusUtilization()
                       : imu->OptimizeBusUtilization();
      });
    };
    ok &= note_failure(s.name, s.bus, s.can_id, "optimize_bus_utilization",
                       operation());
    restore_operations_.push_back(
        {s.id, [operation] { return operation().IsOK(); }});
  }
  if (!ok && last_error_) {
    const auto& error = *last_error_;
    std::cerr << "[phoenix] configure failed on " << error.device_name
              << " (bus=" << error.bus << ", can_id=" << error.can_id
              << ", op=" << error.operation << "): " << error.message
              << " (status=" << error.vendor_status << ")\n";
  }
  // Structural validity succeeded; unavailable devices are isolated and
  // retried.
  return true;
}

bool PhoenixBackend::PollDeviceResets(std::vector<uint16_t>& reset_ids) {
  // HasResetOccurred reports (and clears) one device's reset latch.
  reset_ids.clear();
  for (std::size_t i = 0; i < motors_.size() && i < config_.motors.size();
       ++i) {
    if (motors_[i]->HasResetOccurred()) {
      const auto& motor = config_.motors[i];
      reset_ids.push_back(motor.id);
      std::cerr << "[phoenix] device reset detected: motor " << motor.name
                << " (id=" << motor.id << ", bus=" << motor.bus
                << ", can_id=" << motor.can_id << ")\n";
    }
  }
  for (std::size_t i = 0; i < sensors_.size() && i < config_.sensors.size();
       ++i) {
    const bool reset = sensors_[i].encoder
                           ? sensors_[i].encoder->HasResetOccurred()
                           : sensors_[i].imu->HasResetOccurred();
    if (reset) {
      const auto& sensor = config_.sensors[i];
      reset_ids.push_back(sensor.id);
      std::cerr << "[phoenix] device reset detected: sensor " << sensor.name
                << " (id=" << sensor.id << ", bus=" << sensor.bus
                << ", can_id=" << sensor.can_id << ")\n";
    }
  }
  return !reset_ids.empty();
}

void PhoenixBackend::BeginRestoreAfterReset(std::span<const uint16_t> ids) {
  restoring_ids_.assign(ids.begin(), ids.end());
  // Re-apply the affected control chain, never unrelated device settings.
  for (std::size_t pass = 0; pass < config_.motors.size(); ++pass) {
    for (const auto& motor : config_.motors) {
      const auto contains = [&](uint16_t id) {
        return std::find(restoring_ids_.begin(), restoring_ids_.end(), id) !=
               restoring_ids_.end();
      };
      if (!contains(motor.id) &&
          ((motor.master_id != 0 && contains(motor.master_id)) ||
           (motor.feedback == hw::Feedback::kRemoteCANcoder &&
            contains(motor.feedback_sensor_id))))
        restoring_ids_.push_back(motor.id);
    }
  }
  for (auto id : restoring_ids_) {
    if (std::find(unavailable_.begin(), unavailable_.end(), id) ==
        unavailable_.end())
      unavailable_.push_back(id);
  }
  restore_ = std::async(std::launch::async, [this, ids = restoring_ids_] {
    std::vector<uint16_t> failures;
    for (const auto& operation : restore_operations_) {
      if (std::find(ids.begin(), ids.end(), operation.id) == ids.end())
        continue;
      if (!operation.apply() && std::find(failures.begin(), failures.end(),
                                          operation.id) == failures.end())
        failures.push_back(operation.id);
    }
    return failures;
  });
}

PhoenixBackend::RestoreStatus PhoenixBackend::PollRestore() {
  if (!restore_.valid()) return RestoreStatus::kDone;
  if (restore_.wait_for(std::chrono::seconds{0}) != std::future_status::ready)
    return RestoreStatus::kPending;
  const auto failures = restore_.get();
  std::erase_if(unavailable_, [this, &failures](uint16_t id) {
    return std::find(restoring_ids_.begin(), restoring_ids_.end(), id) !=
               restoring_ids_.end() &&
           std::find(failures.begin(), failures.end(), id) == failures.end();
  });
  return RestoreStatus::kDone;
}

bool PhoenixBackend::Read(hw::State& s) {
  s.health = {true, frc::RobotController::GetBatteryVoltage().value(),
              frc::RobotController::IsBrownedOut(),
              frc::RobotController::GetCANStatus().percentBusUtilization};
  if (is_simulation_) {
    const auto now = std::chrono::steady_clock::now();
    double dt =
        last_sim_time_
            ? std::chrono::duration<double>(now - *last_sim_time_).count()
            : 0.005;
    if (dt <= 0.0 || dt > 0.05) dt = 0.005;
    last_sim_time_ = now;
    UpdateSimulation(dt);
  }
  ph::BaseStatusSignal::RefreshAll(
      false, std::span<ph::BaseStatusSignal* const>{signals_});
  bool healthy = motors_.empty() && sensors_.empty();
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    auto& motor = *motors_[i];
    auto& out = s.motors[i];
    auto& p = motor.GetPosition(false);
    auto& v = motor.GetVelocity(false);
    auto& volts = motor.GetMotorVoltage(false);
    auto& amps = motor.GetStatorCurrent(false);
    auto& temperature = motor.GetDeviceTemp(false);
    auto& supply = motor.GetSupplyCurrent(false);
    out.position_rot = p.GetValue().value();
    out.velocity_rps = v.GetValue().value();
    out.position_age_us = Age(p);
    out.velocity_age_us = Age(v);
    out.fault_flags = motor.GetFaultField(false).GetValue();
    // Stale current/voltage is telemetry-only degradation. The
    // effort fields zero but the sample stays valid and the backend stays
    // healthy; lost motion feedback isolates only the affected motor.
    const bool effort_ok = Fresh(volts) && Fresh(amps) &&
                           std::isfinite(volts.GetValue().value()) &&
                           std::isfinite(amps.GetValue().value());
    out.voltage = effort_ok ? volts.GetValue().value() : 0.0;
    // Phoenix opposition changes the follower's effective rotor polarity.
    // Expose the configured mechanism frame, not that dynamically flipped
    // controller frame, just as the platform simulation backends do.
    const auto& motor_config = config_.motors[i];
    if (motor_config.master_id != 0 && motor_config.oppose_master) {
      out.voltage = -out.voltage;
      if (motor_config.feedback == hw::Feedback::kRotor) {
        out.position_rot = -out.position_rot;
        out.velocity_rps = -out.velocity_rps;
      }
    }
    out.stator_current_a = effort_ok ? amps.GetValue().value() : 0.0;
    // Temperature can legitimately be 250 ms old at its requested 4 Hz.
    // Neither health signal participates in motion-feedback validity.
    out.temperature_c = Fresh(temperature, 500_ms) &&
                                std::isfinite(temperature.GetValue().value())
                            ? temperature.GetValue().value()
                            : 0.0;
    out.supply_current_a =
        Fresh(supply) && std::isfinite(supply.GetValue().value())
            ? supply.GetValue().value()
            : 0.0;
    if (is_simulation_) {
      // Phoenix models supply current from rotor physics, but provides no
      // temperature input. Report ambient, not a fictitious thermal model.
      out.temperature_c = 25.0;
    }
    if (out.fault_flags != last_fault_flags_[i]) {
      const uint32_t prev = last_fault_flags_[i];
      const uint32_t raised = out.fault_flags & ~prev;
      const uint32_t cleared = prev & ~out.fault_flags;
      std::cerr << "[phoenix] motor " << motor_config.name << " (mask_bit=" << i
                << ", id=" << motor_config.id << ", bus=" << motor_config.bus
                << ", can_id=" << motor_config.can_id << ") fault_flags=0x"
                << std::hex << out.fault_flags << " (was 0x" << prev << ")"
                << std::dec;
      if (raised != 0)
        std::cerr << " raised=[" << DecodeTalonFaultBits(raised) << "]";
      if (cleared != 0)
        std::cerr << " cleared=[" << DecodeTalonFaultBits(cleared) << "]";
      std::cerr << " active=[" << DecodeTalonFaultBits(out.fault_flags) << "]"
                << " pos=" << out.position_rot
                << " rot vel=" << out.velocity_rps << " rps V=" << out.voltage
                << "V I=" << out.stator_current_a
                << "A (limits: supply=" << motor_config.supply_limit_a
                << "A stator=" << motor_config.stator_limit_a << "A)\n";
      last_fault_flags_[i] = out.fault_flags;
    }
    out.valid = Fresh(p) && Fresh(v) && std::isfinite(out.position_rot) &&
                std::isfinite(out.velocity_rps) &&
                std::find(unavailable_.begin(), unavailable_.end(),
                          motor_config.id) == unavailable_.end();
    if (out.valid != last_motor_valid_[i]) {
      LogFeedbackChange("motor", motor_config.name, motor_config.id,
                        motor_config.bus, motor_config.can_id, out.valid, p, v);
      last_motor_valid_[i] = out.valid;
    }
    healthy |= out.valid;
    if (!out.valid) {
      out.position_rot = 0;
      out.velocity_rps = 0;
      out.voltage = 0;
      out.stator_current_a = 0;
    }
  }
  for (std::size_t i = 0; i < sensors_.size(); ++i) {
    auto& device = sensors_[i];
    auto& out = s.sensors[i];
    const ph::BaseStatusSignal* position_signal;
    const ph::BaseStatusSignal* velocity_signal;
    if (device.encoder) {
      auto& p = device.encoder->GetAbsolutePosition(false);
      auto& v = device.encoder->GetVelocity(false);
      position_signal = &p;
      velocity_signal = &v;
      out.position_rot = p.GetValue().value();
      out.velocity_rps = v.GetValue().value();
      out.valid = Fresh(p) && Fresh(v);
      out.position_age_us = Age(p);
      out.velocity_age_us = Age(v);
    } else {
      auto& p = device.imu->GetYaw(false);
      auto& v = device.imu->GetAngularVelocityZWorld(false);
      position_signal = &p;
      velocity_signal = &v;
      const double sign = config_.sensors[i].inverted ? -1.0 : 1.0;
      out.position_rot = sign * p.GetValue().value() / 360.0;
      out.velocity_rps = sign * v.GetValue().value() / 360.0;
      out.valid = Fresh(p) && Fresh(v);
      out.position_age_us = Age(p);
      out.velocity_age_us = Age(v);
    }
    out.valid &=
        std::isfinite(out.position_rot) && std::isfinite(out.velocity_rps);
    if (out.valid != last_sensor_valid_[i]) {
      const auto& sensor = config_.sensors[i];
      LogFeedbackChange("sensor", sensor.name, sensor.id, sensor.bus,
                        sensor.can_id, out.valid, *position_signal,
                        *velocity_signal);
      last_sensor_valid_[i] = out.valid;
    }
    healthy |= out.valid;
    if (!out.valid) {
      out.position_rot = 0;
      out.velocity_rps = 0;
    }
  }
  for (std::size_t i = 0; i < digital_inputs_.size(); ++i) {
    auto& out = s.digital_inputs[i];
    out.value = digital_inputs_[i]->Get();
    out.valid = true;
  }
  for (std::size_t i = 0; i < digital_outputs_.size(); ++i) {
    auto& out = s.digital_outputs[i];
    out.value = digital_outputs_[i]->Get();
    out.valid = true;
  }
  for (std::size_t i = 0; i < analog_inputs_.size(); ++i) {
    auto& out = s.analog_inputs[i];
    out.voltage = analog_inputs_[i]->GetVoltage();
    out.valid = true;
    out.age_us = 0;
  }
  for (std::size_t i = 0; i < rio_encoders_.size(); ++i) {
    auto& out = s.encoders[i];
    auto& dev = rio_encoders_[i];
    if (dev.duty_cycle) {
      const bool connected = dev.duty_cycle->IsConnected();
      const double pos = dev.duty_cycle->Get() - dev.config.offset_rot;
      double vel = 0.0;
      if (connected) {
        const auto now = std::chrono::steady_clock::now();
        if (dev.last_time && now > *dev.last_time) {
          const double dt =
              std::chrono::duration<double>(now - *dev.last_time).count();
          const double delta_rot =
              std::remainder(pos - dev.last_position_rot, 1.0);
          if (dt > 0.0) vel = delta_rot / dt;
        }
        dev.last_position_rot = pos;
        dev.last_time = now;
      } else {
        dev.last_time.reset();
        dev.last_position_rot = 0.0;
      }
      out.position_rot = pos;
      out.velocity_rps = vel;
      out.valid = connected && std::isfinite(pos) && std::isfinite(vel);
      out.age_us = 0;
    } else {
      const double pos = dev.quadrature->GetDistance() - dev.config.offset_rot;
      const double vel = dev.quadrature->GetRate();
      out.position_rot = pos;
      out.velocity_rps = vel;
      out.valid = std::isfinite(pos) && std::isfinite(vel);
      out.age_us = 0;
    }
    // RIO encoders are auxiliary feedback (TalonFX loops use
    // rotor/RemoteCANcoder), so an invalid encoder marks only its own sample:
    // mechanism scope, never a whole-robot stop.
    if (!out.valid) {
      out.position_rot = 0;
      out.velocity_rps = 0;
    }
  }
  for (std::size_t i = 0; i < pwm_outputs_.size(); ++i) {
    auto& out = s.pwm_outputs[i];
    out.output = pwm_outputs_[i]->GetSpeed();
    out.valid = true;
  }
  uint32_t motor_fault_mask = 0;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    if (s.motors[i].fault_flags != 0) motor_fault_mask |= uint32_t{1} << i;
  }
  if (motor_fault_mask != last_motor_fault_mask_) {
    std::cerr << "[phoenix] motor_fault_mask changed: 0x" << std::hex
              << last_motor_fault_mask_ << " -> 0x" << motor_fault_mask
              << std::dec << " (active motors: ";
    bool first = true;
    for (std::size_t i = 0; i < motors_.size(); ++i) {
      if (s.motors[i].fault_flags == 0) continue;
      if (!first) std::cerr << ", ";
      const auto& motor = config_.motors[i];
      std::cerr << i << ":" << motor.name << "(" << motor.can_id << ")";
      first = false;
    }
    if (first) std::cerr << "none";
    std::cerr << ")\n";
    last_motor_fault_mask_ = motor_fault_mask;
  }
  if (s.health.brownout != last_brownout_) {
    std::cerr << "[phoenix] power brownout "
              << (s.health.brownout ? "ACTIVE" : "CLEARED")
              << " (battery_v=" << s.health.battery_v << "V)\n";
    last_brownout_ = s.health.brownout;
  }
  return healthy;
}
bool PhoenixBackend::Apply(std::span<const hw::MotorRequest> commands) {
  apply_failed_.clear();
  if (commands.size() != motors_.size()) return false;
  // A failed send is local to its motor: try neutral on that motor only, keep
  // applying the rest, and report it so Gateway quarantines its chain. Log
  // only ok<->failing transitions; an isolated motor is retried per command.
  const auto note_send = [&](std::size_t i,
                             const ctre::phoenix::StatusCode& status) {
    const bool failed = !status.IsOK();
    const auto& config = config_.motors[i];
    if (failed != apply_failing_[i]) {
      std::cerr << "[phoenix] apply control "
                << (failed ? "failed" : "recovered") << " on motor "
                << config.name << " (can_id=" << config.can_id
                << "): " << status.GetName()
                << (failed ? "; neutralizing only this motor" : "") << "\n";
    }
    apply_failing_[i] = failed;
    if (!failed) return;
    motors_[i]->SetControl(ph::controls::NeutralOut{}.WithUpdateFreqHz(0_Hz));
    sim_neutral_[i] = true;
    apply_failed_.push_back(config.id);
  };
  for (std::size_t i = 0; i < commands.size(); ++i) {
    const auto& r = commands[i];
    auto& m = *motors_[i];
    ctre::phoenix::StatusCode result;
    const auto& config = config_.motors[i];
    if (std::find(unavailable_.begin(), unavailable_.end(), config.id) !=
        unavailable_.end()) {
      // A missing device cannot acknowledge NeutralOut. Keep trying neutral,
      // but do not turn its transport failure into a whole-robot Apply failure.
      m.SetControl(ph::controls::NeutralOut{}.WithUpdateFreqHz(0_Hz));
      sim_neutral_[i] = true;
      continue;
    }
    if (config.master_id != 0) {
      const auto master =
          std::find_if(config_.motors.begin(), config_.motors.end(),
                       [&](const auto& candidate) {
                         return candidate.id == config.master_id;
                       });
      if (master == config_.motors.end()) return false;
      const auto master_index =
          static_cast<std::size_t>(master - config_.motors.begin());
      auto& position = motors_[master_index]->GetPosition(false);
      auto& velocity = motors_[master_index]->GetVelocity(false);
      bool feedback_valid =
          ValidMotionFeedback(m.GetPosition(false), m.GetVelocity(false));
      if (config.feedback == hw::Feedback::kRemoteCANcoder) {
        bool sensor_valid = false;
        for (std::size_t j = 0; j < config_.sensors.size(); ++j) {
          if (config_.sensors[j].id == config.feedback_sensor_id &&
              sensors_[j].encoder) {
            sensor_valid = ValidMotionFeedback(
                sensors_[j].encoder->GetAbsolutePosition(false),
                sensors_[j].encoder->GetVelocity(false));
            break;
          }
        }
        feedback_valid &= sensor_valid;
      }
      const bool neutral = r.mode == hw::Mode::kNeutral ||
                           commands[master_index].mode == hw::Mode::kNeutral ||
                           !ValidMotionFeedback(position, velocity) ||
                           !feedback_valid;
      sim_neutral_[i] = neutral;
      if (neutral) {
        result = m.SetControl(ph::controls::NeutralOut{});
      } else {
        result = m.SetControl(
            ph::controls::Follower{master->can_id, config.oppose_master}
                .WithUpdateFreqHz(0_Hz));
      }
      note_send(i, result);
      continue;
    }
    sim_neutral_[i] = r.mode == hw::Mode::kNeutral;
    switch (r.mode) {
      case hw::Mode::kNeutral:
        result = m.SetControl(ph::controls::NeutralOut{});
        break;
      case hw::Mode::kDutyCycle:
        result = m.SetControl(ph::controls::DutyCycleOut{r.demand}
                                  .WithEnableFOC(false)
                                  .WithUpdateFreqHz(0_Hz));
        break;
      case hw::Mode::kVoltage:
        result = m.SetControl(ph::controls::VoltageOut{units::volt_t{r.demand}}
                                  .WithEnableFOC(false)
                                  .WithUpdateFreqHz(0_Hz));
        break;
      case hw::Mode::kVelocity:
        result = m.SetControl(
            ph::controls::VelocityVoltage{units::turns_per_second_t{r.demand}}
                .WithSlot(r.slot)
                .WithFeedForward(units::volt_t{r.feedforward_v})
                .WithEnableFOC(false)
                .WithUpdateFreqHz(0_Hz));
        break;
      case hw::Mode::kPosition:
        result =
            m.SetControl(ph::controls::PositionVoltage{units::turn_t{r.demand}}
                             .WithSlot(r.slot)
                             .WithFeedForward(units::volt_t{r.feedforward_v})
                             .WithEnableFOC(false)
                             .WithUpdateFreqHz(0_Hz));
        break;
      case hw::Mode::kMotionMagic:
        result = m.SetControl(
            ph::controls::MotionMagicVoltage{units::turn_t{r.demand}}
                .WithSlot(r.slot)
                .WithFeedForward(units::volt_t{r.feedforward_v})
                .WithEnableFOC(false)
                .WithUpdateFreqHz(0_Hz));
        break;
      default:
        return false;
    }
    note_send(i, result);
  }
  return true;
}
bool PhoenixBackend::ApplyOutputs(
    std::span<const hw::DigitalOutputRequest> digital_outputs,
    std::span<const hw::PwmRequest> pwm_outputs) {
  for (std::size_t i = 0;
       i < digital_outputs.size() && i < digital_outputs_.size(); ++i) {
    digital_outputs_[i]->Set(digital_outputs[i].value);
  }
  for (std::size_t i = 0; i < pwm_outputs.size() && i < pwm_outputs_.size();
       ++i) {
    pwm_outputs_[i]->SetSpeed(pwm_outputs[i].output);
  }
  return true;
}
void PhoenixBackend::Neutral() {
  // Gateway refreshes neutral every tick without an active command, so a
  // failed send here is retried on the next tick. Count every failure and log
  // only each motor's ok<->failing transition to avoid a 200 Hz log flood.
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const auto status = motors_[i]->SetControl(
        ph::controls::NeutralOut{}.WithUpdateFreqHz(0_Hz));
    const bool failed = !status.IsOK();
    if (failed) ++neutral_send_failure_count_;
    if (failed != neutral_send_failing_[i] && i < config_.motors.size()) {
      const auto& motor = config_.motors[i];
      std::cerr << "[phoenix] neutral send "
                << (failed ? "failing" : "recovered") << " on motor "
                << motor.name << " (can_id=" << motor.can_id
                << "): " << status.GetName() << "\n";
    }
    neutral_send_failing_[i] = failed;
  }
  for (auto& pwm : pwm_outputs_) pwm->SetDisabled();
  // Support matrix: DIO outputs de-energize on every safe-state transition.
  for (auto& dio : digital_outputs_) dio->Set(false);
  // Sim: neutral keeps rotor momentum; UpdateSimulation decays it per brake.
  if (is_simulation_) sim_neutral_.fill(true);
}

void PhoenixBackend::UpdateSimulation(double dt) {
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const auto& m_cfg = config_.motors[i];
    auto& sim = motors_[i]->GetSimState();
    double gear_ratio =
        m_cfg.rotor_to_sensor_ratio * m_cfg.sensor_to_mechanism_ratio;
    if (gear_ratio <= 0.0) gear_ratio = 1.0;

    // Voltage is the only driver (Kraken X60 free speed: 100 rps at 12 V).
    // A closed-loop request with missing gains produces ~0 V, so the rotor
    // coasts to a stop here instead of jumping to the requested demand. Never
    // consult the request demand for physics: that shortcut hides bad tuning.
    // Neutral is unpowered: velocity decays per the brake config and the
    // position keeps integrating while the rotor rolls on.
    double target_rotor_rps = 0.0;
    double tau = 0.01;
    if (sim_neutral_[i]) {
      tau = m_cfg.brake ? kSimNeutralBrakeTauS : kSimNeutralCoastTauS;
    } else {
      const double volts = sim.GetMotorVoltage().value();
      target_rotor_rps = (volts / 12.0) * 100.0;
    }
    const double alpha = std::min(1.0, dt / tau);
    sim_rotor_velocity_[i] +=
        (target_rotor_rps - sim_rotor_velocity_[i]) * alpha;
    if (sim_neutral_[i] &&
        std::abs(sim_rotor_velocity_[i]) < kSimNeutralStandstillRps) {
      sim_rotor_velocity_[i] = 0.0;
    }

    sim_rotor_position_[i] += sim_rotor_velocity_[i] * dt;
    sim.SetRotorVelocity(units::turns_per_second_t{sim_rotor_velocity_[i]});
    sim.SetRawRotorPosition(units::turn_t{sim_rotor_position_[i]});

    // Sync remote CANcoder if mapped
    int sensor_idx = motor_to_sensor_map_[i];
    if (sensor_idx >= 0 &&
        static_cast<std::size_t>(sensor_idx) < sensors_.size() &&
        sensors_[sensor_idx].encoder) {
      double mech_rot = sim_rotor_position_[i] / gear_ratio;
      double mech_rps = sim_rotor_velocity_[i] / gear_ratio;
      sensors_[sensor_idx].encoder->GetSimState().SetRawPosition(
          units::turn_t{mech_rot});
      sensors_[sensor_idx].encoder->GetSimState().SetVelocity(
          units::turns_per_second_t{mech_rps});
    }
  }
}

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
#include <limits>
#include <string>

#include "hardware/gate.h"

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
// Diagnostics only (effort, fault field, temperature). Motion feedback is not
// held to an age here: Read reports its ages and the gateway judges them
// (Backend::Read, FeedbackFresh).
bool Fresh(const ph::BaseStatusSignal& signal,
           units::second_t max_age = 50_ms) {
  return signal.GetStatus().IsOK() &&
         signal.GetTimestamp().GetLatency() < max_age;
}
// One clock_gettime(CLOCK_MONOTONIC) per phase boundary of the tick.
uint64_t NowNs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}
// Adds the time from construction to destruction to `phase`, for a call
// with several returns.
class PhaseTimer {
 public:
  PhaseTimer(talos::rio::RioLog& log, talos::rio::TickPhase phase)
      : log_{log}, phase_{phase}, start_ns_{NowNs()} {}
  ~PhaseTimer() { log_.AddPhase(phase_, NowNs() - start_ns_); }
  PhaseTimer(const PhaseTimer&) = delete;
  PhaseTimer& operator=(const PhaseTimer&) = delete;

 private:
  talos::rio::RioLog& log_;
  talos::rio::TickPhase phase_;
  uint64_t start_ns_;
};
uint32_t Age(const ph::BaseStatusSignal& signal) {
  const double age = signal.GetTimestamp().GetLatency().value() * 1e6;
  if (!std::isfinite(age) || age >= std::numeric_limits<uint32_t>::max())
    return std::numeric_limits<uint32_t>::max();
  return static_cast<uint32_t>(std::max(0.0, std::ceil(age)));
}
// Apply's follower check, between the gateway's judgements: the master's or
// the follower's own (or remote) feedback, by the gateway's rule.
template <typename Position, typename Velocity>
bool ValidMotionFeedback(const Position& position, const Velocity& velocity,
                         uint64_t command_timeout_us) {
  return position.GetStatus().IsOK() && velocity.GetStatus().IsOK() &&
         hw::FeedbackFresh(Age(position), command_timeout_us) &&
         hw::FeedbackFresh(Age(velocity), command_timeout_us) &&
         std::isfinite(position.GetValue().value()) &&
         std::isfinite(velocity.GetValue().value());
}

// Posts a feedback validity change: the loop thread only copies it.
void LogFeedbackChange(talos::rio::RioLog& log, talos::rio::LogKind kind,
                       std::size_t index, const std::string& name, uint16_t id,
                       const std::string& bus, int can_id, bool valid,
                       const ph::BaseStatusSignal& p,
                       const ph::BaseStatusSignal& v) {
  talos::rio::LogEvent event;
  event.kind = kind;
  event.index = static_cast<uint16_t>(index);
  event.id = id;
  event.can_id = can_id;
  event.flag = valid;
  event.status = p.GetStatus().GetName();
  event.status2 = v.GetStatus().GetName();
  event.age_us = Age(p);
  event.age2_us = Age(v);
  event.SetName(name);
  event.SetBus(bus);
  log.Post(event);
}

// Posts LATE when the older of a device's position and velocity samples
// reaches `late_us` (four status periods), and ON TIME, with the oldest age of
// that spell, once both are younger again: a CAN silence shorter than the
// gateway's bound changes no validity, so this line is what shows it.
// Loop thread: a copy into the queue, never a write.
void NoteLateness(talos::rio::RioLog& log, talos::rio::LogKind kind,
                  std::size_t index, const std::string& name, uint16_t id,
                  const std::string& bus, int can_id, uint32_t late_us,
                  const ph::BaseStatusSignal& p, const ph::BaseStatusSignal& v,
                  bool& late, uint32_t& peak_age_us) {
  const uint32_t age_us = std::max(Age(p), Age(v));
  const bool now_late = age_us >= late_us;
  if (now_late) peak_age_us = std::max(peak_age_us, age_us);
  if (now_late == late) return;
  talos::rio::LogEvent event;
  event.kind = kind;
  event.index = static_cast<uint16_t>(index);
  event.id = id;
  event.can_id = can_id;
  event.flag = now_late;
  event.value = peak_age_us;
  event.previous = late_us;
  event.status = p.GetStatus().GetName();
  event.status2 = v.GetStatus().GetName();
  event.age_us = Age(p);
  event.age2_us = Age(v);
  event.SetName(name);
  event.SetBus(bus);
  log.Post(event);
  late = now_late;
  if (!now_late) peak_age_us = 0;
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
// RioLog kMessage index of the configure-failure line (its rate-limit key).
constexpr uint16_t kConfigureFailedMessage = 0;
}  // namespace

PhoenixBackend::PhoenixBackend(bool is_simulation, talos::rio::RioLog* log)
    : owned_log_{log ? nullptr : std::make_unique<talos::rio::RioLog>()},
      log_{log ? log : owned_log_.get()},
      is_simulation_{is_simulation} {
  if (owned_log_) owned_log_->Start();
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
  const std::lock_guard devices_lock{devices_mutex_};
  hw::Validate(c);
  restore_operations_.clear();
  unavailable_.clear();
  signal_ids_.clear();
  last_error_.reset();
  last_fault_flags_.fill(0);
  last_motor_valid_.fill(true);
  last_sensor_valid_.fill(true);
  motor_late_.fill(false);
  motor_peak_age_us_.fill(0);
  sensor_late_.fill(false);
  sensor_peak_age_us_.fill(0);
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
    // The gateway applies the config on every session start; no beep for it.
    // The boot beep stays: it is how a power cycle is heard.
    cfg.Audio.BeepOnConfig = false;
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
    // Switches on the TalonFX's own limit pins; the defaults (enabled,
    // normally open) are Phoenix's, so an unconfigured motor is unchanged.
    auto& limits = cfg.HardwareLimitSwitch;
    limits.ForwardLimitSource =
        ph::signals::ForwardLimitSourceValue::LimitSwitchPin;
    limits.ForwardLimitEnable =
        m.forward_limit_switch != hw::LimitSwitch::kDisabled;
    limits.ForwardLimitType =
        m.forward_limit_switch == hw::LimitSwitch::kNormallyClosed
            ? ph::signals::ForwardLimitTypeValue::NormallyClosed
            : ph::signals::ForwardLimitTypeValue::NormallyOpen;
    limits.ReverseLimitSource =
        ph::signals::ReverseLimitSourceValue::LimitSwitchPin;
    limits.ReverseLimitEnable =
        m.reverse_limit_switch != hw::LimitSwitch::kDisabled;
    limits.ReverseLimitType =
        m.reverse_limit_switch == hw::LimitSwitch::kNormallyClosed
            ? ph::signals::ReverseLimitTypeValue::NormallyClosed
            : ph::signals::ReverseLimitTypeValue::NormallyOpen;
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
    // Configure runs on the configure worker; it still posts, so its line
    // stays ordered with the loop thread's.
    talos::rio::LogEvent event;
    event.index = kConfigureFailedMessage;
    event.SetText("[phoenix] " + hw::FormatConfigError(error));
    log_->Post(event);
  }
  // Structural validity succeeded; unavailable devices are isolated and
  // retried.
  return true;
}

bool PhoenixBackend::PollDeviceResets(std::vector<uint16_t>& reset_ids) {
  const PhaseTimer timer{*log_, talos::rio::TickPhase::kDevices};
  // HasResetOccurred reports (and clears) one device's reset latch.
  reset_ids.clear();
  for (std::size_t i = 0; i < motors_.size() && i < config_.motors.size();
       ++i) {
    if (motors_[i]->HasResetOccurred()) {
      const auto& motor = config_.motors[i];
      reset_ids.push_back(motor.id);
      talos::rio::LogEvent event;
      event.kind = talos::rio::LogKind::kDeviceReset;
      event.index = static_cast<uint16_t>(i);
      event.id = motor.id;
      event.can_id = motor.can_id;
      event.SetName(motor.name);
      event.SetBus(motor.bus);
      log_->Post(event);
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
      talos::rio::LogEvent event;
      event.kind = talos::rio::LogKind::kDeviceReset;
      // Sensor slots follow the motor slots in the rate-limit key.
      event.index = static_cast<uint16_t>(hw::kMaxMotors + i);
      event.flag = true;
      event.id = sensor.id;
      event.can_id = sensor.can_id;
      event.SetName(sensor.name);
      event.SetBus(sensor.bus);
      log_->Post(event);
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
  using talos::rio::TickPhase;
  const uint64_t read_start_ns = NowNs();
  s.health = {true, frc::RobotController::GetBatteryVoltage().value(),
              frc::RobotController::IsBrownedOut(),
              frc::RobotController::GetCANStatus().percentBusUtilization};
  uint64_t refresh_start_ns = NowNs();
  log_->AddPhase(TickPhase::kHealth, refresh_start_ns - read_start_ns);
  if (is_simulation_) {
    const auto now = std::chrono::steady_clock::now();
    double dt =
        last_sim_time_
            ? std::chrono::duration<double>(now - *last_sim_time_).count()
            : 0.005;
    if (dt <= 0.0 || dt > 0.05) dt = 0.005;
    last_sim_time_ = now;
    UpdateSimulation(dt);
    const uint64_t simulated_ns = NowNs();
    log_->AddPhase(TickPhase::kOther, simulated_ns - refresh_start_ns);
    refresh_start_ns = simulated_ns;
  }
  ph::BaseStatusSignal::RefreshAll(
      false, std::span<ph::BaseStatusSignal* const>{signals_});
  const uint64_t devices_start_ns = NowNs();
  log_->AddPhase(TickPhase::kRefresh, devices_start_ns - refresh_start_ns);
  bool healthy = motors_.empty() && sensors_.empty();
  // Four status periods: the LATE line's threshold (NoteLateness).
  const auto late_us =
      static_cast<uint32_t>(std::ceil(4'000'000.0 / config_.status_hz));
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
    // FaultField is its own 50 Hz signal, separate from the motion feedback
    // that sets `valid`. A stale FaultField reports no asserted limit, so a
    // limit getter never reports an old switch state, but keeps its last
    // fault bits: losing the signal must not un-fault a faulted motor.
    auto& faults = motor.GetFaultField(false);
    out.fault_flags = faults.GetValue();
    if (!Fresh(faults)) out.fault_flags &= ~hw::kMotorLimitAssertedFlags;
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
      // Current-limit bits toggle at the fault-field rate under hard
      // acceleration: post only, and let the logger coalesce them.
      talos::rio::LogEvent event;
      event.kind = talos::rio::LogKind::kMotorFault;
      event.index = static_cast<uint16_t>(i);
      event.id = motor_config.id;
      event.can_id = motor_config.can_id;
      event.value = out.fault_flags;
      event.previous = last_fault_flags_[i];
      event.a = out.position_rot;
      event.b = out.velocity_rps;
      event.c = out.voltage;
      event.d = out.stator_current_a;
      event.limit1 = motor_config.supply_limit_a;
      event.limit2 = motor_config.stator_limit_a;
      event.SetName(motor_config.name);
      event.SetBus(motor_config.bus);
      log_->Post(event);
      last_fault_flags_[i] = out.fault_flags;
    }
    // Status, value and isolation only: the gateway judges the ages.
    out.valid =
        p.GetStatus().IsOK() && v.GetStatus().IsOK() &&
        std::isfinite(out.position_rot) && std::isfinite(out.velocity_rps) &&
        std::find(unavailable_.begin(), unavailable_.end(), motor_config.id) ==
            unavailable_.end();
    if (out.valid != last_motor_valid_[i]) {
      LogFeedbackChange(*log_, talos::rio::LogKind::kMotorFeedback, i,
                        motor_config.name, motor_config.id, motor_config.bus,
                        motor_config.can_id, out.valid, p, v);
      last_motor_valid_[i] = out.valid;
    }
    NoteLateness(*log_, talos::rio::LogKind::kMotorFeedbackLate, i,
                 motor_config.name, motor_config.id, motor_config.bus,
                 motor_config.can_id, late_us, p, v, motor_late_[i],
                 motor_peak_age_us_[i]);
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
      out.valid = p.GetStatus().IsOK() && v.GetStatus().IsOK();
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
      out.valid = p.GetStatus().IsOK() && v.GetStatus().IsOK();
      out.position_age_us = Age(p);
      out.velocity_age_us = Age(v);
    }
    // Status and value only: the gateway judges the ages.
    out.valid &=
        std::isfinite(out.position_rot) && std::isfinite(out.velocity_rps);
    const auto& sensor = config_.sensors[i];
    if (out.valid != last_sensor_valid_[i]) {
      LogFeedbackChange(*log_, talos::rio::LogKind::kSensorFeedback, i,
                        sensor.name, sensor.id, sensor.bus, sensor.can_id,
                        out.valid, *position_signal, *velocity_signal);
      last_sensor_valid_[i] = out.valid;
    }
    NoteLateness(*log_, talos::rio::LogKind::kSensorFeedbackLate, i,
                 sensor.name, sensor.id, sensor.bus, sensor.can_id, late_us,
                 *position_signal, *velocity_signal, sensor_late_[i],
                 sensor_peak_age_us_[i]);
    healthy |= out.valid;
    if (!out.valid) {
      out.position_rot = 0;
      out.velocity_rps = 0;
    }
  }
  const uint64_t devices_end_ns = NowNs();
  log_->AddPhase(TickPhase::kDevices, devices_end_ns - devices_start_ns);
  log_->SetTickDevices(motors_.size(), sensors_.size());
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
    if (hw::MotorFaulted(s.motors[i].fault_flags))
      motor_fault_mask |= uint32_t{1} << i;
  }
  if (motor_fault_mask != last_motor_fault_mask_) {
    talos::rio::LogEvent event;
    event.kind = talos::rio::LogKind::kMotorFaultMask;
    event.value = motor_fault_mask;
    event.previous = last_motor_fault_mask_;
    log_->Post(event);
    last_motor_fault_mask_ = motor_fault_mask;
  }
  if (s.health.brownout != last_brownout_) {
    talos::rio::LogEvent event;
    event.kind = talos::rio::LogKind::kBrownout;
    event.flag = s.health.brownout;
    event.a = s.health.battery_v;
    log_->Post(event);
    last_brownout_ = s.health.brownout;
  }
  log_->AddPhase(TickPhase::kOther, NowNs() - devices_end_ns);
  return healthy;
}
bool PhoenixBackend::Apply(std::span<const hw::MotorRequest> commands) {
  const PhaseTimer timer{*log_, talos::rio::TickPhase::kControl};
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
      talos::rio::LogEvent event;
      event.kind = talos::rio::LogKind::kApplyControl;
      event.index = static_cast<uint16_t>(i);
      event.can_id = config.can_id;
      event.flag = failed;
      event.status = status.GetName();
      event.SetName(config.name);
      log_->Post(event);
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
      const uint64_t bound = config_.command_timeout_us;
      bool feedback_valid = ValidMotionFeedback(m.GetPosition(false),
                                                m.GetVelocity(false), bound);
      if (config.feedback == hw::Feedback::kRemoteCANcoder) {
        bool sensor_valid = false;
        for (std::size_t j = 0; j < config_.sensors.size(); ++j) {
          if (config_.sensors[j].id == config.feedback_sensor_id &&
              sensors_[j].encoder) {
            sensor_valid = ValidMotionFeedback(
                sensors_[j].encoder->GetAbsolutePosition(false),
                sensors_[j].encoder->GetVelocity(false), bound);
            break;
          }
        }
        feedback_valid &= sensor_valid;
      }
      const bool neutral = r.mode == hw::Mode::kNeutral ||
                           commands[master_index].mode == hw::Mode::kNeutral ||
                           !ValidMotionFeedback(position, velocity, bound) ||
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
  const PhaseTimer timer{*log_, talos::rio::TickPhase::kOther};
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
  const uint64_t control_start_ns = NowNs();
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const auto status = motors_[i]->SetControl(
        ph::controls::NeutralOut{}.WithUpdateFreqHz(0_Hz));
    const bool failed = !status.IsOK();
    if (failed) ++neutral_send_failure_count_;
    if (failed != neutral_send_failing_[i] && i < config_.motors.size()) {
      const auto& motor = config_.motors[i];
      talos::rio::LogEvent event;
      event.kind = talos::rio::LogKind::kNeutralSend;
      event.index = static_cast<uint16_t>(i);
      event.can_id = motor.can_id;
      event.flag = failed;
      event.status = status.GetName();
      event.SetName(motor.name);
      log_->Post(event);
    }
    neutral_send_failing_[i] = failed;
  }
  const uint64_t control_end_ns = NowNs();
  log_->AddPhase(talos::rio::TickPhase::kControl,
                 control_end_ns - control_start_ns);
  for (auto& pwm : pwm_outputs_) pwm->SetDisabled();
  // Support matrix: DIO outputs de-energize on every safe-state transition.
  for (auto& dio : digital_outputs_) dio->Set(false);
  // Sim: neutral keeps rotor momentum; UpdateSimulation decays it per brake.
  if (is_simulation_) sim_neutral_.fill(true);
  log_->AddPhase(talos::rio::TickPhase::kOther, NowNs() - control_end_ns);
}

bool PhoenixBackend::WatchdogNeutral() {
  const std::unique_lock devices_lock{devices_mutex_, std::try_to_lock};
  if (!devices_lock.owns_lock()) return false;
  // Phoenix 6 serializes SetControl per device (ParentDevice's control lock)
  // and the HAL DIO and PWM writes are safe from any thread, so this may run
  // beside the loop thread's own sends; the last one sent wins. PWM and DIO
  // first: a motor whose control lock the stalled loop holds would hold them
  // back too.
  for (auto& pwm : pwm_outputs_) pwm->SetDisabled();
  for (auto& dio : digital_outputs_) dio->Set(false);
  bool sent = true;
  for (auto& motor : motors_) {
    sent &= motor->SetControl(ph::controls::NeutralOut{}.WithUpdateFreqHz(0_Hz))
                .IsOK();
  }
  return sent;
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

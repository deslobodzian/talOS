#include "PhoenixBackend.h"

#include <algorithm>
#include <cmath>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <limits>

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
}
bool Fresh(const ph::BaseStatusSignal& signal) {
  return signal.GetStatus().IsOK() &&
         signal.GetTimestamp().GetLatency() < 50_ms;
}
uint32_t Age(const ph::BaseStatusSignal& signal) {
  const double age = signal.GetTimestamp().GetLatency().value() * 1e6;
  if (!std::isfinite(age) || age >= std::numeric_limits<uint32_t>::max())
    return std::numeric_limits<uint32_t>::max();
  return static_cast<uint32_t>(std::max(0.0, std::ceil(age)));
}
}  // namespace

PhoenixBackend::PhoenixBackend(bool is_simulation) : is_simulation_{is_simulation} {}

bool PhoenixBackend::Configure(const hw::Config& c) {
  hw::Validate(c);
  config_ = c;
  signals_.clear();
  motors_.clear();
  sensors_.clear();
  digital_inputs_.clear();
  digital_outputs_.clear();
  analog_inputs_.clear();
  encoders_.clear();
  pwm_outputs_.clear();

  sim_rotor_velocity_.fill(0.0);
  sim_rotor_position_.fill(0.0);
  motor_to_sensor_map_.fill(-1);
  last_commands_.fill(hw::MotorRequest{});
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
    if (e.dio_b >= 0) {
      encoders_.push_back(std::make_unique<frc::Encoder>(e.dio_a, e.dio_b));
    } else {
      encoders_.push_back(std::make_unique<frc::Encoder>(e.dio_a, e.dio_a));
    }
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
      ok &= device.encoder->GetConfigurator().Apply(cfg).IsOK();
      signals_.push_back(&device.encoder->GetAbsolutePosition(false));
      signals_.push_back(&device.encoder->GetVelocity(false));
    } else {
      device.imu =
          std::make_unique<ph::hardware::Pigeon2>(s.can_id, ph::CANBus{s.bus});
      ph::configs::Pigeon2Configuration cfg;
      ok &= device.imu->GetConfigurator().Apply(cfg).IsOK();
      signals_.push_back(&device.imu->GetYaw(false));
      signals_.push_back(&device.imu->GetAngularVelocityZWorld(false));
    }
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
    ok &= motor->GetConfigurator().Apply(cfg).IsOK();
    signals_.push_back(&motor->GetPosition(false));
    signals_.push_back(&motor->GetVelocity(false));
    signals_.push_back(&motor->GetMotorVoltage(false));
    signals_.push_back(&motor->GetStatorCurrent(false));
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

  for (auto* signal : signals_)
    ok &= signal->SetUpdateFrequency(units::hertz_t{c.status_hz}).IsOK();
  return ok;
}

bool PhoenixBackend::Read(hw::State& s) {
  if (is_simulation_) {
    const auto now = std::chrono::steady_clock::now();
    double dt = last_sim_time_
                    ? std::chrono::duration<double>(now - *last_sim_time_).count()
                    : 0.005;
    if (dt <= 0.0 || dt > 0.05) dt = 0.005;
    last_sim_time_ = now;
    UpdateSimulation(dt);
  }
  ph::BaseStatusSignal::RefreshAll(
      false, std::span<ph::BaseStatusSignal* const>{signals_});
  bool healthy = true;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    auto& motor = *motors_[i];
    auto& out = s.motors[i];
    auto& p = motor.GetPosition(false);
    auto& v = motor.GetVelocity(false);
    auto& volts = motor.GetMotorVoltage(false);
    auto& amps = motor.GetStatorCurrent(false);
    out.position_rot = p.GetValue().value();
    out.velocity_rps = v.GetValue().value();
    out.position_age_us = Age(p);
    out.velocity_age_us = Age(v);
    out.voltage = volts.GetValue().value();
    out.stator_current_a = amps.GetValue().value();
    out.valid = Fresh(p) && Fresh(v) && Fresh(volts) && Fresh(amps) &&
                std::isfinite(out.position_rot) &&
                std::isfinite(out.velocity_rps) && std::isfinite(out.voltage) &&
                std::isfinite(out.stator_current_a);
    healthy &= out.valid;
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
    if (device.encoder) {
      auto& p = device.encoder->GetAbsolutePosition(false);
      auto& v = device.encoder->GetVelocity(false);
      out.position_rot = p.GetValue().value();
      out.velocity_rps = v.GetValue().value();
      out.valid = Fresh(p) && Fresh(v);
      out.position_age_us = Age(p);
      out.velocity_age_us = Age(v);
    } else {
      auto& p = device.imu->GetYaw(false);
      auto& v = device.imu->GetAngularVelocityZWorld(false);
      out.position_rot = p.GetValue().value() / 360.0;
      out.velocity_rps = v.GetValue().value() / 360.0;
      out.valid = Fresh(p) && Fresh(v);
      out.position_age_us = Age(p);
      out.velocity_age_us = Age(v);
    }
    out.valid &=
        std::isfinite(out.position_rot) && std::isfinite(out.velocity_rps);
    healthy &= out.valid;
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
  for (std::size_t i = 0; i < encoders_.size(); ++i) {
    auto& out = s.encoders[i];
    out.position_rot = encoders_[i]->GetDistance();
    out.velocity_rps = encoders_[i]->GetRate();
    out.valid = true;
    out.age_us = 0;
  }
  for (std::size_t i = 0; i < pwm_outputs_.size(); ++i) {
    auto& out = s.pwm_outputs[i];
    out.output = pwm_outputs_[i]->GetSpeed();
    out.valid = true;
  }
  return healthy;
}
bool PhoenixBackend::Apply(std::span<const hw::MotorRequest> commands) {
  if (commands.size() != motors_.size()) return false;
  if (is_simulation_) {
    for (std::size_t i = 0; i < commands.size() && i < last_commands_.size();
         ++i) {
      last_commands_[i] = commands[i];
    }
  }
  for (std::size_t i = 0; i < commands.size(); ++i) {
    const auto& r = commands[i];
    auto& m = *motors_[i];
    ctre::phoenix::StatusCode result;
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
    if (!result.IsOK()) {
      Neutral();
      return false;
    }
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
  for (auto& motor : motors_)
    motor->SetControl(ph::controls::NeutralOut{}.WithUpdateFreqHz(0_Hz));
  for (auto& pwm : pwm_outputs_) pwm->SetSpeed(0.0);
  if (is_simulation_) {
    for (auto& cmd : last_commands_) cmd = hw::MotorRequest{};
    for (std::size_t i = 0; i < motors_.size(); ++i) {
      sim_rotor_velocity_[i] = 0.0;
      motors_[i]->GetSimState().SetRotorVelocity(0_tps);
    }
  }
}

void PhoenixBackend::UpdateSimulation(double dt) {
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    const auto& m_cfg = config_.motors[i];
    auto& sim = motors_[i]->GetSimState();
    double gear_ratio =
        m_cfg.rotor_to_sensor_ratio * m_cfg.sensor_to_mechanism_ratio;
    if (gear_ratio <= 0.0) gear_ratio = 1.0;

    double volts = sim.GetMotorVoltage().value();
    if (std::abs(volts) > 1e-3) {
      // Kraken X60 free speed: 100 rps at 12V
      double target_rotor_rps = (volts / 12.0) * 100.0;
      double alpha = std::min(1.0, dt / 0.01);
      sim_rotor_velocity_[i] +=
          (target_rotor_rps - sim_rotor_velocity_[i]) * alpha;
    } else if (last_commands_[i].mode == hw::Mode::kVelocity &&
               std::abs(last_commands_[i].demand) > 1e-6) {
      // Fallback if closed-loop gains are zero in slot0
      double target_rotor_rps = last_commands_[i].demand * gear_ratio;
      sim_rotor_velocity_[i] = target_rotor_rps;
    } else if ((last_commands_[i].mode == hw::Mode::kPosition ||
                last_commands_[i].mode == hw::Mode::kMotionMagic) &&
               std::abs(last_commands_[i].demand) > 1e-6) {
      sim_rotor_position_[i] = last_commands_[i].demand * gear_ratio;
      sim_rotor_velocity_[i] = 0.0;
    } else {
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

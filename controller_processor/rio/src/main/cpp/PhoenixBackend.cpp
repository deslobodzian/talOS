#include "PhoenixBackend.h"

#include <frc/RobotController.h>
#include <hal/CAN.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/TorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <exception>
#include <limits>
#include <string>
#include <string_view>

#include "hardware/feedback.h"
#include "hardware/gate.h"

#ifndef __FRC_ROBORIO__
#include <frc/simulation/RoboRioSim.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/moment_of_inertia.h>
#include <units/voltage.h>

#include "hardware/sim_backend.h"
#endif

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
bool Fresh(const ph::BaseStatusSignal& signal, units::second_t max_age) {
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
  ~PhaseTimer() {
    const uint64_t end_ns = NowNs();
    log_.AddPhase(phase_, end_ns > start_ns_ ? end_ns - start_ns_ : 0);
  }
  PhaseTimer(const PhaseTimer&) = delete;
  PhaseTimer& operator=(const PhaseTimer&) = delete;

 private:
  talos::rio::RioLog& log_;
  talos::rio::TickPhase phase_;
  uint64_t start_ns_;
};

// Defense-in-depth at the backend boundary: a driving motor request must carry
// finite demand/feedforward, a valid closed-loop gain slot (0..2), and satisfy
// TorqueCurrentAllowed for kTorqueCurrent. Any disallowed request neutralizes
// that motor (and its followers) instead of reaching Phoenix SetControl.
bool MotorRequestAllowed(const hw::MotorConfig& config,
                         const hw::MotorRequest& r) {
  if (r.mode == hw::Mode::kNeutral) return true;
  if (!std::isfinite(r.demand) || !std::isfinite(r.feedforward_v)) {
    return false;
  }
  switch (r.mode) {
    case hw::Mode::kDutyCycle:
    case hw::Mode::kVoltage:
      return true;
    case hw::Mode::kVelocity:
    case hw::Mode::kPosition:
    case hw::Mode::kMotionMagic:
      return r.slot <= 2;
    case hw::Mode::kTorqueCurrent:
      return hw::TorqueCurrentAllowed(config, r);
    default:
      return false;
  }
}
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

// Maximum absolute duty cycle corresponding to max_voltage at battery_v:
// applied to DutyCycleOut (which ignores PeakForwardVoltage/PeakReverseVoltage)
// and TorqueCurrentFOC's MaxAbsDutyCycle, so neither mode applies more than
// max_voltage. 0 (no output) while no valid battery reading has been taken.
double VoltageDutyCap(double max_voltage, double battery_v) {
  if (!(battery_v > 0)) return 0.0;
  return std::clamp(max_voltage / battery_v, 0.0, 1.0);
}

// Posts a feedback validity change: the loop thread only copies it.
void LogFeedbackChange(talos::rio::RioLog& log, talos::rio::LogKind kind,
                       std::size_t index, std::string_view name, uint16_t id,
                       std::string_view bus, int can_id, bool valid,
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
                  std::size_t index, std::string_view name, uint16_t id,
                  std::string_view bus, int can_id, uint32_t late_us,
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

#ifndef __FRC_ROBORIO__
constexpr double kPi = 3.14159265358979323846;
constexpr double kTau = 2.0 * kPi;
// Rotor-side inertia reflected through each motor's gear ratio (J_mech =
// J_rotor * gear_ratio^2), giving a ~44.5 ms electrical time constant for a
// Kraken X60 across gearing ratios.
constexpr double kSimRotorInertiaKgM2 = 0.0005;
// Open-circuit viscous coast time constant when neutral with brake = false.
// Brake neutral instead steps WPILib's DCMotorSystem with u = {0 V} (shorted
// windings back-EMF braking).
constexpr double kSimNeutralCoastTauS = 1.0;
// Friction holds a nearly-stopped unpowered rotor instead of creeping forever
// (same standstill snap as sim_physics.h StepAxis).
constexpr double kSimNeutralStandstillRps = 0.01;
// Minimum supply voltage Phoenix 6 simulation accepts without clamping.
constexpr double kSimMinSupplyV = 4.0;
// Default wheel radius (4 in wheel) and half-span (0.60 m trackwidth / ±0.30 m
// swerve corners), matching talOS/hardware/sim_physics.h.
constexpr double kSimWheelRadiusM = 0.0508;
constexpr double kSimChassisHalfSpanM = 0.30;

frc::DCMotor MakeSimDcMotor(const hw::MotorConfig& m) {
  switch (m.motor_type.value) {
    case hw::MotorType::kKrakenX44:
      return m.foc ? frc::DCMotor::KrakenX44FOC(1) : frc::DCMotor::KrakenX44(1);
    case hw::MotorType::kFalcon500:
      return m.foc ? frc::DCMotor::Falcon500FOC(1) : frc::DCMotor::Falcon500(1);
    case hw::MotorType::kKrakenX60:
    default:
      return m.foc ? frc::DCMotor::KrakenX60FOC(1) : frc::DCMotor::KrakenX60(1);
  }
}
#endif
// The slowest a master's output status signal may run under a Phoenix
// Follower ("no slower than 20 Hz", controls/Follower.hpp).
constexpr double kFollowerSignalMinHz = 20.0;
// RioLog kMessage indices (rate-limit keys) of this file's lines; Robot.cpp
// uses 1 to 6, LoopConsole 9 to 12. A licence refusal's line is keyed by its
// motor (16 to 31),
// so the logger coalesces no motor's line into another's.
constexpr uint16_t kConfigureFailedMessage = 0;
constexpr uint16_t kCanBusReaderMessage = 8;
constexpr uint16_t kRestoreFailedMessage = 13;
constexpr uint16_t kProLicenceRefusedMessage = 16;
static_assert(kProLicenceRefusedMessage + hw::kMaxMotors <= 32);

// One CAN bus's status, utilization from 0 to 1 (the unit of State's
// CanBusSample::utilization_frac) and its controller's counters: the
// RoboRIO's own bus through HAL_CAN_GetCANStatus (a NetComm library call;
// frc::RobotController::GetCANStatus would throw on a failed one), a
// CANivore through Phoenix 6, which blocks up to 1 ms (CANBus.hpp; about
// 1 ms on the bench). Only CanBusSampler's thread calls it.
talos::rio::CanBusHealth::Sample ReadCanBus(std::string_view bus) {
  if (bus == "rio") {
    float utilization = 0;
    uint32_t bus_off = 0;
    uint32_t tx_full = 0;
    uint32_t receive_errors = 0;
    uint32_t transmit_errors = 0;
    int32_t status = 0;
    HAL_CAN_GetCANStatus(&utilization, &bus_off, &tx_full, &receive_errors,
                         &transmit_errors, &status);
    return talos::rio::CanBusHealth::FromHal(
        utilization, bus_off, tx_full, receive_errors, transmit_errors, status);
  }
  const auto status = ph::CANBus{bus}.GetStatus();
  return {status.Status.IsOK(),
          status.BusUtilization,
          status.BusOffCount,
          status.TxFullCount,
          status.REC,
          status.TEC,
          status.Status.GetName()};
}

uint64_t SteadyUs() { return NowNs() / 1000; }

// Records an exception from Configure() or its future as a failed configure
// without letting a secondary exception escape onto the SCHED_FIFO loop thread.
void RecordConfigureException(std::optional<hw::ConfigError>& last_error,
                              talos::rio::RioLog& log,
                              const char* detail) noexcept {
  try {
    hw::ConfigError err;
    err.device_name = "phoenix";
    err.operation = "configure";
    err.message = (detail != nullptr && detail[0] != '\0')
                      ? detail
                      : "unexpected exception";
    last_error = err;
    talos::rio::LogEvent event;
    event.index = kConfigureFailedMessage;
    event.SetText("[phoenix] " + hw::FormatConfigError(err));
    log.Post(event);
  } catch (...) {  // NOLINT(bugprone-empty-catch)
    // Never let a secondary exception (e.g. bad_alloc) escape.
  }
}

// Logs an exception from BeginRestoreAfterReset() or PollRestore() without
// heap allocation on the SCHED_FIFO loop thread or letting any exception
// escape.
void LogRestoreException(talos::rio::RioLog& log, const char* detail) noexcept {
  talos::rio::LogEvent event;
  event.index = kRestoreFailedMessage;
  const char* message = (detail != nullptr && detail[0] != '\0')
                            ? detail
                            : "unexpected exception";
  std::snprintf(event.text, sizeof event.text, "[phoenix] restore failed: %s",
                message);
  log.Post(event);
}
}  // namespace

PhoenixBackend::PhoenixBackend(bool is_simulation, talos::rio::RioLog* log)
    : owned_log_{log ? nullptr : std::make_unique<talos::rio::RioLog>()},
      log_{log ? log : owned_log_.get()},
      // The sampler reports an abandoned reader from the configure worker or
      // this backend's destructor; log_ outlives it.
      can_sampler_{can_health_, ReadCanBus, SteadyUs,
                   [this](std::string_view line) {
                     talos::rio::LogEvent event;
                     event.index = kCanBusReaderMessage;
                     event.SetText(line);
                     log_->Post(event);
                   }},
      is_simulation_{is_simulation} {
  unavailable_.reserve(hw::kMaxMotors + hw::kMaxSensors);
  restoring_ids_.reserve(hw::kMaxMotors + hw::kMaxSensors);
  if (owned_log_) owned_log_->Start();
#ifndef __FRC_ROBORIO__
  // WPILib's simulated battery defaults to 12 V, below a pre-match battery
  // check; rest at the charged supply sim_gateway reports instead.
  if (is_simulation_) {
    frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{hw::kSimRestingSupplyV});
  }
#endif
}

std::optional<hw::ConfigError> PhoenixBackend::last_config_error() const {
  return last_error_;
}

void PhoenixBackend::BeginConfigure(const hw::Config& config) {
  try {
    configure_ = std::async(std::launch::async, [this, config] {
      try {
        return Configure(config);
      } catch (const std::exception& e) {
        RecordConfigureException(last_error_, *log_, e.what());
        return false;
      } catch (...) {
        RecordConfigureException(last_error_, *log_, "unknown exception");
        return false;
      }
    });
  } catch (const std::exception& e) {
    configure_ = {};
    RecordConfigureException(last_error_, *log_, e.what());
  } catch (...) {
    configure_ = {};
    RecordConfigureException(last_error_, *log_, "unknown exception");
  }
}

PhoenixBackend::ConfigureStatus PhoenixBackend::PollConfigure() {
  if (!configure_.valid()) return ConfigureStatus::kFailed;
  if (configure_.wait_for(std::chrono::seconds{0}) != std::future_status::ready)
    return ConfigureStatus::kPending;
  try {
    return configure_.get() ? ConfigureStatus::kDone : ConfigureStatus::kFailed;
  } catch (const std::exception& e) {
    RecordConfigureException(last_error_, *log_, e.what());
    return ConfigureStatus::kFailed;
  } catch (...) {
    RecordConfigureException(last_error_, *log_, "unknown exception");
    return ConfigureStatus::kFailed;
  }
}

bool PhoenixBackend::Configure(const hw::Config& c) {
  hw::Validate(c);
  // Before the device lock, so WatchdogNeutral never waits on it: stops the
  // CAN bus reader (at most CanBusSampler::kStopBound, abandoning one stuck
  // in a vendor call) and starts one for a configuration with a device, none
  // under simulation.
  can_sampler_.Reconfigure(c, is_simulation_);
  const std::lock_guard devices_lock{devices_mutex_};
  restore_operations_.clear();
  unavailable_.clear();
  unavailable_.reserve(hw::kMaxMotors + hw::kMaxSensors);
  restoring_ids_.clear();
  restoring_ids_.reserve(hw::kMaxMotors + hw::kMaxSensors);
  restore_launch_failed_ = false;
  foc_unlicensed_.clear();
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
  const auto note_failure = [this](std::string_view name, std::string_view bus,
                                   int can_id, const std::string& operation,
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
  refresh_signals_.clear();
  refresh_group_count_ = 0;
  signals_.clear();
  motors_.clear();
  sensors_.clear();
  digital_inputs_.clear();
  digital_outputs_.clear();
  analog_inputs_.clear();
  rio_encoders_.clear();
  pwm_outputs_.clear();

  sim_neutral_.fill(true);

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
    // TorqueCurrentFOC's peak, both ways: max_torque_current_a, 0 A on a
    // motor without the mode, so a torque request produces nothing there.
    // Validate refuses the key on a follower and on a motor with one, so no
    // Follower copies a torque output. The stator limit above applies in
    // addition, the supply limit in every mode; TorqueNeutralDeadband keeps
    // Phoenix's 0 A.
    cfg.TorqueCurrent.PeakForwardTorqueCurrent =
        units::ampere_t{m.max_torque_current_a};
    cfg.TorqueCurrent.PeakReverseTorqueCurrent =
        units::ampere_t{-m.max_torque_current_a};
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
    // A follower copies its master's output signal (MotorVoltage above for
    // voltage-based requests, DutyCycle for duty-cycle ones), which the rate
    // loop below keeps at kFollowerSignalMinHz or more for a master.
    if (std::any_of(c.motors.begin(), c.motors.end(), [&m](const auto& other) {
          return other.master_id == m.id;
        }))
      signals_.push_back(&motor->GetDutyCycle(false));
    signal_ids_.resize(signals_.size(), m.id);
    motors_.push_back(std::move(motor));
  }
  // FOC needs a Phoenix Pro licence, and so does a follower of a foc master,
  // which copies its master's output. Read after each device's
  // configuration. A motor read unlicensed is a failed device until the next
  // Configure, with its followers: never sent a request, never reported
  // valid, and not retried, since a licence does not arrive by retrying. A
  // read that got no answer (the device absent, or its frame late) is an
  // unavailable device, and its restore reads the licence again.
  licence_refused_.fill(false);
  licence_read_.fill(false);
  foc_unlicensed_.reserve(hw::kMaxMotors);
  bool first_error_posted = false;
  const auto check_licence = [&](std::size_t i) {
    const auto& m = c.motors[i];
    const bool master_foc =
        m.master_id != 0 &&
        std::any_of(c.motors.begin(), c.motors.end(), [&](const auto& other) {
          return other.id == m.master_id && other.foc;
        });
    if (!m.foc && !master_foc) return;
    restore_operations_.push_back(
        {m.id, [this, i] { return RestoreProLicence(i); }});
    if (licence_refused_[i]) return;  // Its master's refusal took it.
    ctre::phoenix::StatusCode status;
    const auto licensed = ReadProLicence(i, status);
    if (!licensed) {
      ok &= note_failure(m.name, m.bus, m.can_id, "check_pro_license", status);
    } else if (!*licensed) {
      ok = false;
      const bool had_error = last_error_.has_value();
      RefuseProLicence(i, true);
      first_error_posted |= !had_error && last_error_.has_value();
    }
  };
  // Pass 1: check masters before followers so a follower preceding its
  // unlicensed foc master in config.motors is never read itself
  // (REQ-CONTRACT-39) and its restore op runs after its master's.
  for (std::size_t i = 0; i < c.motors.size(); ++i) {
    if (c.motors[i].master_id == 0) check_licence(i);
  }
  // Pass 2: check followers of foc masters not already refused with their
  // master.
  for (std::size_t i = 0; i < c.motors.size(); ++i) {
    if (c.motors[i].master_id != 0) check_licence(i);
  }
  GroupSignalsByNetwork(c);

#ifndef __FRC_ROBORIO__
  if (is_simulation_) {
    ConfigureSimulation(c);
  }
#endif

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
      // StatorCurrent, SupplyCurrent and DeviceTemp are SupplyAndTemp SPNs,
      // and a master's DutyCycle shares the MotorOutput frame with
      // MotorVoltage (PRO_MotorOutput_*, 100 Hz by default on CAN 2.0 where
      // position and velocity are 50 Hz), so it takes MotorVoltage's cap.
      auto& motor = *motors_[&m - c.motors.data()];
      if (signal == &motor.GetMotorVoltage(false) ||
          signal == &motor.GetDutyCycle(false) ||
          signal == &motor.GetStatorCurrent(false) ||
          signal == &motor.GetSupplyCurrent(false) ||
          signal == &motor.GetFaultField(false))
        hz = std::min(hz, 50.0);
      if (signal == &motor.GetDeviceTemp(false)) hz = std::min(hz, 4.0);
      // Phoenix's Follower (controls/Follower.hpp) updates the follower's
      // output at the rate of its master's output signal, which must be no
      // slower than 20 Hz; a status_hz as slow as config load allows (8 Hz
      // at the 500 ms timeout) or OptimizeBusUtilization's 4 Hz would not
      // be. TorqueCurrent is not needed: no motor with a follower runs
      // torque-current requests (config load).
      const bool master = std::any_of(
          c.motors.begin(), c.motors.end(),
          [&m](const auto& other) { return other.master_id == m.id; });
      if (master && (signal == &motor.GetMotorVoltage(false) ||
                     signal == &motor.GetDutyCycle(false)))
        hz = std::max(hz, kFollowerSignalMinHz);
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
  // A licence refusal is permanent until the next Configure: populate
  // foc_unlicensed_ and erase every refused motor (and follower) from
  // unavailable_ so no failed config or signal-rate apply leaves it churning
  // the restore loop.
  SyncUnlicensedAndPruneUnavailable();
  // A licence refusal posted its own line already.
  if (!ok && last_error_ && !first_error_posted) {
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

void PhoenixBackend::GroupSignalsByNetwork(const hw::Config& c) {
  // Phoenix 6's BaseStatusSignal::RefreshAll (StatusSignal.hpp) "\return
  // InvalidNetwork if signals are on different CAN bus networks", and then
  // refreshes none of them. Each bus a device names is its own network, so
  // the signals are reordered bus by bus (hw::CanBuses order) and Read
  // refreshes each bus's span with one call. Allocates here, never in Read.
  const auto bus_of = [&c](uint16_t id) -> std::string_view {
    for (const auto& m : c.motors)
      if (m.id == id) return m.bus;
    for (const auto& s : c.sensors)
      if (s.id == id) return s.bus;
    return {};
  };
  refresh_signals_.clear();
  refresh_signals_.reserve(signals_.size());
  refresh_group_count_ = 0;
  const hw::CanBusList buses = hw::CanBuses(c);
  for (std::size_t b = 0; b < buses.count; ++b) {
    const std::size_t begin = refresh_signals_.size();
    for (std::size_t i = 0; i < signals_.size(); ++i)
      if (bus_of(signal_ids_[i]) == buses.names[b])
        refresh_signals_.push_back(signals_[i]);
    if (refresh_signals_.size() > begin)
      refresh_groups_[refresh_group_count_++] = {
          begin, refresh_signals_.size() - begin};
  }
}

std::optional<bool> PhoenixBackend::ReadProLicence(
    std::size_t index, ctre::phoenix::StatusCode& status) {
  status = ctre::phoenix::StatusCode::OK;
  std::optional<bool> licensed;
  if (pro_licensed_) {
    licensed = pro_licensed_(index);
    if (!licensed) status = ctre::phoenix::StatusCode::RxTimeout;
  } else if (is_simulation_) {
    licensed = true;
  } else {
    auto& signal = motors_[index]->GetIsProLicensed(false);
    status = signal.WaitForUpdate(500_ms).GetStatus();
    if (status.IsOK()) licensed = signal.GetValue();
  }
  licence_read_[index] = licensed.has_value();
  return licensed;
}

void PhoenixBackend::RefuseProLicence(std::size_t index, bool record_error) {
  const auto& master = config_.motors[index];
  // On a worker thread: the line is formatted here and posted, never written.
  const auto refuse = [&](std::size_t i, std::string message) {
    licence_refused_[i] = true;
    const auto& m = config_.motors[i];
    hw::ConfigError err;
    err.device_name = m.name;
    err.bus = m.bus;
    err.can_id = m.can_id;
    err.operation = "check_pro_license";
    err.message = std::move(message);
    talos::rio::LogEvent event;
    event.index = static_cast<uint16_t>(kProLicenceRefusedMessage + i);
    event.SetText("[phoenix] " + hw::FormatConfigError(err));
    log_->Post(event);
    if (record_error && !last_error_) last_error_ = std::move(err);
  };
  // LogEvent::text keeps 159 bytes. With 15-byte motor names, a 16-byte bus
  // and a two-digit CAN id, the line up to the message takes 123 bytes
  // ("[phoenix] config failed for '<name>' on bus '<bus>' CAN <id> during
  // check_pro_license (vendor status 0): "), so each message stays within
  // 36: the follower's 33, the motor's own 31.
  refuse(index, "foc needs a Phoenix Pro licence");
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    if (config_.motors[i].master_id == master.id && !licence_refused_[i])
      refuse(i, "master " + master.name + " unlicensed");
  }
}

bool PhoenixBackend::RestoreProLicence(std::size_t index) {
  // Refused: failed until the next Configure. Read licensed: a reset does
  // not take the licence, so the restore reads only an unanswered one.
  if (licence_refused_[index] || licence_read_[index]) return true;
  ctre::phoenix::StatusCode status;
  const auto licensed = ReadProLicence(index, status);
  if (!licensed) return false;  // Still unread: unavailable, retried.
  if (!*licensed) RefuseProLicence(index, false);
  return true;
}

void PhoenixBackend::SyncUnlicensedAndPruneUnavailable() {
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    const uint16_t id = config_.motors[i].id;
    if (licence_refused_[i] &&
        std::find(foc_unlicensed_.begin(), foc_unlicensed_.end(), id) ==
            foc_unlicensed_.end()) {
      foc_unlicensed_.push_back(id);
    }
  }
  std::erase_if(unavailable_, [this](uint16_t id) {
    return std::find(foc_unlicensed_.begin(), foc_unlicensed_.end(), id) !=
           foc_unlicensed_.end();
  });
}

bool PhoenixBackend::PollDeviceResets(std::vector<uint16_t>& reset_ids) {
  const PhaseTimer timer{*log_, talos::rio::TickPhase::kDevices};
  // HasResetOccurred reports (and clears) one device's reset latch.
  const auto has_reset = [this](ph::hardware::ParentDevice& device) {
    return poll_device_reset_ ? poll_device_reset_(device)
                              : device.HasResetOccurred();
  };
  reset_ids.clear();
  for (std::size_t i = 0; i < motors_.size() && i < config_.motors.size();
       ++i) {
    if (has_reset(*motors_[i])) {
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
    const bool reset = sensors_[i].encoder ? has_reset(*sensors_[i].encoder)
                                           : has_reset(*sensors_[i].imu);
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
  restore_launch_failed_ = false;
  restoring_ids_.clear();
  const auto restoring_contains = [&](uint16_t id) {
    return std::find(restoring_ids_.begin(), restoring_ids_.end(), id) !=
           restoring_ids_.end();
  };
  for (const uint16_t id : ids) {
    if (!restoring_contains(id) &&
        restoring_ids_.size() < restoring_ids_.capacity()) {
      restoring_ids_.push_back(id);
    }
  }
  // Re-apply the affected control chain, never unrelated device settings.
  for (std::size_t pass = 0; pass < config_.motors.size(); ++pass) {
    for (const auto& motor : config_.motors) {
      if (!restoring_contains(motor.id) &&
          ((motor.master_id != 0 && restoring_contains(motor.master_id)) ||
           (motor.feedback == hw::Feedback::kRemoteCANcoder &&
            restoring_contains(motor.feedback_sensor_id))) &&
          restoring_ids_.size() < restoring_ids_.capacity()) {
        restoring_ids_.push_back(motor.id);
      }
    }
  }
  for (const uint16_t id : restoring_ids_) {
    if (std::find(unavailable_.begin(), unavailable_.end(), id) ==
            unavailable_.end() &&
        unavailable_.size() < unavailable_.capacity()) {
      unavailable_.push_back(id);
    }
  }
  hw::FixedVector<uint16_t, hw::kMaxMotors + hw::kMaxSensors> worker_ids;
  for (const uint16_t id : restoring_ids_) {
    if (worker_ids.size() < worker_ids.capacity()) worker_ids.push_back(id);
  }
  auto work = [this,
               ids = worker_ids]() -> std::optional<std::vector<uint16_t>> {
    try {
      std::vector<uint16_t> failures;
      for (const auto& operation : restore_operations_) {
        if (std::find(ids.begin(), ids.end(), operation.id) == ids.end())
          continue;
        if (!operation.apply() && std::find(failures.begin(), failures.end(),
                                            operation.id) == failures.end()) {
          failures.push_back(operation.id);
        }
      }
      return failures;
    } catch (const std::exception& e) {
      LogRestoreException(*log_, e.what());
      return std::nullopt;
    } catch (...) {
      LogRestoreException(*log_, "unknown exception");
      return std::nullopt;
    }
  };
  try {
    restore_ = launch_restore_
                   ? launch_restore_(std::move(work))
                   : std::async(std::launch::async, std::move(work));
  } catch (const std::exception& e) {
    restore_ = {};
    restore_launch_failed_ = true;
    LogRestoreException(*log_, e.what());
  } catch (...) {
    restore_ = {};
    restore_launch_failed_ = true;
    LogRestoreException(*log_, "unknown exception");
  }
}

PhoenixBackend::RestoreStatus PhoenixBackend::PollRestore() {
  if (restore_launch_failed_) {
    restore_launch_failed_ = false;
    return RestoreStatus::kFailed;
  }
  if (!restore_.valid()) return RestoreStatus::kDone;
  if (restore_.wait_for(std::chrono::seconds{0}) != std::future_status::ready)
    return RestoreStatus::kPending;
  std::optional<std::vector<uint16_t>> failures;
  try {
    failures = restore_.get();
  } catch (const std::exception& e) {
    restore_ = {};
    LogRestoreException(*log_, e.what());
    SyncUnlicensedAndPruneUnavailable();
    return RestoreStatus::kFailed;
  } catch (...) {
    restore_ = {};
    LogRestoreException(*log_, "unknown exception");
    SyncUnlicensedAndPruneUnavailable();
    return RestoreStatus::kFailed;
  }
  // A licence the restore read unlicensed fails its motor before the motor
  // leaves unavailable_, so it is never reported valid in between. Reserved
  // at Configure: no allocation on the loop thread.
  SyncUnlicensedAndPruneUnavailable();
  if (!failures.has_value()) {
    return RestoreStatus::kFailed;
  }
  std::erase_if(unavailable_, [this, &failures](uint16_t id) {
    return std::find(restoring_ids_.begin(), restoring_ids_.end(), id) !=
               restoring_ids_.end() &&
           std::find(failures->begin(), failures->end(), id) == failures->end();
  });
  return RestoreStatus::kDone;
}

bool PhoenixBackend::Read(hw::State& s) {
  using talos::rio::TickPhase;
  const uint64_t read_start_ns = NowNs();
  // Every CAN bus as the sampler thread last published it (no vendor call
  // here); State carries the slot the gateway asked for.
  can_health_.Poll(read_start_ns / 1000,
                   [this](const talos::rio::CanBusHealth::Change& change) {
                     talos::rio::LogCanBusStatus(*log_,
                                                 can_health_.bus(change.bus),
                                                 can_health_.StaleUs(), change);
                   });
  s.health.valid = true;
  s.health.battery_v = frc::RobotController::GetBatteryVoltage().value();
  // The duty-cycle and torque-current duty cap's divisor (VoltageDutyCap): a
  // lost reading keeps the last real one.
  if (std::isfinite(s.health.battery_v) && s.health.battery_v > 1.0)
    last_battery_v_ = s.health.battery_v;
  s.health.brownout = frc::RobotController::IsBrownedOut();
  s.health.can_bus = can_health_.Slot(s.health.can_bus_index);
  uint64_t refresh_start_ns = NowNs();
  log_->AddPhase(TickPhase::kHealth, refresh_start_ns - read_start_ns);
#ifndef __FRC_ROBORIO__
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
#endif
  // One RefreshAll per CAN network: Phoenix refreshes nothing for a call
  // whose signals span two (GroupSignalsByNetwork).
  for (std::size_t g = 0; g < refresh_group_count_; ++g) {
    const RefreshGroup& group = refresh_groups_[g];
    ph::BaseStatusSignal::RefreshAll(
        false, std::span<ph::BaseStatusSignal* const>{
                   refresh_signals_.data() + group.begin, group.count});
  }
  const uint64_t devices_start_ns = NowNs();
  log_->AddPhase(TickPhase::kRefresh, devices_start_ns - refresh_start_ns);
  bool healthy = motors_.empty() && sensors_.empty();
  // Four status periods: the LATE line's threshold (NoteLateness).
  const auto late_us =
      static_cast<uint32_t>(std::ceil(4'000'000.0 / config_.status_hz));
  const units::second_t diagnostic_max_age = units::microsecond_t{
      static_cast<double>(hw::DiagnosticMaxAgeUs(config_.status_hz))};
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
    // FaultField is its own signal (requested at min(status_hz, 50) Hz),
    // separate from the motion feedback that sets `valid`. A stale FaultField
    // reports no asserted limit and no active current limiter
    // (kMotorNotFaultFlags), so neither a limit getter nor a display shows an
    // old state, but keeps its last fault bits: losing the signal must not
    // un-fault a faulted motor.
    auto& faults = motor.GetFaultField(false);
    out.fault_flags = faults.GetValue();
    if (!Fresh(faults, diagnostic_max_age))
      out.fault_flags &= ~hw::kMotorNotFaultFlags;
    // Stale current/voltage is telemetry-only degradation. The
    // effort fields zero but the sample stays valid and the backend stays
    // healthy; lost motion feedback isolates only the affected motor.
    const bool effort_ok = Fresh(volts, diagnostic_max_age) &&
                           Fresh(amps, diagnostic_max_age) &&
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
    out.supply_current_a = Fresh(supply, diagnostic_max_age) &&
                                   std::isfinite(supply.GetValue().value())
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
            unavailable_.end() &&
        std::find(foc_unlicensed_.begin(), foc_unlicensed_.end(),
                  motor_config.id) == foc_unlicensed_.end();
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
    // An invalid sample carries zero in every measured field
    // (talOS/hardware/README.md, fault scope).
    if (!out.valid) {
      out.position_rot = 0;
      out.velocity_rps = 0;
      out.voltage = 0;
      out.stator_current_a = 0;
      out.supply_current_a = 0;
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
      if (connected && std::isfinite(pos)) {
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
  const auto device_blocked = [&](uint16_t id) {
    return std::find(unavailable_.begin(), unavailable_.end(), id) !=
               unavailable_.end() ||
           std::find(foc_unlicensed_.begin(), foc_unlicensed_.end(), id) !=
               foc_unlicensed_.end() ||
           std::find(apply_failed_.begin(), apply_failed_.end(), id) !=
               apply_failed_.end();
  };
  const auto remote_sensor_valid = [&](const hw::MotorConfig& cfg,
                                       uint64_t bound) {
    if (cfg.feedback != hw::Feedback::kRemoteCANcoder) return true;
    for (std::size_t j = 0; j < config_.sensors.size(); ++j) {
      if (config_.sensors[j].id == cfg.feedback_sensor_id &&
          sensors_[j].encoder) {
        return ValidMotionFeedback(
            sensors_[j].encoder->GetAbsolutePosition(false),
            sensors_[j].encoder->GetVelocity(false), bound);
      }
    }
    return false;
  };
  for (std::size_t i = 0; i < commands.size(); ++i) {
    const auto& r = commands[i];
    auto& m = *motors_[i];
    ctre::phoenix::StatusCode result;
    const auto& config = config_.motors[i];
    if (std::find(unavailable_.begin(), unavailable_.end(), config.id) !=
            unavailable_.end() ||
        std::find(foc_unlicensed_.begin(), foc_unlicensed_.end(), config.id) !=
            foc_unlicensed_.end()) {
      // A missing device cannot acknowledge NeutralOut. Keep trying neutral,
      // but do not turn its transport failure into a whole-robot Apply failure.
      // A foc motor not read licensed (or its follower) is held the same way.
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
      const bool feedback_valid =
          ValidMotionFeedback(m.GetPosition(false), m.GetVelocity(false),
                              bound) &&
          remote_sensor_valid(config, bound) &&
          remote_sensor_valid(*master, bound);
      const bool master_allowed =
          !device_blocked(master->id) &&
          commands[master_index].mode != hw::Mode::kNeutral &&
          MotorRequestAllowed(*master, commands[master_index]);
      const bool neutral = r.mode == hw::Mode::kNeutral || !master_allowed ||
                           !ValidMotionFeedback(position, velocity, bound) ||
                           !feedback_valid;
      sim_neutral_[i] = neutral;
      if (neutral) {
        result =
            m.SetControl(ph::controls::NeutralOut{}.WithUpdateFreqHz(0_Hz));
      } else {
        result = m.SetControl(
            ph::controls::Follower{master->can_id, config.oppose_master}
                .WithUpdateFreqHz(0_Hz));
      }
      note_send(i, result);
      continue;
    }
    if (r.mode != hw::Mode::kNeutral && r.mode <= hw::kLastMode &&
        !MotorRequestAllowed(config, r)) {
      sim_neutral_[i] = true;
      result = m.SetControl(ph::controls::NeutralOut{}.WithUpdateFreqHz(0_Hz));
      note_send(i, result);
      continue;
    }
    sim_neutral_[i] = r.mode == hw::Mode::kNeutral;
    switch (r.mode) {
      case hw::Mode::kNeutral:
        result =
            m.SetControl(ph::controls::NeutralOut{}.WithUpdateFreqHz(0_Hz));
        break;
      case hw::Mode::kDutyCycle: {
        // PeakForwardVoltage/PeakReverseVoltage apply only to Phoenix's
        // voltage-based controls, so clamp DutyCycleOut to +-max_voltage /
        // battery_v (matching TorqueCurrentFOC and PhysicsSimBackend).
        const double duty_cap =
            VoltageDutyCap(config.max_voltage, last_battery_v_);
        const double duty = std::clamp(r.demand, -duty_cap, duty_cap);
        result = m.SetControl(ph::controls::DutyCycleOut{duty}
                                  .WithEnableFOC(config.foc)
                                  .WithUpdateFreqHz(0_Hz));
        break;
      }
      case hw::Mode::kVoltage:
        result = m.SetControl(ph::controls::VoltageOut{units::volt_t{r.demand}}
                                  .WithEnableFOC(config.foc)
                                  .WithUpdateFreqHz(0_Hz));
        break;
      case hw::Mode::kVelocity:
        result = m.SetControl(
            ph::controls::VelocityVoltage{units::turns_per_second_t{r.demand}}
                .WithSlot(r.slot)
                .WithFeedForward(units::volt_t{r.feedforward_v})
                .WithEnableFOC(config.foc)
                .WithUpdateFreqHz(0_Hz));
        break;
      case hw::Mode::kPosition:
        result =
            m.SetControl(ph::controls::PositionVoltage{units::turn_t{r.demand}}
                             .WithSlot(r.slot)
                             .WithFeedForward(units::volt_t{r.feedforward_v})
                             .WithEnableFOC(config.foc)
                             .WithUpdateFreqHz(0_Hz));
        break;
      case hw::Mode::kMotionMagic:
        result = m.SetControl(
            ph::controls::MotionMagicVoltage{units::turn_t{r.demand}}
                .WithSlot(r.slot)
                .WithFeedForward(units::volt_t{r.feedforward_v})
                .WithEnableFOC(config.foc)
                .WithUpdateFreqHz(0_Hz));
        break;
      case hw::Mode::kTorqueCurrent:
        // Stator amperes, within the peak Configure set. The other fields
        // keep Phoenix's defaults: Deadband 0 A, OverrideCoastDurNeutral
        // false, limit switches and soft limits honoured, no timesync. A
        // 0 A demand is sent as TorqueCurrentFOC, not NeutralOut; what the
        // motor does at exactly 0 A is unmeasured (talOS/hardware/README.md,
        // "Supported Hardware Requests"). A request the gateway would have
        // neutralized is this motor's neutral here too.
        if (!hw::TorqueCurrentAllowed(config, r)) {
          sim_neutral_[i] = true;
          result =
              m.SetControl(ph::controls::NeutralOut{}.WithUpdateFreqHz(0_Hz));
          break;
        }
        result = m.SetControl(
            ph::controls::TorqueCurrentFOC{units::ampere_t{r.demand}}
                .WithMaxAbsDutyCycle(
                    VoltageDutyCap(config.max_voltage, last_battery_v_))
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
  if (digital_outputs.size() != digital_outputs_.size() ||
      pwm_outputs.size() != pwm_outputs_.size()) {
    return false;
  }
  for (std::size_t i = 0; i < digital_outputs_.size(); ++i) {
    digital_outputs_[i]->Set(digital_outputs[i].value);
  }
  // Off is the output's neutral, no pulse; SetSpeed(0) would be the 1.5 ms
  // centre pulse, which drives a servo to its centre. The gateway has refused
  // any other mode.
  for (std::size_t i = 0; i < pwm_outputs_.size(); ++i) {
    const auto& request = pwm_outputs[i];
    if (request.mode == hw::PwmMode::kPulse && std::isfinite(request.output) &&
        request.output >= -1.0 && request.output <= 1.0) {
      pwm_outputs_[i]->SetSpeed(request.output);
    } else {
      pwm_outputs_[i]->SetDisabled();
    }
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
                 control_end_ns > control_start_ns
                     ? control_end_ns - control_start_ns
                     : 0);
  for (auto& pwm : pwm_outputs_) pwm->SetDisabled();
  // Support matrix: DIO outputs de-energize on every safe-state transition.
  for (auto& dio : digital_outputs_) dio->Set(false);
  // Sim: neutral keeps rotor momentum; UpdateSimulation decays it per brake.
  if (is_simulation_) sim_neutral_.fill(true);
  const uint64_t other_end_ns = NowNs();
  log_->AddPhase(
      talos::rio::TickPhase::kOther,
      other_end_ns > control_end_ns ? other_end_ns - control_end_ns : 0);
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

#ifndef __FRC_ROBORIO__
void PhoenixBackend::ConfigureSimulation(const hw::Config& c) {
  motor_to_sensor_map_.fill(-1);
  sim_chassis_module_count_ = 0;
  sim_yaw_deg_ = 0.0;
  last_sim_time_ = std::nullopt;

  const double battery_v = frc::RobotController::GetBatteryVoltage().value();
  const double supply_v = (std::isfinite(battery_v) && battery_v > 0.0)
                              ? std::max(battery_v, kSimMinSupplyV)
                              : hw::kSimRestingSupplyV;
  last_sim_supply_v_ = supply_v;

  for (std::size_t i = 0; i < motors_.size() && i < sim_plants_.size(); ++i) {
    const auto& m = c.motors[i];
    double gear_ratio =
        m.feedback == hw::Feedback::kRemoteCANcoder
            ? m.rotor_to_sensor_ratio * m.sensor_to_mechanism_ratio
            : m.sensor_to_mechanism_ratio;
    if (!std::isfinite(gear_ratio) || gear_ratio <= 0.0) gear_ratio = 1.0;
    const units::kilogram_square_meter_t j_mech{kSimRotorInertiaKgM2 *
                                                gear_ratio * gear_ratio};
    sim_plants_[i] = SimMotorPlant{frc::LinearSystemId::DCMotorSystem(
                                       MakeSimDcMotor(m), j_mech, gear_ratio),
                                   frc::Vectord<2>::Zero(), gear_ratio};

    auto& sim = motors_[i]->GetSimState();
    // Phoenix 6 TalonFXSimState::MotorType only defines KrakenX60 and
    // KrakenX44 (no Falcon500); WPILib's DCMotorSystem models Falcon500 via
    // MakeSimDcMotor().
    sim.SetMotorType(m.motor_type.value == hw::MotorType::kKrakenX44
                         ? ph::sim::TalonFXSimState::MotorType::KrakenX44
                         : ph::sim::TalonFXSimState::MotorType::KrakenX60);
    sim.SetSupplyVoltage(units::volt_t{supply_v});
    sim.SetRawRotorPosition(0_tr);
    sim.SetRotorVelocity(0_tps);
    sim.SetRotorAcceleration(units::turns_per_second_squared_t{0.0});

    if (m.feedback == hw::Feedback::kRemoteCANcoder) {
      for (std::size_t s_idx = 0; s_idx < c.sensors.size(); ++s_idx) {
        if (c.sensors[s_idx].id == m.feedback_sensor_id) {
          motor_to_sensor_map_[i] = static_cast<int>(s_idx);
          break;
        }
      }
    }
  }
  for (auto& device : sensors_) {
    if (device.encoder) {
      auto& sim = device.encoder->GetSimState();
      sim.SetSupplyVoltage(units::volt_t{supply_v});
      sim.SetRawPosition(0_tr);
      sim.SetVelocity(0_tps);
    } else if (device.imu) {
      auto& sim = device.imu->GetSimState();
      sim.SetSupplyVoltage(units::volt_t{supply_v});
      sim.SetRawYaw(0_deg);
      sim.SetAngularVelocityZ(units::degrees_per_second_t{0.0});
    }
  }

  // Infer swerve modules from conventional names (<prefix>_drive +
  // <prefix>_steer with front/back/rear and left/right -> ±0.30 m, matching
  // talOS/hardware/sim_physics.h).
  for (std::size_t i = 0;
       i < c.motors.size() &&
       sim_chassis_module_count_ < sim_chassis_modules_.size();
       ++i) {
    const std::string_view name = c.motors[i].name.view();
    if (!name.ends_with("_drive")) continue;
    const std::string_view prefix = name.substr(0, name.size() - 6);
    for (std::size_t j = 0; j < c.motors.size(); ++j) {
      if (j == i) continue;
      const std::string_view other = c.motors[j].name.view();
      if (other.size() != prefix.size() + 6 || !other.starts_with(prefix) ||
          !other.ends_with("_steer")) {
        continue;
      }
      const double x = prefix.find("front") != std::string_view::npos
                           ? kSimChassisHalfSpanM
                       : (prefix.find("back") != std::string_view::npos ||
                          prefix.find("rear") != std::string_view::npos)
                           ? -kSimChassisHalfSpanM
                           : 0.0;
      const double y = prefix.find("left") != std::string_view::npos
                           ? kSimChassisHalfSpanM
                       : prefix.find("right") != std::string_view::npos
                           ? -kSimChassisHalfSpanM
                           : 0.0;
      if (x != 0.0 || y != 0.0) {
        sim_chassis_modules_[sim_chassis_module_count_++] =
            SimChassisModule{i, static_cast<int>(j), x, y};
        break;
      }
    }
  }
  // Fall back to differential left/right drive pairs (trackwidth 0.60 m) when
  // no swerve pairs were inferred.
  if (sim_chassis_module_count_ == 0) {
    std::array<std::size_t, hw::kMaxMotors> left_drives{};
    std::array<std::size_t, hw::kMaxMotors> right_drives{};
    std::size_t left_count = 0;
    std::size_t right_count = 0;
    for (std::size_t i = 0; i < c.motors.size(); ++i) {
      if (c.motors[i].master_id != 0) continue;
      const std::string_view name = c.motors[i].name.view();
      const bool is_drive_name = name.find("drive") != std::string_view::npos ||
                                 name == "left" || name == "right";
      if (!is_drive_name) continue;
      const bool has_left = name.find("left") != std::string_view::npos;
      const bool has_right = name.find("right") != std::string_view::npos;
      if (has_left && !has_right && left_count < left_drives.size()) {
        left_drives[left_count++] = i;
      } else if (has_right && !has_left && right_count < right_drives.size()) {
        right_drives[right_count++] = i;
      }
    }
    if (left_count > 0 && right_count > 0) {
      for (std::size_t k = 0; k < left_count && sim_chassis_module_count_ <
                                                    sim_chassis_modules_.size();
           ++k) {
        sim_chassis_modules_[sim_chassis_module_count_++] =
            SimChassisModule{left_drives[k], -1, 0.0, kSimChassisHalfSpanM};
      }
      for (std::size_t k = 0;
           k < right_count &&
           sim_chassis_module_count_ < sim_chassis_modules_.size();
           ++k) {
        sim_chassis_modules_[sim_chassis_module_count_++] =
            SimChassisModule{right_drives[k], -1, 0.0, -kSimChassisHalfSpanM};
      }
    }
  }
}

void PhoenixBackend::UpdateSimulation(double dt) {
  // Forward the RoboRIO's simulated battery voltage to Phoenix devices whenever
  // it changes. Never call RoboRioSim::SetVInVoltage here so battery injections
  // from tests or the Sim GUI read back untouched.
  const double battery_v = frc::RobotController::GetBatteryVoltage().value();
  if (std::isfinite(battery_v) && battery_v > 0.0) {
    const double supply_v = std::max(battery_v, kSimMinSupplyV);
    if (supply_v != last_sim_supply_v_) {
      last_sim_supply_v_ = supply_v;
      const units::volt_t supply{supply_v};
      for (auto& motor : motors_) {
        motor->GetSimState().SetSupplyVoltage(supply);
      }
      for (auto& device : sensors_) {
        if (device.encoder) {
          device.encoder->GetSimState().SetSupplyVoltage(supply);
        } else if (device.imu) {
          device.imu->GetSimState().SetSupplyVoltage(supply);
        }
      }
    }
  }

  for (std::size_t i = 0; i < motors_.size() && i < sim_plants_.size(); ++i) {
    const auto& m_cfg = config_.motors[i];
    auto& sim = motors_[i]->GetSimState();
    auto& plant = sim_plants_[i];
    const double prev_vel_radps = plant.state(1);

    // Voltage is the only driver: a closed-loop request with missing gains
    // produces ~0 V, so the plant brakes/coasts instead of jumping to the
    // requested demand. Powered and brake-neutral step WPILib's DCMotorSystem
    // plant (0 V for brake neutral = shorted windings back-EMF braking). Under
    // TorqueCurrentFOC Phoenix reports the volts its current loop applies
    // against this rotor's speed, up to the duty cap; at 0 A it reports 0 V
    // (talOS/hardware/README.md says what is known of 0 A). Coast neutral opens
    // the windings, decaying exponentially with tau = 1.0 s.
    if (!sim_neutral_[i] || m_cfg.brake) {
      const double raw_v =
          sim_neutral_[i] ? 0.0 : sim.GetMotorVoltage().value();
      const double applied_v = std::isfinite(raw_v) ? raw_v : 0.0;
      const frc::Vectord<1> u{applied_v};
      plant.state =
          plant.system.CalculateX(plant.state, u, units::second_t{dt});
    } else {
      const double decay = std::exp(-dt / kSimNeutralCoastTauS);
      plant.state(0) += prev_vel_radps * kSimNeutralCoastTauS * (1.0 - decay);
      plant.state(1) = prev_vel_radps * decay;
    }

    double rotor_vel_rps = plant.state(1) * plant.gear_ratio / kTau;
    if (sim_neutral_[i] && std::abs(rotor_vel_rps) < kSimNeutralStandstillRps) {
      plant.state(1) = 0.0;
      rotor_vel_rps = 0.0;
    }
    const double rotor_pos_rot = plant.state(0) * plant.gear_ratio / kTau;
    const double rotor_accel_rps2 =
        dt > 0.0
            ? ((plant.state(1) - prev_vel_radps) / dt) * plant.gear_ratio / kTau
            : 0.0;

    sim.SetRawRotorPosition(units::turn_t{rotor_pos_rot});
    sim.SetRotorVelocity(units::turns_per_second_t{rotor_vel_rps});
    sim.SetRotorAcceleration(
        units::turns_per_second_squared_t{rotor_accel_rps2});

    // Sync remote CANcoder if mapped. The CANcoder sits on the sensor shaft
    // (rotor / rotor_to_sensor_ratio = mechanism * sensor_to_mechanism_ratio),
    // and Phoenix 6's TalonFX then divides the remote CANcoder reading by
    // SensorToMechanismRatio once to report mechanism rotations and rps.
    const int sensor_idx = motor_to_sensor_map_[i];
    if (sensor_idx >= 0 &&
        static_cast<std::size_t>(sensor_idx) < sensors_.size() &&
        sensors_[sensor_idx].encoder) {
      const double sensor_to_mech =
          (std::isfinite(m_cfg.sensor_to_mechanism_ratio) &&
           m_cfg.sensor_to_mechanism_ratio > 0.0)
              ? m_cfg.sensor_to_mechanism_ratio
              : 1.0;
      const double sensor_rot = (plant.state(0) / kTau) * sensor_to_mech;
      const double sensor_rps = (plant.state(1) / kTau) * sensor_to_mech;
      sensors_[sensor_idx].encoder->GetSimState().SetRawPosition(
          units::turn_t{sensor_rot});
      sensors_[sensor_idx].encoder->GetSimState().SetVelocity(
          units::turns_per_second_t{sensor_rps});
    }
  }

  // Only drive Pigeon2 yaw when chassis modules were inferred, so non-chassis
  // unit tests injecting SetRawYaw are not overwritten with 0.
  if (sim_chassis_module_count_ >= 2 && dt > 0.0) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_vx = 0.0;
    double sum_vy = 0.0;
    double moment = 0.0;
    double lever_squared = 0.0;
    for (std::size_t k = 0; k < sim_chassis_module_count_; ++k) {
      const auto& mod = sim_chassis_modules_[k];
      const double drive_sign =
          config_.motors[mod.drive_index].inverted ? -1.0 : 1.0;
      const double speed =
          drive_sign * sim_plants_[mod.drive_index].state(1) * kSimWheelRadiusM;
      double angle = 0.0;
      if (mod.steer_index >= 0) {
        const auto s_idx = static_cast<std::size_t>(mod.steer_index);
        const int enc_idx = motor_to_sensor_map_[s_idx];
        const bool has_remote_encoder =
            enc_idx >= 0 &&
            static_cast<std::size_t>(enc_idx) < config_.sensors.size();
        const double steer_sign =
            has_remote_encoder
                ? (config_.sensors[enc_idx].inverted ? -1.0 : 1.0)
                : (config_.motors[s_idx].inverted ? -1.0 : 1.0);
        angle = steer_sign * sim_plants_[s_idx].state(0);
        if (has_remote_encoder) {
          const double sensor_to_mech =
              (std::isfinite(config_.motors[s_idx].sensor_to_mechanism_ratio) &&
               config_.motors[s_idx].sensor_to_mechanism_ratio > 0.0)
                  ? config_.motors[s_idx].sensor_to_mechanism_ratio
                  : 1.0;
          angle +=
              (config_.sensors[enc_idx].offset_rot / sensor_to_mech) * kTau;
        }
      }
      const double vx = speed * std::cos(angle);
      const double vy = speed * std::sin(angle);
      sum_x += mod.x_m;
      sum_y += mod.y_m;
      sum_vx += vx;
      sum_vy += vy;
      moment += mod.x_m * vy - mod.y_m * vx;
      lever_squared += mod.x_m * mod.x_m + mod.y_m * mod.y_m;
    }
    const double n = static_cast<double>(sim_chassis_module_count_);
    const double centered_lever =
        lever_squared - (sum_x * sum_x + sum_y * sum_y) / n;
    if (centered_lever > 1e-12) {
      const double omega_radps =
          (moment - (sum_x * sum_vy - sum_y * sum_vx) / n) / centered_lever;
      const double omega_degps = omega_radps * (180.0 / kPi);
      sim_yaw_deg_ += omega_degps * dt;
      for (auto& device : sensors_) {
        if (device.imu) {
          auto& imu_sim = device.imu->GetSimState();
          imu_sim.SetRawYaw(units::degree_t{sim_yaw_deg_});
          imu_sim.SetAngularVelocityZ(units::degrees_per_second_t{omega_degps});
        }
      }
    }
  }
}
#endif

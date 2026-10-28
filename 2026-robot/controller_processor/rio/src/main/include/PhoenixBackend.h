#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/PWM.h>

#include "hardware/gateway.h"

class PhoenixBackend final : public talos::hardware::Backend {
 public:
  explicit PhoenixBackend(bool is_simulation = false);

  bool Configure(const talos::hardware::Config&) override;
  bool Read(talos::hardware::State&) override;
  bool Apply(std::span<const talos::hardware::MotorRequest>) override;
  bool ApplyOutputs(std::span<const talos::hardware::DigitalOutputRequest> digital_outputs,
                    std::span<const talos::hardware::PwmRequest> pwm_outputs) override;
  void Neutral() override;
  // Precise failure behind the last false Configure().
  std::optional<talos::hardware::ConfigError> last_config_error() const override;
  // Per-device reset detection (HasResetOccurred) and restore.
  bool PollDeviceResets(std::vector<uint16_t>& reset_ids) override;
  bool RestoreAfterReset() override;

 private:
  void UpdateSimulation(double dt);

  struct Sensor {
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> encoder;
    std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> imu;
  };
  talos::hardware::Config config_;
  std::vector<std::unique_ptr<ctre::phoenix6::hardware::TalonFX>> motors_;
  std::vector<Sensor> sensors_;
  std::vector<ctre::phoenix6::BaseStatusSignal*> signals_;
  std::vector<std::unique_ptr<frc::DigitalInput>> digital_inputs_;
  std::vector<std::unique_ptr<frc::DigitalOutput>> digital_outputs_;
  std::vector<std::unique_ptr<frc::AnalogInput>> analog_inputs_;
  struct RioEncoder {
    talos::hardware::EncoderConfig config{};
    std::unique_ptr<frc::Encoder> quadrature;
    std::unique_ptr<frc::DutyCycleEncoder> duty_cycle;
    // Duty-cycle devices report absolute position only; velocity is the
    // finite difference of offset-corrected position across Reads.
    double last_position_rot{0.0};
    std::optional<std::chrono::steady_clock::time_point> last_time;
  };
  std::vector<RioEncoder> rio_encoders_;
  std::vector<std::unique_ptr<frc::PWM>> pwm_outputs_;

  std::optional<talos::hardware::ConfigError> last_error_;

  bool is_simulation_{false};
  std::optional<std::chrono::steady_clock::time_point> last_sim_time_;
  std::array<double, talos::hardware::kMaxMotors> sim_rotor_velocity_{};
  std::array<double, talos::hardware::kMaxMotors> sim_rotor_position_{};
  std::array<int, talos::hardware::kMaxMotors> motor_to_sensor_map_{};
  std::array<talos::hardware::MotorRequest, talos::hardware::kMaxMotors> last_commands_{};
  double sim_yaw_deg_{0.0};
};

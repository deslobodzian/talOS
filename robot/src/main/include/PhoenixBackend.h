#pragma once

#include <array>
#include <chrono>
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
  std::vector<std::unique_ptr<frc::Encoder>> encoders_;
  std::vector<std::unique_ptr<frc::PWM>> pwm_outputs_;

  bool is_simulation_{false};
  std::optional<std::chrono::steady_clock::time_point> last_sim_time_;
  std::array<double, talos::hardware::kMaxMotors> sim_rotor_velocity_{};
  std::array<double, talos::hardware::kMaxMotors> sim_rotor_position_{};
  std::array<int, talos::hardware::kMaxMotors> motor_to_sensor_map_{};
  std::array<talos::hardware::MotorRequest, talos::hardware::kMaxMotors> last_commands_{};
  double sim_yaw_deg_{0.0};
};

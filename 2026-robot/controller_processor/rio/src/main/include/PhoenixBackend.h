#pragma once

#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/PWM.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "RioLog.h"
#include "hardware/gateway.h"

class PhoenixBackend final : public talos::hardware::Backend {
 public:
  // Reports go to `log`, which must outlive the backend; without one the
  // backend starts its own. Read, Apply and Neutral run on the RoboRIO loop
  // thread, so they only post fixed-size events (RioLog.h), never write.
  explicit PhoenixBackend(bool is_simulation = false,
                          talos::rio::RioLog* log = nullptr);

  bool Configure(const talos::hardware::Config&) override;
  void BeginConfigure(const talos::hardware::Config&) override;
  ConfigureStatus PollConfigure() override;
  std::vector<uint16_t> UnavailableDevices() const override {
    return unavailable_;
  }
  bool Read(talos::hardware::State&) override;
  bool Apply(std::span<const talos::hardware::MotorRequest>) override;
  std::span<const uint16_t> ApplyFailedDevices() const override {
    return apply_failed_;
  }
  bool ApplyOutputs(
      std::span<const talos::hardware::DigitalOutputRequest> digital_outputs,
      std::span<const talos::hardware::PwmRequest> pwm_outputs) override;
  void Neutral() override;
  // Motor NeutralOut sends that returned a non-OK status, cumulative since
  // construction. State has no counter for this without a wire change, so it
  // stays RIO-local (with a transition log line); Gateway retries neutral
  // every tick while no command is active.
  uint64_t neutral_send_failure_count() const {
    return neutral_send_failure_count_;
  }
  // Precise failure behind the last false Configure().
  std::optional<talos::hardware::ConfigError> last_config_error()
      const override;
  // Per-device reset detection (HasResetOccurred) and restore.
  bool PollDeviceResets(std::vector<uint16_t>& reset_ids) override;
  void BeginRestoreAfterReset(std::span<const uint16_t> ids) override;
  RestoreStatus PollRestore() override;

 private:
  friend class PhoenixBackendFollowerFeedback;
  friend class PhoenixBackendRecovery;
  void UpdateSimulation(double dt);

  // Declared first, so destroyed last: the configure worker (configure_) may
  // still post while the backend is destroyed.
  std::unique_ptr<talos::rio::RioLog> owned_log_;
  talos::rio::RioLog* log_;

  struct Sensor {
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> encoder;
    std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> imu;
  };
  talos::hardware::Config config_;
  std::vector<std::unique_ptr<ctre::phoenix6::hardware::TalonFX>> motors_;
  std::vector<Sensor> sensors_;
  std::array<uint32_t, talos::hardware::kMaxMotors> last_fault_flags_{};
  std::array<bool, talos::hardware::kMaxMotors> last_motor_valid_{};
  std::array<bool, talos::hardware::kMaxSensors> last_sensor_valid_{};
  uint32_t last_motor_fault_mask_{0};
  std::array<bool, talos::hardware::kMaxMotors> neutral_send_failing_{};
  uint64_t neutral_send_failure_count_{0};
  std::array<bool, talos::hardware::kMaxMotors> apply_failing_{};
  std::vector<uint16_t> apply_failed_;
  bool last_brownout_{false};
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
  struct RestoreOperation {
    uint16_t id;
    std::function<bool()> apply;
  };
  std::vector<RestoreOperation> restore_operations_;
  std::vector<uint16_t> signal_ids_;
  std::vector<uint16_t> unavailable_;
  std::vector<uint16_t> restoring_ids_;

  bool is_simulation_{false};
  std::optional<std::chrono::steady_clock::time_point> last_sim_time_;
  std::array<double, talos::hardware::kMaxMotors> sim_rotor_velocity_{};
  std::array<double, talos::hardware::kMaxMotors> sim_rotor_position_{};
  std::array<int, talos::hardware::kMaxMotors> motor_to_sensor_map_{};
  // Desktop sim only: true while the motor is commanded neutral (NeutralOut),
  // so UpdateSimulation decays the rotor per brake/coast instead of tracking
  // voltage.
  std::array<bool, talos::hardware::kMaxMotors> sim_neutral_{};
  double sim_yaw_deg_{0.0};
  std::future<bool> configure_;
  std::future<std::vector<uint16_t>> restore_;
};

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
#include <ctre/phoenix6/hardware/ParentDevice.hpp>
#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "CanBusHealth.h"
#include "RioLog.h"
#include "hardware/gateway.h"

#ifndef __FRC_ROBORIO__
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#endif

class PhoenixBackend final : public talos::hardware::Backend {
 public:
  // Reports go to `log`, which must outlive the backend; without one the
  // backend starts its own. Read, Apply and Neutral run on the RoboRIO loop
  // thread, so they only post fixed-size events (RioLog.h), never write.
  // They and PollDeviceResets and ApplyOutputs also time their phases into
  // `log` (RioLog::AddPhase, TickPhase): one steady-clock read per phase
  // boundary, ten per disabled tick on the RoboRIO.
  // Under desktop simulation (`is_simulation`) it sets WPILib's simulated
  // battery to hardware::kSimRestingSupplyV, the charged supply sim_gateway
  // reports, in place of WPILib's 12 V.
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
  // The loop watchdog's thread (LoopWatchdog.h), while the loop thread is
  // stalled: PWM outputs disabled, DIO outputs false, then NeutralOut to every
  // motor. It touches only the device handles, never the loop thread's
  // bookkeeping, and posts nothing. False when a configure holds the device
  // lists or a NeutralOut send failed. A loop preempted inside a device call
  // can hold that device's lock (Phoenix's per-device control mutex, the HAL
  // DIO mutex; neither inherits priority), and then this waits until the
  // loop runs again, and so does every output after it.
  bool WatchdogNeutral();
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
  friend class PhoenixBackendFoc;
  friend class PhoenixBackendFollowerFeedback;
  friend class PhoenixBackendRecovery;
  friend class PhoenixBackendWatchdog;
#ifndef __FRC_ROBORIO__
  void ConfigureSimulation(const talos::hardware::Config& c);
  void UpdateSimulation(double dt);
#endif
  // Builds refresh_signals_ and refresh_groups_ from signals_ (Configure).
  void GroupSignalsByNetwork(const talos::hardware::Config& c);
  // The licence check of the motor at `index` (MotorConfig::foc), on the
  // configure or restore worker. ReadProLicence: true or false as read;
  // nullopt, with the status in `status`, when the read got no answer (up
  // to 0.5 s). RefuseProLicence: marks the motor and its followers in
  // licence_refused_ and posts a console line naming each; with
  // `record_error`, the first becomes last_error_ unless one is set.
  // RestoreProLicence: the restore loop's step: false while still unread;
  // reads only a motor whose licence got no answer before.
  std::optional<bool> ReadProLicence(std::size_t index,
                                     ctre::phoenix::StatusCode& status);
  void RefuseProLicence(std::size_t index, bool record_error);
  bool RestoreProLicence(std::size_t index);
  // Copies licence_refused_ into foc_unlicensed_ (reserved at Configure) and
  // erases every unlicensed id from unavailable_, so an unlicensed motor or
  // follower never churns the restore loop. Loop-thread safe (no allocation).
  void SyncUnlicensedAndPruneUnavailable();

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
  // Whether a device's motion feedback is four status periods old (the LATE
  // line), and the oldest age of its current late spell.
  std::array<bool, talos::hardware::kMaxMotors> motor_late_{};
  std::array<uint32_t, talos::hardware::kMaxMotors> motor_peak_age_us_{};
  std::array<bool, talos::hardware::kMaxSensors> sensor_late_{};
  std::array<uint32_t, talos::hardware::kMaxSensors> sensor_peak_age_us_{};
  uint32_t last_motor_fault_mask_{0};
  std::array<bool, talos::hardware::kMaxMotors> neutral_send_failing_{};
  uint64_t neutral_send_failure_count_{0};
  std::array<bool, talos::hardware::kMaxMotors> apply_failing_{};
  std::vector<uint16_t> apply_failed_;
  bool last_brownout_{false};
  // State's CAN health: one slot per configured CAN bus, the RoboRIO's own
  // first (CanBusHealth.h). Configure rebuilds both through the sampler,
  // whose thread reads the buses; Read polls what it published. Declared
  // before configure_, so a Configure still running finishes before the
  // sampler stops its reader (bounded: CanBusSampler::Stop), and the sampler
  // after the health it reconfigures.
  talos::rio::CanBusHealth can_health_;
  talos::rio::CanBusSampler can_sampler_;
  std::vector<ctre::phoenix6::BaseStatusSignal*> signals_;
  // signals_ again, each CAN bus's contiguous, and each bus's span of them:
  // Read refreshes one span per RefreshAll, since Phoenix refuses a call
  // whose signals are on two networks. Built by Configure.
  struct RefreshGroup {
    std::size_t begin{0};
    std::size_t count{0};
  };
  std::vector<ctre::phoenix6::BaseStatusSignal*> refresh_signals_;
  std::array<RefreshGroup, talos::hardware::kMaxCanBuses> refresh_groups_{};
  std::size_t refresh_group_count_{0};
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
  // Motors with foc set, or following a master with it, read not Phoenix Pro
  // licensed (MotorConfig::foc), and the followers of such a master, by
  // logical id: failed devices until the next Configure. Read reports them
  // invalid and Apply sends them NeutralOut. They are not in
  // UnavailableDevices(): a licence does not arrive by retrying, so the
  // restore loop never churns on them. The loop thread's copy of
  // licence_refused_, taken at Configure and at each finished restore;
  // reserved for every motor, so PollRestore adds to it without allocating.
  std::vector<uint16_t> foc_unlicensed_;
  // By motor index, the motors refused as foc_unlicensed_ describes. Written
  // only by the configure and restore workers (ReadProLicence's callers);
  // the loop thread reads it once the worker's future is ready.
  std::array<bool, talos::hardware::kMaxMotors> licence_refused_{};
  // By motor index, whether the motor's licence was read (true or false)
  // under this configuration: a restore reads only one that got no answer,
  // so a device reset costs no read. Written only by the configure and
  // restore workers, read by the next of them; the gateway never runs a
  // configure and a restore at once, and each starts after the last one's
  // future was taken, which orders their accesses.
  std::array<bool, talos::hardware::kMaxMotors> licence_read_{};
  // Whether the motor at `motor_index` of the configuration is Phoenix Pro
  // licensed: true or false as read, nullopt when the read got no answer.
  // Empty: its IsProLicensed signal off simulation, and true under
  // simulation, which needs no licence. phoenixTest sets it.
  std::function<std::optional<bool>(std::size_t motor_index)> pro_licensed_;
  // The battery voltage a duty-cycle or torque-current request's duty cap
  // divides max_voltage by (Apply): Read's last reading that was finite and
  // above 1 V, kept across a reading that is not. 0 until the first such
  // reading, and the cap is then 0: neither mode drives on a guess.
  double last_battery_v_{0};

  bool is_simulation_{false};
  // Desktop sim only: true while the motor is commanded neutral (NeutralOut),
  // so UpdateSimulation decays the rotor per brake/coast instead of tracking
  // voltage.
  std::array<bool, talos::hardware::kMaxMotors> sim_neutral_{};
#ifndef __FRC_ROBORIO__
  struct SimMotorPlant {
    frc::LinearSystem<2, 1, 2> system{frc::LinearSystemId::DCMotorSystem(
        frc::DCMotor::KrakenX60(1), units::kilogram_square_meter_t{0.0005},
        1.0)};
    frc::Vectord<2> state{frc::Vectord<2>::Zero()};
    double gear_ratio{1.0};
  };
  struct SimChassisModule {
    std::size_t drive_index{0};
    // -1 for differential-drive wheels (fixed 0 rad azimuth).
    int steer_index{-1};
    double x_m{0.0};
    double y_m{0.0};
  };
  std::optional<std::chrono::steady_clock::time_point> last_sim_time_;
  std::array<SimMotorPlant, talos::hardware::kMaxMotors> sim_plants_{};
  std::array<int, talos::hardware::kMaxMotors> motor_to_sensor_map_{};
  std::array<SimChassisModule, talos::hardware::kMaxMotors>
      sim_chassis_modules_{};
  std::size_t sim_chassis_module_count_{0};
  double sim_yaw_deg_{0.0};
  double last_sim_supply_v_{0.0};
#endif
  using RestoreTask = std::function<std::optional<std::vector<uint16_t>>()>;
  // Launches the restore worker; empty calls std::async(std::launch::async).
  // phoenixTest sets it to inject thread-exhaustion (EAGAIN) failures.
  std::function<std::future<std::optional<std::vector<uint16_t>>>(RestoreTask)>
      launch_restore_;
  // Polls (and clears) one Phoenix device's reset latch. Empty calls
  // device.HasResetOccurred(). phoenixTest sets it because Phoenix 6 desktop
  // simulation provides no reset-latch trigger.
  std::function<bool(ctre::phoenix6::hardware::ParentDevice&)>
      poll_device_reset_;
  bool restore_launch_failed_{false};
  // Held by Configure while it rebuilds the device lists, and by
  // WatchdogNeutral, which only tries it: it never waits on a configure.
  // Declared before configure_, whose destruction waits for a Configure
  // still holding it.
  std::mutex devices_mutex_;
  std::future<bool> configure_;
  std::future<std::optional<std::vector<uint16_t>>> restore_;
};

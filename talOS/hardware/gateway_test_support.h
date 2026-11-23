#pragma once

// Fixtures shared by the gateway's tests (gateway_test.cc,
// gateway_endpoint_test.cc): a swerve configuration, a backend that records
// what the gateway asks of it, and a command built from the gateway's State.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "config.h"
#include "config_error.h"
#include "gateway.h"
#include "messages.h"

namespace talos::hardware::gateway_test {

struct ModuleConfig {
  const char* name;
  uint16_t drive_id, steer_id, encoder_id;
  double x_m, y_m;
  double wheel_radius_m;
  int drive_can_id, steer_can_id, encoder_can_id;
  const char* bus{"rio"};
  double encoder_offset_rot{0};
  bool drive_inverted{false}, steer_inverted{false}, encoder_inverted{false};
};

inline constexpr std::array<ModuleConfig, 4> kTestSwerveModules{{
    {"back_left", 1, 3, 2, -0.30, 0.30, 0.0508, 5, 6, 22},
    {"back_right", 4, 6, 5, -0.30, -0.30, 0.0508, 7, 8, 23},
    {"front_left", 8, 10, 9, 0.30, 0.30, 0.0508, 1, 2, 20},
    {"front_right", 11, 13, 12, 0.30, -0.30, 0.0508, 3, 4, 21},
}};

inline Config SwerveConfig(bool simulation = false) {
  Config c;
  c.commissioned = simulation;
  for (std::size_t i = 0; i < kTestSwerveModules.size(); ++i) {
    const auto& module = kTestSwerveModules[i];
    SensorConfig encoder;
    encoder.id = module.encoder_id;
    encoder.name = std::string{module.name} + "_encoder";
    encoder.can_id = module.encoder_can_id;
    encoder.bus = module.bus;
    encoder.offset_rot = module.encoder_offset_rot;
    encoder.inverted = module.encoder_inverted;
    c.sensors.push_back(encoder);
    MotorConfig drive;
    drive.id = module.drive_id;
    drive.name = std::string{module.name} + "_drive";
    drive.can_id = module.drive_can_id;
    drive.bus = module.bus;
    drive.inverted = module.drive_inverted;
    drive.sensor_to_mechanism_ratio = 6.75;
    drive.max_velocity_rps = 20;
    drive.slots[0].p = 0;
    drive.slots[0].v = 0;
    c.motors.push_back(drive);
    MotorConfig steer;
    steer.id = module.steer_id;
    steer.name = std::string{module.name} + "_steer";
    steer.can_id = module.steer_can_id;
    steer.bus = module.bus;
    steer.inverted = module.steer_inverted;
    steer.feedback = Feedback::kRemoteCANcoder;
    steer.feedback_sensor_id = encoder.id;
    steer.rotor_to_sensor_ratio = 12.8;
    steer.continuous_wrap = true;
    steer.supply_limit_a = 20;
    steer.stator_limit_a = 40;
    steer.cruise_velocity_rps = 5;
    steer.acceleration_rps2 = 20;
    steer.slots[0].p = 0;
    c.motors.push_back(steer);
  }
  c.sensors.push_back({7, "drivetrain_imu", SensorKind::kPigeon2, 30, "rio"});
  std::sort(c.sensors.begin(), c.sensors.end(),
            [](const auto& a, const auto& b) { return a.name < b.name; });
  std::sort(c.motors.begin(), c.motors.end(),
            [](const auto& a, const auto& b) { return a.name < b.name; });
  return c;
}
class FakeBackend : public Backend {
 public:
  bool Configure(const Config&) override {
    ++configure_calls;
    events.push_back("configure");
    return configure_ok;
  }
  // The asynchronous path records its start, then completes through
  // Configure as the default does.
  void BeginConfigure(const Config& config) override {
    events.push_back("begin_configure");
    Backend::BeginConfigure(config);
  }
  std::vector<uint16_t> UnavailableDevices() const override {
    return unavailable;
  }
  bool Read(State& s) override {
    for (auto& m : s.motors) {
      m.valid = healthy && m.id != invalid_motor;
      if (const auto it = positions.find(m.id); it != positions.end()) {
        m.position_rot = it->second;
      }
      m.velocity_rps = m.voltage = m.stator_current_a = m.supply_current_a =
          measurement;
      m.position_age_us = m.velocity_age_us = age_us;
      if (const auto it = position_ages.find(m.id); it != position_ages.end()) {
        m.position_age_us = it->second;
      }
      if (const auto it = velocity_ages.find(m.id); it != velocity_ages.end()) {
        m.velocity_age_us = it->second;
      }
    }
    for (auto& sensor : s.sensors) {
      sensor.valid = healthy && sensor.id != invalid_sensor;
      sensor.position_rot = sensor.velocity_rps = measurement;
      sensor.position_age_us = sensor.velocity_age_us = age_us;
      if (const auto it = position_ages.find(sensor.id);
          it != position_ages.end()) {
        sensor.position_age_us = it->second;
      }
      if (const auto it = velocity_ages.find(sensor.id);
          it != velocity_ages.end()) {
        sensor.velocity_age_us = it->second;
      }
    }
    for (auto& input : s.digital_inputs) {
      input.valid = true;
      input.value = true;
    }
    s.health = {.valid = true, .battery_v = 12};
    return healthy && !read_fails;
  }
  bool Apply(std::span<const MotorRequest> requests) override {
    last_requests.assign(requests.begin(), requests.end());
    calls.push_back({Call::kApply, last_requests});
    ++applied;
    // Device-local send failures. last_requests keeps what the gateway asked
    // for, so tests observe the gateway's own neutralization.
    apply_failed.clear();
    const bool fail_all =
        fail_all_after_neutral &&
        std::any_of(requests.begin(), requests.end(), [](const auto& request) {
          return request.mode == Mode::kNeutral;
        });
    for (const auto& request : requests) {
      if (fail_all || std::find(failing_motors.begin(), failing_motors.end(),
                                request.id) != failing_motors.end())
        apply_failed.push_back(request.id);
    }
    return apply_ok;
  }
  std::span<const uint16_t> ApplyFailedDevices() const override {
    return apply_failed;
  }
  bool ApplyOutputs(std::span<const DigitalOutputRequest> digital,
                    std::span<const PwmRequest> pwm) override {
    last_digital_outputs.assign(digital.begin(), digital.end());
    last_pwm_outputs.assign(pwm.begin(), pwm.end());
    return outputs_ok;
  }
  void Neutral() override {
    ++neutral;
    events.push_back("neutral");
    calls.push_back({Call::kNeutral, {}});
  }
  std::optional<ConfigError> last_config_error() const override {
    return config_error;
  }
  bool PollDeviceResets(std::vector<uint16_t>& ids) override {
    ids = resets;
    resets.clear();
    return !ids.empty();
  }
  void BeginRestoreAfterReset(std::span<const uint16_t> ids) override {
    restoring_ids.assign(ids.begin(), ids.end());
    events.push_back("restore");
    calls.push_back({Call::kBeginRestore, {}});
  }
  RestoreStatus PollRestore() override {
    if (pending_ticks-- > 0) return RestoreStatus::kPending;
    if (restore_ok) {
      std::erase_if(unavailable, [&](uint16_t id) {
        return std::find(restoring_ids.begin(), restoring_ids.end(), id) !=
               restoring_ids.end();
      });
    }
    calls.push_back({Call::kRestoreFinished, {}});
    return restore_ok ? RestoreStatus::kDone : RestoreStatus::kFailed;
  }
  // Every output and restore call in order, with the requests each Apply
  // carried, so a test can see what reached a device around its restore.
  struct Call {
    enum Kind { kApply, kNeutral, kBeginRestore, kRestoreFinished } kind;
    std::vector<MotorRequest> requests;
  };
  std::vector<Call> calls;
  int pending_ticks{};
  bool configure_ok{true}, healthy{true}, apply_ok{true}, restore_ok{true};
  bool outputs_ok{true};
  // Read reports a failed pass (returns false) while its samples still
  // follow `healthy`: total loss with valid samples, which the Backend
  // contract does not rule out.
  bool read_fails{false};
  bool fail_all_after_neutral{false};
  std::vector<uint16_t> failing_motors, apply_failed;
  std::vector<DigitalOutputRequest> last_digital_outputs;
  std::vector<PwmRequest> last_pwm_outputs;
  int applied{}, neutral{}, configure_calls{};
  uint16_t invalid_motor{}, invalid_sensor{};
  std::vector<MotorRequest> last_requests{};
  std::vector<std::string> events{};
  std::vector<uint16_t> resets{};
  std::vector<uint16_t> restoring_ids;
  std::vector<uint16_t> unavailable;
  std::optional<ConfigError> config_error{};
  // Reported motor positions by id; others keep theirs. Every motor's and
  // sensor's ages are `age_us`, except a position or velocity age (motor or
  // sensor) named here. The fake never judges an age itself: the gateway
  // does, for every backend.
  std::map<uint16_t, double> positions;
  // Every motor's velocity, voltage, stator and supply current, and every
  // sensor's position and velocity, valid or not.
  double measurement{};
  uint32_t age_us{};
  std::map<uint16_t, uint32_t> position_ages, velocity_ages;
};
// A command observing `g`'s State: every motor at velocity 1, no digital or
// PWM output.
inline Command Request(const Gateway& g) {
  const auto s = g.snapshot();
  Command c{};
  c.config_id = s.config_id;
  c.boot_id = s.boot_id;
  c.epoch = s.epoch;
  c.observed_time_us = s.sample_time_us;
  c.count = s.motor_count;
  for (std::size_t i = 0; i < c.count; ++i) {
    c.motors[i] = {s.motors[i].id, Mode::kVelocity, 0, 1, 0};
  }
  return c;
}

}  // namespace talos::hardware::gateway_test

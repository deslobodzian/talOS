// Startup restore of the persisted hardware configuration: it must not wait
// for device configuration (a mid-match reboot with absent devices would hold
// robot code start for a CAN timeout per device), and nothing drives until
// the restore completes.
#include "ConfigStore.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <span>
#include <string>

#include "hardware/gateway.h"

namespace hw = talos::hardware;

namespace {

class PendingBackend final : public hw::Backend {
 public:
  bool Configure(const hw::Config&) override {
    ++blocking_configures;
    return true;
  }
  void BeginConfigure(const hw::Config&) override { ++begun; }
  ConfigureStatus PollConfigure() override {
    return done ? ConfigureStatus::kDone : ConfigureStatus::kPending;
  }
  bool Read(hw::State& state) override {
    for (auto& motor : state.motors) motor.valid = true;
    return true;
  }
  bool Apply(std::span<const hw::MotorRequest>) override {
    ++applied;
    return true;
  }
  std::span<const uint16_t> ApplyFailedDevices() const override { return {}; }
  void Neutral() override {}
  int blocking_configures{}, begun{}, applied{};
  bool done{false};
};

hw::Command DutyCycle(const hw::Gateway& gateway) {
  const auto state = gateway.snapshot();
  hw::Command command{};
  command.config_id = state.config_id;
  command.boot_id = state.boot_id;
  command.epoch = state.epoch;
  command.observed_time_us = state.sample_time_us;
  command.count = state.motor_count;
  command.motors[0] = {state.motors[0].id, hw::Mode::kDutyCycle, 0, 0.2, 0};
  return command;
}

// REQ-CONTRACT-06
TEST(ConfigStore, StartupRestoreDoesNotWaitForDevicesAndDrivesOnlyAfter) {
  hw::Config config;
  config.commissioned = true;
  hw::MotorConfig motor;
  motor.id = 1;
  motor.name = "test_motor";
  motor.can_id = 1;
  config.motors.push_back(motor);
  const auto path =
      (std::filesystem::temp_directory_path() / "talos_config_store_test.bin")
          .string();
  ASSERT_TRUE(talos::rio::SaveConfigFile(path, config));

  PendingBackend backend;
  hw::Gateway gateway{backend, 7};
  EXPECT_EQ(talos::rio::BeginRestore(gateway, path), config);
  // The blocking Configure() is never called on this thread.
  EXPECT_EQ(backend.blocking_configures, 0);
  EXPECT_EQ(backend.begun, 1);

  // A mid-match reboot: the Driver Station is enabled while devices are still
  // being configured; the gateway stays unconfigured and drives nothing.
  gateway.Tick(1000, true);
  EXPECT_FALSE(gateway.configured());
  EXPECT_EQ(backend.applied, 0);

  backend.done = true;
  gateway.Tick(6000, true);
  ASSERT_TRUE(gateway.configured());
  EXPECT_EQ(gateway.snapshot().config_id, hw::ConfigurationId(config));
  EXPECT_TRUE(gateway.Accept(DutyCycle(gateway), 2, 6000));
  EXPECT_EQ(backend.applied, 1);

  std::filesystem::remove(path);
}

}  // namespace

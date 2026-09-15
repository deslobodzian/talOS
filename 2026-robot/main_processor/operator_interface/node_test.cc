#include "talos_nodes/year2026_operator_interface/node.h"

#include <gtest/gtest.h>

#include <chrono>

#include "talOS/events/simulated_event_loop.h"

namespace talos::oi::year2026 {
namespace {

using namespace std::chrono_literals;
using driver_station::DriverStationState;
using driver_station::FmsState;
using driver_station::JoystickState;

JoystickState Stick(float axis0, float axis1, float axis2, uint32_t buttons = 0,
                    bool connected = true) {
  return JoystickState{connected, 4, 12, 1, buttons, axis0, axis1, axis2, 0, 0};
}

DriverStationState DsState(uint32_t flags, JoystickState stick) {
  return DriverStationState{0, 0, FmsState{flags, 0, 0, 0, 0, 0, 0.0f}, stick,
                            JoystickState{}};
}

constexpr uint32_t kTeleopEnabled = driver_station::kEnabled |
                                    driver_station::kTeleop |
                                    driver_station::kDsAttached;

class Harness {
 public:
  Harness() : loop_{env_}, node_{loop_, YearOperatorInterfaceConfig{}} {}

  void Push(const DriverStationState& state) {
    loop_.inject(driver_station::kDsStateTopic, state);
    loop_.run_for(10ms);
  }
  talos::shooter::ShooterTarget last_shooter() {
    auto& channel = env_.channel(talos::shooter::kTeleopShooterTargetTopic,
                                 sizeof(talos::shooter::ShooterTarget));
    talos::shooter::ShooterTarget target{};
    EXPECT_GT(channel.next_sequence(), 0u);
    if (channel.next_sequence() == 0) return target;
    channel.copy_to(channel.next_sequence() - 1,
                    {reinterpret_cast<std::byte*>(&target), sizeof(target)});
    return target;
  }
  talos::intake::IntakeTarget last_intake() {
    auto& channel = env_.channel(talos::intake::kTeleopIntakeTargetTopic,
                                 sizeof(talos::intake::IntakeTarget));
    talos::intake::IntakeTarget target{};
    EXPECT_GT(channel.next_sequence(), 0u);
    if (channel.next_sequence() == 0) return target;
    channel.copy_to(channel.next_sequence() - 1,
                    {reinterpret_cast<std::byte*>(&target), sizeof(target)});
    return target;
  }
  talos::drive::ChassisTarget last_chassis() {
    auto& channel = env_.channel(talos::drive::kTeleopTargetTopic,
                                 sizeof(talos::drive::ChassisTarget));
    talos::drive::ChassisTarget target{};
    EXPECT_GT(channel.next_sequence(), 0u);
    if (channel.next_sequence() == 0) return target;
    channel.copy_to(channel.next_sequence() - 1,
                    {reinterpret_cast<std::byte*>(&target), sizeof(target)});
    return target;
  }

 private:
  event::SimulationEnvironment env_;
  event::SimulatedEventLoop<> loop_;
  YearOperatorInterfaceNode<event::SimulatedEventLoop<>> node_;
};

TEST(YearOperatorInterface, RunsShooterOnButtonOne) {
  Harness harness{};
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/1)));
  const YearOperatorInterfaceConfig defaults{};
  EXPECT_TRUE(harness.last_shooter().enabled());
  EXPECT_NEAR(harness.last_shooter().target_velocity_rps(),
              defaults.shooter_target_rps, 1e-9);
  // Chassis core still publishes alongside the mechanism binding.
  EXPECT_TRUE(harness.last_chassis().enabled());

  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/0)));
  EXPECT_FALSE(harness.last_shooter().enabled());
}

TEST(YearOperatorInterface, RunsIntakeOnButtonTwo) {
  Harness harness{};
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/1)));
  EXPECT_FALSE(harness.last_intake().enabled());
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/2)));
  const YearOperatorInterfaceConfig defaults{};
  EXPECT_TRUE(harness.last_intake().enabled());
  EXPECT_NEAR(harness.last_intake().roller_velocity_rps(),
              defaults.intake_target_rps, 1e-6);
}

TEST(YearOperatorInterface, ReleasesMechanismsOnDisableEdge) {
  Harness harness{};
  harness.Push(DsState(kTeleopEnabled, Stick(0, 0, 0, /*buttons=*/3)));
  EXPECT_TRUE(harness.last_shooter().enabled());
  EXPECT_TRUE(harness.last_intake().enabled());
  harness.Push(DsState(driver_station::kDsAttached, Stick(0, 0, 0)));
  EXPECT_FALSE(harness.last_shooter().enabled());
  EXPECT_FALSE(harness.last_intake().enabled());
}

}  // namespace
}  // namespace talos::oi::year2026

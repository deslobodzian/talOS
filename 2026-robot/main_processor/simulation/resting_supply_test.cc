// REQ-DEV-05: a healthy simulated session passes the 2026 dashboard's
// pre-match Battery check, read from dashboard.toml, at rest and after a
// supply_voltage fault ends.
#include <gtest/gtest.h>

#include <string>

#include "talOS/hardware/gateway.h"
#include "talOS/hardware/sim_backend.h"
#include "talOS/hardware/sim_physics.h"
#include "talOS/simulation/schedules.h"
#include "toml.hpp"

namespace talos::simulation {
namespace {

double BatteryThreshold() {
  const toml::table dashboard = toml::parse_file(
      "2026-robot/main_processor/configuration/dashboard.toml");
  for (const auto& widget : *dashboard["widgets"].as_array()) {
    const auto* items = widget.as_table()->get_as<toml::array>("items");
    if (items == nullptr) continue;
    for (const auto& item : *items) {
      const auto& check = *item.as_table();
      if (check["channel"].value_or(std::string{}) == "hw/status/battery_v")
        return check["at_least"].value_or(0.0);
    }
  }
  ADD_FAILURE() << "dashboard.toml has no hw/status/battery_v check";
  return 0.0;
}

talos::hardware::Config OneMotor() {
  talos::hardware::Config config;
  config.commissioned = true;
  config.command_timeout_us = 100000;
  config.motors.push_back({.id = 1, .name = "drive", .can_id = 1});
  return config;
}

template <typename Backend>
double RestingBattery(Backend& backend) {
  talos::hardware::Gateway gateway{OneMotor(), backend, 1};
  gateway.Tick(1000, false);
  return gateway.snapshot().health.battery_v;
}

TEST(RestingSupply, HealthySimulatedRobotPassesTheBatteryCheck) {
  const double threshold = BatteryThreshold();
  ASSERT_GT(threshold, 0.0);
  talos::hardware::SimBackend ideal;
  EXPECT_GE(RestingBattery(ideal), threshold);
  talos::hardware::sim::PhysicsSimBackend physics;
  EXPECT_GE(RestingBattery(physics), threshold);

  // A supply fault overrides the rest, and its end restores the rest.
  auto events = ParseFaults("1:2:supply_voltage:all:6");
  ResolveFaults(events, OneMotor());
  ApplyFaults(events, 1, physics);
  EXPECT_DOUBLE_EQ(physics.supply_voltage(), 6);
  ApplyFaults(events, 2, physics);
  EXPECT_GE(physics.supply_voltage(), threshold);
}

}  // namespace
}  // namespace talos::simulation

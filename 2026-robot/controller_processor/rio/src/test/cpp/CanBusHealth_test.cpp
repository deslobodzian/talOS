// What State's CAN health reports: the busiest of the RoboRIO's own bus and
// every CANivore the applied configuration has a device on, read one
// CANivore per 100 ms on the 5 ms loop (a read can block up to 1 ms), a
// failed read never passing for a healthy bus, and nothing read on a CANivore
// for a configuration or a simulation that has none.
#include "CanBusHealth.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "hardware/config.h"

namespace {

namespace hw = talos::hardware;
using talos::rio::CanBusHealth;

constexpr uint64_t kLoopPeriodUs = 5'000;
// The first Read after a configuration: an arbitrary steady-clock time.
constexpr uint64_t kStartUs = 7'000'000;

hw::Config Devices(const std::vector<std::string>& motor_buses,
                   const std::vector<std::string>& sensor_buses) {
  hw::Config config;
  uint16_t id = 1;
  for (const auto& bus : motor_buses) {
    hw::MotorConfig motor;
    motor.id = id;
    motor.name = "motor" + std::to_string(id);
    motor.can_id = id;
    motor.bus = bus;
    config.motors.push_back(motor);
    ++id;
  }
  for (const auto& bus : sensor_buses) {
    hw::SensorConfig sensor;
    sensor.id = id;
    sensor.name = "sensor" + std::to_string(id);
    sensor.can_id = id;
    sensor.bus = bus;
    config.sensors.push_back(sensor);
    ++id;
  }
  return config;
}

// A CANivore reader answering each bus's current figure (OK), and recording
// which bus each read asked for.
struct Buses {
  std::map<std::string, double, std::less<>> frac;
  std::vector<std::string> reads;
  CanBusHealth::Sample operator()(std::string_view bus) {
    reads.emplace_back(bus);
    const auto it = frac.find(bus);
    if (it == frac.end()) return {false, 0.0, "CANBusNotFound"};
    return {true, it->second, "OK"};
  }
};

// Scenario (bench, 2026-11-14): three Talon FX, a CANcoder and a Pigeon 2,
// all on the CANivore "Shooter"; the RoboRIO's own bus carries nothing and
// reads 0. A 30 s session read can_utilization_frac 0 throughout, so a
// saturated CANivore was invisible. The figure must be the CANivore's, read
// once for the five devices.
// REQ-OPS-20
TEST(CanBusHealth, ReportsTheCanivoreTheBenchDevicesAreOn) {
  CanBusHealth health;
  health.Configure(
      Devices({"Shooter", "Shooter", "Shooter"}, {"Shooter", "Shooter"}),
      /*simulation=*/false);
  Buses buses;
  buses.frac["Shooter"] = 0.31;
  const auto result = health.Poll(kStartUs, 0.0, buses);
  EXPECT_DOUBLE_EQ(result.utilization_frac, 0.31)
      << "the RoboRIO bus's 0 hid the CANivore's load";
  EXPECT_EQ(buses.reads, std::vector<std::string>{"Shooter"});
}

// The field is one number for every bus: the busiest of them, whichever
// that is, including the RoboRIO's own bus, which carries the power and
// pneumatics modules whether or not a configured device is on it.
// REQ-OPS-20
TEST(CanBusHealth, ReportsTheBusiestOfTheRoboRioBusAndEveryCanivore) {
  CanBusHealth health;
  health.Configure(Devices({"rio", "Shooter", "Arm"}, {"Arm"}), false);
  Buses buses;
  buses.frac["Shooter"] = 0.5;
  buses.frac["Arm"] = 0.1;
  CanBusHealth::Result result;
  uint64_t now = kStartUs;
  for (; now < kStartUs + 2 * CanBusHealth::kSamplePeriodUs;
       now += kLoopPeriodUs)
    result = health.Poll(now, 0.2, buses);
  EXPECT_DOUBLE_EQ(result.utilization_frac, 0.5);
  EXPECT_DOUBLE_EQ(health.Poll(now, 0.6, buses).utilization_frac, 0.6);
}

// The 2026 robot and every simulation config put every device on "rio", and
// under WPILib simulation Phoenix simulates a CANivore's devices without one:
// neither may call into a CANivore, and the field stays the RoboRIO bus's
// figure exactly, as before CANivores counted.
// REQ-OPS-20
TEST(CanBusHealth, ARoboRioOnlyOrSimulatedConfigReadsNoCanivore) {
  for (const bool simulation : {false, true}) {
    CanBusHealth health;
    health.Configure(simulation ? Devices({"Shooter"}, {"Shooter"})
                                : Devices({"rio", "rio"}, {"rio"}),
                     simulation);
    Buses buses;
    for (uint64_t now = kStartUs; now < kStartUs + 1'000'000;
         now += kLoopPeriodUs) {
      const double rio = 0.01 * static_cast<double>((now / kLoopPeriodUs) % 50);
      EXPECT_EQ(health.Poll(now, rio, buses).utilization_frac, rio);
    }
    EXPECT_TRUE(buses.reads.empty()) << "simulation=" << simulation;
  }
}

// A CANivore whose status read fails (unplugged, renamed, its daemon down)
// must not read as an idle bus, nor keep its last figure as if current: it
// drops out of the maximum until it reads again. The console sees one line
// when the spell starts and one, with its count, when it ends.
// REQ-OPS-20
TEST(CanBusHealth, AFailedReadDropsItsBusOutUntilItReadsAgain) {
  CanBusHealth health;
  health.Configure(Devices({"Shooter"}, {}), false);
  uint64_t now = kStartUs;
  const auto poll = [&](CanBusHealth::Sample sample) {
    const auto result =
        health.Poll(now, 0.05, [&](std::string_view) { return sample; });
    now += CanBusHealth::kSamplePeriodUs;
    return result;
  };
  EXPECT_DOUBLE_EQ(poll({true, 0.7, "OK"}).utilization_frac, 0.7);

  auto result = poll({false, 0.0, "CANBusNotFound"});
  EXPECT_DOUBLE_EQ(result.utilization_frac, 0.05)
      << "a failed bus kept its earlier figure";
  EXPECT_EQ(result.event, CanBusHealth::Event::kFailing);
  EXPECT_STREQ(result.status, "CANBusNotFound");

  result = poll({false, 0.0, "CANBusNotFound"});
  EXPECT_EQ(result.event, CanBusHealth::Event::kNone);
  // An OK status with a figure that is no fraction is a failed read too.
  result = poll({true, std::numeric_limits<double>::quiet_NaN(), "OK"});
  EXPECT_DOUBLE_EQ(result.utilization_frac, 0.05);
  EXPECT_EQ(result.event, CanBusHealth::Event::kNone);

  result = poll({true, 0.4, "OK"});
  EXPECT_DOUBLE_EQ(result.utilization_frac, 0.4);
  EXPECT_EQ(result.event, CanBusHealth::Event::kRecovered);
  EXPECT_EQ(result.spell_failures, 3u);
}

// A CANivore status read can block the 5 ms loop for up to 1 ms, so the loop
// makes at most one per 100 ms, taking the buses in turn, and between reads
// reports each bus's last figure rather than dropping it.
// REQ-OPS-20
TEST(CanBusHealth, ReadsOneCanivorePerSamplePeriodInTurnAndHoldsTheFigures) {
  CanBusHealth health;
  health.Configure(Devices({"Shooter", "Arm"}, {}), false);
  Buses buses;
  buses.frac["Shooter"] = 0.3;
  buses.frac["Arm"] = 0.6;
  bool arm_read = false;
  for (uint64_t now = kStartUs; now < kStartUs + 1'000'000;
       now += kLoopPeriodUs) {
    const double frac = health.Poll(now, 0.0, buses).utilization_frac;
    arm_read =
        arm_read || (!buses.reads.empty() && buses.reads.back() == "Arm");
    EXPECT_DOUBLE_EQ(frac, arm_read ? 0.6 : 0.3) << "at +" << now - kStartUs;
  }
  std::vector<std::string> expected;
  for (int i = 0; i < 5; ++i) {
    expected.emplace_back("Shooter");
    expected.emplace_back("Arm");
  }
  EXPECT_EQ(buses.reads, expected);
}

// The bus set is the applied configuration's: a new one forgets a bus no
// device is on any more, and reads its own first bus on its first tick.
// REQ-OPS-20
TEST(CanBusHealth, ANewConfigurationRebuildsTheBusSet) {
  CanBusHealth health;
  health.Configure(Devices({"Shooter"}, {}), false);
  Buses buses;
  buses.frac["Shooter"] = 0.9;
  buses.frac["Arm"] = 0.2;
  EXPECT_DOUBLE_EQ(health.Poll(kStartUs, 0.0, buses).utilization_frac, 0.9);
  health.Configure(Devices({}, {"Arm"}), false);
  EXPECT_DOUBLE_EQ(
      health.Poll(kStartUs + kLoopPeriodUs, 0.0, buses).utilization_frac, 0.2);
  EXPECT_EQ(buses.reads, (std::vector<std::string>{"Shooter", "Arm"}));
}

}  // namespace

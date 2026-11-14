// What State's CAN health reports: the busiest of the RoboRIO's own bus and
// every CANivore the applied configuration has a device on. A CANivore's
// status read blocks about 1 ms, so a sampler thread makes it and the 5 ms
// loop only copies what it published; a failed or stale read never passes for
// a healthy bus, a configuration change never mixes the old buses' reads into
// the new set, and nothing reads a CANivore for a configuration or a
// simulation that has none.
#include "CanBusHealth.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "hardware/config.h"

namespace {

namespace hw = talos::hardware;
using talos::rio::CanBusHealth;
using talos::rio::CanBusSampler;
using Change = CanBusHealth::Change;
using Event = CanBusHealth::Event;

constexpr uint64_t kLoopPeriodUs = 5'000;
constexpr uint64_t kPeriodUs = CanBusHealth::kSamplePeriodUs;
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

// The loop's side: Poll, collecting what it reports.
struct Loop {
  explicit Loop(CanBusHealth& polled) : health{polled} {}
  CanBusHealth& health;
  std::vector<Change> changes;
  double Poll(uint64_t now_us, double rio_frac) {
    return health.Poll(now_us, rio_frac, [this](const Change& change) {
      changes.push_back(change);
    });
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
  health.SampleNext(kStartUs, buses);
  Loop loop{health};
  EXPECT_DOUBLE_EQ(loop.Poll(kStartUs + kLoopPeriodUs, 0.0), 0.31)
      << "the RoboRIO bus's 0 hid the CANivore's load";
  EXPECT_EQ(buses.reads, std::vector<std::string>{"Shooter"});
  EXPECT_TRUE(loop.changes.empty());
}

// The field is one number for every bus: the busiest of them, whichever
// that is, including the RoboRIO's own bus, which carries the power and
// pneumatics modules whether or not a configured device is on it. The
// CANivores are read in turn, so each one's figure is refreshed every
// (buses) sampling periods.
// REQ-OPS-20
TEST(CanBusHealth, ReportsTheBusiestOfTheRoboRioBusAndEveryCanivore) {
  CanBusHealth health;
  health.Configure(Devices({"rio", "Shooter", "Arm"}, {"Arm"}), false);
  Buses buses;
  buses.frac["Shooter"] = 0.5;
  buses.frac["Arm"] = 0.1;
  for (int i = 0; i < 4; ++i)
    health.SampleNext(kStartUs + static_cast<uint64_t>(i) * kPeriodUs, buses);
  EXPECT_EQ(buses.reads,
            (std::vector<std::string>{"Shooter", "Arm", "Shooter", "Arm"}));
  Loop loop{health};
  const uint64_t now = kStartUs + 3 * kPeriodUs + kLoopPeriodUs;
  EXPECT_DOUBLE_EQ(loop.Poll(now, 0.2), 0.5);
  EXPECT_DOUBLE_EQ(loop.Poll(now + kLoopPeriodUs, 0.6), 0.6);
}

// The 2026 robot and every simulation config put every device on "rio", and
// under WPILib simulation Phoenix simulates a CANivore's devices without one:
// neither has a bus to read, and the field stays the RoboRIO bus's figure
// exactly, as before CANivores counted.
// REQ-OPS-20
TEST(CanBusHealth, ARoboRioOnlyOrSimulatedConfigHasNoCanivoreToRead) {
  for (const bool simulation : {false, true}) {
    CanBusHealth health;
    health.Configure(simulation ? Devices({"Shooter"}, {"Shooter"})
                                : Devices({"rio", "rio"}, {"rio"}),
                     simulation);
    EXPECT_EQ(health.bus_count(), 0u);
    Buses buses;
    health.SampleNext(kStartUs, buses);
    EXPECT_TRUE(buses.reads.empty()) << "simulation=" << simulation;
    Loop loop{health};
    for (uint64_t now = kStartUs; now < kStartUs + 1'000'000;
         now += kLoopPeriodUs) {
      const double rio = 0.01 * static_cast<double>((now / kLoopPeriodUs) % 50);
      EXPECT_EQ(loop.Poll(now, rio), rio);
    }
    EXPECT_TRUE(loop.changes.empty());
  }
}

// A CANivore whose status read fails (unplugged, renamed, its daemon down)
// must not read as an idle bus, nor keep its last figure as if current: it
// drops out of the maximum until it reads again. The loop reports the spell
// once when it starts and once, with its count, when it ends.
// REQ-OPS-20
TEST(CanBusHealth, AFailedReadDropsItsBusOutUntilItReadsAgain) {
  CanBusHealth health;
  health.Configure(Devices({"Shooter"}, {}), false);
  Loop loop{health};
  uint64_t now = kStartUs;
  const auto read = [&](CanBusHealth::Sample sample) {
    health.SampleNext(now, [&](std::string_view) { return sample; });
    now += kLoopPeriodUs;
    const double frac = loop.Poll(now, 0.05);
    now += kPeriodUs - kLoopPeriodUs;
    return frac;
  };
  EXPECT_DOUBLE_EQ(read({true, 0.7, "OK"}), 0.7);
  EXPECT_TRUE(loop.changes.empty());

  EXPECT_DOUBLE_EQ(read({false, 0.0, "CANBusNotFound"}), 0.05)
      << "a failed bus kept its earlier figure";
  ASSERT_EQ(loop.changes.size(), 1u);
  EXPECT_EQ(loop.changes[0].event, Event::kFailing);
  EXPECT_FALSE(loop.changes[0].stale);
  EXPECT_STREQ(loop.changes[0].status, "CANBusNotFound");

  read({false, 0.0, "CANBusNotFound"});
  // An OK status with a figure that is no fraction is a failed read too.
  EXPECT_DOUBLE_EQ(read({true, std::numeric_limits<double>::quiet_NaN(), "OK"}),
                   0.05);
  EXPECT_EQ(loop.changes.size(), 1u);

  EXPECT_DOUBLE_EQ(read({true, 0.4, "OK"}), 0.4);
  ASSERT_EQ(loop.changes.size(), 2u);
  EXPECT_EQ(loop.changes[1].event, Event::kRecovered);
  EXPECT_EQ(loop.changes[1].spell_failures, 3u);
}

// A vendor call that never returns, or a sampler starved of the CPU, leaves
// the last figure published. It counts only until the bus's read is three of
// its own sampling periods old on the loop's clock (two buses: 600 ms), then
// the bus drops out as a failed read would; a bus never read drops out the
// same time after the loop's first tick. A read stamped after the loop's
// clock reading (published between the two) is current.
// REQ-OPS-20
TEST(CanBusHealth, AFigureNotRefreshedWithinThreeOfItsPeriodsDropsOut) {
  CanBusHealth health;
  health.Configure(Devices({"Shooter", "Arm"}, {}), false);
  ASSERT_EQ(health.StaleUs(), 600'000u);
  Buses buses;
  buses.frac["Shooter"] = 0.3;
  buses.frac["Arm"] = 0.6;
  Loop loop{health};
  // The loop's first tick comes before any read; Arm's first read is late.
  EXPECT_DOUBLE_EQ(loop.Poll(kStartUs, 0.05), 0.05);
  health.SampleNext(kStartUs + 1'000, buses);  // Shooter
  EXPECT_DOUBLE_EQ(loop.Poll(kStartUs, 0.05), 0.3);
  EXPECT_DOUBLE_EQ(loop.Poll(kStartUs + 600'000 - 1, 0.05), 0.3);
  EXPECT_TRUE(loop.changes.empty());
  EXPECT_DOUBLE_EQ(loop.Poll(kStartUs + 600'000, 0.05), 0.3);
  ASSERT_EQ(loop.changes.size(), 1u) << "Arm, never read, did not drop out";
  EXPECT_EQ(loop.changes[0].bus, 1u);
  EXPECT_TRUE(loop.changes[0].stale);
  EXPECT_EQ(loop.changes[0].status, nullptr);

  // The sampler hangs in its next call: Shooter's figure ages out.
  EXPECT_DOUBLE_EQ(loop.Poll(kStartUs + 601'000 - 1, 0.05), 0.3);
  EXPECT_DOUBLE_EQ(loop.Poll(kStartUs + 601'000, 0.05), 0.05)
      << "a figure older than the stale bound still counted";
  ASSERT_EQ(loop.changes.size(), 2u);
  EXPECT_EQ(loop.changes[1].bus, 0u);
  EXPECT_TRUE(loop.changes[1].stale);
  EXPECT_STREQ(loop.changes[1].status, "OK");

  // It returns: Arm, then Shooter, each back with no failed read counted.
  health.SampleNext(kStartUs + 700'000, buses);
  health.SampleNext(kStartUs + 800'000, buses);
  EXPECT_DOUBLE_EQ(loop.Poll(kStartUs + 800'000, 0.05), 0.6);
  ASSERT_EQ(loop.changes.size(), 4u);
  EXPECT_EQ(loop.changes[2].event, Event::kRecovered);
  EXPECT_EQ(loop.changes[3].event, Event::kRecovered);
  EXPECT_EQ(loop.changes[2].spell_failures, 0u);
}

// A configuration can be applied while the sampler is inside a read of a bus
// the new configuration drops (the vendor call cannot be interrupted). That
// read keeps a name of its own to the end, and publishes nothing: the new
// set's figures come only from its own reads.
// REQ-OPS-20
TEST(CanBusHealth, AReadInFlightAcrossAConfigurationPublishesNothing) {
  CanBusHealth health;
  health.Configure(Devices({"Shooter"}, {}), false);
  std::vector<std::string> reads;
  health.SampleNext(kStartUs, [&](std::string_view bus) {
    health.Configure(Devices({}, {"Arm"}), false);
    reads.emplace_back(bus);
    return CanBusHealth::Sample{true, 0.9, "OK"};
  });
  EXPECT_EQ(reads, std::vector<std::string>{"Shooter"});
  Loop loop{health};
  EXPECT_DOUBLE_EQ(loop.Poll(kStartUs + kLoopPeriodUs, 0.05), 0.05)
      << "the dropped bus's read landed in the new set";
  EXPECT_TRUE(loop.changes.empty());
  Buses buses;
  buses.frac["Arm"] = 0.2;
  health.SampleNext(kStartUs + kPeriodUs, buses);
  EXPECT_EQ(buses.reads, std::vector<std::string>{"Arm"});
  EXPECT_DOUBLE_EQ(loop.Poll(kStartUs + kPeriodUs, 0.05), 0.2);
}

// Records each read the sampler thread makes, for the test to wait on.
struct Recorder {
  std::mutex mutex;
  std::condition_variable read;
  std::vector<std::string> reads;

  // Waits (on reads, not on time; the bound only fails the test) until
  // `count` reads of `bus` were made.
  bool WaitFor(std::string_view bus, std::ptrdiff_t count) {
    std::unique_lock lock{mutex};
    return read.wait_for(lock, std::chrono::seconds{10}, [&] {
      return std::count(reads.begin(), reads.end(), bus) >= count;
    });
  }
};

// The thread runs only for a configuration with a CANivore, never under
// simulation, and a new configuration stops and joins it first: once
// Reconfigure returns, no bus the new configuration dropped is read again.
// REQ-OPS-20
TEST(CanBusSampler, RunsOnlyForCanivoresAndStopsReadingDroppedBuses) {
  CanBusHealth health;
  Recorder recorder;
  CanBusSampler sampler{health,
                        [&recorder](std::string_view bus) {
                          {
                            const std::lock_guard lock{recorder.mutex};
                            recorder.reads.emplace_back(bus);
                          }
                          recorder.read.notify_all();
                          return CanBusHealth::Sample{true, 0.2, "OK"};
                        },
                        +[] { return kStartUs; }, /*period_us=*/1'000};

  sampler.Reconfigure(Devices({"rio"}, {"rio"}), false);
  EXPECT_FALSE(sampler.running());
  sampler.Reconfigure(Devices({"Shooter"}, {}), /*simulation=*/true);
  EXPECT_FALSE(sampler.running());
  EXPECT_TRUE(recorder.reads.empty());

  sampler.Reconfigure(Devices({"Shooter"}, {}), false);
  EXPECT_TRUE(sampler.running());
  ASSERT_TRUE(recorder.WaitFor("Shooter", 2));
  sampler.Reconfigure(Devices({"Arm"}, {}), false);
  ASSERT_TRUE(recorder.WaitFor("Arm", 2));
  sampler.Stop();
  EXPECT_FALSE(sampler.running());

  // Every Shooter read, then every Arm read: none of Shooter after the first
  // of Arm.
  const std::lock_guard lock{recorder.mutex};
  const auto& reads = recorder.reads;
  const auto first_arm = std::find(reads.begin(), reads.end(), "Arm");
  EXPECT_EQ(std::count(reads.begin(), first_arm, "Shooter"),
            first_arm - reads.begin());
  EXPECT_EQ(std::count(first_arm, reads.end(), "Arm"), reads.end() - first_arm)
      << "a bus the new configuration dropped was read after it";
  Loop loop{health};
  EXPECT_DOUBLE_EQ(loop.Poll(kStartUs, 0.0), 0.2);
}

}  // namespace

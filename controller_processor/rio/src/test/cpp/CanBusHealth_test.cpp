// What State's CAN health reports: one slot per configured CAN bus, the
// RoboRIO's own first, each with its utilization, its controller's error
// counters, its read state and its failed reads. A bus status read is a
// vendor library call (HAL_CAN_GetCANStatus, CANBus::GetStatus), so a
// sampler thread makes it and the 5 ms loop only copies what it published; a
// failed or stale read never passes for a healthy bus, a configuration
// change never mixes the old buses' reads into the new set, a read that
// never returns blocks neither a configuration, the loop nor destruction,
// and nothing is read for a simulation or a configuration with no device.
// The CanBusSampler tests run real threads and wait on their reads
// (TESTING.md §6.3).
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
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "hardware/config.h"
#include "hardware/messages.h"

namespace {

namespace hw = talos::hardware;
using talos::rio::CanBusHealth;
using talos::rio::CanBusSampler;
using Change = CanBusHealth::Change;
using Event = CanBusHealth::Event;
using hw::CanBusReadState;

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

CanBusHealth::Sample Ok(double frac) { return {true, frac, 0, 0, 0, 0, "OK"}; }

// A bus reader answering each bus's current read (OK with its figure, or
// what `samples` holds), and recording which bus each read asked for.
struct Buses {
  std::map<std::string, CanBusHealth::Sample, std::less<>> samples;
  std::vector<std::string> reads;
  CanBusHealth::Sample operator()(std::string_view bus) {
    reads.emplace_back(bus);
    const auto it = samples.find(bus);
    if (it == samples.end()) return {false, 0.0, 0, 0, 0, 0, "CANBusNotFound"};
    return it->second;
  }
};

// The loop's side: Poll, collecting what it reports.
struct Loop {
  explicit Loop(CanBusHealth& polled) : health{polled} {}
  CanBusHealth& health;
  std::vector<Change> changes;
  void Poll(uint64_t now_us) {
    health.Poll(now_us,
                [this](const Change& change) { changes.push_back(change); });
  }
};

// Scenario (bench, 2026-11-14): three Talon FX, a CANcoder and a Pigeon 2,
// all on the CANivore "Shooter", and nothing on the RoboRIO's own bus. One
// number for every bus read 0 throughout a 30 s session, so a saturated
// CANivore was invisible. Each bus now has its own slot, "rio" first, with
// its utilization and its controller's counters, the error counters
// saturating at 255.
// REQ-OPS-20
TEST(CanBusHealth, ReportsEachBusInItsOwnSlot) {
  CanBusHealth health;
  health.Configure(
      Devices({"Shooter", "Shooter", "Shooter"}, {"Shooter", "Shooter"}),
      /*simulation=*/false);
  ASSERT_EQ(health.bus_count(), 2u);
  Buses buses;
  buses.samples["rio"] = Ok(0.0);
  buses.samples["Shooter"] = {true, 0.31, 2, 7, 300, 96, "OK"};
  health.SampleNext(kStartUs, buses);
  health.SampleNext(kStartUs + kPeriodUs, buses);
  EXPECT_EQ(buses.reads, (std::vector<std::string>{"rio", "Shooter"}));
  Loop loop{health};
  loop.Poll(kStartUs + kPeriodUs + kLoopPeriodUs);
  EXPECT_EQ(health.Slot(0),
            (hw::CanBusSample{.read_state = CanBusReadState::kOk}));
  EXPECT_EQ(health.Slot(1),
            (hw::CanBusSample{CanBusReadState::kOk, 0.31F, 2, 7, 255, 96, 0}))
      << "the RoboRIO bus's 0 hid the CANivore's load";
  EXPECT_EQ(health.Slot(2), hw::CanBusSample{}) << "no third bus";
  EXPECT_TRUE(loop.changes.empty());
}

// Under WPILib simulation Phoenix simulates every device whatever its bus:
// nothing is read, and every slot the configuration lists reads SIMULATED.
// A configuration with no device at all reads no bus either (no reader
// thread, REQ-CONTRACT-35), and its "rio" slot is not reported.
// REQ-OPS-20, REQ-CONTRACT-35
TEST(CanBusHealth, SimulationAndAnEmptyConfigReadNoBus) {
  CanBusHealth health;
  health.Configure(Devices({"Shooter"}, {"rio"}), /*simulation=*/true);
  EXPECT_EQ(health.bus_count(), 0u);
  EXPECT_EQ(health.board(), nullptr);
  Loop loop{health};
  loop.Poll(kStartUs + 1'000'000);
  for (std::size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(health.Slot(i).read_state, CanBusReadState::kSimulated)
        << "slot " << i;
  }
  EXPECT_EQ(health.Slot(2).read_state, CanBusReadState::kNone);

  health.Configure(hw::Config{}, /*simulation=*/false);
  EXPECT_EQ(health.board(), nullptr);
  loop.Poll(kStartUs + 2'000'000);
  EXPECT_EQ(health.Slot(0).read_state, CanBusReadState::kNone);
  EXPECT_TRUE(loop.changes.empty());
}

// A bus whose status read fails (a CANivore unplugged or renamed, the
// RoboRIO's NetComm call refused) reads FAILED, never an idle bus, keeping
// its last good figures, and counts every failed read since the
// configuration was applied. The loop reports the spell once when it starts
// and once, with its count, when it ends.
// REQ-OPS-20
TEST(CanBusHealth, AFailedReadReadsFailedAndIsCounted) {
  CanBusHealth health;
  health.Configure(Devices({"rio"}, {}), false);
  Loop loop{health};
  uint64_t now = kStartUs;
  const auto read = [&](CanBusHealth::Sample sample) {
    health.SampleNext(now, [&](std::string_view) { return sample; });
    now += kLoopPeriodUs;
    loop.Poll(now);
    now += kPeriodUs - kLoopPeriodUs;
    return health.Slot(0);
  };
  EXPECT_EQ(read({true, 0.7, 1, 2, 3, 4, "OK"}),
            (hw::CanBusSample{CanBusReadState::kOk, 0.7F, 1, 2, 3, 4, 0}));
  EXPECT_TRUE(loop.changes.empty());

  EXPECT_EQ(read(CanBusHealth::FromHal(0.2F, 9, 9, 9, 9, -1)),
            (hw::CanBusSample{CanBusReadState::kFailed, 0.7F, 1, 2, 3, 4, 1}))
      << "a failed read did not keep the last good figures";
  ASSERT_EQ(loop.changes.size(), 1u);
  EXPECT_EQ(loop.changes[0].event, Event::kFailing);
  EXPECT_FALSE(loop.changes[0].stale);

  read({false, 0.0, 0, 0, 0, 0, "CANBusNotFound"});
  // An OK status with a figure that is no fraction is a failed read too.
  EXPECT_EQ(read(Ok(std::numeric_limits<double>::quiet_NaN())).read_state,
            CanBusReadState::kFailed);
  EXPECT_EQ(loop.changes.size(), 1u);

  const auto recovered = read(Ok(0.4));
  EXPECT_EQ(recovered.read_state, CanBusReadState::kOk);
  EXPECT_FLOAT_EQ(recovered.utilization_frac, 0.4F);
  EXPECT_EQ(recovered.read_failure_count, 3u)
      << "failed reads count since the configuration, not since the spell";
  ASSERT_EQ(loop.changes.size(), 2u);
  EXPECT_EQ(loop.changes[1].event, Event::kRecovered);
  EXPECT_EQ(loop.changes[1].spell_failures, 3u);

  health.Configure(Devices({"rio"}, {}), false);
  EXPECT_EQ(health.Slot(0).read_failure_count, 0u)
      << "a new configuration kept the previous one's failures";
}

// Multiple failed reads before Poll() observes the spell (for example after a
// try-lock miss or delayed poll), or a failed read that ages to STALE before
// Poll() runs, must all be counted in spell_failures on both kFailing and
// kRecovered rather than losing earlier failures in the spell.
// REQ-OPS-20
TEST(CanBusHealth, SpellFailuresCountAllFailuresBeforePollAndAcrossStale) {
  CanBusHealth health;
  health.Configure(Devices({"rio"}, {}), false);
  Loop loop{health};

  // Start healthy so the bus has an established OK baseline.
  health.SampleNext(kStartUs, [](std::string_view) { return Ok(0.2); });
  loop.Poll(kStartUs + kLoopPeriodUs);
  ASSERT_EQ(health.Slot(0).read_state, CanBusReadState::kOk);
  ASSERT_TRUE(loop.changes.empty());

  // Two failed reads before Poll() runs, then recovery: both failures count.
  const CanBusHealth::Sample failed{false, 0.0, 0, 0, 0, 0, "CANBusNotFound"};
  health.SampleNext(kStartUs + kPeriodUs,
                    [&](std::string_view) { return failed; });
  health.SampleNext(kStartUs + 2 * kPeriodUs,
                    [&](std::string_view) { return failed; });
  loop.Poll(kStartUs + 2 * kPeriodUs + kLoopPeriodUs);
  ASSERT_EQ(loop.changes.size(), 1u);
  EXPECT_EQ(loop.changes[0].event, Event::kFailing);
  EXPECT_FALSE(loop.changes[0].stale);
  EXPECT_EQ(loop.changes[0].spell_failures, 2u);

  health.SampleNext(kStartUs + 3 * kPeriodUs,
                    [](std::string_view) { return Ok(0.3); });
  loop.Poll(kStartUs + 3 * kPeriodUs + kLoopPeriodUs);
  ASSERT_EQ(loop.changes.size(), 2u);
  EXPECT_EQ(loop.changes[1].event, Event::kRecovered);
  EXPECT_EQ(loop.changes[1].spell_failures, 2u);

  // A failed read that ages to STALE before Poll() runs still counts its
  // failure in the new spell (and does not recount the earlier spell's two).
  const uint64_t fail_time = kStartUs + 4 * kPeriodUs;
  health.SampleNext(fail_time, [&](std::string_view) { return failed; });
  loop.Poll(fail_time + health.StaleUs());
  EXPECT_EQ(health.Slot(0).read_state, CanBusReadState::kStale);
  ASSERT_EQ(loop.changes.size(), 3u);
  EXPECT_EQ(loop.changes[2].event, Event::kFailing);
  EXPECT_TRUE(loop.changes[2].stale);
  EXPECT_EQ(loop.changes[2].spell_failures, 1u);

  const uint64_t recover_time = fail_time + health.StaleUs() + kPeriodUs;
  health.SampleNext(recover_time, [](std::string_view) { return Ok(0.4); });
  loop.Poll(recover_time + kLoopPeriodUs);
  ASSERT_EQ(loop.changes.size(), 4u);
  EXPECT_EQ(loop.changes[3].event, Event::kRecovered);
  EXPECT_EQ(loop.changes[3].spell_failures, 1u);
  EXPECT_EQ(health.Slot(0).read_failure_count, 3u);
}

// A vendor call that never returns, or a sampler starved of the CPU, leaves
// the last read published. It reads current only until it is three of its
// bus's own sampling periods old on the loop's clock (two buses: 600 ms),
// then the slot reads STALE; a bus never read reads PENDING until the same
// time after the loop's first tick, then STALE. A read stamped after the
// loop's clock reading (published between the two) is current.
// REQ-OPS-20
TEST(CanBusHealth, AReadNotRefreshedWithinThreeOfItsPeriodsReadsStale) {
  CanBusHealth health;
  health.Configure(Devices({"Shooter"}, {}), false);
  ASSERT_EQ(health.StaleUs(), 600'000u);
  Buses buses;
  buses.samples["rio"] = Ok(0.3);
  buses.samples["Shooter"] = Ok(0.6);
  Loop loop{health};
  // The loop's first tick comes before any read; Shooter's first is late.
  loop.Poll(kStartUs);
  EXPECT_EQ(health.Slot(0).read_state, CanBusReadState::kPending);
  health.SampleNext(kStartUs + 1'000, buses);  // rio
  loop.Poll(kStartUs);
  EXPECT_EQ(health.Slot(0).read_state, CanBusReadState::kOk);
  loop.Poll(kStartUs + 600'000 - 1);
  EXPECT_EQ(health.Slot(1).read_state, CanBusReadState::kPending);
  EXPECT_TRUE(loop.changes.empty());
  loop.Poll(kStartUs + 600'000);
  EXPECT_EQ(health.Slot(1).read_state, CanBusReadState::kStale);
  ASSERT_EQ(loop.changes.size(), 1u) << "Shooter, never read, did not fail";
  EXPECT_EQ(loop.changes[0].bus, 1u);
  EXPECT_TRUE(loop.changes[0].stale);
  EXPECT_EQ(loop.changes[0].status, nullptr);

  // The sampler hangs in its next call: rio's read ages out, its figure kept.
  loop.Poll(kStartUs + 601'000 - 1);
  EXPECT_EQ(health.Slot(0).read_state, CanBusReadState::kOk);
  loop.Poll(kStartUs + 601'000);
  EXPECT_EQ(health.Slot(0),
            (hw::CanBusSample{.read_state = CanBusReadState::kStale,
                              .utilization_frac = 0.3F}))
      << "a read older than the stale bound still read current";
  ASSERT_EQ(loop.changes.size(), 2u);
  EXPECT_EQ(loop.changes[1].bus, 0u);
  EXPECT_TRUE(loop.changes[1].stale);
  EXPECT_STREQ(loop.changes[1].status, "OK");

  // It returns: Shooter, then rio, each back with no failed read counted.
  health.SampleNext(kStartUs + 700'000, buses);
  health.SampleNext(kStartUs + 800'000, buses);
  loop.Poll(kStartUs + 800'000);
  EXPECT_EQ(health.Slot(1).read_state, CanBusReadState::kOk);
  ASSERT_EQ(loop.changes.size(), 4u);
  EXPECT_EQ(loop.changes[2].event, Event::kRecovered);
  EXPECT_EQ(loop.changes[3].event, Event::kRecovered);
  EXPECT_EQ(loop.changes[2].spell_failures, 0u);
}

// A configuration can be applied while the sampler is inside a read of a bus
// the new configuration drops (the vendor call cannot be interrupted). That
// read keeps a name of its own to the end, and publishes nothing: the new
// set's slots come only from its own reads.
// REQ-OPS-20
TEST(CanBusHealth, AReadInFlightAcrossAConfigurationPublishesNothing) {
  CanBusHealth health;
  health.Configure(Devices({}, {"Shooter"}), false);
  std::vector<std::string> reads;
  health.SampleNext(kStartUs, [&](std::string_view bus) {
    health.Configure(Devices({}, {"Arm"}), false);
    reads.emplace_back(bus);
    return Ok(0.9);
  });
  EXPECT_EQ(reads, std::vector<std::string>{"rio"});
  Loop loop{health};
  loop.Poll(kStartUs + kLoopPeriodUs);
  EXPECT_EQ(health.Slot(0).read_state, CanBusReadState::kPending)
      << "the old configuration's read landed in the new set";
  EXPECT_TRUE(loop.changes.empty());
  Buses buses;
  buses.samples["rio"] = Ok(0.2);
  health.SampleNext(kStartUs + kPeriodUs, buses);
  EXPECT_EQ(buses.reads, std::vector<std::string>{"rio"});
  loop.Poll(kStartUs + kPeriodUs);
  EXPECT_FLOAT_EQ(health.Slot(0).utilization_frac, 0.2F);
}

// The reader threads' reads, for a test to steer and wait on. A bus's reads
// return at once while it has `free_reads` left, then block until the test
// releases that bus: a vendor call that does not return, until it does.
// Shared (std::shared_ptr) with the read function, so a reader the sampler
// abandoned keeps it alive. Every wait is on a read, never on time; the 10 s
// bound only fails a test that would otherwise hang (TESTING.md §6.3).
struct Gate {
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<std::string> reads;
  std::vector<std::string> blocked;
  std::map<std::string, int, std::less<>> free_reads;
  std::map<std::string, double, std::less<>> frac;
  std::vector<std::string> released;

  CanBusHealth::Sample Read(std::string_view bus) {
    std::unique_lock lock{mutex};
    reads.emplace_back(bus);
    const auto free = free_reads.find(bus);
    if (free != free_reads.end() && free->second > 0) {
      --free->second;
    } else {
      blocked.emplace_back(bus);
      changed.notify_all();
      changed.wait(lock, [&] { return Count(released, bus) > 0; });
      blocked.erase(std::find(blocked.begin(), blocked.end(), bus));
    }
    changed.notify_all();
    const auto it = frac.find(bus);
    return Ok(it == frac.end() ? 0.0 : it->second);
  }

  static std::ptrdiff_t Count(const std::vector<std::string>& in,
                              std::string_view bus) {
    return std::count(in.begin(), in.end(), bus);
  }
  template <typename Predicate>
  bool WaitUntil(Predicate predicate) {
    std::unique_lock lock{mutex};
    return changed.wait_for(lock, std::chrono::seconds{10}, predicate);
  }
  bool WaitForReads(std::string_view bus, std::ptrdiff_t count) {
    return WaitUntil([&] { return Count(reads, bus) >= count; });
  }
  bool WaitBlocked(std::string_view bus) {
    return WaitUntil([&] { return Count(blocked, bus) > 0; });
  }
  void Release(std::string_view bus) {
    {
      const std::lock_guard lock{mutex};
      released.emplace_back(bus);
    }
    changed.notify_all();
  }
};

CanBusSampler::Read ReadThrough(const std::shared_ptr<Gate>& gate) {
  return [gate](std::string_view bus) { return gate->Read(bus); };
}

constexpr auto kNeverAbandon = std::chrono::seconds{10};
constexpr auto kAbandonAtOnce = std::chrono::microseconds{0};

// The thread runs for every configuration with a device, the RoboRIO's own
// bus being read even with every device on it, but not for one with no
// device (REQ-CONTRACT-35) and never under simulation; a new configuration
// stops it before applying: once Reconfigure returns, no read of a bus it
// dropped starts. (A stop bound that never abandons, so a slow host cannot
// turn this into an abandonment.)
// REQ-OPS-20, REQ-CONTRACT-35
TEST(CanBusSampler, RunsForEveryConfigWithADeviceAndStopsReadingDroppedBuses) {
  CanBusHealth health;
  auto gate = std::make_shared<Gate>();
  gate->free_reads["rio"] = 1'000'000;
  gate->free_reads["Shooter"] = 1'000'000;
  gate->free_reads["Arm"] = 1'000'000;
  gate->frac["Arm"] = 0.2;
  CanBusSampler sampler{health,  ReadThrough(gate),   +[] { return kStartUs; },
                        nullptr, /*period_us=*/1'000, kNeverAbandon};

  sampler.Reconfigure(hw::Config{}, false);
  EXPECT_FALSE(sampler.running()) << "an empty configuration started one";
  sampler.Reconfigure(Devices({"Shooter"}, {}), /*simulation=*/true);
  EXPECT_FALSE(sampler.running()) << "simulation started one";
  sampler.Reconfigure(Devices({"rio"}, {"rio"}), false);
  EXPECT_TRUE(sampler.running());
  ASSERT_TRUE(gate->WaitForReads("rio", 2));

  sampler.Reconfigure(Devices({"Shooter"}, {}), false);
  ASSERT_TRUE(gate->WaitForReads("Shooter", 2));
  sampler.Reconfigure(Devices({"Arm"}, {}), false);
  ASSERT_TRUE(gate->WaitForReads("Arm", 2));
  sampler.Stop();
  EXPECT_FALSE(sampler.running());

  // Shooter is never read after the first read of Arm.
  const std::lock_guard lock{gate->mutex};
  const auto& reads = gate->reads;
  const auto first_arm = std::find(reads.begin(), reads.end(), "Arm");
  EXPECT_EQ(std::count(first_arm, reads.end(), "Shooter"), 0)
      << "a bus the new configuration dropped was read after it";
  Loop loop{health};
  loop.Poll(kStartUs);
  EXPECT_FLOAT_EQ(health.Slot(1).utilization_frac, 0.2F);
}

// The loop's copy never waits on a read in flight: with the reader blocked
// in one bus's vendor call, a Poll returns at once with what the other bus's
// finished read published.
// REQ-OPS-20
TEST(CanBusSampler, TheLoopCopiesWithoutWaitingOnAReadInFlight) {
  CanBusHealth health;
  auto gate = std::make_shared<Gate>();
  gate->free_reads["rio"] = 1;
  gate->frac["rio"] = 0.05;
  auto stuck = std::make_shared<CanBusSampler::StuckReaders>();
  {
    CanBusSampler sampler{
        health,  ReadThrough(gate),   +[] { return kStartUs; },
        nullptr, /*period_us=*/1'000, kAbandonAtOnce,
        stuck};
    sampler.Reconfigure(Devices({"Shooter"}, {}), false);
    ASSERT_TRUE(gate->WaitBlocked("Shooter"));
    Loop loop{health};
    loop.Poll(kStartUs);
    EXPECT_EQ(health.Slot(0),
              (hw::CanBusSample{.read_state = CanBusReadState::kOk,
                                .utilization_frac = 0.05F}));
    EXPECT_EQ(health.Slot(1).read_state, CanBusReadState::kPending);
  }
  gate->Release("Shooter");
  EXPECT_TRUE(stuck->WaitUntilNone(std::chrono::seconds{10}));
}

// A status read that never returns must block neither a new configuration
// nor the backend's destruction (both run Stop, the first outside the device
// lock the loop watchdog's neutral needs). Past the stop bound (here 0, so
// the test waits on nothing) the reader is abandoned with one console line
// naming the bus; the new configuration's reader reads the new buses; the
// abandoned read's late result lands in a Board nobody reads; and the
// reader, once its read returns, exits on its own and is counted out.
// REQ-OPS-20
TEST(CanBusSampler, AReadThatNeverReturnsIsAbandonedNotWaitedFor) {
  CanBusHealth health;
  auto gate = std::make_shared<Gate>();
  gate->free_reads["rio"] = 1'000'000;
  gate->frac["Shooter"] = 0.9;
  gate->frac["Arm"] = 0.2;
  gate->free_reads["Arm"] = 1;
  auto stuck = std::make_shared<CanBusSampler::StuckReaders>();
  std::vector<std::string> lines;
  {
    CanBusSampler sampler{
        health,
        ReadThrough(gate),
        +[] { return kStartUs; },
        [&lines](std::string_view line) { lines.emplace_back(line); },
        /*period_us=*/1'000,
        kAbandonAtOnce,
        stuck};
    sampler.Reconfigure(Devices({"Shooter"}, {}), false);
    ASSERT_TRUE(gate->WaitBlocked("Shooter"));

    sampler.Reconfigure(Devices({"Arm"}, {}), false);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_NE(lines[0].find("Shooter"), std::string::npos) << lines[0];
    EXPECT_EQ(stuck->count(), 1);
    EXPECT_TRUE(sampler.running());
    // Arm's first read returns, its second blocks: the first is published.
    ASSERT_TRUE(gate->WaitBlocked("Arm"));
    Loop loop{health};
    loop.Poll(kStartUs);
    EXPECT_FLOAT_EQ(health.Slot(1).utilization_frac, 0.2F);

    gate->Release("Shooter");
    ASSERT_TRUE(stuck->WaitUntilNone(std::chrono::seconds{10}));
    loop.Poll(kStartUs);
    EXPECT_FLOAT_EQ(health.Slot(1).utilization_frac, 0.2F)
        << "the abandoned read's 0.9 reached the new configuration";
    EXPECT_TRUE(loop.changes.empty());
    // Destroyed with Arm's read blocked: returns, abandoning it too.
  }
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_NE(lines[1].find("Arm"), std::string::npos) << lines[1];
  EXPECT_EQ(stuck->count(), 1);
  // Let it finish, so no reader is still running when the test ends.
  gate->Release("Arm");
  EXPECT_TRUE(stuck->WaitUntilNone(std::chrono::seconds{10}));
}

// A bus whose every read hangs must not cost a thread per new configuration:
// with two abandoned readers still blocked, Reconfigure starts none, says so
// once, and that configuration's buses go stale. Once those reads return,
// the next configuration starts a reader again.
// REQ-OPS-20
TEST(CanBusSampler, StartsNoReaderWhileTwoAbandonedOnesAreStillBlocked) {
  CanBusHealth health;
  auto gate = std::make_shared<Gate>();
  auto stuck = std::make_shared<CanBusSampler::StuckReaders>();
  std::vector<std::string> lines;
  {
    CanBusSampler sampler{
        health,
        ReadThrough(gate),
        +[] { return kStartUs; },
        [&lines](std::string_view line) { lines.emplace_back(line); },
        /*period_us=*/1'000,
        kAbandonAtOnce,
        stuck};
    // Every read of rio blocks: each configuration's reader is stuck in it.
    sampler.Reconfigure(Devices({"A"}, {}), false);
    ASSERT_TRUE(gate->WaitUntil([&] { return gate->blocked.size() == 1; }));
    sampler.Reconfigure(Devices({"B"}, {}), false);
    ASSERT_TRUE(gate->WaitUntil([&] { return gate->blocked.size() == 2; }));
    sampler.Reconfigure(Devices({"C"}, {}), false);
    EXPECT_EQ(stuck->count(), 2);
    EXPECT_FALSE(sampler.running()) << "a third reader started";
    ASSERT_EQ(lines.size(), 3u) << "two abandoned, then none started";
    Loop loop{health};
    loop.Poll(kStartUs);
    loop.Poll(kStartUs + health.StaleUs());
    ASSERT_EQ(loop.changes.size(), 2u);
    EXPECT_TRUE(loop.changes[0].stale);
    EXPECT_EQ(health.Slot(1).read_state, CanBusReadState::kStale);

    gate->Release("rio");
    ASSERT_TRUE(stuck->WaitUntilNone(std::chrono::seconds{10}));
    gate->free_reads["rio"] = 1;
    sampler.Reconfigure(Devices({"D"}, {}), false);
    EXPECT_TRUE(sampler.running());
    ASSERT_TRUE(gate->WaitBlocked("D"));
  }
  gate->Release("D");
  EXPECT_TRUE(stuck->WaitUntilNone(std::chrono::seconds{10}));
  {
    const std::lock_guard lock{gate->mutex};
    EXPECT_EQ(Gate::Count(gate->reads, "C"), 0);
  }
}

}  // namespace

// Callback entry-to-exit wall-clock p99 benchmark for representative C++ bench
// and framework nodes (REQ-RT-07; docs/METHODOLOGY.md §10; docs/TESTING.md
// §6.7, §6.8).
//
// Each node runs in its full bench configuration under testing::Sim +
// SimHardware in active driving steady state (not neutral idle), with 100
// warmup ticks followed by 1,000 measured ticks timed with
// talos::event::steady_now() around update() + after_update() (the devices
// flush). A StallMeter thread checks host scheduling noise before judging.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "motor/motor.h"
#include "swerve_module/swerve_module.h"
#include "talOS/devices/testing.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/time.h"
#include "talOS/node/node.h"
#include "talOS/node/testing.h"
#include "talOS/process/host_qualification.h"
#include "talOS/telemetry/testing.h"
#include "talOS/teleop/testing.h"

namespace {

using namespace std::chrono_literals;
namespace ds = talos::driver_station;
using talos::hardware::Mode;
using talos::process::ComputePercentiles;
using talos::process::DurationPercentiles;
using talos::process::StallMeter;
using talos::testing::DsPacket;
using talos::testing::Sim;
using talos::testing::SimHardware;
using talos::testing::TimedNode;

constexpr const char* kConfig = "bench/main_processor/configuration/robot.toml";

// REQ-RT-07, docs/METHODOLOGY.md §10: update() p99 wall time <= 10% of
// period_us (500 us for a 5 ms control loop, 1000 us for 10 ms telemetry).
constexpr double kUpdateBudgetFraction = 0.10;
constexpr std::size_t kWarmupTicks = 100;
constexpr std::size_t kMeasuredTicks = 1000;

// Host-qualification ceiling matching talOS/bridge/run_timing_test.cc's 25 ms
// stall ceiling (docs/TESTING.md §6.7): an ordinary thread's 1 ms sleep
// overrunning by 25 ms or more means the host slept or was severely starved.
constexpr std::uint64_t kMaxHostStallUs = 25'000;

std::string FormatUs(double us) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(2) << us;
  return out.str();
}

struct BenchGraph {
  // Declared before `sim` so they outlive the hosted nodes' callbacks.
  std::vector<std::int64_t> motor_samples_ns;
  std::vector<std::int64_t> swerve_samples_ns;
  std::vector<std::int64_t> teleop_samples_ns;
  std::vector<std::int64_t> telemetry_samples_ns;
  bool recording_motor = false;
  bool recording_swerve = false;
  bool recording_teleop = false;
  bool recording_telemetry = false;

  Sim sim{kConfig};
  TimedNode<motor::BenchMotor>& motor =
      sim.add<TimedNode<motor::BenchMotor>>("motor");
  TimedNode<swerve_module::BenchSwerveModule>& swerve =
      sim.add<TimedNode<swerve_module::BenchSwerveModule>>("swerve_module");
  SimHardware hw{sim};
  ds::DriverStationData packet = DsPacket(ds::kEnabled | ds::kTeleop);

  BenchGraph() {
    motor.bind(&motor_samples_ns, &recording_motor);
    swerve.bind(&swerve_samples_ns, &recording_swerve);
    talos::teleop::AddTimedTeleop(sim, teleop_samples_ns, recording_teleop);
    talos::telemetry::AddTimedTelemetry(sim, telemetry_samples_ns,
                                        recording_telemetry);

    hw.enable();
    hw.set_motor("motor", "kraken", 1.0, 5.0);
    hw.set_motor("swerve_module", "drive", 2.0, 3.0);
    hw.set_motor("swerve_module", "steer", 0.1, 0.0);
    hw.set_cancoder("swerve_module", "encoder", 0.1, 0.0);

    auto& stick = packet.joysticks[0];
    stick.connected = true;
    stick.axis_count = 6;
    stick.button_count = 6;

    // Neutral evaluation arms teleop's OperatorArming before active drive.
    hw.set_driver_station(packet);
    sim.step(20ms);

    // Engage active drive for both bench actuator nodes:
    // - motor: axis 1 -> VOLTAGE mode
    // - swerve_module: button 6 (bit 5) + axes 4/5 -> DRIVE mode
    stick.axes[1] = -0.6f;
    stick.axes[5] = -0.6f;
    stick.axes[4] = 0.4f;
    stick.buttons = (1u << 5);
    hw.set_driver_station(packet);
    sim.step(50ms);
  }

  void VaryInputs(std::size_t tick_index) {
    const double phase = static_cast<double>(tick_index % 64) / 64.0;
    const double wave = std::sin(phase * 6.283185307179586);
    hw.set_motor("motor", "kraken",
                 1.0 + 0.05 * static_cast<double>(tick_index),
                 5.0 + 0.5 * wave);
    hw.set_motor("swerve_module", "drive",
                 2.0 + 0.05 * static_cast<double>(tick_index),
                 3.0 + 0.4 * wave);
    const double steer_pos = 0.1 + 0.25 * wave;
    hw.set_motor("swerve_module", "steer", steer_pos, 0.2 * wave);
    hw.set_cancoder("swerve_module", "encoder", steer_pos, 0.2 * wave);

    auto& stick = packet.joysticks[0];
    stick.axes[1] = static_cast<float>(-0.5 - 0.3 * wave);
    stick.axes[5] = static_cast<float>(-0.5 - 0.3 * wave);
    stick.axes[4] = static_cast<float>(0.5 * wave);
    hw.set_driver_station(packet);
  }
};

void BenchmarkNode(std::string_view node_name, BenchGraph& graph,
                   std::vector<std::int64_t>& samples_ns, bool& recording) {
  // Confirm every node is genuinely driving / publishing before timing.
  ASSERT_EQ(graph.hw.applied_motor("motor", "kraken").mode, Mode::kVoltage);
  ASSERT_EQ(graph.hw.applied_motor("swerve_module", "drive").mode,
            Mode::kVelocity);
  ASSERT_EQ(graph.hw.applied_motor("swerve_module", "steer").mode,
            Mode::kPosition);
  ASSERT_TRUE(graph.sim.latest_bytes("/talos/telemetry").has_value());

  const talos::Duration period = graph.sim.node(node_name).period();
  ASSERT_GT(period.count(), 0);
  const double period_us = static_cast<double>(period.count()) / 1000.0;
  const double budget_us = kUpdateBudgetFraction * period_us;

  // Warmup: settle caches and telemetry keyframe/dictionary state.
  for (std::size_t i = 0; i < kWarmupTicks; ++i) {
    graph.VaryInputs(i);
    graph.sim.step(period);
  }

  samples_ns.clear();
  samples_ns.reserve(kMeasuredTicks + 16);

  StallMeter host;
  recording = true;
  for (std::size_t i = 0; i < kMeasuredTicks; ++i) {
    graph.VaryInputs(kWarmupTicks + i);
    graph.sim.step(period);
  }
  recording = false;
  const std::uint64_t stall_us = host.Stop();

  ASSERT_GE(samples_ns.size(), kMeasuredTicks)
      << "node '" << node_name << "' recorded fewer than " << kMeasuredTicks
      << " ticks";

  const DurationPercentiles stats = ComputePercentiles(samples_ns);

  ::testing::Test::RecordProperty("node", std::string{node_name});
  ::testing::Test::RecordProperty("samples",
                                  static_cast<int>(samples_ns.size()));
  ::testing::Test::RecordProperty("period_us", FormatUs(period_us));
  ::testing::Test::RecordProperty("budget_us", FormatUs(budget_us));
  ::testing::Test::RecordProperty("p50_us", FormatUs(stats.p50_us));
  ::testing::Test::RecordProperty("p95_us", FormatUs(stats.p95_us));
  ::testing::Test::RecordProperty("p99_us", FormatUs(stats.p99_us));
  ::testing::Test::RecordProperty("max_us", FormatUs(stats.max_us));
  ::testing::Test::RecordProperty("host_stall_us", std::to_string(stall_us));

  std::printf(
      "[REQ-RT-07] %.*s (period=%.0f us, budget=%.0f us, n=%zu): "
      "p50=%.2f us, p95=%.2f us, p99=%.2f us, max=%.2f us, "
      "host_stall=%llu us\n",
      static_cast<int>(node_name.size()), node_name.data(), period_us,
      budget_us, samples_ns.size(), stats.p50_us, stats.p95_us, stats.p99_us,
      stats.max_us, static_cast<unsigned long long>(stall_us));

  // Host qualification (docs/TESTING.md §6.7): fail distinctly as
  // [HOST_UNQUALIFIED] if the host stalled severely (>= 25 ms) or if a host
  // scheduling stall at least as large as a failing p99 contaminated the run.
  if (stall_us >= kMaxHostStallUs ||
      (stats.p99_us > budget_us &&
       static_cast<double>(stall_us) >= stats.p99_us)) {
    ::testing::Test::RecordProperty("host_unqualified", "true");
    FAIL() << "[HOST_UNQUALIFIED] host unqualified while benchmarking '"
           << node_name << "': 1 ms sleep overran by " << stall_us
           << " us (limit " << kMaxHostStallUs
           << " us; measured p99=" << FormatUs(stats.p99_us)
           << " us, budget=" << FormatUs(budget_us) << " us)";
  }

  EXPECT_LE(stats.p99_us, budget_us)
      << "REQ-RT-07 / METHODOLOGY.md §10: node '" << node_name
      << "' update() p99=" << FormatUs(stats.p99_us)
      << " us exceeds 10% of period_us=" << FormatUs(period_us)
      << " us (budget=" << FormatUs(budget_us)
      << " us; p50=" << FormatUs(stats.p50_us)
      << " us, p95=" << FormatUs(stats.p95_us)
      << " us, max=" << FormatUs(stats.max_us) << " us, host_stall=" << stall_us
      << " us)";
}

// REQ-RT-07
TEST(UpdateP99PerfTest, MotorUpdateP99WithinBudget) {
  BenchGraph graph;
  BenchmarkNode("motor", graph, graph.motor_samples_ns, graph.recording_motor);
}

// REQ-RT-07
TEST(UpdateP99PerfTest, SwerveModuleUpdateP99WithinBudget) {
  BenchGraph graph;
  BenchmarkNode("swerve_module", graph, graph.swerve_samples_ns,
                graph.recording_swerve);
}

// REQ-RT-07
TEST(UpdateP99PerfTest, TeleopUpdateP99WithinBudget) {
  BenchGraph graph;
  BenchmarkNode("teleop", graph, graph.teleop_samples_ns,
                graph.recording_teleop);
}

// REQ-RT-07
TEST(UpdateP99PerfTest, TelemetryUpdateP99WithinBudget) {
  BenchGraph graph;
  BenchmarkNode("telemetry", graph, graph.telemetry_samples_ns,
                graph.recording_telemetry);
}

}  // namespace

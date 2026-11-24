// Callback entry-to-exit wall-clock p99 benchmark for representative C++ bench,
// framework, and mechanism nodes (REQ-RT-07; docs/METHODOLOGY.md §10;
// docs/TESTING.md §6.7, §6.8).
//
// Each node runs under testing::Sim + SimHardware in active driving steady
// state (not neutral idle), with 100 warmup ticks followed by 1,000 measured
// ticks timed with talos::event::steady_now() around update() + after_update()
// (the devices flush). A StallMeter thread checks host scheduling noise before
// judging.

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

#include "control/control_generated.h"
#include "motor/motor.h"
#include "swerve_module/swerve_module.h"
#include "talOS/devices/devices.h"
#include "talOS/devices/testing.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/time.h"
#include "talOS/mechanisms/position_mechanism.h"
#include "talOS/mechanisms/roller_with_sensor.h"
#include "talOS/mechanisms/velocity_mechanism.h"
#include "talOS/node/node.h"
#include "talOS/node/testing.h"
#include "talOS/process/host_qualification.h"
#include "talOS/telemetry/testing.h"
#include "talOS/teleop/testing.h"

namespace {

using namespace std::chrono_literals;
namespace ds = talos::driver_station;
using talos::hardware::Mode;
using talos::mechanisms::PositionMechanism;
using talos::mechanisms::RollerCommand;
using talos::mechanisms::RollerPhase;
using talos::mechanisms::RollerWithSensor;
using talos::mechanisms::VelocityMechanism;
using talos::process::ComputePercentiles;
using talos::process::DurationPercentiles;
using talos::process::StallMeter;
using talos::testing::DsPacket;
using talos::testing::PublishTarget;
using talos::testing::Sim;
using talos::testing::SimHardware;
using talos::testing::TimedNode;

constexpr const char* kConfig = "bench/main_processor/configuration/robot.toml";

// Inline 5 ms robot config hosting a representative composite mechanism node
// combining PositionMechanism (arm with remote CANcoder, wrap, arm gravity,
// stall/timeout guards), VelocityMechanism (flywheel with spin-down and
// readiness), and RollerWithSensor (intake roller with debounced beam break).
constexpr const char* kMechanismsToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.mechanisms.motors.arm]
can_id = 1
feedback = "RemoteCANcoder"
feedback_sensor = "arm_cc"
position_frame = "absolute"
[subsystems.mechanisms.sensors.arm_cc]
type = "CANcoder"
can_id = 2
[subsystems.mechanisms.arm]
tolerance_rot = 0.03
at_goal_debounce_us = 20000
wrap_rot = 1.0
min_rot = -0.5
max_rot = 0.5
gravity = "arm"
gravity_v = 0.8
stall_time_us = 200000
stall_velocity_rps = 0.05
stall_current_a = 20.0
move_timeout_us = 1000000

[subsystems.mechanisms.motors.flywheel]
can_id = 3
max_velocity_rps = 80.0
[subsystems.mechanisms.flywheel]
at_speed_entry_rps = 2.0
at_speed_exit_frac = 0.1
at_speed_entry_tick_count = 3
spin_down_rps2 = 200.0

[subsystems.mechanisms.motors.roller]
can_id = 4
[subsystems.mechanisms.digital_inputs.beam]
dio = 0
[subsystems.mechanisms.roller]
sensor = "beam"
sensor_debounce_us = 15000
idle_runs = true
intake_v = 6.0
hold_v = 1.0
eject_v = -8.0
eject_us = 200000
feed_v = 10.0
feed_us = 100000
)";

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

class MechanismProbeNode {
 public:
  explicit MechanismProbeNode(talos::Node& node)
      : arm_{node, "arm"},
        flywheel_{node, "flywheel"},
        roller_{node, "roller"},
        target_{node},
        arm_state_{node.publish<control::PositionState>("state/arm")},
        flywheel_state_{
            node.publish<control::VelocityState>("state/flywheel")} {}

  void update() {
    if (const control::PositionTarget* t = target_.get()) {
      arm_.set_goal(t->position_rot(), t->feedforward_v());
      const double flywheel_goal_rps = t->feedforward_v() >= 0.0 ? 65.0 : 45.0;
      flywheel_.set_goal(flywheel_goal_rps, t->feedforward_v());
      if (roller_.phase() == RollerPhase::kHolding &&
          t->feedforward_v() < 0.0) {
        roller_.set_goal(RollerCommand::kFeed);
      } else {
        roller_.set_goal(RollerCommand::kIntake);
      }
    } else {
      arm_.stop();
      flywheel_.stop();
      roller_.stop();
    }
    arm_.update();
    flywheel_.update();
    roller_.update();
    arm_state_.send(arm_.state());
    flywheel_state_.send(flywheel_.state());
  }

 private:
  PositionMechanism arm_;
  VelocityMechanism flywheel_;
  RollerWithSensor roller_;
  talos::Target<control::PositionTarget> target_;
  talos::Publisher<control::PositionState> arm_state_;
  talos::Publisher<control::VelocityState> flywheel_state_;
};

struct MechanismsGraph {
  // Declared before `sim` so they outlive the hosted node's callback.
  std::vector<std::int64_t> samples_ns;
  bool recording = false;

  Sim sim = Sim::FromToml(kMechanismsToml);
  TimedNode<MechanismProbeNode>& mechanisms =
      sim.add<TimedNode<MechanismProbeNode>>("mechanisms");
  SimHardware hw{sim};

  MechanismsGraph() {
    mechanisms.bind(&samples_ns, &recording);
    hw.enable();
    hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
    hw.set_motor("mechanisms", "arm", 0.05, 0.5, 0, 0, 30.0, 2.0, 2.0);
    hw.set_cancoder("mechanisms", "arm_cc", 0.05, 0.5);
    hw.set_motor("mechanisms", "flywheel", 0.0, 65.0, 0, 0, 35.0, 12.0, 15.0);
    hw.set_motor("mechanisms", "roller", 0.0, 8.0);
    hw.set_digital_input("mechanisms", "beam", false);
    sim.step(10ms);

    PublishTarget(sim, "mechanisms", "teleop",
                  control::PositionTarget{0.1, 0.1});
    sim.step(20ms);
  }

  void VaryInputs(std::size_t tick_index) {
    const double phase = static_cast<double>(tick_index % 64) / 64.0;
    const double angle = phase * 6.283185307179586;
    const double wave = std::sin(angle);
    const double dwave = std::cos(angle);

    const double arm_goal = 0.2 * wave;
    const double arm_pos = arm_goal - 0.035 * wave;
    const double arm_vel = 0.2 * (6.283185307179586 / 0.32) * dwave;
    hw.set_motor("mechanisms", "arm", arm_pos, arm_vel, 0, 0, 30.0, 2.0, 2.0);
    hw.set_cancoder("mechanisms", "arm_cc", arm_pos, arm_vel);

    const bool second_half = ((tick_index / 32) % 2) == 1;
    const double flywheel_goal = second_half ? 45.0 : 65.0;
    const double flywheel_vel = flywheel_goal - 0.5 * wave;
    hw.set_motor("mechanisms", "flywheel",
                 0.25 * static_cast<double>(tick_index), flywheel_vel, 0, 0,
                 35.0, 12.0, 15.0);

    hw.set_motor("mechanisms", "roller", 0.1 * static_cast<double>(tick_index),
                 8.0);
    hw.set_digital_input("mechanisms", "beam", second_half);

    PublishTarget(sim, "mechanisms", "teleop",
                  control::PositionTarget{arm_goal, second_half ? -0.1 : 0.1});
  }
};

template <class StepFn>
void RunP99Benchmark(std::string_view node_name, talos::Duration period,
                     std::vector<std::int64_t>& samples_ns, bool& recording,
                     StepFn&& step_tick) {
  ASSERT_GT(period.count(), 0);
  const double period_us = static_cast<double>(period.count()) / 1000.0;
  const double budget_us = kUpdateBudgetFraction * period_us;

  // Warmup: settle caches and telemetry keyframe/dictionary state.
  for (std::size_t i = 0; i < kWarmupTicks; ++i) {
    step_tick(i);
  }

  samples_ns.clear();
  samples_ns.reserve(kMeasuredTicks + 16);

  StallMeter host;
  recording = true;
  for (std::size_t i = 0; i < kMeasuredTicks; ++i) {
    step_tick(kWarmupTicks + i);
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
  RunP99Benchmark(node_name, period, samples_ns, recording,
                  [&](std::size_t tick_index) {
                    graph.VaryInputs(tick_index);
                    graph.sim.step(period);
                  });
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

// REQ-RT-07
TEST(UpdateP99PerfTest, MechanismsUpdateP99WithinBudget) {
  MechanismsGraph graph;
  ASSERT_EQ(graph.hw.applied_motor("mechanisms", "arm").mode,
            Mode::kMotionMagic);
  ASSERT_EQ(graph.hw.applied_motor("mechanisms", "flywheel").mode,
            Mode::kVelocity);
  ASSERT_EQ(graph.hw.applied_motor("mechanisms", "roller").mode,
            Mode::kVoltage);
  const auto initial_arm =
      graph.sim.latest<control::PositionState>("/mechanisms/state/arm");
  ASSERT_TRUE(initial_arm.has_value());
  ASSERT_TRUE(initial_arm->enabled());
  ASSERT_TRUE(initial_arm->connected());
  const auto initial_flywheel =
      graph.sim.latest<control::VelocityState>("/mechanisms/state/flywheel");
  ASSERT_TRUE(initial_flywheel.has_value());
  ASSERT_TRUE(initial_flywheel->enabled());
  ASSERT_TRUE(initial_flywheel->connected());

  const talos::Duration period = graph.sim.node("mechanisms").period();
  RunP99Benchmark("mechanisms", period, graph.samples_ns, graph.recording,
                  [&](std::size_t tick_index) {
                    graph.VaryInputs(tick_index);
                    graph.sim.step(period);
                  });

  // Confirm all three mechanisms stayed actively driving through the full run
  // (no latched PositionMechanism stall/timeout fault or silent neutral).
  EXPECT_EQ(graph.hw.applied_motor("mechanisms", "arm").mode,
            Mode::kMotionMagic);
  EXPECT_EQ(graph.hw.applied_motor("mechanisms", "flywheel").mode,
            Mode::kVelocity);
  EXPECT_EQ(graph.hw.applied_motor("mechanisms", "roller").mode,
            Mode::kVoltage);
  const auto final_arm =
      graph.sim.latest<control::PositionState>("/mechanisms/state/arm");
  ASSERT_TRUE(final_arm.has_value());
  EXPECT_TRUE(final_arm->enabled());
  const auto final_flywheel =
      graph.sim.latest<control::VelocityState>("/mechanisms/state/flywheel");
  ASSERT_TRUE(final_flywheel.has_value());
  EXPECT_TRUE(final_flywheel->enabled());
}

}  // namespace

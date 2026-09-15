#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <thread>

#include "node.h"

// Performance evidence for bridge items 42/43/47: command-path histograms,
// change-driven publication with bounded heartbeat, and a realistic-load
// benchmark. Subsystem names are perf_* so these topics never collide with
// the node_test binary if bazel runs both at once; UDP ports likewise sit
// above node_test's range.

namespace talos::hardware {
namespace {

constexpr std::string_view kPerfToml = R"(
[robot]
name = "perf_robot"
period_us = 5000
command_timeout_us = 100000

[subsystems.perf_drive]
period_us = 5000

[subsystems.perf_drive.motors.drive]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.perf_roller]
period_us = 20000

[subsystems.perf_roller.motors.roller]
type = "TalonFX"
bus = "rio"
can_id = 2
)";

uint64_t NodeConfigId(const talos::config::RobotConfig& cfg) {
  Config hw = cfg.hardware;
  hw.commissioned = true;
  return ConfigurationId(hw);
}

void PublishCommand(ipc::Publisher<talos::hardware::Packet>& pub,
                    const Command& cmd) {
  talos::hardware::Packet pkt{};
  pkt.size = static_cast<uint32_t>(Encode(cmd, pkt.data));
  pub.write(pkt);
}

State PerfState(uint64_t config_id, uint64_t boot_id, uint64_t epoch,
                uint64_t sample_time_us) {
  State state{};
  state.config_id = config_id;
  state.boot_id = boot_id;
  state.epoch = epoch;
  state.sample_time_us = sample_time_us;
  state.flags = kConfigured | kEnabled;
  return state;
}

bool InjectState(HardwareNode& node, protocol::RuntimeUdpPeer& gateway,
                 const State& state, uint64_t now_us) {
  talos::hardware::Packet buf{};
  buf.size = static_cast<uint32_t>(Encode(state, buf.data));
  if (gateway.SendFrame(protocol::FrameType::kHardwareState, 1, now_us,
                        buf.data.data(),
                        buf.size) != protocol::UdpStatus::kOk) {
    return false;
  }
  const uint64_t before = node.states_received();
  for (int i = 0; i < 200; ++i) {
    node.Tick(now_us);
    if (node.states_received() > before) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

TEST(BridgePerfTest, HistogramReportsP50P99Max) {
  Histogram empty;
  EXPECT_EQ(empty.count(), 0u);
  EXPECT_EQ(empty.P50(), 0u);
  EXPECT_EQ(empty.P99(), 0u);
  EXPECT_EQ(empty.Max(), 0u);

  Histogram hist;
  for (uint64_t i = 1; i <= 100; ++i) hist.Record(i);
  EXPECT_EQ(hist.count(), 100u);
  EXPECT_EQ(hist.P50(), 50u);
  EXPECT_EQ(hist.P99(), 99u);
  EXPECT_EQ(hist.Max(), 100u);

  Histogram single;
  single.Record(7);
  EXPECT_EQ(single.P50(), 7u);
  EXPECT_EQ(single.P99(), 7u);
  EXPECT_EQ(single.Max(), 7u);
}

TEST(BridgePerfTest, SignalAgesAreDurationsNeverCrossClockDifferences) {
  CommandPathMetrics metrics;
  // Phoenix ages arrive already aged; a value that looks like a raw gateway
  // timestamp must be stored verbatim, never differenced against the bridge.
  metrics.RecordSignalAge(150);
  metrics.RecordSignalAge(9000);
  EXPECT_EQ(metrics.signal_age_us.count(), 2u);
  EXPECT_EQ(metrics.signal_age_us.Max(), 9000u);

  // Bridge-clock gaps live in a separate histogram and are unaffected.
  EXPECT_EQ(metrics.state_gap_us.count(), 0u);
  metrics.OnStateReceived(1000000);
  metrics.OnStateReceived(1005000);
  EXPECT_EQ(metrics.state_gap_us.count(), 1u);
  EXPECT_EQ(metrics.state_gap_us.P50(), 5000u);
  EXPECT_EQ(metrics.signal_age_us.count(), 2u);
}

TEST(BridgePerfTest, ChangeDrivenPublicationSuppressesSteadyTraffic) {
  auto cfg = talos::config::ParseRobotConfigString(kPerfToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  HardwareNode node{cfg, "127.0.0.1", 25920, 25921, true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  // Heartbeat is timeout/5 at 5x margin inside the gateway expiry.
  EXPECT_EQ(node.heartbeat_us(), 20000u);
  EXPECT_LE(5 * node.heartbeat_us(), cfg.hardware.command_timeout_us);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", 25920, "127.0.0.1", 25921),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(
      InjectState(node, gateway, PerfState(config_id, 21, 1, 1000), 1000000));

  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("perf_drive")};
  ipc::Publisher<talos::hardware::Packet> roller_pub{
      RequestTopic("perf_roller")};

  Command drive_cmd{};
  drive_cmd.config_id = config_id;
  drive_cmd.boot_id = 21;
  drive_cmd.epoch = 1;
  drive_cmd.observed_time_us = 1000;
  drive_cmd.count = 1;
  drive_cmd.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  Command roller_cmd{};
  roller_cmd.config_id = config_id;
  roller_cmd.boot_id = 21;
  roller_cmd.epoch = 1;
  roller_cmd.observed_time_us = 1000;
  roller_cmd.count = 1;
  roller_cmd.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};

  // First merge goes out immediately.
  PublishCommand(drive_pub, drive_cmd);
  PublishCommand(roller_pub, roller_cmd);
  node.Tick(1001000);
  EXPECT_EQ(node.commands_sent(), 1u);
  EXPECT_EQ(node.metrics().suppressed_sends, 0u);

  // Identical re-merge: suppressed, still live.
  PublishCommand(drive_pub, drive_cmd);
  PublishCommand(roller_pub, roller_cmd);
  node.Tick(1002000);
  EXPECT_EQ(node.commands_sent(), 1u);
  EXPECT_EQ(node.metrics().suppressed_sends, 1u);
  EXPECT_FALSE(node.is_subsystem_timed_out("perf_drive"));

  // Still live one tick later with no republish: suppressed again.
  node.Tick(1003000);
  EXPECT_EQ(node.commands_sent(), 1u);
  EXPECT_EQ(node.metrics().suppressed_sends, 2u);

  // Heartbeat elapses with an unchanged but live merge: resend.
  PublishCommand(drive_pub, drive_cmd);
  PublishCommand(roller_pub, roller_cmd);
  node.Tick(1022000);
  EXPECT_EQ(node.commands_sent(), 2u);

  // Changed merge sends immediately, without waiting for the heartbeat.
  drive_cmd.motors[0].demand = 99.0;
  PublishCommand(drive_pub, drive_cmd);
  node.Tick(1023000);
  EXPECT_EQ(node.commands_sent(), 3u);
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 99.0);

  // Timeout neutralization changes the merge, so it sends immediately too.
  node.Tick(1045000);
  EXPECT_TRUE(node.is_subsystem_timed_out("perf_drive"));
  EXPECT_FALSE(node.is_subsystem_timed_out("perf_roller"));
  EXPECT_EQ(node.commands_sent(), 4u);
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kNeutral);

  // Send-gap evidence lines up with the four sends above.
  EXPECT_EQ(node.metrics().send_gap_us.count(), 3u);
  EXPECT_GT(node.metrics().tick_us.count(), 0u);
}

// Realistic-load benchmark (perf item 47): companion-rate bridge load with
// every actuator-owning node publishing at its configured period, states at
// the 200 Hz gateway rate, plus a logging-style and a Studio-style
// shared-memory reader. Prints the config+measurement identity to record
// alongside `git rev-parse --short HEAD` and `bazel version`.
//
// Timing honesty: the 1 kHz synthetic tick loop runs ~300x faster than wall
// time, while loopback UDP delivery costs tens of microseconds. Publishing a
// request stamped with a just-sent (not yet received) sample would be
// correctly rejected by the identity gate, so each cycle first spins Ticks
// at a fixed now_us until the bridge ingests the state, exactly like a
// production subsystem that stamps requests with the sample it observed on
// /hw/state. Requests therefore cite only observed samples, by construction.
TEST(BridgePerfTest, RealisticLoadBenchmark) {
  auto cfg = talos::config::ParseRobotConfigString(kPerfToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  HardwareNode node{cfg, "127.0.0.1", 25922, 25923, true};
  ASSERT_TRUE(node.Open());

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", 25922, "127.0.0.1", 25923),
            protocol::UdpStatus::kOk);

  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("perf_drive")};
  ipc::Publisher<talos::hardware::Packet> roller_pub{
      RequestTopic("perf_roller")};
  // Logging-style (command copy) and Studio-style (state copy) readers. Same
  // OVERWRITE_OLDEST policy as production: a slow reader is lapped, never
  // stalls the bridge.
  ipc::Subscriber<talos::hardware::Packet> log_sub{kCommandTopic};
  ipc::Subscriber<talos::hardware::Packet> studio_sub{kStateTopic};
  uint64_t log_reads = 0;
  uint64_t studio_reads = 0;

  // One drain tick first: LATEST jumps past any packets an earlier test in
  // this process left on the shared perf_* topics, so the run below starts
  // clean. The drain itself may reject that stale backlog; baseline after.
  uint64_t now_us = 2000000;
  node.Tick(now_us);
  const uint64_t rej_before = node.identity_rejected();

  constexpr int kCycles = 400;
  // Synthetic gateway clock for sample_time_us. It deliberately shares no
  // origin with now_us: the bridge never differences the two.
  uint64_t gateway_sample = 50000;
  for (int cycle = 0; cycle < kCycles; ++cycle) {
    gateway_sample += 5000;
    State state = PerfState(config_id, 31, 1, gateway_sample);
    state.motor_count = 2;
    state.motors[0].id = drive;
    state.motors[0].valid = true;
    state.motors[0].position_age_us = 150;
    state.motors[0].velocity_age_us = 200;
    state.motors[1].id = roller;
    state.motors[1].valid = true;
    state.motors[1].position_age_us = 180;
    state.motors[1].velocity_age_us = 210;
    talos::hardware::Packet buf{};
    buf.size = static_cast<uint32_t>(Encode(state, buf.data));
    ASSERT_EQ(gateway.SendFrame(protocol::FrameType::kHardwareState, 1, now_us,
                                buf.data.data(), buf.size),
              protocol::UdpStatus::kOk);

    // Spin at fixed synthetic time until the state is ingested. Bounded and
    // wall-fast; the synthetic clock does not advance while waiting for the
    // kernel, so no request can cite an unobserved sample.
    const uint64_t states_before = node.states_received();
    bool ingested = false;
    for (int spin = 0; spin < 10000 && !ingested; ++spin) {
      node.Tick(now_us);
      ingested = node.states_received() > states_before;
    }
    ASSERT_TRUE(ingested) << "cycle " << cycle;

    // Actuator nodes at their configured periods, stamped with the sample
    // the bridge just ingested: drive every cycle (5 ms), roller every 4th
    // (20 ms).
    Command drive_cmd{};
    drive_cmd.config_id = config_id;
    drive_cmd.boot_id = 31;
    drive_cmd.epoch = 1;
    drive_cmd.observed_time_us = gateway_sample;
    drive_cmd.count = 1;
    drive_cmd.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
    PublishCommand(drive_pub, drive_cmd);
    if (cycle % 4 == 0) {
      Command roller_cmd{};
      roller_cmd.config_id = config_id;
      roller_cmd.boot_id = 31;
      roller_cmd.epoch = 1;
      roller_cmd.observed_time_us = gateway_sample;
      roller_cmd.count = 1;
      roller_cmd.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};
      PublishCommand(roller_pub, roller_cmd);
    }

    // Five 1 ms ticks to the next cycle: 400 cycles over 2 synthetic seconds
    // at companion rates, states at the 200 Hz gateway rate. The first tick
    // carries the fresh merge; the rest suppress.
    for (int k = 0; k < 5; ++k) {
      now_us += 1000;
      node.Tick(now_us);
      if (log_sub.read().has_value()) ++log_reads;
      if (studio_sub.read().has_value()) ++studio_reads;
    }
  }

  const auto& metrics = node.metrics();
  std::printf(
      "[benchmark] config_id=0x%llx periods_us={drive:5000,roller:20000} "
      "cycles=%d states=%llu commands=%llu suppressed=%llu "
      "rejected=%llu tick_us={p50:%llu,p99:%llu,max:%llu} "
      "state_gap_us={p50:%llu,max:%llu} signal_age_us={p50:%llu,max:%llu} "
      "log_reads=%llu studio_reads=%llu\n",
      static_cast<unsigned long long>(config_id), kCycles,
      static_cast<unsigned long long>(node.states_received()),
      static_cast<unsigned long long>(node.commands_sent()),
      static_cast<unsigned long long>(metrics.suppressed_sends),
      static_cast<unsigned long long>(node.identity_rejected() - rej_before),
      static_cast<unsigned long long>(metrics.tick_us.P50()),
      static_cast<unsigned long long>(metrics.tick_us.P99()),
      static_cast<unsigned long long>(metrics.tick_us.Max()),
      static_cast<unsigned long long>(metrics.state_gap_us.P50()),
      static_cast<unsigned long long>(metrics.state_gap_us.Max()),
      static_cast<unsigned long long>(metrics.signal_age_us.P50()),
      static_cast<unsigned long long>(metrics.signal_age_us.Max()),
      static_cast<unsigned long long>(log_reads),
      static_cast<unsigned long long>(studio_reads));

  EXPECT_EQ(node.states_received(), static_cast<uint64_t>(kCycles));
  // Every request cited an observed sample: the gate must stay silent.
  EXPECT_EQ(node.identity_rejected(), rej_before);
  EXPECT_EQ(node.ownership_rejected(), 0u);
  EXPECT_GT(node.commands_sent(), 0u);
  // Steady-state traffic is mostly heartbeat-suppressed, not rebroadcast.
  EXPECT_GT(metrics.suppressed_sends, node.commands_sent());
  // Generous stall tripwire, not a performance claim: catches a pathological
  // Tick, not a slow host.
  EXPECT_LT(metrics.tick_us.Max(), 20000u);
}

}  // namespace
}  // namespace talos::hardware

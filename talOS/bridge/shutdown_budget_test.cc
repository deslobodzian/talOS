#include <gtest/gtest.h>
#include <poll.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "talOS/bridge/node.h"
#include "talOS/bridge/node_test_support.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/os/poller.h"
#include "talOS/hardware/gateway.h"
#include "talOS/hardware/messages.h"
#include "talOS/introspection/registry.h"
#include "talOS/ipc/publisher.h"
#include "talOS/ipc/subscriber.h"
#include "talOS/process/process.h"
#include "talOS/protocol/frame.h"

namespace talos::hardware {
extern std::function<void(event::log::LogWriterOptions&)> log_options_for_test;

namespace {
using namespace bridge_test;  // NOLINT(google-build-using-namespace)

// Minimal Backend for Gateway::Accept verification: records the last applied
// motor, digital, and PWM requests and reports healthy feedback for all
// configured motors.
class RecordingBackend : public Backend {
 public:
  bool Configure(const Config& config) override {
    config_ = config;
    return true;
  }

  bool Read(State& state) override {
    state.motor_count = static_cast<uint16_t>(config_.motors.size());
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      MotorSample& sample = state.motors[i];
      sample = MotorSample{};
      sample.id = config_.motors[i].id;
      sample.valid = true;
      sample.position_age_us = 0;
      sample.velocity_age_us = 0;
    }
    state.health.valid = true;
    state.health.battery_v = 12.5;
    return true;
  }

  bool Apply(std::span<const MotorRequest> requests) override {
    last_motors.assign(requests.begin(), requests.end());
    ++apply_calls;
    return true;
  }

  std::span<const uint16_t> ApplyFailedDevices() const override { return {}; }

  bool ApplyOutputs(std::span<const DigitalOutputRequest> digital_outputs,
                    std::span<const PwmRequest> pwm_outputs) override {
    last_digital_outputs.assign(digital_outputs.begin(), digital_outputs.end());
    last_pwm_outputs.assign(pwm_outputs.begin(), pwm_outputs.end());
    return true;
  }

  void Neutral() override {
    last_motors.clear();
    for (const auto& m : config_.motors) {
      last_motors.push_back(MotorRequest{m.id, Mode::kNeutral, 0, 0.0, 0.0});
    }
    last_digital_outputs.clear();
    for (const auto& d : config_.digital_outputs) {
      last_digital_outputs.push_back(DigitalOutputRequest{d.id, false});
    }
    last_pwm_outputs.clear();
    for (const auto& p : config_.pwm_outputs) {
      last_pwm_outputs.push_back(PwmRequest{p.id, PwmMode::kOff, 0.0});
    }
    ++neutral_calls;
  }

  Config config_{};
  std::vector<MotorRequest> last_motors;
  std::vector<DigitalOutputRequest> last_digital_outputs;
  std::vector<PwmRequest> last_pwm_outputs;
  uint64_t apply_calls{0};
  uint64_t neutral_calls{0};
};

struct ReceivedCommandFrame {
  uint64_t sequence{0};
  uint64_t sent_us{0};
  Command command{};
};

std::optional<ReceivedCommandFrame> ReceiveCommandFrame(
    protocol::RuntimeUdpPeer& gateway) {
  std::optional<ReceivedCommandFrame> out;
  const bool got = WaitFor([&] {
    protocol::DecodedFrame frame{};
    while (gateway.TryReceive(&frame) == protocol::UdpStatus::kOk) {
      if (frame.header.type != protocol::FrameType::kHardwareCommand) continue;
      ReceivedCommandFrame received{};
      received.sequence = frame.header.sequence;
      received.sent_us = frame.header.monotonic_time_us;
      if (Decode(std::span<const uint8_t>{frame.payload, frame.payload_size},
                 received.command)) {
        out = received;
      }
      return true;
    }
    return false;
  });
  return got ? out : std::nullopt;
}

bool SocketReadable(const HardwareNode& node) {
  if (node.udp_fd() < 0) return false;
  pollfd pfd{node.udp_fd(), POLLIN, 0};
  return ::poll(&pfd, 1, 0) == 1 && (pfd.revents & POLLIN) != 0;
}

void ExpectAllNeutral(const Command& cmd, const Config& hw_cfg) {
  ASSERT_EQ(cmd.count, hw_cfg.motors.size());
  for (std::size_t i = 0; i < cmd.count; ++i) {
    EXPECT_EQ(cmd.motors[i].id, hw_cfg.motors[i].id) << i;
    EXPECT_EQ(cmd.motors[i].mode, Mode::kNeutral) << i;
    EXPECT_LE(cmd.motors[i].slot, 2u) << i;
    EXPECT_DOUBLE_EQ(cmd.motors[i].demand, 0.0) << i;
    EXPECT_DOUBLE_EQ(cmd.motors[i].feedforward_v, 0.0) << i;
  }
  ASSERT_EQ(cmd.digital_output_count, hw_cfg.digital_outputs.size());
  for (std::size_t i = 0; i < cmd.digital_output_count; ++i) {
    EXPECT_EQ(cmd.digital_outputs[i].id, hw_cfg.digital_outputs[i].id) << i;
    EXPECT_FALSE(cmd.digital_outputs[i].value) << i;
  }
  ASSERT_EQ(cmd.pwm_output_count, hw_cfg.pwm_outputs.size());
  for (std::size_t i = 0; i < cmd.pwm_output_count; ++i) {
    EXPECT_EQ(cmd.pwm_outputs[i].id, hw_cfg.pwm_outputs[i].id) << i;
    EXPECT_EQ(cmd.pwm_outputs[i].mode, PwmMode::kOff) << i;
    EXPECT_DOUBLE_EQ(cmd.pwm_outputs[i].output, 0.0) << i;
  }
}

std::atomic<int> shutdown_send_error{0};
int TakeShutdownSendError() { return shutdown_send_error.exchange(0); }

// An active driving bridge calling SendShutdownNeutral() immediately sends a
// fully neutral Command (motors, digital outputs, PWM outputs) stamped with
// the current gateway identity and latest observed sample time, accepted by a
// real Gateway::Accept() with zero rejections, and publishes it on /hw/command.
// REQ-SAFE-01
// REQ-OPS-20
TEST(HardwareNodeShutdownBudgetTest,
     SendShutdownNeutralSendsAcceptedNeutralForAllActuators) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Config hw_cfg = cfg.hardware;
  hw_cfg.commissioned = true;
  const uint64_t config_id = ConfigurationId(hw_cfg);

  // Set up a real Gateway so we prove the RoboRIO gateway accepts both the
  // driving command and the subsequent shutdown neutral command.
  RecordingBackend backend;
  constexpr uint64_t kBootId = 11;
  Gateway real_gateway{hw_cfg, backend, kBootId};
  real_gateway.Tick(10'000, /*driver_station_enabled=*/true);
  const State live_state = real_gateway.snapshot();
  ASSERT_EQ(live_state.config_id, config_id);
  ASSERT_EQ(live_state.boot_id, kBootId);
  ASSERT_NE(live_state.epoch, 0u);
  ASSERT_EQ(live_state.sample_time_us, 10'000u);

  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  ipc::Publisher<Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Publisher<Packet> intake_pub{RequestTopic("intake")};
  ipc::Subscriber<Packet> cmd_sub{
      kCommandTopic,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};

  constexpr uint64_t kStartUs = 1'000'000;
  ASSERT_TRUE(InjectState(node, link.gateway, live_state, kStartUs));

  // Drive both subsystems (motors with non-zero slot/demand, DIO high, PWM on).
  Command drive_req{};
  drive_req.config_id = config_id;
  drive_req.boot_id = kBootId;
  drive_req.epoch = live_state.epoch;
  drive_req.observed_time_us = live_state.sample_time_us;
  drive_req.count = 1;
  drive_req.motors[0] = {cfg.subsystems.at("drivetrain").motors[0],
                         Mode::kVoltage, /*slot=*/1, 6.0, 0.0};
  PublishCommand(drive_pub, drive_req);

  Command intake_req{};
  intake_req.config_id = config_id;
  intake_req.boot_id = kBootId;
  intake_req.epoch = live_state.epoch;
  intake_req.observed_time_us = live_state.sample_time_us;
  intake_req.count = 1;
  intake_req.motors[0] = {cfg.subsystems.at("intake").motors[0], Mode::kVoltage,
                          /*slot=*/2, 4.0, 0.0};
  intake_req.digital_output_count = 1;
  intake_req.digital_outputs[0] = {
      cfg.subsystems.at("intake").digital_outputs[0], true};
  intake_req.pwm_output_count = 1;
  intake_req.pwm_outputs[0] = {cfg.subsystems.at("intake").pwm_outputs[0],
                               PwmMode::kPulse, 0.5};
  PublishCommand(intake_pub, intake_req);

  node.Tick(kStartUs + 1'000);
  const auto driving = ReceiveCommandFrame(link.gateway);
  ASSERT_TRUE(driving.has_value()) << "expected initial driving command";
  ASSERT_TRUE(real_gateway.Accept(driving->command, driving->sequence, 12'000));
  ASSERT_EQ(backend.last_motors.size(), 2u);
  EXPECT_EQ(backend.last_motors[0].mode, Mode::kVoltage);
  EXPECT_EQ(backend.last_motors[1].mode, Mode::kVoltage);
  ASSERT_EQ(backend.last_digital_outputs.size(), 1u);
  EXPECT_TRUE(backend.last_digital_outputs[0].value);
  ASSERT_EQ(backend.last_pwm_outputs.size(), 1u);
  EXPECT_EQ(backend.last_pwm_outputs[0].mode, PwmMode::kPulse);

  // Drain /hw/command so the next read comes from SendShutdownNeutral().
  while (cmd_sub.read().has_value()) {
  }
  const uint64_t commands_before = node.commands_sent();

  // Trigger clean shutdown neutral deterministically.
  constexpr uint64_t kStopUs = kStartUs + 2'000;
  ASSERT_TRUE(node.SendShutdownNeutral(kStopUs));
  EXPECT_EQ(node.commands_sent(), commands_before + 1);

  const auto shutdown = ReceiveCommandFrame(link.gateway);
  ASSERT_TRUE(shutdown.has_value()) << "expected shutdown neutral command";
  EXPECT_GT(shutdown->sequence, driving->sequence);
  EXPECT_EQ(shutdown->sent_us, kStopUs);
  EXPECT_EQ(shutdown->command.config_id, config_id);
  EXPECT_EQ(shutdown->command.boot_id, kBootId);
  EXPECT_EQ(shutdown->command.epoch, live_state.epoch);
  EXPECT_EQ(shutdown->command.observed_time_us, live_state.sample_time_us);
  ExpectAllNeutral(shutdown->command, hw_cfg);

  // A real Gateway accepts the shutdown neutral command and neutralizes every
  // actuator immediately without incrementing rejected_command_count.
  EXPECT_TRUE(
      real_gateway.Accept(shutdown->command, shutdown->sequence, 15'000));
  EXPECT_EQ(real_gateway.snapshot().counters.rejected_command_count, 0u);
  ASSERT_EQ(backend.last_motors.size(), 2u);
  EXPECT_EQ(backend.last_motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(backend.last_motors[1].mode, Mode::kNeutral);
  ASSERT_EQ(backend.last_digital_outputs.size(), 1u);
  EXPECT_FALSE(backend.last_digital_outputs[0].value);
  ASSERT_EQ(backend.last_pwm_outputs.size(), 1u);
  EXPECT_EQ(backend.last_pwm_outputs[0].mode, PwmMode::kOff);

  // /hw/command also receives the shutdown neutral command.
  const auto published = cmd_sub.read();
  ASSERT_TRUE(published.has_value()) << "expected neutral on /hw/command";
  Command published_cmd{};
  ASSERT_TRUE(Decode(published->bytes(), published_cmd));
  EXPECT_EQ(published_cmd, shutdown->command);
}

// SendShutdownNeutral() sends nothing before a State has arrived or when the
// UDP socket is closed.
// REQ-SAFE-01
// REQ-OPS-20
TEST(HardwareNodeShutdownBudgetTest,
     SendShutdownNeutralNoOpsBeforeFirstStateOrWhenSocketClosed) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  ipc::Subscriber<Packet> cmd_sub{
      kCommandTopic,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};

  // Before any State: last_boot_id_ == 0 -> no-op.
  EXPECT_FALSE(node.SendShutdownNeutral(1'000'000));
  EXPECT_EQ(node.commands_sent(), 0u);
  EXPECT_FALSE(cmd_sub.read().has_value());

  // After a State, keep the State fresh while forcing a permanent send error
  // on the probe retry so peer_ closes.
  ASSERT_TRUE(InjectState(node, link.gateway,
                          TrustState(NodeConfigId(cfg), 11, 1, 1000),
                          1'000'000));
  ASSERT_TRUE(InjectState(node, link.gateway,
                          TrustState(NodeConfigId(cfg), 11, 1, 51'000),
                          1'050'000));
  shutdown_send_error.store(ENETUNREACH);
  node.set_seams({.send_error = &TakeShutdownSendError});
  node.Tick(1'100'001);  // Probe retry fails with ENETUNREACH -> closes peer_.
  ASSERT_EQ(node.udp_fd(), -1);
  node.set_seams({});

  const uint64_t commands_before = node.commands_sent();
  EXPECT_FALSE(node.SendShutdownNeutral(1'101'000));
  EXPECT_EQ(node.commands_sent(), commands_before);
}

// SendShutdownNeutral() is a no-op when the last accepted State is stale
// (>= command_timeout_us old) or the gateway is not ready / running another
// configuration, preventing stale or retired-boot neutrals from incrementing
// rejected_command_count on recovery.
// REQ-SAFE-01
// REQ-SAFE-17
// REQ-OPS-20
TEST(HardwareNodeShutdownBudgetTest,
     SendShutdownNeutralNoOpsWhenStateStaleOrGatewayNotReady) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const uint64_t config_id = NodeConfigId(cfg);
  const uint64_t timeout_us = cfg.hardware.command_timeout_us;

  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  ipc::Subscriber<Packet> cmd_sub{
      kCommandTopic,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};

  constexpr uint64_t kStartUs = 1'000'000;
  ASSERT_TRUE(InjectState(node, link.gateway,
                          TrustState(config_id, 11, 1, 1'000), kStartUs));

  // Stale link: State age at or past command_timeout_us -> no-op.
  EXPECT_FALSE(node.SendShutdownNeutral(kStartUs + timeout_us));
  EXPECT_FALSE(node.SendShutdownNeutral(kStartUs + timeout_us + 50'000));
  EXPECT_EQ(node.commands_sent(), 0u);
  EXPECT_FALSE(cmd_sub.read().has_value());

  // Gateway disabled (not ready), even with a fresh State -> no-op.
  State disabled = TrustState(config_id, 11, 2, 200'000);
  disabled.flags = kConfigured;
  ASSERT_TRUE(InjectState(node, link.gateway, disabled, kStartUs + 200'000));
  EXPECT_FALSE(node.SendShutdownNeutral(kStartUs + 201'000));
  EXPECT_EQ(node.commands_sent(), 0u);

  // Gateway faulted (not ready), even with a fresh State -> no-op.
  State faulted = TrustState(config_id, 11, 3, 210'000);
  faulted.flags = kConfigured | kEnabled | kHardwareFault;
  ASSERT_TRUE(InjectState(node, link.gateway, faulted, kStartUs + 210'000));
  EXPECT_FALSE(node.SendShutdownNeutral(kStartUs + 211'000));
  EXPECT_EQ(node.commands_sent(), 0u);

  // Gateway running another configuration -> no-op.
  State wrong_config = TrustState(config_id ^ 1u, 11, 4, 220'000);
  ASSERT_TRUE(
      InjectState(node, link.gateway, wrong_config, kStartUs + 220'000));
  EXPECT_FALSE(node.SendShutdownNeutral(kStartUs + 221'000));
  EXPECT_EQ(node.commands_sent(), 0u);
  EXPECT_FALSE(cmd_sub.read().has_value());

  // Fresh, ready State matching config_id right inside command_timeout_us ->
  // sends the shutdown neutral.
  ASSERT_TRUE(InjectState(node, link.gateway,
                          TrustState(config_id, 11, 5, 230'000),
                          kStartUs + 230'000));
  EXPECT_TRUE(node.SendShutdownNeutral(kStartUs + 230'000 + timeout_us - 1));
  EXPECT_EQ(node.commands_sent(), 1u);
}

// HardwareNode::Run() sends the shutdown neutral right after its loop exits
// (before CloseWaits()), proving clean shutdown neutral is wired into Run().
// REQ-SAFE-01
// REQ-OPS-20
TEST(HardwareNodeShutdownBudgetTest, RunSendsShutdownNeutralOnCleanExit) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  // Use the largest valid command_timeout_us (500 ms) so Run()'s startup
  // (OpenWaits() + clock read) cannot age out the freshly injected State under
  // TSan or CI load; the exact command_timeout_us staleness boundary is proved
  // deterministically in
  // SendShutdownNeutralNoOpsWhenStateStaleOrGatewayNotReady.
  cfg.hardware.command_timeout_us = 500'000;
  Config hw_cfg = cfg.hardware;
  hw_cfg.commissioned = true;
  const uint64_t config_id = ConfigurationId(hw_cfg);

  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  ipc::Publisher<Packet> intake_pub{RequestTopic("intake")};
  ipc::Subscriber<Packet> cmd_sub{
      kCommandTopic,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};

  const uint64_t start_us =
      static_cast<uint64_t>(event::Poller::now().nanos() / 1000);
  ASSERT_TRUE(InjectState(node, link.gateway,
                          TrustState(config_id, 11, 2, 5'000), start_us));
  Command req = IntakeRequest(cfg, 5.0);
  req.epoch = 2;
  req.observed_time_us = 5'000;
  PublishCommand(intake_pub, req);
  node.Tick(start_us);

  const auto driving = ReceiveCommandFrame(link.gateway);
  ASSERT_TRUE(driving.has_value());
  ASSERT_EQ(
      MotorOf(driving->command, cfg.subsystems.at("intake").motors[0]).mode,
      Mode::kVoltage);

  // Refresh the accepted State after the driving UDP round-trip so setup
  // latency cannot age out command_timeout_us before Run() calls
  // SendShutdownNeutral(stop_us). Stamp receipt once the datagram is already
  // readable on the bridge socket.
  constexpr uint64_t kRefreshSampleUs = 10'000;
  const uint64_t states_before_refresh = node.states_received();
  const uint64_t commands_before_refresh = node.commands_sent();
  ASSERT_TRUE(
      SendState(link.gateway, TrustState(config_id, 11, 2, kRefreshSampleUs)));
  ASSERT_TRUE(WaitFor([&] { return SocketReadable(node); }));
  const uint64_t refresh_us =
      static_cast<uint64_t>(event::Poller::now().nanos() / 1000);
  node.Tick(refresh_us);
  ASSERT_EQ(node.states_received(), states_before_refresh + 1);
  uint64_t last_sequence = driving->sequence;
  for (uint64_t c = commands_before_refresh; c < node.commands_sent(); ++c) {
    const auto extra = ReceiveCommandFrame(link.gateway);
    ASSERT_TRUE(extra.has_value());
    last_sequence = extra->sequence;
  }
  while (cmd_sub.read().has_value()) {
  }
  const uint64_t commands_before = node.commands_sent();

  // Exit Run()'s loop immediately without signals or sleeps.
  struct StopGuard {
    ~StopGuard() {
      process::stop_requested.store(false);
      static_cast<void>(process::ConsumeStopWake());
    }
  } guard;
  process::stop_requested.store(true);
  ::testing::internal::CaptureStdout();
  const int exit_code = node.Run();
  static_cast<void>(::testing::internal::GetCapturedStdout());

  EXPECT_EQ(exit_code, 0);
  ASSERT_EQ(node.commands_sent(), commands_before + 1)
      << "Run() exited without sending a shutdown neutral command";

  const auto shutdown = ReceiveCommandFrame(link.gateway);
  ASSERT_TRUE(shutdown.has_value());
  EXPECT_GT(shutdown->sequence, last_sequence);
  EXPECT_EQ(shutdown->command.config_id, config_id);
  EXPECT_EQ(shutdown->command.boot_id, 11u);
  EXPECT_EQ(shutdown->command.epoch, 2u);
  EXPECT_EQ(shutdown->command.observed_time_us, kRefreshSampleUs);
  ExpectAllNeutral(shutdown->command, hw_cfg);

  const auto published = cmd_sub.read();
  ASSERT_TRUE(published.has_value());
  Command published_cmd{};
  ASSERT_TRUE(Decode(published->bytes(), published_cmd));
  EXPECT_EQ(published_cmd, shutdown->command);
}

// HardwareNode::Run() sends its shutdown neutral, closes its waits and UDP
// socket, publishes introspection, and marks its registry row kFlagQuiesced
// BEFORE log_->finish() + log_->flush() blocks on disk I/O (REQ-OPS-20).
// REQ-OPS-20
TEST(HardwareNodeShutdownBudgetTest,
     RunQuiescesAndClosesUdpBeforeLogFlushUnblocks) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  cfg.hardware.command_timeout_us = 500'000;
  Config hw_cfg = cfg.hardware;
  hw_cfg.commissioned = true;
  const uint64_t config_id = ConfigurationId(hw_cfg);
  const std::string log_path = ::testing::TempDir() + "/bridge_quiesce_" +
                               std::to_string(::getpid()) + ".tlog";
  constexpr uint64_t kSessionId = 61;

  struct StopGuard {
    ~StopGuard() {
      process::stop_requested.store(false);
      static_cast<void>(process::ConsumeStopWake());
    }
  };
  struct HeldLog {
    HeldLog() {
      log_options_for_test = [this](event::log::LogWriterOptions& options) {
        options.write_interval = std::chrono::hours{1};
        options.sync_interval = std::chrono::hours{1};
        options.before_write = [this] {
          write_blocked.store(true, std::memory_order_release);
          std::unique_lock lock{mutex};
          cv.wait(lock, [this] { return released; });
        };
        options.before_flush_wait = [this] {
          flush_entered.store(true, std::memory_order_release);
        };
      };
    }
    ~HeldLog() {
      Release();
      log_options_for_test = nullptr;
    }
    HeldLog(const HeldLog&) = delete;
    HeldLog& operator=(const HeldLog&) = delete;
    void Release() {
      {
        std::lock_guard lock{mutex};
        released = true;
      }
      cv.notify_all();
    }
    std::mutex mutex;
    std::condition_variable cv;
    bool released{false};
    std::atomic<bool> write_blocked{false};
    std::atomic<bool> flush_entered{false};
  };

  {
    Link link;
    StopGuard stop_guard;
    HeldLog held;
    ASSERT_TRUE(link.Open(cfg, true, log_path, kSessionId));
    HardwareNode& node = link.node.value();
    ipc::Publisher<Packet> intake_pub{RequestTopic("intake")};

    const uint64_t start_us =
        static_cast<uint64_t>(event::Poller::now().nanos() / 1000);
    ASSERT_TRUE(InjectState(node, link.gateway,
                            TrustState(config_id, 11, 2, 5'000), start_us));
    Command req = IntakeRequest(cfg, 5.0);
    req.epoch = 2;
    req.observed_time_us = 5'000;
    PublishCommand(intake_pub, req);
    node.Tick(start_us);
    const auto driving = ReceiveCommandFrame(link.gateway);
    ASSERT_TRUE(driving.has_value());

    // Refresh accepted State right before Run() so command_timeout_us stays
    // fresh for SendShutdownNeutral(stop_us).
    constexpr uint64_t kRefreshSampleUs = 10'000;
    const uint64_t states_before_refresh = node.states_received();
    const uint64_t commands_before_refresh = node.commands_sent();
    ASSERT_TRUE(SendState(link.gateway,
                          TrustState(config_id, 11, 2, kRefreshSampleUs)));
    ASSERT_TRUE(WaitFor([&] { return SocketReadable(node); }));
    const uint64_t refresh_us =
        static_cast<uint64_t>(event::Poller::now().nanos() / 1000);
    node.Tick(refresh_us);
    ASSERT_EQ(node.states_received(), states_before_refresh + 1);
    for (uint64_t c = commands_before_refresh; c < node.commands_sent(); ++c) {
      ASSERT_TRUE(ReceiveCommandFrame(link.gateway).has_value());
    }

    process::stop_requested.store(true);
    std::atomic<int> exit_code{-1};
    std::thread runner{[&] { exit_code.store(node.Run()); }};
    struct JoinOnExit {
      HeldLog& held;
      std::thread& thread;
      ~JoinOnExit() {
        held.Release();
        if (thread.joinable()) thread.join();
      }
    } join_on_exit{held, runner};

    // Wait until Run() is blocked inside log_->flush() while the background
    // LogWriter thread is held inside before_write.
    ASSERT_TRUE(WaitFor(
        [&] { return held.flush_entered.load(std::memory_order_acquire); }));
    ASSERT_TRUE(WaitFor(
        [&] { return held.write_blocked.load(std::memory_order_acquire); }));

    // While before_write is still blocked, the bridge must already have sent
    // its shutdown neutral, closed its UDP socket, and marked kFlagQuiesced.
    const auto reader = introspect::RegistryReader::open();
    ASSERT_TRUE(reader.has_value());
    std::optional<introspect::NodeSnapshot> bridge_row;
    for (const auto& row : reader->nodes()) {
      if (row.name == kNodeName &&
          row.pid == static_cast<uint64_t>(::getpid()) &&
          row.session_id == kSessionId &&
          (!bridge_row || row.generation > bridge_row->generation)) {
        bridge_row = row;
      }
    }
    ASSERT_TRUE(bridge_row.has_value()) << "bridge registry row missing";
    EXPECT_TRUE(bridge_row->quiesced())
        << "kFlagQuiesced was not set before log_->flush() blocked";
    EXPECT_EQ(node.udp_fd(), -1)
        << "UDP socket was not closed before log_->flush() blocked";

    const auto shutdown = ReceiveCommandFrame(link.gateway);
    ASSERT_TRUE(shutdown.has_value())
        << "SendShutdownNeutral did not reach gateway while log flush blocked";
    ExpectAllNeutral(shutdown->command, hw_cfg);

    held.Release();
    runner.join();
    EXPECT_EQ(exit_code.load(), 0);
  }
  std::filesystem::remove(log_path);
}

// Staggered bursts (31 + 32 + 32) within a single 1 ms window must never
// exceed 64 received datagrams, including if a timed duty wakes Pass() again
// before the window expires; after the 1 ms window rolls over, the remaining
// queued datagrams drain cleanly.
// REQ-RT-20
TEST(HardwareNodeShutdownBudgetTest,
     StaggeredBurstNeverExceeds64DatagramsPerMillisecondWindow) {
  // Guard literal REQ-RT-20 bounds against constant drift.
  EXPECT_EQ(HardwareNode::kCpuWindowUs, 1'000u);
  EXPECT_EQ(HardwareNode::kPassBudget, 8u);
  EXPECT_EQ(HardwareNode::kDatagramBudget, 64u);

  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;

  constexpr uint64_t kStartUs = 1'000'000;
  ASSERT_TRUE(InjectState(
      node, gateway, TrustState(NodeConfigId(cfg), 11, 1, 1000), kStartUs));

  uint64_t next_sample_us = 1001;
  auto send_ds_burst = [&](uint64_t count, uint64_t frame_time_us) {
    driver_station::DriverStationData data{};
    data.boot_id = 11;
    std::array<uint8_t, 256> bytes{};
    for (uint64_t i = 0; i < count; ++i) {
      data.sample_time_us = next_sample_us++;
      const std::size_t size = driver_station::Encode(data, bytes);
      ASSERT_EQ(gateway.SendFrame(protocol::FrameType::kDriverStation, 0,
                                  frame_time_us, bytes.data(), size),
                protocol::UdpStatus::kOk);
    }
  };

  // Start a clean 1 ms CPU window at kWindowStartUs (no timed duty falls in
  // [kWindowStartUs, kWindowStartUs + 1000)).
  constexpr uint64_t kWindowStartUs = kStartUs + 10'000;
  uint64_t in_window = 0;

  // Burst 1: 31 datagrams.
  send_ds_burst(31, kWindowStartUs);
  ASSERT_TRUE(WaitFor([&] {
    if (!SocketReadable(node)) return false;
    const uint64_t before = node.driver_station_packets_received();
    const HardwareNode::Wait wait = node.Pass(kWindowStartUs);
    in_window += node.driver_station_packets_received() - before;
    EXPECT_LE(in_window, 64u);
    EXPECT_FALSE(wait.ignore_input);
    return in_window == 31u;
  })) << "received "
      << in_window << " of 31";

  // Burst 2: 32 datagrams in the same 1 ms window -> 63 total, still under 64.
  send_ds_burst(32, kWindowStartUs + 100);
  ASSERT_TRUE(WaitFor([&] {
    if (!SocketReadable(node)) return false;
    const uint64_t before = node.driver_station_packets_received();
    const HardwareNode::Wait wait = node.Pass(kWindowStartUs + 100);
    in_window += node.driver_station_packets_received() - before;
    EXPECT_LE(in_window, 64u);
    EXPECT_FALSE(wait.ignore_input);
    return in_window == 63u;
  })) << "received "
      << in_window << " of 63";

  // Burst 3: 32 more datagrams in the same 1 ms window. Only 1 datagram of
  // budget remains, so Pass() must cap Tick()'s receive loop to 1 (total 64),
  // never 32 (which would hit 95).
  send_ds_burst(32, kWindowStartUs + 200);
  ASSERT_TRUE(WaitFor([&] { return SocketReadable(node); }));
  {
    const uint64_t before = node.driver_station_packets_received();
    const uint64_t now_us = kWindowStartUs + 200;
    const HardwareNode::Wait wait = node.Pass(now_us);
    const uint64_t received = node.driver_station_packets_received() - before;
    in_window += received;
    EXPECT_EQ(received, 1u) << "expected remaining budget of 1 datagram";
    EXPECT_LE(in_window, 64u);
    EXPECT_EQ(in_window, 64u);
    EXPECT_TRUE(wait.ignore_input);
    EXPECT_GT(wait.until_us, now_us);
    EXPECT_LE(wait.until_us, now_us + 1'000u);
    EXPECT_EQ(wait.until_us, kWindowStartUs + 1'000u);
  }

  // A mid-window pass (e.g., a timed duty waking Run() before the 1 ms window
  // ends) with 31 datagrams still queued must receive 0 datagrams.
  ASSERT_TRUE(SocketReadable(node));
  {
    const uint64_t before = node.driver_station_packets_received();
    const uint64_t now_us = kWindowStartUs + 500;
    const HardwareNode::Wait wait = node.Pass(now_us);
    const uint64_t received = node.driver_station_packets_received() - before;
    EXPECT_EQ(received, 0u) << "mid-window pass exceeded 64-datagram cap";
    EXPECT_TRUE(wait.ignore_input);
    EXPECT_GT(wait.until_us, now_us);
    EXPECT_LE(wait.until_us, now_us + 1'000u);
    EXPECT_EQ(wait.until_us, kWindowStartUs + 1'000u);
  }

  // Once the 1 ms window expires, the budget resets and the remaining 31
  // datagrams drain cleanly.
  uint64_t window2_received = 0;
  constexpr uint64_t kNextWindowUs = kWindowStartUs + 1'000u;
  ASSERT_TRUE(WaitFor([&] {
    if (!SocketReadable(node)) return false;
    const uint64_t before = node.driver_station_packets_received();
    const HardwareNode::Wait wait = node.Pass(kNextWindowUs);
    window2_received += node.driver_station_packets_received() - before;
    EXPECT_LE(window2_received, 64u);
    EXPECT_FALSE(wait.ignore_input);
    return node.driver_station_packets_received() == 95u;
  })) << "drained "
      << node.driver_station_packets_received() << " of 95";
  EXPECT_EQ(window2_received, 31u);
}

}  // namespace
}  // namespace talos::hardware

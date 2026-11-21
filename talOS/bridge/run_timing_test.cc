// Run() wall-clock timing against real time and loopback UDP (the integration
// tier, docs/TESTING.md §2, §6.7): isolated into an exclusive target so
// parallel Bazel jobs on 3-vCPU macOS CI do not starve StallMeter's 1 ms
// host-qualification probe.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stop_token>
#include <thread>
#include <vector>

#include "talOS/bridge/merge.h"
#include "talOS/bridge/node.h"
#include "talOS/bridge/node_test_support.h"

namespace talos::hardware {
namespace {
using namespace bridge_test;  // NOLINT(google-build-using-namespace)

// The host's scheduling noise over a test: the longest a 1 ms sleep of an
// ordinary thread overran, for a timed test to read before it judges
// (docs/TESTING.md §6.7), so a stalled host fails as unqualified.
class StallMeter {
 public:
  StallMeter()
      : thread_{[this](const std::stop_token& stop) {
          while (!stop.stop_requested()) {
            const auto before = std::chrono::steady_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
            const auto over = std::chrono::steady_clock::now() - before -
                              std::chrono::milliseconds{1};
            const auto over_us =
                std::chrono::duration_cast<std::chrono::microseconds>(over)
                    .count();
            worst_us_.store(std::max<std::int64_t>(worst_us_.load(), over_us));
          }
        }} {}

  // Stops measuring; the worst overrun, in microseconds.
  uint64_t Stop() {
    thread_.request_stop();
    if (thread_.joinable()) thread_.join();
    return static_cast<uint64_t>(std::max<std::int64_t>(worst_us_.load(), 0));
  }

 private:
  std::atomic<std::int64_t> worst_us_{0};
  std::jthread thread_;
};

// Under Run(), with one request and nothing after it, the heartbeats and the
// owner timeout's neutral command still go out on their own. The bounds are
// the requirement's: each frame no earlier than its duty, and before the
// gateway's command_timeout_us would have expired the last one.
// NextDutyUsIsWhenTickNextHasWork pins the exact instants; this proves Run()
// sleeps until them rather than past them.
// REQ-SAFE-15
// REQ-RT-20
TEST(HardwareNodeTest, RunSendsHeartbeatsAndTheTimeoutNeutralOnItsOwn) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.subsystems.at("intake").motors[0];
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  ipc::Subscriber<Packet> states{
      kStateTopic,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};
  ipc::Publisher<Packet> pub{RequestTopic("intake")};
  std::vector<SentCommand> commands;
  StallMeter host;
  {
    RunningBridge run{node};
    ASSERT_TRUE(
        SendState(link.gateway, TrustState(NodeConfigId(cfg), 11, 1, 1000)));
    ASSERT_TRUE(WaitFor([&] { return Forwarded(states, 1000); }));
    PublishCommand(pub, IntakeRequest(cfg, 4.0));
    do {
      auto sent = NextCommand(link.gateway);
      ASSERT_TRUE(sent) << "after " << commands.size() << " commands";
      commands.push_back(*sent);
    } while (MotorOf(commands.back().command, roller).mode != Mode::kNeutral);
  }
  const uint64_t heartbeat_us = node.heartbeat_us();
  const uint64_t command_timeout_us = cfg.hardware.command_timeout_us;
  const uint64_t timeout_us = OwnerTimeoutUs(5000, command_timeout_us);
  // Judged only on a host whose stalls are well inside the margin the
  // bounds below leave (the neutral's: command_timeout_us less the owner
  // timeout); otherwise the host, not the bridge, is unqualified.
  const uint64_t stall_us = host.Stop();
  const uint64_t max_stall_us = (command_timeout_us - timeout_us) / 2;
  if (stall_us >= max_stall_us) {
    ::testing::Test::RecordProperty("host_unqualified", "true");
  }
  ASSERT_LT(stall_us, max_stall_us)
      << "[HOST_UNQUALIFIED] host unqualified: an ordinary thread's 1 ms sleep "
         "overran by "
      << stall_us << " us (limit " << max_stall_us << " us)";
  // The request's command, at least one heartbeat, the neutral command.
  ASSERT_GE(commands.size(), 3u);
  for (std::size_t i = 1; i + 1 < commands.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(MotorOf(commands[i].command, roller).mode, Mode::kVoltage);
    const uint64_t gap_us = commands[i].sent_us - commands[i - 1].sent_us;
    EXPECT_GE(gap_us, heartbeat_us);
    EXPECT_LT(gap_us, command_timeout_us);
  }
  const uint64_t neutral_after_us =
      commands.back().sent_us - commands.front().sent_us;
  EXPECT_GT(neutral_after_us, timeout_us);
  EXPECT_LT(neutral_after_us, command_timeout_us);
}

}  // namespace
}  // namespace talos::hardware

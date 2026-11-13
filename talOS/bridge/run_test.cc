// Run() against real time and loopback UDP (the integration tier,
// docs/TESTING.md §2): its wakeups, its duties and its registry progress.
// Bounds are ratios of counts that hold however loaded the host is, or a
// requirement's margin judged only on a host whose stalls are inside it.
// Run()'s logic, the exact duty instants included, is tested on synthetic
// time in hardware_node_test.cc.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <thread>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "talOS/bridge/merge.h"
#include "talOS/bridge/node.h"
#include "talOS/bridge/node_test_support.h"
#include "talOS/introspection/registry.h"
#include "talOS/process/process.h"

namespace talos::hardware {
namespace {
using namespace bridge_test;  // NOLINT(google-build-using-namespace)

// Runs the bridge's Run() on its own thread until Stop() or destruction,
// which stop it as SIGTERM does (process::RequestStop) and join it. The
// node's counters are read only once it has stopped.
class RunningBridge {
 public:
  explicit RunningBridge(HardwareNode& node)
      : thread_{[&node] { node.Run(); }} {}
  RunningBridge(const RunningBridge&) = delete;
  RunningBridge& operator=(const RunningBridge&) = delete;
  ~RunningBridge() { Stop(); }

  void Stop() {
    if (!thread_.joinable()) return;
    process::RequestStop(0);
    thread_.join();
    process::stop_requested.store(false);
  }

 private:
  std::thread thread_;
};

// The arm seam of RunEndsItsWaitOnAStop: on the first arm after
// `stop_at_next_arm`, it notes the deadline wakes so far and requests a stop
// from the loop thread, after the loop's flag check and before its wait.
std::atomic<bool> stop_at_next_arm{false};
const HardwareNode* stopping_node = nullptr;
uint64_t deadline_wakes_at_stop = 0;

bool StopWhileArming(ipc::Subscriber<Packet>& subscriber) {
  if (stop_at_next_arm.exchange(false)) {
    deadline_wakes_at_stop = stopping_node->wakeups().deadline;
    process::RequestStop();
  }
  return subscriber.arm_wake();
}

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

// The launcher kills a running node whose dispatch count stops for its stall
// timeout (500 ms for the bridge). Run() must store it far more often than
// its 250 ms full sample (PublishIntrospection), or a 300 ms pause on top of
// an old sample reads as a stall. With no gateway the count still moves:
// /hw/status is published every kStatusPeriodUs (20 ms).
TEST(HardwareNodeTest, RunKeepsRegistryProgressCurrent) {
  auto cfg = talos::config::ParseRobotConfigString(kTestToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg, /*open_gateway=*/false));
  HardwareNode& node = link.node.value();
  std::thread runner([&node] { node.Run(1); });
  std::optional<introspect::RegistryReader> reader;
  std::optional<std::uint64_t> last_dispatch;
  auto last_change = std::chrono::steady_clock::now();
  std::vector<std::chrono::steady_clock::duration> gaps;
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds{1};
  while (std::chrono::steady_clock::now() < end) {
    if (!reader) reader = introspect::RegistryReader::open();
    if (reader) {
      for (const auto& row : reader->nodes()) {
        if (row.name != kNodeName ||
            row.pid != static_cast<std::uint64_t>(getpid()) ||
            row.dispatch_count == 0)
          continue;
        const auto now = std::chrono::steady_clock::now();
        if (last_dispatch && row.dispatch_count != *last_dispatch)
          gaps.push_back(now - last_change);
        if (!last_dispatch || row.dispatch_count != *last_dispatch) {
          last_dispatch = row.dispatch_count;
          last_change = now;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  runner.join();
  ASSERT_GT(gaps.size(), 10u);
  // The launcher kills the bridge once its progress is 500 ms old, and the
  // soak stops it for 300 ms, so the count must move within 200 ms. The
  // median tells a 20 ms store from the 250 ms full-sample cadence however
  // slowly a sanitizer build ticks; the worst gap is the requirement itself.
  std::sort(gaps.begin(), gaps.end());
  EXPECT_LT(gaps[gaps.size() / 2], std::chrono::milliseconds{100});
  EXPECT_LT(gaps.back(), std::chrono::milliseconds{200});
}

// A request published while Run() is asleep wakes it: the command goes out
// when the request is published, not at the loop's next deadline. Each
// request waits for the previous one's command, so the bridge has gone back
// to sleep before it, and the wakeup that finds it is counted as a
// request's. A loop that only polled counts none.
// REQ-RT-20
TEST(HardwareNodeTest, RunSendsTheCommandWhenARequestIsPublished) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.subsystems.at("intake").motors[0];
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  ipc::Subscriber<Packet> states{
      kStateTopic,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};
  ipc::Publisher<Packet> pub{RequestTopic("intake")};
  constexpr int kRequests = 20;
  {
    RunningBridge run{node};
    ASSERT_TRUE(
        SendState(link.gateway, TrustState(NodeConfigId(cfg), 11, 1, 1000)));
    ASSERT_TRUE(WaitFor([&] { return Forwarded(states, 1000); }));
    for (int i = 1; i <= kRequests; ++i) {
      PublishCommand(pub, IntakeRequest(cfg, i));
      std::optional<SentCommand> sent;
      do {
        sent = NextCommand(link.gateway);
        ASSERT_TRUE(sent) << "no command for request " << i;
      } while (MotorOf(sent->command, roller).demand != i);
    }
  }
  const auto& wakeups = node.wakeups();
  EXPECT_GE(wakeups.request, uint64_t{kRequests / 2})
      << "passes " << wakeups.passes << ", udp " << wakeups.udp << ", deadline "
      << wakeups.deadline;
}

// A State arriving while Run() is asleep wakes it and reaches /hw/state at
// once. Each State waits for the previous one on /hw/state, as above.
// REQ-RT-20
TEST(HardwareNodeTest, RunForwardsAStateWhenItArrives) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  ipc::Subscriber<Packet> states{
      kStateTopic,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};
  constexpr uint64_t kStates = 20;
  {
    RunningBridge run{node};
    for (uint64_t i = 1; i <= kStates; ++i) {
      const uint64_t sample_us = i * 1000;
      ASSERT_TRUE(SendState(link.gateway,
                            TrustState(NodeConfigId(cfg), 11, 1, sample_us)));
      ASSERT_TRUE(WaitFor([&] { return Forwarded(states, sample_us); }))
          << "State " << i;
    }
  }
  const auto& wakeups = node.wakeups();
  EXPECT_GE(wakeups.udp, kStates / 2)
      << "passes " << wakeups.passes << ", request " << wakeups.request
      << ", deadline " << wakeups.deadline;
}

// An idle bridge sleeps until its next duty or its longest wait: that wait
// every 10 ms, /hw/status every 20 ms, a clock probe retried every 100 ms and
// the full registry sample every 250 ms. Over any window it makes at most one
// pass per duty instant (fewer where they coincide, and fewer, never more,
// on a loaded host, where a late wake does every duty then due), where a
// 1 ms poll made one a millisecond. The window ends on an event, the 25th
// /hw/status, and nothing arrives during it.
// REQ-RT-20
TEST(HardwareNodeTest, RunSleepsWhileNothingArrives) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  ipc::Subscriber<bridge::BridgeStatus> statuses{bridge::topics::kStatus};
  uint64_t seen = 0;
  const auto start = std::chrono::steady_clock::now();
  {
    RunningBridge run{node};
    ASSERT_TRUE(WaitFor([&] {
      while (statuses.read()) ++seen;
      return seen >= 25;
    }));
  }
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - start)
                                .count();
  // One per period started in the window, each rounded up, and the first.
  const double duty_instants = (elapsed_ms / 10) + (elapsed_ms / 20) +
                               (elapsed_ms / 100) + (elapsed_ms / 250) + 5;
  const auto& wakeups = node.wakeups();
  EXPECT_EQ(wakeups.udp + wakeups.request, 0u);
  EXPECT_LE(static_cast<double>(wakeups.passes), duty_instants)
      << "in " << elapsed_ms << " ms: deadline " << wakeups.deadline;
}

// A stop ends the bridge's wait at once rather than at its kMaxWaitUs cap:
// the poller watches the stop pipe. The stop lands inside a pass, after the
// loop's flag check and before its wait, so that wait either is skipped (a
// pass with work due at once) or ends on the stop pipe; none after the stop
// reaches its deadline.
// REQ-OPS-06
// REQ-RT-20
TEST(HardwareNodeTest, RunEndsItsWaitOnAStop) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  stopping_node = &node;
  node.set_seams({.arm_request = &StopWhileArming});
  ipc::Subscriber<bridge::BridgeStatus> statuses{bridge::topics::kStatus};
  std::thread runner{[&node] { node.Run(); }};
  uint64_t seen = 0;
  const bool running = WaitFor([&] {
    while (statuses.read()) ++seen;
    return seen >= 3;
  });
  stop_at_next_arm.store(true);
  const bool armed = WaitFor([] { return !stop_at_next_arm.load(); });
  if (!armed) process::RequestStop();  // ends the run either way
  runner.join();
  process::stop_requested.store(false);

  ASSERT_TRUE(running) << "no /hw/status while Run() ran";
  ASSERT_TRUE(armed) << "Run() never armed a request subscriber";
  EXPECT_EQ(node.wakeups().deadline, deadline_wakes_at_stop)
      << "a wait begun after the stop ran to its deadline";
}

// Run()'s exit status with no gateway answering: a stop is a requested end
// whether or not a State ever arrived, so 0 (the launcher stops a session
// whose controller is not up yet: a RoboRIO still booting, or the simulated
// gateway, spawned after the bridge, not spawned yet), while a run that
// reaches its --duration-s without one State is a failure, 1.
// REQ-OPS-06
TEST(HardwareNodeTest, ExitStatusWithoutAStateIsZeroOnlyForAStop) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  for (const bool stop : {true, false}) {
    SCOPED_TRACE(stop ? "stopped" : "--duration-s 1 elapsed");
    Link link;
    ASSERT_TRUE(link.Open(cfg, /*open_gateway=*/false));
    HardwareNode& node = link.node.value();
    ipc::Subscriber<bridge::BridgeStatus> statuses{bridge::topics::kStatus};
    int status = -1;
    std::thread runner{
        [&node, &status, stop] { status = node.Run(stop ? 0 : 1); }};
    if (stop) {
      const bool running = WaitFor([&] { return statuses.read().has_value(); });
      process::RequestStop();
      runner.join();
      process::stop_requested.store(false);
      ASSERT_TRUE(running) << "no /hw/status while Run() ran";
    } else {
      runner.join();
    }
    EXPECT_EQ(node.states_received(), 0u);
    EXPECT_EQ(status, stop ? 0 : 1);
  }
}

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
  ASSERT_LT(stall_us, (command_timeout_us - timeout_us) / 2)
      << "host unqualified: an ordinary thread's 1 ms sleep overran by "
      << stall_us << " us";
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

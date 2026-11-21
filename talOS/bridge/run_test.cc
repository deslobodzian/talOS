// Run() against real time and loopback UDP (the integration tier,
// docs/TESTING.md §2): its wakeups, its duties and its registry progress.
// Bounds are ratios of counts that hold however loaded the host is; the
// wall-clock StallMeter timing test is isolated in run_timing_test.cc.
// Run()'s logic, the exact duty instants included, is tested on synthetic
// time in hardware_node_test.cc.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
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

// The wait seam of RunSendsTheCommandWhenARequestIsPublished: at the first
// wait input can end after `publish_at_next_wait` is set, it publishes the
// intake request with that demand from the loop thread, after every arm, so
// the bridge is as good as asleep when the request lands.
std::atomic<int> publish_at_next_wait{0};
ipc::Publisher<Packet>* waiting_publisher = nullptr;
const config::RobotConfig* waiting_config = nullptr;

void PublishBeforeWait() {
  const int demand = publish_at_next_wait.exchange(0);
  if (demand != 0) {
    PublishCommand(*waiting_publisher, IntakeRequest(*waiting_config, demand));
  }
}

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
// request is published by the before_wait seam, after the bridge armed
// every request subscriber and decided to sleep, so the wait it then enters
// ends on the request's notification and the pass after it reads the
// request: one request wakeup per request, whatever the host's timing. A
// loop that did not wait on its request subscribers would sleep to its
// deadline and count none.
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
  waiting_publisher = &pub;
  waiting_config = &cfg;
  node.set_seams({.before_wait = &PublishBeforeWait});
  constexpr int kRequests = 20;
  {
    RunningBridge run{node};
    ASSERT_TRUE(
        SendState(link.gateway, TrustState(NodeConfigId(cfg), 11, 1, 1000)));
    ASSERT_TRUE(WaitFor([&] { return Forwarded(states, 1000); }));
    for (int i = 1; i <= kRequests; ++i) {
      publish_at_next_wait.store(i);
      std::optional<SentCommand> sent;
      do {
        sent = NextCommand(link.gateway);
        ASSERT_TRUE(sent) << "no command for request " << i;
      } while (MotorOf(sent->command, roller).demand != i);
    }
  }
  const auto& wakeups = node.wakeups();
  EXPECT_EQ(wakeups.request, uint64_t{kRequests})
      << "passes " << wakeups.passes << ", udp " << wakeups.udp << ", deadline "
      << wakeups.deadline << ", throttled " << wakeups.throttled;
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

// Run()'s exit status and its no-State warning. A stop is a requested end
// whether or not a State ever arrived, so 0 (the launcher stops a session
// whose controller is not up yet: a RoboRIO still booting, or the simulated
// gateway, spawned after the bridge, not spawned yet), while a run that
// reaches its --duration-s without one State is a failure, 1. Either way a
// run that heard no State says so on stderr, since under the launcher (which
// always stops the bridge) the exit status cannot; a run that heard one
// does not.
// REQ-OPS-06
TEST(HardwareNodeTest, ExitStatusWithoutAStateIsZeroOnlyForAStop) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  struct Row {
    const char* name;
    bool stop;
    bool state;
  };
  for (const Row row : {Row{"stopped", true, false},
                        Row{"--duration-s 1 elapsed", false, false},
                        Row{"stopped after a State", true, true}}) {
    SCOPED_TRACE(row.name);
    Link link;
    ASSERT_TRUE(link.Open(cfg, /*open_gateway=*/row.state));
    HardwareNode& node = link.node.value();
    ipc::Subscriber<bridge::BridgeStatus> statuses{bridge::topics::kStatus};
    ipc::Subscriber<Packet> states{
        kStateTopic,
        RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};
    int status = -1;
    ::testing::internal::CaptureStderr();
    std::thread runner{
        [&node, &status, &row] { status = node.Run(row.stop ? 0 : 1); }};
    bool running = true;
    bool forwarded = true;
    if (row.stop) {
      running = WaitFor([&] { return statuses.read().has_value(); });
      if (row.state) {
        forwarded = SendState(link.gateway,
                              TrustState(NodeConfigId(cfg), 11, 1, 1000)) &&
                    WaitFor([&] { return Forwarded(states, 1000); });
      }
      process::RequestStop();
    }
    runner.join();
    process::stop_requested.store(false);
    const std::string stderr_text = ::testing::internal::GetCapturedStderr();
    ASSERT_TRUE(running) << "no /hw/status while Run() ran";
    ASSERT_TRUE(forwarded) << "the gateway's State was not forwarded";
    EXPECT_EQ(node.states_received(), row.state ? 1u : 0u);
    EXPECT_EQ(status, row.stop ? 0 : 1);
    EXPECT_EQ(stderr_text.find("ended without receiving a State") !=
                  std::string::npos,
              !row.state)
        << stderr_text;
  }
}

std::atomic<int> run_pending_recv_error{0};
int TakeRunRecvError() { return run_pending_recv_error.exchange(0); }

// Under Run() with its poller active, a transient receive error (ECONNREFUSED)
// keeps the socket open, while a permanent receive error (ENETUNREACH) closes
// the socket, removes its fd from the poller, reopens it, registers the new fd
// so subsequent States still arrive, and reports the reopen on /hw/status.
// REQ-RT-22
TEST(HardwareNodeTest, RunReopensSocketOnPermanentReceiveErrorAndKeepsRunning) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  run_pending_recv_error.store(0);
  node.set_seams({.recv_error = &TakeRunRecvError});
  ipc::Subscriber<Packet> states{
      kStateTopic,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};
  ipc::Subscriber<bridge::BridgeStatus> statuses{
      bridge::topics::kStatus,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};
  {
    RunningBridge run{node};
    ASSERT_TRUE(
        SendState(link.gateway, TrustState(NodeConfigId(cfg), 11, 1, 1000)));
    ASSERT_TRUE(WaitFor([&] { return Forwarded(states, 1000); }));

    // Transient ECONNREFUSED keeps the socket open and frames keep arriving.
    run_pending_recv_error.store(ECONNREFUSED);
    ASSERT_TRUE(
        SendState(link.gateway, TrustState(NodeConfigId(cfg), 11, 1, 2000)));
    ASSERT_TRUE(WaitFor([&] { return Forwarded(states, 2000); }));

    // Permanent ENETUNREACH triggers a reopen and re-adds the fd to poller_.
    run_pending_recv_error.store(ENETUNREACH);
    ASSERT_TRUE(WaitFor([&] {
      const auto status = statuses.read();
      return status && status->socket_reopens_count() >= 1u;
    }));
    ASSERT_TRUE(
        SendState(link.gateway, TrustState(NodeConfigId(cfg), 11, 1, 3000)));
    ASSERT_TRUE(WaitFor([&] { return Forwarded(states, 3000); }));
  }
  EXPECT_EQ(node.socket_counters().reopens, 1u);
  EXPECT_EQ(node.wakeups().wait_fallbacks, 0u);
}

// Under Run(), a HardwareNode pinned to an absent local_address starts cleanly,
// keeps its poller active without falling back to 1 ms polling, and retries
// opening on the 100 ms cadence rather than falling back to 0.0.0.0.
// REQ-RT-22, REQ-OPS-25
TEST(HardwareNodeTest,
     RunRetriesAbsentLocalAddressWithoutCrashingOrPollingFallback) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  uint16_t gateway_port = 0;
  uint16_t node_port = 0;
  int gw_res = ReservePort(gateway_port);
  int node_res = ReservePort(node_port);
  ASSERT_GE(gw_res, 0);
  ASSERT_GE(node_res, 0);
  Release(node_res);

  HardwareNode node{cfg, "127.0.0.1", gateway_port, node_port,  true, 0,
                    {},  0,           {},           "192.0.2.1"};
  ASSERT_TRUE(node.Open());
  ipc::Subscriber<bridge::BridgeStatus> statuses{bridge::topics::kStatus};
  uint64_t status_count = 0;
  {
    RunningBridge run{node};
    // 10 status publications at 20 ms each (~180 ms) guarantees at least two
    // 100 ms reopen attempts on the bridge clock.
    ASSERT_TRUE(WaitFor([&] {
      while (statuses.read()) ++status_count;
      return status_count >= 10;
    }));
  }
  EXPECT_EQ(node.udp_fd(), -1);
  EXPECT_EQ(node.wakeups().wait_fallbacks, 0u);
  EXPECT_GE(node.socket_counters().reopen_attempts, 2u);
  EXPECT_EQ(node.socket_counters().reopens, 0u);
  Release(gw_res);
}

}  // namespace
}  // namespace talos::hardware

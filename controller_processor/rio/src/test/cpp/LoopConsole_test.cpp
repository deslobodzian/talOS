// The RoboRIO console's lines for what the loop finds after a tick and no
// State carries: each command expiry once, with its time and its delay from
// the command's observation; each changed configuration refused while
// enabled; and each rise in the loop watchdog's dropped stall reports.
// Driven through a real gateway and watchdog on explicit times, as the loop
// drives them.
#include "LoopConsole.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "LoopWatchdog.h"
#include "RioLog.h"
#include "hardware/config.h"
#include "hardware/gateway.h"
#include "hardware/gateway_test_support.h"
#include "hardware/sim_backend.h"

namespace {

namespace hw = talos::hardware;
using talos::rio::LoopConsole;
using talos::rio::RioLog;

constexpr uint64_t kPeriodUs = 5'000;

// A commissioned gateway with no devices (REQ-CONTRACT-35), and the console
// lines its LoopConsole posts, as RioLog's logger thread would write them.
class LoopConsoleTest : public ::testing::Test {
 protected:
  static hw::Config Commissioned() {
    hw::Config config;
    config.commissioned = true;
    return config;
  }

  // Ticks the gateway enabled every period from `from_us` up to `to_us`,
  // the console observing after each tick, as Robot.cpp's loop does.
  void TickEnabled(uint64_t from_us, uint64_t to_us) {
    for (uint64_t now_us = from_us; now_us <= to_us; now_us += kPeriodUs) {
      gateway_.Tick(now_us, true);
      console_.ObserveGateway(gateway_);
    }
  }

  // Accepts a command observing the gateway's last State at `now_us`.
  void AcceptAt(uint64_t now_us) {
    ASSERT_TRUE(gateway_.Accept(hw::gateway_test::Request(gateway_),
                                ++sequence_, now_us));
  }

  std::vector<std::string> lines_;
  RioLog log_{[this](std::string_view line) { lines_.emplace_back(line); }};
  hw::SimBackend backend_;
  hw::Gateway gateway_{Commissioned(), backend_, 7};
  LoopConsole console_{log_};
  uint64_t sequence_{0};
};

// A command left to expire (the link, the bridge or the launcher gone) makes
// one console line, however many neutral ticks follow, naming when the
// gateway invalidated it and how long after the State it was built from;
// the next expiry makes the next line.
// REQ-SAFE-16
TEST_F(LoopConsoleTest, EachCommandExpiryPrintsOneLineWithItsTime) {
  TickEnabled(1'000'000, 1'000'000);
  AcceptAt(1'002'000);  // observing the State of 1.000 s
  // The timeout (100 ms) runs out at 1.100 s; a second of neutral ticks
  // follows.
  TickEnabled(1'005'000, 2'100'000);
  log_.Drain(5'000'000);
  ASSERT_EQ(lines_.size(), 1u) << "one line per expiry, not per tick";
  EXPECT_NE(lines_[0].find("command expired"), std::string::npos) << lines_[0];
  EXPECT_NE(lines_[0].find("[1.100]"), std::string::npos)
      << "the tick that went neutral: " << lines_[0];
  EXPECT_NE(lines_[0].find("100.0 ms after its observation"), std::string::npos)
      << lines_[0];
  EXPECT_NE(lines_[0].find("98.0 ms after its acceptance"), std::string::npos)
      << lines_[0];

  AcceptAt(2'101'000);  // observing the State of 2.100 s
  TickEnabled(2'105'000, 2'300'000);
  log_.Drain(10'000'000);
  ASSERT_EQ(lines_.size(), 2u);
  EXPECT_NE(lines_[1].find("[2.200]"), std::string::npos) << lines_[1];
  EXPECT_NE(lines_[1].find("(expiry 2)"), std::string::npos) << lines_[1];
}

// After a deploy that changed the hardware configuration, the RoboRIO
// restores the old one and the Driver Station may already be enabled: the
// gateway refuses the changed push, every node stays neutral, and only the
// console says why. It names each refused push once, both configurations
// and what to do; disabled, the same push is accepted.
// REQ-SAFE-21
TEST_F(LoopConsoleTest, APushRefusedWhileEnabledNamesTheFixOncePerPush) {
  hw::Config changed = Commissioned();
  changed.status_hz = 100;
  const std::string expected =
      "hardware gateway refused configuration " +
      std::to_string(hw::ConfigurationId(changed)) +
      " while enabled (running " +
      std::to_string(hw::ConfigurationId(Commissioned())) +
      "): disable to accept the new configuration";
  TickEnabled(1'000'000, 1'000'000);
  uint64_t now_us = 1'005'000;
  for (std::size_t push = 1; push <= 2; ++push) {
    EXPECT_FALSE(gateway_.BeginReconfigure(changed));
    // A second of ticks after each push.
    TickEnabled(now_us, now_us + 1'000'000);
    now_us += 1'000'000 + kPeriodUs;
    log_.Drain(push * 10'000'000);
    ASSERT_EQ(lines_.size(), push) << "one line per refused push";
    EXPECT_NE(lines_.back().find(expected), std::string::npos) << lines_.back();
  }

  gateway_.Tick(now_us, false);
  ASSERT_TRUE(gateway_.BeginReconfigure(changed));
  gateway_.Tick(now_us + kPeriodUs, false);  // the configure completes
  console_.ObserveGateway(gateway_);
  EXPECT_EQ(gateway_.snapshot().config_id, hw::ConfigurationId(changed));
  log_.Drain(30'000'000);
  EXPECT_EQ(lines_.size(), 2u);
}

// Watchdog outputs whose reports are never delivered: the reporter thread
// blocked in a Driver Station call.
class UndeliveredOutputs final : public talos::rio::WatchdogOutputs {
 public:
  bool Neutralize() override { return true; }
  void Report(std::string_view) override {}
};

// One Driver Station error per stall is the watchdog's contract; a stall
// past its kReportSlots undelivered reports loses its error. The console
// says so whenever the dropped count rises, with the stalls in all, so a
// lost stall report is never silent.
// REQ-SAFE-25
TEST_F(LoopConsoleTest, DroppedWatchdogReportsArePrintedWhenTheyRise) {
  UndeliveredOutputs outputs;
  talos::rio::LoopWatchdog watchdog{outputs};
  uint64_t now_us = 1'000'000;
  // Each stall: a beat whose deadline then passes, and a poll that finds it.
  const auto stall = [&] {
    watchdog.Beat(now_us, now_us + 1);
    watchdog.Poll(now_us + 1);
    now_us += kPeriodUs;
  };
  for (std::size_t i = 0; i < talos::rio::LoopWatchdog::kReportSlots; ++i)
    stall();
  console_.ObserveWatchdog(watchdog);
  log_.Drain(5'000'000);
  ASSERT_TRUE(lines_.empty()) << "nothing was dropped yet: " << lines_[0];

  stall();
  ASSERT_EQ(watchdog.dropped_reports(), 1u);
  console_.ObserveWatchdog(watchdog);
  console_.ObserveWatchdog(watchdog);  // the next tick: no new drop
  log_.Drain(10'000'000);
  ASSERT_EQ(lines_.size(), 1u);
  EXPECT_NE(lines_[0].find("1 stall report dropped (1 since start), 5 stalls"),
            std::string::npos)
      << lines_[0];

  stall();
  stall();
  console_.ObserveWatchdog(watchdog);
  log_.Drain(20'000'000);
  ASSERT_EQ(lines_.size(), 2u);
  EXPECT_NE(lines_[1].find("2 stall reports dropped (3 since start), 7 stalls"),
            std::string::npos)
      << lines_[1];
}

}  // namespace

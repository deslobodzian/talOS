// The drive team's companion link report: each change reaches the Driver
// Station once, its text starting with the console time prefix; the
// dashboard carries every companion/ value; and the loop thread's Observe
// never waits on the reporter.
#include "CompanionLink.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <gtest/gtest.h>
#include <hal/HAL.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <mutex>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "RioLog.h"
#include "protocol/types.h"

namespace {

using talos::rio::CompanionContact;
using talos::rio::CompanionReporter;
using talos::rio::CompanionSink;
using talos::rio::CompanionStatus;

// Every Driver Station message, whichever change it reports, starts with
// the console time prefix, "[<s>.<ms>] " (RioLog's ConsoleTime), so it can be
// placed against the program's console lines (REQ-OPS-24).
class Recorder final : public CompanionSink {
 public:
  void Publish(const CompanionStatus& status) override { last = status; }
  void Report(bool error, std::string_view text) override {
    static const std::regex kConsoleTime{R"(^\[[0-9]+\.[0-9]{3}\] )"};
    EXPECT_TRUE(std::regex_search(std::string{text}, kConsoleTime))
        << "a Driver Station message without the console time prefix: " << text;
    reports.emplace_back(error, std::string{text});
  }
  // The reports since the last call.
  std::vector<std::pair<bool, std::string>> Take() {
    return std::exchange(reports, {});
  }
  CompanionStatus last;
  std::vector<std::pair<bool, std::string>> reports;
};

constexpr uint64_t kStart = 1'000'000;

CompanionContact HeardAt(uint64_t frame_us) {
  CompanionContact contact;
  contact.frame_us = frame_us;
  return contact;
}

// Unheard past the startup grace, connected, steady, lost, back: one report
// per change, and none while the link holds or stays down.
// REQ-OPS-24
TEST(CompanionReporter, ReportsEachLinkChangeOnce) {
  Recorder sink;
  CompanionReporter reporter{sink};
  reporter.Poll(kStart);
  reporter.Poll(kStart + CompanionReporter::kStartupGraceUs - 1);
  EXPECT_TRUE(sink.Take().empty());
  EXPECT_FALSE(sink.last.connected);

  uint64_t now = kStart + CompanionReporter::kStartupGraceUs;
  reporter.Poll(now);
  reporter.Poll(now + 100'000);
  auto reports = sink.Take();
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_TRUE(reports[0].first);
  EXPECT_NE(reports[0].second.find("not heard"), std::string::npos);

  for (int i = 0; i < 5; ++i) {
    now += 100'000;
    reporter.Observe(HeardAt(now - 20'000));
    reporter.Poll(now);
  }
  reports = sink.Take();
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_FALSE(reports[0].first);
  EXPECT_TRUE(sink.last.connected);
  EXPECT_DOUBLE_EQ(sink.last.frame_age_s, 0.02);
  EXPECT_LT(sink.last.command_age_s, 0.0);

  // The last frame ages past the timeout: lost, once.
  const uint64_t last_frame = now - 20'000;
  reporter.Poll(last_frame + CompanionReporter::kLinkTimeoutUs - 1);
  EXPECT_TRUE(sink.Take().empty());
  for (int i = 0; i < 5; ++i) {
    reporter.Poll(last_frame + CompanionReporter::kLinkTimeoutUs + i * 100'000);
  }
  reports = sink.Take();
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_TRUE(reports[0].first);
  EXPECT_NE(reports[0].second.find("lost"), std::string::npos);
  // Stamped with the poll's time on the console's clock, so the message
  // lines up with the program's time-stamped console lines.
  EXPECT_EQ(reports[0].second.rfind(
                talos::rio::ConsoleTime(last_frame +
                                        CompanionReporter::kLinkTimeoutUs),
                0),
            0u)
      << reports[0].second;
  EXPECT_FALSE(sink.last.connected);

  now = last_frame + 2 * CompanionReporter::kLinkTimeoutUs;
  reporter.Observe(HeardAt(now));
  reporter.Poll(now);
  reports = sink.Take();
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_FALSE(reports[0].first);
}

// A companion of another protocol version: one error naming both versions
// (and not a second "not heard" for the same cause), then one clear.
// REQ-OPS-24
TEST(CompanionReporter, ReportsAProtocolMismatchOnceWithBothVersions) {
  Recorder sink;
  CompanionReporter reporter{sink};
  reporter.Poll(kStart);
  CompanionContact contact;
  contact.mismatched_version = 99;
  uint64_t now = kStart;
  for (int i = 0; i < 400; ++i) {
    now += 100'000;
    contact.version_mismatch_us = now - 20'000;
    reporter.Observe(contact);
    reporter.Poll(now);
  }
  ASSERT_GT(now - kStart, CompanionReporter::kStartupGraceUs);
  auto reports = sink.Take();
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_TRUE(reports[0].first);
  EXPECT_NE(reports[0].second.find("protocol 99"), std::string::npos);
  EXPECT_FALSE(sink.last.protocol_ok);
  EXPECT_EQ(sink.last.peer_protocol, 99);

  // The companion is redeployed from this build.
  now += 100'000;
  contact.frame_us = now;
  reporter.Observe(contact);
  reporter.Poll(contact.version_mismatch_us +
                CompanionReporter::kLinkTimeoutUs);
  reports = sink.Take();
  ASSERT_EQ(reports.size(), 2u);
  EXPECT_FALSE(reports[0].first);
  EXPECT_NE(reports[0].second.find("cleared"), std::string::npos);
  EXPECT_FALSE(reports[1].first);
  EXPECT_TRUE(sink.last.protocol_ok);
  EXPECT_TRUE(sink.last.connected);
}

// Probes alone keep the link up while every actuator node is down (or crash
// looping): one warning once the link has been up a startup grace, none while
// commands flow, and one again when they stop.
// REQ-OPS-24
TEST(CompanionReporter, ReportsALinkWithoutCommandsOncePerStretch) {
  Recorder sink;
  CompanionReporter reporter{sink};
  CompanionContact contact;
  uint64_t now = kStart;
  const auto run_for = [&](uint64_t duration_us, bool commands) {
    for (const uint64_t end = now + duration_us; now < end;) {
      now += 100'000;
      contact.frame_us = now - 20'000;
      if (commands) contact.command_us = now - 20'000;
      reporter.Observe(contact);
      reporter.Poll(now);
    }
  };
  run_for(CompanionReporter::kStartupGraceUs - 200'000, false);
  auto reports = sink.Take();
  ASSERT_EQ(reports.size(), 1u);  // connected
  EXPECT_LT(sink.last.command_age_s, 0.0);

  run_for(10'000'000, false);
  reports = sink.Take();
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_FALSE(reports[0].first);
  EXPECT_NE(reports[0].second.find("no actuator commands"), std::string::npos);

  run_for(5'000'000, true);
  EXPECT_TRUE(sink.Take().empty());
  EXPECT_DOUBLE_EQ(sink.last.command_age_s, 0.02);

  run_for(10'000'000, false);
  reports = sink.Take();
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_NE(reports[0].second.find("no actuator commands"), std::string::npos);
}

// The production sink puts every companion/ value the drive team reads
// (talOS/hardware/README.md "RoboRIO") on the dashboard, each following the
// status it publishes, and config_id as 16 hex digits: a 64-bit id does not
// survive a double.
// REQ-OPS-24
TEST(CompanionDashboard, PublishesEveryCompanionValue) {
  HAL_Initialize(500, 0);
  using frc::SmartDashboard;
  const auto sink = talos::rio::MakeDashboardSink();
  for (const bool up : {true, false}) {
    SCOPED_TRACE(up ? "link up" : "link down");
    CompanionStatus status;
    status.connected = up;
    status.protocol_ok = !up;
    status.peer_protocol = up ? 99 : 0;
    status.frame_age_s = up ? 0.25 : -1.0;
    status.command_age_s = up ? 1.5 : -1.0;
    status.configured = up;
    status.commissioned = up;
    status.config_id = up ? uint64_t{0xab} : uint64_t{0xfedcba9876543211};
    sink->Publish(status);
    // Each default is the value the key must not hold.
    EXPECT_EQ(SmartDashboard::GetBoolean("companion/connected", !up), up);
    EXPECT_EQ(SmartDashboard::GetBoolean("companion/protocol_ok", up), !up);
    EXPECT_EQ(SmartDashboard::GetNumber("companion/protocol", -2.0),
              static_cast<double>(talos::protocol::kProtocolVersion));
    EXPECT_EQ(SmartDashboard::GetNumber("companion/peer_protocol", -2.0),
              up ? 99.0 : 0.0);
    EXPECT_EQ(SmartDashboard::GetNumber("companion/frame_age_s", -2.0),
              up ? 0.25 : -1.0);
    EXPECT_EQ(SmartDashboard::GetNumber("companion/command_age_s", -2.0),
              up ? 1.5 : -1.0);
    EXPECT_EQ(SmartDashboard::GetBoolean("companion/configured", !up), up);
    EXPECT_EQ(SmartDashboard::GetBoolean("companion/commissioned", !up), up);
    EXPECT_EQ(SmartDashboard::GetString("companion/config_id", ""),
              up ? "00000000000000ab" : "fedcba9876543211");
  }
}

// The reporter thread stuck in a dashboard write (a NetworkTables or Driver
// Station call that blocks) must not hold the loop thread: Observe completes.
TEST(CompanionReporter, ObserveDoesNotWaitOnABlockedSink) {
  class Blocking final : public CompanionSink {
   public:
    void Publish(const CompanionStatus&) override {
      std::unique_lock lock{mutex};
      entered = true;
      changed.notify_all();
      changed.wait(lock, [this] { return released; });
    }
    void Report(bool, std::string_view) override {}
    std::mutex mutex;
    std::condition_variable changed;
    bool entered = false;
    bool released = false;
  } sink;
  CompanionReporter reporter{sink};
  reporter.Start();
  {
    std::unique_lock lock{sink.mutex};
    ASSERT_TRUE(sink.changed.wait_for(lock, std::chrono::seconds{5},
                                      [&] { return sink.entered; }));
  }
  auto loop = std::async(std::launch::async, [&] {
    for (uint64_t i = 1; i <= 100'000; ++i) reporter.Observe(HeardAt(i));
  });
  const bool finished =
      loop.wait_for(std::chrono::seconds{5}) == std::future_status::ready;
  {
    std::lock_guard lock{sink.mutex};
    sink.released = true;
  }
  sink.changed.notify_all();
  reporter.Stop();
  EXPECT_TRUE(finished);
}

}  // namespace

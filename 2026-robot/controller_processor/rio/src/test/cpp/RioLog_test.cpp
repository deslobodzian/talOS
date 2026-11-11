// The RoboRIO loop thread's reports: posting never waits on the logger's
// output, a full queue drops and counts, and a toggling fault is coalesced to
// one line per second for that motor.
#include "RioLog.h"

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

namespace {

using talos::rio::LogEvent;
using talos::rio::LogKind;
using talos::rio::RioLog;

LogEvent FaultChange(uint16_t motor, uint32_t value, uint32_t previous) {
  LogEvent event;
  event.kind = LogKind::kMotorFault;
  event.index = motor;
  event.value = value;
  event.previous = previous;
  event.SetName("motor" + std::to_string(motor));
  return event;
}

// A burst of fault changes posted while the logger thread is stuck in a
// blocked write (a full console) completes without waiting for it: the queue
// fills, the excess is dropped and counted, and the counted drops are
// reported once the write unblocks.
TEST(RioLog, BurstWhileTheWriterIsBlockedDropsAndCountsInsteadOfWaiting) {
  std::mutex mutex;
  std::condition_variable changed;
  bool writing = false;
  bool release = false;
  std::vector<std::string> lines;
  RioLog log{[&](std::string_view line) {
    std::unique_lock lock{mutex};
    writing = true;
    changed.notify_all();
    changed.wait(lock, [&] { return release; });
    lines.emplace_back(line);
  }};
  log.Start();
  ASSERT_TRUE(log.Post(FaultChange(0, 0x100, 0)));
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(changed.wait_for(lock, std::chrono::seconds{5},
                                 [&] { return writing; }));
  }

  constexpr int kBurst = 10 * static_cast<int>(RioLog::kCapacity);
  int accepted = 0;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kBurst; ++i) {
    accepted +=
        log.Post(FaultChange(1, (i & 1) != 0 ? 0x300 : 0x100, 0)) ? 1 : 0;
  }
  const auto elapsed = std::chrono::steady_clock::now() - start;
  // At most the queue's capacity (one cell may already be drained).
  EXPECT_LE(accepted, static_cast<int>(RioLog::kCapacity));
  EXPECT_EQ(log.dropped(), static_cast<uint64_t>(kBurst - accepted));
  // Generous: a blocking post would never return at all.
  EXPECT_LT(elapsed, std::chrono::seconds{1});

  {
    std::lock_guard lock{mutex};
    release = true;
  }
  changed.notify_all();
  log.Stop();
  bool reported = false;
  for (const auto& line : lines)
    reported |= line.find("log queue full: " + std::to_string(log.dropped()) +
                          " events dropped") != std::string::npos;
  EXPECT_TRUE(reported) << "no line reports the dropped events";
}

// A current-limit bit toggling at the fault-field rate, for several seconds,
// prints its first change at once and then one coalesced line each second
// with the count and the toggled bits, never a bare change line; another
// motor's first change is not held back by it.
TEST(RioLog, FaultTogglesCoalesceToOneLinePerSecondPerMotor) {
  std::vector<std::string> lines;
  RioLog log{[&](std::string_view line) { lines.emplace_back(line); }};
  constexpr uint32_t kSupplyLimit = 1u << 8;

  ASSERT_TRUE(log.Post(FaultChange(3, kSupplyLimit, 0)));
  log.Drain(0);
  ASSERT_EQ(lines.size(), 1u);
  // The limiter engaging is a limit acting as configured, not a fault
  // (hardware::kMotorNotFaultFlags).
  EXPECT_NE(lines[0].find("limit raised=[SupplyCurrLimit"), std::string::npos)
      << lines[0];
  EXPECT_NE(lines[0].find("faults=[none]"), std::string::npos) << lines[0];

  // 50 Hz of toggles through three full seconds, drained every 20 ms.
  uint32_t flags = kSupplyLimit;
  for (uint64_t t = 20'000; t <= 3'000'000; t += 20'000) {
    const uint32_t next = flags ^ kSupplyLimit;
    ASSERT_TRUE(log.Post(FaultChange(3, next, flags)));
    flags = next;
    if (t == 500'000) {
      ASSERT_TRUE(log.Post(FaultChange(4, kSupplyLimit, 0)));
      const std::size_t before = lines.size();
      log.Drain(t);
      ASSERT_EQ(lines.size(), before + 1);
      EXPECT_NE(lines.back().find("motor4"), std::string::npos);
      continue;
    }
    log.Drain(t);
  }

  std::vector<std::string> motor3;
  for (const auto& line : lines)
    if (line.find("motor3") != std::string::npos) motor3.push_back(line);
  ASSERT_EQ(motor3.size(), 4u);
  EXPECT_EQ(motor3[0].find("coalesced"), std::string::npos);
  for (std::size_t i = 1; i < motor3.size(); ++i) {
    EXPECT_NE(motor3[i].find("[50 changes coalesced"), std::string::npos)
        << motor3[i];
    EXPECT_NE(motor3[i].find("bits toggled: SupplyCurrLimit"),
              std::string::npos)
        << motor3[i];
  }

  // Quiet for a second: the window closes and the next change prints at once.
  const std::size_t settled = lines.size();
  log.Drain(4'000'000);
  ASSERT_EQ(lines.size(), settled);
  ASSERT_TRUE(log.Post(FaultChange(3, 0, flags)));
  log.Drain(4'000'001);
  ASSERT_EQ(lines.size(), settled + 1);
  EXPECT_EQ(lines.back().find("coalesced"), std::string::npos);
}

// Overruns (work longer than the period) and late ticks (a short tick that
// started long after the previous one, a starved loop) are counted and
// reported at most once per second, each report with that window's maxima.
TEST(RioLog, LoopOverrunsAndLateTicksAreReportedOncePerSecond) {
  std::vector<std::string> lines;
  RioLog log{[&](std::string_view line) { lines.emplace_back(line); }};
  log.RecordTick(0, 4'000, 5'000);
  log.RecordTick(7'000, 5'000, 5'000);  // jitter, not late
  log.Drain(0);
  EXPECT_TRUE(lines.empty());

  log.RecordTick(5'000, 12'000, 5'000);
  log.RecordTick(30'000, 1'000, 5'000);  // woke 25 ms late, 1 ms of work
  EXPECT_EQ(log.overrun_count(), 1u);
  EXPECT_EQ(log.late_tick_count(), 1u);
  log.Drain(500'000);
  EXPECT_TRUE(lines.empty()) << "a second summary within the same second";
  log.Drain(1'000'000);
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_NE(lines[0].find("1 overruns"), std::string::npos) << lines[0];
  EXPECT_NE(lines[0].find("1 late ticks"), std::string::npos) << lines[0];
  EXPECT_NE(lines[0].find("max work 12000 us, max interval 30000 us"),
            std::string::npos)
      << lines[0];

  // The next window reports its own maxima, not the earlier ones.
  log.RecordTick(20'000, 6'000, 5'000);
  log.Drain(2'000'000);
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_NE(lines[1].find("max work 6000 us, max interval 20000 us"),
            std::string::npos)
      << lines[1];
}

// Where a tick's backend time goes, the measurement TODO.md RT-5 sizes a
// robot from: each 10 s window reports every phase's mean and longest time
// per tick over that window's ticks only, with the device counts.
TEST(RioLog, TickPhasesReportEachWindowsMeanAndMaxThenReset) {
  using talos::rio::TickPhase;
  std::vector<std::string> lines;
  RioLog log{[&](std::string_view line) { lines.emplace_back(line); }};
  log.Drain(0);  // starts the window clock

  log.SetTickDevices(16, 4);
  log.AddPhase(TickPhase::kRefresh, 1'000);
  log.AddPhase(TickPhase::kControl, 3'000);
  log.AddPhase(TickPhase::kHealth, 500);
  log.AddPhase(TickPhase::kHealth, 500);  // phases add within a tick
  log.RecordTick(0, 4'000, 5'000);
  log.AddPhase(TickPhase::kRefresh, 2'000);
  log.AddPhase(TickPhase::kDevices, 7'000);
  log.AddPhase(TickPhase::kOther, 400);
  log.RecordTick(5'000, 4'000, 5'000);

  log.Drain(9'999'999);
  log.Drain(RioLog::kPhaseWindowUs);  // asks the loop for the window
  EXPECT_TRUE(lines.empty());
  // The next tick hands it over and opens the next window.
  log.AddPhase(TickPhase::kRefresh, 9'000);
  log.RecordTick(5'000, 4'000, 5'000);
  log.Drain(RioLog::kPhaseWindowUs + 20'000);
  ASSERT_EQ(lines.size(), 1u);
  for (const char* expected :
       {"refresh mean/max 1.5/2.0 us", "devices 3.5/7.0 us",
        "control 1.5/3.0 us", "health 0.5/1.0 us", "other 0.2/0.4 us",
        "over 2 ticks", "(motors=16 sensors=4)"}) {
    EXPECT_NE(lines[0].find(expected), std::string::npos)
        << "expected \"" << expected << "\" in: " << lines[0];
  }

  // The next window holds only the handover tick: nothing carried over.
  log.Drain(2 * RioLog::kPhaseWindowUs + 20'000);
  log.RecordTick(5'000, 4'000, 5'000);
  log.Drain(2 * RioLog::kPhaseWindowUs + 40'000);
  ASSERT_EQ(lines.size(), 2u);
  for (const char* expected :
       {"refresh mean/max 9.0/9.0 us", "devices 0.0/0.0 us",
        "control 0.0/0.0 us", "over 1 ticks"}) {
    EXPECT_NE(lines[1].find(expected), std::string::npos)
        << "expected \"" << expected << "\" in: " << lines[1];
  }
}

// A window in which the loop never ticked prints no phase line: neither while
// the loop does not answer, nor once its first tick hands over the empty
// window.
TEST(RioLog, TickPhasesWindowWithoutTicksPrintsNothing) {
  std::vector<std::string> lines;
  RioLog log{[&](std::string_view line) { lines.emplace_back(line); }};
  log.Drain(0);
  log.Drain(RioLog::kPhaseWindowUs);
  log.Drain(RioLog::kPhaseWindowUs + 20'000);
  EXPECT_TRUE(lines.empty()) << lines.front();
  log.RecordTick(0, 4'000, 5'000);
  log.Drain(RioLog::kPhaseWindowUs + 40'000);
  EXPECT_TRUE(lines.empty()) << lines.front();
}

}  // namespace

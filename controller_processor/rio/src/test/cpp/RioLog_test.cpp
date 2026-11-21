// The RoboRIO loop thread's reports: posting never waits on the logger's
// output, a full queue drops and counts, and a toggling fault is coalesced to
// one line per second for that motor.
#include "RioLog.h"

#include <gtest/gtest.h>

#include <algorithm>
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
  // The phase line, then the latency line beside it.
  ASSERT_EQ(lines.size(), 2u);
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
  ASSERT_EQ(lines.size(), 4u);
  for (const char* expected :
       {"refresh mean/max 9.0/9.0 us", "devices 0.0/0.0 us",
        "control 0.0/0.0 us", "over 1 ticks"}) {
    EXPECT_NE(lines[2].find(expected), std::string::npos)
        << "expected \"" << expected << "\" in: " << lines[2];
  }
}

// Bench 2026-11-13: when the RoboRIO's address returned, a tick's work was
// 9419 us while every phase in the window stayed under 0.3 ms; the time was
// in a part of the tick no phase measured. The loop's own phases take what
// lies between their boundaries less the backend's phases nested inside, so
// a tick's phases add up to its work: here 200 + 150 + 100 + 9150 + 100 us
// = 9700 us, the link holding the endpoint tick's time outside the backend.
// REQ-OPS-29
TEST(RioLog, TickPhasesSumToTheTicksWork) {
  using talos::rio::TickPhase;
  std::vector<std::string> lines;
  RioLog log{[&](std::string_view line) { lines.emplace_back(line); }};
  log.Drain(0);  // starts the window clock

  constexpr uint64_t kStartNs = 7'000'000'000;
  log.BeginTick(kStartNs);
  log.ClosePhase(TickPhase::kDriverStation, kStartNs + 200'000);
  // The endpoint's tick, with the backend's phases inside it.
  log.AddPhase(TickPhase::kRefresh, 150'000);
  log.AddPhase(TickPhase::kControl, 100'000);
  log.ClosePhase(TickPhase::kLink, kStartNs + 9'600'000);
  log.ClosePhase(TickPhase::kLoop, kStartNs + 9'700'000);
  log.RecordTick(0, 9'700, 5'000);

  log.Drain(RioLog::kPhaseWindowUs);  // asks the loop for the window
  log.BeginTick(kStartNs + 10'000'000);
  log.RecordTick(10'000, 0, 5'000);  // hands it over
  log.Drain(RioLog::kPhaseWindowUs + 20'000);
  // Besides the loop timing line the overrun prints.
  const auto phases =
      std::find_if(lines.begin(), lines.end(), [](const std::string& line) {
        return line.find("[rio] tick phases:") != std::string::npos;
      });
  ASSERT_NE(phases, lines.end());
  for (const char* expected :
       {"refresh mean/max 150.0/150.0 us", "control 100.0/100.0 us",
        "link 9150.0/9150.0 us", "driver station 200.0/200.0 us",
        "loop 100.0/100.0 us", "over 1 ticks"}) {
    EXPECT_NE(phases->find(expected), std::string::npos)
        << "expected \"" << expected << "\" in: " << *phases;
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

// The RoboRIO's latency line (REQ-RT-25), one window's lines as the logger
// writes them: the window is the phase window, handed over the same way.
std::vector<std::string> LatencyLines(const std::vector<std::string>& lines) {
  std::vector<std::string> latency;
  for (const auto& line : lines) {
    if (line.find("[rio] latency: ") != std::string::npos) {
      latency.push_back(line);
    }
  }
  return latency;
}

// Each 10 s window prints, per figure, the p50 and p99 of its samples to the
// 100 us bucket (never above the max), the exact max and the count: here
// accepted ages 99, 199, ..., 9999 us, one refused age past the 51.1 ms the
// buckets cover, and a Driver Station age every fourth tick. A sample joins
// the window of the tick that records it, as the tick's phases do, so the
// handover tick's sample opens the next window, which holds nothing else.
// REQ-RT-25
TEST(RioLog, LatencyReportsEachWindowsPercentilesMaxAndCountThenResets) {
  std::vector<std::string> lines;
  RioLog log{[&](std::string_view line) { lines.emplace_back(line); }};
  log.Drain(0);  // starts the window clock
  for (uint64_t i = 1; i <= 100; ++i) {
    log.RecordCommandAge(i * 100 - 1, true);
    if (i % 4 == 0) log.RecordDriverStationAge(20'000 + i);
    if (i == 50) log.RecordCommandAge(60'123, false);
    log.RecordTick(5'000, 1'000, 5'000);
  }
  log.Drain(RioLog::kPhaseWindowUs);  // asks the loop for the window
  log.RecordCommandAge(4'321, true);
  log.RecordTick(5'000, 1'000, 5'000);  // hands it over
  log.Drain(RioLog::kPhaseWindowUs + 20'000);
  auto latency = LatencyLines(lines);
  ASSERT_EQ(latency.size(), 1u);
  EXPECT_EQ(latency[0],
            "[rio] latency: command age accepted p50/p99/max 5000/9900/9999 "
            "us over 100, refused 60123/60123/60123 us over 1; driver station "
            "refresh to send 20100/20100/20100 us over 25\n");

  log.Drain(2 * RioLog::kPhaseWindowUs + 20'000);
  log.RecordTick(5'000, 1'000, 5'000);
  log.Drain(2 * RioLog::kPhaseWindowUs + 40'000);
  latency = LatencyLines(lines);
  ASSERT_EQ(latency.size(), 2u);
  EXPECT_EQ(latency[1],
            "[rio] latency: command age accepted p50/p99/max 4321/4321/4321 "
            "us over 1, refused -/-/- us over 0; driver station refresh to "
            "send -/-/- us over 0\n");
}

// An age is a difference on the RoboRIO's clock: an observation ahead of
// that clock (one from before an OS reboot, a corrupt stamp) wraps to an age
// of 2^63 us or more, which is no age and is not counted, rather than a max
// of centuries. (A previous program run's command lies in the past of the
// same clock and counts.)
// REQ-RT-25
TEST(RioLog, LatencyIgnoresAnAgeThatWrapped) {
  std::vector<std::string> lines;
  RioLog log{[&](std::string_view line) { lines.emplace_back(line); }};
  log.Drain(0);
  const uint64_t now_us = 1'000'000;
  log.RecordCommandAge(now_us - (now_us + 1), false);
  log.RecordDriverStationAge(now_us - (now_us + 5'000));
  log.RecordTick(5'000, 1'000, 5'000);
  log.Drain(RioLog::kPhaseWindowUs);
  log.RecordTick(5'000, 1'000, 5'000);
  log.Drain(RioLog::kPhaseWindowUs + 20'000);
  const auto latency = LatencyLines(lines);
  ASSERT_EQ(latency.size(), 1u);
  EXPECT_NE(latency[0].find("refused -/-/- us over 0"), std::string::npos)
      << latency[0];
  EXPECT_NE(latency[0].find("refresh to send -/-/- us over 0"),
            std::string::npos)
      << latency[0];
}

// The 1 s loop timing summary names the exact 8-phase split of the single
// worst-work tick in that window (never the per-phase max or sum across
// different ticks), and resets across windows.
// REQ-OPS-29
TEST(RioLog, LoopTimingCarriesWorstTicksOwnPhaseSplitAndResets) {
  using talos::rio::TickPhase;
  std::vector<std::string> lines;
  RioLog log{[&](std::string_view line) { lines.emplace_back(line); }};
  log.Drain(0);

  // Tick 1: moderate work (3000 us), with a large devices phase (2500 us).
  log.AddPhase(TickPhase::kRefresh, 500'000);
  log.AddPhase(TickPhase::kDevices, 2'500'000);
  log.RecordTick(5'000, 3'000, 5'000);

  // Tick 2: worst work in window (12000 us, overrun), with smaller devices
  // (200 us) than Tick 1 and distinct values across all 8 phases.
  log.AddPhase(TickPhase::kRefresh, 1'100'000);
  log.AddPhase(TickPhase::kDevices, 200'000);
  log.AddPhase(TickPhase::kControl, 300'000);
  log.AddPhase(TickPhase::kHealth, 400'000);
  log.AddPhase(TickPhase::kOther, 500'000);
  log.AddPhase(TickPhase::kLink, 8'000'000);
  log.AddPhase(TickPhase::kDriverStation, 700'000);
  log.AddPhase(TickPhase::kLoop, 800'000);
  log.RecordTick(5'000, 12'000, 5'000);

  // Tick 3: shorter work (4000 us, late tick), with a larger control phase
  // (3900 us) than Tick 2; must not overwrite Tick 2's snapshot.
  log.AddPhase(TickPhase::kControl, 3'900'000);
  log.AddPhase(TickPhase::kLoop, 100'000);
  log.RecordTick(30'000, 4'000, 5'000);

  log.Drain(1'000'000);
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_NE(
      lines[0].find(
          "max work 12000 us, max interval 30000 us, period 5000 us; "
          "worst tick phases: refresh 1100.0 us, devices 200.0 us, "
          "control 300.0 us, health 400.0 us, other 500.0 us, link 8000.0 us, "
          "driver station 700.0 us, loop 800.0 us\n"),
      std::string::npos)
      << lines[0];

  // Next 1 s window: a smaller overrun (6000 us) must replace the previous
  // window's worst tick rather than carrying 12000 us or its phases forward.
  log.AddPhase(TickPhase::kRefresh, 100'000);
  log.AddPhase(TickPhase::kControl, 5'500'000);
  log.AddPhase(TickPhase::kLoop, 400'000);
  log.RecordTick(5'000, 6'000, 5'000);

  log.Drain(2'000'000);
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_NE(lines[1].find(
                "max work 6000 us, max interval 5000 us, period 5000 us; "
                "worst tick phases: refresh 100.0 us, devices 0.0 us, "
                "control 5500.0 us, health 0.0 us, other 0.0 us, link 0.0 us, "
                "driver station 0.0 us, loop 400.0 us\n"),
            std::string::npos)
      << lines[1];
}

// Runs shorter than the 10 s phase window emit their final partial tick phases
// and latency lines on Flush() and on Stop(), then reset so a second flush
// emits nothing.
// REQ-OPS-29
TEST(RioLog, FlushAndStopEmitFinalPartialPhaseAndLatencyWindow) {
  using talos::rio::TickPhase;
  {
    std::vector<std::string> lines;
    RioLog log{[&](std::string_view line) { lines.emplace_back(line); }};
    log.Drain(0);
    log.SetTickDevices(4, 2);
    log.AddPhase(TickPhase::kRefresh, 100'000);
    log.AddPhase(TickPhase::kLink, 300'000);
    log.RecordCommandAge(1'200, true);
    log.RecordDriverStationAge(4'500);
    log.RecordTick(0, 400, 5'000);

    log.AddPhase(TickPhase::kRefresh, 200'000);
    log.AddPhase(TickPhase::kLoop, 50'000);
    log.RecordCommandAge(9'000, false);
    log.RecordTick(5'000, 250, 5'000);

    log.Drain(5'000'000);  // < 10 s: nothing emitted yet
    EXPECT_TRUE(lines.empty());

    log.Flush(5'000'000);
    ASSERT_EQ(lines.size(), 2u);
    for (const char* expected :
         {"[rio] tick phases: refresh mean/max 150.0/200.0 us",
          "link 150.0/300.0 us", "loop 25.0/50.0 us",
          "over 2 ticks (motors=4 sensors=2)"}) {
      EXPECT_NE(lines[0].find(expected), std::string::npos)
          << "expected \"" << expected << "\" in: " << lines[0];
    }
    EXPECT_EQ(lines[1],
              "[rio] latency: command age accepted p50/p99/max 1200/1200/1200 "
              "us over 1, refused 9000/9000/9000 us over 1; driver station "
              "refresh to send 4500/4500/4500 us over 1\n");

    // Reset after flushing: a second flush emits nothing.
    log.Flush(6'000'000);
    EXPECT_EQ(lines.size(), 2u);
  }

  // Real logger thread: Stop()'s release/acquire handshake makes the loop
  // thread's last ticks and latency samples visible to Flush() without races.
  {
    std::mutex mutex;
    std::vector<std::string> lines;
    RioLog log{[&](std::string_view line) {
      std::lock_guard lock{mutex};
      lines.emplace_back(line);
    }};
    log.Start();
    log.SetTickDevices(2, 1);
    for (int i = 0; i < 4; ++i) {
      log.AddPhase(TickPhase::kRefresh, 250'000);
      log.AddPhase(TickPhase::kControl, 150'000);
      log.RecordCommandAge(2'000, true);
      log.RecordDriverStationAge(3'000);
      log.RecordTick(5'000, i == 2 ? 6'000 : 1'000, 5'000);
    }
    log.Stop();

    bool has_phases = false;
    bool has_latency = false;
    bool has_worst = false;
    for (const auto& line : lines) {
      if (line.find("[rio] tick phases:") != std::string::npos &&
          line.find("over 4 ticks (motors=2 sensors=1)") != std::string::npos) {
        has_phases = true;
      }
      if (line.find("[rio] latency: command age accepted p50/p99/max "
                    "2000/2000/2000 us over 4") != std::string::npos &&
          line.find(
              "driver station refresh to send 3000/3000/3000 us over 4") !=
              std::string::npos) {
        has_latency = true;
      }
      if (line.find("[rio] loop timing:") != std::string::npos &&
          line.find("; worst tick phases: refresh 250.0 us, devices 0.0 us, "
                    "control 150.0 us,") != std::string::npos) {
        has_worst = true;
      }
    }
    EXPECT_TRUE(has_phases) << "Stop() did not emit final partial tick phases";
    EXPECT_TRUE(has_latency) << "Stop() did not emit final partial latency";
    EXPECT_TRUE(has_worst) << "Stop() did not emit loop timing worst phases";
  }
}

}  // namespace

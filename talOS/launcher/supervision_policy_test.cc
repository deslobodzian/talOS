// The supervisor's restart and kill decisions as functions of time: every
// case steps a simulated `now` (or a scripted clock) by hand from
// SupervisionTime{} + 1h, with the limits the launcher builds from
// LauncherOptions{} and kDefaultMaxRestarts, so a production constant that
// moves turns a case red. No file, process or real clock is touched.

#include "talOS/launcher/supervision_policy.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/launcher/roster.h"
#include "talOS/launcher/session.h"

namespace talos::launcher {
namespace {

using namespace std::chrono_literals;
using std::chrono::milliseconds;

constexpr SupervisionTime kT0 = SupervisionTime{} + 1h;

// What Launcher::Run builds for a session without --max-restarts.
RestartLimits SessionRestartLimits(int max_restarts) {
  const LauncherOptions options;
  return {max_restarts, milliseconds{options.restart_window_ms}};
}

// What Launcher::PrepareLaunch builds for a member at the default 5 ms period.
LivenessLimits SessionLivenessLimits() {
  const LauncherOptions options;
  return {HeartbeatTimeout(5000, options.heartbeat_periods),
          milliseconds{options.heartbeat_startup_grace_ms},
          options.construction_timeout};
}

std::string At(SupervisionTime t) {
  return "t = T0 + " +
         std::to_string(
             std::chrono::duration_cast<std::chrono::nanoseconds>(t - kT0)
                 .count()) +
         " ns";
}

// REQ-SAFE-29
TEST(SupervisionPolicy, RestartBackoffSkipsACriticalFirstCrashThenDoublesTo1s) {
  // RestartBackoff(restart, clean_exit, critical).
  // A critical member's first crash in the window restarts it at once...
  EXPECT_EQ(RestartBackoff(1, false, true), milliseconds{0});
  // ...but not its clean exit, which waits for a completing roster, nor a
  // non-critical member's crash.
  EXPECT_EQ(RestartBackoff(1, true, true), milliseconds{50});
  EXPECT_EQ(RestartBackoff(1, false, false), milliseconds{50});
  // A crash loop still backs off from the second restart on, capped at 1 s.
  EXPECT_EQ(RestartBackoff(2, false, true), milliseconds{100});
  EXPECT_EQ(RestartBackoff(3, false, true), milliseconds{200});
  EXPECT_EQ(RestartBackoff(4, false, false), milliseconds{400});
  EXPECT_EQ(RestartBackoff(5, true, true), milliseconds{800});
  EXPECT_EQ(RestartBackoff(6, false, true), milliseconds{1000});
  EXPECT_EQ(RestartBackoff(1000, false, true), milliseconds{1000});
}

// REQ-SAFE-29, REQ-OPS-04
TEST(SupervisionPolicy, ShouldRestartNodePredicate) {
  // A crash restarts a critical and a non-critical member alike.
  EXPECT_TRUE(ShouldRestartNode(false, true, 5, false));
  EXPECT_TRUE(ShouldRestartNode(false, false, 5, false));
  // A critical member exiting 0 while its session runs on is lost, not done.
  EXPECT_TRUE(ShouldRestartNode(true, true, 5, false));
  // A non-critical member exiting 0 finished.
  EXPECT_FALSE(ShouldRestartNode(true, false, 5, false));
  // Zero is a deliberate permanent stop, not a quarantine.
  EXPECT_FALSE(ShouldRestartNode(false, true, 0, false));
  EXPECT_FALSE(ShouldRestartNode(true, true, 0, false));
  // A stopping session (signal, deadline, shutdown) restarts nothing.
  EXPECT_FALSE(ShouldRestartNode(false, true, 5, true));
  EXPECT_FALSE(ShouldRestartNode(true, true, 5, true));
}

// A critical member past its budget keeps restarting on the capped backoff:
// 1000 runs that each crash as they start, far past max_restarts = 1, are
// never quarantined, and wait 0, 100, 200, 400, 800, then 1000 ms each.
// REQ-SAFE-29
TEST(SupervisionPolicy, CriticalCrashLoopIsNeverQuarantined) {
  const RestartLimits limits = SessionRestartLimits(1);
  RestartSchedule schedule{/*critical=*/true};
  const std::vector<milliseconds> first{0ms, 100ms, 200ms, 400ms, 800ms};
  SupervisionTime now = kT0;
  for (std::size_t cycle = 0; cycle < 1000; ++cycle) {
    SCOPED_TRACE("cycle " + std::to_string(cycle) + ", " + At(now));
    schedule.RunEnded(/*clean=*/false);
    const milliseconds delay = schedule.Schedule(now, limits);
    ASSERT_FALSE(schedule.quarantined());
    ASSERT_EQ(delay, cycle < first.size() ? first[cycle] : 1000ms);
    now = schedule.due_at();
    ASSERT_TRUE(schedule.Due(now));
    schedule.Started(now);
  }
}

// A non-critical member that crashes as it starts restarts max_restarts times
// on the backoff, then is quarantined until its oldest start leaves the
// 60 s window, plus the next backoff.
// REQ-OPS-04
TEST(SupervisionPolicy, CrashingNodeQuarantinesAtTheRollingLimit) {
  const RestartLimits limits = SessionRestartLimits(kDefaultMaxRestarts);
  RestartSchedule schedule{/*critical=*/false};
  SupervisionTime now = kT0;
  SupervisionTime first_start{};
  const std::vector<milliseconds> backoffs{50ms, 100ms, 200ms, 400ms, 800ms};
  ASSERT_EQ(backoffs.size(), static_cast<std::size_t>(kDefaultMaxRestarts));
  for (std::size_t restart = 0; restart < backoffs.size(); ++restart) {
    SCOPED_TRACE("restart " + std::to_string(restart + 1) + ", " + At(now));
    schedule.RunEnded(/*clean=*/false);
    EXPECT_EQ(schedule.Schedule(now, limits), backoffs[restart]);
    EXPECT_FALSE(schedule.quarantined());
    now = schedule.due_at();
    if (restart == 0) first_start = now;
    schedule.Started(now);
  }
  schedule.RunEnded(/*clean=*/false);
  const milliseconds delay = schedule.Schedule(now, limits);
  EXPECT_TRUE(schedule.quarantined());
  EXPECT_EQ(schedule.due_at(), first_start + 60s + 1000ms) << At(now);
  EXPECT_EQ(now + delay, schedule.due_at());
  EXPECT_FALSE(schedule.Due(schedule.due_at() - 1ns));
  EXPECT_TRUE(schedule.Due(schedule.due_at()));
}

// The quarantine's one retry clears it; a crash of that retry inside the new
// window is quarantined again until the retry leaves it, not retried at once.
// REQ-OPS-04
TEST(SupervisionPolicy, ExhaustedBudgetRetriesOnceAfterTheWindowAndRecovers) {
  const RestartLimits limits = SessionRestartLimits(1);
  RestartSchedule schedule{/*critical=*/false};
  schedule.RunEnded(/*clean=*/false);
  EXPECT_EQ(schedule.Schedule(kT0, limits), 50ms);
  const SupervisionTime first_start = schedule.due_at();
  schedule.Started(first_start);

  schedule.RunEnded(/*clean=*/false);
  schedule.Schedule(first_start + 10ms, limits);
  ASSERT_TRUE(schedule.quarantined());
  ASSERT_EQ(schedule.due_at(), first_start + 60s + 100ms);

  const SupervisionTime retry = schedule.due_at();
  schedule.Started(retry);
  EXPECT_FALSE(schedule.quarantined());
  EXPECT_FALSE(schedule.pending());

  schedule.RunEnded(/*clean=*/false);
  schedule.Schedule(retry + 10ms, limits);
  EXPECT_TRUE(schedule.quarantined());
  EXPECT_EQ(schedule.due_at(), retry + 60s + 100ms);
}

// A start counts against the budget for exactly the window: a crash 60 s
// after it is a first restart again, one 1 ns sooner is quarantined.
// REQ-OPS-04
TEST(SupervisionPolicy, RestartsOutsideRollingWindowDoNotExhaustBudget) {
  const RestartLimits limits = SessionRestartLimits(1);
  RestartSchedule started{/*critical=*/false};
  started.RunEnded(/*clean=*/false);
  started.Schedule(kT0, limits);
  const SupervisionTime start = started.due_at();
  started.Started(start);
  started.RunEnded(/*clean=*/false);

  RestartSchedule outside = started;
  EXPECT_EQ(outside.Schedule(start + 60s, limits), 50ms);
  EXPECT_FALSE(outside.quarantined());

  RestartSchedule inside = started;
  inside.Schedule(start + 60s - 1ns, limits);
  EXPECT_TRUE(inside.quarantined());
  EXPECT_EQ(inside.due_at(), start + 60s + 100ms);
}

// Over two simulated hours of runs of random length (half of them crashing
// as they start), a non-critical member never has more than max_restarts
// starts in any 60 s window, however late each pass takes its due start.
// TALOS_SUPERVISION_SEED sets the seed (default 0).
// REQ-OPS-04
TEST(SupervisionPolicy, NonCriticalRestartsNeverExceedTheBudgetInAnyWindow) {
  const char* env = std::getenv("TALOS_SUPERVISION_SEED");
  const uint64_t seed = env == nullptr ? 0 : std::strtoull(env, nullptr, 10);
  for (const int max_restarts : {1, 2, 5}) {
    SCOPED_TRACE("seed " + std::to_string(seed) + ", max_restarts " +
                 std::to_string(max_restarts));
    std::mt19937_64 random{seed};
    std::uniform_int_distribution<int64_t> run_ms{0, 90'000};
    std::uniform_int_distribution<int64_t> pass_lateness_ms{0, 20};
    std::bernoulli_distribution crashes_at_start{0.5};
    const RestartLimits limits = SessionRestartLimits(max_restarts);
    RestartSchedule schedule{/*critical=*/false};
    std::vector<SupervisionTime> starts;
    SupervisionTime now = kT0;
    while (now < kT0 + 2h) {
      now += crashes_at_start(random) ? 0ms : milliseconds{run_ms(random)};
      schedule.RunEnded(/*clean=*/false);
      schedule.Schedule(now, limits);
      now = schedule.due_at() + milliseconds{pass_lateness_ms(random)};
      ASSERT_TRUE(schedule.Due(now));
      schedule.Started(now);
      starts.push_back(now);
      const auto in_window =
          std::count_if(starts.begin(), starts.end(), [&](SupervisionTime s) {
            return s > now - limits.window && s <= now;
          });
      ASSERT_LE(in_window, max_restarts)
          << "start " << starts.size() << " at " << At(now);
    }
    // The walk spent the budget, so the bound was exercised.
    EXPECT_GT(starts.size(), 100u);
  }
}

// A constructing run that keeps heartbeating is killed at the 30 s
// construction deadline, measured from its spawn, not before.
// REQ-OPS-05
TEST(SupervisionPolicy, ConstructionPastThirtySecondsIsKilled) {
  const LivenessLimits limits = SessionLivenessLimits();
  RunLiveness run;
  const SupervisionTime spawned = kT0;
  run.Begin(spawned);
  RowObservation row{.found = true, .constructing = true};
  for (SupervisionTime now = spawned + 10ms; now < spawned + 30s; now += 10ms) {
    row.heartbeat += 10'000'000;
    ASSERT_EQ(run.Judge(row, now, limits), std::string_view{}) << At(now);
  }
  row.heartbeat += 10'000'000;
  EXPECT_EQ(run.Judge(row, spawned + 30s - 1ns, limits), std::string_view{});
  row.heartbeat += 10'000'000;
  EXPECT_EQ(run.Judge(row, spawned + 30s, limits), kRunConstructionDeadline);
}

// While it constructs a run has no loop, so its heartbeat is its progress:
// it outlives the 5 s startup grace on its heartbeat alone, and is killed as
// stalled one heartbeat timeout after that stops.
// REQ-OPS-05
TEST(SupervisionPolicy,
     ConstructingHeartbeatKeepsARunAlivePastTheStartupGrace) {
  const LivenessLimits limits = SessionLivenessLimits();
  ASSERT_EQ(limits.heartbeat_timeout, 500ms);
  RunLiveness run;
  run.Begin(kT0);
  RowObservation row{.found = true, .constructing = true};
  SupervisionTime now = kT0;
  for (; now < kT0 + 6s; now += 10ms) {
    row.heartbeat += 10'000'000;
    ASSERT_EQ(run.Judge(row, now, limits), std::string_view{}) << At(now);
  }
  const SupervisionTime last_beat = now - 10ms;
  EXPECT_EQ(run.Judge(row, last_beat + 500ms - 1ns, limits),
            std::string_view{});
  EXPECT_EQ(run.Judge(row, last_beat + 500ms, limits), kRunStalled);
}

// A running loop whose dispatch count stops is stalled though the reporting
// thread's heartbeat still moves: killed one heartbeat timeout after its last
// dispatch, not 1 ns sooner.
// REQ-OPS-05
TEST(SupervisionPolicy, StalledLoopIsKilledThoughItsHeartbeatMoves) {
  const LivenessLimits limits = SessionLivenessLimits();
  ASSERT_EQ(limits.heartbeat_timeout, 500ms);
  RunLiveness run;
  run.Begin(kT0);
  RowObservation row{.found = true};
  SupervisionTime now = kT0;
  for (; now < kT0 + 6s; now += 10ms) {
    row.heartbeat += 10'000'000;
    ++row.dispatch;
    ASSERT_EQ(run.Judge(row, now, limits), std::string_view{}) << At(now);
  }
  const SupervisionTime last_dispatch = now - 10ms;
  for (; now < last_dispatch + 500ms; now += 10ms) {
    row.heartbeat += 10'000'000;
    ASSERT_EQ(run.Judge(row, now, limits), std::string_view{}) << At(now);
  }
  row.heartbeat += 10'000'000;
  EXPECT_EQ(run.Judge(row, last_dispatch + 500ms - 1ns, limits),
            std::string_view{});
  row.heartbeat += 10'000'000;
  EXPECT_EQ(run.Judge(row, last_dispatch + 500ms, limits), kRunStalled);
}

// The launcher's own stall inside a pass -- after it samples the clock and
// before it reads a row (its stderr behind a full pipe, a page fault, the
// host descheduling it) -- never counts against a run that dispatched
// throughout, even when the next pass follows at once. A run whose dispatch
// count then really stops is still killed one heartbeat timeout after the
// last read that saw it move, not 1 ns sooner.
// REQ-OPS-05
TEST(SupervisionPolicy, LaunchersOwnStallInsideAPassIsNotARunStall) {
  const LivenessLimits limits = SessionLivenessLimits();
  ASSERT_EQ(limits.heartbeat_timeout, 500ms);
  SupervisionTime now = kT0;
  const SupervisionClock clock = [&now] { return now; };
  RowObservation row{.found = true};
  const auto read = [&row] { return row; };
  RunLiveness run;
  run.Begin(kT0);
  for (; now < kT0 + 6s; now += 10ms) {
    ++row.dispatch;
    ASSERT_EQ(run.Watch(clock, read, limits).kill, std::string_view{})
        << At(now);
  }
  // The pass samples the clock, then stalls 600 ms before the row is read;
  // the run dispatched all along.
  const auto read_after_stall = [&] {
    now += 600ms;
    row.dispatch += 60;
    return row;
  };
  EXPECT_EQ(run.Watch(clock, read_after_stall, limits).kill,
            std::string_view{});
  const SupervisionTime last_moved_read = now;
  // The next pass follows 1 ms later, before the run's next dispatch.
  now += 1ms;
  EXPECT_EQ(run.Watch(clock, read, limits).kill, std::string_view{}) << At(now);
  // From here the dispatch count does not move.
  for (; now < last_moved_read + 500ms; now += 10ms) {
    ASSERT_EQ(run.Watch(clock, read, limits).kill, std::string_view{})
        << At(now);
  }
  now = last_moved_read + 500ms - 1ns;
  EXPECT_EQ(run.Watch(clock, read, limits).kill, std::string_view{});
  now = last_moved_read + 500ms;
  EXPECT_EQ(run.Watch(clock, read, limits).kill, kRunStalled);
}

// The converse: a loop that keeps dispatching while its reporting thread is
// starved and stops heartbeating is making progress, and is not killed.
// REQ-OPS-05
TEST(SupervisionPolicy, DispatchingLoopIsNotKilledWhenItsHeartbeatStops) {
  const LivenessLimits limits = SessionLivenessLimits();
  RunLiveness run;
  run.Begin(kT0);
  RowObservation row{.found = true, .heartbeat = 1};
  for (SupervisionTime now = kT0; now < kT0 + 10s; now += 10ms) {
    ++row.dispatch;
    ASSERT_EQ(run.Judge(row, now, limits), std::string_view{}) << At(now);
  }
}

// A run with no registry row is killed as heartbeat_missing once both the
// startup grace and the heartbeat timeout since its spawn have passed.
// REQ-OPS-05
TEST(SupervisionPolicy, UnregisteredRunIsKilledOnceTheGraceAndTimeoutPass) {
  const RowObservation none;
  LivenessLimits limits = SessionLivenessLimits();
  ASSERT_EQ(limits.startup_grace, 5s);
  RunLiveness run;
  run.Begin(kT0);
  EXPECT_EQ(run.Judge(none, kT0 + 5s - 1ns, limits), std::string_view{});
  EXPECT_EQ(run.Judge(none, kT0 + 5s, limits), kRunHeartbeatMissing);

  limits.startup_grace = 0ms;
  RunLiveness ungraced;
  ungraced.Begin(kT0);
  EXPECT_EQ(ungraced.Judge(none, kT0 + 500ms - 1ns, limits),
            std::string_view{});
  EXPECT_EQ(ungraced.Judge(none, kT0 + 500ms, limits), kRunHeartbeatMissing);
}

// What the supervision loop's looks at one run decided, and when.
struct Killed {
  std::string_view reason;
  SupervisionTime at;
};

// Looks at `run` only when its NextLook says, from kT0, at each look reading
// the row `row_at` gives for that time, until a look kills it. Every look
// must fall after the last and at most one look interval later.
Killed LookUntilKilled(
    RunLiveness& run, const LivenessLimits& limits,
    const std::function<RowObservation(SupervisionTime)>& row_at) {
  SupervisionTime now = kT0;
  for (int looks = 0; looks < 100'000; ++looks) {
    const SupervisionTime next = run.NextLook(limits);
    if (next <= now || next - now > LivenessLookInterval(limits)) {
      ADD_FAILURE() << "after the look at " << At(now) << " the next is at "
                    << At(next);
      return {{}, now};
    }
    now = next;
    const std::string_view kill = run.Judge(row_at(now), now, limits);
    if (!kill.empty()) return {kill, now};
  }
  ADD_FAILURE() << "no look killed the run by " << At(now);
  return {{}, now};
}

// Looking only when NextLook says, never on a poll, a stalled run is killed
// at least one heartbeat timeout and at most that plus one look interval
// after its last dispatch, wherever in a look interval the stall begins.
// REQ-OPS-05
TEST(SupervisionPolicy, ScheduledLooksKillAStallWithinTheTimeoutAndOneLook) {
  const LivenessLimits limits = SessionLivenessLimits();
  const auto look = LivenessLookInterval(limits);
  for (auto offset = 0ms; offset < look; offset += 1ms) {
    // Dispatches every millisecond from kT0 through `last`.
    const SupervisionTime last = kT0 + 6s + offset;
    SCOPED_TRACE("last dispatch at " + At(last));
    RunLiveness run;
    run.Begin(kT0);
    const Killed killed = LookUntilKilled(run, limits, [&](SupervisionTime t) {
      return RowObservation{
          .found = true,
          .dispatch = static_cast<uint64_t>((std::min(t, last) - kT0) / 1ms)};
    });
    ASSERT_EQ(killed.reason, kRunStalled) << At(killed.at);
    EXPECT_GE(killed.at - last, limits.heartbeat_timeout) << At(killed.at);
    EXPECT_LE(killed.at - last, limits.heartbeat_timeout + look)
        << At(killed.at);
  }
}

// The deadlines a look decides on are look times themselves, not the next
// look interval after them: the startup grace's end for a run that never
// registered, the construction deadline for one still constructing; and a
// deadline already decided (the construction deadline of a run that has no
// row, which only the grace ends) never holds the next look back.
// REQ-OPS-05
TEST(SupervisionPolicy, ScheduledLooksFallOnTheGraceAndConstructionDeadlines) {
  const RowObservation none;
  int64_t beat = 0;
  const auto constructing = [&beat](SupervisionTime) {
    return RowObservation{
        .found = true, .constructing = true, .heartbeat = ++beat};
  };
  struct Case {
    const char* name;
    std::chrono::milliseconds grace;
    std::chrono::milliseconds construction;
    std::function<RowObservation(SupervisionTime)> row_at;
    std::string_view reason;
    std::chrono::milliseconds at;
  };
  const Case cases[] = {
      {"unregistered", 5001ms, 30s, [&](SupervisionTime) { return none; },
       kRunHeartbeatMissing, 5001ms},
      {"constructing", 5s, 30001ms, constructing, kRunConstructionDeadline,
       30001ms},
      {"unregistered past the construction deadline", 60s, 30s,
       [&](SupervisionTime) { return none; }, kRunHeartbeatMissing, 60s},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    LivenessLimits limits = SessionLivenessLimits();
    limits.startup_grace = c.grace;
    limits.construction_timeout = c.construction;
    RunLiveness run;
    run.Begin(kT0);
    const Killed killed = LookUntilKilled(run, limits, c.row_at);
    EXPECT_EQ(killed.reason, c.reason) << At(killed.at);
    EXPECT_EQ(killed.at, kT0 + c.at) << At(killed.at);
  }
}

// --heartbeat-periods periods of the member's period_us, never under 500 ms.
// REQ-OPS-05
TEST(SupervisionPolicy, HeartbeatTimeoutIsTenPeriodsWithA500MsFloor) {
  const int periods = LauncherOptions{}.heartbeat_periods;
  EXPECT_EQ(HeartbeatTimeout(5000, periods), 500ms);
  EXPECT_EQ(HeartbeatTimeout(100000, periods), 1s);
  EXPECT_EQ(HeartbeatTimeout(1000, periods), 500ms);
}

}  // namespace
}  // namespace talos::launcher

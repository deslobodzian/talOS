#include "talOS/bridge/merge.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <vector>

// The request merge rules the bridge (HardwareNode::Tick) and
// talos::testing::SimHardware both run, on an explicit clock. Seen times are
// the merging side's clock, observations the gateway's.

namespace talos::hardware {
namespace {

constexpr std::uint64_t kCommandTimeoutUs = 100000;
// A 5 ms owner under a 100 ms command timeout: every 2026-robot subsystem.
constexpr std::uint64_t kOwnerTimeoutUs = 50000;

MergeOwner Owner() {
  MergeOwner owner;
  owner.timeout_us = OwnerTimeoutUs(5000, kCommandTimeoutUs);
  return owner;
}

std::optional<std::uint64_t> Merged(const std::vector<MergeOwner>& owners) {
  return MergedObservationUs(
      owners,
      [](const MergeOwner& owner) -> const MergeOwner& { return owner; });
}

// Configuration refuses an actuator owner whose period exceeds a quarter of
// the command timeout, so every owner that reaches the bridge leaves the
// merge at exactly half the timeout: before the State its last request was
// built from reaches the gateway's limit, whenever that State was younger
// than half the timeout on admission.
// REQ-SAFE-15
TEST(MergeTest, ValidatedOwnerTimeoutIsHalfTheCommandTimeout) {
  for (std::uint64_t timeout_us = 4; timeout_us <= 500000; timeout_us += 997) {
    for (std::uint64_t period_us = 1; period_us <= timeout_us / 4;
         period_us += 113) {
      ASSERT_EQ(OwnerTimeoutUs(period_us, timeout_us), timeout_us / 2)
          << "period_us=" << period_us << " command_timeout_us=" << timeout_us;
    }
  }
}

// Outside the validated range the timeout still covers a late or missed tick
// (two periods) and never outlasts the command timeout.
// REQ-SAFE-15
TEST(MergeTest, UnvalidatedOwnerTimeoutStaysWithinTwoPeriodsAndTheTimeout) {
  struct Row {
    const char* label;
    std::uint64_t period_us;
    std::uint64_t expected_us;
  };
  for (const Row& row : {
           Row{"two periods above half the timeout", 30000, 60000},
           Row{"two periods above the timeout: clamped", 80000, 100000},
       }) {
    SCOPED_TRACE(row.label);
    EXPECT_EQ(OwnerTimeoutUs(row.period_us, kCommandTimeoutUs),
              row.expected_us);
  }
}

// A 5 ms owner that misses ticks and arrives late (12 ms gaps) for longer than
// its owner timeout stays merged, including a gap of exactly the owner
// timeout: each admitted request restarts it.
// REQ-SAFE-15
TEST(MergeTest, RequestGapsUpToTheOwnerTimeoutKeepTheOwnerMerged) {
  MergeOwner owner = Owner();
  std::uint64_t now_us = 1000000;
  for (int i = 0; i < 20; ++i) {
    owner.Accept(now_us, 1000);
    now_us += 12000;
    ASSERT_FALSE(owner.Expire(now_us))
        << "12 ms after request " << i << ", at " << now_us;
  }
  owner.Accept(now_us, 1000);
  EXPECT_FALSE(owner.Expire(now_us + kOwnerTimeoutUs))
      << "silent for exactly the owner timeout, " << kOwnerTimeoutUs << " us";
  EXPECT_FALSE(owner.timed_out);
}

// REQ-SAFE-15
TEST(MergeTest, OwnerSilentPastItsTimeoutLeavesTheMerge) {
  MergeOwner owner = Owner();
  owner.Accept(1000000, 1000);
  EXPECT_TRUE(owner.Expire(1000000 + kOwnerTimeoutUs + 1));
  EXPECT_TRUE(owner.timed_out);
  EXPECT_EQ(Merged({owner}), std::nullopt);
}

// Both stamps are gateway clock. A request built from an older sample stays
// admissible while newer samples arrive, until its age reaches the command
// timeout (the gateway's own bound).
TEST(MergeTest, RequestAgeBoundIsTheCommandTimeout) {
  const MergeOwner owner = Owner();
  constexpr std::uint64_t kLatestUs = 500000;
  struct Row {
    std::uint64_t age_us;
    bool admitted;
  };
  for (const Row& row : {Row{0, true}, Row{kCommandTimeoutUs - 1, true},
                         Row{kCommandTimeoutUs, false}}) {
    EXPECT_EQ(
        owner.Admits(kLatestUs - row.age_us, kLatestUs, kCommandTimeoutUs),
        row.admitted)
        << "observation " << row.age_us << " us older than the latest State";
  }
}

// An observation older than the owner's last admitted one is a replay,
// refused even inside the age bound; a re-publish from the same sample is
// not.
TEST(MergeTest, ObservationOlderThanTheLastAdmittedIsRefused) {
  MergeOwner owner = Owner();
  owner.Accept(1000000, 6000);
  EXPECT_FALSE(owner.Admits(5000, 6000, kCommandTimeoutUs));
  EXPECT_TRUE(owner.Admits(6000, 6000, kCommandTimeoutUs));
}

// A controller transition takes the owner out of the merge at once, without
// waiting for its owner timeout.
TEST(MergeTest, ResetTakesTheOwnerOutOfTheMergeAtOnce) {
  MergeOwner owner = Owner();
  owner.Accept(1000000, 6000);
  owner.Reset();
  EXPECT_EQ(Merged({owner}), std::nullopt);
  EXPECT_TRUE(owner.Expire(1000001));
}

// A rebooted gateway's clock restarts low: after Reset, a request built from
// the new boot's first State merges although its observation is older than
// the last one admitted before.
TEST(MergeTest, ResetAdmitsAnObservationOlderThanThePreviousBoots) {
  MergeOwner owner = Owner();
  owner.Accept(1000000, 900000);
  owner.Reset();
  EXPECT_TRUE(owner.Admits(2000, 2000, kCommandTimeoutUs));
}

// The merged command carries the oldest observation among owners still
// merged, wherever that owner sits: a fresh owner never refreshes a quiet
// one's cached demand.
// REQ-SAFE-16
TEST(MergeTest, MergedObservationIsTheOldestAmongMergedOwners) {
  std::vector<MergeOwner> owners(3, Owner());
  owners[0].Accept(1000000, 6000);
  owners[1].Accept(1000000, 5000);
  owners[2].Accept(1000000, 7000);
  EXPECT_EQ(Merged(owners), 5000u);
}

// An owner that timed out was neutralized, so its old observation stops
// holding the merged command back; with no owner merged there is none.
// REQ-SAFE-16
TEST(MergeTest, TimedOutOwnerStopsPinningTheMergedObservation) {
  std::vector<MergeOwner> owners(2, Owner());
  owners[0].Accept(1000000, 1000);
  owners[1].Accept(1040000, 6000);
  ASSERT_TRUE(owners[0].Expire(1000000 + kOwnerTimeoutUs + 1));
  ASSERT_FALSE(owners[1].Expire(1000000 + kOwnerTimeoutUs + 1));
  EXPECT_EQ(Merged(owners), 6000u);
  ASSERT_TRUE(owners[1].Expire(1040000 + kOwnerTimeoutUs + 1));
  EXPECT_EQ(Merged(owners), std::nullopt);
}

}  // namespace
}  // namespace talos::hardware

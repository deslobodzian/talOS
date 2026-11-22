#include "talOS/launcher/sleep_guard.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace talos::launcher {
namespace {

struct FakeBackend {
  bool fail_create{false};
  std::uint32_t next_id{100};
  std::vector<std::string> created_reasons;
  std::vector<std::uint32_t> created_ids;
  std::vector<std::uint32_t> released_ids;

  SleepGuard::Ops ops() {
    return SleepGuard::Ops{
        .create = &Create,
        .release = &Release,
        .context = this,
    };
  }

  static bool Create(std::string_view reason, std::uint32_t& assertion_id,
                     bool& active, std::string& error, void* context) {
    auto* self = static_cast<FakeBackend*>(context);
    self->created_reasons.emplace_back(reason);
    if (self->fail_create) {
      error = "injected IOKit failure";
      active = false;
      assertion_id = 0;
      return false;
    }
    assertion_id = self->next_id++;
    self->created_ids.push_back(assertion_id);
    active = true;
    return true;
  }

  static void Release(std::uint32_t assertion_id, void* context) noexcept {
    auto* self = static_cast<FakeBackend*>(context);
    self->released_ids.push_back(assertion_id);
  }
};

// TODO.md DEPLOY-1: RAII acquire and destruction release each assertion ID
// exactly once.
TEST(SleepGuard, AcquireAndDestroyReleasesOnce) {
  FakeBackend fake;
  std::string error = "stale";
  {
    const SleepGuard guard =
        SleepGuard::AcquireWithOps("talOS session 42", fake.ops(), &error);
    EXPECT_TRUE(guard.active());
    EXPECT_TRUE(error.empty());
    ASSERT_EQ(fake.created_reasons.size(), 1U);
    EXPECT_EQ(fake.created_reasons[0], "talOS session 42");
    EXPECT_TRUE(fake.released_ids.empty());
  }
  ASSERT_EQ(fake.created_ids.size(), 1U);
  EXPECT_EQ(fake.released_ids, fake.created_ids);
}

TEST(SleepGuard, MoveConstructTransfersOwnershipWithoutDoubleRelease) {
  FakeBackend fake;
  {
    SleepGuard first =
        SleepGuard::AcquireWithOps("session-move-ctor", fake.ops());
    ASSERT_TRUE(first.active());
    {
      const SleepGuard second{std::move(first)};
      EXPECT_TRUE(second.active());
      EXPECT_TRUE(fake.released_ids.empty());
    }
    EXPECT_EQ(fake.released_ids, (std::vector<std::uint32_t>{100}));
  }
  // Destroying the moved-from `first` must not double-release.
  EXPECT_EQ(fake.released_ids, (std::vector<std::uint32_t>{100}));
}

TEST(SleepGuard, MoveAssignReleasesPreviousAssertionAndTransfersNew) {
  FakeBackend fake;
  {
    SleepGuard target = SleepGuard::AcquireWithOps("first", fake.ops());
    SleepGuard source = SleepGuard::AcquireWithOps("second", fake.ops());
    ASSERT_TRUE(target.active());
    ASSERT_TRUE(source.active());

    target = std::move(source);
    EXPECT_TRUE(target.active());
    // Overwritten assertion (100) must be released immediately on assignment.
    EXPECT_EQ(fake.released_ids, (std::vector<std::uint32_t>{100}));

    // Assigning an inactive default guard releases the currently held one.
    target = SleepGuard{};
    EXPECT_FALSE(target.active());
    EXPECT_EQ(fake.released_ids, (std::vector<std::uint32_t>{100, 101}));
  }
  // Destroying moved-from `source` must not double-release 101.
  EXPECT_EQ(fake.released_ids, (std::vector<std::uint32_t>{100, 101}));
}

TEST(SleepGuard, ExplicitReleaseIsIdempotent) {
  FakeBackend fake;
  {
    SleepGuard guard = SleepGuard::AcquireWithOps("early-release", fake.ops());
    ASSERT_TRUE(guard.active());
    guard.Release();
    EXPECT_FALSE(guard.active());
    guard.Release();
    EXPECT_EQ(fake.released_ids, (std::vector<std::uint32_t>{100}));
  }
  EXPECT_EQ(fake.released_ids, (std::vector<std::uint32_t>{100}));
}

TEST(SleepGuard, AcquireFailureReturnsInactiveWithErrorAndSkipsRelease) {
  FakeBackend fake;
  fake.fail_create = true;
  std::string error;
  {
    const SleepGuard guard =
        SleepGuard::AcquireWithOps("failing-session", fake.ops(), &error);
    EXPECT_FALSE(guard.active());
    EXPECT_EQ(error, "injected IOKit failure");
  }
  EXPECT_TRUE(fake.released_ids.empty());
}

TEST(SleepGuard, PlatformAcquireAndReleaseLifecycle) {
  std::string error = "should-be-cleared";
  SleepGuard guard = SleepGuard::Acquire("talOS sleep_guard_test", &error);
  EXPECT_TRUE(error.empty());
#ifdef __APPLE__
  EXPECT_TRUE(guard.active());
  SleepGuard moved{std::move(guard)};
  EXPECT_TRUE(moved.active());
  moved.Release();
  EXPECT_FALSE(moved.active());
#else
  EXPECT_FALSE(guard.active());
  SleepGuard moved{std::move(guard)};
  EXPECT_FALSE(moved.active());
  moved.Release();
#endif
}

}  // namespace
}  // namespace talos::launcher

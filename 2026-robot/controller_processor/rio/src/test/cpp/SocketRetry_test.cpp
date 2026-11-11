// When the RoboRIO loop retries opening the gateway's socket: soon enough
// after the network comes up that a rebooted RoboRIO's State is back as soon
// as it can be, and seldom enough that a network that stays down costs the
// 5 ms loop nothing.
#include "SocketRetry.h"

#include <gtest/gtest.h>

#include <cstdint>

namespace {

using talos::rio::SocketRetry;

constexpr uint64_t kLoopPeriodUs = 5'000;
// The program's first tick: an arbitrary steady-clock time.
constexpr uint64_t kStartUs = 7'000'000;

// Scenario (bench, RoboRIO reboot during a session): the program starts
// before the RoboRIO's network has a route to the companion, so the first
// open fails, and the route appears 2.0013 s later, between two loop ticks.
// The socket must open, and State leave, within one retry period and one
// loop tick of the route appearing; the open reports how many attempts
// failed since when, for the console line that times the outage.
// REQ-OPS-28
TEST(SocketRetry, OpensWithinOneRetryPeriodOfTheRouteAppearing) {
  constexpr uint64_t kRouteUpUs = kStartUs + 2'001'300;
  constexpr uint64_t kBoundUs = 100'000 + kLoopPeriodUs;
  SocketRetry retry;
  uint64_t opened_us = 0;
  uint32_t attempts = 0;
  for (uint64_t now = kStartUs; now < kStartUs + 10'000'000;
       now += kLoopPeriodUs) {
    const auto event = retry.Poll(now, [&] {
      ++attempts;
      return now >= kRouteUpUs;
    });
    if (event == SocketRetry::Event::kOpened) {
      opened_us = now;
      break;
    }
  }
  ASSERT_NE(opened_us, 0u) << "never opened";
  EXPECT_LE(opened_us - kRouteUpUs, kBoundUs)
      << "opened " << opened_us - kRouteUpUs
      << " us after the route appeared; bound: one retry period (100 ms) "
         "plus one loop period (5 ms)";
  EXPECT_EQ(retry.failures(), attempts - 1);
  EXPECT_EQ(retry.first_failure_us(), kStartUs);
}

// A network that never comes up: one attempt per retry period, not one per
// loop tick, and none once the socket is open.
TEST(SocketRetry, AttemptsAtMostOncePerPeriod) {
  SocketRetry retry;
  bool up = false;
  uint32_t attempts = 0;
  uint64_t last_attempt_us = 0;
  for (uint64_t now = kStartUs; now < kStartUs + 10'000'000;
       now += kLoopPeriodUs) {
    retry.Poll(now, [&] {
      if (attempts > 0) {
        EXPECT_GE(now - last_attempt_us, SocketRetry::kPeriodUs)
            << "attempt " << attempts << " at " << now;
      }
      ++attempts;
      last_attempt_us = now;
      return up;
    });
  }
  EXPECT_EQ(attempts, 10'000'000 / SocketRetry::kPeriodUs);

  up = true;
  const uint64_t later = kStartUs + 10'000'000 + SocketRetry::kPeriodUs;
  ASSERT_EQ(retry.Poll(later, [&] { return up; }), SocketRetry::Event::kOpened);
  EXPECT_EQ(retry.Poll(later + SocketRetry::kPeriodUs,
                       [&] {
                         ADD_FAILURE() << "attempt on an open socket";
                         return true;
                       }),
            SocketRetry::Event::kNone);
}

}  // namespace

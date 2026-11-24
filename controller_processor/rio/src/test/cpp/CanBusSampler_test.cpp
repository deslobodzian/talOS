// Regression tests for CanBusSampler's background reader thread resilience
// (CanBusHealth.cpp).
#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "CanBusHealth.h"
#include "hardware/config.h"
#include "hardware/messages.h"

namespace {

namespace hw = talos::hardware;
using talos::rio::CanBusHealth;
using talos::rio::CanBusSampler;

uint64_t FixedClock() { return 7'000'000; }

hw::Config SingleMotorConfig() {
  hw::Config config;
  hw::MotorConfig motor;
  motor.id = 1;
  motor.name = "drive";
  motor.can_id = 1;
  motor.bus = "rio";
  config.motors.push_back(motor);
  return config;
}

// An exception thrown by a vendor CAN bus status read (e.g. CANBus::GetStatus)
// must be caught on the sampler thread and recorded as a failed read rather
// than escaping std::thread and calling std::terminate() mid-match.
// REQ-OPS-20
TEST(CanBusSamplerException, ThrowingReadRecordsFailureAndContinuesSampling) {
  std::mutex mutex;
  std::condition_variable cv;
  int reads = 0;
  bool allow_second = false;
  bool second_completed = false;
  std::vector<std::string> reports;

  CanBusHealth health;
  CanBusSampler sampler{
      health,
      [&](std::string_view) -> CanBusHealth::Sample {
        int call = 0;
        {
          const std::lock_guard lock{mutex};
          call = ++reads;
        }
        cv.notify_all();
        if (call == 1) {
          throw std::runtime_error("simulated vendor CAN status exception");
        }
        {
          std::unique_lock lock{mutex};
          cv.wait_for(lock, std::chrono::seconds{5},
                      [&] { return allow_second; });
          second_completed = true;
        }
        cv.notify_all();
        return {true, 0.25, 0, 0, 0, 0, "OK"};
      },
      FixedClock,
      [&](std::string_view line) {
        const std::lock_guard lock{mutex};
        reports.emplace_back(line);
      },
      /*period_us=*/100,
      /*stop_bound=*/std::chrono::seconds{5}};

  sampler.Reconfigure(SingleMotorConfig(), /*simulation=*/false);

  // Wait until the throwing first read has completed and the second read has
  // entered (proving SampleNext finished publishing the failed first read and
  // the thread survived).
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds{5}, [&] {
      return reads >= 2;
    })) << "sampler did not survive the throwing first read";
  }

  health.Poll(FixedClock(), [](const CanBusHealth::Change&) {});
  const hw::CanBusSample after_throw = health.Slot(0);
  EXPECT_EQ(after_throw.read_state, hw::CanBusReadState::kFailed);
  EXPECT_EQ(after_throw.read_failure_count, 1u);

  // Release the second read, wait for it to return, and stop cleanly.
  {
    const std::lock_guard lock{mutex};
    allow_second = true;
  }
  cv.notify_all();
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds{5},
                            [&] { return second_completed; }));
  }
  sampler.Stop();

  health.Poll(FixedClock(), [](const CanBusHealth::Change&) {});
  const hw::CanBusSample after_recover = health.Slot(0);
  EXPECT_EQ(after_recover.read_state, hw::CanBusReadState::kOk);
  EXPECT_FLOAT_EQ(after_recover.utilization_frac, 0.25f);
  EXPECT_EQ(after_recover.read_failure_count, 1u);
  EXPECT_TRUE(reports.empty()) << "sampler should have stopped cleanly";
}

}  // namespace

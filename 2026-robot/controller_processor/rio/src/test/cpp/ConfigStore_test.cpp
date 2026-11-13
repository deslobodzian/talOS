// The persisted hardware configuration. Its startup restore must not wait
// for device configuration (a mid-match reboot with absent devices would hold
// robot code start for a CAN timeout per device), and nothing drives until
// the restore completes. Its save must not hold the loop thread, and must
// leave the newest acknowledged configuration on disk.
#include "ConfigStore.h"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <new>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "hardware/gateway.h"

namespace hw = talos::hardware;

// Which thread frees a watched block, for
// ConfigPersister.TheSaverFreesNothingTheLoopAllocated. Global new and delete
// are replaced for this test binary with malloc and free (the pair the
// default ones use), so every block is freed through Freed().
namespace {
std::array<std::atomic<const void*>, 4> watched{};
std::atomic<int> freed_by_loop{0};
std::atomic<int> freed_elsewhere{0};
thread_local bool on_loop_thread = false;

void Freed(void* block) {
  if (block == nullptr) return;
  for (auto& slot : watched) {
    const void* expected = block;
    if (slot.load(std::memory_order_relaxed) == block &&
        slot.compare_exchange_strong(expected, nullptr)) {
      (on_loop_thread ? freed_by_loop : freed_elsewhere)
          .fetch_add(1, std::memory_order_relaxed);
      return;
    }
  }
}

void* Allocate(std::size_t size) {
  if (void* block = std::malloc(size == 0 ? 1 : size)) return block;
  throw std::bad_alloc{};
}
}  // namespace

// Every unaligned form, so a sanitizer's own new is never paired with free.
void* operator new(std::size_t size) { return Allocate(size); }
void* operator new[](std::size_t size) { return Allocate(size); }
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  return std::malloc(size == 0 ? 1 : size);
}
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
  return std::malloc(size == 0 ? 1 : size);
}
void operator delete(void* block, const std::nothrow_t&) noexcept {
  Freed(block);
  std::free(block);
}
void operator delete[](void* block, const std::nothrow_t&) noexcept {
  Freed(block);
  std::free(block);
}
void operator delete(void* block) noexcept {
  Freed(block);
  std::free(block);
}
void operator delete[](void* block) noexcept {
  Freed(block);
  std::free(block);
}
void operator delete(void* block, std::size_t) noexcept {
  Freed(block);
  std::free(block);
}
void operator delete[](void* block, std::size_t) noexcept {
  Freed(block);
  std::free(block);
}

namespace {

class PendingBackend final : public hw::Backend {
 public:
  bool Configure(const hw::Config&) override {
    ++blocking_configures;
    return true;
  }
  void BeginConfigure(const hw::Config&) override { ++begun; }
  ConfigureStatus PollConfigure() override {
    return done ? ConfigureStatus::kDone : ConfigureStatus::kPending;
  }
  bool Read(hw::State& state) override {
    for (auto& motor : state.motors) motor.valid = true;
    return true;
  }
  bool Apply(std::span<const hw::MotorRequest>) override {
    ++applied;
    return true;
  }
  std::span<const uint16_t> ApplyFailedDevices() const override { return {}; }
  void Neutral() override {}
  int blocking_configures{}, begun{}, applied{};
  bool done{false};
};

hw::Command DutyCycle(const hw::Gateway& gateway) {
  const auto state = gateway.snapshot();
  hw::Command command{};
  command.config_id = state.config_id;
  command.boot_id = state.boot_id;
  command.epoch = state.epoch;
  command.observed_time_us = state.sample_time_us;
  command.count = state.motor_count;
  command.motors[0] = {state.motors[0].id, hw::Mode::kDutyCycle, 0, 0.2, 0};
  return command;
}

// REQ-CONTRACT-06
TEST(ConfigStore, StartupRestoreDoesNotWaitForDevicesAndDrivesOnlyAfter) {
  hw::Config config;
  config.commissioned = true;
  hw::MotorConfig motor;
  motor.id = 1;
  motor.name = "test_motor";
  motor.can_id = 1;
  config.motors.push_back(motor);
  const auto path =
      (std::filesystem::temp_directory_path() / "talos_config_store_test.bin")
          .string();
  ASSERT_TRUE(talos::rio::SaveConfigFile(path, config));

  PendingBackend backend;
  hw::Gateway gateway{backend, 7};
  EXPECT_EQ(talos::rio::BeginRestore(gateway, path), config);
  // The blocking Configure() is never called on this thread.
  EXPECT_EQ(backend.blocking_configures, 0);
  EXPECT_EQ(backend.begun, 1);

  // A mid-match reboot: the Driver Station is enabled while devices are still
  // being configured; the gateway stays unconfigured and drives nothing.
  gateway.Tick(1000, true);
  EXPECT_FALSE(gateway.configured());
  EXPECT_EQ(backend.applied, 0);

  backend.done = true;
  gateway.Tick(6000, true);
  ASSERT_TRUE(gateway.configured());
  EXPECT_EQ(gateway.snapshot().config_id, hw::ConfigurationId(config));
  EXPECT_TRUE(gateway.Accept(DutyCycle(gateway), 2, 6000));
  EXPECT_EQ(backend.applied, 1);

  std::filesystem::remove(path);
}

// A store whose saves block until the test releases them (or a 10 s deadline
// that only bounds a hang) and record each configuration's id, in order.
class GatedStore {
 public:
  bool Save(const hw::Config& config) {
    std::unique_lock lock{mutex_};
    started_.push_back(hw::ConfigurationId(config));
    changed_.notify_all();
    changed_.wait_for(lock, kHang, [&] { return released_; });
    const bool ok = !fail_next_;
    fail_next_ = false;
    finished_.push_back(hw::ConfigurationId(config));
    return ok;
  }
  // Waits for the save of the `n`th configuration to start.
  bool AwaitStarted(std::size_t n) {
    std::unique_lock lock{mutex_};
    return changed_.wait_for(lock, kHang, [&] { return started_.size() >= n; });
  }
  void Release() {
    std::lock_guard lock{mutex_};
    released_ = true;
    changed_.notify_all();
  }
  void FailNext() {
    std::lock_guard lock{mutex_};
    fail_next_ = true;
  }
  std::vector<uint64_t> finished() {
    std::lock_guard lock{mutex_};
    return finished_;
  }

 private:
  static constexpr std::chrono::seconds kHang{10};
  std::mutex mutex_;
  std::condition_variable changed_;
  std::vector<uint64_t> started_;
  std::vector<uint64_t> finished_;
  bool released_{false};
  bool fail_next_{false};
};

hw::Config ConfigWithMotor(int can_id) {
  hw::Config config;
  hw::MotorConfig motor;
  motor.id = 1;
  motor.name = "test_motor";
  motor.can_id = can_id;
  config.motors.push_back(motor);
  return config;
}

// The save syncs storage twice; a 360 ms injected delay in desktop simulation
// held the loop thread for its whole length (`max work 360505 us`), so no
// State or Driver Station packet left and the command timeout went
// unenforced. With a save blocked, the loop's ticks go on (here 200 of them,
// a second of 5 ms ticks) and finish while it is still blocked, and the
// configurations acknowledged meanwhile reach the disk after it, newest last:
// B, handed over while A's save runs, is replaced by C before its save
// starts.
// REQ-CONTRACT-06
TEST(ConfigPersister, TicksDoNotWaitForASlowSaveAndTheNewestIsSavedLast) {
  const hw::Config a = ConfigWithMotor(1);
  const hw::Config b = ConfigWithMotor(2);
  const hw::Config c = ConfigWithMotor(3);
  GatedStore store;
  talos::rio::ConfigPersister persister{
      [&](const hw::Config& config) { return store.Save(config); }, 0};
  persister.Start();

  persister.Acknowledged(a);
  persister.Tick(true);
  ASSERT_TRUE(store.AwaitStarted(1)) << "A's save never started";

  for (int tick = 0; tick < 200; ++tick) {
    if (tick == 10) persister.Acknowledged(b);
    if (tick == 20) persister.Acknowledged(c);
    EXPECT_EQ(persister.Tick(true), 0u) << "tick " << tick;
  }
  EXPECT_TRUE(store.finished().empty())
      << "the ticks waited for a save: " << store.finished().size()
      << " finished before the test released it";

  store.Release();
  persister.Stop();
  EXPECT_EQ(store.finished(), (std::vector<uint64_t>{hw::ConfigurationId(a),
                                                     hw::ConfigurationId(c)}));
}

// A save that fails leaves the previous file; it is reported once, and the
// same configuration acknowledged again (an identical push) is saved again.
// REQ-CONTRACT-06
TEST(ConfigPersister, AFailedSaveIsReportedOnceAndAnIdenticalPushRetriesIt) {
  const hw::Config a = ConfigWithMotor(1);
  GatedStore store;
  store.FailNext();
  store.Release();
  talos::rio::ConfigPersister persister{
      [&](const hw::Config& config) { return store.Save(config); }, 0};
  persister.Start();

  persister.Acknowledged(a);
  uint32_t failures = persister.Tick(true);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds{10};
  while (failures == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
    failures += persister.Tick(true);
  }
  EXPECT_EQ(failures, 1u);
  EXPECT_EQ(persister.Tick(true), 0u) << "the failure was reported twice";

  persister.Acknowledged(a);
  persister.Tick(true);
  persister.Stop();
  EXPECT_EQ(store.finished(), (std::vector<uint64_t>{hw::ConfigurationId(a),
                                                     hw::ConfigurationId(a)}));
}

// The loop thread allocates the configuration it hands over (Acknowledged
// copies it); were the saver to free it, glibc's free, once the saver's
// tcache is full, takes the arena lock the loop thread allocates under, and
// the nice-19 saver preempted holding it stalls the loop. So the saver hands
// each saved configuration back, and the loop's next Tick() frees it: here
// the copy's motor array and a motor name too long for the string's inline
// buffer, watched from the save until they are freed.
// REQ-CONTRACT-06
TEST(ConfigPersister, TheSaverFreesNothingTheLoopAllocated) {
  on_loop_thread = true;
  freed_by_loop = 0;
  freed_elsewhere = 0;
  hw::Config a = ConfigWithMotor(1);
  a.motors[0].name = "a_motor_name_longer_than_any_inline_string_buffer";
  std::mutex mutex;
  bool done = false;
  talos::rio::ConfigPersister persister{
      [&](const hw::Config& config) {
        watched[0].store(config.motors.data());
        watched[1].store(config.motors[0].name.data());
        std::lock_guard lock{mutex};
        done = true;
        return true;
      },
      0};
  persister.Start();
  persister.Acknowledged(a);

  // Ticks, as the loop does, until the save ran and both blocks are freed
  // (10 s bounds a hang): a tick that finds the saver holding the mutex
  // hands nothing over, and the next one does. Then the saver has finished
  // too, and has freed neither.
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds{10};
  while (freed_by_loop.load() + freed_elsewhere.load() < 2 &&
         std::chrono::steady_clock::now() < deadline) {
    persister.Tick(true);
    std::this_thread::yield();
  }
  persister.Stop();
  {
    std::lock_guard lock{mutex};
    ASSERT_TRUE(done) << "the save never ran";
  }
  EXPECT_EQ(freed_elsewhere.load(), 0)
      << "blocks of the handed-over configuration the saver thread freed";
  EXPECT_EQ(freed_by_loop.load(), 2);
  on_loop_thread = false;
}

}  // namespace

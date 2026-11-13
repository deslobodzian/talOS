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
#include <cstring>
#include <filesystem>
#include <mutex>
#include <new>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "hardware/config_wire.h"
#include "hardware/gateway.h"

#ifndef _WIN32
#include <pthread.h>
#endif

namespace hw = talos::hardware;

// What the config saver's own thread allocates and frees, and which thread
// frees a watched block, for
// ConfigPersister.TheSaverThreadNeitherAllocatesNorFrees. Global new and delete
// are replaced for this test binary with malloc and free (the pair the default
// ones use), so every block passes through Allocate() and Freed().
namespace {
std::array<std::atomic<const void*>, 8> watched{};
std::atomic<int> freed_by_loop{0};
std::atomic<int> freed_elsewhere{0};
thread_local bool on_loop_thread = false;
std::atomic<bool> counting{false};
std::atomic<int> saver_allocations{0};
std::atomic<int> saver_frees{0};

// The saver thread names itself (ConfigPersister::Run); other threads of this
// binary (the Phoenix simulation's) allocate as they please.
bool OnSaverThread() {
#ifdef _WIN32
  return false;
#else
  char name[16] = {};
  return pthread_getname_np(pthread_self(), name, sizeof(name)) == 0 &&
         std::strcmp(name, "talos-config") == 0;
#endif
}

void CountOnSaver(std::atomic<int>& count) {
  if (counting.load(std::memory_order_relaxed) && OnSaverThread())
    count.fetch_add(1, std::memory_order_relaxed);
}

void Watch(const void* block) {
  for (auto& slot : watched) {
    const void* empty = nullptr;
    if (slot.compare_exchange_strong(empty, block)) return;
  }
}

void Freed(void* block) {
  if (block == nullptr) return;
  CountOnSaver(saver_frees);
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

void* MaybeAllocate(std::size_t size) {
  CountOnSaver(saver_allocations);
  return std::malloc(size == 0 ? 1 : size);
}

void* Allocate(std::size_t size) {
  if (void* block = MaybeAllocate(size)) return block;
  throw std::bad_alloc{};
}
}  // namespace

// Every unaligned form, so a sanitizer's own new is never paired with free.
void* operator new(std::size_t size) { return Allocate(size); }
void* operator new[](std::size_t size) { return Allocate(size); }
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  return MaybeAllocate(size);
}
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
  return MaybeAllocate(size);
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
  ASSERT_TRUE(talos::rio::ConfigFile{path}.Save(hw::PersistedConfig(config)));

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

// The ConfigurationId of persisted bytes, 0 when they do not decode.
uint64_t IdOf(std::span<const uint8_t> persisted) {
  hw::Config config;
  return hw::LoadPersistedConfig(persisted, config)
             ? hw::ConfigurationId(config)
             : 0;
}

// A store whose saves block until the test releases them (or a 10 s deadline
// that only bounds a hang) and record each configuration's id, in order.
class GatedStore {
 public:
  bool Save(std::span<const uint8_t> persisted) {
    const uint64_t id = IdOf(persisted);
    std::unique_lock lock{mutex_};
    started_.push_back(id);
    changed_.notify_all();
    changed_.wait_for(lock, kHang, [&] { return released_; });
    const bool ok = !fail_next_;
    fail_next_ = false;
    finished_.push_back(id);
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
      [&](std::span<const uint8_t> persisted) { return store.Save(persisted); },
      0};
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
      [&](std::span<const uint8_t> persisted) { return store.Save(persisted); },
      0};
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

// The CRC32 trailer of persisted bytes, which tells configurations apart
// without allocating.
uint32_t Trailer(std::span<const uint8_t> persisted) {
  uint32_t crc = 0;
  for (std::size_t i = 0; i < 4; ++i)
    crc |= uint32_t{persisted[persisted.size() - 4 + i]} << (8 * i);
  return crc;
}

// The saver thread runs at nice 19 without priority inheritance. Were it to
// allocate or free, glibc could take the lock of an arena the loop thread
// shares (on the RoboRIO, 32-bit with two cores, glibc caps arenas at four,
// and the saver starts after HAL, WPILib and Phoenix threads exist), and the
// saver
// preempted holding it would stall the loop's next allocation. So once
// started it does neither, across saves that succeed, one that fails (its
// directory does not exist) and the save Stop() runs while the last saved
// configuration is still uncollected; each handed-over configuration's bytes
// are freed on the loop thread (watched from the save until freed), and the
// file holds the newest one handed over.
// REQ-CONTRACT-06
TEST(ConfigPersister, TheSaverThreadNeitherAllocatesNorFrees) {
  on_loop_thread = true;
  freed_by_loop = 0;
  freed_elsewhere = 0;
  saver_allocations = 0;
  saver_frees = 0;
  const auto directory = std::filesystem::temp_directory_path();
  const auto path = (directory / "talos_config_saver_test.bin").string();
  const auto unwritable =
      (directory / "talos_no_such_directory" / "talos_config.bin").string();
  std::filesystem::remove(path);
  const std::array<hw::Config, 4> configs{
      ConfigWithMotor(1), ConfigWithMotor(2), ConfigWithMotor(3),
      ConfigWithMotor(4)};
  std::array<uint32_t, 8> expected{};
  for (std::size_t i = 0; i < configs.size(); ++i)
    expected[i] = Trailer(hw::PersistedConfig(configs[i]));
  const talos::rio::ConfigFile file{path};
  const talos::rio::ConfigFile unwritable_file{unwritable};
  constexpr std::chrono::seconds kHang{10};

  std::mutex mutex;
  std::condition_variable changed;
  bool gate_open = true;
  bool fail_next = false;
  bool named = true;
  std::size_t started = 0;
  std::size_t finished = 0;
  // Each finished save's CRC32 trailer, in order.
  std::array<uint32_t, 8> saved{};
  {
    talos::rio::ConfigPersister persister{
        [&](std::span<const uint8_t> persisted) {
          Watch(persisted.data());
          std::unique_lock lock{mutex};
          named = named && OnSaverThread();
          ++started;
          changed.notify_all();
          changed.wait_for(lock, kHang, [&] { return gate_open; });
          const bool fail = fail_next;
          fail_next = false;
          lock.unlock();
          const bool ok = (fail ? unwritable_file : file).Save(persisted);
          lock.lock();
          if (finished < saved.size()) saved[finished] = Trailer(persisted);
          ++finished;
          changed.notify_all();
          return ok;
        },
        0};
    persister.Start();
    counting = true;

    uint32_t failures = 0;
    // Ticks, as the loop does, until `done` (10 s bounds a hang).
    const auto tick_until = [&](auto done) {
      const auto deadline = std::chrono::steady_clock::now() + kHang;
      for (;;) {
        failures += persister.Tick(true);
        {
          std::lock_guard lock{mutex};
          if (done()) return true;
        }
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::yield();
      }
    };

    // A save, collected: its configuration comes back to be freed here.
    persister.Acknowledged(configs[0]);
    ASSERT_TRUE(
        tick_until([&] { return finished == 1 && freed_by_loop == 1; }));
    // A save that fails.
    {
      std::lock_guard lock{mutex};
      fail_next = true;
    }
    persister.Acknowledged(configs[1]);
    ASSERT_TRUE(
        tick_until([&] { return failures == 1 && freed_by_loop == 2; }));
    // A save held while the next configuration is handed over, then Stop()
    // with that one pending and the held one finished but not collected.
    {
      std::lock_guard lock{mutex};
      gate_open = false;
    }
    persister.Acknowledged(configs[2]);
    ASSERT_TRUE(tick_until([&] { return started == 3; }));
    persister.Acknowledged(configs[3]);
    for (int tick = 0; tick < 10; ++tick) failures += persister.Tick(true);
    {
      std::unique_lock lock{mutex};
      gate_open = true;
      changed.notify_all();
      ASSERT_TRUE(changed.wait_for(lock, kHang, [&] { return finished == 3; }));
    }
    persister.Stop();
    counting = false;
    failures += persister.Tick(false);

    EXPECT_EQ(failures, 1u);
    std::lock_guard lock{mutex};
    EXPECT_TRUE(named) << "the saver thread was not identified by its name";
    EXPECT_EQ(finished, 4u);
    EXPECT_EQ(saved, expected);
  }
  EXPECT_EQ(saver_allocations.load(), 0)
      << "allocations on the saver thread after it started";
  EXPECT_EQ(saver_frees.load(), 0)
      << "frees on the saver thread after it started";
  EXPECT_EQ(freed_elsewhere.load(), 0)
      << "handed-over configurations the saver thread freed";
  EXPECT_EQ(freed_by_loop.load(), 4);
  EXPECT_EQ(talos::rio::LoadConfigFile(path), configs[3]);
  std::filesystem::remove(path);
  on_loop_thread = false;
}
}  // namespace

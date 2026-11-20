// The persisted hardware configuration. Its startup restore must not wait
// for device configuration (a mid-match reboot with absent devices would hold
// robot code start for a CAN timeout per device), and nothing drives until
// the restore completes. Its save must not hold the loop thread, and must
// leave the newest acknowledged configuration on disk.
#include "ConfigStore.h"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <new>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "RioLog.h"
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
// ones use), so every block passes through Allocate() and Freed(). With glibc
// (the RoboRIO's C library) and no sanitizer, which brings its own allocator,
// malloc, calloc, realloc, the aligned forms and free are replaced too, over
// glibc's own (__libc_malloc, ...), so a C allocation on the saver thread
// (the C library's, or a vendor library's) counts as well; elsewhere only new
// and delete count.
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || \
    __has_feature(memory_sanitizer)
#define TALOS_TEST_SANITIZER 1
#endif
#endif
#if defined(__GLIBC__) && !defined(__SANITIZE_ADDRESS__) && \
    !defined(__SANITIZE_THREAD__) && !defined(TALOS_TEST_SANITIZER)
#define TALOS_TEST_COUNTS_MALLOC 1
#endif
namespace {
thread_local bool on_loop_thread = false;
std::atomic<bool> counting{false};
std::atomic<int> saver_allocations{0};
std::atomic<int> saver_frees{0};
std::atomic<int> loop_allocations{0};
std::atomic<int> loop_frees{0};

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

void CountAllocation() {
  if (!counting.load(std::memory_order_relaxed)) return;
  if (OnSaverThread()) {
    saver_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  if (on_loop_thread) {
    loop_allocations.fetch_add(1, std::memory_order_relaxed);
  }
}

void CountFree() {
  if (!counting.load(std::memory_order_relaxed)) return;
  if (OnSaverThread()) {
    saver_frees.fetch_add(1, std::memory_order_relaxed);
  }
  if (on_loop_thread) {
    loop_frees.fetch_add(1, std::memory_order_relaxed);
  }
}

void* MaybeAllocate(std::size_t size) {
#ifndef TALOS_TEST_COUNTS_MALLOC
  CountAllocation();  // Otherwise malloc counts it.
#endif
  return std::malloc(size == 0 ? 1 : size);
}

void* Allocate(std::size_t size) {
  if (void* block = MaybeAllocate(size)) return block;
  throw std::bad_alloc{};
}

// The free side of the replaced deletes, counted once.
void Release(void* block) {
#ifndef TALOS_TEST_COUNTS_MALLOC
  if (block != nullptr) CountFree();
#endif
  std::free(block);
}
}  // namespace

#ifdef TALOS_TEST_COUNTS_MALLOC
// glibc's allocator under its exported internal names, which the
// replacements below forward to.
extern "C" {
// NOLINTBEGIN(bugprone-reserved-identifier): glibc's own names.
void* __libc_malloc(std::size_t size);
void* __libc_calloc(std::size_t count, std::size_t size);
void* __libc_realloc(void* block, std::size_t size);
void* __libc_memalign(std::size_t alignment, std::size_t size);
void __libc_free(void* block);
// NOLINTEND(bugprone-reserved-identifier)

void* malloc(std::size_t size) noexcept {
  CountAllocation();
  return __libc_malloc(size);
}
void* calloc(std::size_t count, std::size_t size) noexcept {
  CountAllocation();
  return __libc_calloc(count, size);
}
void* realloc(void* block, std::size_t size) noexcept {
  CountAllocation();
  return __libc_realloc(block, size);
}
void* memalign(std::size_t alignment, std::size_t size) noexcept {
  CountAllocation();
  return __libc_memalign(alignment, size);
}
void* aligned_alloc(std::size_t alignment, std::size_t size) noexcept {
  CountAllocation();
  return __libc_memalign(alignment, size);
}
int posix_memalign(void** block, std::size_t alignment,
                   std::size_t size) noexcept {
  CountAllocation();
  if (alignment == 0 || alignment % sizeof(void*) != 0 ||
      (alignment & (alignment - 1)) != 0)
    return EINVAL;
  void* allocated = __libc_memalign(alignment, size);
  if (allocated == nullptr) return ENOMEM;
  *block = allocated;
  return 0;
}
void free(void* block) noexcept {
  if (block != nullptr) CountFree();
  __libc_free(block);
}
}  // extern "C"
#endif

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
  Release(block);
}
void operator delete[](void* block, const std::nothrow_t&) noexcept {
  Release(block);
}
void operator delete(void* block) noexcept { Release(block); }
void operator delete[](void* block) noexcept { Release(block); }
void operator delete(void* block, std::size_t) noexcept { Release(block); }
void operator delete[](void* block, std::size_t) noexcept { Release(block); }

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
  // Ticks `persister`, as the loop does, until the save of the `n`th
  // configuration starts. Tick() only try-locks, so one tick can lose the
  // mutex to the saver thread (as it starts, or as it moves a save in or out)
  // and hand nothing over; the next tick retries. The deadline only bounds a
  // hang.
  bool TickUntilStarted(talos::rio::ConfigPersister& persister, std::size_t n) {
    const auto deadline = std::chrono::steady_clock::now() + kHang;
    for (;;) {
      persister.Tick(true);
      {
        std::lock_guard lock{mutex_};
        if (started_.size() >= n) return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) return false;
      std::this_thread::yield();
    }
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
  ASSERT_TRUE(store.TickUntilStarted(persister, 1)) << "A's save never started";

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
  ASSERT_TRUE(store.TickUntilStarted(persister, 2))
      << "the retry never started";
  persister.Stop();
  EXPECT_EQ(store.finished(), (std::vector<uint64_t>{hw::ConfigurationId(a),
                                                     hw::ConfigurationId(a)}));
}

// A configuration acknowledged on the last tick before a clean exit, with no
// tick after it to hand it over (or only enabled ones, which never do), is
// still saved by Stop(): otherwise the next boot restores the one before it.
// REQ-CONTRACT-06
TEST(ConfigPersister, StopSavesAConfigurationNeverHandedOver) {
  const hw::Config a = ConfigWithMotor(1);
  GatedStore store;
  store.Release();
  talos::rio::ConfigPersister persister{
      [&](std::span<const uint8_t> persisted) { return store.Save(persisted); },
      0};
  persister.Start();
  persister.Acknowledged(a);
  EXPECT_EQ(persister.Tick(false), 0u);
  persister.Stop();
  EXPECT_EQ(store.finished(), (std::vector<uint64_t>{hw::ConfigurationId(a)}));
}

// A save Stop() runs while the previous one is still uncollected is left
// behind in the saver's own slot; a saver started again must still save the
// next configuration handed over, not return at once on finding that slot
// occupied.
// REQ-CONTRACT-06
TEST(ConfigPersister, ARestartAfterStopSavesTheNextConfiguration) {
  const hw::Config a = ConfigWithMotor(1);
  const hw::Config b = ConfigWithMotor(2);
  const hw::Config c = ConfigWithMotor(3);
  GatedStore store;
  talos::rio::ConfigPersister persister{
      [&](std::span<const uint8_t> persisted) { return store.Save(persisted); },
      0};
  persister.Start();
  persister.Acknowledged(a);
  ASSERT_TRUE(store.TickUntilStarted(persister, 1)) << "A's save never started";
  // B is handed over while A's save runs (the saver holds no lock then).
  persister.Acknowledged(b);
  for (int tick = 0; tick < 10; ++tick) persister.Tick(true);
  // A finishes uncollected; Stop() saves B behind it.
  store.Release();
  persister.Stop();
  ASSERT_EQ(store.finished(), (std::vector<uint64_t>{hw::ConfigurationId(a),
                                                     hw::ConfigurationId(b)}));

  persister.Start();
  persister.Acknowledged(c);
  EXPECT_TRUE(store.TickUntilStarted(persister, 3))
      << "the restarted saver never saved C";
  persister.Stop();
  EXPECT_EQ(store.finished(), (std::vector<uint64_t>{hw::ConfigurationId(a),
                                                     hw::ConfigurationId(b),
                                                     hw::ConfigurationId(c)}));
}

// The CRC32 trailer of persisted bytes, which tells configurations apart
// without allocating.
uint32_t Trailer(std::span<const uint8_t> persisted) {
  uint32_t crc = 0;
  for (std::size_t i = 0; i < 4; ++i)
    crc |= uint32_t{persisted[persisted.size() - 4 + i]} << (8 * i);
  return crc;
}

// The saver thread runs at nice 19 without priority inheritance. Were either
// thread to allocate or free during persistence, glibc could contend on a
// shared arena lock on the RoboRIO (two cores, four arenas). With preallocated
// fixed-size slots swapped by index under mutex_, neither the saver thread nor
// the loop thread (Acknowledged, Tick, Stop) allocates or frees (with glibc,
// malloc and free included), across saves that succeed, one that fails after
// writing and syncing its temporary file (the rename over a directory fails,
// so the save unlinks the temporary and leaves none behind) and a save
// queued while another is in flight; the file holds the newest one handed
// over.
// REQ-CONTRACT-06, REQ-RT-01, REQ-RT-06
TEST(ConfigPersister, TheSaverThreadNeitherAllocatesNorFrees) {
  saver_allocations = 0;
  saver_frees = 0;
  loop_allocations = 0;
  loop_frees = 0;
  const auto directory = std::filesystem::temp_directory_path();
  const auto path = (directory / "talos_config_saver_test.bin").string();
  // A directory where the file should be: the rename over it fails.
  const auto unrenamable = directory / "talos_config_saver_test_directory";
  std::filesystem::remove(path);
  std::filesystem::remove_all(unrenamable);
  ASSERT_TRUE(std::filesystem::create_directory(unrenamable));
  const std::array<hw::Config, 4> configs{
      ConfigWithMotor(1), ConfigWithMotor(2), ConfigWithMotor(3),
      ConfigWithMotor(4)};
  std::array<uint32_t, 8> expected{};
  for (std::size_t i = 0; i < configs.size(); ++i)
    expected[i] = Trailer(hw::PersistedConfig(configs[i]));
  const talos::rio::ConfigFile file{path};
  const talos::rio::ConfigFile unrenamable_file{unrenamable.string()};
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
          std::unique_lock lock{mutex};
          named = named && OnSaverThread();
          ++started;
          changed.notify_all();
          changed.wait_for(lock, kHang, [&] { return gate_open; });
          const bool fail = fail_next;
          fail_next = false;
          lock.unlock();
          const bool ok = (fail ? unrenamable_file : file).Save(persisted);
          lock.lock();
          if (finished < saved.size()) saved[finished] = Trailer(persisted);
          ++finished;
          changed.notify_all();
          return ok;
        },
        0};
    persister.Start();
    on_loop_thread = true;
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

    // A save, collected with zero loop or saver allocations/frees.
    persister.Acknowledged(configs[0]);
    const bool first_done = tick_until([&] { return finished == 1; });
    // A save that fails.
    {
      std::lock_guard lock{mutex};
      fail_next = true;
    }
    persister.Acknowledged(configs[1]);
    const bool second_done = tick_until([&] { return failures == 1; });
    // A save held while the next configuration is handed over, then Stop().
    {
      std::lock_guard lock{mutex};
      gate_open = false;
    }
    persister.Acknowledged(configs[2]);
    const bool third_started = tick_until([&] { return started == 3; });
    persister.Acknowledged(configs[3]);
    for (int tick = 0; tick < 10; ++tick) failures += persister.Tick(true);
    bool third_done = false;
    {
      std::unique_lock lock{mutex};
      gate_open = true;
      changed.notify_all();
      third_done = changed.wait_for(lock, kHang, [&] { return finished >= 3; });
    }
    persister.Stop();
    failures += persister.Tick(false);
    counting = false;
    on_loop_thread = false;

    ASSERT_TRUE(first_done);
    ASSERT_TRUE(second_done);
    ASSERT_TRUE(third_started);
    ASSERT_TRUE(third_done);
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
  EXPECT_EQ(loop_allocations.load(), 0)
      << "allocations on the loop thread during Acknowledged/Tick/Stop";
  EXPECT_EQ(loop_frees.load(), 0)
      << "frees on the loop thread during Acknowledged/Tick/Stop";
  EXPECT_EQ(talos::rio::LoadConfigFile(path), configs[3]);
  EXPECT_FALSE(std::filesystem::exists(unrenamable.string() + ".tmp"))
      << "the failed save left its temporary file";
  std::filesystem::remove(path);
  std::filesystem::remove_all(unrenamable);
}

// Guards TEST-20: configuration transfer, validation, enabled refusal,
// reconfiguration commit, persistence handover, and RioLog's default console
// sink all execute with zero heap allocations and zero frees, even when every
// device and bus name exceeds std::string's SSO threshold.
// REQ-RT-01, REQ-RT-06, REQ-SAFE-24
TEST(ConfigStore, ConfigTransferReconfigurePersistAndRioLogSinkZeroHeapAlloc) {
  auto make_long_config = [](int base_can, bool second_motor) {
    hw::Config c;
    c.commissioned = true;
    hw::MotorConfig m1;
    m1.id = 1;
    m1.name = "drivetrain_front_left_drive_motor_with_long_name_for_no_sso";
    m1.can_id = base_can;
    m1.bus = "canivore_swerve_drivetrain_bus_name_exceeding_sso";
    c.motors.push_back(m1);
    if (second_motor) {
      hw::MotorConfig m2;
      m2.id = 2;
      m2.name = "drivetrain_front_left_steer_motor_with_long_name_for_no_sso";
      m2.can_id = base_can + 1;
      m2.bus = "canivore_swerve_drivetrain_bus_name_exceeding_sso";
      c.motors.push_back(m2);
    }
    hw::SensorConfig s;
    s.id = 10;
    s.name = "drivetrain_front_left_cancoder_long_name_exceeding_sso";
    s.kind = hw::SensorKind::kCANcoder;
    s.can_id = base_can + 10;
    s.bus = "canivore_swerve_drivetrain_bus_name_exceeding_sso";
    c.sensors.push_back(s);
    c.digital_inputs.push_back(
        {20, "superstructure_elevator_home_limit_switch_long_name", 0});
    c.digital_outputs.push_back(
        {21, "superstructure_intake_solenoid_output_long_name", 1});
    c.analog_inputs.push_back(
        {22, "superstructure_pressure_transducer_input_long_name", 0});
    c.encoders.push_back({23, "superstructure_arm_quadrature_encoder_long_name",
                          hw::EncoderKind::kQuadrature, 2, 3, 2048.0, 0.0});
    c.pwm_outputs.push_back(
        {24, "superstructure_climber_servo_pwm_output_long_name", 0});
    return c;
  };

  const hw::Config config_a = make_long_config(1, false);
  const hw::Config config_b = make_long_config(5, true);
  hw::Config invalid_config = config_b;
  invalid_config.period_us = 0;  // fails IsValid()
  hw::Config ceiling_config = config_b;
  ceiling_config.motors[0].supply_limit_a = hw::kCeilingMaxSupplyCurrentA + 1.0;

  const uint64_t id_a = hw::ConfigurationId(config_a);
  const uint64_t id_b = hw::ConfigurationId(config_b);
  const auto chunks_a = hw::CreateConfigChunks(config_a, 200);
  const auto chunks_b = hw::CreateConfigChunks(config_b, 200);
  auto bad_crc_chunks = hw::CreateConfigChunks(config_a, 200);
  bad_crc_chunks.back().back() ^= 0xFF;
  const auto ceiling_chunks = hw::CreateConfigChunks(ceiling_config, 200);

  // Pre-open a temporary FILE* and make it unbuffered (like stderr in
  // production) before counting so glibc's lazy stdio buffer allocation is
  // not triggered on the first fwrite().
  std::FILE* log_stream = std::tmpfile();
  ASSERT_NE(log_stream, nullptr);
  ASSERT_EQ(std::setvbuf(log_stream, nullptr, _IONBF, 0), 0);
  std::string long_line(2500, 'x');
  long_line.push_back('\n');

  hw::ConfigAssembler assembler;
  PendingBackend backend;
  hw::Gateway gateway{backend, 42};
  std::atomic<uint64_t> saved_count{0};
  talos::rio::ConfigPersister persister{[&](std::span<const uint8_t>) {
                                          saved_count.fetch_add(
                                              1, std::memory_order_relaxed);
                                          return true;
                                        },
                                        0};
  persister.Start();

  std::array<char, 160> err_buf{};
  std::array<uint8_t, hw::kMaxSerializedConfigSize> ser_buf{};
  std::array<uint8_t, hw::kMaxPersistedConfigSize> persist_buf{};

  saver_allocations = 0;
  saver_frees = 0;
  loop_allocations = 0;
  loop_frees = 0;
  on_loop_thread = true;
  counting = true;

  // 1. SerializeConfigInto, PersistedConfigInto, TryConfigurationId, IsValid.
  const std::size_t ser_a = hw::SerializeConfigInto(config_a, ser_buf);
  const std::size_t per_a = hw::PersistedConfigInto(config_a, persist_buf);
  const uint64_t try_id_a = hw::TryConfigurationId(config_a);
  const uint64_t try_id_invalid = hw::TryConfigurationId(invalid_config);
  const int allocs_after_wire = loop_allocations.load();
  const int frees_after_wire = loop_frees.load();

  // 2. ConfigAssembler::AddChunk: success, CRC error, ceiling error, second
  //    config with different device counts.
  hw::ConfigAssembler::Status status_a =
      hw::ConfigAssembler::Status::kIncomplete;
  for (const auto& ch : chunks_a) {
    status_a = assembler.AddChunk(ch, std::span<char>{err_buf});
  }
  hw::ConfigAssembler::Status status_crc =
      hw::ConfigAssembler::Status::kIncomplete;
  for (const auto& ch : bad_crc_chunks) {
    status_crc = assembler.AddChunk(ch, std::span<char>{err_buf});
  }
  hw::ConfigAssembler::Status status_ceil =
      hw::ConfigAssembler::Status::kIncomplete;
  for (const auto& ch : ceiling_chunks) {
    status_ceil = assembler.AddChunk(ch, std::span<char>{err_buf});
  }
  hw::ConfigAssembler::Status status_b =
      hw::ConfigAssembler::Status::kIncomplete;
  for (const auto& ch : chunks_b) {
    status_b = assembler.AddChunk(ch, std::span<char>{err_buf});
  }
  const int allocs_after_assembler = loop_allocations.load();
  const int frees_after_assembler = loop_frees.load();

  // 3. Gateway::BeginReconfigure, CommitConfiguration, RecordEnabledRefusal.
  const bool begin_a = gateway.BeginReconfigure(config_a);
  backend.done = true;
  gateway.Tick(5000, false);  // CommitConfiguration(config_a)
  gateway.Tick(10000, true);  // enable
  const bool refused_b = gateway.BeginReconfigure(config_b);
  const bool refused_invalid = gateway.BeginReconfigure(invalid_config);
  const bool idempotent_a = gateway.BeginReconfigure(config_a);
  gateway.Tick(15000, false);  // disable
  const bool invalid_disabled = gateway.BeginReconfigure(invalid_config);
  backend.done = false;
  const bool begin_b = gateway.BeginReconfigure(config_b);
  backend.done = true;
  gateway.Tick(20000, false);  // CommitConfiguration(config_b)
  const int allocs_after_gateway = loop_allocations.load();
  const int frees_after_gateway = loop_frees.load();

  // 4. ConfigPersister::Acknowledged and Tick.
  persister.Acknowledged(config_a);
  const uint32_t fail_1 = persister.Tick(true);
  persister.Acknowledged(config_b);
  const uint32_t fail_2 = persister.Tick(true);
  persister.Stop();
  const int allocs_after_persister = loop_allocations.load();
  const int frees_after_persister = loop_frees.load();

  // 5. RioLog default sink formatting (normal line and > 2048 B fallback).
  talos::rio::RioLog::WriteConsoleLine(12'345'000, "[rio] hello\n", log_stream);
  talos::rio::RioLog::WriteConsoleLine(12'346'000, long_line, log_stream);

  counting = false;
  on_loop_thread = false;

  EXPECT_EQ(allocs_after_wire, 0);
  EXPECT_EQ(frees_after_wire, 0);
  EXPECT_EQ(allocs_after_assembler, 0);
  EXPECT_EQ(frees_after_assembler, 0);
  EXPECT_EQ(allocs_after_gateway, 0);
  EXPECT_EQ(frees_after_gateway, 0);
  EXPECT_EQ(allocs_after_persister, 0);
  EXPECT_EQ(frees_after_persister, 0);
  EXPECT_EQ(loop_allocations.load(), 0)
      << "loop thread allocated during config transfer / reconfigure / persist "
         "/ "
         "RioLog sink";
  EXPECT_EQ(loop_frees.load(), 0)
      << "loop thread freed during config transfer / reconfigure / persist / "
         "RioLog sink";
  EXPECT_EQ(saver_allocations.load(), 0)
      << "saver thread allocated during persistence";
  EXPECT_EQ(saver_frees.load(), 0) << "saver thread freed during persistence";

  EXPECT_GT(ser_a, 0u);
  EXPECT_GT(per_a, 0u);
  EXPECT_EQ(try_id_a, id_a);
  EXPECT_EQ(try_id_invalid, 0u);
  EXPECT_EQ(status_a, hw::ConfigAssembler::Status::kComplete);
  EXPECT_EQ(status_crc, hw::ConfigAssembler::Status::kCrcMismatch);
  EXPECT_EQ(status_ceil, hw::ConfigAssembler::Status::kCeilingExceeded);
  EXPECT_EQ(status_b, hw::ConfigAssembler::Status::kComplete);
  EXPECT_EQ(assembler.config(), config_b);
  EXPECT_TRUE(begin_a);
  EXPECT_FALSE(refused_b);
  EXPECT_FALSE(refused_invalid);
  EXPECT_TRUE(idempotent_a);
  EXPECT_FALSE(invalid_disabled);
  EXPECT_TRUE(begin_b);
  EXPECT_EQ(gateway.last_enabled_refusal().count, 2u);
  EXPECT_EQ(gateway.last_enabled_refusal().refused_id, 0u);
  EXPECT_EQ(gateway.last_enabled_refusal().active_id, id_a);
  EXPECT_TRUE(gateway.configured());
  EXPECT_EQ(gateway.snapshot().config_id, id_b);
  EXPECT_EQ(fail_1, 0u);
  EXPECT_EQ(fail_2, 0u);
  EXPECT_GE(saved_count.load(), 1u);

  std::rewind(log_stream);
  std::string logged(3000, '\0');
  const std::size_t n = std::fread(logged.data(), 1, logged.size(), log_stream);
  logged.resize(n);
  std::fclose(log_stream);
  EXPECT_EQ(logged, "[12.345] [rio] hello\n[12.346] " + long_line);
}
}  // namespace

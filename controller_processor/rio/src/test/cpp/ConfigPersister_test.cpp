// Regression tests for ConfigPersister on-disk state tracking after a failed
// save (REQ-CONTRACT-06).
#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

#include "ConfigStore.h"
#include "hardware/config.h"
#include "hardware/config_wire.h"

namespace {

namespace hw = talos::hardware;

hw::Config ConfigWithMotor(int can_id) {
  hw::Config config;
  hw::MotorConfig motor;
  motor.id = 1;
  motor.name = "test_motor";
  motor.can_id = can_id;
  config.motors.push_back(motor);
  return config;
}

uint64_t IdOf(std::span<const uint8_t> persisted) {
  hw::Config config;
  return hw::LoadPersistedConfig(persisted, config)
             ? hw::ConfigurationId(config)
             : 0;
}

// Scenario: Config A is saved to disk, then Config B is saved and fails AFTER
// mutating the file on disk (as when ConfigFile::Save's SyncDirectory() fails
// after std::rename() already replaced the file with B). ConfigPersister must
// treat the on-disk contents as unknown (not assume A is still on disk), so
// re-acknowledging A runs save_ again and restores A to disk.
// REQ-CONTRACT-06
TEST(ConfigPersister, FailedSaveClearsSavedIdSoReacknowledgingPreviousSaves) {
  const hw::Config a = ConfigWithMotor(1);
  const hw::Config b = ConfigWithMotor(2);
  const uint64_t id_a = hw::ConfigurationId(a);
  const uint64_t id_b = hw::ConfigurationId(b);

  const auto path = (std::filesystem::temp_directory_path() /
                     "talos_config_persister_stale_id_test.bin")
                        .string();
  std::filesystem::remove(path);
  const talos::rio::ConfigFile file{path};

  std::mutex mutex;
  std::condition_variable changed;
  std::vector<uint64_t> saved_ids;

  talos::rio::ConfigPersister persister{
      [&](std::span<const uint8_t> persisted) {
        const uint64_t id = IdOf(persisted);
        // Write to disk on every attempt; return false for B to simulate a
        // failure after std::rename() already replaced the file (e.g.
        // SyncDirectory() failing).
        const bool wrote = file.Save(persisted);
        const bool ok = wrote && (id != id_b);
        {
          const std::lock_guard lock{mutex};
          saved_ids.push_back(id);
        }
        changed.notify_all();
        return ok;
      },
      0};
  persister.Start();

  // 1. Save Config A successfully.
  persister.Acknowledged(a);
  constexpr int kMaxWaits = 10'000;
  bool saved_a = false;
  for (int i = 0; i < kMaxWaits && !saved_a; ++i) {
    EXPECT_EQ(persister.Tick(true), 0u);
    std::unique_lock lock{mutex};
    saved_a = changed.wait_for(lock, std::chrono::milliseconds{1},
                               [&] { return saved_ids.size() >= 1; });
  }
  ASSERT_TRUE(saved_a) << "Config A save never finished";
  ASSERT_EQ(talos::rio::LoadConfigFile(path), a);

  // 2. Save Config B: mutates the file to B, then reports failure.
  // Polling Tick(true) until failures == 1 guarantees Tick() collected both A
  // and B's completion (outstanding_ == 0 and on_disk_id_ updated from
  // saved_id_).
  persister.Acknowledged(b);
  uint32_t failures = 0;
  int waits = 0;
  for (int yields = 0; waits < kMaxWaits && failures == 0;) {
    failures += persister.Tick(true);
    if (failures > 0) break;
    std::unique_lock lock{mutex};
    if (saved_ids.size() < 2) {
      changed.wait_for(lock, std::chrono::milliseconds{1},
                       [&] { return saved_ids.size() >= 2; });
      ++waits;
    } else if (++yields % 100 == 0) {
      changed.wait_for(lock, std::chrono::milliseconds{1},
                       [] { return false; });
      ++waits;
    } else {
      lock.unlock();
      std::this_thread::yield();
    }
  }
  ASSERT_EQ(failures, 1u) << "Config B failure was never reported by Tick";
  ASSERT_EQ(talos::rio::LoadConfigFile(path), b);

  // 3. Re-acknowledge Config A: must not be skipped as "already on disk".
  persister.Acknowledged(a);
  persister.Stop();

  EXPECT_EQ(saved_ids, (std::vector<uint64_t>{id_a, id_b, id_a}))
      << "re-acknowledging A after B's failed save skipped saving A";
  EXPECT_EQ(talos::rio::LoadConfigFile(path), a)
      << "disk still holds Config B instead of Config A";

  std::filesystem::remove(path);
}

}  // namespace

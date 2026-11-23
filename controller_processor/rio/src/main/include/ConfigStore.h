#pragma once

#include <array>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>

#ifdef _WIN32
#include <thread>
#else
#include <pthread.h>
#endif

#include "hardware/config.h"
#include "hardware/config_wire.h"
#include "hardware/gateway.h"

// The RoboRIO's copy of the last acknowledged hardware configuration, so a
// reboot (a brownout mid-match) drives again without waiting for the companion
// to push it.
namespace talos::rio {

// The persisted configuration, or nullopt when the file is missing, was
// written under another protocol version or before the file carried one, or
// fails its CRC, decoding, ceilings or validation (hardware::
// LoadPersistedConfig). Such a file is never migrated: the RoboRIO boots
// unconfigured and waits for the bridge's push.
std::optional<hardware::Config> LoadConfigFile(const std::string& path);

// Where the configuration is persisted. Save() writes durably and
// atomically: a synced temporary file is renamed over the path and the
// directory is synced, so a power loss leaves the previous file or the new
// one, never a torn one.
class ConfigFile {
 public:
  // Builds every path a save needs, so Save() allocates nothing.
  explicit ConfigFile(std::string path);

  // Writes `persisted` (hardware::PersistedConfig: a magic and protocol
  // version, the config wire bytes, then a CRC32 of all three). Blocks on
  // storage (two fsyncs): never call it from the loop thread; ConfigPersister
  // runs it on a thread of its own. Off Windows it makes system calls only
  // (open, write, fsync, close, rename, unlink) and allocates and frees
  // nothing; desktop simulation on Windows, which has no fsync, writes through
  // the standard library.
  bool Save(std::span<const uint8_t> persisted) const;

 private:
  std::string path_;
  std::string temporary_;
  std::string directory_;
};

// Starts restoring the persisted configuration on the backend's configure
// worker and returns it, or nullopt when none was started. It never waits for
// device traffic, so robot code reports ready at once; until the worker
// finishes the gateway is unconfigured, refuses every command and drives
// nothing.
std::optional<hardware::Config> BeginRestore(hardware::Gateway& gateway,
                                             const std::string& path);

// Persists acknowledged configurations without the loop thread ever waiting
// on storage.
//
// The loop thread (Robot.cpp) hands each acknowledged configuration to
// Acknowledged() and calls Tick() once per tick, as it polls a configure
// (Gateway::Tick, Backend::PollConfigure). A saver thread of its own runs
// `save` (ConfigFile::Save: two fsyncs, hundreds of milliseconds on a slow
// flash). The two share one mutex, which the saver holds only to move a
// configuration in or a result out, never across a save, and which the loop
// thread only ever try-locks: when it is taken, the tick goes on and the next
// one tries again. Three fixed-size persisted-byte slots are allocated once at
// construction (unsaved, pending, saving) and swapped by index under the mutex,
// so once constructed neither the loop thread (Acknowledged, Tick) nor the
// saver thread allocates or frees (and `save` allocates nothing either:
// ConfigFile::Save off Windows).
//
// The newest acknowledged configuration is the one left on disk: one
// acknowledged while a save is in flight is saved after it, and one handed
// over but not yet started is replaced by a newer one, so an older
// configuration is never written last. A configuration is handed over only
// on a disabled tick, as before this ran on its own thread (REQ-CONTRACT-06);
// its save may finish after an enable. A configuration the file already holds
// is not saved again unless another was handed over since. A failed save
// leaves the previous file (ConfigFile::Save is atomic); Tick() counts it for
// the loop to report, and the same configuration acknowledged again is saved
// again.
class ConfigPersister {
 public:
  // Writes one configuration's persisted bytes (hardware::PersistedConfig);
  // true once they are on disk. Runs on the saver thread, so it must neither
  // allocate nor free.
  using Save = std::function<bool(std::span<const uint8_t> persisted)>;

  // `on_disk_id`: the ConfigurationId the file holds (the restored one), 0
  // when there is none.
  ConfigPersister(Save save, uint64_t on_disk_id);
  ~ConfigPersister();
  ConfigPersister(const ConfigPersister&) = delete;
  ConfigPersister& operator=(const ConfigPersister&) = delete;

  // Starts the saver thread (nice 19 on Linux). The thread takes the
  // scheduling of the thread that starts it, so Robot.cpp starts it before
  // the loop raises itself to SCHED_FIFO; started later, SCHED_RESET_ON_FORK
  // on the loop would still give it the default scheduling. Throws
  // std::system_error when the thread cannot be created.
  void Start();
  // Saves the newest acknowledged configuration not yet saved, whether or
  // not a tick handed it over (a clean exit right after an acknowledgement,
  // or while enabled), then joins the thread. Called on the loop thread after
  // the loop has stopped ticking (also by the destructor).
  //
  // The join is unbounded: a save stuck in fsync keeps Stop() from
  // returning. Bounding it could not let the program exit sooner: Linux does
  // not finish exiting a process while one of its threads sleeps
  // uninterruptibly in fsync, and returning without the join would leave the
  // saver touching this object after its destruction. Robot.cpp calls Stop()
  // only after neutralizing every output. On the RoboRIO it is not reached in
  // normal operation: with no HAL main loop, StartCompetition returns only on
  // EndCompetition, and the program is otherwise ended by a signal
  // (redeploy, restart robot code), which cuts a save short; the save's
  // rename keeps the file whole either way.
  void Stop();

  // Loop thread: a configuration the gateway acknowledged
  // (Endpoint::OnConfigured). Encodes it into the preallocated unsaved slot
  // (hardware::PersistedConfigInto); never waits, allocates or frees.
  void Acknowledged(const hardware::Config& config);

  // Loop thread, every tick: collects finished saves and, on a `disabled`
  // tick, hands the newest acknowledged configuration not yet handed over to
  // the saver. Returns how many saves failed since the previous call that
  // collected results. Never waits, allocates or frees; touches the mutex only
  // while a save is pending or outstanding.
  uint32_t Tick(bool disabled);

 private:
  // One configuration's ConfigurationId and persisted bytes in fixed storage.
  struct Persisted {
    uint64_t id{0};
    std::size_t size{0};
    std::array<uint8_t, hardware::kMaxPersistedConfigSize> bytes{};
    std::span<const uint8_t> span() const noexcept {
      return {bytes.data(), size};
    }
  };

  void Run();
#ifndef _WIN32
  static void* RunThread(void* persister);
#endif

  const Save save_;
  // Three disjoint preallocated slots (allocated once in the constructor) so
  // unsaved_, pending_, and saving_ hand over by index swap with zero
  // allocation, zero free, and zero 10 KB copy under mutex_.
  std::unique_ptr<std::array<Persisted, 3>> slots_;

  // Loop-thread state.
  std::size_t unsaved_slot_{0};
  bool has_unsaved_{false};
  // The ConfigurationId the file holds as of the last results collected.
  uint64_t on_disk_id_;
  // Handed over and not yet collected as finished.
  uint64_t outstanding_{0};
  uint64_t collected_finished_{0};
  uint64_t collected_failed_{0};

  // Shared, under mutex_.
  std::mutex mutex_;
  std::condition_variable wake_;
  std::size_t pending_slot_{1};
  bool has_pending_{false};
  uint64_t finished_{0};
  uint64_t failed_{0};
  uint64_t saved_id_;
  bool stop_{false};

  // Saver-thread state: the slot currently being written outside mutex_.
  std::size_t saving_slot_{2};

  bool started_{false};
#ifdef _WIN32
  std::thread thread_;
#else
  // Not std::thread: its start state is freed by the new thread as it exits.
  pthread_t thread_{};
#endif
};

}  // namespace talos::rio

#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <vector>

#ifdef _WIN32
#include <thread>
#else
#include <pthread.h>
#endif

#include "hardware/config.h"
#include "hardware/gateway.h"

// The RoboRIO's copy of the last acknowledged hardware configuration, so a
// reboot (a brownout mid-match) drives again without waiting for the companion
// to push it.
namespace talos::rio {

// The persisted configuration, or nullopt when the file is missing or fails
// its CRC, decoding, ceilings or validation.
std::optional<hardware::Config> LoadConfigFile(const std::string& path);

// Where the configuration is persisted. Save() writes durably and
// atomically: a synced temporary file is renamed over the path and the
// directory is synced, so a power loss leaves the previous file or the new
// one, never a torn one.
class ConfigFile {
 public:
  // Builds every path a save needs, so Save() allocates nothing.
  explicit ConfigFile(std::string path);

  // Writes `persisted` (hardware::PersistedConfig: the config wire bytes and
  // their CRC32). Blocks on storage (two fsyncs): never call it from the loop
  // thread; ConfigPersister runs it on a thread of its own. Off Windows it
  // makes system calls only (open, write, fsync, close, rename, unlink) and
  // allocates and frees nothing; desktop simulation on Windows, which has no
  // fsync, writes through the standard library.
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
// one tries again. The loop thread never waits on the save, nor on an
// allocator lock the saver holds: once started, the saver thread allocates
// and frees nothing. The loop thread encodes each configuration it hands over
// (Acknowledged); the saver passes those bytes to `save`, which allocates
// nothing either (ConfigFile::Save off Windows), and hands them back to
// Tick() to free.
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
  // Saves what was handed over and not yet saved, then joins the thread.
  // Called after the loop has stopped ticking (also by the destructor).
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
  // (Endpoint::OnConfigured). Encodes it (hardware::PersistedConfig); never
  // waits.
  void Acknowledged(const hardware::Config& config);

  // Loop thread, every tick: collects finished saves and, on a `disabled`
  // tick, hands the newest acknowledged configuration not yet handed over to
  // the saver. Returns how many saves failed since the previous call that
  // collected results. Never waits; touches the mutex only while a save is
  // pending or outstanding.
  uint32_t Tick(bool disabled);

 private:
  // One configuration's ConfigurationId and persisted bytes.
  struct Persisted {
    uint64_t id{0};
    std::vector<uint8_t> bytes;
  };

  void Run();
#ifndef _WIN32
  static void* RunThread(void* persister);
#endif

  const Save save_;

  // Loop-thread state.
  std::optional<Persisted> unsaved_;
  // The ConfigurationId the file holds as of the last results collected.
  uint64_t on_disk_id_;
  // Handed over and not yet collected as finished.
  uint64_t outstanding_{0};
  uint64_t collected_finished_{0};
  uint64_t collected_failed_{0};

  // Shared, under mutex_.
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<Persisted> pending_;
  // The last configuration saved, handed back for Tick() to free.
  std::optional<Persisted> done_;
  uint64_t finished_{0};
  uint64_t failed_{0};
  uint64_t saved_id_;
  bool stop_{false};

  // Saver-thread state: the configuration being saved, read outside the
  // mutex. One saved while stopping, when the loop no longer ticks to collect
  // done_, stays here for the destructor to free.
  std::optional<Persisted> saving_;

  bool started_{false};
#ifdef _WIN32
  std::thread thread_;
#else
  // Not std::thread: its start state is freed by the new thread as it exits.
  pthread_t thread_{};
#endif
};

}  // namespace talos::rio

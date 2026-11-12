#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "hardware/config.h"
#include "hardware/gateway.h"

// The RoboRIO's copy of the last acknowledged hardware configuration, so a
// reboot (a brownout mid-match) drives again without waiting for the companion
// to push it.
namespace talos::rio {

// The persisted configuration, or nullopt when the file is missing or fails
// its CRC, decoding, ceilings or validation.
std::optional<hardware::Config> LoadConfigFile(const std::string& path);

// Writes the configuration durably and atomically: a synced temporary file is
// renamed over `path` and the directory is synced, so a power loss leaves the
// previous file or the new one, never a torn one. Blocks on storage (two
// fsyncs): never call it from the loop thread; ConfigPersister runs it on a
// thread of its own.
bool SaveConfigFile(const std::string& path, const hardware::Config& config);

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
// `save` (SaveConfigFile: two fsyncs, hundreds of milliseconds on a slow
// flash). The two share one mutex, which the saver holds only to move a
// configuration in or a result out, never across a save, and which the loop
// thread only ever try-locks: when it is taken, the tick goes on and the next
// one tries again. The loop thread never waits on the save.
//
// The newest acknowledged configuration is the one left on disk: one
// acknowledged while a save is in flight is saved after it, and one handed
// over but not yet started is replaced by a newer one, so an older
// configuration is never written last. A configuration is handed over only
// on a disabled tick, as before this ran on its own thread (REQ-CONTRACT-06);
// its save may finish after an enable. A configuration the file already holds
// is not saved again unless another was handed over since. A failed save
// leaves the previous file (SaveConfigFile is atomic); Tick() counts it for
// the loop to report, and the same configuration acknowledged again is saved
// again.
class ConfigPersister {
 public:
  using Save = std::function<bool(const hardware::Config&)>;

  // `on_disk_id`: the ConfigurationId the file holds (the restored one), 0
  // when there is none.
  ConfigPersister(Save save, uint64_t on_disk_id);
  ~ConfigPersister();
  ConfigPersister(const ConfigPersister&) = delete;
  ConfigPersister& operator=(const ConfigPersister&) = delete;

  // Starts the saver thread (nice 19 on Linux). The thread takes the
  // scheduling of the thread that starts it, so Robot.cpp starts it before
  // the loop raises itself to SCHED_FIFO; started later, SCHED_RESET_ON_FORK
  // on the loop would still give it the default scheduling.
  void Start();
  // Saves what was handed over and not yet saved, then joins the thread.
  // Called after the loop has stopped ticking (also by the destructor).
  void Stop();

  // Loop thread: a configuration the gateway acknowledged
  // (Endpoint::OnConfigured). Copies it; never waits.
  void Acknowledged(const hardware::Config& config);

  // Loop thread, every tick: collects finished saves and, on a `disabled`
  // tick, hands the newest acknowledged configuration not yet handed over to
  // the saver. Returns how many saves failed since the previous call that
  // collected results. Never waits; touches the mutex only while a save is
  // pending or outstanding.
  uint32_t Tick(bool disabled);

 private:
  void Run();

  const Save save_;

  // Loop-thread state.
  std::optional<hardware::Config> unsaved_;
  // The ConfigurationId the file holds as of the last results collected.
  uint64_t on_disk_id_;
  // Handed over and not yet collected as finished.
  uint64_t outstanding_{0};
  uint64_t collected_finished_{0};
  uint64_t collected_failed_{0};

  // Shared, under mutex_.
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<hardware::Config> pending_;
  uint64_t finished_{0};
  uint64_t failed_{0};
  uint64_t saved_id_;
  bool stop_{false};

  std::thread thread_;
};

}  // namespace talos::rio

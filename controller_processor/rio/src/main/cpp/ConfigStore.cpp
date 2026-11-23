#include "ConfigStore.h"

#include <array>
#include <filesystem>
#include <fstream>
#include <span>
#include <system_error>
#include <utility>

#include "hardware/config_wire.h"

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>

#include "process/durable_sync.h"
#endif

#ifdef __linux__
#include <sys/resource.h>
#include <sys/syscall.h>
#endif

namespace talos::rio {
namespace {

#ifndef _WIN32
bool WriteAll(int fd, std::span<const uint8_t> bytes) {
  std::size_t written = 0;
  while (written < bytes.size()) {
    const auto result =
        ::write(fd, bytes.data() + written, bytes.size() - written);
    if (result < 0 && errno == EINTR) continue;
    if (result <= 0) return false;
    written += static_cast<std::size_t>(result);
  }
  return true;
}

// The rename is durable only once the directory entry is synced.
bool SyncDirectory(const char* directory) {
  const int fd = ::open(directory, O_RDONLY | O_CLOEXEC);
  if (fd < 0) return false;
  const bool synced = process::DurableSync(fd) == 0;
  return ::close(fd) == 0 && synced;
}
#endif

}  // namespace

std::optional<hardware::Config> LoadConfigFile(const std::string& path) {
  std::ifstream file{path, std::ios::binary};
  if (!file) return std::nullopt;
  // Bound the read so a corrupt or oversized file on disk cannot exhaust
  // RoboRIO memory at boot; LoadPersistedConfig rejects any payload larger
  // than kMaxPersistedConfigSize.
  std::array<uint8_t, hardware::kMaxPersistedConfigSize + 1> bytes{};
  file.read(reinterpret_cast<char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
  if (file.bad()) return std::nullopt;
  const auto count = static_cast<std::size_t>(file.gcount());
  if (count == 0 || count > hardware::kMaxPersistedConfigSize) {
    return std::nullopt;
  }
  hardware::Config config;
  if (!hardware::LoadPersistedConfig(std::span{bytes.data(), count}, config)) {
    return std::nullopt;
  }
  return config;
}

ConfigFile::ConfigFile(std::string path)
    : path_{std::move(path)},
      temporary_{path_ + ".tmp"},
      directory_{std::filesystem::path{path_}.parent_path().string()} {
  if (directory_.empty()) directory_ = ".";
}

bool ConfigFile::Save(std::span<const uint8_t> persisted) const {
#ifndef _WIN32
  const int fd = ::open(temporary_.c_str(),
                        O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) return false;
  bool ok = WriteAll(fd, persisted) && process::DurableSync(fd) == 0;
  ok = ::close(fd) == 0 && ok;
  if (ok) ok = std::rename(temporary_.c_str(), path_.c_str()) == 0;
  if (!ok) {
    ::unlink(temporary_.c_str());
    return false;
  }
  return SyncDirectory(directory_.c_str());
#else
  // Desktop simulation only: Windows has no fsync; the rename keeps it atomic.
  std::error_code error;
  bool ok = false;
  {
    std::ofstream file{temporary_, std::ios::binary | std::ios::trunc};
    file.write(reinterpret_cast<const char*>(persisted.data()),
               static_cast<std::streamsize>(persisted.size()));
    file.flush();
    ok = static_cast<bool>(file);
  }
  if (ok) std::filesystem::rename(temporary_, path_, error);
  if (!ok || error) {
    std::filesystem::remove(temporary_, error);
    return false;
  }
  return true;
#endif
}

std::optional<hardware::Config> BeginRestore(hardware::Gateway& gateway,
                                             const std::string& path) {
  auto config = LoadConfigFile(path);
  if (!config || !gateway.BeginReconfigure(*config)) return std::nullopt;
  return config;
}

ConfigPersister::ConfigPersister(Save save, uint64_t on_disk_id)
    : save_{std::move(save)},
      slots_{std::make_unique<std::array<Persisted, 3>>()},
      on_disk_id_{on_disk_id},
      saved_id_{on_disk_id} {}

ConfigPersister::~ConfigPersister() { Stop(); }

void ConfigPersister::Start() {
  if (started_) return;
  stop_ = false;
#ifdef _WIN32
  thread_ = std::thread([this] { Run(); });
#else
  if (const int error = pthread_create(&thread_, nullptr, &RunThread, this))
    throw std::system_error{error, std::generic_category(),
                            "config saver thread"};
#endif
  started_ = true;
}

void ConfigPersister::Stop() {
  if (!started_) return;
  {
    std::lock_guard lock{mutex_};
    // The newest acknowledged configuration not yet handed over (no tick
    // since, an enabled one, or a lost try-lock): the loop no longer ticks,
    // so it is handed over here, superseding one not yet started, as Tick()
    // would. Saving it now after an enable is harmless: the loop has stopped.
    if (has_unsaved_) {
      if (!has_pending_) ++outstanding_;
      std::swap(pending_slot_, unsaved_slot_);
      has_pending_ = true;
      has_unsaved_ = false;
    }
    stop_ = true;
  }
  wake_.notify_one();
#ifdef _WIN32
  thread_.join();
#else
  pthread_join(thread_, nullptr);
#endif
  started_ = false;
}

void ConfigPersister::Acknowledged(const hardware::Config& config) {
  const uint64_t id = hardware::TryConfigurationId(config);
  if (id == 0) return;
  // Already on disk, with nothing handed over since that could be written
  // after it: nothing to save. Otherwise this is the newest, even when the
  // file holds it (A on disk, B in flight, A acknowledged: A must follow B).
  if (outstanding_ == 0 && !has_unsaved_ && id == on_disk_id_) return;
  // Encoded directly into the preallocated unsaved slot on the loop thread:
  // neither thread allocates or frees.
  Persisted& slot = (*slots_)[unsaved_slot_];
  const std::size_t size = hardware::PersistedConfigInto(config, slot.bytes);
  if (size == 0) return;
  slot.id = id;
  slot.size = size;
  has_unsaved_ = true;
}

uint32_t ConfigPersister::Tick(bool disabled) {
  if (!has_unsaved_ && outstanding_ == 0) return 0;
  std::unique_lock lock{mutex_, std::try_to_lock};
  if (!lock.owns_lock()) return 0;  // the saver is moving one; next tick
  outstanding_ -= finished_ - collected_finished_;
  collected_finished_ = finished_;
  const auto failed = static_cast<uint32_t>(failed_ - collected_failed_);
  collected_failed_ = failed_;
  on_disk_id_ = saved_id_;
  bool wake = false;
  if (has_unsaved_ && disabled) {
    // A configuration handed over but not yet started is superseded: its
    // save never runs, so it was never outstanding twice.
    if (!has_pending_) ++outstanding_;
    std::swap(pending_slot_, unsaved_slot_);
    has_pending_ = true;
    has_unsaved_ = false;
    wake = true;
  }
  lock.unlock();
  if (wake) wake_.notify_one();
  return failed;
}

#ifndef _WIN32
void* ConfigPersister::RunThread(void* persister) {
  static_cast<ConfigPersister*>(persister)->Run();
  return nullptr;
}
#endif

// Allocates and frees nothing: every Persisted stays in its preallocated slot
// (swapped by index under mutex_), and save_ is required not to allocate.
void ConfigPersister::Run() {
#ifdef __linux__
  // Below the loop thread, like the logger: a save is not urgent.
  pthread_setname_np(pthread_self(), "talos-config");
  setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 19);
#elif defined(__APPLE__)
  pthread_setname_np("talos-config");
#endif
  std::unique_lock lock{mutex_};
  for (;;) {
    wake_.wait(lock, [this] { return has_pending_ || stop_; });
    // Stop saves what was handed over before returning.
    if (!has_pending_) return;
    std::swap(saving_slot_, pending_slot_);
    has_pending_ = false;
    lock.unlock();
    const Persisted& saving = (*slots_)[saving_slot_];
    const bool ok = save_(saving.span());
    lock.lock();
    ++finished_;
    if (ok) {
      saved_id_ = saving.id;
    } else {
      ++failed_;
    }
  }
}

}  // namespace talos::rio

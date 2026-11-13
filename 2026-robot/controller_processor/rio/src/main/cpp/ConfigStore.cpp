#include "ConfigStore.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <system_error>
#include <utility>
#include <vector>

#include "hardware/config_wire.h"

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>

#include "process/durable_sync.h"
#endif

#ifdef __linux__
#include <pthread.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#endif

namespace talos::rio {
namespace {

#ifndef _WIN32
bool WriteAll(int fd, const std::vector<uint8_t>& bytes) {
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
bool SyncDirectoryOf(const std::filesystem::path& path) {
  auto directory = path.parent_path();
  if (directory.empty()) directory = ".";
  const int fd = ::open(directory.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) return false;
  const bool synced = process::DurableSync(fd) == 0;
  return ::close(fd) == 0 && synced;
}
#endif

}  // namespace

std::optional<hardware::Config> LoadConfigFile(const std::string& path) {
  std::ifstream file{path, std::ios::binary};
  if (!file) return std::nullopt;
  const std::vector<uint8_t> bytes{std::istreambuf_iterator<char>{file}, {}};
  hardware::Config config;
  if (!hardware::LoadPersistedConfig(bytes, config)) return std::nullopt;
  return config;
}

bool SaveConfigFile(const std::string& path, const hardware::Config& config) {
  const auto bytes = hardware::PersistedConfig(config);
  const std::string temporary = path + ".tmp";
  std::error_code error;
#ifndef _WIN32
  const int fd =
      ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) return false;
  bool ok = WriteAll(fd, bytes) && process::DurableSync(fd) == 0;
  ok = ::close(fd) == 0 && ok;
  if (ok) std::filesystem::rename(temporary, path, error);
  if (!ok || error) {
    std::filesystem::remove(temporary, error);
    return false;
  }
  return SyncDirectoryOf(path);
#else
  // Desktop simulation only: Windows has no fsync; the rename keeps it atomic.
  bool ok = false;
  {
    std::ofstream file{temporary, std::ios::binary | std::ios::trunc};
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
    file.flush();
    ok = static_cast<bool>(file);
  }
  if (ok) std::filesystem::rename(temporary, path, error);
  if (!ok || error) {
    std::filesystem::remove(temporary, error);
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
    : save_{std::move(save)}, on_disk_id_{on_disk_id}, saved_id_{on_disk_id} {}

ConfigPersister::~ConfigPersister() { Stop(); }

void ConfigPersister::Start() {
  if (thread_.joinable()) return;
  stop_ = false;
  thread_ = std::thread([this] { Run(); });
}

void ConfigPersister::Stop() {
  if (!thread_.joinable()) return;
  {
    std::lock_guard lock{mutex_};
    stop_ = true;
  }
  wake_.notify_one();
  thread_.join();
}

void ConfigPersister::Acknowledged(const hardware::Config& config) {
  // Already on disk, with nothing handed over since that could be written
  // after it: nothing to save. Otherwise this is the newest, even when the
  // file holds it (A on disk, B in flight, A acknowledged: A must follow B).
  if (outstanding_ == 0 && !unsaved_ &&
      hardware::ConfigurationId(config) == on_disk_id_)
    return;
  unsaved_ = config;
}

uint32_t ConfigPersister::Tick(bool disabled) {
  if (!unsaved_ && outstanding_ == 0) return 0;
  std::unique_lock lock{mutex_, std::try_to_lock};
  if (!lock.owns_lock()) return 0;  // the saver is moving one; next tick
  outstanding_ -= finished_ - collected_finished_;
  collected_finished_ = finished_;
  const auto failed = static_cast<uint32_t>(failed_ - collected_failed_);
  collected_failed_ = failed_;
  on_disk_id_ = saved_id_;
  // A saved configuration comes back to be freed here, after the unlock: its
  // memory is this thread's (see done_).
  std::optional<hardware::Config> saved = std::move(done_);
  done_.reset();
  bool wake = saved.has_value() && pending_.has_value();
  if (unsaved_ && disabled) {
    // A configuration handed over but not yet started is superseded: its
    // save never runs, so it was never outstanding twice.
    if (!pending_) ++outstanding_;
    pending_ = std::move(unsaved_);
    unsaved_.reset();
    wake = true;
  }
  lock.unlock();
  if (wake) wake_.notify_one();
  return failed;
}

void ConfigPersister::Run() {
#ifdef __linux__
  // Below the loop thread, like the logger: a save is not urgent.
  pthread_setname_np(pthread_self(), "talos-config");
  setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 19);
#endif
  std::unique_lock lock{mutex_};
  for (;;) {
    // The next save waits for the loop to take the last one back (done_),
    // except when stopping.
    wake_.wait(lock, [this] {
      return (pending_.has_value() && !done_.has_value()) || stop_;
    });
    // Stop saves what was handed over first.
    if (!pending_) return;
    // Moves the configuration's storage, and leaves pending_ empty with
    // nothing to free.
    hardware::Config config = std::move(*pending_);
    pending_.reset();
    lock.unlock();
    const bool ok = save_(config);
    const uint64_t id = ok ? hardware::ConfigurationId(config) : 0;
    lock.lock();
    ++finished_;
    if (ok) {
      saved_id_ = id;
    } else {
      ++failed_;
    }
    // Back to the loop thread, which allocated it, to free: `config` is left
    // empty. Only while stopping, when the loop no longer ticks, can done_
    // still hold the last one; this thread frees that.
    done_ = std::move(config);
  }
}

}  // namespace talos::rio

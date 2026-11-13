#include "ConfigStore.h"

#include <filesystem>
#include <fstream>
#include <iterator>
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
  const std::vector<uint8_t> bytes{std::istreambuf_iterator<char>{file}, {}};
  hardware::Config config;
  if (!hardware::LoadPersistedConfig(bytes, config)) return std::nullopt;
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
    : save_{std::move(save)}, on_disk_id_{on_disk_id}, saved_id_{on_disk_id} {}

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
  const uint64_t id = hardware::ConfigurationId(config);
  // Already on disk, with nothing handed over since that could be written
  // after it: nothing to save. Otherwise this is the newest, even when the
  // file holds it (A on disk, B in flight, A acknowledged: A must follow B).
  if (outstanding_ == 0 && !unsaved_ && id == on_disk_id_) return;
  // Encoded here, on the loop thread, which owns the bytes from here to the
  // Tick() that frees them: the saver neither allocates nor frees.
  unsaved_ = Persisted{id, hardware::PersistedConfig(config)};
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
  // A saved configuration comes back to be freed here, after the unlock.
  std::optional<Persisted> saved = std::move(done_);
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

#ifndef _WIN32
void* ConfigPersister::RunThread(void* persister) {
  static_cast<ConfigPersister*>(persister)->Run();
  return nullptr;
}
#endif

// Allocates and frees nothing: every Persisted moves between slots that are
// empty (moving a vector out leaves nothing to free), and save_ is required
// not to.
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
    // The next save waits for the loop to take the last one back (done_),
    // except when stopping.
    wake_.wait(lock, [this] {
      return (pending_.has_value() && !done_.has_value()) || stop_;
    });
    // Stop saves what was handed over first. saving_ is always empty here:
    // it stays occupied only after a save while stopping, and the loop, no
    // longer ticking, hands over nothing after that one.
    if (!pending_ || saving_) return;
    saving_.emplace(std::move(*pending_));
    pending_.reset();
    lock.unlock();
    const bool ok = save_(saving_->bytes);
    lock.lock();
    ++finished_;
    if (ok) {
      saved_id_ = saving_->id;
    } else {
      ++failed_;
    }
    // Back to the loop thread, which allocated it, to free. Only while
    // stopping can done_ still hold the last one; this one then stays in
    // saving_ (see there).
    if (!done_) {
      done_.emplace(std::move(*saving_));
      saving_.reset();
    }
  }
}

}  // namespace talos::rio

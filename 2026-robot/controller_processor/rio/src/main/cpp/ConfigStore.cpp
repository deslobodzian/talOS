#include "ConfigStore.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <system_error>
#include <vector>

#include "hardware/config_wire.h"

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>

#include "process/durable_sync.h"
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

}  // namespace talos::rio

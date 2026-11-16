#include "talOS/memory/residency.h"

#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <system_error>
#include <utility>
#include <vector>

namespace talos::memory {

namespace {

// Defined once, in this file, for the reason process.h gives for
// stop_requested: a node library loaded RTLD_LOCAL must share these with
// libtalos_node.so.
std::mutex mappings_mutex;
std::vector<std::pair<void*, std::size_t>> mappings;
std::atomic<bool> keep_resident{false};
std::atomic<bool> reported_late_failure{false};

std::size_t PageSize() {
  static const auto page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
  return page;
}

}  // namespace

int MakeResident(void* address, std::size_t size) noexcept {
  if (address == nullptr || size == 0) return 0;
  // A write fault, not a read: a read can map a shared page without write
  // permission, and the first publish would then fault anyway. OR with zero
  // changes nothing, even under a concurrent writer in another process.
  auto* bytes = static_cast<unsigned char*>(address);
  for (std::size_t offset = 0; offset < size; offset += PageSize()) {
    __atomic_fetch_or(reinterpret_cast<std::uint64_t*>(bytes + offset),
                      std::uint64_t{0}, __ATOMIC_RELAXED);
  }
  return ::mlock(address, size) == 0 ? 0 : errno;
}

Residency KeepMappingsResident() noexcept {
  Residency result;
  try {
    const std::scoped_lock lock{mappings_mutex};
    keep_resident.store(true, std::memory_order_relaxed);
    for (const auto& [address, size] : mappings) {
      ++result.ranges;
      result.bytes += size;
      if (const int error = MakeResident(address, size); error != 0) {
        ++result.failed;
        if (result.first_error == 0) result.first_error = error;
      }
    }
  } catch (const std::system_error& e) {
    // The mutex refused; nothing registered could be reached.
    result.first_error = e.code().value();
    result.failed = 1;
  }
  return result;
}

bool MappingsKeptResident() noexcept {
  return keep_resident.load(std::memory_order_relaxed);
}

void TrackMapping(void* address, std::size_t size) {
  const std::scoped_lock lock{mappings_mutex};
  mappings.emplace_back(address, size);
  if (!keep_resident.load(std::memory_order_relaxed)) return;
  const int error = MakeResident(address, size);
  if (error != 0 && !reported_late_failure.exchange(true)) {
    std::fprintf(stderr,
                 "talos: shared memory mapped after realtime setup is "
                 "prefaulted but not locked (mlock: %s); further refusals "
                 "are not reported\n",
                 std::strerror(error));
  }
}

// Not caught: std::mutex::lock throws only on a broken mutex, and a stale
// entry would have KeepMappingsResident touch unmapped memory, so ending the
// process (this runs from a noexcept destructor) is the safer failure.
void UntrackMapping(void* address) noexcept {
  const std::scoped_lock lock{mappings_mutex};
  const auto found = std::find_if(
      mappings.begin(), mappings.end(),
      [address](const auto& entry) { return entry.first == address; });
  if (found != mappings.end()) mappings.erase(found);
}

}  // namespace talos::memory

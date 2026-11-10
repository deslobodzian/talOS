#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#if defined(__APPLE__)
#include <sys/sysctl.h>
#endif

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

#include "talOS/memory/shared_memory_ptr.h"

namespace autonomous {

// What one armed routine has done, so a restarted process resumes it instead
// of firing again. Times are on the node clock, which in production is the
// boot's monotonic clock (CLOCK_MONOTONIC on Linux, mach_continuous_time on
// macOS: talOS/events/os): it survives a process restart and restarts with
// the boot, so `boot` ties the times to the boot that measured them.
struct Progress {
  std::uint64_t boot = 0;       // BootId() of the writer
  std::uint64_t signature = 0;  // the routine list the indices refer to
  std::int64_t armed_at_ns = 0;
  std::int64_t spinup_started_ns = 0;
  std::int64_t drive_started_ns = 0;
  std::int64_t feed_elapsed_ns = 0;  // feed time commanded so far
  std::uint32_t action = 0;          // step within the routine
  std::uint16_t match_number = 0;
  std::uint8_t routine = 0;
  std::uint8_t match_type = 0;
  std::uint8_t replay_number = 0;
  std::uint8_t armed = 0;    // 0: no routine armed (cleared by a new match)
  std::uint8_t aborted = 0;  // a deadline aborted the routine at `action`
  std::uint8_t reserved[5] = {};
  char event_name[32] = {};

  bool operator==(const Progress&) const = default;
};
static_assert(sizeof(Progress) == 96 &&
                  std::has_unique_object_representations_v<Progress>,
              "Progress is hashed bytewise: no padding");

// Each copy of the record: a version and checksum, and a sequence number so
// the newer of two whole copies wins.
struct ProgressSlot {
  std::uint32_t magic;
  std::uint32_t version;
  std::uint64_t sequence;
  Progress progress;
  std::uint64_t checksum;  // FNV-1a 64 of every byte before it
};
static_assert(std::has_unique_object_representations_v<ProgressSlot>);
struct ProgressFile {
  ProgressSlot slots[2];
};

// The shared-memory object holding the record, named through ShmObjectName
// (so TALOS_SHM_NAMESPACE or Bazel's TEST_TMPDIR isolates it). Not a topic:
// the launcher creates and unlinks only declared topics' segments, so it
// survives node and launcher restarts and lasts until reboot.
inline constexpr std::string_view kProgressObject = "/autonomous/progress";

// The latest Progress, in kProgressObject. Mapped once at construction;
// Write is a memory copy into memory-only shared memory (tmpfs on Linux),
// with no system call and no disk behind it, so update() never waits on
// writeback. A record from another boot is ignored.
//
// Two slots, written alternately with a rising sequence number. A process
// killed mid-write leaves that slot torn and the other one whole: the whole
// one is what the node had written before its last commands went out (Write
// precedes them), so it is still the truth. Both slots damaged, or an object
// of another size, is kCorrupt.
class ProgressStore {
 public:
  enum class Loaded { kNone, kValid, kCorrupt };

  // No store without a session directory (tests, --describe, --replay).
  explicit ProgressStore(const std::optional<std::filesystem::path>& session) {
    if (!session) return;
    boot_ = BootId();
    const std::string name = ::ShmObjectName(kProgressObject);
    int fd = ::shm_open(name.c_str(), O_RDWR | O_CREAT, 0600);
    if (fd < 0) throw std::runtime_error("autonomous: open progress failed");
    struct stat status{};
    bool damaged = false;
    // macOS rounds the size up to a page, so only a smaller object is
    // another layout. macOS sizes a shared-memory object only once, so
    // replace it; what it held is unreadable.
    if (::fstat(fd, &status) == 0 && status.st_size != 0 &&
        status.st_size < static_cast<off_t>(sizeof(ProgressFile))) {
      ::close(fd);
      ::shm_unlink(name.c_str());
      fd = ::shm_open(name.c_str(), O_RDWR | O_CREAT, 0600);
      if (fd < 0) throw std::runtime_error("autonomous: open progress failed");
      status.st_size = 0;
      damaged = true;
    }
    if (status.st_size == 0 && ::ftruncate(fd, sizeof(ProgressFile)) != 0) {
      ::close(fd);
      throw std::runtime_error("autonomous: size progress failed");
    }
    void* memory = ::mmap(nullptr, sizeof(ProgressFile), PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd, 0);
    ::close(fd);
    if (memory == MAP_FAILED)
      throw std::runtime_error("autonomous: map progress failed");
    file_ = static_cast<ProgressFile*>(memory);
    Load(damaged);
  }
  ~ProgressStore() {
    if (file_) ::munmap(file_, sizeof(ProgressFile));
  }
  ProgressStore(const ProgressStore&) = delete;
  ProgressStore& operator=(const ProgressStore&) = delete;

  bool enabled() const { return file_ != nullptr; }
  // What construction found. kValid only for an armed record of this boot.
  Loaded loaded() const { return loaded_; }
  const Progress& progress() const { return last_; }

  // Records `progress` (stamped with this boot) if it differs from the last
  // record. No-op without a store.
  void Write(Progress progress) {
    if (!file_) return;
    progress.boot = boot_;
    if (written_ && progress == last_) return;
    Slot& slot = file_->slots[++sequence_ % 2];
    Slot next{kMagic, kVersion, sequence_, progress, 0};
    next.checksum = Checksum(next);
    // Plain stores: the only reader is a later process, after this one has
    // died, and stores a process executed are in the shared pages whatever
    // happens to it. The compiler cannot defer them past the lane publishes
    // that follow, which are calls that may read memory.
    std::memcpy(&slot, &next, sizeof(Slot));
    last_ = progress;
    written_ = true;
  }

 private:
  static constexpr std::uint32_t kMagic = 0x6f747561;  // "auto"
  static constexpr std::uint32_t kVersion = 1;
  using Slot = ProgressSlot;

  static std::uint64_t Checksum(const Slot& slot) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(&slot);
    std::uint64_t hash = 14695981039346656037ULL;
    for (std::size_t i = 0; i < offsetof(Slot, checksum); ++i)
      hash = (hash ^ bytes[i]) * 1099511628211ULL;
    return hash;
  }

  // Checks local copies of the slots and restores from the checked copy,
  // never from the mapping, which another process may be writing.
  void Load(bool damaged) {
    std::optional<Slot> newest;
    for (const Slot& stored : file_->slots) {
      Slot slot{};
      std::memcpy(&slot, &stored, sizeof(Slot));
      const auto* bytes = reinterpret_cast<const unsigned char*>(&slot);
      const bool empty = std::all_of(bytes, bytes + sizeof(Slot),
                                     [](unsigned char b) { return b == 0; });
      if (empty) continue;
      if (slot.magic != kMagic || slot.version != kVersion ||
          slot.checksum != Checksum(slot)) {
        damaged = true;
        continue;
      }
      if (!newest || slot.sequence > newest->sequence) newest = slot;
    }
    if (!newest) {
      loaded_ = damaged ? Loaded::kCorrupt : Loaded::kNone;
      return;
    }
    sequence_ = newest->sequence;
    if (newest->progress.boot != boot_ || !newest->progress.armed) return;
    loaded_ = Loaded::kValid;
    last_ = newest->progress;
    written_ = true;
  }

  // Identifies this boot of the host; 0 never names one.
  static std::uint64_t BootId() {
    std::string id;
#if defined(__APPLE__)
    // A UUID per boot. Not kern.boottime, which moves when the calendar clock
    // is stepped (NTP), and would make this boot's record look foreign.
    char uuid[64] = {};
    std::size_t size = sizeof uuid - 1;
    if (::sysctlbyname("kern.bootsessionuuid", uuid, &size, nullptr, 0) == 0)
      id.assign(uuid, ::strnlen(uuid, sizeof uuid));
#else
    std::ifstream file{"/proc/sys/kernel/random/boot_id"};
    std::getline(file, id);
#endif
    if (id.empty()) throw std::runtime_error("autonomous: no boot id");
    std::uint64_t hash = 14695981039346656037ULL;
    for (unsigned char c : id) hash = (hash ^ c) * 1099511628211ULL;
    return hash | 1;
  }

  ProgressFile* file_ = nullptr;
  std::uint64_t boot_ = 0;
  std::uint64_t sequence_ = 0;
  Loaded loaded_ = Loaded::kNone;
  Progress last_{};
  bool written_ = false;
};

}  // namespace autonomous

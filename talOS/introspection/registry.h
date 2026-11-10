#pragma once

/*
 * Live node registry.
 *
 * RTMS topics are POSIX shared-memory objects, and there is no portable way to
 * list them: a process holding a topic open is invisible to everyone else. So a
 * tool cannot answer "what is running, and who publishes what" by looking at
 * the transport. It has to be told.
 *
 * This is where every node says so. One shared-memory segment holds a fixed
 * array of slots; a node claims one at startup, writes its event-loop manifest
 * into it -- every timer, watcher, fetcher and sender, by name and message size
 * -- and then, while it runs, keeps current a heartbeat, a set of counters,
 * the lane each of its Targets selected and whether its event log stopped
 * early. Studio and any agent map the segment read-only and get the whole
 * system: which processes exist, which topics each one publishes and
 * subscribes to, how much traffic has actually moved, who drives each
 * subsystem, and which recordings are incomplete.
 *
 * The layout is frozen and the version is part of the segment name, so a node
 * from an older build cannot be misread by a newer reader: it lands in a
 * different segment instead.
 *
 * Writers touch only their own slot. Every field a reader observes while a node
 * runs is a lock-free atomic, and identity fields are fenced behind a
 * generation counter, so reading requires no cooperation from the nodes and
 * cannot block them.
 *
 * The counters move together behind a sample sequence (see NodeRecord): the
 * reporting thread brackets each refresh, and a reader that copies a slot
 * mid-sample retries a bounded number of times before accepting the plain copy
 * an older reader would have taken. A torn snapshot is re-read, never waited
 * on, so a writer that dies mid-sample cannot wedge a viewer. The residual
 * staleness is bounded by one refresh period, which no consumer branches on:
 * liveness reads the heartbeat alone against a 5 s timeout, and health
 * aggregates cumulative gauges. An event-loop node's dispatch count is the
 * exception: its loop thread stores it at every dispatch, outside the sample,
 * because the launcher's stall check reads it as the loop's own progress
 * and must not depend on when the reporting thread gets scheduled.
 */

#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <new>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/introspection/lock_file.h"
#include "talOS/introspection/names.h"
#include "talOS/memory/shared_memory_ptr.h"
#include "talOS/rtms/process_liveness.h"

namespace talos::introspect {

// Bumped whenever the records below change shape. It is part of the segment
// name, so a mismatched build gets its own segment rather than a garbled view
// of someone else's.
//
// Endpoint attributes did not bump it, and that is deliberate rather than an
// oversight. They took over SourceRecord::reserved, a word that every build
// which has ever written this layout wrote as zero: no offset, size or
// alignment moves, and a writer that predates attributes stores 0, which reads
// as "no attributes declared" -- the correct answer for a node that declares
// none. So old writers and new readers, and new writers and old readers, all
// agree. Bumping would have bought nothing and cost a flag day, because the
// version is in the segment name: a viewer would stop seeing every node that
// had not been rebuilt yet.
//
// The sample sequence did not bump it either, for the same reason: it reuses
// NodeRecord::reserved, a word every build which has ever written this layout
// wrote as zero, which reads as "no sample in progress" -- the correct answer
// for a writer that predates the guard, whose stores land around reads of an
// undisturbed zero.
//
// Version 2 is a real layout change: NodeRecord grew the selection records at
// its end, which moves sizeof(NodeRecord) and every slot after the first. No
// reserved word was left to reuse, and a table of source names per Target does
// not fit in one anyway. A version-1 viewer therefore sees an empty registry
// rather than misreading the new records, until it is rebuilt.
// Version 3 changes the heartbeat epoch from wall time to the host monotonic
// clock. Mixing epochs would make healthy nodes appear stale.
// Version 4 adds NodeRecord::pid_start, so a row whose claimant died is
// freed at once and a reused pid does not keep a dead claimant's row.
// Version 5 narrows the pid to 22 bits of the process word and records the
// claimant's pid namespace above its start (process_liveness.h): a version-4
// reader would misread the start and free live rows.
//
// The event log flags (kFlagRecorderFailed, kFlagLogCapped) did not bump it:
// they are bits of `flags` that every earlier writer left clear, which reads
// as "no stopped log seen" -- all an earlier build could have said -- and
// every earlier reader tests only the bits it knows.
//
// Nor did naming::kSourceFlagLatestOnly: a bit of SourceRecord::flags that
// every earlier writer left clear, which reads as "every drop is a loss" --
// what an earlier reader assumed of every source.
inline constexpr std::uint32_t kRegistryVersion = 5;
inline constexpr const char* kRegistryPath = "/talos_registry.5";
// The segment's mode, whatever the creator's umask: every node registers,
// whichever user runs it, and the registry provides no authentication.
inline constexpr mode_t kRegistryMode = 0666;

inline constexpr std::size_t kMaxNodes = 32;
inline constexpr std::size_t kMaxSourcesPerNode = 64;

// A node holds one Target per subsystem it drives -- in practice one -- and a
// policy lists a handful of lanes; both caps are far above either.
inline constexpr std::size_t kMaxSelectionsPerNode = 4;
inline constexpr std::size_t kMaxSelectionSources = 8;

// event::MAX_SOURCE_NAME plus the terminator, so any name the event loop
// accepts fits here without truncation.
inline constexpr std::size_t kMaxNameBytes = event::MAX_SOURCE_NAME + 1;
inline constexpr std::size_t kMaxTargetBytes = 128;

// A node that has not checked in for this long is treated as gone. Generous on
// purpose: a process stopped in a debugger is not a dead process, and the only
// cost of waiting is a stale row in a viewer.
inline constexpr std::chrono::seconds kLivenessTimeout{5};

enum class SlotState : std::uint64_t {
  FREE = 0,
  CLAIMING = 1,
  ACTIVE = 2,
};

// Bit flags describing how a node is running.
inline constexpr std::uint32_t kFlagSimulation = 1u << 0;
inline constexpr std::uint32_t kFlagReplay = 1u << 1;
// Lifecycle flag: identity is visible, but construction/topology is incomplete.
// SlotState::ACTIVE means the slot is owned, not that the node is dispatching.
inline constexpr std::uint32_t kFlagConstructing = 1u << 2;
inline constexpr std::chrono::seconds kConstructionTimeout{30};

// Event log flags: the node's event log stopped early, so the rest of its run
// is not recorded. kFlagRecorderFailed mirrors LogWriter::failed() -- a disk
// error, or capture abandoned because the writer fell behind or a record did
// not fit -- and kFlagLogCapped mirrors capped(), the log reaching its byte
// budget. Set while the node runs (NodeRegistration::set_recording) and never
// cleared, because capture never resumes; a new claim starts without them.
inline constexpr std::uint32_t kFlagRecorderFailed = 1u << 3;
inline constexpr std::uint32_t kFlagLogCapped = 1u << 4;

// One event source, as the owning node registered it. The descriptive fields
// are written once, before the slot goes ACTIVE; the counters keep moving.
struct SourceRecord {
  std::uint16_t id;
  std::uint16_t kind;  // event::SourceKind
  std::uint32_t message_bytes;
  std::uint32_t alignment;

  // naming::kSourceFlagExternal, naming::kSourceFlagOptional: how this end's
  // far end is expected to behave, so a graph tool can tell a designed dead
  // end from a fault. naming::kSourceFlagLatestOnly: `dropped` below includes
  // skips by design. Written once with the descriptive fields, never after.
  //
  // This field was `reserved`. See kRegistryVersion for why reusing it needed
  // no version bump.
  std::uint32_t flags;

  std::int64_t period_ns;
  char name[kMaxNameBytes];

  // Dispatches for a timer or watcher, messages published for a sender,
  // successful reads for a fetcher: in every case, how often this source did
  // its job.
  std::atomic<std::uint64_t> events;

  // Messages the transport lapped before this source could read them, messages
  // a latest-only watcher (kSourceFlagLatestOnly) skipped by design, publishes
  // the transport refused, or reads it failed (SourceCounters).
  std::atomic<std::uint64_t> dropped;

  // Transport sequence last observed, which is what makes a subscriber's lag
  // computable against its publisher.
  std::atomic<std::uint64_t> sequence;

  std::atomic<std::int64_t> last_monotonic_ns;
  std::atomic<std::int64_t> last_latency_ns;
  std::atomic<std::int64_t> max_latency_ns;
};

// One talos::Target (talOS/devices/target.h): the lanes its policy names, and
// which of them it selected last. The names are written once, before the
// selection count is published; `selected` keeps moving.
struct SelectionRecord {
  // The policy's sources, in TargetPolicy::Sources() order, as far as
  // kMaxSelectionSources holds; declared_source_count is how many it named.
  std::uint32_t source_count;
  std::uint32_t declared_source_count;
  char sources[kMaxSelectionSources][kMaxNameBytes];

  // Index into the policy's sources of the lane the Target's last get()
  // returned, or -1 when it returned null: robot disabled, Driver Station
  // stale, or no live lane in the current mode.
  std::atomic<std::int32_t> selected;
};

// One node. Padded to a cache line so two nodes writing their own counters do
// not contend on the same line.
struct alignas(64) NodeRecord {
  std::atomic<std::uint64_t> state;

  // Bumped on every claim. A reader that sees the same generation before and
  // after copying the identity fields knows the slot was not recycled
  // underneath it.
  std::atomic<std::uint64_t> generation;

  // Host monotonic time is shared by processes and cannot jump with NTP.
  std::atomic<std::int64_t> heartbeat_monotonic_ns;

  // Published after the source records are written, so a reader never walks
  // entries that are still being filled in.
  std::atomic<std::uint32_t> source_count;
  std::atomic<std::uint32_t> flags;

  // The node's progress, which the launcher's stall check reads. An
  // event-loop node's loop thread stores it at every dispatch
  // (event::EventLoopBase::set_dispatch_observer), outside the sample
  // sequence; a node with its own loop (the hardware bridge) stores it in its
  // samples.
  std::atomic<std::uint64_t> dispatch_count;

  // How many sources the node registered, which exceeds source_count when it
  // registered more than this segment can hold.
  std::atomic<std::uint32_t> declared_source_count;

  // Seqlock for one refresh: the writer brackets the heartbeat, (for a node
  // without an event loop) dispatch, per-source counter and selection stores
  // with begin_sample/end_sample (odd
  // while a sample is landing, even when the slot is stable), and the reader
  // retries a copy taken across a change. This reuses the word `reserved`
  // held, so it moved no field; see kRegistryVersion for why that reuse is
  // sound. Claim resets it: a recycled slot may hold an odd sequence from a
  // writer that died mid-sample.
  std::atomic<std::uint32_t> sample_seq;

  std::int64_t start_wall_ns;
  std::uint64_t pid;
  // The claimant's process word above kProcessPidBits: its start time and
  // pid namespace (talOS/rtms/process_liveness.h); 0 compares the pid alone.
  std::uint64_t pid_start;
  std::uint64_t session_id;
  char name[kMaxNameBytes];
  char target[kMaxTargetBytes];

  SourceRecord sources[kMaxSourcesPerNode];

  // The node's Targets, in the order it opened them. Published like the
  // sources: the records first, then the count, with release.
  std::atomic<std::uint32_t> selection_count;
  std::atomic<std::uint32_t> declared_selection_count;
  SelectionRecord selections[kMaxSelectionsPerNode];
};

struct RegistryHeader {
  // Initialization is serialized by RegistryLock. kMagic is committed last;
  // an opener holding the lock can restart any uncommitted initialization.
  std::atomic<std::uint64_t> magic;
  std::uint32_t version;
  std::uint32_t node_capacity;
  std::uint32_t source_capacity;
  std::uint32_t node_record_bytes;
  std::uint32_t source_record_bytes;
  std::uint32_t reserved;

  NodeRecord nodes[kMaxNodes];
};

inline constexpr std::uint64_t kMagic = 0x74616c4f53726567ull;  // "talOSreg"
inline constexpr std::uint64_t kInitializing = 1;

static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "the registry is read across processes without a lock");
static_assert(std::atomic<std::int64_t>::is_always_lock_free);
static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
static_assert(std::atomic<std::int32_t>::is_always_lock_free);
static_assert(std::is_standard_layout_v<RegistryHeader>);

// Why endpoint attributes needed no kRegistryVersion bump: `flags` occupies
// the word `reserved` held, immediately after `alignment`, so no field after it
// moved. A future edit that inserts something here instead of reusing a
// reserved word has changed the layout and must bump the version.
static_assert(std::is_standard_layout_v<SourceRecord>);
static_assert(offsetof(SourceRecord, flags) ==
              offsetof(SourceRecord, alignment) + sizeof(std::uint32_t));

// `sample_seq` reuses a plain word, so it must not grow or move the record.
static_assert(sizeof(std::atomic<std::uint32_t>) == sizeof(std::uint32_t));
static_assert(offsetof(NodeRecord, sample_seq) ==
              offsetof(NodeRecord, declared_source_count) +
                  sizeof(std::uint32_t));
static_assert(offsetof(NodeRecord, start_wall_ns) ==
              offsetof(NodeRecord, sample_seq) + sizeof(std::uint32_t));

inline std::int64_t wall_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

inline std::int64_t monotonic_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

inline void copy_name(char* destination, std::size_t capacity,
                      std::string_view value) {
  const std::size_t n =
      value.size() < capacity - 1 ? value.size() : capacity - 1;
  std::memcpy(destination, value.data(), n);
  std::memset(destination + n, 0, capacity - n);
}

inline std::string read_name(const char* value, std::size_t capacity) {
  const std::size_t n = ::strnlen(value, capacity);
  return std::string{value, n};
}

// Optional startup-only fault-injection seams. Production callers leave these
// null; tests stop a child while it owns the lock, before either commit.
struct RegistryTestHooks {
  void (*initializing)(RegistryHeader*){nullptr};
  void (*claiming)(NodeRecord*){nullptr};
};

// Serializes the writers: segment creation, sizing and initialization
// (RegistryMapping) and slot claims (NodeRegistration); a reader never takes
// it. Use a regular file (LockFilePath): Darwin's shm descriptors are not
// vnodes, so flock on them fails with ENOTSUP. Each acquisition opens a
// separate file description, which also serializes threads in one process.
// close (including process death) releases the lock. Nothing in talos
// unlinks this file; if something else does, an acquirer that wins the old
// inode finds it no longer current and starts over.
class RegistryLock {
 public:
  explicit RegistryLock(const std::string& mapped_path)
      : RegistryLock(mapped_path, std::nullopt) {}

  // Waits at most `wait` when it is set, then throws: a holder that keeps
  // the lock that long is stuck (stopped mid-registration), not registering.
  RegistryLock(const std::string& mapped_path,
               std::optional<std::chrono::milliseconds> wait) {
    // Namespace selection is entirely ShmObjectName's job: the lock file is
    // named after its mapped result.
    const std::string file_path = LockFilePath(mapped_path);
    const auto waited = wait.value_or(std::chrono::milliseconds::zero());
    const auto deadline = std::chrono::steady_clock::now() + waited;
    const int operation = wait ? (LOCK_EX | LOCK_NB) : LOCK_EX;
    // A lock on a file unlinked meanwhile (a /tmp cleaner) excludes nobody
    // who opens the path anew, so it is taken again on the file the path
    // names (LockFileIsCurrent).
    for (;;) {
      // flock does not require write access, so the file is opened
      // read-only: any process that may read it can lock it, whichever user
      // created it.
      fd_ = OpenLockFile(file_path, O_RDONLY);
      if (fd_ < 0) {
        throw std::runtime_error("registry lock open failed for " + file_path +
                                 ": " + std::strerror(errno));
      }
      while (::flock(fd_, operation) < 0) {
        const int error = errno;
        if (error == EINTR) continue;
        if (error == EWOULDBLOCK &&
            std::chrono::steady_clock::now() < deadline) {
          ::usleep(1000);
          continue;
        }
        ::close(fd_);
        if (error == EWOULDBLOCK) {
          throw std::runtime_error("registry lock " + file_path +
                                   " has been held for " +
                                   std::to_string(waited.count()) +
                                   " ms by another process, likely one "
                                   "stopped while it registered a node");
        }
        throw std::runtime_error("registry lock failed for " + file_path +
                                 ": " + std::strerror(error));
      }
      if (LockFileIsCurrent(fd_, file_path)) return;
      ::close(fd_);
    }
  }

  ~RegistryLock() { ::close(fd_); }
  RegistryLock(const RegistryLock&) = delete;
  RegistryLock& operator=(const RegistryLock&) = delete;

 private:
  int fd_{-1};
};

// Gives a segment this process's user owns kRegistryMode whatever umask
// created it, so a node or tool running as another user can open it too.
// Best effort: on macOS shm_open applies no umask and fchmod refuses shm
// descriptors, and a mode this cannot fix is reported by whoever it stops
// (PrepareRegistry).
inline void ShareRegistrySegment(int fd, const struct stat& status) {
  if (status.st_uid == ::geteuid() &&
      (status.st_mode & 07777) != kRegistryMode) {
    static_cast<void>(::fchmod(fd, kRegistryMode));
  }
}

// Maps the registry segment, creating it if this is the first process to ask.
//
// Deliberately not SharedMemoryPtr: that unlinks on destruction, which is right
// for a topic owned by one publisher and wrong here. The registry outlives any
// single node, so nothing unlinks it -- a stale segment is harmless because
// slots are reclaimed by heartbeat or claimant death and the version is in the
// name.
class RegistryMapping {
 public:
  explicit RegistryMapping(std::string_view path = kRegistryPath,
                           bool read_only = false,
                           const RegistryTestHooks* test_hooks = nullptr)
      : path_{ShmObjectName(path)} {
    // Only a writer creates, sizes or initializes the segment, so only a
    // writer takes RegistryLock. A reader validates the commit instead: kMagic
    // is release-stored after every header field and never reset once stored,
    // so a segment a writer is still creating or initializing opens as absent,
    // too small or uncommitted, for the reader to retry. No lock holder -- a
    // node stopped mid-registration, or a descriptor a fork inherited -- can
    // therefore block a reader, such as the launcher's supervision loop.
    std::optional<RegistryLock> lock;
    if (!read_only) lock.emplace(path_);
    map(read_only, test_hooks);
  }

  // A writer's mapping for a caller that already holds `held`, this path's
  // RegistryLock (PrepareRegistry); taking it again would wait on itself.
  RegistryMapping(std::string_view path, const RegistryLock& held)
      : path_{ShmObjectName(path)} {
    static_cast<void>(held);
    map(false, nullptr);
  }

  ~RegistryMapping() {
    if (header_ != nullptr) {
      ::munmap(header_, sizeof(RegistryHeader));
    }
  }

  RegistryMapping(const RegistryMapping&) = delete;
  RegistryMapping& operator=(const RegistryMapping&) = delete;

  RegistryMapping(RegistryMapping&& other) noexcept
      : path_{std::move(other.path_)},
        header_{std::exchange(other.header_, nullptr)} {}

  RegistryMapping& operator=(RegistryMapping&& other) noexcept {
    if (this != &other) {
      if (header_ != nullptr) {
        ::munmap(header_, sizeof(RegistryHeader));
      }
      path_ = std::move(other.path_);
      header_ = std::exchange(other.header_, nullptr);
    }
    return *this;
  }

  RegistryHeader* header() const { return header_; }
  const std::string& path() const { return path_; }

  // Removes the segment's name. Only tests need this; a robot leaves the
  // registry in place between runs.
  static void unlink(std::string_view path = kRegistryPath) {
    ::shm_unlink(ShmObjectName(path).c_str());
  }

 private:
  void map(bool read_only, const RegistryTestHooks* test_hooks) {
    const std::size_t size = sizeof(RegistryHeader);
    const int flags = read_only ? O_RDONLY : (O_CREAT | O_RDWR);

    int fd = ::shm_open(path_.c_str(), flags, kRegistryMode);
    if (fd < 0) {
      throw std::runtime_error("registry shm_open failed for " + path_ + ": " +
                               std::strerror(errno));
    }

    struct stat status {};
    if (::fstat(fd, &status) < 0) {
      ::close(fd);
      throw std::runtime_error("registry fstat failed for " + path_);
    }
    if (!read_only) ShareRegistrySegment(fd, status);

    if (!read_only && static_cast<std::size_t>(status.st_size) < size &&
        ::ftruncate(fd, static_cast<off_t>(size)) < 0) {
      ::close(fd);
      throw std::runtime_error("registry ftruncate failed for " + path_);
    }

    if (read_only && static_cast<std::size_t>(status.st_size) < size) {
      ::close(fd);
      throw std::runtime_error("registry segment " + path_ + " is too small");
    }

    void* address =
        ::mmap(nullptr, size, read_only ? PROT_READ : (PROT_READ | PROT_WRITE),
               MAP_SHARED, fd, 0);
    ::close(fd);
    if (address == MAP_FAILED) {
      throw std::runtime_error("registry mmap failed for " + path_);
    }

    header_ = static_cast<RegistryHeader*>(address);

    try {
      if (read_only) {
        validate_initialized();
      } else {
        initialize(test_hooks);
      }
    } catch (...) {
      ::munmap(header_, size);
      header_ = nullptr;
      throw;
    }
  }

  void initialize(const RegistryTestHooks* test_hooks) {
    if (header_->magic.load(std::memory_order_acquire) != kMagic) {
      // No committed registry exists, and the file lock excludes any live
      // initializer. Reset all residue, even if its owner died mid-reset.
      ::new (header_) RegistryHeader{};
      header_->magic.store(kInitializing, std::memory_order_relaxed);
      header_->version = kRegistryVersion;
      if (test_hooks != nullptr && test_hooks->initializing != nullptr) {
        test_hooks->initializing(header_);
      }
      header_->node_capacity = static_cast<std::uint32_t>(kMaxNodes);
      header_->source_capacity = static_cast<std::uint32_t>(kMaxSourcesPerNode);
      header_->node_record_bytes = sizeof(NodeRecord);
      header_->source_record_bytes = sizeof(SourceRecord);
      header_->magic.store(kMagic, std::memory_order_release);
    }
    validate_initialized();
  }

  void validate_initialized() const {
    if (header_->magic.load(std::memory_order_acquire) != kMagic) {
      throw std::runtime_error("registry " + path_ +
                               " has not been initialized by a writer");
    }
    if (header_->version != kRegistryVersion ||
        header_->node_record_bytes != sizeof(NodeRecord) ||
        header_->source_record_bytes != sizeof(SourceRecord)) {
      throw std::runtime_error(
          "registry " + path_ +
          " was created by a process with a different record layout");
    }
  }

  std::string path_;
  RegistryHeader* header_{nullptr};
};

// How long PrepareRegistry waits for the writer lock. A writer holds it for
// microseconds; one that holds it this long is stuck.
inline constexpr std::chrono::milliseconds kRegistryLockWait{5000};

// Makes sure every node the launcher starts can register: called once before
// the launcher spawns any. A node that cannot open the registry runs without a
// row (Reporter), and the launcher kills a node that has no row once its
// startup grace ends, so an unusable registry would restart the whole roster
// forever. This opens the segment read-write -- creating and initializing it
// when absent or uncommitted -- gives it and its writer lock file the modes
// every user's nodes and tools need (kRegistryMode, kLockFileMode), and opens
// that lock file as every writer must. Nodes run as the launcher's user, so
// what it can open they can. The lock is waited for only when the segment must
// be created or initialized, and at most `lock_wait`. Throws
// std::runtime_error naming the segment or file and what to do about it.
inline void PrepareRegistry(
    std::string_view path = kRegistryPath,
    std::chrono::milliseconds lock_wait = kRegistryLockWait) {
  const std::string mapped = ShmObjectName(path);
  const std::string lock_path = LockFilePath(mapped);
  const auto refuse = [&mapped](std::string_view what, const std::string& name,
                                int error, const std::string& owner) {
    std::string message = "the node registry's " + std::string{what} + " " +
                          name + " cannot be opened by uid " +
                          std::to_string(::geteuid()) + ": " +
                          std::strerror(error);
    if (!owner.empty()) message += " (" + owner + ")";
    message +=
        "; its nodes could not register and would be killed. Remove it as its "
        "owner or root";
    if (what == "segment") {
#if defined(__linux__)
      message += " (rm /dev/shm" + mapped + ")";
#else
      message += " (shm_unlink " + mapped + ", or reboot)";
#endif
    }
    return std::runtime_error(message);
  };

  // Every writer locks this file first, so one the nodes cannot open is a
  // registry they cannot write.
  const int lock_fd = OpenLockFile(lock_path, O_RDONLY);
  if (lock_fd < 0) {
    const int error = errno;
    std::string owner;
    struct stat status {};
    if (::stat(lock_path.c_str(), &status) == 0) owner = DescribeOwner(status);
    throw refuse("lock file", lock_path, error, owner);
  }
  ::close(lock_fd);

  std::optional<RegistryLock> lock;
  int fd = ::shm_open(mapped.c_str(), O_RDWR, 0);
  if (fd < 0 && errno == ENOENT) {
    lock.emplace(mapped, lock_wait);
    fd = ::shm_open(mapped.c_str(), O_RDWR | O_CREAT, kRegistryMode);
  }
  if (fd < 0) {
    const int error = errno;
    std::string owner;
    const int peek = ::shm_open(mapped.c_str(), O_RDONLY, 0);
    if (peek >= 0) {
      struct stat status {};
      if (::fstat(peek, &status) == 0) owner = DescribeOwner(status);
      ::close(peek);
    }
    throw refuse("segment", mapped, error, owner);
  }
  struct stat status {};
  if (::fstat(fd, &status) == 0) ShareRegistrySegment(fd, status);
  ::close(fd);

  // A committed segment of this layout is ready; reading it takes no lock.
  try {
    const RegistryMapping ready{path, /*read_only=*/true};
    return;
  } catch (const std::runtime_error&) {  // NOLINT(bugprone-empty-catch)
    // Too small, uncommitted or another layout: a writer settles which.
  }
  if (!lock) lock.emplace(mapped, lock_wait);
  const RegistryMapping initialized{path, *lock};
}

// One node's claim on a registry slot. Owns the slot for its lifetime and
// frees it on destruction, so a clean shutdown leaves no ghost rows.
class NodeRegistration {
 public:
  struct Identity {
    std::string name;
    std::string target;
    std::uint64_t session_id{0};
    std::uint32_t flags{0};
  };

  explicit NodeRegistration(Identity identity,
                            std::string_view path = kRegistryPath,
                            const RegistryTestHooks* test_hooks = nullptr)
      : mapping_{path, false, test_hooks}, identity_{std::move(identity)} {
    const RegistryLock lock{mapping_.path()};
    record_ = claim(mapping_.header(), identity_, test_hooks);
    if (record_ == nullptr) {
      throw std::runtime_error(
          "registry is full: " + std::to_string(kMaxNodes) +
          " nodes are already registered");
    }
    generation_ = record_->generation.load(std::memory_order_acquire);
    pid_ = record_->pid;
  }

  ~NodeRegistration() { release(); }

  NodeRegistration(const NodeRegistration&) = delete;
  NodeRegistration& operator=(const NodeRegistration&) = delete;

  // Writes the node's event sources, then publishes the count. Call once, after
  // the event loop has closed registration: the manifest is fixed from then on,
  // so it can be read from another thread without synchronization.
  void publish(const event::Manifest& manifest) {
    write_sources(manifest, [](std::size_t, std::string_view) {
      return std::uint32_t{0};
    });
  }

  // The same, plus endpoint attributes keyed by topic name, and flags the loop
  // holds per source (naming::kSourceFlagLatestOnly) indexed like the
  // manifest; `source_flags` may be shorter than it, and the rest get none.
  //
  // The attributes cannot come from the manifest, because the loop does not
  // know them: `/hw/command` looks like any other sender to it. They come from
  // the node, which is the side that knows its consumer is a RoboRIO over UDP,
  // or that the publisher it watches for is an autonomous mode nobody has
  // written yet. A topic name is the key rather than a source id because the
  // node declares them next to the topic strings it already spells out, and ids
  // depend on registration order.
  //
  // Templated on the container instead of naming
  // `const std::vector<EndpointAttribute>&` because that type lives in
  // describe.h, and this header must not reach for it: the hardware bridge and
  // Studio's reader take `:registry` on its own, and the description of what
  // would run has no business in the record of what is running. Any range whose
  // elements have `.topic` and `.flags` fits, which is exactly
  // `introspect::EndpointAttribute` -- what the Reporter passes.
  template <typename Attributes>
  void publish(const event::Manifest& manifest, const Attributes& attributes,
               std::span<const std::uint32_t> source_flags = {}) {
    write_sources(manifest, [&attributes, source_flags](std::size_t index,
                                                        std::string_view name) {
      std::uint32_t flags =
          index < source_flags.size() ? source_flags[index] : 0u;
      for (const auto& attribute : attributes) {
        if (attribute.topic == name) {
          flags |= attribute.flags;
        }
      }
      return flags;
    });
  }

  // Writes the source table of each of the node's Targets, in the order it
  // opened them, then publishes the count; every selection starts as none.
  // Call once, after publish(): a Target is opened in the node's constructor,
  // so the list is fixed by then. Any range whose elements have `.sources`, a
  // range of strings, fits -- Reporter::Selection is what the Reporter passes,
  // and naming it here would make this header reach for reporter.h.
  template <typename Selections>
  void publish_selections(const Selections& selections) {
    if (!owns_slot()) return;
    std::uint32_t total = 0;
    for (const auto& selection : selections) {
      if (total < kMaxSelectionsPerNode) {
        SelectionRecord& record = record_->selections[total];
        std::uint32_t declared = 0;
        for (const auto& source : selection.sources) {
          if (declared < kMaxSelectionSources) {
            copy_name(record.sources[declared], kMaxNameBytes, source);
          }
          ++declared;
        }
        record.declared_source_count = declared;
        record.source_count =
            declared < kMaxSelectionSources
                ? declared
                : static_cast<std::uint32_t>(kMaxSelectionSources);
        record.selected.store(-1, std::memory_order_relaxed);
      }
      ++total;
    }
    record_->declared_selection_count.store(total, std::memory_order_relaxed);
    // Release: the records above must be visible to any reader that sees
    // this count.
    record_->selection_count.store(
        total < kMaxSelectionsPerNode
            ? total
            : static_cast<std::uint32_t>(kMaxSelectionsPerNode),
        std::memory_order_release);
  }

  // Opens one refresh sample. Bracket the heartbeat, dispatch and
  // per-source counter stores so a reader sees the whole sample or retries:
  // begin_sample, the stores, then end_sample. Adjacent samples must not
  // overlap; the reporting thread is the only writer, so plain bracketing is
  // enough, with no RAII and no nesting. An event-loop node's loop thread
  // stores the dispatch count itself, unbracketed (NodeRecord).
  void begin_sample() {
    if (!owns_slot()) return;
    record_->sample_seq.fetch_add(1, std::memory_order_relaxed);
  }

  void end_sample() {
    if (!owns_slot()) return;
    // Release: every counter store above must be visible to a reader that
    // observes the closing even sequence.
    record_->sample_seq.fetch_add(1, std::memory_order_release);
  }

  void heartbeat() {
    if (!owns_slot()) return;
    record_->heartbeat_monotonic_ns.store(monotonic_now_ns(),
                                          std::memory_order_relaxed);
  }

  // Also called from an event loop's thread at every dispatch (the
  // Reporter's DispatchObserver): one relaxed store, guarded like every
  // other write against a slot that is no longer this registration's.
  void set_dispatch_count(std::uint64_t count) {
    if (!owns_slot()) return;
    record_->dispatch_count.store(count, std::memory_order_relaxed);
  }

  // Copies one source's counters into the slot. Called from the reporting
  // thread with values read out of the loop's own atomics.
  void set_source(std::uint32_t index, std::uint64_t events,
                  std::uint64_t dropped, std::uint64_t sequence,
                  std::int64_t last_monotonic_ns, std::int64_t last_latency_ns,
                  std::int64_t max_latency_ns) {
    if (!owns_slot()) return;
    if (index >= record_->source_count.load(std::memory_order_relaxed)) {
      return;
    }
    SourceRecord& source = record_->sources[index];
    source.events.store(events, std::memory_order_relaxed);
    source.dropped.store(dropped, std::memory_order_relaxed);
    source.sequence.store(sequence, std::memory_order_relaxed);
    source.last_monotonic_ns.store(last_monotonic_ns,
                                   std::memory_order_relaxed);
    source.last_latency_ns.store(last_latency_ns, std::memory_order_relaxed);
    source.max_latency_ns.store(max_latency_ns, std::memory_order_relaxed);
  }

  // Which lane Target `index` selected last: an index into its sources, or
  // -1 for none. Called from the reporting thread inside a sample.
  void set_selected(std::uint32_t index, std::int32_t selected) {
    if (!owns_slot()) return;
    if (index >= record_->selection_count.load(std::memory_order_relaxed)) {
      return;
    }
    record_->selections[index].selected.store(selected,
                                              std::memory_order_relaxed);
  }

  // Sets kFlagRecorderFailed / kFlagLogCapped for an event log that failed or
  // reached its cap; neither is ever cleared. Called inside a sample.
  void set_recording(bool failed, bool capped) {
    if (!owns_slot()) return;
    const std::uint32_t flags =
        (failed ? kFlagRecorderFailed : 0u) | (capped ? kFlagLogCapped : 0u);
    if (flags != 0) record_->flags.fetch_or(flags, std::memory_order_relaxed);
  }

  void activate() {
    if (owns_slot()) {
      record_->flags.fetch_and(~kFlagConstructing, std::memory_order_release);
    }
  }

  const std::string& name() const { return identity_.name; }
  std::size_t slot() const {
    return static_cast<std::size_t>(record_ - mapping_.header()->nodes);
  }

 private:
  bool owns_slot() const noexcept {
    return record_ != nullptr &&
           record_->generation.load(std::memory_order_relaxed) == generation_ &&
           record_->state.load(std::memory_order_relaxed) ==
               static_cast<std::uint64_t>(SlotState::ACTIVE) &&
           record_->pid == pid_;
  }

  template <typename FlagsFor>
  void write_sources(const event::Manifest& manifest, FlagsFor flags_for) {
    if (!owns_slot()) return;
    check_names(manifest);

    const std::uint32_t total = static_cast<std::uint32_t>(manifest.size());
    const std::uint32_t stored =
        total > kMaxSourcesPerNode
            ? static_cast<std::uint32_t>(kMaxSourcesPerNode)
            : total;

    for (std::uint32_t i = 0; i < stored; ++i) {
      const event::Registration& registration = manifest[i];
      SourceRecord& source = record_->sources[i];
      source.id = registration.id;
      source.kind = static_cast<std::uint16_t>(registration.kind);
      source.message_bytes = registration.message_bytes;
      source.alignment = registration.alignment;
      source.flags = flags_for(i, registration.name);
      source.period_ns = registration.period_ns;
      copy_name(source.name, kMaxNameBytes, registration.name);
      source.events.store(0, std::memory_order_relaxed);
      source.dropped.store(0, std::memory_order_relaxed);
      source.sequence.store(0, std::memory_order_relaxed);
      source.last_monotonic_ns.store(0, std::memory_order_relaxed);
      source.last_latency_ns.store(0, std::memory_order_relaxed);
      source.max_latency_ns.store(0, std::memory_order_relaxed);
    }

    record_->declared_source_count.store(total, std::memory_order_relaxed);
    // Release: the source records above must be visible to any reader that
    // sees this count.
    record_->source_count.store(stored, std::memory_order_release);
    heartbeat();
  }

  // Checks this node's own name and its topic names against the naming
  // protocol, and prints what it finds to stderr. Once, at publish time: the
  // manifest is frozen by then, so there is nothing new to find later, and the
  // reporting thread must not turn a bad name into a message every 250 ms.
  //
  // A warning, never a throw. A node that refused to start because one of its
  // topics is misspelled would be a worse failure than the misspelling -- it
  // takes a subsystem off the robot to fix a report -- and the launcher already
  // checks the same rules from names.h and can refuse before anything spawns.
  // But a misspelling nobody is ever told about is exactly how the two-spelling
  // bug survives: `/hw/req/drive` against `/hw/req/drivetrain` are both valid
  // shared-memory objects, both ends work, and the two never meet. So a node
  // started by hand, outside the launcher, still says so out loud.
  void check_names(const event::Manifest& manifest) {
    if (std::exchange(names_checked_, true)) {
      return;
    }
    const char* who = identity_.name.c_str();
    if (const auto problem = naming::CheckNodeName(identity_.name)) {
      std::fprintf(stderr, "%s: naming: %s\n", who, problem->c_str());
    }
    for (const event::Registration& registration : manifest) {
      // A timer's name is a label, not an address: nothing subscribes to it, so
      // the topic grammar does not apply.
      if (!naming::IsSender(registration.kind) &&
          !naming::IsReader(registration.kind)) {
        continue;
      }
      if (const auto problem = naming::CheckTopicName(registration.name)) {
        std::fprintf(stderr, "%s: naming: %s\n", who, problem->c_str());
      }
    }
  }

  void release() {
    if (record_ != nullptr) {
      // Only free what is still ours: the slot may have been swept and
      // reclaimed after this owner went quiet (see sweep_stale), and freeing
      // it then would yank the new owner's row out from under it.
      if (owns_slot()) {
        record_->source_count.store(0, std::memory_order_relaxed);
        record_->selection_count.store(0, std::memory_order_relaxed);
        record_->state.store(static_cast<std::uint64_t>(SlotState::FREE),
                             std::memory_order_release);
      }
      record_ = nullptr;
    }
  }

  // Frees every slot whose owner is gone: its process no longer runs
  // (process_liveness.h: no process with its pid, a zombie, or a pid reused
  // by a process that started later; never decided for an owner in another
  // pid namespace), or it stopped heartbeating long enough ago to be
  // considered gone. So crash residue from an earlier session never
  // lingers as a permanent "stopped heartbeating" row in the viewer, and
  // SIGKILLed nodes -- a crash-looping roster leaves one row per run -- never
  // fill the registry for the restarts that follow. Runs on every claim -- a
  // fresh session starts one claim per node, which sweeps the previous
  // session's ghosts -- and only touches slots whose owner is gone.
  static void sweep_stale(RegistryHeader* header) {
    const std::int64_t now = monotonic_now_ns();
    const std::int64_t timeout =
        std::chrono::duration_cast<std::chrono::nanoseconds>(kLivenessTimeout)
            .count();
    for (std::size_t i = 0; i < kMaxNodes; ++i) {
      NodeRecord& record = header->nodes[i];
      // A live claimant/sweeper holds RegistryLock until its state commit.
      // Under that lock any remaining CLAIMING row is abandoned, even if
      // death preceded its first pid or heartbeat store. Reclaim immediately.
      if (record.state.load(std::memory_order_acquire) ==
          static_cast<std::uint64_t>(SlotState::CLAIMING)) {
        record.state.store(static_cast<std::uint64_t>(SlotState::FREE),
                           std::memory_order_release);
        continue;
      }
      if (record.state.load(std::memory_order_acquire) !=
          static_cast<std::uint64_t>(SlotState::ACTIVE)) {
        continue;
      }
      // The claimant's identity is written under this lock before ACTIVE
      // and never after, so it is stable while the row is ACTIVE.
      const std::uint64_t claimant =
          (record.pid_start << kProcessPidBits) | record.pid;
      const auto& beat = record.heartbeat_monotonic_ns;
      if (now - beat.load(std::memory_order_relaxed) < timeout &&
          talos::ProcessWordAlive(claimant)) {
        continue;
      }
      // Take the slot before deciding, so two nodes starting at once cannot
      // free the same row twice; hand it back when the owner heartbeated
      // while we were deciding and still runs.
      std::uint64_t expected = static_cast<std::uint64_t>(SlotState::ACTIVE);
      if (!record.state.compare_exchange_strong(
              expected, static_cast<std::uint64_t>(SlotState::CLAIMING),
              std::memory_order_acq_rel)) {
        continue;
      }
      const bool gone = now - beat.load(std::memory_order_relaxed) >= timeout ||
                        !talos::ProcessWordAlive(claimant);
      record.state.store(gone ? static_cast<std::uint64_t>(SlotState::FREE)
                              : static_cast<std::uint64_t>(SlotState::ACTIVE),
                         std::memory_order_release);
    }
  }

  // Takes a free slot, or -- only when none is free -- one whose owner stopped
  // heartbeating long enough ago to be considered gone. Rows of gone owners
  // are swept first (see sweep_stale), so the fallback below only fires for a
  // slot that went quiet between the sweep and the claim.
  static NodeRecord* claim(RegistryHeader* header, const Identity& identity,
                           const RegistryTestHooks* test_hooks) {
    sweep_stale(header);
    NodeRecord* record = claim_where(header, SlotState::FREE, false);
    if (record == nullptr) {
      record = claim_where(header, SlotState::ACTIVE, true);
    }
    if (record == nullptr) {
      return nullptr;
    }
    if (test_hooks != nullptr && test_hooks->claiming != nullptr) {
      test_hooks->claiming(record);
    }

    const std::uint64_t process = talos::OwnProcessWord();
    record->pid = process & kProcessPidMask;
    record->pid_start = process >> kProcessPidBits;
    record->session_id = identity.session_id;
    record->start_wall_ns = wall_now_ns();
    copy_name(record->name, kMaxNameBytes, identity.name);
    copy_name(record->target, kMaxTargetBytes, identity.target);
    record->flags.store(identity.flags, std::memory_order_relaxed);
    record->source_count.store(0, std::memory_order_relaxed);
    record->declared_source_count.store(0, std::memory_order_relaxed);
    record->selection_count.store(0, std::memory_order_relaxed);
    record->declared_selection_count.store(0, std::memory_order_relaxed);
    record->dispatch_count.store(0, std::memory_order_relaxed);
    // A recycled slot may hold an odd sequence from a writer that died
    // mid-sample; a fresh claim is quiescent by definition.
    record->sample_seq.store(0, std::memory_order_relaxed);
    record->heartbeat_monotonic_ns.store(monotonic_now_ns(),
                                         std::memory_order_relaxed);

    // A reader that saw the previous occupant must observe a different
    // generation, so bump it before the slot goes live.
    record->generation.fetch_add(1, std::memory_order_acq_rel);
    record->state.store(static_cast<std::uint64_t>(SlotState::ACTIVE),
                        std::memory_order_release);
    return record;
  }

  static NodeRecord* claim_where(RegistryHeader* header, SlotState from,
                                 bool require_stale) {
    const std::int64_t now = monotonic_now_ns();
    const std::int64_t timeout =
        std::chrono::duration_cast<std::chrono::nanoseconds>(kLivenessTimeout)
            .count();

    for (std::size_t i = 0; i < kMaxNodes; ++i) {
      NodeRecord& record = header->nodes[i];
      if (require_stale) {
        const std::int64_t beat =
            record.heartbeat_monotonic_ns.load(std::memory_order_relaxed);
        if (now - beat < timeout) {
          continue;
        }
      }
      std::uint64_t expected = static_cast<std::uint64_t>(from);
      if (record.state.compare_exchange_strong(
              expected, static_cast<std::uint64_t>(SlotState::CLAIMING),
              std::memory_order_acq_rel)) {
        return &record;
      }
    }
    return nullptr;
  }

  RegistryMapping mapping_;
  Identity identity_;
  NodeRecord* record_{nullptr};
  // Generation of record_ when claimed. Guards release() against freeing a
  // slot that was swept and reclaimed after this owner went quiet.
  std::uint64_t generation_{0};
  std::uint64_t pid_{0};
  bool names_checked_{false};
};

// --- reading ---------------------------------------------------------------

struct SourceSnapshot {
  std::uint16_t id{0};
  event::SourceKind kind{event::SourceKind::TIMER};
  std::string name;
  std::uint32_t message_bytes{0};
  std::uint32_t alignment{0};
  std::uint32_t flags{0};
  std::int64_t period_ns{0};
  std::uint64_t events{0};
  std::uint64_t dropped{0};
  std::uint64_t sequence{0};
  std::int64_t last_monotonic_ns{0};
  std::int64_t last_latency_ns{0};
  std::int64_t max_latency_ns{0};

  // Named the same as naming::SourceShape's, so a graph check asks a running
  // node and a `--describe` the same question and gets the answer the same way.
  bool external() const { return (flags & naming::kSourceFlagExternal) != 0; }
  bool optional() const { return (flags & naming::kSourceFlagOptional) != 0; }
  bool observer() const { return (flags & naming::kSourceFlagObserver) != 0; }
  bool latest_only() const {
    return (flags & naming::kSourceFlagLatestOnly) != 0;
  }
};

// One Target of a node: the lanes its policy names and the one it selected
// last. This is who drives the subsystem -- "teleop", "auto", or nobody.
struct SelectionSnapshot {
  std::vector<std::string> sources;
  std::uint32_t declared_source_count{0};
  std::int32_t selected{-1};

  // The selected lane's source, or nullopt when the Target's last get() was
  // null. A source past the slot's name table (more than
  // kMaxSelectionSources in one policy) reads as "#<index>".
  std::optional<std::string> selected_source() const {
    if (selected < 0) return std::nullopt;
    const auto index = static_cast<std::size_t>(selected);
    if (index < sources.size()) return sources[index];
    return "#" + std::to_string(index);
  }
};

struct NodeSnapshot {
  std::size_t slot{0};
  std::string name;
  std::string target;
  std::uint64_t pid{0};
  std::uint64_t session_id{0};
  std::uint64_t generation{0};
  std::uint64_t dispatch_count{0};
  std::int64_t start_wall_ns{0};
  std::int64_t heartbeat_monotonic_ns{0};
  std::uint32_t flags{0};
  std::uint32_t declared_source_count{0};
  bool alive{false};
  std::vector<SourceSnapshot> sources;
  std::uint32_t declared_selection_count{0};
  std::vector<SelectionSnapshot> selections;
};

// Read-only view of the registry. Safe to poll at any rate: it takes no locks
// and writes nothing, so it cannot perturb the nodes it is observing.
class RegistryReader {
 public:
  explicit RegistryReader(std::string_view path = kRegistryPath)
      : mapping_{path, /*read_only=*/true} {}

  // Attaches only if some node has already created the registry. Returns
  // nullopt rather than throwing, because "no nodes are running" is an ordinary
  // state for a viewer to start up in.
  static std::optional<RegistryReader> open(
      std::string_view path = kRegistryPath) {
    try {
      return RegistryReader{path};
    } catch (const std::exception&) {
      return std::nullopt;
    }
  }

  std::uint32_t node_capacity() const {
    return mapping_.header()->node_capacity;
  }
  std::uint32_t source_capacity() const {
    return mapping_.header()->source_capacity;
  }

  // The rows a claim cannot take: ACTIVE rows whose owner still runs and
  // heartbeated within kLivenessTimeout, constructing or not -- what
  // NodeRegistration's sweep keeps. kMaxNodes minus their count is how many
  // nodes can register now. Owner liveness is the sweep's own test, so a
  // reader takes no lock here either.
  std::vector<NodeSnapshot> held_rows() const {
    std::vector<NodeSnapshot> out;
    const RegistryHeader* header = mapping_.header();
    const std::int64_t now = monotonic_now_ns();
    const std::int64_t timeout =
        std::chrono::duration_cast<std::chrono::nanoseconds>(kLivenessTimeout)
            .count();
    for (std::size_t i = 0; i < kMaxNodes; ++i) {
      const NodeRecord& record = header->nodes[i];
      if (record.state.load(std::memory_order_acquire) !=
          static_cast<std::uint64_t>(SlotState::ACTIVE)) {
        continue;
      }
      const std::uint64_t claimant =
          (record.pid_start << kProcessPidBits) | record.pid;
      if (now - record.heartbeat_monotonic_ns.load(std::memory_order_relaxed) >=
              timeout ||
          !talos::ProcessWordAlive(claimant)) {
        continue;
      }
      NodeSnapshot node = copy(record, i);
      node.alive = (node.flags & kFlagConstructing) == 0;
      out.push_back(std::move(node));
    }
    return out;
  }

  std::vector<NodeSnapshot> nodes() const {
    std::vector<NodeSnapshot> out;
    const RegistryHeader* header = mapping_.header();
    const std::int64_t now = monotonic_now_ns();
    const std::int64_t timeout =
        std::chrono::duration_cast<std::chrono::nanoseconds>(kLivenessTimeout)
            .count();

    for (std::size_t i = 0; i < kMaxNodes; ++i) {
      const NodeRecord& record = header->nodes[i];
      if (record.state.load(std::memory_order_acquire) !=
          static_cast<std::uint64_t>(SlotState::ACTIVE)) {
        continue;
      }

      // Copy, then check the slot was not recycled while copying. Two attempts
      // is enough: claiming happens once per process start, not per read.
      for (int attempt = 0; attempt < 2; ++attempt) {
        const std::uint64_t generation =
            record.generation.load(std::memory_order_acquire);
        auto node = copy_stable(record, i);
        if (!node ||
            record.state.load(std::memory_order_acquire) !=
                static_cast<std::uint64_t>(SlotState::ACTIVE) ||
            record.generation.load(std::memory_order_acquire) != generation) {
          continue;
        }
        node->generation = generation;
        node->alive = now - node->heartbeat_monotonic_ns < timeout &&
                      (node->flags & kFlagConstructing) == 0;
        out.push_back(std::move(*node));
        break;
      }
    }
    return out;
  }

 private:
  // Seqlock read of one slot: an odd sequence means a refresh is landing, and
  // a sequence that moved across the copy means one landed mid-copy. Either
  // way the copy is retried, bounded so a writer that died mid-sample -- or a
  // refresh that never quiesces -- degrades to one plain copy instead of a
  // wedged viewer. That fallback is exactly what a reader without the guard
  // takes every time, so it is no worse than the status quo ante.
  static std::optional<NodeSnapshot> copy_stable(const NodeRecord& record,
                                                 std::size_t slot) {
    const auto generation = record.generation.load(std::memory_order_acquire);
    for (int attempt = 0; attempt < 4; ++attempt) {
      const std::uint32_t before =
          record.sample_seq.load(std::memory_order_acquire);
      // Retain the bounded plain-copy fallback for a stalled sample writer.
      if ((before & 1u) != 0u && attempt < 3) continue;
      NodeSnapshot node = copy(record, slot);
      if (record.state.load(std::memory_order_acquire) !=
              static_cast<std::uint64_t>(SlotState::ACTIVE) ||
          record.generation.load(std::memory_order_acquire) != generation) {
        return std::nullopt;
      }
      if (attempt == 3 ||
          record.sample_seq.load(std::memory_order_acquire) == before) {
        return node;
      }
    }
    return std::nullopt;
  }

  static NodeSnapshot copy(const NodeRecord& record, std::size_t slot) {
    NodeSnapshot node;
    node.slot = slot;
    node.name = read_name(record.name, kMaxNameBytes);
    node.target = read_name(record.target, kMaxTargetBytes);
    node.pid = record.pid;
    node.session_id = record.session_id;
    node.start_wall_ns = record.start_wall_ns;
    node.heartbeat_monotonic_ns =
        record.heartbeat_monotonic_ns.load(std::memory_order_relaxed);
    node.flags = record.flags.load(std::memory_order_relaxed);
    node.dispatch_count = record.dispatch_count.load(std::memory_order_relaxed);
    node.declared_source_count =
        record.declared_source_count.load(std::memory_order_relaxed);

    // Acquire pairs with publish(): a visible count means the records it
    // describes are visible too.
    const std::uint32_t count =
        record.source_count.load(std::memory_order_acquire);
    const std::uint32_t bounded =
        count > kMaxSourcesPerNode
            ? static_cast<std::uint32_t>(kMaxSourcesPerNode)
            : count;

    node.sources.reserve(bounded);
    for (std::uint32_t s = 0; s < bounded; ++s) {
      const SourceRecord& source = record.sources[s];
      SourceSnapshot out;
      out.id = source.id;
      out.kind = static_cast<event::SourceKind>(source.kind);
      out.name = read_name(source.name, kMaxNameBytes);
      out.message_bytes = source.message_bytes;
      out.alignment = source.alignment;
      out.flags = source.flags;
      out.period_ns = source.period_ns;
      out.events = source.events.load(std::memory_order_relaxed);
      out.dropped = source.dropped.load(std::memory_order_relaxed);
      out.sequence = source.sequence.load(std::memory_order_relaxed);
      out.last_monotonic_ns =
          source.last_monotonic_ns.load(std::memory_order_relaxed);
      out.last_latency_ns =
          source.last_latency_ns.load(std::memory_order_relaxed);
      out.max_latency_ns =
          source.max_latency_ns.load(std::memory_order_relaxed);
      node.sources.push_back(std::move(out));
    }

    // Acquire pairs with publish_selections(), as source_count does above.
    const std::uint32_t selections =
        record.selection_count.load(std::memory_order_acquire);
    const std::uint32_t held =
        selections > kMaxSelectionsPerNode
            ? static_cast<std::uint32_t>(kMaxSelectionsPerNode)
            : selections;
    node.declared_selection_count =
        record.declared_selection_count.load(std::memory_order_relaxed);
    node.selections.reserve(held);
    for (std::uint32_t t = 0; t < held; ++t) {
      const SelectionRecord& selection = record.selections[t];
      SelectionSnapshot out;
      const std::uint32_t names =
          selection.source_count > kMaxSelectionSources
              ? static_cast<std::uint32_t>(kMaxSelectionSources)
              : selection.source_count;
      out.sources.reserve(names);
      for (std::uint32_t s = 0; s < names; ++s) {
        out.sources.push_back(read_name(selection.sources[s], kMaxNameBytes));
      }
      out.declared_source_count = selection.declared_source_count;
      out.selected = selection.selected.load(std::memory_order_relaxed);
      node.selections.push_back(std::move(out));
    }
    return node;
  }

  RegistryMapping mapping_;
};

}  // namespace talos::introspect

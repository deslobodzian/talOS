#include "talOS/introspection/registry.h"

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
#include <exception>
#include <new>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/introspection/lock_file.h"
#include "talOS/introspection/names.h"
#include "talOS/memory/shared_memory_ptr.h"
#include "talOS/process/realtime_outcome.h"
#include "talOS/rtms/process_liveness.h"

namespace talos::introspect {

namespace {

// kLivenessTimeout in the registry's heartbeat unit (monotonic_now_ns).
constexpr std::int64_t kLivenessTimeoutNs =
    std::chrono::nanoseconds{kLivenessTimeout}.count();

}  // namespace

RegistryLock::RegistryLock(const std::string& mapped_path)
    : RegistryLock(mapped_path, std::nullopt) {}

RegistryLock::RegistryLock(const std::string& mapped_path,
                           std::optional<std::chrono::milliseconds> wait,
                           const RegistryTestHooks* test_hooks) {
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
    if (test_hooks != nullptr && test_hooks->lock_file_opened != nullptr) {
      test_hooks->lock_file_opened();
      test_hooks = nullptr;
    }
    while (::flock(fd_, operation) < 0) {
      const int error = errno;
      if (error == EINTR) continue;
      if (error == EWOULDBLOCK && std::chrono::steady_clock::now() < deadline) {
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
      throw std::runtime_error("registry lock failed for " + file_path + ": " +
                               std::strerror(error));
    }
    if (LockFileIsCurrent(fd_, file_path)) return;
    ::close(fd_);
  }
}

RegistryLock::~RegistryLock() {
  if (fd_ >= 0) ::close(fd_);
}

void ShareRegistrySegment(int fd, const struct stat& status) {
  if (status.st_uid == ::geteuid() &&
      (status.st_mode & 07777) != kRegistryMode) {
    static_cast<void>(::fchmod(fd, kRegistryMode));
  }
}

RegistryMapping::RegistryMapping(std::string_view path, bool read_only,
                                 const RegistryTestHooks* test_hooks)
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

RegistryMapping::RegistryMapping(std::string_view path,
                                 const RegistryLock& held,
                                 const RegistryTestHooks* test_hooks)
    : path_{ShmObjectName(path)} {
  static_cast<void>(held);
  map(false, test_hooks);
}

RegistryMapping::~RegistryMapping() {
  if (header_ != nullptr) {
    ::munmap(header_, sizeof(RegistryHeader));
  }
}

RegistryMapping::RegistryMapping(RegistryMapping&& other) noexcept
    : path_{std::move(other.path_)},
      header_{std::exchange(other.header_, nullptr)} {}

RegistryMapping& RegistryMapping::operator=(RegistryMapping&& other) noexcept {
  if (this != &other) {
    if (header_ != nullptr) {
      ::munmap(header_, sizeof(RegistryHeader));
    }
    path_ = std::move(other.path_);
    header_ = std::exchange(other.header_, nullptr);
  }
  return *this;
}

void RegistryMapping::unlink(std::string_view path) {
  ::shm_unlink(ShmObjectName(path).c_str());
}

void RegistryMapping::map(bool read_only, const RegistryTestHooks* test_hooks) {
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

void RegistryMapping::initialize(const RegistryTestHooks* test_hooks) {
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

void RegistryMapping::validate_initialized() const {
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

void PrepareRegistry(std::string_view path,
                     std::chrono::milliseconds lock_wait) {
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
  const RegistryLock& held = lock ? *lock : lock.emplace(mapped, lock_wait);
  const RegistryMapping initialized{path, held};
}

bool RemoveRegistryUnlessHeld(std::uint64_t session_id, std::string_view path) {
  const std::string mapped = ShmObjectName(path);
  try {
    // Not waited for: a holder now is no member of the session (every one is
    // reaped), and the registry stays for it.
    const RegistryLock lock{mapped, std::chrono::milliseconds{0}};
    {
      const RegistryMapping registry{path, /*read_only=*/true};
      // A row of another session keeps the registry while its claimant runs;
      // one whose process is gone (a SIGKILLed node of an earlier session,
      // not yet swept by a claim) is no one's.
      for (const auto& row : registry.header()->nodes) {
        if (row.state.load(std::memory_order_acquire) !=
                static_cast<std::uint64_t>(SlotState::FREE) &&
            row.session_id != session_id &&
            talos::ProcessWordAlive((row.pid_start << kProcessPidBits) |
                                    row.pid)) {
          return false;
        }
      }
    }
    RegistryMapping::unlink(path);
    // Under the lock: a writer waiting on this file finds it no longer
    // current once it locks it, and takes the one the path names anew
    // (RegistryLock).
    ::unlink(LockFilePath(mapped).c_str());
    return true;
  } catch (const std::runtime_error&) {
    return false;
  }
}

NodeRegistration::NodeRegistration(Identity identity, std::string_view path,
                                   const RegistryTestHooks* test_hooks)
    // The temporary lock lives until the delegated constructor returns.
    : NodeRegistration(std::move(identity), path, test_hooks,
                       RegistryLock{ShmObjectName(path)}) {}

NodeRegistration::NodeRegistration(Identity identity, std::string_view path,
                                   const RegistryTestHooks* test_hooks,
                                   const RegistryLock& held)
    : mapping_{path, held, test_hooks}, identity_{std::move(identity)} {
  if (test_hooks != nullptr && test_hooks->mapped != nullptr) {
    test_hooks->mapped(mapping_.header());
  }
  record_ = claim(mapping_.header(), identity_, test_hooks);
  if (record_ == nullptr) {
    throw std::runtime_error("registry is full: " + std::to_string(kMaxNodes) +
                             " nodes are already registered");
  }
  generation_ = record_->generation.load(std::memory_order_acquire);
  pid_ = record_->pid;
}

NodeRegistration::~NodeRegistration() { release(); }

void NodeRegistration::publish(const event::Manifest& manifest) {
  write_sources(manifest,
                [](std::size_t, std::string_view) { return std::uint32_t{0}; });
}

void NodeRegistration::set_recording(bool failed, bool capped, bool gap) {
  if (!owns_slot()) return;
  const std::uint32_t flags = (failed ? kFlagRecorderFailed : 0u) |
                              (capped ? kFlagLogCapped : 0u) |
                              (gap ? kFlagLogGap : 0u);
  if (flags != 0) record_->flags.fetch_or(flags, std::memory_order_relaxed);
}

void NodeRegistration::set_outputs_held(bool held) {
  if (!owns_slot()) return;
  const bool was =
      (record_->flags.load(std::memory_order_relaxed) & kFlagOutputsHeld) != 0;
  if (was == held) return;
  if (held) {
    record_->flags.fetch_or(kFlagOutputsHeld, std::memory_order_relaxed);
  } else {
    record_->flags.fetch_and(~kFlagOutputsHeld, std::memory_order_relaxed);
  }
}

void NodeRegistration::set_realtime(const process::RealtimeOutcome& outcome) {
  if (!owns_slot()) return;
  record_->flags.fetch_or(RealtimeFlags(outcome), std::memory_order_release);
}

void NodeRegistration::note_realtime_refused(std::uint8_t steps) {
  if (!owns_slot()) return;
  const std::uint32_t masked =
      static_cast<std::uint32_t>(steps & process::kAllSteps);
  if (masked == 0) return;
  const std::uint32_t current = record_->flags.load(std::memory_order_acquire);
  const std::uint32_t rt_bits =
      (current & kFlagRealtimeMask) >> kFlagRealtimeShift;
  if ((rt_bits & 3u) == 0 || (rt_bits & 4u) == 0) return;
  const std::uint32_t add = masked << (kFlagRealtimeShift + 3);
  if ((current & add) == add) return;
  record_->flags.fetch_or(add, std::memory_order_release);
}

void NodeRegistration::activate() {
  if (owns_slot()) {
    record_->flags.fetch_and(~kFlagConstructing, std::memory_order_release);
  }
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
void NodeRegistration::check_names(const event::Manifest& manifest) {
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

void NodeRegistration::release() {
  if (record_ != nullptr) {
    // Only free what is still ours: the slot may have been swept and
    // reclaimed after this owner went quiet (see sweep_stale), and freeing
    // it then would yank the new owner's row out from under it. The row
    // keeps its last sample, sources included, until a claim takes it
    // (claim says which first): the launcher reads a reaped run's final
    // delivery loss and event log flags there (DeliveryLossOf; TODO.md
    // RT-7).
    if (owns_slot()) {
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
void NodeRegistration::sweep_stale(RegistryHeader* header) {
  const std::int64_t now = monotonic_now_ns();
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
    if (now - beat.load(std::memory_order_relaxed) < kLivenessTimeoutNs &&
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
    const bool gone =
        now - beat.load(std::memory_order_relaxed) >= kLivenessTimeoutNs ||
        !talos::ProcessWordAlive(claimant);
    record.state.store(gone ? static_cast<std::uint64_t>(SlotState::FREE)
                            : static_cast<std::uint64_t>(SlotState::ACTIVE),
                       std::memory_order_release);
  }
}

// Takes a free slot, or -- only when none is free -- one whose owner stopped
// heartbeating long enough ago to be considered gone. Rows of gone owners
// are swept first (see sweep_stale), so the fallback below only fires for a
// slot that went quiet between the sweep and the claim. A released row
// holds its run's last sample until the launcher reaps the run and reads
// it (its delivery loss and event log flags), so of the free slots a claim
// takes the lowest whose heartbeat is at least kLivenessTimeout old (never
// used, swept, or released long ago); failing that, the free slot whose
// heartbeat is oldest, the one released longest ago. A row released moments
// ago is therefore overwritten before its reap only when every other slot
// is held or was released after it (a restart storm). Two rules, not one
// oldest-first scan, so a swept slot is reclaimed in place.
NodeRecord* NodeRegistration::claim(RegistryHeader* header,
                                    const Identity& identity,
                                    const RegistryTestHooks* test_hooks) {
  sweep_stale(header);
  NodeRecord* record = claim_where(header, SlotState::FREE, true);
  if (record == nullptr) {
    record = claim_oldest_free(header);
  }
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
  record->wake_failures.store(0, std::memory_order_relaxed);
  record->stalled_arms.store(0, std::memory_order_relaxed);
  record->wake_fallbacks.store(0, std::memory_order_relaxed);
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

// The FREE slot with the oldest heartbeat, taken (CLAIMING); null when
// none is free. Claims hold RegistryLock, so the slot found stays FREE;
// a lost exchange (a sweeper's) rescans.
NodeRecord* NodeRegistration::claim_oldest_free(RegistryHeader* header) {
  for (std::size_t attempt = 0; attempt < kMaxNodes; ++attempt) {
    NodeRecord* oldest = nullptr;
    std::int64_t oldest_beat = 0;
    for (std::size_t i = 0; i < kMaxNodes; ++i) {
      NodeRecord& record = header->nodes[i];
      if (record.state.load(std::memory_order_acquire) !=
          static_cast<std::uint64_t>(SlotState::FREE)) {
        continue;
      }
      const std::int64_t beat =
          record.heartbeat_monotonic_ns.load(std::memory_order_relaxed);
      if (oldest == nullptr || beat < oldest_beat) {
        oldest = &record;
        oldest_beat = beat;
      }
    }
    if (oldest == nullptr) return nullptr;
    std::uint64_t expected = static_cast<std::uint64_t>(SlotState::FREE);
    if (oldest->state.compare_exchange_strong(
            expected, static_cast<std::uint64_t>(SlotState::CLAIMING),
            std::memory_order_acq_rel)) {
      return oldest;
    }
  }
  return nullptr;
}

NodeRecord* NodeRegistration::claim_where(RegistryHeader* header,
                                          SlotState from, bool require_stale) {
  const std::int64_t now = monotonic_now_ns();

  for (std::size_t i = 0; i < kMaxNodes; ++i) {
    NodeRecord& record = header->nodes[i];
    if (require_stale) {
      const std::int64_t beat =
          record.heartbeat_monotonic_ns.load(std::memory_order_relaxed);
      if (now - beat < kLivenessTimeoutNs) {
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

bool IsRequestRead(event::SourceKind kind, std::string_view name) {
  return (kind == event::SourceKind::FETCHER ||
          kind == event::SourceKind::WATCHER) &&
         naming::IsHardwareRequest(name);
}

DeliveryLoss DeliveryLossOf(const NodeRecord& record) {
  DeliveryLoss loss{
      .wake_failures = record.wake_failures.load(std::memory_order_relaxed),
      .stalled_arms = record.stalled_arms.load(std::memory_order_relaxed),
      .wake_fallbacks = record.wake_fallbacks.load(std::memory_order_relaxed)};
  const std::uint32_t count =
      record.source_count.load(std::memory_order_acquire);
  for (std::uint32_t s = 0; s < count && s < kMaxSourcesPerNode; ++s) {
    const SourceRecord& source = record.sources[s];
    if (IsRequestRead(
            static_cast<event::SourceKind>(source.kind),
            std::string_view{source.name,
                             ::strnlen(source.name, kMaxNameBytes)})) {
      loss.requests_skipped += source.dropped.load(std::memory_order_relaxed);
    }
  }
  return loss;
}

std::optional<process::RealtimeOutcome> NodeSnapshot::realtime() const {
  return RealtimeOf(flags);
}

DeliveryLoss NodeSnapshot::delivery_loss() const {
  DeliveryLoss loss{.wake_failures = wake_failures,
                    .stalled_arms = stalled_arms,
                    .wake_fallbacks = wake_fallbacks};
  for (const SourceSnapshot& source : sources) {
    if (IsRequestRead(source.kind, source.name)) {
      loss.requests_skipped += source.dropped;
    }
  }
  return loss;
}

RegistryReader::RegistryReader(std::string_view path)
    : mapping_{path, /*read_only=*/true} {}

std::optional<RegistryReader> RegistryReader::open(std::string_view path) {
  try {
    return RegistryReader{path};
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::vector<NodeSnapshot> RegistryReader::held_rows() const {
  std::vector<NodeSnapshot> out;
  const RegistryHeader* header = mapping_.header();
  const std::int64_t now = monotonic_now_ns();
  for (std::size_t i = 0; i < kMaxNodes; ++i) {
    auto row = read_stable_row(header->nodes[i], i);
    if (!row) continue;
    if (now - row->node.heartbeat_monotonic_ns >= kLivenessTimeoutNs ||
        !talos::ProcessWordAlive(row->claimant)) {
      continue;
    }
    row->node.alive = (row->node.flags & kFlagConstructing) == 0;
    out.push_back(std::move(row->node));
  }
  return out;
}

std::vector<NodeSnapshot> RegistryReader::nodes() const {
  std::vector<NodeSnapshot> out;
  const RegistryHeader* header = mapping_.header();
  const std::int64_t now = monotonic_now_ns();

  for (std::size_t i = 0; i < kMaxNodes; ++i) {
    auto row = read_stable_row(header->nodes[i], i);
    if (!row) continue;
    // A crashed owner's heartbeat stays fresh for kLivenessTimeout, so
    // its process is checked too, as the claim sweep checks it.
    row->node.alive =
        now - row->node.heartbeat_monotonic_ns < kLivenessTimeoutNs &&
        (row->node.flags & kFlagConstructing) == 0 &&
        talos::ProcessWordAlive(row->claimant);
    out.push_back(std::move(row->node));
  }
  return out;
}

// Generation-verified, seqlock-copied snapshot of one ACTIVE slot together
// with its claimant process word. Two attempts is enough: claiming happens
// once per process start, not per read.
std::optional<RegistryReader::StableRow> RegistryReader::read_stable_row(
    const NodeRecord& record, std::size_t slot) {
  for (int attempt = 0; attempt < 2; ++attempt) {
    const std::uint64_t generation =
        record.generation.load(std::memory_order_acquire);
    if (record.state.load(std::memory_order_acquire) !=
        static_cast<std::uint64_t>(SlotState::ACTIVE)) {
      return std::nullopt;
    }
    auto node = copy_stable(record, slot);
    const std::uint64_t claimant =
        (record.pid_start << kProcessPidBits) | record.pid;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (!node ||
        record.state.load(std::memory_order_relaxed) !=
            static_cast<std::uint64_t>(SlotState::ACTIVE) ||
        record.generation.load(std::memory_order_relaxed) != generation) {
      continue;
    }
    node->generation = generation;
    return StableRow{std::move(*node), claimant};
  }
  return std::nullopt;
}

// Seqlock read of one slot: an odd sequence means a refresh is landing, and
// a sequence that moved across the copy means one landed mid-copy. Either
// way the copy is retried, bounded so a writer that died mid-sample -- or a
// refresh that never quiesces -- degrades to one plain copy instead of a
// wedged viewer. That fallback is exactly what a reader without the guard
// takes every time, so it is no worse than the status quo ante.
std::optional<NodeSnapshot> RegistryReader::copy_stable(
    const NodeRecord& record, std::size_t slot) {
  const auto generation = record.generation.load(std::memory_order_acquire);
  if (record.state.load(std::memory_order_acquire) !=
      static_cast<std::uint64_t>(SlotState::ACTIVE)) {
    return std::nullopt;
  }
  for (int attempt = 0; attempt < 4; ++attempt) {
    const std::uint32_t before =
        record.sample_seq.load(std::memory_order_acquire);
    // Retain the bounded plain-copy fallback for a stalled sample writer.
    if ((before & 1u) != 0u && attempt < 3) continue;
    NodeSnapshot node = copy(record, slot);
    std::atomic_thread_fence(std::memory_order_acquire);
    if (record.state.load(std::memory_order_relaxed) !=
            static_cast<std::uint64_t>(SlotState::ACTIVE) ||
        record.generation.load(std::memory_order_relaxed) != generation) {
      return std::nullopt;
    }
    if (attempt == 3 ||
        record.sample_seq.load(std::memory_order_relaxed) == before) {
      return node;
    }
  }
  return std::nullopt;
}

NodeSnapshot RegistryReader::copy(const NodeRecord& record, std::size_t slot) {
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
  node.wake_failures = record.wake_failures.load(std::memory_order_relaxed);
  node.stalled_arms = record.stalled_arms.load(std::memory_order_relaxed);
  node.wake_fallbacks = record.wake_fallbacks.load(std::memory_order_relaxed);

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
    out.max_latency_ns = source.max_latency_ns.load(std::memory_order_relaxed);
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

}  // namespace talos::introspect

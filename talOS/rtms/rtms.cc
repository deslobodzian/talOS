#include "rtms.h"

#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include "talOS/memory/shared_memory_ptr.h"
#include "talOS/rtms/process_liveness.h"

namespace {

// How an operator clears a segment that a killed standalone process left in
// an older format: no attach replaces it.
std::string Remedy(const std::string& path) {
  return " (stop the processes using it, then shm_unlink " +
         ShmObjectName(path) + ", start a launcher session, or reboot)";
}

std::runtime_error VersionMismatchError(const std::string& path,
                                        std::uint64_t version) {
  return std::runtime_error("RTMS shared-memory version mismatch on " + path +
                            ": segment version " + std::to_string(version) +
                            ", expected " + std::to_string(kRtmsVersion) +
                            Remedy(path));
}

// Minimum prefix needed to identify a talOS RTMS segment and its format
// version, even when an older format had a smaller RTMSHeader.
struct HeaderPrefix {
  std::atomic<std::uint64_t> magic{0};
  std::uint64_t version{0};
};
static_assert(offsetof(RTMSHeader, magic) == offsetof(HeaderPrefix, magic));
static_assert(offsetof(RTMSHeader, version) == offsetof(HeaderPrefix, version));
static_assert(sizeof(HeaderPrefix) ==
              offsetof(RTMSHeader, version) + sizeof(RTMSHeader::version));

// Checks an existing segment's magic/version prefix before SharedMemoryPtr
// refuses a smaller older-version segment by size. Does not wait on a
// segment still being created: if it is absent, too small for the prefix, or
// has not published magic yet, open() proceeds to SharedMemoryPtr.
void ThrowIfVersionMismatch(const std::string& path) {
  const int fd = ::shm_open(ShmObjectName(path).c_str(), O_RDONLY, 0666);
  if (fd == -1) return;
  struct stat info {};
  std::optional<std::uint64_t> version;
  if (::fstat(fd, &info) == 0 && info.st_size >= 0 &&
      static_cast<std::uint64_t>(info.st_size) >= sizeof(HeaderPrefix)) {
    void* mapped =
        ::mmap(nullptr, sizeof(HeaderPrefix), PROT_READ, MAP_SHARED, fd, 0);
    if (mapped != MAP_FAILED) {
      const auto* prefix = static_cast<const HeaderPrefix*>(mapped);
      if (prefix->magic.load(std::memory_order_acquire) == kRtmsMagic) {
        version = prefix->version;
      }
      ::munmap(mapped, sizeof(HeaderPrefix));
    }
  }
  ::close(fd);
  if (version.has_value() && *version != kRtmsVersion) {
    throw VersionMismatchError(path, *version);
  }
}

// Bounded like SharedMemoryPtr's size wait: a creator initializes a header
// within microseconds, so 50 ms separates "still being created" from "never
// will be" without masking either.
constexpr int kInitAttempts = 200;
constexpr useconds_t kInitRetryUs = 250;
// How many times in all, kInitRetryUs apart (at least 1 s), one open() waits
// for live claimants of closing segments to unlink them before giving up
// (RTMSQueue::claimant_wait_attempts_, which tests lower).
// Not a death test -- among processes of one build a live claimant is never
// superseded however long it is descheduled -- only a bound on the wait for
// one that never proceeds (stopped).
constexpr int kClaimantWaitAttempts = 4000;
// How many times a session owner claims and unlinks a standalone segment, or
// loses the race to claim one, before it stops replacing the name and lets
// its create report the name as taken. A pass that waits for a live claimant
// does not count. Without a sleep between passes: only a name recreated as
// fast as it is replaced reaches it.
constexpr int kSessionReplacePasses = 200;

std::string ValidatePath(std::string_view path) {
  std::string_view name = path;
  if (name.starts_with('/')) {
    name.remove_prefix(1);
  }
  if (name.size() > 30) {
    throw std::invalid_argument(
        "Topic name '" + std::string(path) +
        "' exceeds limit of 30 characters after leading slash");
  }
  // Keep the logical topic for diagnostics; every OS boundary applies the
  // shared naming policy (including the process/test namespace).
  return std::string{path};
}

std::uint64_t OwnPid() { return static_cast<std::uint64_t>(::getpid()); }

// This process as the segment records it (process_liveness.h).
std::uint64_t OwnProcess() { return talos::OwnProcessWord(); }

bool ProcessAlive(std::uint64_t word) { return talos::ProcessWordAlive(word); }

std::uint64_t ReaderWord(ReaderState state) {
  return (OwnProcess() << 2) | static_cast<std::uint64_t>(state);
}

void ClaimWriter(RTMSHeader& header, const std::string& path) {
  std::uint64_t owner = header.writer.owner_pid.load(std::memory_order_acquire);
  for (;;) {
    if (owner != 0 && ProcessAlive(owner)) {
      throw std::runtime_error(
          "RTMS topic " + path + " already has writer pid " +
          std::to_string(owner & kProcessPidMask) + "; refusing writer pid " +
          std::to_string(OwnPid()));
    }
    if (header.writer.owner_pid.compare_exchange_weak(
            owner, OwnProcess(), std::memory_order_acq_rel,
            std::memory_order_acquire)) {
      return;
    }
  }
}

std::string SchemaOf(const RTMSHeader& header) {
  const std::size_t length =
      ::strnlen(header.schema_name, sizeof(header.schema_name));
  return talos::schema::Describe(std::string_view{header.schema_name, length},
                                 header.schema_hash);
}

bool WaitInitialized(const RTMSHeader& header) {
  for (int attempt = 0; attempt < kInitAttempts; ++attempt) {
    if (header.magic.load(std::memory_order_acquire) == kRtmsMagic) return true;
    ::usleep(kInitRetryUs);
  }
  return false;
}

// A mapping of the header of an existing segment (read-only unless opened
// with Access::kWrite), or nothing when the name is absent or too small to
// hold a header within the wait, or when it could not be checked (failed()).
class HeaderView {
 public:
  enum class Access { kRead, kWrite };

  explicit HeaderView(const std::string& path, Access access = Access::kRead) {
    const bool write = access == Access::kWrite;
    const int fd = ::shm_open(ShmObjectName(path).c_str(),
                              write ? O_RDWR : O_RDONLY, 0666);
    if (fd == -1) {
      failed_ = errno != ENOENT;
      return;
    }
    present_ = true;
    struct stat info {};
    bool sized = false;
    for (int attempt = 0; attempt < kInitAttempts; ++attempt) {
      if (::fstat(fd, &info) < 0) {
        failed_ = true;
        break;
      }
      if (info.st_size >= 0 &&
          static_cast<std::uint64_t>(info.st_size) >= sizeof(RTMSHeader)) {
        sized = true;
        break;
      }
      ::usleep(kInitRetryUs);
    }
    if (sized) {
      void* mapped =
          ::mmap(nullptr, sizeof(RTMSHeader),
                 write ? PROT_READ | PROT_WRITE : PROT_READ, MAP_SHARED, fd, 0);
      if (mapped != MAP_FAILED) {
        header_ = static_cast<RTMSHeader*>(mapped);
        writable_ = write;
      } else {
        failed_ = true;
      }
    }
    ::close(fd);
  }
  ~HeaderView() {
    if (header_ != nullptr) {
      ::munmap(header_, sizeof(RTMSHeader));
    }
  }
  HeaderView(const HeaderView&) = delete;
  HeaderView& operator=(const HeaderView&) = delete;

  // Null when the segment is too small to be an RTMS segment, absent, or
  // could not be checked.
  const RTMSHeader* header() const { return header_; }
  // header(), writable; null unless opened with Access::kWrite.
  RTMSHeader* writable_header() const { return writable_ ? header_ : nullptr; }
  // Whether the name held a segment when it was opened.
  bool present() const { return present_; }
  // Whether the name could not be checked: shm_open failed other than for
  // an absent name, or fstat or mmap failed.
  bool failed() const { return failed_; }

 private:
  RTMSHeader* header_{nullptr};
  bool writable_{false};
  bool present_{false};
  bool failed_{false};
};

// Why a session owner cannot replace `path`: the claimant `claimant` (a
// process word) counts as still unlinking the segment `name` holds. A
// claimant this process cannot check counts as running whether or not it
// does (process_liveness.h), and its pid may name another process here, so
// the remedy names its pid only when ProcessAlive() looked that pid up.
std::runtime_error SessionClaimantError(const std::string& path,
                                        const std::string& name,
                                        std::uint64_t claimant) {
  const std::string pid = std::to_string(claimant & kProcessPidMask);
  if (talos::ProcessWordCheckable(claimant)) {
    return std::runtime_error("RTMS: a session cannot replace " + path +
                              ": its claimant, pid " + pid +
                              ", is still unlinking it (stop pid " + pid +
                              " or wait for it to exit, then start the "
                              "session again)");
  }
  return std::runtime_error(
      "RTMS: a session cannot replace " + path + ": its claimant, pid " + pid +
      ", which this process cannot check (another pid namespace, or /proc is "
      "not this one's), counts as still unlinking it (once that process is "
      "gone, shm_unlink " +
      name + ", then start the session again)");
}

// Unlinks `path` only while it still names the segment `segment_id`: a
// session may have replaced it since this process mapped it, and that newer
// segment is not ours to remove. Returns whether the name no longer holds
// `segment_id`: true when the name is absent, holds another segment or one
// still being created, or the unlink succeeded; false when the name could
// not be checked or shm_unlink failed.
//
// Run by a closing segment's claimant (kLifetimeClaimantMask) and by a
// session owner's close(). Among processes of one build only the claimant
// unlinks a closing segment, and a session owner's open() replaces a
// standalone segment only as its claimant (RTMSQueue::replace_for_session),
// so none of them can replace the name between the check and the unlink; an
// shm_unlink from outside talOS in that interval is not excluded.
// `on_verified` is RTMSQueue::on_unlink_verified_.
bool UnlinkIfSame(const std::string& path, std::uint64_t segment_id,
                  void (*on_verified)(std::uint64_t)) {
  {
    const HeaderView view{path};
    if (view.failed()) return false;
    const bool same =
        view.header() != nullptr &&
        view.header()->magic.load(std::memory_order_acquire) == kRtmsMagic &&
        view.header()->segment_id == segment_id;
    if (!same) return true;
  }
  if (on_verified != nullptr) on_verified(segment_id);
  return ::shm_unlink(ShmObjectName(path).c_str()) == 0 || errno == ENOENT;
}

// Run by a claimant whose UnlinkIfSame() returned false, while it still maps
// the segment: gives up this process's claim, leaving the segment closing
// with no claimant, so a later opener claims and unlinks it rather than wait
// for this process to exit.
void GiveUpClaim(RTMSHeader& header) noexcept {
  std::uint64_t claimed = kLifetimeClosing | OwnProcess();
  header.lifetime.compare_exchange_strong(claimed, kLifetimeClosing,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire);
}

std::uint64_t NewSegmentId() {
  std::random_device random;
  return (static_cast<std::uint64_t>(random()) << 32) ^ random() ^
         (OwnPid() << 16);
}

bool HeldByLiveProcess(const RTMSHeader& header) {
  if (ProcessAlive(header.writer.owner_pid.load(std::memory_order_acquire))) {
    return true;
  }
  for (const Reader& reader : header.readers) {
    const std::uint64_t word = reader.state.load(std::memory_order_acquire);
    if (word != 0 && ProcessAlive(word >> 2)) return true;
  }
  return false;
}

// Releases one counted attachment to a standalone segment using the CAS
// close protocol. Returns true when this caller dropped the count to zero,
// marking the segment closing with this process as its claimant in the same
// step, so it must unlink the segment (and, if that fails, give up the claim
// while it still maps it). Called once per counted attachment, by the process
// that took the count (owner_process_). A closing word here was claimed from
// the count by a session owner replacing the name
// (RTMSQueue::replace_for_session), which unlinks the segment: the release
// leaves it as it is and returns false.
bool ReleaseCountedLifetime(RTMSHeader& header) noexcept {
  std::uint64_t word = header.lifetime.load(std::memory_order_acquire);
  for (;;) {
    if ((word & kLifetimeClosing) != 0) return false;
    const std::uint64_t count = word & kLifetimeCountMask;
    const std::uint64_t next =
        count <= 1 ? (kLifetimeClosing | OwnProcess()) : word - 1;
    if (header.lifetime.compare_exchange_weak(
            word, next, std::memory_order_acq_rel, std::memory_order_acquire)) {
      return count <= 1;
    }
  }
}

}  // namespace

void (*RTMSQueue::on_reclaim_wait_)(std::uint64_t, int) = nullptr;
void (*RTMSQueue::on_unlink_verified_)(std::uint64_t) = nullptr;
void (*RTMSQueue::on_session_claimed_)(std::uint64_t) = nullptr;
int RTMSQueue::claimant_wait_attempts_ = kClaimantWaitAttempts;

RTMSQueue::RTMSQueue(std::string_view path, std::size_t message_size,
                     std::size_t message_alignment,
                     const talos::schema::Identity& schema, std::size_t slots,
                     RTMSOptions options)
    : path_{ValidatePath(path)},
      slots_{slots},
      message_size_{message_size},
      message_alignment_{message_alignment},
      options_{options} {
  if (message_size == 0 || !is_pow_2(message_alignment)) {
    throw std::invalid_argument("message size and alignment must be valid");
  }
  if (slots < 2 || slots > MAX_SLOTS || !is_pow_2(slots)) {
    throw std::invalid_argument(
        "slots must be a power of two in [2, MAX_SLOTS]");
  }

  if (schema.name.empty() ||
      schema.name.size() > talos::schema::kMaxSchemaName) {
    throw std::invalid_argument(
        "RTMS topic " + std::string{path} + " needs a schema name of 1 to " +
        std::to_string(talos::schema::kMaxSchemaName) + " characters");
  }

  constexpr std::size_t kMaxSize = std::numeric_limits<std::size_t>::max();
  constexpr auto kMaxOff =
      static_cast<std::size_t>(std::numeric_limits<off_t>::max());
  const std::size_t slot_align = rtms_slot_alignment(message_alignment);
  if (sizeof(RTMSHeader) > kMaxSize - (slot_align - 1) ||
      message_size > kMaxSize - (alignof(std::int64_t) - 1)) {
    throw std::overflow_error("RTMS segment size exceeds address space");
  }
  data_offset_ = align_up(sizeof(RTMSHeader), slot_align);
  publish_time_offset_ = rtms_publish_time_offset(message_size);
  if (publish_time_offset_ > kMaxSize - sizeof(std::int64_t)) {
    throw std::overflow_error("RTMS segment size exceeds address space");
  }
  const std::size_t unaligned_stride =
      publish_time_offset_ + sizeof(std::int64_t);
  if (unaligned_stride > kMaxSize - (slot_align - 1)) {
    throw std::overflow_error("RTMS segment size exceeds address space");
  }
  stride_ = align_up(unaligned_stride, slot_align);
  if (stride_ > (kMaxSize - data_offset_) / slots_) {
    throw std::overflow_error("RTMS segment size exceeds address space");
  }
  total_bytes_ = data_offset_ + stride_ * slots_;
  if (total_bytes_ > kMaxOff) {
    throw std::overflow_error("RTMS segment size exceeds off_t range");
  }

  open(schema);
}

void RTMSQueue::open(const talos::schema::Identity& schema) {
  const auto layout_matches = [this](const RTMSHeader& header) {
    return header.version == kRtmsVersion && header.slots == slots_ &&
           header.message_bytes == message_size_ &&
           header.message_alignment == message_alignment_ &&
           header.data_offset == data_offset_ &&
           header.slot_stride == stride_ && header.total_bytes == total_bytes_;
  };
  const auto schema_matches = [&schema](const RTMSHeader& header) {
    return header.schema_hash == schema.hash &&
           std::string_view{
               header.schema_name,
               ::strnlen(header.schema_name, sizeof(header.schema_name))} ==
               schema.name;
  };

  if (options_.session_owner) {
    replace_for_session();
  }

  // How often this open() has waited for a live claimant, whatever segment,
  // and the closing segment it last claimed and tried to unlink.
  int claimant_waits = 0;
  std::optional<std::uint64_t> unlinked_segment_id;
  for (;;) {
    if (!options_.session_owner) {
      ThrowIfVersionMismatch(path_);
    }
    SharedMemoryPtr ptr;
    try {
      ptr = SharedMemoryPtr{path_, total_bytes_};
    } catch (const std::runtime_error&) {
      if (!options_.session_owner) {
        ThrowIfVersionMismatch(path_);
      }
      throw;
    }
    auto* header = static_cast<RTMSHeader*>(ptr.ptr());

    if (ptr.mode() == SharedMemoryMode::CREATE) {
      std::uint64_t segment_id = 0;
      try {
        segment_id = NewSegmentId();
      } catch (...) {
        ptr = SharedMemoryPtr{};
        ::shm_unlink(ShmObjectName(path_).c_str());
        throw;
      }
      // Learned about this, pretty cool, it creates the object at
      // preallocated memory
      header = std::construct_at(header);
      header->version = kRtmsVersion;
      header->segment_id = segment_id;
      header->total_bytes = total_bytes_;
      header->slots = slots_;
      header->message_bytes = message_size_;
      header->message_alignment = message_alignment_;
      header->data_offset = data_offset_;
      header->slot_stride = stride_;
      header->schema_hash = schema.hash;
      std::memcpy(header->schema_name, schema.name.data(), schema.name.size());
      header->lifetime.store(options_.session_owner ? kLifetimeSessionOwned : 1,
                             std::memory_order_relaxed);
      counted_ = !options_.session_owner;
      const std::uint64_t self = OwnProcess();
      header->writer.sequence.store(0, std::memory_order_relaxed);
      header->writer.owner_pid.store(options_.publisher ? self : 0,
                                     std::memory_order_relaxed);
      header->writer.reliable_sequence.store(0, std::memory_order_relaxed);
      header->writer.waiters.store(0, std::memory_order_relaxed);
      for (auto& reader : header->readers) {
        reader.sequence.store(0, std::memory_order_relaxed);
        reader.state.store(0, std::memory_order_relaxed);
        reader.wake_id.store(0, std::memory_order_relaxed);
      }
      // Last, so an attacher that sees the magic sees everything above.
      header->magic.store(kRtmsMagic, std::memory_order_release);
      owner_process_ = self;
      ptr_ = std::move(ptr);
      header_ = header;
      return;
    }

    if (options_.session_owner) {
      throw std::runtime_error("RTMS: another process created " + path_ +
                               " while the session was creating it");
    }
    if (!WaitInitialized(*header)) {
      throw std::runtime_error(
          "RTMS shared memory " + path_ +
          " is not a talOS segment of this build's format (or its creator "
          "died initializing it); stop the processes using it, or remove it "
          "with shm_unlink, and restart");
    }
    if (header->version != kRtmsVersion) {
      throw VersionMismatchError(path_, header->version);
    }
    if (!layout_matches(*header)) {
      throw std::runtime_error(
          "RTMS shared-memory layout mismatch on " + path_ +
          ": this process was built against a different message layout than "
          "the segment's creator (" +
          std::to_string(header->message_bytes) + " bytes, " +
          std::to_string(header->slots) + " slots, here " +
          std::to_string(message_size_) + " bytes, " + std::to_string(slots_) +
          " slots)" + Remedy(path_));
    }
    if (!schema_matches(*header)) {
      throw std::runtime_error("RTMS schema mismatch on " + path_ +
                               ": the segment carries " + SchemaOf(*header) +
                               ", this process uses " +
                               talos::schema::Describe(schema) + Remedy(path_));
    }

    bool closing = false;
    std::uint64_t word = header->lifetime.load(std::memory_order_acquire);
    for (;;) {
      if ((word & kLifetimeSessionOwned) != 0) {
        counted_ = false;
        break;
      }
      if ((word & kLifetimeClosing) != 0) {
        closing = true;
        break;
      }
      if (header->lifetime.compare_exchange_weak(word, word + 1,
                                                 std::memory_order_acq_rel,
                                                 std::memory_order_acquire)) {
        counted_ = true;
        break;
      }
    }
    if (closing) {
      const std::uint64_t segment_id = header->segment_id;
      // This opener tried to unlink it, yet the name still holds it: give up.
      if (unlinked_segment_id == segment_id) break;
      // Its claimant, the last holder or an opener that took it over, is the
      // one process that unlinks it; the next shm_open creates a fresh
      // segment once the name is gone. Among processes of one build a live
      // claimant is waited for however long it is descheduled, never
      // superseded: an unlink in its place could race its own and remove a
      // segment created meanwhile.
      const std::uint64_t claimant = word & kLifetimeClaimantMask;
      if (claimant != 0 && ProcessAlive(claimant)) {
        if (claimant_waits >= claimant_wait_attempts_) break;
        ++claimant_waits;
        if (on_reclaim_wait_ != nullptr) {
          on_reclaim_wait_(segment_id, claimant_waits);
        }
        ::usleep(kInitRetryUs);
        continue;
      }
      // A claimant killed before unlinking it never will, and one whose
      // unlink failed gave its claim up. The count reached zero, so no queue
      // is attached (unless a session owner claimed it from its holders and
      // was killed before unlinking it, or gave its claim up when its unlink
      // failed): unless a live process still holds
      // its writer claim or a reader slot, this opener claims it from the
      // dead claimant (or from none) and unlinks it, under UnlinkIfSame()'s
      // condition only this segment and not a newer one under the name. An
      // opener that loses the claim retries and finds the winner, alive,
      // unlinking it.
      if (HeldByLiveProcess(*header)) break;
      if (header->lifetime.compare_exchange_strong(
              word, kLifetimeClosing | OwnProcess(), std::memory_order_acq_rel,
              std::memory_order_acquire)) {
        std::fprintf(stderr,
                     "rtms: reclaiming shared memory %s, whose claimant "
                     "died or gave up before unlinking it\n",
                     path_.c_str());
        if (!UnlinkIfSame(path_, segment_id, on_unlink_verified_)) {
          GiveUpClaim(*header);
        }
        unlinked_segment_id = segment_id;
      }
      continue;
    }
    if (options_.publisher) {
      try {
        ClaimWriter(*header, path_);
      } catch (...) {
        if (counted_) {
          const std::uint64_t segment_id = header->segment_id;
          counted_ = false;
          if (ReleaseCountedLifetime(*header) &&
              !UnlinkIfSame(path_, segment_id, on_unlink_verified_)) {
            GiveUpClaim(*header);
          }
        }
        throw;
      }
    }
    owner_process_ = OwnProcess();
    ptr_ = std::move(ptr);
    header_ = header;
    if (options_.publisher) wake_all_readers();
    return;
  }
  throw std::runtime_error(
      "RTMS: " + path_ +
      " stayed closing; its claimant never unlinked it, or a live process "
      "still holds it" +
      Remedy(path_));
}

void RTMSQueue::replace_for_session() {
  const std::string name = ShmObjectName(path_);
  // How often this open() has waited for a live claimant, whatever segment,
  // and how many other passes it has made (kSessionReplacePasses).
  int claimant_waits = 0;
  int replace_passes = 0;
  for (;;) {
    const HeaderView view{path_, HeaderView::Access::kWrite};
    if (!view.present() && !view.failed()) return;  // nothing to replace
    RTMSHeader* header = view.writable_header();
    // Not a segment whose lifetime word this build can read (it could not be
    // checked, is too small, never published its magic, or is of another
    // version), or a previous session's: no standalone claimant of this
    // build unlinks it, so the session does.
    if (header == nullptr || !WaitInitialized(*header) ||
        header->version != kRtmsVersion) {
      ::shm_unlink(name.c_str());
      return;
    }
    std::uint64_t word = header->lifetime.load(std::memory_order_acquire);
    if ((word & kLifetimeSessionOwned) != 0) {
      ::shm_unlink(name.c_str());
      return;
    }
    // A standalone segment. Its live claimant is the one process that
    // unlinks it, and is waited for as an opener waits for it, never
    // superseded: its unlink could follow this session's segment's creation.
    const std::uint64_t segment_id = header->segment_id;
    const std::uint64_t claimant = word & kLifetimeClaimantMask;
    if ((word & kLifetimeClosing) != 0 && claimant != 0 &&
        ProcessAlive(claimant)) {
      if (claimant_waits >= claimant_wait_attempts_) {
        throw SessionClaimantError(path_, name, claimant);
      }
      ++claimant_waits;
      if (on_reclaim_wait_ != nullptr) {
        on_reclaim_wait_(segment_id, claimant_waits);
      }
      ::usleep(kInitRetryUs);
      continue;
    }
    // Attached queues (a count), or a claimant that died or gave its claim
    // up: this session owner becomes the claimant, so no holder that closes
    // afterwards unlinks the name (ReleaseCountedLifetime), and unlinks it.
    // Holders keep their mapping of the unlinked segment until they close.
    if (++replace_passes > kSessionReplacePasses) return;
    if (!header->lifetime.compare_exchange_strong(
            word, kLifetimeClosing | OwnProcess(), std::memory_order_acq_rel,
            std::memory_order_acquire)) {
      continue;
    }
    if (on_session_claimed_ != nullptr) on_session_claimed_(segment_id);
    // On failure the name may still hold it, and open() then fails as it
    // does on a name taken while the session creates it.
    if (!UnlinkIfSame(path_, segment_id, on_unlink_verified_)) {
      GiveUpClaim(*header);
      return;
    }
    // The name no longer holds the claimed segment, but may hold one created
    // since (its claimant unlinked it before this claim, and an opener
    // created another): look again until the name is free.
  }
}

void RTMSQueue::close() noexcept {
  if (header_ == nullptr) return;
  // A forked child inherits its parent's RTMSQueue objects without having
  // opened its own attachment: it must not release the parent's reader slots,
  // writer claim, counted lifetime, or session-owned segment.
  const bool owns_attachment = owner_process_ == OwnProcess();
  std::uint64_t segment_id = 0;
  bool unlink = false;
  if (owns_attachment) {
    for (std::size_t id = 0; id < MAX_READERS; ++id) {
      if (registered_[id] != 0) release_reader(id);
    }
    if (options_.publisher) {
      std::uint64_t expected = owner_process_;
      header_->writer.owner_pid.compare_exchange_strong(
          expected, 0, std::memory_order_acq_rel);
    }
    segment_id = header_->segment_id;
    unlink = options_.session_owner;
    if (counted_) {
      unlink = ReleaseCountedLifetime(*header_);
    }
  }
  // Unlinked while still mapped, so a claim whose unlink failed can be given
  // up (a session-owned segment has no claim to give up).
  if (unlink && !UnlinkIfSame(path_, segment_id, on_unlink_verified_)) {
    GiveUpClaim(*header_);
  }
  registered_.fill(0);
  header_ = nullptr;
  counted_ = false;
  owner_process_ = 0;
  ptr_ = SharedMemoryPtr{};
}

RTMSQueue::RTMSQueue(RTMSQueue&& other) noexcept
    : path_{std::move(other.path_)},
      slots_{other.slots_},
      message_size_{other.message_size_},
      message_alignment_{other.message_alignment_},
      data_offset_{other.data_offset_},
      publish_time_offset_{other.publish_time_offset_},
      stride_{other.stride_},
      total_bytes_{other.total_bytes_},
      options_{other.options_},
      header_{std::exchange(other.header_, nullptr)},
      ptr_{std::move(other.ptr_)},
      counted_{std::exchange(other.counted_, false)},
      owner_process_{std::exchange(other.owner_process_, 0)},
      registered_{std::exchange(other.registered_, {})},
      wake_senders_{std::move(other.wake_senders_)},
      wakes_sent_{std::exchange(other.wakes_sent_, 0)},
      wake_failures_{
          other.wake_failures_.exchange(0, std::memory_order_relaxed)} {}

RTMSQueue& RTMSQueue::operator=(RTMSQueue&& other) noexcept {
  if (this != &other) {
    close();
    path_ = std::move(other.path_);
    slots_ = other.slots_;
    message_size_ = other.message_size_;
    message_alignment_ = other.message_alignment_;
    data_offset_ = other.data_offset_;
    publish_time_offset_ = other.publish_time_offset_;
    stride_ = other.stride_;
    total_bytes_ = other.total_bytes_;
    options_ = other.options_;
    header_ = std::exchange(other.header_, nullptr);
    ptr_ = std::move(other.ptr_);
    counted_ = std::exchange(other.counted_, false);
    owner_process_ = std::exchange(other.owner_process_, 0);
    registered_ = std::exchange(other.registered_, {});
    wake_senders_ = std::move(other.wake_senders_);
    wakes_sent_ = std::exchange(other.wakes_sent_, 0);
    wake_failures_.store(
        other.wake_failures_.exchange(0, std::memory_order_relaxed),
        std::memory_order_relaxed);
  }
  return *this;
}

RTMSQueue::~RTMSQueue() { close(); }

std::string_view RTMSQueue::schema_name() const {
  return std::string_view{
      header_->schema_name,
      ::strnlen(header_->schema_name, sizeof(header_->schema_name))};
}

// Need to find out where the "slowest" reader is.
uint64_t RTMSQueue::minimum_read_position() const {
  std::uint64_t minimum =
      header_->writer.sequence.load(std::memory_order_acquire);

  for (std::size_t i = 0; i < MAX_READERS; ++i) {
    // Sequentially consistent, completing the four-operation store-buffering
    // pair with write()'s seq_cst sequence store and activate_reader_slot()'s
    // seq_cst activation and sequence load: on ARMv8.3-RCPC (Apple Silicon) an
    // acquire load emits ldapr, which may reorder before the writer's prior
    // sequence store. Under seq_cst, either this DROP_NEWEST admission check
    // observes ACTIVE (and reads the reader's cursor below) or
    // activate_reader_slot() observes the published sequence and advances the
    // cursor past any overwritten slot.
    if ((header_->readers[i].state.load(std::memory_order_seq_cst) & 3) !=
        static_cast<std::uint64_t>(ReaderState::ACTIVE)) {
      continue;
    }
    const auto position =
        header_->readers[i].sequence.load(std::memory_order_acquire);
    minimum = std::min(minimum, position);
  }
  return minimum;
}

std::size_t RTMSQueue::reclaim_dead_readers() {
  std::size_t freed = 0;
  for (auto& reader : header_->readers) {
    std::uint64_t word = reader.state.load(std::memory_order_acquire);
    if (word == 0 || ProcessAlive(word >> 2)) continue;
    // Read while the dead holder still has the slot: nothing changes it.
    std::uint64_t wake_id = reader.wake_id.load(std::memory_order_acquire);
    if (reader.state.compare_exchange_strong(word, 0,
                                             std::memory_order_acq_rel)) {
      ++freed;
      // Only the dead holder's receiver: a new claimant may already have
      // recorded its own. The waiters bit stays: clearing it here could
      // erase a new claimant's arm, and a set bit with no receiver costs a
      // writer one exchange and no notification.
      reader.wake_id.compare_exchange_strong(wake_id, 0,
                                             std::memory_order_acq_rel);
    }
  }
  return freed;
}

WriteStatus RTMSQueue::write(std::span<const std::byte> message) {
  struct timespec now {};
#if defined(__APPLE__)
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &now);
#else
  ::clock_gettime(CLOCK_MONOTONIC, &now);
#endif
  return write(
      message,
      static_cast<std::int64_t>(now.tv_sec) * 1'000'000'000LL + now.tv_nsec);
}

WriteStatus RTMSQueue::write(std::span<const std::byte> message,
                             std::int64_t publish_time_ns) {
  // Low-level queues may omit publisher at construction; their first write
  // must still claim exclusive ownership before touching the ring.
  if (!options_.publisher) {
    ClaimWriter(*header_, path_);
    options_.publisher = true;
    wake_all_readers();
  }
  const std::uint64_t slot_mask = slots_ - 1;

  if (message.size() > header_->message_bytes) {
    std::cerr << "Cannot write message larger than slot size\n"
              << "message_bytes: " << header_->message_bytes
              << " > message_size: " << message.size() << "\n";
    return {WriteResult::ERROR_SIZE_MISSMATCH, 0};
  }

  const uint64_t writer_position =
      header_->writer.sequence.load(std::memory_order_relaxed);

  const uint64_t next_position = writer_position + 1;

  // DROP_NEWEST is the reliable mode: the writer refuses to pass the slowest
  // reader, so no reader ever misses a message. Every other policy keeps the
  // writer running at the cost of lapping slow readers, which read_next()
  // reports back to them as dropped messages. A stuck subscriber must never
  // be able to stall a control loop.
  if (options_.overflow_policy == OverflowPolicy::DROP_NEWEST) {
    // example max slots is 10, min reader is at 10 and new position is wrapping
    // to 20 will cause us to be writting over the reader.
    //
    // A reader whose process died holds its position forever, which would
    // wedge this writer for good; it is freed before the write is refused.
    // Only on the full path, so the steady state makes no system call: a
    // publication makes one only to wake a reader that armed (below). A
    // refused write publishes nothing and wakes nobody.
    if (next_position - minimum_read_position() > slots_) {
      // Re-check even when reclaim_dead_readers() returns 0: a concurrent peer
      // may have reclaimed the blocking dead reader (or a live reader may have
      // advanced) before our reclaim pass ran.
      (void)reclaim_dead_readers();
      if (next_position - minimum_read_position() > slots_) {
        return {WriteResult::BUFFER_FULL, 0};
      }
    }
  }

  // Revoke a former reliable publisher's full-capacity guarantee before any
  // overwrite, including one interrupted by writer death. The release fence
  // keeps this invalidation (and the prior sequence publication) before the
  // relaxed atomic slot word stores below; a reader that observes any word of
  // this overwrite synchronizes-with this fence at slot_still_valid()'s
  // acquire fence ([atomics.fences]/2) and must reject the candidate.
  if (options_.overflow_policy != OverflowPolicy::DROP_NEWEST) {
    header_->writer.reliable_sequence.store(0, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
  }

  const std::uint64_t slot_index = writer_position & slot_mask;
  auto* base = static_cast<std::byte*>(ptr_.ptr());
  auto* next_segment =
      base + header_->data_offset + (slot_index * header_->slot_stride);

  // Copy payload and timestamp as uniform 8-byte relaxed atomic words so a
  // concurrent read_next() on a lapped slot is well-defined (no C++ data
  // race). Short writes and sub-8-byte tails are zero-padded in the word
  // before storing, avoiding mixed-width atomic accesses.
  for (std::size_t off = 0; off < publish_time_offset_;
       off += sizeof(std::uint64_t)) {
    std::uint64_t word = 0;
    if (off < message.size()) {
      const std::size_t chunk =
          std::min<std::size_t>(sizeof(std::uint64_t), message.size() - off);
      std::memcpy(&word, message.data() + off, chunk);
    }
    std::atomic_ref<std::uint64_t>(
        *reinterpret_cast<std::uint64_t*>(next_segment + off))
        .store(word, std::memory_order_relaxed);
  }
  std::atomic_ref<std::int64_t>(
      *reinterpret_cast<std::int64_t*>(next_segment + publish_time_offset_))
      .store(publish_time_ns, std::memory_order_relaxed);
  if (options_.overflow_policy == OverflowPolicy::DROP_NEWEST) {
    header_->writer.reliable_sequence.store(next_position,
                                            std::memory_order_release);
  }
  // Sequentially consistent, pairing with arm_wake() (reader sets its waiters
  // bit then loads sequence; writer stores sequence then loads waiters) and
  // with activate_reader_slot() / minimum_read_position() (reader marks ACTIVE
  // then loads sequence; DROP_NEWEST writer stores sequence then loads reader
  // state on its next admission check). Under seq_cst at least one side of
  // each pair observes the other's store.
  header_->writer.sequence.store(next_position, std::memory_order_seq_cst);
  // No reader asleep: no exchange and no system call.
  if (header_->writer.waiters.load(std::memory_order_seq_cst) != 0) {
    // Exchange, then notify: each sleep of a reader costs at most one send.
    notify_readers(
        header_->writer.waiters.exchange(0, std::memory_order_seq_cst));
  }
  return {WriteResult::SUCCESS, writer_position};
}

std::byte* RTMSQueue::slot_address(std::uint64_t message_sequence) {
  const std::uint64_t slot_index = message_sequence & (header_->slots - 1);

  return static_cast<std::byte*>(ptr_.ptr()) + header_->data_offset +
         (slot_index * header_->slot_stride);
}

bool RTMSQueue::select_sequence(const Reader& reader, MessageInfo& info) const {
  const std::uint64_t reader_position =
      reader.sequence.load(std::memory_order_relaxed);

  const std::uint64_t writer_position =
      header_->writer.sequence.load(std::memory_order_acquire);

  // reader.sequence and writer.sequence both refer to the next sequence.
  if (reader_position >= writer_position) {
    return false;
  }

  std::uint64_t message_sequence = reader_position;

  // Published position P may already be writing sequence P, into the slot of
  // P - slots, so only the last `slots - 1` published sequences are
  // guaranteed intact. Under DROP_NEWEST the writer refuses to pass the
  // slowest reader, so this reader may also take P - slots while the
  // reliable-publication marker shows that slot's write completed: the full
  // ring, nothing skipped. Otherwise a reader `slots` or more behind was
  // lapped -- the writer overwrites, whatever this end's policy assumed --
  // and skips to the oldest intact sequence, reporting the rest as dropped,
  // rather than failing every read from then on.
  const std::uint64_t distance = writer_position - message_sequence;
  const bool full_ring =
      options_.overflow_policy == OverflowPolicy::DROP_NEWEST &&
      distance == header_->slots &&
      header_->writer.reliable_sequence.load(std::memory_order_acquire) ==
          writer_position;
  if (distance >= header_->slots && !full_ring) {
    message_sequence = writer_position - header_->slots + 1;
  }

  if (options_.read_mode == ReadMode::LATEST) {
    message_sequence = writer_position - 1;
  }

  info.sequence = message_sequence;
  info.dropped = message_sequence - reader_position;
  return true;
}

bool RTMSQueue::slot_still_valid(std::uint64_t message_sequence) const {
  // Pairs with the writer's release store. Everything read out of the slot
  // must be ordered before this load, or the check proves nothing.
  std::atomic_thread_fence(std::memory_order_acquire);

  const std::uint64_t writer_position =
      header_->writer.sequence.load(std::memory_order_relaxed);

  // Published position P means an overwriting writer may be writing sequence
  // P, in slot P & mask. Slot s is intact only while P < s + slots, even if
  // that write never completes or endpoint policies disagree. At equality,
  // only an explicitly completed reliable publication guarantees intact data.
  const std::uint64_t distance = writer_position - message_sequence;
  return distance < header_->slots ||
         (distance == header_->slots &&
          header_->writer.reliable_sequence.load(std::memory_order_acquire) ==
              writer_position);
}

ReadResult RTMSQueue::read_next(std::uint64_t reader_id,
                                std::span<std::byte> destination,
                                MessageInfo& info) {
  info = MessageInfo{};

  if (reader_id >= MAX_READERS || destination.size() < header_->message_bytes) {
    return ReadResult::INVALID;
  }

  if (!holds_reader(reader_id)) {
    return ReadResult::INACTIVE;
  }

  auto& reader = header_->readers[reader_id];

  for (int attempt = 0; attempt < MAX_READ_ATTEMPTS; ++attempt) {
    MessageInfo candidate{};

    if (!select_sequence(reader, candidate)) {
      return ReadResult::EMPTY;
    }

    std::byte* slot = slot_address(candidate.sequence);
    for (std::size_t off = 0; off < publish_time_offset_;
         off += sizeof(std::uint64_t)) {
      const std::uint64_t word =
          std::atomic_ref<std::uint64_t>(
              *reinterpret_cast<std::uint64_t*>(slot + off))
              .load(std::memory_order_relaxed);
      const std::size_t chunk =
          std::min<std::size_t>(sizeof(std::uint64_t), message_size_ - off);
      std::memcpy(destination.data() + off, &word, chunk);
    }
    candidate.publish_time_ns =
        std::atomic_ref<std::int64_t>(
            *reinterpret_cast<std::int64_t*>(slot + publish_time_offset_))
            .load(std::memory_order_relaxed);

    if (!slot_still_valid(candidate.sequence)) {
      continue;
    }

    // The cursor always points to the next sequence to consume.
    reader.sequence.store(candidate.sequence + 1, std::memory_order_release);

    info = candidate;
    return ReadResult::OK;
  }

  return ReadResult::TORN;
}

std::optional<std::uint64_t> RTMSQueue::claim_reader_slot(
    std::size_t reader_id) {
  auto& reader = header_->readers[reader_id];
  std::uint64_t expected = 0;

  // The claim carries this process's pid, so a claimant that dies at any
  // point after it -- even before it marks the slot active -- leaves a
  // slot reclaim_dead_readers() can free.
  if (!reader.state.compare_exchange_strong(
          expected, ReaderWord(ReaderState::CLAIMING),
          std::memory_order_acq_rel, std::memory_order_relaxed)) {
    return std::nullopt;
  }

  // A previous holder's receiver and arm do not carry over to this one.
  header_->writer.waiters.fetch_and(~(std::uint64_t{1} << reader_id),
                                    std::memory_order_acq_rel);
  reader.wake_id.store(0, std::memory_order_release);

  const auto current = header_->writer.sequence.load(std::memory_order_acquire);

  reader.sequence.store(current, std::memory_order_relaxed);
  return current;
}

bool RTMSQueue::activate_reader_slot(std::size_t reader_id,
                                     std::uint64_t current) {
  auto& reader = header_->readers[reader_id];
  const std::uint64_t active = ReaderWord(ReaderState::ACTIVE);

  // Only while the claim is still this one: a process that judged this
  // one dead may have freed it meanwhile, and another claimed the slot.
  // Both this activation CAS and the writer.sequence load below are
  // sequentially consistent, forming a four-operation store-buffering pair
  // with write()'s seq_cst sequence store and minimum_read_position()'s
  // seq_cst reader.state load (on ARMv8.3-RCPC / Apple Silicon, release +
  // acquire compiles to casl + ldapr, allowing ldapr to reorder before casl).
  // Under seq_cst, either a DROP_NEWEST writer's admission check sees ACTIVE
  // and respects this reader's cursor, or writer_after sees the publication
  // and advances the cursor past any overwritten slot.
  std::uint64_t expected = ReaderWord(ReaderState::CLAIMING);
  if (!reader.state.compare_exchange_strong(expected, active,
                                            std::memory_order_seq_cst,
                                            std::memory_order_relaxed)) {
    return false;
  }
  registered_[reader_id] = active;
  const auto writer_after =
      header_->writer.sequence.load(std::memory_order_seq_cst);
  if (writer_after > current && writer_after - current >= header_->slots) {
    reader.sequence.store(writer_after, std::memory_order_release);
  }
  return true;
}

std::optional<std::size_t> RTMSQueue::register_reader() {
  for (int pass = 0; pass < 2; ++pass) {
    for (std::size_t i = 0; i < MAX_READERS; ++i) {
      const std::optional<std::uint64_t> current = claim_reader_slot(i);
      if (!current.has_value()) continue;
      if (!activate_reader_slot(i, *current)) continue;
      return i;
    }
    if (pass == 0) {
      // Even if reclaim_dead_readers() frees 0 slots here, a concurrent peer
      // may have just reclaimed dead slots (or released one) after our first
      // scan passed them; rescan once if any slot is now free.
      const std::size_t freed = reclaim_dead_readers();
      bool any_free = freed > 0;
      for (std::size_t i = 0; !any_free && i < MAX_READERS; ++i) {
        if (header_->readers[i].state.load(std::memory_order_acquire) == 0) {
          any_free = true;
        }
      }
      if (!any_free) break;
    }
  }
  return std::nullopt;
}

void RTMSQueue::release_reader(std::size_t id) noexcept {
  if (header_ == nullptr || id >= MAX_READERS) return;
  // Only a slot this queue claimed in this process -- a forked child
  // inherits its parent's registrations -- and only while it still holds it.
  std::uint64_t expected = std::exchange(registered_[id], 0);
  if (expected != ReaderWord(ReaderState::ACTIVE)) return;
  auto& reader = header_->readers[id];
  // Atomically transition ACTIVE -> CLAIMING before clearing waiters/wake_id so
  // we cannot clobber a new claimant's wake state if this slot was taken over.
  // While CLAIMING with our live process word, no other process can claim the
  // slot and minimum_read_position() already ignores it.
  std::uint64_t releasing = ReaderWord(ReaderState::CLAIMING);
  if (!reader.state.compare_exchange_strong(expected, releasing,
                                            std::memory_order_acq_rel,
                                            std::memory_order_acquire)) {
    return;
  }
  // Disarmed before the slot is freed, so no writer notifies a receiver its
  // holder may close, nor the slot's next holder for this one's arm.
  header_->writer.waiters.fetch_and(~(std::uint64_t{1} << id),
                                    std::memory_order_acq_rel);
  reader.wake_id.store(0, std::memory_order_release);
  reader.state.compare_exchange_strong(releasing, 0, std::memory_order_acq_rel,
                                       std::memory_order_acquire);
}

void RTMSQueue::set_reader_wake(std::size_t reader_id, std::uint64_t wake_id) {
  if (header_ == nullptr || reader_id >= MAX_READERS ||
      !holds_reader(reader_id)) {
    return;
  }
  const std::uint64_t clear_mask = ~(std::uint64_t{1} << reader_id);
  if (wake_id == 0) {
    header_->readers[reader_id].wake_id.store(0, std::memory_order_release);
    header_->writer.waiters.fetch_and(clear_mask, std::memory_order_acq_rel);
    return;
  }
  // Disarm any stale waiters bit from an earlier receiver (or wake_id == 0)
  // before storing the new wake_id so the next arm_wake() must run its seq_cst
  // fetch_or after this release store, ordering wake_id before any writer
  // notification.
  header_->writer.waiters.fetch_and(clear_mask, std::memory_order_acq_rel);
  header_->readers[reader_id].wake_id.store(wake_id, std::memory_order_release);
}

bool RTMSQueue::arm_wake(std::size_t reader_id) {
  if (header_ == nullptr || reader_id >= MAX_READERS ||
      !holds_reader(reader_id)) {
    return false;
  }
  const std::uint64_t bit = std::uint64_t{1} << reader_id;
  // A bit still set from an earlier arm was stored by an earlier
  // sequentially consistent read-modify-write, which orders it before the
  // load below just as a new one would; a writer that has since taken it
  // has notified, and that notification is still pending.
  if ((header_->writer.waiters.load(std::memory_order_relaxed) & bit) == 0) {
    header_->writer.waiters.fetch_or(bit, std::memory_order_seq_cst);
  }
  // The same comparison select_sequence() makes to report EMPTY.
  const std::uint64_t reader_position =
      header_->readers[reader_id].sequence.load(std::memory_order_relaxed);
  const std::uint64_t writer_position =
      header_->writer.sequence.load(std::memory_order_seq_cst);
  return reader_position >= writer_position;
}

void RTMSQueue::notify_readers(std::uint64_t mask) {
  for (std::size_t i = 0; i < MAX_READERS; ++i) {
    if ((mask & (std::uint64_t{1} << i)) == 0) continue;
    const std::uint64_t wake_id =
        header_->readers[i].wake_id.load(std::memory_order_acquire);
    if (wake_id == 0) continue;
    // A receiver that is gone needs nothing: the slot's holder, or the
    // reclaim of its dead process, clears the id. Any other failure lost the
    // hint, and the reader waits for its loop's backstop.
    const talos::WakeStatus status = wake_senders_[i].send(wake_id);
    ++wakes_sent_;
    if (status.outcome == talos::WakeOutcome::FAILED) {
      if (wake_failures_.fetch_add(1, std::memory_order_relaxed) == 0) {
        std::fprintf(stderr,
                     "talos: RTMS %s: a wake notification failed (%s); a "
                     "reader may wait for its backstop. Further failures on "
                     "this topic are counted, not logged.\n",
                     path_.c_str(), std::strerror(status.error));
      }
    }
  }
}

void RTMSQueue::wake_all_readers() {
  // An exchange, not a store: reading the waiters word through the chain of
  // read-modify-writes on it (a reader's arm, a dead writer's exchange)
  // makes each armed reader's wake_id visible here.
  header_->writer.waiters.exchange(0, std::memory_order_seq_cst);
  notify_readers(~std::uint64_t{0});
}

#include "rtms.h"

#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
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

// Bounded like SharedMemoryPtr's size wait: a creator initializes a header
// within microseconds, so 50 ms separates "still being created" from "never
// will be" without masking either.
constexpr int kInitAttempts = 200;
constexpr useconds_t kInitRetryUs = 250;
// How often open() waits for the last holder of a closing segment to unlink
// it before deciding that holder died first.
constexpr int kOpenAttempts = 200;

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

// A read-only mapping of the header of an existing segment, or nothing when
// the name is absent or too small to hold a header within the wait.
class HeaderView {
 public:
  explicit HeaderView(const std::string& path) {
    const int fd = ::shm_open(ShmObjectName(path).c_str(), O_RDONLY, 0666);
    if (fd == -1) return;
    struct stat info {};
    bool sized = false;
    for (int attempt = 0; attempt < kInitAttempts; ++attempt) {
      if (::fstat(fd, &info) < 0) break;
      if (static_cast<std::uint64_t>(info.st_size) >= sizeof(RTMSHeader)) {
        sized = true;
        break;
      }
      ::usleep(kInitRetryUs);
    }
    if (sized) {
      void* mapped =
          ::mmap(nullptr, sizeof(RTMSHeader), PROT_READ, MAP_SHARED, fd, 0);
      if (mapped != MAP_FAILED) {
        header_ = static_cast<const RTMSHeader*>(mapped);
      }
    }
    ::close(fd);
  }
  ~HeaderView() {
    if (header_ != nullptr) {
      ::munmap(const_cast<RTMSHeader*>(header_), sizeof(RTMSHeader));
    }
  }
  HeaderView(const HeaderView&) = delete;
  HeaderView& operator=(const HeaderView&) = delete;

  // Null when the segment is too small to be an RTMS segment.
  const RTMSHeader* header() const { return header_; }

 private:
  const RTMSHeader* header_{nullptr};
};

// Unlinks `path` only while it still names the segment `segment_id`: a
// session may have replaced it, or an opener recovered it from a killed
// closer, since this process mapped it, and that newer segment is not ours
// to remove.
void UnlinkIfSame(const std::string& path, std::uint64_t segment_id) {
  bool same = false;
  {
    const HeaderView view{path};
    same = view.header() != nullptr &&
           view.header()->magic.load(std::memory_order_acquire) == kRtmsMagic &&
           view.header()->segment_id == segment_id;
  }
  if (same) ::shm_unlink(ShmObjectName(path).c_str());
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

}  // namespace

RTMSQueue::RTMSQueue(std::string_view path, std::size_t message_size,
                     std::size_t message_alignment,
                     const talos::schema::Identity& schema, std::size_t slots,
                     RTMSOptions options)
    : path_{ValidatePath(path)},
      slots_{slots},
      message_size_{message_size},
      message_alignment_{message_alignment},
      data_offset_{is_pow_2(message_alignment)
                       ? align_up(sizeof(RTMSHeader),
                                  rtms_slot_alignment(message_alignment))
                       : 0},
      publish_time_offset_{rtms_publish_time_offset(message_size)},
      stride_{is_pow_2(message_alignment)
                  ? rtms_slot_stride(message_size, message_alignment)
                  : 0},
      total_bytes_{data_offset_ + stride_ * slots_},
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
    // A new session replaces whatever the name held: a segment of a
    // previous session that ended without cleaning up, or a standalone run's.
    ::shm_unlink(ShmObjectName(path_).c_str());
  }

  for (int attempt = 0; attempt <= kOpenAttempts; ++attempt) {
    SharedMemoryPtr ptr{path_, total_bytes_};
    auto* header = static_cast<RTMSHeader*>(ptr.ptr());

    if (ptr.mode() == SharedMemoryMode::CREATE) {
      // Learned about this, pretty cool, it creates the object at
      // preallocated memory
      header = std::construct_at(header);
      header->version = kRtmsVersion;
      header->segment_id = NewSegmentId();
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
      header->writer.sequence.store(0, std::memory_order_relaxed);
      header->writer.owner_pid.store(options_.publisher ? OwnProcess() : 0,
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
      throw std::runtime_error("RTMS shared-memory version mismatch on " +
                               path_ + ": segment version " +
                               std::to_string(header->version) + ", expected " +
                               std::to_string(kRtmsVersion) + Remedy(path_));
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
      // Its last holder is unlinking it; the next shm_open creates a fresh
      // segment once the name is gone.
      if (attempt + 1 < kOpenAttempts) {
        ::usleep(kInitRetryUs);
        continue;
      }
      // A holder killed after marking it closing never will. Its count
      // reached zero, so no queue is attached; once the wait is spent this
      // segment -- never a newer one under the name -- is unlinked here,
      // unless a live process still holds its writer claim or a reader slot.
      if (attempt + 1 == kOpenAttempts && !HeldByLiveProcess(*header)) {
        std::fprintf(stderr,
                     "rtms: reclaiming shared memory %s, whose last holder "
                     "died before unlinking it\n",
                     path_.c_str());
        UnlinkIfSame(path_, header->segment_id);
        continue;
      }
      break;
    }
    if (options_.publisher) {
      try {
        ClaimWriter(*header, path_);
      } catch (...) {
        if (counted_) {
          header->lifetime.fetch_sub(1, std::memory_order_acq_rel);
          counted_ = false;
        }
        throw;
      }
    }
    ptr_ = std::move(ptr);
    header_ = header;
    if (options_.publisher) wake_all_readers();
    return;
  }
  throw std::runtime_error(
      "RTMS: " + path_ + " stayed closing; its last holder never unlinked it");
}

void RTMSQueue::close() noexcept {
  if (header_ == nullptr) return;
  if (options_.publisher) {
    std::uint64_t expected = OwnProcess();
    header_->writer.owner_pid.compare_exchange_strong(
        expected, 0, std::memory_order_acq_rel);
  }
  const std::uint64_t segment_id = header_->segment_id;
  bool unlink = options_.session_owner;
  if (counted_) {
    std::uint64_t word = header_->lifetime.load(std::memory_order_acquire);
    for (;;) {
      const std::uint64_t count = word & kLifetimeCountMask;
      const std::uint64_t next =
          count <= 1 ? ((word & ~kLifetimeCountMask) | kLifetimeClosing)
                     : word - 1;
      if (header_->lifetime.compare_exchange_weak(word, next,
                                                  std::memory_order_acq_rel,
                                                  std::memory_order_acquire)) {
        unlink = count <= 1;
        break;
      }
    }
  }
  header_ = nullptr;
  counted_ = false;
  ptr_ = SharedMemoryPtr{};
  if (unlink) UnlinkIfSame(path_, segment_id);
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
      header_->writer.sequence.load(std::memory_order_relaxed);

  for (std::size_t i = 0; i < MAX_READERS; ++i) {
    // Reader hasn't started yet, so position isn't relevent
    if (!is_active(header_->readers[i])) {
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
    if (next_position - minimum_read_position() > slots_ &&
        (reclaim_dead_readers() == 0 ||
         next_position - minimum_read_position() > slots_)) {
      return {WriteResult::BUFFER_FULL, 0};
    }
  }

  // Revoke a former reliable publisher's full-capacity guarantee before any
  // overwrite, including one interrupted by writer death. The release fence
  // keeps this invalidation before slot writes; readers fence their slot reads
  // before checking the marker, as with a seqlock's write-in-progress marker.
  if (options_.overflow_policy != OverflowPolicy::DROP_NEWEST) {
    header_->writer.reliable_sequence.store(0, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
  }

  const std::uint64_t slot_index = writer_position & slot_mask;
  auto* base = static_cast<std::byte*>(ptr_.ptr());
  auto* next_segment =
      base + header_->data_offset + (slot_index * header_->slot_stride);

  std::memcpy(next_segment, message.data(), message.size());
  if (message.size() < header_->message_bytes) {
    std::memset(next_segment + message.size(), 0,
                header_->message_bytes - message.size());
  }
  std::memcpy(next_segment + publish_time_offset_, &publish_time_ns,
              sizeof(publish_time_ns));
  if (options_.overflow_policy == OverflowPolicy::DROP_NEWEST) {
    header_->writer.reliable_sequence.store(next_position,
                                            std::memory_order_release);
  }
  // Sequentially consistent, pairing with arm_wake(): a reader sets its
  // waiters bit and then loads the sequence, the writer stores the sequence
  // and then loads the waiters word, so either the reader sees this
  // publication and does not sleep, or the writer sees the bit and wakes it.
  header_->writer.sequence.store(next_position, std::memory_order_seq_cst);
  // No reader asleep: no exchange and no system call.
  if (header_->writer.waiters.load(std::memory_order_seq_cst) != 0) {
    // Exchange, then notify: each sleep of a reader costs at most one send.
    notify_readers(
        header_->writer.waiters.exchange(0, std::memory_order_seq_cst));
  }
  return {WriteResult::SUCCESS, writer_position};
}

const std::byte* RTMSQueue::slot_address(std::uint64_t message_sequence) const {
  const std::uint64_t slot_index = message_sequence & (header_->slots - 1);

  return static_cast<const std::byte*>(ptr_.ptr()) + header_->data_offset +
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

    const std::byte* slot = slot_address(candidate.sequence);
    std::memcpy(destination.data(), slot, header_->message_bytes);
    std::memcpy(&candidate.publish_time_ns, slot + publish_time_offset_,
                sizeof(candidate.publish_time_ns));

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

std::optional<std::size_t> RTMSQueue::register_reader() {
  const std::uint64_t claiming = ReaderWord(ReaderState::CLAIMING);
  const std::uint64_t active = ReaderWord(ReaderState::ACTIVE);
  for (int pass = 0; pass < 2; ++pass) {
    for (std::size_t i = 0; i < MAX_READERS; ++i) {
      auto& reader = header_->readers[i];
      std::uint64_t expected = 0;

      // The claim carries this process's pid, so a claimant that dies at any
      // point after it -- even before it marks the slot active -- leaves a
      // slot reclaim_dead_readers() can free.
      if (!reader.state.compare_exchange_strong(expected, claiming,
                                                std::memory_order_acq_rel,
                                                std::memory_order_relaxed)) {
        continue;
      }

      // A previous holder's receiver and arm do not carry over to this one.
      header_->writer.waiters.fetch_and(~(std::uint64_t{1} << i),
                                        std::memory_order_acq_rel);
      reader.wake_id.store(0, std::memory_order_release);

      const auto current =
          header_->writer.sequence.load(std::memory_order_acquire);

      reader.sequence.store(current, std::memory_order_relaxed);

      // Only while the claim is still this one: a process that judged this
      // one dead may have freed it meanwhile, and another claimed the slot.
      expected = claiming;
      if (!reader.state.compare_exchange_strong(expected, active,
                                                std::memory_order_release,
                                                std::memory_order_relaxed)) {
        continue;
      }
      registered_[i] = active;
      const auto writer_after =
          header_->writer.sequence.load(std::memory_order_acquire);
      if (writer_after > current && writer_after - current > header_->slots) {
        reader.sequence.store(writer_after, std::memory_order_release);
      }
      return i;
    }
    if (reclaim_dead_readers() == 0) break;
  }
  return std::nullopt;
}

void RTMSQueue::release_reader(std::size_t id) {
  if (header_ == nullptr || id >= MAX_READERS) return;
  // Only a slot this queue claimed in this process -- a forked child
  // inherits its parent's registrations -- and only while it still holds it.
  std::uint64_t expected = std::exchange(registered_[id], 0);
  if (expected != ReaderWord(ReaderState::ACTIVE)) return;
  auto& reader = header_->readers[id];
  if (reader.state.load(std::memory_order_acquire) != expected) return;
  // Disarmed before the slot is freed, so no writer notifies a receiver its
  // holder may close, nor the slot's next holder for this one's arm.
  header_->writer.waiters.fetch_and(~(std::uint64_t{1} << id),
                                    std::memory_order_acq_rel);
  reader.wake_id.store(0, std::memory_order_release);
  reader.state.compare_exchange_strong(expected, 0, std::memory_order_acq_rel,
                                       std::memory_order_acquire);
}

void RTMSQueue::set_reader_wake(std::size_t reader_id, std::uint64_t wake_id) {
  if (header_ == nullptr || reader_id >= MAX_READERS ||
      !holds_reader(reader_id)) {
    return;
  }
  // Release: a writer that takes this slot's waiters bit with its acquiring
  // exchange then reads this receiver.
  header_->readers[reader_id].wake_id.store(wake_id, std::memory_order_release);
  if (wake_id == 0) {
    header_->writer.waiters.fetch_and(~(std::uint64_t{1} << reader_id),
                                      std::memory_order_acq_rel);
  }
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

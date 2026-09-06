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
 * -- and then keeps a heartbeat and a set of counters current while it runs.
 * Studio and any agent map the segment read-only and get the whole system:
 * which processes exist, which topics each one publishes and subscribes to, and
 * how much traffic has actually moved.
 *
 * The layout is frozen and the version is part of the segment name, so a node
 * from an older build cannot be misread by a newer reader: it lands in a
 * different segment instead.
 *
 * Writers touch only their own slot. Every field a reader observes while a node
 * runs is a lock-free atomic, and identity fields are fenced behind a
 * generation counter, so reading requires no cooperation from the nodes and
 * cannot block them.
 */

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/introspection/names.h"

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
inline constexpr std::uint32_t kRegistryVersion = 1;
inline constexpr const char* kRegistryPath = "/talos_registry.1";

inline constexpr std::size_t kMaxNodes = 32;
inline constexpr std::size_t kMaxSourcesPerNode = 64;

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

// One event source, as the owning node registered it. The descriptive fields
// are written once, before the slot goes ACTIVE; the counters keep moving.
struct SourceRecord {
  std::uint16_t id;
  std::uint16_t kind;  // event::SourceKind
  std::uint32_t message_bytes;
  std::uint32_t alignment;

  // naming::kSourceFlagExternal, naming::kSourceFlagOptional: how this end's
  // far end is expected to behave, so a graph tool can tell a designed dead
  // end from a fault. Written once with the descriptive fields, never after.
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

  // Messages the transport lapped before this source could read them, or
  // publishes the transport refused.
  std::atomic<std::uint64_t> dropped;

  // Transport sequence last observed, which is what makes a subscriber's lag
  // computable against its publisher.
  std::atomic<std::uint64_t> sequence;

  std::atomic<std::int64_t> last_monotonic_ns;
  std::atomic<std::int64_t> last_latency_ns;
  std::atomic<std::int64_t> max_latency_ns;
};

// One node. Padded to a cache line so two nodes writing their own counters do
// not contend on the same line.
struct alignas(64) NodeRecord {
  std::atomic<std::uint64_t> state;

  // Bumped on every claim. A reader that sees the same generation before and
  // after copying the identity fields knows the slot was not recycled
  // underneath it.
  std::atomic<std::uint64_t> generation;

  // Wall clock, because the reader is a different process and the monotonic
  // clocks need not agree.
  std::atomic<std::int64_t> heartbeat_wall_ns;

  // Published after the source records are written, so a reader never walks
  // entries that are still being filled in.
  std::atomic<std::uint32_t> source_count;
  std::atomic<std::uint32_t> flags;

  std::atomic<std::uint64_t> dispatch_count;

  // How many sources the node registered, which exceeds source_count when it
  // registered more than this segment can hold.
  std::atomic<std::uint32_t> declared_source_count;
  std::uint32_t reserved;

  std::int64_t start_wall_ns;
  std::uint64_t pid;
  std::uint64_t session_id;
  char name[kMaxNameBytes];
  char target[kMaxTargetBytes];

  SourceRecord sources[kMaxSourcesPerNode];
};

struct RegistryHeader {
  // 0 in a fresh segment. The first writer CASes it to kInitializing, fills in
  // the rest, then stores kMagic; everyone else waits for kMagic.
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
static_assert(std::is_standard_layout_v<RegistryHeader>);

// The claim that let kRegistryVersion stay at 1 when endpoint attributes
// arrived: `flags` occupies the word `reserved` held, immediately after
// `alignment`, so no field after it moved. A future edit that inserts something
// here instead of reusing a reserved word has changed the layout and must bump
// the version.
static_assert(std::is_standard_layout_v<SourceRecord>);
static_assert(offsetof(SourceRecord, flags) ==
              offsetof(SourceRecord, alignment) + sizeof(std::uint32_t));

inline std::int64_t wall_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
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

// Maps the registry segment, creating it if this is the first process to ask.
//
// Deliberately not SharedMemoryPtr: that unlinks on destruction, which is right
// for a topic owned by one publisher and wrong here. The registry outlives any
// single node, so nothing unlinks it -- a stale segment is harmless because
// slots are reclaimed by heartbeat and the version is in the name.
class RegistryMapping {
 public:
  explicit RegistryMapping(std::string_view path = kRegistryPath,
                           bool read_only = false)
      : path_{path} {
    const std::size_t size = sizeof(RegistryHeader);
    const int flags = read_only ? O_RDONLY : (O_CREAT | O_RDWR);

    int fd = ::shm_open(path_.c_str(), flags, 0666);
    if (fd < 0) {
      throw std::runtime_error("registry shm_open failed for " + path_ + ": " +
                               std::strerror(errno));
    }

    struct stat status{};
    if (::fstat(fd, &status) < 0) {
      ::close(fd);
      throw std::runtime_error("registry fstat failed for " + path_);
    }

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

    if (read_only) {
      await_initialized();
    } else {
      initialize();
    }
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
    ::shm_unlink(std::string{path}.c_str());
  }

 private:
  void initialize() {
    std::uint64_t expected = 0;
    if (header_->magic.compare_exchange_strong(expected, kInitializing,
                                               std::memory_order_acq_rel)) {
      // A fresh segment is zero-filled, so only the descriptive fields need
      // writing. Re-initializing an existing segment would blow away live
      // nodes, which is exactly what the CAS prevents.
      header_->version = kRegistryVersion;
      header_->node_capacity = static_cast<std::uint32_t>(kMaxNodes);
      header_->source_capacity = static_cast<std::uint32_t>(kMaxSourcesPerNode);
      header_->node_record_bytes = sizeof(NodeRecord);
      header_->source_record_bytes = sizeof(SourceRecord);
      header_->magic.store(kMagic, std::memory_order_release);
      return;
    }
    await_initialized();
  }

  void await_initialized() const {
    // The winner of the CAS above publishes kMagic microseconds later. Bounded
    // so a segment left in kInitializing by a process that died mid-setup is
    // reported instead of hanging a node forever.
    for (int attempt = 0; attempt < 200; ++attempt) {
      if (header_->magic.load(std::memory_order_acquire) == kMagic) {
        if (header_->version != kRegistryVersion ||
            header_->node_record_bytes != sizeof(NodeRecord) ||
            header_->source_record_bytes != sizeof(SourceRecord)) {
          throw std::runtime_error(
              "registry " + path_ +
              " was created by a process with a different record layout");
        }
        return;
      }
      ::usleep(250);
    }
    throw std::runtime_error("registry " + path_ +
                             " was never initialized; remove it with "
                             "shm_unlink and restart the nodes");
  }

  std::string path_;
  RegistryHeader* header_{nullptr};
};

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

  NodeRegistration(Identity identity, std::string_view path = kRegistryPath)
      : mapping_{path}, identity_{std::move(identity)} {
    record_ = claim(mapping_.header(), identity_);
    if (record_ == nullptr) {
      throw std::runtime_error(
          "registry is full: " + std::to_string(kMaxNodes) +
          " nodes are already registered");
    }
  }

  ~NodeRegistration() { release(); }

  NodeRegistration(const NodeRegistration&) = delete;
  NodeRegistration& operator=(const NodeRegistration&) = delete;

  // Writes the node's event sources, then publishes the count. Call once, after
  // the event loop has closed registration: the manifest is fixed from then on,
  // so it can be read from another thread without synchronization.
  void publish(const event::Manifest& manifest) {
    write_sources(manifest, [](std::string_view) { return std::uint32_t{0}; });
  }

  // The same, plus endpoint attributes keyed by topic name.
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
  void publish(const event::Manifest& manifest, const Attributes& attributes) {
    write_sources(manifest, [&attributes](std::string_view name) {
      std::uint32_t flags = 0;
      for (const auto& attribute : attributes) {
        if (attribute.topic == name) {
          flags |= attribute.flags;
        }
      }
      return flags;
    });
  }

  void heartbeat() {
    record_->heartbeat_wall_ns.store(wall_now_ns(), std::memory_order_relaxed);
  }

  void set_dispatch_count(std::uint64_t count) {
    record_->dispatch_count.store(count, std::memory_order_relaxed);
  }

  // Copies one source's counters into the slot. Called from the reporting
  // thread with values read out of the loop's own atomics.
  void set_source(std::uint32_t index, std::uint64_t events,
                  std::uint64_t dropped, std::uint64_t sequence,
                  std::int64_t last_monotonic_ns, std::int64_t last_latency_ns,
                  std::int64_t max_latency_ns) {
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

  const std::string& name() const { return identity_.name; }
  std::size_t slot() const {
    return static_cast<std::size_t>(record_ - mapping_.header()->nodes);
  }

 private:
  template <typename FlagsFor>
  void write_sources(const event::Manifest& manifest, FlagsFor flags_for) {
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
      source.flags = flags_for(registration.name);
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
      record_->source_count.store(0, std::memory_order_relaxed);
      record_->state.store(static_cast<std::uint64_t>(SlotState::FREE),
                           std::memory_order_release);
      record_ = nullptr;
    }
  }

  // Takes a free slot, or -- only when none is free -- one whose owner stopped
  // heartbeating long enough ago to be considered gone.
  static NodeRecord* claim(RegistryHeader* header, const Identity& identity) {
    NodeRecord* record = claim_where(header, SlotState::FREE, false);
    if (record == nullptr) {
      record = claim_where(header, SlotState::ACTIVE, true);
    }
    if (record == nullptr) {
      return nullptr;
    }

    record->pid = static_cast<std::uint64_t>(::getpid());
    record->session_id = identity.session_id;
    record->start_wall_ns = wall_now_ns();
    copy_name(record->name, kMaxNameBytes, identity.name);
    copy_name(record->target, kMaxTargetBytes, identity.target);
    record->flags.store(identity.flags, std::memory_order_relaxed);
    record->source_count.store(0, std::memory_order_relaxed);
    record->declared_source_count.store(0, std::memory_order_relaxed);
    record->dispatch_count.store(0, std::memory_order_relaxed);
    record->heartbeat_wall_ns.store(wall_now_ns(), std::memory_order_relaxed);

    // A reader that saw the previous occupant must observe a different
    // generation, so bump it before the slot goes live.
    record->generation.fetch_add(1, std::memory_order_acq_rel);
    record->state.store(static_cast<std::uint64_t>(SlotState::ACTIVE),
                        std::memory_order_release);
    return record;
  }

  static NodeRecord* claim_where(RegistryHeader* header, SlotState from,
                                 bool require_stale) {
    const std::int64_t now = wall_now_ns();
    const std::int64_t timeout =
        std::chrono::duration_cast<std::chrono::nanoseconds>(kLivenessTimeout)
            .count();

    for (std::size_t i = 0; i < kMaxNodes; ++i) {
      NodeRecord& record = header->nodes[i];
      if (require_stale) {
        const std::int64_t beat =
            record.heartbeat_wall_ns.load(std::memory_order_relaxed);
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
  std::int64_t heartbeat_wall_ns{0};
  std::uint32_t flags{0};
  std::uint32_t declared_source_count{0};
  bool alive{false};
  std::vector<SourceSnapshot> sources;
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

  std::vector<NodeSnapshot> nodes() const {
    std::vector<NodeSnapshot> out;
    const RegistryHeader* header = mapping_.header();
    const std::int64_t now = wall_now_ns();
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
        NodeSnapshot node = copy(record, i);
        if (record.generation.load(std::memory_order_acquire) != generation &&
            attempt == 0) {
          continue;
        }
        node.generation = generation;
        node.alive = now - node.heartbeat_wall_ns < timeout;
        out.push_back(std::move(node));
        break;
      }
    }
    return out;
  }

 private:
  static NodeSnapshot copy(const NodeRecord& record, std::size_t slot) {
    NodeSnapshot node;
    node.slot = slot;
    node.name = read_name(record.name, kMaxNameBytes);
    node.target = read_name(record.target, kMaxTargetBytes);
    node.pid = record.pid;
    node.session_id = record.session_id;
    node.start_wall_ns = record.start_wall_ns;
    node.heartbeat_wall_ns =
        record.heartbeat_wall_ns.load(std::memory_order_relaxed);
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
    return node;
  }

  RegistryMapping mapping_;
};

}  // namespace talos::introspect

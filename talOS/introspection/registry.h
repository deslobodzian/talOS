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
 * the lane each of its Targets selected, whether the devices flush is
 * holding back outputs it set, whether its event log stopped early, the
 * realtime scheduling its loop got, and what its wake channel lost. Studio
 * and any agent map
 * the segment read-only and get the whole system: which processes exist,
 * which topics each one publishes and subscribes to, how much traffic has
 * actually moved, who drives each subsystem and which node asks to move
 * with nobody driving, which recordings are incomplete and which loops run
 * without their realtime grant.
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

#include <sys/stat.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/introspection/lock_file.h"
#include "talOS/introspection/names.h"
#include "talOS/process/realtime_outcome.h"

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
// Version 6 adds the wake counters at NodeRecord's end (wake_failures,
// stalled_arms, wake_fallbacks). They sit in what was tail padding, so
// sizeof(NodeRecord) stays 11136 B, but no earlier writer zeroes that
// padding on a claim: a slot one build wrote and an older build reclaimed
// would show the first node's counts against the second.
//
// The event log flags (kFlagRecorderFailed, kFlagLogCapped) did not bump it:
// they are bits of `flags` that every earlier writer left clear, which reads
// as "no stopped log seen" -- all an earlier build could have said -- and
// every earlier reader tests only the bits it knows.
//
// Nor did naming::kSourceFlagLatestOnly: a bit of SourceRecord::flags that
// every earlier writer left clear, which reads as "every drop is a loss" --
// what an earlier reader assumed of every source.
//
// Nor did the realtime outcome (kFlagRealtimeMask): bits of `flags` that
// every earlier writer left clear, which reads as "not reported" -- all an
// earlier build could have said -- and every earlier reader ignores.
//
// Nor did naming::kSourceFlagUntargetedOutputs: a bit of SourceRecord::flags
// that every earlier writer left clear, which reads as "gated on a live
// target" -- all an earlier build could have said -- and every earlier
// reader tests only the bits it knows.
//
// Nor did naming::kSourceFlagOperatorArming (bit 6) or
// naming::kSourceFlagInternalDriverStation (bit 7): bits of
// SourceRecord::flags that every earlier writer left clear, which fail closed
// as "unarmed" and "explicit Driver Station reader", and every earlier reader
// ignores.
//
// Nor did counting a timer's missed periods in SourceRecord::dropped: a
// timer's `dropped` was always 0, which still reads as "no overrun".
//
// Nor did kFlagOutputsHeld: a bit of `flags` that every earlier writer left
// clear, which reads as "nothing held back" -- all an earlier build could
// have said -- and every earlier reader tests only the bits it knows.
//
// Nor did kFlagLogGap: a bit of `flags` that every earlier writer left clear,
// which reads as "no gap seen" -- all an earlier build could have said (its
// writer stopped capture instead, kFlagRecorderFailed) -- and every earlier
// reader tests only the bits it knows.
inline constexpr std::uint32_t kRegistryVersion = 6;
inline constexpr const char* kRegistryPath = "/talos_registry.6";
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

// Event log flags: what the node's event log lost. kFlagRecorderFailed
// mirrors LogWriter::failed() -- the disk refused a write or the log file
// could not be created, so the rest of the run is not recorded --
// kFlagLogCapped mirrors capped(), the log reaching its byte budget, and
// kFlagLogGap (below) mirrors gaps() > 0, records dropped while capture
// went on. Set while the node runs
// (NodeRegistration::set_recording) and never cleared: each says what the
// run's log lost, which stays lost; a new claim starts without them.
inline constexpr std::uint32_t kFlagRecorderFailed = 1u << 3;
inline constexpr std::uint32_t kFlagLogCapped = 1u << 4;

// The loop's realtime scheduling (process::ConfigureRealtime's outcome), as
// bits 5 to 12 of `flags`: the tier plus one in the low two (0: not reported
// yet, or a writer that predates them), whether the steps were requested,
// then the process::kStep* bits the OS refused. Written once per claim
// (NodeRegistration::set_realtime), after the node has asked; RealtimeOf
// decodes it.
inline constexpr std::uint32_t kFlagRealtimeShift = 5;
inline constexpr std::uint32_t kFlagRealtimeMask = 0xFFu << kFlagRealtimeShift;

inline constexpr std::uint32_t RealtimeFlags(
    const process::RealtimeOutcome& outcome) {
  const std::uint32_t tier = static_cast<std::uint32_t>(outcome.tier) + 1u;
  const std::uint32_t requested = outcome.requested ? 1u : 0u;
  const auto refused =
      static_cast<std::uint32_t>(outcome.refused & process::kAllSteps);
  return (tier | requested << 2 | refused << 3) << kFlagRealtimeShift;
}

// The outcome a row's `flags` carry, or nullopt while none is reported.
inline constexpr std::optional<process::RealtimeOutcome> RealtimeOf(
    std::uint32_t flags) {
  const std::uint32_t bits = (flags & kFlagRealtimeMask) >> kFlagRealtimeShift;
  if ((bits & 3u) == 0) return std::nullopt;
  return process::RealtimeOutcome{
      .tier = static_cast<process::LoopTier>((bits & 3u) - 1u),
      .requested = (bits & 4u) != 0,
      .refused = static_cast<std::uint8_t>(bits >> 3)};
}

// The devices flush is holding back outputs the node set (REQ-SAFE-41): its
// last flush ran with the robot enabled, the node not opted out
// (AllowUntargetedOutputs), no talos::Target returning a target in that
// update(), and that update() setting an output non-neutral, so the flush
// sent every output neutral. Unlike the event log flags it clears: the
// Reporter copies the node's current answer in each full sample
// (NodeRegistration::set_outputs_held), so it is at most one refresh behind.
inline constexpr std::uint32_t kFlagOutputsHeld = 1u << 13;
static_assert((kFlagOutputsHeld & kFlagRealtimeMask) == 0,
              "kFlagOutputsHeld must not share a bit with the realtime "
              "outcome");

// The node's event log has a gap: records its writer could not take were
// dropped (LogWriter gaps() > 0). Capture resumes after a GAP record that
// marks them. The bit is set as the gap opens: the writer ORs it into the
// row (LogWriterOptions::gap_flags, NodeRegistration::flags_word()) when it
// drops the gap's first record, while the row is still its claim's, so a
// run killed during a gap, which has no GAP record, still has the bit. Each
// sample also copies gaps() > 0 (set_recording), for a writer not given the
// row. An event log flag (above): never cleared.
inline constexpr std::uint32_t kFlagLogGap = 1u << 14;
static_assert((kFlagLogGap & kFlagRealtimeMask) == 0,
              "kFlagLogGap must not share a bit with the realtime outcome");
static_assert((kFlagLogGap & kFlagOutputsHeld) == 0,
              "kFlagLogGap must not share a bit with kFlagOutputsHeld");

static_assert(process::kAllSteps < (1u << 5),
              "the refused steps must fit the five bits kFlagRealtimeMask "
              "leaves them");
static_assert(static_cast<std::uint32_t>(process::LoopTier::kBridge) + 1u < 4u,
              "the tier plus one must fit two bits");

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
  // the transport refused, or reads it failed; for a timer, the periods it
  // missed because the loop overran (SourceCounters).
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

  // What the node's wake channel lost, cumulative since the claim, written in
  // each sample (NodeRegistration::set_wake_loss; TODO.md RT-7). Each should
  // stay 0. wake_failures: notifications its writers failed to send for a
  // reason other than a gone receiver (RTMSQueue::wake_failures(): no
  // descriptor left, say), each leaving a reader asleep until its backstop.
  // stalled_arms: prompt watchers whose failed arms stopped holding its loop
  // awake. wake_fallbacks: runs that could not build their wake receiver and
  // drain on a poll tick instead (the bridge: OpenWaits() fallbacks).
  std::atomic<std::uint64_t> wake_failures;
  std::atomic<std::uint64_t> stalled_arms;
  std::atomic<std::uint64_t> wake_fallbacks;
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

// The segment's layout, as docs/DATA_FLOW.md states it: a change to either
// size is a change of layout, which bumps kRegistryVersion and that table.
static_assert(alignof(NodeRecord) == 64);
static_assert(sizeof(NodeRecord) == 11136);
static_assert(sizeof(RegistryHeader) == 356416);

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

// Optional startup-only fault-injection seams, each called at most once per
// mapping or registration, with RegistryLock held. Production callers leave
// these null; tests stop a child while it owns the lock, before either
// commit, or act between a registration's map and its claim (`mapped`).
// `lock_file_opened` is RegistryLock's alone, called without the lock once
// it has opened the lock file the first time, before it locks it: the point
// after which unlinking the path leaves that acquirer on a stale file.
struct RegistryTestHooks {
  void (*initializing)(RegistryHeader*){nullptr};
  void (*mapped)(RegistryHeader*){nullptr};
  void (*claiming)(NodeRecord*){nullptr};
  void (*lock_file_opened)(){nullptr};
};

// Serializes the writers: segment creation, sizing and initialization
// (RegistryMapping, PrepareRegistry), a registration's map and slot claim,
// taken once for both (NodeRegistration), and a namespaced launcher
// session's removal of the registry; a reader never takes it. No holder
// waits for another lock; the launcher holds its session locks while it
// takes this one, and no holder of this one takes those. Use a regular file
// (LockFilePath): Darwin's shm descriptors are not vnodes, so flock on them
// fails with ENOTSUP. Each acquisition opens a separate file description, which
// also serializes threads in one process. close (including process death)
// releases the lock. Only a launcher session in a shared-memory namespace
// unlinks this file, under the lock, once its members are reaped and no other
// session holds a row (NamespaceNames, launcher.cc); if anything unlinks it, an
// acquirer that wins the old inode finds it no longer current and starts over.
class RegistryLock {
 public:
  explicit RegistryLock(const std::string& mapped_path);

  // Waits at most `wait` when it is set, then throws: a holder that keeps
  // the lock that long is stuck (stopped mid-registration), not registering.
  RegistryLock(const std::string& mapped_path,
               std::optional<std::chrono::milliseconds> wait,
               const RegistryTestHooks* test_hooks = nullptr);

  ~RegistryLock();
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
void ShareRegistrySegment(int fd, const struct stat& status);

// Maps the registry segment, creating it if this is the first process to ask.
//
// Deliberately not SharedMemoryPtr: that unlinks on destruction, which is right
// for a topic owned by one publisher and wrong here. The registry outlives any
// single node: outside a shared-memory namespace nothing unlinks it -- a
// stale segment is harmless because slots are reclaimed by heartbeat or
// claimant death and the version is in the name. A launcher session in a
// namespace unlinks it once its members are reaped, unless a row of another
// session whose claimant is still running remains (NamespaceNames,
// launcher.cc).
class RegistryMapping {
 public:
  explicit RegistryMapping(std::string_view path = kRegistryPath,
                           bool read_only = false,
                           const RegistryTestHooks* test_hooks = nullptr);

  // A writer's mapping for a caller that already holds `held`, this path's
  // RegistryLock (PrepareRegistry, NodeRegistration); taking it again would
  // wait on itself.
  RegistryMapping(std::string_view path, const RegistryLock& held,
                  const RegistryTestHooks* test_hooks = nullptr);

  ~RegistryMapping();

  RegistryMapping(const RegistryMapping&) = delete;
  RegistryMapping& operator=(const RegistryMapping&) = delete;

  RegistryMapping(RegistryMapping&& other) noexcept;
  RegistryMapping& operator=(RegistryMapping&& other) noexcept;

  RegistryHeader* header() const { return header_; }
  const std::string& path() const { return path_; }

  // Removes the segment's name. Only tests need this; a robot leaves the
  // registry in place between runs.
  static void unlink(std::string_view path = kRegistryPath);

 private:
  void map(bool read_only, const RegistryTestHooks* test_hooks);
  void initialize(const RegistryTestHooks* test_hooks);
  void validate_initialized() const;

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
void PrepareRegistry(std::string_view path = kRegistryPath,
                     std::chrono::milliseconds lock_wait = kRegistryLockWait);

// What a launcher session in a shared-memory namespace does to the registry
// once its members are reaped (the launcher's NamespaceNames): removes the
// segment and then its writer lock file, unless the lock is held or a row of
// another session than `session_id` names a claimant that still runs. True
// when it removed them; held, absent or not this layout, it leaves them.
bool RemoveRegistryUnlessHeld(std::uint64_t session_id,
                              std::string_view path = kRegistryPath);

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

  // Maps the registry and claims a slot under one RegistryLock acquisition,
  // so a claim lands in the segment the path names: a launcher session that
  // removes the registry (NamespaceNames, launcher.cc) does so under that
  // lock, either before the map, and the map creates a new segment, or after
  // the claim, when the row keeps it. Locked for microseconds: shm_open,
  // mmap, a header check and a scan of kMaxNodes rows, all before the node's
  // constructor and its loop run.
  explicit NodeRegistration(Identity identity,
                            std::string_view path = kRegistryPath,
                            const RegistryTestHooks* test_hooks = nullptr);

  ~NodeRegistration();

  NodeRegistration(const NodeRegistration&) = delete;
  NodeRegistration& operator=(const NodeRegistration&) = delete;

  // Writes the node's event sources, then publishes the count. Call once, after
  // the event loop has closed registration: the manifest is fixed from then on,
  // so it can be read from another thread without synchronization.
  void publish(const event::Manifest& manifest);

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

  // Copies the node's wake counters into the row (NodeRecord::wake_failures
  // and after). Called from the reporting thread inside a sample.
  void set_wake_loss(std::uint64_t wake_failures, std::uint64_t stalled_arms,
                     std::uint64_t wake_fallbacks) {
    if (!owns_slot()) return;
    record_->wake_failures.store(wake_failures, std::memory_order_relaxed);
    record_->stalled_arms.store(stalled_arms, std::memory_order_relaxed);
    record_->wake_fallbacks.store(wake_fallbacks, std::memory_order_relaxed);
  }

  // Sets kFlagRecorderFailed / kFlagLogCapped / kFlagLogGap for an event log
  // that failed, reached its cap or has a gap; none is ever cleared. Called
  // inside a sample.
  void set_recording(bool failed, bool capped, bool gap);

  // Sets or clears kFlagOutputsHeld, leaving every other bit of `flags` as
  // it is. Called inside a sample; writes only when the answer changed.
  void set_outputs_held(bool held);

  // Publishes the loop's realtime outcome (kFlagRealtimeMask). Call once,
  // from the thread that asked, right after process::ConfigureRealtime, or
  // with requested false when the node did not ask. The bits are clear
  // until then: a claim starts without them.
  void set_realtime(const process::RealtimeOutcome& outcome);

  // Records additional refused realtime steps (process::kStep*) after
  // set_realtime() has already published a requested outcome (tier != 0 and
  // requested bit set), e.g. a late shared-memory mapping whose mlock was
  // refused. No-op until then.
  void note_realtime_refused(std::uint8_t steps);

  void activate();

  // The row's `flags` word, for a writer that ORs an event log flag in the
  // moment it applies (event::log::LogWriterOptions::gap_flags with
  // kFlagLogGap) rather than at the next sample, with the row's
  // `generation` and the value this claim took; all null when the slot is
  // not this claim's. Lives as long as this registration. Every write to
  // `flags` after the claim is an atomic read-modify-write (fetch_or /
  // fetch_and of the bits it owns), so a writer of another bit never clears
  // it. A writer sets its bit only while `generation` still holds
  // `claimed`: a row swept from a live owner whose heartbeat went
  // kLivenessTimeout stale and claimed by another node (a new generation)
  // is left alone, so the new owner's run is not marked log_gap. The
  // swept owner's own gap is then in no row: its row is gone, and a run
  // the launcher kills during that gap has no GAP record either. A swept
  // row nobody claimed keeps its generation and takes the bit.
  struct FlagsWord {
    std::atomic<std::uint32_t>* flags{nullptr};
    const std::atomic<std::uint64_t>* generation{nullptr};
    std::uint64_t claimed{0};
  };
  FlagsWord flags_word() noexcept {
    if (!owns_slot()) return {};
    return {&record_->flags, &record_->generation, generation_};
  }

  const std::string& name() const { return identity_.name; }
  std::size_t slot() const {
    return static_cast<std::size_t>(record_ - mapping_.header()->nodes);
  }

 private:
  // The public constructor's work, with `held` taken for the map and the
  // claim alike.
  NodeRegistration(Identity identity, std::string_view path,
                   const RegistryTestHooks* test_hooks,
                   const RegistryLock& held);

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

  void check_names(const event::Manifest& manifest);
  void release();

  static void sweep_stale(RegistryHeader* header);
  static NodeRecord* claim(RegistryHeader* header, const Identity& identity,
                           const RegistryTestHooks* test_hooks);
  static NodeRecord* claim_oldest_free(RegistryHeader* header);
  static NodeRecord* claim_where(RegistryHeader* header, SlotState from,
                                 bool require_stale);

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

// What a row says its node lost of what it should have delivered or been
// woken for, all cumulative since the claim and all normally 0 (TODO.md
// RT-7): the row's wake counters (NodeRecord::wake_failures and after), and
// requests_skipped, the requests the hardware bridge's latest-value reads of
// `/hw/request/<node>` passed over (its per-subsystem source rows' `dropped`,
// summed; 0 for every other node).
struct DeliveryLoss {
  std::uint64_t wake_failures{0};
  std::uint64_t stalled_arms{0};
  std::uint64_t wake_fallbacks{0};
  std::uint64_t requests_skipped{0};

  bool any() const {
    return (wake_failures | stalled_arms | wake_fallbacks | requests_skipped) !=
           0;
  }
};

// A reader of a hardware request topic: the bridge's per-subsystem source,
// whose `dropped` counts the requests its reads skipped.
bool IsRequestRead(event::SourceKind kind, std::string_view name);

// The same, read straight from a row by a reader that copies no snapshot
// (the launcher's supervision pass). Relaxed loads: each counter only grows,
// and a value one sample old is as good as the next.
DeliveryLoss DeliveryLossOf(const NodeRecord& record);

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
  // The loop's realtime outcome, once the node reported it.
  std::optional<process::RealtimeOutcome> realtime() const;
  // kFlagOutputsHeld: the devices flush is holding back outputs it set.
  bool outputs_held() const { return (flags & kFlagOutputsHeld) != 0; }
  // kFlagLogGap: the node's event log has dropped records (set as a gap
  // opens).
  bool log_gap() const { return (flags & kFlagLogGap) != 0; }
  std::vector<SourceSnapshot> sources;
  std::uint32_t declared_selection_count{0};
  std::vector<SelectionSnapshot> selections;
  // NodeRecord's wake counters.
  std::uint64_t wake_failures{0};
  std::uint64_t stalled_arms{0};
  std::uint64_t wake_fallbacks{0};

  DeliveryLoss delivery_loss() const;
};

// Read-only view of the registry. Safe to poll at any rate: it takes no locks
// and writes nothing, so it cannot perturb the nodes it is observing.
class RegistryReader {
 public:
  explicit RegistryReader(std::string_view path = kRegistryPath);

  // Attaches only if some node has already created the registry. Returns
  // nullopt rather than throwing, because "no nodes are running" is an ordinary
  // state for a viewer to start up in.
  static std::optional<RegistryReader> open(
      std::string_view path = kRegistryPath);

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
  std::vector<NodeSnapshot> held_rows() const;

  std::vector<NodeSnapshot> nodes() const;

 private:
  struct StableRow {
    NodeSnapshot node;
    std::uint64_t claimant{0};
  };

  static std::optional<StableRow> read_stable_row(const NodeRecord& record,
                                                  std::size_t slot);
  static std::optional<NodeSnapshot> copy_stable(const NodeRecord& record,
                                                 std::size_t slot);
  static NodeSnapshot copy(const NodeRecord& record, std::size_t slot);

  RegistryMapping mapping_;
};

}  // namespace talos::introspect

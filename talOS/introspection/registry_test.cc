#include "talOS/introspection/registry.h"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#if defined(__linux__)
#include <sched.h>
#include <sys/mount.h>
#include <sys/prctl.h>
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "talOS/events/log/log_writer.h"
#include "talOS/events/manifest.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/introspection/reporter.h"
#include "talOS/process/process.h"
#include "talOS/rtms/rtms.h"
#include "talOS/rtms/wake.h"

namespace talos::introspect {
namespace {

using namespace std::chrono_literals;

// macOS caps shared-memory object names at 31 characters, and concurrent test
// targets must not share a segment, so every test gets a short unique name.
std::string temp_registry(std::string_view tag) {
  return "/tr_" + std::string{tag} + "_" + std::to_string(::getpid());
}

// The schema the raw 8-byte test topics register under.
const schema::Identity kCountSchema{"registry_test.Count", 0xc0};

class Registry : public ::testing::Test {
 protected:
  void SetUp() override { RegistryMapping::unlink(path_); }
  void TearDown() override { RegistryMapping::unlink(path_); }

  std::string path_ = temp_registry("reg");
};
// Always reap children, including on an assertion failure while one is stopped.
class Children {
 public:
  ~Children() {
    for (const pid_t pid : pids_) ::kill(pid, SIGKILL);
    for (const pid_t pid : pids_) Wait(pid, nullptr, 0);
  }

  void add(pid_t pid) { pids_.push_back(pid); }

  static pid_t Wait(pid_t pid, int* status, int options) {
    pid_t result;
    do {
      result = ::waitpid(pid, status, options);
    } while (result < 0 && errno == EINTR);
    return result;
  }

  int reap(pid_t pid) {
    int status = -1;
    if (Wait(pid, &status, 0) == pid) {
      pids_.erase(std::find(pids_.begin(), pids_.end(), pid));
    }
    return status;
  }

 private:
  std::vector<pid_t> pids_;
};

class Pipe {
 public:
  Pipe() {
    if (::pipe(fds_) < 0) throw std::runtime_error("pipe failed");
  }
  ~Pipe() {
    ::close(fds_[0]);
    ::close(fds_[1]);
  }
  int reader() const { return fds_[0]; }
  int writer() const { return fds_[1]; }

 private:
  int fds_[2];
};

bool read_all(int fd, void* destination, std::size_t size) {
  auto* bytes = static_cast<char*>(destination);
  while (size != 0) {
    const ssize_t count = ::read(fd, bytes, size);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) return false;
    bytes += count;
    size -= static_cast<std::size_t>(count);
  }
  return true;
}

bool write_all(int fd, const void* source, std::size_t size) {
  const auto* bytes = static_cast<const char*>(source);
  while (size != 0) {
    const ssize_t count = ::write(fd, bytes, size);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) return false;
    bytes += count;
    size -= static_cast<std::size_t>(count);
  }
  return true;
}

TEST_F(Registry, KilledInitializerIsRecoveredByTheNextWriter) {
  Children children;
  const pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    const RegistryTestHooks hooks{.initializing = [](RegistryHeader* header) {
      // Leave partial metadata and slot residue, not just an empty header.
      header->node_capacity = 999;
      header->nodes[0].state.store(
          static_cast<std::uint64_t>(SlotState::CLAIMING));
      ::raise(SIGSTOP);
    }};
    try {
      RegistryMapping mapping{path_, false, &hooks};
      ::_exit(1);  // The parent must kill us before initialization commits.
    } catch (...) {
      ::_exit(2);
    }
  }
  children.add(pid);
  int status = 0;
  ASSERT_EQ(Children::Wait(pid, &status, WUNTRACED), pid);
  ASSERT_TRUE(WIFSTOPPED(status));
  ASSERT_EQ(WSTOPSIG(status), SIGSTOP);
  ASSERT_EQ(::kill(pid, SIGKILL), 0);
  status = children.reap(pid);
  ASSERT_TRUE(WIFSIGNALED(status));
  ASSERT_EQ(WTERMSIG(status), SIGKILL);

  // A read-only observer cannot publish initialization on a writer's behalf.
  EXPECT_THROW((RegistryMapping{path_, true}), std::runtime_error);
  NodeRegistration registration{{.name = "recovered"}, path_};
  RegistryMapping mapping{path_};
  EXPECT_EQ(mapping.header()->node_capacity, kMaxNodes);
  EXPECT_EQ(mapping.header()->magic.load(), kMagic);
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.front().name, "recovered");
  EXPECT_TRUE(nodes.front().alive);
}

TEST_F(Registry, KilledClaimantDoesNotConsumeAnyOfThe32Slots) {
  RegistryMapping mapping{path_};
  Children children;
  const pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    const RegistryTestHooks hooks{
        .claiming = [](NodeRecord*) { ::raise(SIGSTOP); }};
    try {
      NodeRegistration registration{{.name = "killed"}, path_, &hooks};
      ::_exit(1);
    } catch (...) {
      ::_exit(2);
    }
  }
  children.add(pid);
  int status = 0;
  ASSERT_EQ(Children::Wait(pid, &status, WUNTRACED), pid);
  ASSERT_TRUE(WIFSTOPPED(status));
  ASSERT_EQ(WSTOPSIG(status), SIGSTOP);
  EXPECT_EQ(mapping.header()->nodes[0].state.load(),
            static_cast<std::uint64_t>(SlotState::CLAIMING));
  // The kill is before even the claimant pid/heartbeat is recorded.
  EXPECT_EQ(mapping.header()->nodes[0].pid, 0u);
  ASSERT_EQ(::kill(pid, SIGKILL), 0);
  status = children.reap(pid);
  ASSERT_TRUE(WIFSIGNALED(status));
  ASSERT_EQ(WTERMSIG(status), SIGKILL);

  std::vector<std::unique_ptr<NodeRegistration>> registrations;
  for (std::size_t i = 0; i < kMaxNodes; ++i) {
    registrations.push_back(std::make_unique<NodeRegistration>(
        NodeRegistration::Identity{.name = "node" + std::to_string(i)}, path_));
    EXPECT_EQ(registrations.back()->slot(), i);
  }
  EXPECT_THROW(NodeRegistration({.name = "overflow"}, path_),
               std::runtime_error);
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  EXPECT_EQ(reader->nodes().size(), kMaxNodes);
}

TEST_F(Registry, ConcurrentProcessesNeverReceiveTheSameSlot) {
  constexpr std::size_t kProcesses = 8;
  constexpr std::size_t kClaimsPerProcess = kMaxNodes / kProcesses;
  Pipe start;
  Pipe results;
  Pipe finish;
  Children children;
  std::array<pid_t, kProcesses> pids{};
  for (std::size_t process = 0; process < kProcesses; ++process) {
    const pid_t pid = ::fork();
    ASSERT_GE(pid, 0);
    if (pid == 0) {
      char token;
      if (!read_all(start.reader(), &token, 1)) ::_exit(2);
      std::array<std::size_t, kClaimsPerProcess> slots;
      slots.fill(kMaxNodes);  // Report failure instead of blocking the parent.
      std::vector<std::unique_ptr<NodeRegistration>> registrations;
      try {
        for (std::size_t i = 0; i < kClaimsPerProcess; ++i) {
          registrations.push_back(std::make_unique<NodeRegistration>(
              NodeRegistration::Identity{.name = "node" +
                                                 std::to_string(process) + "_" +
                                                 std::to_string(i)},
              path_));
          slots[i] = registrations.back()->slot();
        }
      } catch (...) {
        slots.fill(
            kMaxNodes);  // Explicitly report a failed claim to the parent.
      }
      // One pipe-atomic report; registrations stay alive until all reports
      // have been checked, so releasing a slot cannot hide double assignment.
      if (!write_all(results.writer(), slots.data(), sizeof(slots))) ::_exit(3);
      if (!read_all(finish.reader(), &token, 1)) ::_exit(4);
      registrations.clear();
      ::_exit(0);
    }
    children.add(pid);
    pids[process] = pid;
  }
  const std::array<char, kProcesses> tokens{};
  ASSERT_TRUE(write_all(start.writer(), tokens.data(), tokens.size()));
  std::array<bool, kMaxNodes> seen{};
  for (std::size_t process = 0; process < kProcesses; ++process) {
    std::array<std::size_t, kClaimsPerProcess> slots{};
    ASSERT_TRUE(read_all(results.reader(), slots.data(), sizeof(slots)));
    for (const std::size_t slot : slots) {
      ASSERT_LT(slot, kMaxNodes);
      EXPECT_FALSE(seen[slot]) << "slot " << slot << " assigned twice";
      seen[slot] = true;
    }
  }
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  EXPECT_EQ(reader->nodes().size(), kMaxNodes);
  EXPECT_THROW(NodeRegistration({.name = "overflow"}, path_),
               std::runtime_error);
  ASSERT_TRUE(write_all(finish.writer(), tokens.data(), tokens.size()));
  for (const pid_t pid : pids) {
    const int status = children.reap(pid);
    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 0);
  }
  EXPECT_TRUE(reader->nodes().empty());
}

// The registry the `mapped` hook below removes, and how often it ran; a hook
// is a plain function.
std::string removal_path;
int removal_attempts = 0;

// REQ-OPS-17, REQ-OPS-03
TEST_F(Registry, AClaimIsSeenThoughASessionRemovesTheRegistryAfterTheMap) {
  // A node outside any session has mapped the registry and not yet claimed;
  // a namespaced launcher session (99) ends then, holding no row, and
  // removes the registry as it does once its members are reaped
  // (RemoveRegistryUnlessHeld, which the launcher calls). The claim
  // that follows must land in a segment a later reader maps, not in the
  // unlinked one: else the node is invisible to Studio and to liveness.
  removal_path = path_;
  removal_attempts = 0;
  const RegistryTestHooks hooks{.mapped = [](RegistryHeader*) {
    ++removal_attempts;
    static_cast<void>(RemoveRegistryUnlessHeld(99, removal_path));
  }};
  NodeRegistration registration{{.name = "outsider"}, path_, &hooks};
  ASSERT_EQ(removal_attempts, 1);

  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value())
      << "the claim landed in a registry no later reader maps";
  const auto nodes = reader->nodes();
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.front().name, "outsider");
  EXPECT_TRUE(nodes.front().alive);
}

event::Manifest sample_manifest() {
  event::Manifest manifest;
  manifest.push_back({.id = 0,
                      .kind = event::SourceKind::TIMER,
                      .name = "tick",
                      .period_ns = 5'000'000});
  manifest.push_back({.id = 1,
                      .kind = event::SourceKind::WATCHER,
                      .name = "/drivetrain/state",
                      .message_bytes = 48,
                      .alignment = 8});
  manifest.push_back({.id = 2,
                      .kind = event::SourceKind::SENDER,
                      .name = "/odometry/state",
                      .message_bytes = 64,
                      .alignment = 8});
  return manifest;
}

// What Reporter::Selection carries into publish_selections: a policy's
// sources.
struct Policy {
  std::vector<std::string> sources;
};

TEST_F(Registry, ATargetsSelectedSourceReadsBackAndClearsToNone) {
  NodeRegistration registration{{.name = "lift"}, path_};
  registration.publish(sample_manifest());
  registration.publish_selections(
      std::vector<Policy>{{{"align", "teleop", "auto"}}});
  auto reader = RegistryReader::open(path_);
  if (!reader.has_value()) {
    FAIL() << "Expected registry reader";
  }
  const auto& registry = reader.value();

  const auto selection = [&registry] {
    const auto nodes = registry.nodes();
    EXPECT_EQ(nodes.size(), 1u);
    EXPECT_EQ(nodes.front().selections.size(), 1u);
    return nodes.front().selections.front();
  };

  // Nothing has selected yet: nobody drives.
  EXPECT_EQ(selection().sources,
            (std::vector<std::string>{"align", "teleop", "auto"}));
  EXPECT_EQ(selection().selected_source(), std::nullopt);

  registration.set_selected(0, 1);
  EXPECT_EQ(selection().selected_source(), "teleop");
  registration.set_selected(0, 2);
  EXPECT_EQ(selection().selected_source(), "auto");
  registration.set_selected(0, -1);
  EXPECT_EQ(selection().selected, -1);
  EXPECT_EQ(selection().selected_source(), std::nullopt);
}

// The flag Studio shows as "outputs held" sets and clears with the node's
// answer, and clearing it leaves the bits written once beside it (the
// realtime outcome, the event log flags).
// REQ-SAFE-41
TEST_F(Registry, OutputsHeldSetsAndClearsWithoutTouchingTheOtherFlags) {
  NodeRegistration registration{{.name = "gate"}, path_};
  registration.publish(sample_manifest());
  registration.set_realtime({.tier = process::LoopTier::kControl});
  registration.set_recording(/*failed=*/false, /*capped=*/true, /*gap=*/false);
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto row = [&reader] {
    const auto nodes = reader->nodes();
    EXPECT_EQ(nodes.size(), 1u);
    return nodes.front();
  };
  const std::uint32_t before = row().flags;
  EXPECT_FALSE(row().outputs_held());

  registration.set_outputs_held(true);
  EXPECT_TRUE(row().outputs_held());
  EXPECT_EQ(row().flags, before | kFlagOutputsHeld);
  registration.set_outputs_held(false);
  EXPECT_FALSE(row().outputs_held());
  EXPECT_EQ(row().flags, before);
}

TEST_F(Registry, SelectionsBeyondTheSlotAreCountedAndARecycledSlotHasNone) {
  std::size_t crowded = 0;
  {
    std::vector<Policy> policies(kMaxSelectionsPerNode + 1,
                                 Policy{{"teleop", "auto"}});
    // One policy longer than the slot's name table.
    for (std::size_t i = 0; i < kMaxSelectionSources + 2; ++i) {
      policies.front().sources.push_back("s" + std::to_string(i));
    }
    NodeRegistration registration{{.name = "crowded"}, path_};
    registration.publish({});
    registration.publish_selections(policies);
    crowded = registration.slot();
    const std::int32_t last =
        static_cast<std::int32_t>(policies.front().sources.size() - 1);
    registration.set_selected(0, last);

    auto reader = RegistryReader::open(path_);
    if (!reader.has_value()) {
      FAIL() << "Expected registry reader";
    }
    const auto& registry = reader.value();
    const auto nodes = registry.nodes();
    ASSERT_EQ(nodes.size(), 1u);
    const NodeSnapshot& node = nodes.front();
    EXPECT_EQ(node.declared_selection_count, kMaxSelectionsPerNode + 1);
    ASSERT_EQ(node.selections.size(), kMaxSelectionsPerNode);
    const SelectionSnapshot& first = node.selections.front();
    EXPECT_EQ(first.sources.size(), kMaxSelectionSources);
    EXPECT_EQ(first.declared_source_count, kMaxSelectionSources + 4);
    // Selected, but its name did not fit: still says something drives.
    EXPECT_EQ(first.selected_source(), "#" + std::to_string(last));
    for (std::size_t i = 1; i < node.selections.size(); ++i) {
      EXPECT_EQ(node.selections[i].selected_source(), std::nullopt);
    }
  }

  // The slot is free again, released long enough ago to be the next claim's
  // (claim): a node without Targets must not inherit the previous owner's.
  {
    RegistryMapping mapping{path_};
    mapping.header()->nodes[crowded].heartbeat_monotonic_ns.store(
        monotonic_now_ns() -
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                kLivenessTimeout)
                .count() -
            1,
        std::memory_order_relaxed);
  }
  NodeRegistration next{{.name = "plain"}, path_};
  ASSERT_EQ(next.slot(), crowded);
  next.publish({});
  auto reader = RegistryReader::open(path_);
  if (!reader.has_value()) {
    FAIL() << "Expected registry reader";
  }
  const auto& registry = reader.value();
  const auto nodes = registry.nodes();
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_TRUE(nodes.front().selections.empty());
  EXPECT_EQ(nodes.front().declared_selection_count, 0u);
}

TEST_F(Registry, ReclaimedSlotRejectsUpdatesFromItsPreviousOwner) {
  auto old = std::make_unique<NodeRegistration>(
      NodeRegistration::Identity{.name = "old"}, path_);
  old->publish(sample_manifest());
  old->publish_selections(std::vector<Policy>{{{"teleop", "auto"}}});
  RegistryMapping mapping{path_};
  NodeRecord& slot = mapping.header()->nodes[old->slot()];
  const auto timeout =
      std::chrono::duration_cast<std::chrono::nanoseconds>(kLivenessTimeout)
          .count();
  slot.heartbeat_monotonic_ns.store(monotonic_now_ns() - timeout - 1,
                                    std::memory_order_relaxed);

  NodeRegistration next{{.name = "new"}, path_};
  ASSERT_EQ(next.slot(), old->slot());
  next.publish(sample_manifest());
  next.publish_selections(std::vector<Policy>{{{"teleop", "auto"}}});
  next.set_dispatch_count(42);
  next.set_source(0, 1, 2, 3, 4, 5, 6);
  next.set_selected(0, 1);
  const auto heartbeat =
      slot.heartbeat_monotonic_ns.load(std::memory_order_relaxed);
  const auto sequence = slot.sample_seq.load(std::memory_order_relaxed);

  old->begin_sample();
  EXPECT_EQ(slot.sample_seq.load(std::memory_order_relaxed), sequence);
  old->heartbeat();
  old->set_dispatch_count(999);
  old->set_source(0, 999, 999, 999, 999, 999, 999);
  old->set_selected(0, 0);
  old->end_sample();
  old.reset();  // Destruction must not free the reclaimed slot either.

  EXPECT_EQ(slot.sample_seq.load(std::memory_order_relaxed), sequence);
  EXPECT_EQ(slot.heartbeat_monotonic_ns.load(std::memory_order_relaxed),
            heartbeat);
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.front().name, "new");
  EXPECT_EQ(nodes.front().dispatch_count, 42u);
  ASSERT_EQ(nodes.front().sources.size(), 3u);
  const auto& source = nodes.front().sources.front();
  EXPECT_EQ(source.events, 1u);
  EXPECT_EQ(source.dropped, 2u);
  EXPECT_EQ(source.sequence, 3u);
  EXPECT_EQ(source.last_monotonic_ns, 4);
  EXPECT_EQ(source.last_latency_ns, 5);
  EXPECT_EQ(source.max_latency_ns, 6);
  ASSERT_EQ(nodes.front().selections.size(), 1u);
  EXPECT_EQ(nodes.front().selections.front().selected, 1);
}

TEST_F(Registry, ReaderReportsNothingBeforeAnyNodeStarts) {
  EXPECT_FALSE(RegistryReader::open(temp_registry("none")).has_value());
}

TEST_F(Registry, ANewClaimSweepsSlotsLeftBehindByACrashedNode) {
  // A node that died without releasing (SIGKILL, crash) leaves an ACTIVE
  // slot whose heartbeat ages out. The next session's first claim must sweep
  // it, or the viewer reports it as "stopped heartbeating" forever.
  auto ghost = std::make_unique<NodeRegistration>(
      NodeRegistration::Identity{.name = "intake", .session_id = 777}, path_);
  ghost->publish(sample_manifest());

  // Age the ghost past the liveness timeout without releasing its slot.
  const std::int64_t timeout_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(kLivenessTimeout)
          .count();
  RegistryMapping mapping{path_};
  mapping.header()->nodes[ghost->slot()].heartbeat_monotonic_ns.store(
      monotonic_now_ns() - timeout_ns - 1, std::memory_order_relaxed);

  {
    auto stale = RegistryReader::open(path_);
    if (!stale.has_value()) {
      FAIL() << "Expected registry reader";
    }
    const auto& registry = stale.value();
    ASSERT_EQ(registry.nodes().size(), 1u);
    EXPECT_FALSE(registry.nodes().front().alive);
  }

  NodeRegistration newcomer({.name = "intake", .session_id = 1788}, path_);
  newcomer.publish(sample_manifest());

  auto view = RegistryReader::open(path_);
  if (!view.has_value()) {
    FAIL() << "Expected registry reader";
  }
  const auto& registry = view.value();
  const auto nodes = registry.nodes();
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.front().session_id, 1788u);
  EXPECT_TRUE(nodes.front().alive);

  // Destroying the swept owner must not yank the new owner's row.
  ghost.reset();
  const auto after = registry.nodes();
  ASSERT_EQ(after.size(), 1u);
  EXPECT_EQ(after.front().session_id, 1788u);
}

// A killed node's row is freed by the next claim, not once its heartbeat is
// kLivenessTimeout old: a crash-looping roster leaves one row per run, and
// those rows must not fill the registry for the restarts that follow. Until
// then the row reads not alive at once, though its heartbeat is fresh: Studio
// must not show a crashed node still driven by its last source.
// REQ-OPS-17
TEST_F(Registry, ADeadClaimantsRowIsFreedByTheNextClaim) {
  Pipe ready;
  Children children;
  const pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    try {
      NodeRegistration registration{{.name = "killed"}, path_};
      const char token = 1;
      if (!write_all(ready.writer(), &token, 1)) ::_exit(2);
      for (;;) ::pause();
    } catch (...) {
      ::_exit(1);
    }
  }
  children.add(pid);
  char token = 0;
  ASSERT_TRUE(read_all(ready.reader(), &token, 1));
  ASSERT_EQ(::kill(pid, SIGKILL), 0);
  children.reap(pid);
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  ASSERT_EQ(reader->nodes().size(), 1u);
  EXPECT_FALSE(reader->nodes().front().alive)
      << "its process is gone, though its heartbeat is fresh";

  std::vector<std::unique_ptr<NodeRegistration>> registrations;
  registrations.push_back(std::make_unique<NodeRegistration>(
      NodeRegistration::Identity{.name = "node0"}, path_));
  {
    const auto after_first = reader->nodes();
    ASSERT_EQ(after_first.size(), 1u);
    EXPECT_EQ(after_first.front().name, "node0");
    EXPECT_TRUE(after_first.front().alive);
  }
  for (std::size_t i = 1; i < kMaxNodes; ++i) {
    registrations.push_back(std::make_unique<NodeRegistration>(
        NodeRegistration::Identity{.name = "node" + std::to_string(i)}, path_));
  }
  const auto nodes = reader->nodes();
  EXPECT_EQ(nodes.size(), kMaxNodes);
  EXPECT_TRUE(std::none_of(nodes.begin(), nodes.end(),
                           [](const auto& n) { return n.name == "killed"; }));
}

// A node in another pid namespace that shares /dev/shm (a container run with
// --ipc=host) has a pid that names another process, or none, in this one. Its
// row must survive this namespace's sweep while its heartbeat is fresh, or a
// live node loses its row to every claim on the host. The node runs as pid 1
// of a new pid namespace with its own /proc, as in a container; where this
// process may not create one, the test is skipped.
TEST_F(Registry, AClaimantInAnotherPidNamespaceKeepsItsRow) {
#if !defined(__linux__)
  GTEST_SKIP() << "pid namespaces are Linux-only";
#else
  Pipe forked;
  Pipe ready;
  Children children;
  const pid_t outer = ::fork();
  ASSERT_GE(outer, 0);
  if (outer == 0) {
    // Unprivileged where user namespaces allow it, else as root.
    if (::unshare(CLONE_NEWUSER | CLONE_NEWPID | CLONE_NEWNS) != 0 &&
        ::unshare(CLONE_NEWPID | CLONE_NEWNS) != 0) {
      const std::int32_t error = -errno;
      ::_exit(write_all(forked.writer(), &error, sizeof(error)) ? 0 : 2);
    }
    const pid_t node = ::fork();
    if (node == 0) {
      ::prctl(PR_SET_PDEATHSIG, SIGKILL);
      // Best effort: without its own /proc the node cannot check pids, but
      // its row is still recorded from the new namespace.
      if (::mount(nullptr, "/", nullptr, MS_REC | MS_PRIVATE, nullptr) == 0) {
        ::mount("proc", "/proc", "proc", MS_NOSUID | MS_NODEV | MS_NOEXEC,
                nullptr);
      }
      char token = 0;
      try {
        NodeRegistration registration{{.name = "contained"}, path_};
        token = 1;
        if (!write_all(ready.writer(), &token, 1)) ::_exit(2);
        for (;;) ::pause();
      } catch (...) {
        write_all(ready.writer(), &token, 1);
        ::_exit(1);
      }
    }
    const std::int32_t sent = node;
    if (node < 0 || !write_all(forked.writer(), &sent, sizeof(sent))) {
      ::_exit(2);
    }
    int status = 0;
    while (::waitpid(node, &status, 0) < 0 && errno == EINTR) {
    }
    ::_exit(0);
  }
  children.add(outer);
  std::int32_t node = 0;
  ASSERT_TRUE(read_all(forked.reader(), &node, sizeof(node)));
  if (node < 0) {
    children.reap(outer);
    GTEST_SKIP() << "cannot create a pid namespace: errno " << -node;
  }
  char token = 0;
  const bool registered = read_all(ready.reader(), &token, 1) && token == 1;
  if (!registered) ::kill(node, SIGKILL);
  ASSERT_TRUE(registered) << "the contained node did not register";

  // This claim sweeps the registry from this namespace.
  NodeRegistration host{{.name = "host"}, path_};
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  ::kill(node, SIGKILL);
  EXPECT_EQ(nodes.size(), 2u);
  EXPECT_TRUE(std::any_of(nodes.begin(), nodes.end(), [](const auto& n) {
    return n.name == "contained" && n.alive;
  })) << "the live contained node's row was swept";
#endif
}

// The launcher prepares the registry before it starts a node. Created under a
// service's restrictive umask, the segment and its writer lock file still get
// the modes another user's nodes and tools need, and a node registers in it.
TEST_F(Registry, PreparedRegistryIsOpenableWhateverTheUmask) {
  const std::string mapped = ShmObjectName(path_);
  const std::string lock = LockFilePath(mapped);
  ::unlink(lock.c_str());  // An earlier test's file has its mode already.
  const mode_t previous = ::umask(077);
  PrepareRegistry(path_);
  ::umask(previous);

  const int fd = ::shm_open(mapped.c_str(), O_RDONLY, 0);
  ASSERT_GE(fd, 0);
  struct stat segment {};
  ASSERT_EQ(::fstat(fd, &segment), 0);
  ::close(fd);
  EXPECT_EQ(segment.st_mode & 0777, kRegistryMode);
  struct stat file {};
  ASSERT_EQ(::stat(lock.c_str(), &file), 0);
  EXPECT_EQ(file.st_mode & 0777, kLockFileMode);

  NodeRegistration registration{{.name = "node"}, path_};
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  EXPECT_EQ(reader->nodes().size(), 1u);
}

// Raised by the waiter below once it has opened the lock file, before it
// locks it; a hook is a plain function.
std::atomic<bool> lock_file_opened{false};

// A writer that wins the lock on a file a /tmp cleaner unlinked while it
// waited takes the lock again on the file the path now names, so a writer
// opening the path anew is still excluded. A Bazel test's lock files live in
// its TEST_TMPDIR, which Bazel removes, rather than accumulating in /dev/shm.
TEST_F(Registry, AWriterNeverHoldsAnUnlinkedLockFile) {
  const std::string mapped = ShmObjectName(path_);
  const std::string lock = LockFilePath(mapped);
  const char* test_tmpdir = std::getenv("TEST_TMPDIR");
  ASSERT_NE(test_tmpdir, nullptr);
  EXPECT_TRUE(lock.starts_with(std::string{test_tmpdir} + "/")) << lock;
  int holder = OpenLockFile(lock, O_RDONLY);
  ASSERT_GE(holder, 0);
  ASSERT_EQ(::flock(holder, LOCK_EX), 0);
  lock_file_opened = false;
  const RegistryTestHooks hooks{.lock_file_opened = [] {
    lock_file_opened = true;
    lock_file_opened.notify_all();
  }};
  std::atomic<bool> locked{false};
  std::atomic<bool> release{false};
  std::jthread waiter{[&] {
    const RegistryLock writer{mapped, std::nullopt, &hooks};
    locked = true;
    locked.notify_all();
    // Released once the main thread has tried the path's file.
    release.wait(false);
  }};
  // Destroyed before `waiter` joins, so a failed ASSERT below unblocks the
  // waiter (its lock freed, its hold released) instead of hanging the test.
  struct Unblock {
    int& holder;
    std::atomic<bool>& release;
    ~Unblock() {
      if (holder >= 0) ::close(holder);
      release = true;
      release.notify_all();
    }
  } unblock{holder, release};
  // The waiter holds a descriptor of `holder`'s file from here, so whether
  // its flock already blocks or starts after the close below, it wins that
  // file only once the path names another one.
  lock_file_opened.wait(false);
  ASSERT_EQ(::unlink(lock.c_str()), 0);
  ::close(holder);
  // Read by `unblock`'s destructor, which the analyzer cannot see.
  holder = -1;  // NOLINT(clang-analyzer-deadcode.DeadStores)
  locked.wait(false);
  const int newcomer = OpenLockFile(lock, O_RDONLY);
  ASSERT_GE(newcomer, 0);
  EXPECT_NE(::flock(newcomer, LOCK_EX | LOCK_NB), 0);
  ::close(newcomer);
}

// A registry this user cannot write -- another user's, created 0644 -- would
// leave every node unregistered and killed; preparing it refuses, naming it.
TEST_F(Registry, AnUnwritableRegistryRefusesPreparation) {
  if (::geteuid() == 0) GTEST_SKIP() << "root writes any segment";
  const std::string mapped = ShmObjectName(path_);
  const int fd = ::shm_open(mapped.c_str(), O_CREAT | O_EXCL | O_RDWR, 0444);
  ASSERT_GE(fd, 0);
  ::close(fd);
  try {
    PrepareRegistry(path_);
    ADD_FAILURE() << "an unwritable registry was accepted";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string{e.what()}.find(mapped), std::string::npos)
        << e.what();
  }
}

TEST_F(Registry, HeartbeatAndSweepUseHostMonotonicTime) {
  const auto before = monotonic_now_ns();
  NodeRegistration healthy{{.name = "healthy"}, path_};
  healthy.heartbeat();
  const auto after = monotonic_now_ns();
  RegistryMapping mapping{path_};
  NodeRecord& row = mapping.header()->nodes[healthy.slot()];
  const auto beat = row.heartbeat_monotonic_ns.load(std::memory_order_relaxed);
  EXPECT_GE(beat, before);
  EXPECT_LE(beat, after);

  // Human-facing wall timestamps may be arbitrarily far in either direction.
  // A claim sweeps existing rows; neither that sweep nor reader liveness may
  // mistake the monotonic heartbeat for a wall-clock timestamp.
  for (const auto wall : {int64_t{0}, int64_t{8'000'000'000'000'000'000}}) {
    row.start_wall_ns = wall;
    NodeRegistration newcomer{{.name = "newcomer"}, path_};
    EXPECT_NE(newcomer.slot(), healthy.slot());
    auto reader = RegistryReader::open(path_);
    ASSERT_TRUE(reader.has_value());
    const auto nodes = reader->nodes();
    const auto found =
        std::find_if(nodes.begin(), nodes.end(),
                     [](const auto& n) { return n.name == "healthy"; });
    ASSERT_NE(found, nodes.end());
    EXPECT_TRUE(found->alive);
    EXPECT_EQ(found->heartbeat_monotonic_ns, beat);
  }
}

TEST_F(Registry, SourcesBeyondCapacityAreCountedButNotStored) {
  event::Manifest manifest;
  for (std::size_t i = 0; i < kMaxSourcesPerNode + 5; ++i) {
    manifest.push_back({.id = static_cast<std::uint16_t>(i),
                        .kind = event::SourceKind::TIMER,
                        .name = "t" + std::to_string(i)});
  }

  NodeRegistration registration{{.name = "busy"}, path_};
  registration.publish(manifest);

  auto reader = RegistryReader::open(path_);
  if (!reader.has_value()) {
    FAIL() << "Expected registry reader";
  }
  const auto& registry = reader.value();
  const auto nodes = registry.nodes();
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.front().sources.size(), kMaxSourcesPerNode);
  EXPECT_EQ(nodes.front().declared_source_count, manifest.size());
}

// The launcher's stall check reads the row's dispatch count as the loop's
// progress. The loop thread stores it at every dispatch, so a reporting thread
// that cannot run -- starved below the realtime loops of a loaded host; here
// blocked in a selection callback mid-sample -- leaves it current. Before, the
// reporting thread copied it every 10 ms, and a starved one froze the row of
// a loop that kept time until the launcher killed the node as stalled.
// REQ-RT-18
TEST_F(Registry, LoopStoresItsProgressWhileTheReporterIsBlocked) {
  event::RealtimeEventLoop<> loop;
  struct Handler {
    std::function<void()> on_tick;
    void OnTick(const event::Context&) { on_tick(); }
  } handler;
  auto timer = event::make_timer<&Handler::OnTick>(loop, "progress", &handler);
  std::atomic<bool> hold{true};
  std::atomic<bool> held{false};
  Reporter reporter{{.name = "starved",
                     .selections = {{.sources = {"teleop"},
                                     .selected =
                                         [&] {
                                           held = true;
                                           hold.wait(true);
                                           return 0;
                                         }}},
                     .registry_path = path_}};
  reporter.start(loop);
  RegistryReader reader{path_};
  const auto row = [&] {
    const auto nodes = reader.nodes();
    return nodes.size() == 1 ? nodes.front().dispatch_count : 0;
  };
  std::optional<std::uint64_t> at_hold;
  int ticks_held = 0;
  std::uint64_t after = 0;
  std::uint64_t loop_count = 0;
  handler.on_tick = [&] {
    if (!held) return;
    if (!at_hold) {
      at_hold = row();
      return;
    }
    // 100 ticks of 2 ms: longer than a 300 ms pause plus the old cadence
    // needed to reach the launcher's 500 ms floor, at a fraction of the cost.
    if (++ticks_held < 100) return;
    after = row();
    loop_count = loop.dispatch_count();
    hold = false;
    hold.notify_all();
    loop.exit();
  };
  timer.setup_periodic(event::Poller::now() + 1ms, 2ms);
  loop.run_for(5s);  // Bound only; exits once the reporter has been held.
  hold = false;
  hold.notify_all();

  ASSERT_TRUE(held) << reporter.error();
  ASSERT_TRUE(at_hold) << "the loop never ticked with the reporter held";
  EXPECT_GE(after, *at_hold + 100);
  EXPECT_EQ(after, loop_count);
}

// A control loop the OS kept at timeshare must be visible to the launcher and
// session_report, which read only the row: the tier, that it asked, and which
// step the OS refused, beside the event log flags. A row whose node has not
// asked yet reads as unreported, never as a full grant.
// REQ-RT-09
TEST_F(Registry, ARefusedRealtimeGrantIsVisibleInTheRow) {
  NodeRegistration registration{{.name = "lift", .flags = kFlagSimulation},
                                path_};
  registration.publish(sample_manifest());
  registration.set_recording(/*failed=*/false, /*capped=*/true, /*gap=*/false);
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto row = [&reader] {
    const auto nodes = reader->nodes();
    EXPECT_EQ(nodes.size(), 1u);
    return nodes.front();
  };
  EXPECT_EQ(row().realtime(), std::nullopt);

  registration.set_realtime(
      {.tier = process::LoopTier::kControl,
       .requested = true,
       .refused = process::kStepPolicy | process::kStepRingLock});
  const NodeSnapshot node = row();
  ASSERT_TRUE(node.realtime().has_value());
  EXPECT_EQ(node.realtime()->tier, process::LoopTier::kControl);
  EXPECT_EQ(node.realtime()->grant(), process::RealtimeGrant::kRefused);
  EXPECT_EQ(process::RefusedStepNames(node.realtime()->refused),
            "policy, ring mlock");
  EXPECT_NE(node.flags & kFlagSimulation, 0u);
  EXPECT_NE(node.flags & kFlagLogCapped, 0u);
  EXPECT_EQ(node.flags & kFlagRecorderFailed, 0u);
}

// A log with a gap is not a failed one: the launcher reads kFlagLogGap from
// the row to mark the member log_gap, and kFlagRecorderFailed only for a log
// whose capture is over. The bit stays once the gap has closed, since the
// records it marks stay lost.
// REQ-OPS-52
TEST_F(Registry, ALogGapIsItsOwnFlagAndOutlivesTheGap) {
  NodeRegistration registration{{.name = "lift"}, path_};
  registration.publish(sample_manifest());
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto row = [&reader] {
    const auto nodes = reader->nodes();
    EXPECT_EQ(nodes.size(), 1u);
    return nodes.front();
  };
  registration.set_recording(/*failed=*/false, /*capped=*/false,
                             /*gap=*/false);
  EXPECT_FALSE(row().log_gap());

  registration.set_recording(/*failed=*/false, /*capped=*/false,
                             /*gap=*/true);
  registration.set_recording(/*failed=*/false, /*capped=*/false,
                             /*gap=*/false);
  const NodeSnapshot node = row();
  EXPECT_TRUE(node.log_gap());
  EXPECT_EQ(node.flags & (kFlagRecorderFailed | kFlagLogCapped), 0u);
}

// A run that exits right after its row showed a log gap is marked only from
// the row it released, which the launcher reads when it reaps the run. A
// claim made before that reap (a peer starting) must not take that row while
// a row unused or released earlier is free, or the run's last sample, its
// gap bit among it, is gone before the launcher reads it.
// REQ-OPS-52
TEST_F(Registry, AClaimTakesARecentlyReleasedRowOnlyWhenNoOtherIsFree) {
  std::size_t released = 0;
  {
    NodeRegistration ended{{.name = "ended", .session_id = 7}, path_};
    ended.publish(sample_manifest());
    ended.set_recording(/*failed=*/false, /*capped=*/false, /*gap=*/true);
    released = ended.slot();
  }
  NodeRegistration next{{.name = "next", .session_id = 7}, path_};
  next.publish(sample_manifest());
  EXPECT_NE(next.slot(), released);
  RegistryMapping mapping{path_};
  const NodeRecord& row = mapping.header()->nodes[released];
  EXPECT_NE(row.flags.load(std::memory_order_relaxed) & kFlagLogGap, 0u)
      << "the released row's last sample was overwritten";

  // With every other row taken, the released one is reused.
  std::vector<std::unique_ptr<NodeRegistration>> others;
  while (others.size() + 2 < kMaxNodes) {
    others.push_back(std::make_unique<NodeRegistration>(
        NodeRegistration::Identity{.name =
                                       "other" + std::to_string(others.size())},
        path_));
  }
  NodeRegistration last{{.name = "last"}, path_};
  EXPECT_EQ(last.slot(), released);
}

// When rows released in the last kLivenessTimeout are all that is free (a
// restart storm), a claim takes the one released longest ago: a run's row is
// overwritten before its reap only if every other row is held or was
// released after it. Here the higher of two such rows was released first.
// REQ-OPS-52
TEST_F(Registry, AClaimTakesTheRecentRowReleasedLongestAgo) {
  std::vector<std::unique_ptr<NodeRegistration>> rows;
  while (rows.size() < kMaxNodes) {
    rows.push_back(std::make_unique<NodeRegistration>(
        NodeRegistration::Identity{.name = "row" + std::to_string(rows.size())},
        path_));
  }
  const std::size_t later = rows[5]->slot();
  const std::size_t earlier = rows[20]->slot();
  ASSERT_LT(later, earlier);
  rows[20].reset();
  rows[5].reset();
  // Released 2 s and 1 s ago: both younger than kLivenessTimeout.
  RegistryMapping mapping{path_};
  const std::int64_t now = monotonic_now_ns();
  mapping.header()->nodes[earlier].heartbeat_monotonic_ns.store(
      now - 2'000'000'000, std::memory_order_relaxed);
  mapping.header()->nodes[later].heartbeat_monotonic_ns.store(
      now - 1'000'000'000, std::memory_order_relaxed);
  NodeRegistration next{{.name = "next"}, path_};
  EXPECT_EQ(next.slot(), earlier);
}

// The Reporter copies a gap in the loop's own event log into the row, which
// is all the launcher has of a run killed during one: a running loop whose
// LogWriter dropped one record too large for its buffers shows kFlagLogGap.
// REQ-OPS-52
TEST_F(Registry, TheReporterCopiesTheLoopsLogGapIntoTheRow) {
  const std::string log = ::testing::TempDir() + "/registry_gap_" +
                          std::to_string(::getpid()) + ".tlog";
  event::RealtimeEventLoop<event::log::LogWriter> loop{
      event::log::LogWriter{log, "gapped", 512}};
  struct Handler {
    std::function<void()> on_tick;
    void OnTick(const event::Context&) { on_tick(); }
  } handler;
  auto timer = event::make_timer<&Handler::OnTick>(loop, "gap_tick", &handler);
  Reporter reporter{{.name = "gapped", .period = 5ms, .registry_path = path_}};
  reporter.start(loop);
  RegistryReader reader{path_};
  const std::vector<std::byte> oversized(1024, std::byte{1});
  bool dropped = false;
  bool flagged = false;
  handler.on_tick = [&] {
    if (!dropped) {
      event::Context context{};
      context.kind = event::EventKind::TIMER;
      loop.recorder().dispatch(context, oversized);
      dropped = true;
      return;
    }
    const auto nodes = reader.nodes();
    if (nodes.size() == 1 && nodes.front().log_gap()) {
      flagged = true;
      loop.exit();
    }
  };
  timer.setup_periodic(event::Poller::now() + 1ms, 2ms);
  loop.run_for(10s);  // Bound only; exits once the row shows the gap.
  ASSERT_EQ(loop.recorder().gaps(), 1u);
  EXPECT_TRUE(flagged) << "the row never showed the loop's log gap";
  ::unlink(log.c_str());
}

// A notification its writer could not send -- here no descriptor is left for
// the sender's socket -- leaves the reader asleep until its backstop, and the
// row is the only place outside the process that can say why: the reporter
// copies the loop's count there, where the launcher and Studio read it
// (TODO.md RT-7). The descriptor limit is lowered in a child, which owns the
// whole scenario: a loop sending on a topic whose reader, in the same child,
// is armed on a receiver.
// REQ-RT-19, REQ-OPS-18
TEST_F(Registry, AWakeItsWriterCouldNotSendIsCountedInTheRow) {
  const std::string topic = "/waker/state/p" + std::to_string(::getpid());
  Children children;
  const pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    int code = 1;
    try {
      event::RealtimeEventLoop<> loop;
      struct Handler {
        std::function<void()> on_tick;
        void OnTick(const event::Context&) { on_tick(); }
      } handler;
      auto timer = event::make_timer<&Handler::OnTick>(loop, "tick", &handler);
      const std::uint16_t sender = loop.register_sender(
          topic, sizeof(std::uint64_t), alignof(std::uint64_t), kCountSchema);
      RTMSQueue subscriber{topic,
                           sizeof(std::uint64_t),
                           alignof(std::uint64_t),
                           kCountSchema,
                           MAX_SLOTS,
                           {.overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
                            .read_mode = ReadMode::SEQUENCE}};
      const auto reader = subscriber.register_reader();
      WakeReceiver receiver;
      Reporter reporter{
          {.name = "waker", .period = 10ms, .registry_path = path_}};
      RegistryReader rows{path_};
      reporter.start(loop);
      rlimit limit{};
      bool sent = false;
      code = 2;
      handler.on_tick = [&] {
        if (!sent) {
          // The lowest free descriptor: with the soft limit there, the
          // sender's socket cannot open. Raised again at once, so nothing
          // else in the child is refused one.
          const int lowest = ::open("/dev/null", O_RDONLY | O_CLOEXEC);
          if (!reader || lowest < 0 || ::close(lowest) != 0 ||
              ::getrlimit(RLIMIT_NOFILE, &limit) != 0) {
            loop.exit();
            return;
          }
          subscriber.set_reader_wake(*reader, receiver.id());
          const rlimit lowered{.rlim_cur = static_cast<rlim_t>(lowest),
                               .rlim_max = limit.rlim_max};
          const std::uint64_t value = 1;
          sent =
              subscriber.arm_wake(*reader) &&
              ::setrlimit(RLIMIT_NOFILE, &lowered) == 0 &&
              loop.send_message(sender, std::as_bytes(std::span{&value, 1})) &&
              ::setrlimit(RLIMIT_NOFILE, &limit) == 0;
          if (!sent) loop.exit();
          code = 3;
          return;
        }
        const auto nodes = rows.nodes();
        if (nodes.size() == 1 && nodes.front().delivery_loss().any()) {
          const DeliveryLoss loss = nodes.front().delivery_loss();
          code = loss.wake_failures == 1 && loss.stalled_arms == 0 &&
                         loss.wake_fallbacks == 0 && loss.requests_skipped == 0
                     ? 0
                     : 5;
          loop.exit();
        }
      };
      timer.setup_periodic(event::Poller::now() + 1ms, 2ms);
      loop.run_for(5s);  // Bound only; exits once the row shows the count.
    } catch (...) {
      code = 4;
    }
    ::_exit(code);
  }
  children.add(pid);
  const int status = children.reap(pid);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0)
      << "2: setup, 3: the row never showed the failed wake, 4: threw, "
         "5: the row counted something else";
}

TEST_F(Registry, ReporterPublishesARunningLoopWithoutTouchingItsManifest) {
  const std::string topic = "/probe/state/p" + std::to_string(::getpid());
  const std::string feed = "/probe/telemetry/p" + std::to_string(::getpid());
  const std::string watched = "/probe/status/p" + std::to_string(::getpid());
  event::RealtimeEventLoop<> loop;

  struct Handler {
    std::uint64_t fired{0};
    std::function<void()> after_tick;
    void OnTick(const event::Context&) {
      ++fired;
      if (after_tick) after_tick();
    }
  } handler;

  auto timer =
      event::make_timer<&Handler::OnTick>(loop, "introspect_tick", &handler);
  const std::uint16_t sender = loop.register_sender(
      topic, /*message_bytes=*/8, /*alignment=*/8, kCountSchema);
  // No callback, so latest-only: its dropped count includes skips by design.
  const std::uint16_t latest =
      loop.register_watcher(watched, /*message_bytes=*/8, /*alignment=*/8,
                            kCountSchema, event::Thunk{});
  const std::size_t sources_before = loop.manifest().size();
  // What the node's Target last selected: index 1, "auto".
  std::atomic<std::int32_t> selected{1};

  Reporter reporter{
      {.name = "probe",
       .target = "//talOS/introspection:registry_test",
       .session_id = 42,
       .simulation = true,
       // A topic the node owns outside the loop, carrying its
       // attribute inline.
       .extra = {{.kind = event::SourceKind::SENDER,
                  .name = feed,
                  .message_bytes = 64,
                  .flags = naming::kSourceFlagOptional}},
       // The loop's own sender, keyed by name because the loop
       // has no idea that this one's consumer is off-box.
       .endpoints = {{topic, naming::kSourceFlagExternal}},
       .selections = {{.sources = {"teleop", "auto"},
                       .selected = [&selected] { return selected.load(); }}},
       .period = 20ms,
       .registry_path = path_}};
  {
    RegistryReader reader{path_};
    const auto nodes = reader.nodes();
    ASSERT_EQ(nodes.size(), 1u);
    EXPECT_TRUE(nodes.front().flags & kFlagConstructing);
    EXPECT_FALSE(nodes.front().alive);
    EXPECT_EQ(nodes.front().pid, static_cast<std::uint64_t>(::getpid()));
    EXPECT_EQ(nodes.front().session_id, 42u);
    EXPECT_TRUE(nodes.front().sources.empty());
    const auto first_beat = nodes.front().heartbeat_monotonic_ns;
    // The construction heartbeat beats every 10 ms on the reporter's own
    // thread and signals nothing, so the row is polled; the deadline only
    // bounds a hang (docs/TESTING.md §6.3).
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (reader.nodes().front().heartbeat_monotonic_ns == first_beat &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(2ms);
    }
    EXPECT_GT(reader.nodes().front().heartbeat_monotonic_ns, first_beat);
  }
  reporter.start(loop);

  // Read the registry while the loop still runs: once it stops, the reporter
  // releases the row, and a loaded host may get there before the test reads.
  std::optional<std::vector<NodeSnapshot>> live;
  handler.after_tick = [&] {
    if (live || handler.fired < 20 || !reporter.published()) return;
    auto reader = RegistryReader::open(path_);
    if (!reader) return;
    auto nodes = reader->nodes();
    if (nodes.size() == 1 && nodes.front().dispatch_count > 0 &&
        nodes.front().sources.size() == 4 &&
        nodes.front().sources[0].events > 0) {
      live = std::move(nodes);
      loop.exit();
    }
  };
  timer.setup_periodic(event::Poller::now() + 1ms, 2ms);
  loop.run_for(5s);  // Bound only; exits as soon as a sample is visible.

  // Introspection must not appear in the manifest: it would change every
  // source id and make existing logs unreplayable.
  EXPECT_EQ(loop.manifest().size(), sources_before);
  ASSERT_TRUE(reporter.published()) << reporter.error();
  EXPECT_GT(handler.fired, 0u);
  ASSERT_TRUE(live) << "no registry sample while the loop ran";
  const auto& nodes = *live;
  ASSERT_EQ(nodes.size(), 1u);
  const NodeSnapshot& node = nodes.front();

  EXPECT_EQ(node.name, "probe");
  EXPECT_EQ(node.session_id, 42u);
  EXPECT_TRUE(node.flags & kFlagSimulation);
  EXPECT_TRUE(node.alive);
  EXPECT_GT(node.dispatch_count, 0u);

  ASSERT_EQ(node.sources.size(), 4u);
  EXPECT_EQ(node.sources[0].kind, event::SourceKind::TIMER);
  EXPECT_EQ(node.sources[0].name, "introspect_tick");
  EXPECT_GT(node.sources[0].events, 0u);
  // A timer has no far end, so it carries no attributes.
  EXPECT_EQ(node.sources[0].flags, 0u);

  EXPECT_EQ(node.sources[1].kind, event::SourceKind::SENDER);
  EXPECT_EQ(node.sources[1].name, topic);
  EXPECT_EQ(node.sources[1].id, sender);
  // Nothing was published on it, so it must read as idle rather than as
  // traffic that never happened -- and it must read as deliberately
  // unsubscribed rather than as a topic whose subscriber is missing.
  EXPECT_EQ(node.sources[1].events, 0u);
  EXPECT_TRUE(node.sources[1].external());
  EXPECT_FALSE(node.sources[1].optional());
  EXPECT_FALSE(node.sources[1].latest_only());

  // Only the loop knows which watchers skip to the newest message; the
  // reporter publishes it so a viewer does not call those skips loss.
  EXPECT_EQ(node.sources[2].id, latest);
  EXPECT_EQ(node.sources[2].flags, naming::kSourceFlagLatestOnly);

  EXPECT_EQ(node.sources[3].name, feed);
  EXPECT_TRUE(node.sources[3].optional());
  EXPECT_FALSE(node.sources[3].external());

  ASSERT_EQ(node.selections.size(), 1u);
  EXPECT_EQ(node.selections[0].sources,
            (std::vector<std::string>{"teleop", "auto"}));
  EXPECT_EQ(node.selections[0].selected_source(), "auto");
}

// While its loop runs, a node's reporting thread wakes once per refresh and
// for nothing else: it used to wake every 10 ms to look for the loop's end,
// 100 wakeups/s in every process, which a host running many nodes pays for.
// Each refresh still moves the heartbeat, which keeps the row alive
// (kLivenessTimeout). The loop runs until the heartbeat has moved twice and
// half a second has passed since the row was published.
// REQ-RT-21
TEST_F(Registry, ARunningReporterWakesOnlyToRefresh) {
  constexpr auto kPeriod = 50ms;
  event::RealtimeEventLoop<> loop;
  struct Handler {
    std::function<void()> on_tick;
    void OnTick(const event::Context&) { on_tick(); }
  } handler;
  auto timer = event::make_timer<&Handler::OnTick>(loop, "tick", &handler);
  Reporter reporter{
      {.name = "quiet", .period = kPeriod, .registry_path = path_}};
  reporter.start(loop);
  RegistryReader reader{path_};
  const auto heartbeat = [&] {
    const auto nodes = reader.nodes();
    return nodes.size() == 1 ? nodes.front().heartbeat_monotonic_ns : 0;
  };
  std::optional<std::chrono::steady_clock::time_point> since;
  std::uint64_t wakeups_before = 0;
  std::uint64_t wakeups = 0;
  std::int64_t last_beat = 0;
  int beats = 0;
  std::chrono::steady_clock::duration window{};
  handler.on_tick = [&] {
    if (!reporter.published()) return;
    const auto now = std::chrono::steady_clock::now();
    if (!since) {
      since = now;
      wakeups_before = reporter.wakeups();
      last_beat = heartbeat();
      return;
    }
    if (const std::int64_t beat = heartbeat(); beat != last_beat) {
      last_beat = beat;
      ++beats;
    }
    if (beats < 2 || now - *since < 500ms) return;
    wakeups = reporter.wakeups() - wakeups_before;
    window = now - *since;
    loop.exit();
  };
  timer.setup_periodic(event::Poller::now() + 1ms, 5ms);
  loop.run_for(10s);  // Bound only; exits after two refreshes.

  ASSERT_GE(beats, 2) << "the heartbeat stopped moving while the loop ran";
  // One wake per period, and one more at each end of the window.
  const auto allowed = static_cast<std::uint64_t>(window / kPeriod) + 2;
  EXPECT_LE(wakeups, allowed)
      << "over "
      << std::chrono::duration_cast<std::chrono::milliseconds>(window).count()
      << " ms at a " << kPeriod.count() << " ms refresh";
}

// talos::Run stops its reporter as soon as the loop returns; the stop must
// not wait out the refresh in progress, here 30 s long.
TEST_F(Registry, StoppingAReporterEndsItsRefreshWaitAtOnce) {
  event::RealtimeEventLoop<> loop;
  struct Handler {
    std::function<void()> on_tick;
    void OnTick(const event::Context&) { on_tick(); }
  } handler;
  auto timer = event::make_timer<&Handler::OnTick>(loop, "tick", &handler);
  Reporter reporter{{.name = "patient", .period = 30s, .registry_path = path_}};
  reporter.start(loop);
  handler.on_tick = [&] {
    if (reporter.published()) loop.exit();
  };
  timer.setup_periodic(event::Poller::now() + 1ms, 1ms);
  loop.run_for(10s);  // Bound only; exits once the row is published.
  ASSERT_TRUE(reporter.published()) << reporter.error();

  const auto before = std::chrono::steady_clock::now();
  reporter.stop();
  EXPECT_LT(std::chrono::steady_clock::now() - before, 10s)
      << "stop() waited for the 30 s refresh";
}

TEST_F(Registry, LoopCountsPublishesAndTimerDispatchesPerSource) {
  const std::string topic = "/tc_" + std::to_string(::getpid());
  event::RealtimeEventLoop<> loop;

  struct Publisher {
    event::RealtimeEventLoop<>* loop{nullptr};
    std::uint16_t sender{0};
    std::uint64_t sent{0};
    void OnTick(const event::Context&) {
      if (loop->send_message(sender, std::as_bytes(std::span{&sent, 1}))) {
        ++sent;
      }
    }
  } publisher;

  publisher.loop = &loop;
  publisher.sender = loop.register_sender(topic, sizeof(std::uint64_t),
                                          alignof(std::uint64_t), kCountSchema);
  auto timer = event::make_timer<&Publisher::OnTick>(loop, "pub", &publisher);
  timer.setup_periodic(event::Poller::now() + 1ms, 2ms);
  loop.run_for(120ms);

  ASSERT_EQ(loop.source_counter_count(), 2u);
  const event::SourceCounters* counters = loop.source_counters();
  ASSERT_NE(counters, nullptr);

  // Source 0 is the sender, 1 the timer: registration order, which is also
  // the order the manifest and the log use.
  EXPECT_GT(publisher.sent, 0u);
  EXPECT_EQ(counters[0].events.load(), publisher.sent);

  // Every tick published once, so the timer fired exactly as often as the
  // sender wrote. The loop's own dispatch count is one higher because it also
  // records the run's EXIT.
  EXPECT_EQ(counters[1].events.load(), publisher.sent);
  EXPECT_GE(loop.dispatch_count(), counters[1].events.load());
  EXPECT_GE(counters[1].last_latency_ns.load(), 0);
}

// The point of the flags word: a reader gets back exactly what the node
// declared, so a graph tool can tell a designed dead end from a fault without
// asking the node again.
TEST_F(Registry, EndpointAttributesSurviveTheSegment) {
  event::Manifest manifest;
  manifest.push_back({.id = 0,
                      .kind = event::SourceKind::SENDER,
                      .name = "/hw/command",
                      .message_bytes = 16,
                      .alignment = 8});
  manifest.push_back({.id = 1,
                      .kind = event::SourceKind::WATCHER,
                      .name = "/drivetrain/target/auto",
                      .message_bytes = 24,
                      .alignment = 8});
  manifest.push_back({.id = 2,
                      .kind = event::SourceKind::SENDER,
                      .name = "/autonomous/request/shooter",
                      .message_bytes = 32,
                      .alignment = 8});
  manifest.push_back({.id = 3,
                      .kind = event::SourceKind::SENDER,
                      .name = "/autonomous/state",
                      .message_bytes = 8,
                      .alignment = 8});

  const std::vector<EndpointAttribute> endpoints = {
      // Consumed by the RoboRIO over UDP: no shared-memory subscriber, ever.
      {"/hw/command", naming::kSourceFlagExternal},
      // Autonomous is not written yet, so nothing publishes this.
      {"/drivetrain/target/auto", naming::kSourceFlagOptional},
      // Two entries for one topic, which is how a node says both things
      // without having to OR the constants at the call site.
      {"/autonomous/request/shooter", naming::kSourceFlagExternal},
      {"/autonomous/request/shooter", naming::kSourceFlagOptional},
  };

  NodeRegistration registration{{.name = "autonomous"}, path_};
  registration.publish(manifest, endpoints);

  auto reader = RegistryReader::open(path_);
  if (!reader.has_value()) {
    FAIL() << "Expected registry reader";
  }
  const auto& registry = reader.value();
  const auto nodes = registry.nodes();
  ASSERT_EQ(nodes.size(), 1u);
  const auto& sources = nodes.front().sources;
  ASSERT_EQ(sources.size(), 4u);

  EXPECT_EQ(sources[0].name, "/hw/command");
  EXPECT_TRUE(sources[0].external());
  EXPECT_FALSE(sources[0].optional());

  EXPECT_EQ(sources[1].name, "/drivetrain/target/auto");
  EXPECT_FALSE(sources[1].external());
  EXPECT_TRUE(sources[1].optional());

  EXPECT_EQ(sources[2].name, "/autonomous/request/shooter");
  EXPECT_TRUE(sources[2].external());
  EXPECT_TRUE(sources[2].optional());

  // Declared nothing, so it reads as an ordinary topic: the attribute is
  // keyed by name and must not leak to its neighbours.
  EXPECT_EQ(sources[3].name, "/autonomous/state");
  EXPECT_EQ(sources[3].flags, 0u);

  // The descriptive fields the flags word sits between are unaffected, which
  // is the claim that let the version stay at 1.
  EXPECT_EQ(sources[0].message_bytes, 16u);
  EXPECT_EQ(sources[0].alignment, 8u);
  EXPECT_EQ(sources[1].kind, event::SourceKind::WATCHER);
}

// A misspelled topic is reported and then published anyway. Taking the node off
// the robot to punish a name would be a worse failure than the name, and the
// registry showing the spelling the node actually used is the whole point: that
// is how the two ends are found not to meet.
TEST_F(Registry, AMisnamedTopicIsStillPublished) {
  event::Manifest manifest;
  manifest.push_back({.id = 0,
                      .kind = event::SourceKind::TIMER,
                      .name = "tick",
                      .period_ns = 5'000'000});
  manifest.push_back({.id = 1,
                      .kind = event::SourceKind::SENDER,
                      .name = "/hw/cmd",
                      .message_bytes = 8,
                      .alignment = 8});

  NodeRegistration registration{{.name = "hardware_bridge"}, path_};

  registration.publish(manifest);
  auto reader = RegistryReader::open(path_);
  if (!reader.has_value()) {
    FAIL() << "Expected registry reader";
  }
  const auto& registry = reader.value();
  const auto nodes = registry.nodes();
  ASSERT_EQ(nodes.size(), 1u);
  ASSERT_EQ(nodes.front().sources.size(), 2u);
  EXPECT_EQ(nodes.front().sources[1].name, "/hw/cmd");
}

// A sample parked open -- the heartbeat advanced but the sample never closed
// -- must not wedge or drop the reader. The bounded retries give up and return
// the plain copy an older reader would have taken; once the sample closes,
// the whole new sample is visible at once.
TEST_F(Registry, ReaderMakesProgressWhileASampleIsOpen) {
  NodeRegistration registration{{.name = "parked"}, path_};
  registration.publish(sample_manifest());
  registration.set_dispatch_count(7);
  registration.set_source(2, /*events=*/7, /*dropped=*/0, /*sequence=*/7,
                          /*last_monotonic_ns=*/0, /*last_latency_ns=*/0,
                          /*max_latency_ns=*/0);

  auto reader = RegistryReader::open(path_);
  if (!reader.has_value()) {
    FAIL() << "Expected registry reader";
  }
  const auto& registry = reader.value();

  registration.begin_sample();
  registration.heartbeat();

  // The sequence stays odd for as long as the sample stays open; the reader
  // still terminates and still reports the node.
  const auto mid = registry.nodes();
  ASSERT_EQ(mid.size(), 1u);
  EXPECT_EQ(mid.front().name, "parked");

  registration.set_dispatch_count(8);
  registration.set_source(2, /*events=*/8, /*dropped=*/0, /*sequence=*/8,
                          /*last_monotonic_ns=*/0, /*last_latency_ns=*/0,
                          /*max_latency_ns=*/0);
  registration.end_sample();

  const auto after = registry.nodes();
  ASSERT_EQ(after.size(), 1u);
  EXPECT_EQ(after.front().dispatch_count, 8u);
  ASSERT_EQ(after.front().sources.size(), 3u);
  EXPECT_EQ(after.front().sources[2].events, 8u);
}

// A writer that predates the guard never touches the sequence word, which
// reads back as the zero its segment was created with: undisturbed zero is
// even, so old writers unconditionally read as stable.
TEST_F(Registry, WriterWithoutTheGuardReadsAsStable) {
  NodeRegistration registration{{.name = "legacy"}, path_};
  registration.publish(sample_manifest());

  // Bypass the guard exactly as an older build does: store the counters and
  // the heartbeat straight into the slot, leaving the sequence word alone.
  RegistryMapping mapping{path_};
  NodeRecord& record = mapping.header()->nodes[registration.slot()];
  record.dispatch_count.store(41, std::memory_order_relaxed);
  record.sources[2].events.store(41, std::memory_order_relaxed);
  record.heartbeat_monotonic_ns.store(monotonic_now_ns(),
                                      std::memory_order_relaxed);
  EXPECT_EQ(record.sample_seq.load(std::memory_order_relaxed), 0u);

  auto reader = RegistryReader::open(path_);
  if (!reader.has_value()) {
    FAIL() << "Expected registry reader";
  }
  const auto& registry = reader.value();
  const auto nodes = registry.nodes();
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.front().dispatch_count, 41u);
  ASSERT_EQ(nodes.front().sources.size(), 3u);
  EXPECT_EQ(nodes.front().sources[2].events, 41u);
  EXPECT_TRUE(nodes.front().alive);
}

// REQ-OPS-17 / REQ-RT-18: held_rows() reads each active row through the same
// generation-verified seqlock copy as nodes(), so concurrent slot recycling or
// sample updates cannot yield torn snapshots and the snapshot carries its slot
// generation.
TEST_F(Registry, HeldRowsUsesGenerationVerifiedCopy) {
  NodeRegistration registration{{.name = "held_node"}, path_};
  registration.publish(sample_manifest());
  registration.begin_sample();
  registration.set_dispatch_count(12);
  registration.set_source(2, /*events=*/12, /*dropped=*/0, /*sequence=*/12,
                          /*last_monotonic_ns=*/0, /*last_latency_ns=*/0,
                          /*max_latency_ns=*/0);
  registration.heartbeat();
  registration.end_sample();

  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  const auto held = reader->held_rows();
  ASSERT_EQ(nodes.size(), 1u);
  ASSERT_EQ(held.size(), 1u);
  EXPECT_GT(nodes.front().generation, 0u);
  EXPECT_EQ(held.front().generation, nodes.front().generation);
  EXPECT_EQ(held.front().name, "held_node");
  EXPECT_EQ(held.front().dispatch_count, 12u);
  EXPECT_TRUE(held.front().alive);

  // A sample parked open still terminates via copy_stable's bounded fallback.
  registration.begin_sample();
  const auto mid = reader->held_rows();
  ASSERT_EQ(mid.size(), 1u);
  EXPECT_EQ(mid.front().generation, nodes.front().generation);
  registration.end_sample();

  // A stale heartbeat is no longer held.
  RegistryMapping mapping{path_};
  mapping.header()->nodes[registration.slot()].heartbeat_monotonic_ns.store(
      monotonic_now_ns() - std::chrono::duration_cast<std::chrono::nanoseconds>(
                               kLivenessTimeout + 1s)
                               .count(),
      std::memory_order_relaxed);
  EXPECT_TRUE(reader->held_rows().empty());
}

// REQ-RT-18: readers take no lock (open, nodes, and held_rows all complete
// while RegistryLock is held on another thread/fd), and a writer updating and
// sampling its own slot leaves every byte of another node's slot untouched.
TEST_F(Registry, ReadersTakeNoLockAndWritersTouchOnlyTheirOwnSlot) {
  NodeRegistration writer_a{{.name = "writer_a"}, path_};
  NodeRegistration writer_b{{.name = "writer_b"}, path_};
  ASSERT_NE(writer_a.slot(), writer_b.slot());
  writer_a.publish(sample_manifest());
  writer_b.publish(sample_manifest());
  writer_b.publish_selections(std::vector<Policy>{{{"teleop", "auto"}}});

  RegistryMapping mapping{path_, /*read_only=*/true};
  const auto snapshot_slot_bytes = [&](std::size_t slot) {
    const auto* bytes =
        reinterpret_cast<const std::byte*>(&mapping.header()->nodes[slot]);
    return std::vector<std::byte>(bytes, bytes + sizeof(NodeRecord));
  };
  const std::vector<std::byte> slot_b_before =
      snapshot_slot_bytes(writer_b.slot());

  const std::string mapped = ShmObjectName(path_);
  std::atomic<bool> locked{false};
  std::atomic<bool> release{false};
  std::jthread holder{[&] {
    const RegistryLock lock{mapped};
    locked = true;
    locked.notify_all();
    release.wait(false);
  }};
  struct Unblock {
    std::atomic<bool>& release;
    ~Unblock() {
      release = true;
      release.notify_all();
    }
  } unblock{release};
  locked.wait(false);

  // Confirm the writer lock is actually held on another fd before reading.
  EXPECT_THROW((RegistryLock{mapped, 0ms}), std::runtime_error);

  // Writing and sampling slot A takes no lock and leaves slot B byte-identical.
  writer_a.publish_selections(std::vector<Policy>{{{"teleop", "auto"}}});
  writer_a.begin_sample();
  writer_a.heartbeat();
  writer_a.set_dispatch_count(123);
  writer_a.set_source(0, /*events=*/10, /*dropped=*/1, /*sequence=*/20,
                      /*last_monotonic_ns=*/30, /*last_latency_ns=*/40,
                      /*max_latency_ns=*/50);
  writer_a.set_selected(0, 1);
  writer_a.set_wake_loss(1, 2, 3);
  writer_a.set_recording(/*failed=*/false, /*capped=*/true, /*gap=*/true);
  writer_a.set_outputs_held(true);
  writer_a.set_realtime(
      {.tier = process::LoopTier::kControl, .requested = true});
  writer_a.end_sample();

  // Readers must return without blocking while RegistryLock is held.
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  const auto held = reader->held_rows();
  ASSERT_EQ(nodes.size(), 2u);
  ASSERT_EQ(held.size(), 2u);
  const auto find_a = [](const std::vector<NodeSnapshot>& list) {
    return std::find_if(list.begin(), list.end(),
                        [](const auto& n) { return n.name == "writer_a"; });
  };
  const auto node_a = find_a(nodes);
  ASSERT_NE(node_a, nodes.end());
  EXPECT_EQ(node_a->dispatch_count, 123u);
  ASSERT_FALSE(node_a->sources.empty());
  EXPECT_EQ(node_a->sources[0].events, 10u);
  ASSERT_EQ(node_a->selections.size(), 1u);
  EXPECT_EQ(node_a->selections[0].selected, 1);
  EXPECT_NE(find_a(held), held.end());

  EXPECT_EQ(snapshot_slot_bytes(writer_b.slot()), slot_b_before);
}

// State for the release() TOCTOU hook below; hooks are plain function pointers.
struct ReleaseRaceState {
  std::string path;
  bool lock_held{false};
  std::unique_ptr<NodeRegistration> reclaimed;
}* release_race_state = nullptr;

// REQ-OPS-17: NodeRegistration::release() must hold RegistryLock across
// owns_slot() and the SlotState::FREE store so a concurrent claim cannot sweep
// and reclaim a stale slot in between and then have its live ACTIVE slot
// clobbered back to FREE.
TEST_F(Registry, ReleaseHoldsRegistryLockAgainstConcurrentReclaim) {
  ReleaseRaceState state{.path = path_};
  release_race_state = &state;
  const RegistryTestHooks hooks{.releasing = [](NodeRecord*) {
    const std::string mapped = ShmObjectName(release_race_state->path);
    try {
      const RegistryLock probe{mapped, 0ms};
      release_race_state->lock_held = false;
    } catch (const std::runtime_error&) {
      release_race_state->lock_held = true;
      return;
    }
    // Without RegistryLock held during release(), a concurrent writer sweeps
    // and reclaims the stale slot right before release() stores
    // SlotState::FREE.
    release_race_state->reclaimed = std::make_unique<NodeRegistration>(
        NodeRegistration::Identity{.name = "reclaimed"},
        release_race_state->path);
  }};

  auto old = std::make_unique<NodeRegistration>(
      NodeRegistration::Identity{.name = "stale"}, path_, &hooks);
  const std::size_t slot_index = old->slot();
  {
    RegistryMapping mapping{path_};
    const auto timeout =
        std::chrono::duration_cast<std::chrono::nanoseconds>(kLivenessTimeout)
            .count();
    mapping.header()->nodes[slot_index].heartbeat_monotonic_ns.store(
        monotonic_now_ns() - timeout - 1, std::memory_order_relaxed);
  }
  old.reset();
  release_race_state = nullptr;

  EXPECT_TRUE(state.lock_held) << "release() ran without holding RegistryLock";
  if (!state.reclaimed) {
    state.reclaimed = std::make_unique<NodeRegistration>(
        NodeRegistration::Identity{.name = "reclaimed"}, path_);
  }
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.front().name, "reclaimed");
  EXPECT_TRUE(nodes.front().alive);
}

// REQ-OPS-17: OpenLockFile must open with O_NONBLOCK and reject non-regular
// files (FIFOs, directories, symlinks) so a sticky-directory pre-existing file
// cannot wedge or hijack lock acquisition.
TEST_F(Registry, OpenLockFileRejectsFifoDirectoryAndSymlinkWithoutBlocking) {
  const char* test_tmpdir = std::getenv("TEST_TMPDIR");
  ASSERT_NE(test_tmpdir, nullptr);
  const std::string base =
      std::string{test_tmpdir} + "/lock_check_" + std::to_string(::getpid());

  const std::string fifo_path = base + ".fifo";
  ::unlink(fifo_path.c_str());
  ASSERT_EQ(::mkfifo(fifo_path.c_str(), 0666), 0);
  {
    // Fork with a short alarm so a regression removing O_NONBLOCK fails fast
    // instead of wedging the test process on open(O_RDONLY) of a FIFO.
    Children children;
    const pid_t pid = ::fork();
    ASSERT_GE(pid, 0);
    if (pid == 0) {
      ::alarm(2);
      errno = 0;
      const int fifo_fd = OpenLockFile(fifo_path, O_RDONLY);
      const int fifo_err = errno;
      if (fifo_fd >= 0) ::close(fifo_fd);
      ::_exit((fifo_fd < 0 && fifo_err == EINVAL) ? 0 : 1);
    }
    children.add(pid);
    const int status = children.reap(pid);
    ::unlink(fifo_path.c_str());
    EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
        << "OpenLockFile on FIFO failed or blocked (status=" << status << ")";
  }

  const std::string dir_path = base + ".dir";
  ::rmdir(dir_path.c_str());
  ASSERT_EQ(::mkdir(dir_path.c_str(), 0755), 0);
  errno = 0;
  const int dir_fd = OpenLockFile(dir_path, O_RDONLY);
  const int dir_err = errno;
  if (dir_fd >= 0) ::close(dir_fd);
  ::rmdir(dir_path.c_str());
  EXPECT_LT(dir_fd, 0);
  EXPECT_EQ(dir_err, EISDIR);

  const std::string real_path = base + ".real";
  const std::string link_path = base + ".link";
  ::unlink(link_path.c_str());
  ::unlink(real_path.c_str());
  const int created = OpenLockFile(real_path, O_RDONLY);
  ASSERT_GE(created, 0);
  ::close(created);
  ASSERT_EQ(::symlink(real_path.c_str(), link_path.c_str()), 0);
  errno = 0;
  const int link_fd = OpenLockFile(link_path, O_RDONLY);
  if (link_fd >= 0) ::close(link_fd);
  ::unlink(link_path.c_str());
  ::unlink(real_path.c_str());
  EXPECT_LT(link_fd, 0);
}

// REQ-CONTRACT-09: Reporter's ExtraSource flags must attach only to the extra
// source's own manifest entry and not smear onto a loop source sharing the same
// topic name.
TEST_F(Registry, ReporterExtraSourceFlagsDoNotSmearOntoLoopSources) {
  const std::string topic = "/shared/state";
  event::RealtimeEventLoop<> loop;
  struct Handler {
    std::function<void()> on_tick;
    void OnTick(const event::Context&) { on_tick(); }
  } handler;
  auto timer = event::make_timer<&Handler::OnTick>(loop, "tick", &handler);
  (void)loop.register_watcher(topic, sizeof(std::uint64_t),
                              alignof(std::uint64_t), kCountSchema, {});

  Reporter reporter{{.name = "mixed",
                     .extra = {{.kind = event::SourceKind::SENDER,
                                .name = topic,
                                .message_bytes = sizeof(std::uint64_t),
                                .alignment = alignof(std::uint64_t),
                                .flags = naming::kSourceFlagExternal |
                                         naming::kSourceFlagOptional}},
                     .period = 20ms,
                     .registry_path = path_}};
  reporter.start(loop);
  std::optional<std::vector<NodeSnapshot>> live;
  handler.on_tick = [&] {
    if (!reporter.published()) return;
    auto reader = RegistryReader::open(path_);
    if (!reader) return;
    auto nodes = reader->nodes();
    if (nodes.size() == 1 && nodes.front().sources.size() == 3) {
      live = std::move(nodes);
      loop.exit();
    }
  };
  timer.setup_periodic(event::Poller::now() + 1ms, 2ms);
  loop.run_for(5s);

  ASSERT_TRUE(live.has_value());
  const auto& sources = live->front().sources;
  ASSERT_EQ(sources.size(), 3u);
  // Loop watcher on `topic` must NOT inherit the ExtraSource's flags.
  EXPECT_EQ(sources[1].kind, event::SourceKind::WATCHER);
  EXPECT_EQ(sources[1].name, topic);
  EXPECT_FALSE(sources[1].external());
  EXPECT_FALSE(sources[1].optional());
  // ExtraSource on `topic` keeps its own flags.
  EXPECT_EQ(sources[2].kind, event::SourceKind::SENDER);
  EXPECT_EQ(sources[2].name, topic);
  EXPECT_TRUE(sources[2].external());
  EXPECT_TRUE(sources[2].optional());
}

// REQ-CONTRACT-09, REQ-SAFE-14: source_id-scoped EndpointAttribute attaches
// only to the matching registration id in NodeRegistration::publish().
TEST_F(Registry, SourceIdScopedEndpointAttributesDoNotSmearAcrossSameTopic) {
  event::Manifest manifest;
  manifest.push_back({.id = 0,
                      .kind = event::SourceKind::WATCHER,
                      .name = "/foo/state",
                      .message_bytes = 16,
                      .alignment = 8});
  manifest.push_back({.id = 1,
                      .kind = event::SourceKind::WATCHER,
                      .name = "/foo/state",
                      .message_bytes = 16,
                      .alignment = 8});
  const std::vector<EndpointAttribute> endpoints = {
      {"/foo/state", naming::kSourceFlagOptional, 0u},
  };
  NodeRegistration registration{{.name = "reader"}, path_};
  registration.publish(manifest, endpoints);
  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  ASSERT_EQ(nodes.size(), 1u);
  ASSERT_EQ(nodes.front().sources.size(), 2u);
  EXPECT_EQ(nodes.front().sources[0].flags, naming::kSourceFlagOptional);
  EXPECT_EQ(nodes.front().sources[1].flags, 0u);
}

// REQ-RT-10: Reporter's helper thread configures itself via
// ConfigureBackgroundThread() (SCHED_OTHER + nice >= kBackgroundNice on Linux,
// QOS_CLASS_UTILITY on macOS) so it runs below every node loop thread,
// verifiable without realtime privilege.
TEST_F(Registry, ReporterThreadRunsAsBackground) {
  // Verify on a fresh std::thread (not the gtest main thread, which macOS CI
  // launchd runners spawn at QOS_CLASS_UTILITY) that new threads do not
  // inherit background priority by default.
  bool plain_thread_background = true;
  std::thread baseline{
      [&] { plain_thread_background = process::ThreadIsBackground(); }};
  baseline.join();
  ASSERT_FALSE(plain_thread_background)
      << "fresh std::thread already runs at background priority";
  event::RealtimeEventLoop<> loop;
  struct Handler {
    std::function<void()> on_tick;
    void OnTick(const event::Context&) { on_tick(); }
  } handler;
  auto timer = event::make_timer<&Handler::OnTick>(loop, "bg_tick", &handler);
  std::atomic<bool> observed{false};
  std::atomic<bool> selection_background{false};
  std::atomic<bool> extra_background{false};
  Reporter reporter{
      {.name = "bg_probe",
       .extra = {{.kind = event::SourceKind::SENDER,
                  .name = "/bg/telemetry/p" + std::to_string(::getpid()),
                  .message_bytes = 8,
                  .events =
                      [&] {
                        extra_background.store(process::ThreadIsBackground(),
                                               std::memory_order_relaxed);
                        return std::uint64_t{1};
                      }}},
       .selections = {{.sources = {"teleop"},
                       .selected =
                           [&] {
                             selection_background.store(
                                 process::ThreadIsBackground(),
                                 std::memory_order_relaxed);
                             observed.store(true, std::memory_order_release);
                             return 0;
                           }}},
       .period = 5ms,
       .registry_path = path_}};
  reporter.start(loop);
  handler.on_tick = [&] {
    if (observed.load(std::memory_order_acquire)) loop.exit();
  };
  timer.setup_periodic(event::Poller::now() + 1ms, 2ms);
  loop.run_for(5s);  // Bound only; exits as soon as a sample is observed.

  ASSERT_TRUE(observed.load(std::memory_order_acquire)) << reporter.error();
  EXPECT_TRUE(extra_background.load(std::memory_order_relaxed));
  EXPECT_TRUE(selection_background.load(std::memory_order_relaxed));
}

}  // namespace
}  // namespace talos::introspect

#include "talOS/introspection/registry.h"

#include <gtest/gtest.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/introspection/reporter.h"

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

TEST_F(Registry, PublishesTopologyAndCountersToAReader) {
  NodeRegistration registration{
      {.name = "odometry", .target = "//2026-robot:odometry", .session_id = 7},
      path_};
  registration.publish(sample_manifest());
  registration.set_dispatch_count(99);
  registration.set_source(2, /*events=*/12, /*dropped=*/1, /*sequence=*/11,
                          /*last_monotonic_ns=*/1234, /*last_latency_ns=*/56,
                          /*max_latency_ns=*/78);

  auto reader = RegistryReader::open(path_);
  if (!reader.has_value()) {
    FAIL() << "Expected registry reader";
  }
  const auto& registry = reader.value();

  const auto nodes = registry.nodes();
  ASSERT_EQ(nodes.size(), 1u);
  const NodeSnapshot& node = nodes.front();

  EXPECT_EQ(node.name, "odometry");
  EXPECT_EQ(node.target, "//2026-robot:odometry");
  EXPECT_EQ(node.session_id, 7u);
  EXPECT_EQ(node.pid, static_cast<std::uint64_t>(::getpid()));
  EXPECT_EQ(node.dispatch_count, 99u);
  EXPECT_TRUE(node.alive);
  ASSERT_EQ(node.sources.size(), 3u);

  EXPECT_EQ(node.sources[0].kind, event::SourceKind::TIMER);
  EXPECT_EQ(node.sources[0].name, "tick");
  EXPECT_EQ(node.sources[0].period_ns, 5'000'000);

  EXPECT_EQ(node.sources[1].kind, event::SourceKind::WATCHER);
  EXPECT_EQ(node.sources[1].name, "/drivetrain/state");
  EXPECT_EQ(node.sources[1].message_bytes, 48u);

  EXPECT_EQ(node.sources[2].kind, event::SourceKind::SENDER);
  EXPECT_EQ(node.sources[2].name, "/odometry/state");
  EXPECT_EQ(node.sources[2].events, 12u);
  EXPECT_EQ(node.sources[2].dropped, 1u);
  EXPECT_EQ(node.sources[2].sequence, 11u);
  EXPECT_EQ(node.sources[2].max_latency_ns, 78);
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

TEST_F(Registry, SelectionsBeyondTheSlotAreCountedAndARecycledSlotHasNone) {
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

  // The slot is free again; a node without Targets must not inherit the
  // previous owner's.
  NodeRegistration next{{.name = "plain"}, path_};
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

TEST_F(Registry, SeveralNodesCoexistAndAReleasedSlotDisappears) {
  auto reader = [this] {
    NodeRegistration first{{.name = "a"}, path_};
    first.publish(sample_manifest());
    {
      NodeRegistration second{{.name = "b"}, path_};
      second.publish({});
      auto view = RegistryReader::open(path_);
      EXPECT_EQ(view->nodes().size(), 2u);
    }
    // `second` released its slot; `first` must still be there and must not
    // have been disturbed by the neighbour coming and going.
    auto view = RegistryReader::open(path_);
    const auto nodes = view->nodes();
    EXPECT_EQ(nodes.size(), 1u);
    EXPECT_EQ(nodes.front().name, "a");
    EXPECT_EQ(nodes.front().sources.size(), 3u);
    return true;
  }();
  EXPECT_TRUE(reader);

  // Both registrations are gone now, so the registry is empty but still valid.
  auto view = RegistryReader::open(path_);
  if (!view.has_value()) {
    FAIL() << "Expected registry reader";
  }
  const auto& registry = view.value();
  EXPECT_TRUE(registry.nodes().empty());
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

TEST_F(Registry, RefusesToRegisterMoreNodesThanTheSegmentHolds) {
  std::vector<std::unique_ptr<NodeRegistration>> held;
  for (std::size_t i = 0; i < kMaxNodes; ++i) {
    held.push_back(std::make_unique<NodeRegistration>(
        NodeRegistration::Identity{.name = "n" + std::to_string(i)}, path_));
    held.back()->publish({});
  }
  // Every slot is claimed and every one is heartbeating, so a further node is
  // refused rather than stealing a live slot.
  EXPECT_THROW(NodeRegistration({.name = "overflow"}, path_),
               std::runtime_error);

  held.pop_back();
  EXPECT_NO_THROW(NodeRegistration({.name = "fits_now"}, path_));
}

TEST_F(Registry, ReporterProgressCadenceLeavesRoomForSubTimeoutPause) {
  RegistryMapping mapping{path_};
  Children children;
  const pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    event::RealtimeEventLoop<> loop;
    struct Handler {
      void OnTick(const event::Context&) {}
    } handler;
    auto timer =
        event::make_timer<&Handler::OnTick>(loop, "progress", &handler);
    Reporter reporter{{.name = "paused", .registry_path = path_}};
    reporter.start(loop);
    timer.setup_periodic(event::Poller::now() + 1ms, 2ms);
    loop.run_for(5s);
    ::_exit(0);
  }
  children.add(pid);
  RegistryReader reader{path_};
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  std::vector<NodeSnapshot> nodes;
  do {
    nodes = reader.nodes();
    if (!nodes.empty() && nodes.front().dispatch_count > 0) break;
    std::this_thread::sleep_for(1ms);
  } while (std::chrono::steady_clock::now() < deadline);
  ASSERT_EQ(nodes.size(), 1u);
  ASSERT_GT(nodes.front().dispatch_count, 0u);
  // Stop near the end of the old 250 ms full-sample interval. Its retained
  // sample would already be 200 ms old, so a 300 ms pause crosses the
  // launcher's unchanged 500 ms floor before SIGCONT can be sent.
  std::this_thread::sleep_for(200ms);
  ASSERT_EQ(::kill(pid, SIGSTOP), 0);
  int status = 0;
  ASSERT_EQ(Children::Wait(pid, &status, WUNTRACED), pid);
  ASSERT_TRUE(WIFSTOPPED(status));
  std::this_thread::sleep_for(300ms);
  nodes = reader.nodes();
  ASSERT_EQ(nodes.size(), 1u);
  const auto age = std::chrono::nanoseconds{
      monotonic_now_ns() - nodes.front().heartbeat_monotonic_ns};
  EXPECT_LT(age, 450ms);
  ASSERT_EQ(::kill(pid, SIGCONT), 0);
}

TEST_F(Registry, ReporterPublishesARunningLoopWithoutTouchingItsManifest) {
  const std::string topic = "/probe/state/p" + std::to_string(::getpid());
  const std::string feed = "/probe/telemetry/p" + std::to_string(::getpid());
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
    const auto deadline = std::chrono::steady_clock::now() + 1s;
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
        nodes.front().sources.size() == 3 &&
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

  ASSERT_EQ(node.sources.size(), 3u);
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

  EXPECT_EQ(node.sources[2].name, feed);
  EXPECT_TRUE(node.sources[2].optional());
  EXPECT_FALSE(node.sources[2].external());

  ASSERT_EQ(node.selections.size(), 1u);
  EXPECT_EQ(node.selections[0].sources,
            (std::vector<std::string>{"teleop", "auto"}));
  EXPECT_EQ(node.selections[0].selected_source(), "auto");
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

// The v1 case, which is also the default case: a node that declares no
// attributes -- or a writer built before attributes existed, which wrote this
// word as a zeroed `reserved` -- must read as neither external nor optional,
// not as some third state.
TEST_F(Registry, ASourceWithNoAttributesIsNeitherExternalNorOptional) {
  NodeRegistration registration{{.name = "odometry"}, path_};
  registration.publish(sample_manifest());

  auto reader = RegistryReader::open(path_);
  if (!reader.has_value()) {
    FAIL() << "Expected registry reader";
  }
  const auto& registry = reader.value();
  const auto nodes = registry.nodes();
  ASSERT_EQ(nodes.size(), 1u);
  const auto& sources = nodes.front().sources;
  ASSERT_EQ(sources.size(), 3u);

  for (const SourceSnapshot& source : sources) {
    EXPECT_EQ(source.flags, 0u) << source.name;
    EXPECT_FALSE(source.external()) << source.name;
    EXPECT_FALSE(source.optional()) << source.name;
  }
}

// A misspelled topic is reported and then published anyway. Taking the node off
// the robot to punish a name would be a worse failure than the name, and the
// registry showing the spelling the node actually used is the whole point: that
// is how the two ends are found not to meet.
TEST_F(Registry, AMisnamedTopicIsWarnedAboutAndStillPublished) {
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

  ::testing::internal::CaptureStderr();
  EXPECT_NO_THROW(registration.publish(manifest));
  // Publishing twice must not double the warning: the reporting thread calls
  // into the registry forever, and a warning that repeats is a warning that
  // gets filtered out.
  registration.publish(manifest);
  const std::string warnings = ::testing::internal::GetCapturedStderr();

  EXPECT_NE(warnings.find("/hw/cmd"), std::string::npos) << warnings;
  EXPECT_NE(warnings.find("hardware_bridge"), std::string::npos) << warnings;
  EXPECT_EQ(std::count(warnings.begin(), warnings.end(), '\n'), 1) << warnings;
  // A timer's name is a label, not an address, so the topic grammar must not
  // be applied to it.
  EXPECT_EQ(warnings.find("tick"), std::string::npos) << warnings;

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

// The heartbeat, the dispatch count and the per-source counters move as one
// sample: a reader that arrives between samples sees the whole sample, never
// a fresh heartbeat paired with the previous refresh's counts.
TEST_F(Registry, WholeSamplesReadWhole) {
  NodeRegistration registration{{.name = "seamed"}, path_};
  registration.publish(sample_manifest());

  auto reader = RegistryReader::open(path_);
  if (!reader.has_value()) {
    FAIL() << "Expected registry reader";
  }
  const auto& registry = reader.value();

  for (std::uint64_t i = 1; i <= 20; ++i) {
    registration.begin_sample();
    registration.heartbeat();
    registration.set_dispatch_count(i);
    registration.set_source(2, /*events=*/i, /*dropped=*/0, /*sequence=*/i,
                            /*last_monotonic_ns=*/0, /*last_latency_ns=*/0,
                            /*max_latency_ns=*/0);
    registration.end_sample();

    const auto nodes = registry.nodes();
    ASSERT_EQ(nodes.size(), 1u);
    EXPECT_EQ(nodes.front().dispatch_count, i);
    ASSERT_EQ(nodes.front().sources.size(), 3u);
    EXPECT_EQ(nodes.front().sources[2].events, i);
    EXPECT_EQ(nodes.front().sources[2].sequence, i);
  }
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

}  // namespace
}  // namespace talos::introspect

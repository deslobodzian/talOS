// Integration tier (fork/exec, signals, a node run on the real clock, the
// live registry): node binaries run as the launcher runs them. Its own
// target, out of node_test's component lane (docs/TESTING.md §2, "One target,
// one lane").
#include <fcntl.h>
#include <gtest/gtest.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "node_test/node_test_message_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/events/input_replay.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/gap_testing.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/introspection/registry.h"
#include "talOS/introspection/reporter.h"
#include "talOS/node/internal.h"
#include "talOS/node/testing.h"
#include "talOS/process/process.h"
#include "talOS/rtms/rtms.h"

namespace talos {
namespace {

using namespace std::chrono_literals;
using node_test::Count;
using testing::Sim;

constexpr const char* kRobotToml = R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.listener]
)";

struct Silent {
  explicit Silent(Node& n) : in{n.subscribe<Count>("/watch/target")} {}
  Subscription<Count> in;
};

// --- the binary
// ----------------------------------------------------------------

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"runfile unavailable: "} + env + " " +
                             error);
  }
  return runfiles->Rlocation(location);
}

struct Result {
  int status;
  std::string output;
};

Result RunBinary(const std::string& args,
                 const char* binary = "TALOS_TEST_NODE") {
  const std::string command = "'" + Runfile(binary) + "' " + args + " 2>&1";
  FILE* pipe = ::popen(command.c_str(), "r");
  if (pipe == nullptr) throw std::runtime_error("popen failed");
  Result result{0, {}};
  char buffer[4096];
  while (const std::size_t n = std::fread(buffer, 1, sizeof buffer, pipe)) {
    result.output.append(buffer, n);
  }
  const int status = ::pclose(pipe);
  result.status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return result;
}

std::string ConfigFlag() {
  return "--config '" + Runfile("TALOS_TEST_CONFIG") + "'";
}

// REQ-CONTRACT-12, REQ-OPS-06
TEST(Binary, DescribeListsEveryEndpointIncludingOptionalOnes) {
  const std::string fresh_ns = "describe_no_shm_" + std::to_string(::getpid());
  struct ScopedNamespace {
    std::optional<std::string> previous;
    explicit ScopedNamespace(const std::string& ns) {
      if (const char* value = std::getenv("TALOS_SHM_NAMESPACE")) {
        previous = value;
      }
      ::setenv("TALOS_SHM_NAMESPACE", ns.c_str(), 1);
    }
    ~ScopedNamespace() {
      if (previous) {
        ::setenv("TALOS_SHM_NAMESPACE", previous->c_str(), 1);
      } else {
        ::unsetenv("TALOS_SHM_NAMESPACE");
      }
    }
  } scoped_ns{fresh_ns};

  for (const char* name : {"/test_node/target", "/test_node/state",
                           "/test_node/frames", introspect::kRegistryPath}) {
    ::shm_unlink(ShmObjectName(name, fresh_ns).c_str());
  }

  const Result result = RunBinary("--describe " + ConfigFlag());
  ASSERT_EQ(result.status, 0) << result.output;
  introspect::Description described;
  std::string error;
  ASSERT_TRUE(
      introspect::DescribeReader::Parse(result.output, described, error))
      << error << "\n"
      << result.output;
  EXPECT_EQ(described.name, "test_node");
  EXPECT_EQ(described.target, "//talOS/node:test_node");
  EXPECT_FALSE(described.reads_graph);
  // A run installs the stop handlers first thing, so the launcher may hold
  // a stop through its start-up.
  EXPECT_TRUE(described.holds_stop);
  // The constructor's one param read is the node's declared parameter.
  EXPECT_EQ(described.params,
            (std::vector<introspect::ParamRead>{{"test_node", "step"}}));
  const auto find = [&](std::string_view name) {
    const auto it =
        std::find_if(described.sources.begin(), described.sources.end(),
                     [&](const auto& source) { return source.name == name; });
    return it == described.sources.end() ? nullptr : &*it;
  };

  const auto* target = find("/test_node/target");
  ASSERT_NE(target, nullptr) << result.output;
  EXPECT_EQ(target->kind, event::SourceKind::WATCHER);
  EXPECT_TRUE(target->optional());
  EXPECT_FALSE(target->external());
  const auto* state = find("/test_node/state");
  ASSERT_NE(state, nullptr) << result.output;
  EXPECT_EQ(state->kind, event::SourceKind::SENDER);
  EXPECT_FALSE(state->optional());
  EXPECT_FALSE(state->external());
  // Every topic endpoint names its schema, with its reflection bytes.
  for (const auto* count : {target, state}) {
    SCOPED_TRACE(count->name);
    EXPECT_FALSE(count->observer());
    EXPECT_EQ(count->message_bytes, 16u);
    EXPECT_EQ(count->schema, "node_test.Count");
    EXPECT_EQ(count->schema_hash, Descriptor<Count>::kHash);
    EXPECT_EQ(count->alignment, 8u);
    EXPECT_EQ(count->slots, 1024u);
    EXPECT_EQ(count->bfbs, schema::Of<Count>().bfbs);
  }
  const auto* frames = find("/test_node/frames");
  ASSERT_NE(frames, nullptr) << result.output;
  EXPECT_EQ(frames->kind, event::SourceKind::SENDER);
  EXPECT_TRUE(frames->external());
  EXPECT_EQ(frames->message_bytes, 64u);
  EXPECT_EQ(frames->schema, "talos.Bytes");
  EXPECT_EQ(frames->schema_hash, schema::BytesHash());
  EXPECT_EQ(frames->alignment, 8u);
  EXPECT_EQ(frames->slots, 16u);
  EXPECT_TRUE(frames->bfbs.empty());
  const auto* tick = find("test_node");
  ASSERT_NE(tick, nullptr) << result.output;
  EXPECT_EQ(tick->kind, event::SourceKind::TIMER);
  // A node with no hardware never touches /hw/*.
  for (const auto& source : described.sources) {
    EXPECT_FALSE(source.name.starts_with("/hw/")) << source.name;
  }

  // --describe touches no shared memory: neither any declared topic nor the
  // registry exists in the fresh namespace after exit 0.
  std::vector<std::string> shm_names{std::string{introspect::kRegistryPath}};
  for (const auto& source : described.sources) {
    if (source.kind == event::SourceKind::SENDER ||
        source.kind == event::SourceKind::WATCHER) {
      shm_names.push_back(source.name);
    }
  }
  for (const std::string& name : shm_names) {
    const std::string shm = ShmObjectName(name, fresh_ns);
    errno = 0;
    const int fd = ::shm_open(shm.c_str(), O_RDONLY, 0);
    if (fd >= 0) {
      ::close(fd);
      ::shm_unlink(shm.c_str());
    }
    EXPECT_EQ(fd, -1) << name << " (" << shm << ") created by --describe";
    EXPECT_EQ(errno, ENOENT) << name << " (" << shm << ")";
  }
}

std::string TempFile(const std::string& name) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  return std::string{tmp != nullptr ? tmp : "/tmp"} + "/" + name;
}

// The launcher's two describe passes and its run, against a node that reads
// the declared graph.
TEST(Binary, AGraphReaderDescribesWhatTheGraphItWasGivenHolds) {
  // First pass: no graph yet; it says it reads one and registers nothing.
  const Result first =
      RunBinary("--describe " + ConfigFlag(), "TALOS_GRAPH_NODE");
  ASSERT_EQ(first.status, 0) << first.output;
  introspect::Description described;
  std::string error;
  ASSERT_TRUE(introspect::DescribeReader::Parse(first.output, described, error))
      << error << "\n"
      << first.output;
  EXPECT_TRUE(described.reads_graph);
  EXPECT_EQ(described.sources.size(), 1u);  // its tick timer

  // Second pass: test_node's description is the graph.
  const Result test_node = RunBinary("--describe " + ConfigFlag());
  ASSERT_EQ(test_node.status, 0) << test_node.output;
  introspect::Description publisher;
  ASSERT_TRUE(
      introspect::DescribeReader::Parse(test_node.output, publisher, error))
      << error;
  const std::string graph = TempFile("talos_declared_graph.json");
  {
    std::ofstream out{graph};
    out << introspect::DeclaredGraphToJson({publisher});
  }
  const Result second = RunBinary(
      "--describe --graph '" + graph + "' " + ConfigFlag(), "TALOS_GRAPH_NODE");
  ASSERT_EQ(second.status, 0) << second.output;
  ASSERT_TRUE(
      introspect::DescribeReader::Parse(second.output, described, error))
      << error;
  const auto observed = std::find_if(
      described.sources.begin(), described.sources.end(),
      [](const auto& source) { return source.name == "/test_node/state"; });
  ASSERT_NE(observed, described.sources.end()) << second.output;
  EXPECT_TRUE(observed->observer());
  EXPECT_EQ(observed->kind, event::SourceKind::WATCHER);
  EXPECT_EQ(observed->schema, "node_test.Count");
  EXPECT_EQ(observed->schema_hash, Descriptor<Count>::kHash);
  EXPECT_EQ(observed->bfbs, schema::Of<Count>().bfbs);

  // Run for real without a graph: it cannot be built.
  const Result run =
      RunBinary("--sim --duration-s 1 --log '" + TempFile("graph_node.tlog") +
                    "' " + ConfigFlag(),
                "TALOS_GRAPH_NODE");
  EXPECT_NE(run.status, 0);
}

// Node::describing() is set only for the launcher's --describe probe: a run
// (and a Sim) builds the node with it clear, so no node skips its backends
// for real.
TEST(Binary, OnlyTheDescribeProbeBuildsANodeThatIsDescribing) {
  const Result described =
      RunBinary("--describe " + ConfigFlag(), "TALOS_DESCRIBING_NODE");
  ASSERT_EQ(described.status, 0) << described.output;
  EXPECT_NE(described.output.find("describing=1"), std::string::npos)
      << described.output;

  const Result run =
      RunBinary("--sim --duration-s 1 --log '" +
                    TempFile("describing_node.tlog") + "' " + ConfigFlag(),
                "TALOS_DESCRIBING_NODE");
  ASSERT_EQ(run.status, 0) << run.output;
  EXPECT_NE(run.output.find("describing=0"), std::string::npos) << run.output;

  auto sim = Sim::FromToml(kRobotToml);
  sim.add<Silent>("listener");
  EXPECT_FALSE(sim.node("listener").describing());
}

// SIGINT while the constructor is blocked in a system call does not kill the
// node: the call keeps waiting, and once the constructor returns the node
// exits cleanly without running its loop.
TEST(Binary, StopDuringConstructionExitsCleanlyOnceConstructed) {
  const std::string binary = Runfile("TALOS_BLOCKED_NODE");
  const std::string config = Runfile("TALOS_TEST_CONFIG");
  const std::string log = TempFile("blocked_node.tlog");
  int input[2];
  int output[2];
  ASSERT_EQ(::pipe(input), 0);
  ASSERT_EQ(::pipe(output), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::dup2(input[0], STDIN_FILENO);
    ::dup2(output[1], STDOUT_FILENO);
    ::close(input[0]);
    ::close(input[1]);
    ::close(output[0]);
    ::close(output[1]);
    ::execl(binary.c_str(), binary.c_str(), "--sim", "--duration-s", "1",
            "--log", log.c_str(), "--config", config.c_str(), nullptr);
    ::_exit(127);
  }
  ::close(input[0]);
  ::close(output[1]);
  std::string printed;
  const auto read_some = [&] {
    char buffer[256];
    const ssize_t got = ::read(output[0], buffer, sizeof buffer);
    if (got > 0) printed.append(buffer, static_cast<std::size_t>(got));
    return got > 0;
  };
  while (printed.find("constructing\n") == std::string::npos && read_some()) {
  }
  const bool blocked = printed.find("constructing\n") != std::string::npos;
  bool stop_landed = false;
  if (blocked) {
    EXPECT_EQ(::kill(child, SIGINT), 0);
    // Each byte wakes the constructor's read, which answers with one line
    // saying whether the stop has landed. The signal may reach another of
    // the node's threads first, so ask until it has. A child whose read was
    // interrupted exits instead, and the answer never comes (nor a SIGPIPE
    // for the write that finds it gone).
    struct sigaction ignore {};
    struct sigaction previous {};
    ignore.sa_handler = SIG_IGN;
    ::sigaction(SIGPIPE, &ignore, &previous);
    while (!stop_landed) {
      const char byte = 0;
      if (::write(input[1], &byte, 1) != 1) break;
      const std::size_t answered = printed.size();
      while (printed.find('\n', answered) == std::string::npos && read_some()) {
      }
      if (printed.find('\n', answered) == std::string::npos) break;
      stop_landed =
          printed.find("stop requested\n", answered) != std::string::npos;
    }
    ::sigaction(SIGPIPE, &previous, nullptr);
  }
  ::close(input[1]);  // the constructor's read returns
  while (read_some()) {
  }
  ::close(output[0]);
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(blocked) << printed;
  EXPECT_TRUE(stop_landed) << printed;
  ASSERT_TRUE(WIFEXITED(status))
      << "killed by signal " << (WIFSIGNALED(status) ? WTERMSIG(status) : 0);
  EXPECT_EQ(WEXITSTATUS(status), 0) << printed;
  EXPECT_NE(printed.find("stopped during construction"), std::string::npos)
      << printed;
  EXPECT_EQ(printed.find("dispatches="), std::string::npos) << printed;
}

// Describing and exact replay are offline: a robot (here a sentinel
// publisher) may own the node's byte topic on this machine, and neither the
// launcher's --describe probe nor replay attaches to it or fails because of
// it. Byte topics are not logged, so replay does not verify them; typed sends
// still are, and a changed parameter diverges, naming the dispatch and topic.
// REQ-OPS-13
TEST(Binary, LoggedSimRunReplaysOfflineWithoutDivergence) {
  const std::string log = TempFile("talos_node_test.tlog");
  // Isolate the sentinel and child replay from live publishers, restoring
  // the caller's namespace afterwards. This process caches its namespace at
  // its first ShmObjectName, before or during this test: under Bazel it is
  // TEST_TMPDIR's either way, so the children (here and in later tests)
  // share it.
  struct Namespace {
    std::optional<std::string> previous;
    bool set = false;
    Namespace() {
      if (const char* value = std::getenv("TALOS_SHM_NAMESPACE"))
        previous = value;
      // Under Bazel TEST_TMPDIR already isolates this test and its
      // children; setting the variable to it would only move the
      // children's registry lock file out of TEST_TMPDIR.
      set = std::getenv("TEST_TMPDIR") == nullptr;
      if (!set) return;
      const std::string name = "replay_offline_" + std::to_string(::getpid());
      ::setenv("TALOS_SHM_NAMESPACE", name.c_str(), 1);
    }
    ~Namespace() {
      if (!set) return;
      if (previous) {
        ::setenv("TALOS_SHM_NAMESPACE", previous->c_str(), 1);
      } else {
        ::unsetenv("TALOS_SHM_NAMESPACE");
      }
    }
  } isolated;
  const Result run =
      RunBinary("--sim --duration-s 1 --log '" + log + "' " + ConfigFlag());
  ASSERT_EQ(run.status, 0) << run.output;
  {
    RTMSQueue sentinel{
        "/test_node/frames",
        64,
        kMessageAlignment,
        schema::Bytes(),
        16,
        RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                    .read_mode = ReadMode::SEQUENCE,
                    .publisher = true}};
    const std::uint64_t written = sentinel.writer_sequence();
    const Result describe = RunBinary("--describe " + ConfigFlag());
    EXPECT_EQ(describe.status, 0) << describe.output;
    const Result replay = RunBinary("--replay '" + log + "' " + ConfigFlag());
    EXPECT_EQ(replay.status, 0) << replay.output;
    EXPECT_EQ(sentinel.writer_sequence(), written);
  }

  std::string config;
  {
    std::ifstream in{Runfile("TALOS_TEST_CONFIG")};
    config.assign(std::istreambuf_iterator<char>{in}, {});
  }
  const std::size_t step = config.find("step = 3");
  ASSERT_NE(step, std::string::npos);
  config.replace(step, 8, "step = 4");
  const std::string changed = TempFile("talos_node_test_step4.toml");
  std::ofstream{changed} << config;
  const Result diverged =
      RunBinary("--replay '" + log + "' --config '" + changed + "'");
  EXPECT_EQ(diverged.status, 1) << diverged.output;
  EXPECT_NE(diverged.output.find("dispatch 2"), std::string::npos)
      << diverged.output;
  EXPECT_NE(diverged.output.find("/test_node/state"), std::string::npos)
      << diverged.output;
  // Only the --sim run wrote the node's byte topic to shared memory.
  ::shm_unlink(ShmObjectName("/test_node/frames").c_str());
}

// A node binary's replay of a log with a gap (a GAP record, as the writer
// leaves where its pool filled) stops and exits 1, naming the log and the
// gap: --replay before the gap's first dropped dispatch, with no divergence;
// --replay-inputs, fed /test_node/target from its own log (it watched the
// topic live), before the gap's start.
// REQ-OPS-13, REQ-OPS-14
TEST(Binary, ReplayOfALogWithAGapStopsThereAndExits1) {
  const std::string log = TempFile("talos_node_test_gap_whole.tlog");
  const Result run =
      RunBinary("--sim --duration-s 1 --log '" + log + "' " + ConfigFlag());
  ASSERT_EQ(run.status, 0) << run.output;
  // Only the --sim run wrote the node's byte topic to shared memory.
  ::shm_unlink(ShmObjectName("/test_node/frames").c_str());

  // Drop an update() dispatch past the middle and the two after it.
  std::vector<event::log::RecordHeader> headers;
  std::uint16_t update = 0;
  {
    event::log::LogReader reader{log};
    for (const event::Registration& source : reader.manifest()) {
      if (source.kind == event::SourceKind::TIMER &&
          source.name == "test_node") {
        update = source.id;
      }
    }
    event::log::LogReader::Record record;
    while (reader.next(record)) headers.push_back(record.header);
  }
  const auto starts = [&](std::size_t i) {
    return headers[i].dispatch_index != headers[i - 1].dispatch_index;
  };
  std::size_t first = headers.size() / 2;
  while (first < headers.size() &&
         !(headers[first].kind ==
               static_cast<std::uint16_t>(event::EventKind::TIMER) &&
           headers[first].source_id == update)) {
    ++first;
  }
  std::size_t end = first + 1;
  for (int dispatches = 0; end < headers.size(); ++end) {
    if (starts(end) && ++dispatches == 3) break;
  }
  ASSERT_LT(end, headers.size() - 1) << "too few records: " << headers.size();
  const std::string dir = TempFile("talos_node_test_gap_session");
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  const std::string gapped = dir + "/test_node.tlog";
  const event::log::LogGap gap =
      event::log::CopyWithGap(log, gapped, first, end);

  {
    SCOPED_TRACE("--replay");
    const Result replay =
        RunBinary("--replay '" + gapped + "' " + ConfigFlag());
    EXPECT_EQ(replay.status, 1) << replay.output;
    EXPECT_NE(replay.output.find("diverged=0 clean_exit=0 stopped_at_gap=1"),
              std::string::npos)
        << replay.output;
    // The stop line names the log, the dropped dispatches and records.
    const std::size_t named = replay.output.find(gapped);
    ASSERT_NE(named, std::string::npos) << replay.output;
    const std::string line =
        replay.output.substr(named, replay.output.find('\n', named) - named);
    const auto names = [&line](std::uint64_t value) {
      const std::string number = " " + std::to_string(value);
      for (std::size_t at = line.find(number); at != std::string::npos;
           at = line.find(number, at + 1)) {
        const std::size_t after = at + number.size();
        if (after == line.size() ||
            std::isdigit(static_cast<unsigned char>(line[after])) == 0) {
          return true;
        }
      }
      return false;
    };
    for (const std::uint64_t value :
         {gap.first_dispatch_index, gap.detail.last_dispatch_index,
          gap.detail.records}) {
      EXPECT_TRUE(names(value)) << value << " in " << line;
    }
  }
  {
    SCOPED_TRACE("--replay-inputs");
    const Result replay = RunBinary(
        "--replay-inputs '" + dir + "' --log '" +
        TempFile("talos_node_test_gap_inputs.tlog") + "' " + ConfigFlag());
    EXPECT_EQ(replay.status, 1) << replay.output;
    // The stop line names the log and the topic.
    const std::size_t named = replay.output.find(gapped);
    ASSERT_NE(named, std::string::npos) << replay.output;
    const std::string line =
        replay.output.substr(named, replay.output.find('\n', named) - named);
    EXPECT_NE(line.find("/test_node/target"), std::string::npos) << line;
  }
  std::filesystem::remove_all(dir);
}

// A node binary's --replay-inputs takes SESSION_DIR and refuses a path that
// is not a directory (a .tlog file or a missing path), naming the path as
// given rather than reading a single log or prefixing a bare name with "./".
// REQ-OPS-14
TEST(Binary, ReplayInputsRefusesANonDirectoryPath) {
  const std::string log = TempFile("talos_node_test_replay_file.tlog");
  const Result recorded =
      RunBinary("--sim --duration-s 1 --log '" + log + "' " + ConfigFlag());
  ASSERT_EQ(recorded.status, 0) << recorded.output;
  ::shm_unlink(ShmObjectName("/test_node/frames").c_str());

  const std::string out = TempFile("talos_node_test_replay_file_out.tlog");
  std::filesystem::remove(out);
  {
    SCOPED_TRACE("existing .tlog file");
    const Result replay = RunBinary("--replay-inputs '" + log + "' --log '" +
                                    out + "' " + ConfigFlag());
    EXPECT_NE(replay.status, 0) << replay.output;
    EXPECT_NE(replay.output.find(log), std::string::npos) << replay.output;
    EXPECT_NE(replay.output.find("not a session directory"), std::string::npos)
        << replay.output;
    EXPECT_FALSE(std::filesystem::exists(out));
  }
  {
    SCOPED_TRACE("bare filename");
    const std::string bare = "missing_session.tlog";
    const Result replay = RunBinary("--replay-inputs '" + bare + "' --log '" +
                                    out + "' " + ConfigFlag());
    EXPECT_NE(replay.status, 0) << replay.output;
    EXPECT_NE(replay.output.find(bare), std::string::npos) << replay.output;
    EXPECT_EQ(replay.output.find("./" + bare), std::string::npos)
        << replay.output;
    EXPECT_NE(replay.output.find("not a session directory"), std::string::npos)
        << replay.output;
    EXPECT_FALSE(std::filesystem::exists(out));
  }
  if (::geteuid() != 0) {
    SCOPED_TRACE("unsearchable parent directory (EACCES)");
    const std::string locked = TempFile("talos_node_test_locked_parent");
    std::filesystem::remove_all(locked);
    std::filesystem::create_directories(locked + "/session");
    std::filesystem::permissions(locked, std::filesystem::perms::none);
    const std::string path = locked + "/session";
    const Result replay = RunBinary("--replay-inputs '" + path + "' --log '" +
                                    out + "' " + ConfigFlag());
    std::filesystem::permissions(locked, std::filesystem::perms::owner_all);
    std::filesystem::remove_all(locked);
    EXPECT_NE(replay.status, 0) << replay.output;
    EXPECT_NE(replay.output.find(path), std::string::npos) << replay.output;
    EXPECT_EQ(replay.output.find("not a session directory"), std::string::npos)
        << replay.output;
    EXPECT_NE(replay.output.find(
                  std::make_error_code(std::errc::permission_denied).message()),
              std::string::npos)
        << replay.output;
    EXPECT_FALSE(std::filesystem::exists(out));
  }
  std::filesystem::remove(log);
}

}  // namespace

namespace detail {
// node.cc: tests only, applied to the options of the log a run writes and
// of a live run's Reporter.
extern std::function<void(event::log::LogWriterOptions&)> log_options_for_test;
extern std::function<void(introspect::Reporter::Options&)>
    reporter_options_for_test;
}  // namespace detail

namespace {

// Holds a run's log writer before its first write, so its two small chunks
// fill and the run's log opens a gap, until the node's update() decides to
// let it go (`release_when`): never before the gap opened, and only where
// nothing after it can open a second one.
struct HeldLog {
  std::atomic<bool> released{false};
  std::function<bool(const Node&)> release_when;
};
HeldLog* held_log = nullptr;

class GapProbe {
 public:
  explicit GapProbe(Node& node)
      : node_{node}, out_{node.publish<Count>("/test_node/state")} {}

  void update() {
    out_.send(
        Count{static_cast<std::int64_t>(node_.tick()), node_.now().nanos()});
    if (!held_log->released.load() && held_log->release_when(node_)) {
      held_log->released.store(true);
      held_log->released.notify_all();
    }
  }

 private:
  Node& node_;
  Publisher<Count> out_;
};

// Runs a GapProbe as the binary `test_node` would, in this process, with
// `args` (quoted as a shell would, by '...'), its log writer held by `held`.
// Returns its exit status and what it printed.
Result RunHeldInProcess(const std::string& args, HeldLog& held) {
  held.released.store(false);
  held_log = &held;
  detail::log_options_for_test = [&held](event::log::LogWriterOptions& o) {
    o.chunk_bytes = 1024;
    o.chunk_count = 2;
    o.before_write = [&held] { held.released.wait(false); };
  };
  std::vector<std::string> words{"test_node"};
  {
    std::istringstream split{args};
    for (std::string word; split >> std::quoted(word, '\'');) {
      words.push_back(word);
    }
  }
  std::vector<char*> argv;
  argv.reserve(words.size());
  for (std::string& word : words) argv.push_back(word.data());
  ::testing::internal::CaptureStdout();
  const int status =
      detail::RunMain(static_cast<int>(argv.size()), argv.data(), "test_node",
                      "//test", detail::MakeInstance<GapProbe>());
  Result result{status, ::testing::internal::GetCapturedStdout()};
  detail::log_options_for_test = nullptr;
  process::stop_requested.store(false);
  held_log = nullptr;
  return result;
}

// Whether this process's newest test_node row in the live registry has
// kFlagLogGap.
bool RowHasLogGap() {
  const auto reader = introspect::RegistryReader::open();
  if (!reader) return false;
  bool gap = false;
  std::uint64_t generation = 0;
  for (const auto& row : reader->nodes()) {
    if (row.name != "test_node" ||
        row.pid != static_cast<std::uint64_t>(::getpid()) ||
        row.generation < generation) {
      continue;
    }
    generation = row.generation;
    gap = row.log_gap();
  }
  return gap;
}

// A live node's registry row says its log has a gap as the gap opens, not
// at the reporter's next refresh: a run killed in between is still marked
// log_gap. The reporter copies no event log flag here
// (Reporter::Options::copy_recording), whenever its samples run, so the bit
// in the row can only be the writer's.
// REQ-OPS-52
TEST(Binary, ANodesRowHasTheLogGapBitAsTheGapOpens) {
  detail::reporter_options_for_test = [](introspect::Reporter::Options& o) {
    o.copy_recording = false;
  };
  bool seen = false;
  HeldLog held;
  held.release_when = [&seen](const Node& node) {
    seen = RowHasLogGap();
    if (!seen && node.tick() < 300) return false;
    process::stop_requested.store(true);
    return true;
  };
  const Result run = RunHeldInProcess(
      "--sim --duration-s 30 --log '" +
          TempFile("talos_node_test_row_gap.tlog") + "' " + ConfigFlag(),
      held);
  detail::reporter_options_for_test = nullptr;
  EXPECT_TRUE(seen) << "no kFlagLogGap in the row after 300 ticks\n"
                    << run.output;
  EXPECT_EQ(run.status, 1) << run.output;
  ::shm_unlink(ShmObjectName("/test_node/state").c_str());
}

// A run whose own output log has a gap is incomplete evidence: the live run
// (RunMain) and a --replay-inputs run (fed from a session with no loss)
// print recording_gaps=1 and exit 1. The writer is held until its pool
// fills; the live run then stops itself (it sees the gap in its registry
// row), and the replay lets it go in its last update(), so no second gap
// can open.
// REQ-OPS-52, REQ-OPS-14
TEST(Binary, ARunWhoseOwnLogHasAGapExits1) {
  const std::string dir = TempFile("talos_node_test_own_gap_session");
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  const Result recorded = RunBinary("--sim --duration-s 1 --log '" + dir +
                                    "/test_node.tlog' " + ConfigFlag());
  ASSERT_EQ(recorded.status, 0) << recorded.output;
  ::shm_unlink(ShmObjectName("/test_node/frames").c_str());
  const Time end = event::RecordedSession{dir, "test_node"}.loop_end();

  struct Row {
    const char* name;
    std::string args;
    std::function<bool(const Node&)> release_when;
  };
  const Row rows[] = {
      // A stop between dispatches: the rest of this one is dropped with the
      // gap, and the EXIT follows.
      {"live", "--sim --duration-s 30",
       [](const Node& node) {
         if (!RowHasLogGap() && node.tick() < 1000) return false;
         process::stop_requested.store(true);
         return true;
       }},
      // The last update() before the session's end: at most one more
      // dispatch and the EXIT follow, which fit one chunk.
      {"replay-inputs", "--replay-inputs '" + dir + "'",
       [end](const Node& node) { return node.now() + node.period() >= end; }},
  };
  for (const Row& row : rows) {
    SCOPED_TRACE(row.name);
    HeldLog held;
    held.release_when = row.release_when;
    const Result run = RunHeldInProcess(
        row.args + " --log '" + TempFile("talos_node_test_own_gap.tlog") +
            "' " + ConfigFlag(),
        held);
    EXPECT_EQ(run.status, 1) << run.output;
    EXPECT_NE(run.output.find("recording_gaps=1 "), std::string::npos)
        << run.output;
    EXPECT_EQ(run.output.find("stopped before"), std::string::npos)
        << run.output;
  }
  ::shm_unlink(ShmObjectName("/test_node/state").c_str());
  std::filesystem::remove_all(dir);
}

// A running node whose loop has no timer and no subscription waits on
// nothing but a stop, and no helper thread polls for one: SIGTERM must end
// that wait from the loop's own thread, and the node exit 0 after its run,
// long before --duration-s (60 s), the only other way out. The signal goes
// once the registry row shows the loop running (endpoints are published only
// then).
// REQ-OPS-06
// REQ-RT-21
TEST(Binary, StopEndsARunningNodeThatWaitsOnNothing) {
  const std::string binary = Runfile("TALOS_BLOCKED_NODE");
  const std::string config = Runfile("TALOS_TEST_CONFIG");
  const std::string log = TempFile("idle_node.tlog");
  int output[2];
  ASSERT_EQ(::pipe(output), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    // Its constructor reads stdin to EOF: at once, from /dev/null.
    const int null = ::open("/dev/null", O_RDONLY);
    ::dup2(null, STDIN_FILENO);
    ::dup2(output[1], STDOUT_FILENO);
    ::close(null);
    ::close(output[0]);
    ::close(output[1]);
    ::execl(binary.c_str(), binary.c_str(), "--sim", "--duration-s", "60",
            "--log", log.c_str(), "--config", config.c_str(), nullptr);
    ::_exit(127);
  }
  ::close(output[1]);
  const auto running = [child] {
    const auto reader = introspect::RegistryReader::open();
    if (!reader) return false;
    for (const auto& node : reader->nodes()) {
      if (node.pid == static_cast<std::uint64_t>(child) && node.alive) {
        return true;
      }
    }
    return false;
  };
  int status = 0;
  bool exited = false;
  bool live = running();
  const auto give_up = std::chrono::steady_clock::now() + 20s;
  while (!live && std::chrono::steady_clock::now() < give_up) {
    if (::waitpid(child, &status, WNOHANG) == child) {
      exited = true;
      break;
    }
    std::this_thread::sleep_for(1ms);  // paces the poll; the row is the event
    live = running();
  }
  const auto signalled = std::chrono::steady_clock::now();
  if (!exited) ::kill(child, live ? SIGTERM : SIGKILL);
  std::string printed;
  char buffer[256];
  for (;;) {
    const ssize_t got = ::read(output[0], buffer, sizeof buffer);
    if (got <= 0) break;
    printed.append(buffer, static_cast<std::size_t>(got));
  }
  ::close(output[0]);
  if (!exited) {
    ASSERT_EQ(::waitpid(child, &status, 0), child);
  }
  const auto stopped_after = std::chrono::steady_clock::now() - signalled;

  ASSERT_TRUE(live) << "the node's loop never showed running: " << printed;
  ASSERT_TRUE(WIFEXITED(status))
      << "killed by signal " << (WIFSIGNALED(status) ? WTERMSIG(status) : 0);
  EXPECT_EQ(WEXITSTATUS(status), 0) << printed;
  EXPECT_NE(printed.find("dispatches="), std::string::npos) << printed;
  EXPECT_LT(stopped_after, 30s) << "the stop waited for --duration-s";
}

struct CapturedRun {
  int status;
  std::string out;
  std::string err;
};

CapturedRun RunInProcess(const std::string& args,
                         const detail::InstanceFactory& make) {
  std::vector<std::string> words{"test_node"};
  {
    std::istringstream split{args};
    for (std::string word; split >> std::quoted(word, '\'');) {
      words.push_back(word);
    }
  }
  std::vector<char*> argv;
  argv.reserve(words.size());
  for (std::string& word : words) argv.push_back(word.data());
  ::testing::internal::CaptureStdout();
  ::testing::internal::CaptureStderr();
  const int status = detail::RunMain(static_cast<int>(argv.size()), argv.data(),
                                     "test_node", "//test", make);
  CapturedRun result{status, ::testing::internal::GetCapturedStdout(),
                     ::testing::internal::GetCapturedStderr()};
  process::stop_requested.store(false);
  return result;
}

// A node binary started with --targeted-outputs (passed by the launcher when
// --describe declared a hardware_request endpoint without untargeted_outputs)
// refuses AllowUntargetedOutputs at construction and never ticks. Without the
// flag (standalone run) it constructs and ticks; under --describe the flag is
// ignored so the probe still reports untargeted_outputs for the graph lint; a
// targeted actuator node runs cleanly with --targeted-outputs.
// REQ-SAFE-42, REQ-CONTRACT-11
TEST(Binary, TargetedOutputsRefusesAllowUntargetedOutputsAtConstruction) {
  const std::string log = TempFile("targeted_outputs.tlog");
  std::atomic<int> ticks{0};

  const auto make_actuator = [&ticks](bool opt_out, bool hide_in_describe) {
    return [&ticks, opt_out, hide_in_describe](Node& node) -> detail::Instance {
      detail::NodeAccess::add_hardware_request(
          node, sizeof(Count), alignof(Count), schema::Of<Count>());
      if (opt_out && (!hide_in_describe || !node.describing())) {
        detail::NodeAccess::mark_untargeted_outputs(node);
      }
      detail::NodeAccess::set_update(node, [&ticks] {
        ticks.fetch_add(1);
        process::stop_requested.store(true);
      });
      return std::make_shared<int>(0);
    };
  };

  {
    SCOPED_TRACE("opt-out refused under --targeted-outputs");
    ticks.store(0);
    const CapturedRun run = RunInProcess(
        "--sim --targeted-outputs --duration-s 5 --log '" + log + "' " +
            ConfigFlag(),
        make_actuator(/*opt_out=*/true, /*hide_in_describe=*/false));
    EXPECT_EQ(run.status, 1) << run.out << "\n" << run.err;
    EXPECT_EQ(ticks.load(), 0) << "refused node must never tick";
    EXPECT_NE(run.err.find("test_node"), std::string::npos) << run.err;
    EXPECT_NE(run.err.find("AllowUntargetedOutputs"), std::string::npos)
        << run.err;
    EXPECT_NE(run.err.find("--targeted-outputs"), std::string::npos) << run.err;
  }

  {
    SCOPED_TRACE("hidden opt-out refused under --targeted-outputs");
    ticks.store(0);
    const CapturedRun probe = RunInProcess(
        "--describe " + ConfigFlag(),
        make_actuator(/*opt_out=*/true, /*hide_in_describe=*/true));
    ASSERT_EQ(probe.status, 0) << probe.err;
    introspect::Description described;
    std::string error;
    ASSERT_TRUE(introspect::DescribeReader::Parse(probe.out, described, error))
        << error;
    const auto req = std::find_if(
        described.sources.begin(), described.sources.end(),
        [](const auto& s) { return s.name == "/hw/request/test_node"; });
    ASSERT_NE(req, described.sources.end()) << probe.out;
    EXPECT_NE(req->flags & introspect::naming::kSourceFlagHardwareRequest, 0u);
    EXPECT_EQ(req->flags & introspect::naming::kSourceFlagUntargetedOutputs,
              0u);

    const CapturedRun run = RunInProcess(
        "--sim --targeted-outputs --duration-s 5 --log '" + log + "' " +
            ConfigFlag(),
        make_actuator(/*opt_out=*/true, /*hide_in_describe=*/true));
    EXPECT_EQ(run.status, 1) << run.out << "\n" << run.err;
    EXPECT_EQ(ticks.load(), 0) << "hidden opt-out must never tick";
    EXPECT_NE(run.err.find("AllowUntargetedOutputs"), std::string::npos)
        << run.err;
    EXPECT_NE(run.err.find("--targeted-outputs"), std::string::npos) << run.err;
  }

  {
    SCOPED_TRACE("standalone opt-out without --targeted-outputs succeeds");
    ticks.store(0);
    const CapturedRun run = RunInProcess(
        "--sim --duration-s 5 --log '" + log + "' " + ConfigFlag(),
        make_actuator(/*opt_out=*/true, /*hide_in_describe=*/false));
    EXPECT_EQ(run.status, 0) << run.out << "\n" << run.err;
    EXPECT_GE(ticks.load(), 1);
    ::shm_unlink(ShmObjectName("/hw/request/test_node").c_str());
  }

  {
    SCOPED_TRACE("--describe ignores --targeted-outputs");
    ticks.store(0);
    const CapturedRun probe = RunInProcess(
        "--describe --targeted-outputs " + ConfigFlag(),
        make_actuator(/*opt_out=*/true, /*hide_in_describe=*/false));
    ASSERT_EQ(probe.status, 0) << probe.err;
    EXPECT_EQ(ticks.load(), 0);
    introspect::Description described;
    std::string error;
    ASSERT_TRUE(introspect::DescribeReader::Parse(probe.out, described, error))
        << error << "\n"
        << probe.out;
    const auto req = std::find_if(
        described.sources.begin(), described.sources.end(),
        [](const auto& s) { return s.name == "/hw/request/test_node"; });
    ASSERT_NE(req, described.sources.end()) << probe.out;
    EXPECT_NE(req->flags & introspect::naming::kSourceFlagHardwareRequest, 0u);
    EXPECT_NE(req->flags & introspect::naming::kSourceFlagUntargetedOutputs,
              0u);
  }

  {
    SCOPED_TRACE("normal targeted actuator runs cleanly with flag");
    ticks.store(0);
    const CapturedRun run = RunInProcess(
        "--sim --targeted-outputs --duration-s 5 --log '" + log + "' " +
            ConfigFlag(),
        make_actuator(/*opt_out=*/false, /*hide_in_describe=*/false));
    EXPECT_EQ(run.status, 0) << run.out << "\n" << run.err;
    EXPECT_GE(ticks.load(), 1);
    ::shm_unlink(ShmObjectName("/hw/request/test_node").c_str());
  }

  std::filesystem::remove(log);
}

// A real running Node's live registry row carries kSourceFlagUntargetedOutputs
// on `/hw/request/<node>` iff AllowUntargetedOutputs (mark_untargeted_outputs)
// was called during construction, so the launcher's live endpoint check catches
// an opt-out hidden from --describe.
// REQ-SAFE-42
TEST(Binary, LiveRegistryRowCarriesUntargetedOutputsIffAllowed) {
  const std::string log = TempFile("live_untargeted_outputs.tlog");

  const auto read_live_request_flags = []() -> std::optional<std::uint32_t> {
    const auto reader = introspect::RegistryReader::open();
    if (!reader) return std::nullopt;
    std::uint64_t best_gen = 0;
    std::optional<std::uint32_t> flags;
    for (const auto& row : reader->nodes()) {
      if (row.name != "test_node" ||
          row.pid != static_cast<std::uint64_t>(::getpid()) || !row.alive ||
          row.generation < best_gen) {
        continue;
      }
      best_gen = row.generation;
      flags.reset();
      for (const auto& src : row.sources) {
        if (src.name == "/hw/request/test_node") {
          flags = src.flags;
          break;
        }
      }
    }
    return flags;
  };

  for (const bool opt_out : {true, false}) {
    SCOPED_TRACE(opt_out ? "with AllowUntargetedOutputs"
                         : "without AllowUntargetedOutputs");
    std::optional<std::uint32_t> observed_flags;
    const CapturedRun run = RunInProcess(
        "--sim --duration-s 5 --log '" + log + "' " + ConfigFlag(),
        [opt_out, &observed_flags,
         &read_live_request_flags](Node& node) -> detail::Instance {
          detail::NodeAccess::add_hardware_request(
              node, sizeof(Count), alignof(Count), schema::Of<Count>());
          if (opt_out) detail::NodeAccess::mark_untargeted_outputs(node);
          detail::NodeAccess::set_update(
              node, [&observed_flags, &read_live_request_flags] {
                observed_flags = read_live_request_flags();
                process::stop_requested.store(true);
              });
          return std::make_shared<int>(0);
        });
    ASSERT_EQ(run.status, 0) << run.out << "\n" << run.err;
    ASSERT_TRUE(observed_flags.has_value())
        << "live registry row missing /hw/request/test_node";
    EXPECT_NE(*observed_flags & introspect::naming::kSourceFlagHardwareRequest,
              0u);
    EXPECT_EQ((*observed_flags &
               introspect::naming::kSourceFlagUntargetedOutputs) != 0u,
              opt_out);
    ::shm_unlink(ShmObjectName("/hw/request/test_node").c_str());
  }

  std::filesystem::remove(log);
}

// REQ-SAFE-14, REQ-CONTRACT-09, REQ-CONTRACT-11: --describe emits
// "operator_arming": true on armed target lanes, emits
// "internal_driver_station": true only when the node has no explicit Driver
// Station reader, and does not throw on an unarmed operator producer so
// LintGraph reports the structured error. Live runs refuse an unarmed operator
// producer (including one hidden behind `if (!node.describing())`) with exit 1
// and 0 ticks even standalone (no launcher flag), and live registry rows carry
// both flags faithfully for EndpointMismatch.
TEST(Binary, OperatorArmingFlagsInDescribeRegistryAndLiveEnforcement) {
  const std::string log = TempFile("operator_arming_binary.tlog");
  const auto find_source =
      [](const std::vector<introspect::naming::SourceShape>& sources,
         std::string_view name) -> const introspect::naming::SourceShape* {
    const auto it = std::find_if(sources.begin(), sources.end(),
                                 [&](const auto& s) { return s.name == name; });
    return it == sources.end() ? nullptr : &*it;
  };
  const auto make_producer = [](bool explicit_ds, bool armed,
                                bool hide_unarmed_in_describe,
                                std::atomic<int>& ticks) {
    return [explicit_ds, armed, hide_unarmed_in_describe,
            &ticks](Node& node) -> detail::Instance {
      detail::NodeAccess::add_internal_driver_station_subscription(
          node, introspect::naming::kDriverStationTopic, sizeof(Count),
          alignof(Count), schema::Of<Count>(), {});
      if (explicit_ds) detail::NodeAccess::mark_driver_station_reader(node);
      const bool use_arming =
          armed || (hide_unarmed_in_describe && node.describing());
      detail::NodeAccess::add_target_lane_publisher(
          node, "/mechanism/target/teleop", sizeof(Count), kMessageAlignment,
          schema::Of<Count>(), use_arming);
      detail::NodeAccess::set_update(node, [&ticks] {
        ticks.fetch_add(1);
        process::stop_requested.store(true);
      });
      return std::make_shared<int>(0);
    };
  };

  {
    SCOPED_TRACE("--describe: armed explicit DS emits operator_arming only");
    std::atomic<int> ticks{0};
    const CapturedRun probe = RunInProcess(
        "--describe " + ConfigFlag(),
        make_producer(/*explicit_ds=*/true, /*armed=*/true, false, ticks));
    ASSERT_EQ(probe.status, 0) << probe.err;
    EXPECT_EQ(ticks.load(), 0);
    EXPECT_NE(probe.out.find("\"operator_arming\": true"), std::string::npos)
        << probe.out;
    EXPECT_EQ(probe.out.find("\"internal_driver_station\": true"),
              std::string::npos)
        << probe.out;
    introspect::Description described;
    std::string error;
    ASSERT_TRUE(introspect::DescribeReader::Parse(probe.out, described, error))
        << error;
    const auto* ds =
        find_source(described.sources, introspect::naming::kDriverStationTopic);
    const auto* lane =
        find_source(described.sources, "/mechanism/target/teleop");
    ASSERT_NE(ds, nullptr);
    ASSERT_NE(lane, nullptr);
    EXPECT_EQ(ds->flags & introspect::naming::kSourceFlagInternalDriverStation,
              0u);
    EXPECT_NE(lane->flags & introspect::naming::kSourceFlagOperatorArming, 0u);
  }

  {
    SCOPED_TRACE("--describe: internal-only DS coordinator emits flag");
    std::atomic<int> ticks{0};
    const CapturedRun probe = RunInProcess(
        "--describe " + ConfigFlag(),
        make_producer(/*explicit_ds=*/false, /*armed=*/false, false, ticks));
    ASSERT_EQ(probe.status, 0) << probe.err;
    EXPECT_NE(probe.out.find("\"internal_driver_station\": true"),
              std::string::npos)
        << probe.out;
    EXPECT_EQ(probe.out.find("\"operator_arming\": true"), std::string::npos)
        << probe.out;
    introspect::Description described;
    std::string error;
    ASSERT_TRUE(introspect::DescribeReader::Parse(probe.out, described, error))
        << error;
    const auto* ds =
        find_source(described.sources, introspect::naming::kDriverStationTopic);
    const auto* lane =
        find_source(described.sources, "/mechanism/target/teleop");
    ASSERT_NE(ds, nullptr);
    ASSERT_NE(lane, nullptr);
    EXPECT_NE(ds->flags & introspect::naming::kSourceFlagInternalDriverStation,
              0u);
    EXPECT_EQ(lane->flags & introspect::naming::kSourceFlagOperatorArming, 0u);
  }

  {
    SCOPED_TRACE(
        "--describe does not throw on unarmed operator node; LintGraph errors");
    std::atomic<int> ticks{0};
    const CapturedRun probe = RunInProcess(
        "--describe " + ConfigFlag(),
        make_producer(/*explicit_ds=*/true, /*armed=*/false, false, ticks));
    ASSERT_EQ(probe.status, 0) << probe.err;
    introspect::Description described;
    std::string error;
    ASSERT_TRUE(introspect::DescribeReader::Parse(probe.out, described, error))
        << error;
    const auto* ds =
        find_source(described.sources, introspect::naming::kDriverStationTopic);
    const auto* lane =
        find_source(described.sources, "/mechanism/target/teleop");
    ASSERT_NE(ds, nullptr);
    ASSERT_NE(lane, nullptr);
    EXPECT_EQ(ds->flags & introspect::naming::kSourceFlagInternalDriverStation,
              0u);
    EXPECT_EQ(lane->flags & introspect::naming::kSourceFlagOperatorArming, 0u);

    const std::vector<introspect::naming::NodeShape> graph = {
        {"hardware_node",
         "//talOS/bridge:hardware_node",
         {{event::SourceKind::SENDER,
           std::string{introspect::naming::kDriverStationTopic}, sizeof(Count),
           0, "node_test.Count", Descriptor<Count>::kHash, alignof(Count),
           1024}},
         /*hardware_bridge=*/true},
        {described.name, "//talOS/node:test_node", described.sources},
        {"mechanism",
         "//robot/mechanism:node",
         {{event::SourceKind::WATCHER, "/mechanism/target/teleop",
           sizeof(Count), 0, "node_test.Count", Descriptor<Count>::kHash,
           kMessageAlignment, 1024}}},
    };
    const auto diagnostics = introspect::naming::LintGraph(
        graph, {{"mechanism", {"teleop"}}},
        {{"mechanism", {{"teleop", {"teleop"}}}}});
    ASSERT_TRUE(introspect::naming::HasError(diagnostics));
    const auto err_it =
        std::find_if(diagnostics.begin(), diagnostics.end(), [](const auto& d) {
          return d.severity == introspect::naming::Severity::ERROR &&
                 d.subject == "/mechanism/target/teleop" &&
                 d.message.find("OperatorArming") != std::string::npos;
        });
    EXPECT_NE(err_it, diagnostics.end());
  }

  {
    SCOPED_TRACE(
        "hidden unarmed lane behind !node.describing() passes --describe but "
        "exits 1 with 0 ticks on live standalone run");
    std::atomic<int> ticks{0};
    const CapturedRun probe =
        RunInProcess("--describe " + ConfigFlag(),
                     make_producer(/*explicit_ds=*/true, /*armed=*/false,
                                   /*hide_unarmed_in_describe=*/true, ticks));
    ASSERT_EQ(probe.status, 0) << probe.err;
    EXPECT_NE(probe.out.find("\"operator_arming\": true"), std::string::npos)
        << probe.out;

    ticks.store(0);
    const CapturedRun live =
        RunInProcess("--sim --duration-s 5 --log '" + log + "' " + ConfigFlag(),
                     make_producer(/*explicit_ds=*/true, /*armed=*/false,
                                   /*hide_unarmed_in_describe=*/true, ticks));
    EXPECT_EQ(live.status, 1) << live.out << "\n" << live.err;
    EXPECT_EQ(ticks.load(), 0) << "refused node must never tick";
    EXPECT_NE(live.err.find("/mechanism/target/teleop"), std::string::npos)
        << live.err;
    EXPECT_NE(live.err.find("OperatorArming"), std::string::npos) << live.err;
    EXPECT_NE(live.err.find("REQ-SAFE-14"), std::string::npos) << live.err;
  }

  {
    SCOPED_TRACE("live registry carries operator_arming & internal DS flags");
    const auto read_live_sources =
        []() -> std::vector<introspect::naming::SourceShape> {
      const auto reader = introspect::RegistryReader::open();
      if (!reader) return {};
      std::uint64_t best_gen = 0;
      std::vector<introspect::naming::SourceShape> out;
      for (const auto& row : reader->nodes()) {
        if (row.name != "test_node" ||
            row.pid != static_cast<std::uint64_t>(::getpid()) || !row.alive ||
            row.generation < best_gen) {
          continue;
        }
        best_gen = row.generation;
        out.clear();
        for (const auto& s : row.sources) {
          out.push_back(
              {s.kind, s.name, s.message_bytes, s.flags, {}, 0, s.alignment});
        }
      }
      return out;
    };

    for (const bool explicit_ds : {true, false}) {
      SCOPED_TRACE(explicit_ds ? "explicit armed operator"
                               : "internal-DS coordinator");
      std::vector<introspect::naming::SourceShape> observed;
      const CapturedRun run = RunInProcess(
          "--sim --duration-s 5 --log '" + log + "' " + ConfigFlag(),
          [explicit_ds, &observed,
           &read_live_sources](Node& node) -> detail::Instance {
            detail::NodeAccess::add_internal_driver_station_subscription(
                node, introspect::naming::kDriverStationTopic, sizeof(Count),
                alignof(Count), schema::Of<Count>(), {});
            if (explicit_ds) {
              detail::NodeAccess::mark_driver_station_reader(node);
            }
            detail::NodeAccess::add_target_lane_publisher(
                node, "/mechanism/target/teleop", sizeof(Count),
                kMessageAlignment, schema::Of<Count>(),
                /*operator_arming=*/explicit_ds);
            detail::NodeAccess::set_update(
                node, [&observed, &read_live_sources] {
                  observed = read_live_sources();
                  process::stop_requested.store(true);
                });
            return std::make_shared<int>(0);
          });
      ASSERT_EQ(run.status, 0) << run.out << "\n" << run.err;
      const auto* ds =
          find_source(observed, introspect::naming::kDriverStationTopic);
      const auto* lane = find_source(observed, "/mechanism/target/teleop");
      ASSERT_NE(ds, nullptr);
      ASSERT_NE(lane, nullptr);
      EXPECT_EQ((ds->flags &
                 introspect::naming::kSourceFlagInternalDriverStation) != 0u,
                !explicit_ds);
      EXPECT_EQ(
          (lane->flags & introspect::naming::kSourceFlagOperatorArming) != 0u,
          explicit_ds);
      ::shm_unlink(
          ShmObjectName(introspect::naming::kDriverStationTopic).c_str());
      ::shm_unlink(ShmObjectName("/mechanism/target/teleop").c_str());
    }
  }

  std::filesystem::remove(log);
}

// A real running Node's live registry row reports each Target's policy sources
// and selected source ("teleop", then nullopt after release), along with its
// realtime outcome and event-log state, via node.cc's Selections() -> Reporter.
// REQ-OPS-18
TEST(Binary, LiveRegistryRowReportsSelectedTargetSourceAndRelease) {
  const std::string log = TempFile("live_target_selection.tlog");
  detail::reporter_options_for_test = [](introspect::Reporter::Options& o) {
    o.period = 5ms;
  };

  const auto read_live_row = []() -> std::optional<introspect::NodeSnapshot> {
    const auto reader = introspect::RegistryReader::open();
    if (!reader) return std::nullopt;
    std::uint64_t best_gen = 0;
    std::optional<introspect::NodeSnapshot> found;
    for (const auto& row : reader->nodes()) {
      if (row.name != "test_node" ||
          row.pid != static_cast<std::uint64_t>(::getpid()) || !row.alive ||
          row.generation < best_gen) {
        continue;
      }
      best_gen = row.generation;
      found = row;
    }
    return found;
  };

  bool seen_teleop = false;
  bool seen_released = false;
  std::vector<std::string> seen_sources;
  std::optional<process::RealtimeOutcome> seen_realtime;
  std::uint32_t seen_flags = 0;

  const CapturedRun run = RunInProcess(
      "--sim --duration-s 5 --log '" + log + "' " + ConfigFlag(),
      [&](Node& node) -> detail::Instance {
        const std::size_t sel =
            detail::NodeAccess::add_selection(node, {"teleop", "auto"});
        detail::NodeAccess::set_selected(node, sel, 0);
        detail::NodeAccess::set_update(node, [&, sel] {
          const auto row = read_live_row();
          if (row.has_value() && row->selections.size() == 1u) {
            const auto& snap = row->selections[0];
            if (!seen_teleop && snap.selected_source() ==
                                    std::optional<std::string>{"teleop"}) {
              seen_teleop = true;
              seen_sources = snap.sources;
              seen_realtime = row->realtime();
              seen_flags = row->flags;
              detail::NodeAccess::set_selected(node, sel, -1);
            } else if (seen_teleop && !snap.selected_source().has_value()) {
              seen_released = true;
              process::stop_requested.store(true);
            }
          }
          if (node.tick() >= 400u) {
            process::stop_requested.store(true);
          }
        });
        return std::make_shared<int>(0);
      });
  detail::reporter_options_for_test = nullptr;
  std::filesystem::remove(log);

  ASSERT_EQ(run.status, 0) << run.out << "\n" << run.err;
  EXPECT_TRUE(seen_teleop) << "registry never reported selected 'teleop'";
  EXPECT_TRUE(seen_released) << "registry never reported released (nullopt)";
  EXPECT_EQ(seen_sources, (std::vector<std::string>{"teleop", "auto"}));
  ASSERT_TRUE(seen_realtime.has_value());
  EXPECT_EQ(seen_realtime->tier, process::LoopTier::kObserver);
  EXPECT_FALSE(seen_realtime->requested);
  EXPECT_EQ(seen_flags &
                (introspect::kFlagRecorderFailed | introspect::kFlagLogCapped),
            0u);
}

// --replay refuses a log with missing mid-records (damaged record between
// intact ones) or a first dispatch != 1 before constructing the node (zero
// constructor side effects), naming the log and exiting 1; on a truncated
// (no EXIT) log it replays every dispatch before the cut, exits 1 with
// diverged=0, and names the log and the cut dispatch.
// REQ-OPS-13
TEST(Binary, ReplayRefusesDamagedOrLaterSegmentAndStopsAtTruncation) {
  const std::string log = TempFile("talos_node_test_replay_edges.tlog");
  const Result recorded =
      RunBinary("--sim --duration-s 1 --log '" + log + "' " + ConfigFlag());
  ASSERT_EQ(recorded.status, 0) << recorded.output;
  ::shm_unlink(ShmObjectName("/test_node/frames").c_str());

  std::vector<char> bytes;
  {
    std::ifstream in{log, std::ios::binary};
    bytes.assign(std::istreambuf_iterator<char>{in}, {});
  }
  ASSERT_GE(bytes.size(), sizeof(event::log::FileHeader));
  event::log::FileHeader file_header{};
  std::memcpy(&file_header, bytes.data(), sizeof(file_header));
  const std::size_t records_start =
      file_header.header_bytes +
      static_cast<std::size_t>(file_header.manifest_count) *
          sizeof(event::log::ManifestEntry) +
      static_cast<std::size_t>(file_header.schema_bytes);
  ASSERT_LT(records_start, bytes.size());

  struct IndexedRecord {
    std::size_t offset;
    event::log::RecordHeader header;
  };
  std::vector<IndexedRecord> records;
  for (std::size_t pos = records_start;
       pos + sizeof(event::log::RecordHeader) <= bytes.size();) {
    event::log::RecordHeader header{};
    std::memcpy(&header, bytes.data() + pos, sizeof(header));
    ASSERT_EQ(header.magic, event::log::RECORD_MAGIC) << "at offset " << pos;
    records.push_back({pos, header});
    pos += sizeof(event::log::RecordHeader) + header.stored_bytes;
  }
  ASSERT_GT(records.size(), 10u);

  int constructed = 0;
  const detail::InstanceFactory probe_factory =
      [&constructed](Node&) -> detail::Instance {
    ++constructed;
    return std::make_shared<int>(0);
  };

  {
    SCOPED_TRACE("missing mid-records (damaged record)");
    std::vector<char> damaged = bytes;
    const std::size_t mid = records.size() / 2;
    damaged[records[mid].offset +
            offsetof(event::log::RecordHeader, dispatch_index)] ^= 1;
    const std::string damaged_path = TempFile("talos_node_test_damaged.tlog");
    {
      std::ofstream out{damaged_path, std::ios::binary | std::ios::trunc};
      out.write(damaged.data(), static_cast<std::streamsize>(damaged.size()));
    }
    constructed = 0;
    const CapturedRun run = RunInProcess(
        "--replay '" + damaged_path + "' " + ConfigFlag(), probe_factory);
    EXPECT_EQ(run.status, 1) << run.out << "\n" << run.err;
    EXPECT_EQ(constructed, 0) << "constructor must not run on damaged log";
    EXPECT_NE(run.err.find(damaged_path), std::string::npos) << run.err;
    std::filesystem::remove(damaged_path);
  }

  {
    SCOPED_TRACE("start dispatch != 1 (later rotated segment)");
    std::size_t later = 0;
    while (later < records.size() && records[later].header.dispatch_index < 5) {
      ++later;
    }
    ASSERT_LT(later, records.size());
    const std::uint64_t start_dispatch = records[later].header.dispatch_index;
    const std::string later_path = TempFile("talos_node_test_later_seg.tlog");
    {
      std::ofstream out{later_path, std::ios::binary | std::ios::trunc};
      out.write(bytes.data(), static_cast<std::streamsize>(records_start));
      out.write(
          bytes.data() + records[later].offset,
          static_cast<std::streamsize>(bytes.size() - records[later].offset));
    }
    constructed = 0;
    const CapturedRun run = RunInProcess(
        "--replay '" + later_path + "' " + ConfigFlag(), probe_factory);
    EXPECT_EQ(run.status, 1) << run.out << "\n" << run.err;
    EXPECT_EQ(constructed, 0)
        << "constructor must not run when first dispatch != 1";
    EXPECT_NE(run.err.find(later_path), std::string::npos) << run.err;
    EXPECT_NE(run.err.find("dispatch " + std::to_string(start_dispatch)),
              std::string::npos)
        << run.err;
    std::filesystem::remove(later_path);
  }

  {
    SCOPED_TRACE("truncated (no EXIT) log");
    std::size_t cut = records.size() - 1;
    while (cut > 0 && records[cut].header.kind !=
                          static_cast<std::uint16_t>(event::EventKind::SEND)) {
      --cut;
    }
    ASSERT_GT(cut, 0u);
    const std::uint64_t cut_dispatch = records[cut].header.dispatch_index;
    ASSERT_GT(cut_dispatch, 1u);
    const std::string truncated_path =
        TempFile("talos_node_test_truncated.tlog");
    {
      std::ofstream out{truncated_path, std::ios::binary | std::ios::trunc};
      out.write(bytes.data(),
                static_cast<std::streamsize>(records[cut].offset + 10));
    }
    const Result replay =
        RunBinary("--replay '" + truncated_path + "' " + ConfigFlag());
    EXPECT_EQ(replay.status, 1) << replay.output;
    EXPECT_NE(replay.output.find("diverged=0 clean_exit=0 stopped_at_gap=0"),
              std::string::npos)
        << replay.output;
    EXPECT_NE(replay.output.find(truncated_path), std::string::npos)
        << replay.output;
    EXPECT_NE(replay.output.find("in dispatch " + std::to_string(cut_dispatch)),
              std::string::npos)
        << replay.output;
    std::filesystem::remove(truncated_path);
  }

  std::filesystem::remove(log);
}

// --replay-inputs is offline: it creates no shared-memory segment (topics or
// registry) in a fresh namespace and leaves a live sentinel RTMSQueue
// untouched.
// REQ-OPS-14
TEST(Binary, ReplayInputsTouchesNoSharedMemoryAndLeavesSentinelUntouched) {
  const std::string dir = TempFile("talos_node_test_replay_inputs_shm");
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  const Result recorded = RunBinary("--sim --duration-s 1 --log '" + dir +
                                    "/test_node.tlog' " + ConfigFlag());
  ASSERT_EQ(recorded.status, 0) << recorded.output;
  ::shm_unlink(ShmObjectName("/test_node/frames").c_str());

  const std::string out_fresh =
      TempFile("talos_node_test_replay_inputs_fresh.tlog");
  const std::string out_sentinel =
      TempFile("talos_node_test_replay_inputs_sentinel.tlog");

  {
    SCOPED_TRACE("fresh namespace has zero SHM segments after --replay-inputs");
    const std::string fresh_ns =
        "replay_inputs_no_shm_" + std::to_string(::getpid());
    struct ScopedNamespace {
      std::optional<std::string> previous;
      explicit ScopedNamespace(const std::string& ns) {
        if (const char* value = std::getenv("TALOS_SHM_NAMESPACE")) {
          previous = value;
        }
        ::setenv("TALOS_SHM_NAMESPACE", ns.c_str(), 1);
      }
      ~ScopedNamespace() {
        if (previous) {
          ::setenv("TALOS_SHM_NAMESPACE", previous->c_str(), 1);
        } else {
          ::unsetenv("TALOS_SHM_NAMESPACE");
        }
      }
    } scoped_ns{fresh_ns};

    const char* const topics[] = {"/test_node/target", "/test_node/state",
                                  "/test_node/frames",
                                  introspect::kRegistryPath};
    for (const char* name : topics) {
      ::shm_unlink(ShmObjectName(name, fresh_ns).c_str());
    }
    const Result replay = RunBinary("--replay-inputs '" + dir + "' --log '" +
                                    out_fresh + "' " + ConfigFlag());
    ASSERT_EQ(replay.status, 0) << replay.output;
    for (const char* name : topics) {
      const std::string shm = ShmObjectName(name, fresh_ns);
      errno = 0;
      const int fd = ::shm_open(shm.c_str(), O_RDONLY, 0);
      if (fd >= 0) {
        ::close(fd);
        ::shm_unlink(shm.c_str());
      }
      EXPECT_EQ(fd, -1) << name << " (" << shm
                        << ") created by --replay-inputs";
      EXPECT_EQ(errno, ENOENT) << name << " (" << shm << ")";
    }
  }

  {
    SCOPED_TRACE("sentinel RTMSQueue stays untouched");
    RTMSQueue frames_sentinel{
        "/test_node/frames",
        64,
        kMessageAlignment,
        schema::Bytes(),
        16,
        RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                    .read_mode = ReadMode::SEQUENCE,
                    .publisher = true}};
    RTMSQueue state_sentinel{
        "/test_node/state",
        sizeof(Count),
        alignof(Count),
        schema::Of<Count>(),
        MAX_SLOTS,
        RTMSOptions{.overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
                    .read_mode = ReadMode::SEQUENCE,
                    .publisher = true}};
    const std::uint64_t frames_seq = frames_sentinel.writer_sequence();
    const std::uint64_t state_seq = state_sentinel.writer_sequence();
    const Result replay = RunBinary("--replay-inputs '" + dir + "' --log '" +
                                    out_sentinel + "' " + ConfigFlag());
    EXPECT_EQ(replay.status, 0) << replay.output;
    EXPECT_EQ(frames_sentinel.writer_sequence(), frames_seq);
    EXPECT_EQ(state_sentinel.writer_sequence(), state_seq);
  }
  ::shm_unlink(ShmObjectName("/test_node/frames").c_str());
  ::shm_unlink(ShmObjectName("/test_node/state").c_str());
  std::filesystem::remove(out_fresh);
  std::filesystem::remove(out_sentinel);
  std::filesystem::remove_all(dir);
}

}  // namespace
}  // namespace talos

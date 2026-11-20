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
#include "talOS/events/log/gap_testing.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/introspection/describe.h"
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
// still are, and a changed parameter diverges.
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
  EXPECT_NE(diverged.status, 0) << diverged.output;
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

}  // namespace
}  // namespace talos

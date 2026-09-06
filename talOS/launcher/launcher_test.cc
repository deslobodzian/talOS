#include "talOS/launcher/launcher.h"

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "talOS/events/log/log_writer.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"

namespace talos::launcher {
namespace {

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_launcher_test_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

// A framework-owned fixture: the launcher's discovery logic is robot-agnostic,
// so its tests must not read any particular robot's configuration.
constexpr std::string_view kFixtureToml = R"(
[robot]
name = "launcher_fixture"
period_us = 5000
sim_gateway = "//example/sim:gateway"

[subsystems.alpha]
node = "//example/alpha:node"
period_us = 5000

[subsystems.alpha.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.beta]
node = "//example/beta:node"
period_us = 10000

[subsystems.beta.motors.m2]
type = "TalonFX"
bus = "rio"
can_id = 2
)";

inline std::string WriteFixtureToml() {
  const std::string path = TempPath("launcher_fixture_toml") + ".toml";
  std::ofstream out{path};
  out << kFixtureToml;
  return path;
}

event::Manifest MakeTestManifest(std::string_view name) {
  event::Manifest manifest;
  event::Registration reg;
  reg.id = 1;
  reg.kind = event::SourceKind::TIMER;
  reg.name = std::string{name};
  reg.message_bytes = 8;
  reg.period_ns = 1'000'000;
  manifest.push_back(reg);
  return manifest;
}

void WriteTestLog(const std::string& path, const std::string& process_name,
                  uint64_t session_id,
                  const std::vector<int64_t>& event_times_ns) {
  event::log::LogWriter writer{
      path, process_name,
      event::log::LogWriterOptions{4096, 1, /*background=*/false}, session_id};
  const auto manifest = MakeTestManifest(process_name + "_src");
  writer.start(manifest,
               event::MonotonicTime::from_nanos(
                   event_times_ns.empty() ? 0 : event_times_ns.front()));

  for (std::size_t i = 0; i < event_times_ns.size(); ++i) {
    event::Context ctx{};
    ctx.kind = event::EventKind::TIMER;
    ctx.source_id = 1;
    ctx.dispatch_index = i + 1;
    ctx.event_time = event::MonotonicTime::from_nanos(event_times_ns[i]);
    ctx.now = event::MonotonicTime::from_nanos(event_times_ns[i] + 100);
    ctx.sequence = i + 1;

    uint64_t payload_val = i;
    std::span<const std::byte> payload{
        reinterpret_cast<const std::byte*>(&payload_val), sizeof(payload_val)};
    writer.dispatch(ctx, payload);
  }

  writer.flush();
  ASSERT_FALSE(writer.failed()) << writer.error();
}

TEST(LauncherTest, SessionIdGenerationIsUniqueAndNonZero) {
  uint64_t id1 = GenerateSessionId();
  uint64_t id2 = GenerateSessionId();
  uint64_t id3 = GenerateSessionId();

  EXPECT_NE(id1, 0u);
  EXPECT_NE(id2, 0u);
  EXPECT_NE(id3, 0u);

  EXPECT_NE(id1, id2);
  EXPECT_NE(id2, id3);
  EXPECT_NE(id1, id3);
}

TEST(LauncherTest, DiscoverNodesFromRobotToml) {
  const auto robot_cfg = config::ParseRobotConfigString(kFixtureToml);
  LauncherOptions options;
  auto nodes = DiscoverNodes(robot_cfg, options);

  ASSERT_GE(nodes.size(), 2u);

  bool found_hw = false;
  bool found_alpha = false;
  bool found_beta = false;

  for (const auto& n : nodes) {
    if (n.name == "hardware_node") {
      found_hw = true;
      EXPECT_EQ(n.target, "//talOS/bridge:hardware_node");
    } else if (n.name == "alpha") {
      found_alpha = true;
      EXPECT_EQ(n.target, "//example/alpha:node");
    } else if (n.name == "beta") {
      found_beta = true;
      EXPECT_EQ(n.target, "//example/beta:node");
    }
  }

  EXPECT_TRUE(found_hw);
  EXPECT_TRUE(found_alpha);
  EXPECT_TRUE(found_beta);
}

TEST(LauncherTest, SimGatewayComesFromRobotConfigNotTheFramework) {
  LauncherOptions options;
  options.simulation = true;
  options.start_sim_gateway = true;

  auto with_gw =
      DiscoverNodes(config::ParseRobotConfigString(kFixtureToml), options);
  EXPECT_EQ(
      std::count_if(with_gw.begin(), with_gw.end(),
                    [](const NodeSpec& n) { return n.name == "sim_gateway"; }),
      1);

  // No [robot] sim_gateway key: the launcher must not invent a robot target.
  constexpr std::string_view kNoGateway = R"(
[robot]
name = "no_gateway"
period_us = 5000

[subsystems.alpha]
node = "//example/alpha:node"

[subsystems.alpha.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
)";
  auto without_gw =
      DiscoverNodes(config::ParseRobotConfigString(kNoGateway), options);
  EXPECT_EQ(
      std::count_if(without_gw.begin(), without_gw.end(),
                    [](const NodeSpec& n) { return n.name == "sim_gateway"; }),
      0);
}

TEST(LauncherTest, MergedLogsMonotonicOrder) {
  const std::string path_a = TempPath("log_a.tlog");
  const std::string path_b = TempPath("log_b.tlog");
  const uint64_t session_id = 0xCAFEBABE12345678ULL;

  // Proc A writes events at 1ms, 3ms, 5ms
  WriteTestLog(path_a, "proc_a", session_id, {1'000'000, 3'000'000, 5'000'000});

  // Proc B writes events at 2ms, 4ms, 6ms
  WriteTestLog(path_b, "proc_b", session_id, {2'000'000, 4'000'000, 6'000'000});

  std::vector<MergedRecord> merged;
  std::string error;
  ASSERT_TRUE(MergeLogs({path_a, path_b}, merged, error)) << error;

  ASSERT_EQ(merged.size(), 6u);

  // Check monotonic order and interleaving
  EXPECT_EQ(merged[0].process_name, "proc_a");
  EXPECT_EQ(merged[0].header.event_time_ns, 1'000'000);

  EXPECT_EQ(merged[1].process_name, "proc_b");
  EXPECT_EQ(merged[1].header.event_time_ns, 2'000'000);

  EXPECT_EQ(merged[2].process_name, "proc_a");
  EXPECT_EQ(merged[2].header.event_time_ns, 3'000'000);

  EXPECT_EQ(merged[3].process_name, "proc_b");
  EXPECT_EQ(merged[3].header.event_time_ns, 4'000'000);

  EXPECT_EQ(merged[4].process_name, "proc_a");
  EXPECT_EQ(merged[4].header.event_time_ns, 5'000'000);

  EXPECT_EQ(merged[5].process_name, "proc_b");
  EXPECT_EQ(merged[5].header.event_time_ns, 6'000'000);

  std::remove(path_a.c_str());
  std::remove(path_b.c_str());
}

TEST(LauncherTest, MergedLogsRejectsSessionIdMismatch) {
  const std::string path_a = TempPath("mismatch_a.tlog");
  const std::string path_b = TempPath("mismatch_b.tlog");

  WriteTestLog(path_a, "proc_a", 1001ULL, {1'000'000});
  WriteTestLog(path_b, "proc_b", 2002ULL, {2'000'000});

  std::vector<MergedRecord> merged;
  std::string error;
  bool ok = MergeLogs({path_a, path_b}, merged, error);

  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("session_id mismatch"), std::string::npos);

  std::remove(path_a.c_str());
  std::remove(path_b.c_str());
}

TEST(LauncherTest, SessionManifestJsonFormatting) {
  SessionManifest manifest;
  manifest.session_id = 987654321ULL;
  manifest.config_path = "2026-robot/main_processor/configuration/robot.toml";
  manifest.output_dir = "/tmp/test_dir";
  manifest.simulation = true;
  manifest.start_wall_ns = 1'000'000'000LL;
  manifest.end_wall_ns = 2'000'000'000LL;

  NodeProcessInfo node1;
  node1.name = "drivetrain";
  node1.target = "//2026-robot/main_processor/drivetrain:node";
  node1.binary_path = "bazel-bin/2026-robot/main_processor/drivetrain/node";
  node1.log_path = "/tmp/test_dir/drivetrain.tlog";
  node1.pid = 1234;
  node1.exit_code = 0;
  node1.record_count = 50;
  node1.failed = false;

  manifest.nodes.push_back(node1);

  std::string json = manifest.ToJson();
  EXPECT_NE(json.find("\"session_id\": 987654321"), std::string::npos);
  EXPECT_NE(json.find("\"name\": \"drivetrain\""), std::string::npos);
  EXPECT_NE(json.find("\"pid\": 1234"), std::string::npos);
  EXPECT_NE(json.find("\"exit_code\": 0"), std::string::npos);
  EXPECT_NE(json.find("\"record_count\": 50"), std::string::npos);
}

TEST(LauncherTest, RunsChildProcessesAndWritesManifest) {
  const std::string out_dir = TempPath("launcher_run_out");
  std::filesystem::create_directories(out_dir);

  LauncherOptions options;
  options.output_dir = out_dir;
  options.session_id = 0x55555555ULL;
  options.duration_s = 1;

  // Use /usr/bin/true (or /bin/echo) for all nodes so the test executes
  // instantly and cleanly
  std::string mock_bin =
      ::access("/usr/bin/true", X_OK) == 0 ? "/usr/bin/true" : "/bin/echo";
  options.config_path = WriteFixtureToml();
  options.binary_overrides["//talOS/bridge:hardware_node"] = mock_bin;
  options.binary_overrides["//example/alpha:node"] = mock_bin;
  options.binary_overrides["//example/beta:node"] = mock_bin;

  Launcher launcher{options};
  int code = launcher.Run();
  EXPECT_EQ(code, 0);

  std::string manifest_path = out_dir + "/manifest.json";
  EXPECT_TRUE(std::filesystem::exists(manifest_path));

  std::ifstream mf(manifest_path);
  std::string content((std::istreambuf_iterator<char>(mf)),
                      std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("\"session_id\":"), std::string::npos);
  EXPECT_NE(content.find("\"nodes\":"), std::string::npos);

  std::filesystem::remove_all(out_dir);
}

// A stand-in for a node binary.
//
// What the launcher wants from a binary is what it prints when asked what it
// is, so a script that prints it is a complete node for this purpose -- and the
// graph under test stays in the test instead of in some other package's source,
// where it would drift.
std::string WriteStubNode(std::string_view name, std::string_view script) {
  const std::string path = TempPath(name);
  {
    std::ofstream out{path};
    out << script;
  }
  ::chmod(path.c_str(), 0755);
  return path;
}

// The JSON a real node prints, written by the real writer: a stub that answered
// with hand-typed JSON would pass the day describe.h changed shape.
std::string DescribeJson(
    std::string_view node, std::string_view target,
    const std::vector<introspect::naming::SourceShape>& sources) {
  introspect::Description description;
  description.name = std::string{node};
  description.target = std::string{target};
  description.sources = sources;
  return introspect::DescribeToJson(description);
}

std::string DescribingStub(
    std::string_view node, std::string_view target,
    const std::vector<introspect::naming::SourceShape>& sources) {
  return std::string{"#!/bin/sh\n"} +
         "for arg in \"$@\"; do\n"
         "  if [ \"$arg\" = \"--describe\" ]; then\n"
         "    cat <<'TALOS_DESCRIBE'\n" +
         DescribeJson(node, target, sources) +
         "TALOS_DESCRIBE\n"
         "    exit 0\n"
         "  fi\n"
         "done\n"
         "exit 0\n";
}

// A node written before --describe existed, or one halfway through being
// written: it rejects the flag the way every node used to.
constexpr std::string_view kStubWithoutDescribe =
    "#!/bin/sh\n"
    "for arg in \"$@\"; do\n"
    "  if [ \"$arg\" = \"--describe\" ]; then\n"
    "    echo \"usage: node [--sim] [--log PATH]\" >&2\n"
    "    exit 1\n"
    "  fi\n"
    "done\n"
    "exit 0\n";

// exec so the pid the launcher kills is the one that is sleeping; a shell that
// forked the sleep would report the kill and leave the sleep behind.
constexpr std::string_view kStubThatHangsOnDescribe =
    "#!/bin/sh\n"
    "for arg in \"$@\"; do\n"
    "  if [ \"$arg\" = \"--describe\" ]; then\n"
    "    exec sleep 30\n"
    "  fi\n"
    "done\n"
    "exit 0\n";

introspect::naming::SourceShape Sender(std::string topic,
                                       std::uint32_t bytes = 48) {
  return {event::SourceKind::SENDER, std::move(topic), bytes, 0};
}
introspect::naming::SourceShape Watcher(std::string topic,
                                        std::uint32_t bytes = 48) {
  return {event::SourceKind::WATCHER, std::move(topic), bytes, 0};
}

std::string ReadFile(const std::string& path) {
  std::ifstream in{path};
  return std::string{std::istreambuf_iterator<char>(in),
                     std::istreambuf_iterator<char>()};
}

// Two stubs and a hardware node that says nothing, wired to the fixture config.
struct GraphFixture {
  LauncherOptions options;
  std::string out_dir;

  GraphFixture(std::string_view tag, std::string_view alpha_script,
               std::string_view beta_script) {
    out_dir = TempPath(std::string{tag} + "_out");
    std::filesystem::create_directories(out_dir);
    options.output_dir = out_dir;
    options.session_id = 0x0123456789ABCDEFULL;
    options.config_path = WriteFixtureToml();
    options.describe_timeout_ms = 4000;
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        ::access("/usr/bin/true", X_OK) == 0 ? "/usr/bin/true" : "/bin/echo";
    options.binary_overrides["//example/alpha:node"] =
        WriteStubNode(std::string{tag} + "_alpha", alpha_script);
    options.binary_overrides["//example/beta:node"] =
        WriteStubNode(std::string{tag} + "_beta", beta_script);
  }

  ~GraphFixture() { std::filesystem::remove_all(out_dir); }

  std::string graph_json() const { return ReadFile(out_dir + "/graph.json"); }
  bool has_manifest() const {
    return std::filesystem::exists(out_dir + "/manifest.json");
  }
};

TEST(LauncherGraphTest, ProbesEveryBinaryAndPutsWhatItSaysInTheGraph) {
  GraphFixture fixture{
      "meets",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node", {Watcher("/alpha/state")})};

  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 0);

  const std::string graph = fixture.graph_json();
  EXPECT_NE(graph.find("\"session_id\": \"81985529216486895\""),
            std::string::npos)
      << graph;
  EXPECT_NE(graph.find("\"name\": \"alpha\""), std::string::npos) << graph;
  EXPECT_NE(graph.find("\"kind\": \"SENDER\", \"name\": \"/alpha/state\""),
            std::string::npos)
      << graph;
  EXPECT_NE(graph.find("\"kind\": \"WATCHER\", \"name\": \"/alpha/state\""),
            std::string::npos)
      << graph;
  // A topic with both ends declared is the case nothing should be said about.
  EXPECT_EQ(graph.find("\"severity\": \"error\""), std::string::npos) << graph;

  ASSERT_TRUE(fixture.has_manifest());
  const std::string manifest = ReadFile(fixture.out_dir + "/manifest.json");
  EXPECT_NE(manifest.find("\"described\": true"), std::string::npos)
      << manifest;
}

TEST(LauncherGraphTest, RefusesToSpawnWhenTheTwoEndsAreSpelledDifferently) {
  GraphFixture fixture{
      "differs",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node",
                     {Watcher("/alpha/status")})};

  Launcher launcher{fixture.options};
  EXPECT_NE(launcher.Run(), 0);

  const std::string graph = fixture.graph_json();
  EXPECT_NE(graph.find("nothing publishes it"), std::string::npos) << graph;
  EXPECT_NE(graph.find("\"severity\": \"error\""), std::string::npos) << graph;

  // Nothing was started, so there is no session: the manifest is the evidence.
  EXPECT_FALSE(fixture.has_manifest());
}

TEST(LauncherGraphTest, AllowGraphErrorsLaunchesTheBrokenGraphAnyway) {
  GraphFixture fixture{
      "allowed",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node",
                     {Watcher("/alpha/status")})};
  fixture.options.allow_graph_errors = true;

  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 0);
  EXPECT_TRUE(fixture.has_manifest());
  EXPECT_NE(fixture.graph_json().find("\"severity\": \"error\""),
            std::string::npos);
}

TEST(LauncherGraphTest, ABinaryThatIgnoresDescribeIsAWarningAndStillLaunches) {
  GraphFixture fixture{"silent", kStubWithoutDescribe, kStubWithoutDescribe};

  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 0);
  EXPECT_TRUE(fixture.has_manifest());

  const std::string graph = fixture.graph_json();
  EXPECT_NE(graph.find("\"described\": false"), std::string::npos) << graph;
  EXPECT_NE(graph.find("did not answer --describe"), std::string::npos)
      << graph;
  EXPECT_NE(graph.find("\"severity\": \"warning\""), std::string::npos)
      << graph;
  EXPECT_EQ(graph.find("\"severity\": \"error\""), std::string::npos) << graph;
}

TEST(LauncherGraphTest, ANodeThatHangsOnDescribeDoesNotHangTheLaunch) {
  GraphFixture fixture{
      "hangs", kStubThatHangsOnDescribe,
      DescribingStub("beta", "//example/beta:node", {Sender("/beta/state")})};
  fixture.options.describe_timeout_ms = 250;

  const auto started = std::chrono::steady_clock::now();
  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 0);
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(),
            10);

  EXPECT_NE(fixture.graph_json().find("no answer within 250ms"),
            std::string::npos)
      << fixture.graph_json();
}

TEST(LauncherGraphTest, DescribeOnlyLintsWithoutStartingAnything) {
  GraphFixture fixture{
      "preflight",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node", {Watcher("/alpha/state")})};
  fixture.options.describe_only = true;

  Launcher launcher{fixture.options};
  EXPECT_EQ(launcher.Run(), 0);
  EXPECT_FALSE(fixture.has_manifest());
  EXPECT_NE(fixture.graph_json().find("\"name\": \"beta\""), std::string::npos);
}

TEST(LauncherGraphTest, DescribeOnlyReportsABrokenGraphAsAFailure) {
  GraphFixture fixture{
      "preflight_broken",
      DescribingStub("alpha", "//example/alpha:node", {Sender("/alpha/state")}),
      DescribingStub("beta", "//example/beta:node",
                     {Watcher("/alpha/status")})};
  fixture.options.describe_only = true;

  Launcher launcher{fixture.options};
  EXPECT_NE(launcher.Run(), 0);
  EXPECT_FALSE(fixture.has_manifest());
}

TEST(LauncherGraphTest, WarnsWhenABinaryDisagreesWithTheConfigAboutItsName) {
  NodeSpec spec;
  spec.name = "alpha";
  spec.target = "//example/alpha:node";
  spec.binary_path =
      WriteStubNode("renamed", DescribingStub("gamma", "//example/gamma:node",
                                              {Sender("/gamma/state")}));

  LauncherOptions options;
  options.describe_timeout_ms = 4000;
  std::vector<NodeDescription> described;
  const auto diagnostics = DescribeGraph({spec}, options, described);

  ASSERT_EQ(described.size(), 1u);
  ASSERT_TRUE(described[0].answered) << described[0].error;
  EXPECT_EQ(described[0].declared.name, "gamma");
  EXPECT_EQ(described[0].roster_name, "alpha");

  const bool warned = std::any_of(
      diagnostics.begin(), diagnostics.end(),
      [](const introspect::naming::Diagnostic& d) {
        return d.severity == introspect::naming::Severity::WARNING &&
               d.message.find("describes itself as 'gamma'") !=
                   std::string::npos;
      });
  EXPECT_TRUE(warned) << "diagnostics did not mention the disagreement";

  std::remove(spec.binary_path.c_str());
}

TEST(LauncherGraphTest, ProbeReportsABinaryThatIsNotThere) {
  introspect::Description out;
  std::string error;
  EXPECT_FALSE(ProbeDescribe("/nonexistent/talos_node",
                             {"/nonexistent/talos_node", "--describe"},
                             std::chrono::milliseconds{500}, out, error));
  EXPECT_NE(error.find("no executable"), std::string::npos) << error;
}

}  // namespace
}  // namespace talos::launcher

// Studio bridge live-registry registration, loop-progress reporting, and
// /system.json deduplication (REQ-OPS-30).

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "sim/sim_generated.h"
#include "sim/topics.h"
#include "talOS/events/manifest.h"
#include "talOS/introspection/registry.h"
#include "talOS/process/realtime_outcome.h"
#include "talOS/telemetry/wire.h"

namespace studio {
namespace {

std::string BridgeBinary() {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles{
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error)};
  if (!runfiles) throw std::runtime_error(error);
  const char* binary = std::getenv("TALOS_STUDIO_BRIDGE");
  if (binary == nullptr) throw std::runtime_error("missing studio_bridge");
  return runfiles->Rlocation(binary);
}

int RunToExit(const std::vector<std::string>& args) {
  std::vector<char*> argv;
  argv.reserve(args.size() + 1);
  for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
  argv.push_back(nullptr);
  const pid_t pid = ::fork();
  if (pid == 0) {
    ::execv(argv[0], argv.data());
    ::_exit(127);
  }
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }
  return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

std::string HttpGet(uint16_t port, std::string_view path) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) throw std::runtime_error("socket failed");
  struct timeval timeout {};
  timeout.tv_sec = 5;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    throw std::runtime_error("connect failed");
  }
  const std::string request = "GET " + std::string{path} +
                              " HTTP/1.1\r\nHost: localhost\r\n"
                              "Connection: close\r\n\r\n";
  std::size_t sent = 0;
  while (sent < request.size()) {
    const ssize_t n =
        ::send(fd, request.data() + sent, request.size() - sent, 0);
    if (n <= 0) {
      ::close(fd);
      throw std::runtime_error("send failed");
    }
    sent += static_cast<std::size_t>(n);
  }
  std::string response;
  char buffer[4096];
  for (;;) {
    const ssize_t n = ::recv(fd, buffer, sizeof(buffer), 0);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) break;
    response.append(buffer, static_cast<std::size_t>(n));
  }
  ::close(fd);
  const std::size_t body = response.find("\r\n\r\n");
  return body == std::string::npos ? std::string{} : response.substr(body + 4);
}

std::optional<talos::introspect::NodeSnapshot> FindRow(pid_t pid) {
  const auto reader = talos::introspect::RegistryReader::open();
  if (!reader) return std::nullopt;
  for (const auto& node : reader->nodes()) {
    if (node.pid == static_cast<std::uint64_t>(pid)) return node;
  }
  return std::nullopt;
}

class RegistryCleanup : public ::testing::Environment {
 public:
  void TearDown() override {
    static_cast<void>(talos::introspect::RemoveRegistryUnlessHeld(0));
  }
};

[[maybe_unused]] ::testing::Environment* const kRegistryCleanup =
    ::testing::AddGlobalTestEnvironment(new RegistryCleanup);

// REQ-OPS-30: studio_bridge registers a live registry row when --session-id is
// nonzero, advances dispatch_count and heartbeat_monotonic_ns on each loop
// pass, and replaces its own row with self() in /system.json so exactly one
// studio_bridge entry appears carrying self()'s counters and shape.
TEST(BridgeRegistration, RegistersAdvancesProgressAndDeduplicatesSystemJson) {
  const std::string binary = BridgeBinary();
  const std::string topic = "/test/telemetry/reg" + std::to_string(::getpid());
  const std::string webroot =
      ::testing::TempDir() + "/bridge_reg_" + std::to_string(::getpid());
  std::filesystem::create_directories(webroot);

  // Invalid --session-id is rejected; --describe ignores --session-id.
  EXPECT_EQ(RunToExit({binary, topic, "0", webroot, "--session-id", "-1"}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", webroot, "--session-id", "abc"}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", webroot, "--session-id=abc"}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", webroot, "--session-id"}), 2);
  EXPECT_EQ(RunToExit({binary, "--describe", "--session-id", "12345"}), 0);

  int stdout_pipe[2]{-1, -1};
  ASSERT_EQ(::pipe(stdout_pipe), 0);
  const std::vector<std::string> args = {binary,  topic,          "0",
                                         webroot, "--session-id", "12345"};
  std::vector<char*> argv;
  argv.reserve(args.size() + 1);
  for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
  argv.push_back(nullptr);

  const pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    ::close(stdout_pipe[0]);
    ::dup2(stdout_pipe[1], STDOUT_FILENO);
    ::close(stdout_pipe[1]);
    ::execv(argv[0], argv.data());
    ::_exit(127);
  }
  ::close(stdout_pipe[1]);

  std::string line;
  char ch = '\0';
  while (::read(stdout_pipe[0], &ch, 1) == 1) {
    if (ch == '\n') break;
    line.push_back(ch);
  }
  ::close(stdout_pipe[0]);
  ASSERT_TRUE(line.starts_with("listening ")) << line;
  const auto port = static_cast<uint16_t>(std::stoul(line.substr(10)));

  const auto first = FindRow(pid);
  ASSERT_TRUE(first.has_value()) << "studio_bridge did not register a row";
  EXPECT_EQ(first->name, "studio_bridge");
  EXPECT_EQ(first->target, "//studio/bridge:studio_bridge");
  EXPECT_EQ(first->session_id, 12345u);
  EXPECT_TRUE(first->alive);
  EXPECT_GE(first->generation, 1u);
  ASSERT_TRUE(first->realtime().has_value());
  EXPECT_EQ(first->realtime()->tier, talos::process::LoopTier::kObserver);
  EXPECT_FALSE(first->realtime()->requested);
  // REQ-DEV-43: the telemetry watcher, then the game pieces' optional
  // observer read, as --describe declares them.
  ASSERT_EQ(first->sources.size(), 2u);
  EXPECT_EQ(first->sources[0].kind, talos::event::SourceKind::WATCHER);
  EXPECT_EQ(first->sources[0].name, topic);
  EXPECT_EQ(first->sources[0].message_bytes,
            static_cast<std::uint32_t>(talos::telemetry_wire::slot_bytes));
  EXPECT_EQ(first->sources[1].kind, talos::event::SourceKind::FETCHER);
  EXPECT_EQ(first->sources[1].name, ::sim::topics::kPieces);
  EXPECT_EQ(first->sources[1].message_bytes, sizeof(::sim::GamePieces));
  EXPECT_TRUE(first->sources[1].optional());
  EXPECT_TRUE(first->sources[1].observer());

  // Two HTTP requests ensure at least one post-handler service() pass completes
  // between the first row read and the second.
  ASSERT_FALSE(HttpGet(port, "/system.json").empty());
  const std::string body = HttpGet(port, "/system.json");
  ASSERT_FALSE(body.empty());
  const auto second = FindRow(pid);
  ASSERT_TRUE(second.has_value());
  EXPECT_GT(second->dispatch_count, first->dispatch_count);
  EXPECT_GT(second->heartbeat_monotonic_ns, first->heartbeat_monotonic_ns);

  const auto doc = nlohmann::json::parse(body);
  ASSERT_TRUE(doc.contains("registry"));
  EXPECT_TRUE(doc["registry"]["available"].get<bool>());
  ASSERT_TRUE(doc.contains("nodes"));
  std::size_t bridge_entries = 0;
  nlohmann::json reported;
  for (const auto& node : doc["nodes"]) {
    if (node["name"] == "studio_bridge") {
      ++bridge_entries;
      reported = node;
    }
  }
  ASSERT_EQ(bridge_entries, 1u) << body;
  // The entry must come from self() rather than the raw live registry row:
  // self() reports generation 0 and dispatch_count == forwarded (0 here, while
  // the live registry row has generation >= 1 and dispatch_count == loop_passes
  // >= 1), so dropping self() or skipping deduplication both fail here.
  EXPECT_EQ(reported["pid"].get<std::string>(), std::to_string(pid)) << body;
  EXPECT_EQ(reported["target"].get<std::string>(),
            "//studio/bridge:studio_bridge")
      << body;
  EXPECT_EQ(reported["session_id"].get<std::string>(), "12345") << body;
  EXPECT_TRUE(reported["alive"].get<bool>()) << body;
  EXPECT_EQ(reported["generation"].get<std::string>(), "0") << body;
  EXPECT_EQ(reported["dispatch_count"].get<std::string>(), "0") << body;
  EXPECT_EQ(reported["declared_sources"].get<int>(), 2) << body;
  ASSERT_TRUE(reported.contains("sources") && reported["sources"].size() == 2u)
      << body;
  EXPECT_EQ(reported["sources"][1]["name"].get<std::string>(),
            ::sim::topics::kPieces)
      << body;
  const auto& source = reported["sources"][0];
  EXPECT_EQ(source["kind"].get<std::string>(), "WATCHER") << body;
  EXPECT_EQ(source["name"].get<std::string>(), topic) << body;
  EXPECT_EQ(source["message_bytes"].get<std::uint32_t>(),
            static_cast<std::uint32_t>(talos::telemetry_wire::slot_bytes))
      << body;
  EXPECT_EQ(source["events"].get<std::string>(), "0") << body;
  EXPECT_EQ(source["dropped"].get<std::string>(), "0") << body;
  EXPECT_EQ(source["sequence"].get<std::string>(), "0") << body;

  ::kill(pid, SIGTERM);
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  EXPECT_FALSE(FindRow(pid).has_value());
  std::filesystem::remove_all(webroot);
}

// REQ-OPS-30: a standalone bridge (no --session-id, or --session-id 0) does
// not claim a /talos_registry row, yet still reports itself via self() in
// /system.json.
TEST(BridgeRegistration, StandaloneDoesNotRegisterARow) {
  const std::string binary = BridgeBinary();
  const std::string topic =
      "/test/telemetry/standalone" + std::to_string(::getpid());
  const std::string webroot =
      ::testing::TempDir() + "/bridge_standalone_" + std::to_string(::getpid());
  std::filesystem::create_directories(webroot);

  int stdout_pipe[2]{-1, -1};
  ASSERT_EQ(::pipe(stdout_pipe), 0);
  const std::vector<std::string> args = {binary, topic, "0", webroot};
  std::vector<char*> argv;
  argv.reserve(args.size() + 1);
  for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
  argv.push_back(nullptr);

  const pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    ::close(stdout_pipe[0]);
    ::dup2(stdout_pipe[1], STDOUT_FILENO);
    ::close(stdout_pipe[1]);
    ::execv(argv[0], argv.data());
    ::_exit(127);
  }
  ::close(stdout_pipe[1]);

  std::string line;
  char ch = '\0';
  while (::read(stdout_pipe[0], &ch, 1) == 1) {
    if (ch == '\n') break;
    line.push_back(ch);
  }
  ::close(stdout_pipe[0]);
  ASSERT_TRUE(line.starts_with("listening ")) << line;
  const auto port = static_cast<uint16_t>(std::stoul(line.substr(10)));

  EXPECT_FALSE(FindRow(pid).has_value())
      << "standalone studio_bridge must not claim a registry row";
  const std::string body = HttpGet(port, "/system.json");
  ASSERT_FALSE(body.empty());
  const auto doc = nlohmann::json::parse(body);
  ASSERT_TRUE(doc.contains("nodes"));
  std::size_t bridge_entries = 0;
  nlohmann::json reported;
  for (const auto& node : doc["nodes"]) {
    if (node["name"] == "studio_bridge" &&
        node["pid"].get<std::string>() == std::to_string(pid)) {
      ++bridge_entries;
      reported = node;
    }
  }
  ASSERT_EQ(bridge_entries, 1u) << body;
  EXPECT_EQ(reported["session_id"].get<std::string>(), "0") << body;
  EXPECT_FALSE(FindRow(pid).has_value());

  ::kill(pid, SIGTERM);
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  std::filesystem::remove_all(webroot);
}

// REQ-OPS-30: when supervised (--session-id nonzero), failing to claim a
// registry row exits nonzero immediately rather than serving until the
// launcher SIGKILLs the bridge as heartbeat_missing.
TEST(BridgeRegistration, SupervisedFailsFastWhenRegistryIsFull) {
  const std::string binary = BridgeBinary();
  const std::string topic = "/test/telemetry/full" + std::to_string(::getpid());
  const std::string webroot =
      ::testing::TempDir() + "/bridge_full_" + std::to_string(::getpid());
  std::filesystem::create_directories(webroot);

  std::vector<std::unique_ptr<talos::introspect::NodeRegistration>> held;
  held.reserve(talos::introspect::kMaxNodes);
  for (std::size_t i = 0; i < talos::introspect::kMaxNodes; ++i) {
    held.push_back(std::make_unique<talos::introspect::NodeRegistration>(
        talos::introspect::NodeRegistration::Identity{
            .name = "held" + std::to_string(i), .session_id = 99999}));
  }

  EXPECT_EQ(RunToExit({binary, topic, "0", webroot, "--session-id", "12345"}),
            1);

  held.clear();
  std::filesystem::remove_all(webroot);
}

// REQ-OPS-34: --config=<val> and --idle-timeout-s=<val> accept the same `=`
// syntax as the other options, reject empty --config= and out-of-range
// --idle-timeout-s=, and --describe honors --config=<val>.
TEST(BridgeRegistration, CliEqualsFlagsAndEmptyConfigValidation) {
  const std::string binary = BridgeBinary();
  const std::string topic = "/test/telemetry/cli" + std::to_string(::getpid());
  const std::string dir =
      ::testing::TempDir() + "/bridge_cli_" + std::to_string(::getpid());
  std::filesystem::create_directories(dir);
  const std::string config_file = dir + "/robot.toml";
  {
    std::ofstream out(config_file);
    out << "[robot]\n"
           "name = \"cli_test\"\n"
           "[studio]\n"
           "topic = \"/talos/telemetry/custom\"\n"
           "[subsystems]\n";
  }

  // Empty --config= and invalid --idle-timeout-s= are rejected with exit 2.
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--config="}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--config", ""}), 2);
  EXPECT_EQ(RunToExit({binary, "--describe", "--config="}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--idle-timeout-s="}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--idle-timeout-s=7"}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--idle-timeout-s=961"}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--idle-timeout-s=abc"}), 2);

  // --describe --config=<path> reads [studio] topic from the config.
  {
    int stdout_pipe[2]{-1, -1};
    ASSERT_EQ(::pipe(stdout_pipe), 0);
    const std::string config_flag = "--config=" + config_file;
    const std::vector<std::string> args = {binary, "--describe", config_flag};
    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
    argv.push_back(nullptr);
    const pid_t pid = ::fork();
    ASSERT_GE(pid, 0);
    if (pid == 0) {
      ::close(stdout_pipe[0]);
      ::dup2(stdout_pipe[1], STDOUT_FILENO);
      ::close(stdout_pipe[1]);
      ::execv(argv[0], argv.data());
      ::_exit(127);
    }
    ::close(stdout_pipe[1]);
    std::string output;
    char buf[4096];
    for (;;) {
      const ssize_t n = ::read(stdout_pipe[0], buf, sizeof(buf));
      if (n <= 0) break;
      output.append(buf, static_cast<std::size_t>(n));
    }
    ::close(stdout_pipe[0]);
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
    ASSERT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0) << output;
    const auto doc = nlohmann::json::parse(output);
    // Two sources: the telemetry topic the flag names, and the simulator's
    // game pieces (REQ-DEV-43), which no flag renames.
    ASSERT_TRUE(doc.contains("sources") && doc["sources"].size() == 2u)
        << output;
    EXPECT_EQ(doc["sources"][0]["name"].get<std::string>(),
              "/talos/telemetry/custom")
        << output;
    EXPECT_EQ(doc["sources"][1]["name"].get<std::string>(),
              "/sim/telemetry/pieces")
        << output;
  }

  // Normal startup accepts --config=<path> and --idle-timeout-s=<valid>.
  {
    int stdout_pipe[2]{-1, -1};
    ASSERT_EQ(::pipe(stdout_pipe), 0);
    const std::vector<std::string> args = {binary,
                                           topic,
                                           "0",
                                           dir,
                                           "--config=" + config_file,
                                           "--idle-timeout-s=20"};
    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
    argv.push_back(nullptr);
    const pid_t pid = ::fork();
    ASSERT_GE(pid, 0);
    if (pid == 0) {
      ::close(stdout_pipe[0]);
      ::dup2(stdout_pipe[1], STDOUT_FILENO);
      ::close(stdout_pipe[1]);
      ::execv(argv[0], argv.data());
      ::_exit(127);
    }
    ::close(stdout_pipe[1]);
    std::string line;
    char ch = '\0';
    while (::read(stdout_pipe[0], &ch, 1) == 1) {
      if (ch == '\n') break;
      line.push_back(ch);
    }
    ::close(stdout_pipe[0]);
    ASSERT_TRUE(line.starts_with("listening ")) << line;
    const auto port = static_cast<uint16_t>(std::stoul(line.substr(10)));
    EXPECT_FALSE(HttpGet(port, "/system.json").empty());
    ::kill(pid, SIGTERM);
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
    EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  }

  std::filesystem::remove_all(dir);
}

}  // namespace
}  // namespace studio

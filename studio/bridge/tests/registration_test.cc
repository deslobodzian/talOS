// Studio bridge live-registry registration, loop-progress reporting, realtime
// scheduling (REQ-RT-08), and /system.json deduplication (REQ-OPS-30).

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
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

bool SendAll(int fd, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const char*>(data);
  std::size_t sent = 0;
  while (sent < size) {
    const ssize_t n = ::send(fd, bytes + sent, size - sent, 0);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) return false;
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

bool RecvExact(int fd, void* data, std::size_t size) {
  auto* bytes = static_cast<char*>(data);
  std::size_t got = 0;
  while (got < size) {
    const ssize_t n = ::recv(fd, bytes + got, size - got, 0);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) return false;
    got += static_cast<std::size_t>(n);
  }
  return true;
}

class WsClient {
 public:
  WsClient(std::uint16_t port, std::string_view path) {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) throw std::runtime_error("ws socket failed");
    struct timeval timeout {};
    timeout.tv_sec = 5;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ::setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      close();
      throw std::runtime_error("ws connect failed");
    }
    const std::string request =
        "GET " + std::string{path} +
        " HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
        "Connection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n";
    if (!SendAll(fd_, request.data(), request.size())) {
      close();
      throw std::runtime_error("ws handshake send failed");
    }
    std::string headers;
    char ch = '\0';
    while (headers.size() < 8192 && RecvExact(fd_, &ch, 1)) {
      headers.push_back(ch);
      if (headers.size() >= 4 &&
          headers.compare(headers.size() - 4, 4, "\r\n\r\n") == 0) {
        break;
      }
    }
    if (headers.find(" 101 ") == std::string::npos) {
      close();
      throw std::runtime_error("ws upgrade refused: " + headers);
    }
  }

  ~WsClient() { close(); }
  WsClient(const WsClient&) = delete;
  WsClient& operator=(const WsClient&) = delete;

  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  std::optional<std::string> ReadTextFrame() {
    for (;;) {
      std::array<std::uint8_t, 2> header{};
      if (!RecvExact(fd_, header.data(), header.size())) return std::nullopt;
      const auto opcode = static_cast<std::uint8_t>(header[0] & 0x0Fu);
      const bool masked = (header[1] & 0x80u) != 0;
      std::uint64_t len = header[1] & 0x7Fu;
      if (len == 126) {
        std::array<std::uint8_t, 2> ext{};
        if (!RecvExact(fd_, ext.data(), ext.size())) return std::nullopt;
        len = (static_cast<std::uint64_t>(ext[0]) << 8) | ext[1];
      } else if (len == 127) {
        std::array<std::uint8_t, 8> ext{};
        if (!RecvExact(fd_, ext.data(), ext.size())) return std::nullopt;
        len = 0;
        for (const std::uint8_t byte : ext) len = (len << 8) | byte;
      }
      if (len > std::uint64_t{1024} * 1024) return std::nullopt;
      std::array<std::uint8_t, 4> mask{};
      if (masked && !RecvExact(fd_, mask.data(), mask.size())) {
        return std::nullopt;
      }
      std::string payload(static_cast<std::size_t>(len), '\0');
      if (len > 0 && !RecvExact(fd_, payload.data(), payload.size())) {
        return std::nullopt;
      }
      if (masked) {
        for (std::size_t i = 0; i < payload.size(); ++i) {
          payload[i] = static_cast<char>(static_cast<std::uint8_t>(payload[i]) ^
                                         mask[i % 4]);
        }
      }
      if (opcode == 0x8u) return std::nullopt;
      if (opcode == 0x9u) {
        if (payload.size() <= 125) {
          std::array<std::uint8_t, 6> pong{
              0x8Au, static_cast<std::uint8_t>(0x80u | payload.size()), 0, 0, 0,
              0,
          };
          if (!SendAll(fd_, pong.data(), pong.size())) return std::nullopt;
          if (!payload.empty() &&
              !SendAll(fd_, payload.data(), payload.size())) {
            return std::nullopt;
          }
        }
        continue;
      }
      if (opcode == 0x1u) return payload;
    }
  }

 private:
  int fd_{-1};
};

bool GraphHasNode(const nlohmann::json& doc, std::string_view name) {
  if (!doc.contains("nodes") || !doc["nodes"].is_array()) return false;
  for (const auto& node : doc["nodes"]) {
    if (node.is_object() && node.contains("name") && node["name"].is_string() &&
        node["name"].get<std::string>() == name) {
      return true;
    }
  }
  return false;
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
    ::unsetenv("TALOS_REALTIME");
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
  EXPECT_TRUE(first->realtime()->requested);
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

  // Empty value-taking options and invalid --idle-timeout-s= exit 2.
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--config="}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--config", ""}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--config"}), 2);
  EXPECT_EQ(RunToExit({binary, "--describe", "--config="}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--declared="}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--declared", ""}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--declared"}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--manifest="}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--manifest", ""}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--manifest"}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--refusal-file="}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--refusal-file", ""}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--refusal-file"}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--refusal-reason="}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--refusal-reason", ""}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--refusal-reason"}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--idle-timeout-s="}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", dir, "--idle-timeout-s"}), 2);
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

// REQ-RT-08: studio_bridge runs at LoopTier::kObserver and invokes
// ConfigureRealtime when RealtimeRequested(simulation) is true (non-sim by
// default, or TALOS_REALTIME=1 even under --sim), and reports requested=false
// when false (--sim without TALOS_REALTIME=1, or TALOS_REALTIME=0).
TEST(BridgeRegistration, RealtimeSchedulingHonorsEnvironmentAndSim) {
  const std::string binary = BridgeBinary();
  const std::string webroot =
      ::testing::TempDir() + "/bridge_rt_" + std::to_string(::getpid());
  std::filesystem::create_directories(webroot);

  struct Case {
    const char* label;
    bool simulation;
    std::optional<const char*> env_realtime;
    bool expect_requested;
  };
  const Case cases[] = {
      {"default_robot", false, std::nullopt, true},
      {"sim_default", true, std::nullopt, false},
      {"sim_with_env_1", true, "1", true},
      {"robot_with_env_1", false, "1", true},
      {"robot_with_env_0", false, "0", false},
      {"sim_with_env_0", true, "0", false},
  };

  for (const Case& c : cases) {
    SCOPED_TRACE(c.label);
    const std::string topic =
        "/test/telemetry/rt_" + std::to_string(::getpid());
    std::vector<std::string> args = {binary,  topic,          "0",
                                     webroot, "--session-id", "12345"};
    if (c.simulation) args.push_back("--sim");
    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
    argv.push_back(nullptr);

    int stdout_pipe[2]{-1, -1};
    ASSERT_EQ(::pipe(stdout_pipe), 0);
    const pid_t pid = ::fork();
    ASSERT_GE(pid, 0);
    if (pid == 0) {
      if (c.env_realtime.has_value()) {
        ::setenv("TALOS_REALTIME", *c.env_realtime, 1);
      } else {
        ::unsetenv("TALOS_REALTIME");
      }
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

    struct ChildGuard {
      pid_t pid;
      bool reaped = false;
      int stop() {
        if (reaped || pid <= 0) return 0;
        ::kill(pid, SIGTERM);
        int status = 0;
        while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
        }
        reaped = true;
        return status;
      }
      ~ChildGuard() { stop(); }
    } child{pid};

    if (!line.starts_with("listening ")) {
      child.stop();
      ADD_FAILURE() << "bridge did not listen: " << line;
      continue;
    }

    const auto row = FindRow(pid);
    ASSERT_TRUE(row.has_value()) << "studio_bridge did not register a row";
    EXPECT_EQ((row->flags & talos::introspect::kFlagSimulation) != 0,
              c.simulation);
    ASSERT_TRUE(row->realtime().has_value());
    EXPECT_EQ(row->realtime()->tier, talos::process::LoopTier::kObserver);
    EXPECT_EQ(row->realtime()->requested, c.expect_requested);
    if (!c.expect_requested) {
      EXPECT_EQ(row->realtime()->refused, 0u);
      EXPECT_EQ(row->realtime()->grant(),
                talos::process::RealtimeGrant::kNotRequested);
    }

    const int status = child.stop();
    EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
    EXPECT_FALSE(FindRow(pid).has_value());
  }

  std::filesystem::remove_all(webroot);
}

// REQ-OPS-30: a standalone bridge detects when /talos_registry is unlinked or
// replaced while it runs, drops its dead mapping (reporting unavailable when
// absent), and attaches cleanly to the replacement segment rather than staying
// pinned to the unlinked mapping.
TEST(BridgeRegistration, StandaloneRecoversFromUnlinkedAndReplacedRegistry) {
  const std::string binary = BridgeBinary();
  const std::string topic =
      "/test/telemetry/reg_unlink_" + std::to_string(::getpid());
  const std::string webroot =
      ::testing::TempDir() + "/bridge_reg_unlink_" + std::to_string(::getpid());
  std::filesystem::create_directories(webroot);

  talos::introspect::RegistryMapping::unlink();
  std::optional<talos::introspect::NodeRegistration> alpha;
  alpha.emplace(talos::introspect::NodeRegistration::Identity{
      .name = "alpha", .session_id = 101});

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

  struct ChildGuard {
    pid_t pid;
    bool reaped = false;
    int stop() {
      if (reaped || pid <= 0) return 0;
      ::kill(pid, SIGTERM);
      int status = 0;
      while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
      }
      reaped = true;
      return status;
    }
    ~ChildGuard() { stop(); }
  } child{pid};

  std::string line;
  char ch = '\0';
  while (::read(stdout_pipe[0], &ch, 1) == 1) {
    if (ch == '\n') break;
    line.push_back(ch);
  }
  ::close(stdout_pipe[0]);
  ASSERT_TRUE(line.starts_with("listening ")) << line;
  const auto port = static_cast<std::uint16_t>(std::stoul(line.substr(10)));

  WsClient ws(port, "/telemetry");
  const auto wait_graph =
      [&](auto&& predicate) -> std::optional<nlohmann::json> {
    for (int i = 0; i < 25; ++i) {
      const auto text = ws.ReadTextFrame();
      if (!text.has_value()) return std::nullopt;
      auto doc = nlohmann::json::parse(*text, nullptr, false);
      if (doc.is_object() && doc.value("kind", "") == "talos.system_graph" &&
          predicate(doc)) {
        return doc;
      }
    }
    return std::nullopt;
  };

  // Initial session: registry is available and shows alpha.
  const auto registry_available = [](const nlohmann::json& doc) {
    return doc.contains("registry") && doc["registry"].is_object() &&
           doc["registry"].value("available", false);
  };
  const auto initial = wait_graph([&](const nlohmann::json& doc) {
    return registry_available(doc) && GraphHasNode(doc, "alpha");
  });
  ASSERT_TRUE(initial.has_value()) << "bridge never saw initial alpha row";

  // Phase 1: unlink the registry while the bridge runs; it must drop the dead
  // mapping and report registry.available == false.
  alpha.reset();
  talos::introspect::RegistryMapping::unlink();
  const auto unlinked = wait_graph([&](const nlohmann::json& doc) {
    return !registry_available(doc) && !GraphHasNode(doc, "alpha");
  });
  ASSERT_TRUE(unlinked.has_value())
      << "bridge stayed pinned to unlinked registry";
  const auto http_unlinked =
      nlohmann::json::parse(HttpGet(port, "/system.json"));
  EXPECT_FALSE(http_unlinked["registry"]["available"].get<bool>());
  EXPECT_FALSE(GraphHasNode(http_unlinked, "alpha"));

  // Phase 2: a fresh registry segment appears; bridge reattaches and sees beta.
  std::optional<talos::introspect::NodeRegistration> beta;
  beta.emplace(talos::introspect::NodeRegistration::Identity{
      .name = "beta", .session_id = 202});
  const auto reattached = wait_graph([&](const nlohmann::json& doc) {
    return registry_available(doc) && GraphHasNode(doc, "beta") &&
           !GraphHasNode(doc, "alpha");
  });
  ASSERT_TRUE(reattached.has_value()) << "bridge failed to reattach to beta";
  const auto http_beta = nlohmann::json::parse(HttpGet(port, "/system.json"));
  EXPECT_TRUE(http_beta["registry"]["available"].get<bool>());
  EXPECT_TRUE(GraphHasNode(http_beta, "beta"));

  // Phase 3: immediate unlink+replace with no ENOENT gap between refreshes;
  // bridge detects the replacement segment and switches from beta to gamma.
  beta.reset();
  talos::introspect::RegistryMapping::unlink();
  std::optional<talos::introspect::NodeRegistration> gamma;
  gamma.emplace(talos::introspect::NodeRegistration::Identity{
      .name = "gamma", .session_id = 303});
  const auto replaced = wait_graph([&](const nlohmann::json& doc) {
    return registry_available(doc) && GraphHasNode(doc, "gamma") &&
           !GraphHasNode(doc, "beta");
  });
  ASSERT_TRUE(replaced.has_value())
      << "bridge failed to detect immediate registry replacement";
  const auto http_gamma = nlohmann::json::parse(HttpGet(port, "/system.json"));
  EXPECT_TRUE(http_gamma["registry"]["available"].get<bool>());
  EXPECT_TRUE(GraphHasNode(http_gamma, "gamma"));
  EXPECT_FALSE(GraphHasNode(http_gamma, "beta"));

  ws.close();
  const int status = child.stop();
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  gamma.reset();
  talos::introspect::RegistryMapping::unlink();
  std::filesystem::remove_all(webroot);
}

// REQ-OPS-35, REQ-OPS-36: in refusal mode, draining a drive slot's burst still
// delivers deferred summaries and heartbeats via WakeSchedule::retry_ns without
// busy-spinning the deadline timer.
TEST(BridgeRegistration, RefusalModeHonorsBudgetRetryWithoutBusySpin) {
  const std::string binary = BridgeBinary();
  const std::string topic =
      "/test/telemetry/refusal_" + std::to_string(::getpid());
  const std::string dir =
      ::testing::TempDir() + "/bridge_refusal_" + std::to_string(::getpid());
  std::filesystem::create_directories(dir);
  {
    std::ofstream page(dir + "/drive.html");
    page << "<html>drive</html>";
  }
  const std::string refusal_file = dir + "/last_refusal.json";
  const std::string reason = "phase-1 refusal: " + std::string(3050, 'r');
  {
    std::ofstream out(refusal_file);
    out << nlohmann::json{{"refused", true}, {"reason", reason}}.dump();
  }

  int stdout_pipe[2]{-1, -1};
  int stderr_pipe[2]{-1, -1};
  ASSERT_EQ(::pipe(stdout_pipe), 0);
  ASSERT_EQ(::pipe(stderr_pipe), 0);
  const std::vector<std::string> args = {binary, topic, "0", dir,
                                         "--refusal-file=" + refusal_file};
  std::vector<char*> argv;
  argv.reserve(args.size() + 1);
  for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
  argv.push_back(nullptr);

  const pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    ::close(stdout_pipe[0]);
    ::close(stderr_pipe[0]);
    ::dup2(stdout_pipe[1], STDOUT_FILENO);
    ::dup2(stderr_pipe[1], STDERR_FILENO);
    ::close(stdout_pipe[1]);
    ::close(stderr_pipe[1]);
    ::execv(argv[0], argv.data());
    ::_exit(127);
  }
  ::close(stdout_pipe[1]);
  ::close(stderr_pipe[1]);

  struct ChildGuard {
    pid_t pid;
    int stderr_fd;
    bool reaped = false;
    std::string stderr_text;
    int stop() {
      if (reaped || pid <= 0) return 0;
      ::kill(pid, SIGTERM);
      if (stderr_fd >= 0) {
        char buf[4096];
        for (;;) {
          const ssize_t n = ::read(stderr_fd, buf, sizeof(buf));
          if (n < 0 && errno == EINTR) continue;
          if (n <= 0) break;
          stderr_text.append(buf, static_cast<std::size_t>(n));
        }
        ::close(stderr_fd);
        stderr_fd = -1;
      }
      int status = 0;
      while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
      }
      reaped = true;
      return status;
    }
    ~ChildGuard() { stop(); }
  } child{pid, stderr_pipe[0]};

  std::string line;
  char ch = '\0';
  while (::read(stdout_pipe[0], &ch, 1) == 1) {
    if (ch == '\n') break;
    line.push_back(ch);
  }
  ::close(stdout_pipe[0]);
  ASSERT_TRUE(line.starts_with("listening ")) << line;
  const auto port = static_cast<std::uint16_t>(std::stoul(line.substr(10)));

  // Sequentially connect and close /drive to exhaust the refusal drive slot
  // burst (64,000 B per slot, ~20 summaries of ~3.2 KiB each; up to 90 covers
  // all 4 slots if TCP FIN lags behind reconnect), forcing a budget-deferred
  // send on the persistent client that follows.
  for (int i = 0; i < 90; ++i) {
    WsClient drain_ws(port, "/drive");
    const auto frame = drain_ws.ReadTextFrame();
    ASSERT_TRUE(frame.has_value()) << "drain connection " << i << " closed";
    const auto doc = nlohmann::json::parse(*frame);
    if (doc.value("kind", "") == "talos.drive.heartbeat") break;
  }

  WsClient ws(port, "/drive");
  bool got_summary = false;
  bool got_heartbeat = false;
  for (int i = 0; i < 12 && (!got_summary || !got_heartbeat); ++i) {
    const auto frame = ws.ReadTextFrame();
    ASSERT_TRUE(frame.has_value()) << "refusal /drive feed stalled";
    const auto doc = nlohmann::json::parse(*frame);
    const std::string kind = doc.value("kind", "");
    if (kind == "talos.drive.summary") {
      EXPECT_TRUE(doc.value("launch_refused", false));
      EXPECT_EQ(doc.value("refusal_reason", ""), reason);
      got_summary = true;
    } else if (kind == "talos.drive.heartbeat") {
      got_heartbeat = true;
    }
  }
  EXPECT_TRUE(got_summary);
  EXPECT_TRUE(got_heartbeat);
  ws.close();

  const int status = child.stop();
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  const std::string_view marker = "deadline_wakes=";
  const std::size_t pos = child.stderr_text.find(marker);
  ASSERT_NE(pos, std::string::npos) << child.stderr_text;
  std::uint64_t deadline_wakes = 0;
  const char* begin = child.stderr_text.data() + pos + marker.size();
  const char* end = child.stderr_text.data() + child.stderr_text.size();
  const auto [parsed_end, error] = std::from_chars(begin, end, deadline_wakes);
  ASSERT_EQ(error, std::errc{}) << child.stderr_text;
  ASSERT_GT(parsed_end, begin) << child.stderr_text;
  EXPECT_LT(deadline_wakes, 100u)
      << "refusal mode deadline timer busy-spun: " << child.stderr_text;

  std::filesystem::remove_all(dir);
}

}  // namespace
}  // namespace studio

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
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
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

// REQ-OPS-30: studio_bridge registers a live registry row, advances
// dispatch_count and heartbeat_monotonic_ns on each loop pass, and replaces
// its own row with self() in /system.json so no duplicate entry appears.
TEST(BridgeRegistration, RegistersAdvancesProgressAndDeduplicatesSystemJson) {
  const std::string binary = BridgeBinary();
  const std::string topic = "/test/telemetry/reg" + std::to_string(::getpid());
  const std::string webroot =
      ::testing::TempDir() + "/bridge_reg_" + std::to_string(::getpid());
  std::filesystem::create_directories(webroot);

  // Invalid --session-id is rejected; --describe ignores --session-id.
  EXPECT_EQ(RunToExit({binary, topic, "0", webroot, "--session-id", "-1"}), 2);
  EXPECT_EQ(RunToExit({binary, topic, "0", webroot, "--session-id", "abc"}), 2);
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
  ASSERT_TRUE(first->realtime().has_value());
  EXPECT_EQ(first->realtime()->tier, talos::process::LoopTier::kObserver);
  EXPECT_FALSE(first->realtime()->requested);
  ASSERT_EQ(first->sources.size(), 1u);
  EXPECT_EQ(first->sources[0].kind, talos::event::SourceKind::WATCHER);
  EXPECT_EQ(first->sources[0].name, topic);
  EXPECT_EQ(first->sources[0].message_bytes,
            static_cast<std::uint32_t>(talos::telemetry_wire::slot_bytes));

  // Two HTTP requests ensure at least one post-handler service() pass completes
  // between the first row read and the second.
  const std::string body = HttpGet(port, "/system.json");
  ASSERT_FALSE(body.empty());
  const auto doc = nlohmann::json::parse(body);
  ASSERT_TRUE(doc.contains("registry"));
  EXPECT_TRUE(doc["registry"]["available"].get<bool>());
  ASSERT_TRUE(doc.contains("nodes"));
  std::size_t bridge_entries = 0;
  std::string reported_pid;
  for (const auto& node : doc["nodes"]) {
    if (node["name"] == "studio_bridge") {
      ++bridge_entries;
      reported_pid = node["pid"].get<std::string>();
    }
  }
  EXPECT_EQ(bridge_entries, 1u) << body;
  EXPECT_EQ(reported_pid, std::to_string(pid)) << body;

  // A second request runs in a later loop iteration, after the first's pass()
  // updated the registry row.
  ASSERT_FALSE(HttpGet(port, "/system.json").empty());
  const auto second = FindRow(pid);
  ASSERT_TRUE(second.has_value());
  EXPECT_GT(second->dispatch_count, first->dispatch_count);
  EXPECT_GT(second->heartbeat_monotonic_ns, first->heartbeat_monotonic_ns);

  ::kill(pid, SIGTERM);
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  EXPECT_FALSE(FindRow(pid).has_value());
  std::filesystem::remove_all(webroot);
}

}  // namespace
}  // namespace studio

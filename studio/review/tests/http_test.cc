// studio_review over loopback: the HTTP shapes of the review API and the
// one address it listens on. What each body holds is review_test's.

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "nlohmann/json.hpp"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "studio/review/testdata/fixture.h"

namespace {

using nlohmann::json;

struct Reply {
  int status{0};
  std::string headers;
  std::string body;
};

// One GET over a fresh connection (the server closes nothing, so the body
// is read by its Content-Length), with Host `host` (by default this server's
// 127.0.0.1:<port>) and any `headers` ("Name: value\r\n" each).
Reply Get(std::uint16_t port, const std::string& target,
          const std::string& host = "", const std::string& headers = "") {
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  Reply reply;
  if (connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof address) != 0) {
    close(fd);
    return reply;
  }
  const std::string request =
      "GET " + target + " HTTP/1.1\r\nHost: " +
      (host.empty() ? "127.0.0.1:" + std::to_string(port) : host) + "\r\n" +
      headers + "\r\n";
  (void)!write(fd, request.data(), request.size());
  std::string data;
  char buffer[65536];
  std::size_t header_end = std::string::npos;
  std::size_t length = 0;
  while (true) {
    const ssize_t n = read(fd, buffer, sizeof buffer);
    if (n <= 0) break;
    data.append(buffer, static_cast<std::size_t>(n));
    if (header_end == std::string::npos) {
      header_end = data.find("\r\n\r\n");
      if (header_end == std::string::npos) continue;
      const auto at = data.find("Content-Length: ");
      length = at < header_end ? std::stoul(data.substr(at + 16)) : 0;
    }
    if (data.size() >= header_end + 4 + length) break;
  }
  close(fd);
  if (header_end == std::string::npos) return reply;
  reply.status = std::stoi(data.substr(9, 3));
  reply.headers = data.substr(0, header_end);
  reply.body = data.substr(header_end + 4, length);
  return reply;
}

class HttpTest : public testing::Test {
 protected:
  void SetUp() override {
    directory_ =
        testing::TempDir() + "/review_http_" + std::to_string(getpid());
    std::filesystem::remove_all(directory_);
    studio::review::testing::WriteFixtureSession(directory_ + "/s1");
    studio::review::testing::WriteFixtureSession(directory_ + "/s2");
    std::filesystem::remove(directory_ + "/s2/manifest.json");
    std::string error;
    std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
        rules_cc::cc::runfiles::Runfiles::CreateForTest(
            BAZEL_CURRENT_REPOSITORY, &error));
    ASSERT_NE(runfiles, nullptr) << error;
    const char* server = std::getenv("TALOS_STUDIO_REVIEW");
    if (server == nullptr) FAIL() << "TALOS_STUDIO_REVIEW is not set";
    const std::string binary = runfiles->Rlocation(server);
    int out[2];
    ASSERT_EQ(pipe(out), 0);
    child_ = fork();
    if (child_ == 0) {
      // The server finds the built Studio it serves at "/" in these
      // runfiles, as it finds its own under bazel run.
      for (const auto& [name, value] : runfiles->EnvVars()) {
        setenv(name.c_str(), value.c_str(), 1);
      }
      dup2(out[1], STDOUT_FILENO);
      close(out[0]);
      execl(binary.c_str(), binary.c_str(), "--library", directory_.c_str(),
            "--port", "0", static_cast<char*>(nullptr));
      _exit(127);
    }
    close(out[1]);
    // The first line, once listening; EOF if it exits first.
    FILE* stream = fdopen(out[0], "r");
    char line[256] = {};
    ASSERT_NE(fgets(line, sizeof line, stream), nullptr);
    fclose(stream);
    listening_ = line;
    const auto colon = listening_.rfind(':');
    ASSERT_NE(colon, std::string::npos) << listening_;
    port_ =
        static_cast<std::uint16_t>(std::stoul(listening_.substr(colon + 1)));
  }

  void TearDown() override {
    if (child_ > 0) {
      kill(child_, SIGTERM);
      waitpid(child_, nullptr, 0);
    }
    std::filesystem::remove_all(directory_);
  }

  std::string directory_;
  pid_t child_{-1};
  std::string listening_;
  std::uint16_t port_{0};
};

// Guards the API's transport: JSON documents, the binary series body's
// header and length, 404 for what is not served (timeline and fetch belong
// to later slices), and the built Studio at "/", so a browser opening the
// server reads the API from its own origin.
// REQ-OPS-41
TEST_F(HttpTest, ServesTheReviewShapes) {
  const Reply sessions = Get(port_, "/review/sessions.json");
  ASSERT_EQ(sessions.status, 200) << sessions.headers;
  EXPECT_NE(sessions.headers.find("application/json"), std::string::npos);
  const json list = json::parse(sessions.body);
  ASSERT_TRUE(list.is_array());
  ASSERT_EQ(list.size(), 2u);
  EXPECT_EQ(list[0]["id"], "s1");
  EXPECT_EQ(list[0]["class"], "other");

  const json channels =
      json::parse(Get(port_, "/review/s1/channels.json").body);
  EXPECT_EQ(channels["kind"], "talos.telemetry.channels");
  ASSERT_FALSE(channels["channels"].empty());

  const Reply series = Get(
      port_,
      "/review/s1/series?c=shooter%2Fstate%2Fposition_m,"
      "hw%2Fstate%2Fdriver_station%2Fenabled&start_ns=" +
          std::to_string(studio::review::testing::kStartNs) + "&end_ns=" +
          std::to_string(studio::review::testing::kStartNs + 2'000'000'000) +
          "&points=5");
  ASSERT_EQ(series.status, 200) << series.body;
  EXPECT_NE(series.headers.find("application/octet-stream"), std::string::npos);
  ASSERT_EQ(series.body.size(), 8u + 2 * 5 * 36);
  std::uint32_t header[2];
  std::memcpy(header, series.body.data(), sizeof header);
  EXPECT_EQ(header[0], 2u);
  EXPECT_EQ(header[1], 5u);

  const Reply timeline = Get(port_, "/review/s1/timeline.json");
  ASSERT_EQ(timeline.status, 200) << timeline.body;
  EXPECT_NE(timeline.headers.find("application/json"), std::string::npos);
  const json tl = json::parse(timeline.body);
  EXPECT_EQ(tl["hardware_not_decoded"], false);
  EXPECT_FALSE(tl["modes"].empty());
  EXPECT_FALSE(tl["alerts"].empty());
  EXPECT_FALSE(tl["runs"].empty());

  const Reply declared = Get(port_, "/review/s1/declared.json");
  ASSERT_EQ(declared.status, 200) << declared.body;
  EXPECT_NE(declared.headers.find("application/json"), std::string::npos);
  EXPECT_FALSE(json::parse(declared.body)["nodes"].empty());

  const Reply manifest = Get(port_, "/review/s1/manifest.json");
  ASSERT_EQ(manifest.status, 200) << manifest.body;
  EXPECT_NE(manifest.headers.find("application/json"), std::string::npos);
  EXPECT_NE(manifest.headers.find("Cache-Control: no-store"),
            std::string::npos);
  const json mf = json::parse(manifest.body);
  EXPECT_EQ(mf["session_id"], studio::review::testing::kSessionId);
  EXPECT_FALSE(mf["nodes"].empty());
  EXPECT_EQ(Get(port_, "/review/s2/manifest.json").status, 404);
  EXPECT_EQ(Get(port_, "/review/../manifest.json").status, 400);

  for (const std::string path :
       {"/review/s1/unknown.json", "/review/robot/fetch", "/missing.js"}) {
    EXPECT_EQ(Get(port_, path).status, 404) << path;
  }
  const Reply studio = Get(port_, "/");
  ASSERT_EQ(studio.status, 200) << studio.headers;
  EXPECT_NE(studio.headers.find("text/html"), std::string::npos);
  EXPECT_NE(studio.body.find("<div id=\"root\">"), std::string::npos)
      << studio.body;
}

// Guards the DNS-rebinding defence: a request whose Host is not a loopback
// name with this server's port (a page that resolved its own name to
// 127.0.0.1) is refused before it reaches a session or a file.
// REQ-OPS-41
TEST_F(HttpTest, RefusesAHostThatIsNotThisServer) {
  const std::string port = std::to_string(port_);
  for (const std::string& host :
       {"127.0.0.1:" + port, "localhost:" + port, "[::1]:" + port}) {
    EXPECT_EQ(Get(port_, "/review/sessions.json", host).status, 200) << host;
  }
  for (const std::string& host :
       {"evil.example:" + port, std::string{"evil.example"},
        std::string{"127.0.0.1"}, "127.0.0.1:" + std::to_string(port_ + 1),
        "localhost.evil:" + port, "127.0.0.2:" + port}) {
    EXPECT_EQ(Get(port_, "/review/sessions.json", host).status, 403) << host;
    EXPECT_EQ(Get(port_, "/", host).status, 403) << host;
  }
}

// Guards CORS: a Studio served from another loopback port (the live bridge,
// the dev server) may read the API, its origin echoed and never "*"; any
// other origin gets no Access-Control-Allow-Origin, so its page cannot read
// a session.
// REQ-OPS-41
TEST_F(HttpTest, AllowsOnlyLoopbackOrigins) {
  const auto allowed = [this](const std::string& origin) {
    const Reply reply =
        Get(port_, "/review/sessions.json", "", "Origin: " + origin + "\r\n");
    EXPECT_EQ(reply.status, 200) << origin;
    constexpr std::string_view kHeader = "Access-Control-Allow-Origin: ";
    const auto at = reply.headers.find(kHeader);
    if (at == std::string::npos) return std::string{};
    const auto from = at + kHeader.size();
    return reply.headers.substr(from, reply.headers.find("\r\n", from) - from);
  };
  for (const std::string origin :
       {"http://127.0.0.1:5800", "http://localhost:5173",
        "http://[::1]:5800"}) {
    EXPECT_EQ(allowed(origin), origin);
  }
  for (const std::string origin :
       {"http://evil.example", "https://127.0.0.1:5800", "null",
        "http://127.0.0.1.evil.example:5800", "http://localhost"}) {
    EXPECT_EQ(allowed(origin), "") << origin;
  }
}

// Guards that nothing off this machine can reach the server: the socket is
// bound to 127.0.0.1 (as the kernel reports it), and a connection to any
// other address of this host is refused.
// REQ-OPS-41
TEST_F(HttpTest, ListensOnLoopbackOnly) {
  EXPECT_EQ(listening_.rfind("listening 127.0.0.1:", 0), 0u) << listening_;
  ifaddrs* interfaces = nullptr;
  ASSERT_EQ(getifaddrs(&interfaces), 0);
  for (const ifaddrs* i = interfaces; i != nullptr; i = i->ifa_next) {
    if (i->ifa_addr == nullptr || i->ifa_addr->sa_family != AF_INET) continue;
    sockaddr_in address{};
    std::memcpy(&address, i->ifa_addr, sizeof address);
    if ((ntohl(address.sin_addr.s_addr) >> 24) == 127) continue;
    address.sin_port = htons(port_);
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    const int result =
        connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof address);
    const int error = errno;
    close(fd);
    char text[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &address.sin_addr, text, sizeof text);
    EXPECT_NE(result, 0) << "connected through " << i->ifa_name << ' ' << text;
    EXPECT_EQ(error, ECONNREFUSED) << text;
  }
  freeifaddrs(interfaces);
}

// Guards that a second server cannot share a port: uSockets sets SO_REUSEPORT
// on a listener unless told LIBUS_LISTEN_EXCLUSIVE_PORT, so a second
// studio_review on a used port bound beside the first and the kernel split
// its connections between two libraries. Held here as uSockets holds a
// shared listener, the port is refused, and the server says which.
// REQ-OPS-41
TEST(HttpListen, RefusesAPortAnotherListenerHolds) {
  const int holder = socket(AF_INET, SOCK_STREAM, 0);
  ASSERT_GE(holder, 0);
  const int on = 1;
  ASSERT_EQ(setsockopt(holder, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on), 0);
  ASSERT_EQ(setsockopt(holder, SOL_SOCKET, SO_REUSEPORT, &on, sizeof on), 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ASSERT_EQ(bind(holder, reinterpret_cast<sockaddr*>(&address), sizeof address),
            0);
  ASSERT_EQ(listen(holder, 1), 0);
  socklen_t length = sizeof address;
  ASSERT_EQ(getsockname(holder, reinterpret_cast<sockaddr*>(&address), &length),
            0);
  const std::string port = std::to_string(ntohs(address.sin_port));

  const std::string library =
      testing::TempDir() + "/review_listen_" + std::to_string(getpid());
  std::filesystem::create_directories(library);
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  ASSERT_NE(runfiles, nullptr) << error;
  const char* server = std::getenv("TALOS_STUDIO_REVIEW");
  ASSERT_NE(server, nullptr) << "TALOS_STUDIO_REVIEW is not set";
  const std::string binary = runfiles->Rlocation(server);
  int out[2];
  int err[2];
  ASSERT_EQ(pipe(out), 0);
  ASSERT_EQ(pipe(err), 0);
  const pid_t child = fork();
  if (child == 0) {
    for (const auto& [name, value] : runfiles->EnvVars()) {
      setenv(name.c_str(), value.c_str(), 1);
    }
    dup2(out[1], STDOUT_FILENO);
    dup2(err[1], STDERR_FILENO);
    close(out[0]);
    close(err[0]);
    close(holder);
    execl(binary.c_str(), binary.c_str(), "--library", library.c_str(),
          "--port", port.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }
  close(out[1]);
  close(err[1]);
  // A server that listens prints its line and runs on; one refused exits,
  // and the line is EOF.
  FILE* stream = fdopen(out[0], "r");
  char line[256] = {};
  const bool listened = fgets(line, sizeof line, stream) != nullptr;
  fclose(stream);
  if (listened) kill(child, SIGTERM);
  int status = 0;
  waitpid(child, &status, 0);
  std::string messages;
  char buffer[4096];
  for (ssize_t n; (n = read(err[0], buffer, sizeof buffer)) > 0;) {
    messages.append(buffer, static_cast<std::size_t>(n));
  }
  close(err[0]);
  close(holder);
  std::filesystem::remove_all(library);

  EXPECT_FALSE(listened) << "listened beside the holder: " << line;
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) != 0) << status;
  EXPECT_NE(messages.find("cannot listen on 127.0.0.1:" + port),
            std::string::npos)
      << messages;
}

}  // namespace

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
// is read by its Content-Length).
Reply Get(std::uint16_t port, const std::string& target) {
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
      "GET " + target + " HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n";
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
    std::string error;
    std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
        rules_cc::cc::runfiles::Runfiles::CreateForTest(
            BAZEL_CURRENT_REPOSITORY, &error));
    ASSERT_NE(runfiles, nullptr) << error;
    const std::string binary =
        runfiles->Rlocation(std::getenv("TALOS_STUDIO_REVIEW"));
    int out[2];
    ASSERT_EQ(pipe(out), 0);
    child_ = fork();
    if (child_ == 0) {
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
// header and length, and 404 for what is not served (timeline and fetch
// belong to later slices).
// REQ-OPS-41
TEST_F(HttpTest, ServesTheReviewShapes) {
  const Reply sessions = Get(port_, "/review/sessions.json");
  ASSERT_EQ(sessions.status, 200) << sessions.headers;
  EXPECT_NE(sessions.headers.find("application/json"), std::string::npos);
  const json list = json::parse(sessions.body);
  ASSERT_TRUE(list.is_array());
  ASSERT_EQ(list.size(), 1u);
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

  for (const std::string path :
       {"/review/s1/timeline.json", "/review/robot/fetch", "/index.html"}) {
    EXPECT_EQ(Get(port_, path).status, 404) << path;
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

}  // namespace

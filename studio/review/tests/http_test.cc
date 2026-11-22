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
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
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
    if (n < 0 && errno == EINTR) continue;
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
    const std::filesystem::path raw_dir =
        std::filesystem::path(testing::TempDir()) /
        ("review_http_" + std::to_string(getpid()));
    std::filesystem::remove_all(raw_dir);
    std::filesystem::create_directories(raw_dir);
    directory_ = std::filesystem::weakly_canonical(raw_dir).string();
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
      while (waitpid(child_, nullptr, 0) < 0 && errno == EINTR) {
      }
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

namespace {

std::map<std::string, std::string> SnapshotDirectory(
    const std::filesystem::path& root) {
  std::map<std::string, std::string> snapshot;
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator(root)) {
    const std::string rel =
        std::filesystem::relative(entry.path(), root).generic_string();
    if (entry.is_directory()) {
      snapshot.emplace(rel + "/", "");
      continue;
    }
    std::ifstream in(entry.path(), std::ios::binary);
    std::ostringstream buf;
    buf << in.rdbuf();
    snapshot.emplace(rel, buf.str());
  }
  return snapshot;
}

#if defined(__linux__)
std::set<std::uint64_t> ProcNetInodes(
    const std::vector<std::string>& tables,
    std::map<std::uint64_t, std::pair<std::string, std::string>>*
        tcp_local_and_state = nullptr) {
  std::set<std::uint64_t> inodes;
  for (const auto& table : tables) {
    std::ifstream in("/proc/net/" + table);
    if (!in.is_open()) continue;
    std::string line;
    std::getline(in, line);  // header
    const std::size_t col = (table == "unix") ? 6 : 9;
    while (std::getline(in, line)) {
      std::istringstream ss(line);
      std::vector<std::string> fields;
      std::string tok;
      while (ss >> tok) fields.push_back(std::move(tok));
      if (fields.size() <= col) continue;
      const std::uint64_t inode = std::stoull(fields[col]);
      inodes.insert(inode);
      if (tcp_local_and_state != nullptr && fields.size() > 3) {
        (*tcp_local_and_state)[inode] = {fields[1], fields[3]};
      }
    }
  }
  return inodes;
}
#elif defined(__APPLE__)
bool RunToolCapture(const char* file, const std::vector<const char*>& argv,
                    std::string& stdout_out, int& exit_code) {
  int pipefd[2];
  if (pipe(pipefd) != 0) return false;
  const pid_t pid = fork();
  if (pid < 0) {
    close(pipefd[0]);
    close(pipefd[1]);
    return false;
  }
  if (pid == 0) {
    dup2(pipefd[1], STDOUT_FILENO);
    close(pipefd[0]);
    close(pipefd[1]);
    execvp(file, const_cast<char* const*>(argv.data()));
    _exit(127);
  }
  close(pipefd[1]);
  stdout_out.clear();
  char buf[4096];
  for (;;) {
    const ssize_t n = read(pipefd[0], buf, sizeof buf);
    if (n < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (n == 0) break;
    stdout_out.append(buf, static_cast<std::size_t>(n));
  }
  close(pipefd[0]);
  int status = 0;
  pid_t waited = -1;
  do {
    waited = waitpid(pid, &status, 0);
  } while (waited < 0 && errno == EINTR);
  if (waited != pid || !WIFEXITED(status)) {
    exit_code = -1;
    return false;
  }
  exit_code = WEXITSTATUS(status);
  return true;
}

bool MatchesExactPort(std::string_view text, std::uint16_t port) {
  const std::string needle = ":" + std::to_string(port);
  std::size_t pos = 0;
  while ((pos = text.find(needle, pos)) != std::string_view::npos) {
    const std::size_t end = pos + needle.size();
    if (end == text.size() || text[end] < '0' || text[end] > '9') {
      return true;
    }
    pos = end;
  }
  return false;
}

bool IsWritableShmVmmapLine(std::string_view line) {
  const std::size_t bracket = line.find(']');
  if (bracket == std::string_view::npos) return false;
  std::size_t i = bracket + 1;
  while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) ++i;
  if (i + 7 > line.size() || line[i + 3] != '/') return false;
  const std::string_view cur_prot = line.substr(i, 3);
  if (cur_prot.find('w') == std::string_view::npos) return false;
  i += 7;
  while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) ++i;
  constexpr std::string_view kSmShm = "SM=SHM";
  if (!line.substr(i).starts_with(kSmShm)) return false;
  const std::size_t after = i + kSmShm.size();
  return after == line.size() || line[after] == ' ' || line[after] == '\t';
}

bool IsDarwinNonSocketType(std::string_view fd_type) {
  return fd_type == "REG" || fd_type == "VREG" || fd_type == "DIR" ||
         fd_type == "VDIR" || fd_type == "CHR" || fd_type == "VCHR" ||
         fd_type == "BLK" || fd_type == "VBLK" || fd_type == "FIFO" ||
         fd_type == "VFIFO" || fd_type == "PIPE" || fd_type == "KQUEUE" ||
         fd_type == "VNODE" || fd_type == "NPOLICY" || fd_type == "GUARD" ||
         fd_type == "FSEVENTS" || fd_type == "NEXUS" || fd_type == "CHANNEL";
}
#endif

}  // namespace

// Guards that studio_review is strictly a read-only loopback viewer: after
// serving every review endpoint it holds only loopback TCP (no UDP or raw
// sockets), opens or maps zero shared memory, and writes nothing to its
// session library directory.
// REQ-SAFE-35
TEST_F(HttpTest, OpensNoDatagramSocketNoSharedMemoryAndWritesNothing) {
  const auto before = SnapshotDirectory(directory_);
  ASSERT_FALSE(before.empty());

  ASSERT_EQ(Get(port_, "/review/sessions.json").status, 200);
  ASSERT_EQ(Get(port_, "/review/s1/channels.json").status, 200);
  ASSERT_EQ(Get(port_, "/review/s1/timeline.json").status, 200);
  ASSERT_EQ(Get(port_, "/review/s1/declared.json").status, 200);
  ASSERT_EQ(Get(port_, "/review/s1/manifest.json").status, 200);
  ASSERT_EQ(
      Get(port_, "/review/s1/series?c=shooter%2Fstate%2Fposition_m&start_ns=" +
                     std::to_string(studio::review::testing::kStartNs) +
                     "&end_ns=" +
                     std::to_string(studio::review::testing::kStartNs +
                                    2'000'000'000) +
                     "&points=5")
          .status,
      200);
  ASSERT_EQ(Get(port_, "/").status, 200);

  // Zero writes to the log directory.
  const auto after = SnapshotDirectory(directory_);
  EXPECT_EQ(before, after);

#if defined(__linux__)
  // Freeze child_ while taking the /proc/<pid>/fd + /proc/net/* snapshot:
  // otherwise the server can close the accepted client socket from Get("/")
  // between reading /proc/<pid>/fd and /proc/net/tcp, making a legitimate TCP
  // inode disappear from /proc/net/tcp mid-check.
  struct ChildStopGuard {
    pid_t pid;
    explicit ChildStopGuard(pid_t p) : pid(p) {}
    ~ChildStopGuard() {
      if (pid > 0) kill(pid, SIGCONT);
    }
    ChildStopGuard(const ChildStopGuard&) = delete;
    ChildStopGuard& operator=(const ChildStopGuard&) = delete;
  };
  ASSERT_EQ(kill(child_, SIGSTOP), 0) << strerror(errno);
  ChildStopGuard resume_child{child_};
  int stop_status = 0;
  pid_t waited = -1;
  while ((waited = waitpid(child_, &stop_status, WUNTRACED)) < 0 &&
         errno == EINTR) {
  }
  ASSERT_EQ(waited, child_) << strerror(errno);
  ASSERT_TRUE(WIFSTOPPED(stop_status)) << "status=" << stop_status;

  const std::string proc_root = "/proc/" + std::to_string(child_);
  std::set<std::uint64_t> held_sockets;
  for (const auto& entry :
       std::filesystem::directory_iterator(proc_root + "/fd")) {
    std::error_code ec;
    const auto target = std::filesystem::read_symlink(entry.path(), ec);
    if (ec) continue;
    const std::string s = target.string();
    EXPECT_EQ(s.rfind("/dev/shm/", 0), std::string::npos)
        << "studio_review opened shared memory: " << s;
    constexpr std::string_view kPrefix = "socket:[";
    if (s.starts_with(kPrefix) && s.back() == ']') {
      held_sockets.insert(
          std::stoull(s.substr(kPrefix.size(), s.size() - kPrefix.size() - 1)));
    }
  }
  {
    std::ifstream maps(proc_root + "/maps");
    ASSERT_TRUE(maps.is_open());
    std::string line;
    while (std::getline(maps, line)) {
      EXPECT_EQ(line.find("/dev/shm/"), std::string::npos)
          << "studio_review mapped shared memory: " << line;
    }
  }
  const auto datagram =
      ProcNetInodes({"udp", "udp6", "udplite", "udplite6", "raw", "raw6"});
  for (const std::uint64_t inode : held_sockets) {
    EXPECT_EQ(datagram.count(inode), 0u)
        << "studio_review holds UDP or raw socket inode " << inode;
  }
  std::map<std::uint64_t, std::pair<std::string, std::string>> tcp_info;
  const auto tcp = ProcNetInodes({"tcp", "tcp6"}, &tcp_info);
  const auto unix_socks = ProcNetInodes({"unix"});
  int loopback_listeners = 0;
  char port_hex[8] = {};
  std::snprintf(port_hex, sizeof port_hex, ":%04X",
                static_cast<unsigned>(port_));
  const std::string want_v4 = std::string{"0100007F"} + port_hex;
  const std::string want_v6 =
      std::string{"00000000000000000000000001000000"} + port_hex;
  for (const std::uint64_t inode : held_sockets) {
    EXPECT_TRUE(tcp.count(inode) > 0 || unix_socks.count(inode) > 0)
        << "studio_review holds non-TCP/non-Unix socket inode " << inode;
    if (const auto it = tcp_info.find(inode); it != tcp_info.end()) {
      const auto& [local_addr, state] = it->second;
      // 0A = TCP_LISTEN; every listener must be bound to loopback on port_.
      if (state == "0A") {
        const bool is_expected =
            (local_addr == want_v4 || local_addr == want_v6);
        EXPECT_TRUE(is_expected)
            << "studio_review listener is not loopback on expected port: "
            << local_addr << " (want " << want_v4 << " or " << want_v6 << ")";
        if (is_expected) ++loopback_listeners;
      }
    }
  }
  EXPECT_EQ(loopback_listeners, 1)
      << "expected exactly 1 loopback TCP listener on port " << port_
      << " in /proc/net/tcp";
#elif defined(__APPLE__)
  const std::string pid_arg = std::to_string(child_);

  // Catch closed-fd SharedMemoryPtr mappings via vmmap: studio_review must
  // hold zero writable SM=SHM regions. Bazel's macOS test PATH may omit
  // /usr/sbin, so prefer the standard system paths when executable.
  const char* vmmap_bin =
      access("/usr/bin/vmmap", X_OK) == 0 ? "/usr/bin/vmmap" : "vmmap";
  const char* lsof_bin =
      access("/usr/sbin/lsof", X_OK) == 0 ? "/usr/sbin/lsof" : "lsof";
  std::string vmmap_out;
  int vmmap_exit = -1;
  ASSERT_TRUE(RunToolCapture(vmmap_bin,
                             {vmmap_bin, "-wide", pid_arg.c_str(), nullptr},
                             vmmap_out, vmmap_exit));
  ASSERT_EQ(vmmap_exit, 0) << vmmap_out;
  ASSERT_FALSE(vmmap_out.empty());
  {
    std::istringstream vlines(vmmap_out);
    std::string vline;
    while (std::getline(vlines, vline)) {
      EXPECT_FALSE(IsWritableShmVmmapLine(vline))
          << "studio_review mapped writable shared memory: " << vline;
    }
  }

  // Strict lsof socket allowlist: reject every socket fd except unix sockets,
  // the single loopback TCP LISTEN socket on exact :{port_}, and any loopback
  // accepted TCP socket on exact local :{port_} still tearing down after Get().
  std::string output;
  int lsof_exit = -1;
  ASSERT_TRUE(RunToolCapture(lsof_bin,
                             {lsof_bin, "-n", "-P", "-a", "-p", pid_arg.c_str(),
                              "-F", "ftPTan", nullptr},
                             output, lsof_exit));
  ASSERT_EQ(lsof_exit, 0) << output;

  struct FdRec {
    std::string type;
    std::string proto;
    std::string state;
    std::string access;
    std::string name;
  };
  std::vector<FdRec> recs;
  std::istringstream lines(output);
  std::string line;
  while (std::getline(lines, line)) {
    if (line.empty()) continue;
    const char tag = line[0];
    const std::string val = line.substr(1);
    if (tag == 'f') {
      recs.emplace_back();
    } else if (!recs.empty()) {
      if (tag == 't') recs.back().type = val;
      if (tag == 'P') recs.back().proto = val;
      if (tag == 'T' && val.starts_with("ST=")) {
        recs.back().state = val.substr(3);
      }
      if (tag == 'a') recs.back().access = val;
      if (tag == 'n') recs.back().name = val;
    }
  }
  ASSERT_FALSE(recs.empty()) << "lsof returned no descriptors";
  int loopback_listeners = 0;
  for (const auto& r : recs) {
    if (r.type == "PSXSHM") {
      ADD_FAILURE() << "studio_review holds shm fd: " << r.name;
      continue;
    }
    if ((r.type == "REG" || r.type == "VREG") &&
        r.name.find(directory_) != std::string::npos) {
      EXPECT_NE(r.access, "w") << "writable library fd: " << r.name;
      EXPECT_NE(r.access, "u") << "read-write library fd: " << r.name;
    }
    if (IsDarwinNonSocketType(r.type) && r.proto.empty()) continue;
    if (r.type == "unix") continue;
    // macOS libnetwork/getaddrinfo opens an internal com.apple.netsrc
    // kernel-control socket for address-selection queries (cannot transmit).
    if (r.type == "systm" && r.name.starts_with("[ctl com.apple.netsrc ")) {
      continue;
    }
    // Strict allowlist: the only allowed non-unix sockets are the single
    // loopback TCP LISTEN socket on exact :{port_} and any lingering loopback
    // accepted connection on exact local :{port_} tearing down after Get().
    const auto is_loopback_endpoint = [](std::string_view ep) {
      return ep.starts_with("127.0.0.1:") || ep.starts_with("[::1]:");
    };
    const bool is_inet = (r.type == "IPv4" || r.type == "IPv6");
    const std::size_t arrow = r.name.find("->");
    const bool is_expected_listener =
        is_inet && r.proto == "TCP" && r.state == "LISTEN" &&
        arrow == std::string::npos && is_loopback_endpoint(r.name) &&
        MatchesExactPort(r.name, port_);
    bool is_loopback_accepted = false;
    if (is_inet && r.proto == "TCP" && arrow != std::string::npos &&
        (r.state == "ESTABLISHED" || r.state == "CLOSE_WAIT" ||
         r.state == "FIN_WAIT_1" || r.state == "FIN_WAIT_2" ||
         r.state == "LAST_ACK")) {
      const std::string_view local = std::string_view{r.name}.substr(0, arrow);
      const std::string_view remote =
          std::string_view{r.name}.substr(arrow + 2);
      is_loopback_accepted = is_loopback_endpoint(local) &&
                             MatchesExactPort(local, port_) &&
                             is_loopback_endpoint(remote);
    }
    EXPECT_TRUE(is_expected_listener || is_loopback_accepted)
        << "studio_review holds disallowed socket: type=" << r.type
        << " proto=" << r.proto << " state=" << r.state << " name=" << r.name;
    if (is_expected_listener) ++loopback_listeners;
  }
  EXPECT_EQ(loopback_listeners, 1)
      << "expected exactly 1 loopback TCP LISTEN socket on port " << port_
      << " in:\n"
      << output;
#endif
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
  while (waitpid(child, &status, 0) < 0 && errno == EINTR) {
  }
  std::string messages;
  char buffer[4096];
  for (;;) {
    const ssize_t n = read(err[0], buffer, sizeof buffer);
    if (n < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (n == 0) break;
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

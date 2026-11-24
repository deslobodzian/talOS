// Integration tier (fork/exec, loopback UDP, the live registry): the gateway
// library's extension hooks (gateway_main.h: BeforeTick, Topics, Counts) as a
// program built on RunSimGateway uses them, through gateway_hooks_fixture.
// This is the plumbing a referee's Driver Station stamp and match topic will
// use; it proves no refereeing.
#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "geometry/geometry_descriptor.h"
#include "geometry/geometry_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/introspection/registry.h"
#include "talOS/protocol/frame.h"

namespace talos {
namespace {

namespace ds = driver_station;

constexpr const char* kTopic = "/test/state/hooks";
// The fixture's mode schedule: disabled, then teleop from this schedule time.
constexpr uint64_t kTeleopFromUs = 200'000;
constexpr const char* kModeSchedule = "disabled:0.2,teleop:0.2";

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

std::string TempPath(const std::string& name) {
  const char* dir = std::getenv("TEST_TMPDIR");
  return (std::filesystem::path{dir != nullptr ? dir : "/tmp"} / name).string();
}

std::string ReadFile(const std::string& path) {
  std::ifstream file{path};
  return {std::istreambuf_iterator<char>{file}, {}};
}

struct Output {
  int status{-1};
  std::string out;
  std::string err;
};

// `binary args` to completion, stdout and stderr apart.
Output RunBinary(const char* binary, const std::string& args) {
  const std::string err_path = TempPath("describe.err");
  const std::string command =
      "'" + Runfile(binary) + "' " + args + " 2>'" + err_path + "'";
  FILE* pipe = ::popen(command.c_str(), "r");
  if (pipe == nullptr) throw std::runtime_error("popen failed");
  Output output;
  std::array<char, 4096> buffer{};
  while (const std::size_t n =
             std::fread(buffer.data(), 1, buffer.size(), pipe)) {
    output.out.append(buffer.data(), n);
  }
  const int status = ::pclose(pipe);
  output.status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  output.err = ReadFile(err_path);
  return output;
}

using Endpoint = std::tuple<std::string, int, uint32_t, uint32_t, uint32_t>;

// The fields the launcher's live-endpoint check (EndpointMismatch,
// talOS/launcher/describe_graph.cc, private to the launcher) compares, with
// its mask of linted flags: kind, name, size, alignment and those flags,
// timers left out, as a sorted multiset.
template <typename Source>
std::vector<Endpoint> Endpoints(const std::vector<Source>& sources) {
  constexpr uint32_t kLinted =
      introspect::naming::kSourceFlagExternal |
      introspect::naming::kSourceFlagOptional |
      introspect::naming::kSourceFlagObserver |
      introspect::naming::kSourceFlagUntargetedOutputs |
      introspect::naming::kSourceFlagOperatorArming |
      introspect::naming::kSourceFlagInternalDriverStation;
  std::vector<Endpoint> out;
  for (const auto& source : sources) {
    if (source.kind == event::SourceKind::TIMER) continue;
    out.emplace_back(source.name, static_cast<int>(source.kind),
                     source.message_bytes, source.alignment,
                     source.flags & kLinted);
  }
  std::sort(out.begin(), out.end());
  return out;
}

int BoundUdpSocket(uint16_t* port) {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) throw std::runtime_error("socket failed");
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  socklen_t length = sizeof address;
  if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof address) != 0 ||
      ::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length) != 0) {
    throw std::runtime_error("bind failed");
  }
  *port = ntohs(address.sin_port);
  return fd;
}

// A free UDP port below the OS ephemeral range, probed and released, as
// talOS/build/udp_ports.py picks one (docs/TESTING.md §6.4).
uint16_t FreePortBelowEphemeral() {
  constexpr int kBase = 20000;
  constexpr int kSpan = 12000;
  const int start = static_cast<int>(std::random_device{}() % kSpan);
  for (int offset = 0; offset < kSpan; ++offset) {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    const auto port = static_cast<uint16_t>(kBase + (start + offset) % kSpan);
    address.sin_port = htons(port);
    const bool free =
        ::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof address) == 0;
    ::close(fd);
    if (free) return port;
  }
  throw std::runtime_error("no free UDP port in 20000..31999");
}

// One run of the hooks fixture. It starts with no schedule start; the first
// Driver Station packet received sets the schedules' time zero 100 ms after
// that packet's own sample time (the gateway's clock), so at least that
// packet precedes it whatever the host's load. Then 0.4 s of schedule.
struct HooksRun {
  uint64_t start_us{0};  // the schedules' time zero
  std::vector<ds::DriverStationData> packets;
  std::optional<introspect::NodeSnapshot> row;  // with kTopic counted
  int status{-1};
  std::string log;
};

HooksRun RunHooksFixture() {
  HooksRun run;
  const std::string config = TempPath("robot.toml");
  std::ofstream{config} << "[robot]\nname = \"hooks_test\"\nperiod_us = 5000\n"
                           "controller_address = \"127.0.0.1\"\n"
                           "[subsystems.test.motors.motor]\ncan_id = 1\n";
  const std::string start_file = TempPath("schedule_start");
  std::filesystem::remove(start_file);

  uint16_t receive_port = 0;
  const int receiver = BoundUdpSocket(&receive_port);
  timeval timeout{.tv_sec = 0, .tv_usec = 50'000};
  ::setsockopt(receiver, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

  const std::string binary = Runfile("GATEWAY_HOOKS_FIXTURE");
  const std::string log = TempPath("fixture.log");
  std::vector<std::string> args{binary,
                                "--config",
                                config,
                                "--port",
                                std::to_string(FreePortBelowEphemeral()),
                                "--remote-port",
                                std::to_string(receive_port),
                                "--schedule-start-file",
                                start_file,
                                "--mode-schedule",
                                kModeSchedule,
                                "--duration-s",
                                "0.4"};
  const pid_t child = ::fork();
  if (child == 0) {
    std::FILE* out = std::freopen(log.c_str(), "w", stdout);
    if (out != nullptr) ::dup2(::fileno(out), STDERR_FILENO);
    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (auto& arg : args) argv.push_back(arg.data());
    argv.push_back(nullptr);
    ::execv(argv[0], argv.data());
    ::_exit(127);
  }
  // The counts only bound a hang: 200 receives that time out (10 s without
  // a datagram), or 20000 datagrams (100 s of State at 200 Hz).
  int idle = 0;
  int received = 0;
  std::array<uint8_t, protocol::kMaxFrameSize> buffer{};
  while (idle < 200 && received < 20000) {
    if (!run.row) {
      if (const auto reader = introspect::RegistryReader::open()) {
        for (const auto& row : reader->nodes()) {
          if (row.pid != static_cast<uint64_t>(child) || !row.alive) continue;
          for (const auto& source : row.sources) {
            if (source.name == kTopic && source.events > 0) run.row = row;
          }
        }
      }
    }
    const ssize_t n = ::recv(receiver, buffer.data(), buffer.size(), 0);
    if (n > 0) {
      ++received;
      protocol::DecodedFrame frame;
      ds::DriverStationData data;
      if (protocol::DecodeFrame(buffer.data(), static_cast<std::size_t>(n),
                                &frame) == protocol::DecodeStatus::kOk &&
          frame.header.type == protocol::FrameType::kDriverStation &&
          ds::Decode({frame.payload, frame.payload_size}, data)) {
        run.packets.push_back(data);
        if (run.start_us == 0) {
          // Atomically, as the gateway expects (it rereads each tick).
          run.start_us = data.sample_time_us + 100'000;
          const std::string partial = start_file + ".partial";
          std::ofstream{partial} << run.start_us * 1000 << "\n";
          std::filesystem::rename(partial, start_file);
        }
      }
      continue;
    }
    ++idle;
    int status = 0;
    if (::waitpid(child, &status, WNOHANG) == child) {
      run.status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
      break;
    }
  }
  if (run.status == -1) {
    ::kill(child, SIGKILL);
    ::waitpid(child, nullptr, 0);
  }
  ::close(receiver);
  run.log = ReadFile(log);
  return run;
}

class GatewayHooks : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { run_ = new HooksRun(RunHooksFixture()); }
  static void TearDownTestSuite() {
    delete run_;
    run_ = nullptr;
  }
  static HooksRun* run_;
};
HooksRun* GatewayHooks::run_ = nullptr;

// Every Driver Station packet carries what BeforeTick stamped on its own
// tick: that tick's flags and its schedule time, absent before time zero,
// on the same clock as the mode schedule (exact, no tolerance).
TEST_F(GatewayHooks, PacketsCarryTheStampOfTheTickTheyGoOutOn) {
  ASSERT_EQ(run_->status, 0) << run_->log;
  ASSERT_NE(run_->start_us, 0U) << "no Driver Station packet\n" << run_->log;
  // A gateway that read the start file only after time zero (a loaded host)
  // says so, and its ticks until then still had no schedule time.
  const bool read_late =
      run_->log.find("schedule start is") != std::string::npos;
  int before = 0;
  int after = 0;
  int teleop = 0;
  for (const auto& packet : run_->packets) {
    const std::string message{packet.fms.game_specific_message};
    SCOPED_TRACE("packet at " + std::to_string(packet.sample_time_us) +
                 " us, time zero " + std::to_string(run_->start_us) +
                 " us, message '" + message + "'");
    const auto space = message.find(' ');
    ASSERT_NE(space, std::string::npos);
    EXPECT_EQ(message.substr(0, space), std::to_string(packet.fms.flags));
    const std::string schedule = message.substr(space + 1);
    const bool teleop_flag = (packet.fms.flags & ds::kTeleop) != 0;
    if (schedule == "-") {
      ++before;
      EXPECT_EQ(packet.fms.match_time_s, -1.0F);
      if (!read_late) EXPECT_LT(packet.sample_time_us, run_->start_us);
      EXPECT_FALSE(teleop_flag);
      continue;
    }
    ++after;
    const uint64_t schedule_us = std::stoull(schedule);
    ASSERT_GE(packet.sample_time_us, run_->start_us);
    EXPECT_EQ(schedule_us, packet.sample_time_us - run_->start_us);
    EXPECT_EQ(packet.fms.match_time_s, static_cast<float>(schedule_us) / 1e6F);
    // The mode schedule switches on the very value the stamp carries.
    EXPECT_EQ(teleop_flag, packet.fms.match_time_s >= 0.2F);
    EXPECT_EQ(teleop_flag, schedule_us >= kTeleopFromUs);
    teleop += teleop_flag ? 1 : 0;
  }
  EXPECT_GT(before, 0);
  EXPECT_GT(after, 0);
  EXPECT_GT(teleop, 0);
}

// BeforeTick runs on every tick, not only on those that send a packet.
TEST_F(GatewayHooks, BeforeTickRunsOnEveryTick) {
  ASSERT_EQ(run_->status, 0) << run_->log;
  unsigned long long before_calls = 0;
  unsigned long long after_calls = 0;
  const auto at = run_->log.find("hooks: ");
  ASSERT_NE(at, std::string::npos) << run_->log;
  ASSERT_EQ(std::sscanf(run_->log.c_str() + at,
                        "hooks: before_tick_calls=%llu after_tick_calls=%llu",
                        &before_calls, &after_calls),
            2)
      << run_->log;
  EXPECT_EQ(before_calls, after_calls);
  EXPECT_GT(before_calls, run_->packets.size());
}

// The launcher refuses a run whose live endpoints differ from its --describe
// probe: the extension's topic is in both, with its schema in the describe,
// and the live row counts what the extension published on it.
TEST_F(GatewayHooks, LiveRowDeclaresAndCountsTheTopicsDescribeDeclares) {
  ASSERT_EQ(run_->status, 0) << run_->log;
  const Output describe = RunBinary("GATEWAY_HOOKS_FIXTURE", "--describe");
  ASSERT_EQ(describe.status, 0) << describe.err;
  introspect::Description described;
  std::string error;
  ASSERT_TRUE(introspect::DescribeReader::Parse(describe.out, described, error))
      << error;
  const auto topic =
      std::find_if(described.sources.begin(), described.sources.end(),
                   [](const auto& source) { return source.name == kTopic; });
  ASSERT_NE(topic, described.sources.end());
  const auto schema = schema::Of<geometry::SwerveModulePosition>();
  EXPECT_EQ(topic->schema, schema.name);
  EXPECT_EQ(topic->schema_hash, schema.hash);
  EXPECT_EQ(topic->message_bytes, sizeof(geometry::SwerveModulePosition));

  ASSERT_TRUE(run_->row.has_value())
      << "no live registry row counted " << kTopic << "\n"
      << run_->log;
  EXPECT_EQ(Endpoints(run_->row->sources), Endpoints(described.sources));
  for (const auto& source : run_->row->sources) {
    if (source.name != kTopic) continue;
    EXPECT_GT(source.events, 0U);
    EXPECT_EQ(source.dropped, 0U);
  }
}

// An extension that overrides none of the hooks describes sim_gateway
// --physics exactly, its target aside.
TEST(GatewayHooksPlain, AnExtensionWithoutHooksDescribesAsSimGateway) {
  const Output plain = RunBinary("GATEWAY_HOOKS_FIXTURE", "--plain --describe");
  const Output sim = RunBinary("SIM_GATEWAY", "--describe --physics");
  ASSERT_EQ(plain.status, 0) << plain.err;
  ASSERT_EQ(sim.status, 0) << sim.err;
  std::string ours = plain.out;
  const std::string target = "//talOS/simulation:gateway_hooks_fixture";
  const auto at = ours.find(target);
  ASSERT_NE(at, std::string::npos) << ours;
  ours.replace(at, target.size(), "//talOS/simulation:sim_gateway");
  EXPECT_EQ(ours, sim.out);
}

// Topics() the registry row could not hold, or that repeat a name, are
// refused before anything runs, --describe included, naming the count or
// the topic; the most a row holds is accepted.
TEST(GatewayHooksTopics, RefusesTopicsTheRowCannotHoldOrThatRepeat) {
  struct Row {
    const char* variant;
    int status;
    std::string named;    // in stderr
    std::size_t sources;  // described, when accepted
  };
  const std::vector<Row> rows{
      {"--topics=63", 0, "", introspect::kMaxSourcesPerNode},
      {"--topics=64", 1, "65", 0},
      {"--topics=twice", 1, kTopic, 0},
      {"--topics=state", 1, "/sim/state", 0},
  };
  for (const auto& row : rows) {
    SCOPED_TRACE(row.variant);
    const Output output = RunBinary("GATEWAY_HOOKS_FIXTURE",
                                    std::string{row.variant} + " --describe");
    EXPECT_EQ(output.status, row.status) << output.err;
    if (row.status != 0) {
      EXPECT_NE(output.err.find(row.named), std::string::npos) << output.err;
      EXPECT_TRUE(output.out.empty()) << output.out;
      continue;
    }
    introspect::Description described;
    std::string error;
    ASSERT_TRUE(introspect::DescribeReader::Parse(output.out, described, error))
        << error;
    EXPECT_EQ(described.sources.size(), row.sources);
  }
}

}  // namespace
}  // namespace talos

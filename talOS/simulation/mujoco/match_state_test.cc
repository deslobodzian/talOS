// Integration tier (fork/exec, loopback UDP, shared memory, the live
// registry), REQ-DEV-40: mujoco_gateway on the 2026 field referees the match
// its mode schedule plays. /sim/state/match reports the periods in order and
// a countdown, the Driver Station packets carry the match time and game
// message, and the live registry row declares what --describe declares, as
// the launcher's live-endpoint check requires.
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
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "sim/sim_descriptor.h"
#include "sim/sim_generated.h"
#include "sim/topics.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/introspection/registry.h"
#include "talOS/ipc/subscriber.h"
#include "talOS/protocol/frame.h"

namespace talos {
namespace {

namespace ds = driver_station;

// 0.2 s disabled, 0.4 s of AUTO, then TELEOP until the run ends.
constexpr const char* kModeSchedule = "disabled:0.2,auto:0.4,teleop:0.6";
constexpr const char* kDurationS = "1.2";

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

using Endpoint = std::tuple<std::string, int, uint32_t, uint32_t, uint32_t>;

// What the launcher's live-endpoint check (EndpointMismatch) compares: kind,
// name, size, alignment and the linted flags, timers left out, sorted.
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

// One run on the 2026 field. It starts with no schedule start; the first
// Driver Station packet received sets the schedules' time zero 100 ms after
// that packet's sample time, and the test then subscribes to
// /sim/state/match, which the gateway has published since its first tick.
struct MatchRun {
  std::vector<ds::DriverStationData> packets;  // from time zero on
  std::vector<::sim::MatchState> matches;
  std::optional<introspect::NodeSnapshot> row;  // with the topic counted
  uint64_t start_us{0};
  int status{-1};
  std::string log;
};

MatchRun RunGateway() {
  MatchRun run;
  const std::string start_file = TempPath("schedule_start");
  std::filesystem::remove(start_file);
  uint16_t receive_port = 0;
  const int receiver = BoundUdpSocket(&receive_port);
  timeval timeout{.tv_sec = 0, .tv_usec = 50'000};
  ::setsockopt(receiver, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

  const std::string log = TempPath("gateway.log");
  std::vector<std::string> args{Runfile("MUJOCO_GATEWAY"),
                                "--config",
                                Runfile("ROVER_CONFIG"),
                                "--port",
                                std::to_string(FreePortBelowEphemeral()),
                                "--remote-port",
                                std::to_string(receive_port),
                                "--schedule-start-file",
                                start_file,
                                "--mode-schedule",
                                kModeSchedule,
                                "--duration-s",
                                kDurationS};
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
  std::optional<ipc::Subscriber<::sim::MatchState>> subscriber;
  // The counts only bound a hang: 200 receives that time out (10 s without
  // a datagram), or 20000 datagrams (100 s of State at 200 Hz).
  int idle = 0;
  int received = 0;
  std::array<uint8_t, protocol::kMaxFrameSize> buffer{};
  while (idle < 200 && received < 20000) {
    if (subscriber) {
      while (const auto message = subscriber->read_next()) {
        run.matches.push_back(message->message);
      }
    }
    if (!run.row) {
      if (const auto reader = introspect::RegistryReader::open()) {
        for (const auto& row : reader->nodes()) {
          if (row.pid != static_cast<uint64_t>(child) || !row.alive) continue;
          for (const auto& source : row.sources) {
            if (source.name == ::sim::topics::kMatchState &&
                source.events > 0) {
              run.row = row;
            }
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
        if (run.start_us == 0) {
          run.start_us = data.sample_time_us + 100'000;
          const std::string partial = start_file + ".partial";
          std::ofstream{partial} << run.start_us * 1000 << "\n";
          std::filesystem::rename(partial, start_file);
          subscriber.emplace(::sim::topics::kMatchState);
        }
        if (data.sample_time_us >= run.start_us) run.packets.push_back(data);
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
  if (subscriber) {
    while (const auto message = subscriber->read_next()) {
      run.matches.push_back(message->message);
    }
  }
  ::close(receiver);
  run.log = ReadFile(log);
  return run;
}

class MujocoGatewayMatch : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { run_ = new MatchRun(RunGateway()); }
  static void TearDownTestSuite() {
    delete run_;
    run_ = nullptr;
  }
  static MatchRun* run_;
};
MatchRun* MujocoGatewayMatch::run_ = nullptr;

// /sim/state/match walks the schedule's match: None, then AUTO and TELEOP,
// each counting its time down.
TEST_F(MujocoGatewayMatch, PublishesThePeriodsInOrderCountingDown) {
  ASSERT_EQ(run_->status, 0) << run_->log;
  std::vector<::sim::MatchPeriod> periods;
  for (const auto& match : run_->matches) {
    if (periods.empty() || periods.back() != match.period()) {
      periods.push_back(match.period());
    }
  }
  EXPECT_EQ(periods,
            (std::vector<::sim::MatchPeriod>{::sim::MatchPeriod::None,
                                             ::sim::MatchPeriod::Autonomous,
                                             ::sim::MatchPeriod::Teleop}))
      << run_->log;
  for (std::size_t i = 1; i < run_->matches.size(); ++i) {
    const auto& before = run_->matches[i - 1];
    const auto& now = run_->matches[i];
    if (now.period() != before.period() ||
        now.period() == ::sim::MatchPeriod::None) {
      continue;
    }
    EXPECT_LT(now.period_remaining_us(), before.period_remaining_us())
        << "message " << i;
    EXPECT_GT(now.sample_time_ns(), before.sample_time_ns()) << "message " << i;
  }
}

// Every Driver Station packet carries the match of the tick it went out on
// (the /sim/state/match message of the same node-clock time): its period's
// time left and, in TELEOP, the alliance inactive first. The first TELEOP
// packet, sent on the tick the mode changes, already carries TELEOP's.
TEST_F(MujocoGatewayMatch, DriverStationPacketsCarryTheirTicksMatch) {
  ASSERT_EQ(run_->status, 0) << run_->log;
  std::map<uint64_t, ::sim::MatchState> by_time;
  for (const auto& match : run_->matches) {
    by_time.emplace(match.sample_time_ns(), match);
  }
  bool first_teleop = true;
  int teleop = 0;
  for (const auto& packet : run_->packets) {
    const auto it = by_time.find(packet.sample_time_us * 1000);
    ASSERT_NE(it, by_time.end())
        << "no /sim/state/match at " << packet.sample_time_us << " us";
    const ::sim::MatchState& match = it->second;
    // As the game converts it (fields/rebuilt_2026/rules.cc Fms).
    EXPECT_EQ(packet.fms.match_time_s,
              static_cast<float>(
                  static_cast<double>(match.period_remaining_us()) / 1e6))
        << packet.sample_time_us << " us";
    const std::string message = packet.fms.game_specific_message;
    if (match.period() == ::sim::MatchPeriod::Teleop) {
      EXPECT_EQ(message, match.blue_inactive_first() ? "B" : "R");
    } else {
      EXPECT_EQ(message, "");
    }
    if ((packet.fms.flags & ds::kTeleop) == 0) continue;
    ++teleop;
    if (first_teleop) {
      first_teleop = false;
      // Sound whatever the tick's step count: the rules observe the input
      // when the plant gets it (WorldRules::ObserveMatch), before any step.
      EXPECT_EQ(match.period(), ::sim::MatchPeriod::Teleop);
      EXPECT_GT(packet.fms.match_time_s, 140.0F - 0.0101F)
          << "TELEOP's time, less at most two 5 ms steps";
    }
  }
  EXPECT_GT(teleop, 0) << run_->log;
}

// The launcher refuses a run whose live endpoints differ from its --describe
// probe: /sim/state/match is in both, with sim.MatchState's schema, and the
// row counts what was published on it.
TEST_F(MujocoGatewayMatch, LiveRowDeclaresWhatDescribeDeclares) {
  ASSERT_EQ(run_->status, 0) << run_->log;
  const std::string out_path = TempPath("describe.out");
  const std::string command =
      "'" + Runfile("MUJOCO_GATEWAY") + "' --describe >'" + out_path + "'";
  ASSERT_EQ(std::system(command.c_str()), 0);
  introspect::Description described;
  std::string error;
  ASSERT_TRUE(
      introspect::DescribeReader::Parse(ReadFile(out_path), described, error))
      << error;
  const auto topic =
      std::find_if(described.sources.begin(), described.sources.end(),
                   [](const auto& source) {
                     return source.name == ::sim::topics::kMatchState;
                   });
  ASSERT_NE(topic, described.sources.end());
  const auto schema = schema::Of<::sim::MatchState>();
  EXPECT_EQ(topic->schema, schema.name);
  EXPECT_EQ(topic->schema_hash, schema.hash);
  EXPECT_EQ(topic->message_bytes, sizeof(::sim::MatchState));
  ASSERT_TRUE(run_->row.has_value()) << run_->log;
  EXPECT_EQ(Endpoints(run_->row->sources), Endpoints(described.sources));
}

}  // namespace
}  // namespace talos

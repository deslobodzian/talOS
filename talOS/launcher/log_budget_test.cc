// A session whose members log at very different rates and record sizes,
// under a cap small enough that every log rotates: each keeps about the same
// tail of the session (REQ-OPS-32).

#include "talOS/launcher/log_budget.h"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/hardware/endpoint.h"
#include "talOS/hardware/gateway.h"
#include "talOS/hardware/sim_backend.h"
#include "talOS/launcher/launcher.h"
#include "talOS/protocol/frame.h"

namespace talos::launcher {
namespace {

namespace ds = driver_station;
using namespace std::chrono_literals;

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

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_log_budget_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

// A loopback UDP port nothing holds now, so parallel tests do not share the
// controller link.
uint16_t FreeUdpPort() {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  socklen_t length = sizeof address;
  if (fd < 0 ||
      ::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof address) != 0 ||
      ::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length) != 0) {
    throw std::runtime_error("no free UDP port");
  }
  ::close(fd);
  return ntohs(address.sin_port);
}

// The RoboRIO's part, in this process, as sim_gateway plays it: one State
// per 5 ms tick, disabled, and a Driver Station packet every 20 ms with one
// joystick at its largest encoding (the size the launcher's model takes for
// a Driver Station packet).
class Controller {
 public:
  Controller(const std::string& config_path, uint16_t port,
             uint16_t bridge_port)
      : gateway_{backend_, boot_id_} {
    auto config = config::ParseRobotConfig(config_path).hardware;
    config.commissioned = true;
    if (!gateway_.Reconfigure(config)) {
      throw std::runtime_error("gateway rejected the test config");
    }
    if (endpoint_.Open("127.0.0.1", port, "127.0.0.1", bridge_port) !=
        protocol::UdpStatus::kOk) {
      throw std::runtime_error("cannot open the gateway socket");
    }
    data_.boot_id = boot_id_;
    ds::JoystickData& stick = data_.joysticks[0];
    stick.connected = true;
    stick.axis_count = ds::kMaxAxes;
    stick.button_count = ds::kMaxButtons;
    stick.pov_count = ds::kMaxPovs;
    stick.buttons = 0x5a5a5a5a;
    stick.axes.fill(0.25f);
    stick.povs.fill(90);
    std::memset(stick.name, 'x', ds::kMaxNameLength - 1);
    thread_ = std::jthread{[this](const std::stop_token& stop) { Run(stop); }};
  }

 private:
  void Run(const std::stop_token& stop) {
    std::array<uint8_t, protocol::kMaxPayloadSize> buffer{};
    uint64_t last_packet_us = 0;
    auto next = std::chrono::steady_clock::now();
    while (!stop.stop_requested()) {
      const uint64_t now_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();
      data_.sample_time_us = now_us;
      data_.fms.flags = ds::kDsAttached;
      tracker_.Stamp(data_);
      endpoint_.Tick(now_us, false);
      if (last_packet_us == 0 ||
          now_us - last_packet_us >= ds::kSamplePeriodUs) {
        const auto size = ds::Encode(data_, buffer);
        endpoint_.SendFrame(protocol::FrameType::kDriverStation, now_us,
                            buffer.data(), size);
        last_packet_us = now_us;
      }
      next += 5ms;
      std::this_thread::sleep_until(next);
    }
  }

  hardware::SimBackend backend_;
  // One boot id for the State and the Driver Station packets alike.
  const uint64_t boot_id_{std::random_device{}() | 1ull};
  hardware::Gateway gateway_;
  hardware::Endpoint endpoint_{gateway_};
  ds::DriverStationData data_;
  ds::ModeTracker tracker_;
  std::jthread thread_;
};

// A member's kept log: its first run's segment files, oldest first (the
// oldest ones deleted by rotation).
std::vector<std::string> KeptFiles(const std::string& session,
                                   const std::string& member) {
  return event::log::LogSegmentFiles(session + "/" + member + ".tlog");
}

// Every message and send record in a member's kept log is on a topic the
// model counts for that member (the bridge: only /hw/state and
// /hw/command) and stores no more bytes than the model gives it, so the
// model never under-sizes this member's records.
void ExpectRecordsWithinModel(const std::string& session,
                              const std::string& member, bool bridge,
                              const TopicLoads& loads) {
  for (const std::string& path : KeptFiles(session, member)) {
    event::log::LogReader reader{path};
    std::map<std::uint16_t, std::string> topics;
    for (const auto& registration : reader.manifest()) {
      topics[registration.id] = registration.name;
    }
    std::map<std::string, std::uint32_t> largest;
    event::log::LogReader::Record record;
    while (reader.next(record)) {
      const auto kind = static_cast<event::EventKind>(record.header.kind);
      if (kind != event::EventKind::MESSAGE && kind != event::EventKind::SEND) {
        continue;
      }
      auto& bytes = largest[topics.at(record.header.source_id)];
      bytes = std::max(bytes, record.header.stored_bytes);
    }
    for (const auto& [topic, bytes] : largest) {
      if (bridge) {
        EXPECT_TRUE(topic == hardware::kStateTopic ||
                    topic == hardware::kCommandTopic)
            << "the bridge logs " << topic << ", which the model omits";
      }
      const auto load = loads.find(topic);
      ASSERT_NE(load, loads.end()) << member << " logs undeclared " << topic;
      EXPECT_LE(bytes, load->second.stored_bytes)
          << member << " stores " << bytes << " B on " << topic
          << ", more than the model's " << load->second.stored_bytes;
    }
  }
}

// REQ-OPS-32. The window the launcher computes from its split reaches every
// logging member, the bridge (its own flag parsing) and a node (the node
// framework's): in a disabled session each one's first segment ends at the
// first record window / 8 or more after its first record, and not before.
// Without the flag a segment ends on bytes, which hold more than window / 8
// of what these members store while disabled. How a log keeps its tail in
// those segments, and what the window is, are proven without processes
// (log_test's LogCapture.Window*, log_budget_split_test). Record time, not
// wall time: a host that delays ticks does not move the bound. The session
// stops once every log has started its second segment. That the model fits
// what these real nodes store is checked per record.
TEST(LogBudget, LauncherWindowEndsEveryMembersSegments) {
  const std::string root = TempPath("out");
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  const std::string config = root + "/robot.toml";
  // Half of log_max_bytes is the session's event-log budget: about 10 s of
  // these two members, so window / 8 is above the writer's 1 s floor.
  std::ofstream{config} << R"([robot]
name = "log_budget"
period_us = 5000
log_max_bytes = 3500000

[subsystems.budget_fast]
node = "//talOS/launcher:budget_fast"
period_us = 5000
device = "m1"

[subsystems.budget_fast.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
)";

  const uint16_t gateway_port = FreeUdpPort();
  const uint16_t bridge_port = FreeUdpPort();
  LauncherOptions options;
  options.config_path = config;
  options.output_dir = root;
  options.session_id = 1;
  options.simulation = true;
  options.remote = "127.0.0.1";
  options.remote_port = gateway_port;
  options.local_port = bridge_port;
  // A ceiling only: the test stops the session once every log rotated.
  options.duration_s = 60;
  options.describe_timeout_ms = 10000;
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      Runfile("TALOS_BRIDGE");
  options.binary_overrides["//talOS/launcher:budget_fast"] =
      Runfile("TALOS_BUDGET_FAST");

  const std::vector<std::string> members = {"hardware_node", "budget_fast"};
  const std::string session = root + "/1";
  Controller controller{config, gateway_port, bridge_port};
  std::atomic<bool> ended{false};
  int status = -1;
  std::jthread run{[&] {
    status = Launcher{options}.Run();
    ended = true;
  }};
  const auto all_rotated = [&] {
    return std::ranges::all_of(members, [&](const std::string& member) {
      return std::filesystem::exists(
          event::log::LogSegmentPath(session + "/" + member + ".tlog", 0));
    });
  };
  while (!ended && !all_rotated()) std::this_thread::sleep_for(20ms);
  ASSERT_FALSE(ended) << "the session ended before every log rotated";
  ::kill(::getpid(), SIGTERM);
  run.join();
  EXPECT_EQ(status, 0);

  // What each member declared, probed as the launcher probes it.
  std::vector<NodeDescription> described;
  for (const std::string& member : members) {
    const std::string target = member == kHardwareNodeName
                                   ? "//talOS/bridge:hardware_node"
                                   : "//talOS/launcher:" + member;
    NodeDescription node;
    node.roster_name = member;
    std::string error;
    ASSERT_TRUE(ProbeDescribe({options.binary_overrides.at(target),
                               "--describe", "--config", config, "--sim"},
                              10s, node.declared, error))
        << member << ": " << error;
    node.answered = true;
    described.push_back(std::move(node));
  }
  // The window the launcher's split gives, from the same model.
  const auto robot = config::ParseRobotConfig(config);
  const TopicLoads loads = DeclaredTopicLoads(described, robot);
  const uint64_t budget = static_cast<uint64_t>(robot.log_max_bytes) / 2;
  std::vector<double> rates;
  std::vector<uint64_t> floors;
  for (const NodeDescription& node : described) {
    const std::string& member = node.roster_name;
    rates.push_back(ExpectedLogRate(node.declared,
                                    MemberPeriodUs(robot, member),
                                    member == kHardwareNodeName, loads));
    floors.push_back(SmallestRotatingShare(node.declared));
  }
  const LogSplit split = SplitLogBudget(
      budget, std::vector<bool>(members.size(), true), rates, floors);
  const int64_t span_ns = std::chrono::nanoseconds{split.window}.count() /
                          static_cast<int64_t>(event::log::kLogSegments);
  // Above the writer's floor (its 1 s sync_interval), or the window's value
  // would not show.
  ASSERT_GT(span_ns, 1'000'000'000)
      << "window " << split.window.count() << " us";

  for (const std::string& member : members) {
    SCOPED_TRACE(member);
    const auto files = KeptFiles(session, member);
    ASSERT_GE(files.size(), 2u);
    // The first segment's first and last records, and the record that
    // started the second.
    int64_t first = 0;
    int64_t last = 0;
    bool any = false;
    event::log::LogReader reader{files[0]};
    event::log::LogReader::Record record;
    while (reader.next(record)) {
      if (!any) first = record.header.now_ns;
      last = record.header.now_ns;
      any = true;
    }
    ASSERT_TRUE(any);
    event::log::LogReader second{files[1]};
    ASSERT_TRUE(second.next(record));
    std::printf(
        "%s: first segment %.3f s, next record at %.3f s, window / 8 %.3f s\n",
        member.c_str(), static_cast<double>(last - first) / 1e9,
        static_cast<double>(record.header.now_ns - first) / 1e9,
        static_cast<double>(span_ns) / 1e9);
    EXPECT_LT(last - first, span_ns);
    EXPECT_GE(record.header.now_ns - first, span_ns);
    ExpectRecordsWithinModel(session, member, member == kHardwareNodeName,
                             loads);
  }
  std::filesystem::remove_all(root);
}

}  // namespace
}  // namespace talos::launcher

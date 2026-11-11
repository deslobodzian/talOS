// A session whose members log at very different rates and record sizes,
// under a cap small enough that every log rotates: each keeps about the same
// tail of the session (REQ-OPS-32).

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
// per 5 ms tick, enabled in teleop, and a Driver Station packet every 20 ms
// with one joystick at its largest encoding (the size the launcher's model
// takes for a Driver Station packet).
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
    thread_ = std::jthread{[this](std::stop_token stop) { Run(stop); }};
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
      data_.fms.flags = ds::kDsAttached | ds::kEnabled | ds::kTeleop;
      tracker_.Stamp(data_);
      endpoint_.Tick(now_us, true);
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

// The time a member's kept log covers: from the first record of its oldest
// kept segment to its last record, over every segment of its first run.
double KeptSeconds(const std::string& session, const std::string& member) {
  int64_t first = std::numeric_limits<int64_t>::max();
  int64_t last = std::numeric_limits<int64_t>::min();
  for (const std::string& path : event::log::FindLogFiles(session)) {
    const std::string file = std::filesystem::path{path}.filename().string();
    if (!file.starts_with(member + ".tlog") &&
        !file.starts_with(member + ".segment")) {
      continue;
    }
    event::log::LogReader reader{path};
    event::log::LogReader::Record record;
    while (reader.next(record)) {
      first = std::min(first, record.header.now_ns);
      last = std::max(last, record.header.now_ns);
    }
  }
  return first < last ? static_cast<double>(last - first) / 1e9 : 0;
}

// REQ-OPS-32. A 5 ms and a 20 ms motor owner (each logging every 200 Hz
// State), the hardware bridge (State and commands only) and a 20 ms Driver
// Station reader (no State at all) log between about 15 and 110 kB/s. Under
// a cap that rotates every log, each member's kept tail is within 1.5x of
// the others'. The session is stopped once every log has dropped its oldest
// segment, not after a fixed time.
TEST(LogBudget, EveryCappedMemberKeepsTheSameTail) {
  const std::string root = TempPath("out");
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  const std::string config = root + "/robot.toml";
  // Half of log_max_bytes is the session's event-log budget: 2 MiB here, a
  // few seconds of this session.
  std::ofstream{config} << R"([robot]
name = "log_budget"
period_us = 5000
log_max_bytes = 4194304

[subsystems.budget_fast]
node = "//talOS/launcher:budget_fast"
period_us = 5000
device = "m1"

[subsystems.budget_fast.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.budget_slow]
node = "//talOS/launcher:budget_slow"
period_us = 20000
device = "m2"

[subsystems.budget_slow.motors.m2]
type = "TalonFX"
bus = "rio"
can_id = 2

[subsystems.budget_listener]
node = "//talOS/launcher:budget_listener"
period_us = 20000
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
  options.duration_s = 120;
  options.describe_timeout_ms = 10000;
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      Runfile("TALOS_BRIDGE");
  options.binary_overrides["//talOS/launcher:budget_fast"] =
      Runfile("TALOS_BUDGET_FAST");
  options.binary_overrides["//talOS/launcher:budget_slow"] =
      Runfile("TALOS_BUDGET_SLOW");
  options.binary_overrides["//talOS/launcher:budget_listener"] =
      Runfile("TALOS_BUDGET_LISTENER");

  const std::vector<std::string> members = {"hardware_node", "budget_fast",
                                            "budget_slow", "budget_listener"};
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
          event::log::CappedMarkerPath(session + "/" + member + ".tlog"));
    });
  };
  while (!ended && !all_rotated()) std::this_thread::sleep_for(50ms);
  ASSERT_FALSE(ended) << "the session ended before every log rotated";
  ::kill(::getpid(), SIGTERM);
  run.join();
  EXPECT_EQ(status, 0);

  std::map<std::string, double> kept;
  for (const std::string& member : members) {
    kept[member] = KeptSeconds(session, member);
    std::printf("%s kept %.2f s\n", member.c_str(), kept[member]);
  }
  const auto [shortest, longest] = std::ranges::minmax_element(
      kept, {}, [](const auto& entry) { return entry.second; });
  ASSERT_GT(shortest->second, 0) << shortest->first << " kept nothing";
  EXPECT_LE(longest->second / shortest->second, 1.5)
      << longest->first << " kept " << longest->second << " s, "
      << shortest->first << " " << shortest->second << " s";
  std::filesystem::remove_all(root);
}

}  // namespace
}  // namespace talos::launcher

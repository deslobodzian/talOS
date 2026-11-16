// A real node's registry row carries outputs_held while the devices flush
// holds its outputs, and drops it on a disable (REQ-SAFE-41): the launcher
// runs the real bridge and a node binary through talos::Run against a
// gateway in this process, and the test reads the row as Studio does.

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/endpoint.h"
#include "talOS/hardware/gateway.h"
#include "talOS/hardware/sim_backend.h"
#include "talOS/introspection/registry.h"
#include "talOS/launcher/launcher.h"
#include "talOS/protocol/frame.h"

namespace talos::launcher {
namespace {

namespace ds = driver_station;
using namespace std::chrono_literals;

constexpr std::string_view kNode = "held_motor";

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
// per 5 ms tick and a Driver Station packet every 20 ms, in teleop, enabled
// until set_enabled(false).
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
    thread_ = std::jthread{[this](const std::stop_token& stop) { Run(stop); }};
  }

  void set_enabled(bool enabled) { enabled_ = enabled; }

 private:
  void Run(const std::stop_token& stop) {
    std::array<uint8_t, protocol::kMaxPayloadSize> buffer{};
    uint64_t last_packet_us = 0;
    bool last_enabled = enabled_;
    auto next = std::chrono::steady_clock::now();
    while (!stop.stop_requested()) {
      const uint64_t now_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();
      const bool enabled = enabled_;
      data_.sample_time_us = now_us;
      data_.fms.flags =
          ds::kDsAttached | ds::kTeleop | (enabled ? ds::kEnabled : 0u);
      tracker_.Stamp(data_);
      endpoint_.Tick(now_us, enabled);
      // A packet at once on a mode change, as the RoboRIO sends one.
      if (last_packet_us == 0 || enabled != last_enabled ||
          now_us - last_packet_us >= ds::kSamplePeriodUs) {
        const auto size = ds::Encode(data_, buffer);
        endpoint_.SendFrame(protocol::FrameType::kDriverStation, now_us,
                            buffer.data(), size);
        last_packet_us = now_us;
        last_enabled = enabled;
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
  std::atomic<bool> enabled_{true};
  std::jthread thread_;
};

// Polls `done` until it holds. The deadline only bounds a hang
// (docs/TESTING.md §6.3): the launcher's describe probes and spawns come
// first, slowly under a sanitizer.
template <typename Done>
bool Eventually(Done done) {
  const auto deadline = std::chrono::steady_clock::now() + 30s;
  while (!done()) {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

// REQ-SAFE-41. The chain from the flush to a registry reader, in a running
// node: Flush computes held, the node exposes it to its Reporter (talos::Run
// passes NodeAccess::outputs_held), the Reporter writes kFlagOutputsHeld into
// the row, and RegistryReader::nodes() returns it, which is what Studio's
// system view prints as "outputs held: no live target". A node that sets
// its motor every tick with no talos::Target is held while enabled, and its
// row (same pid, still alive) drops the bit once the robot is disabled.
// Which ticks hold is proven in process (target_test's
// TargetGate.TheNodeReportsOutputsHeldOnlyWhileTheFlushHoldsThem); the bit's
// encoding in the row by registry_test.
TEST(OutputsHeld, ARunningNodesRegistryRowShowsHeldOutputsUntilADisable) {
  const std::string root = ::testing::TempDir() + "/talos_outputs_held_" +
                           std::to_string(::getpid());
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  const std::string config = root + "/robot.toml";
  std::ofstream{config} << R"([robot]
name = "outputs_held"
period_us = 5000

[subsystems.held_motor]
node = "//talOS/launcher:held_motor"
period_us = 10000

[subsystems.held_motor.motors.m1]
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
  // A ceiling only: the test stops the session once the row has set and
  // cleared the bit.
  options.duration_s = 50;
  options.describe_timeout_ms = 10000;
  options.binary_overrides["//talOS/bridge:hardware_node"] =
      Runfile("TALOS_BRIDGE");
  options.binary_overrides["//talOS/launcher:held_motor"] =
      Runfile("TALOS_HELD_MOTOR");

  Controller controller{config, gateway_port, bridge_port};
  std::atomic<bool> ended{false};
  int status = -1;
  std::jthread run{[&] {
    status = Launcher{options}.Run();
    ended = true;
  }};

  // The node's live row as a registry reader returns it; the registry exists
  // once the launcher has prepared it.
  std::optional<introspect::RegistryReader> reader;
  std::optional<introspect::NodeSnapshot> last;
  const auto row = [&]() -> std::optional<introspect::NodeSnapshot> {
    if (!reader) reader = introspect::RegistryReader::open();
    if (!reader) return std::nullopt;
    for (auto& node : reader->nodes()) {
      if (node.name == kNode && node.alive) return last = std::move(node);
    }
    return std::nullopt;
  };
  const auto describe = [&] {
    return last ? "pid " + std::to_string(last->pid) + " flags " +
                      std::to_string(last->flags) + " outputs_held " +
                      std::to_string(last->outputs_held())
                : std::string{"no live row"};
  };

  // Each wait also ends when the session does, which the asserts report.
  std::uint64_t pid = 0;
  Eventually([&] {
    const auto node = row();
    if (node && node->outputs_held()) pid = node->pid;
    return pid != 0 || ended;
  });
  const bool held = pid != 0;
  const std::string while_enabled = describe();
  bool cleared = false;
  if (held) {
    controller.set_enabled(false);
    Eventually([&] {
      const auto node = row();
      cleared = node && node->pid == pid && !node->outputs_held();
      return cleared || ended;
    });
  }
  const std::string after_disable = describe();
  const bool ended_first = ended;
  ::kill(::getpid(), SIGTERM);
  run.join();

  ASSERT_FALSE(ended_first) << "the session ended before the test stopped it";
  ASSERT_TRUE(held) << kNode << "'s row never carried outputs_held while "
                    << "enabled with its motor set and no target: "
                    << while_enabled;
  EXPECT_TRUE(cleared) << kNode << "'s row (pid " << pid
                       << ") kept outputs_held after the disable: "
                       << after_disable;
  EXPECT_EQ(status, 0);
  std::filesystem::remove_all(root);
}

}  // namespace
}  // namespace talos::launcher

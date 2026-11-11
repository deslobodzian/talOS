#pragma once

// Helpers shared by the bridge's tests (hardware_node_test.cc, run_test.cc):
// a bridge and fake gateway on loopback ports, the test configs, and
// requests, States and commands as the two ends exchange them.

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <thread>

#include "talOS/bridge/node.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/protocol/udp_test_helpers.h"

namespace talos::hardware::bridge_test {

// Binds a loopback UDP socket to a port the OS chooses and connects it to
// itself, so it accepts no datagram from another socket: a bridge sending to
// the port sees it closed (ECONNREFUSED), as if nothing were bound, while no
// other socket can be given the port. Returns the socket, or -1.
inline int ReservePort(uint16_t& port) {
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return -1;
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  socklen_t size = sizeof(address);
  if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
      getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size) != 0 ||
      connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) !=
          0) {
    close(fd);
    return -1;
  }
  port = ntohs(address.sin_port);
  return fd;
}

inline void Release(int& fd) {
  if (fd >= 0) close(fd);
  fd = -1;
}

// A bridge under test and the fake gateway on the other end of its UDP link,
// each on a port the OS chose. A port stays reserved (ReservePort) until the
// endpoint that owns it binds it, so a port picked earlier in the test is
// never one another process has since been given. Another process can still
// bind a port in the instant between the release and the bind; Open() then
// starts over on a new pair, and a later OpenGateway() fails the test.
struct Link {
  Link() = default;
  Link(const Link&) = delete;
  Link& operator=(const Link&) = delete;
  ~Link() {
    Release(gateway_reservation);
    Release(node_reservation);
  }

  // Opens the bridge and, when `open_gateway`, the gateway; a test whose
  // bridge must first run with no gateway listening calls OpenGateway()
  // later, keeping the gateway port reserved until then.
  bool Open(const config::RobotConfig& cfg, bool open_gateway = true) {
    for (int attempt = 0; attempt < 64; ++attempt) {
      node.reset();
      gateway.Close();
      Release(gateway_reservation);
      Release(node_reservation);
      gateway_reservation = ReservePort(gateway_port);
      node_reservation = ReservePort(node_port);
      if (gateway_reservation < 0 || node_reservation < 0) return false;
      if (open_gateway && !OpenGateway()) continue;
      Release(node_reservation);
      node.emplace(cfg, "127.0.0.1", gateway_port, node_port, true);
      if (node->Open()) return true;
    }
    return false;
  }

  bool OpenGateway() {
    Release(gateway_reservation);
    return gateway.Open("127.0.0.1", gateway_port, "127.0.0.1", node_port) ==
           protocol::UdpStatus::kOk;
  }

  std::optional<HardwareNode> node;
  protocol::RuntimeUdpPeer gateway;
  uint16_t gateway_port = 0;
  uint16_t node_port = 0;
  int gateway_reservation = -1;
  int node_reservation = -1;
};

inline constexpr std::string_view kTestToml = R"(
[robot]
name = "test_robot"
period_us = 5000

[subsystems.drivetrain]
period_us = 5000

[subsystems.drivetrain.motors.drive]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.shooter]
period_us = 10000

[subsystems.shooter.motors.flywheel]
type = "TalonFX"
bus = "rio"
can_id = 2

[subsystems.driver]
period_us = 20000

[subsystems.beam_break]
period_us = 5000

[subsystems.beam_break.digital_inputs.gate]
dio = 3
)";

// A --sim node forces commissioned, which participates in the config ID.
inline uint64_t NodeConfigId(const talos::config::RobotConfig& cfg) {
  Config hw = cfg.hardware;
  hw.commissioned = true;
  return ConfigurationId(hw);
}

inline constexpr std::string_view kTrustToml = R"(
[robot]
name = "trust_robot"
period_us = 5000

[subsystems.drivetrain]
period_us = 5000

[subsystems.drivetrain.motors.drive]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.intake]
period_us = 5000

[subsystems.intake.motors.roller]
type = "TalonFX"
bus = "rio"
can_id = 2

[subsystems.intake.digital_outputs.beacon]
dio = 1

[subsystems.intake.pwm_outputs.strip]
channel = 0
)";

inline void PublishCommand(ipc::Publisher<talos::hardware::Packet>& pub,
                           const Command& cmd) {
  talos::hardware::Packet pkt{};
  pkt.size = static_cast<uint32_t>(Encode(cmd, pkt.data));
  pub.write(pkt);
}

// Sends one controller state frame from a fake gateway, then ticks until the
// node ingests it. False means the state never arrived (a test failure, not a
// hang).
inline bool InjectState(HardwareNode& node, protocol::RuntimeUdpPeer& gateway,
                        const State& state, uint64_t now_us) {
  talos::hardware::Packet buf{};
  buf.size = static_cast<uint32_t>(Encode(state, buf.data));
  if (gateway.SendFrame(protocol::FrameType::kHardwareState, 1, now_us,
                        buf.data.data(),
                        buf.size) != protocol::UdpStatus::kOk) {
    return false;
  }
  const uint64_t before = node.states_received();
  for (int i = 0; i < 200; ++i) {
    node.Tick(now_us);
    if (node.states_received() > before) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

// UDP send completion does not imply that the peer's receive queue is ready,
// even on loopback (notably on BSD). Wait for the protocol event, without
// advancing the bridge's simulated clock or causing another retry.
inline unsigned ReceiveConfigs(protocol::RuntimeUdpPeer& gateway) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  protocol::DecodedFrame frame;
  while (protocol::testing::ReceiveFrame(gateway, &frame, deadline) ==
         protocol::UdpStatus::kOk) {
    if (frame.header.type != protocol::FrameType::kHardwareConfig) continue;
    unsigned count = 1;
    while (gateway.TryReceive(&frame) == protocol::UdpStatus::kOk)
      count += frame.header.type == protocol::FrameType::kHardwareConfig;
    return count;
  }
  return 0;
}

inline State TrustState(uint64_t config_id, uint64_t boot_id, uint64_t epoch,
                        uint64_t sample_time_us) {
  State state{};
  state.config_id = config_id;
  state.boot_id = boot_id;
  state.epoch = epoch;
  state.sample_time_us = sample_time_us;
  state.flags = kConfigured | kEnabled;
  return state;
}

// Polls `done` until it holds; false after 10 s, a bound on a hang only.
template <class Done>
inline bool WaitFor(Done done) {
  const auto give_up =
      std::chrono::steady_clock::now() + std::chrono::seconds{10};
  while (!done()) {
    if (std::chrono::steady_clock::now() >= give_up) return false;
    std::this_thread::sleep_for(std::chrono::microseconds{100});
  }
  return true;
}

// Sends one State from the fake gateway without ticking the bridge, for a
// bridge that Run() drives.
inline bool SendState(protocol::RuntimeUdpPeer& gateway, const State& state) {
  Packet packet{};
  packet.size = static_cast<uint32_t>(Encode(state, packet.data));
  return gateway.SendFrame(protocol::FrameType::kHardwareState, 0,
                           state.sample_time_us, packet.data.data(),
                           packet.size) == protocol::UdpStatus::kOk;
}

// Whether /hw/state now carries the State sampled at `sample_time_us`.
inline bool Forwarded(ipc::Subscriber<Packet>& states,
                      uint64_t sample_time_us) {
  const auto packet = states.read();
  State state{};
  return packet && Decode(packet->bytes(), state) &&
         state.sample_time_us == sample_time_us;
}

// A command the fake gateway received, with the bridge-clock time it was
// sent at (the frame header's).
struct SentCommand {
  uint64_t sent_us;
  Command command;
};

// The next command frame the gateway receives, skipping every other frame;
// nullopt after 10 s, a bound on a hang only.
inline std::optional<SentCommand> NextCommand(
    protocol::RuntimeUdpPeer& gateway) {
  const auto give_up =
      std::chrono::steady_clock::now() + std::chrono::seconds{10};
  protocol::DecodedFrame frame;
  while (protocol::testing::ReceiveFrame(gateway, &frame, give_up) ==
         protocol::UdpStatus::kOk) {
    if (frame.header.type != protocol::FrameType::kHardwareCommand) continue;
    SentCommand sent{frame.header.monotonic_time_us, {}};
    if (!Decode(std::span<const uint8_t>{frame.payload, frame.payload_size},
                sent.command)) {
      return std::nullopt;
    }
    return sent;
  }
  return std::nullopt;
}

// The merged command's entry for motor `id`; a neutral default when absent.
inline MotorRequest MotorOf(const Command& command, uint16_t id) {
  for (std::size_t i = 0; i < command.count; ++i) {
    if (command.motors[i].id == id) return command.motors[i];
  }
  return MotorRequest{};
}

// An intake request built on the State of boot 11, epoch 1, sampled at 1000.
inline Command IntakeRequest(const config::RobotConfig& cfg, double demand) {
  Command request{};
  request.config_id = NodeConfigId(cfg);
  request.boot_id = 11;
  request.epoch = 1;
  request.observed_time_us = 1000;
  request.count = 1;
  request.motors[0] = {cfg.subsystems.at("intake").motors[0], Mode::kVoltage, 0,
                       demand, 0.0};
  return request;
}

}  // namespace talos::hardware::bridge_test

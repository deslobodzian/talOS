#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "node.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/protocol/udp_test_helpers.h"

namespace talos::hardware {
namespace {
// Resolve each endpoint label once per process to an OS-selected port.
uint16_t TestPort(uint16_t label) {
  static std::map<uint16_t, uint16_t> ports;
  if (const auto it = ports.find(label); it != ports.end()) return it->second;
  for (;;) {
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) throw std::runtime_error("cannot allocate test UDP socket");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t size = sizeof(address);
    const bool ok =
        bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0 &&
        getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size) == 0;
    close(fd);
    if (!ok) throw std::runtime_error("cannot allocate test UDP port");
    const uint16_t port = ntohs(address.sin_port);
    bool used = false;
    for (const auto& [key, value] : ports) used |= value == port;
    if (used) continue;
    ports.emplace(label, port);
    return port;
  }
}

constexpr std::string_view kTestToml = R"(
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

// The loopback node forces commissioned, which participates in the config ID.
uint64_t NodeConfigId(const talos::config::RobotConfig& cfg) {
  Config hw = cfg.hardware;
  hw.commissioned = true;
  return ConfigurationId(hw);
}

TEST(HardwareNodeTest, RejectsMalformedDriverStationPackets) {
  auto cfg = talos::config::ParseRobotConfigString(kTestToml);
  HardwareNode node{cfg, "127.0.0.1", TestPort(25804), TestPort(25805), true};
  ASSERT_TRUE(node.Open());

  protocol::RuntimeUdpPeer gateway_peer;
  ASSERT_EQ(gateway_peer.Open("127.0.0.1", TestPort(25804), "127.0.0.1",
                              TestPort(25805)),
            protocol::UdpStatus::kOk);

  ipc::Subscriber<talos::hardware::Packet> ds_sub{kDriverStationTopic};

  driver_station::DriverStationData data{};
  data.sample_time_us = 1000;
  std::vector<uint8_t> dummy_payload(1024);
  dummy_payload.resize(driver_station::Encode(data, dummy_payload));
  ASSERT_FALSE(dummy_payload.empty());
  ASSERT_EQ(gateway_peer.SendFrame(protocol::FrameType::kDriverStation, 1, 1000,
                                   dummy_payload.data(), dummy_payload.size()),
            protocol::UdpStatus::kOk);

  for (int i = 0; i < 20; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    node.Tick(2000 + i * 1000);
    if (node.driver_station_packets_received() > 0) break;
  }

  EXPECT_EQ(node.driver_station_packets_received(), 1u);
  auto pkt = ds_sub.read();
  ASSERT_TRUE(pkt.has_value());
  const talos::hardware::Packet packet =
      pkt.value_or(talos::hardware::Packet{});
  EXPECT_EQ(packet.size, dummy_payload.size());
  EXPECT_EQ(std::memcmp(packet.data.data(), dummy_payload.data(), packet.size),
            0);

  const auto send = [&](const uint8_t* bytes, std::size_t size) {
    ASSERT_EQ(gateway_peer.SendFrame(protocol::FrameType::kDriverStation, 2,
                                     2000, bytes, size),
              protocol::UdpStatus::kOk);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    node.Tick(30000);
  };
  const uint8_t malformed[] = {1, 2, 3};
  send(malformed, sizeof malformed);
  EXPECT_EQ(node.driver_station_packets_received(), 1u);
  EXPECT_FALSE(ds_sub.read());
}

constexpr std::string_view kTrustToml = R"(
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

void PublishCommand(ipc::Publisher<talos::hardware::Packet>& pub,
                    const Command& cmd) {
  talos::hardware::Packet pkt{};
  pkt.size = static_cast<uint32_t>(Encode(cmd, pkt.data));
  pub.write(pkt);
}

// Sends one controller state frame from a fake gateway, then ticks until the
// node ingests it. False means the state never arrived (a test failure, not a
// hang).
bool InjectState(HardwareNode& node, protocol::RuntimeUdpPeer& gateway,
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
unsigned ReceiveConfigs(protocol::RuntimeUdpPeer& gateway) {
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

State TrustState(uint64_t config_id, uint64_t boot_id, uint64_t epoch,
                 uint64_t sample_time_us) {
  State state{};
  state.config_id = config_id;
  state.boot_id = boot_id;
  state.epoch = epoch;
  state.sample_time_us = sample_time_us;
  state.flags = kConfigured | kEnabled;
  return state;
}

TEST(HardwareNodeTest, LastSubsystemTimeoutSendsOneFreshNeutralFrame) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  HardwareNode node{cfg, "127.0.0.1", TestPort(26000), TestPort(26001), true};
  ASSERT_TRUE(node.Open());
  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(26000), "127.0.0.1", TestPort(26001)),
      protocol::UdpStatus::kOk);
  const auto config_id = NodeConfigId(cfg);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 1000), 1000000));
  ipc::Publisher<Packet> pub{RequestTopic("intake")};
  Command request{};
  request.config_id = config_id;
  request.boot_id = 11;
  request.epoch = 1;
  request.observed_time_us = 1000;
  request.count = 1;
  request.motors[0] = {cfg.subsystems.at("intake").motors[0], Mode::kVoltage, 0,
                       4.0, 1.0};
  request.digital_output_count = 1;
  request.digital_outputs[0] = {cfg.hardware.digital_outputs[0].id, true};
  request.pwm_output_count = 1;
  request.pwm_outputs[0] = {cfg.hardware.pwm_outputs[0].id, 0.7};
  PublishCommand(pub, request);
  node.Tick(1001000);
  ASSERT_EQ(node.commands_sent(), 1u);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 12000), 1011000));
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));

  // The last active subsystem expires before the heartbeat is due. Its neutral
  // command must reference the new state, not the old request's observation.
  node.Tick(1011001);
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  ASSERT_EQ(node.commands_sent(), 2u);

  const auto receive_commands = [&] {
    std::vector<Command> commands;
    for (int i = 0; i < 20; ++i) {
      protocol::DecodedFrame frame;
      while (gateway.TryReceive(&frame) == protocol::UdpStatus::kOk) {
        if (frame.header.type != protocol::FrameType::kHardwareCommand)
          continue;
        Command command{};
        EXPECT_TRUE(
            Decode(std::span<const uint8_t>{frame.payload, frame.payload_size},
                   command));
        commands.push_back(command);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return commands;
  };
  const auto commands = receive_commands();
  ASSERT_EQ(commands.size(), 2u);
  EXPECT_EQ(commands[0].motors[1].mode, Mode::kVoltage);
  const auto& neutral = commands[1];
  EXPECT_EQ(neutral.config_id, config_id);
  EXPECT_EQ(neutral.boot_id, 11u);
  EXPECT_EQ(neutral.epoch, 1u);
  EXPECT_EQ(neutral.observed_time_us, 12000u);
  for (std::size_t i = 0; i < neutral.count; ++i) {
    EXPECT_EQ(neutral.motors[i].mode, Mode::kNeutral);
    EXPECT_DOUBLE_EQ(neutral.motors[i].demand, 0);
    EXPECT_DOUBLE_EQ(neutral.motors[i].feedforward_v, 0);
  }
  ASSERT_EQ(neutral.digital_output_count, 1u);
  EXPECT_FALSE(neutral.digital_outputs[0].value);
  ASSERT_EQ(neutral.pwm_output_count, 1u);
  EXPECT_DOUBLE_EQ(neutral.pwm_outputs[0].output, 0);

  for (uint64_t now_us = 1012000; now_us <= 1112000; now_us += 1000) {
    node.Tick(now_us);
  }
  EXPECT_EQ(node.commands_sent(), 2u);
  EXPECT_TRUE(receive_commands().empty());
}

TEST(HardwareNodeTest, SubsystemTimeoutInheritsRobotPeriod) {
  auto cfg = config::ParseRobotConfigString(R"(
[robot]
period_us = 20000
[subsystems.mech.motors.motor]
can_id = 1
)");
  HardwareNode node{cfg, "127.0.0.1", TestPort(26002), TestPort(26003), true};
  ASSERT_TRUE(node.Open());
  ipc::Publisher<Packet> pub{RequestTopic("mech")};
  Command request{};
  request.config_id = NodeConfigId(cfg);
  request.count = 1;
  request.motors[0] = {cfg.hardware.motors[0].id, Mode::kVoltage, 0, 4.0, 0.0};
  PublishCommand(pub, request);
  node.Tick(1000);
  node.Tick(11001);  // Past the former hardcoded 2 * 5000 us timeout.
  EXPECT_FALSE(node.is_subsystem_timed_out("mech"));
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kVoltage);
  node.Tick(41000);
  EXPECT_FALSE(node.is_subsystem_timed_out("mech"));
  node.Tick(41001);
  EXPECT_TRUE(node.is_subsystem_timed_out("mech"));
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kNeutral);
}

TEST(HardwareNodeTest, SimulationRejectsNonLoopback) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  EXPECT_THROW((HardwareNode{cfg, "10.56.87.2", 5802, 5803, true}),
               std::invalid_argument);
  HardwareNode real{cfg, "10.56.87.2"};
  EXPECT_EQ(real.current_command().config_id, ConfigurationId(cfg.hardware));
  HardwareNode loopback{cfg};
  EXPECT_EQ(loopback.current_command().config_id, NodeConfigId(cfg));
  EXPECT_NE(loopback.current_command().config_id,
            real.current_command().config_id);
}

TEST(HardwareNodeTest, LatestRequestNeutralizesOmittedOutputs) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  HardwareNode node{cfg, "127.0.0.1", TestPort(25940), TestPort(25941), true};
  ASSERT_TRUE(node.Open());
  node.Tick(1000000);
  ipc::Publisher<Packet> pub{RequestTopic("intake")};
  Command request{};
  request.config_id = NodeConfigId(cfg);
  request.count = 1;
  request.motors[0] = {cfg.subsystems.at("intake").motors[0], Mode::kVoltage, 0,
                       4.0, 0.0};
  request.digital_output_count = 1;
  request.digital_outputs[0] = {cfg.hardware.digital_outputs[0].id, true};
  request.pwm_output_count = 1;
  request.pwm_outputs[0] = {cfg.hardware.pwm_outputs[0].id, 0.7};
  PublishCommand(pub, request);
  node.Tick(1001000);
  EXPECT_TRUE(node.current_command().digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(node.current_command().pwm_outputs[0].output, 0.7);
  request.count = 0;
  request.digital_output_count = 0;
  request.pwm_output_count = 0;
  PublishCommand(pub, request);
  node.Tick(1002000);
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_FALSE(node.current_command().digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(node.current_command().pwm_outputs[0].output, 0.0);
  EXPECT_EQ(node.current_command().motors[1].mode, Mode::kNeutral);
}

TEST(HardwareNodeTest, ReportsLatestGatewayHealthAndConfiguredMotorFaults) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  HardwareNode node{cfg, "127.0.0.1", TestPort(25942), TestPort(25943), true};
  ASSERT_TRUE(node.Open());
  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(25942), "127.0.0.1", TestPort(25943)),
      protocol::UdpStatus::kOk);
  State state = TrustState(NodeConfigId(cfg), 42, 1, 1000);
  state.health = {true, 9.5, true, 0.81};
  state.motor_count = 2;
  state.motors[0].id = cfg.hardware.motors[1].id;
  state.motors[0].valid = true;
  state.motors[0].fault_flags = 5;
  state.motors[1].id = cfg.hardware.motors[0].id;
  state.motors[1].valid = true;
  ASSERT_TRUE(InjectState(node, gateway, state, 1000000));
  const auto status = node.Status(1001000);
  EXPECT_TRUE(status.health_valid());
  EXPECT_DOUBLE_EQ(status.battery_v(), 9.5);
  EXPECT_TRUE(status.brownout());
  EXPECT_DOUBLE_EQ(status.can_utilization_frac(), 0.81);
  EXPECT_EQ(status.motor_fault_mask(), 2u);
  state.health = {};
  state.motors[0].fault_flags = 0;
  state.sample_time_us++;
  ASSERT_TRUE(InjectState(node, gateway, state, 1002000));
  EXPECT_FALSE(node.Status(1002000).health_valid());
  EXPECT_EQ(node.Status(1002000).motor_fault_mask(), 0u);
  // An invalid sample needs attention even without a controller fault code.
  // IDs are deliberately in reverse order: mask bits follow configuration.
  state.motors[1].valid = false;
  state.sample_time_us++;
  ASSERT_TRUE(InjectState(node, gateway, state, 1003000));
  EXPECT_EQ(node.Status(1003000).motor_fault_mask(), 1u);
  state.motors[1].valid = true;
  state.sample_time_us++;
  ASSERT_TRUE(InjectState(node, gateway, state, 1004000));
  EXPECT_EQ(node.Status(1004000).motor_fault_mask(), 0u);
}

TEST(HardwareNodeTest, LogOptionRecordsSentCommandsAndReceivedStates) {
  const std::string base = ::testing::TempDir() + "bridge";
  const std::string config_path = base + ".toml";
  const std::string log_path = base + ".tlog";
  {
    std::ofstream file(config_path);
    file << kTrustToml;
  }
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  ASSERT_TRUE(runfiles) << error;
  const char* executable_path = std::getenv("HARDWARE_NODE");
  ASSERT_NE(executable_path, nullptr);
  const auto executable = runfiles->Rlocation(executable_path);
  // Keep both ephemeral ports reserved until their respective owners open.
  // Fixed ports collide when Bazel runs repetitions concurrently.
  const auto reserve_port = [](int& fd, uint16_t& port) {
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return false;
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      close(fd);
      return false;
    }
    socklen_t size = sizeof(address);
    if (getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size) != 0) {
      close(fd);
      return false;
    }
    port = ntohs(address.sin_port);
    return true;
  };
  int gateway_reservation = -1;
  int node_reservation = -1;
  uint16_t gateway_port = 0;
  uint16_t node_port = 0;
  ASSERT_TRUE(reserve_port(gateway_reservation, gateway_port));
  const bool reserved_node = reserve_port(node_reservation, node_port);
  close(gateway_reservation);
  ASSERT_TRUE(reserved_node);
  const std::string gateway_port_arg = std::to_string(gateway_port);
  const std::string node_port_arg = std::to_string(node_port);
  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", gateway_port, "127.0.0.1", node_port),
            protocol::UdpStatus::kOk);
  ipc::Publisher<Packet> pub{RequestTopic("intake")};
  close(node_reservation);
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    execl(executable.c_str(), executable.c_str(), "--config",
          config_path.c_str(), "--sim", "--remote-port",
          gateway_port_arg.c_str(), "--local-port", node_port_arg.c_str(),
          "--duration-s", "1", "--log", log_path.c_str(), nullptr);
    _exit(127);
  }
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  State state = TrustState(NodeConfigId(cfg), 42, 1, 1000);
  Packet packet{};
  packet.size = static_cast<uint32_t>(Encode(state, packet.data));
  Command request{};
  request.config_id = state.config_id;
  request.boot_id = state.boot_id;
  request.epoch = state.epoch;
  request.observed_time_us = state.sample_time_us;
  request.count = 1;
  request.motors[0] = {cfg.subsystems.at("intake").motors[0], Mode::kVoltage, 0,
                       3.5, 0.0};
  bool received_command = false;
  bool published_request = false;
  for (int i = 0; i < 150; ++i) {
    gateway.SendFrame(protocol::FrameType::kHardwareState, 0, 1000,
                      packet.data.data(), packet.size);
    protocol::DecodedFrame frame;
    while (gateway.TryReceive(&frame) == protocol::UdpStatus::kOk) {
      EXPECT_EQ(frame.header.flags, 0u);
      // Configuration is sent only after accepting the gateway state. Wait
      // for that identity barrier, then publish exactly once. Re-publishing
      // while waiting for UDP receipt could legitimately reactivate a demand
      // after its first subsystem timeout on a delayed parent process.
      if (frame.header.type == protocol::FrameType::kHardwareConfig &&
          !published_request) {
        PublishCommand(pub, request);
        published_request = true;
      }
      if (frame.header.type == protocol::FrameType::kHardwareCommand) {
        received_command = true;
      }
    }
    if (received_command) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  int status = 0;
  ASSERT_EQ(waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  ASSERT_EQ(WEXITSTATUS(status), 0);
  ASSERT_TRUE(received_command);
  ASSERT_TRUE(published_request);
  event::log::LogReader reader(log_path);
  bool logged_command = false;
  bool logged_neutral = false;
  bool logged_state = false;
  event::log::LogReader::Record record;
  while (reader.next(record)) {
    ASSERT_LT(record.header.source_id, reader.manifest().size());
    const auto& source = reader.manifest()[record.header.source_id];
    ASSERT_EQ(record.payload.size(), sizeof(Packet));
    Packet logged{};
    std::memcpy(&logged, record.payload.data(), sizeof(logged));
    if (source.name == kCommandTopic) {
      Command command{};
      ASSERT_TRUE(Decode(logged.bytes(), command));
      EXPECT_EQ(command.boot_id, 42u);
      // One publication may produce active heartbeats, then a final neutral
      // command. With no subsequent requests it must never reactivate.
      if (command.motors[1].mode == Mode::kVoltage) {
        EXPECT_FALSE(logged_neutral);
        EXPECT_DOUBLE_EQ(command.motors[1].demand, 3.5);
        logged_command = true;
      } else {
        EXPECT_TRUE(logged_command);
        EXPECT_EQ(command.motors[1].mode, Mode::kNeutral);
        EXPECT_DOUBLE_EQ(command.motors[1].demand, 0.0);
        logged_neutral = true;
      }
    } else if (source.name == kStateTopic) {
      State decoded{};
      ASSERT_TRUE(Decode(logged.bytes(), decoded));
      EXPECT_EQ(decoded.boot_id, 42u);
      logged_state = true;
    }
  }
  EXPECT_TRUE(logged_command);
  EXPECT_TRUE(logged_neutral);
  EXPECT_TRUE(logged_state);
}

TEST(HardwareNodeTest, RejectsRequestsForDevicesOwnedByAnotherSubsystem) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  ASSERT_EQ(cfg.hardware.motors.size(), 2u);
  ASSERT_EQ(cfg.hardware.digital_outputs.size(), 1u);
  ASSERT_EQ(cfg.hardware.pwm_outputs.size(), 1u);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint16_t beacon = cfg.hardware.digital_outputs[0].id;
  const uint16_t strip = cfg.hardware.pwm_outputs[0].id;
  ASSERT_TRUE(cfg.subsystems.at("drivetrain").motors.size() == 1u &&
              cfg.subsystems.at("drivetrain").motors[0] == drive);
  ASSERT_TRUE(cfg.subsystems.at("intake").motors.size() == 1u &&
              cfg.subsystems.at("intake").motors[0] == roller);

  HardwareNode node{cfg, "127.0.0.1", TestPort(25812), TestPort(25813), true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);  // Drain any stale packet a previous test left behind.

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(25812), "127.0.0.1", TestPort(25813)),
      protocol::UdpStatus::kOk);
  const uint64_t config_id = NodeConfigId(cfg);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 1000), 1000000));

  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};

  // An intake request that accidentally addresses the drivetrain motor: the
  // whole request is rejected, so neither the stray drive demand nor the
  // intake's own roller demand merges.
  Command cross{};
  cross.config_id = config_id;
  cross.boot_id = 11;
  cross.epoch = 1;
  cross.observed_time_us = 1000;
  cross.count = 2;
  cross.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};
  cross.motors[1] = {drive, Mode::kVelocity, 0, 99.0, 0.0};
  PublishCommand(intake_pub, cross);
  node.Tick(1001000);

  EXPECT_EQ(node.ownership_rejected(), 1u);
  EXPECT_EQ(node.identity_rejected(), 0u);
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  const Command& cmd1 = node.current_command();
  EXPECT_EQ(cmd1.motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(cmd1.motors[1].mode, Mode::kNeutral);

  // Same for the other actuator kinds: drivetrain must not drive the intake's
  // digital output or PWM channel.
  Command cross_do{};
  cross_do.config_id = config_id;
  cross_do.boot_id = 11;
  cross_do.epoch = 1;
  cross_do.observed_time_us = 1000;
  cross_do.count = 1;
  cross_do.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  cross_do.digital_output_count = 1;
  cross_do.digital_outputs[0] = {beacon, true};
  PublishCommand(drive_pub, cross_do);
  node.Tick(1002000);

  EXPECT_EQ(node.ownership_rejected(), 2u);
  EXPECT_EQ(cmd1.motors[0].mode, Mode::kNeutral);
  EXPECT_FALSE(node.current_command().digital_outputs[0].value);

  Command cross_pwm{};
  cross_pwm.config_id = config_id;
  cross_pwm.boot_id = 11;
  cross_pwm.epoch = 1;
  cross_pwm.observed_time_us = 1000;
  cross_pwm.count = 1;
  cross_pwm.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  cross_pwm.pwm_output_count = 1;
  cross_pwm.pwm_outputs[0] = {strip, 0.9};
  PublishCommand(drive_pub, cross_pwm);
  node.Tick(1003000);

  EXPECT_EQ(node.ownership_rejected(), 3u);
  EXPECT_DOUBLE_EQ(node.current_command().pwm_outputs[0].output, 0.0);

  // Positive control: requests that stay inside their own ownership still
  // merge.
  Command intake_ok{};
  intake_ok.config_id = config_id;
  intake_ok.boot_id = 11;
  intake_ok.epoch = 1;
  intake_ok.observed_time_us = 1000;
  intake_ok.count = 1;
  intake_ok.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};
  intake_ok.digital_output_count = 1;
  intake_ok.digital_outputs[0] = {beacon, true};
  intake_ok.pwm_output_count = 1;
  intake_ok.pwm_outputs[0] = {strip, 0.5};
  PublishCommand(intake_pub, intake_ok);
  Command drive_ok{};
  drive_ok.config_id = config_id;
  drive_ok.boot_id = 11;
  drive_ok.epoch = 1;
  drive_ok.observed_time_us = 1000;
  drive_ok.count = 1;
  drive_ok.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  PublishCommand(drive_pub, drive_ok);
  node.Tick(1004000);

  EXPECT_EQ(node.ownership_rejected(), 3u);
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  const Command& cmd2 = node.current_command();
  EXPECT_DOUBLE_EQ(cmd2.motors[0].demand, 15.0);
  EXPECT_DOUBLE_EQ(cmd2.motors[1].demand, 30.0);
  EXPECT_TRUE(cmd2.digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(cmd2.pwm_outputs[0].output, 0.5);
}

// A request built from an older sample of the same boot and epoch stays
// acceptable while its gateway-clock age (last_sample - observed) is below
// the command timeout, and is rejected beyond it. Either way the merge keeps
// the older observation instead of being re-stamped: stale stays visibly
// stale.
TEST(HardwareNodeTest, RequestAgeIsBoundedByTheCommandTimeout) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);
  const uint64_t timeout_us = cfg.hardware.command_timeout_us;

  HardwareNode node{cfg, "127.0.0.1", TestPort(25816), TestPort(25817), true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(25816), "127.0.0.1", TestPort(25817)),
      protocol::UdpStatus::kOk);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 5000), 1000000));

  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  Command fresh{};
  fresh.config_id = config_id;
  fresh.boot_id = 11;
  fresh.epoch = 1;
  fresh.observed_time_us = 5000;
  fresh.count = 1;
  fresh.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};
  PublishCommand(intake_pub, fresh);
  node.Tick(1001000);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
  EXPECT_EQ(node.current_command().observed_time_us, 5000u);

  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 6000), 1002000));
  PublishCommand(intake_pub, fresh);
  node.Tick(1002500);
  EXPECT_EQ(node.identity_rejected(), 0u);
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().observed_time_us, 5000u);

  // A newer accepted observation retires the previous demand even while
  // both observations remain inside the age bound.
  Command newer = fresh;
  newer.observed_time_us = 6000;
  newer.motors[0].demand = 31.0;
  PublishCommand(intake_pub, newer);
  node.Tick(1002600);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 31.0);
  EXPECT_EQ(node.current_command().observed_time_us, 6000u);
  PublishCommand(intake_pub, fresh);
  node.Tick(1002700);
  EXPECT_EQ(node.identity_rejected(), 1u);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 31.0);
  EXPECT_EQ(node.current_command().observed_time_us, 6000u);

  const uint64_t stale_sample = 5000 + timeout_us + 1000;
  ASSERT_TRUE(InjectState(node, gateway,
                          TrustState(config_id, 11, 1, stale_sample), 1003000));
  PublishCommand(intake_pub, fresh);
  node.Tick(1003500);
  EXPECT_EQ(node.identity_rejected(), 2u);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 31.0);
  EXPECT_EQ(node.current_command().observed_time_us, 6000u);
}

TEST(HardwareNodeTest, FreshSubsystemCannotRefreshStaleSubsystem) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  HardwareNode node{cfg, "127.0.0.1", TestPort(25824), TestPort(25825), true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(25824), "127.0.0.1", TestPort(25825)),
      protocol::UdpStatus::kOk);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 5000), 1000000));

  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};

  Command d{};
  d.config_id = config_id;
  d.boot_id = 11;
  d.epoch = 1;
  d.observed_time_us = 5000;
  d.count = 1;
  d.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  Command r{};
  r.config_id = config_id;
  r.boot_id = 11;
  r.epoch = 1;
  r.observed_time_us = 5000;
  r.count = 1;
  r.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};
  PublishCommand(drive_pub, d);
  PublishCommand(intake_pub, r);
  node.Tick(1001000);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().observed_time_us, 5000u);

  // A newer sample arrives. Only drivetrain publishes from it; intake stays
  // quiet but within its bridge liveness timeout, so its cached demand still
  // drives the merge. The merged observation must stay at intake's 5000:
  // drivetrain's freshness must not refresh intake's stale demand.
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 6000), 1002000));
  Command d_new = d;
  d_new.observed_time_us = 6000;
  d_new.motors[0].demand = 16.0;
  PublishCommand(drive_pub, d_new);
  node.Tick(1002500);

  EXPECT_EQ(node.identity_rejected(), 0u);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 16.0);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
  EXPECT_EQ(node.current_command().observed_time_us, 5000u);

  // Intake catches up from the newer sample; now both agree and the merge
  // advances.
  Command r_new = r;
  r_new.observed_time_us = 6000;
  r_new.motors[0].demand = 31.0;
  PublishCommand(intake_pub, r_new);
  node.Tick(1003000);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 31.0);
  EXPECT_EQ(node.current_command().observed_time_us, 6000u);

  // Monotonicity is per subsystem: drivetrain replaying its own older demand
  // is rejected without disturbing intake's merged demand.
  Command d_replay = d;
  d_replay.motors[0].demand = 99.0;
  PublishCommand(drive_pub, d_replay);
  node.Tick(1003500);
  EXPECT_EQ(node.identity_rejected(), 1u);
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 16.0);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 31.0);
  EXPECT_EQ(node.current_command().observed_time_us, 6000u);
}

TEST(HardwareNodeTest, RejectsWrongIdentityWordsEvenWithinBound) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  HardwareNode node{cfg, "127.0.0.1", TestPort(25826), TestPort(25827), true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(25826), "127.0.0.1", TestPort(25827)),
      protocol::UdpStatus::kOk);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 5000), 1000000));

  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};

  // Age zero (observed == last_sample): freshness passes, so each rejection
  // below proves the config/boot/epoch words are still exact-match, and a
  // future observation is still rejected.
  Command base{};
  base.config_id = config_id;
  base.boot_id = 11;
  base.epoch = 1;
  base.observed_time_us = 5000;
  base.count = 1;
  base.motors[0] = {roller, Mode::kVelocity, 0, 30.0, 0.0};

  Command bad_config = base;
  bad_config.config_id = config_id + 1;
  PublishCommand(intake_pub, bad_config);
  node.Tick(1001000);

  Command bad_boot = base;
  bad_boot.boot_id = 12;
  PublishCommand(intake_pub, bad_boot);
  node.Tick(1002000);

  Command bad_epoch = base;
  bad_epoch.epoch = 2;
  PublishCommand(intake_pub, bad_epoch);
  node.Tick(1003000);

  Command bad_future = base;
  bad_future.observed_time_us = 6000;
  PublishCommand(intake_pub, bad_future);
  node.Tick(1004000);

  EXPECT_EQ(node.identity_rejected(), 4u);
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[1].mode, Mode::kNeutral);

  PublishCommand(intake_pub, base);
  node.Tick(1005000);
  EXPECT_EQ(node.identity_rejected(), 4u);
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
  EXPECT_EQ(node.current_command().observed_time_us, 5000u);
}

TEST(HardwareNodeTest, RequiresFreshRequestsAfterControllerTransitions) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  HardwareNode node{cfg, "127.0.0.1", TestPort(25818), TestPort(25819), true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(25818), "127.0.0.1", TestPort(25819)),
      protocol::UdpStatus::kOk);
  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};

  const auto publish_valid = [&](uint64_t boot, uint64_t epoch,
                                 uint64_t observed, double drive_demand,
                                 double roller_demand) {
    Command d{};
    d.config_id = config_id;
    d.boot_id = boot;
    d.epoch = epoch;
    d.observed_time_us = observed;
    d.count = 1;
    d.motors[0] = {drive, Mode::kVelocity, 0, drive_demand, 0.0};
    PublishCommand(drive_pub, d);
    Command r{};
    r.config_id = config_id;
    r.boot_id = boot;
    r.epoch = epoch;
    r.observed_time_us = observed;
    r.count = 1;
    r.motors[0] = {roller, Mode::kVelocity, 0, roller_demand, 0.0};
    PublishCommand(intake_pub, r);
  };

  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 1000), 1000000));
  publish_valid(11, 1, 1000, 15.0, 30.0);
  node.Tick(1001000);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 15.0);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 30.0);
  // Single tick, single send. Later phases capture the counter instead: the
  // inject loop above may itself tick-and-send while polling for the frame.
  const uint64_t sent_after_first_merge = node.commands_sent();
  EXPECT_EQ(sent_after_first_merge, 1u);

  // Reboot: new boot id. Without fresh requests nothing merges and nothing is
  // sent; the cached demands are neutralized.
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 12, 1, 2000), 1002000));
  const uint64_t sent_after_reboot = node.commands_sent();
  node.Tick(1003000);
  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(node.current_command().motors[1].mode, Mode::kNeutral);
  EXPECT_EQ(node.commands_sent(), sent_after_reboot);

  // Fresh requests stamped with the new identity resume motion.
  publish_valid(12, 1, 2000, 16.0, 31.0);
  node.Tick(1004000);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 16.0);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 31.0);
  const uint64_t sent_after_resume = node.commands_sent();
  EXPECT_EQ(sent_after_resume, sent_after_reboot + 1);

  // Disable/re-enable surfaces as an epoch bump: same treatment.
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 12, 2, 3000), 1005000));
  const uint64_t sent_after_epoch = node.commands_sent();
  node.Tick(1006000);
  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(node.commands_sent(), sent_after_epoch);

  publish_valid(12, 2, 3000, 17.0, 32.0);
  node.Tick(1007000);
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 17.0);
  const uint64_t sent_after_second_resume = node.commands_sent();
  EXPECT_EQ(sent_after_second_resume, sent_after_epoch + 1);

  // A bare enabled->disabled edge invalidates even when the epoch has not
  // moved yet.
  State disabled = TrustState(config_id, 12, 2, 4000);
  disabled.flags = kConfigured;
  ASSERT_TRUE(InjectState(node, gateway, disabled, 1008000));
  const uint64_t sent_after_disable = node.commands_sent();
  node.Tick(1009000);
  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(node.commands_sent(), sent_after_disable);

  // Fault recovery (fault bit clears, epoch advances) needs fresh requests too.
  State recovered = TrustState(config_id, 12, 3, 5000);
  ASSERT_TRUE(InjectState(node, gateway, recovered, 1010000));
  const uint64_t sent_after_recovery = node.commands_sent();
  node.Tick(1011000);
  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_EQ(node.commands_sent(), sent_after_recovery);

  publish_valid(12, 3, 5000, 18.0, 33.0);
  node.Tick(1012000);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 18.0);
  EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 33.0);
  EXPECT_EQ(node.commands_sent(), sent_after_recovery + 1);
  EXPECT_EQ(node.identity_rejected(), 0u);
  EXPECT_EQ(node.ownership_rejected(), 0u);
}

// A State is accepted only when it is newer than the last one accepted in its
// gateway boot, or opens a boot the bridge has not seen. A duplicate, a
// reordered sample (even one carrying an older epoch or enable edge) and a
// datagram delayed from a boot the gateway has since left are dropped before
// they reach identity, freshness, the cached requests, /hw/state or the config
// push. A genuine new boot still recovers.
TEST(HardwareNodeTest, ReorderedAndRetiredBootStatesChangeNothing) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint64_t config_id = NodeConfigId(cfg);

  HardwareNode node{cfg, "127.0.0.1", TestPort(26200), TestPort(26201), true};
  ASSERT_TRUE(node.Open());
  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(26200), "127.0.0.1", TestPort(26201)),
      protocol::UdpStatus::kOk);
  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Subscriber<talos::hardware::Packet> state_sub{kStateTopic};
  const auto drain_configs = [&] {
    unsigned count = 0;
    protocol::DecodedFrame frame;
    while (gateway.TryReceive(&frame) == protocol::UdpStatus::kOk)
      count += frame.header.type == protocol::FrameType::kHardwareConfig;
    return count;
  };
  const auto publish = [&](uint64_t boot, uint64_t epoch, uint64_t observed,
                           double demand) {
    Command d{};
    d.config_id = config_id;
    d.boot_id = boot;
    d.epoch = epoch;
    d.observed_time_us = observed;
    d.count = 1;
    d.motors[0] = {drive, Mode::kVelocity, 0, demand, 0.0};
    PublishCommand(drive_pub, d);
    Command r = d;
    r.motors[0] = {roller, Mode::kVelocity, 0, demand, 0.0};
    PublishCommand(intake_pub, r);
  };

  // Boot 10 ran for a long time; boot 11 restarts the gateway clock low.
  ASSERT_TRUE(InjectState(node, gateway,
                          TrustState(config_id, 10, 1, 900'000'000), 1000000));
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 2, 1000), 1001000));
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 2, 2000), 1002000));
  // Configured, so nothing but a transition pushes the configuration again.
  ConfigAckPayload ack{};
  ack.config_id = config_id;
  ack.status = ConfigAckStatus::kOk;
  std::array<uint8_t, 128> ack_bytes{};
  const auto ack_size = EncodeConfigAck(ack, ack_bytes);
  ASSERT_EQ(gateway.SendFrame(protocol::FrameType::kHardwareConfigAck, 0,
                              1002000, ack_bytes.data(), ack_size),
            protocol::UdpStatus::kOk);
  for (int i = 0; i < 200 && !node.is_configured(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    node.Tick(1002000);
  }
  ASSERT_TRUE(node.is_configured());
  publish(11, 2, 2000, 15.0);
  node.Tick(1002500);
  ASSERT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  ASSERT_FALSE(node.is_subsystem_timed_out("intake"));
  ASSERT_DOUBLE_EQ(node.current_command().motors[0].demand, 15.0);
  while (state_sub.read()) {
  }
  drain_configs();
  const uint64_t ignored_before = node.Status(1002500).clock_ignored_count();

  State duplicate_new_epoch = TrustState(config_id, 11, 3, 2000);
  State older_disabled = TrustState(config_id, 11, 1, 1500);
  older_disabled.flags = kConfigured;
  State retired = TrustState(config_id, 10, 1, 900'005'000);
  uint64_t now_us = 1003000;
  for (const State& stale : {duplicate_new_epoch, older_disabled, retired}) {
    ASSERT_TRUE(InjectState(node, gateway, stale, now_us));
    EXPECT_FALSE(state_sub.read()) << "boot " << stale.boot_id << " sample "
                                   << stale.sample_time_us << " published";
    EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
    EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
    EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 15.0);
    EXPECT_DOUBLE_EQ(node.current_command().motors[1].demand, 15.0);
    EXPECT_TRUE(node.is_configured());
    EXPECT_EQ(drain_configs(), 0u);
    now_us += 500;
  }
  const bridge::BridgeStatus status = node.Status(now_us);
  EXPECT_EQ(status.clock_ignored_count(), ignored_before + 3);
  EXPECT_EQ(status.clock_boot_id(), 11u);
  // Freshness was not refreshed by the dropped frames.
  EXPECT_EQ(status.state_age_us(), now_us - 1002000);
  // The identity still is boot 11, epoch 2, sample 2000: a request built from
  // it merges, and one built from a dropped frame's words does not.
  const uint64_t rejected_before = node.identity_rejected();
  publish(11, 2, 2000, 16.0);
  node.Tick(now_us);
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 16.0);
  EXPECT_EQ(node.identity_rejected(), rejected_before);
  publish(11, 3, 2000, 17.0);
  node.Tick(now_us + 100);
  EXPECT_EQ(node.identity_rejected(), rejected_before + 2);
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 16.0);

  // A genuine reboot: a boot never seen before is accepted, published, pushes
  // the configuration and invalidates every cached request.
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 12, 1, 500), 1010000));
  EXPECT_TRUE(state_sub.read());
  EXPECT_EQ(ReceiveConfigs(gateway), 1u);
  node.Tick(1010100);
  EXPECT_TRUE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kNeutral);
  EXPECT_EQ(node.Status(1010100).clock_boot_id(), 12u);
  publish(12, 1, 500, 18.0);
  node.Tick(1010200);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 18.0);
}

// Driver Station packets carry the gateway boot id. The bridge forwards only
// the accepted State's boot, so a low-uptime boot B's packets go through
// right after the reboot, and a boot-A packet delayed past it, or one from a
// boot whose State has not arrived, does not.
TEST(HardwareNodeTest, ForwardsDriverStationPacketsOnlyFromTheCurrentBoot) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint64_t config_id = NodeConfigId(cfg);
  HardwareNode node{cfg, "127.0.0.1", TestPort(26202), TestPort(26203), true};
  ASSERT_TRUE(node.Open());
  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(26202), "127.0.0.1", TestPort(26203)),
      protocol::UdpStatus::kOk);
  ipc::Subscriber<talos::hardware::Packet> ds_sub{kDriverStationTopic};

  // Sends one packet, ticks until the node has taken the frame in, and
  // returns the boot id of what it forwarded (nullopt: nothing).
  const auto send = [&](uint64_t boot_id, uint64_t sample_time_us,
                        uint64_t now_us) -> std::optional<uint64_t> {
    driver_station::DriverStationData data{};
    data.boot_id = boot_id;
    data.sample_time_us = sample_time_us;
    data.mode_since_us = sample_time_us;
    data.fms.flags = driver_station::kDsAttached;
    std::array<uint8_t, driver_station::kMaxEncodedSize> payload{};
    const auto size = driver_station::Encode(data, payload);
    EXPECT_GT(size, 0u);
    EXPECT_EQ(gateway.SendFrame(protocol::FrameType::kDriverStation, 1, now_us,
                                payload.data(), size),
              protocol::UdpStatus::kOk);
    // A State frame behind it marks when the node has drained the packet.
    const auto marker = TrustState(config_id, node.clock().boot_id(), 1, 0);
    talos::hardware::Packet buf{};
    buf.size = static_cast<uint32_t>(Encode(marker, buf.data));
    const uint64_t before = node.states_received();
    EXPECT_EQ(gateway.SendFrame(protocol::FrameType::kHardwareState, 1, now_us,
                                buf.data.data(), buf.size),
              protocol::UdpStatus::kOk);
    for (int i = 0; i < 200 && node.states_received() == before; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      node.Tick(now_us);
    }
    EXPECT_GT(node.states_received(), before);
    const auto pkt = ds_sub.read();
    if (!pkt) return std::nullopt;
    driver_station::DriverStationData forwarded{};
    EXPECT_TRUE(driver_station::Decode(pkt->bytes(), forwarded));
    return forwarded.boot_id;
  };
  constexpr uint64_t kBootA = 0xA1, kBootB = 0xB1, kBootC = 0xC1;

  ASSERT_TRUE(InjectState(
      node, gateway, TrustState(config_id, kBootA, 1, 900'000'000), 1000000));
  EXPECT_EQ(send(kBootA, 900'000'000, 1001000), kBootA);
  // Boot C has sent no State: its packet is not forwarded yet.
  EXPECT_EQ(send(kBootC, 500, 1002000), std::nullopt);

  // Boot B: its State is accepted, so its low-uptime packets go through.
  ASSERT_TRUE(InjectState(node, gateway, TrustState(config_id, kBootB, 1, 1000),
                          1003000));
  EXPECT_EQ(send(kBootB, 1000, 1004000), kBootB);
  EXPECT_EQ(send(kBootB, 21000, 1005000), kBootB);
  // A's packet, delayed past the reboot, and B's own reordered one: dropped.
  EXPECT_EQ(send(kBootA, 900'020'000, 1006000), std::nullopt);
  EXPECT_EQ(send(kBootB, 11000, 1007000), std::nullopt);
  EXPECT_EQ(send(kBootB, 41000, 1008000), kBootB);
  EXPECT_EQ(node.driver_station_packets_received(), 4u);
}

// /hw/status carries the bridge's counters, the age of the last controller
// state on the bridge clock, which actuator subsystems have gone quiet and
// whether the RoboRIO accepted the configuration.
TEST(HardwareNodeTest, PublishesBridgeStatus) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  HardwareNode node{cfg, "127.0.0.1", TestPort(25828), TestPort(25829), true};
  ASSERT_TRUE(node.Open());
  ipc::Subscriber<bridge::BridgeStatus> status_sub{
      bridge::topics::kStatus,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};

  // The first tick publishes. Nothing has arrived from the controller, and
  // both actuator subsystems (drivetrain bit 0, intake bit 1: sorted name
  // order) are timed out.
  node.Tick(999000);
  const bridge::BridgeStatus initial = node.Status(999000);
  EXPECT_EQ(initial.states_received_count(), 0u);
  EXPECT_EQ(initial.state_age_us(), 0u);
  EXPECT_EQ(initial.subsystem_count(), 2u);
  EXPECT_EQ(initial.timed_out_mask(), 0b11u);
  EXPECT_FALSE(initial.configured());
  auto published = status_sub.read();
  ASSERT_TRUE(published.has_value());
  const bridge::BridgeStatus first = published.value_or(bridge::BridgeStatus{});
  EXPECT_TRUE(first == initial);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(25828), "127.0.0.1", TestPort(25829)),
      protocol::UdpStatus::kOk);
  const uint64_t config_id = NodeConfigId(cfg);

  // The RoboRIO acknowledges this configuration.
  ConfigAckPayload ack{};
  ack.config_id = config_id;
  std::array<uint8_t, sizeof(ConfigAckPayload)> ack_bytes{};
  ASSERT_EQ(EncodeConfigAck(ack, ack_bytes), sizeof(ConfigAckPayload));
  ASSERT_EQ(gateway.SendFrame(protocol::FrameType::kHardwareConfigAck, 1,
                              999500, ack_bytes.data(), ack_bytes.size()),
            protocol::UdpStatus::kOk);
  for (int i = 0; i < 200 && !node.is_configured(); ++i) {
    node.Tick(999500);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(node.is_configured());
  EXPECT_TRUE(node.Status(999500).configured());

  // State age runs on the bridge clock from the frame's receipt.
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 1000), 1000000));
  EXPECT_EQ(node.Status(1000000).states_received_count(), 1u);
  EXPECT_EQ(node.Status(1000000).state_age_us(), 0u);
  EXPECT_EQ(node.Status(1003500).state_age_us(), 3500u);

  // Only the drivetrain publishes: the silent intake keeps its bit.
  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};
  Command drive_ok{};
  drive_ok.config_id = config_id;
  drive_ok.boot_id = 11;
  drive_ok.epoch = 1;
  drive_ok.observed_time_us = 1000;
  drive_ok.count = 1;
  drive_ok.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  PublishCommand(drive_pub, drive_ok);
  node.Tick(1001000);
  ASSERT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  ASSERT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.Status(1001000).timed_out_mask(), 0b10u);
  EXPECT_EQ(node.Status(1001000).commands_sent_count(), 1u);

  // Every 20 ms of bridge time, not every tick: the last publication was the
  // first tick at 999000.
  EXPECT_FALSE(status_sub.read().has_value());
  node.Tick(1018999);
  EXPECT_FALSE(status_sub.read().has_value());
  node.Tick(1019000);
  published = status_sub.read();
  ASSERT_TRUE(published.has_value());
  const bridge::BridgeStatus later = published.value_or(bridge::BridgeStatus{});
  EXPECT_TRUE(later == node.Status(1019000));
  // By now the drivetrain has been quiet past twice its period as well.
  EXPECT_EQ(later.timed_out_mask(), 0b11u);
  EXPECT_EQ(later.state_age_us(), 19000u);
  EXPECT_TRUE(later.configured());
}

TEST(HardwareNodeTest, TwoWayClockBoundNeedsNoActiveRequests) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  HardwareNode node{cfg, "127.0.0.1", TestPort(25834), TestPort(25835), true};
  ASSERT_TRUE(node.Open());
  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(25834), "127.0.0.1", TestPort(25835)),
      protocol::UdpStatus::kOk);
  node.Tick(2000000);
  protocol::DecodedFrame probe;
  ASSERT_EQ(protocol::testing::ReceiveFrame(gateway, &probe),
            protocol::UdpStatus::kOk);
  ASSERT_EQ(probe.header.type, protocol::FrameType::kClockProbe);
  const auto sent = probe.header.monotonic_time_us;
  auto state = TrustState(NodeConfigId(cfg), 11, 1, 1000);
  state.flags = 0;
  state.clock_probe_time_us = sent;
  ASSERT_TRUE(InjectState(node, gateway, state, 2005000));
  EXPECT_FALSE(node.Status(2005000).clock_offset_valid());
  node.Tick(2006000);  // First known bridge-clock instant after receipt.
  for (uint64_t i = 1; i < 8; ++i) {
    state.sample_time_us = 1000 + i * 1000;
    state.clock_probe_time_us = 0;
    ASSERT_TRUE(InjectState(node, gateway, state, 2006000 + i * 1000));
  }
  const auto status = node.Status(2013000);
  EXPECT_TRUE(status.clock_offset_valid());
  EXPECT_EQ(status.clock_offset_us(), 2002000);
  EXPECT_EQ(status.clock_min_round_trip_us(), 6000u);
  EXPECT_GE(status.clock_offset_uncertainty_us(), 3000u);
  EXPECT_EQ(node.commands_sent(), 0u);
}

// A timed-out subsystem neutralizes exactly its own motors, digital outputs
// and PWM channels; another subsystem's merged demand stays in place.
TEST(HardwareNodeTest, TimeoutNeutralizesOnlyTheSilentSubsystemsActuators) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  const uint16_t beacon = cfg.hardware.digital_outputs[0].id;
  const uint16_t strip = cfg.hardware.pwm_outputs[0].id;
  HardwareNode node{cfg, "127.0.0.1", TestPort(25830), TestPort(25831), true};
  ASSERT_TRUE(node.Open());
  node.Tick(999000);

  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(25830), "127.0.0.1", TestPort(25831)),
      protocol::UdpStatus::kOk);
  const uint64_t config_id = NodeConfigId(cfg);
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 1000), 1000000));

  ipc::Publisher<talos::hardware::Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Publisher<talos::hardware::Packet> intake_pub{RequestTopic("intake")};
  Command drive_ok{};
  drive_ok.config_id = config_id;
  drive_ok.boot_id = 11;
  drive_ok.epoch = 1;
  drive_ok.observed_time_us = 1000;
  drive_ok.count = 1;
  drive_ok.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  Command intake_ok = drive_ok;
  intake_ok.motors[0] = {roller, Mode::kDutyCycle, 0, 0.4, 0.0};
  intake_ok.digital_output_count = 1;
  intake_ok.digital_outputs[0] = {beacon, true};
  intake_ok.pwm_output_count = 1;
  intake_ok.pwm_outputs[0] = {strip, 0.5};
  PublishCommand(drive_pub, drive_ok);
  PublishCommand(intake_pub, intake_ok);
  node.Tick(1001000);

  const Command& merged = node.current_command();
  ASSERT_EQ(merged.count, 2u);
  EXPECT_EQ(merged.motors[0].id, drive);
  EXPECT_EQ(merged.motors[0].mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(merged.motors[0].demand, 15.0);
  EXPECT_EQ(merged.motors[1].id, roller);
  EXPECT_EQ(merged.motors[1].mode, Mode::kDutyCycle);
  EXPECT_DOUBLE_EQ(merged.motors[1].demand, 0.4);
  EXPECT_TRUE(merged.digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(merged.pwm_outputs[0].output, 0.5);

  // The intake goes quiet past twice its 5 ms period; the drivetrain keeps
  // publishing.
  PublishCommand(drive_pub, drive_ok);
  node.Tick(1006000);
  PublishCommand(drive_pub, drive_ok);
  node.Tick(1011001);

  EXPECT_TRUE(node.is_subsystem_timed_out("intake"));
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  const Command& neutral = node.current_command();
  EXPECT_EQ(neutral.motors[0].id, drive);
  EXPECT_EQ(neutral.motors[0].mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(neutral.motors[0].demand, 15.0);
  EXPECT_EQ(neutral.motors[1].id, roller);
  EXPECT_EQ(neutral.motors[1].mode, Mode::kNeutral);
  EXPECT_DOUBLE_EQ(neutral.motors[1].demand, 0.0);
  EXPECT_EQ(neutral.digital_outputs[0].id, beacon);
  EXPECT_FALSE(neutral.digital_outputs[0].value);
  EXPECT_EQ(neutral.pwm_outputs[0].id, strip);
  EXPECT_DOUBLE_EQ(neutral.pwm_outputs[0].output, 0.0);
  EXPECT_EQ(node.Status(1011001).timed_out_mask(), 0b10u);
  PublishCommand(intake_pub, intake_ok);
  node.Tick(1012000);
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.current_command().motors[1].mode, Mode::kDutyCycle);
  EXPECT_TRUE(node.current_command().digital_outputs[0].value);
  EXPECT_DOUBLE_EQ(node.current_command().pwm_outputs[0].output, 0.5);
}

TEST(HardwareNodeTest, FailedConfigPushBacksOffWithoutStoppingCommands) {
  auto cfg = config::ParseRobotConfigString(kTestToml);
  HardwareNode node{cfg, "127.0.0.1", TestPort(26180), TestPort(26181), true};
  ASSERT_TRUE(node.Open());
  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(
      gateway.Open("127.0.0.1", TestPort(26180), "127.0.0.1", TestPort(26181)),
      protocol::UdpStatus::kOk);
  const auto id = NodeConfigId(cfg);
  uint64_t now = 1000000;
  ASSERT_TRUE(InjectState(node, gateway, TrustState(id, 11, 1, now), now));
  const auto drain_configs = [&] {
    unsigned count = 0;
    protocol::DecodedFrame frame;
    while (gateway.TryReceive(&frame) == protocol::UdpStatus::kOk)
      count += frame.header.type == protocol::FrameType::kHardwareConfig;
    return count;
  };
  ASSERT_EQ(ReceiveConfigs(gateway), 1u);
  ipc::Publisher<Packet> pub{RequestTopic("drivetrain")};
  for (uint64_t delay :
       {100000u, 200000u, 400000u, 800000u, 1600000u, 2000000u, 2000000u}) {
    ConfigAckPayload ack{};
    ack.config_id = id;
    ack.status = ConfigAckStatus::kBackendFailed;
    std::array<uint8_t, 128> bytes{};
    const auto size = EncodeConfigAck(ack, bytes);
    ASSERT_EQ(gateway.SendFrame(protocol::FrameType::kHardwareConfigAck, 0, now,
                                bytes.data(), size),
              protocol::UdpStatus::kOk);
    // The following state is an in-order receive barrier for the ACK: unlike
    // sleeping, InjectState proves Tick consumed the preceding datagram.
    ASSERT_TRUE(
        InjectState(node, gateway, TrustState(id, 11, 1, now + 1), now));
    EXPECT_EQ(drain_configs(), 0u);
    const auto before = node.commands_sent();
    ASSERT_TRUE(InjectState(node, gateway,
                            TrustState(id, 11, 1, now + delay - 1),
                            now + delay - 1));
    EXPECT_EQ(drain_configs(), 0u);
    Command request{};
    request.config_id = id;
    request.boot_id = 11;
    request.epoch = 1;
    request.observed_time_us = now + delay - 1;
    request.count = 1;
    request.motors[0] = {cfg.subsystems.at("drivetrain").motors[0],
                         Mode::kDutyCycle, 0, 0.2};
    PublishCommand(pub, request);
    node.Tick(now + delay - 1);
    EXPECT_GT(node.commands_sent(), before);
    EXPECT_EQ(node.current_command().motors[0].mode, Mode::kDutyCycle);
    EXPECT_EQ(drain_configs(), 0u);
    now += delay;
    ASSERT_TRUE(InjectState(node, gateway, TrustState(id, 11, 1, now), now));
    ASSERT_EQ(ReceiveConfigs(gateway), 1u);
  }
}

}  // namespace
}  // namespace talos::hardware

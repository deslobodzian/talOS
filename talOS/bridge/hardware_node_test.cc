// HardwareNode against a fake gateway on loopback UDP, ticked on synthetic
// time: the integration tier (docs/TESTING.md §2). The request trust rules
// are tested without a socket in trust_test.cc, through the frame handler
// this file's datagrams reach (HardwareNode::HandleGatewayFrame); here a
// State, a Driver Station packet and an acknowledgement in, and a probe, a
// configuration and a command out, cross the socket on their way to and
// from it.

#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "node.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/bridge/merge.h"
#include "talOS/bridge/node_test_support.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/context.h"
#include "talOS/events/input_replay.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/manifest.h"
#include "talOS/events/os/poller.h"
#include "talOS/events/time.h"
#include "talOS/introspection/registry.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/protocol/udp_test_helpers.h"

namespace talos::hardware {
namespace {
using namespace bridge_test;  // NOLINT(google-build-using-namespace)

TEST(HardwareNodeTest, RejectsMalformedDriverStationPackets) {
  auto cfg = talos::config::ParseRobotConfigString(kTestToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway_peer = link.gateway;

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

// Commissioning is forced by --sim alone, the flag every node also reads
// (HardwareLink). A loopback bridge without --sim keeps the config's own id,
// as nodes launched without --sim do, or they would drop every State.
TEST(HardwareNodeTest, OnlySimulationForcesCommissioning) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  EXPECT_THROW((HardwareNode{cfg, "10.56.87.2", 5802, 5803, true}),
               std::invalid_argument);
  const uint64_t own_id = ConfigurationId(cfg.hardware);
  HardwareNode real{cfg, "10.56.87.2"};
  EXPECT_EQ(real.current_command().config_id, own_id);
  HardwareNode loopback{cfg};
  EXPECT_EQ(loopback.current_command().config_id, own_id);
  HardwareNode simulation{cfg, "127.0.0.1", 5802, 5803, true};
  EXPECT_EQ(simulation.current_command().config_id, NodeConfigId(cfg));
  EXPECT_NE(NodeConfigId(cfg), own_id);
}

// The bridge reads each /hw/request/<node> latest-value, so a request
// published while an earlier one waits unread is passed over: when that
// node's wakes to the bridge fail, the bridge reads it only on passes
// something else starts, and its subsystem merges at a fraction of its rate.
// Each such skip counts in that subsystem's request row of the bridge's
// registry row, which the launcher and Studio read (TODO.md RT-7).
// REQ-RT-20, REQ-OPS-18
TEST(HardwareNodeTest, ARequestPassedOverBetweenTwoReadsIsCounted) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg, /*open_gateway=*/false));
  HardwareNode& node = link.node.value();
  const auto reader = introspect::RegistryReader::open();
  ASSERT_TRUE(reader.has_value());
  // This process's bridge row.
  const auto row = [&reader] {
    std::optional<introspect::NodeSnapshot> bridge;
    for (auto& node : reader->nodes()) {
      if (node.name == kNodeName &&
          node.pid == static_cast<std::uint64_t>(getpid()) &&
          (!bridge || node.generation > bridge->generation))
        bridge = std::move(node);
    }
    return bridge;
  };
  const auto skipped = [](const introspect::NodeSnapshot& bridge,
                          std::string_view topic) {
    std::optional<std::uint64_t> dropped;
    for (const auto& source : bridge.sources) {
      if (source.name == topic) dropped = source.dropped;
    }
    return dropped;
  };
  ipc::Publisher<Packet> intake{RequestTopic("intake")};
  ipc::Publisher<Packet> drivetrain{RequestTopic("drivetrain")};
  Command request{};
  request.config_id = NodeConfigId(cfg);

  // One request per read skips nothing; two between reads skip the first.
  PublishCommand(intake, request);
  node.Tick(1'000'000);
  PublishCommand(intake, request);
  PublishCommand(intake, request);
  PublishCommand(drivetrain, request);
  node.Tick(1'001'000);
  node.PublishIntrospection();

  const auto bridge = row();
  ASSERT_TRUE(bridge.has_value());
  EXPECT_EQ(skipped(*bridge, RequestTopic("intake")), 1u);
  EXPECT_EQ(skipped(*bridge, RequestTopic("drivetrain")), 0u)
      << "per subsystem";
  EXPECT_EQ(bridge->delivery_loss().requests_skipped, 1u);
}

TEST(HardwareNodeTest, ReportsLatestGatewayHealthAndConfiguredMotorFaults) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
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
  EXPECT_EQ(status.motor_forward_limit_mask(), 0u);
  EXPECT_EQ(status.motor_reverse_limit_mask(), 0u);
  // Soft and hard limits asserted (FaultField bits 14-17) are normal
  // operation, such as a homed mechanism resting on its limit switch, but
  // the hard-limit bits are reported per direction: a normally closed switch
  // with a cut wire reads asserted forever and is a fault in nothing else.
  state.motors[0].fault_flags = 0x3C000;
  state.sample_time_us++;
  ASSERT_TRUE(InjectState(node, gateway, state, 1002000));
  EXPECT_FALSE(node.Status(1002000).health_valid());
  EXPECT_EQ(node.Status(1002000).motor_fault_mask(), 0u);
  EXPECT_EQ(node.Status(1002000).motor_forward_limit_mask(), 2u);
  EXPECT_EQ(node.Status(1002000).motor_reverse_limit_mask(), 2u);
  // REQ-OPS-20: the supply or stator current limiter engaging (FaultField
  // bits 8 and 9), as it does for a few milliseconds on any hard acceleration,
  // is the limit working as configured, not a fault; with a real fault bit
  // beside it the motor is faulted.
  state.motors[0].fault_flags = (1u << 8) | (1u << 9);
  state.sample_time_us++;
  ASSERT_TRUE(InjectState(node, gateway, state, 1002100));
  EXPECT_EQ(node.Status(1002100).motor_fault_mask(), 0u);
  state.motors[0].fault_flags = (1u << 9) | 1u;  // StatorCurrLimit, Hardware
  state.sample_time_us++;
  ASSERT_TRUE(InjectState(node, gateway, state, 1002200));
  EXPECT_EQ(node.Status(1002200).motor_fault_mask(), 2u);
  state.motors[0].fault_flags = 0x3C000;
  // An invalid sample needs attention even without a controller fault code,
  // and its limit bits say nothing. IDs are deliberately in reverse order:
  // mask bits follow configuration.
  state.motors[1].valid = false;
  state.motors[1].fault_flags = kMotorForwardHardLimitFlag;
  state.sample_time_us++;
  ASSERT_TRUE(InjectState(node, gateway, state, 1003000));
  EXPECT_EQ(node.Status(1003000).motor_fault_mask(), 1u);
  EXPECT_EQ(node.Status(1003000).motor_forward_limit_mask(), 2u);
  state.motors[1].valid = true;
  state.sample_time_us++;
  ASSERT_TRUE(InjectState(node, gateway, state, 1004000));
  EXPECT_EQ(node.Status(1004000).motor_fault_mask(), 0u);
  EXPECT_EQ(node.Status(1004000).motor_forward_limit_mask(), 3u);
  EXPECT_EQ(node.Status(1004000).motor_reverse_limit_mask(), 2u);
}

// /hw/status names the build of the gateway boot the bridge accepted, and a
// new boot's build replaces it: after a RoboRIO rollback the record shows the
// build that drove the robot, not the first one the bridge heard.
TEST(HardwareNodeTest, ReportsTheBuildOfTheAcceptedGatewayBoot) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  EXPECT_EQ(node.Status(999000).gateway_program(),
            bridge::GatewayProgram::UNKNOWN);
  EXPECT_EQ(node.Status(999000).gateway_build_id(), 0u);

  State state = TrustState(NodeConfigId(cfg), 42, 1, 1000);
  state.build = {0x0123456789abcdefull, GatewayProgram::kRoboRio, true};
  ASSERT_TRUE(InjectState(node, link.gateway, state, 1000000));
  auto status = node.Status(1000000);
  EXPECT_EQ(status.gateway_program(), bridge::GatewayProgram::ROBORIO);
  EXPECT_TRUE(status.gateway_build_dirty());
  EXPECT_EQ(status.gateway_build_id(), 0x0123456789abcdefull);

  State rollback = TrustState(NodeConfigId(cfg), 43, 1, 50);
  rollback.build = {0x00fedcba98765432ull, GatewayProgram::kRoboRio, false};
  ASSERT_TRUE(InjectState(node, link.gateway, rollback, 1001000));
  status = node.Status(1001000);
  EXPECT_EQ(status.gateway_program(), bridge::GatewayProgram::ROBORIO);
  EXPECT_FALSE(status.gateway_build_dirty());
  EXPECT_EQ(status.gateway_build_id(), 0x00fedcba98765432ull);
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
  uint16_t gateway_port = 0;
  uint16_t node_port = 0;
  int gateway_reservation = ReservePort(gateway_port);
  int node_reservation = ReservePort(node_port);
  const bool reserved = gateway_reservation >= 0 && node_reservation >= 0;
  Release(gateway_reservation);
  if (!reserved) Release(node_reservation);
  ASSERT_TRUE(reserved);
  const std::string gateway_port_arg = std::to_string(gateway_port);
  const std::string node_port_arg = std::to_string(node_port);
  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", gateway_port, "127.0.0.1", node_port),
            protocol::UdpStatus::kOk);
  ipc::Publisher<Packet> pub{RequestTopic("intake")};
  Release(node_reservation);
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
  // Send until the child has answered or exited: its one second starts only
  // after its startup, which a loaded host can stretch past any fixed window.
  int status = 0;
  bool reaped = false;
  const auto give_up =
      std::chrono::steady_clock::now() + std::chrono::seconds{20};
  while (!reaped && std::chrono::steady_clock::now() < give_up) {
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
    reaped = waitpid(child, &status, WNOHANG) == child;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!reaped) {
    ASSERT_EQ(waitpid(child, &status, 0), child);
  }
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

// REQ-OPS-14: replaying a session into a node that did not watch /hw/state
// feeds it every State the bridge published once, with the transport
// sequence and publish stamp a live reader saw, including after a State the
// bridge received and refused (a duplicate). The session holds the bridge's
// --log and the log of one live reader, written as a node's loop writes it.
TEST(HardwareNodeTest, ReplayFeedsEachPublishedStateOnceAsReadersSawIt) {
  constexpr uint64_t kSession = 41;
  const std::filesystem::path session = ::testing::TempDir() +
                                        "/bridge_replay_session_" +
                                        std::to_string(::getpid());
  std::filesystem::remove_all(session);
  std::filesystem::create_directories(session);
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg, true, (session / "hardware_node.tlog").string(),
                        kSession));
  HardwareNode& node = link.node.value();

  RTMSQueue reader{
      kStateTopic,
      sizeof(Packet),
      alignof(Packet),
      schema::Of<Packet>(),
      MAX_SLOTS,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::SEQUENCE}};
  const std::optional<std::size_t> reader_id = reader.register_reader();
  ASSERT_TRUE(reader_id);
  constexpr std::uint16_t kWatcher = 0;
  auto reader_log = std::make_unique<event::log::LogWriter>(
      (session / "state_reader.tlog").string(), "state_reader",
      event::log::LogWriter::Options{}, kSession);
  reader_log->start(event::Manifest{event::Registration{
                        .id = kWatcher,
                        .kind = event::SourceKind::WATCHER,
                        .name = kStateTopic,
                        .message_bytes = sizeof(Packet),
                        .alignment = alignof(Packet),
                        .schema = schema::Of<Packet>(),
                    }},
                    event::Poller::now());

  // What the live reader saw: (sequence, publish time), in order.
  std::vector<std::pair<uint64_t, int64_t>> seen;
  const auto read_all = [&] {
    Packet packet{};
    MessageInfo info{};
    while (reader.read_next(*reader_id,
                            std::as_writable_bytes(std::span{&packet, 1}),
                            info) == ReadResult::OK) {
      seen.emplace_back(info.sequence, info.publish_time_ns);
      const event::MonotonicTime now = event::Poller::now();
      reader_log->dispatch(
          event::Context{
              .kind = event::EventKind::MESSAGE,
              .source_id = kWatcher,
              .event_time = now,
              .now = now,
              .publish_time =
                  event::MonotonicTime::from_nanos(info.publish_time_ns),
              .sequence = info.sequence,
          },
          std::as_bytes(std::span{&packet, 1}));
    }
  };

  const uint64_t config_id = NodeConfigId(cfg);
  // The third State repeats the second's sample time: received, refused.
  for (const auto& [sample_us, now_us] :
       std::vector<std::pair<uint64_t, uint64_t>>{
           {1000, 1000}, {2000, 2000}, {2000, 3000}, {4000, 4000}}) {
    ASSERT_TRUE(InjectState(node, link.gateway,
                            TrustState(config_id, 11, 1, sample_us), now_us));
    read_all();
  }
  ASSERT_EQ(node.states_received(), 4u);
  ASSERT_EQ(seen.size(), 3u);
  link.node.reset();  // flushes the bridge's log
  reader_log.reset();

  const event::RecordedSession replay{session.string(), "replay_consumer"};
  std::vector<std::pair<uint64_t, int64_t>> fed;
  for (const event::RecordedMessage& message : replay.messages()) {
    if (replay.topics()[message.topic].name == kStateTopic) {
      fed.emplace_back(message.sequence, message.time.nanos());
    }
  }
  EXPECT_EQ(fed, seen);
}

// /hw/status carries the bridge's counters, the age of the last controller
// state on the bridge clock, which actuator subsystems have gone quiet and
// whether the RoboRIO accepted the configuration.
TEST(HardwareNodeTest, PublishesBridgeStatus) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  Link link;
  ASSERT_TRUE(link.Open(cfg, /*open_gateway=*/false));
  HardwareNode& node = link.node.value();
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

  ASSERT_TRUE(link.OpenGateway());
  protocol::RuntimeUdpPeer& gateway = link.gateway;
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
  // The drivetrain, quiet for 18 ms, is still inside its owner timeout.
  EXPECT_EQ(later.timed_out_mask(), 0b10u);
  EXPECT_EQ(later.state_age_us(), 19000u);
  EXPECT_TRUE(later.configured());
}

// REQ-OPS-20: /hw/status.configured is the current gateway boot's
// acknowledgement of this bridge's configuration. A RoboRIO program restart (a
// new boot id) voids the last boot's acknowledgement, even when the new boot's
// States name our config id (restored from its saved copy, or named after a
// failed apply); so does a State naming another config id within one boot.
// Either holds until the next acknowledgement. Measured on a RoboRIO: a new
// boot's first 19 States (94 ms) carried config id 0.
TEST(HardwareNodeTest, ConfiguredDropsOnGatewayRebootUntilAcknowledged) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
  const uint64_t config_id = NodeConfigId(cfg);
  ConfigAckPayload ack{};
  ack.config_id = config_id;
  ack.status = ConfigAckStatus::kOk;
  std::array<uint8_t, 128> ack_bytes{};
  const auto ack_size = EncodeConfigAck(ack, ack_bytes);
  // Only Tick reads the socket, so the ack crosses real loopback; WaitFor's
  // 10 s deadline bounds a hang only (docs/TESTING.md §6).
  const auto acknowledge = [&](uint64_t now_us) {
    return gateway.SendFrame(protocol::FrameType::kHardwareConfigAck, 0, now_us,
                             ack_bytes.data(),
                             ack_size) == protocol::UdpStatus::kOk &&
           WaitFor([&] {
             node.Tick(now_us);
             return node.is_configured();
           });
  };

  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 1000), 1000000));
  ASSERT_TRUE(acknowledge(1000500));
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 2000), 1001000));
  ASSERT_TRUE(node.Status(1001000).configured());

  // The RoboRIO program restarts; boot 12 names our config before it acks.
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 12, 1, 500), 1050000));
  EXPECT_FALSE(node.Status(1050000).configured());
  ASSERT_TRUE(acknowledge(1050500));
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 12, 1, 1500), 1051000));
  EXPECT_TRUE(node.Status(1051000).configured());

  // Within boot 12 the gateway takes another configuration, then ours again.
  ASSERT_TRUE(InjectState(node, gateway, TrustState(config_id + 1, 12, 2, 2500),
                          1052000));
  EXPECT_FALSE(node.Status(1052000).configured());
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 12, 3, 3500), 1053000));
  EXPECT_FALSE(node.Status(1053000).configured());
  ASSERT_TRUE(acknowledge(1053500));
  EXPECT_TRUE(node.Status(1053500).configured());
}

// REQ-RT-20
TEST(HardwareNodeTest, TwoWayClockBoundNeedsNoActiveRequests) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
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
  // The echo is completed by the next Tick, the first whose clock reading
  // follows receipt; Run() makes it at once rather than at a later duty.
  EXPECT_EQ(node.NextDutyUs(2005000), 2005000u);
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

// One lost probe must not open a gap: the bridge replaces it long before the
// exchange it already holds (one second lifetime) expires, so the mapping
// stays valid through the whole second after the last good exchange.
TEST(HardwareNodeTest, OneLostClockProbeLeavesTheMappingValid) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
  // The next clock probe the bridge sent, skipping config pushes.
  const auto next_probe = [&gateway]() -> std::optional<uint64_t> {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    protocol::DecodedFrame frame;
    while (protocol::testing::ReceiveFrame(gateway, &frame, deadline) ==
           protocol::UdpStatus::kOk) {
      if (frame.header.type == protocol::FrameType::kClockProbe) {
        return frame.header.monotonic_time_us;
      }
    }
    return std::nullopt;
  };
  // The gateway clock runs 2'004'000 us behind the bridge's.
  constexpr uint64_t kBehindUs = 2'004'000;
  auto state = TrustState(NodeConfigId(cfg), 11, 1, 0);
  state.flags = 0;
  const auto inject = [&](uint64_t bridge_us, uint64_t echo) {
    state.sample_time_us = bridge_us - kBehindUs;
    state.clock_probe_time_us = echo;
    return InjectState(node, gateway, state, bridge_us);
  };

  // One answered exchange (completed on the 2'006'000 tick) and eight States.
  node.Tick(2'000'000);
  const auto first = next_probe();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(inject(2'005'000, *first));
  for (uint64_t t = 2'006'000; t <= 2'012'000; t += 1000) {
    ASSERT_TRUE(inject(t, 0));
  }
  ASSERT_TRUE(node.Status(2'012'000).clock_offset_valid());

  // The next probe is lost.
  node.Tick(2'020'000);
  ASSERT_TRUE(next_probe().has_value());

  // A gateway that answers every later probe in its next State. The first
  // exchange expires 1 s after 2'006'000; validity must not lapse.
  uint64_t echo = 0;
  for (uint64_t t = 2'030'000; t <= 3'040'000; t += 10'000) {
    ASSERT_TRUE(inject(t, echo));
    echo = 0;
    EXPECT_TRUE(node.Status(t).clock_offset_valid()) << "at bridge " << t;
    protocol::DecodedFrame frame;
    while (gateway.TryReceive(&frame) == protocol::UdpStatus::kOk) {
      if (frame.header.type == protocol::FrameType::kClockProbe) {
        echo = frame.header.monotonic_time_us;
      }
    }
  }
}

TEST(HardwareNodeTest, FailedConfigPushBacksOffWithoutStoppingCommands) {
  auto cfg = config::ParseRobotConfigString(kTestToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
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

// Run() sleeps between inputs, so every time-driven duty must wake it at the
// instant Tick would do it. NextDutyUs() is that instant: ticking just
// before it does nothing, and ticking only at it sends each command, probe
// and /hw/status exactly when Tick's own conditions fall due. With a
// gateway that sends one State and then nothing: the request's command,
// heartbeats every heartbeat_us(), the neutral command one microsecond past
// the intake's 50 ms owner timeout, and nothing after it. The request lands
// 7 ms after the first status, and the gateway answers the probe retried
// 1 us after a status, so no duty shares another's instant: the answered
// probe's successor is due 20 ms after it, not with the next status.
// REQ-SAFE-15
// REQ-RT-20
TEST(HardwareNodeTest, NextDutyUsIsWhenTickNextHasWork) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.subsystems.at("intake").motors[0];
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
  ipc::Subscriber<bridge::BridgeStatus> statuses{
      bridge::topics::kStatus,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};
  constexpr uint64_t kStartUs = 1'000'000;
  ASSERT_TRUE(InjectState(
      node, gateway, TrustState(NodeConfigId(cfg), 11, 1, 1000), kStartUs));
  ipc::Publisher<Packet> pub{RequestTopic("intake")};
  constexpr uint64_t kRequestUs = kStartUs + 7000;
  PublishCommand(pub, IntakeRequest(cfg, 4.0));
  node.Tick(kRequestUs);
  ASSERT_EQ(node.commands_sent(), 1u);
  while (statuses.read()) {
  }

  std::vector<uint64_t> status_us;
  uint64_t now_us = kRequestUs;
  bool answered = false;
  while (now_us < kStartUs + 300'000) {
    if (!answered && now_us == kStartUs + 100'001) {
      // The State answering the retried probe, then the pass right after
      // it, which completes the exchange.
      answered = true;
      State echo = TrustState(NodeConfigId(cfg), 11, 1, 2000);
      echo.clock_probe_time_us = now_us;
      now_us = kStartUs + 105'000;
      ASSERT_TRUE(InjectState(node, gateway, echo, now_us));
      ASSERT_EQ(node.NextDutyUs(now_us), now_us);
      node.Tick(now_us);
    }
    const uint64_t due_us = node.NextDutyUs(now_us);
    ASSERT_GT(due_us, now_us) << "nothing is pending at " << now_us;
    if (due_us - 1 > now_us) {
      const uint64_t commands = node.commands_sent();
      node.Tick(due_us - 1);
      EXPECT_EQ(node.commands_sent(), commands) << "early, at " << due_us - 1;
      EXPECT_FALSE(statuses.read()) << "early, at " << due_us - 1;
    }
    now_us = due_us;
    node.Tick(now_us);
    if (statuses.read()) status_us.push_back(now_us);
  }

  // Every frame went out at a duty instant: the request's command, the two
  // heartbeats, the timeout's neutral command, and the clock probe with its
  // retry 100 ms and 1 us after it, the next 20 ms after the answered
  // retry, and that one's retry.
  std::vector<SentCommand> commands;
  std::vector<uint64_t> probe_us;
  protocol::DecodedFrame frame;
  while (protocol::testing::ReceiveFrame(gateway, &frame) ==
         protocol::UdpStatus::kOk) {
    if (frame.header.type == protocol::FrameType::kClockProbe) {
      probe_us.push_back(frame.header.monotonic_time_us);
    } else if (frame.header.type == protocol::FrameType::kHardwareCommand) {
      SentCommand sent{frame.header.monotonic_time_us, {}};
      ASSERT_TRUE(
          Decode(std::span<const uint8_t>{frame.payload, frame.payload_size},
                 sent.command));
      commands.push_back(sent);
    }
  }
  const uint64_t heartbeat_us = node.heartbeat_us();
  ASSERT_EQ(heartbeat_us, 20'000u);
  const uint64_t timeout_us =
      OwnerTimeoutUs(5000, cfg.hardware.command_timeout_us);
  ASSERT_EQ(commands.size(), 4u);
  const std::array<uint64_t, 4> expected_us = {
      kRequestUs, kRequestUs + heartbeat_us, kRequestUs + (2 * heartbeat_us),
      kRequestUs + timeout_us + 1};
  for (std::size_t i = 0; i < commands.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(commands[i].sent_us, expected_us[i]);
    EXPECT_EQ(MotorOf(commands[i].command, roller).mode,
              i + 1 < commands.size() ? Mode::kVoltage : Mode::kNeutral);
  }
  EXPECT_EQ(probe_us,
            (std::vector<uint64_t>{kStartUs, kStartUs + 100'001,
                                   kStartUs + 120'001, kStartUs + 220'002}));
  ASSERT_EQ(status_us.size(), 15u);
  for (std::size_t i = 0; i < status_us.size(); ++i) {
    EXPECT_EQ(status_us[i], kStartUs + ((i + 1) * 20'000)) << i;
  }
}

// A burst beyond one Tick's receive budget is drained pass after pass
// without sleeping in between, until the window's datagram budget is spent:
// then the loop sleeps through input to the end of the window, and drains
// the rest after it. The old 1 ms poll took 32 datagrams a millisecond;
// this takes at most kDatagramBudget plus the one pass that crosses it.
// REQ-RT-20
TEST(HardwareNodeTest, PassDrainsABacklogWithinItsCpuBudget) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
  constexpr uint64_t kStartUs = 1'000'000;
  ASSERT_TRUE(InjectState(
      node, gateway, TrustState(NodeConfigId(cfg), 11, 1, 1000), kStartUs));
  // Many more than one pass takes, so the first pass after the sends finds
  // a full receive budget queued even where loopback delivery lags the send.
  constexpr uint64_t kBurst = 100;
  driver_station::DriverStationData data{};
  data.boot_id = 11;
  std::array<uint8_t, 256> bytes{};
  for (uint64_t i = 1; i <= kBurst; ++i) {
    data.sample_time_us = 1000 + i;
    const std::size_t size = driver_station::Encode(data, bytes);
    ASSERT_EQ(gateway.SendFrame(protocol::FrameType::kDriverStation, 0,
                                kStartUs, bytes.data(), size),
              protocol::UdpStatus::kOk);
  }
  constexpr auto kReceiveBudget =
      static_cast<uint64_t>(HardwareNode::kReceiveBudget);
  uint64_t now_us = kStartUs + 1;
  uint64_t in_window = 0;
  bool saw_full_receive = false;
  bool saw_spent = false;
  const bool drained = WaitFor([&] {
    const uint64_t before = node.driver_station_packets_received();
    const HardwareNode::Wait wait = node.Pass(now_us);
    const uint64_t received = node.driver_station_packets_received() - before;
    in_window += received;
    EXPECT_LE(in_window, HardwareNode::kDatagramBudget + kReceiveBudget)
        << "at " << now_us;
    if (wait.ignore_input) {
      saw_spent = true;
      EXPECT_GT(wait.until_us, now_us);
      EXPECT_LE(wait.until_us, now_us + HardwareNode::kCpuWindowUs);
      now_us = wait.until_us;
      in_window = 0;
    } else if (received == kReceiveBudget) {
      saw_full_receive = true;
      EXPECT_EQ(wait.until_us, now_us) << "slept with datagrams queued";
    }
    return node.driver_station_packets_received() == kBurst;
  });
  ASSERT_TRUE(drained) << node.driver_station_packets_received() << " of "
                       << kBurst;
  EXPECT_TRUE(saw_full_receive);
  EXPECT_TRUE(saw_spent);
  EXPECT_GE(node.wakeups().throttled, 1u);
}

// A node publishing requests without pause (rejected ones here, on the
// drivetrain's topic) cannot hold the bridge, the highest-priority loop,
// runnable: each window allows kPassBudget passes before the loop sleeps
// through input to the window's end. Each pass costs kPassCostUs of
// simulated time, and every wait the flood would cut short at once. Duties
// are not delayed: the intake's heartbeats, its owner timeout's neutral
// command and each /hw/status go out at their instants, at most one pass
// late. The windows start mid-way between statuses, so a status falls in a
// window's sleep and must end it.
// REQ-SAFE-15
// REQ-RT-20
TEST(HardwareNodeTest, AFloodOfRequestsIsHeldToThePassBudget) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const uint16_t roller = cfg.subsystems.at("intake").motors[0];
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
  constexpr uint64_t kStartUs = 1'000'000;
  ASSERT_TRUE(InjectState(
      node, gateway, TrustState(NodeConfigId(cfg), 11, 1, 1000), kStartUs));
  ipc::Publisher<Packet> intake{RequestTopic("intake")};
  ipc::Publisher<Packet> drivetrain{RequestTopic("drivetrain")};
  ipc::Subscriber<bridge::BridgeStatus> statuses{
      bridge::topics::kStatus,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};
  while (statuses.read()) {
  }
  Command rejected{};
  rejected.config_id = NodeConfigId(cfg) + 1;

  constexpr uint64_t kRequestUs = kStartUs + 7500;
  constexpr uint64_t kFloodUs = 100'000;
  constexpr uint64_t kPassCostUs = 10;
  PublishCommand(intake, IntakeRequest(cfg, 4.0));
  uint64_t now_us = kRequestUs;
  const uint64_t passes_before = node.wakeups().passes;
  const uint64_t rejected_before = node.identity_rejected();
  std::vector<uint64_t> status_us;
  while (now_us < kRequestUs + kFloodUs) {
    PublishCommand(drivetrain, rejected);
    const HardwareNode::Wait wait = node.Pass(now_us);
    if (statuses.read()) status_us.push_back(now_us);
    now_us = wait.ignore_input ? std::max(wait.until_us, now_us + kPassCostUs)
                               : now_us + kPassCostUs;
  }
  const uint64_t passes = node.wakeups().passes - passes_before;
  EXPECT_EQ(node.identity_rejected() - rejected_before, passes)
      << "a pass that did not read the flood";
  // kPassBudget a window, plus one pass per duty that ends a sleep early
  // (100 ms: 5 statuses, 3 heartbeats or the neutral, a probe retry; the
  // registry's liveness rides the passes); unbudgeted, the flood makes
  // kFloodUs / kPassCostUs.
  const uint64_t windows = kFloodUs / HardwareNode::kCpuWindowUs + 1;
  EXPECT_LE(passes, (windows * HardwareNode::kPassBudget) + 10)
      << passes << " passes, " << node.wakeups().throttled << " throttled";

  std::vector<SentCommand> commands;
  while (const auto sent = NextCommand(gateway)) {
    commands.push_back(*sent);
    if (MotorOf(sent->command, roller).mode == Mode::kNeutral) break;
  }
  const uint64_t heartbeat_us = node.heartbeat_us();
  const uint64_t timeout_us =
      OwnerTimeoutUs(5000, cfg.hardware.command_timeout_us);
  const std::array<uint64_t, 4> due_us = {kRequestUs, kRequestUs + heartbeat_us,
                                          kRequestUs + (2 * heartbeat_us),
                                          kRequestUs + timeout_us + 1};
  ASSERT_EQ(commands.size(), due_us.size());
  for (std::size_t i = 0; i < commands.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_GE(commands[i].sent_us, due_us[i]);
    EXPECT_LE(commands[i].sent_us, due_us[i] + kPassCostUs);
  }
  ASSERT_EQ(status_us.size(), 5u);
  for (std::size_t i = 0; i < status_us.size(); ++i) {
    const uint64_t status_due_us = kStartUs + ((i + 1) * 20'000);
    EXPECT_GE(status_us[i], status_due_us) << i;
    EXPECT_LE(status_us[i], status_due_us + kPassCostUs) << i;
  }
}

WakeReceiver NoDescriptorsLeft() {
  throw std::system_error(EMFILE, std::generic_category(),
                          "talos wake receiver: socket");
}

// A bridge that cannot build its wake receiver (out of descriptors) still
// runs: it says so on stderr, counts it, and polls its inputs every kRetryUs
// where it would otherwise sleep until its next duty (here its longest
// wait, 10 ms on).
// REQ-RT-20
TEST(HardwareNodeTest, WithoutAWakeReceiverTheBridgePollsItsInputs) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  constexpr uint64_t kStartUs = 1'000'000;
  node.Pass(kStartUs);  // The first pass's duties: probe, status, registry.

  node.set_seams({.make_wake_receiver = &NoDescriptorsLeft});
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(node.OpenWaits());
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(node.wakeups().wait_fallbacks, 1u);
  EXPECT_NE(stderr_text.find(std::strerror(EMFILE)), std::string::npos)
      << stderr_text;
  EXPECT_EQ(node.Pass(kStartUs + 100).until_us, kStartUs + 1100);

  node.set_seams({});
  ASSERT_TRUE(node.OpenWaits());
  EXPECT_EQ(node.Pass(kStartUs + 200).until_us, kStartUs + 10'200);
  node.CloseWaits();
}

bool NeverCaughtUp(ipc::Subscriber<Packet>& /*subscriber*/) { return false; }

// An arm that keeps reporting unread data no read returns (a read the
// writer keeps tearing) holds the loop awake for kMaxIdlePasses passes, then
// the loop waits for its next duty anyway: it cannot spin.
// REQ-RT-20
TEST(HardwareNodeTest, AnArmThatNeverCatchesUpCannotSpinTheLoop) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  node.set_seams({.arm_request = &NeverCaughtUp});
  constexpr uint64_t kStartUs = 1'000'000;
  for (uint64_t i = 0; i < HardwareNode::kMaxIdlePasses; ++i) {
    EXPECT_EQ(node.Pass(kStartUs + i).until_us, kStartUs + i) << i;
  }
  const uint64_t now_us = kStartUs + HardwareNode::kMaxIdlePasses;
  const HardwareNode::Wait wait = node.Pass(now_us);
  EXPECT_FALSE(wait.ignore_input);
  EXPECT_EQ(wait.until_us, now_us + 10'000);  // the longest wait
}

// The send time of the next clock probe the gateway receives, skipping other
// frames; nullopt when none arrives within 200 ms.
std::optional<uint64_t> NextProbeUs(protocol::RuntimeUdpPeer& gateway) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  protocol::DecodedFrame frame;
  while (protocol::testing::ReceiveFrame(gateway, &frame, deadline) ==
         protocol::UdpStatus::kOk) {
    if (frame.header.type == protocol::FrameType::kClockProbe) {
      return frame.header.monotonic_time_us;
    }
  }
  return std::nullopt;
}

// A UDP socket bound to `port` on every address, so binding the port again
// fails; -1 when it cannot be bound.
int HoldPort(uint16_t port) {
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return -1;
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(port);
  if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
    close(fd);
    return -1;
  }
  return fd;
}

// A socket that can no longer send (shut down for writing: EPIPE, as from a
// macOS socket the kernel marked defunct around sleep and wake) is closed by
// the send that finds it and replaced on the same ports: the first attempt
// on the next Tick, any later one kReopenRetryUs after the last. Nothing is
// sent while it is closed, and the duty that could not be sent goes out on the
// new socket. On macOS without SO_NOSIGPIPE that first send kills the test
// with SIGPIPE instead.
// REQ-RT-22
TEST(HardwareNodeTest, ASocketThatCannotSendIsReplacedAndFramesFlowAgain) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
  constexpr uint64_t kStartUs = 1'000'000;
  node.Tick(kStartUs);
  ASSERT_EQ(NextProbeUs(gateway), kStartUs);

  // The unanswered probe's retry, 100001 us on, finds the socket broken.
  ASSERT_EQ(shutdown(node.udp_fd(), SHUT_WR), 0);
  const uint64_t lost_us = kStartUs + 100'001;
  node.Tick(lost_us);
  EXPECT_EQ(node.udp_fd(), -1);
  EXPECT_EQ(node.socket_counters().last_send_error, EPIPE);
  // The first reopen is the next Tick's, kRetryUs (1 ms) on at the latest.
  const uint64_t first_us = lost_us + 1'000;
  EXPECT_EQ(node.NextDutyUs(lost_us), first_us);

  // It fails: another socket holds the port meanwhile.
  int holder = HoldPort(link.node_port);
  ASSERT_GE(holder, 0);
  node.Tick(first_us);
  EXPECT_EQ(node.udp_fd(), -1);
  Release(holder);

  // The port is free again, but no attempt is made before the retry.
  const uint64_t retry_us = first_us + HardwareNode::kReopenRetryUs;
  node.Tick(retry_us - 1);
  EXPECT_EQ(node.udp_fd(), -1);
  EXPECT_EQ(node.socket_counters().reopens, 0u);

  node.Tick(retry_us);
  EXPECT_GE(node.udp_fd(), 0);
  EXPECT_EQ(node.socket_counters().reopens, 1u);
  EXPECT_EQ(NextProbeUs(gateway), retry_us);
}

int pending_send_error = 0;
int TakeSendError() { return std::exchange(pending_send_error, 0); }

// A send error the network clears by itself keeps the socket: the failed
// probe goes out on the same socket kRetryUs (1 ms) later, and nothing is
// reopened.
// REQ-RT-22
TEST(HardwareNodeTest, ATransientSendErrorKeepsTheSocket) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
  node.set_seams({.send_error = &TakeSendError});
  uint64_t now_us = 1'000'000;
  for (const int error :
       {ECONNREFUSED, ENETDOWN, ENETUNREACH, EHOSTUNREACH, ENOBUFS}) {
    SCOPED_TRACE(std::strerror(error));
    pending_send_error = error;
    node.Tick(now_us);  // the probe is due, and fails
    EXPECT_EQ(node.socket_counters().last_send_error, error);
    node.Tick(now_us + 1'000);
    EXPECT_EQ(node.socket_counters().reopens, 0u);
    EXPECT_EQ(NextProbeUs(gateway), now_us + 1'000);
    now_us += 200'000;  // past the next unanswered probe's retry
  }
}

// This process's bridge row's heartbeat in the live registry, which a
// liveness sample stamps; 0 while it has no row.
std::int64_t BridgeHeartbeat(const introspect::RegistryReader& reader) {
  std::int64_t heartbeat_ns = 0;
  std::uint64_t generation = 0;
  for (const auto& row : reader.nodes()) {
    if (row.name != kNodeName ||
        row.pid != static_cast<std::uint64_t>(getpid()) ||
        row.generation < generation)
      continue;
    generation = row.generation;
    heartbeat_ns = row.heartbeat_monotonic_ns;
  }
  return heartbeat_ns;
}

// The registry's liveness sample (heartbeat and dispatch count, which the
// launcher's stall check reads) rides every pass rather than waking the
// bridge on a timer of its own. Driven as Run() drives it, with a State
// every 5 ms (a few hundred microseconds of jitter) for 1 s: every pass
// stores the sample, and the bridge wakes at a deadline only for /hw/status
// (every 20 ms), the unanswered clock probe's retry (every 100001 us) and
// the full registry sample (every 250 ms), never between States for the
// liveness sample alone. Then 200 ms with no input at all: every pass
// still stores it, and none is more than kLivenessBackstopUs (50 ms) after
// the last.
// REQ-RT-18
// REQ-RT-20
TEST(HardwareNodeTest, TheLivenessSampleRidesEveryPass) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
  const auto reader = introspect::RegistryReader::open();
  ASSERT_TRUE(reader.has_value());
  constexpr uint64_t kStartUs = 1'000'000;
  ASSERT_TRUE(InjectState(
      node, gateway, TrustState(NodeConfigId(cfg), 11, 1, 1000), kStartUs));

  uint64_t now_us = kStartUs;
  std::int64_t heartbeat_ns = 0;
  const auto pass = [&](uint64_t at_us) {
    EXPECT_LE(at_us - now_us, 50'000u) << "no pass from " << now_us;
    now_us = at_us;
    const HardwareNode::Wait wait = node.Pass(now_us);
    EXPECT_FALSE(wait.ignore_input) << "at " << now_us;
    EXPECT_GT(wait.until_us, now_us) << "input waiting at " << now_us;
    const std::int64_t stored_ns = BridgeHeartbeat(*reader);
    EXPECT_NE(stored_ns, heartbeat_ns) << "no liveness sample at " << now_us;
    heartbeat_ns = stored_ns;
    return wait;
  };

  constexpr uint64_t kStates = 200;
  HardwareNode::Wait wait = pass(kStartUs);
  std::vector<uint64_t> deadline_us;
  uint64_t state_us = kStartUs;
  for (uint64_t k = 1; k <= kStates; ++k) {
    state_us = kStartUs + (5000 * k) - 2500 + ((k % 4) * 100);
    while (wait.until_us < state_us) {
      deadline_us.push_back(wait.until_us);
      wait = pass(wait.until_us);
    }
    const State state = TrustState(NodeConfigId(cfg), 11, 1, 1000 + (5000 * k));
    ASSERT_TRUE(InjectState(node, gateway, state, state_us));
    wait = pass(state_us);
  }
  // Every instant a timed duty other than the liveness sample falls due
  // before the last State. No State shares one: States land 2500-2800 us
  // into each 5 ms.
  std::set<uint64_t> duties_us;
  for (uint64_t t = kStartUs + 20'000; t < state_us; t += 20'000) {
    duties_us.insert(t);
  }
  for (uint64_t t = kStartUs + 100'001; t < state_us; t += 100'001) {
    duties_us.insert(t);
  }
  for (uint64_t t = kStartUs + 250'000; t < state_us; t += 250'000) {
    duties_us.insert(t);
  }
  EXPECT_EQ(deadline_us,
            std::vector<uint64_t>(duties_us.begin(), duties_us.end()));

  const uint64_t idle_end_us = state_us + 200'000;
  while (now_us < idle_end_us) wait = pass(wait.until_us);
}

// RULES.md 3: the bridge's loop allocates nothing in steady state: a
// configured link with a State every millisecond (answering each clock
// probe), a Driver Station packet every 20 ms, two owners' requests every
// 5 ms, the merged command, /hw/status, and each Pass() Run() makes: the
// registry samples, arming the request wakes and the next deadline. Tick()
// and Pass() run on this thread, which is the only one AllocationProbe
// counts.
TEST(HardwareNodeTest, SteadyStateTickDoesNotAllocate) {
  if (const auto why = talos::testing::AllocationProbe::Unsupported();
      !why.empty()) {
    GTEST_SKIP() << why;
  }
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  const uint16_t drive = cfg.hardware.motors[0].id;
  const uint16_t roller = cfg.hardware.motors[1].id;
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
  const uint64_t config_id = NodeConfigId(cfg);
  // The gateway clock runs 999'000 us behind the bridge's.
  constexpr uint64_t kBehindUs = 999'000;
  uint64_t now_us = 1'000'000;
  State state = TrustState(config_id, 11, 1, now_us - kBehindUs);
  ASSERT_TRUE(InjectState(node, gateway, state, now_us));
  ASSERT_GE(ReceiveConfigs(gateway), 1u);
  ConfigAckPayload ack{};
  ack.config_id = config_id;
  std::array<uint8_t, 128> ack_bytes{};
  const auto ack_size = EncodeConfigAck(ack, ack_bytes);
  ASSERT_EQ(gateway.SendFrame(protocol::FrameType::kHardwareConfigAck, 0,
                              now_us, ack_bytes.data(), ack_size),
            protocol::UdpStatus::kOk);

  ipc::Publisher<Packet> drive_pub{RequestTopic("drivetrain")};
  ipc::Publisher<Packet> intake_pub{RequestTopic("intake")};
  Command drive_request{};
  drive_request.config_id = config_id;
  drive_request.boot_id = 11;
  drive_request.epoch = 1;
  drive_request.count = 1;
  drive_request.motors[0] = {drive, Mode::kVelocity, 0, 15.0, 0.0};
  Command intake_request = drive_request;
  intake_request.motors[0] = {roller, Mode::kDutyCycle, 0, 0.4, 0.0};
  driver_station::DriverStationData driver_station{};
  driver_station.boot_id = 11;
  driver_station.fms.flags = driver_station::kDsAttached |
                             driver_station::kEnabled | driver_station::kTeleop;
  std::array<uint8_t, 256> driver_station_bytes{};

  uint64_t echo = 0;
  // One bridge millisecond; false if the State never arrived.
  const auto tick = [&](unsigned i) {
    now_us += 1000;
    protocol::DecodedFrame frame;
    while (gateway.TryReceive(&frame) == protocol::UdpStatus::kOk) {
      if (frame.header.type == protocol::FrameType::kClockProbe) {
        echo = frame.header.monotonic_time_us;
      }
    }
    state.sample_time_us = now_us - kBehindUs;
    state.clock_probe_time_us = echo;
    echo = 0;
    if (i % 20 == 0) {
      driver_station.sample_time_us = state.sample_time_us;
      driver_station.fms.match_time_s = static_cast<float>(i) / 1000.0f;
      const std::size_t size =
          driver_station::Encode(driver_station, driver_station_bytes);
      if (gateway.SendFrame(protocol::FrameType::kDriverStation, 0, now_us,
                            driver_station_bytes.data(),
                            size) != protocol::UdpStatus::kOk) {
        return false;
      }
    }
    if (!InjectState(node, gateway, state, now_us)) return false;
    if (i % 5 == 0) {
      drive_request.observed_time_us = state.sample_time_us;
      drive_request.motors[0].demand = 15.0 + i;
      intake_request.observed_time_us = state.sample_time_us;
      PublishCommand(drive_pub, drive_request);
      PublishCommand(intake_pub, intake_request);
    }
    node.Pass(now_us);
    return true;
  };
  for (unsigned i = 0; i < 300; ++i) ASSERT_TRUE(tick(i)) << i;
  ASSERT_TRUE(node.is_configured());
  ASSERT_TRUE(node.Status(now_us).clock_offset_valid());
  const uint64_t states = node.states_received();
  const uint64_t commands = node.commands_sent();
  const uint64_t driver_station_packets =
      node.driver_station_packets_received();

  talos::testing::AllocationProbe probe;
  probe.start();
  bool delivered = true;
  for (unsigned i = 300; i < 600 && delivered; ++i) delivered = tick(i);
  const uint64_t allocations = probe.stop();
  ASSERT_TRUE(delivered);
  EXPECT_EQ(allocations, 0u) << probe.sites();

  EXPECT_EQ(node.states_received(), states + 300);
  EXPECT_GT(node.commands_sent(), commands + 50);
  EXPECT_GT(node.driver_station_packets_received(), driver_station_packets);
  EXPECT_FALSE(node.is_subsystem_timed_out("drivetrain"));
  EXPECT_FALSE(node.is_subsystem_timed_out("intake"));
  EXPECT_EQ(node.identity_rejected(), 0u);
  EXPECT_DOUBLE_EQ(node.current_command().motors[0].demand, 15.0 + 595);
}

}  // namespace
}  // namespace talos::hardware

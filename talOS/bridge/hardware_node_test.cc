// HardwareNode against a fake gateway on loopback UDP, ticked on synthetic
// time: the integration tier (docs/TESTING.md §2). The request trust rules
// are tested without a socket in trust_test.cc, through the frame handler
// this file's datagrams reach (HardwareNode::HandleGatewayFrame); here a
// State, a Driver Station packet and an acknowledgement in, and a probe, a
// configuration and a command out, cross the socket on their way to and
// from it.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
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
#include "talOS/bridge/node_test_support.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/context.h"
#include "talOS/events/input_replay.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/manifest.h"
#include "talOS/events/os/poller.h"
#include "talOS/events/time.h"
#include "talOS/hardware/endpoint.h"
#include "talOS/hardware/gateway.h"
#include "talOS/hardware/merge.h"
#include "talOS/introspection/registry.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/process/process.h"
#include "talOS/protocol/udp_test_helpers.h"

namespace talos::hardware {
namespace {
using namespace bridge_test;  // NOLINT(google-build-using-namespace)

// Waits until the bridge's socket is readable, so the next Tick reads what is
// there: a loopback send can complete before the datagram is queued (notably
// on BSD). Readable is not "a datagram is queued": after the bridge sent to a
// port nobody had bound, the connected socket is readable for the pending
// error alone (macOS: POLLIN with FIONREAD 0), and the Tick that reads it
// consumes only the error. A caller waiting for a datagram's effect therefore
// ticks until it sees that effect, or checks QueuedBytes. The 10 s deadline
// only bounds a hang (TESTING.md 6.3).
bool DatagramQueued(const HardwareNode& node) {
  pollfd readable{node.udp_fd(), POLLIN, 0};
  return ::poll(&readable, 1, 10'000) == 1 && (readable.revents & POLLIN) != 0;
}

// Bytes of datagrams queued on the bridge's socket (FIONREAD), 0 when none.
int QueuedBytes(const HardwareNode& node) {
  int bytes = 0;
  return ::ioctl(node.udp_fd(), FIONREAD, &bytes) == 0 ? bytes : -1;
}

TEST(HardwareNodeTest, RejectsMalformedDriverStationPackets) {
  auto cfg = talos::config::ParseRobotConfigString(kTestToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway_peer = link.gateway;

  ipc::Subscriber<talos::hardware::Packet> ds_sub{kDriverStationTopic};

  ASSERT_TRUE(InjectState(node, gateway_peer,
                          TrustState(NodeConfigId(cfg), 42, 1, 1000), 1000));

  driver_station::DriverStationData data{};
  data.boot_id = 42;
  data.sample_time_us = 1000;
  std::vector<uint8_t> dummy_payload(1024);
  dummy_payload.resize(driver_station::Encode(data, dummy_payload));
  ASSERT_FALSE(dummy_payload.empty());
  ASSERT_EQ(gateway_peer.SendFrame(protocol::FrameType::kDriverStation, 1, 1000,
                                   dummy_payload.data(), dummy_payload.size()),
            protocol::UdpStatus::kOk);

  ASSERT_TRUE(DatagramQueued(node));
  node.Tick(2000);

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
    // Nothing has been sent to an unbound port, but a "nothing forwarded"
    // check must not pass on a datagram the tick never read.
    ASSERT_TRUE(DatagramQueued(node));
    ASSERT_GT(QueuedBytes(node), 0);
    node.Tick(30000);
    ASSERT_EQ(QueuedBytes(node), 0);
  };
  const uint8_t malformed[] = {1, 2, 3};
  send(malformed, sizeof malformed);
  EXPECT_EQ(node.driver_station_packets_received(), 1u);
  EXPECT_FALSE(ds_sub.read());
}

// Commissioning is forced by --sim alone, the flag every node also reads
// (HardwareLink). A loopback bridge without --sim keeps the config's own id,
// as nodes launched without --sim do, or they would drop every State.
// REQ-SAFE-20
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
  state.health = {true, 9.5, true};
  state.motor_count = 2;
  state.motors[0].id = cfg.hardware.motors[1].id;
  state.motors[0].valid = true;
  state.motors[0].fault_flags = 5;
  state.motors[1].id = cfg.hardware.motors[0].id;
  state.motors[1].valid = true;
  // REQ-OPS-20: the gateway's counters, each its own value so a swap shows.
  state.counters = {.rejected_command_count = 7,
                    .neutralized_motor_count = 5,
                    .apply_failure_count = 3,
                    .device_reset_count = 2};
  ASSERT_TRUE(InjectState(node, gateway, state, 1000000));
  const auto status = node.Status(1001000);
  EXPECT_TRUE(status.health_valid());
  EXPECT_DOUBLE_EQ(status.battery_v(), 9.5);
  EXPECT_TRUE(status.brownout());
  EXPECT_EQ(status.motor_fault_mask(), 2u);
  EXPECT_EQ(status.gateway_rejected_command_count(), 7u);
  EXPECT_EQ(status.gateway_neutralized_motor_count(), 5u);
  EXPECT_EQ(status.gateway_apply_failure_count(), 3u);
  EXPECT_EQ(status.gateway_device_reset_count(), 2u);
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

// The RoboRIO's States each report one CAN bus, in turn; over the real link
// /hw/status carries every configured bus's slot with the figures its last
// report read, and alert_warning_can_bus_unread while one of them fails.
// REQ-OPS-20
TEST(HardwareNodeTest, PublishesEachCanBusSlotAndAlertsOnAFailingBus) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  cfg.hardware.motors[1].bus = "Shooter";  // Slots: 0 "rio", 1 "Shooter".
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  State state = TrustState(NodeConfigId(cfg), 42, 1, 1000);
  state.health = {.valid = true,
                  .battery_v = 12.5,
                  .can_bus_count = 2,
                  .can_bus_index = 0,
                  .can_bus = {CanBusReadState::kOk, 0.04F, 0, 0, 0, 0, 0}};
  ASSERT_TRUE(InjectState(node, link.gateway, state, 1'000'000));
  state.sample_time_us = 6000;
  state.health.can_bus_index = 1;
  state.health.can_bus = {CanBusReadState::kFailed, 0.16F, 2, 5, 96, 130, 3};
  ASSERT_TRUE(InjectState(node, link.gateway, state, 1'005'000));

  const auto status = node.Status(1'005'000);
  EXPECT_EQ(status.can_bus_count(), 2);
  const auto& rio = *status.can_buses()->Get(0);
  EXPECT_EQ(rio.read_state(), bridge::CanBusReadState::OK);
  EXPECT_FLOAT_EQ(rio.utilization_frac(), 0.04F);
  const auto& shooter = *status.can_buses()->Get(1);
  EXPECT_EQ(shooter.read_state(), bridge::CanBusReadState::FAILED);
  EXPECT_FLOAT_EQ(shooter.utilization_frac(), 0.16F);
  EXPECT_EQ(shooter.bus_off_count(), 2u);
  EXPECT_EQ(shooter.transmit_buffer_full_count(), 5u);
  EXPECT_EQ(shooter.receive_error_count(), 96);
  EXPECT_EQ(shooter.transmit_error_count(), 130);
  EXPECT_EQ(shooter.read_failure_count(), 3u);
  EXPECT_TRUE(status.alert_warning_can_bus_unread());
}

// A gateway backend that configures as the RoboRIO's does, asynchronously:
// PollConfigure reports kPending for `pending_polls` polls, then kDone.
// It reads no device and leaves every CAN bus slot unread.
class AsyncBackend final : public Backend {
 public:
  bool Configure(const Config&) override { return true; }
  void BeginConfigure(const Config&) override { polls_ = 0; }
  ConfigureStatus PollConfigure() override {
    return ++polls_ > pending_polls ? ConfigureStatus::kDone
                                    : ConfigureStatus::kPending;
  }
  bool Read(State&) override { return true; }
  bool Apply(std::span<const MotorRequest>) override { return true; }
  std::span<const uint16_t> ApplyFailedDevices() const override { return {}; }
  void Neutral() override {}
  int pending_polls{3};

 private:
  int polls_{0};
};

// The bridge against a real Gateway and Endpoint, as the RoboRIO runs them:
// the bridge pushes its configuration, the gateway applies it
// asynchronously and acknowledges it, and the States after it carry the
// configuration's buses. Scenario (bench, a 60 s session): the gateway's
// count from before the asynchronous commit was written back over the
// committed one, so every State carried can_bus_count 0 and /hw/status no
// bus, while a test that hands the bridge built States could not see it.
// REQ-OPS-20
TEST(HardwareNodeTest, ReportsTheBusesOfAnAsynchronouslyAppliedConfiguration) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  cfg.hardware.motors[1].bus = "Shooter";  // Slots: 0 "rio", 1 "Shooter".
  Link link;
  ASSERT_TRUE(link.Open(cfg, /*open_gateway=*/false));
  HardwareNode& node = link.node.value();
  AsyncBackend backend;
  Gateway gateway{backend, 42};
  Endpoint endpoint{gateway};
  Release(link.gateway_reservation);
  ASSERT_EQ(endpoint.Open("127.0.0.1", link.gateway_port, "127.0.0.1",
                          link.node_port),
            protocol::UdpStatus::kOk);
  uint64_t now = 1'000'000;
  ASSERT_TRUE(WaitFor([&] {
    now += 5000;
    endpoint.Tick(now, false);
    node.Tick(now);
    return node.is_configured() && node.Status(now).can_bus_count() != 0;
  })) << "can_bus_count "
      << int{node.Status(now).can_bus_count()};
  EXPECT_EQ(node.Status(now).can_bus_count(), 2);
}

// A RoboRIO program one protocol version behind (whose States and Commands
// another build's codecs would misread) is refused frame by frame: no State
// is accepted, so the link stays lost and nothing is commanded, and the
// bridge names the version on stderr once, not per frame.
// REQ-CONTRACT-01
TEST(HardwareNodeTest, RefusesAnOlderProtocolsStatesAndSaysSoOnce) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  talos::hardware::Packet payload{};
  payload.size = static_cast<uint32_t>(
      Encode(TrustState(NodeConfigId(cfg), 42, 1, 1000), payload.data));
  std::array<uint8_t, protocol::kMaxFrameSize> frame{};
  const std::size_t size = protocol::EncodeFrame(
      protocol::FrameType::kHardwareState, protocol::kFlagNone, 1, 1000,
      payload.data.data(), payload.size, frame.data(), frame.size());
  ASSERT_NE(size, 0u);
  frame[8] = static_cast<uint8_t>(protocol::kProtocolVersion - 1);
  frame[9] = 0;

  ::testing::internal::CaptureStderr();
  for (int i = 0; i < 2; ++i) {
    ASSERT_EQ(::send(link.gateway.NativeFd(), frame.data(), size, 0),
              static_cast<ssize_t>(size));
  }
  // A current frame behind them: once the bridge has it, it read both.
  ConfigAckPayload ack{};
  ack.config_id = NodeConfigId(cfg);
  ack.status = ConfigAckStatus::kOk;
  std::array<uint8_t, sizeof(ConfigAckPayload)> ack_bytes{};
  ASSERT_EQ(EncodeConfigAck(ack, ack_bytes), sizeof(ConfigAckPayload));
  ASSERT_EQ(link.gateway.SendFrame(protocol::FrameType::kHardwareConfigAck, 1,
                                   1000, ack_bytes.data(), ack_bytes.size()),
            protocol::UdpStatus::kOk);
  const bool delivered = WaitFor([&] {
    node.Tick(1'000'000);
    return node.is_configured();
  });
  const std::string err = ::testing::internal::GetCapturedStderr();
  ASSERT_TRUE(delivered);
  EXPECT_EQ(node.states_received(), 0u);
  EXPECT_EQ(node.commands_sent(), 0u);
  EXPECT_TRUE(node.Status(1'000'000).alert_error_link_lost());
  const std::string version =
      "protocol " + std::to_string(protocol::kProtocolVersion - 1);
  std::size_t lines = 0;
  for (auto at = err.find(version); at != std::string::npos;
       at = err.find(version, at + 1)) {
    ++lines;
  }
  EXPECT_EQ(lines, 1u) << err;
}

// REQ-OPS-20: receiving a UDP frame with an unsupported protocol version raises
// alert_error_protocol_mismatch and reports mismatched_protocol_version_id on
// /hw/status; stopping mismatched frames clears both after command_timeout_us
// (non-latching), and receiving any valid-version frame clears both at once.
TEST(HardwareNodeTest, ReportsAndClearsProtocolMismatchOnStatus) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const uint64_t timeout_us = cfg.hardware.command_timeout_us;
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();

  constexpr uint64_t kStartUs = 1'000'000;
  node.Tick(kStartUs);
  const bridge::BridgeStatus initial = node.Status(kStartUs);
  EXPECT_FALSE(initial.alert_error_protocol_mismatch());
  EXPECT_EQ(initial.mismatched_protocol_version_id(), 0u);
  EXPECT_EQ(initial.protocol_version_id(), protocol::kProtocolVersion);

  const auto send_wrong_version = [&](uint16_t version, uint64_t seq) {
    talos::hardware::Packet payload{};
    payload.size = static_cast<uint32_t>(
        Encode(TrustState(NodeConfigId(cfg), 42, 1, 1000), payload.data));
    std::array<uint8_t, protocol::kMaxFrameSize> frame{};
    const std::size_t size = protocol::EncodeFrame(
        protocol::FrameType::kHardwareState, protocol::kFlagNone, seq, 1000,
        payload.data.data(), payload.size, frame.data(), frame.size());
    ASSERT_NE(size, 0u);
    frame[8] = static_cast<uint8_t>(version & 0xFFu);
    frame[9] = static_cast<uint8_t>((version >> 8) & 0xFFu);
    ASSERT_EQ(::send(link.gateway.NativeFd(), frame.data(), size, 0),
              static_cast<ssize_t>(size));
  };

  // Wrong-version frame raises the alert and records the peer's version.
  const uint16_t older = static_cast<uint16_t>(protocol::kProtocolVersion - 1);
  send_wrong_version(older, 1);
  ASSERT_TRUE(DatagramQueued(node));
  ::testing::internal::CaptureStderr();
  node.Tick(kStartUs + 1000);
  (void)::testing::internal::GetCapturedStderr();
  const bridge::BridgeStatus mismatched = node.Status(kStartUs + 1000);
  EXPECT_TRUE(mismatched.alert_error_protocol_mismatch());
  EXPECT_EQ(mismatched.mismatched_protocol_version_id(), older);
  EXPECT_EQ(mismatched.protocol_version_id(), protocol::kProtocolVersion);

  // Stays active through command_timeout_us, then clears once packets stop.
  const bridge::BridgeStatus at_boundary =
      node.Status(kStartUs + 1000 + timeout_us);
  EXPECT_TRUE(at_boundary.alert_error_protocol_mismatch());
  EXPECT_EQ(at_boundary.mismatched_protocol_version_id(), older);
  const bridge::BridgeStatus after_timeout =
      node.Status(kStartUs + 1000 + timeout_us + 1);
  EXPECT_FALSE(after_timeout.alert_error_protocol_mismatch());
  EXPECT_EQ(after_timeout.mismatched_protocol_version_id(), 0u);

  // Another mismatch raises again, and a valid-version frame clears both
  // immediately (well before command_timeout_us).
  const uint16_t newer = static_cast<uint16_t>(protocol::kProtocolVersion + 1);
  const uint64_t t1 = kStartUs + 2 * timeout_us;
  send_wrong_version(newer, 2);
  ASSERT_TRUE(DatagramQueued(node));
  node.Tick(t1);
  EXPECT_TRUE(node.Status(t1).alert_error_protocol_mismatch());
  EXPECT_EQ(node.Status(t1).mismatched_protocol_version_id(), newer);

  ConfigAckPayload ack{};
  ack.config_id = NodeConfigId(cfg);
  ack.status = ConfigAckStatus::kOk;
  std::array<uint8_t, sizeof(ConfigAckPayload)> ack_bytes{};
  ASSERT_EQ(EncodeConfigAck(ack, ack_bytes), sizeof(ConfigAckPayload));
  ASSERT_EQ(link.gateway.SendFrame(protocol::FrameType::kHardwareConfigAck, 3,
                                   2000, ack_bytes.data(), ack_bytes.size()),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(WaitFor([&] {
    node.Tick(t1 + 1000);
    return node.is_configured();
  }));
  const bridge::BridgeStatus cleared = node.Status(t1 + 1000);
  EXPECT_FALSE(cleared.alert_error_protocol_mismatch());
  EXPECT_EQ(cleared.mismatched_protocol_version_id(), 0u);
}

// /hw/status names the build of the gateway boot the bridge accepted, and a
// new boot's build replaces it: after a RoboRIO rollback the record shows the
// build that drove the robot, not the first one the bridge heard. Its stderr
// names each boot's build once, however many States the boot sends.
// REQ-CONTRACT-30, REQ-OPS-21
TEST(HardwareNodeTest, ReportsTheBuildOfTheAcceptedGatewayBoot) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  EXPECT_EQ(node.Status(999000).gateway_program(),
            bridge::GatewayProgram::UNKNOWN);
  EXPECT_EQ(node.Status(999000).gateway_build_id(), 0u);

  ::testing::internal::CaptureStderr();
  State state = TrustState(NodeConfigId(cfg), 42, 1, 1000);
  state.build = {0x0123456789abcdefull, GatewayProgram::kRoboRio, true};
  ASSERT_TRUE(InjectState(node, link.gateway, state, 1000000));
  auto status = node.Status(1000000);
  EXPECT_EQ(status.gateway_program(), bridge::GatewayProgram::ROBORIO);
  EXPECT_TRUE(status.gateway_build_dirty());
  EXPECT_EQ(status.gateway_build_id(), 0x0123456789abcdefull);
  state.sample_time_us = 6000;
  ASSERT_TRUE(InjectState(node, link.gateway, state, 1000500));

  State rollback = TrustState(NodeConfigId(cfg), 43, 1, 50);
  rollback.build = {0x00fedcba98765432ull, GatewayProgram::kRoboRio, false};
  ASSERT_TRUE(InjectState(node, link.gateway, rollback, 1001000));
  status = node.Status(1001000);
  EXPECT_EQ(status.gateway_program(), bridge::GatewayProgram::ROBORIO);
  EXPECT_FALSE(status.gateway_build_dirty());
  EXPECT_EQ(status.gateway_build_id(), 0x00fedcba98765432ull);
  rollback.sample_time_us = 5050;
  ASSERT_TRUE(InjectState(node, link.gateway, rollback, 1006000));
  const std::string err = ::testing::internal::GetCapturedStderr();
  const auto count = [&err](const std::string& revision) {
    std::size_t lines = 0;
    for (auto at = err.find(revision); at != std::string::npos;
         at = err.find(revision, at + 1)) {
      ++lines;
    }
    return lines;
  };
  EXPECT_EQ(count("0123456789abcdef"), 1u) << err;
  EXPECT_EQ(count("00fedcba98765432"), 1u) << err;
}

// Guards: hardware_node --log records the merged /hw/command, the decoded
// /hw/state and every /hw/status it publishes, each status SEND with the
// transport sequence and publication stamp a live reader received for it
// (what input replay matches a reader's copy to the bridge's SEND by),
// its payload the status with the bridge's protocol. A run that ends on its
// own (here at --duration-s) ends the log with an EXIT record, as an event
// loop's does: input replay judges a log_gap session's log with its EXIT by
// its GAP records, and refuses one without it.
// REQ-OPS-21, REQ-OPS-44, REQ-OPS-14
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
  // A live /hw/status reader, in sequence, registered before the bridge
  // starts: every status it publishes, (transport sequence, publish stamp).
  RTMSQueue status_reader{
      bridge::topics::kStatus,
      sizeof(bridge::BridgeStatus),
      alignof(bridge::BridgeStatus),
      schema::Of<bridge::BridgeStatus>(),
      MAX_SLOTS,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::SEQUENCE}};
  const std::optional<std::size_t> status_reader_id =
      status_reader.register_reader();
  ASSERT_TRUE(status_reader_id);
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
  // The ring holds the run's ~50 statuses (1 s at 50 Hz) well within its
  // window, so reading after the bridge exited loses none.
  std::vector<std::pair<uint64_t, int64_t>> statuses_seen;
  {
    bridge::BridgeStatus published{};
    MessageInfo info{};
    while (status_reader.read_next(
               *status_reader_id,
               std::as_writable_bytes(std::span{&published, 1}),
               info) == ReadResult::OK) {
      statuses_seen.emplace_back(info.sequence, info.publish_time_ns);
    }
  }
  ASSERT_FALSE(statuses_seen.empty());
  event::log::LogReader reader(log_path);
  bool logged_command = false;
  bool logged_neutral = false;
  bool logged_state = false;
  std::vector<std::pair<uint64_t, int64_t>> statuses_logged;
  std::optional<event::EventKind> last_kind;
  event::log::LogReader::Record record;
  while (reader.next(record)) {
    const auto kind = static_cast<event::EventKind>(record.header.kind);
    ASSERT_NE(last_kind, event::EventKind::EXIT) << "a record after the EXIT";
    last_kind = kind;
    if (kind == event::EventKind::EXIT) continue;
    ASSERT_LT(record.header.source_id, reader.manifest().size());
    const auto& source = reader.manifest()[record.header.source_id];
    if (source.name == bridge::topics::kStatus) {
      ASSERT_EQ(record.header.kind,
                static_cast<std::uint16_t>(event::EventKind::SEND));
      ASSERT_EQ(record.payload.size(), sizeof(bridge::BridgeStatus));
      bridge::BridgeStatus logged{};
      std::memcpy(&logged, record.payload.data(), sizeof(logged));
      EXPECT_EQ(logged.protocol_version_id(), protocol::kProtocolVersion);
      statuses_logged.emplace_back(record.header.sequence,
                                   record.header.publish_time_ns);
      continue;
    }
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
  EXPECT_EQ(statuses_logged, statuses_seen);
  EXPECT_EQ(last_kind, event::EventKind::EXIT)
      << "the log of a run that ended on its own has no EXIT record";
}

// A config that declares itself for simulation only (here
// through its own [robot] over an included deployable base, as a scaffolded
// robot_sim.toml does) never reaches a controller from a hardware_node started
// by hand without --sim: it exits non-zero naming the key, the config and the
// flag, having sent nothing. Its --describe, which opens nothing, still
// answers, so the launcher's --describe-only probe without --sim works.
// REQ-SAFE-40
TEST(HardwareNodeTest, ASimulationOnlyConfigRunsOnlyWithSim) {
  const std::string base = ::testing::TempDir() + "simulation_only";
  const std::string deployable_path = base + "_deployable.toml";
  const std::string config_path = base + ".toml";
  {
    std::ofstream file(deployable_path);
    file << kTrustToml;
  }
  {
    std::ofstream file(config_path);
    file << "include = \""
         << std::filesystem::path(deployable_path).filename().string()
         << "\"\n[robot]\nsimulation_only = true\n";
  }
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  ASSERT_TRUE(runfiles) << error;
  const char* executable_path = std::getenv("HARDWARE_NODE");
  ASSERT_NE(executable_path, nullptr);
  const auto executable = runfiles->Rlocation(executable_path);
  // Runs hardware_node with `args`, its stdout and stderr in `output_path`,
  // and returns its exit status, or -1 if it did not exit.
  const auto run = [&executable](const std::vector<std::string>& args,
                                 const std::string& output_path) {
    std::vector<char*> argv;
    argv.reserve(args.size() + 2);
    argv.push_back(const_cast<char*>(executable.c_str()));
    for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
    argv.push_back(nullptr);
    const pid_t child = fork();
    if (child < 0) return -1;
    if (child == 0) {
      const int out =
          open(output_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
      if (out < 0 || dup2(out, STDOUT_FILENO) < 0 ||
          dup2(out, STDERR_FILENO) < 0) {
        _exit(126);
      }
      execv(executable.c_str(), argv.data());
      _exit(127);
    }
    int status = 0;
    if (waitpid(child, &status, 0) != child || !WIFEXITED(status)) return -1;
    return WEXITSTATUS(status);
  };
  const auto read = [](const std::string& path) {
    std::ifstream file(path);
    return std::string{std::istreambuf_iterator<char>(file), {}};
  };

  // The controller's end: a bound loopback socket the bridge would send its
  // probes and configuration to. The bridge's own port stays reserved
  // through its run: a bridge that refuses before it opens a socket never
  // needs it, and one that opened it first would fail to bind and say so
  // instead of naming the key, so the refusal is proved to come first.
  uint16_t gateway_port = 0;
  uint16_t node_port = 0;
  int gateway_reservation = ReservePort(gateway_port);
  int node_reservation = ReservePort(node_port);
  const bool reserved = gateway_reservation >= 0 && node_reservation >= 0;
  Release(gateway_reservation);
  if (!reserved) Release(node_reservation);
  ASSERT_TRUE(reserved);
  protocol::RuntimeUdpPeer gateway;
  ASSERT_EQ(gateway.Open("127.0.0.1", gateway_port, "127.0.0.1", node_port),
            protocol::UdpStatus::kOk);

  // --duration-s bounds a bridge that wrongly runs: it then exits 0 after
  // sending to the gateway, instead of hanging the test.
  const std::string run_output = base + "_run.txt";
  EXPECT_EQ(run({"--config", config_path, "--remote", "127.0.0.1",
                 "--remote-port", std::to_string(gateway_port), "--local-port",
                 std::to_string(node_port), "--duration-s", "1"},
                run_output),
            1);
  Release(node_reservation);
  const std::string message = read(run_output);
  EXPECT_NE(message.find(config_path + " sets [robot] simulation_only = true; "
                                       "hardware_node runs it only with --sim"),
            std::string::npos)
      << message;
  // Exact, not a wait: the bridge has exited, and a datagram it sent over
  // loopback was queued on this socket before its send returned.
  protocol::DecodedFrame frame;
  EXPECT_EQ(gateway.TryReceive(&frame), protocol::UdpStatus::kWouldBlock)
      << "the bridge sent a frame to the controller";

  const std::string describe_output = base + "_describe.json";
  EXPECT_EQ(run({"--config", config_path, "--describe"}, describe_output), 0)
      << read(describe_output);
  EXPECT_NE(read(describe_output).find("\"describe_version\""),
            std::string::npos)
      << read(describe_output);
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

  // The first tick publishes. Nothing has arrived from the controller, so
  // no actuator subsystem is reported silent yet.
  node.Tick(999000);
  const bridge::BridgeStatus initial = node.Status(999000);
  EXPECT_EQ(initial.states_received_count(), 0u);
  EXPECT_EQ(initial.state_age_us(), 0u);
  EXPECT_EQ(initial.subsystem_count(), 2u);
  EXPECT_EQ(initial.timed_out_mask(), 0u);
  EXPECT_FALSE(initial.configured());
  auto published = status_sub.read();
  ASSERT_TRUE(published.has_value());
  const bridge::BridgeStatus first = published.value_or(bridge::BridgeStatus{});
  EXPECT_TRUE(first == initial);
  // Guards: every status names the hardware protocol of the bridge's own
  // build, before any State, so a session's log tools read its packets
  // under what the bridge spoke, not the launcher's constant. REQ-OPS-51
  EXPECT_EQ(first.protocol_version_id(), protocol::kProtocolVersion);

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
  // The first tick sent to the gateway port before anything was bound there,
  // so the socket may first be readable for that error alone: tick until the
  // ACK has been read. Each wait is bounded; with no ACK accepted the last one
  // times out.
  while (!node.is_configured()) {
    ASSERT_TRUE(DatagramQueued(node)) << "no config ACK configured the bridge";
    node.Tick(999500);
  }
  ASSERT_TRUE(node.is_configured());
  EXPECT_TRUE(node.Status(999500).configured());

  // State age runs on the bridge clock from the frame's receipt.
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(config_id, 11, 1, 1000), 1000000));
  EXPECT_EQ(node.Status(1000000).states_received_count(), 1u);
  EXPECT_EQ(node.Status(1000000).state_age_us(), 0u);
  EXPECT_EQ(node.Status(1003500).state_age_us(), 3500u);

  // Only the drivetrain publishes, once: it (bit 0; intake is bit 1, sorted
  // name order) gets its bit one owner timeout later. The intake, never
  // heard, gets none.
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
  EXPECT_EQ(node.Status(1001000).timed_out_mask(), 0u);
  const uint64_t owner_timeout_us =
      OwnerTimeoutUs(5000, cfg.hardware.command_timeout_us);
  EXPECT_EQ(node.Status(1001000 + owner_timeout_us + 1).timed_out_mask(),
            0b01u);
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
  EXPECT_EQ(later.timed_out_mask(), 0u);
  EXPECT_EQ(later.state_age_us(), 19000u);
  EXPECT_TRUE(later.configured());
}

// Guards: the bridge's --log registers /hw/status as a sender of this build's
// BridgeStatus schema and records a status SEND among its first Tick's
// records, carrying the bridge's own hardware protocol. That is what the log
// tools' reader of a session's protocol relies on
// (talOS/launcher/hardware_record.cc FirstBridgeStatusVersion, which takes
// the first such SEND within a log's first second, else falls back to the
// manifest's version, the launcher's).
// REQ-OPS-51
TEST(HardwareNodeTest, TheFirstTickLogsTheBridgesProtocol) {
  const std::string log_path = ::testing::TempDir() + "/bridge_first_tick_" +
                               std::to_string(::getpid()) + ".tlog";
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg, true, log_path, 43));
  link.node->Tick(1000);
  link.node.reset();  // flushes the bridge's log

  event::log::LogReader reader{log_path};
  std::optional<uint16_t> status_id;
  for (const auto& source : reader.manifest()) {
    if (source.kind == event::SourceKind::SENDER &&
        source.name == bridge::topics::kStatus &&
        source.schema == schema::Of<bridge::BridgeStatus>()) {
      status_id = source.id;
    }
  }
  ASSERT_TRUE(status_id) << "no /hw/status sender of this build's schema";
  std::vector<uint16_t> versions;
  event::log::LogReader::Record record;
  while (reader.next(record)) {
    if (record.header.source_id != *status_id ||
        record.header.kind !=
            static_cast<std::uint16_t>(event::EventKind::SEND)) {
      continue;
    }
    ASSERT_EQ(record.payload.size(), sizeof(bridge::BridgeStatus));
    bridge::BridgeStatus status;
    std::memcpy(&status, record.payload.data(), sizeof status);
    versions.push_back(status.protocol_version_id());
  }
  EXPECT_EQ(versions, std::vector<uint16_t>{protocol::kProtocolVersion});
  std::filesystem::remove(log_path);
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

// The send time of the next clock probe the gateway receives, skipping other
// frames; nullopt when none arrives within kHangBound, a bound on a hang only:
// every caller expects a probe a Tick already sent.
std::optional<uint64_t> NextProbeUs(protocol::RuntimeUdpPeer& gateway) {
  const auto deadline = std::chrono::steady_clock::now() + kHangBound;
  protocol::DecodedFrame frame;
  while (protocol::testing::ReceiveFrame(gateway, &frame, deadline) ==
         protocol::UdpStatus::kOk) {
    if (frame.header.type == protocol::FrameType::kClockProbe) {
      return frame.header.monotonic_time_us;
    }
  }
  return std::nullopt;
}

// REQ-RT-20
TEST(HardwareNodeTest, TwoWayClockBoundNeedsNoActiveRequests) {
  auto cfg = talos::config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
  node.Tick(2000000);
  const auto probe = NextProbeUs(gateway);
  ASSERT_TRUE(probe.has_value());
  const uint64_t sent = *probe;
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
  const auto first = NextProbeUs(gateway);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(inject(2'005'000, *first));
  for (uint64_t t = 2'006'000; t <= 2'012'000; t += 1000) {
    ASSERT_TRUE(inject(t, 0));
  }
  ASSERT_TRUE(node.Status(2'012'000).clock_offset_valid());

  // The next probe is lost.
  node.Tick(2'020'000);
  ASSERT_TRUE(NextProbeUs(gateway).has_value());

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

// REQ-CONTRACT-04
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

  // After backing off to 2 s via rejected ACKs, a State from a new boot_id
  // triggers an immediate push and resets the next retry gate to 50 ms on no
  // ACK.
  now += 1000;
  uint64_t reboot_sample_us = 1000;
  ASSERT_TRUE(
      InjectState(node, gateway, TrustState(id, 12, 1, reboot_sample_us), now));
  ASSERT_EQ(ReceiveConfigs(gateway), 1u);
  ASSERT_TRUE(InjectState(node, gateway,
                          TrustState(id, 12, 1, reboot_sample_us + 49'999),
                          now + 49'999));
  EXPECT_EQ(drain_configs(), 0u);
  ASSERT_TRUE(InjectState(node, gateway,
                          TrustState(id, 12, 1, reboot_sample_us + 50'000),
                          now + 50'000));
  ASSERT_EQ(ReceiveConfigs(gateway), 1u);
}

// REQ-CONTRACT-04: when no ConfigAck arrives (lost push or ACK), the bridge
// retries on the 50 ms gate while subsystem commands keep flowing.
TEST(HardwareNodeTest, LostConfigPushRetriesAt50MsWithoutStoppingCommands) {
  auto cfg = config::ParseRobotConfigString(kTestToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
  const auto id = NodeConfigId(cfg);
  uint64_t now = 1'000'000;
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
  const auto before = node.commands_sent();
  ASSERT_TRUE(InjectState(node, gateway, TrustState(id, 11, 1, now + 49'999),
                          now + 49'999));
  EXPECT_EQ(drain_configs(), 0u);
  Command request{};
  request.config_id = id;
  request.boot_id = 11;
  request.epoch = 1;
  request.observed_time_us = now + 49'999;
  request.count = 1;
  request.motors[0] = {cfg.subsystems.at("drivetrain").motors[0],
                       Mode::kDutyCycle, 0, 0.2};
  PublishCommand(pub, request);
  node.Tick(now + 49'999);
  EXPECT_GT(node.commands_sent(), before);
  EXPECT_EQ(node.current_command().motors[0].mode, Mode::kDutyCycle);
  EXPECT_EQ(drain_configs(), 0u);

  ASSERT_TRUE(InjectState(node, gateway, TrustState(id, 11, 1, now + 50'000),
                          now + 50'000));
  ASSERT_EQ(ReceiveConfigs(gateway), 1u);
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
  // retry, and that one's retry. A Tick a second later sends at least the
  // unanswered probe's retry, stamped with its own instant; one socket's
  // datagrams arrive in order, so its frame ends the stream, and the read
  // waits for that event rather than for nothing to arrive.
  const uint64_t end_us = kStartUs + 1'300'000;
  node.Tick(end_us);
  std::vector<SentCommand> commands;
  std::vector<uint64_t> probe_us;
  protocol::DecodedFrame frame;
  bool ended = false;
  const auto give_up = std::chrono::steady_clock::now() + kHangBound;
  while (!ended && protocol::testing::ReceiveFrame(gateway, &frame, give_up) ==
                       protocol::UdpStatus::kOk) {
    ended = frame.header.monotonic_time_us == end_us;
    if (ended) continue;
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
  ASSERT_TRUE(ended) << "no frame from the Tick at " << end_us;
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
// this takes at most the 64 a 1000 us window allows, the requirement's
// numbers, asserted as such rather than as the constants that set them.
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
    EXPECT_LE(in_window, 64u) << "at " << now_us;
    if (wait.ignore_input) {
      saw_spent = true;
      EXPECT_GT(wait.until_us, now_us);
      EXPECT_LE(wait.until_us, now_us + 1000);
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
  // 8 passes a 1000 us window (the requirement's numbers, not the constants
  // that set them), plus one pass per duty that ends a sleep early (100 ms:
  // 5 statuses, 3 heartbeats or the neutral, a probe retry; the registry's
  // liveness rides the passes); unbudgeted, the flood makes
  // kFloodUs / kPassCostUs.
  const uint64_t windows = (kFloodUs / 1000) + 1;
  EXPECT_LE(passes, (windows * 8) + 10)
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
// the loop waits for its next duty anyway: it cannot spin. Each request
// topic whose failed arm the guard stops holding the loop awake counts once
// in stalled_arms, which reaches the bridge's registry row (TODO.md RT-7),
// stays stalled across subsequent passes without re-spinning, and arms fresh
// again once a read returns a request from that topic.
// REQ-RT-20, REQ-OPS-43
TEST(HardwareNodeTest, AnArmThatNeverCatchesUpCannotSpinTheLoop) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  const auto reader = introspect::RegistryReader::open();
  ASSERT_TRUE(reader.has_value());
  node.set_seams({.arm_request = &NeverCaughtUp});
  uint64_t now_us = 1'000'000;
  // `retries` immediate passes, then one that waits (the guard) until the time
  // it returns, where the next pass runs.
  const auto stall = [&node, &now_us](uint64_t retries) {
    for (uint64_t i = 0; i < retries; ++i, ++now_us) {
      EXPECT_EQ(node.Pass(now_us).until_us, now_us) << i;
    }
    const HardwareNode::Wait wait = node.Pass(now_us);
    EXPECT_FALSE(wait.ignore_input);
    EXPECT_GT(wait.until_us, now_us);
    const uint64_t waited_us = wait.until_us - now_us;
    now_us = wait.until_us;
    return waited_us;
  };
  EXPECT_EQ(stall(HardwareNode::kMaxIdlePasses), 10'000u);  // the longest wait
  // Both subsystems' arms (drivetrain, intake) stalled.
  EXPECT_EQ(node.wakeups().stalled_arms, 2u);
  // Once stalled, subsequent passes wait immediately (here until the next
  // 20 ms /hw/status duty) without re-spinning or recounting stalled_arms.
  EXPECT_EQ(stall(0), 10'000u - HardwareNode::kMaxIdlePasses);
  EXPECT_EQ(node.wakeups().stalled_arms, 2u) << "once per stall, not per pass";

  ipc::Publisher<Packet> intake{RequestTopic("intake")};
  Command request{};
  request.config_id = NodeConfigId(cfg);
  PublishCommand(intake, request);
  // Reading intake's request resets intake's StallGuard; its failed arm in the
  // same pass uses the first of its kMaxIdlePasses retries.
  EXPECT_EQ(node.Pass(now_us).until_us, now_us) << "read the request";
  ++now_us;
  EXPECT_GT(stall(HardwareNode::kMaxIdlePasses - 1), 0u);
  EXPECT_EQ(node.wakeups().stalled_arms, 3u) << "intake stalled again";

  node.PublishIntrospection();
  std::optional<introspect::NodeSnapshot> bridge;
  for (auto& row : reader->nodes()) {
    if (row.name == kNodeName &&
        row.pid == static_cast<std::uint64_t>(getpid()) &&
        (!bridge || row.generation > bridge->generation))
      bridge = std::move(row);
  }
  ASSERT_TRUE(bridge.has_value());
  EXPECT_EQ(bridge->delivery_loss().stalled_arms, 3u);
}

// Guards ARCH-5 / RT-7: each request subscriber tracks its own StallGuard via
// WakeChannelWait::arm_reader, so steady UDP State datagrams arriving on every
// tick must NOT reset a stalled request subscriber's guard or hide it from
// stalled_arms.
// REQ-RT-20, REQ-OPS-43
TEST(HardwareNodeTest, StalledRequestArmTripsUnderSteadyUdpTraffic) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  const auto reader = introspect::RegistryReader::open();
  ASSERT_TRUE(reader.has_value());
  node.set_seams({.arm_request = &NeverCaughtUp});

  uint64_t now_us = 1'000'000;
  uint64_t sample_us = 1'000;
  const uint64_t config_id = NodeConfigId(cfg);
  const auto pass_with_udp_state = [&]() {
    sample_us += 1'000;
    const uint64_t states_before = node.states_received();
    EXPECT_TRUE(
        SendState(link.gateway, TrustState(config_id, 11, 1, sample_us)));
    EXPECT_TRUE(DatagramQueued(node));
    const HardwareNode::Wait wait = node.Pass(now_us);
    EXPECT_EQ(node.states_received(), states_before + 1)
        << "each pass must consume a UDP datagram";
    return wait;
  };

  // kMaxIdlePasses retries, each consuming a real UDP datagram, then the next
  // pass trips both request subscribers' guards and waits instead of spinning.
  for (uint64_t i = 0; i < HardwareNode::kMaxIdlePasses; ++i, now_us += 2'000) {
    const HardwareNode::Wait wait = pass_with_udp_state();
    EXPECT_FALSE(wait.ignore_input) << i;
    EXPECT_EQ(wait.until_us, now_us) << i;
  }
  const HardwareNode::Wait tripped = pass_with_udp_state();
  EXPECT_FALSE(tripped.ignore_input);
  EXPECT_GT(tripped.until_us, now_us)
      << "stalled request arms must stop spinning even with UDP traffic";
  EXPECT_EQ(node.wakeups().stalled_arms, 2u);

  // Another pass with another UDP datagram stays stalled without recounting.
  now_us += 2'000;
  const HardwareNode::Wait still_stalled = pass_with_udp_state();
  EXPECT_FALSE(still_stalled.ignore_input);
  EXPECT_GT(still_stalled.until_us, now_us);
  EXPECT_EQ(node.wakeups().stalled_arms, 2u);

  node.PublishIntrospection();
  std::optional<introspect::NodeSnapshot> bridge;
  for (auto& row : reader->nodes()) {
    if (row.name == kNodeName &&
        row.pid == static_cast<std::uint64_t>(getpid()) &&
        (!bridge || row.generation > bridge->generation))
      bridge = std::move(row);
  }
  ASSERT_TRUE(bridge.has_value());
  EXPECT_EQ(bridge->delivery_loss().stalled_arms, 2u);
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

// Which send errors replace the socket. One the network clears by itself
// (the gateway's port closed, no interface yet, an ICMP host unreachable, a
// full interface queue) keeps it: nothing is reopened. One that every later
// send on the same socket repeats closes it, and the next Tick opens a new
// one: ENETUNREACH is what Linux returns once the socket's source address
// has left the host, even after a new address appears
// (UdpAddressChangeTest), EADDRNOTAVAIL what macOS returns. Either way the
// failed probe goes out kRetryUs (1 ms) later.
// REQ-RT-22
TEST(HardwareNodeTest, OnlyASendErrorTheSocketKeepsRepeatingReplacesIt) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  protocol::RuntimeUdpPeer& gateway = link.gateway;
  node.set_seams({.send_error = &TakeSendError});
  struct Row {
    int error;
    bool replaced;
  };
  uint64_t now_us = 1'000'000;
  uint64_t reopens = 0;
  for (const Row row : {Row{ECONNREFUSED, false}, Row{ENETDOWN, false},
                        Row{EHOSTUNREACH, false}, Row{ENOBUFS, false},
                        Row{ENETUNREACH, true}, Row{EADDRNOTAVAIL, true}}) {
    SCOPED_TRACE(std::strerror(row.error));
    pending_send_error = row.error;
    node.Tick(now_us);  // the probe is due, and fails
    EXPECT_EQ(node.socket_counters().last_send_error, row.error);
    EXPECT_EQ(node.udp_fd() < 0, row.replaced);
    // The loop wakes for it 1 ms later, not at the probe's next period.
    EXPECT_EQ(node.NextDutyUs(now_us), now_us + 1'000);
    node.Tick(now_us + 1'000);
    reopens += row.replaced ? 1 : 0;
    EXPECT_EQ(node.socket_counters().reopens, reopens);
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

}  // namespace

// node.cc: tests only, applied to the options of the bridge's --log, and
// whether its full registry sample copies the log's flags.
extern std::function<void(event::log::LogWriterOptions&)> log_options_for_test;
extern bool copy_log_flags_for_test;

namespace {

// Whether this process's newest bridge row in the live registry has
// kFlagLogGap.
bool BridgeHasLogGap(const introspect::RegistryReader& reader) {
  bool gap = false;
  std::uint64_t generation = 0;
  for (const auto& row : reader.nodes()) {
    if (row.name != kNodeName ||
        row.pid != static_cast<std::uint64_t>(getpid()) ||
        row.generation < generation)
      continue;
    generation = row.generation;
    gap = row.log_gap();
  }
  return gap;
}

// A bridge with a --log whose writer is held before its first write, so
// its two chunks (each sized for the largest record, a hardware packet)
// fill with /hw/status records and the log opens a gap; its full registry
// samples copy no log flag meanwhile, so the row's kFlagLogGap can only be
// the writer's. Release() lets the writer go.
struct HeldBridgeLog {
  HeldBridgeLog() {
    copy_log_flags_for_test = false;
    log_options_for_test = [this](event::log::LogWriterOptions& options) {
      options.chunk_bytes = 1;
      options.chunk_count = kChunks;
      options.before_write = [this] { released.wait(false); };
    };
  }
  ~HeldBridgeLog() {
    Release();
    log_options_for_test = nullptr;
    copy_log_flags_for_test = true;
  }
  HeldBridgeLog(const HeldBridgeLog&) = delete;
  HeldBridgeLog& operator=(const HeldBridgeLog&) = delete;
  void Release() {
    released.store(true);
    released.notify_all();
  }
  static constexpr std::size_t kChunks = 2;
  std::atomic<bool> released{false};
};

// The /hw/status publications that fill a held HeldBridgeLog's pool and open
// a gap, from the record sizes: each chunk is sized for the largest record
// (LogWriter::start), a hardware packet's, and holds whole status records.
constexpr std::size_t kStatusesToOpenAGap = [] {
  const std::size_t chunk =
      std::max({sizeof(event::log::RecordHeader) + sizeof(Packet),
                sizeof(event::log::RecordHeader) + sizeof(bridge::BridgeStatus),
                event::log::GAP_RECORD_BYTES});
  const std::size_t status =
      sizeof(event::log::RecordHeader) + sizeof(bridge::BridgeStatus);
  return HeldBridgeLog::kChunks * (chunk / status) + 1;
}();

// Passes from `start_us` every 5 ms until the bridge's row shows a log gap,
// or for twice the time a held log's pool takes to fill with /hw/status
// (one every 20 ms). Returns whether the row showed it.
bool PassUntilTheRowShowsAGap(HardwareNode& node, uint64_t start_us,
                              const introspect::RegistryReader& reader) {
  const uint64_t end_us = start_us + 2 * kStatusesToOpenAGap * 20'000;
  for (uint64_t now_us = start_us; now_us < end_us; now_us += 5'000) {
    node.Pass(now_us);
    if (BridgeHasLogGap(reader)) return true;
  }
  return false;
}

// The bridge's registry row says its log has a gap as the gap opens, not
// at its next full registry sample: a bridge killed in between is still
// marked log_gap. /hw/status records, one every 20 ms, fill the held
// writer's pool.
// REQ-OPS-52
TEST(HardwareNodeTest, ItsRowHasTheLogGapBitAsTheGapOpens) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const std::string log_path = ::testing::TempDir() + "/bridge_row_gap_" +
                               std::to_string(getpid()) + ".tlog";
  Link link;  // destroyed after `held` lets its writer go
  HeldBridgeLog held;
  ASSERT_TRUE(link.Open(cfg, true, log_path, 51));
  const auto reader = introspect::RegistryReader::open();
  ASSERT_TRUE(reader.has_value());
  EXPECT_TRUE(PassUntilTheRowShowsAGap(*link.node, 1'000'000, *reader))
      << "no kFlagLogGap in the bridge's row once its pool filled";
  held.Release();
  link.node.reset();
  std::filesystem::remove(log_path);
}

// A bridge run whose own --log has a gap exits 1 when stopped, as a node
// does (Run()'s incomplete() call), and says recording_gaps=1. The writer
// is held until the gap opens, then released before the stop, so no second
// gap can open.
// REQ-OPS-52
TEST(HardwareNodeTest, ARunWhoseOwnLogHasAGapExits1) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const std::string log_path = ::testing::TempDir() + "/bridge_exit_gap_" +
                               std::to_string(getpid()) + ".tlog";
  Link link;  // destroyed after `held` lets its writer go
  HeldBridgeLog held;
  ASSERT_TRUE(link.Open(cfg, true, log_path, 52));
  const auto reader = introspect::RegistryReader::open();
  ASSERT_TRUE(reader.has_value());
  ASSERT_TRUE(PassUntilTheRowShowsAGap(*link.node, 1'000'000, *reader));
  held.Release();
  // Stopped before its first wait: it exits through the summary at once,
  // having reached no duration of its own.
  process::stop_requested.store(true);
  ::testing::internal::CaptureStdout();
  const int status = link.node->Run();
  const std::string output = ::testing::internal::GetCapturedStdout();
  process::stop_requested.store(false);
  EXPECT_EQ(status, 1) << output;
  EXPECT_NE(output.find("recording_gaps=1 "), std::string::npos) << output;
  link.node.reset();
  std::filesystem::remove(log_path);
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
// REQ-RT-06
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

// REQ-OPS-21, REQ-RT-22: /hw/command is published and logged only when
// SendToGateway succeeds, so IPC observers and the event log match what
// actually left on the wire.
TEST(HardwareNodeTest, FailedCommandSendIsNotPublishedOrLogged) {
  const std::string log_path = ::testing::TempDir() + "/bridge_failed_send_" +
                               std::to_string(getpid()) + ".tlog";
  std::filesystem::remove(log_path);
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg, true, log_path, 61));
  HardwareNode& node = link.node.value();
  ipc::Subscriber<Packet> cmd_sub{
      kCommandTopic,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::SEQUENCE}};
  ipc::Publisher<Packet> intake{RequestTopic("intake")};

  ASSERT_TRUE(InjectState(node, link.gateway,
                          TrustState(NodeConfigId(cfg), 11, 1, 1000),
                          1'000'000));
  // Fail the first command send with a transient socket error.
  pending_send_error = ECONNREFUSED;
  node.set_seams({.send_error = &TakeSendError});
  PublishCommand(intake, IntakeRequest(cfg, 4.0));
  node.Tick(1'001'000);
  EXPECT_EQ(node.commands_sent(), 0u);
  EXPECT_FALSE(cmd_sub.read().has_value())
      << "failed UDP send must not publish on /hw/command";

  // Next retry succeeds: published and logged once.
  node.Tick(1'002'000);
  EXPECT_EQ(node.commands_sent(), 1u);
  EXPECT_TRUE(cmd_sub.read().has_value());

  link.node.reset();  // flushes the log
  event::log::LogReader reader{log_path};
  std::size_t logged_commands = 0;
  event::log::LogReader::Record record;
  while (reader.next(record)) {
    if (record.header.kind !=
        static_cast<std::uint16_t>(event::EventKind::SEND)) {
      continue;
    }
    if (reader.manifest()[record.header.source_id].name == kCommandTopic) {
      ++logged_commands;
    }
  }
  EXPECT_EQ(logged_commands, 1u);
  std::filesystem::remove(log_path);
}

// REQ-RT-22 (TODO.md RT-28): the exit summary prints UDP transport loss and
// decode counters from peer_.stats().
TEST(HardwareNodeTest, ExitSummaryReportsUdpGapsDecodeFailuresAndWouldBlock) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();

  // Send seq 1, a corrupt frame (decode failure), then seq 5 (gap of 3) over
  // the connected gateway socket so the bridge accepts them.
  auto send_raw_seq = [&](uint64_t seq) {
    std::array<uint8_t, protocol::kFrameHeaderSize> buf{};
    const std::size_t n = protocol::EncodeFrame(
        protocol::FrameType::kClockProbe, protocol::kFlagNone, seq, seq * 1000,
        nullptr, 0, buf.data(), buf.size());
    ASSERT_EQ(::send(link.gateway.NativeFd(), buf.data(), n, 0),
              static_cast<ssize_t>(n));
  };

  // Deliver each datagram and drain it before sending the next so macOS
  // loopback delivery of later datagrams cannot lag behind the first
  // DatagramQueued() wake.
  uint64_t now_us = 1'000'000;
  auto deliver = [&](auto&& send_one) {
    send_one();
    for (;;) {
      ASSERT_TRUE(DatagramQueued(node));
      const bool had_bytes = QueuedBytes(node) > 0;
      now_us += 1'000;
      node.Tick(now_us);
      if (had_bytes && QueuedBytes(node) == 0) break;
    }
  };

  deliver([&] { send_raw_seq(1); });
  const uint8_t garbage[] = {0xDE, 0xAD, 0xBE, 0xEF};
  deliver([&] {
    ASSERT_EQ(::send(link.gateway.NativeFd(), garbage, sizeof(garbage), 0),
              static_cast<ssize_t>(sizeof(garbage)));
  });
  deliver([&] { send_raw_seq(5); });

  process::stop_requested.store(true);
  ::testing::internal::CaptureStdout();
  static_cast<void>(node.Run());
  const std::string output = ::testing::internal::GetCapturedStdout();
  process::stop_requested.store(false);

  EXPECT_NE(output.find("sequence_gaps=3"), std::string::npos) << output;
  EXPECT_NE(output.find("decode_failures=1"), std::string::npos) << output;
  EXPECT_NE(output.find("send_would_block=0"), std::string::npos) << output;
}

// REQ-OPS-20: the bridge's exit summary prints each subsystem's count of
// owner-timeout episodes (timeouts=) and whether it was silent at the last
// Tick (timed_out=), leaving an unheard subsystem at timeouts=0 timed_out=0.
TEST(HardwareNodeTest, ExitSummaryReportsOwnerTimeoutEpisodes) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  const uint64_t config_id = NodeConfigId(cfg);
  const uint64_t owner_timeout_us =
      OwnerTimeoutUs(5000, cfg.hardware.command_timeout_us);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();

  constexpr uint64_t kStartUs = 1'000'000;
  ConfigAckPayload ack{};
  ack.config_id = config_id;
  ack.status = ConfigAckStatus::kOk;
  std::array<uint8_t, sizeof(ConfigAckPayload)> ack_bytes{};
  ASSERT_EQ(EncodeConfigAck(ack, ack_bytes), sizeof(ConfigAckPayload));
  ASSERT_EQ(link.gateway.SendFrame(protocol::FrameType::kHardwareConfigAck, 1,
                                   kStartUs - 500, ack_bytes.data(),
                                   ack_bytes.size()),
            protocol::UdpStatus::kOk);
  while (!node.is_configured()) {
    ASSERT_TRUE(DatagramQueued(node)) << "no config ACK configured the bridge";
    node.Tick(kStartUs - 500);
  }
  ASSERT_TRUE(InjectState(node, link.gateway,
                          TrustState(config_id, 11, 1, 1000), kStartUs));

  ipc::Publisher<Packet> intake_pub{RequestTopic("intake")};
  constexpr uint64_t kRequestUs = kStartUs + 1'000;
  PublishCommand(intake_pub, IntakeRequest(cfg, 4.0));
  node.Tick(kRequestUs);
  ASSERT_FALSE(node.is_subsystem_timed_out("intake"));
  ASSERT_EQ(node.owner_timeouts("intake"), 0u);

  // Keep the link fresh while intake goes silent past its owner timeout, then
  // recovers, times out a second time, and recovers again so requests (3),
  // cumulative timeouts (2), and final timed_out (0) are pairwise distinct.
  const uint64_t timeout1_us = kRequestUs + owner_timeout_us + 1;
  ASSERT_TRUE(InjectState(node, link.gateway,
                          TrustState(config_id, 11, 1, 2000),
                          kRequestUs + owner_timeout_us));
  node.Tick(timeout1_us);
  ASSERT_EQ(node.owner_timeouts("intake"), 1u);
  ASSERT_TRUE(node.is_subsystem_timed_out("intake"));

  const uint64_t request2_us = timeout1_us + 1'000;
  PublishCommand(intake_pub, IntakeRequest(cfg, 4.0));
  node.Tick(request2_us);
  ASSERT_FALSE(node.is_subsystem_timed_out("intake"));

  const uint64_t timeout2_us = request2_us + owner_timeout_us + 1;
  ASSERT_TRUE(InjectState(node, link.gateway,
                          TrustState(config_id, 11, 1, 3000),
                          request2_us + owner_timeout_us));
  node.Tick(timeout2_us);
  ASSERT_EQ(node.owner_timeouts("intake"), 2u);
  ASSERT_TRUE(node.is_subsystem_timed_out("intake"));

  const uint64_t request3_us = timeout2_us + 1'000;
  PublishCommand(intake_pub, IntakeRequest(cfg, 4.0));
  node.Tick(request3_us);
  ASSERT_FALSE(node.is_subsystem_timed_out("intake"));
  ASSERT_EQ(node.owner_timeouts("intake"), 2u);

  struct RunStopGuard {
    bool capturing{false};
    ~RunStopGuard() {
      if (capturing) (void)::testing::internal::GetCapturedStdout();
      process::stop_requested.store(false);
      static_cast<void>(process::ConsumeStopWake());
    }
    RunStopGuard() = default;
    RunStopGuard(const RunStopGuard&) = delete;
    RunStopGuard& operator=(const RunStopGuard&) = delete;
  };
  RunStopGuard guard;
  process::stop_requested.store(true);
  ::testing::internal::CaptureStdout();
  guard.capturing = true;
  const int exit_code = node.Run();
  guard.capturing = false;
  const std::string output = ::testing::internal::GetCapturedStdout();
  process::stop_requested.store(false);
  static_cast<void>(process::ConsumeStopWake());

  EXPECT_EQ(exit_code, 0) << output;
  EXPECT_NE(
      output.find(
          "hardware_node: sub=intake requests=3 timeouts=2 timed_out=0\n"),
      std::string::npos)
      << output;
  EXPECT_NE(
      output.find(
          "hardware_node: sub=drivetrain requests=0 timeouts=0 timed_out=0\n"),
      std::string::npos)
      << output;
}

// REQ-OPS-20 (TODO.md RT-17, RT-28): /hw/status carries the bridge's UDP
// socket and incoming State loss counters (socket_send_failures_count,
// socket_reopens_count, state_sequence_gaps_count, state_decode_failures_count,
// socket_send_would_block_count).
TEST(HardwareNodeTest, BridgeStatusReportsUdpSocketAndStateLossCounters) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  ipc::Subscriber<bridge::BridgeStatus> status_sub{
      bridge::topics::kStatus,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};

  // 1. Trigger a permanent send error (reopens the socket) and a transient
  // send error (counts a send failure without reopening).
  node.set_seams({.send_error = &TakeSendError});
  pending_send_error = EPIPE;
  node.Tick(1'000'000);  // clock probe fails -> send_failures=1, socket closed
  ASSERT_EQ(node.socket_counters().send_failures, 1u);
  ASSERT_EQ(node.socket_counters().reopens, 0u);
  node.Tick(1'001'000);  // reopens socket -> reopens=1, probe succeeds
  ASSERT_EQ(node.socket_counters().reopens, 1u);

  pending_send_error = ECONNREFUSED;
  node.Tick(1'200'000);  // next probe fails transiently -> send_failures=2
  ASSERT_EQ(node.socket_counters().send_failures, 2u);
  ASSERT_EQ(node.socket_counters().reopens, 1u);

  // 2. Send seq 1, a corrupt frame (decode failure), then seq 6 (gap of 4).
  auto send_raw_seq = [&](uint64_t seq) {
    std::array<uint8_t, protocol::kFrameHeaderSize> buf{};
    const std::size_t n = protocol::EncodeFrame(
        protocol::FrameType::kClockProbe, protocol::kFlagNone, seq, seq * 1000,
        nullptr, 0, buf.data(), buf.size());
    ASSERT_EQ(::send(link.gateway.NativeFd(), buf.data(), n, 0),
              static_cast<ssize_t>(n));
  };
  // Deliver each datagram and drain it before sending the next so macOS
  // loopback delivery of later datagrams cannot lag behind the first
  // DatagramQueued() wake.
  uint64_t now_us = 1'220'000;
  auto deliver = [&](auto&& send_one) {
    send_one();
    for (;;) {
      ASSERT_TRUE(DatagramQueued(node));
      const bool had_bytes = QueuedBytes(node) > 0;
      now_us += 5'000;
      node.Tick(now_us);
      if (had_bytes && QueuedBytes(node) == 0) break;
    }
  };
  deliver([&] { send_raw_seq(1); });
  const uint8_t garbage[] = {0xDE, 0xAD, 0xBE, 0xEF};
  deliver([&] {
    ASSERT_EQ(::send(link.gateway.NativeFd(), garbage, sizeof(garbage), 0),
              static_cast<ssize_t>(sizeof(garbage)));
  });
  deliver([&] { send_raw_seq(6); });

  // Ensure a fresh /hw/status publication after draining.
  now_us += 20'000;
  node.Tick(now_us);

  const auto published = status_sub.read();
  ASSERT_TRUE(published.has_value());
  const bridge::BridgeStatus status = *published;
  EXPECT_EQ(status.socket_send_failures_count(), 2u);
  EXPECT_EQ(status.socket_reopens_count(), 1u);
  EXPECT_EQ(status.state_sequence_gaps_count(), 4u);
  EXPECT_EQ(status.state_decode_failures_count(), 1u);
  EXPECT_EQ(status.socket_send_would_block_count(), 0u);
  EXPECT_TRUE(status == node.Status(now_us));

  // 3. A send the socket cannot take now (EAGAIN), from a socket whose send
  // buffer is full: a stream pair, filled to the last byte, put in place of
  // the bridge's socket on its descriptor. A datagram socket's sends on
  // loopback do not block, and a full Unix datagram socket says ENOBUFS on
  // macOS.
  std::array<int, 2> pair{-1, -1};
  ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, pair.data()), 0);
  ASSERT_EQ(::fcntl(pair[0], F_SETFL, O_NONBLOCK), 0);
  std::array<uint8_t, 4096> fill{};
  while (::send(pair[0], fill.data(), fill.size(), 0) > 0) {
  }
  while (::send(pair[0], fill.data(), 1, 0) > 0) {
  }
  ASSERT_TRUE(errno == EAGAIN || errno == EWOULDBLOCK) << std::strerror(errno);
  ASSERT_GE(::dup2(pair[0], node.udp_fd()), 0);
  ::close(pair[0]);
  uint64_t would_block = 0;
  // Each Tick to the next unanswered probe's retry, which finds it full.
  for (int i = 0; i < 10 && would_block == 0; ++i) {
    now_us += 20'000;
    node.Tick(now_us);
    const auto latest = status_sub.read();
    if (latest) would_block = latest->socket_send_would_block_count();
  }
  EXPECT_GE(would_block, 1u);
  link.node.reset();  // closes the full end before its peer
  ::close(pair[1]);
}

int pending_recv_error = 0;
int TakeRecvError() { return std::exchange(pending_recv_error, 0); }

// Permanent receive errors close and reopen the socket on the 100 ms cadence
// (incrementing /hw/status socket_reopens_count), while transient errors such
// as ECONNREFUSED keep the socket open.
// REQ-RT-22
TEST(HardwareNodeTest,
     PermanentReceiveErrorReopensSocketWhileTransientKeepsIt) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  ipc::Subscriber<bridge::BridgeStatus> status_sub{
      bridge::topics::kStatus,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST}};

  // 1. Real OS EBADF on TryReceive(): tick with no send due so TryReceive()
  // hits the closed fd directly.
  constexpr uint64_t kStartUs = 1'000'000;
  node.Tick(kStartUs);
  ASSERT_EQ(NextProbeUs(link.gateway), kStartUs);
  const int fd = node.udp_fd();
  ASSERT_GE(fd, 0);
  ASSERT_EQ(::close(fd), 0);

  const uint64_t lost_us = kStartUs + 5'000;
  node.Tick(lost_us);
  EXPECT_EQ(node.udp_fd(), -1);
  EXPECT_EQ(node.socket_counters().recv_failures, 1u);
  EXPECT_EQ(node.socket_counters().last_recv_error, EBADF);
  EXPECT_EQ(node.socket_counters().reopens, 0u);
  // First reopen is scheduled on the next Tick (1 ms later at the latest).
  const uint64_t reopen_us = lost_us + 1'000;
  EXPECT_EQ(node.NextDutyUs(lost_us), reopen_us);
  node.Tick(reopen_us);
  EXPECT_GE(node.udp_fd(), 0);
  EXPECT_EQ(node.socket_counters().reopens, 1u);
  ASSERT_TRUE(InjectState(node, link.gateway,
                          TrustState(NodeConfigId(cfg), 11, 1, 1000),
                          reopen_us + 1'000));

  // Publish /hw/status and verify socket_reopens_count reflects the reopen.
  uint64_t now_us = reopen_us + 25'000;
  node.Tick(now_us);
  auto published = status_sub.read();
  ASSERT_TRUE(published.has_value());
  EXPECT_EQ(published->socket_reopens_count(), 1u);

  // 2. Transient ECONNREFUSED keeps the socket open without reopening.
  node.set_seams({.recv_error = &TakeRecvError});
  pending_recv_error = ECONNREFUSED;
  now_us += 20'000;
  const int fd_before = node.udp_fd();
  node.Tick(now_us);
  EXPECT_EQ(node.udp_fd(), fd_before);
  EXPECT_EQ(node.socket_counters().last_recv_error, ECONNREFUSED);
  EXPECT_EQ(node.socket_counters().reopens, 1u);

  // 3. Permanent ENETUNREACH closes and reopens once kReopenRetryUs has passed
  // since the previous reopen, incrementing socket_reopens_count on /hw/status.
  now_us = reopen_us + HardwareNode::kReopenRetryUs;
  pending_recv_error = ENETUNREACH;
  node.Tick(now_us);
  EXPECT_EQ(node.udp_fd(), -1);
  EXPECT_EQ(node.socket_counters().last_recv_error, ENETUNREACH);
  EXPECT_EQ(node.socket_counters().reopens, 1u);

  now_us += 1'000;
  node.Tick(now_us);
  EXPECT_GE(node.udp_fd(), 0);
  EXPECT_EQ(node.socket_counters().reopens, 2u);

  // 4. Another permanent receive error inside kReopenRetryUs is throttled.
  const uint64_t second_reopen_us = now_us;
  pending_recv_error = EADDRNOTAVAIL;
  node.Tick(second_reopen_us + 5'000);
  EXPECT_EQ(node.udp_fd(), -1);
  node.Tick(second_reopen_us + HardwareNode::kReopenRetryUs - 1);
  EXPECT_EQ(node.udp_fd(), -1);
  EXPECT_EQ(node.socket_counters().reopens, 2u);

  now_us = second_reopen_us + HardwareNode::kReopenRetryUs;
  node.Tick(now_us);
  EXPECT_GE(node.udp_fd(), 0);
  EXPECT_EQ(node.socket_counters().reopens, 3u);

  now_us += 25'000;
  node.Tick(now_us);
  published = status_sub.read();
  ASSERT_TRUE(published.has_value());
  EXPECT_EQ(published->socket_reopens_count(), 3u);
}

// A HardwareNode pinned to an absent local_address (EADDRNOTAVAIL) does not
// crash or fall back to 0.0.0.0: Open() logs one stderr line naming the
// address and errno, enters deferred open, keeps OpenWaits() out of polling
// fallback, and retries every 100 ms until the interface appears. An address
// conflict (EADDRINUSE) still fails hard.
// REQ-RT-22, REQ-OPS-25
TEST(HardwareNodeTest, AbsentLocalAddressRetriesEvery100MsWithoutFallback) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  // RFC 5737 TEST-NET-1: never assigned to a local interface -> EADDRNOTAVAIL.
  constexpr const char* kAbsentIp = "192.0.2.1";
  uint16_t gateway_port = 0;
  uint16_t node_port = 0;
  int gw_res = ReservePort(gateway_port);
  int node_res = ReservePort(node_port);
  ASSERT_GE(gw_res, 0);
  ASSERT_GE(node_res, 0);
  Release(node_res);

  HardwareNode node{cfg, "127.0.0.1", gateway_port, node_port, true, 0,
                    {},  0,           {},           kAbsentIp};
  ::testing::internal::CaptureStderr();
  const bool opened = node.Open();
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  EXPECT_TRUE(opened) << stderr_text;
  EXPECT_NE(stderr_text.find(kAbsentIp), std::string::npos) << stderr_text;
  EXPECT_NE(stderr_text.find(std::to_string(EADDRNOTAVAIL)), std::string::npos)
      << stderr_text;
  EXPECT_EQ(node.udp_fd(), -1)
      << "must not fall back to 0.0.0.0 when local_address is absent";

  // OpenWaits() must succeed without falling back to polling.
  EXPECT_TRUE(node.OpenWaits());
  EXPECT_EQ(node.wakeups().wait_fallbacks, 0u);
  node.CloseWaits();

  // First Tick makes the initial retry attempt; subsequent retries happen on
  // the 100 ms kReopenRetryUs cadence and never before.
  constexpr uint64_t kStartUs = 1'000'000;
  node.Tick(kStartUs);
  EXPECT_EQ(node.udp_fd(), -1);
  EXPECT_EQ(node.socket_counters().reopen_attempts, 1u);
  EXPECT_EQ(node.socket_counters().reopens, 0u);

  node.Tick(kStartUs + 20'000);
  node.Tick(kStartUs + 50'000);
  node.Tick(kStartUs + HardwareNode::kReopenRetryUs - 1);
  EXPECT_EQ(node.socket_counters().reopen_attempts, 1u);

  node.Tick(kStartUs + HardwareNode::kReopenRetryUs);
  EXPECT_EQ(node.udp_fd(), -1);
  EXPECT_EQ(node.socket_counters().reopen_attempts, 2u);

  node.Tick(kStartUs + 2 * HardwareNode::kReopenRetryUs - 1);
  EXPECT_EQ(node.socket_counters().reopen_attempts, 2u);
  node.Tick(kStartUs + 2 * HardwareNode::kReopenRetryUs);
  EXPECT_EQ(node.socket_counters().reopen_attempts, 3u);
  EXPECT_TRUE(node.Status(kStartUs + 2 * HardwareNode::kReopenRetryUs)
                  .alert_error_link_lost());

  // EADDRINUSE on a pinned local address still fails Open() hard.
  int holder = HoldPort(node_port);
  ASSERT_GE(holder, 0);
  HardwareNode conflicting{cfg, "127.0.0.1", gateway_port, node_port,  true, 0,
                           {},  0,           {},           "127.0.0.1"};
  ::testing::internal::CaptureStderr();
  EXPECT_FALSE(conflicting.Open());
  const std::string conflict_err = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(conflict_err.find("127.0.0.1"), std::string::npos) << conflict_err;
  Release(holder);
  Release(gw_res);
}

int AlwaysTransientRecvError() { return ECONNREFUSED; }

// Persistent transient receive errors (such as ECONNREFUSED across all
// kReceiveBudget iterations with zero datagrams consumed) must not leave
// tick_receive_capped_ set and busy-spin Pass() into CPU throttling.
// REQ-RT-20
TEST(HardwareNodeTest, PersistentTransientReceiveErrorsDoNotBusySpinPass) {
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  node.set_seams({.recv_error = &AlwaysTransientRecvError});

  constexpr uint64_t kStartUs = 1'000'000;
  const HardwareNode::Wait wait = node.Pass(kStartUs);
  EXPECT_EQ(node.socket_counters().recv_failures,
            static_cast<uint64_t>(HardwareNode::kReceiveBudget));
  EXPECT_GT(wait.until_us, kStartUs)
      << "transient receive errors with 0 datagrams requested immediate retry";
  EXPECT_FALSE(wait.ignore_input);
  EXPECT_EQ(node.wakeups().throttled, 0u);
}

// RULES.md 3: config pushes executed on the RT bridge loop (initial push,
// retry after enabled refusal, or gateway reboot) must not heap-allocate.
// REQ-RT-06
TEST(HardwareNodeTest, PushConfigDuringTickDoesNotAllocate) {
  if (const auto why = talos::testing::AllocationProbe::Unsupported();
      !why.empty()) {
    GTEST_SKIP() << why;
  }
  auto cfg = config::ParseRobotConfigString(kTrustToml);
  Link link;
  ASSERT_TRUE(link.Open(cfg));
  HardwareNode& node = link.node.value();
  const uint64_t config_id = NodeConfigId(cfg);

  // Warm up the first State/boot stderr log and initial config push before
  // measuring.
  uint64_t now_us = 1'000'000;
  ASSERT_TRUE(InjectState(node, link.gateway,
                          TrustState(config_id + 1, 11, 1, 1'000), now_us));
  // Drain the initial config chunks sent to the gateway.
  protocol::DecodedFrame drained;
  while (link.gateway.TryReceive(&drained) == protocol::UdpStatus::kOk) {
  }

  // Trigger a retry push via a second mismatched-config State past the 50 ms
  // backoff, plus a direct PushConfig() call: zero heap allocations allowed.
  now_us += 100'000;
  std::array<uint8_t, kMaxStateSize> encoded{};
  const std::size_t n =
      Encode(TrustState(config_id + 1, 11, 1, 101'000), encoded);
  ASSERT_GT(n, 0u);
  ASSERT_EQ(link.gateway.SendFrame(protocol::FrameType::kHardwareState, 0,
                                   now_us, encoded.data(), n),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(DatagramQueued(node));

  talos::testing::AllocationProbe probe;
  probe.start();
  node.Tick(now_us);
  node.PushConfig(now_us + 1'000);
  const uint64_t allocations = probe.stop();
  EXPECT_EQ(allocations, 0u) << probe.sites();
}

}  // namespace
}  // namespace talos::hardware

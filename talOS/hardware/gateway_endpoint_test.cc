// The gateway's UDP endpoint over a loopback socket: integration tier
// (docs/TESTING.md §2), kept apart from gateway_test so that target waits on
// no wall clock. Each wait polls a socket with a deadline that only bounds a
// hang (§6.3).

#include <arpa/inet.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <optional>
#include <thread>
#include <vector>

#include "config_wire.h"
#include "endpoint.h"
#include "gateway.h"
#include "gateway_test_support.h"
#include "messages.h"
#include "sim_backend.h"
#include "talOS/protocol/frame.h"
#include "talOS/protocol/types.h"
#include "talOS/protocol/udp.h"

namespace talos::hardware {
namespace {

using namespace gateway_test;  // NOLINT(google-build-using-namespace)

// Polls `done` until it holds. The deadline only bounds a hang
// (docs/TESTING.md §6.3); no test asserts how long a wait took.
template <typename Done>
[[nodiscard]] bool Eventually(Done done) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds{10};
  while (!done()) {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  return true;
}

// The bridge's side of an Endpoint's link, on a raw loopback socket.
// Connect() binds an ephemeral port, opens the endpoint toward it from an
// ephemeral port of its own, and learns that port from a frame the endpoint
// sends, so no test binds a fixed port (docs/TESTING.md §6.4). A raw socket,
// since a RuntimeUdpPeer must know its peer's port before it binds, and sends
// only this build's protocol version.
class Companion {
 public:
  Companion() = default;
  ~Companion() {
    if (fd_ >= 0) ::close(fd_);
  }
  Companion(const Companion&) = delete;
  Companion& operator=(const Companion&) = delete;

  ::testing::AssertionResult Connect(Endpoint& endpoint) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return Failed("socket");
    // As a RuntimeUdpPeer's: room for the States a tick loop piles up.
    const int buffer_bytes = 256 * 1024;
    if (::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &buffer_bytes,
                     sizeof(buffer_bytes)) != 0 ||
        ::fcntl(fd_, F_SETFL, O_NONBLOCK) != 0)
      return Failed("configure socket");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t length = sizeof(address);
    if (::bind(fd_, reinterpret_cast<const sockaddr*>(&address),
               sizeof(address)) != 0 ||
        ::getsockname(fd_, reinterpret_cast<sockaddr*>(&address), &length) != 0)
      return Failed("bind port 0");
    const auto port = ntohs(address.sin_port);
    port_ = port;
    if (endpoint.Open("127.0.0.1", 0, "127.0.0.1", port) !=
        protocol::UdpStatus::kOk)
      return ::testing::AssertionFailure()
             << "Endpoint::Open toward port " << port;
    // The passthrough sends without ticking the gateway.
    const std::array<uint8_t, 1> hello{};
    if (endpoint.SendFrame(protocol::FrameType::kDriverStation, 0, hello.data(),
                           hello.size()) != protocol::UdpStatus::kOk)
      return ::testing::AssertionFailure() << "the endpoint did not send";
    sockaddr_in from{};
    socklen_t from_length = 0;
    const bool heard = Eventually([&] {
      from_length = sizeof(from);
      const auto got =
          ::recvfrom(fd_, rx_.data(), rx_.size(), 0,
                     reinterpret_cast<sockaddr*>(&from), &from_length);
      protocol::DecodedFrame frame;
      return got > 0 &&
             protocol::DecodeFrame(rx_.data(), static_cast<std::size_t>(got),
                                   &frame) == protocol::DecodeStatus::kOk &&
             frame.header.type == protocol::FrameType::kDriverStation;
    });
    if (!heard)
      return ::testing::AssertionFailure() << "no frame from the endpoint";
    if (::connect(fd_, reinterpret_cast<const sockaddr*>(&from), from_length) !=
        0)
      return Failed("connect");
    return ::testing::AssertionSuccess();
  }

  // The endpoint's socket, the one connected to this companion (the Endpoint
  // does not expose it), or -1: for a test to make its sends fail.
  int EndpointFd() const {
    for (int fd = 0; fd < 1024; ++fd) {
      if (fd == fd_) continue;
      sockaddr_in peer{};
      socklen_t length = sizeof(peer);
      int type = 0;
      socklen_t type_length = sizeof(type);
      if (::getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &type_length) == 0 &&
          type == SOCK_DGRAM &&
          ::getpeername(fd, reinterpret_cast<sockaddr*>(&peer), &length) == 0 &&
          peer.sin_family == AF_INET && ntohs(peer.sin_port) == port_)
        return fd;
    }
    return -1;
  }

  // Connects to the endpoint's socket again, after Endpoint::Reopen: opened
  // on port 0, the new socket has a port of its own.
  ::testing::AssertionResult Reconnect() {
    const int endpoint_fd = EndpointFd();
    if (endpoint_fd < 0)
      return ::testing::AssertionFailure() << "no endpoint socket";
    sockaddr_in address{};
    socklen_t length = sizeof(address);
    if (::getsockname(endpoint_fd, reinterpret_cast<sockaddr*>(&address),
                      &length) != 0 ||
        ::connect(fd_, reinterpret_cast<const sockaddr*>(&address), length) !=
            0)
      return Failed("reconnect");
    return ::testing::AssertionSuccess();
  }

  // Frames carry this companion's own sequence, from 1, like a
  // RuntimeUdpPeer's; `version` stamps another protocol version.
  protocol::UdpStatus SendFrame(protocol::FrameType type, uint16_t flags,
                                uint64_t monotonic_time_us, const void* payload,
                                std::size_t payload_size,
                                uint16_t version = protocol::kProtocolVersion) {
    std::array<uint8_t, protocol::kMaxFrameSize> frame{};
    const auto size = protocol::EncodeFrame(
        type, flags, next_send_sequence_, monotonic_time_us, payload,
        payload_size, frame.data(), frame.size());
    if (size == 0) return protocol::UdpStatus::kInvalidArgument;
    frame[8] = static_cast<uint8_t>(version);  // The header's version field.
    frame[9] = static_cast<uint8_t>(version >> 8);
    if (::send(fd_, frame.data(), size, 0) != static_cast<ssize_t>(size))
      return protocol::UdpStatus::kSocketError;
    ++next_send_sequence_;
    return protocol::UdpStatus::kOk;
  }

  protocol::UdpStatus TryReceive(protocol::DecodedFrame* out) {
    const auto got = ::recv(fd_, rx_.data(), rx_.size(), 0);
    if (got < 0)
      return errno == EAGAIN || errno == EWOULDBLOCK
                 ? protocol::UdpStatus::kWouldBlock
                 : protocol::UdpStatus::kSocketError;
    return protocol::DecodeFrame(rx_.data(), static_cast<std::size_t>(got),
                                 out) == protocol::DecodeStatus::kOk
               ? protocol::UdpStatus::kOk
               : protocol::UdpStatus::kDecodeError;
  }

  // Waits for the next frame: a sent datagram need not be readable at once,
  // even on loopback.
  protocol::UdpStatus Receive(protocol::DecodedFrame* out) {
    auto status = protocol::UdpStatus::kWouldBlock;
    (void)Eventually([&] {
      status = TryReceive(out);
      return status != protocol::UdpStatus::kWouldBlock;
    });
    return status;
  }

  // Waits for the next frame of `type`, skipping the others (the State every
  // tick sends).
  bool Receive(protocol::FrameType type, protocol::DecodedFrame* out) {
    while (Receive(out) == protocol::UdpStatus::kOk) {
      if (out->header.type == type) return true;
    }
    return false;
  }

  uint64_t next_send_sequence() const { return next_send_sequence_; }

 private:
  static ::testing::AssertionResult Failed(const char* step) {
    return ::testing::AssertionFailure()
           << step << ": " << std::strerror(errno);
  }

  int fd_{-1};
  uint16_t port_{0};
  uint64_t next_send_sequence_{1};
  std::array<uint8_t, protocol::kMaxFrameSize> rx_{};
};

// Ticks the endpoint and returns the State that tick sent, skipping the
// frames sent before it.
std::optional<State> TickForState(Endpoint& endpoint, Companion& companion,
                                  uint64_t now_us, bool enabled) {
  if (endpoint.Tick(now_us, enabled) != protocol::UdpStatus::kOk)
    return std::nullopt;
  protocol::DecodedFrame frame;
  while (companion.Receive(protocol::FrameType::kHardwareState, &frame)) {
    State state;
    if (!Decode({frame.payload, frame.payload_size}, state))
      return std::nullopt;
    if (state.sample_time_us == now_us) return state;
  }
  return std::nullopt;
}

TEST(Endpoint, ClockProbeEchoUsesLaterSampleWhileDisabledAndUnconfigured) {
  SimBackend backend;
  Gateway gateway{backend, 2003};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  ASSERT_EQ(companion.SendFrame(protocol::FrameType::kClockProbe, 0, 9000,
                                nullptr, 0),
            protocol::UdpStatus::kOk);
  // The State of the tick that receives the probe cannot echo it yet: that
  // tick's sample was taken before its receive.
  const auto expect_state = [&](uint64_t tick, uint64_t echo_us) {
    const auto state = TickForState(endpoint, companion, tick, false);
    ASSERT_TRUE(state.has_value()) << "tick " << tick;
    EXPECT_EQ(state->clock_probe_time_us, echo_us) << "tick " << tick;
    EXPECT_FALSE(state->flags & (kEnabled | kCommandActive));
    EXPECT_EQ(state->counters.rejected_command_count, 0u);
  };
  uint64_t tick = 1000;
  ASSERT_TRUE(Eventually([&] {
    expect_state(tick++, 0);
    return HasFailure() || endpoint.peer_contact().frame_us != 0;
  }));
  ASSERT_FALSE(HasFailure());
  expect_state(6000, 9000);
}

TEST(Endpoint, OutboundBlackoutPreservesReceiveAndSafetyTicks) {
  SimBackend backend;
  Gateway gateway{backend, 2002};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));

  endpoint.SetSendEnabled(false);
  const auto cfg = SwerveConfig(true);
  const auto chunks = CreateConfigChunks(cfg);
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig, i + 1,
                                  1000, chunks[i].data(), chunks[i].size()),
              protocol::UdpStatus::kOk);
  }
  // Receives config, but drops its ACK.
  ASSERT_TRUE(Eventually([&] {
    EXPECT_EQ(endpoint.Tick(2000, false), protocol::UdpStatus::kOk);
    return gateway.configured();
  }));
  ASSERT_EQ(endpoint.Tick(3000, true), protocol::UdpStatus::kOk);
  EXPECT_TRUE(gateway.snapshot().flags & kEnabled);
  Command command = Request(gateway);
  command.motors[0].demand = 5.0;
  std::array<uint8_t, kMaxCommandSize> command_bytes{};
  const auto command_size = Encode(command, command_bytes);
  ASSERT_GT(command_size, 0u);
  ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareCommand, 100,
                                3000, command_bytes.data(), command_size),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(Eventually([&] {
    EXPECT_EQ(endpoint.Tick(4000, true), protocol::UdpStatus::kOk);
    return (gateway.snapshot().flags & kCommandActive) != 0;
  }));
  ASSERT_EQ(endpoint.Tick(5000, true), protocol::UdpStatus::kOk);
  EXPECT_TRUE(gateway.snapshot().flags & kCommandActive);
  EXPECT_DOUBLE_EQ(gateway.snapshot().motors[0].velocity_rps, 5.0);
  ASSERT_EQ(endpoint.Tick(5000 + cfg.command_timeout_us, true),
            protocol::UdpStatus::kOk);
  EXPECT_FALSE(gateway.snapshot().flags & kCommandActive);
  EXPECT_DOUBLE_EQ(gateway.snapshot().motors[0].velocity_rps, 0.0);
  const std::array<uint8_t, 1> ds_payload{0};
  ASSERT_EQ(endpoint.SendFrame(protocol::FrameType::kDriverStation, 3000,
                               ds_payload.data(), ds_payload.size()),
            protocol::UdpStatus::kOk);

  // One sender to one receiver on loopback keeps its order, so the first
  // frame after the blackout shows that the ACK, every State and the Driver
  // Station frame of the blackout were dropped, not queued.
  endpoint.SetSendEnabled(true);
  const uint64_t resumed_at = 6000 + cfg.command_timeout_us;
  ASSERT_EQ(endpoint.Tick(resumed_at, false), protocol::UdpStatus::kOk);
  ASSERT_EQ(endpoint.SendFrame(protocol::FrameType::kDriverStation, resumed_at,
                               ds_payload.data(), ds_payload.size()),
            protocol::UdpStatus::kOk);
  protocol::DecodedFrame frame;
  ASSERT_EQ(companion.Receive(&frame), protocol::UdpStatus::kOk);
  EXPECT_EQ(frame.header.type, protocol::FrameType::kHardwareState);
  State state;
  ASSERT_TRUE(Decode({frame.payload, frame.payload_size}, state));
  EXPECT_EQ(state.sample_time_us, resumed_at);
  EXPECT_FALSE(state.flags & kEnabled);
  ASSERT_EQ(companion.Receive(&frame), protocol::UdpStatus::kOk);
  EXPECT_EQ(frame.header.type, protocol::FrameType::kDriverStation);
  EXPECT_EQ(frame.header.monotonic_time_us, resumed_at);
}

// The RoboRIO program reports to the drive team whether its companion is
// talking, and in which protocol. A clock probe alone (all the bridge sends
// while nothing is commanded) is contact; a frame of another protocol version
// is recorded with that version, not dropped unseen, and is not contact: a
// Command from a bridge one protocol behind (whose PWM requests, for one,
// have another layout) never reaches the gateway.
// REQ-CONTRACT-01
TEST(Endpoint, PeerContactRecordsProbesCommandsAndForeignProtocolVersions) {
  SimBackend backend;
  Gateway gateway{backend, 2004};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  // Ticks at `now_us` until the endpoint has received what was sent.
  const auto tick_until = [&](uint64_t now_us, const uint64_t& heard_us) {
    return Eventually([&] {
      endpoint.Tick(now_us, false);
      return heard_us != 0;
    });
  };
  const auto& contact = endpoint.peer_contact();

  ASSERT_EQ(
      companion.SendFrame(protocol::FrameType::kClockProbe, 0, 0, nullptr, 0),
      protocol::UdpStatus::kOk);
  ASSERT_TRUE(tick_until(1000, contact.frame_us));
  EXPECT_EQ(contact.frame_us, 1000u);
  EXPECT_EQ(contact.command_us, 0u);

  std::array<uint8_t, kMaxCommandSize> command{};
  const auto command_size = Encode(Command{}, command);
  ASSERT_GT(command_size, 0u);
  ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareCommand, 0, 0,
                                command.data(), command_size),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(tick_until(2000, contact.command_us));
  EXPECT_EQ(contact.command_us, 2000u);
  EXPECT_EQ(contact.version_mismatch_us, 0u);

  const auto foreign = static_cast<uint16_t>(protocol::kProtocolVersion - 1);
  ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareCommand, 0, 0,
                                command.data(), command_size, foreign),
            protocol::UdpStatus::kOk);
  ASSERT_TRUE(tick_until(3000, contact.version_mismatch_us));
  EXPECT_EQ(contact.version_mismatch_us, 3000u);
  EXPECT_EQ(contact.mismatched_version, foreign);
  EXPECT_EQ(contact.frame_us, 2000u);
  EXPECT_EQ(contact.command_us, 2000u);
}

TEST(Endpoint, PushesConfigOverUdpAndAcknowledgesIt) {
  SimBackend backend;
  Gateway gateway{backend, 2002};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));

  // 1. Companion sees state with config_id = 0
  auto state = TickForState(endpoint, companion, 1000, true);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->config_id, 0u);
  EXPECT_EQ(state->boot_id, 2002u);

  // 2. Companion pushes config chunks
  auto cfg = SwerveConfig(true);
  auto chunks = CreateConfigChunks(cfg);
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig, i + 1,
                                  1000, chunks[i].data(), chunks[i].size()),
              protocol::UdpStatus::kOk);
  }

  // 3. Endpoint ticks, processes chunks, sends ACK. Hardware configuration
  // requires disabled operation, so the config push is serviced disabled.
  ASSERT_TRUE(Eventually([&] {
    endpoint.Tick(2000, false);
    return gateway.configured();
  }));

  // 4. Companion receives ACK
  protocol::DecodedFrame ack_frame;
  ASSERT_TRUE(
      companion.Receive(protocol::FrameType::kHardwareConfigAck, &ack_frame));
  ConfigAckPayload ack{};
  ASSERT_TRUE(
      DecodeConfigAck({ack_frame.payload, ack_frame.payload_size}, ack));
  EXPECT_EQ(ack.status, ConfigAckStatus::kOk);
  EXPECT_EQ(ack.config_id, ConfigurationId(cfg));

  // 5. Subsequent state from endpoint has config_id set and kConfigured set
  state = TickForState(endpoint, companion, 3000, true);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->config_id, ConfigurationId(cfg));
  EXPECT_TRUE(state->flags & kConfigured);
}

// Pushes `cfg` and ticks until the endpoint acknowledges it or begins a
// backend configure that has not finished (whose ACK comes later): nullopt.
std::optional<ConfigAckPayload> PushConfig(Endpoint& endpoint,
                                           const Gateway& gateway,
                                           Companion& companion,
                                           const Config& cfg,
                                           bool enabled = false) {
  for (const auto& chunk : CreateConfigChunks(cfg)) {
    EXPECT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig,
                                  protocol::kFlagNone, 1000, chunk.data(),
                                  chunk.size()),
              protocol::UdpStatus::kOk);
  }
  std::optional<ConfigAckPayload> ack;
  const bool answered = Eventually([&] {
    endpoint.Tick(2000, enabled);
    protocol::DecodedFrame frame;
    while (!ack && companion.TryReceive(&frame) == protocol::UdpStatus::kOk) {
      if (frame.header.type != protocol::FrameType::kHardwareConfigAck)
        continue;
      ack.emplace();
      EXPECT_TRUE(DecodeConfigAck({frame.payload, frame.payload_size}, *ack));
    }
    return ack.has_value() || gateway.configuring();
  });
  EXPECT_TRUE(answered) << "no ACK, and no configure begun";
  return ack;
}

TEST(Endpoint, FailedConfigurationAcknowledgementBoundsReason) {
  FakeBackend backend;
  backend.configure_ok = false;
  ConfigError err;
  err.device_name = "front_left_drive";
  err.bus = "rio";
  err.can_id = 3;
  err.operation = "apply_talonfx_config";
  err.vendor_status = -1;
  err.message = "timeout waiting for device response; retries exhausted";
  backend.config_error = err;
  Gateway gateway{backend, 3003};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  const auto ack = PushConfig(endpoint, gateway, companion, SwerveConfig(true));
  if (!ack.has_value()) {
    FAIL() << "no config ack";
  }
  EXPECT_EQ(ack->status, ConfigAckStatus::kBackendFailed);
  EXPECT_NE(std::find(std::begin(ack->reason), std::end(ack->reason), '\0'),
            std::end(ack->reason));
  EXPECT_EQ(ack->reason[sizeof(ack->reason) - 1], '\0');
}

// A configuration ACK whose send fails is sent again on a later tick, here
// once the socket that could not send it is replaced, as the RoboRIO program
// replaces it (Endpoint::Reopen). Lost, it would leave the bridge reporting
// the link unconfigured until its next push, at least 50 ms later. The tick
// reports the failed send, so the program learns the socket is lost.
// REQ-CONTRACT-04
TEST(Endpoint, SendsAgainAConfigAckWhoseSendFailed) {
  FakeBackend backend;
  Gateway gateway{backend, 3006};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  const int endpoint_fd = companion.EndpointFd();
  ASSERT_GE(endpoint_fd, 0) << "the endpoint's socket was not found";
  const auto cfg = SwerveConfig(true);
  for (const auto& chunk : CreateConfigChunks(cfg)) {
    ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig,
                                  protocol::kFlagNone, 1000, chunk.data(),
                                  chunk.size()),
              protocol::UdpStatus::kOk);
  }
  // The socket can no longer send (EPIPE): the ACK of the tick that
  // completes the configuration fails, with that tick's State.
  ASSERT_EQ(::shutdown(endpoint_fd, SHUT_WR), 0);
  ASSERT_TRUE(Eventually([&] {
    EXPECT_EQ(endpoint.Tick(2000, false), protocol::UdpStatus::kSocketError);
    return gateway.configured();
  }));
  EXPECT_EQ(endpoint.udp_stats().last_send_error, EPIPE);

  ASSERT_EQ(endpoint.Reopen(), protocol::UdpStatus::kOk);
  ASSERT_TRUE(companion.Reconnect());
  ASSERT_EQ(endpoint.Tick(3000, false), protocol::UdpStatus::kOk);
  protocol::DecodedFrame frame;
  ASSERT_TRUE(
      companion.Receive(protocol::FrameType::kHardwareConfigAck, &frame))
      << "the ACK whose send failed was never sent again";
  ConfigAckPayload ack{};
  ASSERT_TRUE(DecodeConfigAck({frame.payload, frame.payload_size}, ack));
  EXPECT_EQ(ack.status, ConfigAckStatus::kOk);
  EXPECT_EQ(ack.config_id, ConfigurationId(cfg));
  // Sent once it went out: no further copy follows.
  ASSERT_EQ(endpoint.Tick(4000, false), protocol::UdpStatus::kOk);
  while (companion.Receive(&frame) == protocol::UdpStatus::kOk) {
    EXPECT_NE(frame.header.type, protocol::FrameType::kHardwareConfigAck);
    State state;
    if (frame.header.type == protocol::FrameType::kHardwareState &&
        Decode({frame.payload, frame.payload_size}, state) &&
        state.sample_time_us == 4000)
      break;
  }
}

// An ACK still unsent when a newer transfer completes into a pending
// configure answers a superseded push and is never sent: only the pending
// configure's ACK goes out. Here the bridge, hearing no answer to a push
// whose configure failed, pushes the same configuration again; the stale
// failure ACK arriving during that configure would double its push backoff.
// REQ-CONTRACT-04
TEST(Endpoint, DropsAnUnsentAckOnceANewerTransferBeginsConfiguring) {
  class SwitchingBackend : public FakeBackend {
   public:
    void BeginConfigure(const Config& config) override {
      if (!pending) {
        FakeBackend::BeginConfigure(config);
        return;
      }
      started = true;
    }
    ConfigureStatus PollConfigure() override {
      if (!pending) return FakeBackend::PollConfigure();
      return complete ? ConfigureStatus::kDone : ConfigureStatus::kPending;
    }
    bool pending{}, started{}, complete{};
  } backend;
  backend.configure_ok = false;
  Gateway gateway{backend, 3012};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  const int endpoint_fd = companion.EndpointFd();
  ASSERT_GE(endpoint_fd, 0) << "the endpoint's socket was not found";
  const auto cfg = SwerveConfig(true);
  const auto push = [&] {
    for (const auto& chunk : CreateConfigChunks(cfg)) {
      ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig,
                                    protocol::kFlagNone, 1000, chunk.data(),
                                    chunk.size()),
                protocol::UdpStatus::kOk);
    }
  };
  // The first push's configure fails at once; its failure ACK cannot leave.
  ASSERT_EQ(::shutdown(endpoint_fd, SHUT_WR), 0);
  push();
  ASSERT_TRUE(Eventually([&] {
    endpoint.Tick(2000, false);
    return backend.configure_calls == 1;
  }));
  ASSERT_FALSE(gateway.configured());
  // The second begins a configure that stays pending.
  backend.pending = true;
  push();
  ASSERT_TRUE(Eventually([&] {
    endpoint.Tick(3000, false);
    return backend.started;
  }));
  ASSERT_TRUE(gateway.configuring());

  // Sends work again; ticks while the configure is pending send no ACK.
  ASSERT_EQ(endpoint.Reopen(), protocol::UdpStatus::kOk);
  ASSERT_TRUE(companion.Reconnect());
  ASSERT_EQ(endpoint.Tick(4000, false), protocol::UdpStatus::kOk);
  protocol::DecodedFrame frame;
  while (companion.Receive(&frame) == protocol::UdpStatus::kOk) {
    EXPECT_NE(frame.header.type, protocol::FrameType::kHardwareConfigAck)
        << "an ACK went out while the newest push was still configuring";
    State state;
    if (frame.header.type == protocol::FrameType::kHardwareState &&
        Decode({frame.payload, frame.payload_size}, state) &&
        state.sample_time_us == 4000)
      break;
  }
  backend.complete = true;
  ASSERT_EQ(endpoint.Tick(5000, false), protocol::UdpStatus::kOk);
  ASSERT_TRUE(
      companion.Receive(protocol::FrameType::kHardwareConfigAck, &frame));
  ConfigAckPayload ack{};
  ASSERT_TRUE(DecodeConfigAck({frame.payload, frame.payload_size}, ack));
  EXPECT_EQ(ack.status, ConfigAckStatus::kOk);
  EXPECT_EQ(ack.config_id, ConfigurationId(cfg));
}

// The ACK is a frame type, not a flag: its header flags stay zero no matter
// which incoming sequence it answers.
TEST(Endpoint, ConfigAckFlagsIgnoreIncomingSequence) {
  FakeBackend backend;
  Gateway gateway{backend, 3005};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  // Advance the companion's sequence so its low bits are nonzero flag bits if
  // the incoming sequence leaked into the ACK header.
  const std::array<uint8_t, 1> junk{};
  for (int i = 0; i < 0xff; ++i) {
    ASSERT_EQ(companion.SendFrame(protocol::FrameType::kDriverStation,
                                  protocol::kFlagNone, 1000, junk.data(),
                                  junk.size()),
              protocol::UdpStatus::kOk);
    if (i % 16 == 15) endpoint.Tick(1000, false);
  }
  for (const auto& chunk : CreateConfigChunks(SwerveConfig(true))) {
    ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig,
                                  protocol::kFlagNone, 1000, chunk.data(),
                                  chunk.size()),
              protocol::UdpStatus::kOk);
  }
  ASSERT_GT(companion.next_send_sequence(), 0xffu);
  std::optional<uint16_t> ack_flags;
  uint64_t tick = 2000;
  ASSERT_TRUE(Eventually([&] {
    endpoint.Tick(tick++, false);
    protocol::DecodedFrame frame;
    while (companion.TryReceive(&frame) == protocol::UdpStatus::kOk) {
      EXPECT_EQ(frame.header.flags, protocol::kFlagNone);
      if (frame.header.type == protocol::FrameType::kHardwareConfigAck)
        ack_flags = frame.header.flags;
    }
    return ack_flags.has_value();
  }));
  EXPECT_EQ(*ack_flags, protocol::kFlagNone);
}

// REQ-SAFE-21: an identical push succeeds enabled without device traffic
TEST(Endpoint, RestoredGatewayAcknowledgesIdenticalConfigWhileEnabled) {
  Config restored;
  ASSERT_TRUE(
      LoadPersistedConfig(PersistedConfig(SwerveConfig(true)), restored));
  FakeBackend backend;
  Gateway gateway{backend, 3010};
  ASSERT_TRUE(gateway.Reconfigure(restored));
  gateway.Tick(1000, true);
  const auto configure_calls = backend.configure_calls;
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  const auto ack = PushConfig(endpoint, gateway, companion, restored, true);
  if (!ack.has_value()) {
    FAIL() << "Expected configuration acknowledgement";
  }
  const auto& acknowledgement = *ack;
  EXPECT_EQ(acknowledgement.status, ConfigAckStatus::kOk);
  EXPECT_EQ(backend.configure_calls, configure_calls);
  EXPECT_TRUE(gateway.Accept(Request(gateway), 1, 2000));
}

// REQ-SAFE-21: a partial or rejected transfer never replaces the active
// config. After an accepted push, a transfer that stops one chunk short, one
// whose bytes fail its CRC, and a changed config pushed while enabled each
// leave the State's config id, the gateway's config and the devices it
// drives as they were, with no backend configure.
TEST(Endpoint, PartialOrRejectedTransferKeepsTheActiveConfig) {
  FakeBackend backend;
  Gateway gateway{backend, 3013};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  const auto active = SwerveConfig(true);
  const auto accepted = PushConfig(endpoint, gateway, companion, active);
  ASSERT_TRUE(accepted.has_value());
  ASSERT_EQ(accepted->status, ConfigAckStatus::kOk);
  const int configures = backend.configure_calls;

  auto partial = active;
  partial.motors[0].slots[0].p = 1.0;
  auto corrupt = active;
  corrupt.motors[0].slots[0].p = 2.0;
  auto changed = active;
  changed.motors[0].slots[0].p = 3.0;
  uint64_t now_us = 3000;
  // Sends one transfer's chunks and ticks until the endpoint has received
  // every one, so no two transfers' chunks interleave (UDP does not keep
  // order, even on loopback). Order within one transfer cannot change the
  // outcome: the assembler places each chunk by its index
  // (ConfigWire.ReassemblesOutOfOrderIgnoresDuplicatesAndChecksTheCrc).
  const auto deliver = [&](const std::vector<std::vector<uint8_t>>& chunks,
                           bool enabled) {
    const uint64_t received = endpoint.udp_stats().received_frames;
    for (const auto& chunk : chunks) {
      ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareConfig,
                                    protocol::kFlagNone, 1000, chunk.data(),
                                    chunk.size()),
                protocol::UdpStatus::kOk);
    }
    ASSERT_TRUE(Eventually([&] {
      endpoint.Tick(now_us, enabled);
      now_us += 1000;
      return endpoint.udp_stats().received_frames >= received + chunks.size();
    }));
  };
  // Ticks until the endpoint's answer to the newest transfer arrives.
  const auto await_ack = [&](bool enabled) {
    std::optional<ConfigAckPayload> ack;
    EXPECT_TRUE(Eventually([&] {
      protocol::DecodedFrame frame;
      while (!ack && companion.TryReceive(&frame) == protocol::UdpStatus::kOk) {
        if (frame.header.type != protocol::FrameType::kHardwareConfigAck)
          continue;
        ack.emplace();
        EXPECT_TRUE(DecodeConfigAck({frame.payload, frame.payload_size}, *ack));
      }
      if (ack) return true;
      endpoint.Tick(now_us, enabled);
      now_us += 1000;
      return false;
    }));
    return ack;
  };
  const auto expect_active = [&](bool enabled) {
    const auto state = TickForState(endpoint, companion, now_us, enabled);
    now_us += 1000;
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->config_id, ConfigurationId(active));
    EXPECT_TRUE(state->flags & kConfigured);
    EXPECT_EQ(std::size_t{state->motor_count}, active.motors.size());
    EXPECT_EQ(std::size_t{state->sensor_count}, active.sensors.size());
    EXPECT_EQ(gateway.config(), active);
    EXPECT_EQ(backend.configure_calls, configures);
  };

  {
    SCOPED_TRACE("a transfer one chunk short");
    auto chunks = CreateConfigChunks(partial, 200);
    ASSERT_GT(chunks.size(), 1u);
    chunks.pop_back();
    deliver(chunks, false);
    expect_active(false);
  }
  {
    SCOPED_TRACE("a transfer failing its CRC");
    auto chunks = CreateConfigChunks(corrupt, 200);
    chunks.front().back() ^= 0xff;  // data; the header stays whole
    deliver(chunks, false);
    const auto ack = await_ack(false);
    ASSERT_TRUE(ack.has_value());
    EXPECT_EQ(ack->status, ConfigAckStatus::kInvalidCrc);
    expect_active(false);
  }
  {
    SCOPED_TRACE("a changed config pushed while enabled");
    expect_active(true);
    deliver(CreateConfigChunks(changed), true);
    const auto ack = await_ack(true);
    ASSERT_TRUE(ack.has_value());
    EXPECT_NE(ack->status, ConfigAckStatus::kOk);
    expect_active(true);
  }
  // Still the active config's devices: a command built for it drives.
  EXPECT_TRUE(gateway.Accept(Request(gateway), 1, now_us));
}

TEST(Endpoint, PendingDegradedConfigureDoesNotBlockTicksAndRecoversDevice) {
  class PendingBackend : public FakeBackend {
   public:
    void BeginConfigure(const Config&) override { started = true; }
    ConfigureStatus PollConfigure() override {
      return complete ? ConfigureStatus::kDone : ConfigureStatus::kPending;
    }
    bool started{}, complete{};
  } backend;
  const auto config = SwerveConfig(true);
  backend.unavailable = {config.motors[0].id};
  Gateway gateway{backend, 3011};
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  EXPECT_FALSE(PushConfig(endpoint, gateway, companion, config).has_value());
  EXPECT_TRUE(backend.started);
  EXPECT_EQ(backend.configure_calls, 0);
  EXPECT_TRUE(gateway.configuring());
  endpoint.Tick(7000, true);
  EXPECT_EQ(gateway.snapshot().sample_time_us, 7000u);
  EXPECT_FALSE(gateway.Accept(Request(gateway), 1, 7000));
  backend.complete = true;
  endpoint.Tick(12000, true);
  EXPECT_TRUE(gateway.configured());
  EXPECT_FALSE(gateway.snapshot().motors[0].valid);
  // The first ACK is the completed configure's: none went out while it was
  // pending.
  protocol::DecodedFrame frame;
  ASSERT_TRUE(
      companion.Receive(protocol::FrameType::kHardwareConfigAck, &frame));
  ConfigAckPayload ack{};
  ASSERT_TRUE(DecodeConfigAck({frame.payload, frame.payload_size}, ack));
  EXPECT_EQ(ack.status, ConfigAckStatus::kOk);
  auto command = Request(gateway);
  for (std::size_t i = 0; i < command.count; ++i)
    command.motors[i] = {config.motors[i].id, Mode::kDutyCycle, 0, 0.2};
  ASSERT_TRUE(gateway.Accept(command, 2, 12000));
  EXPECT_EQ(backend.last_requests[0].mode, Mode::kNeutral);
  EXPECT_EQ(backend.last_requests[1].mode, Mode::kDutyCycle);
  endpoint.Tick(262000, true);
  endpoint.Tick(267000, true);  // Restored, in a State read before it was.
  endpoint.Tick(272000, true);
  command.observed_time_us = 272000;
  command.epoch = gateway.snapshot().epoch;
  ASSERT_TRUE(gateway.Accept(command, 3, 272000));
  EXPECT_EQ(backend.last_requests[0].mode, Mode::kDutyCycle);
}

// After a stall the socket holds a command backlog; replaying it would put
// every superseded demand on the CAN bus back-to-back.
// REQ-SAFE-17: only the newest command a tick drains is applied.
TEST(Endpoint, AppliesOnlyTheNewestQueuedCommand) {
  FakeBackend backend;
  Gateway gateway{backend, 3012};
  ASSERT_TRUE(gateway.Reconfigure(SwerveConfig(true)));
  gateway.Tick(1000, true);
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  auto command = Request(gateway);
  // Each command's demand is its sequence, so the applied demand names the
  // command it came from.
  const auto send_backlog = [&] {
    for (int i = 0; i < 5; ++i) {
      command.motors[0].demand =
          static_cast<double>(companion.next_send_sequence());
      std::array<uint8_t, kMaxCommandSize> bytes{};
      const auto size = Encode(command, bytes);
      ASSERT_GT(size, 0u);
      ASSERT_EQ(
          companion.SendFrame(protocol::FrameType::kHardwareCommand,
                              protocol::kFlagNone, 1000, bytes.data(), size),
          protocol::UdpStatus::kOk);
    }
  };
  send_backlog();
  // Tick until one tick drains at least two queued commands: datagrams may
  // become readable one at a time, and a tick that drains a single command
  // proves nothing here. Should they all trickle in, queue another backlog.
  ASSERT_TRUE(Eventually([&] {
    const auto sequence_before = gateway.snapshot().last_command_sequence;
    const int applied_before = backend.applied;
    EXPECT_EQ(endpoint.Tick(2000, true), protocol::UdpStatus::kOk);
    const auto sequence = gateway.snapshot().last_command_sequence;
    if (sequence - sequence_before >= 2) {
      EXPECT_EQ(backend.applied - applied_before, 1)
          << "commands " << sequence_before + 1 << " to " << sequence
          << " were drained by one tick";
      EXPECT_FALSE(backend.last_requests.empty());
      if (!backend.last_requests.empty()) {
        EXPECT_DOUBLE_EQ(backend.last_requests[0].demand,
                         static_cast<double>(sequence));
      }
      return true;
    }
    if (sequence == companion.next_send_sequence() - 1) send_backlog();
    return HasFailure();
  }));
  EXPECT_EQ(gateway.snapshot().counters.rejected_command_count, 0u);
}

// The RoboRIO's latency line ages each drained command's observation on the
// RoboRIO's own clock, accepted and refused apart, so the endpoint reports
// the newest command a Tick decoded, what it observed and whether the gateway
// took it; a Tick that decoded none reports none.
// REQ-RT-25
TEST(Endpoint, ReportsTheNewestDrainedCommandAndWhetherItWasAccepted) {
  SimBackend backend;
  Gateway gateway{backend, 3013};
  ASSERT_TRUE(gateway.Reconfigure(SwerveConfig(true)));
  Endpoint endpoint{gateway};
  Companion companion;
  ASSERT_TRUE(companion.Connect(endpoint));
  ASSERT_EQ(endpoint.Tick(5000, true), protocol::UdpStatus::kOk);
  EXPECT_FALSE(endpoint.drained().present);
  auto command = Request(gateway);
  ASSERT_EQ(command.observed_time_us, 5000u);
  const auto send = [&](uint64_t observed_time_us) {
    command.observed_time_us = observed_time_us;
    std::array<uint8_t, kMaxCommandSize> bytes{};
    const auto size = Encode(command, bytes);
    ASSERT_GT(size, 0u);
    ASSERT_EQ(companion.SendFrame(protocol::FrameType::kHardwareCommand,
                                  protocol::kFlagNone, observed_time_us,
                                  bytes.data(), size),
              protocol::UdpStatus::kOk);
  };
  // Ticks at `now_us` until a Tick decodes a command.
  const auto drain = [&](uint64_t now_us, bool enabled) {
    return Eventually([&] {
      EXPECT_EQ(endpoint.Tick(now_us, enabled), protocol::UdpStatus::kOk);
      return endpoint.drained().present;
    });
  };

  // Observed at 5000, drained by the tick at 10000 and taken.
  send(5000);
  ASSERT_TRUE(drain(10000, true));
  EXPECT_TRUE(endpoint.drained().accepted);
  EXPECT_EQ(endpoint.drained().observed_time_us, 5000u);
  EXPECT_EQ(gateway.snapshot().last_command_sequence, 1u);
  // Nothing sent, nothing decoded.
  ASSERT_EQ(endpoint.Tick(15000, true), protocol::UdpStatus::kOk);
  EXPECT_FALSE(endpoint.drained().present);

  // Disabled, the gateway refuses it; the endpoint still reports it.
  send(15000);
  ASSERT_TRUE(drain(20000, false));
  EXPECT_FALSE(endpoint.drained().accepted);
  EXPECT_EQ(endpoint.drained().observed_time_us, 15000u);

  // Of a backlog drained by one tick, the newest is reported. Each command's
  // observation is 30000 plus its sequence, so the report names the command.
  ASSERT_EQ(endpoint.Tick(40000, true), protocol::UdpStatus::kOk);
  command = Request(gateway);
  const auto send_backlog = [&] {
    for (int i = 0; i < 5; ++i) send(30000 + companion.next_send_sequence());
  };
  send_backlog();
  ASSERT_TRUE(Eventually([&] {
    const auto sequence_before = gateway.snapshot().last_command_sequence;
    EXPECT_EQ(endpoint.Tick(40000, true), protocol::UdpStatus::kOk);
    const auto sequence = gateway.snapshot().last_command_sequence;
    if (endpoint.drained().present) {
      EXPECT_TRUE(endpoint.drained().accepted) << "sequence " << sequence;
      EXPECT_EQ(endpoint.drained().observed_time_us, 30000 + sequence);
    }
    if (sequence - sequence_before >= 2) return true;
    if (sequence == companion.next_send_sequence() - 1) send_backlog();
    return HasFailure();
  }));
}
}  // namespace
}  // namespace talos::hardware

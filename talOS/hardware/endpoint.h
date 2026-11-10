#pragma once

#include <array>
#include <cstdio>
#include <functional>
#include <string>

#include "../protocol/udp.h"
#include "config_wire.h"
#include "gateway.h"

namespace talos::hardware {
// When an Endpoint last heard its peer, each the now_us of the Tick that
// received it (0 before any): any valid frame (the bridge probes the clock
// every 20 ms even with nothing to command), a command frame, and a frame of
// another protocol version, with that version. Nothing in the gateway reads
// these; the RoboRIO program reports them to the drive team.
struct PeerContact {
  uint64_t frame_us{};
  uint64_t command_us{};
  uint64_t version_mismatch_us{};
  uint16_t mismatched_version{};
};

// The same endpoint runs under WPILib on RoboRIO and under a future platform
// adapter on SystemCore. Each Tick has a bounded receive budget.
class Endpoint {
 public:
  explicit Endpoint(Gateway& gateway) : gateway_{gateway} {}
  void OnConfigured(std::function<void(const Config&)> callback) {
    on_configured_ = std::move(callback);
  }
  protocol::UdpStatus Open(const char* local, uint16_t local_port,
                           const char* remote, uint16_t remote_port) {
    return peer_.Open(local, local_port, remote, remote_port);
  }
  // Link fault injection drops outbound traffic, never inbound commands or
  // gateway safety ticks. Suppressed datagrams are not queued for recovery.
  void SetSendEnabled(bool enabled) { send_enabled_ = enabled; }
  protocol::UdpStatus Tick(uint64_t now_us, bool enabled) {
    gateway_.Tick(now_us, enabled);
    // Only echo probes received on a previous tick: now_us was sampled before
    // this tick's receive loop, so same-tick echo would violate causality.
    const auto clock_probe_time_us = pending_probe_time_us_;
    if (pending_ack_ && !gateway_.configuring()) {
      ConfigAckPayload ack{};
      ack.config_id = pending_config_id_;
      ack.status = gateway_.configured() &&
                           gateway_.snapshot().config_id == pending_config_id_
                       ? ConfigAckStatus::kOk
                       : ConfigAckStatus::kBackendFailed;
      if (ack.status != ConfigAckStatus::kOk) {
        const auto error = gateway_.config_error();
        if (error)
          std::snprintf(ack.reason, sizeof(ack.reason), "%s: %s",
                        error->device_name.c_str(), error->message.c_str());
      }
      if (ack.status == ConfigAckStatus::kOk && on_configured_)
        on_configured_(gateway_.config());
      std::array<uint8_t, 128> bytes{};
      const auto size = EncodeConfigAck(ack, bytes);
      Send(protocol::FrameType::kHardwareConfigAck, now_us, bytes.data(), size);
      pending_ack_ = false;
    }
    // Only the newest command of the drained batch reaches the gateway: after
    // a stall the socket holds a backlog, and applying each queued command
    // would replay superseded demands back-to-back as CAN bursts.
    bool have_command = false;
    uint64_t command_sequence = 0;
    Command newest;
    for (int i = 0; i < kReceiveBudget; ++i) {
      protocol::DecodedFrame frame;
      const auto result = peer_.TryReceive(&frame);
      if (result == protocol::UdpStatus::kWouldBlock) break;
      if (result == protocol::UdpStatus::kDecodeError &&
          peer_.stats().last_decode_status ==
              protocol::DecodeStatus::kUnsupportedVersion) {
        contact_.version_mismatch_us = now_us;
        contact_.mismatched_version = peer_.stats().last_unsupported_version;
        continue;
      }
      if (result != protocol::UdpStatus::kOk) continue;
      contact_.frame_us = now_us;

      if (frame.header.type == protocol::FrameType::kClockProbe &&
          frame.payload_size == 0) {
        pending_probe_time_us_ = frame.header.monotonic_time_us;
        continue;
      }
      if (frame.header.type == protocol::FrameType::kHardwareConfig) {
        std::string err;
        auto status =
            assembler_.AddChunk({frame.payload, frame.payload_size}, err);
        if (status == ConfigAssembler::Status::kComplete) {
          if (gateway_.configuring()) continue;
          bool ok = gateway_.BeginReconfigure(assembler_.config());
          // Bounded backends can complete immediately; Phoenix only polls its
          // worker here and returns without waiting for CAN transactions.
          if (ok && gateway_.configuring()) gateway_.Tick(now_us, enabled);
          if (ok && !gateway_.configuring())
            ok = gateway_.configured() &&
                 gateway_.snapshot().config_id == assembler_.config_id();
          if (ok && gateway_.configuring()) {
            pending_ack_ = true;
            pending_config_id_ = assembler_.config_id();
            continue;
          }
          ConfigAckPayload ack{};
          ack.config_id = assembler_.config_id();
          if (ok) {
            if (on_configured_) on_configured_(gateway_.config());
            ack.status = ConfigAckStatus::kOk;
          } else {
            ack.status = ConfigAckStatus::kBackendFailed;
            // Report the failing device and cause instead of the
            // generic string. Compact "device: cause" summary so both survive
            // truncation to the 64-byte reason; snprintf always
            // NUL-terminates.
            const auto config_error = gateway_.config_error();
            if (config_error.has_value()) {
              std::snprintf(ack.reason, sizeof(ack.reason), "%s: %s",
                            config_error->device_name.c_str(),
                            config_error->message.c_str());
            } else {
              std::snprintf(ack.reason, sizeof(ack.reason),
                            "Backend configuration failed");
            }
          }
          std::array<uint8_t, 128> ack_buf{};
          const auto ack_size = EncodeConfigAck(ack, ack_buf);
          Send(protocol::FrameType::kHardwareConfigAck, now_us, ack_buf.data(),
               ack_size);
        } else if (status != ConfigAssembler::Status::kIncomplete) {
          ConfigAckPayload ack{};
          ack.config_id = assembler_.config_id();
          if (status == ConfigAssembler::Status::kCeilingExceeded) {
            ack.status = ConfigAckStatus::kCeilingExceeded;
          } else if (status == ConfigAssembler::Status::kCrcMismatch) {
            ack.status = ConfigAckStatus::kInvalidCrc;
          } else {
            ack.status = ConfigAckStatus::kMalformed;
          }
          std::snprintf(ack.reason, sizeof(ack.reason), "%s", err.c_str());
          std::array<uint8_t, 128> ack_buf{};
          const auto ack_size = EncodeConfigAck(ack, ack_buf);
          Send(protocol::FrameType::kHardwareConfigAck, now_us, ack_buf.data(),
               ack_size);
        }
        continue;
      }

      if (frame.header.type != protocol::FrameType::kHardwareCommand) continue;
      if (have_command && frame.header.sequence <= command_sequence) continue;
      Command command;
      if (Decode({frame.payload, frame.payload_size}, command)) {
        contact_.command_us = now_us;
        newest = command;
        have_command = true;
        command_sequence = frame.header.sequence;
      }
    }
    if (have_command) gateway_.Accept(newest, command_sequence, now_us);
    auto state = gateway_.snapshot();
    state.clock_probe_time_us = clock_probe_time_us;
    const auto size = Encode(state, buffer_);
    return Send(protocol::FrameType::kHardwareState, now_us, buffer_.data(),
                size);
  }
  const PeerContact& peer_contact() const { return contact_; }
  // Payload-agnostic passthrough for frames this endpoint does not own, so
  // their encoding stays with the caller rather than in the gateway layer.
  protocol::UdpStatus SendFrame(protocol::FrameType type, uint64_t now_us,
                                const void* payload, std::size_t payload_size) {
    return Send(type, now_us, payload, payload_size);
  }

 private:
  // The UDP peer assigns outgoing sequences; ACK is a frame type, so every
  // frame goes out with no header flags.
  protocol::UdpStatus Send(protocol::FrameType type, uint64_t now_us,
                           const void* payload, std::size_t payload_size) {
    if (!send_enabled_) return protocol::UdpStatus::kOk;
    return peer_.SendFrame(type, protocol::kFlagNone, now_us, payload,
                           payload_size);
  }
  // Frames drained per Tick. Receiving only decodes and the gateway applies at
  // most one command per Tick, so a post-stall backlog drains in a few Ticks
  // with one CAN write pass each.
  static constexpr int kReceiveBudget = 64;
  bool send_enabled_{true};
  PeerContact contact_;
  uint64_t pending_probe_time_us_{};
  Gateway& gateway_;
  protocol::RuntimeUdpPeer peer_;
  ConfigAssembler assembler_;
  bool pending_ack_{false};
  uint64_t pending_config_id_{};
  std::function<void(const Config&)> on_configured_;
  std::array<uint8_t, protocol::kMaxPayloadSize> buffer_{};
};
}  // namespace talos::hardware

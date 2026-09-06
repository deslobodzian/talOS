#pragma once

#include <array>
#include <cstdio>
#include <string>

#include "../protocol/udp.h"
#include "config_wire.h"
#include "gateway.h"

namespace talos::hardware {
// The same endpoint runs under WPILib on RoboRIO and under a future platform
// adapter on SystemCore. Each Tick has a bounded receive budget.
class Endpoint {
 public:
  explicit Endpoint(Gateway& gateway) : gateway_{gateway} {}
  protocol::UdpStatus Open(const char* local, uint16_t local_port,
                           const char* remote, uint16_t remote_port) {
    return peer_.Open(local, local_port, remote, remote_port);
  }
  protocol::UdpStatus Tick(uint64_t now_us, bool enabled) {
    gateway_.Tick(now_us, enabled);
    for (int i = 0; i < 16; ++i) {
      protocol::DecodedFrame frame;
      const auto result = peer_.TryReceive(&frame);
      if (result == protocol::UdpStatus::kWouldBlock) break;
      if (result != protocol::UdpStatus::kOk) continue;

      if (frame.header.type == protocol::FrameType::kHardwareConfig) {
        std::string err;
        auto status = assembler_.AddChunk(
            {frame.payload, frame.payload_size}, err);
        if (status == ConfigAssembler::Status::kComplete) {
          bool ok = gateway_.Reconfigure(assembler_.config());
          ConfigAckPayload ack{};
          ack.config_id = assembler_.config_id();
          if (ok) {
            ack.status = ConfigAckStatus::kOk;
          } else {
            ack.status = ConfigAckStatus::kBackendFailed;
            std::snprintf(ack.reason, sizeof(ack.reason), "Backend configuration failed");
          }
          std::array<uint8_t, 128> ack_buf{};
          const auto ack_size = EncodeConfigAck(ack, ack_buf);
          peer_.SendFrame(protocol::FrameType::kHardwareConfigAck,
                          frame.header.sequence, now_us, ack_buf.data(), ack_size);
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
          peer_.SendFrame(protocol::FrameType::kHardwareConfigAck,
                          frame.header.sequence, now_us, ack_buf.data(), ack_size);
        }
        continue;
      }

      if (frame.header.type != protocol::FrameType::kHardwareCommand) continue;
      Command command;
      if (Decode({frame.payload, frame.payload_size}, command))
        gateway_.Accept(command, frame.header.sequence, now_us);
    }
    const auto size = Encode(gateway_.snapshot(), buffer_);
    return peer_.SendFrame(protocol::FrameType::kHardwareState, 0, now_us,
                           buffer_.data(), size);
  }

 private:
  Gateway& gateway_;
  protocol::RuntimeUdpPeer peer_;
  ConfigAssembler assembler_;
  std::array<uint8_t, protocol::kMaxPayloadSize> buffer_{};
};
}  // namespace talos::hardware

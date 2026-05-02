#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "talos/protocol/frame.h"

namespace talos::protocol {

enum class UdpStatus : uint8_t {
  kOk = 0,
  kWouldBlock,
  kInvalidArgument,
  kSocketError,
  kDecodeError,
};

struct UdpSocketOptions {
  bool non_blocking = true;
  bool reuse_address = true;
  bool connect_peer = true;
  int receive_buffer_bytes = 256 * 1024;
  int send_buffer_bytes = 256 * 1024;
  uint8_t ip_tos = 0xB8;  // DSCP EF: low latency on networks that honor it.
};

struct UdpStats {
  uint64_t sent_frames = 0;
  uint64_t sent_bytes = 0;
  uint64_t received_frames = 0;
  uint64_t received_bytes = 0;
  uint64_t receive_would_block = 0;
  uint64_t decode_failures = 0;
  uint64_t sequence_gaps = 0;
  uint64_t last_received_sequence = 0;
  DecodeStatus last_decode_status = DecodeStatus::kOk;
};

// Connected UDP runtime peer for the Mac<->Rio hot path.
//
// The class owns fixed TX/RX datagram buffers. SendFrame() encodes into the TX
// buffer and sends immediately. TryReceive() decodes in place from the RX buffer;
// the returned DecodedFrame payload pointer stays valid until the next receive
// call on this peer.
class RuntimeUdpPeer {
 public:
  RuntimeUdpPeer();
  ~RuntimeUdpPeer();

  RuntimeUdpPeer(const RuntimeUdpPeer&) = delete;
  RuntimeUdpPeer& operator=(const RuntimeUdpPeer&) = delete;

  RuntimeUdpPeer(RuntimeUdpPeer&& other) noexcept;
  RuntimeUdpPeer& operator=(RuntimeUdpPeer&& other) noexcept;

  UdpStatus Open(const char* local_address, uint16_t local_port,
                 const char* remote_address, uint16_t remote_port,
                 const UdpSocketOptions& options = {});

  void Close();

  bool IsOpen() const;

  UdpStatus SendFrame(FrameType type, uint16_t flags,
                      uint64_t monotonic_time_us, const void* payload,
                      std::size_t payload_size);

  uint8_t* MutablePayloadBuffer(std::size_t payload_size);

  UdpStatus SendBufferedFrame(FrameType type, uint16_t flags,
                              uint64_t monotonic_time_us,
                              std::size_t payload_size);

  UdpStatus TryReceive(DecodedFrame* out);

  const UdpStats& stats() const { return stats_; }
  uint64_t next_send_sequence() const { return next_send_sequence_; }

 private:
  UdpStatus ConfigureSocket(const UdpSocketOptions& options);

  uintptr_t socket_;
  uint64_t next_send_sequence_;
  UdpStats stats_;
  std::array<uint8_t, kMaxFrameSize> tx_buffer_;
  std::array<uint8_t, kMaxFrameSize> rx_buffer_;
};

const char* UdpStatusName(UdpStatus status);

}  // namespace talos::protocol

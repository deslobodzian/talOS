#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "frame.h"

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
  // SO_REUSEADDR. Off, so a second process cannot bind a port a live peer
  // holds (on Linux it would then share the port's datagrams with it): a
  // second hardware bridge fails to open instead. A peer killed and restarted
  // rebinds without it, since UDP leaves no TIME_WAIT behind.
  bool reuse_address = false;
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
  // The header version of the last kUnsupportedVersion frame; 0 before one.
  uint16_t last_unsupported_version = 0;
  // The OS error (errno; WSAGetLastError() on Windows) of the last send that
  // returned kSocketError from an open socket; 0 before one, or when that
  // send was short with no error.
  int last_send_error = 0;
};

// Connected UDP runtime peer for the Mac<->Rio hot path.
//
// The class owns fixed TX/RX datagram buffers. SendFrame() encodes into the TX
// buffer and sends immediately. TryReceive() decodes in place from the RX
// buffer; the returned DecodedFrame payload pointer stays valid until the next
// receive call on this peer.
//
// A send never raises SIGPIPE: the socket is opened with SO_NOSIGPIPE where
// the OS has it (macOS) and every send passes MSG_NOSIGNAL where it is
// defined (Linux), so a socket that can no longer send (EPIPE) fails the
// send with kSocketError and stats().last_send_error instead of killing the
// process.
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

  // Closes the socket and opens a new one as the last successful Open()
  // asked (same addresses, ports and options), keeping the send sequence and
  // the stats: the remote's replay gate (Gateway::Accept) still takes the
  // frames that follow. kInvalidArgument when Open() never succeeded; on any
  // other failure the peer is left closed.
  UdpStatus Reopen();

  void Close();

  bool IsOpen() const;

  // The socket's file descriptor, for a poller to wait on until a datagram
  // is readable (talos::event::Poller::add_fd); -1 while closed, and always
  // -1 on Windows, whose SOCKET is no fd. The peer keeps ownership.
  int NativeFd() const;

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
  // Creates, configures, binds and connects socket_ from the addresses and
  // options Open() stored; closes it again on failure.
  UdpStatus OpenSocket();
  UdpStatus ConfigureSocket(const UdpSocketOptions& options);
  UdpStatus Send(std::size_t frame_size);

  uintptr_t socket_;
  uint64_t next_send_sequence_;
  UdpStats stats_;
  std::array<uint8_t, kMaxFrameSize> tx_buffer_;
  std::array<uint8_t, kMaxFrameSize> rx_buffer_;
  // What the last successful Open() asked for, for Reopen(): IPv4 addresses
  // and ports in network byte order.
  bool opened_{false};
  uint32_t local_address_{0};
  uint16_t local_port_{0};
  uint32_t remote_address_{0};
  uint16_t remote_port_{0};
  UdpSocketOptions options_{};
};

// Whether a send that failed with OS error `error` (UdpStats::last_send_error)
// left a socket that will never send again, whatever the network does, so
// only a new one can (Reopen): EPIPE, ENOTCONN, EDESTADDRREQ, EADDRNOTAVAIL
// and EBADF (on Windows their WSA equivalents: WSAESHUTDOWN, WSAENOTCONN,
// WSAEDESTADDRREQ, WSAEADDRNOTAVAIL, WSAENOTSOCK). Any other error clears by
// itself on the same socket. Both ends' callers (the bridge's HardwareNode,
// the RoboRIO program) decide with this one function.
bool SendErrorNeedsNewSocket(int error);

const char* UdpStatusName(UdpStatus status);

}  // namespace talos::protocol

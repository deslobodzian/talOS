#include "udp.h"

#include <cstring>
#include <utility>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace talos::protocol {
namespace {

#if defined(_WIN32)
constexpr uintptr_t kInvalidSocket = static_cast<uintptr_t>(INVALID_SOCKET);
using NativeSocket = SOCKET;

bool EnsureSocketRuntime() {
  static const bool initialized = [] {
    WSADATA data;
    return WSAStartup(MAKEWORD(2, 2), &data) == 0;
  }();
  return initialized;
}

int LastSocketError() { return WSAGetLastError(); }

bool IsWouldBlockError(int error) {
  return error == WSAEWOULDBLOCK || error == WSAEINPROGRESS;
}

void CloseSocket(uintptr_t socket_handle) {
  closesocket(static_cast<NativeSocket>(socket_handle));
}

bool SetNonBlocking(uintptr_t socket_handle) {
  u_long enabled = 1;
  return ioctlsocket(static_cast<NativeSocket>(socket_handle), FIONBIO,
                     &enabled) == 0;
}
#else
constexpr uintptr_t kInvalidSocket = static_cast<uintptr_t>(-1);
using NativeSocket = int;

bool EnsureSocketRuntime() { return true; }

int LastSocketError() { return errno; }

bool IsWouldBlockError(int error) {
  return error == EAGAIN || error == EWOULDBLOCK || error == EINPROGRESS;
}

void CloseSocket(uintptr_t socket_handle) {
  close(static_cast<NativeSocket>(socket_handle));
}

bool SetNonBlocking(uintptr_t socket_handle) {
  const NativeSocket fd = static_cast<NativeSocket>(socket_handle);
  const int flags = fcntl(fd, F_GETFL, 0);
  return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}
#endif

// A send that fails returns its error, never SIGPIPE: MSG_NOSIGNAL on every
// send where the SDK defines it (Linux, and macOS SDKs that define it, as
// macOS 26.3's does), and SO_NOSIGPIPE on the socket where the OS has it
// (macOS; ConfigureSocket), which alone covers an SDK without MSG_NOSIGNAL.
// Either guard suffices; talOS/rtms/wake.cc does the same. Windows raises no
// signal.
#if defined(MSG_NOSIGNAL)
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;
#endif

NativeSocket ToNativeSocket(uintptr_t socket_handle) {
  return static_cast<NativeSocket>(socket_handle);
}

sockaddr_in MakeAddress(const char* address, uint16_t port, bool* ok) {
  sockaddr_in out{};
  out.sin_family = AF_INET;
  out.sin_port = htons(port);
  if (address == nullptr || std::strcmp(address, "0.0.0.0") == 0 ||
      std::strcmp(address, "") == 0) {
    out.sin_addr.s_addr = htonl(INADDR_ANY);
    *ok = true;
    return out;
  }
  *ok = inet_pton(AF_INET, address, &out.sin_addr) == 1;
  return out;
}

}  // namespace

RuntimeUdpPeer::RuntimeUdpPeer()
    : socket_(kInvalidSocket), next_send_sequence_(1), stats_{} {}

RuntimeUdpPeer::~RuntimeUdpPeer() { Close(); }

RuntimeUdpPeer::RuntimeUdpPeer(RuntimeUdpPeer&& other) noexcept
    : socket_(std::exchange(other.socket_, kInvalidSocket)),
      next_send_sequence_(other.next_send_sequence_),
      stats_(other.stats_),
      tx_buffer_(other.tx_buffer_),
      rx_buffer_(other.rx_buffer_),
      opened_(other.opened_),
      local_address_(other.local_address_),
      local_port_(other.local_port_),
      remote_address_(other.remote_address_),
      remote_port_(other.remote_port_),
      options_(other.options_) {}

RuntimeUdpPeer& RuntimeUdpPeer::operator=(RuntimeUdpPeer&& other) noexcept {
  if (this != &other) {
    Close();
    socket_ = std::exchange(other.socket_, kInvalidSocket);
    next_send_sequence_ = other.next_send_sequence_;
    stats_ = other.stats_;
    tx_buffer_ = other.tx_buffer_;
    rx_buffer_ = other.rx_buffer_;
    opened_ = other.opened_;
    local_address_ = other.local_address_;
    local_port_ = other.local_port_;
    remote_address_ = other.remote_address_;
    remote_port_ = other.remote_port_;
    options_ = other.options_;
  }
  return *this;
}

UdpStatus RuntimeUdpPeer::Open(const char* local_address, uint16_t local_port,
                               const char* remote_address, uint16_t remote_port,
                               const UdpSocketOptions& options) {
  Close();
  opened_ = false;
  if (!EnsureSocketRuntime()) {
    return UdpStatus::kSocketError;
  }

  bool local_ok = false;
  const sockaddr_in local = MakeAddress(local_address, local_port, &local_ok);
  if (!local_ok || remote_address == nullptr || remote_address[0] == '\0') {
    return UdpStatus::kInvalidArgument;
  }
  bool remote_ok = false;
  const sockaddr_in remote =
      MakeAddress(remote_address, remote_port, &remote_ok);
  if (!remote_ok || remote.sin_addr.s_addr == htonl(INADDR_ANY)) {
    return UdpStatus::kInvalidArgument;
  }
  local_address_ = local.sin_addr.s_addr;
  local_port_ = local.sin_port;
  remote_address_ = remote.sin_addr.s_addr;
  remote_port_ = remote.sin_port;
  options_ = options;

  const UdpStatus status = OpenSocket();
  if (status != UdpStatus::kOk) {
    return status;
  }
  opened_ = true;
  next_send_sequence_ = 1;
  stats_ = {};
  return UdpStatus::kOk;
}

UdpStatus RuntimeUdpPeer::Reopen() {
  if (!opened_) {
    return UdpStatus::kInvalidArgument;
  }
  Close();
  return OpenSocket();
}

UdpStatus RuntimeUdpPeer::OpenSocket() {
  const uintptr_t new_socket =
      static_cast<uintptr_t>(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
  if (new_socket == kInvalidSocket) {
    return UdpStatus::kSocketError;
  }
  socket_ = new_socket;

  const UdpStatus status = ConfigureSocket(options_);
  if (status != UdpStatus::kOk) {
    Close();
    return status;
  }

  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = local_address_;
  local.sin_port = local_port_;
  if (bind(ToNativeSocket(socket_), reinterpret_cast<const sockaddr*>(&local),
           sizeof(local)) != 0) {
    Close();
    return UdpStatus::kSocketError;
  }

  if (options_.connect_peer) {
    sockaddr_in remote{};
    remote.sin_family = AF_INET;
    remote.sin_addr.s_addr = remote_address_;
    remote.sin_port = remote_port_;
    if (connect(ToNativeSocket(socket_),
                reinterpret_cast<const sockaddr*>(&remote),
                sizeof(remote)) != 0) {
      const int error = LastSocketError();
      if (!options_.non_blocking || !IsWouldBlockError(error)) {
        Close();
        return UdpStatus::kSocketError;
      }
    }
  }
  return UdpStatus::kOk;
}

void RuntimeUdpPeer::Close() {
  if (socket_ != kInvalidSocket) {
    CloseSocket(socket_);
    socket_ = kInvalidSocket;
  }
}

bool RuntimeUdpPeer::IsOpen() const { return socket_ != kInvalidSocket; }

int RuntimeUdpPeer::NativeFd() const {
#if defined(_WIN32)
  return -1;
#else
  return IsOpen() ? ToNativeSocket(socket_) : -1;
#endif
}

UdpStatus RuntimeUdpPeer::SendFrame(FrameType type, uint16_t flags,
                                    uint64_t monotonic_time_us,
                                    const void* payload,
                                    std::size_t payload_size) {
  if (!IsOpen()) {
    return UdpStatus::kSocketError;
  }
  const std::size_t actual_frame_size =
      EncodeFrame(type, flags, next_send_sequence_, monotonic_time_us, payload,
                  payload_size, tx_buffer_.data(), tx_buffer_.size());
  if (actual_frame_size == 0) {
    return UdpStatus::kInvalidArgument;
  }

  return Send(actual_frame_size);
}

uint8_t* RuntimeUdpPeer::MutablePayloadBuffer(std::size_t payload_size) {
  if (payload_size > kMaxPayloadSize) {
    return nullptr;
  }
  return tx_buffer_.data() + kFrameHeaderSize;
}

UdpStatus RuntimeUdpPeer::SendBufferedFrame(FrameType type, uint16_t flags,
                                            uint64_t monotonic_time_us,
                                            std::size_t payload_size) {
  if (!IsOpen()) {
    return UdpStatus::kSocketError;
  }
  const std::size_t frame_size =
      EncodeFrameInPlace(type, flags, next_send_sequence_, monotonic_time_us,
                         payload_size, tx_buffer_.data(), tx_buffer_.size());
  if (frame_size == 0) {
    return UdpStatus::kInvalidArgument;
  }

  return Send(frame_size);
}

UdpStatus RuntimeUdpPeer::Send(std::size_t frame_size) {
#if defined(_WIN32)
  const int sent = send(ToNativeSocket(socket_),
                        reinterpret_cast<const char*>(tx_buffer_.data()),
                        static_cast<int>(frame_size), kSendFlags);
#else
  const ssize_t sent =
      send(ToNativeSocket(socket_), tx_buffer_.data(), frame_size, kSendFlags);
#endif
  if (sent < 0) {
    const int error = LastSocketError();
    if (IsWouldBlockError(error)) {
      return UdpStatus::kWouldBlock;
    }
    stats_.last_send_error = error;
    return UdpStatus::kSocketError;
  }
  if (static_cast<std::size_t>(sent) != frame_size) {
    stats_.last_send_error = 0;  // short, with no error to report
    return UdpStatus::kSocketError;
  }

  ++next_send_sequence_;
  ++stats_.sent_frames;
  stats_.sent_bytes += frame_size;
  return UdpStatus::kOk;
}

UdpStatus RuntimeUdpPeer::TryReceive(DecodedFrame* out) {
  if (!IsOpen() || out == nullptr) {
    return UdpStatus::kInvalidArgument;
  }

#if defined(_WIN32)
  const int received =
      recv(ToNativeSocket(socket_), reinterpret_cast<char*>(rx_buffer_.data()),
           static_cast<int>(rx_buffer_.size()), 0);
#else
  const ssize_t received =
      recv(ToNativeSocket(socket_), rx_buffer_.data(), rx_buffer_.size(), 0);
#endif
  if (received < 0) {
    const int error = LastSocketError();
    if (IsWouldBlockError(error)) {
      ++stats_.receive_would_block;
      return UdpStatus::kWouldBlock;
    }
    return UdpStatus::kSocketError;
  }

  const auto received_size = static_cast<std::size_t>(received);
  DecodedFrame decoded;
  const DecodeStatus decode_status =
      DecodeFrame(rx_buffer_.data(), received_size, &decoded);
  stats_.last_decode_status = decode_status;
  if (decode_status == DecodeStatus::kUnsupportedVersion) {
    stats_.last_unsupported_version = decoded.header.version;
  }
  if (decode_status != DecodeStatus::kOk) {
    ++stats_.decode_failures;
    return UdpStatus::kDecodeError;
  }

  if (stats_.last_received_sequence != 0 &&
      decoded.header.sequence != stats_.last_received_sequence + 1) {
    ++stats_.sequence_gaps;
  }
  stats_.last_received_sequence = decoded.header.sequence;
  ++stats_.received_frames;
  stats_.received_bytes += received_size;
  *out = decoded;
  return UdpStatus::kOk;
}

UdpStatus RuntimeUdpPeer::ConfigureSocket(const UdpSocketOptions& options) {
#if defined(SO_NOSIGPIPE)
  int no_sigpipe = 1;
  if (setsockopt(ToNativeSocket(socket_), SOL_SOCKET, SO_NOSIGPIPE, &no_sigpipe,
                 sizeof(no_sigpipe)) != 0) {
    return UdpStatus::kSocketError;
  }
#endif

  if (options.reuse_address) {
    int enabled = 1;
    if (setsockopt(ToNativeSocket(socket_), SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast<const char*>(&enabled),
                   sizeof(enabled)) != 0) {
      return UdpStatus::kSocketError;
    }
  }

  if (options.receive_buffer_bytes > 0) {
    int size = options.receive_buffer_bytes;
    if (setsockopt(ToNativeSocket(socket_), SOL_SOCKET, SO_RCVBUF,
                   reinterpret_cast<const char*>(&size), sizeof(size)) != 0) {
      return UdpStatus::kSocketError;
    }
  }

  if (options.send_buffer_bytes > 0) {
    int size = options.send_buffer_bytes;
    if (setsockopt(ToNativeSocket(socket_), SOL_SOCKET, SO_SNDBUF,
                   reinterpret_cast<const char*>(&size), sizeof(size)) != 0) {
      return UdpStatus::kSocketError;
    }
  }

  int tos = options.ip_tos;
  setsockopt(ToNativeSocket(socket_), IPPROTO_IP, IP_TOS,
             reinterpret_cast<const char*>(&tos), sizeof(tos));

  if (options.non_blocking && !SetNonBlocking(socket_)) {
    return UdpStatus::kSocketError;
  }

  return UdpStatus::kOk;
}

const char* UdpStatusName(UdpStatus status) {
  switch (status) {
    case UdpStatus::kOk:
      return "Ok";
    case UdpStatus::kWouldBlock:
      return "WouldBlock";
    case UdpStatus::kInvalidArgument:
      return "InvalidArgument";
    case UdpStatus::kSocketError:
      return "SocketError";
    case UdpStatus::kDecodeError:
      return "DecodeError";
  }
  return "Unknown";
}

}  // namespace talos::protocol

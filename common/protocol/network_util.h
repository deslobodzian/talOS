#pragma once

#include <cstring>
#include <cstdint>

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
#if defined (_WIN32)
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

// Windows needs this so keeping parity
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

NativeSocket ToNativeSocket(uintptr_t socket_handle) {
    return static_cast<NativeSocket>(socket_handle);
}

sockaddr_in MakeAddress(const char* address, uint16_t port, bool* ok) {
    sockaddr_in out{};
    out.sin_family = AF_INET;
    out.sin_port = htons(port);
    if (address == nullptr || std::strcmp(address, "0.0.0.0") == 0 ||
        std::strcmp(address, "") == 0) {
        out.sin_addr.s_addr = htonl(INADDR_ANY) ;
        *ok = true;
        return out;
    }
    *ok = inet_pton(AF_INET, address, &out.sin_addr) == 1;
    return out;
}

} /* namespace talos::protocol */

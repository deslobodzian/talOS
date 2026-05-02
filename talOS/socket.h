#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

namespace Socket {
inline auto set_non_blocking(int fd) -> bool {
  const auto flags = fcntl(fd, F_GETFL, 0);
  if (flags & O_NONBLOCK) {
    return true;
  }

  return (fcntl(fd, F_SETFL, flags | O_NONBLOCK) != -1);
}

inline auto set_no_deplay(int fd) -> bool {
  int one{1};
  return setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<void*>(&one),
                    sizeof(one) != -1);
}

} /* namespace Socket */

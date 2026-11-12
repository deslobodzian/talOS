// studio_review: serves recorded sessions to Studio (studio/README.md
// "Reviewing a session").
//
//   studio_review --library DIR [--port 5805] [--cache-mb 256]
//
// Listens on 127.0.0.1 only; there is no flag to listen anywhere else. Port
// 0 asks the OS for one. Once listening, the address the socket is bound to,
// as the kernel reports it (getsockname), is printed on stdout:
// "listening 127.0.0.1:<port>".

#include <App.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>
#include <system_error>

#include "studio/review/review.h"

namespace {

// The one host this server binds: a review server reads the programmer's
// logs and has no authentication, so nothing off this machine may reach it.
constexpr const char* kLoopback = "127.0.0.1";

const char* StatusLine(int status) {
  switch (status) {
    case 200:
      return "200 OK";
    case 400:
      return "400 Bad Request";
    case 404:
      return "404 Not Found";
    case 422:
      return "422 Unprocessable Content";
    default:
      return "500 Internal Server Error";
  }
}

// The listening socket's bound address and port, "host:port".
std::string BoundAddress(us_listen_socket_t* listener) {
  const auto fd = static_cast<int>(
      reinterpret_cast<std::intptr_t>(us_socket_get_native_handle(
          0, reinterpret_cast<us_socket_t*>(listener))));
  sockaddr_storage address{};
  socklen_t length = sizeof address;
  if (getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length) != 0) {
    return "?";
  }
  char host[INET6_ADDRSTRLEN] = {};
  std::uint16_t port = 0;
  if (address.ss_family == AF_INET) {
    const auto* v4 = reinterpret_cast<const sockaddr_in*>(&address);
    inet_ntop(AF_INET, &v4->sin_addr, host, sizeof host);
    port = ntohs(v4->sin_port);
  } else if (address.ss_family == AF_INET6) {
    const auto* v6 = reinterpret_cast<const sockaddr_in6*>(&address);
    inet_ntop(AF_INET6, &v6->sin6_addr, host, sizeof host);
    port = ntohs(v6->sin6_port);
  }
  return std::string{host} + ':' + std::to_string(port);
}

bool ParseUnsigned(std::string_view text, std::uint64_t max,
                   std::uint64_t& out) {
  const auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), out);
  return error == std::errc{} && end == text.data() + text.size() && out <= max;
}

int Usage(const std::string& error) {
  std::cerr << "studio_review: " << error
            << "\nusage: studio_review --library DIR [--port 5805] "
               "[--cache-mb 256]\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path library;
  std::uint64_t port = 5805;
  std::uint64_t cache_mb = 256;
  for (int i = 1; i < argc; ++i) {
    const std::string_view flag = argv[i];
    if (i + 1 == argc) return Usage("missing value for " + std::string{flag});
    const std::string_view value = argv[++i];
    if (flag == "--library") {
      library = value;
    } else if (flag == "--port") {
      if (!ParseUnsigned(value, 65535, port)) {
        return Usage("invalid --port " + std::string{value});
      }
    } else if (flag == "--cache-mb") {
      if (!ParseUnsigned(value, 1u << 20, cache_mb)) {
        return Usage("invalid --cache-mb " + std::string{value});
      }
    } else {
      return Usage("unknown flag " + std::string{flag});
    }
  }
  std::error_code error;
  if (library.empty() || !std::filesystem::is_directory(library, error)) {
    return Usage("--library must name a directory");
  }

  studio::review::Review review{library, cache_mb << 20};
  uWS::App app;
  app.get("/*", [&review](auto* res, auto* req) {
    const studio::review::Response response =
        review.Get(req->getUrl(), req->getQuery());
    res->writeStatus(StatusLine(response.status))
        ->writeHeader("Content-Type", response.content_type)
        ->writeHeader("X-Content-Type-Options", "nosniff")
        ->writeHeader("Cache-Control", "no-store")
        ->end(response.body);
  });
  us_listen_socket_t* listener = nullptr;
  app.listen(kLoopback, static_cast<int>(port),
             [&listener](us_listen_socket_t* token) { listener = token; });
  if (listener == nullptr) {
    std::cerr << "studio_review: cannot listen on " << kLoopback << ':' << port
              << '\n';
    return 1;
  }
  std::cout << "listening " << BoundAddress(listener) << '\n' << std::flush;
  app.run();
  return 0;
}

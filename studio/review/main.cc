// studio_review: serves recorded sessions, and the Studio that reviews them,
// to a browser on this machine (studio/README.md "Reviewing a recorded
// session").
//
//   studio_review --library DIR [--port 5805] [--cache-mb 256]
//                 [--webroot DIR]
//
// Listens on 127.0.0.1 only; there is no flag to listen anywhere else. Port
// 0 asks the OS for one. Once listening, the address the socket is bound to,
// as the kernel reports it (getsockname), is printed on stdout:
// "listening 127.0.0.1:<port>".
//
// /review/* is the review API (review.h); every other path is a file of the
// built Studio (--webroot, by default the //studio:web_dist this binary was
// built with, from its runfiles), so a Studio opened here reads the API from
// its own origin. A request whose Host header is not this server's loopback
// name and port is refused (403); a request from another loopback origin
// gets CORS headers for it (http.h). Relative paths are taken from the
// directory `bazel run` was invoked in.

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
#include <memory>
#include <string>
#include <string_view>
#include <system_error>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "studio/review/http.h"
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
    case 403:
      return "403 Forbidden";
    case 404:
      return "404 Not Found";
    case 413:
      return "413 Content Too Large";
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
               "[--cache-mb 256] [--webroot DIR]\n";
  return 2;
}

// `path` from the directory `bazel run` was started in, when it set one.
std::filesystem::path FromInvocation(std::string_view path) {
  std::filesystem::path given{path};
  const char* invoked = std::getenv("BUILD_WORKING_DIRECTORY");
  if (given.is_absolute() || invoked == nullptr) return given;
  return std::filesystem::path{invoked} / given;
}

// The built Studio this binary was built with, from its runfiles; empty
// when there are none (a binary copied out of bazel-bin).
std::filesystem::path BuiltStudio(const char* argv0) {
  std::string error;
  const std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::Create(argv0, BAZEL_CURRENT_REPOSITORY,
                                               &error));
  if (runfiles == nullptr) return {};
  const std::filesystem::path root = runfiles->Rlocation(TALOS_WEB_DIST);
  std::error_code ignored;
  return std::filesystem::is_directory(root, ignored) ? root
                                                      : std::filesystem::path{};
}

}  // namespace

int main(int argc, char** argv) {
  std::filesystem::path library;
  std::filesystem::path webroot;
  std::uint64_t port = 5805;
  std::uint64_t cache_mb = 256;
  for (int i = 1; i < argc; ++i) {
    const std::string_view flag = argv[i];
    if (i + 1 == argc) return Usage("missing value for " + std::string{flag});
    const std::string_view value = argv[++i];
    if (flag == "--library") {
      library = FromInvocation(value);
    } else if (flag == "--webroot") {
      webroot = FromInvocation(value);
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
  if (webroot.empty()) {
    webroot = BuiltStudio(argv[0]);
  } else if (!std::filesystem::is_directory(webroot, error)) {
    return Usage("--webroot must name a directory");
  }
  if (webroot.empty()) {
    std::cerr << "studio_review: no built Studio beside this binary; only "
                 "/review/* is served (pass --webroot DIR)\n";
  }

  studio::review::Review review{library, cache_mb << 20};
  std::uint16_t bound_port = 0;  // set once listening, before any request
  uWS::App app;
  app.get("/*", [&](auto* res, auto* req) {
    studio::review::Response response;
    const std::string_view url = req->getUrl();
    if (!studio::review::AllowedHost(req->getHeader("host"), bound_port)) {
      response = {403, "text/plain; charset=utf-8",
                  "studio_review answers only 127.0.0.1, localhost or [::1] "
                  "with its port\n"};
    } else if (url.starts_with("/review/")) {
      response = review.Get(url, req->getQuery());
    } else {
      response = studio::review::StaticFile(webroot, url);
    }
    const std::string origin =
        studio::review::AllowedOrigin(req->getHeader("origin"));
    res->writeStatus(StatusLine(response.status));
    if (!origin.empty())
      res->writeHeader("Access-Control-Allow-Origin", origin);
    res->writeHeader("Vary", "Origin")
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
  const std::string bound = BoundAddress(listener);
  std::uint64_t parsed_port = 0;
  if (!ParseUnsigned(std::string_view{bound}.substr(bound.rfind(':') + 1),
                     65535, parsed_port)) {
    std::cerr << "studio_review: cannot read the bound port\n";
    return 1;
  }
  bound_port = static_cast<std::uint16_t>(parsed_port);
  std::cout << "listening " << bound << '\n' << std::flush;
  app.run();
  return 0;
}

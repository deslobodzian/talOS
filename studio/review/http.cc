#include "studio/review/http.h"

#include <algorithm>
#include <charconv>

#include "studio/bridge/static_files.h"

namespace studio::review {
namespace {

namespace fs = std::filesystem;

constexpr std::string_view kLoopbackNames[] = {"127.0.0.1", "localhost",
                                               "[::1]"};

// `text` is one of the loopback names, then ':' and a decimal port; the
// port, or -1.
int LoopbackPort(std::string_view text) {
  for (const std::string_view name : kLoopbackNames) {
    if (!text.starts_with(name) || text.size() <= name.size() + 1 ||
        text[name.size()] != ':') {
      continue;
    }
    const std::string_view digits = text.substr(name.size() + 1);
    int port = -1;
    const auto [end, error] =
        std::from_chars(digits.data(), digits.data() + digits.size(), port);
    if (error != std::errc{} || end != digits.data() + digits.size() ||
        digits.front() == '0' || port < 1 || port > 65535) {
      return -1;
    }
    return port;
  }
  return -1;
}

Response Plain(int status, std::string message) {
  return {status, "text/plain; charset=utf-8", std::move(message) + "\n"};
}

}  // namespace

bool AllowedHost(std::string_view host, std::uint16_t port) {
  // A browser omits the scheme's default port from Host.
  constexpr std::uint16_t kHttpPort = 80;
  if (port == kHttpPort &&
      std::find(std::begin(kLoopbackNames), std::end(kLoopbackNames), host) !=
          std::end(kLoopbackNames)) {
    return true;
  }
  return LoopbackPort(host) == port;
}

std::string AllowedOrigin(std::string_view origin) {
  constexpr std::string_view kScheme = "http://";
  if (!origin.starts_with(kScheme)) return {};
  return LoopbackPort(origin.substr(kScheme.size())) > 0 ? std::string{origin}
                                                         : std::string{};
}

Response StaticFile(const fs::path& root, std::string_view url) {
  // Contained by name, not by resolving symlinks: the root is a build output,
  // which Bazel may stage as symlinks to files elsewhere, and only the URL is
  // the requester's.
  const std::string_view mapped = url == "/" ? "/index.html" : url;
  const studio::StaticFileResult result = studio::ReadStaticFile(
      root, mapped, {.containment = studio::Containment::kLexical});
  switch (result.status) {
    case 200:
      return {200, std::string{result.content_type}, result.body};
    case 400:
      return Plain(400, "bad path");
    case 413:
      return Plain(413, "too large");
    default:
      return Plain(404, "not found");
  }
}

}  // namespace studio::review

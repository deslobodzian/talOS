#include "studio/review/http.h"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <iterator>
#include <system_error>

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

std::string_view ContentType(const fs::path& file) {
  const std::string ext = file.extension().string();
  if (ext == ".html") return "text/html; charset=utf-8";
  if (ext == ".js") return "text/javascript";
  if (ext == ".css") return "text/css";
  if (ext == ".json") return "application/json";
  if (ext == ".svg") return "image/svg+xml";
  if (ext == ".wasm") return "application/wasm";
  return "application/octet-stream";
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
  if (url.find('%') != std::string_view::npos ||
      url.find('\\') != std::string_view::npos) {
    return Plain(400, "bad path");
  }
  if (root.empty() || !url.starts_with('/')) return Plain(404, "not found");
  // Contained by name, not by resolving symlinks: the root is a build
  // output, which Bazel may stage as symlinks to files elsewhere, and only
  // the URL is the requester's.
  const fs::path relative =
      fs::path{url == "/" ? std::string_view{"index.html"} : url.substr(1)}
          .lexically_normal();
  if (relative.empty() || relative.is_absolute() || *relative.begin() == "..") {
    return Plain(404, "not found");
  }
  std::error_code error;
  const fs::path file = root / relative;
  if (!fs::is_regular_file(file, error)) return Plain(404, "not found");
  if (fs::file_size(file, error) > kMaxStaticFileBytes || error) {
    return Plain(413, "too large");
  }
  std::ifstream input{file, std::ios::binary};
  if (!input) return Plain(404, "not found");
  return {200, std::string{ContentType(file)},
          std::string{std::istreambuf_iterator<char>{input}, {}}};
}

}  // namespace studio::review

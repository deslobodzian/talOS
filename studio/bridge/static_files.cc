#include "studio/bridge/static_files.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <system_error>
#include <utility>

namespace studio {
namespace {

namespace fs = std::filesystem;

constexpr std::size_t kAssetHashChars = 8;

struct ScopedFd {
  int fd{-1};
  explicit ScopedFd(int descriptor) : fd(descriptor) {}
  ~ScopedFd() {
    if (fd >= 0) ::close(fd);
  }
  ScopedFd(const ScopedFd&) = delete;
  ScopedFd& operator=(const ScopedFd&) = delete;
};

}  // namespace

std::string_view MimeType(const fs::path& path) {
  const std::string ext = path.extension().string();
  if (ext == ".html") return "text/html; charset=utf-8";
  if (ext == ".js") return "text/javascript";
  if (ext == ".css") return "text/css";
  if (ext == ".json") return "application/json";
  if (ext == ".svg") return "image/svg+xml";
  if (ext == ".wasm") return "application/wasm";
  return "application/octet-stream";
}

bool ContentHashed(const fs::path& relative) {
  auto part = relative.begin();
  if (part == relative.end() || *part != "assets") return false;
  if (++part == relative.end()) return false;
  // Copy by value before incrementing: path::iterator may stash its element.
  const fs::path name = *part;
  if (++part != relative.end()) return false;  // not directly under assets/
  const std::string stem = name.stem().string();
  if (name.extension().empty() || stem.size() < kAssetHashChars + 2 ||
      stem[stem.size() - kAssetHashChars - 1] != '-') {
    return false;
  }
  return std::all_of(stem.end() - static_cast<std::ptrdiff_t>(kAssetHashChars),
                     stem.end(), [](char c) {
                       return (c >= 'a' && c <= 'z') ||
                              (c >= 'A' && c <= 'Z') ||
                              (c >= '0' && c <= '9') || c == '-' || c == '_';
                     });
}

std::string_view CacheControl(const fs::path& relative) {
  return ContentHashed(relative) ? kImmutableCacheControl
                                 : kRevalidateCacheControl;
}

fs::path LinkedOut(const fs::path& webroot, std::string_view page) {
  if (page.find('\0') != std::string_view::npos) return {};
  std::error_code error;
  const fs::path root = fs::weakly_canonical(webroot, error);
  if (error) return {};
  fs::path file = fs::weakly_canonical(root / page, error);
  if (error || !fs::is_regular_file(file, error)) return {};
  const fs::path relative = file.lexically_relative(root);
  if (!relative.empty() && *relative.begin() != "..") return {};
  return file;
}

StaticFileResult ReadStaticFile(const fs::path& root, std::string_view url,
                                StaticFileOptions options) {
  if (url.find('%') != std::string_view::npos ||
      url.find('\\') != std::string_view::npos ||
      url.find('\0') != std::string_view::npos) {
    return {.status = 400};
  }
  if (root.empty() || !url.starts_with('/')) {
    return {.status = 404};
  }
  const std::string_view sub = url.substr(1);
  if (sub.empty()) {
    return {.status = 404};
  }
  const fs::path sub_path{sub};
  if (sub_path.is_absolute()) {
    return {.status = 404};
  }

  fs::path file;
  fs::path relative;
  std::error_code error;
  if (options.containment == Containment::kCanonical) {
    const fs::path canonical_root = fs::weakly_canonical(root, error);
    if (error) return {.status = 404};
    file = fs::weakly_canonical(canonical_root / sub_path, error);
    if (error) return {.status = 404};
    relative = file.lexically_relative(canonical_root);
    if (relative.empty() || relative == "." || *relative.begin() == "..") {
      return {.status = 404};
    }
  } else {
    relative = sub_path.lexically_normal();
    if (relative.empty() || relative == "." || relative.is_absolute() ||
        *relative.begin() == "..") {
      return {.status = 404};
    }
    file = root / relative;
  }

  // Open non-blocking (+ O_NOFOLLOW for kCanonical) and verify with fstat +
  // bounded chunked reads so FIFOs, device nodes, TOCTOU symlink swaps, and
  // growing files cannot hang the loop, escape confinement, or OOM.
  int open_flags = O_RDONLY | O_NONBLOCK | O_CLOEXEC;
  if (options.containment == Containment::kCanonical) {
    open_flags |= O_NOFOLLOW;
  }
  const ScopedFd opened{::open(file.c_str(), open_flags)};
  if (opened.fd < 0) {
    return {.status = 404};
  }
  struct stat sb {};
  if (::fstat(opened.fd, &sb) != 0 || !S_ISREG(sb.st_mode) || sb.st_size < 0) {
    return {.status = 404};
  }
  if (static_cast<std::uintmax_t>(sb.st_size) > options.max_bytes) {
    return {.status = 413};
  }
  std::string body;
  body.reserve(static_cast<std::size_t>(sb.st_size));
  std::array<char, 8192> chunk{};
  for (;;) {
    const ssize_t got = ::read(opened.fd, chunk.data(), chunk.size());
    if (got == 0) break;
    if (got < 0) {
      if (errno == EINTR) continue;
      return {.status = 404};
    }
    const auto got_size = static_cast<std::size_t>(got);
    if (static_cast<std::uintmax_t>(got_size) >
        options.max_bytes - body.size()) {
      return {.status = 413};
    }
    body.append(chunk.data(), got_size);
  }
  return {
      .status = 200,
      .content_type = MimeType(file),
      .cache_control = CacheControl(relative),
      .body = std::move(body),
  };
}

std::optional<std::string> ReadBoundedRegularFile(const fs::path& path,
                                                  std::uintmax_t max_bytes) {
  std::error_code error;
  const fs::file_status st = fs::symlink_status(path, error);
  if (error || fs::is_symlink(st) || !fs::is_regular_file(st)) {
    return std::nullopt;
  }
  struct ScopedFd {
    int fd{-1};
    ~ScopedFd() {
      if (fd >= 0) ::close(fd);
    }
  };
  const ScopedFd file{
      ::open(path.c_str(), O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC)};
  if (file.fd < 0) return std::nullopt;
  struct stat sb {};
  if (::fstat(file.fd, &sb) != 0 || !S_ISREG(sb.st_mode) || sb.st_size <= 0 ||
      static_cast<std::uintmax_t>(sb.st_size) > max_bytes) {
    return std::nullopt;
  }
  std::string body;
  body.reserve(static_cast<std::size_t>(sb.st_size));
  std::array<char, 8192> chunk{};
  for (;;) {
    const ssize_t got = ::read(file.fd, chunk.data(), chunk.size());
    if (got == 0) break;
    if (got < 0) {
      if (errno == EINTR) continue;
      return std::nullopt;
    }
    if (body.size() + static_cast<std::size_t>(got) > max_bytes) {
      return std::nullopt;
    }
    body.append(chunk.data(), static_cast<std::size_t>(got));
  }
  if (body.empty()) return std::nullopt;
  return body;
}

}  // namespace studio

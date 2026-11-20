#include "studio/bridge/static_files.h"

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <system_error>

namespace studio {
namespace {

namespace fs = std::filesystem;

constexpr std::size_t kAssetHashChars = 8;

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
  const fs::path& name = *part;
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
  std::error_code error;
  const fs::path root = fs::weakly_canonical(webroot, error);
  if (error) return {};
  const fs::path file = fs::weakly_canonical(root / page, error);
  if (error || !fs::is_regular_file(file, error)) return {};
  const fs::path relative = file.lexically_relative(root);
  if (!relative.empty() && *relative.begin() != "..") return {};
  return file;
}

StaticFileResult ReadStaticFile(const fs::path& root, std::string_view url,
                                StaticFileOptions options) {
  if (url.find('%') != std::string_view::npos ||
      url.find('\\') != std::string_view::npos) {
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

  if (!fs::is_regular_file(file, error) || error) {
    return {.status = 404};
  }
  const std::uintmax_t size = fs::file_size(file, error);
  if (error) {
    return {.status = 404};
  }
  if (size > options.max_bytes) {
    return {.status = 413};
  }
  std::ifstream input(file, std::ios::binary);
  if (!input) {
    return {.status = 404};
  }
  return {
      .status = 200,
      .content_type = MimeType(file),
      .cache_control = CacheControl(relative),
      .body = std::string{std::istreambuf_iterator<char>{input}, {}},
  };
}

}  // namespace studio

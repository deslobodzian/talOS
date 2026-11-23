#pragma once

// Static HTTP file serving shared by studio_bridge and studio_review:
// MIME lookup, Vite hashed-asset Cache-Control, path containment, symlink
// escape detection (LinkedOut), size capping and file reading; and the
// bounded, no-follow read of one regular file studio_review's session
// library uses (ReadBoundedRegularFile).

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace studio {

// Static files are read synchronously on the server loop thread, so each is
// capped at a few times the built Studio bundle's largest asset.
inline constexpr std::uintmax_t kMaxStaticFileBytes = std::uintmax_t{4} << 20;

// Cache-Control (REQ-OPS-34). Studio's Vite build names every asset it emits
// under assets/ `<name>-<hash>.<ext>`, the hash eight characters of base64url
// (Rollup's default; its alphabet includes '-' and '_'): such a file never
// changes under its name, so a browser keeps it for a year. Every other static
// file, index.html and drive.html above all, is revalidated on each load
// (`no-cache`), so a deploy's new page and the asset names it carries are seen
// at once. Live JSON snapshots have no validator to revalidate against, so
// they are never stored (`no-store`).
inline constexpr std::string_view kImmutableCacheControl =
    "public, max-age=31536000, immutable";
inline constexpr std::string_view kRevalidateCacheControl = "no-cache";
inline constexpr std::string_view kLiveCacheControl = "no-store";

// MIME type for a static file path by extension.
std::string_view MimeType(const std::filesystem::path& path);

// Whether `relative` (a path under the webroot) is a Vite content-hashed asset
// (`assets/<name>-<8-char-base64url>.<ext>`).
bool ContentHashed(const std::filesystem::path& relative);

// Cache-Control header value for a static file at `relative` under the webroot.
std::string_view CacheControl(const std::filesystem::path& relative);

// Where `page`, a file name in `webroot`, resolves when it is a regular file
// there that links out of the webroot's real tree (empty otherwise).
// studio_bridge serves only files inside that tree, so such a page is present
// but never served: a sandboxed Bazel test's runfiles stage each file of a
// tree artifact as a link to the execroot. `bazel run` links the webroot
// directory itself, and an installed bundle holds no links.
std::filesystem::path LinkedOut(const std::filesystem::path& webroot,
                                std::string_view page);

// How ReadStaticFile checks that the resolved file stays inside `root`:
// - kCanonical: resolves symlinks via weakly_canonical and rejects any file
//   whose real path lies outside `root`'s real tree (studio_bridge).
// - kLexical: normalizes the relative URL path lexically without resolving
//   symlinks, so symlinks inside a Bazel-staged runfiles webroot are followed
//   while `..` and absolute URL paths are still rejected (studio_review).
enum class Containment {
  kCanonical,
  kLexical,
};

struct StaticFileOptions {
  Containment containment = Containment::kCanonical;
  std::uintmax_t max_bytes = kMaxStaticFileBytes;
};

struct StaticFileResult {
  // HTTP status: 200, 400 (bad path), 404 (not found / outside root), or 413
  // (too large).
  int status = 404;
  std::string_view content_type;
  std::string_view cache_control;
  std::string body;
};

// Reads a static file at URL path `url` under `root`.
// `url` must start with '/' and contain none of '%', '\\', or '\0' (else
// 400 / 404).
// The caller resolves any route aliases (such as "/" -> "/index.html" or
// "/drive" -> "/drive.html") before calling.
StaticFileResult ReadStaticFile(const std::filesystem::path& root,
                                std::string_view url,
                                StaticFileOptions options = {});

// Reads one regular non-symlink file up to `max_bytes` using O_NOFOLLOW +
// O_NONBLOCK + fstat + bounded chunked reads, so symlinks, FIFOs, device nodes,
// and growing files never escape confinement, hang, or allocate unbounded
// memory (REQ-OPS-42, REQ-SAFE-35). Empty when the path is a symlink, not a
// regular file, empty, larger than `max_bytes` (at open or while read), or
// unreadable.
std::optional<std::string> ReadBoundedRegularFile(
    const std::filesystem::path& path, std::uintmax_t max_bytes);

}  // namespace studio

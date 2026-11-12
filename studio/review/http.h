#pragma once

// What studio_review decides about a request before the review API sees it,
// as functions of strings (no socket), so all of it is tested in process.
//
// The server is reachable only from this machine (main.cc binds 127.0.0.1),
// but a page in this machine's browser can still name it: the Host check
// refuses a page that reached it under another name (DNS rebinding), and
// CORS lets only a Studio served from another loopback port (the live
// bridge's, or the dev server's) read its answers.

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

#include "studio/review/review.h"

namespace studio::review {

// Whether `host` (a request's Host header) names this server: 127.0.0.1,
// localhost or [::1], with `port`, the port the server listens on.
bool AllowedHost(std::string_view host, std::uint16_t port);

// The Access-Control-Allow-Origin a request with Origin header `origin` gets:
// the origin itself for http://127.0.0.1:<port>, http://localhost:<port> or
// http://[::1]:<port> (any port, never "*"); empty for any other origin, or
// none.
std::string AllowedOrigin(std::string_view origin);

// The largest file the static handler serves: a few times the built
// Studio's largest asset, as the live bridge's cap.
inline constexpr std::uintmax_t kMaxStaticFileBytes = std::uintmax_t{4} << 20;

// The built Studio (//studio:web_dist) under `root` at URL path `url`, "/"
// meaning index.html. 400 for a path holding '%' or '\\' (an encoded or
// ambiguous name), 404 for one whose normal form leaves `root` (by ".."), is
// not a regular file, or when `root` is empty; 413 past kMaxStaticFileBytes.
// A symlink inside `root` is followed: the root is a build output (Bazel
// stages one as symlinks), and only the URL comes from the requester.
Response StaticFile(const std::filesystem::path& root, std::string_view url);

}  // namespace studio::review

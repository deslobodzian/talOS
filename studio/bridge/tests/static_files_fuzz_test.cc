// Regression and seeded mutation fuzz tests for studio/bridge static file
// serving, refusal summary truncation, and the patched uWebSockets HTTP and
// WebSocket parsers (REQ-OPS-34, REQ-OPS-35, REQ-OPS-41, REQ-SAFE-35).

#include <App.h>
#include <AsyncSocketData.h>
#include <HttpParser.h>
#include <HttpRouter.h>
#include <QueryParser.h>
#include <WebSocketHandshake.h>
#include <WebSocketProtocol.h>
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "studio/bridge/drive.h"
#include "studio/bridge/static_files.h"

namespace studio {
namespace {

namespace fs = std::filesystem;

struct Rng {
  std::uint64_t state;
  explicit Rng(std::uint64_t seed) : state(seed) {}
  std::uint64_t Next() {
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    return state;
  }
  std::size_t Range(std::size_t n) { return n == 0 ? 0 : Next() % n; }
  bool Chance(std::size_t num, std::size_t den) { return Range(den) < num; }
};

std::string Mutate(Rng& rng, std::string text,
                   const std::vector<std::string>& dict, std::size_t max_len) {
  const std::size_t ops = 1 + rng.Range(5);
  for (std::size_t op = 0; op < ops; ++op) {
    const std::size_t kind = rng.Range(7);
    if (kind == 0 && !text.empty()) {
      const std::size_t idx = rng.Range(text.size());
      const unsigned char flipped =
          static_cast<unsigned char>(text[idx]) ^
          static_cast<unsigned char>(1u << rng.Range(8));
      text[idx] = static_cast<char>(flipped);
    } else if (kind == 1 && !text.empty()) {
      text[rng.Range(text.size())] = static_cast<char>(rng.Next() & 0xff);
    } else if (kind == 2 && text.size() < max_len) {
      static constexpr char kBytes[] = {'\0', '\r', '\n', ' ',    '\t',  '/',
                                        '\\', '%',  '.',  '?',    '&',   '=',
                                        ':',  ';',  ',',  '\x7f', '\xff'};
      const char byte = rng.Chance(2, 3) ? kBytes[rng.Range(sizeof(kBytes))]
                                         : static_cast<char>(rng.Next() & 0xff);
      text.insert(text.begin() +
                      static_cast<std::ptrdiff_t>(rng.Range(text.size() + 1)),
                  byte);
    } else if (kind == 3 && !text.empty()) {
      const std::size_t pos = rng.Range(text.size());
      const std::size_t len =
          1 + rng.Range(std::min<std::size_t>(12, text.size() - pos));
      text.erase(pos, len);
    } else if (kind == 4 && !dict.empty() && text.size() < max_len) {
      const std::string& token = dict[rng.Range(dict.size())];
      if (text.size() + token.size() <= max_len) {
        text.insert(rng.Range(text.size() + 1), token);
      }
    } else if (kind == 5 && !dict.empty()) {
      text = dict[rng.Range(dict.size())];
    } else if (kind == 6 && text.size() >= 2) {
      text.resize(rng.Range(text.size()));
    }
  }
  return text;
}

struct ScopedTempDir {
  fs::path path;
  explicit ScopedTempDir(std::string_view tag) {
    path = fs::path(testing::TempDir()) /
           (std::string{tag} + "_" + std::to_string(::getpid()));
    fs::remove_all(path);
    fs::create_directories(path);
  }
  ~ScopedTempDir() {
    std::error_code ignored;
    fs::remove_all(path, ignored);
  }
  ScopedTempDir(const ScopedTempDir&) = delete;
  ScopedTempDir& operator=(const ScopedTempDir&) = delete;
};

struct ParseOutcome {
  int parsed = 0;
  bool parser_error = false;
  unsigned int error_code = 0;
  std::string first_host;
  std::string first_origin;
  std::string first_method;
  std::string first_url;
  std::string first_query;
  bool smuggling_invariant_violated = false;
};

ParseOutcome ParseHttpStream(std::string_view raw,
                             const std::vector<std::size_t>& splits = {}) {
  uWS::HttpParser parser;
  ParseOutcome out;
  int dummy_user = 1;
  std::size_t pos = 0;
  std::vector<std::size_t> cuts = splits;
  cuts.push_back(raw.size());
  for (const std::size_t end : cuts) {
    if (end <= pos) continue;
    const std::size_t len = std::min(end, raw.size()) - pos;
    std::vector<char> buf(len + uWS::MINIMUM_HTTP_POST_PADDING, '\0');
    std::memcpy(buf.data(), raw.data() + pos, len);
    pos += len;
    const auto res = parser.consumePostPadded(
        buf.data(), static_cast<unsigned int>(len), &dummy_user, nullptr,
        [&](void* user, uWS::HttpRequest* req) -> void* {
          ++out.parsed;
          int host_count = 0;
          int cl_count = 0;
          int te_count = 0;
          for (const auto& [k, v] : *req) {
            if (k == "host") ++host_count;
            if (k == "content-length") ++cl_count;
            if (k == "transfer-encoding") ++te_count;
          }
          if (host_count != 1 || cl_count > 1 || te_count > 1 ||
              (cl_count > 0 && te_count > 0)) {
            out.smuggling_invariant_violated = true;
          }
          if (out.parsed == 1) {
            out.first_host = std::string{req->getHeader("host")};
            out.first_origin = std::string{req->getHeader("origin")};
            out.first_method = std::string{req->getCaseSensitiveMethod()};
            out.first_url = std::string{req->getUrl()};
            out.first_query = std::string{req->getQuery()};
          }
          return user;
        },
        [](void* user, std::string_view, bool) -> void* { return user; });
    if (res.second == uWS::FULLPTR) {
      out.parser_error = true;
      out.error_code = res.first;
      break;
    }
    if (res.second != &dummy_user) break;
  }
  return out;
}

struct FuzzWsContext {
  static constexpr std::uint64_t kMaxPayload = 1024;
  std::string fragment_buffer;
  unsigned int control_tip_length = 0;
  bool closed = false;
};

struct FuzzWsImpl {
  static bool setCompressed(uWS::WebSocketState<true>*, void*) { return false; }

  static void forceClose(uWS::WebSocketState<true>*, void* user,
                         std::string_view = {}) {
    static_cast<FuzzWsContext*>(user)->closed = true;
  }

  static bool refusePayloadLength(std::uint64_t length,
                                  uWS::WebSocketState<true>*, void*) {
    return FuzzWsContext::kMaxPayload < length;
  }

  static bool handleFragment(char* data, std::size_t length,
                             unsigned int remaining_bytes, int op_code,
                             bool fin, uWS::WebSocketState<true>* w_state,
                             void* user) {
    auto* ctx = static_cast<FuzzWsContext*>(user);
    if (ctx->closed) return true;
    if (op_code < 3) {
      if (!remaining_bytes && fin && ctx->fragment_buffer.empty()) {
        if (op_code == 1 &&
            !uWS::protocol::isValidUtf8(reinterpret_cast<unsigned char*>(data),
                                        length)) {
          forceClose(w_state, user, uWS::ERR_INVALID_TEXT);
          return true;
        }
      } else {
        if (ctx->fragment_buffer.empty()) {
          ctx->fragment_buffer.reserve(length + remaining_bytes);
        }
        if (refusePayloadLength(length + ctx->fragment_buffer.length(), w_state,
                                user)) {
          forceClose(w_state, user, uWS::ERR_TOO_BIG_MESSAGE);
          return true;
        }
        ctx->fragment_buffer.append(data, length);
        if (!remaining_bytes && fin) {
          if (op_code == 1 &&
              !uWS::protocol::isValidUtf8(
                  reinterpret_cast<unsigned char*>(ctx->fragment_buffer.data()),
                  ctx->fragment_buffer.length())) {
            forceClose(w_state, user, uWS::ERR_INVALID_TEXT);
            return true;
          }
          ctx->fragment_buffer.clear();
        }
      }
    } else if (!remaining_bytes && fin && ctx->control_tip_length == 0) {
      if (op_code == uWS::CLOSE) {
        const auto cf = uWS::protocol::parseClosePayload(data, length);
        char out[125];
        (void)uWS::protocol::formatClosePayload(
            out, cf.code, cf.message, std::min<std::size_t>(123, cf.length));
        ctx->closed = true;
        return true;
      }
    } else {
      ctx->fragment_buffer.append(data, length);
      ctx->control_tip_length += static_cast<unsigned int>(length);
      if (!remaining_bytes && fin) {
        char* control_buf = ctx->fragment_buffer.data() +
                            ctx->fragment_buffer.length() -
                            ctx->control_tip_length;
        if (op_code == uWS::CLOSE) {
          const auto cf = uWS::protocol::parseClosePayload(
              control_buf, ctx->control_tip_length);
          char out[125];
          (void)uWS::protocol::formatClosePayload(
              out, cf.code, cf.message, std::min<std::size_t>(123, cf.length));
          ctx->closed = true;
          return true;
        }
        ctx->fragment_buffer.resize(ctx->fragment_buffer.length() -
                                    ctx->control_tip_length);
        ctx->control_tip_length = 0;
      }
    }
    return false;
  }
};

// REQ-OPS-34, REQ-OPS-41
// Embedded NUL bytes in a URL used to pass through std::filesystem::path into
// open(), truncating the OS path at the NUL while MimeType and ContentHashed
// read the trailing suffix, allowing MIME and Cache-Control spoofing.
TEST(StaticFilesTest, RejectsNulBytesPreventingMimeAndCacheSpoofing) {
  ScopedTempDir tmp("sf_nul");
  const fs::path root = tmp.path / "webroot";
  fs::create_directories(root / "assets");
  std::ofstream(root / "index.html") << "<html>index</html>";
  std::ofstream(root / "assets" / "plain") << "unhashed";

  const std::string mime_spoof("/index.html\0.js", 15);
  const std::string cache_spoof("/assets/plain\0-12345678.js", 26);
  for (const Containment mode :
       {Containment::kCanonical, Containment::kLexical}) {
    EXPECT_EQ(ReadStaticFile(root, mime_spoof, {.containment = mode}).status,
              400);
    EXPECT_EQ(ReadStaticFile(root, cache_spoof, {.containment = mode}).status,
              400);
  }
  EXPECT_TRUE(
      LinkedOut(root, std::string_view("index.html\0extra", 16)).empty());
}

// REQ-OPS-34, REQ-OPS-41
// Static file reads enforce path containment, reject FIFOs without blocking,
// handle symlink loops and escaping symlinks under kCanonical, and cap reads at
// max_bytes.
TEST(StaticFilesTest, EnforcesContainmentFifoNonBlockingAndSizeCap) {
  ScopedTempDir tmp("sf_contain");
  const fs::path root = tmp.path / "webroot";
  const fs::path sibling = tmp.path / "webroot_sibling";
  fs::create_directories(root / "assets");
  fs::create_directories(sibling);

  std::ofstream(root / "index.html") << "ok";
  std::ofstream(root / "assets" / "index-3-nMjUae.js") << "hashed";
  std::ofstream(root / "exact.bin", std::ios::binary) << std::string(64, 'x');
  std::ofstream(root / "over.bin", std::ios::binary) << std::string(65, 'y');
  std::ofstream(tmp.path / "secret.txt") << "SECRET";
  std::ofstream(sibling / "secret.txt") << "SECRET";
  fs::create_symlink(tmp.path / "secret.txt", root / "escape.txt");
  fs::create_symlink(root / "loop_b", root / "loop_a");
  fs::create_symlink(root / "loop_a", root / "loop_b");
  ASSERT_EQ(::mkfifo((root / "pipe.txt").c_str(), 0600), 0);

  const auto hashed = ReadStaticFile(root, "/assets/index-3-nMjUae.js");
  EXPECT_EQ(hashed.status, 200);
  EXPECT_EQ(hashed.content_type, "text/javascript");
  EXPECT_EQ(hashed.cache_control, kImmutableCacheControl);

  EXPECT_EQ(ReadStaticFile(root, "/exact.bin", {.max_bytes = 64}).status, 200);
  EXPECT_EQ(ReadStaticFile(root, "/over.bin", {.max_bytes = 64}).status, 413);

  for (const std::string_view bad_404 :
       {"/../secret.txt", "/../webroot_sibling/secret.txt",
        "/assets/../../secret.txt", "//etc/passwd", "/escape.txt", "/loop_a",
        "/pipe.txt", "/assets", "/."}) {
    SCOPED_TRACE(bad_404);
    const auto res = ReadStaticFile(root, bad_404);
    EXPECT_EQ(res.status, 404);
    EXPECT_EQ(res.body.find("SECRET"), std::string::npos);
  }
  EXPECT_EQ(ReadStaticFile(root, "/%2e%2e/secret.txt").status, 400);
  EXPECT_EQ(ReadStaticFile(root, "/..\\secret.txt").status, 400);
}

// REQ-OPS-35
// DriveSummaryJson refusal truncation runs in O(max_bytes) rather than O(N^2)
// so a large refusal file (up to 1 MiB) does not stall the bridge loop, and
// truncates cleanly on multi-byte UTF-8 and JSON-escaped control characters.
TEST(DriveSummaryTest, RefusalTruncationHandlesLargePayloadAndUtf8Boundaries) {
  {
    const DriveSummary large =
        BuildRefusalDriveSummary("", std::string(65536, 'A'));
    const std::string json = DriveSummaryJson(large);
    EXPECT_LE(json.size(), talos::telemetry_wire::kDriveSummaryMaxBytes);
    const auto parsed = nlohmann::json::parse(json, nullptr, false);
    ASSERT_TRUE(parsed.is_object());
    EXPECT_TRUE(parsed["launch_refused"].get<bool>());
    const std::string reason = parsed["refusal_reason"].get<std::string>();
    EXPECT_TRUE(reason.ends_with("..."));
    EXPECT_GT(reason.size(), 3000u);
  }
  {
    // 4-byte UTF-8 code points must never be split mid-sequence.
    std::string emojis;
    for (int i = 0; i < 1500; ++i) emojis += "\xf0\x9f\x98\x80";
    const DriveSummary summary = BuildRefusalDriveSummary("", emojis);
    const std::string json = DriveSummaryJson(summary);
    EXPECT_LE(json.size(), talos::telemetry_wire::kDriveSummaryMaxBytes);
    const auto parsed = nlohmann::json::parse(json, nullptr, false);
    ASSERT_TRUE(parsed.is_object()) << json;
    EXPECT_TRUE(parsed["refusal_reason"].get<std::string>().ends_with("..."));
  }
  {
    // Control bytes expand 6x (\u0001); output must still fit max_bytes.
    const DriveSummary ctrl =
        BuildRefusalDriveSummary("", std::string(5000, '\x01'));
    const std::string json = DriveSummaryJson(ctrl);
    EXPECT_LE(json.size(), talos::telemetry_wire::kDriveSummaryMaxBytes);
    ASSERT_TRUE(nlohmann::json::parse(json, nullptr, false).is_object());
  }
}

// REQ-OPS-34, REQ-OPS-41
// Patched uWebSockets HttpParser rejects RFC 9112 smuggling duplicates
// (duplicate Host, duplicate Content-Length, empty Content-Length, empty or
// duplicate Transfer-Encoding, TE+CL, and non-hex chunk sizes) and preserves
// non-ASCII header bytes instead of silently stripping them via signed char.
TEST(UwsParserTest, RejectsHttpSmugglingAndPreservesNonAsciiHeaderBytes) {
  const std::vector<std::pair<std::string, std::string>> smuggling_cases = {
      {"missing_host", "GET / HTTP/1.1\r\nUser-Agent: x\r\n\r\n"},
      {"dup_host",
       "GET / HTTP/1.1\r\nHost: 127.0.0.1:5805\r\nHost: evil.example\r\n\r\n"},
      {"dup_content_length",
       "GET / HTTP/1.1\r\nHost: localhost\r\nContent-Length: 0\r\n"
       "Content-Length: 40\r\n\r\nGET /smuggled HTTP/1.1\r\nHost: x\r\n\r\n"},
      {"empty_content_length",
       "GET / HTTP/1.1\r\nHost: localhost\r\nContent-Length: \r\n\r\n"},
      {"empty_te_with_cl",
       "GET / HTTP/1.1\r\nHost: localhost\r\nTransfer-Encoding: \r\n"
       "Content-Length: 5\r\n\r\n12345"},
      {"empty_te_alone",
       "GET / HTTP/1.1\r\nHost: localhost\r\nTransfer-Encoding: \r\n\r\n"},
      {"dup_te",
       "GET / HTTP/1.1\r\nHost: localhost\r\nTransfer-Encoding: chunked\r\n"
       "Transfer-Encoding: chunked\r\n\r\n0\r\n\r\n"},
      {"invalid_chunk_hex_g",
       "POST / HTTP/1.1\r\nHost: localhost\r\nTransfer-Encoding: "
       "chunked\r\n\r\n"
       "g\r\n0\r\n\r\n"},
      {"invalid_chunk_hex_colon",
       "POST / HTTP/1.1\r\nHost: localhost\r\nTransfer-Encoding: "
       "chunked\r\n\r\n"
       ":\r\n0\r\n\r\n"},
  };
  for (const auto& [name, request] : smuggling_cases) {
    SCOPED_TRACE(name);
    const ParseOutcome res = ParseHttpStream(request);
    EXPECT_TRUE(res.parser_error);
    EXPECT_EQ(res.error_code, uWS::HTTP_ERROR_400_BAD_REQUEST);
    if (name.find("chunk") == std::string::npos) {
      EXPECT_EQ(res.parsed, 0);
    }
  }

  // Non-ASCII 0xFF wrapping Host/Origin must not be trimmed away.
  const std::string obs_req =
      "GET / HTTP/1.1\r\nHost: \xff"
      "127.0.0.1:5805\xff\r\nOrigin: \xffhttp://127.0.0.1:5800\xff\r\n\r\n";
  const ParseOutcome obs = ParseHttpStream(obs_req);
  ASSERT_FALSE(obs.parser_error);
  ASSERT_EQ(obs.parsed, 1);
  EXPECT_EQ(obs.first_host,
            "\xff"
            "127.0.0.1:5805\xff");
  EXPECT_EQ(obs.first_origin, "\xffhttp://127.0.0.1:5800\xff");
}

// REQ-OPS-34, REQ-OPS-35
// Patched uWebSockets avoids memcpy(dst, nullptr, 0) in >=16 KiB WebSocket
// header formatting and BackPressure::append(nullptr, 0) on partial drains.
TEST(UwsWebSocketTest, ZeroLengthFormatMessageAndBackPressureAppendHaveNoUb) {
  char header[10] = {};
  const std::size_t header_len = uWS::protocol::formatMessage<true>(
      header, nullptr, 0, uWS::BINARY, 20000, false, true);
  EXPECT_EQ(header_len, 4u);

  uWS::BackPressure backpressure;
  backpressure.append(nullptr, 0);
  EXPECT_EQ(backpressure.length(), 0u);
  backpressure.append("abc", 3);
  backpressure.append(nullptr, 0);
  EXPECT_EQ(backpressure.length(), 3u);
}

// REQ-OPS-34, REQ-OPS-41, REQ-SAFE-35
// Deterministic seeded mutation fuzzing across HTTP parsing, WebSocket frame
// parsing, and static file containment (runs well under 2 s under ASan+UBSan).
TEST(StudioParsersFuzzTest, SeededMutationFuzzStaysSafeUnderSanitizers) {
  Rng rng(0x29a057d10f022001ULL);

  // 1. Mutated HTTP streams (split across random chunks).
  const std::vector<std::string> http_seeds = {
      "GET / HTTP/1.1\r\nHost: localhost\r\n\r\n",
      "GET /telemetry HTTP/1.1\r\nHost: localhost\r\n"
      "User-Agent: Mozilla/5.0 Version/15.1 Safari/605\r\n"
      "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n",
      "POST /x HTTP/1.1\r\nHost: a\r\nTransfer-Encoding: chunked\r\n\r\n"
      "5\r\nhello\r\n0\r\n\r\n",
      "GET /a HTTP/1.1\r\nHost: 127.0.0.1:5805\r\nContent-Length: 3\r\n\r\nabc",
  };
  const std::vector<std::string> http_dict = {
      "GET ",
      "POST ",
      " HTTP/1.1\r\n",
      "\r\n\r\n",
      "Host: ",
      "Content-Length: ",
      "Transfer-Encoding: ",
      "chunked",
      "0\r\n\r\n",
      "Version/15.2 Safari/605",
      "/../",
      "/%2e%2e/",
      std::string("\0", 1),
  };
  uWS::HttpRouter<int> router;
  auto noop = [](auto*) { return true; };
  router.add({"GET"}, "/telemetry", noop, router.HIGH_PRIORITY);
  router.add({"GET"}, "/drive", noop, router.HIGH_PRIORITY);
  router.add({"GET"}, "/*", noop);

  for (int i = 0; i < 2000; ++i) {
    const std::string raw =
        Mutate(rng, http_seeds[rng.Range(http_seeds.size())], http_dict, 2048);
    std::vector<std::size_t> cuts;
    if (!raw.empty() && rng.Chance(2, 3)) {
      const std::size_t n = 1 + rng.Range(4);
      for (std::size_t c = 0; c < n; ++c)
        cuts.push_back(1 + rng.Range(raw.size()));
      std::sort(cuts.begin(), cuts.end());
    }
    const ParseOutcome res = ParseHttpStream(raw, cuts);
    EXPECT_FALSE(res.smuggling_invariant_violated);
    if (!res.parser_error && res.parsed > 0) {
      router.route(res.first_method, res.first_url);
      (void)uWS::getDecodedQueryValue("c", res.first_query);
    }
  }

  // 2. Mutated WebSocket frame streams with exact 32-byte uSockets padding.
  const std::vector<std::string> ws_seeds = {
      std::string("\x81\x82\x12\x34\x56\x78\x7a\x5d", 8),
      std::string("\x01\x83\x00\x00\x00\x00hel", 9) +
          std::string("\x89\x84\x00\x00\x00\x00ping", 10) +
          std::string("\x80\x82\x00\x00\x00\x00lo", 8),
      std::string("\x88\x84\x00\x00\x00\x00\x03\xe8ok", 10),
      std::string("\x82\xfe\x00\x7e\xaa\xbb\xcc\xdd", 8) +
          std::string(126, 'x'),
      std::string("\x82\xff\xff\xff\xff\xff\xff\xff\xff\xff\x11\x22\x33\x44",
                  14),
  };
  const std::vector<std::string> ws_dict = {
      std::string("\x81\x80\x00\x00\x00\x00", 6),
      std::string("\x88\x82\x00\x00\x00\x00\x03\xe8", 8),
      std::string("\x89\x80\x00\x00\x00\x00", 6),
      std::string("\xff\xfe\xfd\xc0\x80\xed\xa0\x80", 8),
  };
  for (int i = 0; i < 2000; ++i) {
    const std::string raw =
        Mutate(rng, ws_seeds[rng.Range(ws_seeds.size())], ws_dict, 1536);
    if (raw.size() >= 24) {
      char accept[29] = {};
      uWS::WebSocketHandshake::generate(raw.data(), accept);
    }
    uWS::WebSocketState<true> w_state;
    FuzzWsContext ctx;
    std::size_t pos = 0;
    const std::size_t chunks = 1 + rng.Range(6);
    for (std::size_t c = 0; c < chunks && pos < raw.size() && !ctx.closed;
         ++c) {
      const std::size_t remain = raw.size() - pos;
      const std::size_t take =
          (c + 1 == chunks) ? remain : (1 + rng.Range(remain));
      constexpr std::size_t kPad = 32;
      std::vector<char> buf(kPad + take + kPad, 0);
      std::memcpy(buf.data() + kPad, raw.data() + pos, take);
      pos += take;
      uWS::WebSocketProtocol<true, FuzzWsImpl>::consume(
          buf.data() + kPad, static_cast<unsigned int>(take), &w_state, &ctx);
    }
  }

  // 3. Mutated static-file URLs under both kCanonical and kLexical.
  ScopedTempDir tmp("sf_fuzz");
  const fs::path root = tmp.path / "webroot";
  fs::create_directories(root / "assets");
  std::ofstream(root / "index.html") << "OK";
  std::ofstream(root / "assets" / "app-Ab12_cd-.js") << "JS";
  std::ofstream(tmp.path / "secret.txt") << "SECRET_OUTSIDE";
  fs::create_symlink(tmp.path / "secret.txt", root / "escape.txt");
  const std::vector<std::string> url_seeds = {
      "/index.html",    "/assets/app-Ab12_cd-.js",
      "/../secret.txt", "/%2e%2e/secret.txt",
      "/escape.txt",    std::string("/index.html\0.js", 15)};
  const std::vector<std::string> url_dict = {"/",
                                             "//",
                                             "..",
                                             "../",
                                             "%2e",
                                             "\\",
                                             std::string("\0", 1),
                                             "index.html",
                                             "assets/",
                                             "secret.txt",
                                             "escape.txt"};
  for (int i = 0; i < 1000; ++i) {
    const std::string url =
        Mutate(rng, url_seeds[rng.Range(url_seeds.size())], url_dict, 256);
    const auto canonical = ReadStaticFile(root, url);
    if (canonical.status == 200) {
      EXPECT_EQ(canonical.body.find("SECRET_OUTSIDE"), std::string::npos);
      EXPECT_EQ(url.find('\0'), std::string::npos);
      EXPECT_EQ(url.find('%'), std::string::npos);
      EXPECT_EQ(url.find('\\'), std::string::npos);
    }
    const auto lexical =
        ReadStaticFile(root, url, {.containment = Containment::kLexical});
    if (lexical.status == 200) {
      EXPECT_EQ(url.find('\0'), std::string::npos);
      EXPECT_EQ(url.find('%'), std::string::npos);
      EXPECT_EQ(url.find('\\'), std::string::npos);
    }
  }
}

}  // namespace
}  // namespace studio

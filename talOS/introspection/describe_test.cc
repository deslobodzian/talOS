#include "talOS/introspection/describe.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/callback.h"
#include "talOS/events/context.h"
#include "talOS/events/manifest.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/introspection/names.h"
#include "talOS/schema/identity.h"

namespace talos::introspect {
namespace {

constexpr std::uint32_t kSlots = 1024;

event::Manifest sample_manifest() {
  event::Manifest manifest;
  manifest.push_back({.id = 0,
                      .kind = event::SourceKind::TIMER,
                      .name = "swerve",
                      .period_ns = 5'000'000});
  manifest.push_back(
      {.id = 1,
       .kind = event::SourceKind::WATCHER,
       .name = "/drivetrain/target",
       .message_bytes = 48,
       .alignment = 8,
       .schema = {"drivetrain.ChassisTarget", 0xfedcba9876543210ULL}});
  manifest.push_back({.id = 2,
                      .kind = event::SourceKind::FETCHER,
                      .name = "/driver_station/state",
                      .message_bytes = 32,
                      .alignment = 4,
                      .schema = {"talos.hardware.Packet", 0x1ULL}});
  manifest.push_back({.id = 3,
                      .kind = event::SourceKind::SENDER,
                      .name = "/hw/request/drivetrain",
                      .message_bytes = 256,
                      .alignment = 8,
                      .schema = {"talos.TargetEnvelope<a.B>", 0x8000ULL}});
  return manifest;
}

// The round trip is the whole contract: the launcher reads what a node wrote,
// so a field the writer emits and the reader ignores is a topic the graph does
// not know about.
void ExpectSameDescription(const Description& written,
                           const Description& read) {
  EXPECT_EQ(read.name, written.name);
  EXPECT_EQ(read.target, written.target);
  EXPECT_EQ(read.reads_graph, written.reads_graph);
  EXPECT_EQ(read.holds_stop, written.holds_stop);
  EXPECT_EQ(read.abi_update, written.abi_update);
  EXPECT_EQ(read.params, written.params);
  ASSERT_EQ(read.sources.size(), written.sources.size());
  for (std::size_t i = 0; i < written.sources.size(); ++i) {
    const auto& a = written.sources[i];
    const auto& b = read.sources[i];
    EXPECT_EQ(b.kind, a.kind) << "source " << i;
    EXPECT_EQ(b.name, a.name) << "source " << i;
    EXPECT_EQ(b.message_bytes, a.message_bytes) << "source " << i;
    EXPECT_EQ(b.flags, a.flags) << "source " << i;
    EXPECT_EQ(b.schema, a.schema) << "source " << i;
    EXPECT_EQ(b.schema_hash, a.schema_hash) << "source " << i;
    EXPECT_EQ(b.alignment, a.alignment) << "source " << i;
    EXPECT_EQ(b.slots, a.slots) << "source " << i;
    EXPECT_EQ(b.bfbs, a.bfbs) << "source " << i;
  }
}

// Every source kind, endpoint flags looked up by key (a missing key must not
// shift onto the next source), schema bytes at every base64 padding length,
// the full 64-bit hash, names that need escaping, and parameter keys holding
// the brackets, braces and field names a scanning reader would stop at.
// REQ-SAFE-42: the hardware request's untargeted_outputs marking survives.
// REQ-SAFE-14: operator_arming and internal_driver_station markings survive.
TEST(Describe, RoundTripsEveryField) {
  event::Manifest manifest = sample_manifest();
  std::string bytes;
  for (int i = 0; i < 256; ++i) bytes += static_cast<char>(i);
  manifest[1].schema.bfbs = bytes;               // 256 bytes: two '='
  manifest[2].schema.bfbs = bytes.substr(0, 5);  // one '='
  manifest[3].schema.bfbs = bytes.substr(7, 3);  // none
  manifest.push_back({.id = 4,
                      .kind = event::SourceKind::SENDER,
                      .name = R"(/odd/state/"quoted\path)",
                      .message_bytes = 8,
                      .schema = {R"(odd."quoted)", 2}});
  Description written = DescribeManifest(
      R"(od"d)", R"(//example/od\d:node)", manifest, kSlots,
      {{"/hw/request/drivetrain", naming::kSourceFlagExternal |
                                      naming::kSourceFlagOptional |
                                      naming::kSourceFlagHardwareRequest |
                                      naming::kSourceFlagUntargetedOutputs},
       {"/drivetrain/target", naming::kSourceFlagOptional},
       {"/driver_station/state",
        naming::kSourceFlagObserver | naming::kSourceFlagInternalDriverStation},
       {R"(/odd/state/"quoted\path)", naming::kSourceFlagOperatorArming}});
  written.reads_graph = true;
  written.holds_stop = true;
  written.abi_update = true;
  written.params = {{"drivetrain", "geometry.modules[0].x_m"},
                    {R"(od"d)", "sources"},
                    {"describe_version", R"(odd]}{["key)"}};
  ASSERT_EQ(written.sources.size(), 5u);
  ASSERT_EQ(written.sources[3].flags, naming::kSourceFlagExternal |
                                          naming::kSourceFlagOptional |
                                          naming::kSourceFlagHardwareRequest |
                                          naming::kSourceFlagUntargetedOutputs);
  ASSERT_EQ(
      written.sources[2].flags,
      naming::kSourceFlagObserver | naming::kSourceFlagInternalDriverStation);
  ASSERT_EQ(written.sources[4].flags, naming::kSourceFlagOperatorArming);
  ASSERT_EQ(written.sources[3].slots, kSlots);
  ASSERT_EQ(written.sources[1].bfbs, bytes);

  Description read;
  std::string error;
  ASSERT_TRUE(DescribeReader::Parse(DescribeToJson(written), read, error))
      << error;
  ExpectSameDescription(written, read);
  EXPECT_EQ(read.sources[1].schema_hash, 0xfedcba9876543210ULL);
}

TEST(Describe, DeclaredGraphRoundTripsEveryNode) {
  event::Manifest manifest = sample_manifest();
  manifest[1].schema.bfbs = std::string{"\x00\x01{}[]\"", 7};
  DeclaredGraph written{
      DescribeManifest("drivetrain", "//d:drivetrain", manifest, kSlots),
      DescribeManifest("empty", "//e:empty", event::Manifest{}, kSlots),
      DescribeManifest("shooter", "//s:shooter", sample_manifest(), kSlots)};
  // Reported, reported as none, and not reported (nullopt) stay distinct.
  written[0].params =
      std::vector<ParamRead>{ParamRead{"describe_version", "describe_version"}};
  written[1].params.emplace();
  DeclaredGraph read;
  std::string error;
  ASSERT_TRUE(
      DescribeReader::ParseGraph(DeclaredGraphToJson(written), read, error))
      << error;
  ASSERT_EQ(read.size(), written.size());
  for (std::size_t i = 0; i < written.size(); ++i) {
    ExpectSameDescription(written[i], read[i]);
  }

  ASSERT_TRUE(DescribeReader::ParseGraph(DeclaredGraphToJson({}), read, error))
      << error;
  EXPECT_TRUE(read.empty());
  EXPECT_FALSE(DescribeReader::ParseGraph("not json", read, error));
}

TEST(Describe, WritesTheKindNamesTheRegistryUses) {
  // Not a formatting preference: the registry and the log print kinds with
  // event::to_string, and a viewer joining declared to running matches on them.
  const std::string json = DescribeToJson(DescribeManifest(
      "drivetrain", "//x:drivetrain", sample_manifest(), kSlots));
  EXPECT_NE(json.find("\"kind\": \"TIMER\""), std::string::npos);
  EXPECT_NE(json.find("\"kind\": \"WATCHER\""), std::string::npos);
  EXPECT_NE(json.find("\"kind\": \"FETCHER\""), std::string::npos);
  EXPECT_NE(json.find("\"kind\": \"SENDER\""), std::string::npos);
  EXPECT_NE(json.find("\"describe_version\": 1"), std::string::npos);
}

TEST(Describe, ReadsThroughOutputPrintedAheadOfTheJson) {
  // The launcher reads a whole child stdout, and a node may have printed
  // something before main reached --describe. Tolerating a banner is not
  // sloppiness: the alternative is a launch that reports a node as
  // undescribable because a library it links said hello.
  const Description written =
      DescribeManifest("odometry", "//2026-robot/main_processor/odometry:node",
                       sample_manifest(), kSlots);

  Description read;
  std::string error;
  ASSERT_TRUE(DescribeReader::Parse(
      "flatbuffers: verifier enabled\n" + DescribeToJson(written), read, error))
      << error;
  ExpectSameDescription(written, read);
}

// The launcher's input is a child process's stdout, which may be anything at
// all, and the answer must be a false with a message -- never a throw, and
// never a true with a partial or empty graph, which would read as "this node
// has these topics" and pass the lint.
TEST(Describe, RejectsOutputThatIsNotACompleteDescription) {
  const std::string valid =
      DescribeToJson(DescribeManifest("d", "//d:d", sample_manifest(), kSlots));
  const auto replaced = [&valid](std::string_view from, std::string_view to) {
    std::string json = valid;
    const std::size_t at = json.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    if (at != std::string::npos) json.replace(at, from.size(), to);
    return json;
  };
  event::Manifest unschematized = sample_manifest();
  unschematized[3].schema = {};
  struct Case {
    const char* what;
    std::string output;
  };
  for (const auto& [what, output] : std::vector<Case>{
           {"usage message",
            "usage: node [--sim] [--duration-s N] [--log PATH]\n"},
           {"stack trace",
            "libc++abi: terminating due to uncaught exception of type "
            "std::runtime_error: topic '/hw/state' used with two different "
            "message sizes\n0   node   0x0000000102d3c1f8 main + 132\n"},
           {"empty", ""},
           {"no node identity",
            "{\n  \"describe_version\": 1,\n  \"sources\": []\n}\n"},
           {"unreadable version",
            replaced("\"describe_version\": 1", "\"describe_version\": 2")},
           // A node killed mid-print starts out well-formed.
           {"unterminated sources",
            valid.substr(0, valid.find("\"/driver_station/state\""))},
           {"unknown source kind", replaced("\"WATCHER\"", "\"LISTENER\"")},
           {"malformed schema bytes",
            replaced("\"bfbs\": \"\"", "\"bfbs\": \"abc\"")},
           {"topic without a schema",
            DescribeToJson(
                DescribeManifest("d", "//d:d", unschematized, kSlots))},
       }) {
    Description out;
    std::string error;
    EXPECT_FALSE(DescribeReader::Parse(output, out, error)) << what;
    EXPECT_FALSE(error.empty()) << what;
  }
}

// REQ-OPS-32: the launcher sizes a member's log share from its describe, and
// a latest-only watcher logs at most one message per drain where a sequence
// watcher logs every message. --describe must therefore say which is which,
// as the loop decided it (an empty callback, or latest_only asked for).
TEST(Describe, MarksExactlyTheLoopsLatestOnlyWatchers) {
  event::SimulationEnvironment environment;
  event::SimulatedEventLoop<> loop{environment};
  const schema::Identity schema{"test.Count", 7};
  const event::Thunk callback{
      nullptr, [](void*, const event::Context&, std::span<const std::byte>) {}};
  const auto sequence =
      loop.register_watcher("/a/state", 8, 8, schema, callback);
  const auto empty = loop.register_watcher("/b/state", 8, 8, schema, {});
  const auto asked =
      loop.register_watcher("/c/state", 8, 8, schema, callback, true);
  const auto sender = loop.register_sender("/d/state", 8, 8, schema);

  const Description written = DescribeManifest(
      "n", "//n:n", loop.manifest(), kSlots, {}, LoopSourceFlags(loop));
  Description read;
  std::string error;
  ASSERT_TRUE(DescribeReader::Parse(DescribeToJson(written), read, error))
      << error;
  ASSERT_EQ(read.sources.size(), 4u);
  EXPECT_EQ(read.sources[sequence].flags & naming::kSourceFlagLatestOnly, 0u);
  EXPECT_EQ(read.sources[empty].flags, naming::kSourceFlagLatestOnly);
  EXPECT_EQ(read.sources[asked].flags, naming::kSourceFlagLatestOnly);
  EXPECT_EQ(read.sources[sender].flags, 0u);
}

// REQ-OPS-01: hostile `--describe` JSON with overflowing or out-of-range
// integers must fail closed with no signed integer overflow UB.
TEST(Describe, RejectsOverflowingAndOutOfRangeNumbers) {
  const std::string valid =
      DescribeToJson(DescribeManifest("d", "//d:d", sample_manifest(), kSlots));
  const auto replaced = [&valid](std::string_view from, std::string_view to) {
    std::string json = valid;
    const std::size_t at = json.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    if (at != std::string::npos) json.replace(at, from.size(), to);
    return json;
  };

  // INT64_MIN and INT64_MAX parse cleanly without overflow and are reported as
  // unexpected describe_version values.
  for (const char* boundary : {"-9223372036854775808", "9223372036854775807"}) {
    Description out;
    std::string error;
    EXPECT_FALSE(DescribeReader::Parse(
        replaced("\"describe_version\": 1",
                 std::string{"\"describe_version\": "} + boundary),
        out, error))
        << boundary;
    EXPECT_NE(error.find(boundary), std::string::npos) << error;
  }

  // Overflowing, negative, non-numeric, or out-of-uint32-range fields fail
  // closed.
  struct Case {
    const char* what;
    std::string output;
  };
  for (const auto& [what, output] : std::vector<Case>{
           {"describe_version > INT64_MAX",
            replaced("\"describe_version\": 1",
                     "\"describe_version\": 9223372036854775808")},
           {"describe_version < INT64_MIN",
            replaced("\"describe_version\": 1",
                     "\"describe_version\": -9223372036854775809")},
           {"describe_version huge",
            replaced("\"describe_version\": 1",
                     "\"describe_version\": 99999999999999999999999999")},
           {"describe_version lone minus",
            replaced("\"describe_version\": 1", "\"describe_version\": -")},
           {"describe_version string",
            replaced("\"describe_version\": 1", "\"describe_version\": \"1\"")},
           {"describe_version trailing garbage",
            replaced("\"describe_version\": 1", "\"describe_version\": 1abc")},
           {"message_bytes huge",
            replaced("\"message_bytes\": 48",
                     "\"message_bytes\": 99999999999999999999999999")},
           {"message_bytes INT64_MIN",
            replaced("\"message_bytes\": 48",
                     "\"message_bytes\": -9223372036854775808")},
           {"message_bytes negative",
            replaced("\"message_bytes\": 48", "\"message_bytes\": -1")},
           {"message_bytes > UINT32_MAX",
            replaced("\"message_bytes\": 48", "\"message_bytes\": 4294967296")},
           {"message_bytes non-numeric",
            replaced("\"message_bytes\": 48", "\"message_bytes\": \"48\"")},
           {"alignment huge",
            replaced("\"alignment\": 8",
                     "\"alignment\": 99999999999999999999999999")},
           {"alignment negative",
            replaced("\"alignment\": 8", "\"alignment\": -1")},
           {"alignment > UINT32_MAX",
            replaced("\"alignment\": 8", "\"alignment\": 4294967296")},
           {"slots huge", replaced("\"slots\": 1024",
                                   "\"slots\": 99999999999999999999999999")},
           {"slots INT64_MIN",
            replaced("\"slots\": 1024", "\"slots\": -9223372036854775808")},
           {"slots negative", replaced("\"slots\": 1024", "\"slots\": -1")},
           {"slots > UINT32_MAX",
            replaced("\"slots\": 1024", "\"slots\": 4294967296")},
       }) {
    Description out;
    std::string error;
    EXPECT_FALSE(DescribeReader::Parse(output, out, error)) << what;
    EXPECT_FALSE(error.empty()) << what;
  }

  // UINT32_MAX is still accepted for uint32_t fields.
  Description out;
  std::string error;
  ASSERT_TRUE(DescribeReader::Parse(
      replaced("\"message_bytes\": 48", "\"message_bytes\": 4294967295"), out,
      error))
      << error;
  ASSERT_GE(out.sources.size(), 2u);
  EXPECT_EQ(out.sources[1].message_bytes, 4294967295u);
}

// REQ-OPS-01: AppendJsonString must validate RFC 3629 UTF-8 and escape invalid
// bytes >= 0x80 as \u00XX so `--describe` and `declared_graph.json` stay valid
// UTF-8 JSON.
TEST(Describe, AppendJsonStringEscapesInvalidUtf8AndPreservesValidUtf8) {
  const auto encode = [](std::string_view input) {
    std::string out;
    AppendJsonString(out, input);
    return out;
  };

  // Valid 2-, 3-, and 4-byte UTF-8 sequences pass through unchanged.
  EXPECT_EQ(encode("a\xC3\xA9"), "\"a\xC3\xA9\"");
  EXPECT_EQ(encode("\xE2\x80\x94"), "\"\xE2\x80\x94\"");
  EXPECT_EQ(encode("\xF0\x9F\x9A\x80"), "\"\xF0\x9F\x9A\x80\"");

  // Invalid UTF-8 bytes (lone continuations, overlongs, UTF-16 surrogates,
  // > U+10FFFF, invalid lead bytes, truncated sequences) are escaped as \u00xx.
  EXPECT_EQ(encode(std::string_view{"ok\xFF\xFE", 4}), "\"ok\\u00ff\\u00fe\"");
  EXPECT_EQ(encode(std::string_view{"\x80\xBF", 2}), "\"\\u0080\\u00bf\"");
  EXPECT_EQ(encode(std::string_view{"\xC0\xAF", 2}), "\"\\u00c0\\u00af\"");
  EXPECT_EQ(encode(std::string_view{"\xC1\xBF", 2}), "\"\\u00c1\\u00bf\"");
  EXPECT_EQ(encode(std::string_view{"\xE0\x80\xAF", 3}),
            "\"\\u00e0\\u0080\\u00af\"");
  EXPECT_EQ(encode(std::string_view{"\xED\xA0\x80", 3}),
            "\"\\u00ed\\u00a0\\u0080\"");
  EXPECT_EQ(encode(std::string_view{"\xF0\x80\x80\x80", 4}),
            "\"\\u00f0\\u0080\\u0080\\u0080\"");
  EXPECT_EQ(encode(std::string_view{"\xF4\x90\x80\x80", 4}),
            "\"\\u00f4\\u0090\\u0080\\u0080\"");
  EXPECT_EQ(encode(std::string_view{"\xF5", 1}), "\"\\u00f5\"");
  EXPECT_EQ(encode(std::string_view{"\xC3", 1}), "\"\\u00c3\"");
  EXPECT_EQ(encode(std::string_view{"\xE2\x80", 2}), "\"\\u00e2\\u0080\"");
  EXPECT_EQ(encode(std::string_view{"\xF0\x9F\x9A", 3}),
            "\"\\u00f0\\u009f\\u009a\"");
  // Invalid byte immediately followed by valid UTF-8 escapes only the bad byte.
  EXPECT_EQ(encode(std::string_view{"\xFF\xC3\xA9", 3}), "\"\\u00ff\xC3\xA9\"");

  // DescribeToJson uses AppendJsonString and emits no raw invalid UTF-8 bytes.
  Description desc;
  desc.name = std::string{"bad\xFFname", 8};
  desc.target = "//x:bad";
  const std::string json = DescribeToJson(desc);
  EXPECT_NE(json.find("\"bad\\u00ffname\""), std::string::npos) << json;
  EXPECT_EQ(json.find('\xFF'), std::string::npos);
}

}  // namespace
}  // namespace talos::introspect

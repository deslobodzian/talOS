#include "talOS/introspection/describe.h"

#include <gtest/gtest.h>

#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/introspection/names.h"

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
                                      naming::kSourceFlagHardwareRequest},
       {"/drivetrain/target", naming::kSourceFlagOptional},
       {"/driver_station/state", naming::kSourceFlagObserver}});
  written.reads_graph = true;
  written.params = {{"drivetrain", "geometry.modules[0].x_m"},
                    {R"(od"d)", "sources"},
                    {"describe_version", R"(odd]}{["key)"}};
  ASSERT_EQ(written.sources.size(), 5u);
  ASSERT_EQ(written.sources[3].flags, naming::kSourceFlagExternal |
                                          naming::kSourceFlagOptional |
                                          naming::kSourceFlagHardwareRequest);
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

}  // namespace
}  // namespace talos::introspect

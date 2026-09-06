#include "talOS/introspection/describe.h"

#include <gtest/gtest.h>

#include <string>

#include "talOS/events/manifest.h"
#include "talOS/introspection/names.h"

namespace talos::introspect {
namespace {

event::Manifest sample_manifest() {
  event::Manifest manifest;
  manifest.push_back({.id = 0,
                      .kind = event::SourceKind::TIMER,
                      .name = "swerve",
                      .period_ns = 5'000'000});
  manifest.push_back({.id = 1,
                      .kind = event::SourceKind::WATCHER,
                      .name = "/drivetrain/target",
                      .message_bytes = 48,
                      .alignment = 8});
  manifest.push_back({.id = 2,
                      .kind = event::SourceKind::FETCHER,
                      .name = "/driver_station/state",
                      .message_bytes = 32,
                      .alignment = 8});
  manifest.push_back({.id = 3,
                      .kind = event::SourceKind::SENDER,
                      .name = "/hw/request/drivetrain",
                      .message_bytes = 256,
                      .alignment = 8});
  return manifest;
}

// The round trip is the whole contract: the launcher reads what a node wrote,
// so a field the writer emits and the reader ignores is a topic the graph does
// not know about.
void ExpectSameDescription(const Description& written,
                           const Description& read) {
  EXPECT_EQ(read.name, written.name);
  EXPECT_EQ(read.target, written.target);
  ASSERT_EQ(read.sources.size(), written.sources.size());
  for (std::size_t i = 0; i < written.sources.size(); ++i) {
    const auto& a = written.sources[i];
    const auto& b = read.sources[i];
    EXPECT_EQ(b.kind, a.kind) << "source " << i;
    EXPECT_EQ(b.name, a.name) << "source " << i;
    EXPECT_EQ(b.message_bytes, a.message_bytes) << "source " << i;
    EXPECT_EQ(b.external(), a.external()) << "source " << i;
    EXPECT_EQ(b.optional(), a.optional()) << "source " << i;
    EXPECT_EQ(b.flags, a.flags) << "source " << i;
  }
}

TEST(Describe, RoundTripsEverySourceKind) {
  const Description written = DescribeManifest(
      "drivetrain", "//2026-robot/main_processor/drivetrain:node",
      sample_manifest());

  Description read;
  std::string error;
  ASSERT_TRUE(DescribeReader::Parse(DescribeToJson(written), read, error))
      << error;
  EXPECT_TRUE(error.empty());
  ExpectSameDescription(written, read);
}

TEST(Describe, RoundTripsEndpointFlags) {
  // Both flags on one topic, one flag on another, none on the rest: the reader
  // looks each up by key, so a missing key must not shift the answer onto the
  // next source.
  const Description written = DescribeManifest(
      "arbiter", "//2026-robot/main_processor/arbiter:node", sample_manifest(),
      {{"/hw/request/drivetrain",
        naming::kSourceFlagExternal | naming::kSourceFlagOptional},
       {"/drivetrain/target", naming::kSourceFlagOptional}});

  ASSERT_EQ(written.sources.size(), 4u);
  EXPECT_TRUE(written.sources[1].optional());
  EXPECT_FALSE(written.sources[1].external());
  EXPECT_TRUE(written.sources[3].external());
  EXPECT_TRUE(written.sources[3].optional());

  Description read;
  std::string error;
  ASSERT_TRUE(DescribeReader::Parse(DescribeToJson(written), read, error))
      << error;
  ExpectSameDescription(written, read);
}

TEST(Describe, RoundTripsANodeThatRegisteredNothing) {
  const Description written =
      DescribeManifest("empty", "//example/empty:node", event::Manifest{});

  Description read;
  std::string error;
  ASSERT_TRUE(DescribeReader::Parse(DescribeToJson(written), read, error))
      << error;
  EXPECT_TRUE(read.sources.empty());
  ExpectSameDescription(written, read);
}

TEST(Describe, RoundTripsNamesThatNeedEscaping) {
  event::Manifest manifest;
  manifest.push_back({.id = 0,
                      .kind = event::SourceKind::SENDER,
                      .name = R"(/odd/state/"quoted\path)",
                      .message_bytes = 8});
  const Description written =
      DescribeManifest(R"(od"d)", R"(//example/od\d:node)", manifest);

  Description read;
  std::string error;
  ASSERT_TRUE(DescribeReader::Parse(DescribeToJson(written), read, error))
      << error;
  ExpectSameDescription(written, read);
}

TEST(Describe, WritesTheKindNamesTheRegistryUses) {
  // Not a formatting preference: the registry and the log print kinds with
  // event::to_string, and a viewer joining declared to running matches on them.
  const std::string json = DescribeToJson(
      DescribeManifest("drivetrain", "//x:drivetrain", sample_manifest()));
  EXPECT_NE(json.find("\"kind\": \"TIMER\""), std::string::npos);
  EXPECT_NE(json.find("\"kind\": \"WATCHER\""), std::string::npos);
  EXPECT_NE(json.find("\"kind\": \"FETCHER\""), std::string::npos);
  EXPECT_NE(json.find("\"kind\": \"SENDER\""), std::string::npos);
  EXPECT_NE(json.find("\"describe_version\": 1"), std::string::npos);
}

// Everything below is a launcher failure path. The launcher's input is a child
// process's stdout, which may be anything at all, and the answer must be a
// false with a message a person can act on -- never a throw, and never a true
// with an empty graph, which would read as "this node has no topics" and pass
// the lint.

TEST(Describe, RejectsAUsageMessage) {
  Description out;
  std::string error;
  EXPECT_FALSE(DescribeReader::Parse(
      "usage: node [--sim] [--duration-s N] [--log PATH]\n", out, error));
  EXPECT_NE(error.find("did not answer --describe"), std::string::npos);
  EXPECT_TRUE(out.sources.empty());
}

TEST(Describe, RejectsAStackTrace) {
  Description out;
  std::string error;
  EXPECT_FALSE(DescribeReader::Parse(
      "libc++abi: terminating due to uncaught exception of type "
      "std::runtime_error: topic '/hw/state' used with two different message "
      "sizes\n0   node   0x0000000102d3c1f8 main + 132\n",
      out, error));
  EXPECT_FALSE(error.empty());
}

TEST(Describe, RejectsEmptyOutput) {
  Description out;
  std::string error;
  EXPECT_FALSE(DescribeReader::Parse("", out, error));
  EXPECT_FALSE(error.empty());
}

TEST(Describe, RejectsAVersionItCannotRead) {
  std::string json = DescribeToJson(
      DescribeManifest("odometry", "//x:odometry", sample_manifest()));
  const std::size_t at = json.find("\"describe_version\": 1");
  ASSERT_NE(at, std::string::npos);
  json.replace(at, 21, "\"describe_version\": 2");

  Description out;
  std::string error;
  EXPECT_FALSE(DescribeReader::Parse(json, out, error));
  EXPECT_NE(error.find("describe_version 2"), std::string::npos);
  EXPECT_TRUE(out.sources.empty());
}

TEST(Describe, RejectsAnUnterminatedSourcesArray) {
  // A node killed mid-print: the JSON starts out well-formed, so a reader that
  // only checked the prefix would report a truncated graph as a complete one.
  std::string json = DescribeToJson(
      DescribeManifest("odometry", "//x:odometry", sample_manifest()));
  json.resize(json.find("\"/driver_station/state\""));

  Description out;
  std::string error;
  EXPECT_FALSE(DescribeReader::Parse(json, out, error));
  EXPECT_NE(error.find("not terminated"), std::string::npos);
}

TEST(Describe, RejectsAnUnknownSourceKind) {
  std::string json = DescribeToJson(
      DescribeManifest("odometry", "//x:odometry", sample_manifest()));
  const std::size_t at = json.find("\"WATCHER\"");
  ASSERT_NE(at, std::string::npos);
  json.replace(at, 9, "\"LISTENER\"");

  Description out;
  std::string error;
  EXPECT_FALSE(DescribeReader::Parse(json, out, error));
  EXPECT_NE(error.find("LISTENER"), std::string::npos);
}

TEST(Describe, ReadsThroughOutputPrintedAheadOfTheJson) {
  // The launcher reads a whole child stdout, and a node may have printed
  // something before main reached --describe. Tolerating a banner is not
  // sloppiness: the alternative is a launch that reports a node as
  // undescribable because a library it links said hello.
  const Description written =
      DescribeManifest("odometry", "//2026-robot/main_processor/odometry:node",
                       sample_manifest());

  Description read;
  std::string error;
  ASSERT_TRUE(DescribeReader::Parse(
      "flatbuffers: verifier enabled\n" + DescribeToJson(written), read, error))
      << error;
  ExpectSameDescription(written, read);
}

TEST(Describe, RejectsOutputWithNoNodeIdentity) {
  Description out;
  std::string error;
  EXPECT_FALSE(DescribeReader::Parse(
      "{\n  \"describe_version\": 1,\n  \"sources\": []\n}\n", out, error));
  EXPECT_NE(error.find("name"), std::string::npos);
}

}  // namespace
}  // namespace talos::introspect

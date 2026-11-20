#include "talOS/launcher/session_retention.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <string>

#include "talOS/events/session_manifest.h"
#include "talOS/launcher/session.h"

namespace talos::launcher {
namespace {

namespace fs = std::filesystem;
namespace manifest = event::session_manifest;

std::string TempDir(const std::string& name) {
  const fs::path dir =
      fs::path{::testing::TempDir()} /
      ("talos_session_retention_" + name + "_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir.string();
}

void WriteManifest(const fs::path& dir, const std::string& content) {
  std::ofstream{dir / manifest::kFileName} << content;
}

TEST(SessionManifestTest, ParsesRetentionFieldsAcrossFormattingAndTypes) {
  SessionManifest written;
  written.session_id = 42;
  written.simulation = true;
  written.start_wall_ns = 1'700'000'000'123'456'789LL;
  written.end_wall_ns = 1'700'000'010'000'000'000LL;
  const manifest::Fields round_trip = manifest::Parse(written.ToJson());
  EXPECT_EQ(round_trip.session_id, 42U);
  EXPECT_EQ(round_trip.simulation, true);
  EXPECT_EQ(round_trip.start_wall_ns, 1'700'000'000'123'456'789ULL);
  EXPECT_EQ(round_trip.end_wall_ns, 1'700'000'010'000'000'000ULL);

  // Compact formatting and nested `runs` fields before top-level fields:
  // nested objects must not shadow top-level keys.
  const manifest::Fields compact = manifest::Parse(
      "{\"nodes\":[{\"runs\":[{\"start_wall_ns\":111,\"end_wall_ns\":222}]}],"
      "\"simulation\":false,\"start_wall_ns\":18446744073709551615,"
      "\"end_wall_ns\":0}");
  EXPECT_EQ(compact.simulation, false);
  EXPECT_EQ(compact.start_wall_ns, std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(compact.end_wall_ns, 0ULL);

  // Missing keys and nested keys alone leave top-level fields unset.
  const manifest::Fields missing = manifest::Parse(
      "{\"nodes\":[{\"runs\":[{\"start_wall_ns\":111,\"end_wall_ns\":222}]}]"
      "}");
  EXPECT_EQ(missing.simulation, std::nullopt);
  EXPECT_EQ(missing.start_wall_ns, std::nullopt);
  EXPECT_EQ(missing.end_wall_ns, std::nullopt);

  // Wrong JSON types skip cleanly without throwing.
  const manifest::Fields invalid_types = manifest::Parse(
      "{\"simulation\":\"true\",\"start_wall_ns\":-5,"
      "\"end_wall_ns\":18446744073709551616}");
  EXPECT_EQ(invalid_types.simulation, std::nullopt);
  EXPECT_EQ(invalid_types.start_wall_ns, std::nullopt);
  EXPECT_EQ(invalid_types.end_wall_ns, std::nullopt);

  const manifest::Fields non_numeric = manifest::Parse(
      "{\"simulation\":1,\"start_wall_ns\":1.5,\"end_wall_ns\":null}");
  EXPECT_EQ(non_numeric.simulation, std::nullopt);
  EXPECT_EQ(non_numeric.start_wall_ns, std::nullopt);
  EXPECT_EQ(non_numeric.end_wall_ns, std::nullopt);

  EXPECT_THROW(manifest::Parse("{\"simulation\": tru}"),
               manifest::ManifestError);
  EXPECT_THROW(manifest::Parse("{broken"), manifest::ManifestError);
}

TEST(SessionRetentionTest, HandlesMissingCorruptPartialAndCompactManifests) {
  const fs::path root = TempDir("parity");

  // Missing manifest.json reads as absent without scanning or writing cache.
  const fs::path missing = root / "missing";
  fs::create_directories(missing);
  const SessionFacts missing_facts = InspectSession(missing);
  EXPECT_EQ(missing_facts.start_wall_ns, 0);
  EXPECT_FALSE(missing_facts.finished);
  EXPECT_EQ(missing_facts.verdict, MatchVerdict::kOther);
  EXPECT_FALSE(fs::exists(missing / std::string{kSessionClassFile}));
  EXPECT_EQ(SessionSimulation(missing), std::nullopt);

  // Corrupt manifest never throws; falls back to empty facts.
  const fs::path corrupt = root / "corrupt";
  fs::create_directories(corrupt);
  WriteManifest(corrupt, "{not valid json");
  const SessionFacts corrupt_facts =
      InspectSession(corrupt, ClassCache::kReadOnly);
  EXPECT_EQ(corrupt_facts.start_wall_ns, 0);
  EXPECT_FALSE(corrupt_facts.finished);
  EXPECT_EQ(corrupt_facts.verdict, MatchVerdict::kOther);
  EXPECT_EQ(SessionSimulation(corrupt), std::nullopt);

  // Invalid field types (out-of-int64-range start_wall_ns, string simulation,
  // null end_wall_ns) do not throw and keep safe defaults.
  const fs::path bad_types = root / "bad_types";
  fs::create_directories(bad_types);
  WriteManifest(bad_types,
                "{\"start_wall_ns\":9223372036854775808,"
                "\"end_wall_ns\":null,\"simulation\":\"true\"}");
  const SessionFacts bad_facts =
      InspectSession(bad_types, ClassCache::kReadOnly);
  EXPECT_EQ(bad_facts.start_wall_ns, 0);
  EXPECT_FALSE(bad_facts.finished);
  EXPECT_EQ(SessionSimulation(bad_types), std::nullopt);

  // Compact whitespace-free JSON and nonzero end_wall_ns / simulation=true.
  const fs::path compact_sim = root / "compact_sim";
  fs::create_directories(compact_sim);
  WriteManifest(compact_sim,
                "{\"start_wall_ns\":123456,\"end_wall_ns\":789,"
                "\"simulation\":true}");
  const SessionFacts sim_facts = InspectSession(compact_sim);
  EXPECT_EQ(sim_facts.start_wall_ns, 123456);
  EXPECT_TRUE(sim_facts.finished);
  EXPECT_EQ(sim_facts.verdict, MatchVerdict::kOther);
  EXPECT_FALSE(fs::exists(compact_sim / std::string{kSessionClassFile}));
  EXPECT_EQ(SessionSimulation(compact_sim), true);

  // simulation=false with end_wall_ns=0.
  const fs::path running = root / "running";
  fs::create_directories(running);
  WriteManifest(running,
                "{\"start_wall_ns\" : 555 , \"end_wall_ns\" : 0 , "
                "\"simulation\" : false}");
  const SessionFacts running_facts =
      InspectSession(running, ClassCache::kReadOnly);
  EXPECT_EQ(running_facts.start_wall_ns, 555);
  EXPECT_FALSE(running_facts.finished);
  EXPECT_EQ(SessionSimulation(running), false);

  fs::remove_all(root);
}

}  // namespace
}  // namespace talos::launcher

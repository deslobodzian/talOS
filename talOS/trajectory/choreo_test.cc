// Choreo .traj loading: what only the loader does. Fields mapped to the
// right members, Choreo's own file, versions, splits, events and their
// cursor, the acceleration rule, and malformed or implausible input
// reported as an error naming the source and location. Sampling, flips and
// split() are Trajectory's (trajectory_test).

#include "talOS/trajectory/choreo.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/trajectory/test_fixtures.h"
#include "talOS/trajectory/trajectory.h"

namespace talos::trajectory {
namespace {

using testing::FixturePath;

// A minimal document: `version`, `samples`, then `extra` spliced into the
// root object and `trajectory_extra` into the trajectory object.
std::string Document(std::string_view version, std::string_view samples,
                     std::string_view extra = "",
                     std::string_view trajectory_extra = "") {
  return std::string{R"({"version": )"} + std::string{version} +
         R"(, "trajectory": {"sampleType": "Swerve", "samples": )" +
         std::string{samples} + std::string{trajectory_extra} + "}" +
         std::string{extra} + "}";
}

constexpr std::string_view kTwoSamples =
    R"([{"t": 0, "x": 1, "y": 2, "heading": 0, "vx": 0, "vy": 0, "omega": 0,
         "ax": 0, "ay": 0, "alpha": 0},
        {"t": 1, "x": 1, "y": 2, "heading": 0, "vx": 0, "vy": 0, "omega": 0,
         "ax": 0, "ay": 0, "alpha": 0}])";

// Guards: each sample key lands in its own member (a swapped pair, say vx
// and vy, would pass every sampling test).
TEST(ChoreoTest, MapsSampleFieldsToMembers) {
  const LoadResult result = ParseChoreo(
      Document("3", R"([{"t": 0.5, "x": 1, "y": 2, "heading": 3, "vx": 4,
                         "vy": 5, "omega": 6, "ax": 7, "ay": 8, "alpha": 9,
                         "fx": [1, 2, 3, 4], "fy": [5, 6, 7, 8]}])"),
      "fields");
  ASSERT_TRUE(result) << result.error;
  EXPECT_TRUE(result.note.empty()) << result.note;
  ASSERT_EQ(result.trajectory.samples().size(), 1U);
  const TrajectorySample& s = result.trajectory.samples()[0];
  EXPECT_EQ(s.t_s, 0.5);
  EXPECT_EQ(s.x_m, 1);
  EXPECT_EQ(s.y_m, 2);
  EXPECT_EQ(s.heading_rad, 3);
  EXPECT_EQ(s.vx_mps, 4);
  EXPECT_EQ(s.vy_mps, 5);
  EXPECT_EQ(s.omega_radps, 6);
  EXPECT_EQ(s.ax_mps2, 7);
  EXPECT_EQ(s.ay_mps2, 8);
  EXPECT_EQ(s.alpha_radps2, 9);
  EXPECT_EQ(result.trajectory.interpolation(),
            Interpolation::kConstantAcceleration);
}

// Guards: a file Choreo itself wrote (schema 3, trimmed from Choreo's
// test-jsons/trajectory/3/swerve.traj, with trajectory.config, waypoint
// times and forces) loads, with its unwrapped end heading and both markers.
// REQ-DEV-13
TEST(ChoreoTest, LoadsChoreoSchema3File) {
  const LoadResult result =
      LoadChoreoFile(FixturePath("TALOS_TRAJECTORY_CHOREO_V3_SWERVE"));
  ASSERT_TRUE(result) << result.error;
  const Trajectory& trajectory = result.trajectory;
  EXPECT_EQ(trajectory.samples().size(), 3U);
  EXPECT_DOUBLE_EQ(trajectory.total_time_s(), 1.19651);
  const TrajectorySample last = trajectory.final_sample();
  EXPECT_DOUBLE_EQ(last.x_m, 3.0);
  EXPECT_DOUBLE_EQ(last.y_m, 3.0);
  EXPECT_DOUBLE_EQ(last.heading_rad, 3.12431);
  ASSERT_EQ(trajectory.events().size(), 2U);
  EXPECT_EQ(trajectory.events()[0].name, "Marker");
  EXPECT_DOUBLE_EQ(trajectory.events()[0].t_s, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.events()[1].t_s, 1.19616);
  EXPECT_EQ(result.dropped_event_count, 0U);
}

// Guards: driven the natural way (reset at the start, advance with elapsed
// time each 20 ms tick, from t = 0), an EventCursor fires the real file's
// marker at 0.0 on the first tick and the one at 1.19616 s exactly once.
// REQ-DEV-14
TEST(ChoreoTest, CursorFiresChoreoMarkersOnceIncludingAtZero) {
  const LoadResult result =
      LoadChoreoFile(FixturePath("TALOS_TRAJECTORY_CHOREO_V3_SWERVE"));
  ASSERT_TRUE(result) << result.error;
  EventCursor cursor;
  cursor.reset();
  std::vector<int> fired_at_tick;
  for (int tick = 0; tick <= 100; ++tick) {
    for (const EventMarker& marker :
         cursor.advance(result.trajectory, 0.02 * tick)) {
      EXPECT_EQ(marker.name, "Marker");
      fired_at_tick.push_back(tick);
    }
  }
  // 1.19616 s falls in the tick ending at 1.20 s.
  EXPECT_EQ(fired_at_tick, (std::vector<int>{0, 60}));
}

// Guards: every released schema version (1, 2, 3) is accepted; the inline
// document is valid, so each failure below is caused by the one thing it
// changes.
// REQ-DEV-13
TEST(ChoreoTest, AcceptsVersionsOneToThree) {
  for (const char* version : {"1", "2", "3"}) {
    SCOPED_TRACE(version);
    const LoadResult result = ParseChoreo(Document(version, kTwoSamples), "v");
    EXPECT_TRUE(result) << result.error;
  }
}

// Guards: trajectory.splits become split starts, 0 prepended when absent.
TEST(ChoreoTest, ReadsSplits) {
  constexpr std::string_view kThree =
      R"([{"t": 0, "x": 0, "y": 0, "heading": 0, "vx": 0, "vy": 0, "omega": 0,
           "ax": 0, "ay": 0, "alpha": 0},
          {"t": 1, "x": 0, "y": 0, "heading": 0, "vx": 0, "vy": 0, "omega": 0,
           "ax": 0, "ay": 0, "alpha": 0},
          {"t": 2, "x": 0, "y": 0, "heading": 0, "vx": 0, "vy": 0, "omega": 0,
           "ax": 0, "ay": 0, "alpha": 0}])";
  for (const char* splits : {R"(, "splits": [0, 1])", R"(, "splits": [1])"}) {
    SCOPED_TRACE(splits);
    const LoadResult result =
        ParseChoreo(Document("1", kThree, "", splits), "splits");
    ASSERT_TRUE(result) << result.error;
    ASSERT_EQ(result.trajectory.split_count(), 2U);
    EXPECT_EQ(result.trajectory.split_starts()[0], 0U);
    EXPECT_EQ(result.trajectory.split_starts()[1], 1U);
  }
}

// Guards: an event's time is from.targetTimestamp + from.offset.val, and
// events are dropped and counted, not fatal, where ChoreoLib drops them
// (targetTimestamp not a number, negative time, empty name); a time past
// the end is kept.
TEST(ChoreoTest, ReadsEventsAndDropsAsChoreoLibDoes) {
  const LoadResult result = ParseChoreo(Document("1", kTwoSamples,
                                                 R"(, "events": [
      {"name": "late", "from": {"targetTimestamp": 7.0}},
      {"name": "offset", "from": {"target": 0, "targetTimestamp": 0.5,
        "offset": {"exp": "0.25 s", "val": 0.25}}},
      {"name": "unattached", "from": {"target": null,
        "targetTimestamp": null}},
      {"name": "negative", "from": {"targetTimestamp": 0.0,
        "offset": {"exp": "-0.2 s", "val": -0.2}}},
      {"name": "", "from": {"targetTimestamp": 0.5}},
      {"name": "no_from"}])"),
                                        "events");
  ASSERT_TRUE(result) << result.error;
  EXPECT_EQ(result.dropped_event_count, 4U);
  const Trajectory& trajectory = result.trajectory;
  ASSERT_EQ(trajectory.events().size(), 2U);
  EXPECT_EQ(trajectory.events()[0].name, "offset");
  EXPECT_EQ(trajectory.events()[0].t_s, 0.75);
  EXPECT_EQ(trajectory.events()[1].name, "late");
  EXPECT_EQ(trajectory.events()[1].t_s, 7.0);
}

// Guards: a file whose samples lack ax/ay/alpha still loads, but the switch
// of the whole file to Hermite interpolation is reported in `note`, naming
// the first such sample.
TEST(ChoreoTest, MissingAccelerationsAreReported) {
  const LoadResult result = ParseChoreo(
      Document("1", R"([{"t": 0, "x": 1, "y": 2, "heading": 0, "vx": 0,
                         "vy": 0, "omega": 0, "ax": 0, "ay": 0, "alpha": 0},
                        {"t": 1, "x": 1, "y": 2, "heading": 0, "vx": 0,
                         "vy": 0, "omega": 0}])"),
      "partial");
  ASSERT_TRUE(result) << result.error;
  EXPECT_EQ(result.trajectory.interpolation(), Interpolation::kHermite);
  EXPECT_NE(result.note.find("trajectory.samples[1]"), std::string::npos)
      << result.note;
}

struct Malformed {
  const char* what;
  std::string json;
  // The JSON location or key the error must name; nullptr where there is
  // none (unparseable text). Identifiers, not wording (TESTING.md §4).
  const char* names;
};

// `depth` nested arrays around `inner`: deep enough that any recursive
// walk of it (nlohmann's dump()) overflows a thread's stack.
std::string Nested(std::size_t depth, std::string_view inner) {
  std::string out(depth, '[');
  out += inner;
  out.append(depth, ']');
  return out;
}

// A flat array of `count` zeros.
std::string Flat(std::size_t count) {
  std::string out = "[0";
  for (std::size_t i = 1; i < count; ++i) out += ",0";
  return out + "]";
}

std::string Sample(std::string_view t, std::string_view x) {
  return std::string{R"({"t": )"} + std::string{t} + R"(, "x": )" +
         std::string{x} +
         R"(, "y": 0, "heading": 0, "vx": 0, "vy": 0, "omega": 0})";
}

// Guards: malformed, unsupported or implausible input returns an error
// naming the source and the offending location or key, short whatever the
// input's size; nothing throws or crashes (a node that crashed on its file
// would crash again on every restart).
// REQ-DEV-13
TEST(ChoreoTest, RejectsMalformedInputWithAClearError) {
  constexpr std::size_t kDeep = 200000;
  constexpr std::size_t kHuge = 1000000;
  const std::string sample_missing_vx =
      R"([{"t": 0, "x": 1, "y": 2, "heading": 0, "vy": 0, "omega": 0}])";
  const Malformed cases[] = {
      {"truncated", R"({"version": 1, "trajectory": {)", nullptr},
      {"empty", "", nullptr},
      {"array root", "[]", nullptr},
      {"no version", R"({"trajectory": {"samples": []}})", "\"version\""},
      {"string version", Document(R"("v2025.0.0")", kTwoSamples),
       "\"version\""},
      {"zero version", Document("0", kTwoSamples), "version 0"},
      {"next version", Document("4", kTwoSamples), "version 4"},
      {"fractional version", Document("1.5", kTwoSamples), "\"version\""},
      {"deeply nested version", Document(Nested(kDeep, "1"), kTwoSamples),
       "\"version\""},
      {"huge version", Document(Flat(kHuge), kTwoSamples), "\"version\""},
      {"deeply nested sampleType",
       R"({"version": 1, "trajectory": {"sampleType": )" + Nested(kDeep, "1") +
           R"(, "samples": []}})",
       "trajectory.sampleType"},
      {"huge sampleType",
       R"({"version": 1, "trajectory": {"sampleType": ")" +
           std::string(kHuge, 'D') + R"(", "samples": []}})",
       "trajectory.sampleType"},
      {"differential",
       R"({"version": 1, "trajectory": {"sampleType": "Differential",
           "samples": []}})",
       "trajectory.sampleType"},
      {"no trajectory", R"({"version": 1})", "\"trajectory\""},
      {"no samples", R"({"version": 1, "trajectory": {}})", "\"samples\""},
      {"empty samples", Document("1", "[]"), "trajectory.samples"},
      {"samples not array", Document("1", "{}"), "trajectory.samples"},
      {"missing vx", Document("1", sample_missing_vx),
       "trajectory.samples[0]: missing \"vx\""},
      {"string x", Document("1", "[" + Sample("0", R"("1")") + "]"),
       "trajectory.samples[0].x"},
      {"nested x", Document("1", "[" + Sample("0", Nested(kDeep, "1")) + "]"),
       "trajectory.samples[0].x"},
      {"decreasing time",
       Document("1", "[" + Sample("1", "0") + "," + Sample("0.5", "0") + "]"),
       "trajectory.samples[1].t"},
      {"repeated time",
       Document("1", "[" + Sample("1", "0") + "," + Sample("1", "0") + "]"),
       "trajectory.samples[1].t"},
      {"negative time", Document("1", "[" + Sample("-1", "0") + "]"),
       "trajectory.samples[0].t"},
      {"time beyond an hour", Document("1", "[" + Sample("3601", "0") + "]"),
       "trajectory.samples[0].t"},
      {"x beyond 100 m", Document("1", "[" + Sample("0", "100.5") + "]"),
       "trajectory.samples[0]"},
      {"x overflows a double", Document("1", "[" + Sample("0", "1e999") + "]"),
       nullptr},
      {"split not an index",
       Document("1", kTwoSamples, "", R"(, "splits": [0, -1])"),
       "trajectory.splits[1]"},
      {"split out of range",
       Document("1", kTwoSamples, "", R"(, "splits": [0, 5])"),
       "trajectory.splits[1]"},
      {"events not an array", Document("1", kTwoSamples, R"(, "events": {})"),
       "events"},
  };
  for (const Malformed& c : cases) {
    SCOPED_TRACE(c.what);
    LoadResult result;
    EXPECT_NO_THROW(result = ParseChoreo(c.json, "auto/bad.traj"));
    EXPECT_FALSE(result);
    EXPECT_TRUE(result.trajectory.empty());
    EXPECT_LT(result.error.size(), 300U);
    EXPECT_EQ(result.error.rfind("auto/bad.traj: ", 0), 0U)
        << result.error.substr(0, 300);
    if (c.names != nullptr) {
      EXPECT_NE(result.error.find(c.names), std::string::npos)
          << result.error.substr(0, 300);
    }
  }
}

// Guards: a file that cannot be read is an error naming it.
// REQ-DEV-13
TEST(ChoreoTest, MissingFileIsAnError) {
  const LoadResult result = LoadChoreoFile("/nonexistent/talos/path.traj");
  EXPECT_FALSE(result);
  EXPECT_NE(result.error.find("/nonexistent/talos/path.traj"),
            std::string::npos);
}

}  // namespace
}  // namespace talos::trajectory

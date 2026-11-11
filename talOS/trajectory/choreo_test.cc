// Choreo .traj loading: the testdata fixtures, and malformed or unsupported
// input reported as an error naming the source and the problem.

#include "talOS/trajectory/choreo.h"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <string>
#include <string_view>

#include "talOS/trajectory/test_fixtures.h"
#include "talOS/trajectory/trajectory.h"

namespace talos::trajectory {
namespace {

using testing::FixturePath;

Trajectory Load(const char* env) {
  const LoadResult result = LoadChoreoFile(FixturePath(env));
  if (!result) ADD_FAILURE() << result.error;
  return result.trajectory;
}

// Guards: a file Choreo itself wrote (schema 3, trimmed from Choreo's
// test-jsons/trajectory/3/swerve.traj) loads: samples, end pose with its
// unwrapped heading, and both waypoint-anchored markers.
TEST(ChoreoTest, LoadsChoreoSchema3File) {
  const LoadResult result =
      LoadChoreoFile(FixturePath("TALOS_TRAJECTORY_CHOREO_V3_SWERVE"));
  ASSERT_TRUE(result) << result.error;
  const Trajectory& trajectory = result.trajectory;
  EXPECT_EQ(trajectory.samples().size(), 3U);
  EXPECT_EQ(trajectory.interpolation(), Interpolation::kConstantAcceleration);
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

// Guards: sample() reproduces ChoreoLib's interpolation on a Choreo solve:
// the earlier sample's state integrated at its constant acceleration
// (x + vx tau + ax tau^2 / 2, v + a tau, accelerations copied), with the
// first sample before the start and the last at or after the end.
TEST(ChoreoTest, SamplesAsChoreoLibDoes) {
  const LoadResult result =
      LoadChoreoFile(FixturePath("TALOS_TRAJECTORY_CHOREO_V3_SWERVE"));
  ASSERT_TRUE(result) << result.error;
  const Trajectory& trajectory = result.trajectory;
  const auto choreolib = [](const TrajectorySample& a, double t) {
    const double tau = t - a.t_s;
    TrajectorySample out = a;
    out.t_s = t;
    out.x_m = a.x_m + a.vx_mps * tau + 0.5 * a.ax_mps2 * tau * tau;
    out.y_m = a.y_m + a.vy_mps * tau + 0.5 * a.ay_mps2 * tau * tau;
    out.heading_rad =
        a.heading_rad + a.omega_radps * tau + 0.5 * a.alpha_radps2 * tau * tau;
    out.vx_mps = a.vx_mps + a.ax_mps2 * tau;
    out.vy_mps = a.vy_mps + a.ay_mps2 * tau;
    out.omega_radps = a.omega_radps + a.alpha_radps2 * tau;
    return out;
  };
  const auto stored = trajectory.samples();
  for (const double t : {0.02, 0.0443, 0.5, 1.19}) {
    SCOPED_TRACE(t);
    const TrajectorySample& earlier = t < 0.0443 ? stored[0] : stored[1];
    const TrajectorySample expected = choreolib(earlier, t);
    const TrajectorySample got = trajectory.sample(t);
    EXPECT_DOUBLE_EQ(got.x_m, expected.x_m);
    EXPECT_DOUBLE_EQ(got.y_m, expected.y_m);
    EXPECT_DOUBLE_EQ(got.heading_rad, expected.heading_rad);
    EXPECT_DOUBLE_EQ(got.vx_mps, expected.vx_mps);
    EXPECT_DOUBLE_EQ(got.vy_mps, expected.vy_mps);
    EXPECT_DOUBLE_EQ(got.omega_radps, expected.omega_radps);
    EXPECT_EQ(got.ax_mps2, earlier.ax_mps2);
    EXPECT_EQ(got.alpha_radps2, earlier.alpha_radps2);
  }
  EXPECT_EQ(trajectory.sample(-1.0).x_m, stored[0].x_m);
  EXPECT_EQ(trajectory.sample(1.19651).heading_rad, 3.12431);
  EXPECT_EQ(trajectory.sample(5.0).x_m, 3.0);
}

// Guards: a generated swerve file loads with every sample field, and
// sampling between samples follows the path (curve.traj is a half circle of
// radius 2 m about (8, 4) whose heading runs continuously from 2.6 to
// 4.0 rad, past pi, as Choreo writes headings).
TEST(ChoreoTest, LoadsCurvedPathFixture) {
  const Trajectory trajectory = Load("TALOS_TRAJECTORY_CURVE");
  ASSERT_FALSE(trajectory.empty());
  EXPECT_EQ(trajectory.samples().size(), 61U);
  EXPECT_EQ(trajectory.interpolation(), Interpolation::kConstantAcceleration);
  EXPECT_DOUBLE_EQ(trajectory.total_time_s(), 3.0);
  EXPECT_EQ(trajectory.split_count(), 1U);
  EXPECT_TRUE(trajectory.events().empty());
  EXPECT_NEAR(trajectory.initial_pose().translation().x(), 8.0, 1e-9);
  EXPECT_NEAR(trajectory.initial_pose().translation().y(), 2.0, 1e-9);
  EXPECT_NEAR(trajectory.initial_pose().angle(), 2.6, 1e-9);
  EXPECT_NEAR(trajectory.final_sample().heading_rad, 4.0, 1e-9);
  EXPECT_NEAR(trajectory.final_pose().translation().y(), 6.0, 1e-9);
  EXPECT_NEAR(trajectory.final_pose().angle(), 4.0 - 2 * std::numbers::pi,
              1e-9);

  // Between samples, against the analytic path the fixture was drawn from.
  // The path has jerk, so constant acceleration over a 0.05 s gap is off by
  // up to jerk * dt^3 / 6, about 3e-4 m here.
  for (int i = 0; 0.0125 + 0.1 * i < 3.0; ++i) {
    const double t = 0.0125 + 0.1 * i;
    SCOPED_TRACE(t);
    const double u = t / 3.0;
    const double s =
        10 * std::pow(u, 3) - 15 * std::pow(u, 4) + 6 * std::pow(u, 5);
    const double angle = -std::numbers::pi / 2 + std::numbers::pi * s;
    const TrajectorySample sample = trajectory.sample(t);
    EXPECT_NEAR(sample.x_m, 8.0 + 2.0 * std::cos(angle), 5e-4);
    EXPECT_NEAR(sample.y_m, 4.0 + 2.0 * std::sin(angle), 5e-4);
    EXPECT_LT(std::abs(WrapAngle(sample.heading_rad - (2.6 + 1.4 * s))), 5e-4);
  }
}

// Guards: splits and events are read; event times are the waypoint's
// targetTimestamp plus offset, sorted; an unattached event is dropped and
// counted.
TEST(ChoreoTest, LoadsSplitsAndEvents) {
  const LoadResult result =
      LoadChoreoFile(FixturePath("TALOS_TRAJECTORY_SPLITS_EVENTS"));
  ASSERT_TRUE(result) << result.error;
  EXPECT_EQ(result.dropped_event_count, 1U);
  const Trajectory& trajectory = result.trajectory;
  EXPECT_DOUBLE_EQ(trajectory.total_time_s(), 4.0);
  ASSERT_EQ(trajectory.split_count(), 2U);
  EXPECT_EQ(trajectory.split_starts()[1], 20U);
  ASSERT_EQ(trajectory.events().size(), 2U);
  EXPECT_EQ(trajectory.events()[0].name, "intake_down");
  EXPECT_DOUBLE_EQ(trajectory.events()[0].t_s, 1.5);
  EXPECT_EQ(trajectory.events()[1].name, "shoot");
  EXPECT_DOUBLE_EQ(trajectory.events()[1].t_s, 4.0);

  // The first segment is an exact constant-acceleration trapezoid:
  // 2 m in 2 s, accelerating at 8/3 m/s^2 for 0.5 s.
  const double a = 8.0 / 3.0;
  EXPECT_NEAR(trajectory.sample(0.25).x_m, 2.0 + 0.5 * a * 0.0625, 1e-8);
  EXPECT_NEAR(trajectory.sample(0.25).vx_mps, a * 0.25, 1e-8);

  const Trajectory second = trajectory.split(1);
  EXPECT_DOUBLE_EQ(second.total_time_s(), 2.0);
  EXPECT_NEAR(second.initial_pose().translation().x(), 4.0, 1e-9);
  EXPECT_NEAR(second.initial_pose().translation().y(), 2.0, 1e-9);
  EXPECT_NEAR(second.final_pose().translation().y(), 4.0, 1e-9);
  EXPECT_NEAR(second.final_pose().angle(), std::numbers::pi / 2, 1e-9);
  ASSERT_EQ(second.events().size(), 1U);
  EXPECT_EQ(second.events()[0].name, "shoot");
  EXPECT_DOUBLE_EQ(second.events()[0].t_s, 2.0);
}

// Guards: a file without accelerations falls back to Hermite interpolation.
TEST(ChoreoTest, LoadsFileWithoutAccelerations) {
  const Trajectory trajectory = Load("TALOS_TRAJECTORY_HERMITE_LINE");
  ASSERT_FALSE(trajectory.empty());
  EXPECT_EQ(trajectory.interpolation(), Interpolation::kHermite);
  EXPECT_NEAR(trajectory.sample(0.5).x_m, 1.5, 1e-12);
  EXPECT_NEAR(trajectory.sample(1.75).x_m, 2.75, 1e-12);
  EXPECT_NEAR(trajectory.sample(1.75).vx_mps, 1.0, 1e-12);
}

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

// Guards: every released schema version (1, 2, 3) is accepted; the inline
// document is valid, so each failure below is caused by the one thing it
// changes.
TEST(ChoreoTest, AcceptsVersionsOneToThree) {
  for (const char* version : {"1", "2", "3"}) {
    SCOPED_TRACE(version);
    const LoadResult result = ParseChoreo(Document(version, kTwoSamples), "v");
    ASSERT_TRUE(result) << result.error;
    EXPECT_DOUBLE_EQ(result.trajectory.total_time_s(), 1.0);
  }
}

// Guards: events are dropped and counted, not fatal, where ChoreoLib drops
// them (targetTimestamp not a number, negative time, empty name); a time
// past the end is kept.
TEST(ChoreoTest, DropsEventsAsChoreoLibDoes) {
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

struct Malformed {
  const char* what;
  std::string json;
  // The JSON location or key the error must name; nullptr where there is
  // none (unparseable text). Identifiers, not wording (TESTING.md §4).
  const char* names;
};

// Guards: malformed or unsupported input returns an error naming the source
// and the offending location or key; nothing throws or crashes.
TEST(ChoreoTest, RejectsMalformedInputWithAClearError) {
  const std::string sample_missing_vx =
      R"([{"t": 0, "x": 1, "y": 2, "heading": 0, "vy": 0, "omega": 0}])";
  const std::string sample_string_x =
      R"([{"t": 0, "x": "1", "y": 2, "heading": 0, "vx": 0, "vy": 0,
           "omega": 0}])";
  const std::string decreasing =
      R"([{"t": 1, "x": 0, "y": 0, "heading": 0, "vx": 0, "vy": 0, "omega": 0},
          {"t": 0.5, "x": 0, "y": 0, "heading": 0, "vx": 0, "vy": 0,
           "omega": 0}])";
  const Malformed cases[] = {
      {"truncated", R"({"version": 1, "trajectory": {)", nullptr},
      {"empty", "", nullptr},
      {"array root", "[]", nullptr},
      {"no version", R"({"trajectory": {"samples": []}})", "\"version\""},
      {"string version", Document(R"("v2025.0.0")", kTwoSamples),
       "\"version\""},
      {"future version", Document("99", kTwoSamples), "version 99"},
      {"zero version", Document("0", kTwoSamples), "version 0"},
      {"next version", Document("4", kTwoSamples), "version 4"},
      {"fractional version", Document("1.5", kTwoSamples), "\"version\""},
      {"no trajectory", R"({"version": 1})", "\"trajectory\""},
      {"no samples", R"({"version": 1, "trajectory": {}})", "\"samples\""},
      {"empty samples", Document("1", "[]"), "trajectory.samples"},
      {"samples not array", Document("1", "{}"), "trajectory.samples"},
      {"missing vx", Document("1", sample_missing_vx),
       "trajectory.samples[0]: missing \"vx\""},
      {"string x", Document("1", sample_string_x), "trajectory.samples[0].x"},
      {"decreasing time", Document("1", decreasing), "sample 1"},
      {"differential",
       R"({"version": 1, "trajectory": {"sampleType": "Differential",
           "samples": []}})",
       "trajectory.sampleType"},
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
    EXPECT_EQ(result.error.rfind("auto/bad.traj: ", 0), 0U) << result.error;
    if (c.names != nullptr) {
      EXPECT_NE(result.error.find(c.names), std::string::npos) << result.error;
    }
  }
}

// Guards: a file that cannot be read is an error naming it.
TEST(ChoreoTest, MissingFileIsAnError) {
  const LoadResult result = LoadChoreoFile("/nonexistent/talos/path.traj");
  EXPECT_FALSE(result);
  EXPECT_NE(result.error.find("/nonexistent/talos/path.traj"),
            std::string::npos);
}

}  // namespace
}  // namespace talos::trajectory

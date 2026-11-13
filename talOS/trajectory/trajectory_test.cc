// Trajectory: interpolation against analytic motion, heading wrap, end
// clamping, validation, event markers, splits and alliance flips.

#include "talOS/trajectory/trajectory.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace talos::trajectory {
namespace {

constexpr double kPi = std::numbers::pi;

// Angular distance, so pi and -pi compare equal.
double AngleDistance(double a_rad, double b_rad) {
  return std::abs(WrapAngle(a_rad - b_rad));
}

// Piecewise constant acceleration in x (2, then -1 m/s^2, switching at
// t = 1 s), constant acceleration in y and heading, sampled every 0.5 s.
struct Analytic {
  static double Ax(double t) { return t < 1 ? 2.0 : -1.0; }
  static double Vx(double t) { return t < 1 ? 0.5 + 2 * t : 2.5 - (t - 1); }
  static double X(double t) {
    return t < 1 ? 1 + 0.5 * t + t * t
                 : 1 + 0.5 + 1 + 2.5 * (t - 1) - 0.5 * (t - 1) * (t - 1);
  }
  static double Y(double t) { return -2 + 0.3 * t + 0.2 * t * t; }
  static double Vy(double t) { return 0.3 + 0.4 * t; }
  static double H(double t) { return 0.1 + 0.2 * t - 0.15 * t * t; }
  static double W(double t) { return 0.2 - 0.3 * t; }

  static TrajectorySample At(double t) {
    return {t, X(t), Y(t), H(t), Vx(t), Vy(t), W(t), Ax(t), 0.4, -0.3};
  }
};

Trajectory AnalyticTrajectory() {
  std::vector<TrajectorySample> samples;
  for (int k = 0; k <= 4; ++k) samples.push_back(Analytic::At(0.5 * k));
  return Trajectory{samples};
}

// Guards: sample() between stored samples is ChoreoLib's interpolation:
// the earlier sample integrated at its constant acceleration, exact for
// motion that obeys the model, and blind to the next sample when a file
// does not (a blend toward it would not reproduce ChoreoLib).
TEST(TrajectoryTest, ConstantAccelerationSamplingIsExact) {
  const Trajectory trajectory = AnalyticTrajectory();
  EXPECT_EQ(trajectory.interpolation(), Interpolation::kConstantAcceleration);
  EXPECT_DOUBLE_EQ(trajectory.total_time_s(), 2.0);
  for (int i = 0; 0.037 * i <= 2.0; ++i) {
    const double t = 0.037 * i;
    const TrajectorySample s = trajectory.sample(t);
    SCOPED_TRACE(t);
    EXPECT_DOUBLE_EQ(s.t_s, t);
    EXPECT_NEAR(s.x_m, Analytic::X(t), 1e-12);
    EXPECT_NEAR(s.y_m, Analytic::Y(t), 1e-12);
    EXPECT_NEAR(s.heading_rad, Analytic::H(t), 1e-12);
    EXPECT_NEAR(s.vx_mps, Analytic::Vx(t), 1e-12);
    EXPECT_NEAR(s.vy_mps, Analytic::Vy(t), 1e-12);
    EXPECT_NEAR(s.omega_radps, Analytic::W(t), 1e-12);
    EXPECT_DOUBLE_EQ(s.ax_mps2, Analytic::Ax(t));
    EXPECT_DOUBLE_EQ(s.ay_mps2, 0.4);
    EXPECT_DOUBLE_EQ(s.alpha_radps2, -0.3);
  }

  std::vector<TrajectorySample> inconsistent(trajectory.samples().begin(),
                                             trajectory.samples().end());
  inconsistent[2].x_m += 5;  // the sample at 1.0 s no longer obeys the model
  inconsistent[2].vx_mps += 5;
  const Trajectory edited{inconsistent};
  EXPECT_NEAR(edited.sample(0.9).x_m, Analytic::X(0.9), 1e-12);
  EXPECT_NEAR(edited.sample(0.9).vx_mps, Analytic::Vx(0.9), 1e-12);
}

// Guards: without accelerations, cubic Hermite interpolation is exact for a
// cubic (x = t^3 - 2t), its velocity and its acceleration.
TEST(TrajectoryTest, HermiteSamplingIsExactForACubic) {
  const auto x = [](double t) { return t * t * t - 2 * t; };
  const auto v = [](double t) { return 3 * t * t - 2; };
  std::vector<TrajectorySample> samples;
  for (const double t : {0.0, 0.7, 2.0}) {
    TrajectorySample s;
    s.t_s = t;
    s.x_m = x(t);
    s.vx_mps = v(t);
    s.ax_mps2 = 99;  // ignored under kHermite
    samples.push_back(s);
  }
  const Trajectory trajectory{samples, Interpolation::kHermite};
  for (int i = 0; 0.01 + 0.093 * i < 2.0; ++i) {
    const double t = 0.01 + 0.093 * i;
    const TrajectorySample s = trajectory.sample(t);
    SCOPED_TRACE(t);
    EXPECT_NEAR(s.x_m, x(t), 1e-12);
    EXPECT_NEAR(s.vx_mps, v(t), 1e-12);
    EXPECT_NEAR(s.ax_mps2, 6 * t, 1e-10);
  }
  EXPECT_EQ(trajectory.samples()[1].ax_mps2, 0.0);
}

// Guards: a heading stored wrapped (3.0 then -3.0 rad) is interpolated the
// short way through pi, never back through 0, in both interpolation modes,
// and is returned continuous (not wrapped), as Choreo writes headings.
TEST(TrajectoryTest, HeadingInterpolatesAcrossPi) {
  const double delta_rad = 2 * kPi - 6.0;  // 3.0 -> -3.0 the short way
  std::vector<TrajectorySample> samples(2);
  samples[0].heading_rad = 3.0;
  samples[0].omega_radps = delta_rad;
  samples[1].t_s = 1.0;
  samples[1].heading_rad = -3.0;
  samples[1].omega_radps = delta_rad;
  for (const Interpolation mode :
       {Interpolation::kConstantAcceleration, Interpolation::kHermite}) {
    const Trajectory trajectory{samples, mode};
    for (int i = 0; 0.05 * i <= 1.0; ++i) {
      const double t = 0.05 * i;
      const TrajectorySample s = trajectory.sample(t);
      SCOPED_TRACE(t);
      EXPECT_LT(AngleDistance(s.heading_rad, 3.0 + delta_rad * t), 1e-12);
      EXPECT_NEAR(s.omega_radps, delta_rad, 1e-12);
    }
    EXPECT_LT(AngleDistance(trajectory.sample(0.5).heading_rad, kPi), 1e-12);
  }
}

// Guards: sample() holds the first state before the start (and for NaN) and
// the last state after the end; an empty trajectory samples to zero.
TEST(TrajectoryTest, SampleClampsAtBothEnds) {
  const Trajectory trajectory = AnalyticTrajectory();
  const TrajectorySample first = trajectory.samples().front();
  const TrajectorySample last = trajectory.samples().back();
  for (const double t : {-5.0, 0.0, -std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    const TrajectorySample s = trajectory.sample(t);
    EXPECT_EQ(s.t_s, first.t_s);
    EXPECT_EQ(s.x_m, first.x_m);
    EXPECT_EQ(s.vx_mps, first.vx_mps);
  }
  for (const double t :
       {2.0, 2.5, 1e9, std::numeric_limits<double>::infinity()}) {
    const TrajectorySample s = trajectory.sample(t);
    EXPECT_EQ(s.t_s, last.t_s);
    EXPECT_EQ(s.x_m, last.x_m);
    EXPECT_EQ(s.y_m, last.y_m);
    EXPECT_EQ(s.vx_mps, last.vx_mps);
  }
  EXPECT_DOUBLE_EQ(trajectory.initial_pose().translation().x(), first.x_m);
  EXPECT_DOUBLE_EQ(trajectory.final_pose().translation().y(), last.y_m);
  EXPECT_DOUBLE_EQ(trajectory.final_pose().angle(), last.heading_rad);

  const Trajectory empty;
  EXPECT_TRUE(empty.empty());
  EXPECT_EQ(empty.total_time_s(), 0.0);
  EXPECT_EQ(empty.sample(1.0).x_m, 0.0);
  EXPECT_TRUE(EventCursor{}.advance(empty, 1.0).empty());
}

// Guards: construction refuses input sample() could not interpolate.
TEST(TrajectoryTest, ConstructorRejectsInvalidInput) {
  EXPECT_THROW(static_cast<void>(Trajectory{std::vector<TrajectorySample>{}}),
               std::invalid_argument);
  std::vector<TrajectorySample> repeated(2);  // both at t = 0
  EXPECT_THROW(static_cast<void>(Trajectory{repeated}), std::invalid_argument);
  std::vector<TrajectorySample> negative(1);
  negative[0].t_s = -0.1;
  EXPECT_THROW(static_cast<void>(Trajectory{negative}), std::invalid_argument);
  std::vector<TrajectorySample> nan(1);
  nan[0].vy_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(static_cast<void>(Trajectory{nan}), std::invalid_argument);

  std::vector<TrajectorySample> three(3);
  three[1].t_s = 1;
  three[2].t_s = 2;
  EXPECT_THROW(static_cast<void>(Trajectory{
                   three, Interpolation::kConstantAcceleration, {}, {0, 3}}),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(Trajectory{
                   three, Interpolation::kConstantAcceleration, {}, {0, 2, 1}}),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(Trajectory{
                   three,
                   Interpolation::kConstantAcceleration,
                   {{"bad", std::numeric_limits<double>::infinity()}}}),
               std::invalid_argument);
  // A missing leading 0 is supplied.
  const Trajectory split{three, Interpolation::kConstantAcceleration, {}, {1}};
  ASSERT_EQ(split.split_count(), 2U);
  EXPECT_EQ(split.split_starts()[0], 0U);
}

// Guards: markers come back in time order, and an EventCursor driven each
// tick fires every marker exactly once, including one at 0.0 on the first
// tick and ties together; a cursor that is not reset fires nothing again.
// REQ-DEV-14
TEST(TrajectoryTest, EventCursorFiresEachMarkerOnce) {
  std::vector<TrajectorySample> samples(2);
  samples[1].t_s = 3.0;
  const Trajectory trajectory{
      samples,
      Interpolation::kConstantAcceleration,
      {{"c", 2.0}, {"a", 0.0}, {"b", 1.0}, {"b2", 1.0}}};
  ASSERT_EQ(trajectory.events().size(), 4U);
  EXPECT_EQ(trajectory.events()[0].name, "a");
  EXPECT_EQ(trajectory.events()[3].name, "c");

  EventCursor cursor;
  std::vector<std::string> fired;
  for (int tick = 0; tick <= 200; ++tick) {
    for (const EventMarker& e : cursor.advance(trajectory, 0.02 * tick)) {
      fired.push_back(e.name + "@" + std::to_string(tick));
    }
  }
  EXPECT_EQ(fired, (std::vector<std::string>{"a@0", "b@50", "b2@50", "c@100"}));
  EXPECT_TRUE(cursor.advance(trajectory, 10.0).empty());
  cursor.reset();
  EXPECT_EQ(cursor.advance(trajectory, 0.0).size(), 1U);
  EXPECT_TRUE(cursor.advance(trajectory, std::nan("")).empty());
}

// Guards: split(i) is the i-th segment re-timed to 0, with its markers; a
// marker on a split boundary belongs to the later segment only, so running
// the segments in turn with a cursor fires every marker exactly once.
TEST(TrajectoryTest, SplitReturnsRetimedSegment) {
  std::vector<TrajectorySample> samples(5);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    samples[i].t_s = static_cast<double>(i);
    samples[i].x_m = 10.0 * static_cast<double>(i);
  }
  const Trajectory trajectory{samples,
                              Interpolation::kHermite,
                              {{"start", 0.0},
                               {"in_first", 0.5},
                               {"boundary", 2.0},
                               {"in_second", 3.5},
                               {"end", 4.0}},
                              {0, 2}};
  ASSERT_EQ(trajectory.split_count(), 2U);
  const Trajectory first = trajectory.split(0);
  ASSERT_EQ(first.samples().size(), 3U);
  EXPECT_EQ(first.total_time_s(), 2.0);
  EXPECT_EQ(first.final_sample().x_m, 20.0);
  const Trajectory second = trajectory.split(1);
  ASSERT_EQ(second.samples().size(), 3U);
  EXPECT_EQ(second.initial_sample().t_s, 0.0);
  EXPECT_EQ(second.initial_sample().x_m, 20.0);
  EXPECT_EQ(second.total_time_s(), 2.0);
  EXPECT_EQ(second.interpolation(), Interpolation::kHermite);
  EXPECT_THROW(static_cast<void>(trajectory.split(2)), std::out_of_range);

  std::vector<std::string> fired;
  for (const Trajectory* segment : {&first, &second}) {
    EventCursor cursor;
    for (int tick = 0; tick <= 100; ++tick) {
      for (const EventMarker& e : cursor.advance(*segment, 0.02 * tick)) {
        fired.push_back(e.name);
      }
    }
  }
  EXPECT_EQ(fired, (std::vector<std::string>{"start", "in_first", "boundary",
                                             "in_second", "end"}));
}

// Guards: assign() replaces the samples (events cleared, one split) and,
// on invalid input, throws and leaves the trajectory as it was.
TEST(TrajectoryTest, AssignReplacesOrLeavesUnchanged) {
  std::vector<TrajectorySample> old_samples(2);
  old_samples[1].t_s = 1.0;
  Trajectory trajectory{old_samples,
                        Interpolation::kConstantAcceleration,
                        {{"old", 0.0}},
                        {0, 1}};
  std::vector<TrajectorySample> replacement(3);
  for (std::size_t i = 0; i < replacement.size(); ++i) {
    replacement[i].t_s = static_cast<double>(i);
    replacement[i].x_m = 1.0 + static_cast<double>(i);
  }
  std::vector<TrajectorySample> bad = replacement;
  bad[2].t_s = 0.5;
  EXPECT_THROW(trajectory.assign(bad), std::invalid_argument);
  EXPECT_EQ(trajectory.samples().size(), 2U);
  EXPECT_EQ(trajectory.events().size(), 1U);

  trajectory.assign(replacement);
  EXPECT_EQ(trajectory.samples().size(), 3U);
  EXPECT_EQ(trajectory.total_time_s(), 2.0);
  EXPECT_EQ(trajectory.final_sample().x_m, 3.0);
  EXPECT_TRUE(trajectory.events().empty());
  EXPECT_EQ(trajectory.split_count(), 1U);
}

constexpr FieldDimensions kField = kField2026.field;

void ExpectSameSample(const TrajectorySample& a, const TrajectorySample& b) {
  EXPECT_NEAR(a.t_s, b.t_s, 1e-12);
  EXPECT_NEAR(a.x_m, b.x_m, 1e-12);
  EXPECT_NEAR(a.y_m, b.y_m, 1e-12);
  EXPECT_LT(AngleDistance(a.heading_rad, b.heading_rad), 1e-12);
  EXPECT_NEAR(a.vx_mps, b.vx_mps, 1e-12);
  EXPECT_NEAR(a.vy_mps, b.vy_mps, 1e-12);
  EXPECT_NEAR(a.omega_radps, b.omega_radps, 1e-12);
  EXPECT_NEAR(a.ax_mps2, b.ax_mps2, 1e-12);
  EXPECT_NEAR(a.ay_mps2, b.ay_mps2, 1e-12);
  EXPECT_NEAR(a.alpha_radps2, b.alpha_radps2, 1e-12);
}

// Guards: each flip maps a blue pose to the red one it names.
// REQ-DEV-14
TEST(FlipTest, MapsPosesForEachSymmetry) {
  TrajectorySample s;
  s.x_m = 2.0;
  s.y_m = 1.0;
  s.heading_rad = kPi / 2;  // facing +y
  s.vx_mps = 1.0;
  s.vy_mps = 0.5;
  s.omega_radps = 0.25;

  const TrajectorySample rotated = Flip(s, AllianceFlip::kRotate, kField);
  EXPECT_NEAR(rotated.x_m, 14.541, 1e-12);
  EXPECT_NEAR(rotated.y_m, 7.0692, 1e-12);
  EXPECT_NEAR(rotated.heading_rad, kPi + kPi / 2, 1e-12);  // not wrapped
  EXPECT_EQ(rotated.vx_mps, -1.0);
  EXPECT_EQ(rotated.vy_mps, -0.5);
  EXPECT_EQ(rotated.omega_radps, 0.25);  // rotation keeps handedness

  const TrajectorySample mirrored = Flip(s, AllianceFlip::kMirror, kField);
  EXPECT_NEAR(mirrored.x_m, 14.541, 1e-12);
  EXPECT_EQ(mirrored.y_m, 1.0);
  EXPECT_NEAR(mirrored.heading_rad, kPi / 2, 1e-12);
  EXPECT_EQ(mirrored.vx_mps, -1.0);
  EXPECT_EQ(mirrored.vy_mps, 0.5);
  EXPECT_EQ(mirrored.omega_radps, -0.25);  // a mirror reverses turning

  // Facing downfield (heading 0) faces back toward blue after either flip.
  s.heading_rad = 0;
  EXPECT_LT(
      AngleDistance(Flip(s, AllianceFlip::kRotate, kField).heading_rad, kPi),
      1e-12);
  EXPECT_LT(
      AngleDistance(Flip(s, AllianceFlip::kMirror, kField).heading_rad, kPi),
      1e-12);

  const geometry::SE2<double> pose =
      Flip(s.pose(), AllianceFlip::kRotate, kField);
  EXPECT_NEAR(pose.translation().x(), 14.541, 1e-12);
  EXPECT_NEAR(pose.translation().y(), 7.0692, 1e-12);
}

// Guards: flipping twice is the identity, and flipping a trajectory commutes
// with sampling it (velocities and accelerations flip consistently with
// positions), for both symmetries.
// REQ-DEV-14
TEST(FlipTest, RoundTripsAndCommutesWithSampling) {
  const Trajectory trajectory = AnalyticTrajectory();
  for (const AllianceFlip flip :
       {AllianceFlip::kRotate, AllianceFlip::kMirror}) {
    const Trajectory red = trajectory.flipped(flip, kField);
    const Trajectory blue = red.flipped(flip, kField);
    for (int i = 0; 0.07 * i <= 2.0; ++i) {
      const double t = 0.07 * i;
      SCOPED_TRACE(t);
      ExpectSameSample(blue.sample(t), trajectory.sample(t));
      ExpectSameSample(red.sample(t), Flip(trajectory.sample(t), flip, kField));
    }
  }
  EXPECT_THROW(
      static_cast<void>(trajectory.flipped(AllianceFlip::kRotate, {0.0, 8.0})),
      std::invalid_argument);
}

// Guards: a flipped trajectory is still self-consistent: inside a segment,
// the central difference of its sampled position and heading equals its
// sampled velocity and omega, and that of its velocity its acceleration,
// for both symmetries (a sign error on any derivative breaks this).
TEST(FlipTest, FlippedTrajectoryIsSelfConsistent) {
  const Trajectory trajectory = AnalyticTrajectory();
  constexpr double kH = 1e-4;
  for (const AllianceFlip flip :
       {AllianceFlip::kRotate, AllianceFlip::kMirror}) {
    const Trajectory red = trajectory.flipped(flip, kField);
    for (const double t : {0.2, 0.7, 1.3, 1.8}) {
      SCOPED_TRACE(t);
      const TrajectorySample before = red.sample(t - kH);
      const TrajectorySample now = red.sample(t);
      const TrajectorySample after = red.sample(t + kH);
      EXPECT_NEAR((after.x_m - before.x_m) / (2 * kH), now.vx_mps, 1e-7);
      EXPECT_NEAR((after.y_m - before.y_m) / (2 * kH), now.vy_mps, 1e-7);
      EXPECT_NEAR((after.heading_rad - before.heading_rad) / (2 * kH),
                  now.omega_radps, 1e-7);
      EXPECT_NEAR((after.vx_mps - before.vx_mps) / (2 * kH), now.ax_mps2, 1e-7);
      EXPECT_NEAR((after.vy_mps - before.vy_mps) / (2 * kH), now.ay_mps2, 1e-7);
      EXPECT_NEAR((after.omega_radps - before.omega_radps) / (2 * kH),
                  now.alpha_radps2, 1e-7);
    }
  }
}

}  // namespace
}  // namespace talos::trajectory

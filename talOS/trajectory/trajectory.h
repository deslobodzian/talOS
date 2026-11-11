#pragma once

// Time-indexed holonomic trajectories (README.md). Field frame: x downfield,
// y left, headings CCW+ in radians, all SI. Storage is sized when a
// Trajectory is built (from samples or a Choreo file); sample(),
// events_between() and the accessors never allocate, so a node can call them
// from update().

#include <cstddef>
#include <span>
#include <string>
#include <vector>

#include "talOS/geometry/se2.h"

namespace talos::trajectory {

// One state along the path, in the field frame. Accelerations are zero when
// the source had none (Interpolation::kHermite).
struct TrajectorySample {
  double t_s = 0;
  double x_m = 0;
  double y_m = 0;
  double heading_rad = 0;
  double vx_mps = 0;
  double vy_mps = 0;
  double omega_radps = 0;
  double ax_mps2 = 0;
  double ay_mps2 = 0;
  double alpha_radps2 = 0;

  geometry::SE2<double> pose() const;
};

// A named instant along the trajectory (a Choreo event), seconds from the
// trajectory's time origin.
struct EventMarker {
  std::string name;
  double t_s = 0;
};

// How sample() fills the state between two stored samples.
enum class Interpolation {
  // Integrates the earlier sample's accelerations over the gap, exactly as
  // ChoreoLib does (Choreo's piecewise-constant-acceleration model).
  kConstantAcceleration,
  // Cubic Hermite on positions and velocities (no accelerations stored).
  kHermite,
};

// Field size for alliance flips. Choreo paths are drawn for the blue
// alliance; the origin is the blue-alliance corner.
struct FieldDimensions {
  double length_m = 0;  // along x
  double width_m = 0;   // along y
};

enum class AllianceFlip {
  // 180-degree rotation about the field centre (a rotationally symmetric
  // field, such as 2026's): (x, y, heading) -> (L - x, W - y, heading + pi).
  kRotate,
  // Mirror across the centre line x = L / 2 (a mirror-symmetric field):
  // (x, y, heading) -> (L - x, y, pi - heading).
  kMirror,
};

// A game's field and the flip its symmetry needs; pass `field` and `flip`
// to flipped(). Symmetry by year: 2022 rotate, 2023 and 2024 mirror, 2025
// and 2026 rotate.
struct FieldPreset {
  FieldDimensions field;
  AllianceFlip flip;
};
inline constexpr FieldPreset kField2025{{17.548, 8.052}, AllianceFlip::kRotate};
inline constexpr FieldPreset kField2026{{16.541, 8.0692},
                                        AllianceFlip::kRotate};

// The flipped state, accelerations and velocities included. Heading is not
// wrapped. Applying a flip twice returns the original (rotate: heading plus
// 2 pi).
TrajectorySample Flip(const TrajectorySample& sample, AllianceFlip flip,
                      const FieldDimensions& field);
geometry::SE2<double> Flip(const geometry::SE2<double>& pose, AllianceFlip flip,
                           const FieldDimensions& field);

class Trajectory {
 public:
  // Empty: sample() returns a default sample, total_time_s() is 0.
  Trajectory() = default;

  // Throws std::invalid_argument when `samples` is empty, holds a non-finite
  // value or a negative or non-increasing time; when `split_starts` is not
  // strictly increasing sample indices within range (0 is prepended when
  // missing); or when an event time is not finite. Events are kept in time
  // order (ties keep their given order). Headings need not be wrapped.
  explicit Trajectory(
      std::vector<TrajectorySample> samples,
      Interpolation interpolation = Interpolation::kConstantAcceleration,
      std::vector<EventMarker> events = {},
      std::vector<std::size_t> split_starts = {});

  bool empty() const { return samples_.empty(); }
  Interpolation interpolation() const { return interpolation_; }

  // Time of the last sample (the first sample is normally at 0).
  double total_time_s() const;

  // The state at `t_s`: the first sample at or before the start (and for
  // NaN), the last at or after the end. Headings are continuous, as Choreo
  // writes them, not wrapped: wrap only an error (WrapAngle). O(log n), no
  // allocation.
  TrajectorySample sample(double t_s) const;

  TrajectorySample initial_sample() const;
  TrajectorySample final_sample() const;
  geometry::SE2<double> initial_pose() const;
  geometry::SE2<double> final_pose() const;

  std::span<const TrajectorySample> samples() const { return samples_; }

  // All markers, in time order.
  std::span<const EventMarker> events() const { return events_; }
  // The markers with after_s < t_s <= until_s, in time order: a node that
  // passes its previous and current elapsed time each tick sees each marker
  // once. No allocation.
  std::span<const EventMarker> events_between(double after_s,
                                              double until_s) const;

  // Choreo splits: the trajectory is split_count() segments; segment i runs
  // from sample split_starts()[i] to the next split's start (inclusive) or
  // the end.
  std::span<const std::size_t> split_starts() const { return split_starts_; }
  std::size_t split_count() const { return split_starts_.size(); }
  // Segment `index` as its own trajectory, re-timed to start at 0, with the
  // markers inside it. Allocates; throws std::out_of_range on a bad index.
  Trajectory split(std::size_t index) const;

  // The whole trajectory for the other alliance. Allocates.
  Trajectory flipped(AllianceFlip flip, const FieldDimensions& field) const;

 private:
  std::vector<TrajectorySample> samples_;
  std::vector<EventMarker> events_;
  std::vector<std::size_t> split_starts_;
  Interpolation interpolation_ = Interpolation::kConstantAcceleration;
};

// Wraps an angle to (-pi, pi].
double WrapAngle(double angle_rad);

}  // namespace talos::trajectory

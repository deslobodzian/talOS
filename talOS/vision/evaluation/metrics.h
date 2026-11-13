#pragma once

// Tracking statistics recording_eval reports over a recording: frame-to-frame
// tag association (id switches), dropouts, and jitter about a locally
// constant-velocity track. Pure functions over per-frame data, so the numbers
// in an evaluation report are the ones metrics_test checks.

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace talos::vision::evaluation {

// One decoded tag in one frame, as association needs it.
struct TagSighting {
  int tag_id{-1};
  Eigen::Vector2d center_px{Eigen::Vector2d::Zero()};
  // Shortest quad edge, pixels: the association radius scales with it.
  double side_px{0};
};

// Nearest-neighbour association of consecutive frames: a sighting in `current`
// is the same physical tag as the closest sighting in `previous` whose centre
// lies within half the smaller of their sides (a tag cannot move half its own
// width between frames without blurring past detection). Returns the number of
// associated pairs whose decoded ids differ: an id switch is a decode error
// on one of them, never a tag swapping places.
inline int CountIdSwitches(const std::vector<TagSighting>& previous,
                           const std::vector<TagSighting>& current) {
  int switches = 0;
  std::vector<bool> taken(previous.size(), false);
  for (const auto& now : current) {
    double best = std::numeric_limits<double>::infinity();
    std::size_t match = previous.size();
    for (std::size_t i = 0; i < previous.size(); ++i) {
      if (taken[i]) continue;
      const double radius = 0.5 * std::min(now.side_px, previous[i].side_px);
      const double distance = (now.center_px - previous[i].center_px).norm();
      if (distance <= radius && distance < best) {
        best = distance;
        match = i;
      }
    }
    if (match == previous.size()) continue;
    taken[match] = true;
    if (previous[match].tag_id != now.tag_id) ++switches;
  }
  return switches;
}

// Longest run of consecutive `false` in `seen` that has a `true` on both
// sides: a dropout of a tag that was in view before and after it. Runs at
// the start or end (the tag entering or leaving view) are not dropouts.
inline std::size_t LongestDropout(const std::vector<bool>& seen) {
  std::size_t longest = 0, run = 0;
  bool started = false;
  for (const bool s : seen) {
    if (s) {
      if (started) longest = std::max(longest, run);
      started = true;
      run = 0;
    } else if (started) {
      ++run;
    }
  }
  return longest;
}

// Jitter of a sampled track about a locally constant-velocity motion: each
// interior sample's distance from the midpoint of its neighbours, for runs of
// samples present in consecutive frames (nullopt-free runs). For independent
// noise of standard deviation sigma per axis, the residual per axis has
// variance 1.5 sigma^2, so `sigma_estimate` divides the per-axis RMS by
// sqrt(1.5). Real acceleration also contributes (a * dt^2 / 2 per axis).
struct JitterStats {
  std::size_t samples{0};
  double rms_per_axis{0};
  double sigma_estimate{0};
};

template <int N>
JitterStats SecondDifferenceJitter(
    const std::vector<std::optional<Eigen::Matrix<double, N, 1>>>& track) {
  JitterStats stats;
  double sum_sq = 0;
  for (std::size_t i = 1; i + 1 < track.size(); ++i) {
    const auto& before = track[i - 1];
    const auto& at = track[i];
    const auto& after = track[i + 1];
    if (!before || !at || !after) continue;
    const Eigen::Matrix<double, N, 1> residual = *at - 0.5 * (*before + *after);
    sum_sq += residual.squaredNorm();
    ++stats.samples;
  }
  if (stats.samples == 0) return stats;
  stats.rms_per_axis =
      std::sqrt(sum_sq / (N * static_cast<double>(stats.samples)));
  stats.sigma_estimate = stats.rms_per_axis / std::sqrt(1.5);
  return stats;
}

// The q-quantile (0..1) of `values` by nearest rank; NaN when empty.
inline double Quantile(std::vector<double> values, double q) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const auto rank = static_cast<std::size_t>(
      std::ceil(std::clamp(q, 0.0, 1.0) * static_cast<double>(values.size())));
  return values[rank == 0 ? 0 : rank - 1];
}

}  // namespace talos::vision::evaluation

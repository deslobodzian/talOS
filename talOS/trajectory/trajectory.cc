#include "talOS/trajectory/trajectory.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "talOS/geometry/se2.h"

namespace talos::trajectory {
namespace {

bool Finite(const TrajectorySample& s) {
  for (const double value :
       {s.t_s, s.x_m, s.y_m, s.heading_rad, s.vx_mps, s.vy_mps, s.omega_radps,
        s.ax_mps2, s.ay_mps2, s.alpha_radps2}) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

// ChoreoLib's interpolation: the earlier sample's accelerations integrated
// over tau = t - a.t (x + vx tau + ax tau^2 / 2, vx + ax tau, likewise y and
// heading), accelerations copied. `b` is not used: for a Choreo solve the
// integration lands on it, and matching ChoreoLib exactly matters more than
// continuity on a file that does not obey the model.
TrajectorySample ConstantAcceleration(const TrajectorySample& a, double t_s) {
  const double tau = t_s - a.t_s;
  TrajectorySample out = a;
  out.t_s = t_s;
  out.x_m = a.x_m + a.vx_mps * tau + 0.5 * a.ax_mps2 * tau * tau;
  out.y_m = a.y_m + a.vy_mps * tau + 0.5 * a.ay_mps2 * tau * tau;
  out.heading_rad =
      a.heading_rad + a.omega_radps * tau + 0.5 * a.alpha_radps2 * tau * tau;
  out.vx_mps = a.vx_mps + a.ax_mps2 * tau;
  out.vy_mps = a.vy_mps + a.ay_mps2 * tau;
  out.omega_radps = a.omega_radps + a.alpha_radps2 * tau;
  return out;
}

// Cubic Hermite on position and velocity; acceleration is its second
// derivative.
TrajectorySample Hermite(const TrajectorySample& a, const TrajectorySample& b,
                         double t_s) {
  const double dt = b.t_s - a.t_s;
  const double u = (t_s - a.t_s) / dt;
  const double u2 = u * u;
  const double u3 = u2 * u;
  struct Axis {
    double position;
    double velocity;
    double acceleration;
  };
  const auto axis = [&](double p0, double v0, double p1, double v1) {
    return Axis{
        (2 * u3 - 3 * u2 + 1) * p0 + (u3 - 2 * u2 + u) * dt * v0 +
            (-2 * u3 + 3 * u2) * p1 + (u3 - u2) * dt * v1,
        (6 * u2 - 6 * u) * (p0 - p1) / dt + (3 * u2 - 4 * u + 1) * v0 +
            (3 * u2 - 2 * u) * v1,
        (12 * u - 6) * (p0 - p1) / (dt * dt) + (6 * u - 4) * v0 / dt +
            (6 * u - 2) * v1 / dt,
    };
  };
  // The end heading taken the shorter way round from the start heading.
  const double heading_end =
      a.heading_rad + WrapAngle(b.heading_rad - a.heading_rad);
  const Axis x = axis(a.x_m, a.vx_mps, b.x_m, b.vx_mps);
  const Axis y = axis(a.y_m, a.vy_mps, b.y_m, b.vy_mps);
  const Axis h = axis(a.heading_rad, a.omega_radps, heading_end, b.omega_radps);
  TrajectorySample out;
  out.t_s = t_s;
  out.x_m = x.position;
  out.y_m = y.position;
  out.heading_rad = h.position;
  out.vx_mps = x.velocity;
  out.vy_mps = y.velocity;
  out.omega_radps = h.velocity;
  out.ax_mps2 = x.acceleration;
  out.ay_mps2 = y.acceleration;
  out.alpha_radps2 = h.acceleration;
  return out;
}

}  // namespace

double WrapAngle(double angle_rad) {
  const double wrapped = std::remainder(angle_rad, 2 * std::numbers::pi);
  return wrapped <= -std::numbers::pi ? std::numbers::pi : wrapped;
}

geometry::SE2<double> TrajectorySample::pose() const {
  return geometry::SE2<double>(WrapAngle(heading_rad),
                               Eigen::Vector2d(x_m, y_m));
}

TrajectorySample Flip(const TrajectorySample& sample, AllianceFlip flip,
                      const FieldDimensions& field) {
  TrajectorySample out = sample;
  out.x_m = field.length_m - sample.x_m;
  out.vx_mps = -sample.vx_mps;
  out.ax_mps2 = -sample.ax_mps2;
  switch (flip) {
    case AllianceFlip::kRotate:
      out.y_m = field.width_m - sample.y_m;
      out.vy_mps = -sample.vy_mps;
      out.ay_mps2 = -sample.ay_mps2;
      out.heading_rad = std::numbers::pi + sample.heading_rad;
      break;
    case AllianceFlip::kMirror:
      out.heading_rad = std::numbers::pi - sample.heading_rad;
      out.omega_radps = -sample.omega_radps;
      out.alpha_radps2 = -sample.alpha_radps2;
      break;
  }
  return out;
}

geometry::SE2<double> Flip(const geometry::SE2<double>& pose, AllianceFlip flip,
                           const FieldDimensions& field) {
  TrajectorySample sample;
  sample.x_m = pose.translation().x();
  sample.y_m = pose.translation().y();
  sample.heading_rad = pose.angle();
  return Flip(sample, flip, field).pose();
}

namespace {

// Throws std::invalid_argument naming the first sample that sample() could
// not interpolate.
void ValidateSamples(std::span<const TrajectorySample> samples) {
  if (samples.empty()) {
    throw std::invalid_argument("trajectory has no samples");
  }
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const TrajectorySample& s = samples[i];
    if (!Finite(s)) {
      throw std::invalid_argument("trajectory sample " + std::to_string(i) +
                                  " has a non-finite value");
    }
    if (s.t_s < 0 || (i > 0 && s.t_s <= samples[i - 1].t_s)) {
      throw std::invalid_argument("trajectory sample " + std::to_string(i) +
                                  " time is negative or not increasing");
    }
  }
}

void ZeroAccelerations(std::span<TrajectorySample> samples) {
  for (TrajectorySample& s : samples) {
    s.ax_mps2 = 0;
    s.ay_mps2 = 0;
    s.alpha_radps2 = 0;
  }
}

}  // namespace

Trajectory::Trajectory(std::vector<TrajectorySample> samples,
                       Interpolation interpolation,
                       std::vector<EventMarker> events,
                       std::vector<std::size_t> split_starts)
    : samples_(std::move(samples)),
      events_(std::move(events)),
      split_starts_(std::move(split_starts)),
      interpolation_(interpolation) {
  ValidateSamples(samples_);
  if (interpolation_ == Interpolation::kHermite) ZeroAccelerations(samples_);
  if (split_starts_.empty() || split_starts_.front() != 0) {
    split_starts_.insert(split_starts_.begin(), 0);
  }
  for (std::size_t i = 0; i < split_starts_.size(); ++i) {
    if (split_starts_[i] >= samples_.size() ||
        (i > 0 && split_starts_[i] <= split_starts_[i - 1])) {
      throw std::invalid_argument("trajectory split " + std::to_string(i) +
                                  " is not an increasing sample index below " +
                                  std::to_string(samples_.size()));
    }
  }
  for (const EventMarker& event : events_) {
    if (!std::isfinite(event.t_s)) {
      throw std::invalid_argument("trajectory event '" + event.name +
                                  "' has a non-finite time");
    }
  }
  std::stable_sort(
      events_.begin(), events_.end(),
      [](const EventMarker& a, const EventMarker& b) { return a.t_s < b.t_s; });
}

void Trajectory::reserve(std::size_t sample_count) {
  samples_.reserve(sample_count);
  split_starts_.reserve(1);
}

void Trajectory::assign(std::span<const TrajectorySample> samples,
                        Interpolation interpolation) {
  ValidateSamples(samples);
  samples_.assign(samples.begin(), samples.end());
  interpolation_ = interpolation;
  if (interpolation_ == Interpolation::kHermite) ZeroAccelerations(samples_);
  events_.clear();
  split_starts_.assign(1, 0);
}

void Trajectory::clear() {
  samples_.clear();
  events_.clear();
  split_starts_.clear();
}

double Trajectory::total_time_s() const {
  return samples_.empty() ? 0 : samples_.back().t_s;
}

TrajectorySample Trajectory::sample(double t_s) const {
  if (samples_.empty()) return {};
  // `!(t > first)` also sends NaN to the first sample.
  if (!(t_s > samples_.front().t_s)) return samples_.front();
  if (t_s >= samples_.back().t_s) return samples_.back();
  // First sample strictly after t_s; it exists and is not the first.
  const auto after = std::upper_bound(
      samples_.begin(), samples_.end(), t_s,
      [](double t, const TrajectorySample& s) { return t < s.t_s; });
  const TrajectorySample& b = *after;
  const TrajectorySample& a = *(after - 1);
  return interpolation_ == Interpolation::kConstantAcceleration
             ? ConstantAcceleration(a, t_s)
             : Hermite(a, b, t_s);
}

TrajectorySample Trajectory::initial_sample() const {
  return samples_.empty() ? TrajectorySample{} : samples_.front();
}

TrajectorySample Trajectory::final_sample() const {
  return samples_.empty() ? TrajectorySample{} : samples_.back();
}

geometry::SE2<double> Trajectory::initial_pose() const {
  return initial_sample().pose();
}

geometry::SE2<double> Trajectory::final_pose() const {
  return final_sample().pose();
}

std::span<const EventMarker> EventCursor::advance(const Trajectory& trajectory,
                                                  double t_s) {
  const std::span<const EventMarker> events = trajectory.events();
  next_ = std::min(next_, events.size());
  if (std::isnan(t_s)) return {};
  const auto first = events.begin() + static_cast<std::ptrdiff_t>(next_);
  const auto last = std::upper_bound(
      first, events.end(), t_s,
      [](double t, const EventMarker& e) { return t < e.t_s; });
  next_ = static_cast<std::size_t>(last - events.begin());
  return {first, last};
}

Trajectory Trajectory::split(std::size_t index) const {
  if (index >= split_starts_.size()) {
    throw std::out_of_range("trajectory split " + std::to_string(index) +
                            " of " + std::to_string(split_starts_.size()));
  }
  const std::size_t begin = split_starts_[index];
  const std::size_t end = index + 1 < split_starts_.size()
                              ? split_starts_[index + 1]
                              : samples_.size() - 1;
  const double start_s = samples_[begin].t_s;
  const double end_s = samples_[end].t_s;
  std::vector<TrajectorySample> samples(
      samples_.begin() + static_cast<std::ptrdiff_t>(begin),
      samples_.begin() + static_cast<std::ptrdiff_t>(end) + 1);
  for (TrajectorySample& s : samples) s.t_s -= start_s;
  const bool last_segment = index + 1 == split_starts_.size();
  std::vector<EventMarker> events;
  for (const EventMarker& e : events_) {
    if (e.t_s >= start_s &&
        (e.t_s < end_s || (last_segment && e.t_s == end_s))) {
      events.push_back({e.name, e.t_s - start_s});
    }
  }
  return Trajectory{std::move(samples), interpolation_, std::move(events)};
}

Trajectory Trajectory::flipped(AllianceFlip flip,
                               const FieldDimensions& field) const {
  if (!(std::isfinite(field.length_m) && field.length_m > 0 &&
        std::isfinite(field.width_m) && field.width_m > 0)) {
    throw std::invalid_argument("field dimensions must be finite and positive");
  }
  if (samples_.empty()) return {};
  std::vector<TrajectorySample> samples;
  samples.reserve(samples_.size());
  for (const TrajectorySample& s : samples_) {
    samples.push_back(Flip(s, flip, field));
  }
  return Trajectory{std::move(samples), interpolation_, events_, split_starts_};
}

}  // namespace talos::trajectory

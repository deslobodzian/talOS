#pragma once

// Item 40: characterization workflow.
//
// Gain estimation borrows the SysId workflow, not its app structure: drive
// one mechanism open-loop through a slow voltage ramp, record synchronized
// (voltage, position, velocity) samples, fit steady-state gains, and save the
// resulting config with provenance. Everything here is transport-free and
// deterministic so tests pin it without hardware:
//
//   Recorder: collects samples for one motor under one config id, enforcing
//     monotonic time and a single mechanism per recording.
//   FitStaticGains: least-squares fit of V = kS*sgn(vel) + kV*vel over samples
//     with |velocity| above a crawl threshold, plus R^2 so a bad dataset says
//     so instead of yielding confident nonsense.
//   ExportCsv / FormatProvenance: the dataset as CSV for external tooling,
//     and a provenance block (config id, motor, sample count, fit, operator
//     note) to store alongside any config the fit produces.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace talos::hardware {

struct CharSample {
  double t_s{0.0};
  double voltage_v{0.0};
  double position_rot{0.0};
  double velocity_rps{0.0};
};

struct CharDataset {
  uint16_t motor_id{0};
  uint64_t config_id{0};
  std::string note;
  std::vector<CharSample> samples;
};

struct CharFit {
  bool ok{false};
  double ks_v{0.0};           // Static feedforward (Volts).
  double kv_vs_per_rps{0.0};  // Velocity feedforward (Volt-seconds per rotation).
  double r_squared{0.0};
  std::size_t n{0};
  std::string detail;
};

class CharRecorder {
 public:
  bool Start(uint16_t motor_id, uint64_t config_id, std::string note = {}) {
    if (recording_) return false;
    recording_ = true;
    dataset_ = CharDataset{};
    dataset_.motor_id = motor_id;
    dataset_.config_id = config_id;
    dataset_.note = std::move(note);
    have_last_t_ = false;
    return true;
  }

  // Rejects samples from another motor/config, non-monotonic time, and
  // non-finite values. Returns false without recording on any violation.
  bool Add(uint16_t motor_id, uint64_t config_id, const CharSample& sample) {
    if (!recording_ || motor_id != dataset_.motor_id ||
        config_id != dataset_.config_id)
      return false;
    if (!std::isfinite(sample.t_s) || !std::isfinite(sample.voltage_v) ||
        !std::isfinite(sample.position_rot) ||
        !std::isfinite(sample.velocity_rps))
      return false;
    if (have_last_t_ && sample.t_s <= last_t_s_) return false;
    last_t_s_ = sample.t_s;
    have_last_t_ = true;
    dataset_.samples.push_back(sample);
    return true;
  }

  bool recording() const { return recording_; }
  std::size_t size() const { return dataset_.samples.size(); }

  // Ends the recording; empty recordings fail.
  bool Finish(CharDataset& out) {
    if (!recording_ || dataset_.samples.empty()) return false;
    out = dataset_;
    recording_ = false;
    return true;
  }

  void Abort() {
    recording_ = false;
    dataset_ = CharDataset{};
    have_last_t_ = false;
  }

 private:
  bool recording_{false};
  CharDataset dataset_;
  double last_t_s_{0.0};
  bool have_last_t_{false};
};

inline CharFit FitStaticGains(const CharDataset& dataset,
                              double min_speed_rps = 0.5) {
  CharFit fit;
  double sum_y = 0.0, sum_y2 = 0.0;
  double sum_ss = 0.0, sum_sv = 0.0, sum_vv = 0.0;
  double sum_sy = 0.0, sum_vy = 0.0;
  std::size_t n = 0;
  for (const auto& s : dataset.samples) {
    if (!std::isfinite(s.voltage_v) || !std::isfinite(s.velocity_rps)) continue;
    if (std::abs(s.velocity_rps) < min_speed_rps) continue;
    const double sign = s.velocity_rps >= 0.0 ? 1.0 : -1.0;
    sum_ss += 1.0;
    sum_sv += sign * s.velocity_rps;
    sum_vv += s.velocity_rps * s.velocity_rps;
    sum_sy += sign * s.voltage_v;
    sum_vy += s.velocity_rps * s.voltage_v;
    sum_y += s.voltage_v;
    sum_y2 += s.voltage_v * s.voltage_v;
    ++n;
  }
  fit.n = n;
  if (n < 3) {
    fit.detail = "need at least 3 steady samples above the crawl threshold";
    return fit;
  }
  // Solve [ss sv; sv vv] [ks; kv] = [sy; vy].
  const double det = sum_ss * sum_vv - sum_sv * sum_sv;
  if (!(det > 1e-9)) {
    fit.detail = "degenerate dataset: no velocity spread (one-sided ramp?)";
    return fit;
  }
  fit.ks_v = (sum_vv * sum_sy - sum_sv * sum_vy) / det;
  fit.kv_vs_per_rps = (sum_ss * sum_vy - sum_sv * sum_sy) / det;
  const double mean_y = sum_y / n;
  double ss_tot = sum_y2 - 2 * mean_y * sum_y + n * mean_y * mean_y;
  if (!(ss_tot > 0.0)) {
    fit.detail = "degenerate dataset: no voltage spread";
    return fit;
  }
  double ss_res = 0.0;
  for (const auto& s : dataset.samples) {
    if (std::abs(s.velocity_rps) < min_speed_rps) continue;
    const double sign = s.velocity_rps >= 0.0 ? 1.0 : -1.0;
    const double pred = fit.ks_v * sign + fit.kv_vs_per_rps * s.velocity_rps;
    const double r = s.voltage_v - pred;
    ss_res += r * r;
  }
  fit.r_squared = 1.0 - ss_res / ss_tot;
  fit.ok = fit.ks_v >= 0.0 && fit.kv_vs_per_rps >= 0.0 && fit.r_squared > 0.0;
  if (!fit.ok) fit.detail = "fit non-physical (negative gain or R^2 <= 0)";
  return fit;
}

inline std::string ExportCharCsv(const CharDataset& dataset) {
  std::ostringstream out;
  out << "t_s,voltage_v,position_rot,velocity_rps\n";
  for (const auto& s : dataset.samples) {
    out << s.t_s << "," << s.voltage_v << "," << s.position_rot << ","
        << s.velocity_rps << "\n";
  }
  return out.str();
}

// Provenance block to store alongside any config produced from this fit:
// which manifest, which mechanism, how many samples, what the fit said, and
// who ran it. Gains without this block are unreviewable.
inline std::string FormatCharProvenance(const CharDataset& dataset,
                                        const CharFit& fit,
                                        const std::string& operator_note) {
  std::ostringstream out;
  out << "characterization provenance:\n";
  out << "  motor_id=" << dataset.motor_id
      << " config_id=" << dataset.config_id << "\n";
  out << "  samples=" << dataset.samples.size() << " fitted=" << fit.n << "\n";
  out << "  ks_v=" << fit.ks_v << " kv_vs_per_rps=" << fit.kv_vs_per_rps
      << " r_squared=" << fit.r_squared << (fit.ok ? " OK" : " REJECTED") << "\n";
  out << "  dataset_note=" << dataset.note << "\n";
  out << "  operator_note=" << operator_note << "\n";
  return out.str();
}

}  // namespace talos::hardware

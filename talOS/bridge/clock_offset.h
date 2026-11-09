#pragma once

// Gateway-to-node clock mapping. Production uses bounded_offset_us() from a
// causal two-way exchange; offset_us()/spread_us() retain the legacy one-way
// diagnostic below and MUST NOT be treated as an accuracy bound.
// accuracy_valid() expires after one second of sample progress. Its error
// bound assumes nonnegative delay and <=1000 ppm relative oscillator drift.
// No fusion accuracy threshold has been specified.
//
// Legacy one-way diagnostic (not the production mapping):
//
// Each gateway state the bridge receives gives one one-way delay
//
//   d = receipt_us (bridge clock) - sample_time_us (gateway clock)
//     = true offset between the clocks + that sample's latency,
//
// where the latency (>= 0) is how long the sample took from the gateway's
// sampler to the bridge's receive: gateway processing, UDP, and up to one
// bridge tick. The estimate is the minimum d over the window, so
//
//   node time of gateway time t  =  t + offset_us()
//
// is LATER than the true node time of t by the smallest latency in the
// window. That latency cannot be observed from one-way traffic, so this is an
// upper-bounded latency estimate, not clock synchronisation: a mapped sample
// time is never earlier than the true one, and node_now - mapped never
// exceeds the sample's true age.
//
// spread_us() is max d - min d over the window: how much the latency varied.
// It bounds how far the latest sample's receipt time lies after its mapped
// sample time (receipt - mapped is in [0, spread]); it does not bound the
// unobservable minimum latency above.
//
// The window is the last kWindowSamples samples accepted in the current
// gateway boot (one second at the default 5 ms period), so the estimate
// follows drift between the two oscillators: a bridge clock that runs slow
// against the gateway lowers d, and the new minimum is taken at once; one
// that runs fast raises d, and the old minimum ages out within the window, so
// the estimate lags by at most window duration x drift rate (about 100 us at
// 100 ppm).
//
// A new gateway boot (a boot_id not seen before) restarts the window: the
// gateway clock restarts with the boot. A sample whose sample_time_us is not
// after the last one accepted in this boot (a duplicate or a reordered frame)
// is ignored, and so is one from a boot the gateway has since left
// (BootHistory: a datagram delayed across a reboot). Observe() is the
// bridge's State ordering gate: a State it ignores is dropped whole.
// The estimate is valid -- converged() -- once the window holds
// kConvergedSamples samples of the current boot (40 ms at 5 ms).

#include <array>
#include <cstddef>
#include <cstdint>

#include "talOS/bridge/boot_history.h"

namespace talos::hardware {

class ClockOffsetEstimator {
 public:
  static constexpr std::size_t kWindowSamples = 200;
  static constexpr std::size_t kConvergedSamples = 8;

  // One received gateway state: its boot, its gateway-clock sample time and
  // the bridge-clock time it was received. Returns whether it entered the
  // window (false: a duplicate, reordered or retired-boot sample, ignored).
  bool Observe(std::uint64_t boot_id, std::uint64_t sample_time_us,
               std::uint64_t receipt_us);

  // Causal two-way interval: probe sent at bridge t1, later gateway sample
  // t2, bridge time t4 sampled AFTER receiving that State. No symmetry
  // assumption: offset lies in [t1 - t2, t4 - t2]. Residence time is retained
  // in RTT, making this looser but safer than subtracting guessed processing.
  void ObserveExchange(std::uint64_t boot_id, std::uint64_t t1,
                       std::uint64_t t2, std::uint64_t t4);
  bool accuracy_valid() const { return exchange_valid_; }
  std::int64_t bounded_offset_us() const { return bounded_offset_us_; }
  std::uint64_t uncertainty_us() const { return uncertainty_us_; }
  std::uint64_t round_trip_us() const { return round_trip_us_; }

  bool converged() const { return count_ >= kConvergedSamples; }
  // The gateway boot the window holds samples of; 0 before the first.
  std::uint64_t boot_id() const { return boots_.id(); }
  // Minimum one-way delay over the window (bridge clock minus gateway clock);
  // 0 while the window is empty. Meaningful once converged().
  std::int64_t offset_us() const { return offset_us_; }
  // Maximum minus minimum one-way delay over the window.
  std::uint64_t spread_us() const { return spread_us_; }
  std::size_t window_count() const { return count_; }
  // Restarts for a new gateway boot after the first.
  std::uint64_t resets_count() const { return resets_; }
  // Samples ignored as duplicates, reordered or from a retired boot.
  std::uint64_t ignored_count() const { return ignored_; }

 private:
  std::array<std::int64_t, kWindowSamples> delays_us_{};  // ring buffer
  std::size_t next_{0};   // where the next delay goes
  std::size_t count_{0};  // delays held, at most kWindowSamples
  BootHistory boots_;
  std::uint64_t last_sample_time_us_{0};
  std::int64_t offset_us_{0};
  std::uint64_t spread_us_{0};
  std::uint64_t resets_{0};
  std::uint64_t ignored_{0};
  bool exchange_valid_{false};
  std::int64_t bounded_offset_us_{};
  std::uint64_t uncertainty_us_{};
  std::uint64_t round_trip_us_{};
  std::uint64_t exchange_time_us_{};
  std::uint64_t exchange_sample_us_{};
};

}  // namespace talos::hardware

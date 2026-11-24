#pragma once

// Gateway-to-node clock mapping. Production uses bounded_offset_us() from a
// causal two-way exchange; offset_us()/spread_us() are the one-way diagnostic
// below and MUST NOT be treated as an accuracy bound.
// The estimator keeps every exchange of the current boot from the last
// second (up to kExchangeSlots) and, at each State, uses the minimum-RTT one
// that is still at most one second old. A kept exchange therefore expires
// only while a younger one is held, and accuracy_valid() stays true as long
// as exchanges keep arriving less than a second apart. Its error bound
// assumes nonnegative delay and <=1000 ppm relative oscillator drift. No
// fusion accuracy threshold has been specified.
//
// One-way diagnostic (not the production mapping):
//
// Each gateway state the bridge receives gives one one-way delay
//
//   d = receipt_us (bridge clock) - sample_time_us (gateway clock)
//     = true offset between the clocks + that sample's latency,
//
// where the latency (>= 0) is how long the sample took from the gateway's
// sampler to the bridge's receive: gateway processing, UDP, and the bridge
// waking for it. The estimate is the minimum d over the window, so
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
// gateway clock restarts with the boot, and its first accepted State sets the
// boot's baseline (gateway and bridge clocks are independent monotonic clocks
// with an arbitrary constant offset, so that first frame's absolute value
// cannot be bounded without rolling back within a boot, which REQ-SAFE-11
// forbids). Within the current boot, a sample whose sample_time_us is not
// after the last one accepted (a duplicate or reordered frame) is ignored, and
// so is one from a boot the gateway has since left (BootHistory: a datagram
// delayed across a reboot). Forward jumps within a boot are checked against a
// per-boot prompt anchor: a sample advancing farther ahead of the anchor than
// elapsed bridge time plus kMaxSampleJumpMarginUs (1 s) and
// 1 / kMaxClockDriftDivisor (2000 ppm, twice the 1000 ppm relative oscillator
// drift assumed above) of elapsed bridge time allows is held as a single-slot
// jump candidate and ignored; any normal accepted sample or boot change clears
// that candidate (a duplicate or reordered sample returns early before
// touching it), while the next sample past the bound strictly advancing past
// the candidate with non-decreasing bridge receipt time and a symmetrically
// consistent physical clock rate re-syncs the anchor and is accepted (so an
// unseen-boot SIGSTOP / SO_RCVBUF tail-drop recovers on the second live frame
// while isolated or inconsistent far-future timestamps are ignored).
// On normal accepted frames, the anchor advances immediately only when the
// sample is at least as prompt as it (sample_delta >= receipt_delta), so a
// delayed buffered frame after a long outage never pulls the anchor back; when
// sustained bridge-fast drift keeps samples less prompt for at least
// kConvergedSamples frames spanning kAnchorRefreshUs (1 s) of bridge receipt
// time, the anchor promotes to the most prompt sample of that window so the
// drift allowance stays bounded over long runs. Observe() is the bridge's
// State ordering gate: a State it ignores is dropped whole. The estimate is
// valid -- converged() -- once the window holds kConvergedSamples samples of
// the current boot (40 ms at 5 ms).

#include <array>
#include <cstddef>
#include <cstdint>

#include "talOS/hardware/boot_history.h"

namespace talos::hardware {

class ClockOffsetEstimator {
 public:
  static constexpr std::size_t kWindowSamples = 200;
  static constexpr std::size_t kConvergedSamples = 8;
  // A second of exchanges at the bridge's 50 Hz probe rate, with margin. A
  // faster caller overwrites the oldest; any exchange kept is still causal.
  static constexpr std::size_t kExchangeSlots = 64;
  static constexpr std::uint64_t kExchangeLifetimeUs = 1'000'000;
  // Forward-jump bound for State sample_time_us within a boot: allows up to
  // kMaxSampleJumpMarginUs (1 s) of one-way jitter / burst catch-up plus
  // 1 / kMaxClockDriftDivisor (2000 ppm) of elapsed bridge time since the
  // prompt anchor so arbitrarily long outages recover cleanly while rejecting
  // corrupt far-future timestamps that would otherwise lock out later States.
  static constexpr std::uint64_t kMaxSampleJumpMarginUs = 1'000'000;
  static constexpr std::uint64_t kMaxClockDriftDivisor = 500;
  // Sustained bridge-receipt window required (with at least kConvergedSamples
  // accepted frames) before a slower-than-anchor minimum promotes to the
  // prompt anchor. Prevents a single delayed packet or burst after a long
  // outage from pulling the anchor back while keeping the drift allowance
  // bounded under steady bridge-fast oscillator drift.
  static constexpr std::uint64_t kAnchorRefreshUs = 1'000'000;

  // One received gateway state: its boot, its gateway-clock sample time and
  // the bridge-clock time it was received. Returns whether it entered the
  // window (false: a duplicate, reordered, far-future or retired-boot sample,
  // ignored).
  bool Observe(std::uint64_t boot_id, std::uint64_t sample_time_us,
               std::uint64_t receipt_us);

  // Causal two-way interval: probe sent at bridge t1, later gateway sample
  // t2, bridge time t4 sampled AFTER receiving that State. No symmetry
  // assumption: offset lies in [t1 - t2, t4 - t2]. Residence time is retained
  // in RTT, making this looser but safer than subtracting guessed processing.
  // Ignored for another boot, t1 == 0, t4 < t1, or a round trip over 1 s.
  void ObserveExchange(std::uint64_t boot_id, std::uint64_t t1,
                       std::uint64_t t2, std::uint64_t t4);
  // Advances exchange expiry to bridge clock now_us without a new State:
  // re-selects among kept exchanges still within kExchangeLifetimeUs of now_us
  // and clears accuracy_valid() (zeroing bounded_offset_us(), round_trip_us()
  // and uncertainty_us()) once all have expired. Safe with no prior
  // observations or now_us earlier than the last selection.
  void Expire(std::uint64_t now_us);
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
  // Samples ignored as duplicates, reordered, far-future or from a retired
  // boot.
  std::uint64_t ignored_count() const { return ignored_; }

 private:
  struct Exchange {
    std::int64_t offset_us;       // midpoint of [t1 - t2, t4 - t2]
    std::uint64_t round_trip_us;  // t4 - t1
    std::uint64_t sample_us;      // t2, gateway clock
    std::uint64_t receipt_us;     // t4, bridge clock
  };

  // Picks the minimum-RTT kept exchange at most kExchangeLifetimeUs old at
  // (bridge receipt_us, gateway sample_time_us), sets accuracy_valid() and
  // the bound from it, or zeroes the bound fields when none remains.
  void SelectExchange(std::uint64_t receipt_us, std::uint64_t sample_time_us);

  std::array<std::int64_t, kWindowSamples> delays_us_{};  // ring buffer
  std::size_t next_{0};   // where the next delay goes
  std::size_t count_{0};  // delays held, at most kWindowSamples
  BootHistory boots_;
  std::uint64_t last_sample_time_us_{0};
  // Prompt baseline of the current boot (always <= last_sample_time_us_), used
  // to bound forward sample_time_us jumps against elapsed bridge receipt time.
  std::uint64_t anchor_sample_us_{0};
  std::uint64_t anchor_receipt_us_{0};
  // Most prompt accepted sample observed during a sustained slower-than-anchor
  // spell; promotes to anchor_* once at least kConvergedSamples frames span
  // kAnchorRefreshUs of bridge receipt time.
  std::uint64_t next_anchor_sample_us_{0};
  std::uint64_t next_anchor_receipt_us_{0};
  std::uint64_t next_anchor_since_us_{0};
  std::size_t next_anchor_count_{0};
  // Single-slot forward-jump candidate: holds the most recent rejected
  // far-future sample until cleared by a normal accepted sample or a new boot
  // (duplicates and reordered samples return early without touching it), or
  // confirmed by the next sample past the bound at a physically consistent
  // rate.
  bool has_jump_candidate_{false};
  std::uint64_t jump_sample_us_{0};
  std::uint64_t jump_receipt_us_{0};
  std::int64_t offset_us_{0};
  std::uint64_t spread_us_{0};
  std::uint64_t resets_{0};
  std::uint64_t ignored_{0};
  bool exchange_valid_{false};
  std::int64_t bounded_offset_us_{};
  std::uint64_t uncertainty_us_{};
  std::uint64_t round_trip_us_{};
  std::array<Exchange, kExchangeSlots> exchanges_{};  // ring buffer
  std::size_t next_exchange_{0};
  std::size_t exchange_count_{0};
  std::uint64_t last_select_receipt_us_{0};
  std::uint64_t max_exchange_sample_us_{0};
};

}  // namespace talos::hardware

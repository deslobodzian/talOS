#pragma once

// Command-path measurements for the hardware bridge (perf items 42/46).
//
// Clock rule, stated once so no caller has to re-derive it: the gateway clock
// (State::sample_time_us, Command::observed_time_us) and the bridge clock
// (the now_us passed to HardwareNode::Tick) are different monotonic clocks
// with no known relationship. This file NEVER takes one timestamp from each
// clock, so it can never subtract across clocks. Gateway-side values enter
// only as opaque durations (Phoenix signal age_us fields, already ages in
// microseconds at poll time); bridge-side values enter only as bridge now_us
// samples from which this file differences consecutive arrivals on the same
// clock. Request publish-to-merge latency is therefore NOT reported as a
// number: with LATEST reads every 1 ms tick and subsystem periods >= 5 ms it
// is bounded above by one tick period plus scheduling jitter, and the
// components of that bound (tick duration, request inter-arrival, missed
// deadlines) are what is measured here.
//
// Item 46 (profile before optimizing; no complexity for small fixed-size
// copies): the windows below are fixed 256-entry rings and the percentile is
// a copy plus std::sort. That is the whole implementation, deliberately.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace talos::hardware {

// Fixed-window reservoir over microsecond samples from ONE clock.
class Histogram {
 public:
  void Record(uint64_t value_us) {
    buffer_[head_ % kWindow] = value_us;
    ++head_;
    if (count_ < kWindow) ++count_;
  }

  uint64_t count() const { return count_; }

  uint64_t Max() const {
    uint64_t worst = 0;
    for (uint64_t i = 0; i < count_; ++i) worst = std::max(worst, buffer_[i]);
    return worst;
  }

  // Nearest-rank percentile in [0, 1]. Empty window reports 0.
  uint64_t Percentile(double quantile) const {
    if (count_ == 0) return 0;
    const double clamped = std::clamp(quantile, 0.0, 1.0);
    std::array<uint64_t, kWindow> sorted{};
    for (uint64_t i = 0; i < count_; ++i) sorted[i] = buffer_[i];
    std::sort(sorted.begin(), sorted.begin() + count_);
    uint64_t rank = static_cast<uint64_t>(clamped * count_);
    if (rank == 0) rank = 1;
    if (rank > count_) rank = count_;
    return sorted[rank - 1];
  }

  uint64_t P50() const { return Percentile(0.5); }
  uint64_t P99() const { return Percentile(0.99); }

 private:
  static constexpr std::size_t kWindow = 256;
  std::array<uint64_t, kWindow> buffer_{};
  uint64_t head_{0};
  uint64_t count_{0};
};

// One bridge's command-path evidence bundle. All On* methods take
// bridge-clock samples, except RecordSignalAge which takes an already-aged
// Phoenix duration. See the clock rule above.
struct CommandPathMetrics {
  Histogram tick_us;         // Tick() wall processing time, bridge clock.
  Histogram state_gap_us;    // Bridge-clock inter-arrival of accepted states.
  Histogram request_gap_us;  // Bridge-clock inter-arrival of decoded requests.
  Histogram send_gap_us;     // Bridge-clock intervals between command sends.
  Histogram signal_age_us;   // Phoenix cached signal ages (opaque durations).
  uint64_t missed_deadlines{0};
  uint64_t suppressed_sends{0};

  void OnTickDone(uint64_t duration_us) { tick_us.Record(duration_us); }

  void OnStateReceived(uint64_t bridge_now_us) {
    if (have_state_) state_gap_us.Record(bridge_now_us - last_state_us_);
    last_state_us_ = bridge_now_us;
    have_state_ = true;
  }

  void OnRequestReceived(uint64_t bridge_now_us) {
    if (have_request_) request_gap_us.Record(bridge_now_us - last_request_us_);
    last_request_us_ = bridge_now_us;
    have_request_ = true;
  }

  void OnCommandSent(uint64_t bridge_now_us) {
    if (have_send_) send_gap_us.Record(bridge_now_us - last_send_us_);
    last_send_us_ = bridge_now_us;
    have_send_ = true;
  }

  void RecordSignalAge(uint32_t age_us) { signal_age_us.Record(age_us); }
  void RecordMissedDeadline() { ++missed_deadlines; }
  void RecordSuppressedSend() { ++suppressed_sends; }

 private:
  uint64_t last_state_us_{0};
  uint64_t last_request_us_{0};
  uint64_t last_send_us_{0};
  bool have_state_{false};
  bool have_request_{false};
  bool have_send_{false};
};

}  // namespace talos::hardware

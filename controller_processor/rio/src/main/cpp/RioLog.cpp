#include "RioLog.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

#include "hardware/messages.h"

#ifdef __linux__
#include <pthread.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace talos::rio {
namespace {

template <std::size_t N>
void CopyBounded(char (&out)[N], std::string_view value) {
  const std::size_t n = std::min(value.size(), N - 1);
  std::memcpy(out, value.data(), n);
  out[n] = '\0';
}

uint64_t SteadyNowUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

const char* Or(const char* value, const char* fallback) {
  return value ? value : fallback;
}

// The least age that is a difference of two readings of one clock gone
// negative, not an age (RioLog::RecordCommandAge).
constexpr uint64_t kWrappedAgeUs = uint64_t{1} << 63;

}  // namespace

void FormatConsoleTime(uint64_t steady_us, char (&out)[kConsoleTimeSize]) {
  std::snprintf(out, sizeof(out), "[%llu.%03llu] ",
                static_cast<unsigned long long>(steady_us / 1'000'000),
                static_cast<unsigned long long>(steady_us / 1'000 % 1'000));
}

std::string ConsoleTime(uint64_t steady_us) {
  char text[kConsoleTimeSize];
  FormatConsoleTime(steady_us, text);
  return text;
}

void LogEvent::SetName(std::string_view value) { CopyBounded(name, value); }
void LogEvent::SetBus(std::string_view value) { CopyBounded(bus, value); }
void LogEvent::SetText(std::string_view value) { CopyBounded(text, value); }

void RioLog::WriteConsoleLine(uint64_t steady_us, std::string_view line,
                              std::FILE* stream) noexcept {
  // One write, so another thread's console line cannot split it, formatted on
  // the stack so the low-priority logger thread does not allocate here.
  char prefix[kConsoleTimeSize];
  FormatConsoleTime(steady_us, prefix);
  const std::size_t prefix_len = std::strlen(prefix);
  char buffer[2048];
  if (prefix_len + line.size() <= sizeof(buffer)) {
    std::copy_n(prefix, prefix_len, buffer);
    if (!line.empty()) {
      std::copy_n(line.data(), line.size(), buffer + prefix_len);
    }
    std::fwrite(buffer, 1, prefix_len + line.size(), stream);
  } else {
    std::fwrite(prefix, 1, prefix_len, stream);
    std::fwrite(line.data(), 1, line.size(), stream);
  }
  std::fflush(stream);
}

RioLog::RioLog(Sink sink)
    : sink_{std::move(sink)},
      cells_{std::make_unique<std::array<Cell, kCapacity>>()},
      windows_{std::make_unique<
          std::array<std::array<Window, kKeysPerKind>,
                     static_cast<std::size_t>(LogKind::kCount)>>()} {
  static_assert((kCapacity & (kCapacity - 1)) == 0,
                "the queue indexes cells with a mask");
  for (std::size_t i = 0; i < kCapacity; ++i)
    (*cells_)[i].sequence.store(i, std::memory_order_relaxed);
  if (!sink_) {
    sink_ = [](std::string_view line) {
      WriteConsoleLine(SteadyNowUs(), line, stderr);
    };
  }
}

RioLog::~RioLog() { Stop(); }

bool RioLog::Post(const LogEvent& event) {
  std::size_t position = enqueue_.load(std::memory_order_relaxed);
  for (;;) {
    Cell& cell = (*cells_)[position & (kCapacity - 1)];
    const std::size_t sequence = cell.sequence.load(std::memory_order_acquire);
    // Unsigned difference first: no signed overflow when the counters wrap.
    const auto difference = static_cast<std::ptrdiff_t>(sequence - position);
    if (difference == 0) {
      if (enqueue_.compare_exchange_weak(position, position + 1,
                                         std::memory_order_relaxed)) {
        cell.event = event;
        cell.sequence.store(position + 1, std::memory_order_release);
        return true;
      }
    } else if (difference < 0) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    } else {
      position = enqueue_.load(std::memory_order_relaxed);
    }
  }
}

bool RioLog::Pop(LogEvent& event) {
  std::size_t position = dequeue_.load(std::memory_order_relaxed);
  for (;;) {
    Cell& cell = (*cells_)[position & (kCapacity - 1)];
    const std::size_t sequence = cell.sequence.load(std::memory_order_acquire);
    const auto difference =
        static_cast<std::ptrdiff_t>(sequence - (position + 1));
    if (difference == 0) {
      if (dequeue_.compare_exchange_weak(position, position + 1,
                                         std::memory_order_relaxed)) {
        event = cell.event;
        cell.sequence.store(position + kCapacity, std::memory_order_release);
        return true;
      }
    } else if (difference < 0) {
      return false;
    } else {
      position = dequeue_.load(std::memory_order_relaxed);
    }
  }
}

void RioLog::RecordTick(uint64_t interval_us, uint64_t duration_us,
                        uint64_t period_us) {
  // Update the current 1 s timing window and publish counters, maxima, and
  // the single worst-work tick's phases atomically into timing_[epoch & 1]
  // via a lock-free atomic seqlock, before tick_phase_ns_ is zeroed.
  const uint64_t epoch = timing_epoch_.load(std::memory_order_acquire);
  const bool new_epoch = epoch != local_timing_epoch_;
  if (new_epoch) {
    local_timing_epoch_ = epoch;
    local_window_overruns_ = 0;
    local_window_late_ticks_ = 0;
    local_max_tick_us_ = 0;
    local_max_interval_us_ = 0;
    local_period_us_ = 0;
    local_worst_phase_ns_ = {};
  }
  const bool is_overrun = duration_us > period_us;
  const bool is_late = interval_us > period_us + period_us / 2;
  if (is_overrun) {
    ++local_window_overruns_;
    ++local_total_overruns_;
    overruns_.store(local_total_overruns_, std::memory_order_relaxed);
  }
  if (is_late) {
    ++local_window_late_ticks_;
    ++local_total_late_ticks_;
    late_ticks_.store(local_total_late_ticks_, std::memory_order_relaxed);
  }
  const bool new_worst = new_epoch || duration_us > local_max_tick_us_;
  if (new_worst) {
    local_max_tick_us_ = duration_us;
    local_worst_phase_ns_ = tick_phase_ns_;
  }
  const bool new_max_interval =
      new_epoch || interval_us > local_max_interval_us_;
  if (new_max_interval) {
    local_max_interval_us_ = interval_us;
  }
  const bool new_period = new_epoch || period_us != local_period_us_;
  if (new_period) {
    local_period_us_ = period_us;
  }
  if (new_epoch || is_overrun || is_late || new_worst || new_max_interval ||
      new_period) {
    TimingSlot& slot = timing_[epoch & 1u];
    const uint64_t seq = slot.seq.load(std::memory_order_relaxed);
    // Lock-free seqlock without standalone fences (-Werror=tsan rejects
    // atomic_thread_fence): odd acq_rel + release payload stores pair with
    // acquire payload loads in ReadTimingSlot() so any torn read from a newer
    // write synchronizes-with the odd sequence write before seq2 is read.
    slot.seq.exchange(seq + 1, std::memory_order_acq_rel);
    slot.epoch.store(epoch, std::memory_order_release);
    slot.overruns.store(local_window_overruns_, std::memory_order_release);
    slot.late_ticks.store(local_window_late_ticks_, std::memory_order_release);
    slot.max_tick_us.store(local_max_tick_us_, std::memory_order_release);
    slot.max_interval_us.store(local_max_interval_us_,
                               std::memory_order_release);
    slot.period_us.store(local_period_us_, std::memory_order_release);
    for (std::size_t i = 0; i < kPhaseCount; ++i) {
      slot.worst_phase_ns[i].store(local_worst_phase_ns_[i],
                                   std::memory_order_release);
    }
    slot.seq.store(seq + 2, std::memory_order_release);
  }

  // Hand the logger the window it asked for, without this tick, which opens
  // the next one. The acquire re-load runs only on a new request, once per
  // window: it reads the value the logger stored with release (it is the only
  // writer, and does not store again until this handover is served), so the
  // logger's clear of the latency set it printed happens-before this
  // switch back to it, in a form ThreadSanitizer models (it does not model a
  // standalone fence).
  if (phase_request_.load(std::memory_order_relaxed) != phase_served_local_) {
    const uint32_t request = phase_request_.load(std::memory_order_acquire);
    published_ticks_.store(phase_window_.ticks, std::memory_order_relaxed);
    for (std::size_t i = 0; i < kPhaseCount; ++i) {
      published_sum_ns_[i].store(phase_window_.sum_ns[i],
                                 std::memory_order_relaxed);
      published_max_ns_[i].store(phase_window_.max_ns[i],
                                 std::memory_order_relaxed);
    }
    published_motors_.store(phase_window_.motors, std::memory_order_relaxed);
    published_sensors_.store(phase_window_.sensors, std::memory_order_relaxed);
    latency_published_ = latency_front_;
    latency_front_ ^= 1;
    phase_served_.store(request, std::memory_order_release);
    phase_served_local_ = request;
    phase_window_ = {};
  }
  ++phase_window_.ticks;
  for (std::size_t i = 0; i < kPhaseCount; ++i) {
    phase_window_.sum_ns[i] += tick_phase_ns_[i];
    phase_window_.max_ns[i] =
        std::max(phase_window_.max_ns[i], tick_phase_ns_[i]);
    tick_phase_ns_[i] = 0;
  }
  tick_total_ns_ = 0;
  boundary_total_ns_ = 0;
  phase_window_.motors = tick_motors_;
  phase_window_.sensors = tick_sensors_;
  FoldCommandAge();
  FoldDriverStationAge();
}

void RioLog::RecordCommandAge(uint64_t age_us, bool accepted) {
  if (age_us >= kWrappedAgeUs) return;
  FoldCommandAge();
  tick_command_age_ = {true, accepted, age_us};
}

void RioLog::RecordDriverStationAge(uint64_t age_us) {
  if (age_us >= kWrappedAgeUs) return;
  FoldDriverStationAge();
  tick_driver_station_age_ = {true, false, age_us};
}

void RioLog::FoldCommandAge() {
  if (!tick_command_age_.present) return;
  auto& window = latency_[latency_front_];
  (tick_command_age_.accepted ? window.accepted : window.refused)
      .Add(tick_command_age_.age_us);
  tick_command_age_.present = false;
}

void RioLog::FoldDriverStationAge() {
  if (!tick_driver_station_age_.present) return;
  latency_[latency_front_].driver_station.Add(tick_driver_station_age_.age_us);
  tick_driver_station_age_.present = false;
}

void RioLog::LatencyHistogram::Add(uint64_t age_us) {
  const uint64_t bucket = age_us / kLatencyBucketUs;
  ++counts[bucket < kLatencyBuckets ? bucket : kLatencyBuckets];
  ++samples;
  max_us = std::max(max_us, age_us);
}

std::string RioLog::LatencyHistogram::Summary() const {
  if (samples == 0) return "-/-/- us over 0";
  // Nearest rank: the upper edge of the bucket holding the ceil(p% of n)th
  // smallest sample, never above the exact max (so the overflow bucket's
  // percentiles read as the max).
  const auto percentile = [this](uint64_t percent) {
    const uint64_t rank = (samples * percent + 99) / 100;
    uint64_t seen = 0;
    for (std::size_t i = 0; i < kLatencyBuckets; ++i) {
      seen += counts[i];
      if (seen >= rank) return std::min((i + 1) * kLatencyBucketUs, max_us);
    }
    return max_us;
  };
  char text[96];
  std::snprintf(text, sizeof(text), "%llu/%llu/%llu us over %llu",
                static_cast<unsigned long long>(percentile(50)),
                static_cast<unsigned long long>(percentile(99)),
                static_cast<unsigned long long>(max_us),
                static_cast<unsigned long long>(samples));
  return text;
}

void RioLog::AddPhase(TickPhase phase, uint64_t duration_ns) {
  const auto i = static_cast<std::size_t>(phase);
  if (i >= kPhaseCount) return;
  tick_phase_ns_[i] += duration_ns;
  tick_total_ns_ += duration_ns;
}

void RioLog::BeginTick(uint64_t now_ns) {
  boundary_ns_ = now_ns;
  boundary_total_ns_ = tick_total_ns_;
}

void RioLog::ClosePhase(TickPhase phase, uint64_t now_ns) {
  const uint64_t elapsed = now_ns - boundary_ns_;
  const uint64_t nested = tick_total_ns_ - boundary_total_ns_;
  // The backend's phases lie inside the interval on the same clock, so
  // `nested` never exceeds it; the guard keeps a clock fault from wrapping.
  AddPhase(phase, elapsed > nested ? elapsed - nested : 0);
  boundary_ns_ = now_ns;
  boundary_total_ns_ = tick_total_ns_;
}

void RioLog::SetTickDevices(std::size_t motors, std::size_t sensors) {
  tick_motors_ = static_cast<uint32_t>(motors);
  tick_sensors_ = static_cast<uint32_t>(sensors);
}

uint64_t RioLog::max_tick_us() const {
  const uint64_t epoch = timing_epoch_.load(std::memory_order_relaxed);
  const TimingSlot& slot = timing_[epoch & 1u];
  return slot.epoch.load(std::memory_order_relaxed) == epoch
             ? slot.max_tick_us.load(std::memory_order_relaxed)
             : 0;
}

uint64_t RioLog::max_interval_us() const {
  const uint64_t epoch = timing_epoch_.load(std::memory_order_relaxed);
  const TimingSlot& slot = timing_[epoch & 1u];
  return slot.epoch.load(std::memory_order_relaxed) == epoch
             ? slot.max_interval_us.load(std::memory_order_relaxed)
             : 0;
}

RioLog::TimingSnapshot RioLog::ReadTimingSlot(uint64_t expected_epoch) const {
  const TimingSlot& slot = timing_[expected_epoch & 1u];
  for (int attempt = 0; attempt < 64; ++attempt) {
    const uint64_t seq1 = slot.seq.load(std::memory_order_acquire);
    if ((seq1 & 1u) != 0) {
      std::this_thread::yield();
      continue;
    }
    TimingSnapshot snapshot{};
    snapshot.epoch = slot.epoch.load(std::memory_order_acquire);
    snapshot.overruns = slot.overruns.load(std::memory_order_acquire);
    snapshot.late_ticks = slot.late_ticks.load(std::memory_order_acquire);
    snapshot.max_tick_us = slot.max_tick_us.load(std::memory_order_acquire);
    snapshot.max_interval_us =
        slot.max_interval_us.load(std::memory_order_acquire);
    snapshot.period_us = slot.period_us.load(std::memory_order_acquire);
    for (std::size_t i = 0; i < kPhaseCount; ++i) {
      snapshot.worst_phase_ns[i] =
          slot.worst_phase_ns[i].load(std::memory_order_acquire);
    }
    const uint64_t seq2 = slot.seq.load(std::memory_order_acquire);
    if (seq1 == seq2) {
      snapshot.valid = true;
      return snapshot;
    }
    std::this_thread::yield();
  }
  return {};
}

void RioLog::EmitPhasesAndLatency(const PhaseWindow& window,
                                  const LatencyWindow& latency) {
  std::array<double, kPhaseCount> mean_us{};
  std::array<double, kPhaseCount> max_us{};
  for (std::size_t i = 0; i < kPhaseCount; ++i) {
    mean_us[i] = static_cast<double>(window.sum_ns[i]) /
                 static_cast<double>(window.ticks) / 1e3;
    max_us[i] = static_cast<double>(window.max_ns[i]) / 1e3;
  }
  const auto at = [](TickPhase phase) {
    return static_cast<std::size_t>(phase);
  };
  char text[512];
  std::snprintf(
      text, sizeof(text),
      "[rio] tick phases: refresh mean/max %.1f/%.1f us, devices "
      "%.1f/%.1f us, control %.1f/%.1f us, health %.1f/%.1f us, other "
      "%.1f/%.1f us, link %.1f/%.1f us, driver station %.1f/%.1f us, "
      "loop %.1f/%.1f us over %llu ticks (motors=%u sensors=%u)\n",
      mean_us[at(TickPhase::kRefresh)], max_us[at(TickPhase::kRefresh)],
      mean_us[at(TickPhase::kDevices)], max_us[at(TickPhase::kDevices)],
      mean_us[at(TickPhase::kControl)], max_us[at(TickPhase::kControl)],
      mean_us[at(TickPhase::kHealth)], max_us[at(TickPhase::kHealth)],
      mean_us[at(TickPhase::kOther)], max_us[at(TickPhase::kOther)],
      mean_us[at(TickPhase::kLink)], max_us[at(TickPhase::kLink)],
      mean_us[at(TickPhase::kDriverStation)],
      max_us[at(TickPhase::kDriverStation)], mean_us[at(TickPhase::kLoop)],
      max_us[at(TickPhase::kLoop)],
      static_cast<unsigned long long>(window.ticks),
      static_cast<unsigned>(window.motors),
      static_cast<unsigned>(window.sensors));
  sink_(text);
  sink_("[rio] latency: command age accepted p50/p99/max " +
        latency.accepted.Summary() + ", refused " + latency.refused.Summary() +
        "; driver station refresh to send " + latency.driver_station.Summary() +
        "\n");
}

void RioLog::WritePhases(uint64_t now_us, bool force) {
  if (phase_awaiting_ &&
      phase_served_.load(std::memory_order_acquire) == phase_requested_) {
    phase_awaiting_ = false;
    const uint64_t ticks = published_ticks_.load(std::memory_order_relaxed);
    if (ticks > 0) {
      PhaseWindow window{};
      window.ticks = ticks;
      for (std::size_t i = 0; i < kPhaseCount; ++i) {
        window.sum_ns[i] = published_sum_ns_[i].load(std::memory_order_relaxed);
        window.max_ns[i] = published_max_ns_[i].load(std::memory_order_relaxed);
      }
      window.motors = published_motors_.load(std::memory_order_relaxed);
      window.sensors = published_sensors_.load(std::memory_order_relaxed);
      EmitPhasesAndLatency(window, latency_[latency_published_]);
    }
    // Empty for the loop's next switch to it, before the next request's
    // release store.
    latency_[latency_published_] = {};
  }
  if (force) {
    // On Stop(), the loop thread's release store to stop_ happens-before
    // Run()'s acquire load and this Flush(), so draining the open partial
    // window here is race-free; a flush asks for nothing further.
    if (phase_window_.ticks > 0) {
      EmitPhasesAndLatency(phase_window_, latency_[latency_front_]);
      phase_window_ = {};
      latency_[latency_front_] = {};
    }
    return;
  }
  if (!phase_clock_started_) {
    phase_clock_started_ = true;
    next_phase_us_ = now_us + kPhaseWindowUs;
    return;
  }
  if (phase_awaiting_ || now_us < next_phase_us_) return;
  // The window closes at the loop's next tick, which the next drain prints.
  ++phase_requested_;
  phase_request_.store(phase_requested_, std::memory_order_release);
  phase_awaiting_ = true;
  next_phase_us_ = now_us + kPhaseWindowUs;
}

void RioLog::Start() {
  if (thread_.joinable()) return;
  stop_.store(false, std::memory_order_relaxed);
  thread_ = std::thread([this] { Run(); });
}

void RioLog::Stop() {
  if (!thread_.joinable()) return;
  stop_.store(true, std::memory_order_release);
  thread_.join();
}

void RioLog::Run() {
#ifdef __linux__
  // Below every other thread of the program, the loop thread above all.
  pthread_setname_np(pthread_self(), "talos-log");
  setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 19);
#endif
  while (!stop_.load(std::memory_order_acquire)) {
    Drain(SteadyNowUs());
    std::this_thread::sleep_for(std::chrono::milliseconds{20});
  }
  Flush(SteadyNowUs());
}

void RioLog::Drain(uint64_t now_us) {
  LogEvent event;
  while (Pop(event)) Handle(event, now_us);
  WriteDue(now_us, false);
}

void RioLog::Flush(uint64_t now_us) {
  LogEvent event;
  while (Pop(event)) Handle(event, now_us);
  WriteDue(now_us, true);
}

void RioLog::Handle(const LogEvent& event, uint64_t now_us) {
  const auto kind = static_cast<std::size_t>(event.kind);
  if (kind >= windows_->size()) return;
  Window& window =
      (*windows_)[kind][std::min<std::size_t>(event.index, kKeysPerKind - 1)];
  const uint32_t toggled =
      event.kind == LogKind::kMotorFault ? event.value ^ event.previous : 0;
  const bool expired = now_us - window.start_us >= kWindowUs;
  if (window.open && (!expired || window.suppressed > 0)) {
    ++window.suppressed;
    window.toggled |= toggled;
    window.last = event;
    // An expired window still holding changes ends with this one: its
    // coalesced line, never a bare change line that would lose the count.
    if (expired) WriteCoalesced(window, now_us);
    return;
  }
  sink_(Format(event));
  window.open = true;
  window.start_us = now_us;
  window.suppressed = 0;
  window.toggled = 0;
}

void RioLog::WriteCoalesced(Window& window, uint64_t now_us) {
  std::string line = Format(window.last);
  line.pop_back();  // the newline
  line += " [" + std::to_string(window.suppressed) +
          " changes coalesced since the previous line";
  if (window.toggled != 0)
    line += "; bits toggled: " + DecodeTalonFaultBits(window.toggled);
  line += "]\n";
  sink_(line);
  // Still changing: keep the window, so the next line is a second away.
  window.start_us = now_us;
  window.suppressed = 0;
  window.toggled = 0;
}

void RioLog::WriteDue(uint64_t now_us, bool force) {
  for (auto& kind : *windows_) {
    for (auto& window : kind) {
      if (!window.open) continue;
      if (!force && now_us - window.start_us < kWindowUs) continue;
      if (window.suppressed == 0) {
        window.open = false;
        continue;
      }
      WriteCoalesced(window, now_us);
    }
  }

  WritePhases(now_us, force);
  if (!force && now_us < next_summary_us_) return;
  next_summary_us_ = now_us + kWindowUs;
  const uint64_t dropped = dropped_.load(std::memory_order_relaxed);
  if (dropped != reported_dropped_) {
    sink_(
        "[rio] log queue full: " + std::to_string(dropped - reported_dropped_) +
        " events dropped (total " + std::to_string(dropped) + ")\n");
    reported_dropped_ = dropped;
  }
  // The maxima, counts and worst-tick phase split cover this window only.
  // Read the current slot first: if the loop thread has already published into
  // `timing_current_epoch_`, its single-writer happens-before guarantees the
  // previous slot (`timing_current_epoch_ - 1`) is quiescent and can be drained
  // of any stragglers before `timing_epoch_` advances and recycles that slot.
  const uint64_t current_epoch = timing_current_epoch_;
  const TimingSnapshot curr_check = ReadTimingSlot(current_epoch);
  const bool curr_active =
      curr_check.valid && curr_check.epoch == current_epoch;

  TimingSnapshot prev{};
  bool prev_present = false;
  if (current_epoch > 1) {
    prev = ReadTimingSlot(current_epoch - 1);
    if (!prev.valid) {
      // Loop thread is mid-seqlock in the previous slot (only possible when
      // !curr_active): retry on the next drain without advancing the epoch.
      if (!force) next_summary_us_ = now_us;
      return;
    }
    prev_present = prev.epoch == current_epoch - 1;
  }

  const uint64_t straggler_overruns =
      prev_present && prev.overruns > prev_reported_overruns_
          ? prev.overruns - prev_reported_overruns_
          : 0;
  const uint64_t straggler_late =
      prev_present && prev.late_ticks > prev_reported_late_ticks_
          ? prev.late_ticks - prev_reported_late_ticks_
          : 0;

  TimingSnapshot curr{};
  bool curr_present = false;
  if (curr_active) {
    // Previous slot is now drained and quiescent; advance timing_epoch_ so
    // subsequent ticks write to the other slot while we take the final read of
    // `current_epoch`.
    timing_epoch_.store(current_epoch + 1, std::memory_order_release);
    curr = ReadTimingSlot(current_epoch);
    if (!curr.valid || curr.epoch != current_epoch) {
      // A straggler was mid-seqlock for all 64 yields; report `curr_check` now
      // and catch the straggler's delta from `prev` on the next window.
      curr = curr_check;
    }
    curr_present = true;
    prev_reported_overruns_ = curr.overruns;
    prev_reported_late_ticks_ = curr.late_ticks;
    timing_current_epoch_ = current_epoch + 1;
  } else {
    // No tick has finished writing to `current_epoch` yet; keep the epoch open
    // so `current_epoch - 1` stays reserved for any in-flight straggler.
    if (prev_present) {
      prev_reported_overruns_ += straggler_overruns;
      prev_reported_late_ticks_ += straggler_late;
    }
    if (!curr_check.valid && !force) {
      next_summary_us_ = now_us;
    }
  }

  const uint64_t curr_overruns = curr_present ? curr.overruns : 0;
  const uint64_t curr_late = curr_present ? curr.late_ticks : 0;
  const uint64_t delta_overruns = straggler_overruns + curr_overruns;
  const uint64_t delta_late = straggler_late + curr_late;
  if (delta_overruns > 0 || delta_late > 0) {
    reported_overruns_ += delta_overruns;
    reported_late_ticks_ += delta_late;
    const bool use_prev = straggler_overruns > 0 || straggler_late > 0;
    const bool use_curr = curr_overruns > 0 || curr_late > 0;
    // Pick the worst work (and its own phase split) from the window(s) that
    // actually contributed a fault, never pairing a fault with a clean window.
    const bool pick_prev_work =
        use_prev && (!use_curr || prev.max_tick_us > curr.max_tick_us);
    const uint64_t reported_max_tick =
        pick_prev_work ? prev.max_tick_us : curr.max_tick_us;
    const std::array<uint64_t, kPhaseCount> worst_ns =
        pick_prev_work ? prev.worst_phase_ns : curr.worst_phase_ns;
    const uint64_t reported_max_interval =
        std::max(use_prev ? prev.max_interval_us : uint64_t{0},
                 use_curr ? curr.max_interval_us : uint64_t{0});
    const uint64_t reported_period = use_curr ? curr.period_us : prev.period_us;
    const auto at = [](TickPhase phase) {
      return static_cast<std::size_t>(phase);
    };
    char phases[320];
    std::snprintf(
        phases, sizeof(phases),
        "; worst tick phases: refresh %.1f us, devices %.1f us, control %.1f "
        "us, health %.1f us, other %.1f us, link %.1f us, driver station %.1f "
        "us, loop %.1f us\n",
        static_cast<double>(worst_ns[at(TickPhase::kRefresh)]) / 1e3,
        static_cast<double>(worst_ns[at(TickPhase::kDevices)]) / 1e3,
        static_cast<double>(worst_ns[at(TickPhase::kControl)]) / 1e3,
        static_cast<double>(worst_ns[at(TickPhase::kHealth)]) / 1e3,
        static_cast<double>(worst_ns[at(TickPhase::kOther)]) / 1e3,
        static_cast<double>(worst_ns[at(TickPhase::kLink)]) / 1e3,
        static_cast<double>(worst_ns[at(TickPhase::kDriverStation)]) / 1e3,
        static_cast<double>(worst_ns[at(TickPhase::kLoop)]) / 1e3);
    sink_("[rio] loop timing: " + std::to_string(delta_overruns) +
          " overruns (work longer than the period), " +
          std::to_string(delta_late) +
          " late ticks (started over 1.5 periods after the previous) since "
          "the last report; totals " +
          std::to_string(reported_overruns_) + " and " +
          std::to_string(reported_late_ticks_) + "; max work " +
          std::to_string(reported_max_tick) + " us, max interval " +
          std::to_string(reported_max_interval) + " us, period " +
          std::to_string(reported_period) + " us" + phases);
  }
}

std::string RioLog::DecodeTalonFaultBits(uint32_t fault_flags) {
  if (fault_flags == 0) return "none";
  // Phoenix 6 TalonFX FaultField, SPN 0x34c / CAN frame 0xad.
  static constexpr std::array<const char*, 32> names{
      "Hardware (hardware fault occurred)",
      "ProcTemp (processor temperature exceeded limit)",
      "DeviceTemp (device temperature exceeded limit)",
      "Undervoltage (supply voltage near brownout)",
      "BootDuringEnable (booted while enabled)",
      "UnlicensedFeatureInUse (unlicensed feature in use)",
      "StaticBrakeDisabled (static brake momentarily disabled)",
      "UsingFusedCANcoderWhileUnlicensed (fused CANcoder requires Pro)",
      "SupplyCurrLimit (supply current limit active)",
      "StatorCurrLimit (stator current limit active)",
      "FusedSensorOutOfSync (remote fused sensor out of sync)",
      "RemoteSensorDataInvalid (remote sensor data invalid)",
      "MissingHardLimitRemote (remote hard limit device missing)",
      "MissingSoftLimitRemote (remote soft limit device missing)",
      "ForwardSoftLimit (forward soft limit asserted)",
      "ReverseSoftLimit (reverse soft limit asserted)",
      "ForwardHardLimit (forward hard limit asserted)",
      "ReverseHardLimit (reverse hard limit asserted)",
      "OverSupplyV (supply voltage exceeded rating)",
      "UnstableSupplyV (supply voltage unstable)",
      "RemoteSensorPosOverflow (remote sensor position overflowed)",
      "MissingDifferentialFX (differential TalonFX missing)",
      "RemoteSensorReset (remote sensor reset)",
      "BridgeBrownout (bridge disabled due to supply voltage)",
      nullptr,
      "RotorFault1 (rotor fault 1)",
      "RotorFault2 (rotor fault 2)",
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      nullptr};
  std::string faults;
  for (unsigned bit = 0; bit < names.size(); ++bit) {
    if ((fault_flags & (uint32_t{1} << bit)) == 0) continue;
    if (!faults.empty()) faults += ", ";
    if (names[bit]) {
      faults += names[bit];
    } else {
      faults += "UnknownBit" + std::to_string(bit);
    }
  }
  return faults;
}

std::string RioLog::Format(const LogEvent& e) {
  std::ostringstream out;
  switch (e.kind) {
    case LogKind::kMotorFault: {
      // A limit or current limiter acting as configured is reported as a
      // limit, never as a fault (hardware::kMotorNotFaultFlags).
      constexpr uint32_t kLimits = hardware::kMotorNotFaultFlags;
      const uint32_t raised = e.value & ~e.previous;
      const uint32_t cleared = e.previous & ~e.value;
      out << "[phoenix] motor " << e.name << " (mask_bit=" << e.index
          << ", id=" << e.id << ", bus=" << e.bus << ", can_id=" << e.can_id
          << ") flags=0x" << std::hex << e.value << " (was 0x" << e.previous
          << ")" << std::dec;
      const auto changed = [&out](const char* label, uint32_t bits) {
        if (bits != 0)
          out << " " << label << "=[" << DecodeTalonFaultBits(bits) << "]";
      };
      changed("fault raised", raised & ~kLimits);
      changed("fault cleared", cleared & ~kLimits);
      changed("limit raised", raised & kLimits);
      changed("limit cleared", cleared & kLimits);
      out << " faults=[" << DecodeTalonFaultBits(e.value & ~kLimits) << "]"
          << " limits=[" << DecodeTalonFaultBits(e.value & kLimits) << "]"
          << " pos=" << e.a << " rot vel=" << e.b << " rps V=" << e.c
          << "V I=" << e.d << "A (limits: supply=" << e.limit1
          << "A stator=" << e.limit2 << "A)";
      break;
    }
    case LogKind::kMotorFeedback:
    case LogKind::kSensorFeedback:
      out << "[phoenix] "
          << (e.kind == LogKind::kMotorFeedback ? "motor" : "sensor") << " "
          << e.name << " (id=" << e.id << ", bus=" << e.bus
          << ", can_id=" << e.can_id << ") feedback "
          << (e.flag ? "RECOVERED" : "INVALID")
          << " (position_status=" << Or(e.status, "?")
          << ", velocity_status=" << Or(e.status2, "?")
          << ", position_age_us=" << e.age_us
          << ", velocity_age_us=" << e.age2_us << ")";
      break;
    case LogKind::kMotorFeedbackLate:
    case LogKind::kSensorFeedbackLate:
      out << "[phoenix] "
          << (e.kind == LogKind::kMotorFeedbackLate ? "motor" : "sensor") << " "
          << e.name << " (id=" << e.id << ", bus=" << e.bus
          << ", can_id=" << e.can_id << ") feedback ";
      if (e.flag) {
        out << "LATE (position_age_us=" << e.age_us
            << ", velocity_age_us=" << e.age2_us << ", late from " << e.previous
            << " us, four status periods";
      } else {
        out << "ON TIME (peak_age_us=" << e.value;
      }
      out << ", position_status=" << Or(e.status, "?")
          << ", velocity_status=" << Or(e.status2, "?") << ")";
      break;
    case LogKind::kMotorFaultMask:
      out << "[phoenix] motor_fault_mask changed: 0x" << std::hex << e.previous
          << " -> 0x" << e.value << std::dec << " (faulted motor slots:";
      if (e.value == 0) out << " none";
      for (unsigned bit = 0; bit < 32; ++bit)
        if (e.value & (uint32_t{1} << bit)) out << " " << bit;
      out << ")";
      break;
    case LogKind::kBrownout:
      out << "[phoenix] power brownout " << (e.flag ? "ACTIVE" : "CLEARED")
          << " (battery_v=" << e.a << "V)";
      break;
    case LogKind::kApplyControl:
      out << "[phoenix] apply control " << (e.flag ? "failed" : "recovered")
          << " on motor " << e.name << " (can_id=" << e.can_id
          << "): " << Or(e.status, "?")
          << (e.flag ? "; neutralizing only this motor" : "");
      break;
    case LogKind::kNeutralSend:
      out << "[phoenix] neutral send " << (e.flag ? "failing" : "recovered")
          << " on motor " << e.name << " (can_id=" << e.can_id
          << "): " << Or(e.status, "?");
      break;
    case LogKind::kDeviceReset:
      out << "[phoenix] device reset detected: "
          << (e.flag ? "sensor " : "motor ") << e.name << " (id=" << e.id
          << ", bus=" << e.bus << ", can_id=" << e.can_id << ")";
      break;
    case LogKind::kMessage:
    case LogKind::kCount:
      out << e.text;
      break;
  }
  out << "\n";
  return out.str();
}

}  // namespace talos::rio

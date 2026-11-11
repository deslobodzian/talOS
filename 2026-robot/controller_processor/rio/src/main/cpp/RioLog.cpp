#include "RioLog.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <sstream>

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

}  // namespace

std::string ConsoleTime(uint64_t steady_us) {
  char text[32];
  std::snprintf(text, sizeof(text), "[%llu.%03llu] ",
                static_cast<unsigned long long>(steady_us / 1'000'000),
                static_cast<unsigned long long>(steady_us / 1'000 % 1'000));
  return text;
}

void LogEvent::SetName(std::string_view value) { CopyBounded(name, value); }
void LogEvent::SetBus(std::string_view value) { CopyBounded(bus, value); }
void LogEvent::SetText(std::string_view value) { CopyBounded(text, value); }

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
      // One write, so another thread's console line cannot split it.
      std::string timed = ConsoleTime(SteadyNowUs());
      timed.append(line);
      std::fwrite(timed.data(), 1, timed.size(), stderr);
      std::fflush(stderr);
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
  period_us_.store(period_us, std::memory_order_relaxed);
  if (duration_us > period_us)
    overruns_.fetch_add(1, std::memory_order_relaxed);
  if (interval_us > period_us + period_us / 2)
    late_ticks_.fetch_add(1, std::memory_order_relaxed);
  // The logger thread resets both maxima each summary window.
  const auto raise = [](std::atomic<uint64_t>& max, uint64_t value) {
    uint64_t current = max.load(std::memory_order_relaxed);
    while (value > current && !max.compare_exchange_weak(
                                  current, value, std::memory_order_relaxed)) {
    }
  };
  raise(max_tick_us_, duration_us);
  raise(max_interval_us_, interval_us);

  // Hand the logger the window it asked for, without this tick, which opens
  // the next one. The fence runs only on a new request, once per window.
  const uint32_t request = phase_request_.load(std::memory_order_relaxed);
  if (request != phase_served_local_) {
    std::atomic_thread_fence(std::memory_order_acquire);
    published_ticks_.store(phase_window_.ticks, std::memory_order_relaxed);
    for (std::size_t i = 0; i < kPhaseCount; ++i) {
      published_sum_ns_[i].store(phase_window_.sum_ns[i],
                                 std::memory_order_relaxed);
      published_max_ns_[i].store(phase_window_.max_ns[i],
                                 std::memory_order_relaxed);
    }
    published_motors_.store(phase_window_.motors, std::memory_order_relaxed);
    published_sensors_.store(phase_window_.sensors, std::memory_order_relaxed);
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
  phase_window_.motors = tick_motors_;
  phase_window_.sensors = tick_sensors_;
}

void RioLog::AddPhase(TickPhase phase, uint64_t duration_ns) {
  const auto i = static_cast<std::size_t>(phase);
  if (i < kPhaseCount) tick_phase_ns_[i] += duration_ns;
}

void RioLog::SetTickDevices(std::size_t motors, std::size_t sensors) {
  tick_motors_ = static_cast<uint32_t>(motors);
  tick_sensors_ = static_cast<uint32_t>(sensors);
}

void RioLog::WritePhases(uint64_t now_us, bool force) {
  if (phase_awaiting_ &&
      phase_served_.load(std::memory_order_acquire) == phase_requested_) {
    phase_awaiting_ = false;
    const uint64_t ticks = published_ticks_.load(std::memory_order_relaxed);
    if (ticks > 0) {
      std::array<double, kPhaseCount> mean_us{};
      std::array<double, kPhaseCount> max_us{};
      for (std::size_t i = 0; i < kPhaseCount; ++i) {
        mean_us[i] = static_cast<double>(
                         published_sum_ns_[i].load(std::memory_order_relaxed)) /
                     static_cast<double>(ticks) / 1e3;
        max_us[i] = static_cast<double>(
                        published_max_ns_[i].load(std::memory_order_relaxed)) /
                    1e3;
      }
      const auto at = [](TickPhase phase) {
        return static_cast<std::size_t>(phase);
      };
      char text[256];
      std::snprintf(
          text, sizeof(text),
          "[rio] tick phases: refresh mean/max %.1f/%.1f us, devices "
          "%.1f/%.1f us, control %.1f/%.1f us, health %.1f/%.1f us, other "
          "%.1f/%.1f us over %llu ticks (motors=%u sensors=%u)\n",
          mean_us[at(TickPhase::kRefresh)], max_us[at(TickPhase::kRefresh)],
          mean_us[at(TickPhase::kDevices)], max_us[at(TickPhase::kDevices)],
          mean_us[at(TickPhase::kControl)], max_us[at(TickPhase::kControl)],
          mean_us[at(TickPhase::kHealth)], max_us[at(TickPhase::kHealth)],
          mean_us[at(TickPhase::kOther)], max_us[at(TickPhase::kOther)],
          static_cast<unsigned long long>(ticks),
          static_cast<unsigned>(
              published_motors_.load(std::memory_order_relaxed)),
          static_cast<unsigned>(
              published_sensors_.load(std::memory_order_relaxed)));
      sink_(text);
    }
  }
  // A flush asks for nothing: the stopped loop would never answer.
  if (force) return;
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
  stop_.store(true, std::memory_order_relaxed);
  thread_.join();
}

void RioLog::Run() {
#ifdef __linux__
  // Below every other thread of the program, the loop thread above all.
  pthread_setname_np(pthread_self(), "talos-log");
  setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 19);
#endif
  while (!stop_.load(std::memory_order_relaxed)) {
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
  // The maxima cover this window only.
  const uint64_t max_tick = max_tick_us_.exchange(0, std::memory_order_relaxed);
  const uint64_t max_interval =
      max_interval_us_.exchange(0, std::memory_order_relaxed);
  const uint64_t overruns = overruns_.load(std::memory_order_relaxed);
  const uint64_t late = late_ticks_.load(std::memory_order_relaxed);
  if (overruns != reported_overruns_ || late != reported_late_ticks_) {
    sink_(
        "[rio] loop timing: " + std::to_string(overruns - reported_overruns_) +
        " overruns (work longer than the period), " +
        std::to_string(late - reported_late_ticks_) +
        " late ticks (started over 1.5 periods after the previous) since "
        "the last report; totals " +
        std::to_string(overruns) + " and " + std::to_string(late) +
        "; max work " + std::to_string(max_tick) + " us, max interval " +
        std::to_string(max_interval) + " us, period " +
        std::to_string(period_us_.load(std::memory_order_relaxed)) + " us\n");
    reported_overruns_ = overruns;
    reported_late_ticks_ = late;
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
      const uint32_t raised = e.value & ~e.previous;
      const uint32_t cleared = e.previous & ~e.value;
      out << "[phoenix] motor " << e.name << " (mask_bit=" << e.index
          << ", id=" << e.id << ", bus=" << e.bus << ", can_id=" << e.can_id
          << ") fault_flags=0x" << std::hex << e.value << " (was 0x"
          << e.previous << ")" << std::dec;
      if (raised != 0)
        out << " raised=[" << DecodeTalonFaultBits(raised) << "]";
      if (cleared != 0)
        out << " cleared=[" << DecodeTalonFaultBits(cleared) << "]";
      out << " active=[" << DecodeTalonFaultBits(e.value) << "]"
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
          << (e.flag ? "RECOVERED" : "INVALID/STALE")
          << " (position_status=" << Or(e.status, "?")
          << ", velocity_status=" << Or(e.status2, "?")
          << ", position_age_us=" << e.age_us
          << ", velocity_age_us=" << e.age2_us << ")";
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

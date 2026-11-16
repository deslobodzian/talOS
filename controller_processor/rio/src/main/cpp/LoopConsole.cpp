#include "LoopConsole.h"

#include <cstdio>
#include <cstring>

namespace talos::rio {
namespace {

// Milliseconds from `from_us` to `to_us`, negative when `to_us` is earlier
// (a clock behind the command, which Tick also expires).
double DelayMs(uint64_t from_us, uint64_t to_us) {
  return static_cast<double>(static_cast<int64_t>(to_us - from_us)) / 1000.0;
}

}  // namespace

void LoopConsole::ObserveGateway(const hardware::Gateway& gateway) {
  ReportExpiry(gateway.last_command_expiry());
  ReportRefusal(gateway.last_enabled_refusal());
}

void LoopConsole::ObserveWatchdog(const LoopWatchdog& watchdog) {
  const uint64_t dropped = watchdog.dropped_reports();
  if (dropped == reported_dropped_reports_) return;
  const uint64_t new_drops = dropped - reported_dropped_reports_;
  reported_dropped_reports_ = dropped;
  // Read after the drops, so it normally covers every stall whose report
  // was dropped (two relaxed loads; the loop was stalled while they rose).
  const uint64_t stalls = watchdog.stall_count();
  char text[160];
  std::snprintf(text, sizeof(text),
                "RoboRIO loop watchdog: %llu stall report%s dropped (%llu "
                "since start), %llu stalls in all: the Driver Station error "
                "channel was not taking them",
                static_cast<unsigned long long>(new_drops),
                new_drops == 1 ? "" : "s",
                static_cast<unsigned long long>(dropped),
                static_cast<unsigned long long>(stalls));
  LogEvent event;
  event.index = kWatchdogReportsDroppedMessage;
  event.SetText(text);
  log_.Post(event);
}

void LoopConsole::ReportRefusal(
    const hardware::Gateway::EnabledRefusal& refusal) {
  if (refusal.count == reported_refusals_) return;
  reported_refusals_ = refusal.count;
  char text[160];
  std::snprintf(text, sizeof(text),
                "hardware gateway refused configuration %llu while enabled "
                "(running %llu): disable to accept the new configuration",
                static_cast<unsigned long long>(refusal.refused_id),
                static_cast<unsigned long long>(refusal.active_id));
  LogEvent event;
  event.index = kConfigRefusedMessage;
  event.SetText(text);
  log_.Post(event);
}

void LoopConsole::ReportExpiry(const hardware::Gateway::CommandExpiry& expiry) {
  if (expiry.count == reported_expiries_) return;
  reported_expiries_ = expiry.count;
  // The neutral's own time, not the line's: RioLog writes it up to a drain
  // later, and coalesces a second expiry within a second into one line.
  char time[kConsoleTimeSize];
  FormatConsoleTime(expiry.neutral_us, time);
  const auto time_size = static_cast<int>(std::strlen(time)) - 1;  // the space
  char text[160];
  std::snprintf(text, sizeof(text),
                "hardware gateway command expired: outputs neutral at %.*s, "
                "%.1f ms after its observation, %.1f ms after its acceptance "
                "(expiry %llu)",
                time_size, time, DelayMs(expiry.observed_us, expiry.neutral_us),
                DelayMs(expiry.accepted_us, expiry.neutral_us),
                static_cast<unsigned long long>(expiry.count));
  LogEvent event;
  event.index = kCommandExpiryMessage;
  event.SetText(text);
  log_.Post(event);
}

}  // namespace talos::rio

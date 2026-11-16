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
  const auto& expiry = gateway.last_command_expiry();
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

#pragma once

#include <cstdint>

// When the RoboRIO loop (Robot.cpp) wakes for its next tick. Each tick moves
// the loop's deadline on by one period; the loop then sleeps to the time this
// returns, on the clock its ticks are timed with.
namespace talos::rio {

// How long the loop still sleeps after a tick that ended past its deadline.
// The loop runs at SCHED_FIFO on the RoboRIO, where a loop that never slept
// would starve every default-scheduled thread on its core (the logger, the
// companion reporter, ssh, and vendor threads of unknown policy); this bounds
// the share of the core a loop that overruns every tick can take.
inline constexpr uint64_t kOverrunSleepUs = 500;

// The time to sleep until after a tick whose work ended at `end_us`, with
// `deadline_us` its next deadline (the previous one plus the period).
//
// A tick that ends by its deadline sleeps to it, so the loop keeps its phase
// however long the work or the wake took. One that ends past it (an overrun,
// or a wake more than a period late) sleeps kOverrunSleepUs: the missed
// deadlines are dropped, not caught up in a burst of back-to-back ticks that
// would starve the Driver Station refresh, and the next period counts from
// that wake, so the loop's phase moves by the overrun.
constexpr uint64_t NextWakeUs(uint64_t deadline_us, uint64_t end_us) {
  return end_us <= deadline_us ? deadline_us : end_us + kOverrunSleepUs;
}

}  // namespace talos::rio

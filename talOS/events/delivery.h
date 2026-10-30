#pragma once

#include <cstdint>

namespace talos::event {

// When a loop hands queued messages to its watchers.
enum class Delivery : std::uint8_t {
  // On the internal poll timer, every tick_period. A watcher's handler runs
  // within one tick of the message arriving, whatever else is scheduled.
  POLL,
  // Immediately before each timer dispatch, and never otherwise: no poll
  // timer is armed, so a loop with only periodic work wakes only for its
  // timers. A message waits for the next timer, which is when a program that
  // reads latched values at its tick would have looked at it anyway.
  BEFORE_TIMERS,
};

}  // namespace talos::event

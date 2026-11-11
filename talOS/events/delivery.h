#pragma once

#include <cstdint>

namespace talos::event {

// When a loop hands queued messages to its watchers. Every policy drains
// immediately before each timer dispatch, so a timer handler sees everything
// published by its deadline.
enum class Delivery : std::uint8_t {
  // Also on the internal poll timer, every tick_period. A watcher's handler
  // runs within one tick of the message arriving, whatever else is scheduled,
  // at the cost of a wakeup every tick.
  POLL,
  // Never otherwise: no poll timer is armed, so a loop with only periodic
  // work wakes only for its timers. A message waits for the next timer,
  // which is when a program that reads latched values at its tick would have
  // looked at it anyway.
  BEFORE_TIMERS,
  // Also whenever a publication on a prompt watcher (set_prompt) wakes the
  // loop; nothing wakes it otherwise, so an idle loop sleeps until its next
  // timer. The realtime loop is woken by the publisher (talOS/rtms/wake.h);
  // the simulated loop treats the publication as an event at its
  // publication time. With no prompt watcher either runs as BEFORE_TIMERS.
  ON_PUBLISH,
};

}  // namespace talos::event

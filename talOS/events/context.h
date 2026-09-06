#pragma once

#include <cstdint>

#include "talOS/events/time.h"

namespace talos::event {

// What produced a record. Values are written to the log, so they are fixed
// forever; append new kinds, never renumber.
enum class EventKind : std::uint16_t {
  TIMER = 1,
  MESSAGE = 2,
  FETCH = 3,
  SEND = 4,
  EXIT = 5,
  ARM_TIMER = 6,
  DISARM_TIMER = 7,
  HANDLER_EXIT = 8,
};

constexpr const char* to_string(EventKind kind) {
  switch (kind) {
    case EventKind::TIMER:
      return "TIMER";
    case EventKind::MESSAGE:
      return "MESSAGE";
    case EventKind::FETCH:
      return "FETCH";
    case EventKind::SEND:
      return "SEND";
    case EventKind::EXIT:
      return "EXIT";
    case EventKind::ARM_TIMER:
      return "ARM_TIMER";
    case EventKind::DISARM_TIMER:
      return "DISARM_TIMER";
    case EventKind::HANDLER_EXIT:
      return "HANDLER_EXIT";
  }
  return "UNKNOWN";
}

// Everything a handler is allowed to know about why it is running.
//
// `event_time` is when the event should have happened (a timer's deadline, or
// the time a message became available). `now` is the loop's observation time,
// frozen for the whole dispatch. In realtime `now >= event_time` and the
// difference is the scheduling latency; in replay both come from the log, so
// handler logic that keys off either is reproduced exactly.
struct Context {
  EventKind kind{EventKind::TIMER};
  std::uint16_t source_id{0};
  std::uint64_t dispatch_index{0};
  MonotonicTime event_time{};
  MonotonicTime now{};

  // Timer: number of periods that elapsed (1 when on time, >1 after an
  // overrun). Message: the RTMS sequence.
  std::uint64_t sequence{0};

  // Messages the writer lapped past before this one. Non-zero means data was
  // lost; a control loop should treat that as a fault, not a hiccup.
  std::uint64_t dropped{0};

  Duration latency() const { return now - event_time; }
};

}  // namespace talos::event

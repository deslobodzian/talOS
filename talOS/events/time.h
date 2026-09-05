#pragma once

#include <chrono>
#include <compare>
#include <cstdint>
#include <limits>

namespace talos::event {

// Every duration in the event system is nanoseconds. Keeping one unit removes
// a whole class of conversion bugs from the replay path, where a rounded
// timestamp would change dispatch order.
using Duration = std::chrono::nanoseconds;

// A point on the loop's monotonic timeline.
//
// This is deliberately not std::chrono::steady_clock::time_point: a replayed
// loop produces times that were recorded on another machine, possibly by
// another process, and must never be confused with a live clock reading. The
// only way to obtain one during a dispatch is EventLoop::monotonic_now().
class MonotonicTime {
 public:
  constexpr MonotonicTime() = default;

  static constexpr MonotonicTime from_nanos(std::int64_t nanos) {
    return MonotonicTime{nanos};
  }

  static constexpr MonotonicTime min() {
    return MonotonicTime{std::numeric_limits<std::int64_t>::min()};
  }

  static constexpr MonotonicTime max() {
    return MonotonicTime{std::numeric_limits<std::int64_t>::max()};
  }

  constexpr std::int64_t nanos() const { return nanos_; }

  constexpr std::chrono::duration<double> seconds() const {
    return std::chrono::duration<double>{Duration{nanos_}};
  }

  constexpr MonotonicTime operator+(Duration duration) const {
    return MonotonicTime{nanos_ + duration.count()};
  }

  constexpr MonotonicTime operator-(Duration duration) const {
    return MonotonicTime{nanos_ - duration.count()};
  }

  constexpr MonotonicTime& operator+=(Duration duration) {
    nanos_ += duration.count();
    return *this;
  }

  constexpr Duration operator-(MonotonicTime other) const {
    return Duration{nanos_ - other.nanos_};
  }

  constexpr auto operator<=>(const MonotonicTime&) const = default;
  constexpr bool operator==(const MonotonicTime&) const = default;

 private:
  constexpr explicit MonotonicTime(std::int64_t nanos) : nanos_{nanos} {}

  std::int64_t nanos_{0};
};

// A raw steady-clock reading, for measuring how long something took.
//
// This is deliberately separate from the loop clock. A replayed loop's
// monotonic_now() returns recorded time, so subtracting two of them would
// always give zero; measuring real elapsed work needs a real clock. Use it
// only for differences, never as an event timestamp.
inline MonotonicTime steady_now() {
  return MonotonicTime::from_nanos(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

}  // namespace talos::event

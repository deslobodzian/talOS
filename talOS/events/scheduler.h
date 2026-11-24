#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

#include "talOS/events/time.h"

namespace talos::event {

// A timer that has come due.
struct Expiration {
  std::uint16_t id{0};
  MonotonicTime deadline{};  // when it was supposed to fire
  std::uint64_t cycles{0};   // periods elapsed; >1 means the loop overran
};

// Deterministic timer queue.
//
// A flat binary heap ordered by (deadline, id). The id tiebreak is the whole
// point: when two timers come due in the same iteration the order they fire in
// must not depend on heap internals or insertion history, or a replayed run
// could dispatch them the other way round.
//
// No OS calls and no allocation once reserve() has been called, so it is
// shared unchanged by the realtime, simulated and replay loops.
class Scheduler {
 public:
  void reserve(std::size_t timers) { heap_.reserve(timers); }

  // Arms a one-shot timer. Re-arming a timer that is already queued replaces
  // its deadline.
  void schedule(std::uint16_t id, MonotonicTime deadline) {
    schedule(id, deadline, Duration::zero());
  }

  // Arms a repeating timer. A zero period is a one-shot.
  void schedule(std::uint16_t id, MonotonicTime deadline, Duration period) {
    disable(id);
    heap_.push_back(Entry{deadline, id, period});
    std::push_heap(heap_.begin(), heap_.end(), Later{});
  }

  void disable(std::uint16_t id) {
    const auto it = std::find_if(heap_.begin(), heap_.end(),
                                 [id](const Entry& e) { return e.id == id; });
    if (it == heap_.end()) {
      return;
    }
    heap_.erase(it);
    std::make_heap(heap_.begin(), heap_.end(), Later{});
  }

  // Moves every armed deadline by `delta`, saturating at MonotonicTime bounds.
  // Saturation can collapse distinct deadlines into a tie, so rebuild the heap
  // when that happens to keep (deadline, id) order intact.
  void shift(Duration delta) {
    const std::int64_t d = delta.count();
    if (d == 0) return;
    bool saturated = false;
    for (Entry& entry : heap_) {
      const std::int64_t nanos = entry.deadline.nanos();
      if (d > 0 && nanos > MonotonicTime::max().nanos() - d) {
        entry.deadline = MonotonicTime::max();
        saturated = true;
      } else if (d < 0 && nanos < MonotonicTime::min().nanos() - d) {
        entry.deadline = MonotonicTime::min();
        saturated = true;
      } else {
        entry.deadline += delta;
      }
    }
    if (saturated) {
      std::make_heap(heap_.begin(), heap_.end(), Later{});
    }
  }

  bool empty() const { return heap_.empty(); }

  // Earliest deadline, or MonotonicTime::max() when nothing is armed.
  MonotonicTime next_deadline() const {
    return heap_.empty() ? MonotonicTime::max() : heap_.front().deadline;
  }

  // Pops the earliest timer if it is due at `now`.
  //
  // A periodic timer that missed deadlines is not replayed once per missed
  // period: it fires once, reporting how many periods elapsed, and its next
  // deadline stays on the original phase grid. This keeps a loop that overran
  // from spiralling, and keeps deadlines a pure function of the start time.
  bool pop_expired(MonotonicTime now, Expiration& expiration) {
    if (heap_.empty() || heap_.front().deadline > now) {
      return false;
    }

    std::pop_heap(heap_.begin(), heap_.end(), Later{});
    Entry entry = heap_.back();
    heap_.pop_back();

    expiration.id = entry.id;
    expiration.deadline = entry.deadline;
    expiration.cycles = 1;

    if (entry.period > Duration::zero()) {
      const std::uint64_t period =
          static_cast<std::uint64_t>(entry.period.count());
      const std::uint64_t late =
          static_cast<std::uint64_t>(now.nanos()) -
          static_cast<std::uint64_t>(entry.deadline.nanos());
      const std::uint64_t missed = late / period;
      const std::uint64_t cycles =
          missed == std::numeric_limits<std::uint64_t>::max() ? missed
                                                              : missed + 1;

      expiration.cycles = cycles;

      const std::uint64_t remaining =
          static_cast<std::uint64_t>(MonotonicTime::max().nanos()) -
          static_cast<std::uint64_t>(entry.deadline.nanos());
      // Only re-arm when the next grid phase fits in MonotonicTime; a phase
      // strictly past MonotonicTime::max() can never become due.
      if (missed < remaining / period) {
        entry.deadline = MonotonicTime::from_nanos(static_cast<std::int64_t>(
            static_cast<std::uint64_t>(entry.deadline.nanos()) +
            cycles * period));
        heap_.push_back(entry);
        std::push_heap(heap_.begin(), heap_.end(), Later{});
      }
    }

    return true;
  }

 private:
  struct Entry {
    MonotonicTime deadline;
    std::uint16_t id;
    Duration period;
  };

  // Greater-than, because std::*_heap builds a max-heap and we want the
  // earliest deadline on top. Ties break on the lower id firing first.
  struct Later {
    bool operator()(const Entry& a, const Entry& b) const {
      if (a.deadline != b.deadline) {
        return a.deadline > b.deadline;
      }
      return a.id > b.id;
    }
  };

  std::vector<Entry> heap_;
};

}  // namespace talos::event

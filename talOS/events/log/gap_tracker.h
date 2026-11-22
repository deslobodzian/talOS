#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>

#include "talOS/events/log/format.h"

namespace talos::event::log {

// A flags word a writer ORs `bit` into as a gap opens (LogWriterOptions::
// gap_flags): a node's registry row, `flags` its flags word and `bit`
// introspect::kFlagLogGap. `owner`, when set, says whose the word is: the
// bit is set only while it still holds `expected` (an acquire load), so a
// row another process has claimed since (the registry's generation) is
// left alone. Every word must outlive the writer, or be taken back first
// (LogWriter::set_gap_flags).
struct GapFlagTarget {
  std::atomic<std::uint32_t>* flags{nullptr};
  std::uint32_t bit{0};
  const std::atomic<std::uint64_t>* owner{nullptr};
  std::uint64_t expected{0};
};

// Producer-side gap accounting for LogWriter: tracks an open gap, counts
// dropped records/bytes/dispatches/sends, holds an EXIT dropped during a gap,
// and exposes lock-free counters to observers on any thread.
//
// All mutating methods run on the producer thread only (or before start()),
// in O(1) without locking, waiting, or allocating (RULES.md 3).
class GapTracker {
 public:
  void set_gap_flags(const GapFlagTarget& target) noexcept {
    gap_flags_ = target;
  }

  void open(const RecordHeader& header, GapCause cause) noexcept {
    losing_ = true;
    losing_flag_.store(true, std::memory_order_relaxed);
    gap_count_.store(gap_count_.load(std::memory_order_relaxed) + 1,
                     std::memory_order_relaxed);
    // Reported now, not at a later sample: a run killed before its GAP
    // record is written has said its log lost records.
    if (gap_flags_.flags != nullptr &&
        (gap_flags_.owner == nullptr ||
         gap_flags_.owner->load(std::memory_order_acquire) ==
             gap_flags_.expected)) {
      gap_flags_.flags->fetch_or(gap_flags_.bit, std::memory_order_relaxed);
    }
    gap_ = GapRecord{};
    gap_.cause = static_cast<std::uint16_t>(cause);
    gap_.first_lost_ns = header.now_ns;
    gap_.last_lost_ns = std::numeric_limits<std::int64_t>::min();
    // The dispatch the gap starts in had records taken before it.
    if (header.dispatch_index != 0 &&
        header.dispatch_index == last_accepted_dispatch_) {
      gap_.flags |= GAP_SPLITS_DISPATCH;
    }
    gap_first_dispatch_ = header.dispatch_index;
    gap_start_ns_ = std::numeric_limits<std::int64_t>::max();
  }

  // A dropped record: counted in the open gap, or, an EXIT, held for the stop.
  // No copy, no checksum, no trailing-zero scan.
  void lose(const RecordHeader& header, std::size_t payload_bytes) noexcept {
    const auto kind = static_cast<EventKind>(header.kind);
    if (kind == EventKind::EXIT) {
      held_exit_ = header;
      return;
    }
    ++gap_.records;
    gap_.bytes += sizeof(RecordHeader) + payload_bytes;
    const bool opens = kind == EventKind::TIMER || kind == EventKind::MESSAGE;
    const bool sends = kind == EventKind::SEND;
    if (opens) ++gap_.dispatches;
    if (sends) ++gap_.sends;
    // Operation records (READ, ARM_TIMER, ...) name no time of their own: they
    // belong to a dispatch whose opener is counted or was recorded.
    if (opens || sends) {
      gap_start_ns_ = std::min(gap_start_ns_, header.event_time_ns);
    }
    if (kind == EventKind::MESSAGE || sends) {
      gap_start_ns_ = std::min(gap_start_ns_, header.publish_time_ns);
    }
    gap_.last_lost_ns = std::max(gap_.last_lost_ns, header.now_ns);
    gap_.last_dispatch_index = header.dispatch_index;
    lost_records_.store(lost_records_.load(std::memory_order_relaxed) + 1,
                        std::memory_order_relaxed);
  }

  // True when `header` starts a new dispatch (or belongs to none), so capture
  // can resume with it without splitting a dropped dispatch.
  bool can_resume_at(const RecordHeader& header) const noexcept {
    return header.dispatch_index == 0 ||
           header.dispatch_index != gap_.last_dispatch_index;
  }

  // Builds and seals the GAP record for the currently open gap, returning the
  // stored bytes slice into `gap_` (valid until the next open()).
  std::span<const std::byte> seal_gap(RecordHeader& header,
                                      std::int64_t now_ns) noexcept {
    header = RecordHeader{};
    header.kind = static_cast<std::uint16_t>(EventKind::GAP);
    header.source_id = GAP_SOURCE_ID;
    header.dispatch_index = gap_first_dispatch_;
    // With only operation records dropped, none named a time: the first
    // dropped record's.
    header.event_time_ns =
        gap_start_ns_ == std::numeric_limits<std::int64_t>::max()
            ? gap_.first_lost_ns
            : gap_start_ns_;
    header.now_ns = now_ns;
    const std::span<const std::byte> payload{
        reinterpret_cast<const std::byte*>(&gap_), sizeof(gap_)};
    return SealRecord(header, payload);
  }

  void mark_closed_at_flush() noexcept { gap_.flags |= GAP_CLOSED_AT_FLUSH; }

  void close() noexcept {
    losing_ = false;
    losing_flag_.store(false, std::memory_order_relaxed);
  }

  void note_accepted(std::uint64_t dispatch_index) noexcept {
    last_accepted_dispatch_ = dispatch_index;
  }

  std::int64_t stop_gap_now_ns() const noexcept {
    return held_exit_.has_value() ? held_exit_->now_ns : gap_.last_lost_ns;
  }

  std::optional<RecordHeader> take_held_exit() noexcept {
    std::optional<RecordHeader> exit = held_exit_;
    held_exit_.reset();
    return exit;
  }

  bool is_losing() const noexcept { return losing_; }
  bool has_pending_stop() const noexcept {
    return losing_ || held_exit_.has_value();
  }

  // Lock-free observers (any thread).
  std::uint64_t gaps() const noexcept {
    return gap_count_.load(std::memory_order_relaxed);
  }
  std::uint64_t lost_records() const noexcept {
    return lost_records_.load(std::memory_order_relaxed);
  }
  bool losing() const noexcept {
    return losing_flag_.load(std::memory_order_relaxed);
  }

 private:
  GapFlagTarget gap_flags_{};
  bool losing_{false};
  std::uint64_t gap_first_dispatch_{0};
  std::int64_t gap_start_ns_{0};
  GapRecord gap_{};
  std::uint64_t last_accepted_dispatch_{0};
  std::optional<RecordHeader> held_exit_;

  std::atomic<bool> losing_flag_{false};
  std::atomic<std::uint64_t> gap_count_{0};
  std::atomic<std::uint64_t> lost_records_{0};
};

}  // namespace talos::event::log

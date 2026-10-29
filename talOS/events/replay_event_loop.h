#pragma once

#include <algorithm>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "talOS/events/event_loop_base.h"
#include "talOS/events/handles.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/recorder.h"
#include "talOS/events/time.h"

namespace talos::event {

// The log does not describe the program that is trying to replay it.
class ReplayError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// The program did something different from what the log says it did.
struct Divergence {
  std::uint64_t dispatch_index{0};
  std::uint16_t source_id{0};
  std::string detail;
};

class ReplayDivergence : public std::runtime_error {
 public:
  explicit ReplayDivergence(const Divergence& divergence)
      : std::runtime_error("replay diverged at dispatch " +
                           std::to_string(divergence.dispatch_index) + ": " +
                           divergence.detail),
        divergence_{divergence} {}

  const Divergence& divergence() const { return divergence_; }

 private:
  Divergence divergence_;
};

// Re-runs a recorded program from its dispatch log.
//
// Nothing is re-derived. Timer firings and message payloads come from the log
// in recorded order, so the handlers see byte-identical inputs at identical
// times no matter how the original run was scheduled. The loop then checks
// what the handlers produce against what they produced originally: every send
// must match, in order, byte for byte.
template <RecorderPolicy Recorder = NullRecorder>
class ReplayEventLoop
    : public EventLoopBase<ReplayEventLoop<Recorder>, Recorder> {
  using Base = EventLoopBase<ReplayEventLoop<Recorder>, Recorder>;
  friend Base;

 public:
  struct Options {
    // Stop at the first difference. Turn this off to collect every divergence
    // in one pass, which is what you want when bisecting a behaviour change.
    bool throw_on_divergence{true};
  };

  explicit ReplayEventLoop(log::LogReader& reader, Options options = Options{})
      : reader_{&reader}, options_{options} {
    load(reader);
  }

  // Replays while recording again. A replay that does not diverge must produce
  // a log identical to the one it was given, which is the strongest statement
  // of correctness this system can make about itself.
  ReplayEventLoop(log::LogReader& reader, Recorder recorder,
                  Options options = Options{})
      : Base{std::move(recorder)}, reader_{&reader}, options_{options} {
    load(reader);
  }

  using SendOutcome = typename Base::SendOutcome;

  MonotonicTime now_impl() const { return now_; }

  // Firings come from the log, but the program must reproduce timer changes.
  void arm_timer(std::uint16_t id, MonotonicTime deadline, Duration period) {
    if (!this->timer_change_is_recorded(id, true, deadline, period)) return;
    const Recorded* recorded = take(EventKind::ARM_TIMER, id);
    if (recorded && (recorded->header.event_time_ns != deadline.nanos() ||
                     recorded->header.sequence !=
                         static_cast<std::uint64_t>(period.count()))) {
      report(Divergence{active_dispatch_, id, "timer schedule changed"});
    }
    this->record_operation(EventKind::ARM_TIMER, id, deadline, period);
  }
  void disarm_timer(std::uint16_t id) {
    if (!this->timer_change_is_recorded(id, false, {}, {})) return;
    take(EventKind::DISARM_TIMER, id);
    this->record_operation(EventKind::DISARM_TIMER, id);
  }

  const std::vector<Divergence>& divergences() const { return divergences_; }
  bool diverged() const { return !divergences_.empty(); }

  // True when the recorded run ended with a clean exit rather than being cut
  // short by a truncated log.
  bool reached_exit() const { return reached_exit_; }

  void run() {
    if (const auto mismatch =
            compare_manifests(reader_->manifest(), this->manifest())) {
      throw ReplayError("log does not match this program: " + *mismatch);
    }

    typename Base::RunScope in_run{*this};
    now_ = reader_->start_time();
    this->begin_run();

    while (cursor_ < records_.size()) {
      const log::RecordHeader& header = records_[cursor_].header;
      const auto kind = static_cast<EventKind>(header.kind);

      if (kind == EventKind::EXIT) {
        ++cursor_;
        now_ = MonotonicTime::from_nanos(header.now_ns);
        reached_exit_ = true;
        break;
      }

      if (!this->running()) {
        report(Divergence{header.dispatch_index, header.source_id,
                          "program exited before the recorded run"});
        break;
      }

      if (is_operation(kind)) {
        // Reached at top level, so the handler that originally performed it
        // did not perform it this time.
        report(Divergence{header.dispatch_index, header.source_id,
                          std::string{"program skipped a recorded "} +
                              to_string(kind) + " on source " +
                              source_name(header.source_id)});
        ++cursor_;
        continue;
      }

      replay_dispatch();
    }

    this->end_run(now_);
  }

 private:
  void load(log::LogReader& reader) {
    // Adopting the recorded origin before any registration is what lets the
    // replayed program build its initial schedule the same way the recorded
    // one did, out of monotonic_now(), instead of being handed the original
    // run's absolute deadlines by the caller.
    now_ = reader.start_time();
    this->set_start_time(now_);

    // Keep only the stored bytes: the zero tail is restored where a payload
    // is used, so a long replay holds what the file holds, not the padding.
    log::LogReader::Record record{};
    reader.rewind();
    while (reader.next(record)) {
      records_.push_back(Recorded{record.header, stored_.size()});
      const auto bytes = record.payload.first(record.header.stored_bytes);
      stored_.insert(stored_.end(), bytes.begin(), bytes.end());
    }
  }

  struct Recorded {
    log::RecordHeader header;
    std::size_t offset;  // of its stored bytes in stored_
  };

  std::span<const std::byte> stored(const Recorded& record) const {
    return {stored_.data() + record.offset, record.header.stored_bytes};
  }

  // The recorded payload at full size. Valid until the next call.
  std::span<const std::byte> full_payload(const Recorded& record) {
    const std::span<const std::byte> bytes = stored(record);
    payload_.assign(record.header.payload_bytes, std::byte{0});
    std::copy(bytes.begin(), bytes.end(), payload_.begin());
    return payload_;
  }

  void on_register(const Registration&) {}
  void on_exit_requested() {}
  void on_handler_exit() {
    take(EventKind::HANDLER_EXIT, this->context().source_id);
  }

  static bool is_operation(EventKind kind) {
    return kind == EventKind::SEND || kind == EventKind::ARM_TIMER ||
           kind == EventKind::DISARM_TIMER || kind == EventKind::HANDLER_EXIT;
  }

  std::string source_name(std::uint16_t id) const {
    const Manifest& manifest = this->manifest();
    if (id < manifest.size()) {
      return "'" + manifest[id].name + "'";
    }
    return "#" + std::to_string(id);
  }

  void report(const Divergence& divergence) {
    divergences_.push_back(divergence);
    if (options_.throw_on_divergence) {
      throw ReplayDivergence(divergence);
    }
  }

  void replay_dispatch() {
    const Recorded& record = records_[cursor_];
    const log::RecordHeader header = record.header;
    ++cursor_;

    now_ = MonotonicTime::from_nanos(header.now_ns);
    const MonotonicTime event_time =
        MonotonicTime::from_nanos(header.event_time_ns);

    if (header.source_id >= this->manifest().size()) {
      report(Divergence{header.dispatch_index, header.source_id,
                        "log refers to a source this program does not have"});
      return;
    }

    active_dispatch_ = header.dispatch_index;

    if (static_cast<EventKind>(header.kind) == EventKind::TIMER) {
      this->dispatch_timer(header.source_id, event_time, now_, header.sequence);
    } else {
      this->dispatch_message(header.source_id, event_time, now_,
                             header.sequence, header.aux, full_payload(record));
    }

    active_dispatch_ = 0;

    // Anything the handler failed to consume belonged to this dispatch and
    // means it stopped short of what it did originally.
    while (cursor_ < records_.size()) {
      const log::RecordHeader& next = records_[cursor_].header;
      const auto kind = static_cast<EventKind>(next.kind);

      if (!is_operation(kind) || next.dispatch_index != header.dispatch_index) {
        break;
      }

      report(Divergence{next.dispatch_index, next.source_id,
                        std::string{"handler stopped before a recorded "} +
                            to_string(kind) + " on source " +
                            source_name(next.source_id)});
      ++cursor_;
    }
  }

  // Consumes the record a handler operation must correspond to, or reports the
  // mismatch. Returns nullptr when there is nothing usable.
  const Recorded* take(EventKind expected, std::uint16_t source_id) {
    if (cursor_ >= records_.size()) {
      report(Divergence{active_dispatch_, source_id,
                        std::string{"program performed an extra "} +
                            to_string(expected) + " after the log ended"});
      return nullptr;
    }

    const log::RecordHeader& header = records_[cursor_].header;
    const auto kind = static_cast<EventKind>(header.kind);

    if (kind != expected || header.source_id != source_id ||
        header.dispatch_index != active_dispatch_) {
      report(Divergence{
          active_dispatch_, source_id,
          std::string{"expected "} + to_string(kind) + " on source " +
              source_name(header.source_id) + " but the program performed " +
              to_string(expected) + " on source " + source_name(source_id)});
      return nullptr;
    }

    return &records_[cursor_++];
  }

  SendOutcome send_impl(std::uint16_t id, std::span<const std::byte> bytes) {
    const Recorded* recorded = take(EventKind::SEND, id);

    if (recorded == nullptr) {
      return SendOutcome{SendStatus::FAILED, 0};
    }

    // Compared against the stored bytes plus the zero tail they stand for,
    // without expanding them.
    const log::RecordHeader& header = recorded->header;
    const std::span<const std::byte> prefix = stored(*recorded);
    if (bytes.size() != header.payload_bytes ||
        !std::equal(prefix.begin(), prefix.end(), bytes.begin()) ||
        std::any_of(bytes.begin() + static_cast<std::ptrdiff_t>(prefix.size()),
                    bytes.end(),
                    [](std::byte b) { return b != std::byte{0}; })) {
      report(Divergence{active_dispatch_, id,
                        "sent different bytes on source " + source_name(id)});
    }

    // Report the recorded outcome so a re-recorded replay reproduces the
    // original log exactly rather than the transport's current state.
    return SendOutcome{static_cast<SendStatus>(header.aux), header.sequence};
  }

  log::LogReader* reader_;
  Options options_;
  std::vector<Recorded> records_;
  std::vector<std::byte> stored_;   // every record's stored bytes, in order
  std::vector<std::byte> payload_;  // the dispatch payload being replayed
  std::vector<Divergence> divergences_;
  std::size_t cursor_{0};
  std::uint64_t active_dispatch_{0};
  MonotonicTime now_{};
  bool reached_exit_{false};
};

}  // namespace talos::event

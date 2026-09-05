#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "talOS/events/event_loop_base.h"
#include "talOS/events/handles.h"
#include "talOS/events/metrics.h"
#include "talOS/events/os/poller.h"
#include "talOS/events/recorder.h"
#include "talOS/events/scheduler.h"
#include "talOS/events/time.h"
#include "talOS/rtms/rtms.h"

namespace talos::event {

// The loop that runs on the robot.
//
// Timers come from the OS clock through the poller; messages are drained on a
// fixed tick rather than woken by the transport, because RTMS carries no
// notification channel and adding one would change the shared-memory layout.
// The tick therefore bounds message latency, and every drained message is
// timestamped with the tick that observed it, which is exactly what replay
// needs to reproduce.
template <RecorderPolicy Recorder = NullRecorder,
          MetricsPolicy Metrics = NullMetrics>
class RealtimeEventLoop
    : public EventLoopBase<RealtimeEventLoop<Recorder, Metrics>, Recorder,
                           Metrics> {
  using Base =
      EventLoopBase<RealtimeEventLoop<Recorder, Metrics>, Recorder, Metrics>;
  friend Base;

 public:
  struct Options {
    // How often watched topics are drained. 1 ms bounds message latency at one
    // control cycle for a 1 kHz loop.
    Duration tick_period{std::chrono::milliseconds{1}};

    // Cap on messages drained per topic per tick, so one flooded topic cannot
    // starve the timers.
    std::size_t max_messages_per_tick{64};

    // Ring slots per topic. Every process on a topic must agree.
    std::size_t slots{MAX_SLOTS};
  };

  explicit RealtimeEventLoop(Options options = Options{}) : options_{options} {}

  explicit RealtimeEventLoop(Recorder recorder, Options options = Options{})
      : Base{std::move(recorder)}, options_{options} {}

  using SendOutcome = typename Base::SendOutcome;
  using FetchResult = typename Base::FetchResult;

  MonotonicTime now_impl() const { return Poller::now(); }

  void arm_timer(std::uint16_t id, MonotonicTime deadline, Duration period) {
    scheduler_.schedule(id, deadline, period);

    // Waking is a syscall, and it is pointless in the two common cases: before
    // the loop starts, and from inside a dispatch, where the loop is not
    // waiting and re-reads the schedule the moment the handler returns. That
    // leaves only the case that needs it, an arm from another thread while the
    // loop sleeps.
    if (this->running() && !this->in_dispatch()) {
      poller_.wake();
    }
  }

  void disarm_timer(std::uint16_t id) { scheduler_.disable(id); }

  // Runs until exit() is called.
  void run() { run_until(MonotonicTime::max()); }

  void run_for(Duration duration) { run_until(Poller::now() + duration); }

  void run_until(MonotonicTime end) {
    MonotonicTime now = Poller::now();
    this->begin_run(now);

    scheduler_.schedule(INTERNAL_POLL_TIMER_ID, now + options_.tick_period,
                        options_.tick_period);

    while (this->running()) {
      now = Poller::now();

      if (now >= end) {
        break;
      }

      Expiration expiration{};
      while (this->running() && scheduler_.pop_expired(now, expiration)) {
        if (expiration.id == INTERNAL_POLL_TIMER_ID) {
          drain_messages(expiration.deadline, now);
        } else {
          this->dispatch_timer(expiration.id, expiration.deadline, now,
                               expiration.cycles);
        }
      }

      if (!this->running()) {
        break;
      }

      const MonotonicTime deadline = std::min(scheduler_.next_deadline(), end);

      poller_.wait_until(deadline);
    }

    this->end_run(Poller::now());
  }

 private:
  struct Transport {
    std::optional<RTMSQueue> queue;
    std::optional<std::size_t> reader_id;
  };

  void on_register(const Registration& registration) {
    transports_.resize(registration.id + 1);

    if (registration.kind == SourceKind::TIMER) {
      return;
    }

    // Watchers must see every message in order; fetchers only ever want the
    // newest. Both let the writer overwrite rather than block, so a subscriber
    // that falls behind cannot stall the publishing process.
    const RTMSOptions rtms_options{
        .overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
        .read_mode = registration.kind == SourceKind::FETCHER
                         ? ReadMode::LATEST
                         : ReadMode::SEQUENCE,
    };

    Transport transport{};
    transport.queue.emplace(registration.name, registration.message_bytes,
                            registration.alignment, options_.slots,
                            rtms_options);

    if (registration.kind != SourceKind::SENDER) {
      transport.reader_id = transport.queue->register_reader();
    }

    if (registration.kind == SourceKind::WATCHER) {
      // Collected once so the drain runs over just the watchers instead of
      // re-scanning every source on every tick.
      watchers_.push_back(registration.id);
    }

    transports_[registration.id] = std::move(transport);
  }

  void on_exit_requested() { poller_.wake(); }

  SendOutcome send_impl(std::uint16_t id, std::span<const std::byte> bytes) {
    RTMSQueue& queue = *transports_[id].queue;

    const WriteStatus status =
        queue.write(RTMSMessage{bytes.size(), bytes.data()});

    switch (status.result) {
      case WriteResult::SUCCESS:
        return SendOutcome{SendStatus::OK, status.sequence};
      case WriteResult::BUFFER_FULL:
        return SendOutcome{SendStatus::DROPPED, 0};
      default:
        return SendOutcome{SendStatus::FAILED, 0};
    }
  }

  FetchResult fetch_impl(std::uint16_t id, std::span<std::byte> destination) {
    Transport& transport = transports_[id];

    if (!transport.reader_id) {
      return FetchResult{FetchOutcome::EMPTY, 0, 0};
    }

    MessageInfo info{};
    const ReadResult result =
        transport.queue->read_next(*transport.reader_id, destination, info);

    if (result != ReadResult::OK) {
      return FetchResult{FetchOutcome::EMPTY, 0, 0};
    }

    return FetchResult{FetchOutcome::VALUE, info.sequence, info.dropped};
  }

  // `tick_deadline` is when this drain was supposed to happen and `now` is when
  // it did. Stamping messages with both makes a watcher's dispatch latency mean
  // something: how late the loop was to deliver. It cannot mean time since
  // publication, because the frozen shared-memory layout carries no publish
  // timestamp; measure that end to end with tools/loop_perf.cc instead.
  void drain_messages(MonotonicTime tick_deadline, MonotonicTime now) {
    auto& sources = this->sources();

    // Registration order, which is the order the log records and replay
    // expects.
    for (const std::uint16_t id : watchers_) {
      Transport& transport = transports_[id];
      if (!transport.reader_id) {
        continue;
      }

      for (std::size_t drained = 0; drained < options_.max_messages_per_tick;
           ++drained) {
        MessageInfo info{};

        const ReadResult result = transport.queue->read_next(
            *transport.reader_id, sources[id].buffer, info);

        if (result != ReadResult::OK) {
          break;
        }

        this->dispatch_message(static_cast<std::uint16_t>(id), tick_deadline,
                               now, info.sequence, info.dropped,
                               sources[id].buffer);

        if (!this->running()) {
          return;
        }
      }
    }
  }

  Options options_;
  Poller poller_;
  Scheduler scheduler_;
  std::vector<Transport> transports_;
  std::vector<std::uint16_t> watchers_;
};

}  // namespace talos::event

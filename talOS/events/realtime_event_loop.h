#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "talOS/events/event_loop_base.h"
#include "talOS/events/handles.h"
#include "talOS/events/os/poller.h"
#include "talOS/events/recorder.h"
#include "talOS/events/scheduler.h"
#include "talOS/events/time.h"
#include "talOS/rtms/rtms.h"

namespace talos::event {

// The loop that runs on the robot.
//
// Timers come from the OS clock through the poller; messages are drained
// rather than woken by the transport, because RTMS carries no notification
// channel and adding one would change the shared-memory layout. Under
// Delivery::POLL the drain runs on a fixed tick, which bounds message latency;
// under Delivery::BEFORE_TIMERS it runs right before each timer dispatch, so a
// periodic program wakes once per period instead of once per tick. Either way
// every drained message is timestamped with the drain that observed it, which
// is exactly what replay needs to reproduce.
template <RecorderPolicy Recorder = NullRecorder>
class RealtimeEventLoop
    : public EventLoopBase<RealtimeEventLoop<Recorder>, Recorder> {
  using Base = EventLoopBase<RealtimeEventLoop<Recorder>, Recorder>;
  friend Base;

 public:
  struct Options {
    // How often watched topics are drained under Delivery::POLL. 1 ms bounds
    // message latency at one control cycle for a 1 kHz loop.
    Duration tick_period{std::chrono::milliseconds{1}};

    // Cap on messages drained per topic per drain, so one flooded topic
    // cannot starve the timers.
    std::size_t max_messages_per_tick{64};

    // Ring slots per topic. Every process on a topic must agree.
    std::size_t slots{MAX_SLOTS};

    Delivery delivery{Delivery::POLL};
  };

  // How often the loop has done each thing so far, for measuring what a
  // delivery policy costs. Read between runs or from a dispatch.
  struct Activity {
    std::uint64_t wakeups{0};  // returns from the poller's wait
    std::uint64_t drains{0};   // passes over the watched topics
  };

  // The origin is fixed here rather than in run(), so that a program can build
  // its initial schedule out of monotonic_now() during setup and have replay
  // arrive at the same numbers.
  explicit RealtimeEventLoop(Options options = Options{}) : options_{options} {
    this->set_start_time(Poller::now());
  }

  explicit RealtimeEventLoop(Recorder recorder, Options options = Options{})
      : Base{std::move(recorder)}, options_{options} {
    this->set_start_time(Poller::now());
  }

  // Chosen once the program has registered everything and before run(): the
  // node layer only knows whether a handler needs messages promptly after
  // the program's constructor has run.
  void set_delivery(Delivery delivery) {
    if (this->registration_closed()) {
      throw std::logic_error("delivery is fixed once the run starts");
    }
    options_.delivery = delivery;
  }
  Delivery delivery() const { return options_.delivery; }
  const Activity& activity() const { return activity_; }

  using SendOutcome = typename Base::SendOutcome;

  MonotonicTime now_impl() const { return Poller::now(); }

  void arm_timer(std::uint16_t id, MonotonicTime deadline, Duration period) {
    const bool record =
        this->timer_change_is_recorded(id, true, deadline, period);
    scheduler_.schedule(id, deadline, period);
    if (record) {
      this->record_operation(EventKind::ARM_TIMER, id, deadline, period);
    }
  }

  void disarm_timer(std::uint16_t id) {
    const bool record = this->timer_change_is_recorded(id, false, {}, {});
    scheduler_.disable(id);
    if (record) this->record_operation(EventKind::DISARM_TIMER, id);
  }

  // Runs until exit() is called.
  void run() { run_until(MonotonicTime::max()); }

  void run_for(Duration duration) { run_until(Poller::now() + duration); }

  void run_until(MonotonicTime end) {
    if (options_.tick_period <= Duration::zero()) {
      throw std::invalid_argument("tick period must be positive");
    }
    scheduler_.reserve(this->manifest().size() + 1);
    typename Base::RunScope in_run{*this};
    this->begin_run();

    MonotonicTime now = Poller::now();
    const bool poll = options_.delivery == Delivery::POLL;
    if (poll) {
      scheduler_.schedule(INTERNAL_POLL_TIMER_ID, now + options_.tick_period,
                          options_.tick_period);
    }

    while (this->running()) {
      now = Poller::now();

      if (now >= end) {
        break;
      }

      Expiration expiration{};
      while (this->running() && scheduler_.pop_expired(now, expiration)) {
        if (expiration.id == INTERNAL_POLL_TIMER_ID) {
          drain_messages(expiration.deadline, now);
          continue;
        }
        if (!poll) {
          drain_messages(expiration.deadline, now);
          if (!this->running()) break;
        }
        this->dispatch_timer(expiration.id, expiration.deadline, now,
                             expiration.cycles);
      }

      if (!this->running()) {
        break;
      }

      const MonotonicTime deadline = std::min(scheduler_.next_deadline(), end);

      poller_.wait_until(deadline);
      ++activity_.wakeups;
    }

    this->end_run(Poller::now());
  }

 private:
  struct Transport {
    Transport() = default;
    Transport(const Transport&) = delete;
    Transport& operator=(const Transport&) = delete;
    Transport(Transport&& other) noexcept
        : queue{std::move(other.queue)},
          reader_id{std::exchange(other.reader_id, std::nullopt)} {}
    Transport& operator=(Transport&& other) noexcept {
      if (this != &other) {
        release();
        queue = std::move(other.queue);
        reader_id = std::exchange(other.reader_id, std::nullopt);
      }
      return *this;
    }
    ~Transport() { release(); }
    void release() {
      if (reader_id) queue->release_reader(*reader_id);
      reader_id.reset();
    }
    std::optional<RTMSQueue> queue;
    std::optional<std::size_t> reader_id;
  };

  void on_register(const Registration& registration) {
    transports_.resize(registration.id + 1);

    if (registration.kind == SourceKind::TIMER) {
      return;
    }

    // Watchers must see every message in order. The writer overwrites rather
    // than blocks, so a subscriber that falls behind cannot stall the
    // publishing process.
    const RTMSOptions rtms_options{
        .overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
        .read_mode = ReadMode::SEQUENCE,
    };

    Transport transport{};
    transport.queue.emplace(registration.name, registration.message_bytes,
                            registration.alignment, options_.slots,
                            rtms_options);

    if (registration.kind == SourceKind::WATCHER) {
      transport.reader_id = transport.queue->register_reader();
      if (!transport.reader_id) {
        throw RegistrationError("no reader slots available for '" +
                                registration.name + "'");
      }
      // Collected once so the drain runs over just the watchers instead of
      // re-scanning every source on every tick.
      watchers_.push_back(registration.id);
    }

    transports_[registration.id] = std::move(transport);
  }

  void on_exit_requested() { poller_.wake(); }
  void on_handler_exit() {}

  SendOutcome send_impl(std::uint16_t id, std::span<const std::byte> bytes) {
    RTMSQueue& queue = *transports_[id].queue;

    const WriteStatus status = queue.write(bytes);

    switch (status.result) {
      case WriteResult::SUCCESS:
        return SendOutcome{SendStatus::OK, status.sequence};
      case WriteResult::BUFFER_FULL:
        return SendOutcome{SendStatus::DROPPED, 0};
      default:
        return SendOutcome{SendStatus::FAILED, 0};
    }
  }

  // `tick_deadline` is when this drain was supposed to happen and `now` is when
  // it did. Stamping messages with both makes a watcher's dispatch latency mean
  // something: how late the loop was to deliver. It cannot mean time since
  // publication, because the frozen shared-memory layout carries no publish
  // timestamp.
  void drain_messages(MonotonicTime tick_deadline, MonotonicTime now) {
    ++activity_.drains;
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
  Activity activity_;
};

}  // namespace talos::event

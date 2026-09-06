#pragma once

#include <cstring>
#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/event_loop_base.h"
#include "talOS/events/handles.h"
#include "talOS/events/metrics.h"
#include "talOS/events/recorder.h"
#include "talOS/events/scheduler.h"
#include "talOS/events/time.h"

namespace talos::event {

// An in-memory topic. Every message ever published is kept, which a real
// transport obviously cannot do, but it makes the simulation exact: a reader
// can always be told precisely how many messages it skipped.
class SimulatedChannel {
 public:
  explicit SimulatedChannel(std::uint32_t message_bytes)
      : message_bytes_{message_bytes} {}

  std::uint32_t message_bytes() const { return message_bytes_; }
  std::uint64_t next_sequence() const { return count_; }

  std::uint64_t publish(std::span<const std::byte> bytes) {
    const std::size_t offset = data_.size();
    data_.resize(offset + message_bytes_, std::byte{0});
    std::memcpy(data_.data() + offset, bytes.data(),
                std::min<std::size_t>(bytes.size(), message_bytes_));
    return count_++;
  }

  void copy_to(std::uint64_t sequence, std::span<std::byte> destination) const {
    std::memcpy(destination.data(), data_.data() + sequence * message_bytes_,
                message_bytes_);
  }

 private:
  std::uint32_t message_bytes_;
  std::uint64_t count_{0};
  std::vector<std::byte> data_;
};

// Topics shared between simulated loops in one test.
class SimulationEnvironment {
 public:
  SimulatedChannel& channel(std::string_view name,
                            std::uint32_t message_bytes) {
    const auto [it, inserted] =
        channels_.try_emplace(std::string{name}, message_bytes);

    if (!inserted && it->second.message_bytes() != message_bytes) {
      throw std::runtime_error("topic '" + std::string{name} +
                               "' used with two different message sizes");
    }
    return it->second;
  }

 private:
  std::map<std::string, SimulatedChannel> channels_;
};

// A loop on a virtual clock.
//
// Time jumps straight to the next deadline, so there is no jitter, no
// scheduling latency and no wall-clock dependence at all: two runs of the same
// program over the same injected inputs produce identical logs. That makes it
// the right place to test the determinism contract itself, with the realtime
// loop then proving the same contract holds when timing is messy.
template <RecorderPolicy Recorder = NullRecorder,
          MetricsPolicy Metrics = NullMetrics>
class SimulatedEventLoop
    : public EventLoopBase<SimulatedEventLoop<Recorder, Metrics>, Recorder,
                           Metrics> {
  using Base =
      EventLoopBase<SimulatedEventLoop<Recorder, Metrics>, Recorder, Metrics>;
  friend Base;

 public:
  struct Options {
    Duration tick_period{std::chrono::milliseconds{1}};
    std::size_t max_messages_per_tick{64};
  };

  explicit SimulatedEventLoop(SimulationEnvironment& environment,
                              Options options = Options{})
      : environment_{&environment}, options_{options} {}

  SimulatedEventLoop(SimulationEnvironment& environment, Recorder recorder,
                     Options options = Options{})
      : Base{std::move(recorder)},
        environment_{&environment},
        options_{options} {}

  using SendOutcome = typename Base::SendOutcome;
  using FetchResult = typename Base::FetchResult;

  MonotonicTime now_impl() const { return now_; }

  void arm_timer(std::uint16_t id, MonotonicTime deadline, Duration period) {
    const bool record = this->timer_change_is_recorded(id, true, deadline,
                                                       period);
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

  // Publishes onto a topic from outside any loop, as a sensor or another
  // process would.
  template <LoopMessage Message>
  std::uint64_t inject(std::string_view topic, const Message& message) {
    return environment_
        ->channel(topic, static_cast<std::uint32_t>(sizeof(Message)))
        .publish(std::span<const std::byte>{
            reinterpret_cast<const std::byte*>(&message), sizeof(Message)});
  }

  void run_for(Duration duration) { run_until(now_ + duration); }

  // Runs to `end` and returns. The loop is quiescent between calls, which is
  // what makes stepping useful: the test can inject messages and re-arm timers
  // in between, exactly as the outside world would.
  void run_until(MonotonicTime end) {
    typename Base::RunScope in_run{*this};

    if (!started_) {
      scheduler_.reserve(this->manifest().size() + 1);
      this->begin_run();
      scheduler_.schedule(INTERNAL_POLL_TIMER_ID, now_ + options_.tick_period,
                          options_.tick_period);
      started_ = true;
    }

    while (this->running_state()) {
      const MonotonicTime deadline = scheduler_.next_deadline();
      if (deadline > end) {
        break;
      }

      now_ = deadline;

      Expiration expiration{};
      while (this->running_state() &&
             scheduler_.pop_expired(now_, expiration)) {
        if (expiration.id == INTERNAL_POLL_TIMER_ID) {
          drain_messages(expiration.deadline);
        } else {
          this->dispatch_timer(expiration.id, expiration.deadline, now_,
                               expiration.cycles);
        }
      }
    }

    if (now_ < end) {
      now_ = end;
    }
  }

  // Ends the run and closes the log. Call once, after the last run_until.
  void finish() {
    if (started_ && !finished_) {
      this->end_run(now_);
      finished_ = true;
    }
  }

  ~SimulatedEventLoop() { finish(); }

 private:
  struct Cursor {
    SimulatedChannel* channel{nullptr};
    std::uint64_t sequence{0};
  };

  bool running_state() const { return this->running(); }

  void on_register(const Registration& registration) {
    cursors_.resize(registration.id + 1);

    if (registration.kind == SourceKind::TIMER) {
      return;
    }

    SimulatedChannel& channel =
        environment_->channel(registration.name, registration.message_bytes);

    // A reader starts at the newest message: it is not entitled to history
    // published before it existed.
    cursors_[registration.id] = Cursor{&channel, channel.next_sequence()};
  }

  void on_exit_requested() {}
  void on_handler_exit() {}

  SendOutcome send_impl(std::uint16_t id, std::span<const std::byte> bytes) {
    Cursor& cursor = cursors_[id];
    return SendOutcome{SendStatus::OK, cursor.channel->publish(bytes)};
  }

  FetchResult fetch_impl(std::uint16_t id, std::span<std::byte> destination) {
    Cursor& cursor = cursors_[id];
    const std::uint64_t available = cursor.channel->next_sequence();

    if (cursor.sequence >= available) {
      return FetchResult{FetchOutcome::EMPTY, 0, 0};
    }

    const std::uint64_t sequence = available - 1;
    const std::uint64_t dropped = sequence - cursor.sequence;

    cursor.channel->copy_to(sequence, destination);
    cursor.sequence = sequence + 1;

    return FetchResult{FetchOutcome::VALUE, sequence, dropped};
  }

  // Virtual time always lands exactly on the deadline, so a message's
  // event_time and observation time coincide and the measured latency is zero.
  // That is the point of the simulated clock, not an omission.
  void drain_messages(MonotonicTime tick_deadline) {
    auto& sources = this->sources();

    for (std::size_t id = 0; id < sources.size(); ++id) {
      if (sources[id].registration.kind != SourceKind::WATCHER) {
        continue;
      }

      Cursor& cursor = cursors_[id];

      for (std::size_t drained = 0;
           drained < options_.max_messages_per_tick &&
           cursor.sequence < cursor.channel->next_sequence();
           ++drained) {
        const std::uint64_t sequence = cursor.sequence;
        cursor.channel->copy_to(sequence, sources[id].buffer);
        cursor.sequence = sequence + 1;

        this->dispatch_message(static_cast<std::uint16_t>(id), tick_deadline,
                               now_, sequence, 0, sources[id].buffer);

        if (!this->running()) {
          return;
        }
      }
    }
  }

  SimulationEnvironment* environment_;
  Options options_;
  Scheduler scheduler_;
  std::vector<Cursor> cursors_;
  MonotonicTime now_{};
  bool started_{false};
  bool finished_{false};
};

}  // namespace talos::event

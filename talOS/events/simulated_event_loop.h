#pragma once

#include <array>
#include <cstring>
#include <functional>
#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/event_loop_base.h"
#include "talOS/events/handles.h"
#include "talOS/events/recorder.h"
#include "talOS/events/scheduler.h"
#include "talOS/events/time.h"
#include "talOS/rtms/rtms.h"

namespace talos::event {

// A bounded overwriting topic, with the same retained window as RTMS.
class SimulatedChannel {
 public:
  explicit SimulatedChannel(std::uint32_t message_bytes)
      : message_bytes_{message_bytes}, data_(MAX_SLOTS * message_bytes) {}

  std::uint32_t message_bytes() const { return message_bytes_; }
  std::uint64_t next_sequence() const { return count_; }
  std::uint64_t oldest_sequence() const {
    // RTMS reserves the potentially in-progress writer slot even when idle.
    return count_ >= MAX_SLOTS ? count_ - MAX_SLOTS + 1 : 0;
  }

  std::uint64_t publish(std::span<const std::byte> bytes,
                        MonotonicTime publish_time) {
    const std::size_t slot = count_ % MAX_SLOTS;
    const std::size_t offset = slot * message_bytes_;
    std::memset(data_.data() + offset, 0, message_bytes_);
    std::memcpy(data_.data() + offset, bytes.data(),
                std::min<std::size_t>(bytes.size(), message_bytes_));
    publish_times_[slot] = publish_time;
    return count_++;
  }

  void copy_to(std::uint64_t sequence, std::span<std::byte> destination) const {
    check_sequence(sequence);
    std::memcpy(destination.data(),
                data_.data() + (sequence % MAX_SLOTS) * message_bytes_,
                message_bytes_);
  }

  MonotonicTime publish_time(std::uint64_t sequence) const {
    check_sequence(sequence);
    return publish_times_[sequence % MAX_SLOTS];
  }

  // The schema of the first endpoint registered on this topic, which every
  // later one must match, as an RTMS segment's header enforces.
  const schema::Identity& schema() const { return schema_; }
  void set_schema(const schema::Identity& schema) {
    schema_ = schema::Identity{schema.name, schema.hash, {}};
  }

 private:
  void check_sequence(std::uint64_t sequence) const {
    if (sequence < oldest_sequence() || sequence >= count_) {
      throw std::out_of_range("simulated topic sequence is not retained");
    }
  }
  std::uint32_t message_bytes_;
  std::uint64_t count_{0};
  std::vector<std::byte> data_;
  std::array<MonotonicTime, MAX_SLOTS> publish_times_{};
  schema::Identity schema_;
};

// Topics shared between simulated loops in one test.
class SimulationEnvironment {
 public:
  // Looks the topic up without building a key string, so a harness that reads
  // and writes topics every tick (SimHardware) does not allocate in a test's
  // AllocationProbe window.
  SimulatedChannel& channel(std::string_view name,
                            std::uint32_t message_bytes) {
    auto it = channels_.find(name);
    if (it == channels_.end()) {
      it = channels_.try_emplace(std::string{name}, message_bytes).first;
    } else if (it->second.message_bytes() != message_bytes) {
      throw std::runtime_error("topic '" + std::string{name} +
                               "' used with two different message sizes");
    }
    return it->second;
  }

  // The channel for an endpoint of `schema`: an endpoint of another schema
  // is refused, naming both, like an RTMS attach.
  SimulatedChannel& channel(std::string_view name, std::uint32_t message_bytes,
                            const schema::Identity& schema) {
    SimulatedChannel& found = channel(name, message_bytes);
    if (found.schema().name.empty()) {
      found.set_schema(schema);
    } else if (!(found.schema() == schema)) {
      throw RegistrationError("topic '" + std::string{name} + "' carries " +
                              schema::Describe(found.schema()) +
                              "; this endpoint uses " +
                              schema::Describe(schema));
    }
    return found;
  }

 private:
  std::map<std::string, SimulatedChannel, std::less<>> channels_;
};

// A loop on a virtual clock.
//
// Without an injected pause, time jumps straight to the next deadline: no
// jitter, scheduling latency or wall-clock dependence. Pauses advance the
// clock without dispatch, reproducing overdue timers and transport overflow.
// Identical inputs and pauses produce identical logs; the realtime loop then
// proves the same dispatch contract holds when timing is messy.
template <RecorderPolicy Recorder = NullRecorder>
class SimulatedEventLoop
    : public EventLoopBase<SimulatedEventLoop<Recorder>, Recorder> {
  using Base = EventLoopBase<SimulatedEventLoop<Recorder>, Recorder>;
  friend Base;

 public:
  struct Options {
    Duration tick_period{std::chrono::milliseconds{1}};
    std::size_t max_messages_per_tick{64};
    // As on the realtime loop, so a simulation delivers when the robot would.
    Delivery delivery{Delivery::POLL};
    // The clock's origin: where it starts and what the manifest's timer
    // offsets count from. Input replay (input_replay.h) starts at the
    // recorded session's, so recorded times mean what they meant live.
    MonotonicTime start{};
  };

  // How often the loop has drained its watched topics, as on the realtime
  // loop.
  std::uint64_t drains() const { return drains_; }

  explicit SimulatedEventLoop(SimulationEnvironment& environment,
                              Options options = Options{})
      : environment_{&environment}, options_{options}, now_{options.start} {
    this->set_start_time(options.start);
  }

  SimulatedEventLoop(SimulationEnvironment& environment, Recorder recorder,
                     Options options = Options{})
      : Base{std::move(recorder)},
        environment_{&environment},
        options_{options},
        now_{options.start} {
    this->set_start_time(options.start);
  }

  // Fixed before the first run, like the realtime loop's.
  void set_delivery(Delivery delivery) {
    if (this->registration_closed()) {
      throw std::logic_error("delivery is fixed once the run starts");
    }
    options_.delivery = delivery;
  }
  Delivery delivery() const { return options_.delivery; }

  using SendOutcome = typename Base::SendOutcome;

  MonotonicTime now_impl() const { return now_; }

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

  // Publishes onto a topic from outside any loop, as a sensor or another
  // process would.
  // Harness input, like a byte topic's write: it carries no schema, so any
  // trivially copyable message of the topic's size goes.
  template <typename Message>
    requires std::is_trivially_copyable_v<Message>
  std::uint64_t inject(std::string_view topic, const Message& message) {
    return environment_
        ->channel(topic, static_cast<std::uint32_t>(sizeof(Message)))
        .publish(
            std::span<const std::byte>{
                reinterpret_cast<const std::byte*>(&message), sizeof(Message)},
            now_);
  }

  void run_for(Duration duration) { run_until(now_ + duration); }

  // Whether the first run has begun, arming the manifest's timers (and the
  // poll tick); before it next_deadline() is not yet meaningful.
  bool started() const { return started_; }

  // The earliest armed deadline: when the next run would first dispatch, or
  // MonotonicTime::max() with nothing armed. A harness stepping several
  // loops advances them all to the earliest of theirs, so none runs past a
  // time another has not reached (talos::testing::Sim).
  MonotonicTime next_deadline() const { return scheduler_.next_deadline(); }

  // A stopped process's clock advances without draining or firing timers.
  void advance_until(MonotonicTime end) {
    if (end < now_)
      throw std::invalid_argument("simulation clock went backward");
    now_ = end;
  }

  // Runs to `end` and returns. The loop is quiescent between calls, which is
  // what makes stepping useful: the test can inject messages and re-arm timers
  // in between, exactly as the outside world would.
  void run_until(MonotonicTime end) {
    typename Base::RunScope in_run{*this};

    if (!started_) {
      scheduler_.reserve(this->manifest().size() + 1);
      this->begin_run();
      if (options_.delivery == Delivery::POLL) {
        scheduler_.schedule(INTERNAL_POLL_TIMER_ID,
                            options_.start + options_.tick_period,
                            options_.tick_period);
      }
      started_ = true;
    }

    while (this->running_state()) {
      const MonotonicTime deadline = scheduler_.next_deadline();
      if (deadline > end) {
        break;
      }

      now_ = std::max(now_, deadline);

      Expiration expiration{};
      while (this->running_state() &&
             scheduler_.pop_expired(now_, expiration)) {
        if (expiration.id == INTERNAL_POLL_TIMER_ID) {
          drain_messages(expiration.deadline);
          continue;
        }
        // Under either delivery, as on the realtime loop.
        drain_messages(expiration.deadline);
        if (!this->running_state()) break;
        this->dispatch_timer(expiration.id, expiration.deadline, now_,
                             expiration.cycles);
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

    SimulatedChannel& channel = environment_->channel(
        registration.name, registration.message_bytes, registration.schema);

    // A reader starts at the newest message: it is not entitled to history
    // published before it existed.
    cursors_[registration.id] = Cursor{&channel, channel.next_sequence()};
  }

  void on_exit_requested() {}
  void on_handler_exit() {}

  SendOutcome send_impl(std::uint16_t id, std::span<const std::byte> bytes) {
    Cursor& cursor = cursors_[id];
    return SendOutcome{SendStatus::OK,
                       cursor.channel->publish(bytes, this->monotonic_now())};
  }

  // Normally deadlines and observation times coincide; a paused process
  // instead observes overdue dispatches with its advanced clock. Only
  // publications made by now_ are visible: loops sharing an environment may
  // stand at different times, and a reader never sees its future. A
  // latest-only watcher takes the newest visible message; an ordered one
  // stops at the first invisible one.
  void drain_messages(MonotonicTime tick_deadline) {
    ++drains_;
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
        const std::uint64_t first =
            std::max(cursor.sequence, cursor.channel->oldest_sequence());
        std::uint64_t sequence = first;
        if (sources[id].latest_only) {
          sequence = cursor.channel->next_sequence() - 1;
          while (sequence > first &&
                 cursor.channel->publish_time(sequence) > now_) {
            --sequence;
          }
        }
        if (cursor.channel->publish_time(sequence) > now_) break;
        const std::uint64_t dropped = sequence - cursor.sequence;
        cursor.channel->copy_to(sequence, sources[id].buffer);
        cursor.sequence = sequence + 1;

        this->dispatch_message(static_cast<std::uint16_t>(id), tick_deadline,
                               now_, sequence, dropped, sources[id].buffer,
                               cursor.channel->publish_time(sequence));

        if (!this->running()) {
          return;
        }
        if (sources[id].latest_only) break;
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
  std::uint64_t drains_{0};
};

}  // namespace talos::event

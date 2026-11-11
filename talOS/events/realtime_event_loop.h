#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <functional>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "talOS/events/event_loop_base.h"
#include "talOS/events/handles.h"
#include "talOS/events/os/poller.h"
#include "talOS/events/recorder.h"
#include "talOS/events/scheduler.h"
#include "talOS/events/time.h"
#include "talOS/rtms/rtms.h"
#include "talOS/rtms/wake.h"

namespace talos::event {

// The loop that runs on the robot.
//
// Timers come from the OS clock through the poller. A drain runs right before
// each timer dispatch, so a timer handler sees what was published by its
// deadline whatever the delivery. Delivery::POLL also drains on a fixed tick;
// under Delivery::BEFORE_TIMERS nothing else drains, so a periodic program
// wakes once per period instead of once per tick. Under Delivery::ON_PUBLISH
// the loop owns a WakeReceiver that each prompt watcher's reader slot names
// (RTMSQueue::set_reader_wake): before blocking it arms every prompt watcher
// (RTMSQueue::arm_wake), and a publication on any of them makes the
// receiver's fd readable and the poller return, so an idle loop sleeps until
// its next timer or the next prompt publication. Every drained message is
// timestamped with the drain that observed it, which is exactly what replay
// needs to reproduce, whatever woke the loop. watch_fd() adds a descriptor
// whose readiness ends the same waits, for what must reach the loop thread
// from outside the program (talOS/node: a stop signal).
template <RecorderPolicy Recorder = NullRecorder>
class RealtimeEventLoop
    : public EventLoopBase<RealtimeEventLoop<Recorder>, Recorder> {
  using Base = EventLoopBase<RealtimeEventLoop<Recorder>, Recorder>;
  friend Base;

 public:
  struct Options {
    // How often watched topics are drained under Delivery::POLL, besides
    // before each timer. 1 ms bounds message latency at one control cycle for
    // a 1 kHz loop.
    Duration tick_period{std::chrono::milliseconds{1}};

    // Cap on messages drained per topic per drain, so one flooded topic
    // cannot starve the timers.
    std::size_t max_messages_per_tick{64};

    // Ring slots per topic. Every process on a topic must agree.
    std::size_t slots{MAX_SLOTS};

    Delivery delivery{Delivery::POLL};

    // Under Delivery::ON_PUBLISH, the longest the loop blocks without
    // draining while it has prompt watchers. A liveness backstop, not a
    // delivery mechanism: a publisher that dies between publishing and
    // notifying leaves its message unannounced, and this bounds how long it
    // waits. A loop whose next timer is nearer never pays for it.
    Duration wake_backstop{std::chrono::seconds{1}};

    // A stop flag the loop reads between dispatches, null for none. Once it
    // reads true the loop exits on its own thread: before its next timer,
    // its next message (a burst or a run of woken drains included) and its
    // next wait, so a stop takes effect after the current dispatch even
    // while overdue timers would never let it wait. One relaxed load per
    // dispatch; the flag only ever wakes the loop through a watched fd
    // (watch_fd), which a sleeping loop needs. talOS/node passes
    // &process::stop_requested.
    const std::atomic<bool>* stop{nullptr};

    // Test seams, null in production. `make_wake_receiver` builds the
    // receiver under Delivery::ON_PUBLISH instead of WakeReceiver's
    // constructor (a test throws from it to reach the POLL fallback).
    // `arm_wake` stands in for RTMSQueue::arm_wake before each block (a test
    // reports unread data the read never returns, to reach the stall guard).
    WakeReceiver (*make_wake_receiver)(){nullptr};
    bool (*arm_wake)(RTMSQueue& queue, std::size_t reader_id){nullptr};
  };

  // How often the loop has done each thing so far, for measuring what a
  // delivery policy costs. Read between runs or from a dispatch.
  struct Activity {
    std::uint64_t wakeups{0};  // returns from the poller's wait
    std::uint64_t drains{0};   // passes over the watched topics
    // Under Delivery::ON_PUBLISH, prompt watchers whose failed arms stopped
    // holding the loop awake: kMaxStalledArms drains in a row, each run
    // because the watcher's arm reported something to read, delivered
    // nothing from it. Should stay 0; see wait_for_publication.
    std::uint64_t stalled_arms{0};
    // Runs that asked for Delivery::ON_PUBLISH but could not build their
    // wake receiver, and ran under Delivery::POLL instead (said once on
    // stderr). Should stay 0.
    std::uint64_t wake_fallbacks{0};
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

  // Watches `fd` for the run: whenever a wait returns with `fd` readable,
  // `on_readable` runs on the loop thread, outside any dispatch, before
  // whatever else that wait returned for. For process plumbing that must
  // reach the loop thread without a thread of its own polling for it
  // (talOS/node watches process::StopFd() and calls exit()), not for program
  // inputs: nothing is recorded, and replay never runs it. The watch is
  // level-triggered: `on_readable` reads `fd` empty or calls exit(), or the
  // loop never blocks again. A wait counts in activity().wakeups whatever
  // woke it; this one alone drains nothing. Call before run(); throws
  // std::logic_error after, and std::system_error when the poller refuses
  // `fd`.
  void watch_fd(int fd, std::function<void()> on_readable) {
    if (this->registration_closed()) {
      throw std::logic_error("fds are watched before the run starts");
    }
    if (fd < 0 || !on_readable) {
      throw std::invalid_argument("watch_fd needs an fd and a handler");
    }
    poller_.add_fd(fd);
    fd_watches_.push_back({fd, std::move(on_readable)});
  }

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
    if (options_.wake_backstop <= Duration::zero()) {
      throw std::invalid_argument("wake backstop must be positive");
    }
    scheduler_.reserve(this->manifest().size() + 1);
    typename Base::RunScope in_run{*this};
    this->begin_run();
    if (options_.delivery == Delivery::ON_PUBLISH) {
      start_wakeups();
    }

    MonotonicTime now = Poller::now();
    // The schedule set up before the run counts from here rather than from
    // the origin: what ran between construction and run() (a node's config,
    // imports and device setup) is not lateness, and would otherwise be
    // reported as the first dispatch's latency and missed periods. Firings
    // are recorded at the deadlines used and replay takes them from the log,
    // so the manifest's origin-relative offsets and replay are unchanged.
    scheduler_.shift(now - this->start_time());
    if (options_.delivery == Delivery::POLL) {
      scheduler_.schedule(INTERNAL_POLL_TIMER_ID, now + options_.tick_period,
                          options_.tick_period);
    }

    while (keep_running()) {
      now = Poller::now();

      if (now >= end) {
        break;
      }

      Expiration expiration{};
      while (keep_running()) {
        now = Poller::now();
        if (!scheduler_.pop_expired(now, expiration)) break;
        if (expiration.id == INTERNAL_POLL_TIMER_ID) {
          drain_messages(expiration.deadline, now);
          continue;
        }
        // Under POLL too: the poll tick due at this deadline would otherwise
        // run after the timer (ties break on id), and update() would see
        // inputs one tick late, and later than a node without callbacks.
        drain_messages(expiration.deadline, now);
        if (!keep_running()) break;
        now = Poller::now();
        this->dispatch_timer(expiration.id, expiration.deadline, now,
                             expiration.cycles);
      }

      if (!keep_running()) {
        break;
      }

      const MonotonicTime deadline = std::min(scheduler_.next_deadline(), end);

      if (!wake_) {
        if (poller_.wait_until(deadline) == Poller::WakeReason::READABLE) {
          serve_watched_fds();
        }
        ++activity_.wakeups;
        continue;
      }

      if (const auto drain = wait_for_publication(deadline)) {
        drain_messages(*drain, Poller::now(), /*woken=*/true);
      }
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
          reader_id{std::exchange(other.reader_id, std::nullopt)},
          next_sequence{other.next_sequence},
          lost{other.lost},
          prompt{other.prompt},
          progress{other.progress},
          stall_progress{other.stall_progress},
          stalls{other.stalls} {}
    Transport& operator=(Transport&& other) noexcept {
      if (this != &other) {
        release();
        queue = std::move(other.queue);
        reader_id = std::exchange(other.reader_id, std::nullopt);
        next_sequence = other.next_sequence;
        lost = other.lost;
        prompt = other.prompt;
        progress = other.progress;
        stall_progress = other.stall_progress;
        stalls = other.stalls;
      }
      return *this;
    }
    ~Transport() { release(); }
    void release() {
      if (reader_id && queue) queue->release_reader(*reader_id);
      reader_id.reset();
    }
    std::optional<RTMSQueue> queue;
    std::optional<std::size_t> reader_id;
    // A watcher's next undelivered sequence, and the messages published
    // while it had no reader slot, owed to its next delivery's dropped count.
    std::uint64_t next_sequence{0};
    std::uint64_t lost{0};
    // Its reader slot names the loop's WakeReceiver (ON_PUBLISH only).
    bool prompt{false};
    // Drains that delivered from it or found its slot freed, and, for the
    // stall guard in wait_for_publication: the progress at its last failed
    // arm, and how many failed arms in a row have drained since it moved.
    std::uint64_t progress{0};
    std::uint64_t stall_progress{0};
    std::uint32_t stalls{0};
  };

  // Failed arms of one prompt watcher that may each drain in place of a
  // wait while nothing is delivered from it (see wait_for_publication).
  static constexpr std::uint32_t kMaxStalledArms = 3;

  // Delivery::ON_PUBLISH, at the start of the run: one receiver for the
  // loop, named by every prompt watcher's reader slot. Without a prompt
  // watcher nothing could wake the loop, so it runs as BEFORE_TIMERS and
  // holds no receiver.
  void start_wakeups() {
    for (const std::uint16_t id : watchers_) {
      transports_[id].prompt = this->source(id).prompt;
      if (transports_[id].prompt) prompt_watchers_.push_back(id);
    }
    if (prompt_watchers_.empty()) return;
    try {
      if (options_.make_wake_receiver != nullptr) {
        wake_.emplace(options_.make_wake_receiver());
      } else {
        wake_.emplace();
      }
      poller_.add_fd(wake_->fd());
    } catch (const std::exception& error) {
      // Out of descriptors, say: a node that cannot be woken still runs,
      // at the cost of POLL's wakeups.
      wake_.reset();
      for (const std::uint16_t id : prompt_watchers_) {
        transports_[id].prompt = false;
      }
      prompt_watchers_.clear();
      options_.delivery = Delivery::POLL;
      ++activity_.wake_fallbacks;
      std::fprintf(stderr,
                   "talos: event loop cannot be woken on publish (%s); it "
                   "drains every %lld us instead\n",
                   error.what(),
                   static_cast<long long>(
                       std::chrono::duration_cast<std::chrono::microseconds>(
                           options_.tick_period)
                           .count()));
      return;
    }
    for (const std::uint16_t id : prompt_watchers_) {
      Transport& transport = transports_[id];
      if (transport.queue && transport.reader_id) {
        transport.queue->set_reader_wake(*transport.reader_id, wake_->id());
      }
    }
  }

  // Delivery::ON_PUBLISH's wait. Arms every prompt watcher and blocks until
  // `deadline`, a prompt publication or the backstop, only once every arm
  // reported its watcher caught up: a publication after a successful arm
  // makes the receiver readable, and one before it fails the arm. Returns
  // the time to stamp a drain with (when the loop saw the publication, or
  // the backstop's deadline, as a poll tick's), or nullopt when the poller
  // returned for a timer, the end or exit(), which the timer pass handles.
  //
  // Stall guard: a failed arm drains instead of waiting, but if
  // kMaxStalledArms such drains in a row deliver nothing from that watcher
  // (no message dispatched, no freed slot found), its failed arm stops
  // keeping the loop awake, counted once in Activity::stalled_arms, until a
  // drain delivers from it again. The loop then waits as if it were armed:
  // a failed arm still requested a notification, and the backstop bounds
  // the wait. Without it, a transport that kept reporting unread data the
  // read never returns (a read the writer keeps tearing) would spin.
  std::optional<MonotonicTime> wait_for_publication(MonotonicTime deadline) {
    for (const std::uint16_t id : prompt_watchers_) {
      Transport& transport = transports_[id];
      // A watcher without a slot cannot be armed; each drain retries its
      // rejoin, and the backstop bounds the wait for the next one.
      if (!transport.queue || !transport.reader_id) continue;
      const bool armed =
          options_.arm_wake != nullptr
              ? options_.arm_wake(*transport.queue, *transport.reader_id)
              : transport.queue->arm_wake(*transport.reader_id);
      if (armed) {
        transport.stalls = 0;
        continue;
      }
      if (transport.progress != transport.stall_progress) {
        transport.stall_progress = transport.progress;
        transport.stalls = 0;
      }
      if (transport.stalls < kMaxStalledArms) {
        ++transport.stalls;
        return Poller::now();
      }
      if (transport.stalls == kMaxStalledArms) {
        ++transport.stalls;
        ++activity_.stalled_arms;
      }
    }
    const MonotonicTime now = Poller::now();
    const MonotonicTime backstop =
        options_.wake_backstop < MonotonicTime::max() - now
            ? now + options_.wake_backstop
            : MonotonicTime::max();
    const auto reason = poller_.wait_until(std::min(deadline, backstop));
    ++activity_.wakeups;
    if (reason == Poller::WakeReason::READABLE) {
      serve_watched_fds();
      // A watched fd alone is no publication: nothing to drain.
      if (!this->running() || !wake_ ||
          !poller_.reported_readable(wake_->fd())) {
        return std::nullopt;
      }
      wake_->drain();
      return Poller::now();
    }
    if (reason == Poller::WakeReason::DEADLINE && backstop < deadline) {
      return backstop;
    }
    return std::nullopt;
  }

  // running(), after exiting once the stop flag (Options::stop) reads true.
  bool keep_running() {
    if (options_.stop != nullptr && this->running() &&
        options_.stop->load(std::memory_order_relaxed)) {
      this->exit();
    }
    return this->running();
  }

  // Runs the handler of each watched fd the last wait reported readable.
  void serve_watched_fds() {
    for (FdWatch& watch : fd_watches_) {
      if (!this->running()) return;
      if (poller_.reported_readable(watch.fd)) watch.on_readable();
    }
  }

  void on_register(const Registration& registration) {
    transports_.resize(registration.id + 1);

    if (registration.kind == SourceKind::TIMER) {
      return;
    }

    // Latest-value subscriptions skip backlog; per-message callbacks retain
    // sequence delivery. Neither kind can block the publishing process.
    const RTMSOptions rtms_options{
        .overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
        .read_mode = this->source(registration.id).latest_only
                         ? ReadMode::LATEST
                         : ReadMode::SEQUENCE,
        .publisher = registration.kind == SourceKind::SENDER,
    };

    Transport transport{};
    transport.queue.emplace(registration.name, registration.message_bytes,
                            registration.alignment, registration.schema,
                            options_.slots, rtms_options);

    if (registration.kind == SourceKind::WATCHER) {
      transport.reader_id = transport.queue->register_reader();
      if (!transport.reader_id) {
        throw RegistrationError("no reader slots available for '" +
                                registration.name + "'");
      }
      transport.next_sequence =
          transport.queue->reader_sequence(*transport.reader_id);
      // Collected once so the drain runs over just the watchers instead of
      // re-scanning every source on every tick.
      watchers_.push_back(registration.id);
    }

    transports_[registration.id] = std::move(transport);
  }

  void on_exit_requested() { poller_.wake(); }
  void on_handler_exit() {}

  SendOutcome send_impl(std::uint16_t id, std::span<const std::byte> bytes) {
    std::optional<RTMSQueue>& slot = transports_[id].queue;
    if (!slot) return SendOutcome{SendStatus::FAILED, 0};
    RTMSQueue& queue = *slot;

    const WriteStatus status =
        queue.write(bytes, this->monotonic_now().nanos());

    switch (status.result) {
      case WriteResult::SUCCESS:
        return SendOutcome{SendStatus::OK, status.sequence};
      case WriteResult::BUFFER_FULL:
        return SendOutcome{SendStatus::DROPPED, 0};
      default:
        return SendOutcome{SendStatus::FAILED, 0};
    }
  }

  // Dispatch scheduling latency remains separate from age: the slot supplies
  // the publication timestamp even when a consumer has stalled before draining.
  //
  // A message's event time is the drain's `tick_deadline` (a timer's or poll
  // tick's deadline, or when a woken loop saw its wake), except in a drain
  // that is `woken` (a wake, a failed arm or the backstop) for a prompt
  // watcher: the loop owed that message a dispatch from the moment it was
  // published, so its event time is its publication time (not after the
  // dispatch's now), and latency() is publish-to-dispatch. A watcher that is
  // not prompt waited for the drain by design and keeps the drain's time.
  void drain_messages(MonotonicTime tick_deadline, MonotonicTime now,
                      bool woken = false) {
    ++activity_.drains;
    auto& sources = this->sources();

    // Registration order, which is the order the log records and replay
    // expects.
    for (const std::uint16_t id : watchers_) {
      Transport& transport = transports_[id];
      if (!transport.queue) {
        continue;
      }
      RTMSQueue& queue = *transport.queue;
      // Left without a slot by a failed rejoin below: keep trying.
      if (!transport.reader_id) {
        rejoin(queue, transport);
      }
      if (!transport.reader_id) {
        continue;
      }
      const std::size_t reader_id = *transport.reader_id;

      for (std::size_t drained = 0; drained < options_.max_messages_per_tick;
           ++drained) {
        MessageInfo info{};

        const ReadResult result =
            queue.read_next(reader_id, sources[id].buffer, info);

        if (result != ReadResult::OK) {
          // EMPTY ends the drain. A read the writer tore, or one that found
          // the reader slot freed by another process that judged this one
          // dead, is counted as dropped rather than ending the subscription
          // silently; a freed slot is claimed again.
          if (result == ReadResult::TORN || result == ReadResult::INACTIVE) {
            this->count_dropped(id, 1);
          }
          if (result == ReadResult::INACTIVE) {
            ++transport.progress;
            transport.reader_id.reset();
            rejoin(queue, transport);
          }
          break;
        }

        transport.next_sequence = info.sequence + 1;
        ++transport.progress;
        now = Poller::now();
        const MonotonicTime published =
            MonotonicTime::from_nanos(info.publish_time_ns);
        const MonotonicTime event_time = woken && transport.prompt
                                             ? std::min(published, now)
                                             : tick_deadline;
        this->dispatch_message(static_cast<std::uint16_t>(id), event_time, now,
                               info.sequence,
                               info.dropped + std::exchange(transport.lost, 0),
                               sources[id].buffer, published);

        if (!keep_running()) {
          return;
        }
        if (sources[id].latest_only) break;
      }
    }
  }

  // Claims a reader slot again for a watcher whose slot was freed under it.
  // The new slot starts at the writer, so what was published since the last
  // delivery is owed to the next one's dropped count. A prompt watcher's new
  // slot names the receiver again, and the next arm covers it.
  void rejoin(RTMSQueue& queue, Transport& transport) {
    transport.reader_id = queue.register_reader();
    if (!transport.reader_id) {
      return;
    }
    if (transport.prompt && wake_) {
      queue.set_reader_wake(*transport.reader_id, wake_->id());
    }
    const std::uint64_t start = queue.reader_sequence(*transport.reader_id);
    if (start > transport.next_sequence) {
      transport.lost += start - transport.next_sequence;
    }
    transport.next_sequence = start;
  }

  Options options_;
  Poller poller_;
  Scheduler scheduler_;
  // Delivery::ON_PUBLISH with a prompt watcher: the receiver their
  // publications wake. Declared before the transports so that their reader
  // slots, which name it, are released first.
  std::optional<WakeReceiver> wake_;
  std::vector<Transport> transports_;
  std::vector<std::uint16_t> watchers_;
  // The prompt watchers, in registration order (ON_PUBLISH only).
  std::vector<std::uint16_t> prompt_watchers_;
  struct FdWatch {
    int fd;
    std::function<void()> on_readable;
  };
  std::vector<FdWatch> fd_watches_;
  Activity activity_;
};

}  // namespace talos::event

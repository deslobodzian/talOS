#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/callback.h"
#include "talOS/events/context.h"
#include "talOS/events/manifest.h"
#include "talOS/events/metrics.h"
#include "talOS/events/recorder.h"
#include "talOS/events/time.h"

namespace talos::event {

// The message-poll timer is owned by the realtime loop, not by user code. It
// is deliberately outside the source id space so it never lands in a manifest:
// replay has nothing to poll, and a source that exists in one mode but not the
// other would break every log comparison.
inline constexpr std::uint16_t INTERNAL_POLL_TIMER_ID = 0xFFFF;
inline constexpr std::size_t MAX_SOURCES = 0xFFFE;

// Outcome of publishing a message, as recorded in the log.
enum class SendStatus : std::uint32_t {
  OK = 0,
  DROPPED = 1,  // transport refused it (buffer full under a reliable policy)
  FAILED = 2,   // transport error
};

class RegistrationError : public std::logic_error {
 public:
  using std::logic_error::logic_error;
};

// Live activity for one event source, published for anything outside the loop
// that wants to know what the program is actually doing: a heartbeat, a
// telemetry viewer, an agent asking which topics are moving.
//
// Separate from LoopMetrics, and not a substitute for it. LoopMetrics keeps
// whole latency distributions in plain arrays for the owning thread to report
// at the end of a run; these are lock-free atomics precisely so another thread
// can read them mid-run without a data race. Six relaxed stores per dispatch,
// no clock reads beyond the ones the dispatch already made -- cheap enough to
// leave on in a 1 kHz loop, which is the point: observability that has to be
// switched on is observability you do not have when you need it.
struct SourceCounters {
  // Timer or watcher dispatches, messages published by a sender, values
  // actually received by a fetcher. In each case: how often this source did
  // its job.
  std::atomic<std::uint64_t> events{0};

  // Messages the transport lapped before this source read them, plus publishes
  // the transport refused.
  std::atomic<std::uint64_t> dropped{0};

  // Last transport sequence this source observed, which is what makes a
  // subscriber's lag measurable against its publisher.
  std::atomic<std::uint64_t> sequence{0};

  std::atomic<std::int64_t> last_now_ns{0};
  std::atomic<std::int64_t> last_latency_ns{0};
  std::atomic<std::int64_t> max_latency_ns{0};
};

// Shared machinery for every loop: source registration, id assignment, the
// dispatch entry point that freezes time and drives the recorder, and the
// send/fetch paths that funnel handler output through the same recorder.
//
// CRTP, not inheritance with virtuals. The derived loop supplies:
//   MonotonicTime now_impl() const
//   void on_register(const Registration&)          (optional side table setup)
//   SendOutcome send_impl(uint16_t id, std::span<const std::byte>)
//   FetchOutcomeBytes fetch_impl(uint16_t id, std::span<std::byte> destination)
//   void on_exit_requested()                        (wake a blocked wait)
template <typename Derived, RecorderPolicy Recorder,
          MetricsPolicy Metrics = NullMetrics>
class EventLoopBase {
 public:
  struct SendOutcome {
    SendStatus status{SendStatus::OK};
    std::uint64_t sequence{0};
  };

  struct FetchResult {
    FetchOutcome outcome{FetchOutcome::EMPTY};
    std::uint64_t sequence{0};
    std::uint64_t dropped{0};
  };

  EventLoopBase() = default;

  // Takes an already-configured recorder, because a log writer needs a
  // destination before the loop can start.
  explicit EventLoopBase(Recorder recorder) : recorder_{std::move(recorder)} {}

  // Handles hold a pointer to the loop, so it must not move once handlers
  // exist. Loops are long-lived and stack- or member-owned; this is not a
  // limitation in practice.
  EventLoopBase(const EventLoopBase&) = delete;
  EventLoopBase& operator=(const EventLoopBase&) = delete;
  EventLoopBase(EventLoopBase&&) = delete;
  EventLoopBase& operator=(EventLoopBase&&) = delete;

  Recorder& recorder() { return recorder_; }
  Metrics& metrics() { return metrics_; }
  const Metrics& metrics() const { return metrics_; }
  const Manifest& manifest() const { return manifest_; }

  // Non-null from begin_run() until destruction, and never reallocated, so a
  // reporting thread may hold the pointer for the life of the run. Null before
  // the run starts, because registration is still open and the array is sized
  // from the finished manifest.
  const SourceCounters* source_counters() const { return counters_.get(); }

  std::size_t source_counter_count() const {
    return counters_ ? manifest_.size() : 0;
  }

  // The only clock a handler may read. Frozen for the duration of a dispatch
  // so that two reads inside one callback cannot disagree, which is what makes
  // the recorded value sufficient to reproduce the run.
  MonotonicTime monotonic_now() const {
    if (in_dispatch()) {
      return context_.now;
    }

    // Before the run: the loop's origin, which is fixed at construction and is
    // therefore the same number when recording and when replaying. A program
    // building its initial schedule out of this gets a manifest that matches.
    //
    // Once the run has started: the loop's own clock, so a harness stepping a
    // simulation between run_for calls sees time where it left it rather than
    // back at the origin.
    return registration_closed_ ? derived().now_impl() : start_time_;
  }

  MonotonicTime start_time() const { return start_time_; }
  const Context& context() const { return context_; }
  bool in_dispatch() const { return active_loop_ == this; }
  bool running() const { return running_; }

  std::uint64_t dispatch_count() const { return dispatch_index_; }

  // --- registration -------------------------------------------------------
  // All of these must happen before run(). Ids are handed out in call order
  // and become the log's identifiers for these sources.

  std::uint16_t register_timer(std::string_view name, Thunk callback) {
    return add_source(SourceKind::TIMER, name, 0, 0, callback);
  }

  std::uint16_t register_watcher(std::string_view topic,
                                 std::uint32_t message_bytes,
                                 std::uint32_t alignment, Thunk callback) {
    return add_source(SourceKind::WATCHER, topic, message_bytes, alignment,
                      callback);
  }

  std::uint16_t register_fetcher(std::string_view topic,
                                 std::uint32_t message_bytes,
                                 std::uint32_t alignment) {
    return add_source(SourceKind::FETCHER, topic, message_bytes, alignment,
                      Thunk{});
  }

  std::uint16_t register_sender(std::string_view topic,
                                std::uint32_t message_bytes,
                                std::uint32_t alignment) {
    return add_source(SourceKind::SENDER, topic, message_bytes, alignment,
                      Thunk{});
  }

  // --- handler-facing operations ------------------------------------------

  // Publishes bytes and records what happened. Returns true when the transport
  // accepted the message.
  bool send_message(std::uint16_t id, std::span<const std::byte> bytes) {
    const SendOutcome outcome = derived().send_impl(id, bytes);
    recorder_.send(context_, id, bytes,
                   static_cast<std::uint32_t>(outcome.status),
                   outcome.sequence);

    if (SourceCounters* counters = counter(id)) {
      if (outcome.status == SendStatus::OK) {
        counters->events.fetch_add(1, std::memory_order_relaxed);
        counters->sequence.store(outcome.sequence, std::memory_order_relaxed);
        counters->last_now_ns.store(context_.now.nanos(),
                                    std::memory_order_relaxed);
      } else {
        counters->dropped.fetch_add(1, std::memory_order_relaxed);
      }
    }
    return outcome.status == SendStatus::OK;
  }

  // Reads the newest value on a fetcher's topic into `destination`, recording
  // the result including the "there was nothing" case: a handler that saw no
  // value must see no value on replay too.
  bool fetch_message(std::uint16_t id, std::span<std::byte> destination) {
    const FetchResult result = derived().fetch_impl(id, destination);
    const bool has_value = result.outcome == FetchOutcome::VALUE;

    recorder_.fetch(context_, id, result.outcome,
                    has_value ? std::span<const std::byte>{destination}
                              : std::span<const std::byte>{},
                    result.sequence, result.dropped);

    if (SourceCounters* counters = counter(id)) {
      // An empty fetch is not an event: a handler that samples a topic every
      // cycle would otherwise report traffic that never arrived.
      if (has_value) {
        counters->events.fetch_add(1, std::memory_order_relaxed);
        counters->sequence.store(result.sequence, std::memory_order_relaxed);
        counters->last_now_ns.store(context_.now.nanos(),
                                    std::memory_order_relaxed);
      }
      if (result.dropped != 0) {
        counters->dropped.fetch_add(result.dropped, std::memory_order_relaxed);
      }
    }

    return has_value;
  }

  // Safe from a handler or another thread, but not from a signal handler.
  // Other operations and lifecycle calls require external serialization.
  void exit() {
    if (in_dispatch()) {
      derived().on_handler_exit();
      record_operation(EventKind::HANDLER_EXIT, context_.source_id);
    }
    running_ = false;
    derived().on_exit_requested();
  }

 protected:
  ~EventLoopBase() = default;

  Derived& derived() { return static_cast<Derived&>(*this); }
  const Derived& derived() const { return static_cast<const Derived&>(*this); }

  struct Source {
    Registration registration;
    Thunk callback;
    std::vector<std::byte> buffer;  // staging for watcher and fetcher payloads
  };

  std::vector<Source>& sources() { return sources_; }
  const std::vector<Source>& sources() const { return sources_; }

  Source& source(std::uint16_t id) { return sources_[id]; }

  void record_operation(EventKind kind, std::uint16_t id,
                        MonotonicTime deadline = {}, Duration period = {}) {
    Context operation = context_;
    operation.kind = kind;
    operation.source_id = id;
    operation.event_time = deadline;
    operation.sequence = static_cast<std::uint64_t>(period.count());
    operation.dropped = 0;
    recorder_.dispatch(operation, {});
  }

  // Classifies a timer change and returns true when it has to appear in the
  // log.
  //
  // Three cases, and the difference between them is who caused the change:
  //  - inside a dispatch: the recorded program did it, so it is an output and
  //    replay must see the program do it again. Recorded.
  //  - before the run: it is part of the program's shape, so it goes in the
  //    manifest and replay is rejected outright if it differs.
  //  - while the loop is stopped between runs: the harness did it from
  //    outside, exactly like injecting a message. Not recorded, because its
  //    only effect is the firings it produces, and those are recorded.
  //
  // A change while the loop is actually running and not dispatching can only
  // come from another thread. That is rejected before any loop-owned state is
  // touched, because the scheduler is not thread-safe and because such a
  // change has no reproducible position in the log.
  bool timer_change_is_recorded(std::uint16_t id, bool armed,
                                MonotonicTime deadline, Duration period) {
    if (in_run_ && !in_dispatch()) {
      throw std::logic_error(
          "timer changes while the loop is running require a loop callback");
    }
    if (id >= manifest_.size() || manifest_[id].kind != SourceKind::TIMER) {
      throw std::invalid_argument("invalid timer id");
    }
    if (period < Duration::zero()) {
      throw std::invalid_argument("timer period must not be negative");
    }
    if (in_dispatch()) return true;

    if (!registration_closed_) {
      // Stored relative to the loop's origin. An absolute deadline is a
      // property of when the machine happened to boot, so recording one would
      // make every realtime log unreplayable unless the caller carried the
      // number across by hand.
      auto& reg = manifest_[id];
      reg.armed = armed;
      reg.offset_ns = armed ? (deadline - start_time_).count() : 0;
      reg.period_ns = armed ? period.count() : 0;
    }
    return false;
  }

  // Fixes the loop's origin. Called by the derived loop before any
  // registration, so that monotonic_now() is meaningful during setup and the
  // manifest's timer offsets mean the same thing on record and on replay.
  void set_start_time(MonotonicTime start) {
    if (registration_closed_) {
      throw std::logic_error("the loop origin is fixed once the run starts");
    }
    start_time_ = start;
  }

  // Marks the loop as being inside its run function, which is what makes a
  // timer change from another thread detectable.
  class RunScope {
   public:
    explicit RunScope(EventLoopBase& loop) : loop_{loop} {
      loop_.in_run_ = true;
    }
    ~RunScope() { loop_.in_run_ = false; }

    RunScope(const RunScope&) = delete;
    RunScope& operator=(const RunScope&) = delete;

   private:
    EventLoopBase& loop_;
  };

  // Freezes registration and opens the log. Every loop calls this once, first
  // thing in run(), after set_start_time().
  void begin_run() {
    if (registration_closed_) {
      throw std::logic_error("event loop cannot be restarted");
    }
    const MonotonicTime start = start_time_;
    registration_closed_ = true;

    // Sized from the finished manifest and never resized, so the pointer a
    // reporting thread reads stays valid for the whole run. std::atomic is not
    // movable, which rules out growing a vector of these as sources register.
    counters_ = std::make_unique<SourceCounters[]>(manifest_.size());

    running_ = true;
    dispatch_index_ = 0;
    context_ = Context{};
    recorder_.start(manifest_, start);
    metrics_.configure(manifest_);
    metrics_.begin_run(steady_now());
  }

  void end_run(MonotonicTime now) {
    running_ = false;

    Context exit_context{};
    exit_context.kind = EventKind::EXIT;
    exit_context.dispatch_index = ++dispatch_index_;
    exit_context.event_time = now;
    exit_context.now = now;

    recorder_.finish(exit_context);
    recorder_.flush();
    metrics_.end_run(steady_now());
    context_ = Context{};
  }

  void dispatch_timer(std::uint16_t id, MonotonicTime deadline,
                      MonotonicTime now, std::uint64_t cycles) {
    Context context{};
    context.kind = EventKind::TIMER;
    context.source_id = id;
    context.dispatch_index = ++dispatch_index_;
    context.event_time = deadline;
    context.now = now;
    context.sequence = cycles;

    invoke(context, std::span<const std::byte>{});
  }

  void dispatch_message(std::uint16_t id, MonotonicTime event_time,
                        MonotonicTime now, std::uint64_t sequence,
                        std::uint64_t dropped,
                        std::span<const std::byte> payload) {
    Context context{};
    context.kind = EventKind::MESSAGE;
    context.source_id = id;
    context.dispatch_index = ++dispatch_index_;
    context.event_time = event_time;
    context.now = now;
    context.sequence = sequence;
    context.dropped = dropped;

    invoke(context, payload);
  }

 private:
  // The dispatch record is written before the handler runs so that the sends
  // and fetches it performs are ordered after their parent in the log.
  void invoke(const Context& context, std::span<const std::byte> payload) {
    struct DispatchScope {
      EventLoopBase& loop;
      EventLoopBase* previous;
      ~DispatchScope() {
        active_loop_ = previous;
        loop.context_ = Context{};
      }
    } scope{*this, active_loop_};
    active_loop_ = this;
    context_ = context;
    recorder_.dispatch(context, payload);

    if (SourceCounters* counters = counter(context.source_id)) {
      const std::int64_t latency = context.latency().count();
      counters->events.fetch_add(1, std::memory_order_relaxed);
      counters->sequence.store(context.sequence, std::memory_order_relaxed);
      counters->last_now_ns.store(context.now.nanos(),
                                  std::memory_order_relaxed);
      counters->last_latency_ns.store(latency, std::memory_order_relaxed);
      if (latency > counters->max_latency_ns.load(std::memory_order_relaxed)) {
        counters->max_latency_ns.store(latency, std::memory_order_relaxed);
      }
      if (context.dropped != 0) {
        counters->dropped.fetch_add(context.dropped, std::memory_order_relaxed);
      }
    }

    const Thunk& callback = sources_[context.source_id].callback;

    // The clock reads exist only when a metrics policy wants them, so an
    // uninstrumented loop does not pay for measurement it never reads.
    if constexpr (Metrics::ENABLED) {
      const MonotonicTime begin = steady_now();
      if (callback.valid()) {
        callback(context, payload);
      }
      metrics_.record(context, steady_now() - begin);
    } else {
      if (callback.valid()) {
        callback(context, payload);
      }
    }

    context_ = Context{};
  }

  std::uint16_t add_source(SourceKind kind, std::string_view name,
                           std::uint32_t message_bytes, std::uint32_t alignment,
                           Thunk callback) {
    if (registration_closed_) {
      throw RegistrationError(
          "event sources must be registered before run(): '" +
          std::string{name} + "'");
    }

    if (sources_.size() >= MAX_SOURCES) {
      throw RegistrationError("too many event sources");
    }

    if (name.size() > MAX_SOURCE_NAME) {
      throw RegistrationError("source name longer than " +
                              std::to_string(MAX_SOURCE_NAME) +
                              " characters: '" + std::string{name} + "'");
    }

    Registration registration{};
    registration.id = static_cast<std::uint16_t>(sources_.size());
    registration.kind = kind;
    registration.name = std::string{name};
    registration.message_bytes = message_bytes;
    registration.alignment = alignment;

    Source source{};
    source.registration = registration;
    source.callback = callback;
    source.buffer.resize(message_bytes);

    sources_.push_back(std::move(source));
    manifest_.push_back(registration);

    try {
      derived().on_register(manifest_.back());
    } catch (...) {
      manifest_.pop_back();
      sources_.pop_back();
      throw;
    }
    return registration.id;
  }

  // Null until begin_run(), and bounds-checked because record_operation()
  // passes ids that belong to the source space while INTERNAL_POLL_TIMER_ID
  // deliberately does not.
  SourceCounters* counter(std::uint16_t id) {
    if (!counters_ || id >= manifest_.size()) {
      return nullptr;
    }
    return &counters_[id];
  }

  std::vector<Source> sources_;
  Manifest manifest_;
  std::unique_ptr<SourceCounters[]> counters_;
  Recorder recorder_{};
  Metrics metrics_{};

  Context context_{};
  std::uint64_t dispatch_index_{0};
  MonotonicTime start_time_{};
  inline static thread_local EventLoopBase* active_loop_{nullptr};
  std::atomic<bool> running_{false};
  std::atomic<bool> in_run_{false};
  bool registration_closed_{false};
};

// What a handler is allowed to assume about the loop it was given. Robot code
// is templated on the loop type and constrained by this, so the same handlers
// compile against the realtime, simulated and replay loops.
template <typename T>
concept EventLoopLike =
    requires(T loop, std::uint16_t id, std::span<const std::byte> bytes,
             std::span<std::byte> destination) {
      { loop.monotonic_now() } -> std::same_as<MonotonicTime>;
      { loop.context() } -> std::same_as<const Context&>;
      { loop.send_message(id, bytes) } -> std::same_as<bool>;
      { loop.fetch_message(id, destination) } -> std::same_as<bool>;
      { loop.exit() } -> std::same_as<void>;
    };

}  // namespace talos::event

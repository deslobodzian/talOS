#pragma once

#include <cstddef>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>

#include "talOS/events/callback.h"
#include "talOS/events/context.h"
#include "talOS/events/event_loop_base.h"
#include "talOS/events/time.h"
#include "talOS/ipc/concepts.h"

namespace talos::event {

// Messages crossing a loop boundary are fixed-size flatbuffer structs. Tables
// have no compile-time size, so a slot could not hold them and replay could
// not compare them byte for byte.
template <typename Message>
concept LoopMessage = std::is_trivially_copyable_v<Message> &&
                      ipc::NotDerivedFromFlatbufferTable<Message>;

// Adapts a byte payload to a typed handler.
//
// The bytes are copied into a local rather than reinterpreted in place: in
// replay they come straight out of a log buffer with no alignment guarantee,
// and a misaligned load of a flatbuffer struct is undefined behaviour that
// happens to work on x86 and trap elsewhere.
template <typename Message, auto Method, typename Class>
Thunk make_message_thunk(Class* instance) {
  return Thunk{instance, [](void* object, const Context& context,
                            std::span<const std::byte> payload) {
                 Message message{};
                 if (payload.size() >= sizeof(Message)) {
                   std::memcpy(&message, payload.data(), sizeof(Message));
                 }
                 (static_cast<Class*>(object)->*Method)(context, message);
               }};
}

// A registered timer. Arming and disarming go through the loop so that the
// realtime and simulated loops share one scheduler, and replay can ignore them
// entirely: on replay the firings come from the log, not from the schedule.
template <typename Loop>
class Timer {
 public:
  Timer() = default;
  Timer(Loop* loop, std::uint16_t id) : loop_{loop}, id_{id} {}

  void setup(MonotonicTime at) { loop_->arm_timer(id_, at, Duration::zero()); }

  // Fires first at `first`, then every `period`. Deadlines stay on the grid
  // defined by `first`, so a late cycle does not shift the phase.
  void setup_periodic(MonotonicTime first, Duration period) {
    loop_->arm_timer(id_, first, period);
  }

  void disable() { loop_->disarm_timer(id_); }

  std::uint16_t id() const { return id_; }
  bool valid() const { return loop_ != nullptr; }

 private:
  Loop* loop_{nullptr};
  std::uint16_t id_{0};
};

template <typename Loop, LoopMessage Message>
class Sender {
 public:
  Sender() = default;
  Sender(Loop* loop, std::uint16_t id) : loop_{loop}, id_{id} {}

  bool send(const Message& message) {
    return loop_->send_message(
        id_,
        std::span<const std::byte>{reinterpret_cast<const std::byte*>(&message),
                                   sizeof(Message)});
  }

  std::uint16_t id() const { return id_; }
  bool valid() const { return loop_ != nullptr; }

 private:
  Loop* loop_{nullptr};
  std::uint16_t id_{0};
};

// Reads the newest value on a topic on demand, skipping anything older. Use a
// fetcher for state a handler samples (a setpoint, a mode) and a watcher for
// events it must not miss.
template <typename Loop, LoopMessage Message>
class Fetcher {
 public:
  Fetcher() = default;
  Fetcher(Loop* loop, std::uint16_t id) : loop_{loop}, id_{id} {}

  std::optional<Message> fetch() {
    Message message{};

    const bool has_value = loop_->fetch_message(
        id_, std::span<std::byte>{reinterpret_cast<std::byte*>(&message),
                                  sizeof(Message)});

    if (!has_value) {
      return std::nullopt;
    }
    return message;
  }

  std::uint16_t id() const { return id_; }
  bool valid() const { return loop_ != nullptr; }

 private:
  Loop* loop_{nullptr};
  std::uint16_t id_{0};
};

// --- registration helpers ---------------------------------------------------

template <auto Method, typename Loop, typename Class>
Timer<Loop> make_timer(Loop& loop, std::string_view name, Class* instance) {
  return Timer<Loop>{&loop,
                     loop.register_timer(name, make_thunk<Method>(instance))};
}

template <LoopMessage Message, auto Method, typename Loop, typename Class>
std::uint16_t watch(Loop& loop, std::string_view topic, Class* instance) {
  return loop.register_watcher(topic,
                               static_cast<std::uint32_t>(sizeof(Message)),
                               static_cast<std::uint32_t>(alignof(Message)),
                               make_message_thunk<Message, Method>(instance));
}

template <LoopMessage Message, typename Loop>
Fetcher<Loop, Message> make_fetcher(Loop& loop, std::string_view topic) {
  return Fetcher<Loop, Message>{
      &loop,
      loop.register_fetcher(topic, static_cast<std::uint32_t>(sizeof(Message)),
                            static_cast<std::uint32_t>(alignof(Message)))};
}

template <LoopMessage Message, typename Loop>
Sender<Loop, Message> make_sender(Loop& loop, std::string_view topic) {
  return Sender<Loop, Message>{
      &loop,
      loop.register_sender(topic, static_cast<std::uint32_t>(sizeof(Message)),
                           static_cast<std::uint32_t>(alignof(Message)))};
}

}  // namespace talos::event

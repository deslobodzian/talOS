#pragma once

// Framework internals shared by node.cc, testing.cc and the node tests: the
// loop port a Node runs on. Node code never includes this.

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "talOS/events/callback.h"
#include "talOS/events/time.h"

namespace talos::detail {

using Time = event::MonotonicTime;
using Duration = event::Duration;

// The loop a Node is registered on, with the loop type erased. The three loop
// types share a CRTP base rather than a virtual interface, because a handler
// call must cost one indirect jump; a Node pays one virtual call per
// registration, send and clock read instead, which is nothing next to the
// work it does per tick.
class LoopPort {
 public:
  virtual ~LoopPort() = default;
  virtual std::uint16_t register_timer(std::string_view name,
                                       event::Thunk callback) = 0;
  virtual std::uint16_t register_watcher(std::string_view topic,
                                         std::uint32_t message_bytes,
                                         std::uint32_t alignment,
                                         event::Thunk callback) = 0;
  virtual std::uint16_t register_sender(std::string_view topic,
                                        std::uint32_t message_bytes,
                                        std::uint32_t alignment) = 0;
  virtual void arm_timer(std::uint16_t id, Time first, Duration period) = 0;
  virtual bool send(std::uint16_t id, std::span<const std::byte> bytes) = 0;
  virtual Time now() const = 0;
};

template <class Loop>
class LoopPortFor final : public LoopPort {
 public:
  explicit LoopPortFor(Loop& loop) : loop_{loop} {}

  std::uint16_t register_timer(std::string_view name,
                               event::Thunk callback) override {
    return loop_.register_timer(name, callback);
  }
  std::uint16_t register_watcher(std::string_view topic,
                                 std::uint32_t message_bytes,
                                 std::uint32_t alignment,
                                 event::Thunk callback) override {
    return loop_.register_watcher(topic, message_bytes, alignment, callback);
  }
  std::uint16_t register_sender(std::string_view topic,
                                std::uint32_t message_bytes,
                                std::uint32_t alignment) override {
    return loop_.register_sender(topic, message_bytes, alignment);
  }
  void arm_timer(std::uint16_t id, Time first, Duration period) override {
    loop_.arm_timer(id, first, period);
  }
  bool send(std::uint16_t id, std::span<const std::byte> bytes) override {
    return loop_.send_message(id, bytes);
  }
  Time now() const override { return loop_.monotonic_now(); }

 private:
  Loop& loop_;
};

}  // namespace talos::detail

#pragma once

// Framework internals shared by node.cc, testing.cc and the node tests: the
// loop port a Node runs on. Node code never includes this.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>

#include "talOS/events/callback.h"
#include "talOS/events/delivery.h"
#include "talOS/events/manifest.h"
#include "talOS/events/time.h"
#include "talOS/introspection/describe.h"
#include "talOS/schema/identity.h"

namespace talos {
class Node;
struct BytePublishOptions;
}  // namespace talos

namespace talos::detail {

using Time = event::MonotonicTime;
using Duration = event::Duration;

// Where Node::publish_bytes writes: a topic outside the loop.
class ByteTopic {
 public:
  virtual ~ByteTopic() = default;
  // False when the message was not published (a drop_newest queue was full).
  // The caller has checked the length against max_bytes.
  virtual bool write(std::span<const std::byte> message) = 0;
};

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
                                         const schema::Identity& schema,
                                         event::Thunk callback,
                                         bool latest_only) = 0;
  virtual std::uint16_t register_sender(std::string_view topic,
                                        std::uint32_t message_bytes,
                                        std::uint32_t alignment,
                                        const schema::Identity& schema) = 0;
  virtual void arm_timer(std::uint16_t id, Time first, Duration period) = 0;
  virtual bool send(std::uint16_t id, std::span<const std::byte> bytes) = 0;
  virtual Time now() const = 0;
  // Logs that the running handler read watcher `source_id`'s message
  // `sequence`, published at `publish_time` (EventKind::READ); nothing
  // outside a dispatch.
  virtual void record_read(std::uint16_t source_id, Time now, Time publish_time,
                           std::uint64_t sequence) = 0;
  // Chosen by NodeAccess::start, once the node has registered everything.
  // A loop that has no choice (replay delivers what the log says) ignores it.
  virtual void set_delivery(event::Delivery delivery) = 0;

  // The world outside the loop: a byte topic, which the loop neither
  // dispatches nor logs. By default an RTMS queue, opened here while the node
  // is built, as the realtime loop opens its typed topics, so a writer
  // conflict fails construction rather than a send; testing::Sim keeps byte
  // topics in memory instead, and --describe, --replay and --replay-inputs
  // discard them (they never touch a live robot's shared memory, and replay
  // cannot verify what was never logged).
  virtual std::unique_ptr<ByteTopic> open_byte_topic(
      std::string_view topic, std::size_t max_bytes,
      const BytePublishOptions& options);
};

template <class Loop>
class LoopPortFor : public LoopPort {
 public:
  explicit LoopPortFor(Loop& loop) : loop_{loop} {}

  std::uint16_t register_timer(std::string_view name,
                               event::Thunk callback) override {
    return loop_.register_timer(name, callback);
  }
  std::uint16_t register_watcher(std::string_view topic,
                                 std::uint32_t message_bytes,
                                 std::uint32_t alignment,
                                 const schema::Identity& schema,
                                 event::Thunk callback,
                                 bool latest_only) override {
    return loop_.register_watcher(topic, message_bytes, alignment, schema,
                                  callback, latest_only);
  }
  std::uint16_t register_sender(std::string_view topic,
                                std::uint32_t message_bytes,
                                std::uint32_t alignment,
                                const schema::Identity& schema) override {
    return loop_.register_sender(topic, message_bytes, alignment, schema);
  }
  void arm_timer(std::uint16_t id, Time first, Duration period) override {
    loop_.arm_timer(id, first, period);
  }
  bool send(std::uint16_t id, std::span<const std::byte> bytes) override {
    return loop_.send_message(id, bytes);
  }
  Time now() const override { return loop_.monotonic_now(); }
  void record_read(std::uint16_t source_id, Time now, Time publish_time,
                   std::uint64_t sequence) override {
    loop_.record_read(source_id, now, publish_time, sequence);
  }
  void set_delivery(event::Delivery delivery) override {
    if constexpr (requires { loop_.set_delivery(delivery); }) {
      loop_.set_delivery(delivery);
    }
  }

 private:
  Loop& loop_;
};

// What `--describe` prints for `node`, built on a loop whose registrations
// are `manifest`: every topic end with its schema (bytes included), ring
// layout and endpoint flags, the byte topics the loop does not carry, and
// whether the node read the declared graph. testing::Sim builds the declared
// graph it hands its nodes the same way.
introspect::Description DescribeNode(const Node& node, std::string_view target,
                                     const event::Manifest& manifest);

}  // namespace talos::detail

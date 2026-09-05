#pragma once

#include <concepts>
#include <cstddef>
#include <span>

#include "talOS/events/context.h"
#include "talOS/events/manifest.h"
#include "talOS/events/time.h"

namespace talos::event {

// How a fetch turned out. Recorded because a handler that saw no message must
// see no message on replay too.
enum class FetchOutcome : std::uint32_t {
  EMPTY = 0,
  VALUE = 1,
};

// A recorder observes every input a handler receives and every output it
// produces. It is a template policy rather than an interface: NullRecorder
// compiles to nothing, so a loop that is not logging pays no call at all.
template <typename T>
concept RecorderPolicy = requires(T recorder, const Manifest& manifest,
                                  MonotonicTime time, const Context& context,
                                  std::uint16_t source_id,
                                  std::span<const std::byte> payload) {
  { recorder.start(manifest, time) } -> std::same_as<void>;
  { recorder.dispatch(context, payload) } -> std::same_as<void>;
  {
    recorder.fetch(context, source_id, FetchOutcome::EMPTY, payload,
                   std::uint64_t{}, std::uint64_t{})
  } -> std::same_as<void>;
  {
    recorder.send(context, source_id, payload, std::uint32_t{}, std::uint64_t{})
  } -> std::same_as<void>;
  { recorder.finish(context) } -> std::same_as<void>;
  { recorder.flush() } -> std::same_as<void>;
};

// The default policy: records nothing, costs nothing.
struct NullRecorder {
  void start(const Manifest&, MonotonicTime) {}
  void dispatch(const Context&, std::span<const std::byte>) {}
  void fetch(const Context&, std::uint16_t, FetchOutcome,
             std::span<const std::byte>, std::uint64_t, std::uint64_t) {}
  void send(const Context&, std::uint16_t, std::span<const std::byte>,
            std::uint32_t, std::uint64_t) {}
  void finish(const Context&) {}
  void flush() {}
};

static_assert(RecorderPolicy<NullRecorder>);

}  // namespace talos::event

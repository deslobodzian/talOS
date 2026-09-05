#pragma once

#include <cstddef>
#include <span>

#include "talOS/events/context.h"

namespace talos::event {

// A bound member function, stored as two pointers.
//
// std::function would heap-allocate and add an indirection we cannot afford at
// 1 kHz; this is the same {object, thunk} pattern the old EpollHandler used,
// generalised. The thunk is generated at compile time from the member function
// pointer, so the call is one indirect jump with no type erasure beyond it.
class Thunk {
 public:
  using Fn = void (*)(void*, const Context&, std::span<const std::byte>);

  constexpr Thunk() = default;
  constexpr Thunk(void* object, Fn fn) : object_{object}, fn_{fn} {}

  void operator()(const Context& context,
                  std::span<const std::byte> payload) const {
    fn_(object_, context, payload);
  }

  constexpr bool valid() const { return fn_ != nullptr; }
  constexpr void* object() const { return object_; }

 private:
  void* object_{nullptr};
  Fn fn_{nullptr};
};

// Binds &Class::method to an instance. The handler may take (Context) or
// (Context, std::span<const std::byte>); timers use the first form, message
// watchers the second.
template <auto Method, typename Class>
Thunk make_thunk(Class* instance) {
  return Thunk{
      instance, [](void* object, const Context& context,
                   [[maybe_unused]] std::span<const std::byte> payload) {
        auto* self = static_cast<Class*>(object);
        if constexpr (requires { (self->*Method)(context, payload); }) {
          (self->*Method)(context, payload);
        } else {
          (self->*Method)(context);
        }
      }};
}

// Binds a captureless lambda or free function.
template <typename Callable>
Thunk make_thunk(Callable* callable)
  requires requires(Callable& c, const Context& ctx) { c(ctx); }
{
  return Thunk{callable,
               [](void* object, const Context& context,
                  [[maybe_unused]] std::span<const std::byte> payload) {
                 auto& self = *static_cast<Callable*>(object);
                 if constexpr (requires { self(context, payload); }) {
                   self(context, payload);
                 } else {
                   self(context);
                 }
               }};
}

}  // namespace talos::event

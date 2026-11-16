#pragma once

#include <cstdint>

namespace talos::event {

// Told the loop's dispatch count at every dispatch, on the loop thread
// (EventLoopBase::set_dispatch_observer). Runs inside the loop, so it must be
// bounded and lock-free: a few stores, no allocation, no lock, no I/O.
class DispatchObserver {
 public:
  virtual void on_dispatch_count(std::uint64_t count) noexcept = 0;

 protected:
  ~DispatchObserver() = default;
};

}  // namespace talos::event

#pragma once

#include <chrono>
#include <cstdint>

#include "talOS/events/handles.h"
#include "talOS/hardware/gate.h"

namespace sample {

template <typename Loop>
class SampleNode {
 public:
  explicit SampleNode(Loop& loop) {
    timer_ = talos::event::make_timer<&SampleNode::Tick>(loop, "sample_timer", this);
  }

  void Start(talos::event::MonotonicTime first) {
    timer_.setup_periodic(first, std::chrono::milliseconds{10});
  }

  uint64_t ticks() const { return ticks_; }

 private:
  void Tick(const talos::event::Context&) {
    ++ticks_;
  }

  uint64_t ticks_{0};
  talos::event::Timer<Loop> timer_;
};

}  // namespace sample

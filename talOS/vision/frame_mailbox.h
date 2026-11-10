#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "talOS/vision/frame.h"

namespace talos::vision {

// Newest-wins handoff of camera frames from one producer thread (a capture
// callback) to one consumer thread (a detector worker): a triple buffer.
//
// Three slots rotate between the producer's filling slot, the shared ready
// slot and the consumer's processing slot. Publish() copies a frame into the
// filling slot and swaps it with ready; TakeNewest() swaps ready with
// processing. Each side writes only the slot it owns, and ownership changes
// only under the index lock, so the producer never overwrites the frame the
// consumer is reading and the consumer always receives the newest complete
// frame. A ready frame replaced before the consumer took it is a drop.
//
// Every drop of a frame on its way to a result is counted here: replacements
// in the ready slot, frames that hold no image, and whatever the consumer
// reports with CountDropped (driver-side losses, a retired capture session).
//
// Slot storage is allocated by Reserve() and grows only for a frame larger
// than every earlier one, so steady-state publication does not allocate.
class FrameMailbox {
 public:
  FrameMailbox() {
    for (auto& slot : slots_) {
      slot.storage = std::make_shared<std::vector<uint8_t>>();
      slot.frame = Frame(0, 0, 0, talos::event::MonotonicTime::from_nanos(0), 0,
                         0, 0, PixelFormat::kGray8, slot.storage);
    }
  }
  FrameMailbox(const FrameMailbox&) = delete;
  FrameMailbox& operator=(const FrameMailbox&) = delete;

  // Allocates `bytes` of capacity per slot. Call before the producer starts.
  void Reserve(size_t bytes) {
    for (auto& slot : slots_) slot.storage->reserve(bytes);
  }

  // Producer: copies `frame` (borrowed, shared or owned) and makes it the
  // ready frame. Returns false, counting a drop, when it holds no image.
  bool Publish(const Frame& frame) {
    if (!frame.holds_image() || frame.data() == nullptr) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    // filling_ is written only by this thread, under the lock below.
    Slot& slot = slots_[filling_];
    const size_t bytes = frame.size();
    slot.storage->resize(bytes);
    std::memcpy(slot.storage->data(), frame.data(), bytes);
    Frame& copy = slot.frame;
    copy.set_camera_id(frame.camera_id());
    copy.set_session_id(frame.session_id());
    copy.set_sequence_number(frame.sequence_number());
    copy.set_capture_time(frame.capture_time());
    copy.set_raw_timestamp_ns(frame.raw_timestamp_ns());
    copy.set_clock_domain(frame.clock_domain());
    copy.set_timestamp_kind(frame.timestamp_kind());
    copy.set_mapping_quality(frame.mapping_quality());
    copy.set_width(frame.width());
    copy.set_height(frame.height());
    copy.set_stride(frame.stride());
    copy.set_format(frame.format());
    {
      std::lock_guard lock(mutex_);
      std::swap(filling_, ready_);
      if (ready_fresh_) dropped_.fetch_add(1, std::memory_order_relaxed);
      ready_fresh_ = true;
    }
    return true;
  }

  // Consumer: the newest frame published since the last take, or nullptr.
  // The frame stays valid and unchanged until this consumer's next
  // TakeNewest(); do not keep a copy past that (copies share the slot), Clone
  // it instead.
  const Frame* TakeNewest() {
    std::lock_guard lock(mutex_);
    if (!ready_fresh_) return nullptr;
    std::swap(processing_, ready_);
    ready_fresh_ = false;
    return &slots_[processing_].frame;
  }

  bool HasReady() const {
    std::lock_guard lock(mutex_);
    return ready_fresh_;
  }

  // Drops the ready frame, if any (e.g. from a retired capture session).
  void DiscardReady() {
    std::lock_guard lock(mutex_);
    if (!ready_fresh_) return;
    ready_fresh_ = false;
    dropped_.fetch_add(1, std::memory_order_relaxed);
  }

  // Counts frames lost outside the mailbox (driver errors, a taken frame
  // abandoned unprocessed, a result replaced before publication).
  void CountDropped(uint64_t frames) {
    dropped_.fetch_add(frames, std::memory_order_relaxed);
  }

  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

 private:
  struct Slot {
    std::shared_ptr<std::vector<uint8_t>> storage;
    Frame frame;  // Shares `storage`; metadata rewritten by each Publish.
  };

  std::array<Slot, 3> slots_;
  mutable std::mutex mutex_;
  size_t filling_ = 0;
  size_t ready_ = 1;
  size_t processing_ = 2;
  bool ready_fresh_ = false;
  std::atomic<uint64_t> dropped_{0};
};

}  // namespace talos::vision

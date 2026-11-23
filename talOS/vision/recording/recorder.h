#pragma once

// A bounded, asynchronous frame recorder: the camera node hands it a frame
// plus its detection context, and a dedicated writer thread encodes PNG +
// JSON under <output_dir> and enforces max_bytes by deleting the oldest
// pairs. Record() converts the frame to 8-bit gray into a preallocated slot
// (no allocation after construction) and never blocks on disk I/O; when the
// queue is full the frame is dropped and counted.

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

#include "talOS/vision/frame.h"
#include "talOS/vision/recording/recording.h"

namespace talos::vision::recording {

class FrameRecorder {
 public:
  struct Options {
    std::filesystem::path output_dir;    // <session>/frames/<camera>
    double sample_hz{1.0};               // 0 records nothing "sampled"
    double rejected_hz{1.0};             // 0 records nothing "rejected"
    std::uint64_t max_bytes{268435456};  // 256 MiB per camera
    std::size_t queue_slots{8};
    CameraContext context;  // camera name, size, intrinsics, mounting, detector
    // Test-only: the writer thread calls this after taking each pair off the
    // queue and before writing it, outside the queue lock. `stop_requested`
    // reports, under that lock, whether the destructor or a write failure has
    // asked the writer to stop. Null (the default) in production.
    std::function<void(const std::function<bool()>& stop_requested)>
        test_before_pair;
  };

  explicit FrameRecorder(Options options);
  ~FrameRecorder();
  FrameRecorder(const FrameRecorder&) = delete;
  FrameRecorder& operator=(const FrameRecorder&) = delete;

  // Converts `frame` to 8-bit gray into a preallocated slot, copies the
  // detection context, and queues the pair for the writer. Returns true when
  // queued, false when rate-limited or dropped (invalid frame, full queue, or
  // stopped recorder). Writer errors stop recording, not the camera node.
  bool Record(const Frame& frame, const FrameRecord& record);

  std::uint64_t recorded() const;  // frames successfully queued
  std::uint64_t dropped() const;   // frames refused (except rate limiting),
                                   // failed writes, or queued at shutdown

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace talos::vision::recording

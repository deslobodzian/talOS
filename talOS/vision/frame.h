#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/events/time.h"

namespace talos::vision {

// Supported image pixel formats.
enum class PixelFormat {
  kGray8,
  kRgb8,
  kBgr8,
  kYuyv,
  kBgra8,
};

constexpr size_t BytesPerPixel(PixelFormat format) {
  switch (format) {
    case PixelFormat::kGray8:
      return 1;
    case PixelFormat::kRgb8:
    case PixelFormat::kBgr8:
      return 3;
    case PixelFormat::kYuyv:
      return 2;
    case PixelFormat::kBgra8:
      return 4;
  }
  return 1;
}

constexpr std::string_view PixelFormatToString(PixelFormat format) {
  switch (format) {
    case PixelFormat::kGray8:
      return "Gray8";
    case PixelFormat::kRgb8:
      return "Rgb8";
    case PixelFormat::kBgr8:
      return "Bgr8";
    case PixelFormat::kYuyv:
      return "Yuyv";
    case PixelFormat::kBgra8:
      return "Bgra8";
  }
  return "Unknown";
}

// Clock domain in which timestamps were originally recorded.
enum class ClockDomain {
  kHostMonotonic,
  kDeviceMonotonic,
  kSystemRealtime,
  kUnknown,
};

// Meaning of the backend's mapped timestamp, before midpoint correction.
enum class TimestampKind {
  kExposure,
  kDelivery,
  kPresentation,  // Source sample PTS; not asserted to be exposure midpoint.
  kStartOfExposure,
  kEndOfFrame,
};

// Quality indicator for device-to-host timestamp mapping.
enum class TimestampMappingQuality {
  kExact,           // Hardware synced or captured in host monotonic time
  kInterpolated,    // Interpolated from periodic camera-host synchronization
  kEstimated,       // Estimated via arrival time minus estimated latency
  kUnsynchronized,  // Raw unmapped or free-running clock
};

// One timing boundary: mapped source time -> estimated integration midpoint.
// The kind's own geometry comes first: start of exposure adds half the
// exposure, end of frame subtracts readout and half the exposure, and every
// other kind (a known midpoint, delivery, presentation time, which has no
// guaranteed exposure relationship) gets none. Then the camera's measured
// stamp bias, `latency_offset_ns`, is subtracted from every kind: a source's
// label does not prove its stamp is unbiased. nullopt on overflow/underflow.
inline std::optional<uint64_t> CaptureMidpoint(uint64_t stamp_ns,
                                               TimestampKind kind,
                                               uint64_t exposure_ns,
                                               uint64_t readout_ns,
                                               uint64_t latency_offset_ns) {
  uint64_t later = 0;
  uint64_t earlier = latency_offset_ns;
  if (kind == TimestampKind::kStartOfExposure) {
    later = exposure_ns / 2;
  } else if (kind == TimestampKind::kEndOfFrame) {
    if (readout_ns > UINT64_MAX - exposure_ns / 2) return std::nullopt;
    const uint64_t geometry = readout_ns + exposure_ns / 2;
    if (geometry > UINT64_MAX - earlier) return std::nullopt;
    earlier += geometry;
  }
  if (stamp_ns > UINT64_MAX - later) return std::nullopt;
  const uint64_t shifted = stamp_ns + later;
  if (shifted < earlier) return std::nullopt;
  return shifted - earlier;
}

// Carries camera identification, capture timing, dimensions, format,
// and owned or shared frame buffer memory.
class Frame {
 public:
  Frame() = default;

  // Constructs a Frame owning an allocated buffer for given dimensions.
  Frame(uint32_t camera_id, uint64_t session_id, uint64_t sequence_number,
        talos::event::MonotonicTime capture_time, uint32_t width,
        uint32_t height, PixelFormat format = PixelFormat::kGray8,
        uint32_t stride = 0)
      : camera_id_{camera_id},
        session_id_{session_id},
        sequence_number_{sequence_number},
        capture_time_{capture_time},
        raw_timestamp_ns_{capture_time.nanos()},
        width_{width},
        height_{height},
        format_{format} {
    size_t bpp = BytesPerPixel(format);
    stride_ = (stride > 0) ? stride : width * static_cast<uint32_t>(bpp);
    size_t bytes = static_cast<size_t>(stride_) * height_;
    owned_storage_.resize(bytes > 0 ? (bytes + 4096) : 0);
  }

  // Constructs a Frame moving an owned buffer.
  Frame(uint32_t camera_id, uint64_t session_id, uint64_t sequence_number,
        talos::event::MonotonicTime capture_time, uint32_t width,
        uint32_t height, uint32_t stride, PixelFormat format,
        std::vector<uint8_t> buffer)
      : camera_id_{camera_id},
        session_id_{session_id},
        sequence_number_{sequence_number},
        capture_time_{capture_time},
        raw_timestamp_ns_{capture_time.nanos()},
        width_{width},
        height_{height},
        stride_{stride},
        format_{format},
        owned_storage_{std::move(buffer)} {}

  // Constructs a Frame sharing buffer storage.
  Frame(uint32_t camera_id, uint64_t session_id, uint64_t sequence_number,
        talos::event::MonotonicTime capture_time, uint32_t width,
        uint32_t height, uint32_t stride, PixelFormat format,
        std::shared_ptr<const std::vector<uint8_t>> shared_buffer)
      : camera_id_{camera_id},
        session_id_{session_id},
        sequence_number_{sequence_number},
        capture_time_{capture_time},
        raw_timestamp_ns_{capture_time.nanos()},
        width_{width},
        height_{height},
        stride_{stride},
        format_{format},
        shared_storage_{std::move(shared_buffer)} {}

  // Constructs a Frame borrowing `borrowed_size` bytes it does not own, such
  // as a capture driver's buffer for the duration of a FrameCallback. Copies
  // borrow the same bytes, so a consumer that keeps the image copies them
  // (FrameMailbox::Publish, Clone); Share() and mutable_data() copy first.
  Frame(uint32_t camera_id, uint64_t session_id, uint64_t sequence_number,
        talos::event::MonotonicTime capture_time, uint32_t width,
        uint32_t height, uint32_t stride, PixelFormat format,
        const uint8_t* borrowed, size_t borrowed_size)
      : camera_id_{camera_id},
        session_id_{session_id},
        sequence_number_{sequence_number},
        capture_time_{capture_time},
        raw_timestamp_ns_{capture_time.nanos()},
        width_{width},
        height_{height},
        stride_{stride},
        format_{format},
        borrowed_{borrowed},
        borrowed_size_{borrowed != nullptr ? borrowed_size : 0} {}

  // Accessors
  uint32_t camera_id() const { return camera_id_; }
  void set_camera_id(uint32_t id) { camera_id_ = id; }

  uint64_t session_id() const { return session_id_; }
  void set_session_id(uint64_t session) { session_id_ = session; }

  uint64_t sequence_number() const { return sequence_number_; }
  void set_sequence_number(uint64_t seq) { sequence_number_ = seq; }

  // Host-mapped source instant, interpreted by timestamp_kind().
  // CaptureMidpoint converts it at the consumer's publication boundary; it is
  // not necessarily an exposure midpoint just because it is in the host clock
  // domain.
  talos::event::MonotonicTime capture_time() const { return capture_time_; }
  void set_capture_time(talos::event::MonotonicTime time) {
    capture_time_ = time;
  }

  int64_t capture_timestamp_ns() const { return capture_time_.nanos(); }

  int64_t raw_timestamp_ns() const { return raw_timestamp_ns_; }
  void set_raw_timestamp_ns(int64_t raw_ns) { raw_timestamp_ns_ = raw_ns; }

  ClockDomain clock_domain() const { return clock_domain_; }
  void set_clock_domain(ClockDomain domain) { clock_domain_ = domain; }

  TimestampKind timestamp_kind() const { return timestamp_kind_; }
  void set_timestamp_kind(TimestampKind kind) { timestamp_kind_ = kind; }
  bool is_exposure_timestamp() const {
    return timestamp_kind_ == TimestampKind::kExposure;
  }

  TimestampMappingQuality mapping_quality() const { return mapping_quality_; }
  void set_mapping_quality(TimestampMappingQuality quality) {
    mapping_quality_ = quality;
  }

  uint32_t width() const { return width_; }
  void set_width(uint32_t width) { width_ = width; }

  uint32_t height() const { return height_; }
  void set_height(uint32_t height) { height_ = height; }

  uint32_t stride() const { return stride_; }
  void set_stride(uint32_t stride) { stride_ = stride; }

  PixelFormat format() const { return format_; }
  void set_format(PixelFormat format) { format_ = format; }

  size_t bytes_per_pixel() const { return BytesPerPixel(format_); }

  // Buffer accessors
  const uint8_t* data() const {
    if (borrowed_ != nullptr) return borrowed_;
    const uint8_t* ptr =
        shared_storage_ ? shared_storage_->data() : owned_storage_.data();
    if (ptr == nullptr) return nullptr;
    size_t required = static_cast<size_t>(stride_) * height_;
    size_t cap =
        shared_storage_ ? shared_storage_->size() : owned_storage_.size();
    if (cap >= required + 4096) {
      uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
      size_t offset = (4096 - (addr % 4096)) % 4096;
      return ptr + offset;
    }
    return ptr;
  }

  uint8_t* mutable_data() {
    if (borrowed_ != nullptr) {
      // Copy out of borrowed storage before mutating.
      owned_storage_.assign(borrowed_, borrowed_ + borrowed_size_);
      borrowed_ = nullptr;
      borrowed_size_ = 0;
    } else if (shared_storage_) {
      // Unshare to mutate (copy-on-write semantics)
      owned_storage_ = *shared_storage_;
      shared_storage_.reset();
    }
    uint8_t* ptr = owned_storage_.data();
    if (ptr == nullptr) return nullptr;
    size_t required = static_cast<size_t>(stride_) * height_;
    if (owned_storage_.size() >= required + 4096) {
      uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
      size_t offset = (4096 - (addr % 4096)) % 4096;
      return ptr + offset;
    }
    return ptr;
  }

  // Declared stride * height bytes, however many the storage holds (the
  // storage size only while stride or height is zero); see holds_image().
  size_t size() const {
    if (stride_ > 0 && height_ > 0) {
      return static_cast<size_t>(stride_) * height_;
    }
    return stored_size();
  }

  bool empty() const { return size() == 0; }

  // True when the declared layout is a non-empty image the backing storage
  // holds: nonzero width and height, a stride covering one row, and at least
  // stride * height stored bytes. Read pixels only from such a frame.
  bool holds_image() const {
    if (width_ == 0 || height_ == 0 ||
        stride_ < static_cast<size_t>(width_) * bytes_per_pixel()) {
      return false;
    }
    return stored_size() >= size();
  }

  // Returns true if buffer storage is shared with other handles.
  bool is_shared() const { return shared_storage_ != nullptr; }

  // Returns true if the bytes belong to someone else (see the borrowing
  // constructor).
  bool is_borrowed() const { return borrowed_ != nullptr; }

  // Clones the frame, guaranteeing an owned, independent buffer.
  Frame Clone() const {
    Frame cloned;
    cloned.camera_id_ = camera_id_;
    cloned.session_id_ = session_id_;
    cloned.sequence_number_ = sequence_number_;
    cloned.capture_time_ = capture_time_;
    cloned.raw_timestamp_ns_ = raw_timestamp_ns_;
    cloned.clock_domain_ = clock_domain_;
    cloned.timestamp_kind_ = timestamp_kind_;
    cloned.mapping_quality_ = mapping_quality_;
    cloned.width_ = width_;
    cloned.height_ = height_;
    cloned.stride_ = stride_;
    cloned.format_ = format_;
    size_t bytes = static_cast<size_t>(stride_) * height_;
    cloned.owned_storage_.resize(bytes > 0 ? (bytes + 4096) : 0);
    // A borrowed frame copies only the bytes it was lent.
    const size_t copied =
        borrowed_ != nullptr && borrowed_size_ < bytes ? borrowed_size_ : bytes;
    if (copied > 0 && data() != nullptr) {
      std::memcpy(cloned.mutable_data(), data(), copied);
    }
    return cloned;
  }

  // Shares the buffer with a new Frame instance without copying data; a
  // borrowed frame is cloned instead, since its bytes may not outlive it.
  Frame Share() const {
    if (borrowed_ != nullptr) return Clone();
    if (!shared_storage_ && !owned_storage_.empty()) {
      shared_storage_ = std::make_shared<const std::vector<uint8_t>>(
          std::move(const_cast<std::vector<uint8_t>&>(owned_storage_)));
    }
    Frame shared;
    shared.camera_id_ = camera_id_;
    shared.session_id_ = session_id_;
    shared.sequence_number_ = sequence_number_;
    shared.capture_time_ = capture_time_;
    shared.raw_timestamp_ns_ = raw_timestamp_ns_;
    shared.clock_domain_ = clock_domain_;
    shared.timestamp_kind_ = timestamp_kind_;
    shared.mapping_quality_ = mapping_quality_;
    shared.width_ = width_;
    shared.height_ = height_;
    shared.stride_ = stride_;
    shared.format_ = format_;
    shared.shared_storage_ = shared_storage_;
    return shared;
  }

 private:
  size_t stored_size() const {
    if (borrowed_ != nullptr) return borrowed_size_;
    return shared_storage_ ? shared_storage_->size() : owned_storage_.size();
  }

  uint32_t camera_id_{0};
  uint64_t session_id_{0};
  uint64_t sequence_number_{0};
  talos::event::MonotonicTime capture_time_{
      talos::event::MonotonicTime::from_nanos(0)};
  int64_t raw_timestamp_ns_{0};
  ClockDomain clock_domain_{ClockDomain::kHostMonotonic};
  TimestampKind timestamp_kind_{TimestampKind::kExposure};
  TimestampMappingQuality mapping_quality_{TimestampMappingQuality::kExact};

  uint32_t width_{0};
  uint32_t height_{0};
  uint32_t stride_{0};
  PixelFormat format_{PixelFormat::kGray8};

  std::vector<uint8_t> owned_storage_;
  mutable std::shared_ptr<const std::vector<uint8_t>> shared_storage_;
  const uint8_t* borrowed_{nullptr};
  size_t borrowed_size_{0};
};

}  // namespace talos::vision

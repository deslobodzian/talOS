#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
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

// The capture formats' sizes are what the sources' stride checks rest on.
static_assert(BytesPerPixel(PixelFormat::kGray8) == 1);
static_assert(BytesPerPixel(PixelFormat::kYuyv) == 2);
static_assert(BytesPerPixel(PixelFormat::kBgra8) == 4);

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
  ~Frame() = default;

  Frame(const Frame& other)
      : borrowed_{other.borrowed_}, borrowed_size_{other.borrowed_size_} {
    CopyMetadataFrom(other);
    if (other.borrowed_ != nullptr) return;
    if (other.shared_storage_) {
      shared_storage_ = other.shared_storage_;
      data_offset_ = other.data_offset_;
    } else if (other.owned_storage_ &&
               other.was_shared_.load(std::memory_order_acquire)) {
      shared_storage_ = other.owned_storage_;
      data_offset_ = other.data_offset_;
    } else if (other.owned_storage_ &&
               other.owned_storage_->size() > other.data_offset_) {
      CopyIntoAlignedOwned(other.owned_storage_->data() + other.data_offset_,
                           other.owned_storage_->size() - other.data_offset_);
    }
  }

  Frame& operator=(const Frame& other) {
    if (this == &other) return *this;
    Frame copy(other);
    *this = std::move(copy);
    return *this;
  }

  Frame(Frame&& other) noexcept
      : owned_storage_{std::move(other.owned_storage_)},
        shared_storage_{std::move(other.shared_storage_)},
        was_shared_{
            other.was_shared_.exchange(false, std::memory_order_relaxed)},
        data_offset_{std::exchange(other.data_offset_, 0)},
        borrowed_{std::exchange(other.borrowed_, nullptr)},
        borrowed_size_{std::exchange(other.borrowed_size_, 0)} {
    CopyMetadataFrom(other);
  }

  Frame& operator=(Frame&& other) noexcept {
    if (this == &other) return *this;
    CopyMetadataFrom(other);
    owned_storage_ = std::move(other.owned_storage_);
    shared_storage_ = std::move(other.shared_storage_);
    was_shared_.store(
        other.was_shared_.exchange(false, std::memory_order_relaxed),
        std::memory_order_relaxed);
    data_offset_ = std::exchange(other.data_offset_, 0);
    borrowed_ = std::exchange(other.borrowed_, nullptr);
    borrowed_size_ = std::exchange(other.borrowed_size_, 0);
    return *this;
  }

  // Constructs a Frame owning an allocated buffer for given dimensions.
  Frame(uint32_t camera_id, uint64_t session_id, uint64_t sequence_number,
        talos::event::MonotonicTime capture_time, uint32_t width,
        uint32_t height, PixelFormat format = PixelFormat::kGray8,
        uint32_t stride = 0)
      : Frame(NoStorage{}, camera_id, session_id, sequence_number, capture_time,
              width, height, ResolveStride(width, format, stride), format) {
    AllocateAlignedOwned(static_cast<size_t>(stride_) * height_);
  }

  // Constructs a Frame moving an owned buffer.
  Frame(uint32_t camera_id, uint64_t session_id, uint64_t sequence_number,
        talos::event::MonotonicTime capture_time, uint32_t width,
        uint32_t height, uint32_t stride, PixelFormat format,
        std::vector<uint8_t> buffer)
      : Frame(NoStorage{}, camera_id, session_id, sequence_number, capture_time,
              width, height, stride, format) {
    owned_storage_ = std::make_shared<std::vector<uint8_t>>(std::move(buffer));
  }

  // Constructs a Frame sharing buffer storage.
  Frame(uint32_t camera_id, uint64_t session_id, uint64_t sequence_number,
        talos::event::MonotonicTime capture_time, uint32_t width,
        uint32_t height, uint32_t stride, PixelFormat format,
        std::shared_ptr<const std::vector<uint8_t>> shared_buffer)
      : Frame(NoStorage{}, camera_id, session_id, sequence_number, capture_time,
              width, height, stride, format) {
    shared_storage_ = std::move(shared_buffer);
  }

  // Constructs a Frame borrowing `borrowed_size` bytes it does not own, such
  // as a capture driver's buffer for the duration of a FrameCallback. Copies
  // borrow the same bytes, so a consumer that keeps the image copies them
  // (FrameMailbox::Publish, Clone); Share() and mutable_data() copy first.
  Frame(uint32_t camera_id, uint64_t session_id, uint64_t sequence_number,
        talos::event::MonotonicTime capture_time, uint32_t width,
        uint32_t height, uint32_t stride, PixelFormat format,
        const uint8_t* borrowed, size_t borrowed_size)
      : Frame(NoStorage{}, camera_id, session_id, sequence_number, capture_time,
              width, height, stride, format) {
    borrowed_ = borrowed;
    borrowed_size_ = borrowed != nullptr ? borrowed_size : 0;
  }

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
    const std::vector<uint8_t>* storage =
        shared_storage_ ? shared_storage_.get() : owned_storage_.get();
    if (storage == nullptr || storage->size() <= data_offset_) return nullptr;
    return storage->data() + data_offset_;
  }

  uint8_t* mutable_data() {
    if (borrowed_ != nullptr) {
      // Copy out of borrowed storage before mutating.
      const uint8_t* src = borrowed_;
      const size_t n = borrowed_size_;
      borrowed_ = nullptr;
      borrowed_size_ = 0;
      CopyIntoAlignedOwned(src, n);
    } else if (shared_storage_) {
      // Unshare to mutate (copy-on-write semantics), preserving exact logical
      // pixel bytes at offset 0 of a freshly page-aligned view.
      const auto prev = std::move(shared_storage_);
      const size_t prev_offset = data_offset_;
      const size_t prev_size =
          prev->size() > prev_offset ? (prev->size() - prev_offset) : 0;
      if (prev_size > 0) {
        CopyIntoAlignedOwned(prev->data() + prev_offset, prev_size);
      } else {
        AllocateAlignedOwned(0);
      }
    } else if (owned_storage_ && was_shared_.load(std::memory_order_acquire)) {
      // A previous Share() handed out a shared_ptr to owned_storage_; detach
      // via copy-on-write so those immutable readers never race with writes.
      const auto prev = owned_storage_;
      owned_storage_.reset();
      const size_t prev_offset = data_offset_;
      const size_t prev_size =
          prev->size() > prev_offset ? (prev->size() - prev_offset) : 0;
      was_shared_.store(false, std::memory_order_relaxed);
      if (prev_size > 0) {
        CopyIntoAlignedOwned(prev->data() + prev_offset, prev_size);
      } else {
        AllocateAlignedOwned(0);
      }
    }
    if (!owned_storage_ || owned_storage_->size() <= data_offset_) {
      return nullptr;
    }
    return owned_storage_->data() + data_offset_;
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
  bool is_shared() const {
    return shared_storage_ != nullptr ||
           was_shared_.load(std::memory_order_acquire);
  }

  // Returns true if the bytes belong to someone else (see the borrowing
  // constructor).
  bool is_borrowed() const { return borrowed_ != nullptr; }

  // Clones the frame, guaranteeing an owned, independent buffer.
  Frame Clone() const {
    Frame cloned;
    cloned.CopyMetadataFrom(*this);
    const size_t bytes = static_cast<size_t>(stride_) * height_;
    // Never read past the bytes actually stored in the source frame.
    const size_t copied = std::min(bytes, stored_size());
    if (copied > 0 && data() != nullptr) {
      cloned.CopyIntoAlignedOwned(data(), copied);
    }
    return cloned;
  }

  // Shares the buffer with a new Frame instance without copying data; a
  // borrowed frame is cloned instead, since its bytes may not outlive it.
  // Safe for concurrent calls on a const Frame: only increments the immutable
  // shared_ptr reference count and sets the atomic COW flag.
  Frame Share() const {
    if (borrowed_ != nullptr) return Clone();
    Frame shared;
    shared.CopyMetadataFrom(*this);
    if (shared_storage_) {
      shared.shared_storage_ = shared_storage_;
      shared.data_offset_ = data_offset_;
    } else if (owned_storage_ && owned_storage_->size() > data_offset_) {
      was_shared_.store(true, std::memory_order_release);
      shared.shared_storage_ = owned_storage_;
      shared.data_offset_ = data_offset_;
    }
    return shared;
  }

 private:
  static constexpr size_t kPageAlignment = 4096;

  static constexpr uint32_t ResolveStride(uint32_t width, PixelFormat format,
                                          uint32_t stride) {
    if (stride > 0) return stride;
    const size_t row_bytes = static_cast<size_t>(width) * BytesPerPixel(format);
    if (row_bytes > std::numeric_limits<uint32_t>::max()) {
      return 0;
    }
    return static_cast<uint32_t>(row_bytes);
  }

  // Selects the constructor that sets the metadata and dimensions only; the
  // public constructors delegate to it and then attach their storage.
  struct NoStorage {};

  Frame(NoStorage /*tag*/, uint32_t camera_id, uint64_t session_id,
        uint64_t sequence_number, talos::event::MonotonicTime capture_time,
        uint32_t width, uint32_t height, uint32_t stride, PixelFormat format)
      : camera_id_{camera_id},
        session_id_{session_id},
        sequence_number_{sequence_number},
        capture_time_{capture_time},
        raw_timestamp_ns_{capture_time.nanos()},
        width_{width},
        height_{height},
        stride_{stride},
        format_{format} {}

  // Copies every field but the pixel storage (owned, shared or borrowed).
  void CopyMetadataFrom(const Frame& other) {
    camera_id_ = other.camera_id_;
    session_id_ = other.session_id_;
    sequence_number_ = other.sequence_number_;
    capture_time_ = other.capture_time_;
    raw_timestamp_ns_ = other.raw_timestamp_ns_;
    clock_domain_ = other.clock_domain_;
    timestamp_kind_ = other.timestamp_kind_;
    mapping_quality_ = other.mapping_quality_;
    width_ = other.width_;
    height_ = other.height_;
    stride_ = other.stride_;
    format_ = other.format_;
  }

  static size_t AlignmentOffset(const uint8_t* ptr) {
    const uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
    return (kPageAlignment - (addr % kPageAlignment)) % kPageAlignment;
  }

  void AllocateAlignedOwned(size_t bytes) {
    was_shared_.store(false, std::memory_order_relaxed);
    if (bytes == 0) {
      owned_storage_.reset();
      data_offset_ = 0;
      return;
    }
    auto storage =
        std::make_shared<std::vector<uint8_t>>(bytes + kPageAlignment);
    data_offset_ = AlignmentOffset(storage->data());
    // Shrinking within capacity never reallocates, keeping data() page-aligned
    // while making (storage->size() - data_offset_) == bytes exact.
    storage->resize(data_offset_ + bytes);
    owned_storage_ = std::move(storage);
  }

  void CopyIntoAlignedOwned(const uint8_t* src, size_t bytes) {
    AllocateAlignedOwned(bytes);
    if (bytes > 0 && src != nullptr && owned_storage_) {
      std::memcpy(owned_storage_->data() + data_offset_, src, bytes);
    }
  }

  size_t stored_size() const {
    if (borrowed_ != nullptr) return borrowed_size_;
    const std::vector<uint8_t>* storage =
        shared_storage_ ? shared_storage_.get() : owned_storage_.get();
    return (storage != nullptr && storage->size() > data_offset_)
               ? (storage->size() - data_offset_)
               : 0;
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

  std::shared_ptr<std::vector<uint8_t>> owned_storage_;
  std::shared_ptr<const std::vector<uint8_t>> shared_storage_;
  mutable std::atomic<bool> was_shared_{false};
  size_t data_offset_{0};
  const uint8_t* borrowed_{nullptr};
  size_t borrowed_size_{0};
};

}  // namespace talos::vision

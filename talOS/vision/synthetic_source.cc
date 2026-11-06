#include "talOS/vision/synthetic_source.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <format>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

namespace talos::vision {

namespace {

// Tag36h11 codebook for first 10 IDs (0..9)
constexpr uint64_t kTag36h11Codes[10] = {
    0x0000000d7e00984bULL,  // ID 0
    0x0000000dda664ca7ULL,  // ID 1
    0x0000000dc4a1c821ULL,  // ID 2
    0x0000000e17b470e9ULL,  // ID 3
    0x0000000ef91d01b1ULL,  // ID 4
    0x0000000f429cdd73ULL,  // ID 5
    0x000000005da29225ULL,  // ID 6
    0x00000001106cba43ULL,  // ID 7
    0x0000000223bed79dULL,  // ID 8
    0x0000000305ab64b5ULL,  // ID 9
};

// Bit position mappings for tag36h11 in border coordinates [0, 7]
constexpr int kTag36h11BitX[36] = {1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6,
                                   6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5,
                                   4, 3, 4, 1, 1, 1, 1, 1, 2, 2, 2, 3};

constexpr int kTag36h11BitY[36] = {1, 1, 1, 1, 1, 2, 2, 2, 3, 1, 2, 3,
                                   4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5,
                                   5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4};

}  // namespace

SyntheticCameraSource::SyntheticCameraSource(uint32_t camera_id,
                                             std::string device_id,
                                             CameraMode default_mode)
    : camera_id_{camera_id},
      device_id_{device_id.empty() ? std::format("synth_cam_{}", camera_id)
                                   : std::move(device_id)},
      name_{std::format("SyntheticCameraSource-{}", camera_id_)},
      mode_{default_mode} {}

SyntheticCameraSource::~SyntheticCameraSource() {
  SyntheticCameraSource::Stop();
}

uint32_t SyntheticCameraSource::camera_id() const { return camera_id_; }

std::string SyntheticCameraSource::device_id() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return device_id_;
}

std::string SyntheticCameraSource::name() const { return name_; }

std::vector<CameraMode> SyntheticCameraSource::SupportedModes() const {
  return {
      {640, 480, 30.0, PixelFormat::kGray8},
      {640, 480, 60.0, PixelFormat::kGray8},
      {1280, 720, 30.0, PixelFormat::kGray8},
      {1280, 720, 60.0, PixelFormat::kGray8},
      {1920, 1080, 30.0, PixelFormat::kGray8},
      {640, 480, 30.0, PixelFormat::kRgb8},
      {1280, 720, 30.0, PixelFormat::kRgb8},
      {640, 480, 30.0, PixelFormat::kBgra8},
  };
}

CameraMode SyntheticCameraSource::CurrentMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

bool SyntheticCameraSource::SetMode(const CameraMode& mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = mode;
  return true;
}

CameraControls SyntheticCameraSource::GetControls() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return controls_;
}

bool SyntheticCameraSource::SetControls(const CameraControls& controls) {
  std::lock_guard<std::mutex> lock(mutex_);
  controls_ = controls;
  return true;
}

void SyntheticCameraSource::SetFrameCallback(FrameCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  frame_callback_ = std::move(callback);
}

void SyntheticCameraSource::SetConnectionCallback(ConnectionCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  connection_callback_ = std::move(callback);
}

bool SyntheticCameraSource::Start() {
  if (is_running_.exchange(true)) {
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_.fps > 0.0) {
      delivery_thread_ = std::thread(&SyntheticCameraSource::WorkerLoop, this);
    }
  }
  return true;
}

void SyntheticCameraSource::Stop() {
  if (!is_running_.exchange(false)) {
    return;
  }

  cv_.notify_all();
  if (delivery_thread_.joinable()) {
    delivery_thread_.join();
  }
}

bool SyntheticCameraSource::IsRunning() const { return is_running_.load(); }

bool SyntheticCameraSource::IsConnected() const { return is_connected_.load(); }

void SyntheticCameraSource::AddTag(const SyntheticTagDescriptor& tag) {
  std::lock_guard<std::mutex> lock(mutex_);
  tags_.push_back(tag);
}

void SyntheticCameraSource::ClearTags() {
  std::lock_guard<std::mutex> lock(mutex_);
  tags_.clear();
}

void SyntheticCameraSource::SetFixtureFrame(const Frame& frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  fixture_frame_ = frame;
}

void SyntheticCameraSource::ClearFixtureFrame() {
  std::lock_guard<std::mutex> lock(mutex_);
  fixture_frame_.reset();
}

uint64_t SyntheticCameraSource::session_id() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return session_id_;
}

uint64_t SyntheticCameraSource::sequence_number() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return sequence_number_;
}

void SyntheticCameraSource::SimulateDisconnect() {
  is_connected_.store(false);
  ConnectionCallback cb;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cb = connection_callback_;
  }
  if (cb) {
    cb(ConnectionState::kDisconnected, "Simulated camera disconnect");
  }
}

void SyntheticCameraSource::SimulateReconnect() {
  ConnectionCallback cb;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    session_id_++;
    sequence_number_ = 0;
    is_connected_.store(true);
    cb = connection_callback_;
  }
  if (cb) {
    cb(ConnectionState::kConnected, "Simulated camera reconnect");
  }
}

Frame SyntheticCameraSource::GenerateFrame() {
  std::lock_guard<std::mutex> lock(mutex_);
  const uint64_t seq = sequence_number_++;
  const talos::event::MonotonicTime now = talos::event::steady_now();

  if (fixture_frame_.has_value()) {
    Frame frame = fixture_frame_->Clone();
    frame.set_camera_id(camera_id_);
    frame.set_session_id(session_id_);
    frame.set_sequence_number(seq);
    frame.set_capture_time(now);
    frame.set_raw_timestamp_ns(now.nanos());
    return frame;
  }

  Frame frame(camera_id_, session_id_, seq, now, mode_.width, mode_.height,
              mode_.format);
  RenderScene(frame);
  return frame;
}

bool SyntheticCameraSource::TriggerFrame() {
  if (!is_connected_.load()) {
    return false;
  }

  FrameCallback cb;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cb = frame_callback_;
  }
  if (!cb) {
    return false;
  }

  Frame frame = GenerateFrame();
  cb(frame);
  return true;
}

void SyntheticCameraSource::WorkerLoop() {
  while (is_running_.load()) {
    double fps = 30.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      fps = mode_.fps;
    }
    if (fps <= 0.0) {
      fps = 30.0;
    }

    auto period = std::chrono::duration<double>(1.0 / fps);
    auto next_time =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::nanoseconds>(period);

    TriggerFrame();

    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_until(lock, next_time, [this]() { return !is_running_.load(); });
  }
}

void SyntheticCameraSource::RenderScene(Frame& frame) {
  const uint32_t width = frame.width();
  const uint32_t height = frame.height();
  const uint32_t stride = frame.stride();
  const PixelFormat format = frame.format();
  uint8_t* buf = frame.mutable_data();

  // Background color: white (255)
  std::memset(buf, 255, frame.size());
  if (buf == nullptr || width == 0 || height == 0) return;

  // Grayscale-equivalent paint colors per pixel format. Every channel
  // carries the same gray value, so Rgb8/Bgr8 share one spelling; Bgra8
  // keeps alpha opaque to match the previous per-pixel writer.
  int cv_type = -1;
  cv::Scalar white;
  cv::Scalar black;
  switch (format) {
    case PixelFormat::kGray8:
      cv_type = CV_8UC1;
      white = cv::Scalar(255);
      black = cv::Scalar(0);
      break;
    case PixelFormat::kRgb8:
    case PixelFormat::kBgr8:
      cv_type = CV_8UC3;
      white = cv::Scalar(255, 255, 255);
      black = cv::Scalar(0, 0, 0);
      break;
    case PixelFormat::kBgra8:
      cv_type = CV_8UC4;
      white = cv::Scalar(255, 255, 255, 255);
      black = cv::Scalar(0, 0, 0, 255);
      break;
    default:
      break;
  }
  if (cv_type < 0) return;

  // Header over the frame buffer: primitives draw in place, no copy.
  // The frame stride becomes the Mat step, so padded rows stay intact.
  cv::Mat img(static_cast<int>(height), static_cast<int>(width), cv_type, buf,
              stride);

  // Rasterize each synthetic tag
  for (const auto& tag : tags_) {
    const double cx = tag.center_x;
    const double cy = tag.center_y;
    const double size = tag.size_pixels;
    if (size <= 0.0) continue;

    // Tag codeword: lookup in table or fallback to deterministic hash
    uint64_t code = 0;
    if (tag.tag_id >= 0 && tag.tag_id < 10) {
      code = kTag36h11Codes[tag.tag_id];
    } else {
      code = kTag36h11Codes[std::abs(tag.tag_id) % 10];
    }

    // Forward map from tag-normalized coords (rx, ry in [0, 1]) to pixels:
    // the exact inverse of the former point-sampled mapping, so each
    // module quad covers the pixels that sampled into it. Rotation keeps
    // its original direction by construction.
    const double cos_t = std::cos(tag.rotation_rad);
    const double sin_t = std::sin(tag.rotation_rad);
    auto to_pixel = [&](double rx, double ry) {
      const double ux = rx - 0.5;
      const double uy = ry - 0.5;
      // fillConvexPoly takes integer points; rounding matches the former
      // point-sampled coverage to the pixel.
      return cv::Point(cvRound(cx + size * (ux * cos_t - uy * sin_t)),
                       cvRound(cy + size * (ux * sin_t + uy * cos_t)));
    };

    // Grid: 8x8 border box (modules 0..7), surrounded by 1 module white
    // margin. Total bounding width = 10 modules. White cells are painted
    // explicitly so overlapping tags occlude exactly as before.
    for (int mod_y = 0; mod_y < 10; ++mod_y) {
      for (int mod_x = 0; mod_x < 10; ++mod_x) {
        cv::Scalar color = white;  // Default white (quiet zone included).
        if (mod_x > 0 && mod_x < 9 && mod_y > 0 && mod_y < 9) {
          // Border frame: mod 1, 8 -> border coordinate 0 or 7 -> black
          const int border_x = mod_x - 1;  // range [0, 7]
          const int border_y = mod_y - 1;  // range [0, 7]

          if (border_x == 0 || border_x == 7 || border_y == 0 ||
              border_y == 7) {
            color = black;  // Black border
          } else {
            // Payload module: search for bit matching (border_x, border_y)
            bool bit_found = false;
            for (int b = 0; b < 36; ++b) {
              if (kTag36h11BitX[b] == border_x &&
                  kTag36h11BitY[b] == border_y) {
                // MSB is bit 0 in apriltag rcode assembly
                const uint64_t bit = (code >> (35 - b)) & 1ULL;
                color = (bit == 1) ? white : black;
                bit_found = true;
                break;
              }
            }
            if (!bit_found) {
              color = black;
            }
          }
        }

        const double rx0 = mod_x / 10.0;
        const double ry0 = mod_y / 10.0;
        const double rx1 = (mod_x + 1) / 10.0;
        const double ry1 = (mod_y + 1) / 10.0;
        const std::vector<cv::Point> poly = {
            to_pixel(rx0, ry0), to_pixel(rx1, ry0), to_pixel(rx1, ry1),
            to_pixel(rx0, ry1)};
        cv::fillConvexPoly(img, poly, color);
      }
    }
  }
}

}  // namespace talos::vision

#include "talOS/vision/recording/recorder.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

namespace talos::vision::recording {
namespace {

constexpr std::uint64_t kNsPerSecond = 1'000'000'000;

// Converts `frame`'s pixels to 8-bit grayscale into `dst` (width*height
// bytes). Reads the caller's frame once, writes the preallocated slot once.
void ToGray(const Frame& frame, std::uint8_t* dst) {
  const std::uint32_t w = frame.width();
  const std::uint32_t h = frame.height();
  const std::uint32_t stride = frame.stride() == 0 ? w : frame.stride();
  const std::uint8_t* src = frame.data();
  if (src == nullptr) return;

  switch (frame.format()) {
    case PixelFormat::kGray8:
      if (stride == w) {
        std::memcpy(dst, src, static_cast<std::size_t>(w) * h);
      } else {
        for (std::uint32_t y = 0; y < h; ++y) {
          std::memcpy(dst + static_cast<std::size_t>(y) * w,
                      src + static_cast<std::size_t>(y) * stride, w);
        }
      }
      break;
    case PixelFormat::kRgb8:
    case PixelFormat::kBgr8: {
      const bool bgr = frame.format() == PixelFormat::kBgr8;
      for (std::uint32_t y = 0; y < h; ++y) {
        const std::uint8_t* row = src + static_cast<std::size_t>(y) * stride;
        std::uint8_t* out = dst + static_cast<std::size_t>(y) * w;
        for (std::uint32_t x = 0; x < w; ++x) {
          const std::uint8_t c0 = row[3 * x], c1 = row[3 * x + 1],
                             c2 = row[3 * x + 2];
          const std::uint8_t r = bgr ? c2 : c0;
          const std::uint8_t b = bgr ? c0 : c2;
          out[x] = static_cast<std::uint8_t>((30 * r + 59 * c1 + 11 * b) / 100);
        }
      }
      break;
    }
    case PixelFormat::kBgra8:
      for (std::uint32_t y = 0; y < h; ++y) {
        const std::uint8_t* row = src + static_cast<std::size_t>(y) * stride;
        std::uint8_t* out = dst + static_cast<std::size_t>(y) * w;
        for (std::uint32_t x = 0; x < w; ++x) {
          const std::uint8_t b = row[4 * x], g = row[4 * x + 1],
                             r = row[4 * x + 2];
          out[x] = static_cast<std::uint8_t>((30 * r + 59 * g + 11 * b) / 100);
        }
      }
      break;
    case PixelFormat::kYuyv:
      for (std::uint32_t y = 0; y < h; ++y) {
        const std::uint8_t* row = src + static_cast<std::size_t>(y) * stride;
        std::uint8_t* out = dst + static_cast<std::size_t>(y) * w;
        for (std::uint32_t x = 0; x < w; ++x) {
          // YUYV: [Y0 U Y1 V] groups; the luminance is the even byte of each
          // two-byte pixel.
          out[x] = row[(x / 2) * 4 + (x % 2) * 2];
        }
      }
      break;
  }
}

void WriteFileAtomic(const std::filesystem::path& path, const void* data,
                     std::size_t size) {
  const std::filesystem::path tmp = path.string() + ".tmp";
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out) throw std::runtime_error("cannot create " + tmp.string());
    out.write(static_cast<const char*>(data),
              static_cast<std::streamsize>(size));
    if (!out) throw std::runtime_error("cannot write " + tmp.string());
    out.close();
    if (!out) throw std::runtime_error("cannot close " + tmp.string());
  }
  std::error_code ec;
  std::filesystem::rename(tmp, path, ec);
  if (ec) {
    std::error_code cleanup_ec;
    std::filesystem::remove(tmp, cleanup_ec);
    throw std::runtime_error("cannot rename " + tmp.string() + " -> " +
                             path.string() + ": " + ec.message());
  }
}

}  // namespace

struct FrameRecorder::Impl {
  struct Slot {
    std::vector<std::uint8_t> gray;
    std::vector<DetectionRecord> detections;
    std::uint64_t capture_ns = 0;
    std::string reason;
    ObservationRecord observation;
    std::optional<TruthRecord> truth;
  };

  struct FileEntry {
    std::string stem;
    std::uint64_t bytes = 0;
  };

  Options options;
  std::vector<Slot> slots;
  std::size_t write_index = 0;
  std::size_t read_index = 0;
  std::size_t count = 0;

  std::mutex mutex;
  std::condition_variable cv;
  bool stop = false;
  std::thread writer;

  // Rate-limit state; Record() is called from one thread (the node's live
  // worker or its loop), so these need no synchronization.
  bool sampled_seen = false;
  bool rejected_seen = false;
  std::uint64_t last_sampled_ns = 0;
  std::uint64_t last_rejected_ns = 0;

  std::atomic<std::uint64_t> recorded_count{0};
  std::atomic<std::uint64_t> dropped_count{0};

  // max_bytes accounting: what this camera has on disk, oldest first.
  std::deque<FileEntry> files;
  std::uint64_t total_bytes = 0;

  explicit Impl(Options o) : options{std::move(o)} {
    const int width = options.context.width;
    const int height = options.context.height;
    if (width <= 0 || height <= 0) {
      throw std::invalid_argument("recorder: positive width and height");
    }
    if (options.context.camera.empty()) {
      throw std::invalid_argument("recorder: camera name required");
    }
    if (!std::isfinite(options.sample_hz) || options.sample_hz < 0 ||
        !std::isfinite(options.rejected_hz) || options.rejected_hz < 0) {
      throw std::invalid_argument(
          "recorder: sample_hz and rejected_hz must be finite and >= 0");
    }
    if (options.queue_slots == 0) {
      throw std::invalid_argument("recorder: queue_slots must be positive");
    }
    if (options.output_dir.empty()) {
      throw std::invalid_argument("recorder: output_dir required");
    }
    const std::size_t pixels =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    slots.resize(options.queue_slots);
    for (auto& slot : slots) {
      slot.gray.resize(pixels);
      slot.detections.reserve(kMaxDetections);
    }
    std::filesystem::create_directories(options.output_dir);
    ScanExisting();
    writer = std::thread([this] { WriterLoop(); });
  }

  static constexpr std::size_t kMaxDetections = 256;

  void ScanExisting() {
    // A JSON's presence means its PNG is complete (atomic pair writes).
    std::vector<std::pair<std::string, std::uint64_t>> found;
    std::error_code ec;
    for (const auto& entry :
         std::filesystem::directory_iterator(options.output_dir, ec)) {
      if (ec) break;
      if (!entry.is_regular_file()) continue;
      const std::string name = entry.path().filename().string();
      if (name.size() < 5 || name.substr(name.size() - 5) != ".json") continue;
      const std::string stem = name.substr(0, name.size() - 5);
      std::error_code ec2;
      const std::uint64_t png =
          std::filesystem::file_size(options.output_dir / (stem + ".png"), ec2);
      if (ec2) continue;
      const std::uint64_t json = std::filesystem::file_size(entry.path(), ec2);
      if (ec2) continue;
      found.emplace_back(stem, png + json);
    }
    std::sort(found.begin(), found.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    for (auto& [stem, bytes] : found) {
      files.push_back(FileEntry{std::move(stem), bytes});
      total_bytes += bytes;
    }
    TrimToBudget();
  }

  void TrimToBudget() {
    std::error_code ec;
    while (total_bytes > options.max_bytes && !files.empty()) {
      const FileEntry& oldest = files.front();
      std::filesystem::remove(options.output_dir / (oldest.stem + ".png"), ec);
      std::filesystem::remove(options.output_dir / (oldest.stem + ".json"), ec);
      total_bytes -= oldest.bytes;
      files.pop_front();
    }
  }

  bool RateLimited(const FrameRecord& record, std::uint64_t* interval_ns) {
    const bool rejected = record.reason == "rejected";
    const double rate = rejected ? options.rejected_hz : options.sample_hz;
    if (rate <= 0.0) return true;  // zero rate records nothing
    *interval_ns =
        static_cast<std::uint64_t>(std::llround(kNsPerSecond / rate));
    bool& seen = rejected ? rejected_seen : sampled_seen;
    std::uint64_t& last = rejected ? last_rejected_ns : last_sampled_ns;
    if (seen && record.capture_ns - last < *interval_ns) return true;
    last = record.capture_ns;
    seen = true;
    return false;
  }

  void WriterLoop() {
    std::vector<std::uint8_t> gray;
    gray.resize(static_cast<std::size_t>(options.context.width) *
                options.context.height);
    std::vector<DetectionRecord> detections;
    detections.reserve(kMaxDetections);
    for (;;) {
      Slot* slot = nullptr;
      {
        std::unique_lock lock(mutex);
        cv.wait(lock, [this] { return stop || count > 0; });
        if (count == 0) return;  // stopping with nothing queued
        slot = &slots[read_index];
        read_index = (read_index + 1) % slots.size();
        --count;
        // Copy out under the lock so the slot is free for Record() before
        // any disk I/O starts.
        gray = slot->gray;
        detections.assign(slot->detections.begin(), slot->detections.end());
        pending_capture_ns = slot->capture_ns;
        pending_reason = slot->reason;
        pending_observation = slot->observation;
        pending_truth = slot->truth;
      }
      try {
        WritePair(gray, detections, pending_capture_ns, pending_reason,
                  pending_observation, pending_truth);
      } catch (const std::exception& error) {
        StopAfterWriteFailure(error.what());
        return;
      } catch (...) {
        StopAfterWriteFailure("unknown writer exception");
        return;
      }
      {
        std::lock_guard lock(mutex);
        if (stop) {
          // Shutdown: finish at most the pair in progress, drop the rest, so
          // destruction is bounded by one encode+write and never blocks on a
          // slow disk draining the whole queue.
          dropped_count += count;
          count = 0;
          return;
        }
      }
    }
  }

  void StopAfterWriteFailure(const char* reason) {
    const std::string stem = RecordStem(pending_capture_ns, pending_reason);
    std::error_code ec;
    std::filesystem::remove(options.output_dir / (stem + ".png.tmp"), ec);
    std::filesystem::remove(options.output_dir / (stem + ".json.tmp"), ec);
    {
      std::lock_guard lock(mutex);
      stop = true;
      dropped_count += count + 1;  // failed pair and everything still queued
      count = 0;
    }
    std::fprintf(stderr, "%s: recording stopped: %s\n",
                 options.context.camera.c_str(), reason);
  }

  // Writer-only scratch for the pair it is about to encode.
  std::uint64_t pending_capture_ns = 0;
  std::string pending_reason;
  ObservationRecord pending_observation;
  std::optional<TruthRecord> pending_truth;

  void WritePair(const std::vector<std::uint8_t>& gray,
                 const std::vector<DetectionRecord>& detections,
                 std::uint64_t capture_ns, const std::string& reason,
                 const ObservationRecord& observation,
                 const std::optional<TruthRecord>& truth) {
    FrameRecord record;
    record.capture_ns = capture_ns;
    record.reason = reason;
    record.detections = detections;
    record.observation = observation;
    record.truth = truth;

    const std::string stem = RecordStem(capture_ns, reason);
    if (options.test_write_delay.count() > 0) {
      std::this_thread::sleep_for(options.test_write_delay);
    }
    cv::Mat image(options.context.height, options.context.width, CV_8UC1,
                  const_cast<std::uint8_t*>(gray.data()));
    std::vector<std::uint8_t> png;
    if (!cv::imencode(".png", image, png)) {
      throw std::runtime_error("recorder: PNG encode failed for " + stem);
    }
    const std::string json = SerializeRecord(options.context, record);

    const std::filesystem::path png_path = options.output_dir / (stem + ".png");
    const std::filesystem::path json_path =
        options.output_dir / (stem + ".json");
    WriteFileAtomic(png_path, png.data(), png.size());
    WriteFileAtomic(json_path, json.data(), json.size());

    // files/total_bytes are writer-owned (Record() never touches them), so no
    // lock is needed; trimming does disk I/O and must not hold the queue lock.
    files.push_back(FileEntry{stem, png.size() + json.size()});
    total_bytes += png.size() + json.size();
    TrimToBudget();
  }
};

FrameRecorder::FrameRecorder(Options options)
    : impl_{std::make_unique<Impl>(std::move(options))} {}

FrameRecorder::~FrameRecorder() {
  {
    std::lock_guard lock(impl_->mutex);
    impl_->stop = true;
  }
  impl_->cv.notify_one();
  if (impl_->writer.joinable()) impl_->writer.join();
}

bool FrameRecorder::Record(const Frame& frame, const FrameRecord& record) {
  {
    std::lock_guard lock(impl_->mutex);
    if (impl_->stop || frame.data() == nullptr ||
        frame.width() !=
            static_cast<std::uint32_t>(impl_->options.context.width) ||
        frame.height() !=
            static_cast<std::uint32_t>(impl_->options.context.height)) {
      ++impl_->dropped_count;
      return false;
    }
  }
  std::uint64_t interval_ns = 0;
  if (impl_->RateLimited(record, &interval_ns)) return false;

  std::size_t slot_index = 0;
  {
    std::lock_guard lock(impl_->mutex);
    if (impl_->stop) {
      ++impl_->dropped_count;
      return false;
    }
    if (impl_->count == impl_->slots.size()) {
      ++impl_->dropped_count;
      return false;
    }
    slot_index = impl_->write_index;
  }

  Impl::Slot& slot = impl_->slots[slot_index];
  ToGray(frame, slot.gray.data());
  slot.capture_ns = record.capture_ns;
  slot.reason = record.reason;
  slot.detections.assign(record.detections.begin(), record.detections.end());
  slot.observation = record.observation;
  slot.truth = record.truth;

  {
    std::lock_guard lock(impl_->mutex);
    if (impl_->stop) {
      ++impl_->dropped_count;
      return false;
    }
    impl_->write_index = (impl_->write_index + 1) % impl_->slots.size();
    ++impl_->count;
    ++impl_->recorded_count;
  }
  impl_->cv.notify_one();
  return true;
}

std::uint64_t FrameRecorder::recorded() const { return impl_->recorded_count; }
std::uint64_t FrameRecorder::dropped() const { return impl_->dropped_count; }

}  // namespace talos::vision::recording

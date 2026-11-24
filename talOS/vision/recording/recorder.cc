#include "talOS/vision/recording/recorder.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "talOS/process/process.h"

namespace talos::vision::recording {
namespace {

constexpr std::uint64_t kNsPerSecond = 1'000'000'000;
constexpr std::size_t kMaxReasonLength = 64;

bool IsSupportedPixelFormat(PixelFormat format) {
  switch (format) {
    case PixelFormat::kGray8:
    case PixelFormat::kRgb8:
    case PixelFormat::kBgr8:
    case PixelFormat::kYuyv:
    case PixelFormat::kBgra8:
      return true;
  }
  return false;
}

class ScopedFd {
 public:
  explicit ScopedFd(int fd = -1) noexcept : fd_(fd) {}
  ~ScopedFd() { Reset(); }
  ScopedFd(const ScopedFd&) = delete;
  ScopedFd& operator=(const ScopedFd&) = delete;
  ScopedFd(ScopedFd&& other) noexcept : fd_(std::exchange(other.fd_, -1)) {}
  ScopedFd& operator=(ScopedFd&& other) noexcept {
    if (this != &other) {
      Reset();
      fd_ = std::exchange(other.fd_, -1);
    }
    return *this;
  }

  int get() const noexcept { return fd_; }
  int Release() noexcept { return std::exchange(fd_, -1); }
  void Reset() noexcept {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

 private:
  int fd_{-1};
};

std::filesystem::path StripTrailingSlashes(const std::filesystem::path& path) {
  std::string s = path.string();
  while (s.size() > 1 && s.back() == '/') {
    s.pop_back();
  }
  return std::filesystem::path(std::move(s));
}

ScopedFd OpenOutputDirNoFollow(const std::filesystem::path& dir) {
  int fd = -1;
  do {
    fd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
  } while (fd < 0 && errno == EINTR);
  if (fd < 0) {
    const int err = errno;
    throw std::runtime_error("cannot open output directory " + dir.string() +
                             ": " + std::strerror(err));
  }
  return ScopedFd(fd);
}

// Converts `frame`'s pixels to 8-bit grayscale into `dst` (width*height
// bytes). Reads the caller's frame once, writes the preallocated slot once.
void ToGray(const Frame& frame, std::uint8_t* dst) {
  const std::uint32_t w = frame.width();
  const std::uint32_t h = frame.height();
  const std::size_t min_stride =
      static_cast<std::size_t>(w) * frame.bytes_per_pixel();
  const std::size_t stride = frame.stride() == 0
                                 ? min_stride
                                 : static_cast<std::size_t>(frame.stride());
  const std::uint8_t* src = frame.data();
  if (src == nullptr || stride < min_stride) {
    return;
  }

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
          const std::size_t offset = 3 * static_cast<std::size_t>(x);
          const std::uint8_t c0 = row[offset], c1 = row[offset + 1],
                             c2 = row[offset + 2];
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
          const std::size_t offset = 4 * static_cast<std::size_t>(x);
          const std::uint8_t b = row[offset], g = row[offset + 1],
                             r = row[offset + 2];
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
          out[x] = row[static_cast<std::size_t>(x) * 2];
        }
      }
      break;
  }
}

void WriteFileAtomic(const std::filesystem::path& dir,
                     const std::string& filename, const void* data,
                     std::size_t size) {
  const ScopedFd dir_fd = OpenOutputDirNoFollow(dir);
  struct stat target_st {};
  if (::fstatat(dir_fd.get(), filename.c_str(), &target_st,
                AT_SYMLINK_NOFOLLOW) == 0 &&
      S_ISLNK(target_st.st_mode)) {
    throw std::runtime_error("refusing to overwrite symlink " +
                             (dir / filename).string());
  }

  const std::string tmp_name = filename + ".tmp";
  int raw_fd = -1;
  do {
    raw_fd =
        ::openat(dir_fd.get(), tmp_name.c_str(),
                 O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0644);
  } while (raw_fd < 0 && errno == EINTR);
  if (raw_fd < 0) {
    const int err = errno;
    throw std::runtime_error("cannot create " + (dir / tmp_name).string() +
                             ": " + std::strerror(err));
  }
  ScopedFd fd(raw_fd);

  const auto* bytes = static_cast<const std::uint8_t*>(data);
  std::size_t written_total = 0;
  while (written_total < size) {
    const ssize_t n =
        ::write(fd.get(), bytes + written_total, size - written_total);
    if (n < 0) {
      if (errno == EINTR) continue;
      const int err = errno;
      fd.Reset();
      ::unlinkat(dir_fd.get(), tmp_name.c_str(), 0);
      throw std::runtime_error("cannot write " + (dir / tmp_name).string() +
                               ": " + std::strerror(err));
    }
    if (n == 0) {
      fd.Reset();
      ::unlinkat(dir_fd.get(), tmp_name.c_str(), 0);
      throw std::runtime_error("short write to " + (dir / tmp_name).string());
    }
    written_total += static_cast<std::size_t>(n);
  }

  // Always check close(): filesystems can report ENOSPC/EIO on close.
  // Never retry close() on EINTR (Linux closes the fd even on EINTR).
  if (::close(fd.Release()) != 0) {
    const int err = errno;
    ::unlinkat(dir_fd.get(), tmp_name.c_str(), 0);
    throw std::runtime_error("cannot close " + (dir / tmp_name).string() +
                             ": " + std::strerror(err));
  }

  if (::renameat(dir_fd.get(), tmp_name.c_str(), dir_fd.get(),
                 filename.c_str()) != 0) {
    const int err = errno;
    ::unlinkat(dir_fd.get(), tmp_name.c_str(), 0);
    throw std::runtime_error("cannot rename " + (dir / tmp_name).string() +
                             " -> " + (dir / filename).string() + ": " +
                             std::strerror(err));
  }
}

bool IsValidReason(std::string_view reason) {
  if (reason.empty() || reason.size() > kMaxReasonLength || reason == "." ||
      reason.find("..") != std::string_view::npos) {
    return false;
  }
  for (const char c : reason) {
    if (c == '/' || c == '\\' || c == '\0') {
      return false;
    }
  }
  return true;
}

std::uint64_t SaturatingAdd(std::uint64_t a, std::uint64_t b) {
  if (a > std::numeric_limits<std::uint64_t>::max() - b) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return a + b;
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

  // Rate-limit state (guarded by mutex; committed only when a slot is
  // enqueued so queue-full or stop drops do not suppress later frames).
  bool sampled_seen = false;
  bool rejected_seen = false;
  std::uint64_t last_sampled_ns = 0;
  std::uint64_t last_rejected_ns = 0;

  std::atomic<std::uint64_t> recorded_count{0};
  std::atomic<std::uint64_t> dropped_count{0};

  // max_bytes accounting: what this camera has on disk, oldest first.
  std::deque<FileEntry> files;
  std::unordered_set<std::string> active_stems;
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
    const auto w_sz = static_cast<std::size_t>(width);
    const auto h_sz = static_cast<std::size_t>(height);
    if (w_sz > std::numeric_limits<std::size_t>::max() / h_sz) {
      throw std::invalid_argument("recorder: frame dimensions overflow");
    }
    const std::size_t pixels = w_sz * h_sz;
    slots.resize(options.queue_slots);
    for (auto& slot : slots) {
      slot.gray.resize(pixels);
      slot.detections.reserve(kMaxDetections);
    }
    options.output_dir = StripTrailingSlashes(options.output_dir);
    std::error_code ec;
    const auto pre_status =
        std::filesystem::symlink_status(options.output_dir, ec);
    if (!ec && std::filesystem::is_symlink(pre_status)) {
      throw std::filesystem::filesystem_error(
          "recorder: output_dir must not be a symlink", options.output_dir,
          std::make_error_code(std::errc::too_many_symbolic_link_levels));
    }
    std::filesystem::create_directories(options.output_dir);
    // Verify the final directory is a real directory, not a symlink swapped in.
    int check_fd = -1;
    do {
      check_fd = ::open(options.output_dir.c_str(),
                        O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    } while (check_fd < 0 && errno == EINTR);
    if (check_fd < 0) {
      throw std::filesystem::filesystem_error(
          "recorder: cannot open output_dir", options.output_dir,
          std::error_code(errno, std::generic_category()));
    }
    ::close(check_fd);
    ScanExisting();
    writer = std::thread([this] {
      // PNG encoding and disk writes run below every node loop.
      talos::process::ConfigureBackgroundThread();
      WriterLoop();
    });
  }

  static constexpr std::size_t kMaxDetections = 256;

  void ScanExisting() {
    // A JSON's presence means its PNG is complete (atomic pair writes).
    std::vector<std::pair<std::string, std::uint64_t>> found;
    std::error_code ec;
    for (const auto& entry :
         std::filesystem::directory_iterator(options.output_dir, ec)) {
      if (ec) {
        break;
      }
      std::error_code ec2;
      const auto json_status = entry.symlink_status(ec2);
      if (ec2 || std::filesystem::is_symlink(json_status) ||
          !std::filesystem::is_regular_file(json_status)) {
        continue;
      }
      const std::string name = entry.path().filename().string();
      if (name.size() < 5 || name.substr(name.size() - 5) != ".json") {
        continue;
      }
      const std::string stem = name.substr(0, name.size() - 5);
      const std::filesystem::path png_path =
          options.output_dir / (stem + ".png");
      const auto png_status = std::filesystem::symlink_status(png_path, ec2);
      if (ec2 || std::filesystem::is_symlink(png_status) ||
          !std::filesystem::is_regular_file(png_status)) {
        continue;
      }
      const std::uint64_t png = std::filesystem::file_size(png_path, ec2);
      if (ec2) {
        continue;
      }
      const std::uint64_t json = std::filesystem::file_size(entry.path(), ec2);
      if (ec2) {
        continue;
      }
      found.emplace_back(stem, SaturatingAdd(png, json));
    }
    std::sort(found.begin(), found.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    for (auto& [stem, bytes] : found) {
      active_stems.insert(stem);
      files.push_back(FileEntry{std::move(stem), bytes});
      total_bytes = SaturatingAdd(total_bytes, bytes);
    }
    TrimToBudget();
  }

  std::string UniqueStem(const std::string& base) const {
    if (!active_stems.contains(base)) {
      return base;
    }
    for (std::uint64_t suffix = 1;; ++suffix) {
      char buf[32];
      std::snprintf(buf, sizeof buf, "_%04llu",
                    static_cast<unsigned long long>(suffix));
      std::string candidate = base + buf;
      if (!active_stems.contains(candidate)) {
        return candidate;
      }
    }
  }

  void TrimToBudget() {
    if (total_bytes <= options.max_bytes || files.empty()) {
      return;
    }
    int raw_fd = -1;
    do {
      raw_fd = ::open(options.output_dir.c_str(),
                      O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    } while (raw_fd < 0 && errno == EINTR);
    ScopedFd dir_fd(raw_fd);
    while (total_bytes > options.max_bytes && !files.empty()) {
      const FileEntry& oldest = files.front();
      if (dir_fd.get() >= 0) {
        const std::string png_name = oldest.stem + ".png";
        const std::string json_name = oldest.stem + ".json";
        ::unlinkat(dir_fd.get(), png_name.c_str(), 0);
        ::unlinkat(dir_fd.get(), json_name.c_str(), 0);
      }
      total_bytes =
          total_bytes > oldest.bytes ? (total_bytes - oldest.bytes) : 0;
      active_stems.erase(oldest.stem);
      files.pop_front();
    }
  }

  bool IsRateLimited(const FrameRecord& record) const {
    const bool rejected = record.reason == "rejected";
    const double rate = rejected ? options.rejected_hz : options.sample_hz;
    if (!std::isfinite(rate) || rate <= 0.0) {
      return true;  // zero or non-finite rate records nothing
    }
    const double raw_interval = static_cast<double>(kNsPerSecond) / rate;
    constexpr double kMaxSafeInterval =
        static_cast<double>(std::numeric_limits<std::int64_t>::max());
    std::uint64_t interval_ns = 0;
    if (!std::isfinite(raw_interval) || raw_interval >= kMaxSafeInterval) {
      interval_ns = std::numeric_limits<std::uint64_t>::max();
    } else if (raw_interval <= 0.0) {
      interval_ns = 0;
    } else {
      interval_ns = static_cast<std::uint64_t>(std::llround(raw_interval));
    }
    const bool seen = rejected ? rejected_seen : sampled_seen;
    const std::uint64_t last = rejected ? last_rejected_ns : last_sampled_ns;
    return seen &&
           (record.capture_ns < last || record.capture_ns - last < interval_ns);
  }

  void CommitRateLimit(const FrameRecord& record) {
    if (record.reason == "rejected") {
      last_rejected_ns = record.capture_ns;
      rejected_seen = true;
    } else {
      last_sampled_ns = record.capture_ns;
      sampled_seen = true;
    }
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
        if (count == 0) {
          return;  // stopping with nothing queued
        }
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
      if (options.test_before_pair) {
        options.test_before_pair([this] {
          std::lock_guard lock(mutex);
          return stop;
        });
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
    if (!pending_stem.empty() && pending_png_written) {
      int raw_fd = -1;
      do {
        raw_fd = ::open(options.output_dir.c_str(),
                        O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
      } while (raw_fd < 0 && errno == EINTR);
      if (raw_fd >= 0) {
        ScopedFd dir_fd(raw_fd);
        const std::string png_name = pending_stem + ".png";
        ::unlinkat(dir_fd.get(), png_name.c_str(), 0);
      }
    }
    pending_stem.clear();
    pending_png_written = false;
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
  std::string pending_stem;
  bool pending_png_written = false;
  ObservationRecord pending_observation;
  std::optional<TruthRecord> pending_truth;

  void WritePair(const std::vector<std::uint8_t>& gray,
                 const std::vector<DetectionRecord>& detections,
                 std::uint64_t capture_ns, const std::string& reason,
                 const ObservationRecord& observation,
                 const std::optional<TruthRecord>& truth) {
    pending_stem.clear();
    pending_png_written = false;
    FrameRecord record;
    record.capture_ns = capture_ns;
    record.reason = reason;
    record.detections = detections;
    record.observation = observation;
    record.truth = truth;

    pending_stem = UniqueStem(RecordStem(capture_ns, reason));
    const std::string& stem = pending_stem;
    cv::Mat image(options.context.height, options.context.width, CV_8UC1,
                  const_cast<std::uint8_t*>(gray.data()));
    std::vector<std::uint8_t> png;
    if (!cv::imencode(".png", image, png)) {
      throw std::runtime_error("recorder: PNG encode failed for " + stem);
    }
    const std::string json = SerializeRecord(options.context, record);

    const std::string png_name = stem + ".png";
    const std::string json_name = stem + ".json";
    WriteFileAtomic(options.output_dir, png_name, png.data(), png.size());
    pending_png_written = true;
    WriteFileAtomic(options.output_dir, json_name, json.data(), json.size());
    pending_png_written = false;

    // files/total_bytes are writer-owned (Record() never touches them), so no
    // lock is needed; trimming does disk I/O and must not hold the queue lock.
    active_stems.insert(stem);
    const std::uint64_t pair_bytes = SaturatingAdd(png.size(), json.size());
    files.push_back(FileEntry{stem, pair_bytes});
    total_bytes = SaturatingAdd(total_bytes, pair_bytes);
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
  if (impl_->writer.joinable()) {
    impl_->writer.join();
  }
}

bool FrameRecorder::Record(const Frame& frame, const FrameRecord& record) {
  if (!IsValidReason(record.reason) ||
      !IsSupportedPixelFormat(frame.format()) || !frame.holds_image() ||
      frame.data() == nullptr ||
      frame.width() !=
          static_cast<std::uint32_t>(impl_->options.context.width) ||
      frame.height() !=
          static_cast<std::uint32_t>(impl_->options.context.height)) {
    ++impl_->dropped_count;
    return false;
  }

  std::size_t slot_index = 0;
  {
    std::lock_guard lock(impl_->mutex);
    if (impl_->stop) {
      ++impl_->dropped_count;
      return false;
    }
    if (impl_->IsRateLimited(record)) {
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
    impl_->CommitRateLimit(record);
  }
  impl_->cv.notify_one();
  return true;
}

std::uint64_t FrameRecorder::recorded() const { return impl_->recorded_count; }
std::uint64_t FrameRecorder::dropped() const { return impl_->dropped_count; }

}  // namespace talos::vision::recording

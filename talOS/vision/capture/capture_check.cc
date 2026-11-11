// capture_check: opens one live camera through talOS's own capture backend
// (V4L2 on Linux, AVFoundation on macOS) and measures what it delivers: the
// mode the source reports, delivered frame rate, frame interval percentiles,
// missing frames, capture-thread CPU per frame (dequeue and, for MJPG, decode)
// and process CPU; optionally runs the CPU AprilTag backend on the newest
// frame and saves frames as PNG. Bring-up and bench tool (docs/BRINGUP.md
// "Camera capture check"); not a node.
#include <sys/resource.h>
#if defined(__linux__)
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "talOS/events/os/poller.h"
#include "talOS/vision/capture/avfoundation_source.h"
#include "talOS/vision/compute_backend.h"
#include "talOS/vision/frame_mailbox.h"

namespace {
using talos::vision::PixelFormat;

double Percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0;
  std::sort(values.begin(), values.end());
  const auto index = static_cast<size_t>(
      std::lround(fraction * static_cast<double>(values.size() - 1)));
  return values[index];
}

void PrintDistribution(const char* name, const std::vector<double>& values) {
  if (values.empty()) {
    std::printf("%-26s n=0\n", name);
    return;
  }
  std::printf("%-26s n=%zu p50=%.3f p90=%.3f p99=%.3f max=%.3f\n", name,
              values.size(), Percentile(values, 0.5), Percentile(values, 0.9),
              Percentile(values, 0.99),
              *std::max_element(values.begin(), values.end()));
}

double ProcessCpuSeconds() {
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  return static_cast<double>(usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) +
         static_cast<double>(usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) *
             1e-6;
}

int64_t ThreadCpuNanos() {
  timespec now{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now);
  return static_cast<int64_t>(now.tv_sec) * 1000000000 + now.tv_nsec;
}

// The calling thread's scheduling, e.g. "SCHED_OTHER nice 10".
std::string ThreadScheduling() {
#if defined(__linux__)
  const int policy = sched_getscheduler(0);
  sched_param param{};
  sched_getparam(0, &param);
  const auto tid = static_cast<id_t>(syscall(SYS_gettid));
  const int nice = getpriority(PRIO_PROCESS, tid);
  const char* name = policy == SCHED_FIFO    ? "SCHED_FIFO"
                     : policy == SCHED_RR    ? "SCHED_RR"
                     : policy == SCHED_BATCH ? "SCHED_BATCH"
                     : policy == SCHED_IDLE  ? "SCHED_IDLE"
                                             : "SCHED_OTHER";
  return std::string(name) + " priority " +
         std::to_string(param.sched_priority) + " nice " + std::to_string(nice);
#else
  return "unknown";
#endif
}

cv::Mat Gray(const talos::vision::Frame& frame) {
  const auto format = frame.format();
  const int type = format == PixelFormat::kGray8   ? CV_8UC1
                   : format == PixelFormat::kBgra8 ? CV_8UC4
                   : format == PixelFormat::kYuyv  ? CV_8UC2
                                                   : CV_8UC3;
  const cv::Mat image(static_cast<int>(frame.height()),
                      static_cast<int>(frame.width()), type,
                      const_cast<uint8_t*>(frame.data()), frame.stride());
  if (type == CV_8UC1) return image.clone();
  cv::Mat gray;
  cv::cvtColor(image, gray,
               format == PixelFormat::kBgra8  ? cv::COLOR_BGRA2GRAY
               : format == PixelFormat::kYuyv ? cv::COLOR_YUV2GRAY_YUY2
               : format == PixelFormat::kRgb8 ? cv::COLOR_RGB2GRAY
                                              : cv::COLOR_BGR2GRAY);
  return gray;
}

const char* FormatName(PixelFormat format) {
  switch (format) {
    case PixelFormat::kGray8:
      return "gray8";
    case PixelFormat::kRgb8:
      return "rgb8";
    case PixelFormat::kBgr8:
      return "bgr8";
    case PixelFormat::kYuyv:
      return "yuyv";
    case PixelFormat::kBgra8:
      return "bgra8";
  }
  return "?";
}
}  // namespace

int main(int argc, char** argv) {
  try {
    talos::vision::CameraSourceConfig config;
    config.fps = 30;
    double seconds = 20;
    std::string save_directory;
    int save_count = 0;
    bool detect = false, list = false;
    talos::vision::apriltag::DetectorOptions detector;
    detector.quad_decimate = 2.0f;
    detector.threads = 2;
    for (int i = 1; i < argc; ++i) {
      const std::string_view arg = argv[i];
      if (arg == "--help") {
        std::cout << "capture_check --device PATH [--width W] [--height H] "
                     "[--fps F]\n"
                     "  [--seconds S] [--exposure-ms MS] [--gain G] "
                     "[--auto-exposure]\n"
                     "  [--auto-gain] [--save DIR --save-count N] [--detect\n"
                     "  [--decimate D] [--threads N]] [--list]\n";
        return 0;
      }
      if (arg == "--detect") {
        detect = true;
        continue;
      }
      if (arg == "--list") {
        list = true;
        continue;
      }
      if (arg == "--auto-exposure") {
        config.auto_exposure = true;
        continue;
      }
      if (arg == "--auto-gain") {
        config.auto_gain = true;
        continue;
      }
      if (i + 1 == argc) {
        throw std::runtime_error("missing value for " + std::string(arg));
      }
      const std::string value = argv[++i];
      if (arg == "--device") {
        config.device_path = value;
      } else if (arg == "--width") {
        config.width = static_cast<uint32_t>(std::stoul(value));
      } else if (arg == "--height") {
        config.height = static_cast<uint32_t>(std::stoul(value));
      } else if (arg == "--fps") {
        config.fps = std::stod(value);
      } else if (arg == "--seconds") {
        seconds = std::stod(value);
      } else if (arg == "--exposure-ms") {
        config.exposure_ms = std::stod(value);
      } else if (arg == "--gain") {
        config.gain = std::stod(value);
      } else if (arg == "--save") {
        save_directory = value;
      } else if (arg == "--save-count") {
        save_count = std::stoi(value);
      } else if (arg == "--decimate") {
        detector.quad_decimate = std::stof(value);
      } else if (arg == "--threads") {
        detector.threads = std::stoi(value);
      } else {
        throw std::runtime_error("unknown option " + std::string(arg));
      }
    }
    if (config.device_path.empty()) throw std::runtime_error("--device needed");
    auto source = talos::vision::CreatePlatformCameraSource(config);
    if (!source)
      throw std::runtime_error("no capture backend on this platform");
    if (list) {
      for (const auto& mode : source->SupportedModes()) {
        std::printf("%ux%u @ %.6f fps %s\n", mode.width, mode.height, mode.fps,
                    FormatName(mode.format));
      }
      return 0;
    }

    std::mutex mutex;
    std::vector<double> intervals_ms;
    intervals_ms.reserve(static_cast<size_t>(seconds * 300) + 16);
    int64_t last_capture_ns = -1, first_capture_ns = -1, first_thread_cpu = -1,
            last_thread_cpu = -1;
    uint64_t frames = 0, missing = 0;
    std::string capture_scheduling;
    double nominal_ms = 1000.0 / config.fps;
    talos::vision::FrameMailbox mailbox;
    source->SetConnectionCallback(
        [](talos::vision::ConnectionState state, std::string_view message) {
          std::cerr << talos::vision::ConnectionStateToString(state) << ": "
                    << message << '\n';
        });
    source->SetFrameCallback([&](const talos::vision::Frame& frame) {
      const int64_t cpu = ThreadCpuNanos();
      const int64_t capture = frame.capture_time().nanos();
      {
        std::lock_guard lock(mutex);
        if (first_thread_cpu < 0) {
          first_thread_cpu = cpu;
          first_capture_ns = capture;
          capture_scheduling = ThreadScheduling();
        } else {
          const double interval =
              static_cast<double>(capture - last_capture_ns) * 1e-6;
          intervals_ms.push_back(interval);
          const auto periods = std::lround(interval / nominal_ms);
          if (periods > 1) missing += static_cast<uint64_t>(periods - 1);
        }
        last_capture_ns = capture;
        last_thread_cpu = cpu;
        ++frames;
      }
      mailbox.Publish(frame);
    });
    const double cpu_start = ProcessCpuSeconds();
    const auto wall_start = std::chrono::steady_clock::now();
    const int64_t start_ns = talos::event::Poller::now().nanos();
    if (!source->Start()) throw std::runtime_error("capture did not start");
    const auto mode = source->CurrentMode();
    nominal_ms = 1000.0 / mode.fps;
    std::printf(
        "requested %ux%u @ %.3f fps; source reports %ux%u @ %.6f fps %s\n",
        config.width, config.height, config.fps, mode.width, mode.height,
        mode.fps, FormatName(mode.format));
    std::printf("controls_applied=%d\n", source->controls_applied() ? 1 : 0);

    std::unique_ptr<talos::vision::CpuAprilTagBackend> backend;
    if (detect) {
      backend = std::make_unique<talos::vision::CpuAprilTagBackend>(detector);
    }
    std::vector<double> latency_ms, total_us, quad_us, decode_us, gray_us;
    uint64_t processed = 0, detections = 0;
    if (!save_directory.empty()) {
      std::filesystem::create_directories(save_directory);
    }
    int saved = 0;
    auto next_save = wall_start + std::chrono::seconds(1);
    const auto end =
        wall_start +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(seconds));
    while (std::chrono::steady_clock::now() < end && source->IsRunning()) {
      const auto* frame = mailbox.TakeNewest();
      if (frame == nullptr) {
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        continue;
      }
      if (backend) {
        const auto result = backend->Execute(*frame);
        const auto now = talos::event::Poller::now().nanos();
        latency_ms.push_back(
            static_cast<double>(now - frame->capture_time().nanos()) * 1e-6);
        total_us.push_back(static_cast<double>(result.timings.total_us));
        quad_us.push_back(static_cast<double>(result.timings.quad_us));
        decode_us.push_back(static_cast<double>(result.timings.decode_us));
        gray_us.push_back(static_cast<double>(result.timings.grayscale_us));
        ++processed;
        detections += result.detections.size();
      }
      if (saved < save_count && std::chrono::steady_clock::now() >= next_save) {
        const auto path = std::filesystem::path(save_directory) /
                          ("frame_" + std::to_string(saved) + ".png");
        cv::imwrite(path.string(), Gray(*frame));
        ++saved;
        next_save += std::chrono::seconds(1);
      }
    }
    const double wall = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - wall_start)
                            .count();
    const double cpu = ProcessCpuSeconds() - cpu_start;
    source->Stop();

    std::lock_guard lock(mutex);
    std::printf("wall %.2f s, frames %llu, delivered %.3f fps\n", wall,
                static_cast<unsigned long long>(frames),
                static_cast<double>(frames) / wall);
    if (frames > 1) {
      std::printf("first frame %.0f ms after start, steady %.3f fps\n",
                  static_cast<double>(first_capture_ns - start_ns) * 1e-6,
                  static_cast<double>(frames - 1) * 1e9 /
                      static_cast<double>(last_capture_ns - first_capture_ns));
    }
    PrintDistribution("interval_ms", intervals_ms);
    std::printf("missing (interval gaps) %llu, source dropped %llu\n",
                static_cast<unsigned long long>(missing),
                static_cast<unsigned long long>(source->dropped_frames()));
    if (frames > 1) {
      std::printf("capture thread CPU %.3f ms/frame (%s)\n",
                  static_cast<double>(last_thread_cpu - first_thread_cpu) *
                      1e-6 / static_cast<double>(frames - 1),
                  capture_scheduling.c_str());
    }
    std::printf("process CPU %.1f%% of one core\n", 100.0 * cpu / wall);
    if (backend) {
      std::printf("detected frames %llu of %llu captured, detections %llu\n",
                  static_cast<unsigned long long>(processed),
                  static_cast<unsigned long long>(frames),
                  static_cast<unsigned long long>(detections));
      PrintDistribution("detect total_us", total_us);
      PrintDistribution("detect grayscale_us", gray_us);
      PrintDistribution("detect quad_us", quad_us);
      PrintDistribution("detect decode_us", decode_us);
      PrintDistribution("capture->result ms", latency_ms);
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "capture_check: " << error.what() << '\n';
    return 1;
  }
}

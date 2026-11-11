#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <vector>

#include "talOS/vision/calibrate/calibration.h"
#include "talOS/vision/capture/avfoundation_source.h"
#include "talOS/vision/frame_mailbox.h"

int main(int argc, char** argv) {
  try {
    std::string directory, device, update, board;
    double interval = 1.0, seconds = 120;
    // Live capture opens exactly this mode (or fails naming it); intrinsics
    // hold only at the size the camera will run at.
    talos::vision::CameraSourceConfig capture;
    capture.width = 0;
    capture.height = 0;
    capture.fps = 0;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "calibrate_camera --images DIR | --device ID --width W "
                     "--height H --fps F [--exposure-ms MS] [--gain G]\n"
                     "    [--interval SECONDS] [--seconds SECONDS] "
                     "[--update FILE]\n"
                     "calibrate_camera --board OUTPUT.png\n"
                     "calibrate_camera --list-devices\n";
        return 0;
      }
      if (arg == "--list-devices") {
#if defined(__APPLE__)
        for (const auto& capture : talos::vision::DiscoverCaptureDevices()) {
          std::cout << capture.id << '\t' << capture.name << '\n';
        }
#elif defined(__linux__)
        const std::filesystem::path directory("/dev/v4l/by-id");
        if (std::filesystem::exists(directory)) {
          std::vector<std::filesystem::path> devices;
          for (const auto& entry :
               std::filesystem::directory_iterator(directory)) {
            devices.push_back(entry.path());
          }
          std::sort(devices.begin(), devices.end());
          for (const auto& path : devices) {
            std::cout << path.string() << '\t' << path.filename().string()
                      << '\n';
          }
        }
#endif
        return 0;
      }
      if (i + 1 == argc) throw std::runtime_error("Missing option value");
      const std::string value = argv[++i];
      if (arg == "--images") {
        directory = value;
      } else if (arg == "--device") {
        device = value;
      } else if (arg == "--update") {
        update = value;
      } else if (arg == "--board") {
        board = value;
      } else if (arg == "--interval") {
        interval = std::stod(value);
      } else if (arg == "--seconds") {
        seconds = std::stod(value);
      } else if (arg == "--width") {
        capture.width = static_cast<uint32_t>(std::stoul(value));
      } else if (arg == "--height") {
        capture.height = static_cast<uint32_t>(std::stoul(value));
      } else if (arg == "--fps") {
        capture.fps = std::stod(value);
      } else if (arg == "--exposure-ms") {
        capture.exposure_ms = std::stod(value);
      } else if (arg == "--gain") {
        capture.gain = std::stod(value);
      } else {
        throw std::runtime_error("Unknown option: " + arg);
      }
    }
    talos::vision::calibrate::Calibration calibration;
    if (!board.empty()) {
      if (!cv::imwrite(board, calibration.BoardImage())) {
        throw std::runtime_error("Cannot write board");
      }
      return 0;
    }
    if ((directory.empty() == device.empty()) || !(interval > 0) ||
        !(seconds > 0)) {
      throw std::runtime_error(
          "Choose exactly one of --images and --device; durations must be "
          "positive");
    }
    if (!directory.empty()) {
      std::vector<std::filesystem::path> paths;
      for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file()) paths.push_back(entry.path());
      }
      std::sort(paths.begin(), paths.end());
      for (const auto& path : paths) {
        const auto image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
        if (calibration.AddImage(image)) {
          std::cerr << "Accepted " << path << '\n';
        }
      }
    } else {
      if (capture.width == 0 || capture.height == 0 || !(capture.fps > 0)) {
        throw std::runtime_error(
            "--device needs --width, --height and --fps: the mode the camera "
            "will run at");
      }
      capture.device_path = device;
      capture.auto_exposure = capture.exposure_ms == 0;
      talos::vision::FrameMailbox mailbox;
      auto source = talos::vision::CreatePlatformCameraSource(capture);
      if (!source) throw std::runtime_error("Unsupported capture platform");
      source->SetFrameCallback(
          [&](const auto& frame) { mailbox.Publish(frame); });
      source->SetConnectionCallback(
          [](auto, auto message) { std::cerr << message << '\n'; });
      if (!source->Start()) throw std::runtime_error("Cannot start capture");
      const auto start = std::chrono::steady_clock::now();
      auto next = start;
      while (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                           start)
                 .count() < seconds) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next) {
          if (const auto* frame = mailbox.TakeNewest()) {
            const auto format = frame->format();
            const int type =
                format == talos::vision::PixelFormat::kGray8   ? CV_8UC1
                : format == talos::vision::PixelFormat::kBgra8 ? CV_8UC4
                : format == talos::vision::PixelFormat::kYuyv  ? CV_8UC2
                                                               : CV_8UC3;
            cv::Mat image(static_cast<int>(frame->height()),
                          static_cast<int>(frame->width()), type,
                          const_cast<uint8_t*>(frame->data()), frame->stride());
            cv::Mat gray;
            if (type == CV_8UC1) {
              gray = image;
            } else {
              cv::cvtColor(image, gray,
                           format == talos::vision::PixelFormat::kBgra8
                               ? cv::COLOR_BGRA2GRAY
                           : format == talos::vision::PixelFormat::kYuyv
                               ? cv::COLOR_YUV2GRAY_YUY2
                           : format == talos::vision::PixelFormat::kRgb8
                               ? cv::COLOR_RGB2GRAY
                               : cv::COLOR_BGR2GRAY);
            }
            if (calibration.AddImage(gray)) {
              std::cerr << "Accepted view " << calibration.views() << '\n';
            }
          }
          next =
              now +
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(interval));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      source->Stop();
    }
    const auto result = calibration.Solve();
    std::cerr << "Views: " << result.views
              << ", RMS reprojection error: " << result.rms << " px\n";
    std::cout << talos::vision::calibrate::Fragment(result);
    if (!update.empty()) {
      std::ifstream input(update, std::ios::binary);
      if (!input) throw std::runtime_error("Cannot read update file");
      const std::string text((std::istreambuf_iterator<char>(input)), {});
      const auto edited = talos::vision::calibrate::UpdateToml(text, result);
      input.close();
      std::ofstream output(update, std::ios::binary | std::ios::trunc);
      output << edited;
      if (!output) throw std::runtime_error("Cannot write update file");
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Calibration failed: " << error.what() << '\n';
    return 1;
  }
}

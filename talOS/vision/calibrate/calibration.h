#pragma once

#include <opencv2/core.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <string>
#include <vector>

namespace talos::vision::calibrate {
struct Result {
  cv::Mat camera, distortion;
  cv::Size size;
  double rms{};
  size_t views{};
};
// Default board: 7 x 5 squares, 40 mm squares, 30 mm DICT_4X4_50 markers.
class Calibration {
 public:
  Calibration();
  cv::Mat BoardImage() const;
  bool AddImage(const cv::Mat& image);
  Result Solve() const;
  size_t views() const { return images_.size(); }

 private:
  cv::aruco::CharucoBoard board_;
  cv::aruco::CharucoDetector detector_;
  cv::Size size_;
  std::vector<std::vector<cv::Point2f>> images_;
  std::vector<std::vector<cv::Point3f>> objects_;
};
std::string Fragment(const Result& result);
// Strict, line-preserving editor. Throws if camera syntax is ambiguous.
std::string UpdateToml(const std::string& text, const Result& result);
}  // namespace talos::vision::calibrate

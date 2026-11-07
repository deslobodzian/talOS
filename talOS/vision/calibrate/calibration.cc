#include "talOS/vision/calibrate/calibration.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <map>
#include <opencv2/calib3d.hpp>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

#include "toml.hpp"

namespace talos::vision::calibrate {
namespace {
std::map<std::string, std::string> Values(const Result& r) {
  std::map<std::string, std::string> values;
  auto put = [&](std::string key, double value) {
    if (!std::isfinite(value)) {
      throw std::runtime_error("Nonfinite calibration");
    }
    std::ostringstream out;
    out << std::setprecision(17) << value;
    values.emplace(std::move(key), out.str());
  };
  put("fx", r.camera.at<double>(0, 0));
  put("fy", r.camera.at<double>(1, 1));
  put("cx", r.camera.at<double>(0, 2));
  put("cy", r.camera.at<double>(1, 2));
  const std::array names{"k1", "k2", "p1", "p2", "k3"};
  for (int i = 0; i < 5; ++i) put(names[i], r.distortion.at<double>(i));
  values["width"] = std::to_string(r.size.width);
  values["height"] = std::to_string(r.size.height);
  return values;
}
}  // namespace
Calibration::Calibration()
    : board_(cv::Size(7, 5), 0.04F, 0.03F,
             cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50)),
      detector_(board_) {}
cv::Mat Calibration::BoardImage() const {
  cv::Mat image;
  board_.generateImage(cv::Size(1400, 1000), image, 0, 1);
  return image;
}
bool Calibration::AddImage(const cv::Mat& image) {
  if (image.empty()) return false;
  if (!images_.empty() && image.size() != size_) {
    throw std::runtime_error("All images must have the same dimensions");
  }
  std::vector<cv::Point2f> corners;
  std::vector<int> ids;
  detector_.detectBoard(image, corners, ids);
  if (corners.size() < 12) return false;
  std::vector<cv::Point3f> object;
  object.reserve(ids.size());
  for (int id : ids) object.push_back(board_.getChessboardCorners().at(id));
  // Compare corresponding board points: translation, scale and perspective all
  // change this image-space signature. Reject near-duplicate views.
  for (size_t v = 0; v < images_.size(); ++v) {
    double squared = 0;
    size_t common = 0;
    for (size_t i = 0; i < object.size(); ++i) {
      for (size_t j = 0; j < objects_[v].size(); ++j) {
        if (object[i] == objects_[v][j]) {
          const auto d = corners[i] - images_[v][j];
          squared += d.dot(d);
          ++common;
        }
      }
    }
    if (common >= 8 &&
        std::sqrt(squared / static_cast<double>(common)) < image.cols * 0.025) {
      return false;
    }
  }
  size_ = image.size();
  images_.push_back(std::move(corners));
  objects_.push_back(std::move(object));
  return true;
}
Result Calibration::Solve() const {
  if (images_.size() < 12) {
    throw std::runtime_error("Need at least 12 distinct board views");
  }
  std::array<bool, 12> cells{};
  for (const auto& view : images_) {
    for (const auto& p : view) {
      int x = std::clamp(
          static_cast<int>(static_cast<double>(p.x) * 4 / size_.width), 0, 3);
      int y = std::clamp(
          static_cast<int>(static_cast<double>(p.y) * 3 / size_.height), 0, 2);
      cells[y * 4 + x] = true;
    }
  }
  if (std::count(cells.begin(), cells.end(), true) < 10) {
    throw std::runtime_error(
        "Poor coverage: board corners must cover 10 of 12 image grid cells");
  }
  Result result;
  result.size = size_;
  result.views = images_.size();
  std::vector<cv::Mat> rotations, translations;
  result.rms = cv::calibrateCamera(objects_, images_, size_, result.camera,
                                   result.distortion, rotations, translations);
  if (!std::isfinite(result.rms) || result.rms > 1.0) {
    throw std::runtime_error("Poor calibration: RMS exceeds 1 pixel");
  }
  return result;
}
std::string Fragment(const Result& result) {
  std::string out = "[camera]\n";
  for (const auto& [key, value] : Values(result)) {
    out.append(key).append(" = ").append(value).append("\n");
  }
  return out;
}
std::string UpdateToml(const std::string& text, const Result& result) {
  // Validate the entire document first: escaped/quoted duplicate tables and
  // multiline constructs must never trick the conservative line editor.
  const auto parsed = toml::parse(text);
  if (!parsed["camera"].is_table()) {
    throw std::runtime_error("Missing camera table");
  }
  auto values = Values(result);
  const std::regex header(R"(^[ \t]*\[camera\][ \t]*(#.*)?\r?$)");
  const std::regex table(R"(^[ \t]*\[)");
  const std::regex assignment(
      R"(^([ \t]*)([A-Za-z0-9_-]+)([ \t]*=[ \t]*)([^#\r]*)(.*)$)");
  const std::regex number(
      R"(^[+-]?([0-9]+(\.[0-9]*)?|\.[0-9]+)([eE][+-]?[0-9]+)?[ \t]*$)");
  std::string out;
  bool inside = false, found = false;
  std::set<std::string> seen;
  auto append_missing = [&] {
    for (const auto& [key, value] : values) {
      if (!seen.contains(key)) {
        if (!out.empty() && out.back() != '\n') out += '\n';
        out.append(key).append(" = ").append(value).append("\n");
      }
    }
  };
  for (size_t pos = 0; pos < text.size();) {
    const auto end = text.find('\n', pos);
    const bool newline = end != std::string::npos;
    std::string line =
        text.substr(pos, newline ? end - pos : text.size() - pos);
    pos = newline ? end + 1 : text.size();
    // Multiline values can contain apparent table headers. This deliberately
    // conservative editor refuses them rather than guessing TOML parser state.
    if (line.find("\"\"\"") != std::string::npos ||
        line.find("'''") != std::string::npos ||
        std::regex_search(line, std::regex(R"(^[ \t]*camera[ \t]*[.=])"))) {
      throw std::runtime_error("Unsupported ambiguous TOML syntax");
    }
    if (std::regex_search(line, table)) {
      if (inside) append_missing();
      inside = false;
      if (std::regex_match(line, header)) {
        if (found) throw std::runtime_error("Duplicate camera table");
        found = inside = true;
      } else if (line.find("camera") != std::string::npos) {
        throw std::runtime_error("Ambiguous camera table syntax");
      }
    } else if (inside) {
      std::smatch match;
      if (std::regex_match(line, match, assignment)) {
        const std::string key = match[2];
        if (values.contains(key)) {
          if (!seen.insert(key).second ||
              !std::regex_match(match[4].str(), number)) {
            throw std::runtime_error("Ambiguous camera calibration key");
          }
          std::string old = match[4];
          const auto last = old.find_last_not_of(" \t");
          line = match[1].str() + key + match[3].str() + values.at(key) +
                 old.substr(last + 1) + match[5].str();
        }
      } else if (line.find_first_not_of(" \t\r") != std::string::npos &&
                 line[line.find_first_not_of(" \t\r")] != '#') {
        throw std::runtime_error("Unsupported camera table syntax");
      }
    }
    out += line;
    if (newline) out += '\n';
  }
  if (!found) throw std::runtime_error("Missing camera table");
  if (inside) append_missing();
  return out;
}
}  // namespace talos::vision::calibrate

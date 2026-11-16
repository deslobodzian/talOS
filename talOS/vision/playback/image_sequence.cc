#include "talOS/vision/playback/image_sequence.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace talos::vision::playback {
namespace {

bool IsImage(const std::filesystem::path& path) {
  auto ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".pgm" ||
         ext == ".bmp";
}

std::string Trim(std::string text) {
  while (!text.empty() &&
         std::isspace(static_cast<unsigned char>(text.back()))) {
    text.pop_back();
  }
  std::size_t start = 0;
  while (start < text.size() &&
         std::isspace(static_cast<unsigned char>(text[start]))) {
    ++start;
  }
  return text.substr(start);
}

bool Fail(std::string* error, std::string message) {
  if (error != nullptr) *error = std::move(message);
  return false;
}

// Reads `<file>,<pts_ns>` lines; the first line may be a header.
bool ReadTimeline(const std::filesystem::path& directory,
                  std::vector<RecordedFrame>* frames, std::string* error) {
  const auto csv = directory / kTimestampsFile;
  std::ifstream in(csv);
  if (!in) return Fail(error, "cannot read " + csv.string());
  std::string line;
  int number = 0;
  while (std::getline(in, line)) {
    ++number;
    line = Trim(line);
    if (line.empty()) continue;
    const auto comma = line.rfind(',');
    if (comma == std::string::npos) {
      if (number == 1) continue;
      return Fail(error, csv.string() + ":" + std::to_string(number) +
                             ": expected <file>,<pts_ns>");
    }
    const auto name = Trim(line.substr(0, comma));
    const auto stamp = Trim(line.substr(comma + 1));
    std::int64_t pts = 0;
    const auto parsed =
        std::from_chars(stamp.data(), stamp.data() + stamp.size(), pts);
    if (parsed.ec != std::errc{} || parsed.ptr != stamp.data() + stamp.size()) {
      if (number == 1) continue;  // header
      return Fail(error, csv.string() + ":" + std::to_string(number) +
                             ": pts_ns is not an integer");
    }
    // A name is a file in this directory, never a path out of it.
    const std::filesystem::path file(name);
    if (name.empty() || file.has_parent_path() || file.is_absolute() ||
        name == "." || name == "..") {
      return Fail(error,
                  csv.string() + ":" + std::to_string(number) +
                      ": file must be a name in the recording directory");
    }
    if (!frames->empty() && pts <= frames->back().pts_ns) {
      return Fail(error, csv.string() + ":" + std::to_string(number) +
                             ": pts_ns must increase strictly");
    }
    if (!std::filesystem::is_regular_file(directory / file)) {
      return Fail(error, csv.string() + ":" + std::to_string(number) + ": " +
                             name + " is not a file");
    }
    frames->push_back({directory / file, pts});
  }
  return true;
}

}  // namespace

std::optional<ImageSequence> ImageSequence::Open(
    const std::filesystem::path& directory, double fps, std::string* error) {
  ImageSequence sequence;
  std::error_code ec;
  if (!std::filesystem::is_directory(directory, ec)) {
    Fail(error, directory.string() + " is not a directory");
    return std::nullopt;
  }
  if (std::filesystem::exists(directory / kTimestampsFile, ec)) {
    if (!ReadTimeline(directory, &sequence.frames_, error)) return std::nullopt;
    if (!sequence.frames_.empty()) {
      sequence.origin_ns_ = sequence.frames_.front().pts_ns;
      for (auto& frame : sequence.frames_) frame.pts_ns -= sequence.origin_ns_;
    }
  } else {
    if (!std::isfinite(fps) || fps <= 0) {
      Fail(error, directory.string() + " has no " + kTimestampsFile +
                      " and the frame rate is not finite and positive");
      return std::nullopt;
    }
    std::vector<std::filesystem::path> files;
    for (const auto& entry :
         std::filesystem::directory_iterator(directory, ec)) {
      if (entry.is_regular_file() && IsImage(entry.path())) {
        files.push_back(entry.path());
      }
    }
    if (ec) {
      Fail(error, "cannot list " + directory.string() + ": " + ec.message());
      return std::nullopt;
    }
    std::sort(files.begin(), files.end());
    for (std::size_t i = 0; i < files.size(); ++i) {
      sequence.frames_.push_back(
          {files[i], static_cast<std::int64_t>(
                         std::llround(static_cast<double>(i) * 1e9 / fps))});
    }
  }
  if (sequence.frames_.empty()) {
    Fail(error, directory.string() + " holds no frames");
    return std::nullopt;
  }
  return sequence;
}

std::int64_t ImageSequence::mean_period_ns() const {
  if (frames_.size() < 2) return 0;
  return frames_.back().pts_ns / static_cast<std::int64_t>(frames_.size() - 1);
}

bool ImageSequence::LoadGray(std::size_t i, std::vector<std::uint8_t>* pixels,
                             int* width, int* height,
                             std::string* error) const {
  if (i >= frames_.size()) return Fail(error, "frame index out of range");
  // IMREAD_GRAYSCALE also reduces 16-bit images to 8 bits.
  const cv::Mat image =
      cv::imread(frames_[i].path.string(), cv::IMREAD_GRAYSCALE);
  if (image.empty() || image.type() != CV_8UC1) {
    return Fail(error, "cannot decode " + frames_[i].path.string());
  }
  pixels->resize(static_cast<std::size_t>(image.cols) * image.rows);
  for (int row = 0; row < image.rows; ++row) {
    std::copy_n(image.ptr<std::uint8_t>(row), image.cols,
                pixels->data() + static_cast<std::size_t>(row) * image.cols);
  }
  *width = image.cols;
  *height = image.rows;
  return true;
}

}  // namespace talos::vision::playback

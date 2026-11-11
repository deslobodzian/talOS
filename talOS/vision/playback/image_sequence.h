#pragma once

// A recording on disk: a directory of image files and the timeline they were
// captured on. The recorded frame source and the hand-run evaluation tool read
// recordings through this one index.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace talos::vision::playback {

// The file a recording's timeline is read from, beside its images.
inline constexpr const char* kTimestampsFile = "timestamps.csv";

struct RecordedFrame {
  std::filesystem::path path;
  // Presentation time on the recording's own timeline, in nanoseconds from
  // its first frame (the first frame is at 0).
  std::int64_t pts_ns{0};
};

// An image sequence and its timeline. With `timestamps.csv` in the directory,
// that file lists the frames, one `<file name>,<pts_ns>` line each (a header
// line starting with a non-digit is skipped), in strictly increasing
// presentation time; the files must be in the directory. Without it, every
// .png/.jpg/.jpeg/.pgm/.bmp file in the directory is a frame, in file-name
// order, at `fps` frames per second from 0.
class ImageSequence {
 public:
  // Indexes `directory`. `fps` is used only without `timestamps.csv` and then
  // must be finite and positive. Returns nullopt with `error` when the
  // directory is unreadable, holds no frames, or the timeline is malformed.
  static std::optional<ImageSequence> Open(
      const std::filesystem::path& directory, double fps, std::string* error);

  std::size_t size() const { return frames_.size(); }
  const RecordedFrame& operator[](std::size_t i) const { return frames_[i]; }
  // Mean frame period of the timeline, 0 for a single frame.
  std::int64_t mean_period_ns() const;
  // The first frame's presentation time as `timestamps.csv` states it (an
  // absolute recording clock, say), 0 without that file.
  std::int64_t origin_ns() const { return origin_ns_; }

  // Decodes frame `i` as 8-bit grayscale into `pixels` (stride = width),
  // reusing its capacity. False with `error` when the file cannot be decoded.
  bool LoadGray(std::size_t i, std::vector<std::uint8_t>* pixels, int* width,
                int* height, std::string* error) const;

 private:
  std::vector<RecordedFrame> frames_;
  std::int64_t origin_ns_{0};
};

}  // namespace talos::vision::playback

// Offline frame replay: re-runs AprilTag detection and the camera node's
// localization over every recorded sidecar, then prints a Markdown summary of
// valid observations, per-reason rejection counts, and position/yaw error RMS
// before/after (against truth when present).
//
//   bazel run //talOS/vision/recording:frame_replay --
//     --frames <session>/frames/<camera> --layout <field_layout.json>

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>

#include "talOS/vision/field_sim/field_layout_loader.h"
#include "talOS/vision/recording/replay.h"

namespace {

using talos::vision::recording::DetectorOverrides;

struct Flags {
  std::string frames;
  std::string layout;
  DetectorOverrides overrides;
};

constexpr double kDefaultTagSizeM = 0.1651;

Flags ParseFlags(int argc, char** argv) {
  Flags flags;
  auto next = [&](int& i) -> std::string {
    if (i + 1 >= argc) {
      throw std::invalid_argument("missing value");
    }
    return argv[++i];
  };
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--frames") {
      flags.frames = next(i);
    } else if (arg == "--layout") {
      flags.layout = next(i);
    } else if (arg == "--decimate") {
      flags.overrides.decimate = std::stod(next(i));
    } else if (arg == "--threads") {
      flags.overrides.threads = std::stoi(next(i));
    } else if (arg == "--sigma") {
      flags.overrides.sigma = std::stod(next(i));
    } else if (arg == "--refine-edges") {
      const std::string v = next(i);
      if (v == "true") {
        flags.overrides.refine_edges = true;
      } else if (v == "false") {
        flags.overrides.refine_edges = false;
      } else {
        throw std::invalid_argument("--refine-edges must be true or false");
      }
    } else if (arg == "--hamming") {
      flags.overrides.hamming = std::stoi(next(i));
    } else if (arg == "--min-decision-margin") {
      flags.overrides.min_decision_margin = std::stod(next(i));
    } else if (arg == "--backend") {
      flags.overrides.backend = next(i);
    } else if (arg == "--max-range-m") {
      flags.overrides.max_range_m = std::stod(next(i));
    } else {
      throw std::invalid_argument("unknown argument: " + arg);
    }
  }
  if (flags.frames.empty()) {
    throw std::invalid_argument("--frames is required");
  }
  if (flags.layout.empty()) {
    throw std::invalid_argument("--layout is required");
  }
  return flags;
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("cannot read " + path);
  }
  return {std::istreambuf_iterator<char>{in}, {}};
}

}  // namespace

int main(int argc, char** argv) {
  using namespace talos::vision::recording;
  try {
    const Flags flags = ParseFlags(argc, argv);

    talos::vision::field_sim::FieldLayoutData layout_data;
    std::string error;
    if (!talos::vision::field_sim::LoadFieldLayoutFromJson(
            ReadFile(flags.layout), kDefaultTagSizeM, &layout_data, &error)) {
      throw std::runtime_error("layout: " + error);
    }

    ReplayStats stats;
    if (!ReplayFrames(flags.frames, layout_data, flags.overrides, &stats,
                      &error)) {
      throw std::runtime_error(error);
    }

    std::cout << "# Frame replay: " << flags.frames << "\n\n";
    std::cout << "Frames: " << stats.frames << "\n\n";
    std::cout << "| | count |\n|---|---|\n";
    std::cout << "| valid (before) | " << stats.valid_before << " |\n";
    std::cout << "| valid (after) | " << stats.valid_after << " |\n\n";

    std::cout << "## Rejections (before / after)\n\n";
    std::cout << "| reason | before | after |\n|---|---|---|\n";
    std::set<std::string> reasons;
    for (const auto& [r, _] : stats.rejections_before) {
      reasons.insert(r);
    }
    for (const auto& [r, _] : stats.rejections_after) {
      reasons.insert(r);
    }
    for (const auto& r : reasons) {
      std::cout << "| " << r << " | " << stats.rejections_before[r] << " | "
                << stats.rejections_after[r] << " |\n";
    }
    std::cout << "\n## Position/yaw error RMS (truth frames)\n\n";
    std::cout << "| | x_m | y_m | yaw_rad |\n|---|---|---|---|\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "| before | " << stats.rms_x_before << " | "
              << stats.rms_y_before << " | " << stats.rms_yaw_before << " |\n";
    std::cout << "| after | " << stats.rms_x_after << " | " << stats.rms_y_after
              << " | " << stats.rms_yaw_after << " |\n";
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "frame_replay: %s\n", e.what());
    return 1;
  }
}

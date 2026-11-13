#pragma once

// Loads Choreo (https://choreo.autos) `.traj` files into a Trajectory.
// Supported: `version` kChoreoMinVersion to kChoreoMaxVersion (Choreo
// v2025.0.x to v2026.0.x), swerve samples only; README.md lists the exact
// fields read. Every other version is refused.

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>

#include "talOS/trajectory/trajectory.h"

namespace talos::trajectory {

// The .traj schema versions implemented (README.md "Choreo files"): 1
// (Choreo v2025.0.x), 2 (v2026.0.0-beta-1), 3 (v2026.0.0 on). Their sample
// layout is identical; 3 adds trajectory.config, which is not read.
inline constexpr int kChoreoMinVersion = 1;
inline constexpr int kChoreoMaxVersion = 3;

// Plausibility bounds: a sample with |x| or |y| above kMaxCoordinate_m, or a
// time above kMaxTime_s, is refused as a corrupt file (an FRC field is under
// 20 m long and a match under 3 minutes).
inline constexpr double kMaxCoordinate_m = 100.0;
inline constexpr double kMaxTime_s = 3600.0;

// A trajectory, or an empty one and an error naming the source and the
// problem.
struct LoadResult {
  Trajectory trajectory;  // empty on failure
  std::string error;      // empty on success
  // Events dropped as ChoreoLib drops them: targetTimestamp not a number,
  // a negative time, or an empty name.
  std::size_t dropped_event_count = 0;
  // Non-empty when the file loaded but not as Choreo writes it, for the node
  // to log: today, samples without ax/ay/alpha, which make the whole file
  // interpolate with cubic Hermite.
  std::string note;

  bool ok() const { return error.empty(); }
  explicit operator bool() const { return ok(); }
};

// Parses `json`; `source` names it in errors (normally the file path).
// Never throws.
LoadResult ParseChoreo(std::string_view json, std::string_view source);

// Reads and parses the file at `path`. Never throws.
LoadResult LoadChoreoFile(const std::filesystem::path& path);

}  // namespace talos::trajectory

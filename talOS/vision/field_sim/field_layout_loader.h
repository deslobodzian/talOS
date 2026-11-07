// Loader from WPILib AprilTag field-layout JSON into FieldLayout.
//
// Two schema generations are accepted because both circulate:
//   - classic: {"tags": [{"ID", "pose": {"translation": {x,y,z},
//     "rotation": {"quaternion": {W,X,Y,Z}}}}], "field": {"length","width"}}
//   - fields-module (2026+): {"field-tags": [...], "field-dimensions":
//     {"length","width"}, ...} with the same per-tag shape.
//
// Tag poses follow WPILib convention: NWU field frame, tag +X out of the tag
// face (normal), tag +Z up, tag +Y completing the right-handed frame. That
// matches the tag frame used by talos::vision::GetTagModelCorners, so sim
// projections line up with real detector output.

#pragma once

#include <string>
#include <string_view>

#include "talOS/vision/calibration.h"

namespace talos::vision::field_sim {

struct FieldDims {
  double length_m = 0.0;  // Field X extent (away from blue wall).
  double width_m = 0.0;   // Field Y extent.
};

struct FieldLayoutData {
  // Default member initializer: FieldLayout's ctor is explicit, so the
  // struct would otherwise not be default-constructible.
  FieldLayout layout{0.1651};
  FieldDims dims;
  std::string name;
};

// Parses a WPILib layout document. tag_size_m is the physical tag edge
// length (layouts do not carry it; 0.1651 m is the 36h11 FRC standard).
// Returns false with a human-readable error on any malformed input.
bool LoadFieldLayoutFromJson(std::string_view text, double tag_size_m,
                             FieldLayoutData* out,
                             std::string* error = nullptr);

}  // namespace talos::vision::field_sim

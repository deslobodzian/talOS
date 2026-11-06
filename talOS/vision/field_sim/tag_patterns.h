// tag36h11 code patterns for the software rasterizer.
//
// Extracts each tag's 6x6 data-bit grid from the vendored AprilTag family at
// runtime, so the rasterizer never hardcodes codes and can never disagree
// with the detector about what a tag looks like. Bit polarity follows the
// decoder: rcode accumulates MSB-first and sets a bit for cells brighter
// than the local threshold, so a set code bit paints WHITE.
//
// Kept behind its own target: the rasterizer stays dependency-free while
// only this header (and its users) link the C AprilTag library.

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>

#include "tag36h11.h"
#include "talOS/vision/field_sim/tag_rasterizer.h"

namespace talos::vision::field_sim {

// Loads every tag36h11 pattern (ids 0..ncodes-1; FRC tag IDs index this
// directly). Returns false when the family data surprises us.
inline bool LoadTag36h11Patterns(std::map<int, TagPattern>* out,
                                 std::string* error = nullptr) {
  if (out == nullptr) return false;
  apriltag_family_t* family = tag36h11_create();
  if (family == nullptr) {
    if (error != nullptr) *error = "tag36h11_create failed";
    return false;
  }
  std::map<int, TagPattern> patterns;
  for (uint32_t id = 0; id < family->ncodes; ++id) {
    TagPattern pattern;
    pattern.id = static_cast<int>(id);
    const uint64_t code = family->codes[id];
    for (uint32_t i = 0; i < family->nbits; ++i) {
      const int bx = static_cast<int>(family->bit_x[i]);
      const int by = static_cast<int>(family->bit_y[i]);
      // Data bits live in the inner 6x6 (bit coords 1..6); anything else
      // means the family changed shape out from under the rasterizer.
      if (bx < 1 || bx > 6 || by < 1 || by > 6) {
        if (error != nullptr) *error = "tag36h11 bit outside the 6x6 data grid";
        tag36h11_destroy(family);
        return false;
      }
      pattern.bits[by - 1][bx - 1] =
          ((code >> (family->nbits - 1 - i)) & 1u) != 0;
    }
    patterns[pattern.id] = pattern;
  }
  tag36h11_destroy(family);
  *out = std::move(patterns);
  return true;
}

}  // namespace talos::vision::field_sim

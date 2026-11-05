#pragma once

// The unit suffixes of docs/METHODOLOGY.md §6: the one table every numeric
// message field name ends with one of. talos_messages fails the build on a
// numeric field without one (talOS/build/descriptor_gen.cc); bools, enums,
// structs and arrays of structs need none. C++17: the generator builds in
// Bazel's exec configuration, which does not get the repo's -std=c++23.

#include <array>
#include <string_view>

namespace talos::schema {

struct SuffixUnit {
  std::string_view suffix;
  std::string_view unit;
};

// The unit suffixes and their display units (docs/METHODOLOGY.md §6).
inline constexpr std::array<SuffixUnit, 25> kUnitSuffixTable = {{
    {"_ns", "ns"},       {"_us", "us"},         {"_m", "m"},
    {"_mps", "m/s"},     {"_mps2", "m/s²"},     {"_rad", "rad"},
    {"_radps", "rad/s"}, {"_radps2", "rad/s²"}, {"_rot", "rot"},
    {"_rps", "rot/s"},   {"_rps2", "rot/s²"},   {"_v", "V"},
    {"_a", "A"},         {"_w", "W"},           {"_degc", "°C"},
    {"_kg", "kg"},       {"_n", "N"},           {"_nm", "N·m"},
    {"_frac", ""},       {"_duty", ""},         {"_ratio", ""},
    {"_count", ""},      {"_index", ""},        {"_id", ""},
    {"_mask", ""},
}};

inline constexpr std::array<std::string_view, 25> kUnitSuffixes = {
    "_ns",     "_us",    "_m",   "_mps",  "_mps2", "_rad",  "_radps",
    "_radps2", "_rot",   "_rps", "_rps2", "_v",    "_a",    "_w",
    "_degc",   "_kg",    "_n",   "_nm",   "_frac", "_duty", "_ratio",
    "_count",  "_index", "_id",  "_mask",
};

// `name` is something followed by one of kUnitSuffixes.
constexpr bool has_unit_suffix(std::string_view name) {
  for (const auto& entry : kUnitSuffixTable) {
    if (name.size() > entry.suffix.size() &&
        name.substr(name.size() - entry.suffix.size()) == entry.suffix) {
      return true;
    }
  }
  return false;
}

// Display unit for a field name derived from its suffix, or empty string if
// none.
constexpr std::string_view unit_for_field(std::string_view name) {
  for (const auto& entry : kUnitSuffixTable) {
    if (name.size() > entry.suffix.size() &&
        name.substr(name.size() - entry.suffix.size()) == entry.suffix) {
      return entry.unit;
    }
  }
  return {};
}

}  // namespace talos::schema

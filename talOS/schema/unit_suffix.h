#pragma once

// The unit suffixes of docs/METHODOLOGY.md §6: the one table every numeric
// message field name ends with one of. talos_messages fails the build on a
// numeric field without one (talOS/build/descriptor_gen.cc); bools, enums,
// structs and arrays of structs need none. C++17: the generator builds in
// Bazel's exec configuration, which does not get the repo's -std=c++23.

#include <array>
#include <string_view>

namespace talos::schema {

inline constexpr std::array<std::string_view, 25> kUnitSuffixes = {
    "_ns",     "_us",    "_m",   "_mps",  "_mps2", "_rad",  "_radps",
    "_radps2", "_rot",   "_rps", "_rps2", "_v",    "_a",    "_w",
    "_degc",   "_kg",    "_n",   "_nm",   "_frac", "_duty", "_ratio",
    "_count",  "_index", "_id",  "_mask",
};

// `name` is something followed by one of kUnitSuffixes.
constexpr bool has_unit_suffix(std::string_view name) {
  for (std::string_view suffix : kUnitSuffixes) {
    if (name.size() > suffix.size() &&
        name.substr(name.size() - suffix.size()) == suffix) {
      return true;
    }
  }
  return false;
}

}  // namespace talos::schema

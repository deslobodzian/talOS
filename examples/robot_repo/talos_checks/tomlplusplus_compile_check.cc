// Compile check for @talos//third_party:tomlplusplus from a dependent
// repository (REQ-DEV-29).
#include "toml.hpp"

namespace talos_checks {

bool CanParseToml();

bool CanParseToml() {
  const toml::table parsed = toml::parse("k = 1\n");
  return parsed["k"].value_or(0) == 1;
}

}  // namespace talos_checks

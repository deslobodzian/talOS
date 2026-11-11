#pragma once

// Locates the testdata/*.traj fixtures in a test's runfiles. BUILD passes
// each fixture's rlocation in TALOS_TRAJECTORY_<NAME> (choreo_v3_swerve,
// curve, splits_events).

#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include "rules_cc/cc/runfiles/runfiles.h"

namespace talos::trajectory::testing {

inline std::string FixturePath(const char* env) {
  std::string error;
  const std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"fixture unavailable: "} + env + " " +
                             error);
  }
  return runfiles->Rlocation(location);
}

}  // namespace talos::trajectory::testing

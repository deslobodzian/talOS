load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

# Time-indexed holonomic trajectories, the Choreo .traj loader, straight
# lines, alliance flips and HolonomicFollower (README.md). Beside the node stack: no
# dependency on talOS/node or any robot.
cc_library(
    name = "trajectory",
    srcs = [
        "choreo.cc",
        "holonomic_follower.cc",
        "straight_line.cc",
        "trajectory.cc",
    ],
    hdrs = [
        "choreo.h",
        "holonomic_follower.h",
        "straight_line.h",
        "trajectory.h",
    ],
    deps = [
        "//talOS/geometry",
        "@nlohmann_json//:json",
    ],
)

cc_library(
    name = "test_fixtures",
    testonly = True,
    hdrs = ["test_fixtures.h"],
    # BAZEL_CURRENT_REPOSITORY, which the header uses, is defined only for a
    # target that depends on the runfiles library directly, so every test
    # including this header also lists @rules_cc//cc/runfiles.
    visibility = ["//visibility:private"],
    deps = ["@rules_cc//cc/runfiles"],
)

cc_test(
    name = "trajectory_test",
    size = "small",
    srcs = ["trajectory_test.cc"],
    deps = [
        ":trajectory",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "straight_line_test",
    size = "small",
    srcs = ["straight_line_test.cc"],
    deps = [
        ":trajectory",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "choreo_test",
    size = "small",
    srcs = ["choreo_test.cc"],
    data = ["testdata/choreo_v3_swerve.traj"],
    env = {
        "TALOS_TRAJECTORY_CHOREO_V3_SWERVE": "$(rlocationpath testdata/choreo_v3_swerve.traj)",
    },
    deps = [
        ":test_fixtures",
        ":trajectory",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

cc_test(
    name = "holonomic_follower_test",
    size = "small",
    srcs = ["holonomic_follower_test.cc"],
    data = ["testdata/curve.traj"],
    env = {
        "TALOS_TRAJECTORY_CURVE": "$(rlocationpath testdata/curve.traj)",
    },
    deps = [
        ":test_fixtures",
        ":trajectory",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# Its own binary: the allocation probe replaces malloc on glibc. Test-only
# use of talOS/node's probe; the library itself never depends on talOS/node.
cc_test(
    name = "no_allocation_test",
    size = "small",
    srcs = ["no_allocation_test.cc"],
    data = ["testdata/splits_events.traj"],
    env = {
        "TALOS_TRAJECTORY_SPLITS_EVENTS": "$(rlocationpath testdata/splits_events.traj)",
    },
    deps = [
        ":test_fixtures",
        ":trajectory",
        "//talOS/node:allocation_probe",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

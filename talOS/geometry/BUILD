load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

# Header-only Lie groups (SO2/SE2/SO3/SE3) on Eigen, and the swerve module
# setpoint built on SO2. No sources, no state: every consumer pays only for
# what it inlines.
cc_library(
    name = "geometry",
    hdrs = [
        "lie.h",
        "se2.h",
        "se3.h",
        "so2.h",
        "so3.h",
        "swerve_module.h",
    ],
    deps = ["@eigen3//:eigen"],
)

cc_test(
    name = "geometry_test",
    size = "small",
    srcs = ["geometry_test.cc"],
    deps = [
        ":geometry",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "lie_property_test",
    size = "small",
    srcs = ["lie_property_test.cc"],
    deps = [
        ":geometry",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "swerve_module_test",
    size = "small",
    srcs = ["swerve_module_test.cc"],
    deps = [
        ":geometry",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

# Microbenchmark: prints ns/op for compose/exp/log/act/Jacobian.
cc_binary(
    name = "geometry_bench",
    srcs = ["geometry_bench.cc"],
    deps = [":geometry"],
)

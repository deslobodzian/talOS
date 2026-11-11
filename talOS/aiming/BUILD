load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

# Shooter aiming maths: an interpolating shot table, a static and
# shoot-on-the-move target solver, and a ready-to-fire gate. A library, no
# node; see README.md for frames, signs and tuning.
cc_library(
    name = "aiming",
    srcs = [
        "readiness.cc",
        "target_solver.cc",
    ],
    hdrs = [
        "interpolating_table.h",
        "readiness.h",
        "target_solver.h",
    ],
    deps = [
        "//talOS/geometry",
        "//talOS/msgs:geometry",
        "@eigen3//:eigen",
    ],
)

cc_test(
    name = "interpolating_table_test",
    size = "small",
    srcs = ["interpolating_table_test.cc"],
    deps = [
        ":aiming",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "target_solver_test",
    size = "small",
    srcs = ["target_solver_test.cc"],
    deps = [
        ":aiming",
        "//talOS/geometry",
        "//talOS/node:allocation_probe",
        "@eigen3//:eigen",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "readiness_test",
    size = "small",
    srcs = ["readiness_test.cc"],
    deps = [
        ":aiming",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

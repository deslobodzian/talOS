load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

# The subsystem's own config file, so the robot manifest filegroup can ship it.
exports_files(["subsystem.toml"])
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "operator_interface",
    hdrs = ["node.h"],
    deps = [
        "//2026-robot/main_processor/driver_station:driver_station",
        "//2026-robot/main_processor/drivetrain:drivetrain",
        "//2026-robot/main_processor/shooter:shooter",
        "//talOS/driver_station:driver_station",
        "//talOS/events:events",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "node",
    srcs = ["main.cc"],
    deps = [
        ":operator_interface",
        "//talOS/configuration:configuration",
        "//talOS/events:events",
        "//talOS/introspection:describe",
        "//talOS/introspection:reporter",
        "//talOS/process:process",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":operator_interface",
        "@googletest//:gtest_main",
    ],
)

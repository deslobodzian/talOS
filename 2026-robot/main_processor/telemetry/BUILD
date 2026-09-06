load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

# The subsystem's own config file, so the robot manifest filegroup can ship it.
exports_files(["subsystem.toml"])
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "telemetry",
    hdrs = ["node.h"],
    deps = [
        "//2026-robot/main_processor/driver_station:driver_station",
        "//2026-robot/main_processor/drivetrain:drivetrain",
        "//2026-robot/main_processor/odometry:odometry",
        "//studio/schema:wire",
        "//studio/schema:telemetry_cc",
        "//talOS/driver_station:driver_station",
        "//talOS/events:events",
        "//talOS/rtms:rtms",
        "@flatbuffers//:flatbuffers",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "node",
    srcs = ["main.cc"],
    deps = [
        ":telemetry",
        "//talOS/configuration:configuration",
        "//talOS/events:events",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/introspection:reporter",
        "//talOS/process:node_main",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":telemetry",
        "@googletest//:gtest_main",
    ],
)

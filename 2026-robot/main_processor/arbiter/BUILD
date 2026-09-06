load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "arbiter",
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
        ":arbiter",
        "//2026-robot/main_processor/operator_interface:operator_interface",
        "//talOS/configuration:configuration",
        "//talOS/process:process",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = ["//2026-robot/main_processor/configuration:robot.toml"],
    deps = [
        ":arbiter",
        "//2026-robot/main_processor/operator_interface:operator_interface",
        "//talOS/configuration:configuration",
        "@googletest//:gtest_main",
    ],
)

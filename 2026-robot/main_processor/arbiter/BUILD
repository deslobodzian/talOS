load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

# The subsystem's own config file, so the robot manifest filegroup can ship it.
exports_files(["subsystem.toml"])
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "arbiter",
    hdrs = ["node.h"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/arbiter",
    deps = [
        "@talos_driver_station//:contract",
        "//2026-robot/main_processor/drivetrain:messages_cc",
        "//2026-robot/main_processor/intake:messages_cc",
        "//2026-robot/main_processor/shooter:messages_cc",
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
        "@talos_operator_interface//:operator_interface",
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
    data = ["//2026-robot/main_processor/configuration:robot_config"],
    deps = [
        ":arbiter",
        "@talos_driver_station//:driver_station",
        "//2026-robot/main_processor/drivetrain:integration_test_support",
        "@talos_operator_interface//:operator_interface",
        "//talOS/configuration:configuration",
        "@googletest//:gtest_main",
    ],
)

load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

# The subsystem's own config file, so the robot manifest filegroup can ship it.
exports_files(["subsystem.toml"])

cc_library(
    name = "operator_interface",
    hdrs = ["node.h"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/year2026_operator_interface",
    deps = [
        "@talos_driver_station//:contract",
        "@talos_operator_interface//:operator_interface",
        "//2026-robot/main_processor/intake:messages_cc",
        "//2026-robot/main_processor/shooter:messages_cc",
        "//talOS/configuration:configuration",
        "//talOS/driver_station:driver_station",
        "//talOS/events:events",
        "//talOS/process:node_main",
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
        ":operator_interface",
        "@googletest//:gtest_main",
    ],
)

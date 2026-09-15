load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@talos//build:defs.bzl", "talos_node")

exports_files(["subsystem.toml"])

cc_library(
    name = "telemetry",
    hdrs = ["node.h"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/telemetry",
    deps = [
        "@talos_driver_station//:contract",
        "@talos_chassis_contracts//:contract",
        "@talos_odometry//:contract",
        "@studio_schema//:schema",
        "@studio_schema//:wire",
        "@talos//talOS/driver_station:driver_station",
        "@talos//talOS/events:events",
        "@talos//talOS/rtms:rtms",
        "@flatbuffers//:flatbuffers",
    ],
    visibility = ["//visibility:public"],
)

# Handwritten entry point (not talos_cc_node): this node starts its timer at
# an explicit first tick, declares its RTMS Studio feed as an external source
# in both --describe and the live Reporter, and prints a custom exit summary.
# The shared macro cannot express those hooks, so the binary is wired here
# while still publishing TalosNodeInfo for talos_bundle.
cc_binary(
    name = "node_bin",
    srcs = ["main.cc"],
    deps = [
        ":telemetry",
        "@talos//talOS/configuration:configuration",
        "@talos//talOS/events:events",
        "@talos//talOS/introspection:describe",
        "@talos//talOS/introspection:names",
        "@talos//talOS/introspection:reporter",
        "@talos//talOS/process:node_main",
    ],
    visibility = ["//visibility:private"],
)

talos_node(
    name = "node",
    binary = ":node_bin",
    node_name = "telemetry",
    node_target = "//:node",
    language = "cpp",
    visibility = ["//visibility:public"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":telemetry",
        "@talos//talOS/memory:shared_memory",
        "@googletest//:gtest_main",
    ],
)

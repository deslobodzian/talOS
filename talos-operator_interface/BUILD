load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@talos//build:defs.bzl", "talos_cc_node")

exports_files(["subsystem.toml"])

cc_library(
    name = "operator_interface",
    hdrs = ["node.h"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/operator_interface",
    deps = [
        "@talos_driver_station//:contract",
        "@talos_chassis_contracts//:contract",
        "@talos//talOS/configuration:configuration",
        "@talos//talOS/driver_station:driver_station",
        "@talos//talOS/events:events",
        "@talos//talOS/process:node_main",
    ],
    visibility = ["//visibility:public"],
)

talos_cc_node(
    name = "node",
    node_name = "operator_interface",
    factory = "talos::oi::CreateNode",
    hardware = False,
    default_config_path = "subsystem.toml",
    deps = [":operator_interface"],
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

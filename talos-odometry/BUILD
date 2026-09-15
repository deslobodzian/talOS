load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@talos//build:defs.bzl", "talos_cc_node", "talos_contract_cc")

exports_files(["subsystem.toml"])

talos_contract_cc(
    name = "contract",
    schema = "odometry_message.fbs",
    headers = ["packet.h"],
    extra_deps = [
        "@talos_chassis_contracts//:contract",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "odometry",
    hdrs = ["node.h"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/odometry",
    deps = [
        ":contract",
        "@talos_chassis_contracts//:contract",
        "@talos//talOS/events:events",
        "@talos//talOS/process:node_main",
    ],
    visibility = ["//visibility:public"],
)

talos_cc_node(
    name = "node",
    node_name = "odometry",
    factory = "talos::odometry::CreateNode",
    contract = ":contract",
    hardware = False,
    default_config_path = "subsystem.toml",
    deps = [":odometry"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":odometry",
        "@googletest//:gtest_main",
    ],
)

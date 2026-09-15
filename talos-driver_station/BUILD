load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@talos//build:defs.bzl", "talos_cc_node", "talos_contract_cc")

exports_files(["subsystem.toml"])

talos_contract_cc(
    name = "contract",
    schema = "driver_station_message.fbs",
    headers = ["packet.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "driver_station",
    hdrs = ["node.h"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/driver_station",
    deps = [
        ":contract",
        "@talos//talOS/configuration:configuration",
        "@talos//talOS/driver_station:driver_station",
        "@talos//talOS/events:events",
        "@talos//talOS/hardware:hardware",
        "@talos//talOS/process:node_main",
    ],
    visibility = ["//visibility:public"],
)

talos_cc_node(
    name = "node",
    node_name = "driver_station",
    factory = "talos::driver_station::CreateNode",
    contract = ":contract",
    hardware = False,
    default_config_path = "subsystem.toml",
    deps = [":driver_station"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "send_joystick",
    srcs = ["send_joystick.cc"],
    deps = [
        ":contract",
        "@talos//talOS/driver_station:driver_station",
        "@talos//talOS/events:events_os",
        "@talos//talOS/ipc:ipc",
        "@talos//talOS/process:process",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":driver_station",
        "@googletest//:gtest_main",
    ],
)

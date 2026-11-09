load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_messages")

# BridgeStatus (namespace bridge) and its topic: hardware_node's health,
# published every 20 ms on /hw/status.
talos_messages(
    name = "messages",
    srcs = ["bridge_status.fbs"],
    include_prefix = "bridge",
    topics = {"status": "/hw/status"},
)

# The gateway boots a stream has been in, current and recently retired: the
# State ordering gate (via clock_offset) and every Driver Station consumer.
cc_library(
    name = "boot_history",
    hdrs = ["boot_history.h"],
    visibility = ["//visibility:public"],
)

# The gateway-to-node clock mapping the bridge publishes on /hw/status, also
# run by talos::testing::SimHardware when it plays the bridge.
cc_library(
    name = "clock_offset",
    srcs = ["clock_offset.cc"],
    hdrs = ["clock_offset.h"],
    visibility = ["//visibility:public"],
    deps = [":boot_history"],
)

cc_library(
    name = "node_lib",
    srcs = ["node.cc"],
    hdrs = ["node.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":clock_offset",
        ":messages",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/events:events_log",
        "//talOS/events:events_os",
        "//talOS/hardware",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/introspection:registry",
        "//talOS/ipc",
        "//talOS/process",
        "//talOS/protocol",
    ],
)

cc_binary(
    name = "hardware_node",
    srcs = ["main.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":node_lib",
        "//talOS/introspection:describe",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["hardware_node_test.cc"],
    data = [":hardware_node"],
    env = {"HARDWARE_NODE": "$(rlocationpath :hardware_node)"},
    deps = [
        ":messages",
        ":node_lib",
        "//talOS/driver_station",
        "//talOS/events:events_log",
        "//talOS/protocol:udp_test_helpers",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

cc_test(
    name = "clock_offset_test",
    size = "small",
    srcs = ["clock_offset_test.cc"],
    deps = [
        ":clock_offset",
        "@googletest//:gtest_main",
    ],
)

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

# The request merge rules (owner timeout, admission, merged observation),
# also run by talos::testing::SimHardware when it plays the bridge.
cc_library(
    name = "merge",
    srcs = ["merge.cc"],
    hdrs = ["merge.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "node_lib",
    srcs = ["node.cc"],
    hdrs = ["node.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":clock_offset",
        ":merge",
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
        "//talOS/rtms:wake",
    ],
)

cc_binary(
    name = "hardware_node",
    srcs = ["main.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":node_lib",
        "//talOS/events:events_log",
        "//talOS/introspection:describe",
    ],
)

# The bridge and fake gateway on loopback ports, the test configs, and the
# frames the two ends exchange, for the two test targets below.
cc_library(
    name = "node_test_support",
    testonly = True,
    hdrs = ["node_test_support.h"],
    deps = [
        ":node_lib",
        "//talOS/configuration",
        "//talOS/ipc",
        "//talOS/protocol",
        "//talOS/protocol:udp_test_helpers",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["hardware_node_test.cc"],
    data = [":hardware_node"],
    env = {"HARDWARE_NODE": "$(rlocationpath :hardware_node)"},
    tags = ["integration"],
    deps = [
        ":merge",
        ":messages",
        ":node_lib",
        ":node_test_support",
        "//talOS/driver_station",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/events:events_os",
        "//talOS/events:input_replay",
        "//talOS/introspection:registry",
        "//talOS/ipc",
        "//talOS/node:allocation_probe",
        "//talOS/protocol:udp_test_helpers",
        "//talOS/rtms",
        "//talOS/schema:identity",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# Run() on real time and loopback UDP: the integration tier
# (docs/TESTING.md §2), kept out of node_test's synthetic-time logic tests.
# Its bounds are ratios of counts, or a requirement's margin judged only on a
# host whose measured stalls are inside it (§6).
cc_test(
    name = "run_test",
    size = "medium",
    srcs = ["run_test.cc"],
    deps = [
        ":merge",
        ":messages",
        ":node_lib",
        ":node_test_support",
        "//talOS/introspection:registry",
        "//talOS/ipc",
        "//talOS/process",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
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

cc_test(
    name = "merge_test",
    size = "small",
    srcs = ["merge_test.cc"],
    deps = [
        ":merge",
        "@googletest//:gtest_main",
    ],
)

load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

package(default_visibility = ["//visibility:public"])

# The Driver Station wire codec (the bridge's packet payload).
cc_library(
    name = "driver_station",
    srcs = ["driver_station.cc"],
    hdrs = ["driver_station.h"],
)

cc_test(
    name = "driver_station_test",
    size = "small",
    srcs = ["driver_station_test.cc"],
    deps = [
        ":driver_station",
        "@googletest//:gtest_main",
    ],
)

# DriverStationState (namespace driver_station) and its topic. The operator
# interface, the arbiter and telemetry consume it.
talos_messages(
    name = "messages",
    srcs = ["driver_station_message.fbs"],
    hdrs = ["topics.h"],
)

cc_library(
    name = "driver_station_node",
    hdrs = ["driver_station_node.h"],
    deps = [
        ":driver_station",
        ":messages",
        "//talOS/hardware",
        "//talOS/node",
    ],
    # The arbiter's teleop-chain test runs the real driver station node.
    visibility = ["//2026-robot/main_processor/arbiter:__pkg__"],
)

talos_cc_node(
    name = "node",
    node_name = "driver_station",
    srcs = ["driver_station_node.cc"],
    deps = [":driver_station_node"],
)

cc_binary(
    name = "send_joystick",
    srcs = ["send_joystick.cc"],
    deps = [
        ":driver_station",
        "//talOS/events:events_os",
        "//talOS/hardware",
        "//talOS/ipc",
        "//talOS/process",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/robot.toml",
        ":node",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
        "TALOS_TEST_NODE": "$(rlocationpath :node)",
    },
    deps = [
        ":driver_station_node",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

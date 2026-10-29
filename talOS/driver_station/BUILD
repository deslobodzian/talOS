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

# DriverStationState (namespace driver_station) and its topic: the decoded
# Driver Station state, republished from the bridge's /hw/state/driver_station.
# The operator interface, the arbiter and telemetry consume it.
talos_messages(
    name = "messages",
    srcs = ["driver_station_message.fbs"],
    topics = {"state": "/driver_station/state"},
)

cc_library(
    name = "driver_station_node",
    hdrs = ["driver_station_node.h"],
    # The arbiter's teleop-chain test runs the real driver station node.
    visibility = ["//2026-robot/main_processor/arbiter:__pkg__"],
    deps = [
        ":driver_station",
        ":messages",
        "//talOS/devices",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["driver_station_node.cc"],
    node_name = "driver_station",
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
        "//talOS/hardware",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

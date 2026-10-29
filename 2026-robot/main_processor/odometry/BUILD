load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# OdometryState (namespace odometry) and the odometry topic: the fused field
# pose is odometry's own state. Telemetry consumes it.
talos_messages(
    name = "messages",
    srcs = ["odometry_message.fbs"],
    topics = {"state": "/odometry/state"},
)

cc_library(
    name = "odometry",
    hdrs = ["odometry.h"],
    include_prefix = "odometry",
    strip_include_prefix = ".",
    visibility = ["//visibility:private"],
    deps = [
        ":messages",
        "//2026-robot/main_processor/drivetrain:messages",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["odometry.cc"],
    node_name = "odometry",
    deps = [":odometry"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        ":node",
        "//2026-robot/main_processor/configuration:robot.toml",
        "//2026-robot/main_processor/configuration:robot_config",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath //2026-robot/main_processor/configuration:robot.toml)",
        "TALOS_TEST_NODE": "$(rlocationpath :node)",
    },
    deps = [
        ":odometry",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

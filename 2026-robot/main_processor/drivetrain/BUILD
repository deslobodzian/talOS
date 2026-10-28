load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# ChassisTarget / DrivetrainState (namespace drivetrain) and the drivetrain's
# topics. Odometry, the operator interface, telemetry and the arbiter consume
# them.
talos_messages(
    name = "messages",
    srcs = ["drive_message.fbs"],
    hdrs = ["topics.h"],
)

# The swerve kinematics and the geometry they run on, read from config.
cc_library(
    name = "swerve",
    hdrs = [
        "geometry.h",
        "swerve.h",
    ],
    strip_include_prefix = ".",
    include_prefix = "drivetrain",
    deps = [
        "//talOS/configuration:configuration",
        "//talOS/devices",
        "//talOS/node",
    ],
    visibility = ["//visibility:private"],
)

cc_library(
    name = "drivetrain",
    hdrs = ["drivetrain.h"],
    strip_include_prefix = ".",
    include_prefix = "drivetrain",
    deps = [
        ":messages",
        ":swerve",
        "//talOS/devices",
        "//talOS/node",
    ],
    # The arbiter's teleop-chain test runs the real drivetrain.
    visibility = ["//2026-robot/main_processor/arbiter:__pkg__"],
)

talos_cc_node(
    name = "node",
    node_name = "drivetrain",
    srcs = ["drivetrain.cc"],
    deps = [":drivetrain"],
)

# Dev tools: publish a chassis target by hand, and watch the drivetrain link.
cc_binary(
    name = "send_target",
    srcs = ["send_target.cc"],
    deps = [
        ":messages",
        "//talOS/events:events_os",
        "//talOS/ipc",
    ],
)

cc_binary(
    name = "monitor",
    srcs = ["monitor.cc"],
    data = ["//2026-robot/main_processor/configuration:robot_config"],
    deps = [
        ":messages",
        ":swerve",
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
        "testdata/drivetrain.toml",
        "testdata/robot.toml",
        ":node",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
        "TALOS_TEST_NODE": "$(rlocationpath :node)",
    },
    deps = [
        ":drivetrain",
        "//talOS/configuration:configuration",
        "//talOS/devices:testing",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

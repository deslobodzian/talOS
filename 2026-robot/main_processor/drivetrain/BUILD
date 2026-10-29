load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# ChassisTarget / DrivetrainState (namespace drivetrain) and the drivetrain's
# topics. Odometry, the operator interface, telemetry and the arbiter consume
# them. /drivetrain/target has one writer, the arbiter; teleop and autonomous
# publish on their own lanes and the arbiter picks one.
talos_messages(
    name = "messages",
    srcs = ["drive_message.fbs"],
    topics = {
        "state": "/drivetrain/state",
        "target": "/drivetrain/target",
        "teleop_target": "/drivetrain/target/teleop",
        "auto_target": "/drivetrain/target/auto",
    },
)

# The swerve kinematics and the geometry they run on, read from config.
cc_library(
    name = "swerve",
    hdrs = [
        "geometry.h",
        "swerve.h",
    ],
    include_prefix = "drivetrain",
    strip_include_prefix = ".",
    visibility = ["//visibility:private"],
    deps = [
        "//talOS/configuration",
        "//talOS/devices",
        "//talOS/node",
    ],
)

cc_library(
    name = "drivetrain",
    hdrs = ["drivetrain.h"],
    include_prefix = "drivetrain",
    strip_include_prefix = ".",
    # The arbiter's teleop-chain test runs the real drivetrain.
    visibility = ["//2026-robot/main_processor/arbiter:__pkg__"],
    deps = [
        ":messages",
        ":swerve",
        "//talOS/devices",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["drivetrain.cc"],
    node_name = "drivetrain",
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
        "//talOS/configuration",
        "//talOS/devices:testing",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

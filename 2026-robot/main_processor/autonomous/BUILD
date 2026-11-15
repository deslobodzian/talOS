load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# AutonomousState (namespace autonomous) and the autonomous node's topic. The
# telemetry node consumes /autonomous/state and forwards the phase enum as the
# `autonomous/state/phase` channel.
talos_messages(
    name = "messages",
    srcs = ["autonomous_message.fbs"],
    topics = {
        "state": "/autonomous/state",
    },
)

cc_library(
    name = "autonomous",
    hdrs = [
        "autonomous.h",
        "progress.h",
        "selection.h",
    ],
    include_prefix = "autonomous",
    strip_include_prefix = ".",
    deps = [
        ":messages",
        "//2026-robot/main_processor/drivetrain:messages",
        "//2026-robot/main_processor/intake:messages",
        "//2026-robot/main_processor/shooter:messages",
        "//talOS/configuration",
        "//talOS/devices",
        "//talOS/geometry",
        "//talOS/memory:shared_memory",
        "//talOS/msgs:geometry",
        "//talOS/node",
        "//talOS/trajectory",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["autonomous.cc"],
    node_name = "autonomous",
    deps = [":autonomous"],
)

# The drive action against the real drivetrain node on the robot config,
# with the test playing the pose estimator (closed_loop_test.cc).
cc_test(
    name = "closed_loop_test",
    size = "small",
    srcs = ["closed_loop_test.cc"],
    data = [
        "//2026-robot/main_processor/configuration:robot.toml",
        "//2026-robot/main_processor/configuration:robot_config",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath //2026-robot/main_processor/configuration:robot.toml)",
    },
    deps = [
        ":autonomous",
        "//2026-robot/main_processor/drivetrain",
        "//talOS/configuration",
        "//talOS/devices:testing",
        "//talOS/geometry",
        "//talOS/node:testing",
        "//talOS/trajectory",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
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
        ":autonomous",
        "//talOS/devices:testing",
        "//talOS/memory:shared_memory",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

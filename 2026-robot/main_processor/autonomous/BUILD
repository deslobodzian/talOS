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
        "trajectory.h",
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
        "//talOS/memory:shared_memory",
        "//talOS/msgs:geometry",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["autonomous.cc"],
    node_name = "autonomous",
    deps = [":autonomous"],
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
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node")

cc_library(
    name = "telemetry",
    hdrs = ["node.h"],
    include_prefix = "telemetry",
    strip_include_prefix = ".",
    visibility = ["//visibility:private"],
    deps = [
        "//2026-robot/main_processor/drivetrain:messages",
        "//2026-robot/main_processor/intake:messages",
        "//2026-robot/main_processor/odometry:messages",
        "//2026-robot/main_processor/shooter:messages",
        "//studio/schema",
        "//studio/schema:wire",
        "//talOS/driver_station",
        "//talOS/driver_station:messages",
        "//talOS/node",
        "@flatbuffers",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["node.cc"],
    node_name = "telemetry",
    deps = [":telemetry"],
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
        ":telemetry",
        "//studio/schema:wire",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

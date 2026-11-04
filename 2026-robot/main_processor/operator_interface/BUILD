load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node")

cc_library(
    name = "operator_interface",
    hdrs = ["operator_interface.h"],
    include_prefix = "operator_interface",
    strip_include_prefix = ".",
    visibility = ["//visibility:private"],
    deps = [
        "//2026-robot/main_processor/drivetrain:messages",
        "//2026-robot/main_processor/intake:messages",
        "//2026-robot/main_processor/shooter:messages",
        "//talOS/configuration",
        "//talOS/devices",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["operator_interface.cc"],
    node_name = "operator_interface",
    deps = [":operator_interface"],
)

# The operator interface on its own, plus the whole teleop chain (driver
# station, operator interface, drivetrain) on the real robot config.
cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "//2026-robot/main_processor/configuration:robot.toml",
        "//2026-robot/main_processor/configuration:robot_config",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath //2026-robot/main_processor/configuration:robot.toml)",
    },
    deps = [
        ":operator_interface",
        "//2026-robot/main_processor/drivetrain",
        "//talOS/configuration",
        "//talOS/devices:testing",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node")

cc_library(
    name = "arbiter",
    hdrs = ["arbiter.h"],
    include_prefix = "arbiter",
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
    srcs = ["arbiter.cc"],
    node_name = "arbiter",
    deps = [":arbiter"],
)

# Arbitration on its own, plus the whole teleop chain (driver station,
# operator interface, arbiter, drivetrain) on the real robot config.
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
        ":arbiter",
        "//2026-robot/main_processor/drivetrain",
        "//2026-robot/main_processor/operator_interface",
        "//talOS/devices:testing",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node")

cc_library(
    name = "arbiter",
    hdrs = ["arbiter.h"],
    strip_include_prefix = ".",
    include_prefix = "arbiter",
    deps = [
        "//2026-robot/main_processor/drivetrain:messages",
        "//2026-robot/main_processor/intake:messages",
        "//2026-robot/main_processor/shooter:messages",
        "//talOS/driver_station",
        "//talOS/driver_station:messages",
        "//talOS/node",
    ],
    visibility = ["//visibility:private"],
)

talos_cc_node(
    name = "node",
    node_name = "arbiter",
    srcs = ["arbiter.cc"],
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
        "//talOS/driver_station:driver_station_node",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

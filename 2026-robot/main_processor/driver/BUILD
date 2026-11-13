load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node")

cc_library(
    name = "driver",
    hdrs = ["driver.h"],
    include_prefix = "driver",
    strip_include_prefix = ".",
    visibility = ["//visibility:private"],
    deps = [
        "//2026-robot/main_processor/drivetrain:messages",
        "//talOS/devices",
        "//talOS/driver_station",
        "//talOS/node",
        "//talOS/teleop:deadband",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["driver.cc"],
    node_name = "driver",
    deps = [":driver"],
)

# The driver on its own, plus the whole teleop chain (driver
# station, driver, drivetrain) on the real robot config.
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
        ":driver",
        "//2026-robot/main_processor/drivetrain",
        "//talOS/configuration",
        "//talOS/devices:testing",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

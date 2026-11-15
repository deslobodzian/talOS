load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node")

cc_library(
    name = "driver",
    hdrs = ["driver.h"],
    include_prefix = "driver",
    strip_include_prefix = ".",
    # The teleop chain test runs the driver with the real drivetrain.
    visibility = ["//2026-robot/main_processor/integration:__pkg__"],
    deps = [
        "//2026-robot/main_processor/drivetrain:messages",
        "//talOS/configuration",
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

# The driver on its own; the teleop chain through the drivetrain is
# //2026-robot/main_processor/integration:teleop_chain_test.
cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":driver",
        "//2026-robot/main_processor/drivetrain:messages",
        "//talOS/devices:testing",
        "//talOS/driver_station",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
    ],
)

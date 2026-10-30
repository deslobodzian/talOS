load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node")

cc_library(
    name = "operator_interface",
    hdrs = ["operator_interface.h"],
    include_prefix = "operator_interface",
    strip_include_prefix = ".",
    # The arbiter's teleop-chain test runs the real operator interface.
    visibility = ["//2026-robot/main_processor/arbiter:__pkg__"],
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

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":operator_interface",
        "//talOS/devices:testing",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
    ],
)

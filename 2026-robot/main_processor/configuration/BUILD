load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_graph_test")

package(default_visibility = ["//visibility:public"])

exports_files(["robot.toml"])

exports_files(glob(["subsystems/*.toml"]))

# The whole robot config: the manifest plus every subsystem file. Any test
# that opens robot.toml must take this, not the manifest alone — sibling
# files resolve relative to robot.toml at parse time, in source and runfiles.
filegroup(
    name = "robot_config",
    srcs = [
        "robot.toml",
    ] + glob(["subsystems/*.toml"]),
)

cc_test(
    name = "config_golden_test",
    size = "small",
    srcs = ["config_golden_test.cc"],
    data = [":robot_config"],
    deps = [
        "//talOS/configuration",
        "//talOS/hardware:config",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

talos_graph_test(
    name = "graph_test",
    config = "robot.toml",
    data = [":robot_config"],
    nodes = [
        "//2026-robot/main_processor/arbiter:node",
        "//2026-robot/main_processor/drivetrain:node",
        "//2026-robot/main_processor/intake:node",
        "//2026-robot/main_processor/odometry:node",
        "//2026-robot/main_processor/operator_interface:node",
        "//2026-robot/main_processor/shooter:node",
        "//2026-robot/main_processor/telemetry:node",
    ],
)

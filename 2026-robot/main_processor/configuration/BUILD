load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

exports_files(["robot.toml", "subsystem.toml"])

# The whole robot config: the manifest plus every subsystem file. Any test
# that opens robot.toml must take this, not the manifest alone — sibling
# files resolve relative to robot.toml at parse time, in source and runfiles.
filegroup(
    name = "robot_config",
    srcs = [
        "robot.toml",
        "//2026-robot/main_processor/arbiter:subsystem.toml",
        "//2026-robot/main_processor/driver_station:subsystem.toml",
        "//2026-robot/main_processor/drivetrain:subsystem.toml",
        "//2026-robot/main_processor/intake:subsystem.toml",
        "//2026-robot/main_processor/odometry:subsystem.toml",
        "//2026-robot/main_processor/operator_interface:subsystem.toml",
        "//2026-robot/main_processor/shooter:subsystem.toml",
        "//2026-robot/main_processor/telemetry:subsystem.toml",
    ],
)

cc_test(
    name = "config_golden_test",
    size = "small",
    srcs = ["config_golden_test.cc"],
    data = [
        ":robot_config",
        "subsystem.toml",
    ],
    deps = [
        "//talOS/configuration:configuration",
        "//talOS/hardware:config",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

load("@rules_cc//cc:cc_test.bzl", "cc_test")

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
        "//talOS/configuration:configuration",
        "//talOS/hardware:config",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

exports_files(["robot.toml", "subsystem.toml"])

cc_library(
    name = "configuration",
    hdrs = [
        "config_parser.h",
        "configuration.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//common/hardware:config",
        "@tomlplusplus//:toml++",
    ],
)

cc_test(
    name = "test",
    size = "small",
    srcs = ["config_test.cc"],
    data = [
        "robot.toml",
        "subsystem.toml",
    ],
    deps = [
        ":configuration",
        "//common/hardware:config",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

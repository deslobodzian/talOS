load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "launcher_lib",
    hdrs = ["launcher.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/configuration:configuration",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
    ],
)

cc_binary(
    name = "launcher",
    srcs = ["main.cc"],
    visibility = ["//visibility:public"],
    deps = [":launcher_lib"],
)

cc_binary(
    name = "log_dump",
    srcs = ["log_dump.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":launcher_lib",
        "//talOS/events:events_core",
    ],
)

cc_test(
    name = "launcher_test",
    size = "small",
    srcs = ["launcher_test.cc"],
    data = ["//talOS/configuration:robot.toml"],
    deps = [
        ":launcher_lib",
        "//talOS/events:events_log",
        "@googletest//:gtest_main",
    ],
)

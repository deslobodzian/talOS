load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "odometry_message",
    srcs = ["odometry_message.fbs"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "odometry",
    hdrs = [
        "node.h",
        "packet.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":odometry_message",
        "//talOS/drivetrain:drive_message",
        "//talOS/events:events",
        "//talOS/shooter:shooter_message",
    ],
)

cc_binary(
    name = "node",
    srcs = ["main.cc"],
    deps = [":odometry"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":odometry",
        "@googletest//:gtest_main",
    ],
)

test_suite(
    name = "...",
    tests = [":node_test"],
)


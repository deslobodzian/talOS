load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

# The subsystem's own config file, so the robot manifest filegroup can ship it.
exports_files(["subsystem.toml"])
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
        # The topic constants as well as the message types: a consumer that
        # spells a topic name itself is a second declaration to keep in step by
        # hand, which is the failure talOS/NAMING.md exists to prevent.
        "//2026-robot/main_processor/drivetrain:drivetrain",
        "//2026-robot/main_processor/drivetrain:drive_message",
        "//2026-robot/main_processor/shooter:shooter",
        "//2026-robot/main_processor/shooter:shooter_message",
        "//talOS/events:events",
    ],
)

cc_binary(
    name = "node",
    srcs = ["main.cc"],
    deps = [
        ":odometry",
        "//talOS/events:events",
        "//talOS/introspection:describe",
        "//talOS/introspection:reporter",
    ],
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


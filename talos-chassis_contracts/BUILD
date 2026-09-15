load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

exports_files(["drive_message.fbs"])

# Neutral chassis frame contract: the robot-relative chassis request and the
# drivetrain state every season reuses. Year mechanisms (drivetrain, intake,
# shooter, arbiter) live per season under 2026-robot/main_processor/ and are
# rewritten each game; these frames are the stable platform surface that
# odometry, the operator interface core, and the telemetry core consume.
# There is exactly one definition of each frame type: the year drivetrain
# package re-exports this contract instead of defining its own copy
# (duplicate FlatBuffers types under one include path fail the build).
flatbuffer_cc_library(
    name = "drive_message",
    srcs = ["drive_message.fbs"],
)

# Re-exported under the long-standing talos_nodes/drivetrain prefix, so
# consumer includes are unchanged by the move to this neutral home.
cc_library(
    name = "contract",
    hdrs = ["packet.h", ":drive_message_srcs"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/drivetrain",
    deps = [
        ":drive_message",
        "@talos//talOS/hardware:hardware",
    ],
)

alias(
    name = "messages_cc",
    actual = ":contract",
)

cc_test(
    name = "contract_test",
    size = "small",
    srcs = ["contract_test.cc"],
    deps = [
        ":contract",
        "@googletest//:gtest_main",
    ],
)

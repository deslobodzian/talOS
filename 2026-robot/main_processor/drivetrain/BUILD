load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "drive_message",
    srcs = ["drive_message.fbs"],
    visibility = ["//visibility:public"],
)
cc_library(
    name = "drivetrain",
    hdrs = ["packet.h", "swerve.h", "geometry.h", "node.h"],
    deps = [
        ":drive_message",
        "//talOS/hardware:hardware",
        "//talOS/protocol:protocol",
        "//talOS/configuration:configuration",
        "//talOS/events:events",
        "//talOS/process:process",
    ],
    visibility = ["//visibility:public"],
)
cc_binary(name = "node", srcs = ["main.cc"], deps = [":drivetrain"])
cc_binary(name = "sim_gateway", srcs = ["sim_gateway.cc"], deps = [":drivetrain", "//talOS/hardware:hardware"])
cc_binary(name = "send_target", srcs = ["send_target.cc"], deps = [":drivetrain", "//talOS/ipc:ipc"])
cc_binary(
    name = "monitor",
    srcs = ["monitor.cc"],
    data = ["//2026-robot/main_processor/configuration:robot.toml"],
    deps = [":drivetrain", "//talOS/ipc:ipc"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = ["//2026-robot/main_processor/configuration:robot.toml"],
    deps = [":drivetrain", "@googletest//:gtest_main"],
)

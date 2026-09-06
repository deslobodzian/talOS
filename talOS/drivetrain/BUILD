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
    hdrs = ["packet.h", "swerve.h", "node.h", "process.h"],
    deps = [
        ":drive_message",
        "//common/hardware:hardware",
        "//common/protocol:protocol",
        "//talOS/configuration:configuration",
        "//talOS/events:events",
    ],
    visibility = ["//visibility:public"],
)
cc_binary(name = "node", srcs = ["main.cc"], deps = [":drivetrain"])
cc_binary(name = "bridge", srcs = ["//talOS/hardware:main.cc"], deps = ["//talOS/hardware:node_lib"])
cc_binary(name = "sim_gateway", srcs = ["sim_gateway.cc"], deps = [":drivetrain", "//common/hardware:hardware"])
cc_binary(name = "send_target", srcs = ["send_target.cc"], deps = [":drivetrain", "//talOS/ipc:ipc"])
cc_binary(name = "monitor", srcs = ["monitor.cc"], deps = [":drivetrain", "//talOS/ipc:ipc"])

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = ["//talOS/configuration:robot.toml"],
    deps = [":drivetrain", "@googletest//:gtest_main"],
)

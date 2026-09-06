load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

# The subsystem's own config file, so the robot manifest filegroup can ship it.
exports_files(["subsystem.toml"])
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "shooter_message",
    srcs = ["shooter_message.fbs"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "shooter",
    hdrs = ["packet.h", "node.h"],
    deps = [
        ":shooter_message",
        "//talOS/hardware:hardware",
        "//talOS/protocol:protocol",
        "//talOS/configuration:configuration",
        "//talOS/events:events",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "node",
    srcs = ["main.cc"],
    deps = [
        ":shooter",
        "//talOS/events:events",
        "//talOS/introspection:describe",
        "//talOS/introspection:reporter",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = ["//2026-robot/main_processor/configuration:robot_config"],
    deps = [
        ":shooter",
        "@googletest//:gtest_main",
    ],
)

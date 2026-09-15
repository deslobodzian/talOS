load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

# The subsystem's own config file, so the robot manifest filegroup can ship it.
exports_files(["subsystem.toml"])
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

cc_library(
    name = "messages_cc",
    hdrs = ["packet.h", ":shooter_message_srcs"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/shooter",
    visibility = ["//visibility:public"],
    deps = [
        ":shooter_message",
        "//talOS/hardware:hardware",
    ],
)

flatbuffer_cc_library(
    name = "shooter_message",
    srcs = ["shooter_message.fbs"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "shooter",
    hdrs = ["node.h"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/shooter",
    deps = [
        ":messages_cc",
        "//talOS/hardware:hardware",
        "//talOS/hardware:gate",
        "//talOS/protocol:protocol",
        "//talOS/configuration:configuration",
        "//talOS/events:events",
    ],
    visibility = ["//visibility:private"],
)

cc_binary(
    name = "node",
    srcs = ["main.cc"],
    deps = [
        ":shooter",
        "//talOS/process:hardware_main",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/robot.toml",
        "testdata/shooter.toml",
    ],
    env = {"TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)"},
    deps = [
        ":shooter",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

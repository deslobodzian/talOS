load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
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
        "//common/hardware:hardware",
        "//common/protocol:protocol",
        "//talOS/configuration:configuration",
        "//talOS/events:events",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(name = "node", srcs = ["main.cc"], deps = [":shooter"])

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = ["//talOS/configuration:robot.toml"],
    deps = [
        ":shooter",
        "@googletest//:gtest_main",
    ],
)

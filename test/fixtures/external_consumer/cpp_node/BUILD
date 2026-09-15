load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "sample_node",
    hdrs = ["node.h"],
    deps = [
        "@talos//talOS/events:events",
        "@talos//talOS/hardware:gate",
    ],
)

cc_binary(
    name = "node",
    srcs = ["main.cc"],
    deps = [
        ":sample_node",
        "@talos//talOS/process:node_main",
    ],
)

cc_test(
    name = "sample_node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":sample_node",
        "@googletest//:gtest_main",
    ],
)

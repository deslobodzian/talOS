load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

exports_files(["main.cc"])

cc_library(
    name = "node_lib",
    srcs = ["node.cc"],
    hdrs = ["node.h"],
    deps = [
        "//talOS/hardware:hardware",
        "//talOS/protocol:protocol",
        "//talOS/configuration:configuration",
        "//talOS/ipc:ipc",
        "//talOS/process:process",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "node",
    srcs = ["main.cc"],
    deps = [":node_lib"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "hardware_node",
    srcs = ["main.cc"],
    deps = [":node_lib"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["hardware_node_test.cc"],
    deps = [
        ":node_lib",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

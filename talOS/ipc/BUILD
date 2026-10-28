load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

flatbuffer_cc_library(
    name = "ipc_test_message",
    srcs = ["ipc_test_message.fbs"],
)

cc_library(
    name = "ipc",
    hdrs = ["publisher.h", "subscriber.h", "concepts.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/rtms:rtms",
        "@flatbuffers",
    ],
)

cc_test(
    name = "ipc_test",
    size = "small",
    srcs = ["test.cc"],
    deps = [
        ":ipc",
        ":ipc_test_message",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ]
)

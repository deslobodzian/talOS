load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

flatbuffer_cc_library(
    name = "test_message_fbs",
    srcs = ["test_message.fbs"],
)

cc_library(
    name = "rtms",
    srcs = ["rtms.cc"],
    hdrs = ["rtms.h"],
    visibility = ["//visibility:public"],
    deps = ["//talOS/memory:shared_memory"],
)

cc_test(
    name = "rtms_test",
    size = "small",
    srcs = ["test.cc"],
    deps = [
        ":rtms",
        ":test_message_fbs",
        "@flatbuffers",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

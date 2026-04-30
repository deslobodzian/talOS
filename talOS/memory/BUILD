load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "ring_buffer",
    hdrs = ["ring_buffer.h"],
)

cc_test(
    name = "ring_buffer_test",
    size = "small",
    srcs = ["ring_buffer_test.cc"],
    deps = [
        ":ring_buffer",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

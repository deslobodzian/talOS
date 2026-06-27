load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "shared_memory",
    hdrs = ["shared_memory_ptr.h"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "shared_memory_test",
    size = "small",
    srcs = ["shared_memory_test.cc"],
    deps = [
        ":shared_memory",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

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

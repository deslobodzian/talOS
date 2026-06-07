load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "shared_memory",
    hdrs = ["shared_memory.h"],
)

cc_test(
    name = "shared_memory_test",
    size = "small",
    srcs = ["test.cc"],
    deps = [
        ":shared_memory",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

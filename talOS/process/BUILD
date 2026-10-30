load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "process",
    srcs = ["process.cc"],
    hdrs = ["process.h"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "process_test",
    size = "small",
    srcs = ["process_test.cc"],
    deps = [
        ":process",
        "@googletest//:gtest_main",
    ],
)

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "process",
    srcs = ["process.cc"],
    hdrs = ["process.h"],
    visibility = ["//visibility:public"],
    deps = ["//talOS/memory:residency"],
)

cc_library(
    name = "durable_sync",
    hdrs = ["durable_sync.h"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "process_test",
    size = "small",
    srcs = ["process_test.cc"],
    deps = [
        ":process",
        "//talOS/memory:residency",
        "@googletest//:gtest_main",
    ],
)

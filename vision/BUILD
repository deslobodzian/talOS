load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "device",
    hdrs = ["device.h", "cpu_device.h"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_library(
    name = "buffer",
    hdrs = ["buffer.h"],
    visibility = ["//visibility:public"],
    deps = [":device"],
)


cc_test(
    name = "device_test",
    size = "small",
    srcs = ["device_test.cc"],
    deps = [
        ":device",
        ":buffer",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)


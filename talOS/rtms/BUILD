load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "rtms",
    hdrs = ["rtms.h"],
)

cc_test(
    name = "rtms_test",
    size = "small",
    srcs = ["test.cc"],
    deps = [
        ":rtms",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ]
)


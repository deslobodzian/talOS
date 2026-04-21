load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_binary(
    name = "main",
    srcs = ["main.cc"],
    deps = []
)

cc_test(
    name = "test",
    size = "small",
    srcs = ["test.cc"],
    deps = [
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

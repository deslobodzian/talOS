load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "configuration",
    hdrs = ["configuration.h"],
    deps = [
        "@tomlplusplus//:toml++",
    ],
)

cc_test(
    name = "test",
    size = "small",
    srcs = ["config_test.cc"],
    deps = [
        ":configuration",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

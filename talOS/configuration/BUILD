load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "configuration",
    hdrs = [
        "config_parser.h",
        "configuration.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/hardware:config",
        "@tomlplusplus//:toml++",
    ],
)

cc_test(
    name = "test",
    size = "small",
    srcs = ["config_test.cc"],
    deps = [
        ":configuration",
        "//talOS/hardware:config",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

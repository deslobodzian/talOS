load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "protocol",
    srcs = [
        "frame.cc",
        "udp.cc",
    ],
    hdrs = [
        "frame.h",
        "udp.h",
        "types.h",
    ],
    linkopts = select({
        "@platforms//os:windows": ["ws2_32.lib"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
)

cc_test(
    name = "frame_test",
    size = "small",
    srcs = ["frame_test.cc"],
    deps = [
        ":protocol",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "udp_test",
    size = "small",
    srcs = ["udp_test.cc"],
    deps = [
        ":protocol",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

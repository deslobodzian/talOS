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
        "types.h",
        "udp.h",
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
        ":udp_test_helpers",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "udp_test_helpers",
    testonly = True,
    hdrs = ["udp_test_helpers.h"],
    visibility = ["//visibility:public"],
    deps = [":protocol"],
)

cc_library(
    name = "fuzz_corpus",
    testonly = True,
    hdrs = ["fuzz_corpus.h"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "frame_fuzz_test",
    size = "small",
    srcs = ["frame_fuzz_test.cc"],
    deps = [
        ":fuzz_corpus",
        ":protocol",
        "@googletest//:gtest_main",
    ],
)

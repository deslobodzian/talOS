load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
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
        "network_util.h",
        "motor_state.h",
    ],
    includes = ["include"],
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

cc_binary(
    name = "udp_perf",
    srcs = ["udp_perf.cc"],
    deps = [":protocol"],
)

cc_binary(
    name = "sim_udp_smoke",
    srcs = ["sim_udp_smoke.cc"],
    deps = [":protocol"],
)

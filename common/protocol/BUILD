load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "protocol",
    srcs = [
        "src/frame.cc",
        "src/runtime_udp.cc",
    ],
    hdrs = [
        "include/talos/protocol/frame.h",
        "include/talos/protocol/runtime_udp.h",
        "include/talos/protocol/types.h",
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
    srcs = ["test/frame_test.cc"],
    deps = [
        ":protocol",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "runtime_udp_test",
    size = "small",
    srcs = ["test/runtime_udp_test.cc"],
    deps = [
        ":protocol",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_binary(
    name = "udp_perf",
    srcs = ["tools/udp_perf.cc"],
    deps = [":protocol"],
)

cc_binary(
    name = "sim_udp_smoke",
    srcs = ["tools/sim_udp_smoke.cc"],
    deps = [":protocol"],
)

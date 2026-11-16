load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "process",
    srcs = ["process.cc"],
    hdrs = ["process.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":realtime_outcome",
        "//talOS/memory:residency",
    ],
)

# The loop tier and ConfigureRealtime's outcome, without the code that applies
# it: the live registry carries it in each node's row, and its readers (the
# launcher, Studio's bridge) need only the types.
cc_library(
    name = "realtime_outcome",
    hdrs = ["realtime_outcome.h"],
    visibility = ["//visibility:public"],
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

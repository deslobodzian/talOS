load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "events",
    hdrs = ["epoll.h", "timerfd.h", "event_loop.h"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "event_test",
    size = "small",
    srcs = ["test.cc"],
    deps = [
        ":events",
        "//talOS/ipc:ipc",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ]
)


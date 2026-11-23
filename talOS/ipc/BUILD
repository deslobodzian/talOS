load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_messages")

# Typed endpoints need a schema descriptor, so the test message comes from
# talos_messages like every node's.
talos_messages(
    name = "ipc_test_message",
    srcs = ["ipc_test_message.fbs"],
    include_prefix = "ipc_test",
    visibility = ["//visibility:private"],
)

# Message-type concept only (no RTMS dependency), so talOS/node exposes
# talos::Message without bringing raw IPC/RTMS headers into node compiles.
cc_library(
    name = "concepts",
    hdrs = ["concepts.h"],
    visibility = ["//visibility:public"],
    deps = ["@flatbuffers"],
)

cc_library(
    name = "ipc",
    hdrs = [
        "publisher.h",
        "subscriber.h",
    ],
    visibility = [
        "//bench/main_processor/swerve_module:__pkg__",
        "//studio:__subpackages__",
        "//talOS:__subpackages__",
        "//tools:__pkg__",
    ],
    deps = [
        ":concepts",
        "//talOS/rtms",
        "//talOS/schema:identity",
        "@flatbuffers",
    ],
)

cc_test(
    name = "ipc_test",
    size = "small",
    srcs = ["test.cc"],
    deps = [
        ":ipc",
        ":ipc_test_message",
        "//talOS/rtms:wake",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

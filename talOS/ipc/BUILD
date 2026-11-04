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

cc_library(
    name = "ipc",
    hdrs = [
        "concepts.h",
        "publisher.h",
        "subscriber.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
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
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

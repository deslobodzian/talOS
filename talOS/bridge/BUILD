load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_messages")

# BridgeStatus (namespace bridge) and its topic: hardware_node's health,
# published every 20 ms on /hw/status.
talos_messages(
    name = "messages",
    srcs = ["bridge_status.fbs"],
    include_prefix = "bridge",
    topics = {"status": "/hw/status"},
)

cc_library(
    name = "node_lib",
    srcs = ["node.cc"],
    hdrs = ["node.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":messages",
        "//talOS/configuration",
        "//talOS/hardware",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/introspection:registry",
        "//talOS/ipc",
        "//talOS/process",
        "//talOS/protocol",
    ],
)

cc_binary(
    name = "hardware_node",
    srcs = ["main.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":node_lib",
        "//talOS/introspection:describe",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["hardware_node_test.cc"],
    deps = [
        ":messages",
        ":node_lib",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

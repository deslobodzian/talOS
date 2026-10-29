load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "node_lib",
    srcs = ["node.cc"],
    hdrs = ["node.h"],
    visibility = ["//visibility:public"],
    deps = [
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
        ":node_lib",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

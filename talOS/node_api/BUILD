load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "internal",
    hdrs = ["internal.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":node_api_h",
        "//talOS/node",
    ],
)

cc_library(
    name = "node_api_h",
    hdrs = ["node_api.h"],
    visibility = ["//visibility:private"],
)

# C ABI over talos::Node and the devices layer, for nodes written in other
# languages (//talOS/python:talos).
cc_library(
    name = "node_api",
    srcs = ["node_api.cc"],
    hdrs = ["node_api.h"],
    deps = [
        ":internal",
        "//talOS/devices",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/node",
    ],
    # Every exported symbol must reach the shared objects below, even though
    # nothing in them references them.
    alwayslink = True,
)

# The Sim / SimHardware section of node_api.h.
cc_library(
    name = "node_api_testing",
    testonly = True,
    srcs = ["node_api_testing.cc"],
    deps = [
        ":internal",
        ":node_api",
        "//talOS/devices:testing",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/node:testing",
    ],
    alwayslink = True,
)

# Loaded via ctypes by the `talos` Python package.
cc_binary(
    name = "libtalos_node.so",
    linkshared = True,
    deps = [":node_api"],
)

cc_binary(
    name = "libtalos_node_testing.so",
    testonly = True,
    linkshared = True,
    deps = [":node_api_testing"],
)

cc_test(
    name = "node_api_test",
    size = "small",
    srcs = ["node_api_test.cc"],
    deps = [
        ":node_api_testing",
        "@googletest//:gtest_main",
    ],
)

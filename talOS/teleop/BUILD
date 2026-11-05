load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "deadband",
    hdrs = ["deadband.h"],
)

cc_library(
    name = "teleop",
    hdrs = ["node.h"],
    deps = [
        ":deadband",
        "//talOS/configuration",
        "//talOS/devices",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/node",
        "//talOS/schema:identity",
        "//talOS/schema:schema_hash",
        "@flatbuffers//:runtime_cc",
    ],
)
cc_binary(
    name = "libteleop.so",
    srcs = ["teleop_abi.cc"],
    linkshared = True,
    deps = [":teleop"],
)


talos_cc_node(
    name = "node",
    srcs = ["node.cc"],
    node_name = "teleop",
    deps = [":teleop"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":deadband",
        ":teleop",
        "//talOS/devices",
        "//talOS/devices:testing",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/msgs:control",
        "//talOS/node:testing",
        "//talOS/schema:test_messages",
        "@googletest//:gtest_main",
    ],
)

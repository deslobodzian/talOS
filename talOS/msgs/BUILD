load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_messages")

package(default_visibility = ["//visibility:public"])

# Shared geometry and control message contracts (ROS *_msgs style), so
# subsystems, arbiters, odometry, and operator interfaces can interoperate
# without depending on each other's packages or repos.
talos_messages(
    name = "geometry",
    srcs = ["geometry.fbs"],
    include_prefix = "geometry",
)

talos_messages(
    name = "control",
    srcs = ["control.fbs"],
    include_prefix = "control",
)

cc_test(
    name = "msgs_test",
    size = "small",
    srcs = ["msgs_test.cc"],
    deps = [
        ":control",
        ":geometry",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
    ],
)

py_test(
    name = "msgs_py_test",
    size = "small",
    srcs = ["msgs_py_test.py"],
    legacy_create_init = False,
    deps = [
        ":control_py",
        ":geometry_py",
        "//talOS/python:testing",
    ],
)

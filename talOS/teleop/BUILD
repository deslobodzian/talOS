load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_cc_shared_node")

package(default_visibility = ["//visibility:private"])

cc_library(
    name = "deadband",
    hdrs = ["deadband.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "teleop",
    hdrs = ["node.h"],
    visibility = [
        "//talOS:__subpackages__",
        "//tools:__pkg__",
    ],
    deps = [
        ":deadband",
        "//talOS/configuration",
        "//talOS/devices",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/node",
        "//talOS/node:access",
        "//talOS/schema:field_path",
        "//talOS/schema:identity",
        "//talOS/schema:schema_hash",
        "@flatbuffers//:runtime_cc",
    ],
)

# Test helper for robot/bench Sim tests to add the declarative teleop node
# without exposing talOS/teleop/node.h or talOS/node/access.h.
cc_library(
    name = "testing",
    testonly = True,
    srcs = ["testing.cc"],
    hdrs = ["testing.h"],
    implementation_deps = [
        ":teleop",
        "//talOS/node:testing",
    ],
    visibility = ["//visibility:public"],
)

talos_cc_node(
    name = "node",
    srcs = ["node.cc"],
    node_name = "teleop",
    visibility = ["//visibility:public"],
    deps = [":teleop"],
)

talos_cc_shared_node(
    name = "libteleop.so",
    srcs = ["teleop_abi.cc"],
    visibility = ["//talOS:__subpackages__"],
    deps = [":teleop"],
)

py_test(
    name = "python_test",
    size = "small",
    srcs = ["teleop_test.py"],
    legacy_create_init = False,
    main = "teleop_test.py",
    # ctypes cannot initialize sanitizer runtimes inside the Python host.
    tags = [
        "no_asan",
        "no_tsan",
    ],
    deps = [
        "//talOS/msgs:control_py",
        "//talOS/python:teleop",
        "//talOS/python:testing",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        ":teleop",
        "//talOS/devices",
        "//talOS/devices:testing",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/msgs:control",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "//talOS/schema:field_path",
        "//talOS/schema:test_messages",
        "@googletest//:gtest_main",
    ],
)

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_cc_shared_node")

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
        "//talOS/schema:field_path",
        "//talOS/schema:identity",
        "//talOS/schema:schema_hash",
        "@flatbuffers//:runtime_cc",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["node.cc"],
    node_name = "teleop",
    deps = [":teleop"],
)

talos_cc_shared_node(
    name = "libteleop.so",
    srcs = ["teleop_abi.cc"],
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
        ":deadband",
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

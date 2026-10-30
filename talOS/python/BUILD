load("@rules_python//python:defs.bzl", "py_library", "py_test")
load("//talOS/build:contract.bzl", "talos_messages")

# `import talos`: Python nodes over the C++ talos::Node and devices layer,
# through //talOS/node_api:libtalos_node.so.
py_library(
    name = "talos",
    srcs = [
        "talos/__init__.py",
        "talos/_abi.py",
        "talos/_messages.py",
        "talos/devices.py",
    ],
    data = ["//talOS/node_api:libtalos_node.so"],
    imports = ["."],
    visibility = ["//visibility:public"],
    deps = ["@pypi//flatbuffers"],
)

# `talos.testing`: Sim and SimHardware.
py_library(
    name = "testing",
    testonly = True,
    srcs = ["talos/testing.py"],
    data = ["//talOS/node_api:libtalos_node_testing.so"],
    imports = ["."],
    visibility = ["//visibility:public"],
    deps = [":talos"],
)

talos_messages(
    name = "test_messages",
    srcs = ["testdata/talos_test_message.fbs"],
    visibility = ["//visibility:private"],
)

py_test(
    name = "talos_test",
    size = "small",
    srcs = ["talos_test.py"],
    legacy_create_init = False,
    deps = [
        ":test_messages_py",
        ":testing",
    ],
)

# The struct.Struct message codec against flatbuffers.Builder: identical
# bytes, round trips, the Builder fallback, and the speedup.
py_test(
    name = "messages_test",
    size = "small",
    srcs = ["messages_test.py"],
    legacy_create_init = False,
    deps = [
        ":talos",
        ":test_messages_py",
        "//talOS/msgs:control_py",
        "//talOS/msgs:geometry_py",
        "@pypi//flatbuffers",
    ],
)

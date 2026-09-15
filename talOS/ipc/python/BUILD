load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_library", "py_test")

# Python RTMS pub/sub client (PLAN section 10, step 5). New package only:
# no C++ target or existing BUILD file is modified for this package.

py_library(
    name = "rtms_py",
    visibility = ["//visibility:public"],
    srcs = [
        "IPCMessage/TestMessage.py",
        "IPCMessage/TimeMessage.py",
        "IPCMessage/__init__.py",
        "messages.py",
        "talos_runtime/__init__.py",
        "talos_runtime/hardware_node.py",
        "talos_runtime/node_api.py",
        "talos_runtime/rtms.py",
        "talos_runtime/runner.py",
    ],
    imports = ["."],
    data = [":native_runtime"],
    deps = [
        "@pypi//flatbuffers",
        "@rules_python//python/runfiles",
    ],
)

genrule(
    name = "native_runtime",
    srcs = ["//talOS/node_api:libtalos_node.so"],
    outs = ["talos_runtime/libtalos_node.so"],
    cmd = "cp $(location //talOS/node_api:libtalos_node.so) $@",
)

# C++ peer for the byte-identical round-trip test. Additive target in this
# package only; reuses //talOS/ipc without touching it.
cc_binary(
    name = "peer",
    srcs = ["peer.cc"],
    deps = [
        "//talOS/ipc:ipc",
        "//talOS/ipc:ipc_test_message",
    ],
)

py_test(
    name = "rtms_roundtrip_test",
    size = "small",
    srcs = ["rtms_roundtrip_test.py"],
    data = [
        ":peer",
        "//talOS/node_api:libtalos_node.so",
    ],
    tags = ["local"],
    deps = [":rtms_py"],
)

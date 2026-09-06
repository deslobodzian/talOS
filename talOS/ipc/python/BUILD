load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_library", "py_test")

# Python RTMS pub/sub client (PLAN section 10, step 5). New package only:
# no C++ target or existing BUILD file is modified for this package.

py_library(
    name = "rtms_py",
    srcs = [
        "IPCMessage/TestMessage.py",
        "IPCMessage/TimeMessage.py",
        "IPCMessage/__init__.py",
        "messages.py",
        "node_api.py",
        "rtms.py",
    ],
    deps = ["@pypi//flatbuffers"],
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

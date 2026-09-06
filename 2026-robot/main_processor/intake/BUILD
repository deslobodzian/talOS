load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

# The subsystem's own config file, so the robot manifest filegroup can ship it.
# Added by a later track alongside subsystem.toml; kept first so appends merge cleanly.
exports_files(["subsystem.toml"])

flatbuffer_cc_library(
    name = "intake_message",
    srcs = ["intake_message.fbs"],
    visibility = ["//visibility:public"],
)
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

# Python intake node (PLAN rev 2, units 3-4). Additive only: the C++
# flatbuffer target and subsystem.toml export above are untouched.

py_library(
    name = "intake_bindings",
    srcs = [
        "talos/__init__.py",
        "talos/intake/IntakeState.py",
        "talos/intake/IntakeTarget.py",
        "talos/intake/__init__.py",
    ],
    imports = ["."],
    deps = ["@pypi//flatbuffers"],
)

py_library(
    name = "intake_py",
    srcs = [
        "node.py",
        "packet.py",
    ],
    imports = [
        ".",
        "talOS/ipc/python",
    ],
    deps = [
        ":intake_bindings",
        "//talOS/ipc/python:rtms_py",
    ],
)

py_binary(
    name = "node",
    srcs = ["main.py"],
    main = "main.py",
    imports = [
        ".",
        "talOS/ipc/python",
    ],
    data = [
        "subsystem.toml",
        "//talOS/node_api:libtalos_node.so",
    ],
    deps = [
        ":intake_bindings",
        ":intake_py",
        "//talOS/ipc/python:rtms_py",
    ],
)

py_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.py"],
    imports = [
        ".",
        "talOS/ipc/python",
    ],
    data = [
        "main.py",
        "subsystem.toml",
        "//talOS/node_api:libtalos_node.so",
    ],
    deps = [
        ":intake_bindings",
        ":intake_py",
        "//talOS/ipc/python:rtms_py",
    ],
)

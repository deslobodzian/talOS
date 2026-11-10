load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_library", "py_test")
load("//talOS/build:contract.bzl", "talos_messages")

# The pip lock, read by //2026-robot/controller_processor/rio:wpilib_lockstep_test.
exports_files(["requirements.txt"])

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
    deps = ["//talOS/schema:reflection_py"],
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

# Optional framework teleop node, backed by the same implementation as C++.
py_library(
    name = "teleop",
    srcs = ["talos/teleop.py"],
    data = ["//talOS/teleop:libteleop.so"],
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
    # node_api.cc's static_asserts are the struct layouts the ctypes mirror
    # is checked against.
    data = ["//talOS/node_api:node_api.cc"],
    env = {"TALOS_ABI_SOURCE": "$(rlocationpath //talOS/node_api:node_api.cc)"},
    legacy_create_init = False,
    # ctypes cannot initialize sanitizer runtimes inside the Python host.
    tags = [
        "no_asan",
        "no_tsan",
    ],
    deps = [
        ":test_messages_py",
        ":testing",
        "@rules_python//python/runfiles",
    ],
)

py_test(
    name = "api_parity_test",
    size = "small",
    srcs = ["api_parity_test.py"],
    data = [
        "talos/_abi.py",
        "talos/devices.py",
        "//talOS/node_api:node_api.h",
    ],
    env = {
        "TALOS_ABI_DEVICES": "$(rlocationpath talos/devices.py)",
        "TALOS_ABI_HEADER": "$(rlocationpath //talOS/node_api:node_api.h)",
        "TALOS_ABI_PYTHON": "$(rlocationpath talos/_abi.py)",
    },
    # A generated empty talos/__init__.py here would collide with the tracked
    # one in every other test's runfiles (see tools/check_runfiles_collisions.py).
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)

# Prints the flatc C++ bytes of fixed //talOS/schema:test_messages values,
# which messages_test compares the Python codec against.
cc_binary(
    name = "messages_golden",
    testonly = True,
    srcs = ["messages_golden.cc"],
    deps = ["//talOS/schema:test_messages"],
)

# The reflection-driven struct.Struct codec: byte-for-byte against flatc C++
# (nested structs, fixed arrays, enums of two widths, mixed alignment) and
# flatc Python, round trips, and bad-field errors.
py_test(
    name = "messages_test",
    size = "small",
    srcs = ["messages_test.py"],
    data = [":messages_golden"],
    env = {"TALOS_MESSAGES_GOLDEN": "$(rlocationpath :messages_golden)"},
    legacy_create_init = False,
    # Imports talos, loading instrumented ctypes libraries in a Python host.
    tags = [
        "no_asan",
        "no_tsan",
    ],
    deps = [
        ":talos",
        ":test_messages_py",
        "//talOS/msgs:control_py",
        "//talOS/msgs:geometry_py",
        "//talOS/schema:test_messages_py",
        "@pypi//flatbuffers",
        "@rules_python//python/runfiles",
    ],
)

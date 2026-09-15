load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

# The subsystem's own config file, so the robot manifest filegroup can ship it.
# Added by a later track alongside subsystem.toml; kept first so appends merge cleanly.
exports_files(["subsystem.toml"])

flatbuffer_cc_library(
    name = "intake_message",
    srcs = ["intake_message.fbs"],
    visibility = ["//visibility:public"],
)

# Public contracts are independent of the Python node implementation.
cc_library(
    name = "messages_cc",
    hdrs = ["packet.h", ":intake_message_srcs"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/intake",
    visibility = ["//visibility:public"],
    deps = [
        ":intake_message",
        "//talOS/hardware:hardware",
    ],
)

# Compatibility for existing external consumers during the migration.
alias(
    name = "intake",
    actual = ":messages_cc",
    visibility = ["//visibility:public"],
)
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

# Python intake node (PLAN rev 2, units 3-4). Additive only: the C++
# flatbuffer target and subsystem.toml export above are untouched.

py_library(
    name = "intake_bindings",
    srcs = [
        "talos/intake/IntakeState.py",
        "talos/intake/IntakeTarget.py",
        "talos/intake/__init__.py",
    ],
    imports = ["."],
    deps = ["@pypi//flatbuffers"],
)

py_library(
    name = "messages_py",
    srcs = ["talos_nodes/intake/__init__.py", "talos_nodes/intake/packet.py"],
    imports = ["."],
    deps = [":intake_bindings"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "intake_py",
    # Scoped teaching-example access (ITEM 32): the three standalone
    # example packages under main_processor/ build on this API without
    # refactoring it. Same __pkg__ pattern as drivetrain's
    # integration_test_support; nothing else gains access.
    visibility = [
        "//2026-robot/main_processor/example_roller:__pkg__",
        "//2026-robot/main_processor/example_flywheel:__pkg__",
        "//2026-robot/main_processor/example_position:__pkg__",
    ],
    srcs = [
        "talos_nodes/intake/node.py",
        "talos_nodes/intake/student_policy.py",
        "talos_nodes/intake/units.py",
        "talos_nodes/intake/motor_commands.py",
        "talos_nodes/intake/control_config.py",
        "talos_nodes/intake/behaviors.py",
        "talos_nodes/intake/examples/__init__.py",
        "talos_nodes/intake/examples/plant.py",
        "talos_nodes/intake/examples/roller_intake.py",
        "talos_nodes/intake/examples/flywheel.py",
        "talos_nodes/intake/examples/position_mechanism.py",
    ],
    imports = [
        ".",
    ],
    deps = [
        ":messages_py",
        "//talOS/ipc/python:rtms_py",
    ],
)

genrule(
    name = "package_config",
    srcs = ["subsystem.toml"],
    outs = ["talos_nodes/intake/subsystem.toml"],
    cmd = "cp $(location subsystem.toml) $@",
)

py_library(
    name = "main_py",
    srcs = ["talos_nodes/intake/main.py"],
    data = [":package_config"],
    deps = [":messages_py", ":intake_py", "//talOS/ipc/python:rtms_py"],
)

load("@talos//build:defs.bzl", "talos_py_node")

talos_py_node(
    name = "node",
    node_name = "intake",
    entry_point = "talos_nodes.intake.node:IntakeNode",
    contract = ":messages_py",
    describe_sources = "talos_nodes.intake.main:describe_source_rows",
    default_config = "talos_nodes.intake:subsystem.toml",
    deps = [
        ":intake_py",
        ":main_py",
    ],
)

py_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.py"],
    legacy_create_init = False,
    env = {"INTAKE_BINARY": "$(rlocationpath :node)"},
    data = [
        ":node",
        "testdata/subsystem.toml",
    ],
    deps = [
        ":messages_py",
        ":intake_py",
        ":main_py",
        "//talOS/ipc/python:rtms_py",
        "@rules_python//python/runfiles",
    ],
)

py_test(
    name = "student_api_test",
    size = "small",
    srcs = ["student_api_test.py"],
    legacy_create_init = False,
    deps = [
        ":messages_py",
        ":intake_py",
    ],
)

py_test(
    name = "examples_test",
    size = "small",
    srcs = ["examples_test.py"],
    legacy_create_init = False,
    deps = [
        ":messages_py",
        ":intake_py",
    ],
)

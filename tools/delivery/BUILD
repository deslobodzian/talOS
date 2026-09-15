"""Delivery tests and tools (items 49-60). New package only: no existing
BUILD file is modified to add these targets."""

load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_library", "py_test")

package(default_visibility = ["//visibility:public"])

exports_files([
    "deploy.sh",
    "gen_goldens.py",
])

py_library(
    name = "delivery_lib",
    srcs = [
        "compat_record.py",
        "readiness.py",
    ],
    imports = ["."],
)

# Item 50, Python side: the flatc-generated bindings re-encode the goldens.
py_test(
    name = "crosslang_py_test",
    size = "small",
    srcs = ["crosslang_py_test.py"],
    data = ["testdata/chassis_target_golden.bin", "testdata/drivetrain_state_golden.bin", "testdata/goldens.json"],
    # messages_py is the public re-export of the private drive_bindings;
    # the generated talos.drive modules ride along transitively.
    deps = ["//2026-robot/main_processor/drivetrain:messages_py"],
)

# Item 50, C++ side: the flatc-generated C++ headers decode the same goldens.
cc_test(
    name = "crosslang_test",
    size = "small",
    srcs = ["crosslang_test.cc"],
    data = ["testdata/chassis_target_golden.bin", "testdata/drivetrain_state_golden.bin"],
    deps = [
        "@talos_chassis_contracts//:contract",
        "@googletest//:gtest_main",
    ],
)

# Items 52-53: scenario matrix + failure/recovery over the real gateway.
cc_test(
    name = "scenario_matrix_test",
    size = "small",
    srcs = ["scenario_matrix_test.cc"],
    deps = [
        "//talOS/hardware:hardware",
        "@googletest//:gtest_main",
    ],
)

# Items 54-55: isolated sessions, clear failures, bounded shutdown.
cc_test(
    name = "supervision_test",
    size = "small",
    srcs = ["supervision_test.cc"],
    deps = [
        "//talOS/launcher:launcher_lib",
        "@googletest//:gtest_main",
    ],
)

# Item 51: fresh scaffolded projects, executed hermetically.
py_test(
    name = "fresh_project_test",
    size = "small",
    srcs = ["fresh_project_test.py"],
    deps = [
        "//talOS/ipc/python:rtms_py",
        "//tools/node_scaffold:node_scaffold_lib",
    ],
)

# Items 56-58: bundle checks, deploy script, offline proof.
py_test(
    name = "deploy_test",
    size = "small",
    srcs = ["deploy_test.py"],
    data = [
        "compat_record.py",
        "deploy.sh",
    ],
    env = {"DEPLOY_SH": "$(rlocationpath :deploy.sh)"},
    deps = [
        ":delivery_lib",
        "@rules_python//python/runfiles",
    ],
)

# Item 60: readiness dimensions over fixture configs.
py_test(
    name = "readiness_test",
    size = "small",
    srcs = ["readiness_test.py"],
    deps = [":delivery_lib"],
)

# Item 59: the docs' commands are executed/checked against the real CLIs.
py_test(
    name = "walkthrough_test",
    size = "small",
    srcs = ["walkthrough_test.py"],
    data = [
        "deploy.sh",
        "//docs:DEPLOYMENT.md",
        "//docs:NODE_DEVELOPMENT_GUIDE.md",
        "//talOS/launcher:launcher",
        "//tools/node_scaffold:node_scaffold",
    ],
    env = {
        "DEPLOYMENT_MD": "$(rlocationpath //docs:DEPLOYMENT.md)",
        "DEPLOY_SH": "$(rlocationpath //tools/delivery:deploy.sh)",
        "GUIDE_MD": "$(rlocationpath //docs:NODE_DEVELOPMENT_GUIDE.md)",
        "LAUNCHER_BIN": "$(rlocationpath //talOS/launcher:launcher)",
        "SCAFFOLD_BIN": "$(rlocationpath //tools/node_scaffold:node_scaffold)",
    },
    deps = [
        ":delivery_lib",
        "@rules_python//python/runfiles",
    ],
)

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@talos//build:defs.bzl", "talos_cc_node")

# The subsystem's own config file, so the robot manifest filegroup can ship it.
exports_files(["subsystem.toml"])
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

# Chassis frames live in the neutral talos-chassis_contracts module: one
# definition per frame type (duplicate FlatBuffers types under one include
# path fail the build). These aliases keep in-tree labels resolving while the
# platform nodes consume @talos_chassis_contracts directly.
alias(
    name = "drive_message",
    actual = "@talos_chassis_contracts//:drive_message",
    visibility = ["//visibility:public"],
)
alias(
    name = "messages_cc",
    actual = "@talos_chassis_contracts//:contract",
    visibility = ["//visibility:public"],
)
alias(
    name = "contract",
    actual = ":messages_cc",
    visibility = ["//visibility:public"],
)

load("@rules_python//python:defs.bzl", "py_library", "py_test")

py_library(
    name = "drive_bindings",
    srcs = [
        "talos/drive/ChassisTarget.py",
        "talos/drive/DrivetrainState.py",
        "talos/drive/__init__.py",
    ],
    imports = ["."],
    deps = ["@pypi//flatbuffers"],
)

py_library(
    name = "messages_py",
    srcs = [
        "talos_nodes/drivetrain/__init__.py",
        "talos_nodes/drivetrain/packet.py",
    ],
    imports = ["."],
    deps = [":drive_bindings"],
    visibility = ["//visibility:public"],
)

py_test(
    name = "messages_test",
    size = "small",
    srcs = ["messages_test.py"],
    deps = [":messages_py"],
)

cc_library(
    name = "drivetrain",
    hdrs = ["swerve.h", "geometry.h", "node.h"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/drivetrain",
    deps = [
        ":messages_cc",
        "//talOS/hardware:hardware",
        "//talOS/hardware:gate",
        "//talOS/protocol:protocol",
        "//talOS/configuration:configuration",
        "//talOS/events:events",
        "//talOS/process:process",
        "//talOS/process:hardware_main",
    ],
    visibility = ["//visibility:private"],
)
# The existing arbiter test also exercises the complete teleop pipeline.
# Keep that access test-only until the pipeline fixture moves to robot tests.
cc_library(
    name = "integration_test_support",
    testonly = True,
    deps = [":drivetrain"],
    visibility = ["//2026-robot/main_processor/arbiter:__pkg__"],
)

talos_cc_node(
    name = "node",
    node_name = "drivetrain",
    hdrs = ["node.h", "geometry.h", "swerve.h"],
    factory = "drive::CreateNode",
    contract = ":contract",
    deps = [":drivetrain"],
    hardware = True,
)
alias(
    name = "sim_gateway",
    actual = "//2026-robot/main_processor/simulation:sim_gateway",
    visibility = ["//visibility:public"],
)
cc_binary(name = "send_target", srcs = ["send_target.cc"], deps = [":messages_cc", "//talOS/events:events_os", "//talOS/ipc:ipc"])
cc_binary(
    name = "monitor",
    srcs = ["monitor.cc"],
    data = ["//2026-robot/main_processor/configuration:robot_config"],
    deps = [":drivetrain", "//talOS/ipc:ipc"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = ["testdata/robot.toml", "testdata/drivetrain.toml"],
    env = {"TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)"},
    deps = [":drivetrain", "@googletest//:gtest_main", "@rules_cc//cc/runfiles"],
)

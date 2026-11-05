load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_test")
load("//2026-robot/main_processor/configuration:robot_manifest.bzl", "CONFIG_FILES", "NODES")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "sim_gateway",
    srcs = ["sim_gateway.cc"],
    deps = [
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/introspection:describe",
        "//talOS/process",
    ],
)

# Startup, full-stack sessions, and transient producer-stall recovery.
py_test(
    name = "startup_test",
    srcs = ["startup_test.py"],
    data = [
        ":sim_gateway",
        "//2026-robot/main_processor/configuration:robot.toml",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
    ] + CONFIG_FILES + NODES,
    tags = ["exclusive"],
)

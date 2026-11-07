load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")
load("//2026-robot/main_processor/configuration:robot_physics_sim_manifest.bzl", "CONFIG_FILES", "NODES")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "sim_gateway",
    srcs = ["sim_gateway.cc"],
    deps = [
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/events:events_os",
        "//talOS/hardware",
        "//talOS/hardware:sim_messages",
        "//talOS/introspection:describe",
        "//talOS/introspection:registry",
        "//talOS/ipc",
        "//talOS/msgs:geometry",
        "//talOS/process",
    ],
)

# Startup, full-stack sessions, and transient producer-stall recovery.
py_test(
    name = "startup_test",
    srcs = ["startup_test.py"],
    data = [
        ":sim_gateway",
        "//2026-robot/main_processor/configuration:robot_physics_sim.toml",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
    ] + CONFIG_FILES + NODES,
    tags = ["exclusive"],
)

py_library(
    name = "accuracy",
    srcs = ["accuracy.py"],
    data = [
        ":sim_gateway",
        "//2026-robot/main_processor/configuration:robot_physics_sim.toml",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
    ] + CONFIG_FILES + NODES,
    imports = ["."],
    deps = ["@rules_python//python/runfiles"],
)

py_binary(
    name = "accuracy_scenario",
    srcs = ["accuracy_scenario.py"],
    deps = [":accuracy"],
)

py_test(
    name = "accuracy_test",
    srcs = ["accuracy_test.py"],
    tags = ["exclusive"],
    deps = [":accuracy"],
)

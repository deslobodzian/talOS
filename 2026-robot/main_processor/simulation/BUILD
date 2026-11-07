load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")
load("//2026-robot/main_processor/configuration:robot_physics_sim_manifest.bzl", "CONFIG_FILES", "NODES")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "sim_gateway",
    srcs = [
        "schedules.h",
        "sim_gateway.cc",
    ],
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

cc_test(
    name = "schedules_test",
    srcs = [
        "schedules.h",
        "schedules_test.cc",
    ],
    deps = [
        "//talOS/driver_station",
        "//talOS/hardware",
        "@googletest//:gtest_main",
    ],
)

# Startup and a bounded scripted match with producer/link/device faults.
py_test(
    name = "startup_test",
    size = "medium",
    srcs = ["startup_test.py"],
    data = [
        ":sim_gateway",
        "//2026-robot/main_processor/configuration:robot_physics_sim.toml",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
    ] + CONFIG_FILES + NODES,
    tags = ["exclusive"],
    timeout = "short",
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

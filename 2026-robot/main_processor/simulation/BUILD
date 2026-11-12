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
        "//talOS/driver_station:schedule",
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
        "//talOS/hardware",
        "@googletest//:gtest_main",
    ],
)

# Removes the autonomous progress record a namespaced sim session leaves.
py_library(
    name = "autonomous_progress",
    srcs = ["autonomous_progress.py"],
    imports = ["."],
)

# Startup and a bounded scripted match with link/device faults.
py_test(
    name = "startup_test",
    size = "medium",
    timeout = "short",
    srcs = ["startup_test.py"],
    data = [
        ":sim_gateway",
        "//2026-robot/main_processor/configuration:robot_physics_sim.toml",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
    ] + CONFIG_FILES + NODES,
    # Launches Python nodes; ctypes cannot load sanitizer runtimes in Python.
    tags = [
        "exclusive",
        "no_asan",
        "no_tsan",
    ],
    deps = [":autonomous_progress"],
)

# Full graph match rehearsal, with an intentional critical-node restart.
py_test(
    name = "match_rehearsal_test",
    size = "medium",
    timeout = "moderate",
    srcs = [
        "match_rehearsal_test.py",
        "startup_test.py",
    ],
    data = [
        ":sim_gateway",
        "//2026-robot/main_processor/configuration:robot_physics_sim.toml",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
    ] + CONFIG_FILES + NODES,
    imports = ["."],
    tags = [
        "exclusive",
        "no_asan",
        "no_tsan",
    ],
    deps = [":autonomous_progress"],
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
    deps = [
        ":autonomous_progress",
        "@rules_python//python/runfiles",
    ],
)

py_binary(
    name = "accuracy_scenario",
    srcs = ["accuracy_scenario.py"],
    deps = [":accuracy"],
)

# Over 5 s, e2e lane: three live sessions of the whole graph run 9 s and
# twice 5 s of scheduled time in real time; the eight estimator replays of
# them run at once on the virtual clock (under 1 s).
py_test(
    name = "accuracy_test",
    srcs = ["accuracy_test.py"],
    # Launches Python nodes; ctypes cannot load sanitizer runtimes in Python.
    tags = [
        "exclusive",
        "no_asan",
        "no_tsan",
    ],
    deps = [":accuracy"],
)

# Minutes of the full graph (Studio bridge included) under SIGKILL/SIGSTOP
# chaos; see soak_test.py. Manual: `bazel test //...` skips it, and the CI soak
# job runs it on its own (TALOS_SOAK_SECONDS sets the teleop length).
py_test(
    name = "soak_test",
    size = "large",
    timeout = "long",
    srcs = [
        "soak_test.py",
        "startup_test.py",
    ],
    data = [
        ":sim_gateway",
        "//2026-robot/main_processor/configuration:robot_physics_sim.toml",
        # The bridge refuses to start without its webroot,
        # [studio] webroot = bazel-bin/studio/dist.
        "//studio:web_dist",
        "//studio/bridge:studio_bridge",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
    ] + CONFIG_FILES + NODES,
    imports = ["."],
    # Launches Python nodes; ctypes cannot load sanitizer runtimes in Python.
    tags = [
        "exclusive",
        "manual",
        "no_asan",
        "no_tsan",
    ],
    deps = [":autonomous_progress"],
)

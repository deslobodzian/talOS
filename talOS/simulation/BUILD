load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_cc_node")

# The lightweight simulator: it stands in for the RoboRIO for any robot
# config, building its devices from the config's hardware (README.md).
cc_binary(
    name = "sim_gateway",
    srcs = ["sim_gateway.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":ports",
        ":schedules",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/driver_station:schedule",
        "//talOS/events:events_os",
        "//talOS/hardware",
        "//talOS/hardware:sim_messages",
        "//talOS/introspection:describe",
        "//talOS/introspection:registry",
        "//talOS/ipc",
        "//talOS/memory:residency",
        "//talOS/msgs:geometry",
        "//talOS/process",
    ],
)

# The simulator's UDP ports: its flags, else the config's (REQ-DEV-31).
cc_library(
    name = "ports",
    hdrs = ["ports.h"],
    deps = ["//talOS/configuration"],
)

cc_test(
    name = "ports_test",
    srcs = ["ports_test.cc"],
    deps = [
        ":ports",
        "//talOS/configuration",
        "@googletest//:gtest_main",
    ],
)

# The simulator's fault schedule: parsed from --fault-schedule, resolved
# against a config's devices and applied to the physics backend. Restricted
# because schedules.h includes hardware/sim_physics.h and depends on
# //talOS/hardware (RULES.md 2 / ARCH-2).
cc_library(
    name = "schedules",
    hdrs = ["schedules.h"],
    visibility = [
        "//talOS:__subpackages__",
        "//tools:__pkg__",
    ],
    deps = ["//talOS/hardware"],
)

cc_test(
    name = "schedules_test",
    srcs = ["schedules_test.cc"],
    deps = [
        ":schedules",
        "@googletest//:gtest_main",
    ],
)

# REQ-DEV-31: the launcher starts the simulator on the config's ports. Over
# 1 s, integration lane: the launcher runs the bridge and the simulator over
# loopback UDP until the bridge accepts a State, then stops the session.
py_test(
    name = "launcher_ports_test",
    size = "small",
    srcs = ["launcher_ports_test.py"],
    data = [
        ":sim_gateway",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
    ],
    env = {
        "LAUNCHER": "$(rlocationpath //talOS/launcher)",
        "LOG_DUMP": "$(rlocationpath //talOS/launcher:log_dump)",
    },
    tags = ["integration"],
    deps = [
        "//talOS/build:udp_ports",
        "//talOS/memory:shm_name",
        "@rules_python//python/runfiles",
    ],
)

# The simulator alone, as a child process: --describe and its flag checks,
# a run without a config, its Driver Station and State datagrams over
# loopback UDP (REQ-DEV-05, REQ-DEV-10, REQ-CONTRACT-30), and /sim/state
# truth publication under --physics. Over 1 s, integration lane: cases run
# the simulator for 0.3 s to 0.6 s of real time each.
py_test(
    name = "sim_gateway_test",
    size = "small",
    srcs = ["sim_gateway_test.py"],
    data = [":sim_gateway"],
    env = {"SIM_GATEWAY": "$(rlocationpath :sim_gateway)"},
    tags = ["integration"],
    deps = [
        "//talOS/build:udp_ports",
        "//talOS/memory:shm_name",
        "@rules_python//python/runfiles",
    ],
)

# Fail-closed CLI flag parsing for sim_gateway (fast lane: no sockets/waits).
py_test(
    name = "sim_gateway_cli_test",
    size = "small",
    srcs = ["sim_gateway_cli_test.py"],
    data = [":sim_gateway"],
    env = {"SIM_GATEWAY": "$(rlocationpath :sim_gateway)"},
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)

talos_cc_node(
    name = "bench_motor",
    testonly = True,
    srcs = ["bench_motor_fixture.cc"],
    node_name = "bench_motor",
    deps = [
        "//talOS/devices",
        "//talOS/msgs:control",
    ],
)

# Integration (REQ-SAFE-15, REQ-SAFE-22, REQ-OPS-20): replays the physical
# bench's 65 ms CAN/feedback silence (RESET-3) and 410 ms link blackout hole
# (RT-28) through sim_gateway --physics and hardware_node. Over 1 s: two real-
# time launcher sessions (~1.5 s and ~2.2 s) over loopback UDP.
py_test(
    name = "bench_faults_test",
    size = "small",
    srcs = [
        "bench_faults_test.py",
        "//talOS/build:sim_check.py",
    ],
    data = [
        ":bench_motor",
        ":sim_gateway",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
        "//talOS/teleop:node",
    ],
    env = {
        "LAUNCHER": "$(rlocationpath //talOS/launcher)",
        "LOG_DUMP": "$(rlocationpath //talOS/launcher:log_dump)",
    },
    legacy_create_init = False,
    main = "bench_faults_test.py",
    # Launches five instrumented processes per session; under macOS TSan on a
    # 3-vCPU hosted runner start-up overruns the ready wait (TODO.md CI-13).
    # The Linux sanitizer legs still run this test.
    tags = [
        "integration",
        "no_macos_sanitizer",
    ],
    deps = [
        "//talOS/memory:shm_name",
        "@rules_python//python/runfiles",
    ],
)

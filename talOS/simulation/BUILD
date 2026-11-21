load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")

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
# against a config's devices and applied to the physics backend. Public for
# a robot's tests of its own checks against the simulated robot.
cc_library(
    name = "schedules",
    hdrs = ["schedules.h"],
    visibility = ["//visibility:public"],
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
# a run without a config, and its Driver Station and State datagrams over
# loopback UDP for configs the cases write (REQ-DEV-10: home switches).
# Over 1 s, integration lane: two cases run the simulator for 0.6 s and
# 0.3 s of real time.
py_test(
    name = "sim_gateway_test",
    size = "small",
    srcs = ["sim_gateway_test.py"],
    data = [":sim_gateway"],
    env = {"SIM_GATEWAY": "$(rlocationpath :sim_gateway)"},
    tags = ["integration"],
    deps = ["@rules_python//python/runfiles"],
)

load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

# Compatibility aliases for BridgeStatus (//talOS/hardware/status:messages) so
# existing C++ and Python consumers of //talOS/bridge:messages[_py] keep their
# labels and `bridge/bridge_status_generated.h` / `bridge.BridgeStatus` paths.
alias(
    name = "messages",
    actual = "//talOS/hardware/status:messages",
    visibility = ["//visibility:public"],
)

alias(
    name = "messages_py",
    actual = "//talOS/hardware/status:messages_py",
    visibility = ["//visibility:public"],
)

# Private forwarding header for talOS/bridge's internal tests; the merge rules
# live in //talOS/hardware:merge.
cc_library(
    name = "merge",
    hdrs = ["merge.h"],
    deps = ["//talOS/hardware:merge"],
)

cc_library(
    name = "node_lib",
    srcs = ["node.cc"],
    hdrs = ["node.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/events:events_os",
        "//talOS/hardware",
        "//talOS/hardware:bridge_messages",
        "//talOS/hardware:can_buses",
        "//talOS/hardware:clock_offset",
        "//talOS/hardware:merge",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/introspection:registry",
        "//talOS/ipc",
        "//talOS/process",
        "//talOS/protocol",
        "//talOS/rtms:wake",
    ],
)

cc_binary(
    name = "hardware_node",
    srcs = ["main.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":node_lib",
        "//talOS/events:events_log",
        "//talOS/introspection:describe",
    ],
)

# The bridge and fake gateway on loopback ports, the test configs, and the
# frames the two ends exchange, for the test targets below.
cc_library(
    name = "node_test_support",
    testonly = True,
    hdrs = ["node_test_support.h"],
    deps = [
        ":node_lib",
        "//talOS/configuration",
        "//talOS/ipc",
        "//talOS/protocol",
        "//talOS/protocol:udp_test_helpers",
    ],
)

# The gateway end of a HardwareNode's link in process (HardwareNode's
# friend), for trust_test: testonly and private to this package, so no
# production code can reach it.
cc_library(
    name = "in_process_gateway",
    testonly = True,
    hdrs = ["in_process_gateway.h"],
    deps = [
        ":node_lib",
        "//talOS/hardware",
        "//talOS/protocol",
    ],
)

# The request trust rules on HardwareNode through the in-process gateway:
# no socket, synthetic time, so the component tier and the fast lane
# (docs/TESTING.md §2, §9).
cc_test(
    name = "trust_test",
    size = "small",
    srcs = ["trust_test.cc"],
    deps = [
        ":in_process_gateway",
        ":merge",
        ":messages",
        ":node_lib",
        ":node_test_support",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/ipc",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

# The UDP path: States, Driver Station packets and acknowledgements in, and
# probes, configurations and commands out, through a loopback fake gateway.
cc_test(
    name = "node_test",
    size = "small",
    srcs = ["hardware_node_test.cc"],
    data = [":hardware_node"],
    env = {"HARDWARE_NODE": "$(rlocationpath :hardware_node)"},
    tags = ["integration"],
    deps = [
        ":merge",
        ":messages",
        ":node_lib",
        ":node_test_support",
        "//talOS/driver_station",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/events:events_os",
        "//talOS/events:input_replay",
        "//talOS/hardware",
        "//talOS/introspection:registry",
        "//talOS/ipc",
        "//talOS/node:allocation_probe",
        "//talOS/process",
        "//talOS/protocol:udp_test_helpers",
        "//talOS/rtms",
        "//talOS/schema:identity",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# Run() on real time and loopback UDP: the integration tier
# (docs/TESTING.md §2), kept out of node_test's synthetic-time logic tests.
# Its bounds are ratios of counts and event-driven waits; the wall-clock
# StallMeter timing test lives in run_timing_test with `exclusive`.
cc_test(
    name = "run_test",
    size = "medium",
    srcs = ["run_test.cc"],
    tags = ["integration"],
    deps = [
        ":messages",
        ":node_lib",
        ":node_test_support",
        "//talOS/introspection:registry",
        "//talOS/ipc",
        "//talOS/process",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

# Run() wall-clock timing on real time and loopback UDP: isolated from
# run_test with `exclusive` so parallel Bazel jobs on 3-vCPU macOS CI do not
# starve its 1 ms StallMeter host-qualification probe (docs/TESTING.md §6.7).
cc_test(
    name = "run_timing_test",
    size = "medium",
    srcs = ["run_timing_test.cc"],
    tags = [
        "exclusive",
        "integration",
    ],
    deps = [
        ":merge",
        ":node_lib",
        ":node_test_support",
        "//talOS/ipc",
        "//talOS/process",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

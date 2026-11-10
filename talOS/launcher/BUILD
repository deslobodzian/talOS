load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_binary", "py_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

cc_library(
    name = "launcher_lib",
    hdrs = [
        "launcher.h",
        "service_notifier.h",
        "session_lock.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/configuration",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/introspection:describe",
        "//talOS/introspection:lock_file",
        "//talOS/introspection:names",
        "//talOS/introspection:registry",
        "//talOS/memory:shared_memory",
        "//talOS/process:durable_sync",
        "//talOS/rtms",
        "//talOS/schema:identity",
        "//talOS/schema:unit_suffix",
        "@tomlplusplus//:toml++",
    ],
)

cc_binary(
    name = "launcher",
    srcs = ["main.cc"],
    visibility = ["//visibility:public"],
    deps = [":launcher_lib"],
)

cc_binary(
    name = "log_dump",
    srcs = ["log_dump.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":launcher_lib",
        "//talOS/driver_station",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/hardware",
        "//talOS/schema:decode",
    ],
)

cc_test(
    name = "log_dump_test",
    size = "small",
    srcs = ["log_dump_test.cc"],
    data = [":log_dump"],
    env = {"TALOS_LOG_DUMP": "$(rlocationpath :log_dump)"},
    deps = [
        ":continuity_message",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

py_binary(
    name = "session_report",
    srcs = ["session_report.py"],
    data = [":log_dump"],
    visibility = ["//visibility:public"],
    deps = ["@rules_python//python/runfiles"],
)

py_test(
    name = "session_report_test",
    size = "small",
    srcs = [
        "session_report.py",
        "session_report_test.py",
    ],
    data = [
        ":session_report",
        ":session_report_fixture",
    ],
    deps = ["@rules_python//python/runfiles"],
)

cc_binary(
    name = "session_report_fixture",
    testonly = True,
    srcs = ["testdata/session_report_fixture.cc"],
    deps = [
        ":launcher_lib",
        "//talOS/driver_station",
        "//talOS/events:events_log",
        "//talOS/hardware",
    ],
)

# bazel run //talOS/launcher:restart_latency -- --config <robot.toml> [...]
# Per-stage SIGKILL-to-first-request restart latency; see README "Restart
# latency". The config's nodes come from the workspace's bazel-bin.
py_binary(
    name = "restart_latency",
    srcs = ["restart_latency.py"],
    data = [":launcher"],
    visibility = ["//visibility:public"],
    deps = ["@rules_python//python/runfiles"],
)

py_test(
    name = "restart_latency_test",
    size = "small",
    srcs = [
        "restart_latency.py",
        "restart_latency_test.py",
    ],
    data = [":restart_latency_fixture"],
    deps = ["@rules_python//python/runfiles"],
)

cc_binary(
    name = "restart_latency_fixture",
    testonly = True,
    srcs = ["testdata/restart_latency_fixture.cc"],
    deps = [
        "//talOS/events:events_log",
        "//talOS/hardware",
    ],
)

cc_test(
    name = "launcher_test",
    size = "small",
    # Each spawn is two fork()s; under macOS ASan/TSan one fork costs about
    # 30/375 ms (1.5 ms plain), so session timing windows cannot hold there.
    # The Linux sanitizer legs still run this test.
    tags = ["no_macos_sanitizer"],
    srcs = ["launcher_test.cc"],
    deps = [
        ":launcher_lib",
        "//talOS/events:events_log",
        "//talOS/events:input_replay",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "@googletest//:gtest_main",
    ],
)

talos_cc_node(
    name = "alpha",
    srcs = ["testdata/construction_node.cc"],
    node_name = "alpha",
    visibility = ["//visibility:private"],
)

# Run as `alpha`: publishes an endpoint its --describe probe did not declare
# when its config sets `drift = 1` (supervision_test's endpoint check).
talos_cc_node(
    name = "endpoint_drift",
    srcs = ["testdata/endpoint_drift_node.cc"],
    node_name = "alpha",
    node_target = "//example/alpha:node",
    visibility = ["//visibility:private"],
    deps = [":continuity_message"],
)

cc_test(
    name = "supervision_test",
    size = "medium",
    # Each spawn is two fork()s; under macOS ASan/TSan one fork costs about
    # 30/375 ms (1.5 ms plain), so session timing windows cannot hold there.
    # The Linux sanitizer legs still run this test.
    tags = ["no_macos_sanitizer"],
    srcs = ["supervision_test.cc"],
    data = [
        ":alpha",
        ":endpoint_drift",
    ],
    env = {
        "TALOS_CONSTRUCTION_NODE": "$(rlocationpath :alpha)",
        "TALOS_ENDPOINT_DRIFT_NODE": "$(rlocationpath :endpoint_drift)",
    },
    deps = [
        ":launcher_lib",
        "//talOS/introspection:registry",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# A publisher that kills itself (SIGKILL) on its first run and exits cleanly
# on its second, and a subscriber that logs what it receives: the launcher's
# session-owned segments keep the subscriber's topic across both.
talos_messages(
    name = "continuity_message",
    srcs = ["testdata/continuity_message.fbs"],
    include_prefix = "continuity_test",
    visibility = ["//visibility:private"],
)

talos_cc_node(
    name = "continuity_source",
    srcs = ["testdata/continuity_source.cc"],
    node_name = "continuity_source",
    visibility = ["//visibility:private"],
    deps = [":continuity_message"],
)

talos_cc_node(
    name = "continuity_sink",
    srcs = ["testdata/continuity_sink.cc"],
    node_name = "continuity_sink",
    visibility = ["//visibility:private"],
    deps = [":continuity_message"],
)

cc_test(
    name = "continuity_test",
    size = "small",
    srcs = ["continuity_test.cc"],
    data = [
        ":continuity_sink",
        ":continuity_source",
    ],
    env = {
        "TALOS_SOURCE_NODE": "$(rlocationpath :continuity_source)",
        "TALOS_SINK_NODE": "$(rlocationpath :continuity_sink)",
    },
    deps = [
        ":continuity_message",
        ":launcher_lib",
        "//talOS/events:events_log",
        "//talOS/rtms",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# --replay-inputs: a producer recorded in a session, a consumer that was not
# in it, and a reader of the producer's topic as another type of the same
# size.
talos_messages(
    name = "replay_message",
    srcs = ["testdata/replay_message.fbs"],
    include_prefix = "replay_inputs_test",
    visibility = ["//visibility:private"],
)

talos_cc_node(
    name = "replay_source",
    srcs = ["testdata/replay_source.cc"],
    node_name = "replay_source",
    visibility = ["//visibility:private"],
    deps = [":replay_message"],
)

talos_cc_node(
    name = "replay_consumer",
    srcs = ["testdata/replay_consumer.cc"],
    node_name = "replay_consumer",
    visibility = ["//visibility:private"],
    deps = [":replay_message"],
)

talos_cc_node(
    name = "replay_impostor",
    srcs = ["testdata/replay_impostor.cc"],
    node_name = "replay_impostor",
    visibility = ["//visibility:private"],
    deps = [":replay_message"],
)

cc_test(
    name = "replay_inputs_test",
    size = "small",
    srcs = ["replay_inputs_test.cc"],
    data = [
        ":replay_consumer",
        ":replay_impostor",
        ":replay_source",
    ],
    env = {
        "TALOS_SOURCE_NODE": "$(rlocationpath :replay_source)",
        "TALOS_CONSUMER_NODE": "$(rlocationpath :replay_consumer)",
        "TALOS_IMPOSTOR_NODE": "$(rlocationpath :replay_impostor)",
    },
    deps = [
        ":launcher_lib",
        ":replay_message",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

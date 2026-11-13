load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_binary", "py_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

# A session's records: its manifest, run history, lock, retention facts and
# the atomic writes that keep them.
cc_library(
    name = "session",
    srcs = [
        "session.cc",
        "session_retention.cc",
    ],
    hdrs = [
        "file_digest.h",
        "session.h",
        "session_lock.h",
        "session_retention.h",
    ],
    visibility = ["//studio/review:__pkg__"],
    deps = [
        "//talOS/driver_station",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/hardware",
        "//talOS/introspection:describe",
        "//talOS/introspection:lock_file",
        "//talOS/memory:shared_memory",
        "//talOS/process:realtime_outcome",
        "//talOS/schema:identity",
    ],
)

# fork + exec into a process group with a guardian, and the child's reports.
cc_library(
    name = "spawn",
    srcs = ["spawn.cc"],
    hdrs = ["spawn.h"],
)

# Drains every roster member's stdout and stderr into its <name>.log, and
# relays the launcher's own stdout and stderr (and talos.log) off its threads.
cc_library(
    name = "child_output",
    srcs = ["child_output.cc"],
    hdrs = ["child_output.h"],
    deps = [":spawn"],
)

# LauncherOptions, the bundle manifest, and the roster a config names.
cc_library(
    name = "roster",
    srcs = ["roster.cc"],
    hdrs = ["roster.h"],
    deps = [
        ":session",
        ":supervision_policy",
        "//talOS/configuration",
        "//talOS/introspection:names",
        "//talOS/introspection:registry",
        "//talOS/process:durable_sync",
        "@tomlplusplus//:toml++",
    ],
)

# Phase one: --describe probes, the declared graph, its lint and segments.
cc_library(
    name = "describe_graph",
    srcs = ["describe_graph.cc"],
    hdrs = ["describe_graph.h"],
    deps = [
        ":roster",
        ":session",
        ":spawn",
        "//talOS/configuration",
        "//talOS/events:events_core",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/rtms",
        "//talOS/schema:identity",
        "//talOS/schema:unit_suffix",
        "@tomlplusplus//:toml++",
    ],
)

# Each member's share of the session's event-log budget.
cc_library(
    name = "log_budget",
    srcs = ["log_budget.cc"],
    hdrs = ["log_budget.h"],
    deps = [
        ":describe_graph",
        "//talOS/bridge:messages",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/hardware",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/schema:identity",
    ],
)

# The split and its window as a pure function.
cc_test(
    name = "log_budget_split_test",
    size = "small",
    srcs = ["log_budget_split_test.cc"],
    deps = [
        ":log_budget",
        "@googletest//:gtest_main",
    ],
)

# The log files a log tool's inputs name, each once (log_dump,
# tlog_to_wpilog).
cc_library(
    name = "log_inputs",
    srcs = ["log_inputs.cc"],
    hdrs = ["log_inputs.h"],
    visibility = ["//studio/review:__pkg__"],
    deps = ["//talOS/events:events_log"],
)

# One session's event logs streamed in time order, a fixed window per log,
# for log_dump and tlog_to_wpilog.
cc_library(
    name = "log_merge",
    srcs = ["log_merge.cc"],
    hdrs = ["log_merge.h"],
    visibility = ["//studio/review:__pkg__"],
    deps = [
        "//talOS/events:events_core",
        "//talOS/events:events_log",
    ],
)

cc_test(
    name = "log_merge_test",
    size = "small",
    srcs = ["log_merge_test.cc"],
    deps = [
        ":log_inputs",
        ":log_merge",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "@googletest//:gtest_main",
    ],
)

# Restart backoff, budget and quarantine, and the stall, missing-row and
# construction kills, as functions of the supervision clock's times.
cc_library(
    name = "supervision_policy",
    srcs = ["supervision_policy.cc"],
    hdrs = ["supervision_policy.h"],
    deps = [":session"],
)

# The policy's decisions stepped by hand through simulated hours: no file,
# process or clock.
cc_test(
    name = "supervision_policy_test",
    size = "small",
    srcs = ["supervision_policy_test.cc"],
    deps = [
        ":roster",
        ":session",
        ":supervision_policy",
        "@googletest//:gtest_main",
    ],
)

# The supervisor (Launcher) and the Studio bridge it supervises.
cc_library(
    name = "launcher_lib",
    srcs = [
        "launcher.cc",
        "launcher_retention.cc",
        "studio_bridge.cc",
    ],
    hdrs = [
        "launcher.h",
        "service_notifier.h",
        "studio_bridge.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":child_output",
        ":describe_graph",
        ":log_budget",
        ":roster",
        ":session",
        ":spawn",
        ":supervision_policy",
        "//talOS/configuration",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/introspection:describe",
        "//talOS/introspection:lock_file",
        "//talOS/introspection:names",
        "//talOS/introspection:registry",
        "//talOS/rtms",
        "@tomlplusplus//:toml++",
    ],
)

cc_binary(
    name = "launcher",
    srcs = ["main.cc"],
    visibility = ["//visibility:public"],
    deps = [":launcher_lib"],
)

# The hardware packet topics decoded with their wire codecs, for log_dump.
cc_library(
    name = "hardware_record",
    srcs = ["hardware_record.cc"],
    hdrs = ["hardware_record.h"],
    visibility = ["//studio/review:__pkg__"],
    deps = [
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/hardware:config",
        "//talOS/schema:identity",
    ],
)

cc_test(
    name = "hardware_record_test",
    size = "small",
    srcs = ["hardware_record_test.cc"],
    deps = [
        ":hardware_record",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "@googletest//:gtest_main",
    ],
)

cc_binary(
    name = "log_dump",
    srcs = ["log_dump.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":hardware_record",
        ":log_inputs",
        ":log_merge",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/events:events_core",
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
        "//talOS/configuration",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/hardware",
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
        ":session",
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

# A self-contained WPILib DataLog (.wpilog) writer: no WPILib dependency.
cc_library(
    name = "wpilog",
    srcs = ["wpilog.cc"],
    hdrs = ["wpilog.h"],
)

cc_test(
    name = "wpilog_test",
    size = "small",
    srcs = ["wpilog_test.cc"],
    deps = [
        ":wpilog",
        "@googletest//:gtest_main",
    ],
)

# .tlog dispatch logs (or a session directory) as one .wpilog for
# AdvantageScope; README.md, "Converting a session to WPILOG".
cc_binary(
    name = "tlog_to_wpilog",
    srcs = ["tlog_to_wpilog.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":log_inputs",
        ":log_merge",
        ":wpilog",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/schema:identity",
        "//talOS/schema:unit_suffix",
        "@flatbuffers//:runtime_cc",
    ],
)

# Peak resident memory of log_dump and tlog_to_wpilog on a generated 200 MB
# session; run by hand (README.md, "Filtering and exporting logs").
cc_binary(
    name = "log_memory_bench",
    srcs = ["log_memory_bench.cc"],
    args = [
        "$(rlocationpath :log_dump)",
        "$(rlocationpath :tlog_to_wpilog)",
    ],
    data = [
        ":log_dump",
        ":tlog_to_wpilog",
    ],
    deps = [
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "@rules_cc//cc/runfiles",
    ],
)

# Every scalar base type, fixed arrays, an enum and a nested geometry.Pose2d.
talos_messages(
    name = "wpilog_message",
    srcs = ["testdata/wpilog_message.fbs"],
    include_prefix = "wpilog_test",
    visibility = ["//visibility:private"],
    deps = ["//talOS/msgs:geometry"],
)

cc_test(
    name = "tlog_to_wpilog_test",
    size = "small",
    srcs = ["tlog_to_wpilog_test.cc"],
    data = [":tlog_to_wpilog"],
    env = {"TALOS_TLOG_TO_WPILOG": "$(rlocationpath :tlog_to_wpilog)"},
    deps = [
        ":wpilog_message",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/msgs:geometry",
        "//talOS/schema:identity",
        "@flatbuffers//:runtime_cc",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# What the launcher's process tests wait on instead of a duration: the
# supervision loop's passes and a supervision clock the test moves ahead.
cc_library(
    name = "supervision_events",
    testonly = True,
    hdrs = ["testdata/supervision_events.h"],
    deps = [
        ":spawn",
        ":supervision_policy",
    ],
)

# Unlinks the test namespace's registry segment after a process test's last
# test, so the launcher's tests leave /dev/shm as they found it.
cc_library(
    name = "registry_cleanup",
    testonly = True,
    srcs = ["testdata/registry_cleanup.cc"],
    deps = [
        "//talOS/introspection:registry",
        "@googletest//:gtest",
    ],
    alwayslink = True,
)

cc_test(
    name = "launcher_test",
    size = "small",
    srcs = ["launcher_test.cc"],
    # Each spawn is two fork()s; under macOS ASan/TSan one fork costs about
    # 30/375 ms (1.5 ms plain), so session timing windows cannot hold there.
    # The Linux sanitizer legs still run this test.
    tags = [
        "integration",
        "no_macos_sanitizer",
    ],
    deps = [
        ":launcher_lib",
        ":log_budget",
        ":log_merge",
        ":registry_cleanup",
        ":supervision_events",
        "//talOS/driver_station",
        "//talOS/events:events_log",
        "//talOS/events:input_replay",
        "//talOS/hardware",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "@googletest//:gtest_main",
    ],
)

# The launcher's log window reaching the real bridge and a node, against a
# gateway in the test process (REQ-OPS-32).
talos_cc_node(
    name = "budget_fast",
    srcs = ["testdata/budget_motor.cc"],
    node_name = "budget_fast",
    visibility = ["//visibility:private"],
    deps = ["//talOS/devices"],
)

cc_test(
    name = "log_budget_test",
    size = "small",
    srcs = ["log_budget_test.cc"],
    data = [
        ":budget_fast",
        "//talOS/bridge:hardware_node",
    ],
    env = {
        "TALOS_BRIDGE": "$(rlocationpath //talOS/bridge:hardware_node)",
        "TALOS_BUDGET_FAST": "$(rlocationpath :budget_fast)",
    },
    # Each spawn is two fork()s; under macOS ASan/TSan one fork costs about
    # 30/375 ms (1.5 ms plain), so session timing windows cannot hold there.
    # The Linux sanitizer legs still run this test.
    tags = [
        "integration",
        "no_macos_sanitizer",
    ],
    deps = [
        ":launcher_lib",
        ":log_budget",
        ":registry_cleanup",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/events:events_log",
        "//talOS/hardware",
        "//talOS/introspection:describe",
        "//talOS/protocol",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
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

# Run as `alpha`: loses one wake notification on its byte topic
# (supervision_test's strict report check).
talos_cc_node(
    name = "byte_wake_loss",
    srcs = ["testdata/byte_wake_loss_node.cc"],
    node_name = "alpha",
    node_target = "//example/alpha:node",
    visibility = ["//visibility:private"],
    deps = [
        "//talOS/rtms",
        "//talOS/rtms:wake",
        "//talOS/schema:identity",
    ],
)

# A member that only keeps a registry row with the delivery-loss counts its
# command line gives (supervision_test's manifest check).
cc_binary(
    name = "delivery_loss_node",
    testonly = True,
    srcs = ["testdata/delivery_loss_node.cc"],
    deps = [
        "//talOS/events:events_core",
        "//talOS/introspection:registry",
    ],
)

cc_test(
    name = "supervision_test",
    size = "medium",
    srcs = ["supervision_test.cc"],
    data = [
        ":alpha",
        ":byte_wake_loss",
        ":delivery_loss_node",
        ":endpoint_drift",
        ":session_report",
    ],
    env = {
        "TALOS_BYTE_WAKE_LOSS_NODE": "$(rlocationpath :byte_wake_loss)",
        "TALOS_CONSTRUCTION_NODE": "$(rlocationpath :alpha)",
        "TALOS_DELIVERY_LOSS_NODE": "$(rlocationpath :delivery_loss_node)",
        "TALOS_ENDPOINT_DRIFT_NODE": "$(rlocationpath :endpoint_drift)",
        "TALOS_SESSION_REPORT": "$(rlocationpath :session_report)",
    },
    # Independent cases; HeartbeatingConstructorStillHasAFiniteDeadline alone
    # waits out a real 3 s construction deadline (its 2 s grace covers a
    # sanitizer build's registration), so shards cut the wall time.
    shard_count = 3,
    # Each spawn is two fork()s; under macOS ASan/TSan one fork costs about
    # 30/375 ms (1.5 ms plain), so session timing windows cannot hold there.
    # The Linux sanitizer legs still run this test.
    tags = [
        "integration",
        "no_macos_sanitizer",
    ],
    deps = [
        ":launcher_lib",
        ":registry_cleanup",
        ":supervision_events",
        "//talOS/introspection:registry",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# Whether a capped event log stopped or records on, from real logs.
cc_test(
    name = "capped_log_test",
    size = "small",
    srcs = ["capped_log_test.cc"],
    tags = ["integration"],
    deps = [
        ":launcher_lib",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "@googletest//:gtest_main",
    ],
)

# A session whose stdout and stderr are closed or full and never read, a
# terminal hangup, and the signal dispositions and mask its children start
# with (REQ-OPS-33).
cc_test(
    name = "closed_output_test",
    size = "small",
    srcs = ["closed_output_test.cc"],
    data = [":signal_probe"],
    env = {"TALOS_SIGNAL_PROBE": "$(rlocationpath :signal_probe)"},
    # openpty: libutil before glibc 2.34, libc on macOS.
    linkopts = select({
        "@platforms//os:linux": ["-lutil"],
        "//conditions:default": [],
    }),
    # Each spawn is two fork()s; see launcher_test.
    tags = [
        "integration",
        "no_macos_sanitizer",
    ],
    deps = [
        ":launcher_lib",
        ":registry_cleanup",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# The launcher binary SIGKILLed under a live roster of shell stubs: every
# member's process group, grandchildren included, dies with it (REQ-SAFE-30).
cc_test(
    name = "launcher_death_test",
    size = "small",
    srcs = ["launcher_death_test.cc"],
    data = [":launcher"],
    env = {"TALOS_LAUNCHER": "$(rlocationpath :launcher)"},
    tags = ["integration"],
    deps = [
        "//talOS/introspection:describe",
        "//talOS/introspection:registry",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# Prints the signals it started with ignored or blocked.
cc_binary(
    name = "signal_probe",
    testonly = True,
    srcs = ["testdata/signal_probe.cc"],
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
    tags = ["integration"],
    deps = [
        ":continuity_message",
        ":launcher_lib",
        ":registry_cleanup",
        "//talOS/events:events_log",
        "//talOS/rtms",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# --replay-inputs: a producer recorded in a session, a consumer that was not
# in it, a sampler that was, a reader of the producer's topic as another type
# of the same size, a reader of the declared graph that keeps a record under
# its session, and a reader of a motor through the devices layer.
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
    name = "replay_sampler",
    srcs = ["testdata/replay_sampler.cc"],
    node_name = "replay_sampler",
    visibility = ["//visibility:private"],
    deps = [":replay_message"],
)

talos_cc_node(
    name = "replay_motor",
    srcs = ["testdata/replay_motor.cc"],
    node_name = "replay_motor",
    visibility = ["//visibility:private"],
    deps = [
        ":replay_message",
        "//talOS/devices",
    ],
)

talos_cc_node(
    name = "replay_impostor",
    srcs = ["testdata/replay_impostor.cc"],
    node_name = "replay_impostor",
    visibility = ["//visibility:private"],
    deps = [":replay_message"],
)

talos_cc_node(
    name = "replay_graph_reader",
    srcs = ["testdata/replay_graph_reader.cc"],
    node_name = "replay_graph_reader",
    visibility = ["//visibility:private"],
    deps = [
        ":replay_message",
        "//talOS/introspection:describe",
    ],
)

cc_test(
    name = "replay_inputs_test",
    size = "small",
    srcs = ["replay_inputs_test.cc"],
    data = [
        ":replay_consumer",
        ":replay_graph_reader",
        ":replay_impostor",
        ":replay_motor",
        ":replay_sampler",
        ":replay_source",
    ],
    env = {
        "TALOS_SOURCE_NODE": "$(rlocationpath :replay_source)",
        "TALOS_CONSUMER_NODE": "$(rlocationpath :replay_consumer)",
        "TALOS_IMPOSTOR_NODE": "$(rlocationpath :replay_impostor)",
        "TALOS_GRAPH_READER_NODE": "$(rlocationpath :replay_graph_reader)",
        "TALOS_SAMPLER_NODE": "$(rlocationpath :replay_sampler)",
        "TALOS_MOTOR_NODE": "$(rlocationpath :replay_motor)",
    },
    # Each spawn is two fork()s; under macOS ASan/TSan one fork costs about
    # 30/375 ms (1.5 ms plain), so session timing windows cannot hold there.
    # The Linux sanitizer legs still run this test.
    tags = [
        "integration",
        "no_macos_sanitizer",
    ],
    deps = [
        ":launcher_lib",
        ":registry_cleanup",
        ":replay_message",
        ":spawn",
        "//talOS/bridge:messages",
        "//talOS/configuration",
        "//talOS/events",
        "//talOS/events:events_core",
        "//talOS/events:events_log",
        "//talOS/hardware",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
